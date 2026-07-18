# GLARE.md — Addressing driver-monitoring glare false-positives

**Status:** analysis + recommended plan; **Layer C band-aid DEPLOYED 2026-07-06** (`4devpnw` @
`2fc78f0fbd`, ungated, live on the 3X — see §10). Layers A (BPS driver-cam pipeline) and B (new
sleep-prob DM model) — the real upstream fixes — remain **NOT ported**. **Symptom owner:** the
"driver inattentive / distracted" false-alert that fires when **low-angle / side sunlight washes out
the driver-monitoring (DM) camera** even though the driver is watching the road.

> TL;DR — There is **no dedicated glare detector** anywhere in openpilot. "Glare handling" is entirely
> *downstream* of the DM model's outputs (`faceProb`, pose-`std`, `sunglassesProb`). The false
> "inattentive" you see is the model failing to *confirm* attention (washed-out image → high pose
> uncertainty or no face), which trips the wheel-touch fallback. The two real fixes are **upstream**:
> (1) feed the model a better-exposed image (camera ISP), and (2) a glare-robust DM model.
> Downstream threshold tuning is a band-aid with a safety cost. **Your xnor/pnw base is missing both
> of the upstream improvements that commaai already shipped in real-0.11.1** (see §5).

---

## 1. The symptom, precisely

With the sun low and off to one side, the comma's driver camera gets blown out on the lit side of
the face. After a short hold the UI escalates to "pay attention" / "driver distracted" and (if
engaged) starts the disengage countdown — while the driver is fully attentive. Brief glints recover;
sustained glare (e.g. a long west-bound afternoon stretch) keeps re-triggering.

This is **not** the relaxed-timeout problem that `dmon2xnor` addresses. `dmon2xnor` only lengthens the
*distraction timeout durations*; it does nothing for the glare path described here. The two efforts
are orthogonal.

---

## 2. How DM actually "sees" glare (root cause)

DM logic lives in `selfdrive/monitoring/helpers.py` on the **xnor base** (renamed to `policy.py` in
real-0.11.1 upstream — see §5). The only signals it has about "is the image usable right now" are the
neural-net outputs per frame:

| Signal (`driverStateV2.{left,right}DriverData`) | Meaning | Threshold (xnor base) |
|---|---|---|
| `faceProb` | confidence a face is present | `_FACE_THRESHOLD = 0.7` (`helpers.py:34`) |
| `faceOrientationStd` (pitch/yaw std) | model's **uncertainty** about head pose | `_POSESTD_THRESHOLD = 0.3` (`helpers.py:59`) |
| `sunglassesProb` | looks like sunglasses | `_SG_THRESHOLD = 0.9` (`helpers.py:36`) |
| `leftEyeProb`/`rightEyeProb`, `*BlinkProb` | eye open/closed, blink | `_EYE_THRESHOLD = 0.65`, `_BLINK_THRESHOLD = 0.865` |

There is **no** raw-image analysis (no overexposure/saturation metric feeding the policy). The
deprecated `poorVision` capnp field is **not emitted by the current model and not read** anywhere — a
dead hook, not a glare guard.

### The three things that happen under glare

1. **Pose uncertainty spikes.** A washed-out face makes `faceOrientationStd` large →
   `model_std_max >= 0.3` → `pose.low_std = False` (`helpers.py:283-286`). A distraction is only
   *declared* when `low_std` is True (`helpers.py:294-297`), so a single uncertain frame does **not**
   by itself mark you distracted — good.

2. **The wheel-touch fallback — this is the false "inattentive".** Sustained high std accumulates in
   `hi_stds` (`helpers.py:321-324`); after `_HI_STD_FALLBACK_TIME` (**10 s**, `helpers.py:60`):
   ```python
   self.is_model_uncertain = self.hi_stds > self.settings._HI_STD_FALLBACK_TIME   # helpers.py:319
   ...
   maybe_distracted = self.hi_stds > self.settings._HI_STD_FALLBACK_TIME or not self.face_detected   # helpers.py:370
   ```
   `maybe_distracted` is **also** True the instant glare washes the face out entirely (`faceProb < 0.7`
   → `not face_detected`). When `maybe_distracted`, awareness decrements (`helpers.py:372-376`) and
   `_set_timers(face_detected and not is_model_uncertain)` (`helpers.py:320`) drops monitoring into
   **passive wheel-touch** mode → the "pay attention / distracted" escalation. This is the path that
   produces the complaint.

3. **The per-drive degraded-camera notice** (diagnostic only). `dcam_uncertain_cnt` counts high-std
   time; reset after `_DCAM_UNCERTAIN_RESET_COUNT` clear frames; after `_DCAM_UNCERTAIN_ALERT_COUNT`
   (60 s cumulative) it raises the offroad alert `Offroad_DriverMonitoringUncertain`
   (`helpers.py:309-318`, `395-397`). This **does not** feed `awareness` — it only nags that the camera
   view is bad.

### The sunglasses gate actually *helps* glare

```python
self.blink.left  = ... * (driver_data.sunglassesProb < self.settings._SG_THRESHOLD)   # helpers.py:287-290
```
Strong side-sun often reads as "sunglasses" to the model, which **disables blink-based distraction**.
So glare will not by itself trigger a *blink* false-positive; the problem is the pose-std / no-face
path (#2), not blink.

### Diagnosing which sub-case you're in

Pull a glare drive and look at `driverMonitoringState` + `driverStateV2`:
- `faceProb` stays **> 0.7** but `isLowStd == False` / `hiStdCount` climbs → **model-uncertainty**
  case (image is exposed enough to find a face, but pose is unreliable). Fix = better model and/or
  better exposure.
- `faceProb` drops **< 0.7** (face lost) → **exposure** case (the image is blown out). Fix = camera
  ISP / exposure first.

`driverMonitoringState` exposes exactly the fields you need: `faceDetected`, `isLowStd`,
`hiStdCount`, `uncertainCount`, `isDistracted`, `awarenessStatus` (`helpers.py:400-421`). The
`tools/jotpluggler/layouts/driver-monitoring-debug.json` layout plots these against the DM video.

---

## 3. The three layers where glare can be fixed

| Layer | What it is | Leverage | Risk |
|---|---|---|---|
| **A. Image (camera ISP / exposure)** | Tone-map & black-level-compensate the driver image so highlights don't blow out before the model sees them | **Highest — fixes the cause** | Low (camerad, **not** panda safety); needs on-device camerad rebuild + snapshot verify |
| **B. Model** | A DM model retrained to be robust to glare / high dynamic range | High — fixes the cause | Low to apply (binary swap); opaque (can't be tuned, only swapped) |
| **C. Policy thresholds** | Tolerate uncertainty longer / loosen std cutoff in `helpers.py` | Low — band-aid, masks symptom | **Safety tradeoff** (genuine inattention looks identical to glare-uncertainty) |
| **D. Physical** | DM camera mount angle, visor/anti-glare, cabin geometry | Situational | None (no code) |

Address **A and B first**. C only if A+B are insufficient, and always toggle-gated + default OFF.

---

## 4. What commaai already shipped for glare (real-0.11.1) — the parts worth porting

Real-0.11.1 (shipped 2026-05-18; note comma bumps `version.h` *early*, so the version string ≠ the
shipped content — see §5) was a DM-focused release. Glare-relevant changes, by layer:

### A. Driver-camera image pipeline — `ca04b70d0a "camerad: driver camera BPS magic"` (2026-04-21)
Routes the driver camera through the full Qualcomm **BPS** ISP block instead of a rawer path. For the
driver image it adds:
- **black-level compensation** + a proper **linearization LUT** (`bps_linearization_lut`, with
  black-level folded in) — corrects sensor response at the bright/dark extremes,
- a **gamma / tonemapping LUT** (`bps_gamma_lut` / `sensor->gamma_lut_rgb`).

Gamma/tonemapping is exactly what compresses bright highlights so a sun-washed face is not clipped to
flat white. **This is the single most glare-relevant change upstream has made** and is the
"Improved image processing pipeline for driver camera" release-note line. Files:
`system/camerad/cameras/spectra.cc` / `spectra.h` / `cdm.{cc,h}` / `bps_blobs.h` / `nv12_info.h`.

### B. New DM model
- 0.11.0 shipped the **"Le Mans GT3"** model (`04dcdf46bc`, 2026-02-26, 7.31 MB).
- 0.11.1 shipped the **"sleep prob" model** (`2ed88a1dff`, 2026-05-17, 7.49 MB).
  (A "Lancia Delta HF Integrale" candidate `d8569b07eb` was added 2026-04-01 and **reverted**
  `2596de8543` 2026-05-06 — it is *not* the shipped model.)
- The retrained model is what governs `faceProb`/pose-`std`/`sunglassesProb` under glare. Opaque
  weights — can only be swapped wholesale, not tuned.

### C/diagnostic. `_DCAM_UNCERTAIN_RESET_COUNT` 20 s → 2 s — `4ecbdb0d7a` (2026-05-12)
Makes the *offroad* "DM camera uncertain" notice far less trigger-happy on transient glare (counter
resets after 2 s of clear vision instead of 20 s). **Diagnostic/alert only** — does not change the
live attention logic, so it won't stop the false "inattentive," only the nag.

### Plumbing (no glare behavior change)
`sleepProb` model output + `cereal/log.capnp` `sleepProb @14` (**logging-only**, not consumed by the
policy); `helpers.py` → `policy.py` rename; `_POSESTD_THRESHOLD` → `_HI_STD_THRESHOLD` (same 0.3);
`DriverMonitoringState` v2; DM alerts renamed to numbered stages.

**Notably *unchanged* in 0.11.1:** the live false-"inattentive" thresholds —
`_HI_STD_FALLBACK_TIME` (10 s), the std threshold (0.3), `faceProb` cutoff (0.7). So upstream's glare
improvement is entirely the better image (A) + better model (B), **not** policy tuning.

---

## 5. Where your fork stands (the catch)

`xnor/openpilot` and `pnw/pnw-pilot` carry the **0.11.1 version string but a *pre*-0.11.1 DM state** —
they were branched early in the cycle (right after the 2026-03-10 bump), *before* the DM work landed
(2026-04-21 … 2026-05-17). Verified:

| Signal | your xnor base | finished real-0.11.1 |
|---|---|---|
| DM file | `helpers.py` | `policy.py` |
| `_DCAM_UNCERTAIN_RESET_COUNT` | 20 s | 2 s |
| `sleepProb` in `log.capnp` | **absent** | present |
| driver-cam BPS gamma pipeline | partial / differs | full |
| DM model | pre-sleep-prob | sleep-prob model |

**You have neither upstream glare fix (A or B).** That is the most important takeaway: before tuning
any thresholds, the high-leverage move is to bring the **BPS driver-cam pipeline** and the **new DM
model** into the xnor base.

---

## 6. Recommended plan (priority order)

### Step 0 — Measure first (always)
Collect a real glare drive, then inspect `driverStateV2` / `driverMonitoringState` (§2 "Diagnosing").
Decide whether you're exposure-limited (`faceProb` dropping) or model-uncertainty-limited (`faceProb`
fine, `std` high). This decides whether A or B is the lever and gives a **before** baseline to prove
any change. Don't tune blind.

### Step 1 — Port the 0.11.1 driver-camera BPS pipeline (Layer A, highest leverage)
- Port `ca04b70d0a` into `xnor/openpilot/system/camerad/` (spectra ISP + gamma/BLC/linearization).
- **Feasibility/safety:** camerad is **not** panda safety — safe to change. The 3X driver sensor is
  `ox03c10` (not the comma-four `os04c10` that most of the surrounding 0.11.1 sensor churn targets);
  confirm the `ox03c10` path provides `gamma_lut_rgb` / `linearization_lut` the BPS code expects.
- **Deploy:** this is a C++/ISP change → needs an **on-device camerad rebuild**
  (`PATH=/usr/local/venv/bin:$PATH scons -u -j$(nproc)`), not just a file overlay. Use a
  `patch-glare-bps.py` + `update-glare-bps.sh` pair (stage+md5, detached, DONE/FAIL sentinel) per the
  workbench deploy convention.
- **Verify:** `system/camerad/snapshot.py` of the **driver camera** before/after under the same glare;
  re-check `faceProb`/`isLowStd`/`hiStdCount` on a glare drive vs the Step-0 baseline.

### Step 2 — Swap in the 0.11.1 DM model (Layer B)
- Bring `selfdrive/modeld/models/dmonitoring_model.onnx` (the `2ed88a1dff` sleep-prob model) plus the
  matching `dmonitoringmodeld.py` parse changes into the xnor base.
- If you also want the `sleepProb` field populated, add `sleepProb @14` to `cereal/custom.capnp`-style
  handling — **but** note the upstream policy does **not** consume it, so it's optional/logging-only.
- **Risk:** model + parser must match (tensor names/outputs). Validate with `dmonitoringmodeld`
  running on-device and a sane `driverStateV2` stream before trusting it.

### Step 3 — Only if A+B are insufficient: a gated `glare2xnor` policy relaxation (Layer C)
Treat as a **band-aid**, default **OFF**, toggle-gated. Anchor-based patch into the existing DM
settings (mirror how `dmon2xnor` layers in). Candidate knobs in `DRIVER_MONITOR_SETTINGS`
(`helpers.py:34-60`), in order of preference:

| Knob | Default | Glare-relaxed candidate | Effect |
|---|---|---|---|
| `_HI_STD_FALLBACK_TIME` | `int(10 / DT_DMON)` | e.g. `int(20 / DT_DMON)` | waits longer before the wheel-touch fallback when uncertain — **most direct** for the symptom |
| `_POSESTD_THRESHOLD` | `0.3` | e.g. `0.35–0.4` | tolerates more pose uncertainty before "high std" |
| `_FACE_THRESHOLD` | `0.7` | (avoid lowering) | lowering keeps a face "detected" in washed-out frames — **riskiest**, easy to mask real inattention |

Wiring: add a param key (e.g. `GlareTolerantDM`) to `common/params_keys.h` (MERGE — preserve other
features' keys), a toggle in `selfdrive/ui/layouts/settings/toggles.py`, read it in
`DriverMonitoring.__init__`, and select relaxed vs default settings. Rebuild
`common/params_pyx.so` on-device. **Behavior-neutral when OFF.**

### Step 4 — Physical (Layer D, no code)
Re-check DM camera mount/angle (a steeper down-angle reduces direct sun ingress), and whether a small
cabin shade/visor helps the worst west-bound stretches. Cheap, immediate, no deploy.

---

## 7. Safety (non-negotiable)

- DM is a **safety feature**. Glare-uncertainty and genuine inattention look **identical** to the
  std/face signal — every Layer-C relaxation widens the window in which a truly-distracted driver is
  not caught. Keep any policy change **toggle-gated, default OFF**, and prefer A/B (which improve
  *perception* rather than *suppress the alert*).
- **Never** touch panda safety for this — none of A/B/C/D should. (They don't.)
- The `helpers.py` header carries comma's "disabling/nerfing DM gets you banned from our servers"
  notice. This fork uploads to **self-hosted S3**, so the server-ban does not apply — but the
  *safety* reasoning does. Document the tradeoff on the branch.
- Follow the workbench deploy rules: persistence guards (`prebuilt`, `.overlay_init`,
  `DisableUpdates=1`, clear `safe_staging/finalized`), connection-loss-safe detached deploy, validate
  against `DEVICE-STATE.md` before/after, rebuild the right artifacts on-device (camerad for Step 1,
  `params_pyx.so` for Step 3), and clear pyc in **both** the top-level and nested `openpilot/`
  trees.

---

## 8. Code reference (xnor base `selfdrive/monitoring/helpers.py`)

| What | Line(s) |
|---|---|
| `DRIVER_MONITOR_SETTINGS` thresholds (`_FACE_THRESHOLD`, `_SG_THRESHOLD`, `_POSESTD_THRESHOLD`, `_HI_STD_FALLBACK_TIME`, `_DCAM_UNCERTAIN_*`) | `34-79` |
| `low_std` computed from `faceOrientationStd` | `283-286` |
| sunglasses gate on blink | `287-290` |
| distraction requires `faceProb > 0.7 and low_std` | `294-297` |
| per-drive DCAM-uncertain accounting | `309-318` |
| `is_model_uncertain` / `hi_stds` accumulation | `319-324` |
| `maybe_distracted` (the false-"inattentive" path) | `369-376` |
| `Offroad_DriverMonitoringUncertain` raise | `395-397` |
| `driverMonitoringState` packet (debug fields) | `400-421` |

Camera ISP (Step 1 target): `system/camerad/cameras/spectra.cc` `config_bps()` / `configICP()`.
DM model + parse (Step 2 target): `selfdrive/modeld/models/dmonitoring_model.onnx`,
`selfdrive/modeld/dmonitoringmodeld.py`.

---

## 9. Appendix — release/model lineage (commaai master)

Ship dates (`RELEASES.md`): 0.11.0 = 2026-03-17, 0.11.1 = 2026-05-18, 0.11.2 = 2026-06-15.
`version.h` bumps are early-cycle bookkeeping and do **not** mark ship points; key off ship dates.

DM model lineage:
- `04dcdf46bc` Le Mans GT3 (2026-02-26) — shipped in **0.11.0**
- `d8569b07eb` Lancia Delta HF Integrale (2026-04-01) — added then **reverted** `2596de8543` (2026-05-06)
- `2ed88a1dff` sleep-prob model (2026-05-17) — shipped in **0.11.1**

Glare-relevant 0.11.1 commits: `ca04b70d0a` (driver-cam BPS pipeline), `2ed88a1dff` (model +
sleepProb), `4ecbdb0d7a` (`_DCAM_UNCERTAIN_RESET_COUNT` 20→2), `6996e87f8d` (helpers→policy rename).

**0.11.1 → 0.11.2 DM delta: essentially none** (the post-bump window touched only `bump msgq` /
`usbgpu` in camerad; no DM model or monitoring-logic change; 0.11.2 release notes are empty).

Related: `DMON2XNOR.md` / `DMON.md` (relaxed timeouts — orthogonal), `DEVICE-STATE.md` (deploy
validation), `BSM2XNOR.md` (other carstate-level DM-adjacent work).

---

## 10. DEPLOYED 2026-07-06 — Layer C applied on `4devpnw` (interim band-aid)

Triggered by a live I-82 westbound low-sun drive that reproduced the symptom cleanly ("DM going
mad"). Root cause confirmed on the road matches §2 case #2: side/back sunlight washes the DM image →
`faceOrientationStd` spikes → after just **10 s** `is_model_uncertain` flips True → monitoring drops
**OUT of the relaxed dual-counter ACTIVE mode** (the 3h-pose / 1h-phone relaxation) **INTO stock
PASSIVE wheel-touch mode** → false "distracted" nag. This is precisely why prior DM relaxation
(`dmon2pnw`) never helped glare: the relaxed timeouts only apply *in* active mode, so a glare burst
bypasses them entirely by kicking DM into passive mode first.

**What shipped** (`selfdrive/monitoring/helpers.py`, three knobs, before → after):

| Knob | Before | After | Why |
|---|---|---|---|
| `_POSESTD_THRESHOLD` | `0.30` | **`0.45`** | tolerate more pose uncertainty before a frame counts as "high std" |
| `_HI_STD_FALLBACK_TIME` | `10 s` | **`30 s`** | wait 3× longer before the passive wheel-touch fallback — keeps DM in active/relaxed mode through a glare burst |
| `_DCAM_UNCERTAIN_RESET_COUNT` | `20 s` | **`2 s`** | ports commaai `4ecbdb0d7a`; the offroad `Offroad_DriverMonitoringUncertain` nag clears faster (diagnostic only — **not** live attention logic) |

`_FACE_THRESHOLD` left at **0.70** — lowering it is the riskiest knob per §3/§6 (masks real
inattention in washed-out frames), so the trade was kept modest.

- **Applied UNGATED** (no param toggle), consistent with how the `dmon2pnw` relaxation was applied —
  this is a deliberate deviation from §6 Step 3's "toggle-gated, default OFF" recommendation. The
  safety tradeoff (a slightly wider inattention window) is inherent to Layer C and was **explicitly
  requested by the driver** ("less aggressive, if in doubt").
- **Commit:** `2fc78f0fbd` on `4devpnw`. **Gemini-reviewed** (gemini-pro-latest, no issues).
- **Verified live on device:** `POSESTD=0.45`, `HI_STD_FALLBACK=30 s`, `DCAM_RESET=2 s`, `FACE=0.70`.

**This is the interim band-aid, not the fix.** Layers **A** (the 0.11.1 driver-camera **BPS** ISP /
gamma pipeline, §4.A / §6 Step 1) and **B** (the 0.11.1 **sleep-prob DM model**, §4.B / §6 Step 2)
remain the real upstream fixes and are **still NOT ported** into the xnor/pnw base. Layer C only
lengthens the tolerance window; it does not improve what the camera/model actually *see* under glare.

---

## 11. DEPLOYED 2026-07-06 (evening) — Layer C round 2: the NO-FACE path

§10's knobs were **verified working the same day** (qlog scan, routes `00000092`/`00000093`:
`hiStdCount` ≈ 0 all hour) — but the driver still got "touch steering wheel" under direct side sun.
The remaining trigger is §2's *other* sub-case, **face loss**, plus a ratchet bug:

1. **Zero debounce on face loss.** `faceProb` washes to 0.27–0.45 under direct side sun (model
   half-sees the face; threshold 0.7). ONE sub-threshold frame flipped
   `_set_timers(face_detected …)` → instant PASSIVE wheel-touch mode. Log signature:
   `faceLost% == passive%` frame-for-frame in every segment.
2. **`awareness_passive` never recovers** — the fork's dual-counter ACTIVE branch returns early
   without restoring it (and in passive mode recovery requires a detected face — contradiction).
   Each glare burst resumed the countdown where the last left off: minAw 0.71 → 0.32 → 0.19 across
   consecutive segments; 0.14 at the 10:18–10:21 PT episode (preDriverUnresponsive ×60,
   promptDriverUnresponsive ×20).

**Fix (`8a692f274d`, deployed with hotfix `86182b1496` on `4devpnw`):**
- `_FACE_LOST_GRACE_TIME = 30 s`: `face_detected_deb` debounces face LOSS for mode-switching +
  passive awareness decay ONLY. Raw `face_detected` still gates all distraction detection, pose
  calibration, and telemetry. The grace starts EXPIRED until a face is first seen in the drive.
- `awareness_passive` recovers at `DT/30 s` while fully attentive in active mode (both counters
  at 1.0) — kills the ratchet.

Safety framing unchanged from §10: ungated, driver-requested ("less aggressive, if in doubt"),
and the debounce widens the unmonitored window by ≤30 s per burst. Layers A/B still unported —
this base runs the 0.11.0 "Le Mans GT3" DM model (7.31 MB, verified via LFS pointer) and lacks
the BPS gamma/tonemap LUT for the driver cam (linearization present, gamma absent).

**Deploy postmortem note:** the same deploy briefly crash-looped the UI onroad (overlay layout
cache stored on `self._layout`, which shadows the raylib `Widget._layout()` method — fixed
`86182b1496`); the respawn churn starved pandad/selfdrived and produced phantom "panda error"
alerts. Render-path bugs are invisible to the parked offline import test — check new widget
attributes against `Widget` base-class names before deploying UI changes.
