# Drive report — 2026-07-18 — Lightning: bp-7.0 ANGLE STEERING first activation (FAILED, sign-inverted)

**Car:** Ford F-150 Lightning · **Config:** op-long OFF (stock ACC + ICBM), `FordAngleLateral=1` (angle
steering ACTIVE), channel `3devpnw` @ `e282a2f29f`, opendbc pin `8796ceaf` (angle safety flashed).
**Road:** CLOSED road, no traffic (driver confirmed) — correct venue for an unvalidated control law.
**Raw data:** `angle_capture.jsonl` (100 Hz carState+carControl, this folder) · device copy
`/data/dirk/angle_capture_signproof.jsonl`.

## ⛔ RESULT: angle steering is SIGN-INVERTED. Disabled. Do not re-enable until the faithful re-port lands.

### Driver-observed
- Earlier public-road drive: left curve barely slowed then **oversteered out of the lane**; the same curve
  from the other side (right) **braked hard 40→15 mph** and then **pendulumed back and forth across the lane**.
- Controlled 25 mph closed-road pass: **left curve fine**, **right curve drifted slowly out of the lane** (wide).

### 🔬 THE PROOF (100 Hz capture, openpilot-in-control samples only, driver not overriding)
```
1115 / 1115 samples (100%)  commanded curvature sign OPPOSITE to actual steering direction
mean actual steering  = −20.3°      (right curve)
mean commanded curv   = +0.00534    (left)
magnitude tracked the curve correctly; only the SIGN was wrong
saturation ruled out: only 23/2687 samples pinned at a command limit
```
→ In a right curve it commanded **left**, so the truck ran wide. The asymmetry the driver felt
(left OK / right bad) is consistent: an inverted command plus driver correction reads very differently
depending on which way the road bends.

## 📚 Authoritative spec: `ALAN-POLK-SPEC.md` (this folder) — READ IT FIRST
Alan Polk's own published design, recovered 2026-07-19 from bluepilot.dev (articles archived verbatim in
`alan-polk-articles/`). Headline: **`path_angle = κ × v_ego × gain_factor` with c2 AND c3 set to ZERO** —
no lookahead, no VLT, no PID, **no negation**. Angle control is *simpler* than what we built. The spec doc
maps each of our deviations against his text.

## Root cause: OUR port deviated from Alan Polk's bp-7.0 implementation
The previous port (`angle2pnw`, opendbc `4e5ffd8d`/`8796ceaf`) made three control-law deviations, all
flagged in the build report as unvalidated, and shipped enabled anyway:
1. **ADDED a sign negation** — `carcontroller.py`: `self._shadow_curvature = -self._latext_angle.bp_kappa_cmd`,
   *inferred* from a bp-7.0 comment. **Prime suspect for the inversion.**
2. **DROPPED his VLT** (variable lookahead adapting to speed + curvature; `_VLT_T_EXTRA_MAX`,
   `_VLT_V_LOW_MS`=25 mph, `_VLT_V_HIGH_MS`=55 mph, `liveDelay`) → replaced with a **fixed 0.2 s** lookup.
   Best explanation for the earlier **oversteer/pendulum** (overshoot worsens with speed + curve tightness).
3. **OMITTED stall-blip / proactive-hand-off-blip** (`_STALL_*`, `_PRESS_BLIP_MIN_S`).
Also adapted: PSCM saturation clamp + exit-biased blend collapse (tuned around the fixed lookahead).

**Review-process lesson:** Fable+Gemini reviewed the **panda safety C** rigorously (can it command something
illegal) and passed it — but **nobody validated the control law**. Safety-C review ≠ control-law validation.

## 🎯 DRIVER DIRECTIVE (2026-07-18): port it 1:1
> "Implement angle steering exactly 100% as Alan Polk suggested it — I want to report back to him whether
> his stuff works. If we're not implementing it exactly like he did, we have no basis for discussion."

So the re-port must be **faithful**, with a **complete deviation manifest** for the report to Alan Polk.

## State / next steps
- **`FordAngleLateral` set to `0`** on device (takes effect at the **next ignition cycle / restart** — it was
  still live in-session when set).
- **In flight:** faithful 1:1 re-port on `angle2pnw-faithful` (off `master-pnw` in `pnw-opendbc`), restoring
  VLT + blips, replicating his exact sign handling, producing `ALAN-POLK-PORT-DEVIATIONS.md` (this folder)
  + a sign-convention trace from his controller to the CAN wire.
- **Permitted deviations only:** import/layout rewiring; MADS absent (`controls_allowed_lateral` →
  `controls_allowed`, rx hooks stripped); our panda reset-latch hardening + separate
  `FORD_PATH_ANGLE_LIMITS_ANGLE` struct. Everything else must be byte-faithful.
- **Do not enable** until the faithful port is reviewed AND re-validated on the closed road at low speed.

## ✅ Also validated this drive — ICBM curve onset FIX WORKS
The map-noise tune (`_map_v_sane`, shipped earlier today) fixed the late curve braking:
| Curve | entry → min | ICBM onset |
|---|---|---|
| 47.6637,-122.3716 | 38.7 → 18.8 mph | **+0.0 s** (at entry) |
| 47.6618,-122.3674 | 43.8 → 35.8 mph | +6.1 s (still late — known 46.5–58 m/s band gap) |
| 47.6637,-122.3709 | 29.3 → 17.2 mph | **+2.0 s** |

vs **+5–7 s late** before the fix. 2 of 3 curves now brake at/near entry. Remaining lateness is the
documented sub-58 m/s garbage band the sanity ceiling can't catch.
</content>
