# op-long-features — what each car can do with longitudinal control ON vs OFF

**Audience:** the driver, deciding how to configure a drive. **Scope:** every PNW feature, mapped to
the three real operating configurations of the fleet — the **Tesla Model S (Raven, HW3)** and the
**Ford F-150 Lightning** with **Alpha Longitudinal ON** or **OFF**. Written 2026-07-18 against the
deployed channel tip (`speedlimitdebug2pnw` = `origin/3devpnw`, incl. `radarless2pnw` `a46d8749b8`).

---

## 1. How to read this — what "Alpha Long" actually switches, per car

The UI toggle **"openpilot Longitudinal Control (Alpha)"** (param `AlphaLongitudinalEnabled`) means
something completely different on the two cars:

### Tesla Model S Raven (HW3)

**openpilot longitudinal is the native and only mode.** The legacy Tesla path
(`opendbc/car/tesla/interface.py :: _get_params_sx`) sets `openpilotLongitudinalControl = True`
**unconditionally** — it does not consult the alpha toggle at all. There is no "stock ACC" to fall
back to: on the Raven install the Autopilot computer's driving role is taken over by the comma
device, so if openpilot isn't doing gas/brake, nobody is (cruise simply isn't available).
Practical consequence: **the Alpha-Long toggle is a no-op on the Tesla** — the full CES / VTSC /
MTSC / speedadjust stack is always the longitudinal system. The Raven also **has a real radar**
(Continental ARS4-B; `radarUnavailable` is only set for HW2), so leads come from Kalman-filtered
radar tracks, not raw vision.

### Ford F-150 Lightning

The Lightning **has no radar openpilot can read** (`radarUnavailable = True` is structural — no
radar bus in its DBC; the truck's own ACC radar is internal to Ford and not exposed). Because of
that, `alphaLongitudinalAvailable = ret.radarUnavailable` → the alpha toggle exists, and it is the
**master A/B switch** between two entirely different longitudinal regimes:

| | **Alpha Long ON** | **Alpha Long OFF** |
|---|---|---|
| Who does gas/brake | **openpilot's own MPC**, every tick, in **BOTH** CES chill and experimental modes (`ford/carcontroller.py` sends openpilot's accel whenever engaged — chill does **not** hand following back to Ford) | **The truck's stock ACC** (Ford's internal radar) does all following, braking, stop-and-go |
| Lead source | **vision-only** (model leads via `radard.py`; now Kalman-filtered by `radarless2pnw` — just shipped, unvalidated) | Ford's internal radar (invisible to openpilot; openpilot's vision lead feeds telemetry/shadow only) |
| CES | full actuating stack (mode switching, VTSC/MTSC caps) | **SHADOW** — decisions/telemetry/overlay only; the planner never actuates |
| Curve control | VTSC/MTSC caps on `v_cruise` (regen-coast decel, friction when needed) | **ICBM only** — SET−/SET+ button taps (0x083) steer the stock ACC *set speed* toward a curve target; Ford does the actual braking |
| Gap button on the wheel | **repurposed silently**: each release decrements `LongitudinalPersonality` (mod 3, `selfdrived.py:457-461`) with **zero dash feedback** — a known gotcha | works **natively** on stock ACC (Ford's own 4-bar gap display) |

⚠️ **Stale-session trap:** the persistent CarParams the UI reads lags one session behind the toggle.
The overlay shows a red **LONG MISMATCH** warning for the dangerous "toggle ON but this session is
still shadow" state; UI capability views take the live toggle. The switch takes effect at the next
fingerprint (ignition cycle).

### Vocabulary (this fork's core distinction)

- **actuating** — the feature commands the car (gas/brake/set-speed/steering).
- **shadow** — the feature runs and logs/decides, but never commands anything (validation mode).
- **display-only** — UI/sound only; zero control surface.
- **n/a** — structurally unreachable on that car/config (usually pinned neutral by
  `pnw_vehicle.py`, provably byte-identical for the other car).

### The capability flags behind the matrix (`pnw_vehicle.py`, both layers)

| Flag | Tesla | Lightning ON | Lightning OFF | Meaning |
|---|---|---|---|---|
| `op_long` | **True** (always) | True | False | openpilot owns gas/brake |
| `stock_acc_buttons` | False | True | True | SET± tap steering possible (ICBM executor exists) |
| `ces_shadow` / `icbm` | False | False | **True** | CES shadow + ICBM is the actuator |
| `ces_capable` | True | True | True | CES can act at all (planner or ICBM) |
| `nudgeless` | True | True | True | nudgeless lane change supported (BSM-gated) |
| `lightning_curve_slow` | False | True | True | extra curve-speed penalty (weak EPS) |
| `gentle_launch` | False | True | (True but moot — op-long-path code) | soft standstill-launch accel ramp |
| `tight_aggressive_follow` | False | **False (reverted 07-16)** | False | 1.0 s Aggressive T_FOLLOW — disabled on data, plumbing kept |
| `four_signal_lat` (opendbc) | False | True | True | BP 4-signal lateral (lateral is independent of Alpha Long) |
| `pc_blend` / `ht_reset` (opendbc) | False | True | True | Ford lateral refinements (embedded in the 4-signal path) |
| `bp_long_follow` (opendbc) | False | **True** | False | BP LongitudinalExt highway follow shaping (op-long only) |

---

## 2. THE MATRIX

Legend: ✅ actuating · 👻 shadow · 🖥 display-only · ➖ n/a (structurally inert / pinned neutral).
Rows marked **[long-independent]** behave the same regardless of Alpha Long.

### 2a. Longitudinal core

| Feature | Tesla Raven | Lightning — Alpha ON | Lightning — Alpha OFF |
|---|---|---|---|
| **openpilot longitudinal (MPC gas/brake)** | ✅ native, the **only** mode (toggle ignored) | ✅ owns the pedal in **both** CES chill & experimental | ➖ stock ACC follows |
| **Stock ACC following** | ➖ does not exist (AP computer replaced) | ➖ overridden — openpilot's accel is sent every engaged tick | ✅ native (Ford radar, stop-and-go, tight ~1.43 s gap) |
| **Experimental mode (end-to-end long)** | ✅ available (forced via button or CES) | ✅ available | ➖ no op-long → no e2e; button cycles **CES↔Chill only** |
| **CES — conditional experimental switching** (`CESMode` 0/1/2 master) | ✅ actuating (chill↔experimental for curves/stops/low-speed/slow-lead) | ✅ actuating | 👻 shadow — full decision pipeline runs for telemetry/overlay; its curve targets drive ICBM. `CESMode=0` kills ICBM too |
| **CES 3-state top-right button** | ✅ CES / Chill / Exp | ✅ CES / Chill / Exp | ✅ CES ↔ Chill only (no forced Exp) |
| **VTSC — vision curve speed cap** | ✅ caps `v_cruise` (decel-limited, reduce-only) | ✅ same, plus Lightning overspeed→friction escalation (`overspeed_margin`) | 👻 as a cap; vision **candidates feed ICBM** but only beyond map reach AND with ≥ 2.75 s to act |
| **MTSC — map-curve braking** (`VtscMapCurves`, default **ON**) | ✅ folded into VTSC (earlier braking, blind curves) | ✅ same | ✅ **map-first is ICBM's primary authority** (500 m horizon, far-map firm-decel candidate, map-scale cap 1.35 + Lightning 0.92 OSM discount) |
| **Sharp-curve full-horizon + regen-coast; twisty-descent trim; ≥1 mph curve cue; highway speed-limit floor** | ✅ (rides `CESMode`) | ✅ | ➖ (VTSC-cap machinery; ICBM has its own analogs: far-map candidate + apex-timed guarded restore) |
| **ICBM — stock-ACC SET± curve slowdown** | ➖ shadow-gated AND pinned neutral (unreachable, bit-identical Tesla behavior proven by test) | ➖ structurally inert (`icbm = buttons AND NOT op_long`) | ✅ the **only** actuator: dec-only taps + latched-ceiling guarded SET+ restore, continuous set-tracking, in-curve suppression, full abort matrix |
| **speedadjust — auto speed reduce** (`AutoSpeedReduce` 0/1/2: police / police+limits) | ✅ caps `v_cruise` (reduce-only, slewed) | ✅ | ➖ returns `v_cruise` unchanged without op-long; UI selector greys out |
| **Lightning curve-speed penalty** (curveslow hump + descent gain + left-curve factor, `/data/pnw/curve.json`) | ➖ returns 0.0 (byte-unchanged Tesla path) | ✅ applied to VTSC/MTSC targets | ✅ applied to ICBM targets (same shared formula — cross-subsystem parity pinned by test) |
| **Rain mode** (`RainMode` 0/1/2, ~3/5 mph, `/data/pnw/rain.json`) | ✅ subtracted from VTSC curve targets — **same reduction both cars** | ✅ VTSC path | ✅ ICBM path (tier pushed live ~1 Hz, mid-drive changes work) |
| **Red-light stop-hold guards / standstill latch / stop-intent fast path / evidence-gated pull-away** | ✅ (these fixed the Tesla red-light lurch) | ✅ | 👻 decisions logged only — stock ACC handles stops natively |
| **Gentle standstill launch** (standstillsoft, 0.6→2.5 m/s² ramp to 9 mph) | ➖ (`gentle_launch=False`; Tesla launches smoothly) | ✅ planner accel-clip + output bind (rolling-stop jolt fix incl.) | ➖ code lives in the planner path; stock ACC launches itself |
| **Ford highway follow — BP LongitudinalExt** (gaining/pacing/trailing, >50 mph, `bp_long_used`-gated builder) | ➖ | ✅ actuating with a lead at highway speed; stock builder otherwise (city-hop fix) | ➖ inert (proven by test) |
| **fordregen Fix A (long PID damping) + Fix B (regen-bite gas compensation)** | ➖ | ✅ deployed via opendbc pins `1ea6638d`/`3f6fd4b0` — ⚠️ `FORDREGEN2PNW.md` header still says "design only"; the pins supersede it; **live validation pending** | ➖ (op-long actuation-layer only) |
| **tightfollow (1.0 s Aggressive T_FOLLOW)** | ➖ never applied | ➖ **REVERTED 07-16 on data** (gap got *looser* 1.84→1.92 s, hunting 1.2–3.8 s, aEgo σ 0.221→0.347); flag False, plumbing kept for v2 (~1.15 s, stability-gated, post-filter) | ➖ |
| **radarless2pnw — VisionLeadFilter (KF on vision lead vLead/aLeadK)** | (present but leads come from radar; vision path is only the dropout fallback — unchanged in normal operation) | ✅ filters the truck's **only** lead path into the MPC — **just shipped 07-18, NOT yet road-validated** | runs, but its lead feeds telemetry/shadow only (Ford ACC uses its own radar) |
| **leadloss — lead-dropout hold** | 👻 shadow (log-only) everywhere; the trigger pattern is a vision-lead artifact so it's effectively a Lightning-ON concern (retuned 07-15: ≤50 m, track-stability gate) | 👻 shadow — actuating version blocked on a re-validated shadow pass | 👻 log-only |
| **Follow gap / personality** | `LongitudinalPersonality` via **Settings only** (follow-distance stalk 0x382 reverse-engineered but **postponed** — `STALK.md`) | personality governs `get_T_FOLLOW()`; ⚠️ **gap button silently cycles it** (no dash feedback — the 07-16 "grandmother gap" was accidental Relaxed); on-screen indicator is on PENDING-WORK | Ford's own 4-tier gap button, native display; measured avg **1.43 s** follow |
| **Green Light Alert + "car ahead is leaving"** [long-independent] | 🖥 always on (no toggle, independent of `CESMode`); lead-departure ding gated on driver attention | 🖥 same | 🖥 same — works on stock ACC too (standstill detector reads carState) |

### 2b. Lateral (orthogonal to Alpha Long — identical in both Lightning columns)

| Feature | Tesla Raven | Lightning (either long mode) |
|---|---|---|
| **Lateral control** | ✅ angle control via `tesla_legacy` panda safety (strong EPS — holds fast sweepers) | ✅ **BP 4-signal lateral** (curvature + rate + path_offset + path_angle) with the hardened custom panda safety (reflash-deployed 07-11; reset-latch hole fixed). Fallback to stock curvature-only on any exception, proven by fault injection |
| **Predicted-curvature blend / human-turn reset** | ➖ | ✅ embedded in `LateralCurvExt` (turn-exit wind-up fix; no post-override lurch) |
| **Nudgeless lane change** (`NudgelessLaneChange`, default OFF) | ✅ supported, **BSM-gated** (blindspot blocks/resets the hold timer) | ✅ same |
| **BSM source** | ✅ Raven `AutopilotStatus`/`DAS_status` 0x399 explicit parser (bsm2pnw; walk-by flip test still pending) | ✅ Ford native BLIS (`Side_Detect_L/R_Stat`) |
| **No disengage on brake** (`NoDisengageOnBrake`, default OFF) | ✅ | ✅ under op-long; with Alpha OFF a brake press also cancels the stock ACC itself (Ford behavior — openpilot can't override that; **verify** exact interaction on the truck) |
| **Ford lateral status indicators** (CALIBRATING x% / STEER: STOCK / NO 4SIG PANDA) | ➖ | 🖥 surfaces silent 4-signal fallback or panda-flash mismatch |
| **Confidence ball / dark-cockpit CES overlay / LONG MISMATCH warning** | 🖥 | 🖥 (LONG MISMATCH is Lightning-specific — flags the stale-session shadow state) |

### 2c. Long-independent platform features (same in all three columns)

| Feature | Notes |
|---|---|
| **Driver monitoring** — `DmMode` Default/Highway(road-gated via mapd)/Relaxed(hidden until CLI-unlocked), dual-counter architecture, `/data/pnw/dm.json` magnitudes, glare Layer-C knobs | identical on both cars; independent of who owns gas/brake |
| **mapd / OSM** — speed-limit + road-name display, PNW auto-download (WA/OR/ID), persistent binary + self-heal watchdog, NaN guards, OSM-limit change flash (speedlimitdebug) | display + data source for MTSC/ICBM/speedadjust/DM road gate |
| **"Happening Ahead"** — police (keyless Waze proxy), rest areas, EV chargers | display-only daemon |
| **Uploads / networking** — two-pass, firehose, defer-HD, tethering, captive portal, locator | no control surface |
| **Auto-recalibrate on car swap, fingerprint hardening, card crash-restart, auto-update channel** | infrastructure, both cars |

---

## 3. Platform pros & cons

### Tesla Model S Raven (HW3) — native op-long, radar, strong EPS

**Pros**
- **Real radar leads** (Continental ARS4-B → Kalman-filtered tracks): stable following, none of the
  vision-lead noise class that sank tightfollow on the truck.
- **Full stack always on** — CES/VTSC/MTSC/speedadjust/stop-and-go with no toggle decision to make;
  the primary car the whole distribution is tuned on (I-5/I-90 calibration drives).
- **Strong EPS**: no curve-speed penalty needed (`curve_speed_penalty_ms` returns 0.0); carries fast
  sweepers the Lightning washes out of.
- Smooth launches (no gentle-launch cap needed); BSM via the Autopilot computer's own rear
  blind-spot output (0x399).
- Behavior is the fork's most-proven configuration — every CES stop/lurch fix was validated here.

**Cons**
- **No fallback**: openpilot longitudinal or nothing — a plannerd crash means no cruise at all (this
  is why the 07-16 capnp-enum crash mattered; card/ui-style restart policies exist, plannerd's did
  not save that drive).
- **No physical gap/personality control**: the follow-distance stalk (0x382) is reverse-engineered
  but **postponed** — personality changes mean the Settings screen.
- CES→Experimental can over-brake long summit curves (the Snoqualmie 55-in-a-65 finding) —
  mitigated by **`CESMode=1` (Light)**, which hands curves entirely to VTSC.
- Legacy platform: Tesla changes stay out of upstream; `tesla_legacy.h` is ours to maintain.

### Lightning, Alpha Long ON — the full openpilot stack, on vision-only leads

**Pros**
- Everything the Tesla gets: CES mode switching, VTSC/MTSC curve braking with real decel authority,
  speedadjust, stop-hold/standstill guards, gentle launch, plus BP highway follow shaping and the
  regen Fix A/B damping.
- The only Lightning mode with **openpilot stop-and-go** and Experimental (e2e) driving.
- Curve handling is the complete envelope: regen-coast early slowdowns, friction escalation on
  overspeed, descent/left-curve penalties.

**Cons**
- **Vision-only leads** — the structural weakness: raw per-frame model noise amplified ~10× into
  obstacle-distance swings at highway speed. Measured: avg gap **~1.84 s** (vs stock 1.43 s), and the
  driver's verdict "stock cruise is much smoother". `radarless2pnw` (KF filter) shipped 07-18 to fix
  exactly this — **unvalidated on the road as of this writing**.
- **The gap button is a trap**: it silently cycles `LongitudinalPersonality` downward with zero
  feedback — you can end up in Relaxed ("grandmother gap") or Aggressive without knowing. On-screen
  indicator is pending.
- EV regen bite made lead-following jolty (over-decel → re-throttle oscillation); Fixes A/B are
  deployed but still awaiting live validation.
- Weak EPS: needs the curve-speed penalty (up to ~5 mph mid-speed, more downhill/left) — the truck
  is deliberately slower through curves than the Tesla.
- One-session lag on the toggle (LONG MISMATCH warning exists for a reason).

### Lightning, Alpha Long OFF — stock ACC + ICBM (the shadow arm)

**Pros**
- **Ford's radar-based ACC**: genuinely tight (measured **1.43 s** average), smooth, native
  stop-and-go, no vision-lead noise — currently the best *following* experience on the truck.
- Gap button works natively with real dash feedback.
- **ICBM still gives curve safety**: map-first 500 m anticipatory SET− walk-downs, dec-only with a
  guarded restore that can never exceed the driver's own set, continuous set-tracking so a lane
  change can't launch the truck into a bend. Curve penalty, descent/left factors, and rain all
  apply through the same shared math.
- CES runs in shadow the whole time — every drive still produces full decision telemetry.

**Cons**
- Curve control is **limited to set-point steering**: ICBM can only move the ACC set speed in 1 mph
  taps (0.4 s cadence); it has no brake authority and can't help once a curve needs more than the
  stock ACC's own decel. No VTSC decel-limited cap, no regen-coast shaping.
- **No Experimental mode, no openpilot stop assistance, no speedadjust** (police/limit caps return
  unchanged without op-long), no gentle-launch shaping (moot — stock ACC launches).
- CES stop/standstill intelligence is spectator-only.
- The stock ACC has zero curve/map awareness of its own — between ICBM episodes it will happily hold
  90 into a bend (the 07-12 19:58 lane-change event that motivated set-tracking).

---

## 4. Which should I drive with? (grounded in this week's measurements)

**Tesla:** there is no choice to make — op-long is the platform. Recommended posture per the I-90
analysis: **`CESMode=1` (Light)** on summit/long-curve routes so curves go to VTSC/MTSC rather than
e2e Experimental braking; Standard (`CESMode=2`) where stop-light behavior matters more.

**Lightning — today (2026-07-18):**
- **For dense highway traffic where following quality dominates: Alpha Long OFF.** The measured
  facts favor it: 1.43 s vs 1.84 s average gap, audibly smoother accel, native gap button, and
  people stop cutting in. ICBM + curve penalty + rain still cover the curve-safety envelope within
  stock-ACC limits.
- **For twisty / descent-heavy routes (I-90 passes, rural two-lane): Alpha Long ON.** Only op-long
  brings real decel authority into curves (MTSC early braking, regen-coast, friction escalation,
  descent/left penalties at full strength) plus stop-and-go and Experimental. Accept the looser,
  noisier following; keep Aggressive selected and **don't touch the gap button** (or check
  personality in Settings after any press).
- **Re-evaluate after the next Alpha-ON validation drive**: `radarless2pnw` is expected to remove
  the gap hunting and much of the accel roughness (it attacks the measured root cause — unfiltered
  vLead noise into the MPC), and tightfollow v2 (~1.15 s, stability-gated) is queued behind it.
  If the filter delivers, Alpha ON should close most of the smoothness/tightness deficit and become
  the default recommendation. Until measured: treat it as **just-deployed / unvalidated**.
- Whatever is chosen: `CESMode` must be ≥ 1 for ANY curve feature (it is the master for both VTSC
  and ICBM), and expect the toggle to take effect only after an ignition cycle (watch for the red
  LONG MISMATCH warning if in doubt).

---

## 5. Footer

**Date:** 2026-07-18. **Code state:** pnw-pilot `speedlimitdebug2pnw` @ `a9bb7cf09f`
(= `origin/3devpnw` channel tip; radarless2pnw `a46d8749b8` shipped, tightfollow reverted
`8fff658a81`), pnw-opendbc `master-pnw` @ `3f6fd4b0`.

**Sources (ground truth, in order of authority):**
- `selfdrive/controls/lib/pnw_vehicle.py` + `opendbc_repo/opendbc/car/pnw_vehicle.py` — the
  capability views (every flag in §1's table).
- `opendbc_repo/opendbc/car/tesla/interface.py` (`_get_params_sx`: unconditional op-long, HW3 radar)
  and `opendbc_repo/opendbc/car/ford/interface.py` (`alphaLongitudinalAvailable = radarUnavailable`).
- `selfdrive/selfdrived/selfdrived.py` (gap-button personality cycling under
  `CP.openpilotLongitudinalControl`).
- `docs/pnw/`: `ICBM2PNW.md`, `FORDLONG2PNW.md`, `FORDSAFETY2PNW.md`, `FORDREGEN2PNW.md`
  (header stale — Fixes A/B shipped via submodule pins), `CURVESLOW2PNW.md`, `CES_I90.md`,
  `DM-CURRENT.md`, `MAPD-SYSTEM.md`, `CHANGELOG-2026-07-18.md`.
- `drives/2026-07-16/i5-nb-corvallis-to-puget-sound/DRIVE_REPORT.md` — the 1.43 s / 1.84 s gap
  measurements, the tightfollow revert data, the gap-button discovery.
- `PNW-PILOT-FEATURES.md` — the user-facing feature index.

**Note:** `~/gh/comma/docs/DEVICE-STATE.md` remains the param-registry / deployed-state source of
truth — validate any "deployed" claim here against it (and the live device) before relying on it.
Items flagged **verify** above (NoDisengageOnBrake × stock-ACC interaction) have not been
code-traced end-to-end and should be confirmed before being treated as fact.
