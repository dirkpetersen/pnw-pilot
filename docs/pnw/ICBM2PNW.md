# ICBM2PNW — Intelligent Cruise Button Management (F-150 Lightning, stock ACC)

**Status: DEPLOYED, heavily evolved over 2026-07-11/12.** Promoted 2026-07-11 (dec-only v1); then a
weekend of field-driven rework — vision source + per-car penalty (`curveslow-lightning`), guarded
SET+ restore (`icbmrestore2pnw`), map-first start policy (`icbmmapfirst2pnw`), continuous
set-tracking + map-scale cap (`icbmtrack2pnw`), descent/left parity (`icbmalign2pnw`) — **all merged
to `3devpnw`** (tip incl. `14c8ea1968`, 2026-07-12 evening). Executor in pnw-opendbc `master-pnw`
(`icbm_pnw.py`, restore `808f5733`/`eccb3863`). Born from
`drives/2026-07-10/lightning-first-drive/`; reworked from `drives/2026-07-11/lightning-icbm-nofire/`
and `drives/2026-07-12/{snoqualmie-ellensburg-icbm,ellensburg-snoqualmie-westbound}/`.

## What it does

Gives the Lightning curve slow-downs on **stock ACC** by tapping the truck's own **SET−/SET+**
buttons over CAN (0x083, already TX-allowlisted — zero panda change), steering the stock set speed
toward a curve apex target from the shared VTSC curve math. Ford's radar'd ACC keeps ALL
braking/accel authority — we only move its set point, exactly as the driver's finger would.

## The A/B switch (unchanged)

- **`CESMode`** is the master on both cars (Off = everything off). ICBM publishes only in the CES
  button state; forced Chill publishes `{}` and clears the ceiling latch (`db8b391368`).
- **Alpha Longitudinal ON** → op-long actuates (CES/VTSC full stack); ICBM structurally inert.
- **Alpha Longitudinal OFF** → CES runs in shadow and the ICBM brain+executor steer the stock set.
  Without op-long the top-right button cycles **CES↔Chill only** (no forced Exp).
- Tesla: unreachable (shadow-gated) AND pinned neutral through the pure penalty path; the full
  3-state button cycle stays bit-identical there.
- ⚠️ Stale-session trap: the UI's persistent CP lags a session after toggling Alpha Long —
  UI contexts now pass the LIVE toggle (`d6ba8f10e8`), and the overlay flags the dangerous
  "toggle ON but session still shadow" state as a red **LONG MISMATCH** warning (`e694ca58bb`).

## Candidate sources & priority (the 2026-07-12 rework)

1. **MAP-FIRST** (`icbmmapfirst2pnw`, driver rule: maps are the anticipatory authority, 500 m
   horizon): with live map coverage over a stretch (`icbm_map_reach` = farthest published path point
   from the CURRENT position — a dead mapd's stale path decays out of coverage), the map verdict
   *including "no slowdown needed"* is authoritative for STARTING episodes.
2. **Vision** may only INITIATE beyond the map's reach AND with ≥ 2.75 s to act
   (`ICBM_VIS_MIN_TTC_S`); mapd-dead fallback: reach 0 → vision keeps initiating (map-first never
   becomes vision-never). Vision candidates get the same per-car penalty as VTSC
   (`curveslow-lightning`: `icbm_vision_apex()` closed the 493-sharp-camera-curves-zero-slowdown gap
   from 2026-07-11).
3. **Far-map candidate** (`descentcurve2pnw`): full 500 m horizon with a drop-scaled firm approach
   decel (0.8 → 1.4 m/s² over 30–60 mph drops) — binds the 90→65 mph curve at 450 m that the old
   10 s window never saw.
4. **In-curve suppression:** no NEW dec episode from ANY source while lateral-loaded; a running
   episode continues untouched.

### ICBM map-scale cap 1.35 (`icbmtrack2pnw` root-cause fix)

The shared `tiered_map_scale` sweeper end (raw ≥ 29 m/s → ×1.8, calibrated for VTSC/MTSC real
braking) inflated a rated 64.9 mph map curve to an effective 107 mph — "not binding vs set 90" was
correct on an absurd target → silence → the 19:58:37Z lane-change event (lead lost, stock ACC
accelerated 72→89 INTO the curve). Fix, ICBM-only (VTSC/MTSC/CES untouched):
`icbm_map_eff_scale = min(tiered, 1.35)`. Field-calibrated against BOTH 2026-07-12 legs: the 19:58
curve → eff 80.6 mph (driver manually chose 80–82) binds at set 90; the morning sweepers (raw 70.9
at set 85 → eff 88) stay silent — the morning over-slow fix survives. Additionally the Lightning
discounts raw OSM speeds by `map_scale` 0.92 before the binding test (curve.json knob).

### Continuous set-tracking (`icbmtrack2pnw`, driver-designed)

Binding-RATED map/far candidates within the **tracking window** start the cap episode even while
lead-bound below the target (drops the v_ego brake-envelope precondition): the set walks down to the
apex early — zero impact while following (ACC never decelerates unless actual speed exceeds the
walked set), but a lane change can never launch the truck into a bend (the 19:58 event, replayed as
a test), and silence now means broken (built-in liveness). Window = steps-to-walk × 0.4 s tap
cadence + 4 s margin at worst-case travel speed max(ref, v_ego), hard cap 350 m. Curves inside the
window merge into one episode (no ping-pong); candidates re-derive from CURRENT GPS every tick so a
wrong walk-down self-heals into the guarded restore. Vision candidates are NEVER tracked (keep all
gates).

## The guarded SET+ restore (`icbmrestore2pnw` — episode state machine)

Driver request: "it never set the speed back." `IcbmEpisode` (pure, in `ces_pnw.py`):
`idle → CAP` (first cap latches ceiling = the driver's own set) `→ HOLD` (sustained-clear 3 s
debounce, brain silent, ceiling retained across detection flicker / S-curve gaps) `→ RESTORE`
(target = latched ceiling, `"dir": "inc"`, ≤ 45 s window) `→ done → idle`. Restore returns ONLY what
ICBM took — never above the ceiling, never against the driver.

**Abort matrix (episode dies entirely → silence → executor stale-stops):** driver gas/brake or
ACC-off in ANY phase; 45 s window expiry; ceiling reached; any set movement across the silent hold;
set moving down at all during restore; set rising faster than our tap cadence (driver SET+ hold =
Ford 5 mph steps, trips instantly); a NEW cap (**DEC ALWAYS WINS**, fresh episode re-latches at the
CURRENT set); driver having lowered the set below ICBM's lowest command; invalid/zero set.
Executor side (`RestoreGuard`): inc pressed only while stock < min(target, ceiling) − deadband; an
inc command can never return dec; unknown `dir` values stand down; cap commands wire-identical (an
old executor ignores restore entirely).

**Apex-timed restore** (`icbmmapfirst2pnw`): passed-the-curve clears shorten the hold 3 s → ~1 s
(VTSC apex-release analog); dropout clears keep the full 3 s. Restore never begins and a running
restore PAUSES while lateral-loaded (pre-restore wait bounded 10 s — no stale ceiling latch).

**The 18:08:26Z false-positive fix:** the truck reports the set with ~1 s lag, so the executor's
final in-flight tap landed after the brain went silent and the 0.6-step driver-lower tolerance
misread our own tap as a driver SET− → killed the restore (stuck 76 vs ceiling 85 on a straight,
33 s). Fix: anchored one-time absorption of ONE late downward tap within a 1.5 s grace (never
upward, adopted at most once — Gemini caught and killed the chain-absorption variant) + tolerance
widened to 1.7 steps. Documented residual: one ambiguous SET− tap at our own floor restores
(ceiling-bounded, all aborts live); two taps still abort.

Gemini adversarial rounds found 3 real intent conflicts in v1 (silent-hold blind spot, ACC off/on
blast, deliberate-tap swallowing) — all fixed by STRENGTHENING guards, each with a regression test.

## Descent / left-curve parity (`icbmalign2pnw`)

ICBM applies the SAME shared `PnwVehicle.curve_speed_penalty_ms(target, pitch, is_left)` as VTSC —
one formula, one set of `/data/pnw/curve.json` knobs (see `CURVESLOW2PNW.md`, branch doc). Direction
for map candidates via pure `map_turn_direction()` path-geometry cross products. Cross-subsystem
parity test pins VTSC cap == ICBM target for identical inputs. Multipliers ≥ 1 → target only moves
DOWN; DEC-only + ceiling latch untouched.

## Telemetry (the A/B + forensics dataset — ces_events + CESStatus overlay)

`icbmT` (published target) · `icbmC` (latched ceiling) · `stockSet`/`stockOn` (truck-reported ACC
set + engagement; `icbmT`+`stockSet` stepping together = executor taps landing) · `icbmSrc`
(`map`/`vis`/`restore`) · `icbmDir` (`dec`/`inc`) · `icbmGate` (`inCurve`/`visCovered`/`visLate` —
why a start was suppressed) · `mapReach` (m; 0 = mapd blind, the outage signature) · `icbmOn`.
Overlay: always-visible in shadow — `ICBM no-ACC` / `ICBM ready` (green) / `ICBM 24>18` (orange,
capping) / `ICBM 55>75` (green, restoring); shadow modes render grey `>> SHADOW EXP/CHILL` (orange
reserved for real actuation, `379b34c32c`/`801c77e6f0`). Yellow CES icon = auto-Experimental vs
orange = forced (`7a8248a0eb`). mapd NaN guard: non-finite map velocities are skipped
(`379b34c32c`).

## Verification record

153 ces_pnw tests green at the `icbmtrack2pnw` merge (full abort matrix, washout registry, field
replays of the 18:08 and 19:58 events with real numbers, 120 s lead-bound no-expiry, route-divergence
self-heal, morning-leg anti-regression); ford executor suites 28 (incl. 10 restore tests: inc gates,
guard matrix, governor cadence). Gemini per-branch: restore "3 real conflicts fixed by strengthening
guards"; mapfirst "1 real catch (absorption anchor), fixed"; track "airtight". Lightning-only via
PnwVehicle capabilities; Tesla unreachable + provably neutral.

## Curve-slowing too LATE/insufficient — root cause + first-cut fix (`icbmcurve2pnw`, 2026-08-11)

**Status: CODE WRITTEN, NOT YET DEPLOYED/DRIVEN.** Built in an isolated worktree stacked on
`steerlimit-log2pnw` (branch `icbmcurve2pnw`). Full root-cause analysis: `docs/ICBM-CURVE-LATE.md`
(workbench root, not committed on this branch — read it first). Summary:

**Root cause:** ICBM's own map-curve candidacy test (`icbm_curve_target`/`_icbm_binding_apex`:
"reject if `eff >= ref - ICBM_MIN_DROP_MS`") compared against an already-inflated target — the
ICBM-only scale (`icbm_map_eff_scale`) flat-capped EVERY raw map speed at `ICBM_MAP_EFF_SCALE_CAP`
(1.35), so a genuine ~50 mph-rated curve at 55 mph cruise inflated to an effective ~62 mph (net
1.35 × the Lightning's 0.92 map-speed discount = 1.242), sat ABOVE cruise, and was silently
discarded before any distance/window logic ran — not a late trigger, a non-trigger. 131 m of
published map lead time went unused.

**Fix (`selfdrive/controls/lib/ces_pnw/ces_pnw.py`, `icbm_map_eff_scale` + new constants):**
replaced the flat cap with an ICBM-only two-point linear ramp:
  - `ICBM_MAP_SCALE_MIN = 1.10` (near-raw) at/below `ICBM_MAP_SCALE_LO_MPH` (50 mph raw) — small,
    documented correction for mapd/GPS curvature-noise, not a padding margin (driver direction:
    target the physics limit `v = sqrt(a_lat/kappa)`, no ~20% derate).
  - `ICBM_MAP_EFF_SCALE_CAP = MAP_SCALE_MIN` (1.35, **unchanged**) at/above `ICBM_MAP_SCALE_HI_MPH`
    (60 mph raw) — identical to the old flat cap, so the two 2026-07-12 field-calibrated events
    (64.9 mph binds at set 90; 70.9 mph stays silent at set 85) are byte-identical.
  - linear between 50-60 mph raw.
Scoped entirely to `icbm_map_eff_scale` inside `ces_pnw.py` — does **not** touch
`vtsc_pnw/vtsc_constants.py`'s shared `MAP_SCALE_MIN`/`MAP_SPEED_SCALE`/`tiered_map_scale`, so
VTSC/MTSC/Tesla are byte-unchanged. Every existing safety gate (reduce-only, `icbm_in_curve`
mid-curve suppression, the abort matrix, ceiling latch, V_MIN floor, tap-rate limits) is untouched —
this only changes which map targets qualify as candidates and what they're worth once they do.

**Test coverage:** added `test_58mph_event_regression_moderate_curve_now_binds` (the field-event
regression) and `test_icbm_scale_is_near_raw_for_tight_moderate_curves` to
`selfdrive/controls/lib/ces_pnw/tests/test_icbm_track.py`; updated two tests
(`test_icbm_scale_cap`, `test_map_beats_vision_when_both_present`) whose fixed inputs encoded the old
flat-1.35 behavior. Full `ces_pnw` suite: 272/272 pass (270 pre-existing + 2 new).

**Not yet done:** on-road validation against the `steerlimit-log2pnw` telemetry
(`docs/STEERING-LIMITS.md`); the two MPH breakpoints and `ICBM_MAP_SCALE_MIN` are a conservative
first estimate, meant to be iterated from real drive data, not a final tune. §7.C's speed-scaled
timing margins (`ICBM_A_DECEL`/`ICBM_MARGIN_M`/`ICBM_VIS_MIN_TTC_S`) from `ICBM-CURVE-LATE.md` were
deliberately NOT included in this first cut (out of scope — targets/candidacy only, not timing).

## Related

`CURVESLOW2PNW.md` + `FORDLONG2PNW.md` *(branch docs, pnw-pilot root)* · `LIGHT_CES.md` (CESMode) ·
`VTSC.md` (shared curve math) · `DEVICE-STATE.md` (params/files) ·
`drives/2026-07-1{1,2}/*/DRIVE_REPORT.md` (the field evidence) · memory `icbm2pnw-design` ·
`docs/ICBM-CURVE-LATE.md` (root-cause analysis + as-built section for the curve-lateness fix).
