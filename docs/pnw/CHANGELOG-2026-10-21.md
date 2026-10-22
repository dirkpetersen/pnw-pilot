# CHANGELOG — 2026-10-21 (Wednesday)

All times PT.

## `icbmvisraise2pnw` (branch `icbmvisraise2pnw`, not yet pushed): vision may lift a running mapd ICBM cap

Drive 2026-10-21, I-5 northbound in Portland (posted 50): ICBM capped the stock set to mapd's curve speed (43-45 mph, `icbmSrc`
map/far) while the camera measured k 0.0047-0.0051 1/m (about 50 mph at 2.5 m/s2). The truck held 42-43 in a 50; the curve DB had
no row there. Owner rule: the mapd rating is not a hard ceiling, confident vision may override it.

- **What:** while a cap runs on mapd's own number (`icbmSrc` map/far, the DB did not change the target), `icbm_vis_raise` gives
  v_vis = sqrt(2.5 / vis_k_max) - rain (the tightest curvature on the model horizon, not the instant one). If it passes every gate,
  the ICBM target becomes min(v_vis, set before the cap) and goes through the DB raise's own path (`IcbmEpisode._db_raise`): bounded
  by the episode ceiling, the posted limit + 5 mph and the restore cap, confirmed for **1.0 s** (`ICBM_VIS_RAISE_CONFIRM_S`).
- **Gates** (`icbmVisRaise` in ces_events): `off` (no capability: `icbm_restore_hold_lat_accel` is 0 on the Tesla), `noCap`,
  `notMap` (vision / DB / gpsHold cap), `lowConf` (no vision curvature or reach), `noReach` (mapd's apex beyond
  min(model reach, 8 s at v_ego)), `kRise` (vis_k more than 10 % above the lowest k of the window), `noLimit`, `noCurve` (vision's
  own speed above the posted limit + 5 mph: a straight reading is not evidence; a phantom and a vision miss look the same),
  `noGain` (v_vis not more than 5 mph above mapd's target, incl. vision sharper than the map). After the gates: the episode's
  `wait` / `acted` / `capped` / `noLimit` / `gas`, or `held` (the set is already at or above the raise). `icbmVisV` = v_vis, m/s.
- **Lowering is never blocked:** any failed gate gives mapd's number back the same tick, and the dec-only cap walks the set down.
- **Marker:** a vision raise is published with `why: "visRaise"`; SpeedAdjustController reads it as ICBM's own SET+, like `dbRaise`.
- **Not changed:** curve-DB, vision, police and limit-drop caps; the layer-2 lat-accel cap (`lataccel2pnw`); the Tesla path. No new toggle (CESMode governs, as for dbRaise).
- **Known risk:** `icbm_map_sanity` (2026-09-12 weekend) found vision under-reading some real curves about 1.6x in speed. The
  limit + 5 bound, the `noCurve` gate and the layer-2 cap are the backstop. `ICBM_VIS_RAISE_K_RISE` (10 %) is not measured.
- Tests: `ces_pnw/tests/test_icbmvisraise2pnw.py` (23): the Portland replay (43 -> 50 mph, not before 1.0 s, never above the pre-cap
  set), apex beyond reach, low confidence, vision sharper, straight reading, tightening curve, dropout, unknown limit, DB-changed
  target, the controller record fields and marker, Tesla `off`. 13 mutants killed (every gate, the confirm time, the k window,
  the target feed, both markers).
- **Review fixes (Gemini 4 Argon, SHIP-WITH-FIXES on 41ee11b9d9):** (1) the camera check covered only the WINNING mapd apex; a
  sharper mapd curve just past the camera could still be raised into. New gate `beyondReach`: every mapd point past the trust
  horizon (passed points removed) is priced as ICBM prices a far candidate, and the raise is refused when the most-binding one's
  decel envelope from here, at the 0.8 m/s2 comfort decel (not the large-drop firm decel), is below v_vis. (2) v_vis skipped the
  Lightning curve penalties: it now goes through `icbm_penalise` as a vision candidate (the base hump is given back, as for any
  vision curve; the descent and left extras and rain still come off). `noCurve` still reads the raw speed. A defect in either
  check is `icbmVisRaise: err`, logged, and mapd's cap stands. Not changed: finding 3 (the kRise window re-arms one tick after a
  refusal, at the tighter k, so the raise then follows the lower speed). Tests: 28; 18 mutants killed.

## `wheelbtnlog2pnw` (branch `wheelbtnlog2pnw`, not yet pushed): ces_events names the driver's wheel buttons

Drive 2026-10-21, I-5 Corvallis to Portland (Lightning). The 85 mph in a 65 was the driver holding SET+ for 4.4 s, and five ACC toggles at 18:52 were the
driver's cancel/resume rocker. The analysis first blamed openpilot and stock ACC, because the tick field `button` is the CES on-screen mode button and no
ces_events field recorded wheel presses. Evidence: `drives/2026-10-21/corvallis-portland-i5/DRIVE_REPORT.md` (private). **Logging only: no control change.**

- **Tick, `adopt` and CES-off `steer` records** gain `wbLast` (last pressed button: the capnp `ButtonEvent.Type` name, for example `accelCruise`, or `cnclRes` / `cncl`
  for the Lightning rocker), `wbAge` (s since that press), `wbN` (press edges since the previous one of these records), `wbHeld` (the most recently pressed button still down, else null).
  `wbRockWhy` appears when the car has a rocker and its state cannot be read; `wbErr` counts buttonEvents read failures. card retries a failed `WheelRocker` publish once a second.
- **`accDrop` records** gain the same four fields at the edge (`wbN` = presses in the 3 s before the edge). The raw `Steering_Data_FD1` bits were already in `chg`.
- **Rocker**: the pinned opendbc Ford carstate does not emit a ButtonEvent for `CcAslButtnCnclResPress` / `CcAslButtnCnclPress`. No submodule change: card's
  accdrop logger reads the two bits from the CAN parser carstate already registers (capability `PnwVehicle.wheel_rocker`, Lightning only) and publishes them on
  each change to the new mem-param `WheelRocker` (`params_keys.h`: needs the params_pyx rebuild that build-on-boot does). ces_pnw merges them into the tick.
- **`cesBtn`** is the new name of the record field `button` (CES on-screen mode button). `button` stays in the record for one release because
  `tools/curvebrain/golden.py` reads it from logged ticks. The overlay feed `CESStatus.button` (read by `ces_status.py`) is unchanged.
- Tests: `test_wheelbtnlog2pnw.py` (13, new), `test_accdrop_pnw.py` (+2). Mutation-checked (12 mutants, all killed).
- Review fixes (Gemini 4 Argon): `adopt` records carry the fields and drain the window; `wbHeld` dates a rocker held from its press edge; a failed `WheelRocker` publish is retried.
