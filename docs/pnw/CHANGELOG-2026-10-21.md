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
