# VTSC release-later, passed-point mask, cruise-off reset (`vtscfloor2pnw` + `vtscpass2pnw`)

Status: pushed to `3devpnw` 2026-09-30 (`f57372d92c`), Fable SHIP, **not installed on the device** at the time of writing.
Builds on [`VTSC.md`](VTSC.md). No new param key, no panda change, no coordinates in this file.

## 1. What it does

1. **Release-later (Raven only).** VTSC's state machine is unchanged. Inside the RELEASE state, the cap is *frozen* while the target ahead is still
   tightening (bounded 5 s budget, not refilled by a one-cycle "no curve" flicker), so the car does not accelerate out of a curve toward an apex that
   is still ahead. It is a freeze, so the cap is never higher than it would be without the feature. Switch: `/data/pnw/curve.json`
   `{"tesla": {"vtsc_release_later": false}}` (default ON; the Lightning ignores it). Telemetry: `vtscRelDefer`; the `curve_brain_cfg_reload` event carries
   `vtsc_release_later`. An "agreed floor" variant was built first and REMOVED as inert (it could never change the cap in 186,020 replayed cycles).
2. **Passed-point mask (both cars; VTSC is shared).** mapd keeps the way just driven in `MapTargetVelocities`; the map fold measured an unsigned distance
   to it, so a node about 170 degrees behind the car could cut the cap (a loop ramp: 84 -> 42 mph). The fold now runs only on points the car has not passed,
   using the same geometry ICBM's behind-gate uses (`icbm_passed_points`). When it cannot tell (under 5 m/s, no heading, path not on our road, an error)
   every point is used, which is the old behaviour, and the reason is logged change-only.
3. **Cruise-off reset.** VTSC computes a cap every planner cycle whether or not openpilot is engaged. A `hold` latched while the driver drove a curve by
   hand was applied at the next stalk re-engage (54 -> 40 mph). Now all latches (state, applied cap, debounce counters, the release-later freeze, the
   curve brain's applied value) are dropped every cycle in which `carControl.enabled` is False, so the first engaged cycle starts from idle. It keys on
   `enabled` (cruise on/off), NOT `longActive`, because `longActive` is also False during a gas override and would drop a cap that is ready.
4. **Hold -> brake exit with hysteresis:** the apex must be receding (time-to-apex above the hold threshold + 0.5 s) for 3 consecutive cycles.

**Side effect:** `vtscCap` / `vtscState` no longer shadow-log while cruise is off (they read idle/none). Analyses that used the cap during manual driving
must not expect it.

## 2. First-drive checks (swaglog and `ces_events`)

- swaglog: `VTSC map path passed-point mask: <state> (<reason>)` lines appear **change-only** (not per cycle).
- **Zero** `VTSC: passed-point mask FAILED` and zero `VTSC: carControl.enabled unreadable` lines (both are loud by design; either means the fallback ran).
- Ramp / loop: `mapD` must not grow while `vtscCap` falls (a growing distance with a falling cap = binding to a point behind the car).
- Re-engage after a manual curve: the first engaged tick reads `vtscState` = idle, no cap, no freeze.
- Lift the gas about 3 s before a curve: the cap is kept (no reset on override).
- Twisty road: no per-cycle "dropped" log spam (the drop logs only when something was actually discarded).
- Kill-switch test (optional, parked): flip `vtsc_release_later` and confirm the `curve_brain_cfg_reload` event within ~2 s.

## 3. Not covered

The Lightning ICBM `upcoming_curve` path uses the same map path and was deliberately not changed; whether it has the same blindness is unproven.
