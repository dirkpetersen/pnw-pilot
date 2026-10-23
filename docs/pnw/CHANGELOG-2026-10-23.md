# CHANGELOG — 2026-10-23 (Friday)

All times PT.

## `handsready2pnw` (branch `handsready2pnw` in pnw-pilot, not yet pushed): "Keep hands ready: sharp turn" alert

Why: drive 2026-10-21 22:39, Seattle (`drives/2026-10-21/seattle-left-curve-overshoot/DRIVE_REPORT.md`, private). On the
F-150 Lightning in angle mode the PSCM unwinds 0.8-1.4 s behind the command after a sharp low-speed turn. At night, with
almost no lane lines (laneLineProbs <= 0.07), the truck went about 1.9 m past the curve exit into the other lane. We cannot fix
the PSCM, so the driver gets a heads-up in that situation. Owner approved 2026-10-23.

- **What it does.** A small orange onroad alert "Keep hands ready: sharp turn" with one soft `prompt` chime. Display only: event
  `sharpTurnHands` (custom.capnp `OnroadEventPnw` @6) carries only `ET.WARNING`, so it cannot disengage, refuse entry or change
  the state machine. It shows in the MADS steering-only state too (selfdrived re-admits WARNING there).
- **When it fires** (`selfdrive/selfdrived/sharp_turn_pnw.py`), all for >= 0.3 s: lateral control active; the car has
  `PnwVehicle.sharp_turn_hands_alert` (Lightning only) AND the Ford carcontroller reports the angle path live (`FordLatStatus`
  mode `angle`, the same poll steersatfloor2pnw uses); the plan's largest |curvature| now or within the next 2 s is >= 0.02 1/m
  (about R 50 m; from `action.desiredCurvature` and the plan's yaw rate / speed); the weaker ego lane line is below 0.6 (the
  lane-centering `min_lane_prob`, the gate behind `lcGate=lowconf`); 3 < vEgo < 15 m/s.
- **Hold.** At least 2 s, then until the turn has ended: plan AND measured curvature (`controlsState.curvature`) both < 0.01 1/m.
  The measured term keeps it up through the slow unwind (at 22:39 the plan was straight 1.5 s before the truck). It clears at once
  when steering stops (driver takeover). Re-fire at most once per 10 s. No flicker.
- **Tesla:** never (no capability; the detector is not even built).
- **Toggle:** `DisableSharpTurnHandsAlert` ("Disable Sharp-Turn Hands Alert"), default `"0"` = alert ON. Greyed (never hidden)
  on a car without the capability, reason "Ford F-150 Lightning only" (`CAR_GATED`). selfdrived re-reads it at 10 Hz, no restart.
  New params key: rebuild `params_pyx.so`.
- **Log (Rule 2).** Every fire writes one ces_events record `{"ev":"alert","name":"sharpTurnHands","phase":"fire", "k", "kMeas",
  "laneConf", "vEgo", "lat", "lon", "heading", ...}` (through `log_take_control_alert`) and a `handsready2pnw: Keep hands ready
  fired` swaglog warning. An unreadable modelV2 logs `handsready2pnw: modelV2 unreadable ... NOT deciding` (throttled), never
  reads as "straight road".
- Tests: `selfdrive/selfdrived/tests/test_sharp_turn_pnw.py` (22, new: the 22:39 pattern fires, confident lanes / gentle curve /
  speed / not armed never, persist, hold, unwind hold, rate limit, Tesla never, toggle off never, curvature mode never, malformed
  model logged, ces record fields); `selfdrive/ui/tests/test_car_gating.py` (the new grey row + default "0"). Every gate
  mutation-checked (16 mutants, all killed).
  `test_capnp_fork_ordinals_pnw.py::test_fork_enum_keeps_the_old_relative_order` now checks the six old events keep @0-@5 in
  order and the enum has no gaps (it asserted the enum was exactly those six, which an append broke).
  Touched suites (selfdrived, ui, ui/onroad, controls/lib, ces_pnw): 3421 passed, 86 skipped (base: 3286 + 107 onroad = 3393).
- Not done: no screenshot (the UI has no harness that renders a single alert); no on-car check yet.

## `curvedbloaded2pnw` (branch `curvedbloaded2pnw` in pnw-pilot, not yet pushed): curve-DB `loaded` is sticky per sha

Why: the cloud job promotes an installed curve-DB table to its baseline only when the device report says `loaded: true`
for that sha. `curvedb_updated` decided `loaded` fresh on each cycle. While driving it does not scan the swaglogs
(`unknown, not checked, driving`), and when parked or offroad the scan only finds a `curvedb_v2_loaded` line while that
swaglog still exists. So a `loaded: true` report (2026-10-21 17:08 for b9066860) was overwritten by later `false`
reports, and the baseline stayed e32ccf54.

- **What changed** (`system/curvedb/curvedb_updated.py`). When the scan finds that selfdrived loaded the live sha (the
  existing `swap.verify()`: a load line newer than the table file's mtime), the daemon writes `loaded_sha` + `loaded_utc`
  into `/data/pnw/curvedb_autoinstall.json`. Later cycles report `loaded: true` for that same sha when the scan answers
  `unknown` or `pending`. The reason then says `selfdrived load: loaded, recorded MM-DD, now <unknown|pending>...`.
- **Resets.** A new install, `--adopt` or a record repair rewrites the record without `loaded_sha`, so a new live sha starts
  at `false`. Evidence for another sha never counts. A `mismatch` (selfdrived loaded a different table) clears it.
- **Unknown stays unknown.** No recorded evidence: `loaded: false` with the reason, as before. The swaglog scan keeps its
  existing gate (never while driving).
- **Report contract unchanged** (same 9 keys; `loaded` is still a bool).
- Tests: `system/curvedb/tests/test_updater.py` (+4: loaded once stays true parked and driving; no evidence is false with the
  reason; a new live sha resets, evidence for another sha does not count; mismatch clears). Curvedb suite 100 -> 104 passed,
  1 skipped. 5 mutants, all killed.
- Note: the device record has no `loaded_sha` yet. b9066860 reports `true` only after the next drive start is followed by a
  parked/offroad cycle that still finds its load line.
