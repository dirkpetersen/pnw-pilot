# CHANGELOG — 2026-10-22 (Thursday)

All times PT.

## `curveexit2pnw` (branch `curveexit2pnw` in pnw-pilot and pnw-opendbc, not yet pushed): Lightning angle-mode curve exit

Drive 2026-10-21 22:39, Seattle, sharp left at 16-22 mph at night (`drives/2026-10-21/seattle-left-curve-overshoot/DRIVE_REPORT.md`,
private). The plan straightened in time, but the wheel kept turning left about 1.5 s, the truck yawed about 5 deg past the road
and drifted about 1.9 m left, and the driver took over. Three causes on the actuation side: (a) the PSCM unwinds 0.8-1.4 s behind
the command; (b) our deviation clip held the command at measured curvature +- 0.002 and sent LEFT while the plan asked RIGHT;
(c) the 300 ms stall blip released steering in the middle of the correction. All three changes are in
`opendbc/car/ford/lateral_angle_pnw.py` (the Ford angle-mode strategy, Lightning only). **No panda safety code was touched.**
The Tesla path is not affected. No new toggle.

- **Fix 1 (opendbc `13396ff2`): no stall blip during a curve-exit correction.** An exit episode starts when measured curvature
  has the opposite sign to desired (above a 0.0005 1/m yaw-noise floor). It ends when the truck tracks the plan again, after
  3.0 s, or on lateral inactive / human-turn. No blip accumulates or fires inside it. A stall on a straight or in a curve the
  plan's way is unchanged. One carlog line per episode that held a blip off. In the replay of 22:39 the old code armed the blip
  at 22:39:22.04; ending the episode only when the truck crossed to the plan's side was not enough (re-armed at 22.84).
- **Fix 2 (opendbc `72ad413c`, taken back by `0417422c` after the Gemini 4 Argon review): the deviation clip's speed gate
  STAYS at bp-7.0's 9.0 m/s.** Panda facts, pinned `ford.h`: `ford_shadow_curvature_error_check` (`:449`) checks the shadow
  curvature (our `bp_kappa_cmd`, sent in Lane_Assist_Data1, `:668`) only while the latest ABS speed (`Veh_V_ActlBrk`, `:473`,
  the same signal as `vEgoRaw`) is above `.angle_error_min_speed = 10.0` m/s (`:168`). Tolerance: measured curvature (yaw /
  speed, last 6 samples, `:485-491`) +- `max_angle_error` 100 CAN units (`:156`) = 0.002 1/m, which equals `CURVATURE_ERROR`.
  A violation drops that LMC2 frame (`:1016`). The 72ad413c gate (`vEgoRaw + max(aEgo, 0) * 0.15 s > 9.5`) was not safe:
  carcontroller sends LMC2 before Lane_Assist_Data1, so each LMC2 is judged against the shadow of the previous steer frame
  (50 ms, pinned by a test on the real CarController; 0.15 s with staleness and latency), `aEgo` is Kalman-lagged, and
  `Veh_V_ActlBrk` has single-sample spikes of up to 0.53 m/s in our logs. The gate would have to be 10.0 - (0.53 + a * 0.15) =
  9.08 m/s at the largest acceleration in our logs (2.6 m/s2), 8.87 at 4 m/s2, so nothing above 9.0 can be un-clipped. The
  22:39:21.34-21.64 wrong-way clip therefore stays (fix 1 still removes the blip). The test suite pins the numbers to ford.h.
- **What a dropped LMC2 frame did (our logs):** the panda blocked two LMC2 frames on this route (22:39:28.09 and 29.04, 10.4-10.8
  m/s, old code, driver nudging, cause not determined). The PSCM status did not change (LatCtlSte 2, Lim 0, Cpblty 2), no
  steerFault, openpilot stayed engaged, no alert. Longer runs of dropped frames: unknown.
- **Fix 3 (opendbc `c29ff553`): unwind lead, SHADOW ONLY.** A lead of the plan's own value 0.4 s further ahead (3-15 m/s, toward
  centre only, never past the plan's future value or past zero, <= 0.010 1/m, slewed) is computed (`unwind_lead_shadow`) and
  logged per episode, never applied. Offline open-loop replay: at 22:39 it changes the command by at most 0.025 rad and does not
  move the 10 % / zero points of the unwind (the lag is wheel-behind-command, which a plan-bounded lead cannot reach); without the
  zero bound it pulled LEFT after the plan crossed to the right; on a normal 65 deg turn (seg 27, 22:35:36-42) the wheel
  followed the unwind within about 0.2 s, so a general lead would over-steer normal exits.
- Tests: `opendbc/car/ford/tests/test_blipguard_pnw.py` (6 -> 14), `opendbc/car/ford/tests/test_curveexit_pnw.py` (13, new:
  ford.h pins, the shadow age from the real CarController's send order, a grid through the real libsafety at the worst observed
  timing, the shadow lead's bounds and that it is never applied, and the shadow episode logged on disengage / human turn). Every
  new branch mutation-checked. The new file needs a `TEST_PATHS` row in `scripts/check-channel-tip.sh` once it is on the channel.
- **Risks / open:** fix 1 can delay a real stall blip by up to 3 s after a curve exit. Even bp-7.0's 9.0 m/s gate is 0.13 m/s
  short of the worst case at 4 m/s2 plus a 0.53 m/s speed spike (a single dropped LMC2 frame); lowering it below 9.0 would
  change every exit near 9 m/s and is the owner's decision. The pre-existing int16 shadow packing saturates at 0.0328 1/m
  (not changed here). The 22:39 exit's main lag (wheel 1.5 s behind the command) is not fixed by this branch.
