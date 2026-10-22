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
- **Fix 2 (opendbc `72ad413c`): the deviation clip's speed gate follows the panda.** Panda facts, pinned `ford.h`:
  `ford_shadow_curvature_error_check` (`:449`) checks the shadow curvature (our `bp_kappa_cmd`, sent in Lane_Assist_Data1,
  `:668`) only while the latest ABS speed (`Veh_V_ActlBrk`, `:473`, the same signal as `vEgoRaw`) is above
  `.angle_error_min_speed = 10.0` m/s (`:168`). Tolerance: measured curvature (yaw / speed, last 6 samples, `:485-491`)
  +- `max_angle_error` 100 CAN units (`:156`) = 0.002 1/m, which equals `CURVATURE_ERROR`, so **the tolerance is not changed**.
  A violation drops that LMC2 frame (`:1016`). bp-7.0 clipped from 9 m/s; the whole 22:39 exit (9.0-9.95 m/s) was below the
  panda's gate. New gate: clip when `vEgoRaw + max(aEgo, 0) * 0.15 s > 9.5 m/s`. Replay: 22:39:21.34-21.74 now commands
  +0.025..+0.063 rad (right) instead of -0.028..+0.004. It also removes a stall blip that fired at a curve ENTRY at 22:35:22.5
  (seg 27, 9.0-9.2 m/s), before the driver took that turn by hand. One carlog line per stretch the relaxed band let a command through.
- **Fix 3 (opendbc `c29ff553`): unwind lead, SHADOW ONLY.** A lead of the plan's own value 0.4 s further ahead (3-15 m/s, toward
  centre only, never past the plan's future value or past zero, <= 0.010 1/m, slewed) is computed (`unwind_lead_shadow`) and
  logged per episode, never applied. Offline open-loop replay: at 22:39 it changes the command by at most 0.025 rad and does not
  move the 10 % / zero points of the unwind (the lag is wheel-behind-command, which a plan-bounded lead cannot reach); without the
  zero bound it pulled LEFT after the plan crossed to the right; on a normal 65 deg turn (seg 27, 22:35:36-42) the wheel
  followed the unwind within about 0.2 s, so a general lead would over-steer normal exits.
- Tests: `opendbc/car/ford/tests/test_blipguard_pnw.py` (6 -> 14), `opendbc/car/ford/tests/test_curveexit_pnw.py` (11, new;
  includes a grid through the real libsafety: every unclipped shadow passes with the panda's speed sample 0.10 s newer plus
  0.3 m/s). Every new branch mutation-checked (13 mutants, all killed). The new file needs a `TEST_PATHS` row in
  `scripts/check-channel-tip.sh` once it is on the channel.
- **Risks:** fix 2 lets the command lead measured curvature by more than 0.002 between about 9.4 and 9.5 m/s; under full-throttle
  acceleration with `aEgo` lagging the real acceleration by more than ~3 m/s2, the panda could drop single LMC2 frames near
  10 m/s. Fix 1 can delay a real stall blip by up to 3 s after a curve exit. The pre-existing int16 shadow packing saturates at
  0.0328 1/m (not changed here).
