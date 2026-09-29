# CHANGELOG — 2026-09-28 (Monday)

Continues [`CHANGELOG-2026-09-27.md`](CHANGELOG-2026-09-27.md). All times PT.

**Channel tip:** `origin/3devpnw` = `841a4bd000`, **GREEN**, 4556 passed. **Installed on the device** 2026-09-28 (PT) by a reboot while
openpilot was not running; `DisableCoopSteer` reads 0, no unknown-key errors.

## 1. 🟡 `coopsteer2pnw` v2 (`841a4bd000`) — the Tesla Raven cooperative-steer nudge now follows the driver's push up to 3.0 Nm; toggle flips to `DisableCoopSteer`, default ON

The 09-27 first drive (`drives/2026-09-27/coopsteer-first-try/DRIVE_REPORT.md`, 20:22–20:35 PT) found
v1 "does not seem to work": it actuated (13.9 s, up to 8.2°, 96.5% sign-correct), but it only acted in
the narrow 0.5–1.0 Nm band and then **froze (HOLD)** — of 31 real pushes, 12 crossed that whole band in
a median 80 ms, so the driver never felt it. Above 1.0 Nm the car resisted exactly as it did with the
nudge off, until the EPS's own hands-on abort (one push: 4.9 s up to 3.35 Nm).

Owner decision (2026-09-27, after reviewing the drive): build the proportional option, full yield at
3.0 Nm, and make the feature **on by default** — "enabling should be the default in the future ...
default always means that a toggle is disabled."

* **Proportional response, no freeze.** `COOP_FULL_NM` moves from 1.0 to 3.0 Nm: the offset now scales
  from the unchanged 0.5 Nm dead zone up to full yield at 3.0 Nm (just under the lowest EPS hands-on
  abort seen, 3.14 Nm), and the target always carries the torque's own sign — the old HOLD/freeze branch
  is gone. A reversal is felt through the existing 0.1 s torque low-pass and shed at the full jerk rate
  once the target flips.
* **Caps unchanged:** 12° / 1.5 m/s² magnitude limit, 5 s washout, slew at 0.5/1.0 of the Tesla's own
  jerk rate, 60°/s ceiling. No panda or safety-model change; the Tesla carcontroller's own steering-angle
  limiter still bounds the total commanded angle.
* **Toggle renamed to the `DisableCoopSteer` opt-out convention** (matching `DisableLaneCentering`):
  `CoopSteer` (default OFF, on = actuate) is replaced by `DisableCoopSteer` (default `"0"`, on =
  disabled). With no setting at all the Raven now actuates the nudge by default; an unreadable param
  fails safe to NO nudge (logged once). UI label: "Disable Cooperative Steering (Tesla)" — plain toggle,
  no restart required, grayed out without the Tesla's `coop_steer` capability. The device's old
  `CoopSteer=1` param file is left in place unread (harmless); it is deleted automatically the next time
  `Params::clearAll` runs at manager start.
* **Verification:** process_replay of the real `controlsd` against the pre-actuation base — Raven
  segment: `DisableCoopSteer=1` reproduces the base exactly (0 of 5,998 messages differ); default
  (feature on) diverges only in `actuators.steeringAngleDeg`, max 10.1°; Lightning segment: 0 differences
  either way (the capability is Raven-only). Full test suite green (2,036 controls/ces tests, 134 UI); 27
  of 27 mutants killed.
* **Open / unmeasured:** this is calibrated open-loop. The yield lowers the very torque that drives it,
  so closed-loop behavior — whether the wheel settles where the push balances the offset, or hunts
  against a stiff/braced hand — is for the next drive to show. Watch items below.

**First-drive watch items** (not yet observed on-car; tracked in
`docs/work-pending/2026-09-27-coopsteer-first-drive-watch.md`, workbench docs):
1. **Closed-loop hunting.** A braced/stiff hand at 20–30 mph could produce a 6–10° peak-to-peak limit
   cycle (bounded by the 12°/1.5 m/s² caps) if the 5 s washout re-admits a sustained push faster than the
   driver relaxes it — this is a deliberate trade-off against v1's no-shed behavior, not a defect if it
   stays within the caps.
2. **Highway ceiling.** At 60 mph the maximum yield is ≤5.3° of wheel angle (capped by the 1.5 m/s²
   lateral-acceleration limit), so "doesn't work" can still recur at highway speed by design.
3. **Slightly slower reversals.** Direction changes settle roughly 0.1 s slower than v1's.

Reviewed under the standing Fable-before-push policy; SHIP (a second post-push review said
FIX-THEN-SHIP: a rigidly braced grip can hunt — see the watch items above). Channel tip GREEN (4,556 passed at this
commit). Pushed to `3devpnw` and installed.

## 2. `teslamads2pnw` (`c53bae6ed8`, opendbc `c0d12514`) - the Tesla Raven's steering now survives a brake press

Brake was the largest single cause of the Raven dropping out of engagement: 17 of 31 engaged-to-disengaged transitions in the
rlogs were a brake press. The Raven now gets the same MADS lateral-through-brake behaviour the Ford already had, without
auto-resume (there is no stock-ACC button path to spoof on the Raven).

* **Panda (opendbc, internal panda only):** the safety code accepts the MADS `alternative_experience` bits for the Raven's internal
  panda (`SAFETY_TESLA_LEGACY` + `FLAG_HW3`, no external-panda flag). Every other Tesla config (external/longitudinal panda, HW1, HW2)
  still refuses them, so the longitudinal message can never see a lateral latch. `acc_main_on` is now written from `DI_cruiseState`
  (revoke-only, as on the Ford). **The internal panda's safety code was reflashed.**
* **openpilot:** `PnwVehicle.mads_lateral` includes the Raven; `mads_resume` stays False. No new param or toggle: it is ON by
  default via `PandaMadsSafety=1` with the existing "Disengage on brake" toggle at its default OFF. That toggle, previously greyed
  out on the Tesla, is now enabled; turning it on reverts the Raven to stock brake-disengage but also affects the Ford (one shared
  param; a per-car opt-out is a follow-up).
* **Measured basis:** the brake frame is logged 19-51 ms before the frame that drops cruise, and cruise goes to STANDBY (never OFF)
  after both a brake and a stalk cancel, so the panda's own "cruise falling edge while not braking" rule is what tells them apart.
  The old claim that the EPS inhibits itself on a brake press is not supported (`eacStatus` goes ACTIVE to AVAILABLE, error 0).
* **Status:** channel tip GREEN (4556 passed). Installed; the owner reports steering continues through braking and it works well.
  Not yet verified: the parked `alternativeExperience` readback on both pandas.
* **Review:** Fable, SHIP conditional on a supervised first test and two accepted gaps: (1) no driver-reachable exit from
  steering-only except a firm steer (or stalk pull then push) - a stalk read is a required follow-up; (2) if the EPS refuses angle
  commands while cruise is STANDBY the failure is silent - an alert is a required follow-up. Also open: black-panda readback,
  the unneeded 600 ms Ford re-latch window on the Raven, and the per-car opt-out.
* The code was written by a Sonnet agent under an owner-approved one-job exception to the never-Sonnet rule.

## 3. Three more `3devpnw` changes: police cap memory, selfdrived loop diagnostics, Tesla stalk + EPS-refusal alert

All three were reviewed before the push and the channel tip was GREEN after each.

### `policecap2pnw` (`cc036ae2ef`) - a released police cap is remembered for 10 s

On the openpilot-longitudinal path (the Tesla) a police cap that had just released was forgotten, so a second alert arriving right after
the first re-ramped from the driver's set speed. On a real drive that produced a 65 to 73 mph surge in the middle of a curve. A released cap
is now remembered for 10 s and a re-engaging cap seeds from it. Police handling is unchanged otherwise (exactly limit + 5; a false positive
is acceptable, a miss is not). Not part of the Ford stock-ACC path.

* **Status:** pushed. Follow-up open: an alert reported at zero distance still releases the cap at the report itself; a fix is in progress.

### `loopdiag2pnw` (`ee3fb55fd1`) - always-on selfdrived slow-loop and event-log write timing (diagnostic only)

A one-off "Communication Issue" alert traced to a 113 ms stall in the selfdrived publish loop, with the cause undetermined. selfdrived now
logs any loop iteration over 50 ms with a per-stage breakdown, and the timing of the ces_events write, so the next occurrence shows whether
it was a stall inside selfdrived or something outside it. No behaviour change.

* **Reading the output:** the sleep/scheduling stage reads about 0 by design, because selfdrived is paced by a blocking carState receive;
  a late carState therefore shows up as time in the data-sample stage, not as sleep.
* **Status:** pushed; channel tip GREEN (4575 passed).

### `teslastalk2pnw` (opendbc `66439dfd`, pnw-pilot `fdaad20168`) - the Raven's stalk ends steering-only, and an EPS refusal is no longer silent

Two follow-ups required by the review of section 2 (steering survives a brake press). **No panda change and no new parameter.**

* **Stalk read:** the cruise stalk (`STW_ACTN_RQ`, `SpdCtrlLvr_Stat`) is decoded passively. Forward push = cancel (`mainCruise`), rearward
  pull = resume (`resumeCruise`). It is wired into the existing off-request path, so a stalk push while in steering-only ends everything.
  Measured basis, 166 Raven logs: the rearward pull was seen on 29 of 32 presses; the forward push was established by a single event, and a
  forward push from STANDBY has never been observed (unverified on-car). The earlier feasibility note had the two positions the wrong way
  round, and the UP/DN 1ST/2ND values are speed detents, not engage/cancel.
* **EPS-refusal detector:** if lateral is active and being commanded, but not fully engaged and the EPS is not reporting ACTIVE, for 50
  consecutive frames, the car raises a temporary steering fault and logs an `epsRef` telemetry event. The review corrected the expectation
  for steering-only: this alert is not loud there. Hands off for 1.5 s or more: lateral ends with a chime and no text. Wheel touched: a
  repeating small warning about every half second. Firm hand: silent apart from log entries. The safety property (the failure is not
  invisible and lateral ends) holds.
* **Status:** pushed; channel tip GREEN (4583 passed). Staged on the device, not yet installed. Still to check on the car: the stalk events
  in steering-only, forward-push-from-STANDBY, and steer-through-brake.
