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
