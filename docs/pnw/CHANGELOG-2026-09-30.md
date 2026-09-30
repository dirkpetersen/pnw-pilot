# CHANGELOG — 2026-09-30 (Wednesday)

Continues [`CHANGELOG-2026-09-28.md`](CHANGELOG-2026-09-28.md) (there is no separate 09-29 file; its ships are section 1 here). All times PT.

**Channel tip:** `origin/3devpnw` = `c0b4bd4499`, **GREEN**, 5601 passed. **Installed on the device:** `848d115fed` (reboot 2026-09-29 ~22:00 PT).
Everything in sections 2-4 is **pushed, not installed**.

## 1. Installed 2026-09-29 (`848d115fed`)

`65db8d76f8` `uimax2pnw` (Lightning screen keeps the driver's max during ICBM) · `1a2e299451` / `b966d7b849` curve brain 3.6 steering ceiling, per-curve override
file and follow-ups · `9778d34d0b` phantom-brake fix · `df77bba028` `teslastalk2b` · `b57f7ed8c1` `policedist2pnw` · `848d115fed` `ovrcar2pnw`: the per-curve
override file is per-car (schema v2), Tesla and Lightning only, direction mandatory. Data installed with it (private, not in this repo): the override file v2
and the curve table `bf1d885b` (19,927 rows).

## 2. Pushed, not installed: VTSC release-later, passed-point mask, cruise-off reset (`f57372d92c` and predecessors)

`vtscfloor2pnw` (release-later freeze, Raven only, switch `tesla.vtsc_release_later`, default ON) and `vtscpass2pnw` (map fold ignores points behind the car;
VTSC state resets when cruise turns off). Found on the 09-29 evening drive: a loop ramp where a point behind the car cut the cap, and a `hold` latched during a
manual curve that was applied at re-engage. Fable SHIP; one HIGH finding (reset must key on `carControl.enabled`, not `longActive`) fixed before the push.
Details and first-drive checks: [`VTSC-RELEASE-LATER.md`](VTSC-RELEASE-LATER.md).

## 3. Pushed, not installed: `latcar2pnw` (`83c152e33f`, `c0b4bd4499`)

An optional per-car `"cars"` section in `/data/pnw/lataccel_limits.json` lets the Tesla's curve target be bounded by its vehicle-model clamp instead of the
shared flat 3.0 above 80 mph. Inert until an entry is installed by hand (the value is an owner decision). A misspelled per-car key is now reported (the car
logs whether it uses its own or the shared schedule) instead of silently running the shared one; a failed default-seed write is logged (Rule 2). The opendbc
angle bound is unchanged; not a panda change.

## 4. Test-only

`9ada1c1d93` seed tests read `a_max` from the private seed instead of hard-coding it. Known open: two test asserts grep a log string that no longer exists
(vacuous), and the curve-brain goldens need deliberate re-recording.
