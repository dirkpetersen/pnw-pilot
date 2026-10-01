# CHANGELOG — 2026-09-30 (Wednesday)

Continues [`CHANGELOG-2026-09-28.md`](CHANGELOG-2026-09-28.md) (there is no separate 09-29 file; its ships are section 1 here). All times PT.

**Channel tip:** `origin/3devpnw` = `a037278713`, **GREEN**, 5814 passed (sections 9-11 pushed 23:16 PT 09-30 to 00:24 PT 10-01; at `1afb551fa6` the tip was green with 5708). **Installed on the device:** the tip as of the evening installs (~19:15 PT, and a reboot at 21:27 PT); `848d115fed` was the 09-29 ~22:00 PT reboot. Sections 2-3 were pushed during the day and are part of that evening install (`83c152e33f` ... `c0b4bd4499` are ancestors of the tip); sections 5-8 shipped the same evening.

## 1. Installed 2026-09-29 (`848d115fed`)

`65db8d76f8` `uimax2pnw` (Lightning screen keeps the driver's max during ICBM) · `1a2e299451` / `b966d7b849` curve brain 3.6 steering ceiling, per-curve override
file and follow-ups · `9778d34d0b` phantom-brake fix · `df77bba028` `teslastalk2b` · `b57f7ed8c1` `policedist2pnw` · `848d115fed` `ovrcar2pnw`: the per-curve
override file is per-car (schema v2), Tesla and Lightning only, direction mandatory. Data installed with it (private, not in this repo): the override file v2
and the curve table `bf1d885b` (19,927 rows).

## 2. Pushed 09-30: VTSC release-later, passed-point mask, cruise-off reset (`f57372d92c` and predecessors)

`vtscfloor2pnw` (release-later freeze, Raven only, switch `tesla.vtsc_release_later`, default ON) and `vtscpass2pnw` (map fold ignores points behind the car;
VTSC state resets when cruise turns off). Found on the 09-29 evening drive: a loop ramp where a point behind the car cut the cap, and a `hold` latched during a
manual curve that was applied at re-engage. Fable SHIP; one HIGH finding (reset must key on `carControl.enabled`, not `longActive`) fixed before the push.
Details and first-drive checks: [`VTSC-RELEASE-LATER.md`](VTSC-RELEASE-LATER.md).

## 3. Pushed 09-30: `latcar2pnw` (`83c152e33f`, `c0b4bd4499`)

An optional per-car `"cars"` section in `/data/pnw/lataccel_limits.json` lets the Tesla's curve target be bounded by its vehicle-model clamp instead of the
shared flat 3.0 above 80 mph. Inert until an entry is installed by hand (the value is an owner decision). A misspelled per-car key is now reported (the car
logs whether it uses its own or the shared schedule) instead of silently running the shared one; a failed default-seed write is logged (Rule 2). The opendbc
angle bound is unchanged; not a panda change.

## 4. Test-only

`9ada1c1d93` seed tests read `a_max` from the private seed instead of hard-coding it. (The two vacuous asserts and the golden flake noted here earlier were fixed in section 5;
the goldens still need deliberate re-recording when the brain changes.)

## 5. `smallfix2pnw` (`a6f838ae2c`, with `b557bc5c46`, `ed04d20942`)

Three small cleanups. The `latcar` "misspelled?" warning fired on every platform key that was not this car's; `report_platform` now warns only on a likely typo
of THIS car's key, and an unknown-platform key still warns (`b557bc5c46`). Two test asserts grepped a log string that no longer exists, so they could not fail;
they now grep the real message, "cruise off" (`ed04d20942`). The curve-brain golden two-process test was flaky because `curvedb_shadow` read its own clock;
it is pinned to the replay clock and the test names the differing field on failure (`a6f838ae2c`).

## 6. `tailscale2pnw`: optional remote SSH over Tailscale (`1610cd2a35`, `a47a1de706`, `27a734b008`, `e39bcb44b0`)

A default-OFF toggle (Settings > Toggles > Remote SSH (Tailscale)) with a pinned installer and a status line. Fable reviewed it three times and returned BLOCK
each time before SHIP. Blockers found: (1) toggle OFF could leave the root `tailscaled` reachable while the UI read "off"; (2) a stuck-daemon error was masked to
"off" in the UI; (3) an orphan `tailscaled` was noticed by nothing; (4) a non-UTF-8 `/proc` `comm` crashed the process scan in a loop (`e39bcb44b0` reads comm as
bytes and logs recurrence). Fixes: OFF now stops `tailscaled` and verifies it gone; the error survives OFF; the `tailscale_pnw` process always runs, is inert while
OFF, and stops a leftover daemon. First enable verified on the car: kernel mode, direct path including over LTE, OFF stops the daemon within 31 s, no iptables or
route changes. Doc: [`TAILSCALE.md`](TAILSCALE.md).

## 7. Test fixtures without key-shaped strings (`5cef4ab8dd`)

GitHub secret scanning raised an alert on a FAKE fixture. The tailscale tests no longer contain any `tskey-` literal. The same commit corrects `TAILSCALE.md`:
the process always runs and is inert while OFF.

## 8. VIN redaction (`53e72e5b33`, `1afb551fa6`)

The Lightning VIN is redacted to its first 13 characters plus `XXXX` in the openpilot skill doc (`53e72e5b33`) and in the pnw-opendbc tests and comments
(`c602973cd7`; pin bump `1afb551fa6`). No behaviour change. Channel tip after the pin bump: green, 5708 passed.

## 9. `mapsl2pnw`: a dead mapd no longer leaves a stuck speed limit (`de1197706d`, `e359070711`, `124426db5b`)

When mapd is silent for 5 s, `MapSpeedLimit`, the conditional limit, `RoadContext`, `MapOneWay` and `MapLanes` are cleared (before: the last limit stayed forever),
the UI speed-limit sign expires, and the driver-monitoring road identity is cleared too, so DM goes **strict** (not relaxed) with no road information.
`location_servicesd` keeps the last `RoadContext` verdict (`RoadCtxHold`) so the police banner never loses its freeway/surface answer when the param is cleared.
Opus review findings: (1) BLOCK, police never-miss: clearing `RoadContext` would have changed the police banner's road class; fixed by `RoadCtxHold`
(`124426db5b`); (2) DM must go strict when the road identity is gone (`e359070711`); (3) UI session reset, clear order, test and comment fixes. Known
limits (work-pending): `RoadCtxHold` is memory-only; a Tesla in a reduced-speed zone returns to the set speed ~12 s after mapd dies until it is relaunched.
Installed state: staged, takes effect at the next reboot.

## 10. `upcgate2pnw`: the upcoming-curve scan drops points the car has passed (`f6b14f5cce`)

Evidence: 2026-09-30 22:36 PT, OR-34 to I-5 loop ramp. A map point bound *behind* the car was read as an upcoming curve and flipped the Tesla CES to
experimental for 8 ticks. The Tesla CES `upcoming_curve` scan now ignores passed points, and the ICBM curve-DB pool masks both sources (the Lightning's
cross-source gap is closed). New change-only log event `ces_passed_mask` (slot, why). Opus review: the mask direction was verified on ~140k positions; the
extra work is 99% cache hits (no perf cost). The Lightning golden replay differs from the old set only by the added `ces_passed_mask` log events
(re-recorded; private data). Not changed (logged only): Lightning fail-open edge cases; `ces_passed_mask` has no dwell hysteresis.

## 11. `vtscnotch2pnw`: Tesla VTSC measures the map-curve notch from the car's speed (`6a5e4fa014`, `fd2d8cd8d5`, `a037278713`)

The minimum-slowdown notch for a map curve is measured from `min(set speed, vEgo)` held for the episode, in two tiers: never shallower than today's value, and a deeper
notch only beyond the hold horizon (`HOLD_TTA_S`). Kill switch: `curve.json` `tesla.vtsc_notch_vego`, default ON. New `mapRef` field on the VTSC tick record.
Replay: bend B (OR-34 left bend) 85 to 80 mph and 4.53 to 4.03 m/s^2, so **improved, not fixed**; fleet 756 same / 31 deeper / 1 shallower. Opus review: the first
version could carry up to +10.6 mph into a vision curve through the hold latch; fixed by the horizon gate (`a037278713`). An older, separate bug in today's
code (the hold latch can freeze the cap while a vision curve approaches) remains open. Staged, takes effect at the next reboot.

## 12. Coding policy note

From the evening of 09-30 the code for these three ships was written by Sonnet coders and reviewed by Opus reviewers. The channel tip was tested after the last
push (green, 5814 passed at `a037278713`).
