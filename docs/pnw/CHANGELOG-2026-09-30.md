# CHANGELOG — 2026-09-30 (Wednesday)

Continues [`CHANGELOG-2026-09-28.md`](CHANGELOG-2026-09-28.md) (there is no separate 09-29 file; its ships are section 1 here). All times PT.

**Channel tip:** `origin/3devpnw` = `1afb551fa6`, **GREEN**, 5708 passed. **Installed on the device:** the tip as of the evening installs (~19:15 PT, and a reboot at 21:27 PT); `848d115fed` was the 09-29 ~22:00 PT reboot. Sections 2-3 were pushed during the day and are part of that evening install (`83c152e33f` ... `c0b4bd4499` are ancestors of the tip); sections 5-8 shipped the same evening.

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
