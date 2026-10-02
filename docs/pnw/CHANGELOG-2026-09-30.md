# CHANGELOG — 2026-09-30 (Wednesday)

Continues [`CHANGELOG-2026-09-28.md`](CHANGELOG-2026-09-28.md) (there is no separate 09-29 file; its ships are section 1 here). All times PT.

**Update 2026-10-01 (evening):** sections 17-20 shipped after this; the channel tip is `9d9de83b78`, **GREEN**, 6115 passed. The device still runs `10ef871`; the tip is staged for the next reboot.

**Update 2026-10-01 (later):** section 16 (`toggles2pnw`) shipped after this; the channel tip is `8e51763978`, **GREEN**, 5943 passed; the device still runs `acc42b5a9f` and has `8e51763978` staged (includes sections 13-14) for the next reboot, not yet rebooted.

**Update 2026-10-01:** sections 12-14 shipped after this; the channel tip is `4c48ef4075`, **GREEN**, 5870 passed; the device runs `acc42b5a9f` and has `4c48ef4075` staged for the next reboot.

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
code (the hold latch can freeze the cap while a vision curve approaches) was open here and is fixed in section 14. Staged, takes effect at the next reboot.

## 12. `madsquiet2pnw`: the Raven's silent MADS transitions are pinned by tests (`acc42b5a9f`)

Test-only plus a docstring fix in `madsquiet_pnw.py`. Pins the Raven's brake-to-steering-only and stalk-pull transitions as chime-free, so a change that makes them
audible fails a test. Shipped by owner decision 2026-10-01. Tip GREEN, 5820 passed. Installed on the device at the next reboot (no behaviour change).

## 13. `dmtests2pnw`: the four stale driver-monitoring tests now pin the fork's contract (`23fdf366ba`)

Test-only; no product code changed. `selfdrive/monitoring/test_monitoring.py` (12 tests) failed 4 tests on the channel base. Root cause: `dmon2pnw` (`eecc900e11`, 2026-06-22)
brought in the BluePilot dual-counter logic in `DriverMonitoring._update_events`; the tests still asserted stock single-counter behaviour and were never in `TEST_PATHS`, so
the channel check never saw them. They passed 10/10 at the upstream state and are not environmental. The tests now pin the fork's contract (the header comment lists each
deviation) and `test_monitoring.py|10` is in `TEST_PATHS`. Tip GREEN, 5832 passed.
**Open (owner decision, work-pending):** the deviations show the fork's DM is **weaker than stock even at `DmMode=0`** (a 2 s camera dodge snaps the counter back from orange/red;
wheel touch or gas clears alerts while still distracted; standstill freezes at the green threshold; 30 s face-loss grace). Whether to restore stock recovery semantics is not decided.

## 14. `vtschold2pnw`: the VTSC apex hold no longer freezes the cap above what a curve needs (`4c48ef4075`)

Tesla (Raven) only. The VTSC `hold` latch could keep the cap frozen above the brake envelope while a curve approached; the hold now only ever LOWERS the held value to the envelope
(never below the brake envelope), never raises it. Kill switch: `curve.json` `tesla.vtsc_hold_envelope`, default ON (OFF is bit-identical to the old behaviour). New VTSC tick field
`vtscHoldEnv` (`''`, `'env'`, `'speed'`). This closes the bug recorded in section 11 as open.
Evidence: replay of 134 engaged stretches 09-27 to 09-30 had 534 hold episodes; in 7 stretches the old car was more than 0.3 m/s faster than the new rule, in 2 more than 2 m/s,
worst 5.13 m/s at 12 m/s (09-30 21:43 PT): about one overspeed per 55 engaged minutes. Closed-loop fuzz: worst faster-than-ideal +4.47 to +1.02 m/s (22 to 1 of 1000 scenarios over 0.9 m/s);
fleet lateral acceleration never worse. The residual +1.02 m/s is the existing 10%-over-safe-speed hold margin. Opus review: SHIP (lower-only proven; switch OFF bit-identical; 12 of 12 mutants killed).
Tip GREEN, 5870. Staged on the device; takes effect at the next reboot. Known limits (work-pending): a car below its set speed that detects a curve can still accelerate under the cap;
a misspelled kill-switch key in `curve.json` is silently ignored.

## 15. Coding policy note

From the evening of 09-30 the code for the ships above was written by Sonnet coders and reviewed by Opus reviewers. The channel tip was tested after the last
push (green, 5814 passed at `a037278713`).

## 16. `toggles2pnw`: one toggle convention, three toggle changes, car graying (`61418263b7` ... `8e51763978`; opendbc `c2bd3fd4`, `7109dccd`)

Shipped 2026-10-01 after a Sonnet coder and three Opus reviewers. Channel tip `8e51763978`, **GREEN**, 5943 passed. Staged on the device, **not yet installed** (needs a reboot). pnw-pilot commits: `61418263b7` (toggle audit + [`TOGGLE-CONVENTIONS.md`](TOGGLE-CONVENTIONS.md)), `60db827c47` (phase 1), `8becdc0061` (phase 2, car graying), `bf9fb16f7b` (phase 3), `62d4569f1d` (review fixes), `8e51763978` (opendbc pin bump). `pnw-opendbc` `master-pnw` is now `7109dccd`.

**The rule (owner).** A toggle's default operational state is OFF; a feature that is on by default gets an opt-out toggle named `Disable X`. The row shows the live state. A toggle that applies to one car is greyed, never hidden, on the other car. An unknown car (no `CarParamsPersistent`, or a mock) leaves everything enabled. Car graying is one `CAR_GATED` table.

**Remote SSH.** Toggle renamed "Disable Remote SSH (Tailscale)", param `DisableTailscale`, default 0. It is enabled by default only once configured (an auth key file or saved node state exists); until then the status reads `unconfigured`, the toggle stays OFF and the device does nothing. Status words: connected, disconnected, unconfigured, connecting, installing, error. Toggle ON shows "disconnected - disabled by this toggle". Two consecutive `NetworkType.none` reads (about 60 s) show "disconnected - no network link". `TailscaleEnabled` is removed.

**Ford camera speed limit.** Toggle "Disable Ford Camera Speed Limit", param `DisableFordSignSpeedLimit`, default 0. Behaviour is identical by default. In British Columbia the owner turns the toggle ON until the camera's km/h limit has been measured. Mainland BC (for example Vancouver) already resolves to Canada, where the camera is off; Prince Rupert and Stewart resolve to AK, Windsor ON to MI and Niagara ON to NY. With no map limit the sign number is still used and read as mph, even with the toggle ON. That error only means less slowdown, never extra speed.

**Ford convenience features (new).** Toggle "Disable Ford Convenience Features", param `DisableFordConvenience`, default 0, Lightning only (greyed on the Tesla). It gates the only non-driving CAN write the comma makes: the Pro Power Onboard re-arm (0x455, `ProPowerArmer`). There is no tailgate or chime CAN write (those are As-Built/FORScan settings; the comma's chimes are on-device audio). The gate is a `ConvenienceGate` in the opendbc carcontroller that reads the persistent param at 1 Hz. It fails open before the first successful read and keeps the last good value after one. Toggle OFF to ON to OFF builds a fresh armer: the 3-press budget and 15-minute window reset, and it presses again about 9 s later at any standstill (including Drive or engaged at a red light). With the toggle ON, Pro Power follows the truck's own behaviour (its keep-on setting may reset at ignition). CAN output with the toggle OFF is byte-identical to before (4765 frames, simulated drive); with it ON, the output is the old one minus the 18 frames on 0x455.

**No migration.** The manager's `clear_all` deletes param files for keys that are no longer registered at start-up. The owner's comma had `TailscaleEnabled=1` and `FordSignSpeedLimit=1`; those files are deleted on first boot and the new `Disable*` params seed 0, so nothing needs migrating.

**Review findings of note.**
- Stale-param deletion hazard: removing or renaming a param key silently drops its stored value (see No migration above). Any rename of a live param must check the device's value first.
- "No internet" needs two consecutive reads, so a single `none` read does not flash "no network link".
- The camera protection in BC is partial (see the last bullet above): the sign number is still read as mph when there is no map limit. The note should read "only where the device resolves to a US state"; the current wording could be read as applying to BC.
- The Pro Power toggle persists while the device is in the Tesla, where it is greyed.

**Known limits, not fixed.** (1) `carcontroller.py` comments are slightly stale: `__init__` says a failed read keeps today's behaviour (true only before the first good read), and `update()` refers to "the guard below", which now lives in `_conv_disabled`. (2) `tailscale_pnw.py` `_none_reads` is not reset when `tick()` returns early (disabled or unconfigured), so after re-enabling a single `none` read can show "no network link" at once (display only). (3) The kept-last-value gate does not survive a card restart; a new gate fails open until its first successful read. (4) The UI tests build a stand-in `self`, not the raylib window, so the real rows have not been seen rendering. Tracked in the workbench work-pending item "toggles2pnw review follow-ups".

## 17. `dbfirst2pnw`: the Tesla's VTSC map notch becomes curve-DB-first (`cecd8d6df1`, `9d9de83b78`)

Owner rule: a curve the car has driven and trusts is decided by the curve DB, not by a flat notch; a never-recorded OSM curve is capped at 1.15 x the
posted limit at the car; everything else keeps today's notch. Kill switch `tesla.vtsc_db_first` in `curve.json`. Tesla only. Rule, codes, telemetry and
the install-order warning (code before a flagged table): [`VTSC-DB-FIRST.md`](VTSC-DB-FIRST.md). With the unflagged table that is deployed, every covered row is
unreliable, so the only driver-visible change is the 1.15 x cap on never-recorded roads with a known limit. Tests: tip GREEN 6115.

**Review (Opus x3).** The first mild-curve idea (`165195e065`) was not shipped: the replay used a lateral-acceleration bound of 2.7 where the shipped value is 3.59;
the polyline was measured over the whole horizon, not at the protected curve; the camera adds nothing at range; hysteresis made "only removes slowing" false.
Open follow-ups, none changing behaviour: the brain comment "at most ~2 mph lower, never higher" overstates (the brain output can differ in both directions with
the switch ON; only lowerings of up to 2 mph reach VTSC); the exporter nearest-first test is weak; coverage keys use rounded floats; the per-curve override circle
can miss a bend that is far from its centre (a data issue); the strict rule is not validated out of sample. Known side effect: the 1.15 x cap can deepen a
slowdown on a surface road with a low posted limit.

## 18. `policeahead2pnw` (`35cdb6206b` ... `9e4e32a9ed`)

A latched confirmed police report now anchors the police target at the announced limit + 5 mph once the report lies beyond an announced lower limit, from
the moment the look-ahead starts. Reduce-only, same slew, held per report. Look-ahead off or shadow = the old cap exactly. Found on a 2026-10-01 drive where the
car held 75 in a 60 zone for about 4 s because the police cap used only the current limit. 22 tests, mutation-checked. Telemetry `polAhead`, `polTgt`.

## 19. `restfar2pnw` (`6b741a0ebf` ... `252bf04366`)

The next rest area is shown up to 50 mi ahead along a tagged corridor (was 15 mi). Display-only; police and EV caps and the perpendicular rule are unchanged; with
no WayRef or heading the old reach applies and the mode change is logged once. Two review rounds added a U-turn guard and tighter tests.

## 20. `dmtext2pnw` (`10ef871923`)

Text only. Driver-monitoring `DmMode` Default is not stock openpilot; the on-screen text, comments and docs now say what it is: standard alert timeouts plus the
fork's recovery rules. No logic, constant, param or test expectation changed.
