# CHANGELOG — 2026-10-19 (Monday)

All times PT.

## `curvedbpull2pnw` (branch `curvedbpull2pnw`, not yet pushed): the device pulls the approved curve-DB table

Owner 2026-10-19: "remove the laptop and run only on cloud in prod". Full description: [CURVEDB-AUTOINSTALL.md](CURVEDB-AUTOINSTALL.md).

- **New daemon** `curvedb_updated` (`system/curvedb/curvedb_updated.py`; `always_run`, TICI, `restart_if_crash`). `deviceState` only; gear via the `GearPark` param.
  Unmetered network only, hourly + on each offroad/Park transition. Never reboots. Each outcome (none published / auth / gateway down / hand-change hold /
  waiting / installed) has its own status and log line.
- **New library** `system/curvedb/swap.py`: port of the private ssh gate (decide, stage, limited loader child, `-auto` backup + rotation, swap with a re-check right before
  the rename, verify, recover). Same lock file as the gate. `decide()` is pinned to the gate by a parity test.
- **New toggle** `Disable Curve DB Auto-Install` (`DisableCurveDbAutoInstall`, default OFF, device-wide, not greyed) with the live state under it (`CurveDbAutoInstallStatus`).
- Tests: `system/curvedb/tests` (new directory, 73 tests) and `selfdrive/ui/tests/test_curvedb_toggle.py` (2). Mutation-checked: 21 mutants of the safety gates, all killed.
- Inactive until the gateway serves `GET /v1/curvedb/{dongle}/approved` (404 reads `none published`).
- Phase 3 switch (owner decision D12): after the first cloud pointer, run `--adopt` on the device once, so the laptop's last install counts as ours.
- Follow-up: the report body now matches the deployed gateway (state enum, `auto_utc` ISO string, null for empty shas); a 400 is logged loudly and not retried in a loop;
  an unreadable `IsOnroad`/`GearPark` param refuses and logs instead of crashing the daemon; `test_report_contract.py` pins the gateway's rules.
- Review fixes (Gemini 4 Argon): a vanished live file or a failed record write is an `error` status, not a daemon crash; the swaglog load check runs only when offroad or in Park with a settled
  selfdrived, reads only logs newer than the install, newest first, at most 50 files / 20 MB, and answers `unknown` (logged) past a cap or on an I/O error; the selfdrived uptime probe cannot throw
  (unknown makes `decide()` refuse); an unreadable toggle counts as disabled and an unreadable `OnPriorityNetwork` as metered, each logged once.

## `icbmdbraise2pnw` (branch `icbmdbraise2pnw`, not yet pushed): a running ICBM cap may be lifted by a curve-DB raise

Owner 2026-10-19 ("yes build the Terwilliger fix"). Drive 2026-10-19 21:00 to 21:01 PT, I-5 S, Terwilliger right curve (Lightning, stock ACC + ICBM taps). The curve DB read
`branchAmbiguous` for 7 s, the target fell to mapd's 46.9 mph and ICBM walked the set 52 to 47. The DB then came back with raise targets 52.6, 53.6, 56.5, but a running cap is
dec-only, so the set stayed at 47 for 15 more seconds (limit 50). Evidence: `drives/2026-10-19/i5-merge-slow/DRIVE_REPORT.md` (private).

- **Change** (`ces_pnw.py` `IcbmEpisode._db_raise`, `curvedb_live.py` `CurveDbLive.backed`): while a cap episode runs, a DB-backed raise (the winning candidate of the DB decision
  is a `raise` or `add` row, and the final source is `map` or `far`) may lift the set toward the DB target. The episode returns the existing `"inc"` command, so the Ford executor,
  its ceiling clamp and its RestoreGuard apply unchanged. A raise from mapd, vision or any other source is still refused.
- **Bounds:** the episode ceiling C (the set before the cap), the posted limit + 5 mph (`ICBM_DB_RAISE_OVER_MS`, owner rule for ICBM restores), and the restore cap. Vision still lowers
  the target (it is in the same minimum). The raise must be stable for `ICBM_RATCHET_CONFIRM_S` (0.6 s, the lowering path's own constant), and the value used is the lower of two ticks,
  so a flapping DB raise never taps. An unknown limit is no raise. A cap resumed after a gas press never raises (its driverSet guard would read our tap as the driver).
- **Not done on purpose:** the DB decision is NOT held across `branchAmbiguous`. On the left curve after Terwilliger that would lift 50.6 to 55.8 (the 2.5 m/s2 speed), the old complaint.
  With this change the left curve is unchanged (no DB number there, so the raise never fires).
- **Telemetry:** `icbmDbRaise` in `ces_events`: `acted` (the raise is published), `wait` (confirming), `capped` (the DB wants more than a bound allows), `noLimit`, `gas`, or null.
- **Lightning only:** through the existing curve-DB capability gate (`curvedb_v2_live`). The Tesla has no curve DB, so its ICBM/VTSC path is unchanged.
- Tests: `ces_pnw/tests/test_icbmdbraise2pnw.py` (14): the Terwilliger replay, the C bound, the limit + 5 bound, mapd/vision raise refused, left curve unchanged, flapping, unknown limit,
  the `backed` flag per DB direction, Tesla. Mutation-checked: no C bound, no limit + 5, no DB-only rule (step and flag), no confirmation each fail a test.
