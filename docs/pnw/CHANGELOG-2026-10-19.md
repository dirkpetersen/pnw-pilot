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
