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
- **Review fixes (Gemini 4 Argon):** (1) a DB-raise SET+ tap still in flight when the curve clears used to move the set after the restore baseline was snapped, and the restore
  died silently; the clear snapshot now follows our own upward taps (inside the grace window of our last raise tap, at most `ICBM_HOLD_LATE_TAPS` taps in total). (2) The raise is
  published with `why: "dbRaise"` and SpeedAdjustController reads those SET+ taps as ICBM's own even while it holds a cap (before: a police/zone cap was dismissed as a driver override).
  Closed loop through the real Ford executor + SpeedAdjustController in `speedadjust_pnw/tests/test_icbmdbraise_closed_loop.py`; mutation-checked (4 mutants killed).

## `curvedbramps2pnw` (branch `curvedbramps2pnw`, not yet pushed): ramp/link branches get curve-DB rows

Owner 2026-10-19 ("ramp option A"): a ramp curve (the I-84 to I-5 ramp, where ICBM used mapd's cautious 38 to 45 mph and the curve needs ~49 at 2.5 m/s2) must be able to get a row.

- **Build** (`tools/curvedb`): new `V2Params.allow_ramps`, **default OFF in code**, set by `v2_build.py --allow-ramps` (the private pipeline passes it when `CURVEDB_ALLOW_RAMPS=1`).
  With it: motorwayLink/trunkLink/primaryLink points may seed anchors (still >= 40 mph), and `row_verdict` no longer refuses `ramp (...)` or `extent reaches a ramp`.
  **Every other refusal stays**: >= 2 PT dates, spread, never Tesla-only, unknown class, classified extent, taint. Without the flag the table, rows and export bytes are unchanged.
- **Pricing**: a granted row that touches a link (a **link-class anchor, or a mainline anchor whose extent reaches a link**) is priced with `k` = the **sharpest single pass** on the branch (`roadtable.k_worst`, the same passes the verdict pools, `exclude_date`
  honoured), not the median across dates. Rows touching no link keep the median. Done inside `row_verdict`, so the export, `v2_replay.py` and `rows.json` use one k. Reason: ramp curvature varies along the ramp; in the first replay 31 % of the new link curve rows had a pass at
  >= 2.8 m/s2 at the row speed (16 % for existing rows). Replay after the change: **0** (every link row now sits at exactly 2.5 at its sharpest pass); link-row speed drop median 3.5 %,
  p90 12 %, 11 of 77 rows more than 10 mph slower.
- **Export**: manifest `flags` gains `ramps` and `ramp_rows`. **Car**: `CurveDbLive.gate` and `CurveBrain._gate` lift the ramp-class refusal only for a file that declares `ramps`; unknown
  class is still refused. `icbm_shape.tick_gate` (polyline shape) is untouched.
- **Install order**: code first, then table. A loader without this change refuses a `ramps` file ("flags not understood", DB OFF on both cars).
- **On-ramp vs off-ramp**: the build cannot tell them apart (no OSM link direction; a pass stores only its minimum extent speed), so all links are allowed.
- Tests: `tools/curvedb/tests/test_v2_ramps.py` (20), car side in `test_curvedblive2pnw.py` and `test_curvebrain_need_pnw.py`. Mutation-checked.
