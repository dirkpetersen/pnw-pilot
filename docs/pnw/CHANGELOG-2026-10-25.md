# CHANGELOG — 2026-10-25 (Sunday)

All times PT.

## `curvedbmarker2pnw` (branch `curvedbmarker2pnw` in pnw-pilot, not yet pushed): curve-DB load marker

Why: cloud alarm `comma-curvedb-run-failed` (2026-10-25). The run held again because the device reported `loaded: false` for
live table eb255743 with the reason `swaglog scan cap (50 files) reached before a load line was found`. selfdrived logs
`curvedb_v2_loaded` once, at load. After a few hours of uptime that line is older than the newest 50 swaglog files, so
`swap.verify()` answers `unknown`. The sticky `loaded_sha` (`curvedbloaded2pnw`, 10-23) only helps after one scan succeeds,
and none did.

- **selfdrived** (`selfdrive/controls/lib/ces_pnw/curvedb_live.py`). After a successful load of the LIVE table
  (`/data/pnw/curvedb_v2`), the load thread writes `/data/pnw/curvedb_loaded.json` = `{"sha256": <manifest sha>, "utc": <wall
  clock>}` (tmp file + `os.replace`). A load of another directory (tools, tests) writes nothing. A write error is a loud
  `cloudlog.error`; the table stays in use. Never in the control loop.
- **curvedb_updated** (`system/curvedb/curvedb_updated.py`). When the live table is the approved one, it reads the marker
  first. Marker sha == live sha: `loaded`. Marker sha != live sha and `utc` >= the rows file mtime (install time):
  `mismatch` (loud error, as before). No marker, or a different sha written before the install: the old swaglog scan, unchanged.
  A marker that exists but is not valid JSON with a 64-hex `sha256` and an int `utc` is logged loudly
  (`verify marker_unreadable`) and the scan runs. The marker is a tiny read, so it is also used while driving (the scan gate
  stays for the scan). The sticky `loaded_sha` logic works on the marker verdict too.
- **Clock rule.** selfdrived often loads at boot before the clock syncs, so the marker `utc` can be too early. A matching sha is
  proof whatever the clock said, so `loaded` does not look at `utc`. Only `mismatch` needs `utc` >= install time; a too-early
  `utc` then falls back to the scan (the safe side).
- Report contract unchanged.
- Tests: `system/curvedb/tests/test_updater.py` (+7: marker match is loaded with no scan, even while driving; other sha after the
  install is a mismatch and clears `loaded_sha`; other sha before the install falls back to the scan; 4 malformed markers are
  loud and fall back). `selfdrive/controls/lib/ces_pnw/tests/test_curvedblive2pnw.py` (+3: the live load writes the manifest
  sha; another dir writes nothing; a write error is loud and the table stays loaded). `roaddb_fixture.isolate` redirects the
  marker. curvedb + test_curvedblive2pnw 226 -> 236 passed, 1 skipped; system/curvedb + ces_pnw 2201 passed, 1 skipped.
  8 mutants, all killed.
- Note: the first `loaded: true` for eb255743 needs one drive start with this code (the marker is written at load).
- Follow-up (channel tip red after the push): `tools/curvebrain/golden.py` patches `curvedb_live.DATA_DIR` to its own
  table, so its loads counted as the live table and tried to write `/data/pnw/curvedb_loaded.json` on the laptop; the error
  text carried the thread-id tmp name, so two runs differed. The golden harness now redirects `LOADED_MARKER` into its temp
  dir (like `roaddb_fixture`). No other tool loads `CurveDbLive` with `DATA_DIR`. tools/curvebrain + curvedb + ces_pnw:
  2221 passed, 1 skipped.
