# CHANGELOG — 2026-10-15 (Thursday)

All times PT. Committed locally on `curvedbraiseonly2pnw`; not yet reviewed, pushed or installed.

## 1. `curvedbraiseonly2pnw` (owner-approved 2026-10-15, "fix that curve database")

- **Problem.** A branch refused at build time by the date-spread rule ("dates disagree", `roadtable.row_verdict`) was exported with `k = null`, so the car saw
  `noAuthority`. A well-driven curve whose branch window pooled a merging side road lost its row, and the Lightning's ICBM fell back to mapd's raw rating
  (~10 mph too low on every pass).
- **Change.** `v2_live_export.py --raise-only` exports such a branch (and ONLY such a branch) as a raise-only row: `k` = the sharpest single pass
  (`roadtable.raise_only_info`; honours `exclude_date`), branch element 5 = bitmask 2, element 6 = the fastest recorded pass speed, manifest flags
  `lowerBound,raiseOnly` + `raise_only_rows`. `PassObs` gains `v_ms` (pass speed at the anchor; NaN in older tables - such a branch does not qualify).
  Lightning: may only raise, capped at fastest pass + 5 mph and every existing cap; never in the ADD scan. Tesla: byte-identical behaviour (coverage 3).
  Semantics: `docs/pnw/VTSC-DB-FIRST.md` section 3a.
- **Telemetry.** `cdb2RO`, `cdb2Last` on the tick; `curvedb_v2_loaded` gains `raise_only_rows`, `age_days` and a warning when the table is older than 14 days.
- **CODE BEFORE TABLE.** The new file is rejected loudly (malformed branch, DB OFF on BOTH cars) by a loader without this change. Install the code, verify
  the boot log shows `curvedb_v2_loaded` with `raise_only_rows`, then install the table; on rollback restore the previous table first. A table built
  WITHOUT `--raise-only` loads in both old and new code.
- **Golden (`tools/curvebrain`).** Against the 2026-09-27 set with `--exclude cdb2RO --exclude cdb2Last`: no `pub`/`rec`/`st` line changes; only `log`
  differs, on the lines carrying the `curvedb_v2_loaded` event (new fields `age_days`, `raise_only_rows`). The goldens need a re-record after review.
- **Test fixture.** `roaddb_fixture.isolate` writes today's date as `last_date` so the stale-table warning cannot fail nothing-logged tests; `write_db`'s default
  stays constant (the golden records through it).
