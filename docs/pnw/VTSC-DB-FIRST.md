# VTSC map notch becomes curve-DB-first (`dbfirst2pnw`)

Status: pushed to `3devpnw` 2026-10-01 (`cecd8d6df1`, `9d9de83b78`), channel tip GREEN. Staged on the device, **not yet running** at the time of writing.
Tesla only: VTSC is inert on the Lightning. Builds on [`VTSC.md`](VTSC.md) and [`VTSC-RELEASE-LATER.md`](VTSC-RELEASE-LATER.md).
No new param key, no panda change. This file holds no table values and no coordinates; the curve table itself is private.

## 1. The rule

Until now, `curvefloor2pnw` (2026-08-18) gave every OSM-flagged curve a minimum slowdown: the map notch of set speed minus 4.5 m/s, whether or not
the car had ever driven that curve. The owner's replacement: ask the curve database first.

| Situation at the curve | What decides the cap | `mapSrc` |
|---|---|---|
| Row exists, **trusted** (reliable) | The DB decides. The OSM notch is not folded in at that point. A per-curve override still wins over the DB. | `db` |
| **Never recorded** (`noAnchor`) and the posted limit at the car is known | Cap = `UNCOVERED_CURVE_LIMIT_RATIO` x posted limit = **1.15 x**. Only the limit AT THE CAR is used. | `cap15` |
| Never recorded and the limit is unknown | Today's notch, unchanged | `notch` |
| Driven but the row was **refused** (`noAuthority`) | Today's notch | `notch` |
| Anchor whose branch was never recorded | Today's notch | `notch` |
| Row marked **lowerBound** (a saturated pass: the true value is at least what was measured, so it is not trusted) | Today's notch | `notch` |
| No DB at all, or the DB is switched off | Today's notch (the OSM fold) | `osm` |

The ratio was 1.20 in the first draft; the owner changed it to 1.15.

**cap15 can deepen a slowdown.** On a never-recorded surface road with a low posted limit the 1.15 x limit cap can be lower than the old notch
would have been. It never applies where a trusted row exists. Tracked as a work item.

## 2. Coverage codes and telemetry

The curve brain publishes a per-vertex coverage code `cov`:

| `cov` | Meaning | VTSC `mapCov` |
|---|---|---|
| 1 | covered and reliable | `1` |
| 0 | uncovered (never recorded) | `0` |
| 2 | covered but unreliable (lowerBound) | `u` |
| 3 | driven but refused | `r` |
| 4 | anchor whose branch was never recorded | `b` |

`mapCov` is `?` when unknown. Per-tick fields: `mapSrc` (`db` / `cap15` / `notch` / `osm`), `mapCov`, `mapCap15` (VTSC overlay and `VTSC_TELE_KEYS`) and `cbCov` (the brain's
`TELE_KEYS`). New fields must also be listed in the key tuple that `ces_pnw` copies into `ces_events`, or they silently never reach the log.

## 3. The table format change

The table gained an optional 5th element per branch (`1` = lowerBound) and a manifest field `"flags":"lowerBound"`. The exporter takes `--flags`.
The `curvedb_v2_loaded` log event carries `flags` and `lower_bound_rows`. A table without the 5th element loads as before, and with it every covered
row counts as unreliable (`mapCov` `u`), so the notch stays. A table that declares `flags` but has none logs a "declares no lowerBound" warning.

### 3a. Raise-only rows (`curvedbraiseonly2pnw`)

A branch refused ONLY by the date-spread rule ("dates disagree": a merging side road pooled into the branch window; it passed every other owner
rule - >= 2 dates, not Tesla-only, no ramp, class known) can be exported (`v2_live_export.py --raise-only`) as a **raise-only row**: `k` = the SHARPEST
single pass on the branch, branch element 5 = `2` (the element is now a bitmask: 1 lowerBound, 2 raiseOnly), element 6 = the fastest recorded pass
speed (m/s), manifest `"flags":"lowerBound,raiseOnly"` (whichever are declared) and `raise_only_rows`. `rows_with_authority` / `cdb2Rows` /
`cbRows` do not count them. Without the option the export is byte-identical to before.

- **Lightning:** a raise-only row may only RAISE a map/far candidate: any other direction is "none"; all existing caps apply (+15 mph, posted + 10,
  <= set) plus fastest recorded pass + 5 mph; vision stays in the min-pool and still bounds the raise. It never takes part in the ADD scan. It ignores
  `lowerBound`, as granted rows do.
- **Tesla: no behaviour change.** `match_at` answers `noAuthority` (exactly a refused row) unless the caller passes `allow_raise_only=True`, which only the
  Lightning's per-candidate replacement does. So the brain's coverage is 3 as before, `scan_ahead` and `extra` never contain one, it never lowers a cap
  and cannot make db-first drop the notch.
- Loader: flag 2 without the manifest declaring `raiseOnly`, a missing speed or `k`, an unknown manifest flag or a wrong `raise_only_rows` is LOAD FAILED
  (DB OFF on both cars). An older loader rejects the whole file (malformed branch), so INSTALL ORDER below applies: code before table.
- Telemetry: `cdb2RO` (`matched` / `acted` / null), `cdb2Last` (the table's last drive date); `curvedb_v2_loaded` carries `raise_only_rows` and `age_days`
  (or "age unknown" without a valid clock), and a `cloudlog.warning` when the table is older than 14 days.

## 4. INSTALL ORDER (critical)

**The code must be on the device before a flagged table (and before a raise-only table, `curvedbraiseonly2pnw`).** An older loader fails on a flagged table (`anchor 1: malformed branch`) and the curve
DB goes OFF on **both** cars. A rollback restores the previous unflagged table first, then the code can move. Validate a new table with the
device's own loader before swapping it in, and keep the previous directory as a backup.

## 5. Kill switch

`/data/pnw/curve.json` key `tesla.vtsc_db_first` (default ON). `{"tesla": {"vtsc_db_first": false}}` restores the old notch behaviour exactly
(proven byte-identical against the previous tip on a replay of 215,673 recorded ticks). A bad value is treated as OFF and reported with a loud error.
Merge the key into the file; never delete the file.

## 6. First-drive checks

1. Any `slSat` / `strPrs` within about 10 s after a `mapSrc=db` tick (the key safety check: a DB-decided curve must not saturate the steering).
2. Gas overrides at `cap15` points.
3. How often `mapCov` is `u`.
4. `cbCov` is non-zero on covered roads.

## 7. Review

Three Opus reviews. The first mild-curve idea (`165195e065`, branch `vtscmild2pnw`) was **not shipped**: its replay used a lateral-acceleration
bound of 2.7 instead of the shipped 3.59; the polyline curvature was measured over the whole horizon rather than at the protected curve; the camera adds
nothing at that range; and the hysteresis/flip logic made the claim "it only removes slowing" false. It was superseded by the owner's DB-first rule.
Open follow-ups (comment wording, a weak exporter ordering test, exact-float coverage keys) are tracked in the workbench work-pending list.
