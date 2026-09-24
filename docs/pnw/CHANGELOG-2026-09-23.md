# CHANGELOG — 2026-09-23 (Wednesday)

Continues [`CHANGELOG-2026-09-21.md`](CHANGELOG-2026-09-21.md). All times PT.

**Channel tip:** `origin/3devpnw` = `1090a08386`, **GREEN: 3,728 passed, 0 failed** (Rule 9 gate).
**Installed on the truck:** `1090a08` — rebooted 22:19 (openpilot disengaged, in Park), back in 62 s, verified (below).

---

## 1. ✅ SHIPPED + INSTALLED `cesarchive2pnw` — telemetry reaches S3 the same day; upload order and metered rules

Commits `0217c78d81`, `aa2c045b4e`, `4c1cc390c8`, `c2ac9399da`, `1090a08386` (Fable fixes). **Fable: SHIP-WITH-FIXES**;
both fixes applied (metered listing materialised so the `pending=` note survives; boot-log stat errors logged).

**Root cause fixed:** a `ces_events` generation only reached `ces_archive` (and so S3) when it fell off the
8-deep ring. Since `parkedlog2pnw` (09-19) stopped logging while parked, the ring spans ~4 days of driving, so
telemetry arrived ~4 days late — nothing after 09-19 07:49 was in S3 on 09-23.

* **ces_events:** each generation is hardlinked into `ces_archive` **at rotation**; eviction unlinks `.N` only when
  its archive link is proven (inode + device + name); the live file also rotates at the **park transition** (≥64 KB).
* **net_events** (`location_servicesd` NetLogger): rotates at 10 MB or the UTC day, archived to `/data/pnw/net_archive`.
  **curvedb_obs:** snapshot copy to `/data/pnw/curvedb_archive` at selfdrived start.
* **Metered links:** only the four small logs (ces_events → curvedb_obs → boot logs → net_events), within
  **50 MB per Pacific day**, persisted in `/data/pnw/metered_budget.json`, fail-closed. qlog/qcamera/rlog/HD stay
  blocked on metered.
* **Unmetered links:** smallest first — small logs → qlog → qcamera → rlog → HD; `PASS2_INTERLEAVE` removed.
  `dcamera` is still never uploaded (privacy rule, unchanged).

**Verified on the device:** processes stable, `UnknownKeyName` = 0 (counted), 0 tracebacks; `curvedb_archive`
got its first snapshot; the 09-05 `net_events` generation was rescued; over the metered Starlink (SSID `KarlMoik`)
the ces backlog started uploading at 22:21 — `metered_budget.json` = `{"date": "2026-09-23", "bytes": 1868301}`.
The 8 stranded generations (09-19 07:49 → 09-23 12:53) had been hand-hardlinked into `ces_archive` earlier that evening.

**Open:** the unsynced device clock reads **2026-07-28**, not 1970, so it passes the pre-2020 "valid clock" guard —
mislabels archive names and can hand the metered budget a fresh "day" right after boot (≤ one extra 50 MB).
Tracked in the workbench `docs/work-pending/`.

## 2. Docs

* `71f0cfcafc` — Alan Polk article re-fetch (16 texts) + `PNW-PILOT-FEATURES.md` Lane Centering row and the
  BluePilot pinion-yaw deviation note (recovered from the retired `angleship-faithful` checkout).
