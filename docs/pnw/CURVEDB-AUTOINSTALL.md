---
updated: 2026-10-19
status: current (code in `curvedbpull2pnw`; inactive until the gateway publishes an approved pointer)
---

# Curve DB auto-install (`curvedbpull2pnw`)

The device PULLS the approved curve-DB table from the upload gateway and swaps it in. No table data is in this repo;
this page describes the mechanism only.

## What runs

- **`system/curvedb/curvedb_updated.py`**: manager process `curvedb_updated` (`always_run`, TICI only, `restart_if_crash`). Subscribes to
  `deviceState` only. "Being driven" is the `GearPark` param (Rule 3), never `IsOnroad` and never `carState`.
- **`system/curvedb/swap.py`**: stdlib library (decide, stage, loader in a limited child, backup, rotate, swap, verify, recover).
  It uses THE SAME lock file as the private ssh gate (`/data/pnw/curvedb-swap-gate.log.lock`), so the two never swap together.
- **`system/curvedb/status.py`**: the text under the settings row.

## One cycle

Runs at boot (after a valid clock), on each offroad/Park transition, then hourly. Each step ends the cycle with its own status:

1. Toggle `Disable Curve DB Auto-Install` ON: `disabled`.
2. Wall clock not valid (`wall_time_valid`): `waiting for a valid clock`.
3. No network: `waiting for a network`. Metered (`effective_metered` from the uploader, so the same cost rule as drive files):
   `waiting for unmetered network`. No request is made in any of these.
4. `GET {API_HOST}/v1/curvedb/{dongle}/approved` with `Authorization: JWT <device token>`:
   200 pointer, 404 `none published`, 401/403 `error: auth HTTP <code>` (logged as an error), 5xx or timeout `gateway unavailable`
   (retry with backoff 60 s doubling to 30 min).
5. Live sha == pointer sha: `installed <sha8> (MM-DD)`; the swaglog `curvedb_v2_loaded` line is checked (`loaded` in the report).
6. Hand-change hold: `/data/pnw/curvedb_autoinstall.json` records the sha WE installed last. If the live sha differs (or there is no
   record) the device HOLDS (`held: table changed by hand`) and never overwrites. A missing live dir is restored from the newest
   `-auto` backup, then held. Resume with `python -m openpilot.system.curvedb.curvedb_updated --adopt` (records the live sha).
7. `decide()`: offroad, or Park with selfdrived up more than 120 s; at least 500 MB free on `/data`. Otherwise `waiting: <reason>`.
8. Download rows and manifest from the presigned URLs (30 s timeouts, size caps 4 MB / 64 KB, sha256 checked; URLs are never logged).
9. Under the lock: stage (fsync), the device loader on the staged copy (child process: nice 19, 600 MB address space, 120 s CPU, one BLAS
   thread, cwd `/`), backup `curvedb_v2.bak-<PT date>-auto[b..]`, rotate ONLY `-auto` backups (keep 5), re-check `decide()` right before
   the rename, rename, verify the live sha.
10. Record, report, log. **It never reboots.** The new table is loaded at the next drive start.

## Where to look

- `/data/pnw/curvedb-autoinstall.log`: one line per step. Also `cloudlog.event("curvedb_autoinstall", step, result, reason)`.
- Param `CurveDbAutoInstallStatus` (cleared at manager start): the one-line state plus ` @<epoch of last check>`; the settings row shows
  it with "checked N h ago".
- Param `DisableCurveDbAutoInstall` (persistent, default 0): the opt-out toggle, device-wide, never greyed.

## Gateway contract (what the device sends and expects)

Pointer response (200, JSON, at most 16 KB): `sha256` (64 hex), `rows_bytes` (int, 1..4194304), `manifest_sha256` (64 hex),
`manifest_bytes` (int, 1..65536), `rows_url`, `manifest_url` (https, presigned GET, short lifetime). Extra fields are ignored.

Report: `POST {API_HOST}/v1/curvedb/{dongle}/report`, JSON `{live_sha, pointer_sha, last_auto_sha, auto_utc, loaded, state, reason, free_mb, git_sha}`.
`state` is one of `current`, `installed`, `none`, `held`, `retry`, `error`, `waiting` (reports are sent only when the network check passed).
`reason` is at most 200 printable ASCII characters. Sent after each state change and at least daily; a failed report is logged and retried next cycle.

## Rollback

Toggle ON stops it within 30 s. The previous table is in `/data/pnw/curvedb_v2.bak-<date>-auto*`; copy it back by hand and run `--adopt`.
