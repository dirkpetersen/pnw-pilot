# CHANGELOG — 2026-09-24 (Thursday)

Continues [`CHANGELOG-2026-09-23.md`](CHANGELOG-2026-09-23.md). All times PT.

**Channel tip:** `origin/3devpnw` = `2c33ad09a8`, **GREEN: 3,763 passed, 0 failed** (Rule 9 gate).
**Installed on the truck:** `2c33ad0` — rebooted 07:46 (openpilot disengaged, in Park), back in 73 s, verified:
processes stable, `UnknownKeyName` = 0, clock-helper errors = 0, tracebacks = 0.

## 1. ✅ `clockvalid2pnw` (`37206c2d06`, `465e0a69ef`) — an unsynced clock is no longer trusted

Before time sync the 3X clock reads **2026-07-28** (systemd's build time — confirmed on the device:
`/lib/systemd/systemd` mtime 2026-07-28 15:04:45 UTC). The fork's checks only rejected pre-2020, so it passed.
Now one shared `wall_time_valid(t)` in `common/time_helpers.py` (upstream's `system_time_valid()` floor applied
to any timestamp: systemd mtime + 1 day, fails CLOSED if unreadable) is used by the metered budget (no metered
upload before sync), archive naming (random `.b<hex>` name before sync), `ces_events` `clockBad`, the NetLogger
day rotation, the curvedb shadow and the offline telemetry check. **Fable: SHIP.** Deferred: a host-only 24 h
test flake after a dev-host systemd upgrade (leadrate/steerpower harnesses); no positive "clock trusted" note.

## 2. ✅ `policeoffway2pnw` (`968e078ab3`, `2c33ad09a8`) — police on 2-lane highways, display-only

Port of the one useful commit from the stale `policemiss2pnw` (the other two would have reverted
`policeship2pnw`). While the police poll is armed at highway speed, reports now show on ANY road (US-97, OR-58, …
that mapd does not class as freeway), in the "NEARBY" box. Off a freeway: **no banner, no speedadjust slowdown**
(`cap` withheld). **Owner decision:** the single soundd chirp for a live report ≤0.5 mi ahead is KEPT off-freeway
(police never-miss rule). **Fable: SHIP-WITH-FIXES** — stale comment fixed; chirp decided by the owner.
