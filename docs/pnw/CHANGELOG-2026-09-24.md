# CHANGELOG — 2026-09-24 (Thursday)

Continues [`CHANGELOG-2026-09-23.md`](CHANGELOG-2026-09-23.md). All times PT.

**Channel tip:** `origin/3devpnw` = `668607cab5`, **GREEN: 4,098 passed** (Rule 9 gate).
**Installed on the truck:** `668607c` — rebooted 11:39 (openpilot disengaged), verified: processes up, MADS
alternativeExperience 1024, safety `ford`, curve DB loaded. Earlier installs today: `2c33ad0` 07:46, `0d24053`
~11:10, `3829160` ~11:28.

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

## 3. ✅ `restorehold2pnw` / `restorehold3pnw` (`0a5cb972dc`, `5cd59751a3`) — restore only waits for a real next curve

After an ICBM curve slowdown the SET+ restore is held only while another curve is ahead (map polyline or vision)
at a speed the truck could not carry; otherwise it restores at once. It never adds a slowdown and never carries a
ceiling below the set. **Fable: SHIP.** Seen working on OR-34 11:19: held 74 through the next bend, then back to 80.

## 4. ✅ `limitdropexact2pnw` (`afbf4c81a4`, `0d2405303b`) — at or under the limit, a drop goes to EXACTLY the new limit

Owner rule 1b: a posted-limit drop with the driver at or under the old limit sets exactly the new limit (09-24
09:18: the truck held 41 mph in a 25 because a 07-14 guard skipped under-limit drivers). Over the limit keeps the
same percentage over (rule 1). A first low reading after a map dropout must persist 2 s. **Fable: SHIP** after
F1 (dropout confirm) and F2 (stock-ACC test).

## 5. ✅ `curvedblive2pnw` (`b4f2588128` … `d0cfe3c728`, `3829160f07`) — learned curve DB v2 LIVE on the Lightning

The road table built from past drives sets ICBM's curve target both ways: lower to exactly sqrt(A/k); raise only
with a 1.25x curvature margin, capped at +15 mph and posted+10; unknown road/branch = no effect, logged as
`cdb2Why`. The data file is private (`/data/pnw/curvedb_v2/`, sha256-checked), not in this repo. A = **2.5 m/s²**
(`curve.json` `curvedb_v2_lat_a`; the owner raised it from 2.2 after a drive — review waived by the owner for that
constant). Tesla unaffected. **Fable: SHIP** after F1 (crash telemetry) and F4 (`notMin`). Open owner call: roads
without a posted limit currently get no DB effect in either direction.

## 6. ✅ `madsbrake2pnw` (`3c7592b616`, `668607cab5`) — a late brake keeps steering once the panda re-latches

09:20:36: the PCM dropped cruise and the brake landed 481 ms later, past openpilot's 450 ms MADS window, so steering
disengaged although the panda (600 ms) had re-allowed it. Frames 46–75 now arm lateral-only only when a fresh,
valid pandaStates says lateral is allowed and the brake was seen; a seen brake that still ends in a full disengage
logs a `madsbrake2pnw` warning. No panda change. **Fable: SHIP.** Not yet exercised on the road.

## 7. Docs — `skills/openpilot` (`4853fded1b`)

Coding rule (owner): code is written only by a background Opus agent at medium effort, never Sonnet, never in the
main session.
