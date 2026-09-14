# CHANGELOG — 2026-09-14 (Monday)

Continues [`CHANGELOG-2026-09-13.md`](CHANGELOG-2026-09-13.md). Repos: **pnw-pilot** (channel `3devpnw`),
**pnw-opendbc**, **pnw-panda**. Overnight autonomous work on the owner's directive (2026-09-14 ~00:45 PT): "keep
working overnight on all open items that do not require my input, push these items to the truck". Each change is
reviewed by **Fable** (the only reviewer, `docs/CODING-POLICY.md`) before push, then installed on the F-150
Lightning's comma 3X on its own reboot while openpilot is disengaged, and health-checked. This file is updated
as each change ships.

**Channel tip:** `origin/3devpnw` = `a0dd8097ee` (unreadhold2pnw). **Installed on the truck:** `a0dd8097ee`
(07:21 PT).

## Networking — arbiter logging

| Commit(s) | What changed | Notes |
|---|---|---|
| `6ee95bac1f` **smallfix0914pnw** | The network arbiter logs every change of the active WiFi connection (`network_arbiter_active_changed from/to/by`), including changes it didn't make (`by=external`, e.g. NetworkManager autoconnecting KarlMoik after the iPhone hotspot drops) and bring-ups that didn't land (`by=not_as_requested`). A failed read is never logged as a change. Bring-up log lines now say what they leave (hotspot / a client connection / nothing / unreadable) instead of always "dropping hotspot". Logging only; no change to what the arbiter decides. | Fable APPROVE (change-only, no new nmcli call, 305/305 tests). Installed 01:14 PT; verified live: `KarlMoik → iPhone, by=arbiter` at boot. Motivated by the unlogged 2026-09-13 22:01 PT switch. |
| `ff811dea0e`, `a0dd8097ee` **unreadhold2pnw** | A failed read of the active WiFi connection no longer makes the arbiter raise the hotspot (or re-`con up`) over a working link. Before, one nmcli hiccup plus the connected AP missing from that scan bounced a working link for 20–40 s. The hold applies only when there is something to hold (a link seen at the last good read, or our own bring-up in flight), so the first connection at boot is never delayed. It is bounded: after 120 s of consecutive failed reads the arbiter acts as before and logs an ERROR once. Hold and release are logged change-only. | Fable: first pass SHIP-with-gate (a boot delay of up to 120 s without it); the gate was applied exactly as Fable prototyped, with a boot test and 2 mutants killed; networkd 297/297. Installed 07:21 PT; healthy (no hold events at boot, on KarlMoik). |

## Diagnostics — ACC dropout logging

| Commit(s) | What changed | Notes |
|---|---|---|
| `a568f3de9a` **smallfix0914pnw** | `accdrop_pnw.STALE_ROUTE_S` 70 → 75 s, clear of loggerd's 72 s fallback segment rotation, so a stalled-encoder segment no longer flags `routeStale` falsely. Logging only. | Fable APPROVE. Same install as above. |

## In progress (not shipped yet)

- **`behindgate2pnw`**: ICBM must not START a slowdown for a map curve the truck already passed (the 09-08
  69 → 44 mph phantom, the 09-05 freeway 40 mph target, the weekend 40 → 28 / 59 → 52 cuts).
- **`policemiss2pnw`** (`cfa47c85c9`, `2607226b63`, `9ad5f391af`, in Fable review): the week's proxy logs show
  the police line displays only on roads mapd tags "freeway" (209 of 460 min at ≥45 mph not displayable, 4
  reports passed hidden). Reports already fetched vanished when polling was gated or failed. The speed gate
  re-checked every 60 s (first poll p50 38 s late). Fixes 1–2 (keep fetched reports; 5 s gate re-check, no
  backoff for our own link) ship after review. Fix 3 (display police off-freeway) contradicts the design doc
  and waits for the owner. Report: `drives/2026-09-13/police-miss-week/DRIVE_REPORT.md`.
- **`gearunknown2pnw`** (pnw-opendbc `7163a85522`, pin bump `6a31327623`, in Fable review): Ford carstate
  reports `gearShifter=unknown` until `PowertrainData_10` is seen. It fixes loggerd's parked-video gate trusting
  an invalid gear. The parknorec2pnw quiet-CAN charging gap needs a capability-gated canValid follow-on (74
  other platforms would break with a global change), which is an owner question.
- **Pro Power (done, analysis only):** APIM `7D0-10-03` reads PPOOOVOS already **on** (partially confirmed; one
  FORScan read-only look settles it). Nothing cleared Pro Power in a 66-min overnight watch. Report:
  `drives/2026-09-14/propower-overnight-watch/DRIVE_REPORT.md`.
- **`leadlossr2pnw`**: lead-loss-hold shadow review (49 events, 08-18 → 09-13), plus a Rule 2 fix for the bare
  `except Exception: pass` around its logger in `longitudinal_planner.py`.

## Deferred to the owner

Tailgate chime FORScan session (tooling ready); Pro Power: FORScan read-only look at APIM `7D0-10-03`; police off-freeway display / off-freeway slowdown / lower the 45 mph gate / raise the proxy's 20-alert cap; capability-gated canValid follow-on for the charging-recording gap; RES restores the truck's memory vs the driver's set; stock
dropout keeps steering (panda change); brake-release auto RES; the deleter policy when storage is full of
un-uploaded drives; map downloads over metered links; the Fix B `coast_bias` default; `mapFlr` keep/drop;
`curveoverride2pnw`; 12 V multimeter; relayMalfunction harness check; a driver-monitoring video check.
