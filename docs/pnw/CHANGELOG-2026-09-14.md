# CHANGELOG — 2026-09-14 (Monday)

Continues [`CHANGELOG-2026-09-13.md`](CHANGELOG-2026-09-13.md). Repos: **pnw-pilot** (channel `3devpnw`),
**pnw-opendbc**, **pnw-panda**. Overnight autonomous work on the owner's directive (2026-09-14 ~00:45 PT): "keep
working overnight on all open items that do not require my input, push these items to the truck". Each change is
reviewed by **Fable** (the only reviewer, `docs/CODING-POLICY.md`) before push, then installed on the F-150
Lightning's comma 3X on its own reboot while openpilot is disengaged, and health-checked. This file is updated
as each change ships.

**Channel tip:** `origin/3devpnw` = `0167cd2575` (smallfix0914pnw). **Installed on the truck:** `0167cd2575`
(01:14 PT).

## Networking — arbiter logging

| Commit(s) | What changed | Notes |
|---|---|---|
| `6ee95bac1f` **smallfix0914pnw** | The network arbiter logs every change of the active WiFi connection (`network_arbiter_active_changed from/to/by`), including changes it didn't make (`by=external`, e.g. NetworkManager autoconnecting KarlMoik after the iPhone hotspot drops) and bring-ups that didn't land (`by=not_as_requested`). A failed read is never logged as a change. Bring-up log lines now say what they leave (hotspot / a client connection / nothing / unreadable) instead of always "dropping hotspot". Logging only; no change to what the arbiter decides. | Fable APPROVE (change-only, no new nmcli call, 305/305 tests). Installed 01:14 PT; verified live: `KarlMoik → iPhone, by=arbiter` at boot. Motivated by the unlogged 2026-09-13 22:01 PT switch. |

## Diagnostics — ACC dropout logging

| Commit(s) | What changed | Notes |
|---|---|---|
| `a568f3de9a` **smallfix0914pnw** | `accdrop_pnw.STALE_ROUTE_S` 70 → 75 s, clear of loggerd's 72 s fallback segment rotation, so a stalled-encoder segment no longer flags `routeStale` falsely. Logging only. | Fable APPROVE. Same install as above. |

## In progress (not shipped yet)

- **`unreadhold2pnw`** (`393acc027f`): an unreadable active-WiFi read no longer raises the hotspot over a working
  link (Fable-confirmed medium bug: a 20–40 s bounce from one nmcli hiccup). The hold is bounded to 120 s, then
  today's behaviour plus a loud ERROR. In Fable review.
- **`behindgate2pnw`**: ICBM must not START a slowdown for a map curve the truck already passed (the 09-08
  69 → 44 mph phantom, the 09-05 freeway 40 mph target, the weekend 40 → 28 / 59 → 52 cuts).
- **`policemiss2pnw`**: Waze police misses. Reports already shown stay visible while polling is gated or
  failing; polling resumes promptly after our own link problems instead of backing off. Measurement still in
  progress.
- **`gearunknown2pnw`**: Ford carstate reports `gearShifter=unknown` until `PowertrainData_10` is seen (opendbc
  `cf50ecad`). Closes parknorec2pnw's quiet-CAN charging gap and loggerd's parked-video gate trusting an
  invalid gear.
- **`leadlossr2pnw`**: lead-loss-hold shadow review (49 events, 08-18 → 09-13), plus a Rule 2 fix for the bare
  `except Exception: pass` around its logger in `longitudinal_planner.py`.
- **Pro Power**: a read-only overnight CAN watch (who clears Pro Power ~20 s after arming) and a decode of APIM
  `7D0-10-03` from the saved As-Built read.

## Deferred to the owner

Tailgate chime FORScan session (tooling ready); RES restores the truck's memory vs the driver's set; stock
dropout keeps steering (panda change); brake-release auto RES; the deleter policy when storage is full of
un-uploaded drives; map downloads over metered links; the Fix B `coast_bias` default; `mapFlr` keep/drop;
`curveoverride2pnw`; 12 V multimeter; relayMalfunction harness check; a driver-monitoring video check.
