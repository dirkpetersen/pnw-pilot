# CHANGELOG — 2026-09-15 (Tuesday)

Continues [`CHANGELOG-2026-09-14.md`](CHANGELOG-2026-09-14.md). Repos: **pnw-pilot** (channel `3devpnw`),
**pnw-opendbc**. Every change is reviewed by **Fable** (the only reviewer, `docs/CODING-POLICY.md`) before push, then
installed on the F-150 Lightning's comma 3X on its own reboot while openpilot is disengaged, and health-checked.

**Channel tip:** `origin/3devpnw` = `974767198b` (cesmodehold2pnw B).
**Installed on the truck:** `974767198b`, 19:04 PT, BootCount 239 — which carries every row below except the two
still queued. Earlier today: `fa540a8c43` (swaglogcap2pnw, 08:03 PT, BootCount 233) and `6896f85b4f`
(behindrun2pnw, confirmed running at BootCount 238, booted 18:29 PT).

## ✅ The map re-download leak is closed

`mapdgrace2pnw` (installed 09-14 19:21 PT) has held across every boot since: `coverage grace resolved: tile loaded
after 0.09–0.16 s (avoided_request=True)`, **no `uncovered … req` line at all**, and **wwan0 rx = 0** on the boots
measured. Today's boots did not even need the grace (the tile was already loaded at the first check). Maps on disk:
286 MB, WA tiles from 08-18 intact beside OR. That was ~14–17 GB of the owner's 23 GB September LTE bill.
Still open (owner's call): the IPv6 route metric that sends map/updater traffic over LTE while on WiFi; persisting
the retry state across boots.

## Networking

| Commit(s) | What changed | Notes |
|---|---|---|
| `4f84801a79` **pinconfigured2pnw** | **Owner decision** ("no just the configued ones"): only a network in `TetheringPriorityNetworks` — stationary or `mobile`, so Hannelore, Visitor and the iPhone — may end a manual WiFi pick. An unconfigured saved profile marked unmetered no longer can. `netcosttier_pin_kept reason=not_configured` says when one is ignored. | Fable SHIP: the unpinned ladder is untouched (probe: unpinned, unconfigured unmetered still wins); the U+2019 SSID matches case/whitespace-insensitively; the dropped arrival tracking is unobservable. 368 networkd + 5 UI tests; 17/18 mutants, 1 equivalent. Installed 07:56 PT (BootCount 232), healthy. **Consequence:** a network marked unmetered but not on the Priority list silently loses that power, and the UI does not show it. |
| `c2f0d96813` + docs `09f490fdc3` **hotspotretry2pnw** (built, queued) | A transient hotspot join failure costs one poll (20 s) instead of 5–15 min. `classify_join_failure` reads nmcli's stderr ("secrets were required", "ssid-not-found", …); the DEFAULT is `real`, so an unrecognised error behaves exactly as today. One free retry per configured `mobile` entry, returned only by a successful join. | Why: on 09-14 18:55 the phone refused with `WRONG_KEY` → `Secrets were required, but not provided`, yet the same profile joined cleanly at 21:50, 21:53, 22:13 and 09-15 07:39 — the phone was not awake. Fable SHIP-WITH-FIX, fix applied: `last_join` is now consumed when its blame is decided (a later unrelated blame inherited it and was swallowed) and the change-only log is re-armed by a successful join. Two tests added that fail without each half. 391 tests. |

## Storage

| Commit(s) | What changed | Notes |
|---|---|---|
| `fa540a8c43` **swaglogcap2pnw** | **Owner decision** ("yes"): the device log cap goes 2,500 → **20,000 files AND a 200 MiB ceiling**, whichever binds first, deleting oldest-first. About 14 days of logs instead of 42 h, ~170 MB typical. The young-file warning now names which limit forced the delete. | Fable SHIP. The running byte total avoids a directory stat per rollover; init scan 48–52 ms for 20,000 files; strict bound is 200 MiB plus the active file. Installed 08:03 PT (BootCount 233): already past the old cap at 2,505 files, `/data` free 9.3 GB, logmessaged clean. **Found:** athenad forwards these logs to comma's server with no metered gate (pre-existing) — see below. |
| `f02d6864e3` **athenalogmeter2pnw** (pushed, staged, rebooted 19:24 PT — install not yet confirmed) | The swaglog forward to comma's athena server now pauses while the link is metered, and resumes when it is not, matching the owner's "zero metered file traffic" rule. Source of truth is the `NetworkMetered` param (no new msgq sub). An unset param pauses and logs at ERROR. | Fable SHIP: the pause cannot interrupt an in-flight file, nothing is lost or double-sent, both edges are change-only. Steady state is ~12 MB/day; the exposure the cap raise added is a one-time ≤200 MiB backlog flush. Checked on the device: the 400 oldest logs carry the acked xattr, so there is no re-send loop. |

## Speed control

| Commit(s) | What changed | Notes |
|---|---|---|
| `57d79657bc` **behindrun2pnw** (installed, BootCount 238) | **Owner decision** ("yes"): a map curve the truck has already passed may no longer lower a RUNNING ICBM slowdown, nor hold back the restore. Start behaviour, vision candidates and curve timing are untouched. | Replay of the real weekend + 09-08 data: a passed point lowered a running target on 9 ticks and held it on 145; after, 173 ticks (43.3 s) of cap removed. Sun 09-13 15:46: restore 15.65 s → 7.15 s, set ends 60 mph not 50. Fable SHIP: on all 21 acted ticks the removed point was 30–71 m behind the TRUE position, an at-node point is never masked, and a multi-node curve keeps binding on the nodes ahead. 746 ces_pnw tests, 14/14 mutants, Tesla hash unchanged. |
| `16e8ebb503` + `974767198b` **cesmodehold2pnw** (installed together, BootCount 239) | **Owner decision** ("yes"): when the CES master mode cannot be read, each caller (CES, VTSC, the UI overlay) keeps its last good mode for 10 s, then falls back to Off with the overlay's NO-SIGNAL alarm. The second commit stops the legacy bool standing in for a failed read. | Fable APPROVE with two conditions, both met: B ships with A (A alone could run Standard while the overlay showed NO SIGNAL), and a **legacy-only** read failure must not arm the hold — fixed here, so a driver who just picked Off is not overridden for 10 s and the alarm cannot stick on while the master is readable. 1498 tests; 3 extra mutants on the fix. Installed 19:04 PT (BootCount 239): 1075 tests green on the rebase, ui/selfdrived/plannerd PIDs unchanged over 25 s, CES publishing ticks normally (`mode=experimental reason=stopLatch` while parked), `CESMode` reads 2, no hold and no NO-SIGNAL — i.e. the master is readable and the new path is inert, which is the intended steady state. |

## In progress

- **Gas-set speed loss (owner report, 08:45 PT):** "once I accelerated and lift the foot of the gas one pedal
  breaking kicks in and the truck breaks for a second and then the cruise control takes over with the then lower
  speed". The 1.0 s wait the owner chose on 09-13 lets one-pedal regen bleed speed before stock ACC engages.
  Measuring the loss per episode from the logs, then options (shorten the wait, fire once the deceleration is steady,
  or tap the speed back up). Report: `drives/2026-09-15/gasset-regen-loss/`.

## Operations

- **The truck's address changes every boot on Visitor.** 18:55 PT it was `10.16.105.112`; after the 19:03 reboot it
  came back as `10.16.106.98` — a different /16 host, so a saved IP is useless. The CloudWatch `CLIENT_IP` locator
  found it both times in one query. Its SSH host key is identical at `10.16.105.112`, `10.16.56.229`,
  `172.20.10.10` and `192.168.1.79`, which is how a new lease is confirmed to be the same device and not a stranger.
- **Earlier in the evening the dev host could not reach the truck at all** — default route `172.27.240.1`, 100 % loss
  to `192.168.1.1` and `.79`, port 22 unreachable — while the truck was online and checking in from `192.168.1.79`.
  That was a dev-host networking problem, not a truck fault, and it cleared when the host joined Visitor.
- **Two pre-existing exceptions seen at every boot, neither ours:** `athenad` fails its websocket connect to comma's
  server and retries (we do not use that server for drive data — uploads go to the self-hosted S3 gateway), and
  `soundd` trips `assert stream.active` once at startup and is respawned. Both were present before tonight's
  installs; logging them here so they are not re-discovered as new.

## Deferred to the owner

The 09-14 list still stands (tailgate chime FORScan session; Pro Power `7D0-10-03`; police off-freeway display and
gate; RES vs truck memory; stock dropout keeps steering; brake-release auto RES; the deleter policy; map downloads
over metered links; 12 V multimeter; relayMalfunction harness; DM video check), plus:
- the **IPv6 route metric** that sends map and updater traffic over LTE while on WiFi (a network change, stranding risk);
- whether a network marked unmetered but NOT on the Priority list should show that state in the UI;
- whether swaglog forwarding and drive uploads should treat a *guessed*-metered priority WiFi the same way;
- the **12 V rail**: the 09-14 evening brown-out reboots at 10.9–11.2 V are unexplained and need a multimeter.
