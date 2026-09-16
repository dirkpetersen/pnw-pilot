# CHANGELOG — 2026-09-15 (Tuesday)

Continues [`CHANGELOG-2026-09-14.md`](CHANGELOG-2026-09-14.md). Repos: **pnw-pilot** (channel `3devpnw`),
**pnw-opendbc**. Every change is reviewed by **Fable** (the only reviewer, `docs/CODING-POLICY.md`) before push, then
installed on the F-150 Lightning's comma 3X on its own reboot while openpilot is disengaged, and health-checked.

**Channel tip:** `origin/3devpnw` = `efaf459124` (gassetwait2pnw).
**Installed and verified on the truck:** `c519b70b25` (hotspotretry2pnw), 21:06 PT, BootCount 241 — which carries
every row below except gassetwait2pnw. Tonight's sequence, one reboot each: `4f84801a79` pinconfigured2pnw
(07:56 PT, BootCount 232) · `fa540a8c43` swaglogcap2pnw (08:03, 233) · `6896f85b4f` carrying behindrun2pnw
`57d79657bc` (confirmed running at 238, booted 18:29) · `974767198b` cesmodehold2pnw A+B (19:04, 239) ·
`f02d6864e3` athenalogmeter2pnw (19:24, 240) · `c519b70b25` hotspotretry2pnw (21:06, 241).
`efaf459124` gassetwait2pnw (21:10, 242) — verified at 21:33 PT once the truck appeared on the iPhone hotspot:
selfdrived/plannerd/ui PIDs unchanged over 25 s, the new constants live in the running tree, and the
`accelUnknown` ERROR path has never fired. Four tracebacks since boot, all benign and all pre-existing
categories: 2 athenad websocket retries, 1 soundd `assert stream.active` at startup, 1 uploader
`upload_failed` (INFO, a transient during the KarlMoik → hotspot switch). **Trap worth recording:** grepping
the swaglogs for `gassetwait2pnw` returns 3 hits that are the **updater echoing the commit message**, not the
feature's own log line — the same trap that made a `ces_mode` grep look alive earlier tonight. Check the
daemon field, never the bare count.

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
| `944f5680f4` + docs `c519b70b25` **hotspotretry2pnw** (installed 21:06 PT, BootCount 241) | A transient hotspot join failure costs one poll (20 s) instead of 5–15 min. `classify_join_failure` reads nmcli's stderr ("secrets were required", "ssid-not-found", …); the DEFAULT is `real`, so an unrecognised error behaves exactly as today. One free retry per configured `mobile` entry, returned only by a successful join. | Why: on 09-14 18:55 the phone refused with `WRONG_KEY` → `Secrets were required, but not provided`, yet the same profile joined cleanly at 21:50, 21:53, 22:13 and 09-15 07:39 — the phone was not awake. Fable SHIP-WITH-FIX, fix applied: `last_join` is now consumed when its blame is decided (a later unrelated blame inherited it and was swallowed) and the change-only log is re-armed by a successful join. Two tests added that fail without each half. 391 tests. |

## Storage

| Commit(s) | What changed | Notes |
|---|---|---|
| `fa540a8c43` **swaglogcap2pnw** | **Owner decision** ("yes"): the device log cap goes 2,500 → **20,000 files AND a 200 MiB ceiling**, whichever binds first, deleting oldest-first. About 14 days of logs instead of 42 h, ~170 MB typical. The young-file warning now names which limit forced the delete. | Fable SHIP. The running byte total avoids a directory stat per rollover; init scan 48–52 ms for 20,000 files; strict bound is 200 MiB plus the active file. Installed 08:03 PT (BootCount 233): already past the old cap at 2,505 files, `/data` free 9.3 GB, logmessaged clean. **Found:** athenad forwards these logs to comma's server with no metered gate (pre-existing) — see below. |
| `f02d6864e3` **athenalogmeter2pnw** (installed 19:24 PT, BootCount 240) | The swaglog forward to comma's athena server now pauses while the link is metered, and resumes when it is not, matching the owner's "zero metered file traffic" rule. Source of truth is the `NetworkMetered` param (no new msgq sub). An unset param pauses and logs at ERROR. | Fable SHIP: the pause cannot interrupt an in-flight file, nothing is lost or double-sent, both edges are change-only. Steady state is ~12 MB/day; the exposure the cap raise added is a one-time ≤200 MiB backlog flush. Checked on the device: the 400 oldest logs carry the acked xattr, so there is no re-send loop. |

## Speed control

| Commit(s) | What changed | Notes |
|---|---|---|
| `57d79657bc` **behindrun2pnw** (installed, BootCount 238) | **Owner decision** ("yes"): a map curve the truck has already passed may no longer lower a RUNNING ICBM slowdown, nor hold back the restore. Start behaviour, vision candidates and curve timing are untouched. | Replay of the real weekend + 09-08 data: a passed point lowered a running target on 9 ticks and held it on 145; after, 173 ticks (43.3 s) of cap removed. Sun 09-13 15:46: restore 15.65 s → 7.15 s, set ends 60 mph not 50. Fable SHIP: on all 21 acted ticks the removed point was 30–71 m behind the TRUE position, an at-node point is never masked, and a multi-node curve keeps binding on the nodes ahead. 746 ces_pnw tests, 14/14 mutants, Tesla hash unchanged. |
| `16e8ebb503` + `974767198b` **cesmodehold2pnw** (installed together, BootCount 239) | **Owner decision** ("yes"): when the CES master mode cannot be read, each caller (CES, VTSC, the UI overlay) keeps its last good mode for 10 s, then falls back to Off with the overlay's NO-SIGNAL alarm. The second commit stops the legacy bool standing in for a failed read. | Fable APPROVE with two conditions, both met: B ships with A (A alone could run Standard while the overlay showed NO SIGNAL), and a **legacy-only** read failure must not arm the hold — fixed here, so a driver who just picked Off is not overridden for 10 s and the alarm cannot stick on while the master is readable. 1498 tests; 3 extra mutants on the fix. Installed 19:04 PT (BootCount 239): 1075 tests green on the rebase, ui/selfdrived/plannerd PIDs unchanged over 25 s, CES publishing ticks normally (`mode=experimental reason=stopLatch` while parked), `CESMode` reads 2, no hold and no NO-SIGNAL — i.e. the master is readable and the new path is inert, which is the intended steady state. |

## Speed control — the gas-set wait

| Commit(s) | What changed | Notes |
|---|---|---|
| `efaf459124` **gassetwait2pnw** (installed 21:10 PT, BootCount 242) | **Owner rule, 09-15 evening:** *"when I accelerate while cruise control is on and then I lift off the gas, that doesn't mean that I want to brake, it means that I want to keep the speed — but only for those few seconds, because normally when I lift off the gas it means that I actually want to brake."* The wait after lift-off is now conditional on `aEgo` **latched on the lift-off frame**: above +0.5 m/s² (on the power) → **0.5 s**; otherwise, including an unreadable reading → **1.0 s**, unchanged. Owner approved shipping it tonight. | Why 0.5 s is enough: in the one clean trace the truck *coasts* for the first half second (+0.50 s, 30.69 mph against a 30.62 mph lift-off, aEgo +0.07); regen only bites between +0.5 and +1.3 s, and at the old 1.0 s the PCM latched 30. Why the braking case is untouched: the episode the 1.0 s was earned by (Sat 12:41:50, brake at +0.69 s) had the driver already slowing for 5 s, so it keeps the full second. **A second fix was needed:** at 0.5 s the `decelUnknown` gate became binding — the decel estimator free-runs on a 0.4 s cadence, so the fire landed at 0.76 s and would jitter across [0.5, 0.8). The window is now anchored at lift-off, for the gas-set path only. Fable SHIP-WITH-FIX, both fixes applied: the threshold was justified on the report's **mean** `a0` while the code latches the **lift-off frame** value (on that metric +0.5 is *above* the firing floor, i.e. conservative), and the wait fields are now stamped on gas-set records only, so a resume can never carry a `waitS` that did not govern it. 1679 tests, **15/15 mutants**, ruff clean. |

**Deliberately NOT built:** tapping the set speed back up to the lift-off speed, which the owner had approved.
Reading the executor first changed the arithmetic — one tap is exactly 1 mph and the set speed is quantised to
whole cluster units, so "lock in the exact speed" is not available and the whole remaining prize is ~0.4 mph.
Adding an acceleration command to a path whose entire safety argument is "a SET commands no speed change at all"
is not worth that on spec. Revisit with the next drive's telemetry (`a0` is now in every record).

**Residual risk, stated:** a driver who was on the power, lifts off, and brakes between 0.5 s and 1.0 s now gets
a SET that has already landed. Fable drove the real executor through that case at 0.55/0.6/0.7/0.9 s across 25
poll phases — 100 combinations: the button is **never** asserted with a pedal down, no acceleration is commanded,
and every combination latches `postResumeBrake`; at some phases a 20–40 ms partial press gets out before the
mid-press abort. No such episode exists in the corpus (of 5 on-power lift-offs, 0 braked within 3 s), so this is
unmeasured rather than measured-safe.

## In progress

- **The felt 5 mph sag is still open.** It is the PCM's own regen→ACC torque hand-off: after cruise engages the
  truck keeps braking ~1.5 s, bottoms 4.85 mph below lift-off and recovers by +7.2 s. openpilot does not own the
  pedals in stock-ACC mode, so nothing here removes it. Next: measure whether engaging earlier shortens it, using
  the `a0`/`waitS` telemetry from the first drives on the new wait. Report: `drives/2026-09-15/gasset-regen-loss/`.

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
