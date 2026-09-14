# CHANGELOG — 2026-09-14 (Monday)

Continues [`CHANGELOG-2026-09-13.md`](CHANGELOG-2026-09-13.md). Repos: **pnw-pilot** (channel `3devpnw`),
**pnw-opendbc**, **pnw-panda**. Overnight autonomous work on the owner's directive (2026-09-14 ~00:45 PT): "keep
working overnight on all open items that do not require my input, push these items to the truck". Each change is
reviewed by **Fable** (the only reviewer, `docs/CODING-POLICY.md`) before push, then installed on the F-150
Lightning's comma 3X on its own reboot while openpilot is disengaged, and health-checked. This file is updated
as each change ships.

**Channel tip:** `origin/3devpnw` = `da0741eb8a` (twistyr2pnw + policer2pnw, pushed 08:31 PT, staging).
**Installed on the truck:** `3a361964c7` (leadlossgate2pnw + docs, 08:27 PT).

## Networking — arbiter logging

| Commit(s) | What changed | Notes |
|---|---|---|
| `6ee95bac1f` **smallfix0914pnw** | The network arbiter logs every change of the active WiFi connection (`network_arbiter_active_changed from/to/by`), including changes it didn't make (`by=external`, e.g. NetworkManager autoconnecting KarlMoik after the iPhone hotspot drops) and bring-ups that didn't land (`by=not_as_requested`). A failed read is never logged as a change. Bring-up log lines now say what they leave (hotspot / a client connection / nothing / unreadable) instead of always "dropping hotspot". Logging only; no change to what the arbiter decides. | Fable APPROVE (change-only, no new nmcli call, 305/305 tests). Installed 01:14 PT; verified live: `KarlMoik → iPhone, by=arbiter` at boot. Motivated by the unlogged 2026-09-13 22:01 PT switch. |
| `ff811dea0e`, `a0dd8097ee` **unreadhold2pnw** | A failed read of the active WiFi connection no longer makes the arbiter raise the hotspot (or re-`con up`) over a working link. Before, one nmcli hiccup plus the connected AP missing from that scan bounced a working link for 20–40 s. The hold applies only when there is something to hold (a link seen at the last good read, or our own bring-up in flight), so the first connection at boot is never delayed. It is bounded: after 120 s of consecutive failed reads the arbiter acts as before and logs an ERROR once. Hold and release are logged change-only. | Fable: first pass SHIP-with-gate (a boot delay of up to 120 s without it); the gate was applied exactly as Fable prototyped, with a boot test and 2 mutants killed; networkd 297/297. Installed 07:21 PT; healthy (no hold events at boot, on KarlMoik). |

## Diagnostics — ACC dropout logging

| Commit(s) | What changed | Notes |
|---|---|---|
| `a568f3de9a` **smallfix0914pnw** | `accdrop_pnw.STALE_ROUTE_S` 70 → 75 s, clear of loggerd's 72 s fallback segment rotation, so a stalled-encoder segment no longer flags `routeStale` falsely. Logging only. | Fable APPROVE. Same install as above. |

## Cars — Ford gear decode

| Commit(s) | What changed | Notes |
|---|---|---|
| pnw-opendbc `7163a85522` (master-pnw) + pin `9f6ca9f0ea` **gearunknown2pnw** | Ford carstate reports `gearShifter=unknown` until `PowertrainData_10` has actually been received, instead of decoding Park from the parser's zero-initialised values. That fixes loggerd's parked-video gate trusting a gear decoded from a silent bus, which could lose a drive's video. Once seen, the gear decodes exactly as before, and a mid-drive outage holds the last gear. One log line when the gear is first received (and one if it isn't within N s). | Fable SHIP both. All 11 Ford platforms read PowertrainData_10 on bus 0 (no friends'-channel risk). No new engagement delay; the Tesla is untouched. Ford suite 196 pass. Installed 07:28 PT; verified: "PowertrainData_10 first received 0.0 s", gear park, carState valid, loggerd stopped in Park. |
| `f93e876310` **gearparkcan2pnw** | On cars whose gear reads `unknown` until it's actually received (capability `gear_unknown_until_seen`: the Lightning via its powertrain parser, the Tesla via chassis), GearPark may be set from a Park decode while global canValid is False, as long as the gear's own parser is valid that tick. This closes parknorec2pnw's gap where a truck waking to charge with the camera bus asleep couldn't confirm Park and kept recording. The check is read-only and stricter than `can_valid`, so a dead powertrain bus can never set Park. Other platforms are unchanged. | Fable SHIP (0/2000 valid-when-invalid ticks; Tesla DI_gear on the chassis parser maps never-received to unknown; no SET/CLEAR race; other platforms truth-identical). Installed 07:50 PT; verified LIVE at boot: `gear_park value=true can_valid=false gear_source_valid=true`, loggerd stopped. |

## Diagnostics — lead-loss shadow

| Commit(s) | What changed | Notes |
|---|---|---|
| `83ac3eb172` **leadlossr2pnw** | The lead-loss-hold shadow detector's failures are logged (first immediately, then at most once a minute) instead of swallowed by `except Exception: pass`. Rule 2. No plan or actuator change; the detector stays log-only. | Fable SHIP. Analysis of 69 shadow events (`drives/2026-09-14/leadloss-shadow-review/`): 3 genuine close drop-outs, none while openpilot controlled speed. Recommendation: don't build the braking version; keep logging with 3 extra gates. |
| `df5733e6e5` **leadlossgate2pnw** | The lead-loss shadow only considers a drop-out at ≥ 5 m/s, with TTC ≤ 8 s and carState valid, which are the review's 3 gates. Rejections are logged with the gates that fired, at most 1 line per 5 s plus a held-back count. Malformed lead fields are logged. Still log-only: it never brakes. | Fable APPROVE: the narrowed except adds no crash path, since the planner's own logged guard wraps it. The 69-event replay reproduces the report: 6/6 useful kept, 6/7 harmful dropped. 15 tests, 18/18 mutants. Installed 08:27 PT (BootCount 200), healthy: control processes up, no tracebacks. |

## Diagnostics — device logs

| Commit(s) | What changed | Notes |
|---|---|---|
| `db2cec0d7c` (upstream #38322), `81aba515a5`, `acb5a21fb7` **swaglogrot2pnw** | swaglog rotation deletes the OLDEST logs when the 2500-file cap is hit, not the newest. Before, every restart deleted the logs it had just written, which is why there were no device logs from 09-05 to 09-12. A log younger than 24 h that rotation deletes now raises a WARNING, and that age check cannot crash logmessaged if another handler removes the file first. | Fable SHIP; the concurrent-delete guard was applied as Fable asked (test + mutant). Installed 08:02 PT, BootCount 199: all 11 logs from before the reboot (24720–24730) survived, and the newest is 24734. No young-delete warnings. The one traceback is soundd's `assert stream.active` at 07:59:08 PT, as the install reboot shut the system down, so it is not a fault. It is visible now only because the pre-reboot logs are kept. Report: `drives/2026-09-14/swaglog-rotation/`. |

## Location services — police misses

| Commit(s) | What changed | Notes |
|---|---|---|
| `9997c8e6ef` **policemiss2pnw** | A gated or failed police poll keeps showing the reports already fetched (amber, never able to slow the car) instead of wiping them. Measured last week: 57 min on freeways below 43 mph, 21 of them with reports within 15 mi. | Fable: BLOCK alone (the failure became invisible), fixed by `47be7e2175`. |
| `20a98e2ffa` **policemiss2pnw** | The police poll resumes promptly: no backoff for failures of our own link, and the speed gate is re-checked every 5 s instead of 60 s (first poll after reaching 45 mph was a median 38 s late, 52 times). Proxy 402/429 still park. | Fable APPROVE: worst case 1334 polls/day with min gap 62.4 s, never above steady highway polling; budget +≤$0.26/week. |
| `47be7e2175` **policeship2pnw** | A held report carries the failure reason and the overlay shows it (e.g. `Police 11.2 mi (2 min) - daily limit`). Held reports are re-checked against the 45 min TTL every tick. | Fable re-review SHIP: replay shows the reason in amber, no held path emits `cap`, empty list + error keeps the red path, TTL boundary exact, 169 tests. Report: `drives/2026-09-13/police-miss-week/DRIVE_REPORT.md`. |

**Held for the owner:** `9ad5f391af` (show police on non-freeway roads at highway speed, display only; contradicts the design doc's "never off-freeway"; the siren would chirp off-freeway if `SIREN_ENABLED` were ever turned on). Also: lower the 45 mph gate (+$0.15–1.18/week); raise the proxy's 20-alert cap (hit on 12% of Seattle/Portland polls).

## In progress (not shipped yet)

- **`behindgate2pnw`** (built, `d5c7367f24`; Fable reviewing): ICBM will not START a slowdown for a map curve the
  truck has already passed. A point counts as passed when it is more than 5 m behind along mapd's path AND behind
  the heading. When that can't be determined, nothing is gated.
  - Tried on the logged truck fixes: 0 of 214,877 points still ahead were wrongly marked passed. Heading alone got
    3,879 wrong.
  - Of 25 starts that came from an already-passed point, all 25 were flagged: 21 suppressed, 2 changed, 2 not
    reproducible. None of the 42 real starts were flagged.
  - It fixes Sun 12:05:28 (60 → 51 mph), Sun 13:55:51, Sat 12:47:21, 09-08 19:36:58, and Sun 12:43:54 / 13:18:37.
  - It does NOT fix 09-08 20:28:51: that curve was 332 m ahead, so the item stays open.
  - Owner question: should a passed point also stop lowering a slowdown that is already running?
- **`twistyr2pnw` + `policer2pnw`** (`da0741eb8a`, pushed 08:31 PT, installing): the twisty-descent cap and the police input
  read log their failures (Rule 2) instead of `except Exception: pass`.
  - Fable REJECTED the first version, which narrowed the excepts. plannerd is not restarted after a crash, so any
    other error (e.g. `UnknownKeyName` from a params build mismatch) would have disengaged both cars with no
    re-engage.
  - Fixed: both catch `Exception` again, keep the fallback, and name the exception type in the rate-limited log.
    Mutants that narrow either except back are killed; 319 tests pass.
  - Follow-ups: the twisty floor on failure while descending; briefly hold the last good police report; the
    still-silent except in `_fold_map_curve`.
- **Pro Power (done, analysis only):** APIM `7D0-10-03` reads PPOOOVOS already **on** (partially confirmed; one
  FORScan read-only look settles it). Nothing cleared Pro Power in a 66-min overnight watch. Report:
  `drives/2026-09-14/propower-overnight-watch/DRIVE_REPORT.md`.

- **`behindgate2pnw`**: Fable SHIP, rebased as `29379e9b5b`. It installs after twistyr2pnw.
- **`coopsteerfix2pnw`** (`ff3980a17d`, Fable SHIP; Tesla coop-steer, shadow-only): torque above 1.0 Nm counts as
  an override on the same tick instead of waiting for the 50 ms-debounced `steeringPressed`. Replay: active ticks
  above 1 Nm 77 → 0, peak offset 11.5° → 10.1°.
- **`foldlog2pnw`** (`a3be3e9ab6`, Fable reviewing): the map-curve fold's silent except now logs with the
  exception type, and a new `mapErr` field reaches ces_events.
- **Building:** `lcabort2pnw` (abort a lane change when the driver steers against it), `arbiterfu2pnw` (two
  arbiter follow-ups), and a PSCM `LimitReached` investigation.

## Deferred to the owner

Tailgate chime FORScan session (tooling ready); Pro Power: FORScan read-only look at APIM `7D0-10-03`; police off-freeway display / off-freeway slowdown / lower the 45 mph gate / raise the proxy's 20-alert cap; behindgate: also gate a running slowdown?; RES restores the truck's memory vs the driver's set; stock
dropout keeps steering (panda change); brake-release auto RES; the deleter policy when storage is full of
un-uploaded drives; map downloads over metered links; the Fix B `coast_bias` default; `mapFlr` keep/drop;
`curveoverride2pnw`; 12 V multimeter; relayMalfunction harness check; a driver-monitoring video check.
