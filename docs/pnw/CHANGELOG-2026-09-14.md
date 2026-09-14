# CHANGELOG — 2026-09-14 (Monday)

Continues [`CHANGELOG-2026-09-13.md`](CHANGELOG-2026-09-13.md). Repos: **pnw-pilot** (channel `3devpnw`),
**pnw-opendbc**, **pnw-panda**. Overnight autonomous work on the owner's directive (2026-09-14 ~00:45 PT): "keep
working overnight on all open items that do not require my input, push these items to the truck". Each change is
reviewed by **Fable** (the only reviewer, `docs/CODING-POLICY.md`) before push, then installed on the F-150
Lightning's comma 3X on its own reboot while openpilot is disengaged, and health-checked. This file is updated
as each change ships.

**Channel tip / installed on the truck:** `d379b4a0f5` (= pin bump `03ee7fc3db` + docs; opendbc `97be35a7`;
installed 09:10 PT, BootCount 208). 10 changes installed today, each on its own reboot.

## Networking — arbiter logging

| Commit(s) | What changed | Notes |
|---|---|---|
| `6ee95bac1f` **smallfix0914pnw** | The network arbiter logs every change of the active WiFi connection (`network_arbiter_active_changed from/to/by`), including changes it didn't make (`by=external`, e.g. NetworkManager autoconnecting KarlMoik after the iPhone hotspot drops) and bring-ups that didn't land (`by=not_as_requested`). A failed read is never logged as a change. Bring-up log lines now say what they leave (hotspot / a client connection / nothing / unreadable) instead of always "dropping hotspot". Logging only; no change to what the arbiter decides. | Fable APPROVE (change-only, no new nmcli call, 305/305 tests). Installed 01:14 PT; verified live: `KarlMoik → iPhone, by=arbiter` at boot. Motivated by the unlogged 2026-09-13 22:01 PT switch. |
| `ff811dea0e`, `a0dd8097ee` **unreadhold2pnw** | A failed read of the active WiFi connection no longer makes the arbiter raise the hotspot (or re-`con up`) over a working link. Before, one nmcli hiccup plus the connected AP missing from that scan bounced a working link for 20–40 s. The hold applies only when there is something to hold (a link seen at the last good read, or our own bring-up in flight), so the first connection at boot is never delayed. It is bounded: after 120 s of consecutive failed reads the arbiter acts as before and logs an ERROR once. Hold and release are logged change-only. | Fable: first pass SHIP-with-gate (a boot delay of up to 120 s without it); the gate was applied exactly as Fable prototyped, with a boot test and 2 mutants killed; networkd 297/297. Installed 07:21 PT; healthy (no hold events at boot, on KarlMoik). |
| `f65fdbdaf9` **arbiterfu2pnw** | (1) An unreadable verification read right after a bring-up no longer blames that network and puts it in backoff. The judgement waits for the next good read, bounded by the unreadable hold; past the bound it is made as before and logged at ERROR (`netcosttier_blamed_unverified`). (2) The fallback line no longer says "no priority network in range" when a priority network was in range but outranked: it lists each candidate and why it wasn't chosen. | Fable SHIP. Its optional hardening is applied: a deferral is gated on the same "something to hold" as the hold. In the double-fault probe the hotspot now comes up at 40 s instead of 160 s; test added, 2 mutants killed; networkd 319. Installed 08:49 PT (BootCount 205): still on KarlMoik, arbiter up, no arbiter tracebacks, nothing blamed or held. |

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
| `df5733e6e5` **leadlossgate2pnw** | The lead-loss shadow only considers a drop-out at ≥ 5 m/s, with TTC ≤ 8 s and carState valid, which are the review's 3 gates. Rejections are logged with the gates that fired, at most 1 line per 5 s plus a held-back count. Malformed lead fields are logged. Still log-only: it never brakes. | Fable APPROVE: the narrowed except adds no crash path, since the planner's own logged guard wraps it. The 69-event replay reproduces the report: 6/6 useful kept, 6/7 harmful dropped. 15 tests, 18/18 mutants. Installed 08:24 PT (BootCount 200), healthy: control processes up, no tracebacks. |

## Diagnostics — device logs

| Commit(s) | What changed | Notes |
|---|---|---|
| `db2cec0d7c` (upstream #38322), `81aba515a5`, `acb5a21fb7` **swaglogrot2pnw** | swaglog rotation deletes the OLDEST logs when the 2500-file cap is hit, not the newest. Before, every restart deleted the logs it had just written, which is why there were no device logs from 09-05 to 09-12. A log younger than 24 h that rotation deletes now raises a WARNING, and that age check cannot crash logmessaged if another handler removes the file first. | Fable SHIP; the concurrent-delete guard was applied as Fable asked (test + mutant). Installed 07:59 PT, BootCount 199: all 11 logs from before the reboot (24720–24730) survived, and the newest is 24734. No young-delete warnings. The one traceback is soundd's `assert stream.active` at 07:59:08 PT, as the install reboot shut the system down, so it is not a fault. It is visible now only because the pre-reboot logs are kept. Report: `drives/2026-09-14/swaglog-rotation/`. |

## Speed control — Rule 2 on silent excepts

| Commit(s) | What changed | Notes |
|---|---|---|
| `da0741eb8a` **twistyr2pnw + policer2pnw** | The twisty-descent cap (`vtsc_controller.py`) and the police input read (`speedadjust_controller.py`) used `except Exception: pass`. They now log, with the exception type: the first failure at once, then at most once a minute with a count. The fallbacks are unchanged: no twisty trim; no police report, so no police cap. | Fable REJECTED the first version, which narrowed the excepts. plannerd is `restart_if_crash=False`, so any other error, e.g. `UnknownKeyName` from a params build mismatch, would have disengaged both cars with no re-engage. Fixed: both catch `Exception` again, and mutants that narrow them back are killed. 319 tests. Installed 08:28 PT (BootCount 201): no plannerd crash, no failure lines. Follow-ups: the twisty floor on failure while descending; briefly hold the last good police report. |
| `8678df74a6` **foldlog2pnw** | The map-curve fold in `vtsc_controller.py` used `except Exception: return <no map curve>`, so when it failed, curve anticipation stopped with no trace. It now logs the same way as the twisty cap, and a new ces_events field `mapErr` holds the exception type on a failed tick (`""` otherwise). | Fable SHIP: nothing new can raise; `mapErr` resets every tick, so it never latches; the change is additive for the UI and scripts. 967 tests, 24/24 mutants; the Tesla is identical when the fold succeeds. Installed 08:39 PT (BootCount 203): `"mapErr": ""` in ces_events, no plannerd/selfdrived tracebacks. Fable's next 3 silent excepts are listed in PENDING-WORK. |

## ICBM — curves already passed

| Commit(s) | What changed | Notes |
|---|---|---|
| `365d287034` **behindgate2pnw** | ICBM (the Lightning's stock-ACC curve slowdowns) can no longer START a slowdown for a map curve the truck has already passed. A point counts as passed when it is more than 5 m behind along mapd's path AND behind the heading. When that can't be determined (under 5 m/s, no heading, off the path), nothing is gated. Logged as `icbmGate "mapPassed"` plus a change-only `ces_icbm_passed` event. | Fable SHIP: no genuinely-ahead curve could be made to read passed (from ICBM's lagged projected position; heading via sin/cos; mapd's path really includes the nodes behind). The gate has its own try, and the Tesla hash is identical. Evidence (`drives/2026-09-12/central-oregon-weekend/behindgate/`): 0 of 214,877 points still ahead read as passed, against 3,879 for heading alone. All 25 passed-point starts were flagged (21 suppressed) and none of the 42 real starts. It fixes Sun 12:05:28 (60 → 51 mph), 13:55:51, 12:43:54, 13:18:37, Sat 12:47:21 and 09-08 19:36:58. It does NOT fix 09-08 20:28:51: that curve was 332 m ahead, and the item stays open. Installed 08:33 PT (BootCount 202), healthy. Owner question: should a passed point also stop lowering a slowdown that is already running? |

## Tesla — coop-steer shadow

| Commit(s) | What changed | Notes |
|---|---|---|
| `ec6dd71a6d` **coopsteerfix2pnw** | The Tesla coop-steer shadow (it logs, never actuates) treats steering torque above 1.0 Nm as an override on the same tick. Before, it waited for the 50 ms-debounced `steeringPressed`, and for those 5 frames it computed the FULL 12° nudge exactly as the driver took over. The carstate debounce is untouched. | Fable SHIP: replay of route `00000105--0a36ee017d` reproduces exactly (active ticks above 1 Nm 77 → 0, peak offset 11.5° → 10.1°); 5/5 mutants; still shadow-only (the actuator command is final before the shadow runs). Keep strict `>`; no separate reason code needed. Installed 08:44 PT (BootCount 204), healthy. Still needed before this could ever actuate: a light-hand-steering drive. |

## Lateral — lane-change override shadow

| Commit(s) | What changed | Notes |
|---|---|---|
| `d1d2a87df6` **lcabort2pnw (1/2, shadow)** | Logs `lane_change_abort_shadow` (ERROR level, so it lands in qlogs) when the driver holds steering torque against an openpilot lane change for 0.3 s. It changes nothing the car does. | Fable SHIP. DesireHelper runs in modeld: the new attributes are read nowhere else, and NaN/−inf/never-received torque can't raise. It fires once per fight. The sign is right on both cars. Replays pin the on-car `laneChangeState` trace on all 171 ticks. Installed 08:56 PT (BootCount 206): modeld up, 0 tracebacks. **Finding:** ending the state would not stop the steering, because modeld feeds the desire as a rising-edge pulse and the model keeps changing lanes for about 4 s. The acting commit `653b9bc79a` is held for the owner: it would have ended 5 of 19 logged changes, 4 of them Lightning resting hands at ~1.8 Nm. |

## Cars — Lightning PSCM limit telemetry

| Commit(s) | What changed | Notes |
|---|---|---|
| `c1f3ffd1ab` **pscmlimlog2pnw** | card logs the Lightning PSCM's own lateral-limit report (`LatCtlLim`) to ces_events as `{"ev":"pscmLim"}`: one record per change plus one at the first frame of a drive, an error record if no frame arrives within 10 s, capped at 20/min. It reads the Ford carstate's existing CAN parser and sets nothing on CarState, so the dead PSCM clamp in `lateral_angle_pnw.py` stays inert. Telemetry only. | Motivation: `drives/2026-09-12/central-oregon-weekend/PSCM_LIMITREACHED.md`. On 09-08 19:44 PT the PSCM hit its own static limit (−24° held for 2.1 s at 58→54 mph) with no software limit binding, and nothing recorded it. Feeding the signal into the clamp would have frozen the command below what the truck was still delivering. Fable SHIP: nothing escapes `step()` (card is `restart_if_crash=True`); 2000/2000 records with a concurrent rotating writer and 0 torn lines; the Tesla builds nothing; 0.23 µs per tick; 24 tests, 49/49 mutants. Installed 09:05 PT (BootCount 207): verified live, first record `to=0`, card 0 tracebacks. |
| pnw-opendbc `97be35a7` (master-pnw) + pin `03ee7fc3db` | Comment-only: corrects the two "`LatCtlLim_D_Stat` does not fire" comments beside the dead clamp. | Fable: AST-identical, worth shipping. Installed 09:10 PT (BootCount 208), opendbc `97be35a7` verified on the device, card healthy. |

## Location services — police misses

| Commit(s) | What changed | Notes |
|---|---|---|
| `9997c8e6ef` **policemiss2pnw** | A gated or failed police poll keeps showing the reports already fetched (amber, never able to slow the car) instead of wiping them. Measured last week: 57 min on freeways below 43 mph, 21 of them with reports within 15 mi. | Fable: BLOCK alone (the failure became invisible), fixed by `47be7e2175`. |
| `20a98e2ffa` **policemiss2pnw** | The police poll resumes promptly: no backoff for failures of our own link, and the speed gate is re-checked every 5 s instead of 60 s (first poll after reaching 45 mph was a median 38 s late, 52 times). Proxy 402/429 still park. | Fable APPROVE: worst case 1334 polls/day with min gap 62.4 s, never above steady highway polling; budget +≤$0.26/week. |
| `47be7e2175` **policeship2pnw** | A held report carries the failure reason and the overlay shows it (e.g. `Police 11.2 mi (2 min) - daily limit`). Held reports are re-checked against the 45 min TTL every tick. | Fable re-review SHIP: replay shows the reason in amber, no held path emits `cap`, empty list + error keeps the red path, TTL boundary exact, 169 tests. Report: `drives/2026-09-13/police-miss-week/DRIVE_REPORT.md`. |

**Held for the owner:** `9ad5f391af` (show police on non-freeway roads at highway speed, display only; contradicts the design doc's "never off-freeway"; the siren would chirp off-freeway if `SIREN_ENABLED` were ever turned on). Also: lower the 45 mph gate (+$0.15–1.18/week); raise the proxy's 20-alert cap (hit on 12% of Seattle/Portland polls).

## In progress (not shipped yet)

- **PSCM `LimitReached` (investigated):** the 09-08 19:44 PT event was the PSCM's own static limit on the I-5
  on-ramp, not a software clip. Nothing decodes the signal, so the clamp in `lateral_angle_pnw.py` is dead code.
  Wiring the signal into that clamp would have frozen the command below what the truck was still delivering.
  Building telemetry only (`pscmlimlog2pnw`). Report: `drives/2026-09-12/central-oregon-weekend/PSCM_LIMITREACHED.md`.
- **`silentexc2pnw`**, 6 commits, Fable SHIP on each. They install one per reboot (the test-only 5/6 rides with 4/6):
  1. `fe4bf3116e` VTSC `_read_enabled`: a read error turned VTSC off silently. **Pushed 10:05 PT, not installed:**
     the truck stopped answering SSH and cloud check-ins at 09:53 PT, reason unknown.
  2. `0efde79665` speedadjust AutoSpeedReduce read and SpeedAdjustTarget publishes.
  3. `16d4afb75a` VTSC map-input/GPS reads and the VTSCStatus publish. A missing GPS fix is not logged.
  4. `8e276bc561` VTSCStatus `mapD` is null instead of `Infinity`.
  5. `961e16d7a5` test-only.
  6. `da617deb16` the ICBM lead-pacing failure log names the exception type and counts failures.
  - Identity when nothing fails: VTSC/speedadjust 168/168 and CES 36/36 scenarios identical. 1314 tests, 123/123
    mutants.
- **Building:** `silentexc3pnw`, Fable's next-ranked silent excepts: `read_ces_mode` → Off (CES and VTSC on both
  cars), speedadjust `_read_speed_limit`, the VTSC freeway-floor inputs, and the RainMode push.
- **ICBM 46.5–58 m/s "garbage band" (analysed, NOT built, no fix needed):**
  `drives/2026-09-14/icbm-garbage-band/DRIVE_REPORT.md`, claims independently verified.
  - In today's code a band read can never change an ICBM/CES decision: ICBM inflates it to ≥ 57.75 m/s, which is
    always above the set speed. Over 11,954 band ticks, removing the value changed nothing.
  - The "+6.1 s late" curve (07-19 06:29:44, Shilshole bend) had no band read: the 6.1 s was measured from the
    10 s look-ahead marker, and against the steering ICBM started ~1 s early.
  - A polyline rule can't separate real from bad reads: at best it catches 48/92 bad and rejects 290/811 real.
  - The no-op holds while the Lightning `map_scale` ≥ ~0.63 (default 0.92).
  - Earlier map anticipation is untouched: it is owner-controlled.

## Deferred to the owner

Tailgate chime FORScan session (tooling ready); Pro Power: FORScan read-only look at APIM `7D0-10-03`; police off-freeway display / off-freeway slowdown / lower the 45 mph gate / raise the proxy's 20-alert cap; behindgate: also gate a running slowdown?; lane-change abort: per-car torque threshold (the Lightning's resting hands read ~1.8 Nm), same-direction abort, clearing the model's desire history; PSCM LimitReached: once logged, should a hands-off LimitReached raise Take Control at once, and tell Alan Polk the signal fires in angle mode?; RES restores the truck's memory vs the driver's set; stock
dropout keeps steering (panda change); brake-release auto RES; the deleter policy when storage is full of
un-uploaded drives; map downloads over metered links; the Fix B `coast_bias` default; `mapFlr` keep/drop;
`curveoverride2pnw`; 12 V multimeter; relayMalfunction harness check; a driver-monitoring video check.
