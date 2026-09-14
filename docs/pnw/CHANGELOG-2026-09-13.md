# CHANGELOG — 2026-09-13 (Sunday)

Continues the September changelog series. Repos: **pnw-pilot** (channel `3devpnw`), **pnw-opendbc**
(`master-pnw`, pin unchanged this session), **pnw-panda** (unchanged this session). Everything below
landed on `3devpnw` today, one change per reboot, each reviewed **Fable** (the only reviewer —
`docs/CODING-POLICY.md`) before push, then installed on the F-150 Lightning's comma 3X.

Drive reports referenced live under `../../../../drives/<date>/` (i.e. `~/gh/comma/drives/<date>/` from
the workbench root).

**Channel tip at time of writing:** `origin/3devpnw` = `3871d12733` (gpsdrgate2pnw — stale-GPS
curve-cap hold). Late additions past midnight (engagegoal2pnw's 10-commit stack, gpsdr2pnw,
gpslag2pnw, gpsdrgate2pnw) verified healthy on the truck **2026-09-14 00:30 PT**; filed under this
date because the work session started 2026-09-13.

## TL;DR

1. **Three driver directives shipped as ICBM/speedadjust changes**, each found live on the road this
   weekend: a curve restore can no longer walk the set above the posted limit; a speed-limit slowdown
   never auto-restores (police is the one exception); a lead car being followed through a curve is now
   trusted over ICBM's own, more conservative target.
2. **MADS goes quiet at traffic lights.** Engagement chimes are suppressed while MADS keeps steering
   through a brake-triggered disengage; a full disengage still chimes.
3. **The network arbiter learned to rank by cost, not by a binary priority/hotspot choice**, and a
   hand-picked WiFi network now sticks instead of being reverted on the next tick.
4. **Code updates fetch over any link, metered included** (driver directive, reaffirmed), with
   exponential backoff on repeated failures and a fix so one offline check can't wipe a staged, ready
   update.
5. **The device stops recording while the shifter is in Park**, on both cars — the "records ~180 MB/h
   parked and charging" item from `docs/PENDING-WORK.md` is closed.
6. **mapd's stock pin moved to v2.3.1**; the truck's own override build is unaffected, and the
   "your pin bump was ignored" warning now actually reaches the device's logs.
7. **New diagnostic-only logging**: every stock-ACC state change on the Lightning now carries a CAN
   trace, so the next unexplained cruise dropout has the evidence already captured.
8. **Non-code:** the lat-accel cap file on the truck was hand-edited per the driver's request.

---

## Speed control — ICBM / speed-limit zones / curves with a lead

| Commit(s) | Feature | What changed for the driver | Param / kill switch |
|---|---|---|---|
| `a0429c3fbc` | **icbmrestorecap2pnw** | After a curve, ICBM's restore of the stock-ACC set speed can no longer rise above **posted limit + 5 mph**. It holds at that cap instead of ending, and follows the limit back up if it rises during the hold. Found live: in a 25 mph zone, restore walked the set to 60 mph over 13 s. | none (internal ICBM logic) |
| `e5c155ffdd` | **sanorestore2pnw** | A cruise-speed reduction for a **lower posted speed limit never auto-restores** — the driver raises the speed himself. A **police-only** slowdown still restores fully (the one exception). A slowdown with both a limit drop and police present is treated as a limit drop (no restore). | rides `AutoSpeedReduce` mode 2 (stock ACC) |
| `5eb5fd486d`, `64b707ec63`, `16dbe723a0` | **sazoneset2pnw** / **zonefollow2pnw** | Entering a lower speed-limit zone sets the cruise speed **once**, at the same percentage above the new limit the driver was running before, then stops managing it — no memory of the old set, no restore on a later limit rise. A curve **inside** that zone restores to the zone-set speed, not the pre-zone speed. Three measured defects fixed before/after shipping: a curve overlapping the zone's entry point could leave the truck up to 75 mph in a 45 zone (fixed by reading ICBM's own SET− taps and not double-counting them as driver overrides); a limit drop happening mid-curve-restore ended below the zone speed; a `plannerd` restart mid-curve could lose the zone bound (now falls back to limit+5 and logs it). | rides `AutoSpeedReduce` mode 2 / op-long cap |
| `2bfdce95b0` | **curvelead2pnw** | When a lead vehicle has been tracked continuously through a curve, ICBM now **paces the set to the lead's speed** (capped at what the truck itself can comfortably take, and never above the driver's own set) instead of computing its own, more conservative curve target. Dec-only: it can only raise ICBM's target toward the lead, never push it lower. A second, more aggressive "trust the map less" rule was designed and measured but **shipped as telemetry only** — it raised several ticks above the truck's safe lateral-accel bound on real curves, so it does not act. | `PnwVehicle.icbm_lead_lat_accel` capability (2.5 on the Lightning, 0.0 elsewhere — off), tunable in `/data/pnw/curve.json` |

**Why:** driver reports during `drives/2026-09-12/central-oregon-weekend/` and live on 2026-09-13 —
"I'm in a 25 mph zone and ICBM puts the target speed to 60, which is wrong"; "if I'm going 50-plus on a
country road and heading into a curve, the car suddenly loses 20 mph"; "why don't you just follow
[the lead] instead of making up your own mind." Recommendation #5 of the weekend drive report
(follow a lead through a curve) is what `curvelead2pnw` implements.

**Installed on the truck 2026-09-13.**

---

## Engagement — MADS quiet chimes

| Commit | What changed for the driver | Param |
|---|---|---|
| `15a4786d80` **madsquiet2pnw** | No audible engagement chime while MADS (lateral-only steering) keeps the truck centered through a brake-triggered disengage — e.g. six traffic lights in a row no longer chime twelve times. The decision cannot be made on one frame (the truck's PCM drops cruise before MADS has armed), so a disengage is silenced **provisionally** and the chime plays late (≤0.47 s) if MADS turns out not to take over. A **full** disengage — cancel press, reverse, door, steering-torque disengage, panda revoke, ACC off, or MADS stopping steering on its own — **always still chimes**, including the one case that never used to (MADS stopping on its own while openpilot stays disabled). | none — driver request, no toggle added |

**Why:** driver, verbatim: "when I go through like 6 traffic lights the constant engaging and disabling
audio is very annoying... we only want the silence while MADS keeps steering. When I disengage fully it
can still chime."

**Installed on the truck 2026-09-13.**

---

## Engagement — gas-set

10-commit stack, `271a4b7d3c..b1288f5004`. Owner goal, verbatim: "If I hit the cruise control button
and anything is on, it needs to be off. If I want to resume longitudinal control and accelerate I can
either push the + or I should be able to hit the gas pedal once and then it should overwrite this and
set the new speed." Full verification of every path (raw CAN, qlogs, `ces_events`): section 10 of
`docs/MADS-RESUME-TO-DRIVER-SPEED.md`.

| Commit(s) | What changed for the driver | Notes |
|---|---|---|
| `271a4b7d3c` **engagegoal2pnw** | After a steering-only stop (e.g. a traffic light), one accelerator press now sets/engages cruise at the current speed — closes the gap where a stop held more than 20 s (armExpired) left the gas pedal doing nothing until the next brake. ON/OFF and SET+/RES behavior while steering-only was verified unchanged, not touched. | Fable: **SHIP WITH FIXES** — F1 (a refused press wrote nothing, fixed next commit), F2 (MADS-unavailable clear untested, fixed here). F3 (cap lift records) declined — measured ~1/s worst case, not a flood. F4 (pre-existing, an inert-branch disarm writes no terminal) flagged only. 229 tests, 12/12 mutants killed. |
| `11388568e1` | A gas press refused by the post-resume double-tap opt-out is now also on record (previously the opt-out held correctly but left no trace). | Fable re-review: **SHIP**, completes F1. 231 tests, 16/16 mutants. |
| `d46ba6130d` **first press only** | Only the FIRST accelerator press after a brake can set the speed; a later press in the same stop (e.g. an overtake minutes later) is ignored and logged as refused. A new brake re-arms it. Owner decision, replacing any time limit. | A press refused by a gate still counts as used (owner's explicit choice, flagged Rule 1 — fails toward cruise staying off). 234 tests, 25/25 mutants. |
| `6cd0ce7c51` **ignore regen** | A gas-set is no longer refused just because the truck is still slowing from regen after lift-off (1.5-1.9 m/s² is normal, and a SET to the current speed commands no acceleration). A real simultaneous brake still wins outright. Owner decision (Q-C4: "Ignore regen, set"). | 237 tests, 31/31 mutants. |
| `d5ca781152` | The gas-set now waits **1.0 s** after lift-off before firing, so a driver who lifts to slow down and brakes a moment later gets the brake, not an unwanted SET. Owner decision (Q-C5: "Wait 1.0 s"). | 266 tests, 35/35 mutants. |
| `03c9c30ba9` **cancel on overshoot** | If our own SET/RES press brings the truck's cruise set back more than **3 mph** above what was asked for, cruise is now cancelled outright — steering drops too — instead of being left running high. New alert: "Cruise set too high - cancelled." Found live: a gas-set SET− made the PCM engage at 55 mph from 34 mph actual with a 42 mph memory (`drives/2026-09-13/corvallis-resume-55/`). Owner decision, option (a): "Cancel, steering drops too." | Adds `EventName.madsResumeSetTooHigh`, **log.capnp `@104`** — the established pnw event pattern (`greenLight @99` … `cruiseOffRequested @103`). A driver's own cruise button in the window is never cancelled. 282 tests, 11/11 mutants. |
| `3b07240cf8` **Fable B1 (blocking)** | Fixed: a driver's own RES/SET pressed just before our gas-set fired was being read as "our overshoot" and wrongly cancelled. Now: no gas-set/RES press of ours at all within **1 s** of a driver cruise-button press. | Fable-requested, blocking. 287 tests, 4/4 mutants. |
| `d444e38b9f` **creep exemption** | Creeping in stop-and-go traffic (never reaching ~11 mph / 5 m/s) no longer uses up the first press — only a press that actually got the truck rolling counts, so the real pull-away afterward can still set. Owner decision (Fable's S4 question: "Creeping doesn't count"). | 289 tests, 5/5 mutants. |
| `b92c798e60` | Verify records now carry the raw cluster set values (`gotDisplayMph`/`wantDisplayMph`) so a future km/h-cluster mismatch would be self-diagnosing instead of silently cancelling every press. Telemetry only, no behavior change. | Fable, non-blocking: the cancel rule assumes an mph cluster and no opendbc signal exists to detect km/h — follow-up tracked in `docs/PENDING-WORK.md`. |
| `b1288f5004` | An offer withdrawn by a rejection brake now writes its own `offerEnd` record (previously the `suppress` record was the only trace). Telemetry only. | **Fable re-review of the full 10-commit stack: SHIP.** 300/300 tests pass, merge-tree clean. |

**Not changed / still open** (owner questions in `docs/MADS-RESUME-TO-DRIVER-SPEED.md` section 10.4): an
automatic RES on brake release ("D3"); repeat ON/OFF within 3 s; ON/OFF from Standby while openpilot is
off; keeping steering through a no-input stock dropout (needs a panda + opendbc change). The "S6
residual" (a PCM raising the set after the engage tick, uncaught) is also open — see `docs/PENDING-WORK.md`.

**Installed on the truck 2026-09-14, verified healthy 00:30 PT** (tip `3871d12733`, alongside the GPS
work below).

---

## Networking — WiFi cost ladder, scan/pin, GPS carry

| Commit(s) | Feature | What changed for the driver | Param / kill switch |
|---|---|---|---|
| `67eacc66c3`, `43036409a4`, `989427d339`, `8546e17288` | **netscanpin2pnw / netrank2pnw** | The arbiter now ranks **every saved WiFi network in range by cost** — explicitly unmetered beats a network with no metered/unmetered setting ("default"), which beats explicitly metered — instead of a binary "configured priority network or the comma's own hotspot" choice. It periodically scans for something cheaper while sitting on a paid link (throttled, client-WiFi only). A **hand-picked network in the Settings app now sticks** instead of being silently reverted by the arbiter on the next tick — it only yields to a stationary, explicitly-unmetered *home* network that genuinely arrives after the pick (not one that was already visible when the pick was made). Two Fable "DO NOT SHIP" findings were fixed before this landed: a failed cost read of the *active* link could tear a working connection down at boot, and a 60-second scan gap could end an at-home pin while GPS showed the truck never left. | `DisableNetworkCostLadder` (default 0 = ladder ON; 1 = pre-ladder binary behaviour, no cost, no pin) |
| `fd91a95dbc` | **gpscarry2pnw** | The network logic keeps the device's last fresh GPS fix in memory while the ignition is off (GPS itself only runs with the ignition on), so parked-at-home network decisions — "is this a home network," geo-gated scanning — don't go blind the moment the truck stops. Network selection only; nothing about location *learning* changes. | none |

**Why:** driver, verbatim: "the tethering network should be the lowest priority if another WiFi
connection is available, because the tethering network is the most expensive"; measured 2026-09-13, the
truck sat 66 minutes on metered Starlink with the unmetered iPhone hotspot never even scanned for. And:
"if you can identify that a hand-picked WiFi was selected, we want that to stick."

Full diagnosis and the throttled-scan design: `docs/NETCOST-STARLINK-TO-HOTSPOT.md`.

**Installed on the truck 2026-09-13.** Foundational work from the two preceding days, referenced by
these commits: `718079e75a` **uploadgate3pnw** (2026-09-10 — the uploader's two passes now agree on a
single notion of "is this connection metered," closing a hole where 2,642 MB had gone out over a
metered priority network while the small-file pass was blocked) and `eedddfa617` **netcosttier2pnw**
(2026-09-11 — the original tier 0-3 cost ladder that today's `netrank2pnw` generalized).

---

## Updates — metered fetch, backoff, keep-staged

| Commit(s) | Feature | What changed for the driver | Param / kill switch |
|---|---|---|---|
| `2a26c4b047` | **updatemetered2pnw** | Code (and driving-model, and AGNOS) updates now fetch over **any** link, metered included — previously a metered-only connection (e.g. the truck's mobile Starlink) could leave the device on stale code for up to 3 days. The metered gate still applies to video/drive-data uploads (unchanged). Every fetch now logs whether the link was metered and what kind. | none — `DisableUpdates` (Pause Updates) still works |
| `0e3956dc9f`, `95dd1d968b` | **updatebackoff2pnw** | A fetch that fails now backs off exponentially (5, 10, 20, 40, 80 min, capped at 1.5 h) instead of retrying every 5 minutes forever — found by Fable while reviewing `updatemetered2pnw`: a failure mid-download can re-download the whole git pack and every changed large file on each retry, which is expensive on a metered link. The "Unable to download updates" alert still raises at the same ~75 minutes of wall-clock failure as stock (after the 5th consecutive failure, not the 16th). | none |
| `4ceac1f777` | **keepstaged2pnw** | A failed connectivity check (e.g. offline, behind a captive portal) no longer throws away a staged, ready-to-install update and its downloaded files — only a genuine fetch/install failure does that. Previously one offline check after an update had finished downloading could delete it and force a full re-download, often over a metered link. | none |

**Why:** driver, standing directive since 2026-07-11, reaffirmed 2026-09-13: "updating 515 lines over
hotspot is absurd" — code updates should never wait for unmetered WiFi. The backoff and keep-staged
fixes are Fable's follow-up hardening on that change, approved by the driver the same day.

**Installed on the truck 2026-09-13.**

---

## Uploads / recording — uploader gate, stop recording in Park

| Commit | Feature | What changed for the driver | Param / kill switch |
|---|---|---|---|
| `e747976d89` | **parknorec2pnw** | The device now **stops writing new route segments** (rlog, qlog, video) once the shifter has read Park for 30 continuous seconds, on **both** cars — and starts again on the very first tick it reads a non-Park gear. Only `loggerd` stops; the camera/model/car/control processes and the video encoders keep running so recording resumes in roughly 1-3 seconds after leaving Park. Closes the item at the top of `docs/PENDING-WORK.md`: a parked, charging Lightning was measured writing ~180 MB/h with the SkipVideoWhenParked/ThinRlogWhenParked thinning already on. **Known gap:** a Lightning that boots into a quiet-CAN charging session (no gear message received yet) cannot confirm Park and keeps recording at the old rate — logged loudly (`gear_park_unconfirmed`), not silent. | `RecordWhileParked` (default 0 = gate ON; 1 = record in Park as before) |

**Why:** driver requirement, 2026-09-05, extended to the Tesla 2026-09-13: "it should never be
recording when the shifter is on Park, never." Full design, failure-mode table and measured latencies:
`docs/pnw/PARKNOREC2PNW.md`.

**Verified on the truck, 21:19 PT:** `loggerd` stopped 30 s into Park — **PASS**.

**Installed on the truck 2026-09-13.**

---

## mapd — pin, logging, docs

| Commit(s) | What changed | Notes |
|---|---|---|
| `848f900403` **mapd231pin2pnw** | Stock mapd pin bumped **v2.3.0 → v2.3.1** (upstream's own fix for the gomsgq shadow-reader `Invalid Msgq message size` panic, plus map-extraction and performance fixes). **Inert on the truck**: the device runs a custom override build (sha `77bad867`) behind `/data/mapd/.override`, which the installer keeps in place regardless of the stock pin. | To actually take the new pin on this device: `rm /data/mapd/.override`. |
| `42ea1c3814`, `6e6af00f08` **mapdlog2pnw / mapdlogmgr2pnw** | The "your pin bump was ignored because of the override" warning now reaches the device's own logs, not just the installer's stdout (which manager never captures at boot — the subprocess exits before the log socket is even up, so the line was silently dropped every boot). Manager itself now logs the warning from a status string the installer computes. | Logging only, no behavior change. |
| `35282a24da`, `60550eee80` **docs** | Fixed three stale "the pin is v2.0.6" mentions across `docs/pnw/MAPD-SYSTEM.md`, `docs/pnw/MAPD2PNW.md` and a code comment in `ces_pnw.py` to say v2.3.1 / the override build. | Documentation only. |

**Installed on the truck 2026-09-13** (the logging fixes; the pin bump itself has no on-device effect
while the override is set).

---

## Diagnostics — ACC dropout logging

| Commit(s) | What changed | Notes |
|---|---|---|
| `9840424966` **accdroplog2pnw**, `67011d2f53` **accdropfix2pnw** | **Logging only — no control, actuation, param, panda or opendbc change.** Every stock-ACC state change on the Lightning (off/standby/active/fault) now writes a detailed record to `/data/pnw/ces_events.jsonl`: a 10 Hz trace of vehicle state for 3 s before and 1 s after the edge, every relevant CAN signal change at up to 100 Hz, every button frame openpilot sent, and which of the device's own route segments to keep. Follow-up fixes the same evening: the CAN-frame buffer now covers a full cancel sequence instead of truncating silently, a spurious "went active" record at every card start was removed, and a stale-route flag was added for when `loggerd` isn't currently recording (e.g. in Park). | Cost measured on a real Lightning rlog: roughly 4-6% of the car interface's own CPU time. |

Follow-up left open: the stale-route threshold `STALE_ROUTE_S=75.0` s was picked to clear loggerd's
own 72 s fallback segment rotation, so a stalled (not stopped) encoder can still flag `routeStale`
falsely — noted in `accdropfix2pnw`'s review, not fixed.

**Why:** `drives/2026-09-12/central-oregon-weekend/DRIVE_REPORT.md` found 4 of 22 unexplained country-road
speed drops where the Lightning's cruise control went from Active to Standby with no brake, no button
press and no camera/radar cancel visible on any signal the existing telemetry captured. This gives the
next occurrence the evidence to explain itself.

**Installed on the truck 2026-09-13.**

---

## GPS — no-fix gate, truck GPS selection

| Commit(s) | What changed | Notes |
|---|---|---|
| `501ecfccd8` **gpsfix2pnw** | Both cars. The position every consumer reads (`LastGPSPosition`: speed limits, police/rest-area alerts, WiFi location) is written only when the GPS actually reports a fix. Before, a no-fix sample could put the truck 2.2-2.4 km off (four times on the weekend). Fix loss and recovery are logged. | Fable APPROVE. Installed on the truck 2026-09-13 22:53 PT. |
| `5f9bb8b3f4` **gpssel2pnw** | Lightning only. While the truck's own CAN GPS is provably live, it becomes the position source (median 1.6 m vs 3.0 m error, 99.98% vs 98.6% availability, stable heading at stops). It falls back to the comma's GPS on a stale, frozen or silent feed. Every switch is logged; the position records `src`. mapd itself stays on the comma's receiver. The Tesla is byte-identical (no CAN GPS). | Fable APPROVE. Installed 22:59 PT; the source switched to `car` after 3 healthy updates. Evidence: `drives/2026-09-12/central-oregon-weekend/GPS_TRUCK_VS_COMMA.md`. |
| `0c0a1e5468` **gpsdr2pnw** | Lightning only. When the truck's own GPS reads degraded (HDOP ≥ 3.8 for 10 s straight — dead-reckoning), the device's fresher GPS fix takes the position over; the truck's fix takes it back the moment the device fix isn't fresh. Owner decision: "Yes, prefer the good comma fix." | Fable: SHIP, with two follow-ups fixed by the next commit — the one weekend firing (a parked truck) was net-negative, and a "fresh" device fix had no steadiness check. |
| `62987f7e5d` **gpsdrgate2pnw** | Narrows the above: the device fix is only preferred while the truck is **moving**, and only once the device fix has itself been steady for 10 s. Fixes what Fable found: a parked, HDOP-degraded truck (exactly right) lost to a device fix jittering 3.7 m mean / 8.7 m max for 381 s. | Fable: **APPROVE** (this commit and its parent together). Weekend replay: DR now switches nowhere unwanted. |
| `1c5ea1d9e2` **gpsdrgate2pnw** | Test-only: the in-episode receiver-switch test can now actually observe a curve-cap restore (the harness's simulated cruise set previously never moved, so the "not a restore" half of the test was unfalsifiable). No driver-visible change. | Fable, gpslag2pnw review point (b). |
| `bdf3094cc2` **gpslag2pnw** | Lightning/ICBM only. ICBM now projects the GPS fix forward to the current tick (it was reading a position up to ~1.8 s stale) instead of the raw 1 Hz read — curve-slowdown start timing is **unchanged by design** (the projection keep-time is calibrated so the median start point doesn't move). A fix older than 5 s counts as no GPS for curve lookups. Owner decision: "Build it, keep curve timing." | Fable: **SHIP**. Projection error stays bounded (~30 m along-track normally, ~14 m at the 5 s ceiling, absorbed by the 60 m point-match). |
| `3871d12733` **gpsdrgate2pnw** | If GPS goes stale (both receivers gone ≥5 s) in the middle of a curve slowdown, the truck now **holds** its current speed cap instead of restoring back up toward a curve it can no longer see — held for up to 60 s, or until the fix returns, a new curve binds by vision, or the truck has driven well past where the cap was set. | Fable re-review: **SHIP** (first pass was REQUEST CHANGES on two findings — a vision co-bind silently erasing the hold, and the 60 s timeout wrongly restoring toward a still-unlocated curve — both fixed here). **Channel tip.** |

**Installed on the truck 2026-09-14, verified healthy 00:30 PT** (gpsdr2pnw through the final
gpsdrgate2pnw commit, tip `3871d12733`).

## Device settings — lat-accel cap

**Not a code change.** At the driver's request, around 21:4x PT, `/data/pnw/lataccel_limits.json` on the
truck was hand-edited:

```diff
- [[50,6.0],[60,5.0],[70,4.0],[80,3.0]]
+ [[50,5.0],[60,5.0],[70,4.0],[80,3.0]]
```

i.e. the lateral-accel cap below 50 mph dropped from 6.0 to 5.0 m/s², matching the 5.0 already used
50-60 mph. Backup of the previous file: `/data/dirk/org/lataccel-2026-09-13/`. Evidence and the
investigation that preceded it (the cap had never actually bound on the weekend's mountain roads, so
this is a margin decision, not a fix for a measured problem): `drives/2026-09-12/central-oregon-weekend/STEERING_LIMIT_ALERTS.md`.

---

## Decisions (owner, 2026-09-13)

- **Speed-limit slowdowns never restore.** Police is the only exception. Curves keep restoring.
- **An entered lower speed-limit zone sets the driver's percentage-above-limit once, then forgets it** —
  no memory of the pre-zone set, no auto-restore on a later limit rise.
- **No Gemini reviews, ever** — Fable is the only reviewer (`docs/CODING-POLICY.md`). The `gemini` skill
  stays installed but its presence is not an invitation.
- **Code updates download over any link, metered included.**
- **Unmetered beats default beats metered, for every saved network** — the generic WiFi cost-ranking
  rule (not just for the truck's specific Starlink/iPhone pair).
- **MADS chimes stay silent while MADS keeps steering**; a full disengage still chimes.
- **mapd default pin is v2.3.1.** Do **not** adopt upstream `pfeiferj/mapd` PR #136. Our own
  tile-validation work was opened upstream as `pfeiferj/mapd#138`.
- **The ON/OFF cruise button means everything off** (reaffirmed).
- **Gas-set (engagegoal2pnw) refinements — SHIPPED** (10-commit stack, `271a4b7d3c..b1288f5004`, see
  *Engagement — gas-set* above): first accelerator press after a brake is the only one that arms; regen
  slowing the truck is ignored (not read as a deceleration); the arm waits 1.0 s after lift-off before
  firing; and — revised after the unexplained 55 mph engagement below — the cancel rule is **option
  (a): cancel, and steering drops too**, if the truck sets itself materially (>3 mph) above the driver's
  wanted speed, rather than a raise-only guard.
- **Keep full (non-deferred) uploads over the iPhone hotspot.**
- **The lateral-accel cap is 5.0 m/s² below 50 mph** (device file updated, see above).
- **Truck GPS selection is approved in principle — the ~1.8 s plumbing lag is fixed** (`gpslag2pnw`
  `bdf3094cc2` projects the GPS fix forward each ICBM tick; curve-slowdown start timing unchanged by
  design). A degraded truck fix yields to a steady, moving device fix (`gpsdr2pnw`/`gpsdrgate2pnw`), and
  a fix going stale mid-curve holds the running cap instead of restoring blind (`gpsdrgate2pnw`
  `3871d12733`, the channel tip).

---

## In progress (not shipped this session)

- **Tailgate chime FORScan session** — tooling (`scripts/read-ford-asbuilt.py`, the `0x313` before/after
  test) is ready; the owner runs the session at the truck.

---

## Investigations and reports (2026-09-10 → 2026-09-13)

These are analysis, not code — several of today's shipped fixes came directly out of them.

- [`drives/2026-09-12/central-oregon-weekend/DRIVE_REPORT.md`](../../../../drives/2026-09-12/central-oregon-weekend/DRIVE_REPORT.md) —
  the weekend's curve-slowdown / lead-car / live 25-zone-restore-bug analysis that produced
  `icbmrestorecap2pnw` and `curvelead2pnw`; also the 22-drop "what ended cruise" study behind
  `accdroplog2pnw`, and a full health sweep (uploads, storage, driver monitoring, lane changes, mapd
  coverage, system health).
- [`drives/2026-09-12/central-oregon-weekend/GPS_TRUCK_VS_COMMA.md`](../../../../drives/2026-09-12/central-oregon-weekend/GPS_TRUCK_VS_COMMA.md) —
  truck GPS vs comma GPS accuracy/heading/availability comparison; recommends switching specific
  consumers only, after fixing the plumbing lag (`gpsfix2pnw`/`gpssel2pnw`, in progress).
- [`drives/2026-09-12/central-oregon-weekend/STEERING_LIMIT_ALERTS.md`](../../../../drives/2026-09-12/central-oregon-weekend/STEERING_LIMIT_ALERTS.md) —
  what actually raises the "Take Control / Turn Exceeds Steering Limit" alert (a tracking-error monitor,
  not a limiter). Of 39 alerts attributed: 0 from our lateral-accel cap, 0 from panda, 28 with no software
  clip involved, up to 10 (4 confirmed/4 probable/2 possible) from the angle port's own curvature-deviation
  clip, and 1 the PSCM itself reporting `LimitReached`. A 5.0 lat-accel cap below 50 mph doesn't change
  this weekend's data — informed the device-settings change above.
- [`drives/2026-09-13/corvallis-resume-55/DRIVE_REPORT.md`](../../../../drives/2026-09-13/corvallis-resume-55/DRIVE_REPORT.md) —
  an unexplained stock-ACC engagement at 55 mph from 34 mph; the raw CAN shows no driver button press,
  only openpilot's own gas-set SET− tap, and the PCM's choice of 55 mph is unexplained. Still open.
- [`drives/2026-09-11/README.md`](../../../../drives/2026-09-11/README.md) (`corvallis-local-trips`,
  `outbound-corvallis-eugene`) — an 18-minute `relayMalfunction` latch, an FCW during stock-ACC braking,
  a 37 s phone-distraction flag with no DM alert, and a ~2h46m gap in the device log (17:38-20:24 PT)
  that storage-floor deletion later destroyed for good.
- [`drives/2026-09-10/corvallis-evening/DRIVE_REPORT.md`](../../../../drives/2026-09-10/corvallis-evening/DRIVE_REPORT.md) —
  a brief `selfdrivedLagging`, two "Take Control" alerts, and recurring mapd limit flicker at one
  Corvallis intersection.
- [`docs/MADS-RESUME-TO-DRIVER-SPEED.md`](../../../../docs/MADS-RESUME-TO-DRIVER-SPEED.md) — design-only review of "resume
  should take my current speed as the set"; found the feature already exists (`gasset2pnw`), traced
  several "wrong speed" reports to RES restoring the PCM's own memory, and specified the `engagegoal2pnw`
  follow-up (in progress).
- [`docs/BLUEPILOT7-FORD-STEERING-AUTHORITY-REVIEW.md`](../../../../docs/BLUEPILOT7-FORD-STEERING-AUTHORITY-REVIEW.md) —
  re-review of BluePilot 7.0's angle-control approach against our measured Lightning limits; prioritized
  follow-up list, none shipped today.
- [`docs/NETCOST-STARLINK-TO-HOTSPOT.md`](../../../../docs/NETCOST-STARLINK-TO-HOTSPOT.md) — the diagnosis and
  prototype behind `netscanpin2pnw`/`netrank2pnw` above.

---

**Channel tip:** `origin/3devpnw` = `3871d12733` (`gpsdrgate2pnw`).
