---
updated: 2026-09-13
status: unreviewed     # current | drifted | superseded | unreviewed
---

# GPSSEL2PNW — which GPS fix reaches `LastGPSPosition`

Evidence: `drives/2026-09-12/central-oregon-weekend/GPS_TRUCK_VS_COMMA.md` (workbench, not in this repo).
Owner decision 2026-09-13: "yes build the truck GPS selection, fix check first".

`system/mapd/mapd_configd.py` is the **only writer** of the `/dev/shm` `LastGPSPosition` blob. Every
Python GPS consumer reads that blob. The mapd Go binary does **not**: it subscribes to `gpsLocation`
itself and is untouched by everything here.

## 1. gpsfix2pnw — the fix check (both cars)

**The bug.** The bridge wrote every `gpsLocation` message while the service was `alive`, with no
`hasFix` check. At the Sat 2026-09-12 06:28:57 PT cold start (NF Road 70) four `hasFix=False` messages
2.18–2.42 km from the truck reached every consumer.

**A second defect in the same lines.** The loop runs at `mapdOut`'s 20 Hz, and `alive` stays True for
10 s after the last message. The bridge rewrote the last message with a fresh `ts` for those 10 s, so
`ts` claimed "now" for a fix up to 10 s old, and a no-fix sample could never let `ts` expire.

**The change.**
- A position is written **once, when a message arrives** (`sm.updated`), and **only if `hasFix`**.
  `ts` is now the arrival time of that fix. Freshness readers (`network_arbiterd._read_gps`,
  `location_servicesd._cur_speed`, 10 s windows) now age a fix from when it arrived.
- The blob gains `"src": "device"` (see §2: it keeps the later selector byte-identical on the Tesla).
- The region download / "Refresh this location map" `has_fix` was `alive` only; it now also needs
  `hasFix`, so a no-fix position cannot choose a region.
- Rule 2: `cloudlog.event("mapd_configd_gps_fix", state=fix|nofix|silent, prev, prev_duration_s,
  nofix_dropped)`, change-only. `silent` = no message for `GPS_SILENT_S` = 3 s. Expect one line at
  boot and about three per drive (silent → nofix → fix → silent).

**Blob schema** (key order as written): `latitude, longitude, bearing (deg), speed (m/s), src, ts
(time.monotonic() at arrival)`.

**Not fixed here, flagged:**
- The mapd binary reads raw `gpsLocation` and does not check `hasFix` either (`mapd v2.3.1 main.go`).
- `ces_pnw`, `vtsc_controller`, `location_servicesd._read_mem/_cur_gps` and `ui/widgets/network.py`
  never check `ts`; they keep the last position through an outage. Unchanged by this commit.
- The first `hasFix=True` fixes after that cold start were still 56–72 m off. `hasFix` is the only
  validity field `qcomgpsd` sets (`horizontalAccuracy` is always 0), so they are not gated.

**Tests** (`system/mapd/tests/test_gps_fix_gate.py`, harness `configd_replay.py`): the real `main()`
loop at 20 Hz, replaying the real cold-start qlog stream, with `network_arbiterd._read_gps` and
`location_servicesd._read_mem` reading the blob after every iteration. Five mutants (no `hasFix`
gate; write while `alive`; region `has_fix` = `alive`; no log; `nofix` reads as `fix`) all fail it.

## 2. gpssel2pnw — select the truck's CAN GPS (Lightning only, by capability)

**Rule.** `LastGPSPosition` carries the car's fix (`"src": "car"`) while the `CarGps` feed is provably
live, and the device fix (`"src": "device"`, §1) otherwise. mapd itself stays on `gpsLocation`: once
mapd sees `gpsLocationExternal` it never falls back, so a frozen truck feed would freeze mapd.

**Capability, not fingerprint.** `PnwVehicle(CarParams).car_gps` (`selfdrive/controls/lib/pnw_vehicle.py`),
re-read every 5 s from `CarParams` (cleared at every onroad transition, rewritten by card for the car
actually attached). Not capable: `CarGps` is never read and every write is §1's, byte for byte. Not
mirrored in `opendbc/car/pnw_vehicle.py`: nothing there consumes it.

**Transport: the existing `CarGps` mem-param**, published by card (`opendbc ford/carstate.py
_publish_car_gps`, every 100th CarState update, ~1 Hz) as `{lat, lon, hdg, spd MPH, sats, hdop, ts, age}`.
No new writer, no msgq subscription in this background daemon (the 2026-07-13 commIssue rule).
- Latency at the blob write (derived from the report's measurements, not re-measured on this code): the fix is ~0.2 s old at CAN receipt, plus the decimation phase
  (`age` 0.00–1.03 s, median ~0.5), plus this loop (≤0.05 s while mapd publishes at 20 Hz, ≤1 s if
  mapd is down): **~0.7 s median**. The device fix after §1 is **~0.6 s** (0.57 s modem, written on
  arrival). So this transport buys accuracy, heading and coverage, **not latency**. Publishing on a new
  CAN frame instead of every 100th update would remove ~0.46 s; that is an opendbc change, not built.

**The car is used only while ALL hold** (`CarGpsSource`, one judgment per loop):

| check | fails as | threshold and why |
|---|---|---|
| a publish exists and parses | `absent` / `unreadable` | — |
| its `ts` changed recently (judged on THIS process's clock, never ours-vs-publisher wall clock: boots run on a bogus clock for 7–68 s) | `silent` | 2.5 s; publishes are ~1 s apart |
| values in range (DBC sentinels: lat raw 255 → 166°, heading 65535 → 655.35, speed 254/255) | `invalid` | 360.0 is valid (`round(359.96, 1)`) |
| CAN frame age at publish | `stale_can` | 2.0 s; 0.00–1.03 s on all 68,175 weekend publishes. The wrong-bus class. |
| lat, lon and heading not identical on 3 consecutive publishes **while moving** | `frozen` | one repeat is normal decimation. Moving = truck ≥ 3 mph, or device fix speed > 5 m/s (device standstill noise reached 4.17 m/s) |
| 3 consecutive healthy publishes | `reacquiring` | no flapping between receivers ~6 m apart |

Residual, undetectable here: truck content frozen at 0 mph while the device has no fix either.

**Weekend replay** (every `car_gps` read in both datasets, 68,176 publishes, through the real
`CarGpsSource`): zero `frozen`, `stale_can`, `silent` or `invalid`; one acquisition per boot. The one
`unreadable` is an analysis artifact (the loader nulls one pre-sync `age`).

**Rule 2 logs** (both change-only): `mapd_configd_car_gps_capability {capable}`, and
`mapd_configd_gps_source {src: car|device|none, prev, car: <kind above>, car_detail, device: fix|nofix|silent}`
on every change of source or of why the car is not used. Expect ~4 lines per Lightning drive.

**Consumers of `LastGPSPosition`** (line numbers on gpssel2pnw rebased onto 3devpnw `67011d2f53`). None needed a change; all read keys by
name, so `src` is ignored except by the ces telemetry.

| consumer | reads | effect of `src: car` |
|---|---|---|
| `ces_pnw.py:2567` `_read_map` → ICBM `icbm_far_map_candidate`/`icbm_map_reach`/`upcoming_curve`, `polyline_curvature(_at)` `:2590`/`:3556`, bearing history `:2809`, `gpsSrc` in records `:3078`/`:3829` | lat, lon, **bearing**, src (new, → `gpsSrc`) | better cross-track and heading; latency unchanged (see above) |
| `vtsc_controller.py:167` | lat, lon, **bearing** | none: Tesla op-long, no capability |
| `location_servicesd.py:703` `_read_mem` (cone/behind gate) · `:459` `_cur_gps` · `:485` `_cur_speed` (police ≥45 mph gate, 10 s ts) · `:1510` net log | lat, lon, **bearing**, speed, ts | heading stable at stops; speed is integer MPH × 0.44704, same factor as the gate |
| `network_arbiterd.py:458` `_read_gps` (10 s ts; gpscarry) | lat, lon, ts | position at boot/tunnels |
| `ui/widgets/network.py:365` geofence capture (no ts check, flagged) | lat, lon | none |

Heading from the blob reaches: ces_pnw (ICBM polyline `ahead` checks, steer-event bearing history,
`bearing`/`heading` telemetry), location_services (forward cone, behind gate, police direction), and
VTSC on the Tesla (unchanged). While the truck is selected, `lat`/`lon` in ces_events equal `car_gps`, so
the side-by-side comparison channel only exists on `gpsSrc: device` ticks (qlog `gpsLocation` keeps the
device track).

**Tesla byte-identity, measured**: the same 52 s replay (cold start + tunnel, with a stray `CarGps`
feed) through gpsfix2pnw's and gpssel2pnw's real loops: 13,754 mem writes + 1 param write, all-writes
sha256 `4aad6a81998d1349` on both, `CarGps` never read. The Lightning run without `CarGps` hashes the same.

**Tests**: `system/mapd/tests/test_gps_source_select.py` (tunnel and cold-start replays, the freeze
classes, parked-and-charging, capability flips, Tesla identity) and
`selfdrive/controls/lib/ces_pnw/tests/test_gpssel_telemetry.py` (blob → real `_read_map` → records).
19 mutants killed; one equivalent (dropping `car_gps_capable and` from `use_car`: the source is only
fed while capable and is replaced on every capability change).

## 3. gpsdr2pnw — a degraded truck fix yields to a fresh device fix

Owner decision 2026-09-13: "Yes, prefer the good comma fix".

**Indicator.** `CarGps` carries one quality signal, `hdop`. `sats` is the DBC's constant Invalid value
(31) and the 0x463 `Gps_B_Falt` / `GPS_Actual_vs_Infer_pos` / 0x464 `GPS_dimension` flags are not
decoded. They are also unverified in a degraded state: all 420 frames in the 7 local rlogs read
0 / 0 / 3D with HDOP < 3.8. HDOP's Unknown/Invalid sentinels (6.0/6.2) count as degraded.

**Rule** (`CarGpsSource.dr`, `CAR_GPS_DR_HDOP = 3.8`, `CAR_GPS_DR_ENTER_S = 10`, `CAR_GPS_DR_EXIT_S = 5`):
degraded after HDOP ≥ 3.8 on every publish for more than 10 s; recovered after HDOP < 3.8 on every
publish for 5 s. One good or bad publish restarts the respective run, so a flapping HDOP never switches.
While degraded, the device fix is used if it is fresh (`hasFix`, ≤ 3 s old); the moment it is not, the
degraded truck fix takes the blob back. Log: `car=dr` (change-only, like every kind), and the `ok`
detail now names the HDOP.

**Weekend + I-5 replay** (68,181 ticks through the real `CarGpsSource` and `main()`'s selection rule;
device "fresh" = the logged device position changed within 3 s, because these logs predate `gpsSrc`
and the old bridge rewrote a stale fix for 10 s):

| HDOP ≥ 3.8 run | duration | device fix | DR entered | switched to device |
|---|---:|---|---|---|
| SR 99 tunnel, Tue 09-08 19:29:23 PT | 120 s | dead | 19:29:34 | 19:31:26–19:31:29 only (tunnel exit, until the truck's HDOP had recovered for 5 s) |
| Cold start, Sat 09-12 06:24:42 | 238 s | none | 06:24:52 | no |
| Parked after boot, Sat 09-12 14:12:14 | 387 s | updating | 14:12:25 | 14:12:43–14:18:48 |

**FR 6010 (Sun 10:44, truck 17 m off at 2.6 m/s) is not caught**: the truck's HDOP read 0.6–1.0
there. No rule based on what `CarGps` carries distinguishes it.

**Tesla**: same 52 s replay, all-writes sha256 `4aad6a81998d1349`, identical to gpssel2pnw.
**Tests**: `system/mapd/tests/test_gps_dr_prefer_device.py`; 9 of 9 mutants killed.

**gpsdrgate2pnw (Fable, gpsdr2pnw review).** The device may take a degraded truck's blob only:
- **while moving**, and for `CAR_GPS_DR_EXIT_S` (5 s) after the last moving publish, so a crawl cannot
  swap receivers every publish. Sat 14:12 was parked, with the truck exactly right and the device
  jittering 3.7 m mean / 8.7 m max;
- **with a device fix that has been a steady `fix` for `CAR_GPS_DR_ENTER_S`** (10 s). qcomgpsd publishes
  no accuracy, and the first `hasFix` samples after the Sat 06:28 cold start were 56–72 m off.

A NaN HDOP is `invalid`. Replayed on the weekend + I-5 logs, DR now switches **nowhere**:
- Sat 14:12 no longer switches.
- The SR 99 exit's 3 s device window is gone too: the device fix had just reacquired (under 10 s
  steady) when the truck's HDOP recovered.

`GPS_Actual_vs_Infer_pos` (0x463) cannot be forwarded without an opendbc change: the pinned opendbc
(`78477c72`) registers only `APIMGPS_Data_Nav_1_FD1`/`_3_FD1`.
Tesla writes identical (sha256 `e9045cfcb417f0d7`). Mutants 7/7 killed.

## 4. gpslag2pnw — ICBM projects the position to every tick, keeping today's curve timing

Owner decision 2026-09-13: "Build it, keep curve timing". Lightning only: ICBM runs only under
`PnwVehicle.ces_shadow`.

**Blob.** `fix_ts` = the monotonic time the fix was valid. Device fix: its arrival minus
`DEVICE_GPS_FIX_LATENCY_S = 0.57` (measured qcomgpsd latency, p1/50/99 0.43/0.57/0.67 s; GNSS time is
wrong until the clock syncs). Truck fix: its CAN receipt (`ts − age`). The truck position is itself
~0.2 s older than its receipt (per boot −0.08…+0.62 s); no constant corrects that.

**ICBM** (`ces_pnw.icbm_project_position`, called on every ~4 Hz `_icbm_step`):
- It projects the 1 Hz-read fix along its bearing to `now − ICBM_GPS_LAG_KEEP_S` (1.59 s), not to now.
- Every ICBM start rule is a distance threshold, and the weekend's starts were decided on a position a
  median 1.79 s old (p5/95 1.09/2.31). Projecting to now would start every slowdown ~v × 1.8 s earlier.
- The keep is centred on the **truck fix**, the Lightning's primary source (owner 2026-09-13): 1.79 s minus
  the truck position's own ~0.20 s receipt lag = 1.59 s.
- **Device-fix starts (the fallback) sit ~0.2 s earlier by design.**
- All ICBM position lookups use the one projected position: the near map candidate (re-derived
  in `_icbm_step`), the far candidate, map reach, turn direction, point-matched curvature, and
  path-behind. CES's `sig`, CES `upcoming_curve` and VTSC are untouched.
- A fix older than `ICBM_GPS_MAX_AGE_S = 5 s`, or from a previous boot, is **no GPS** for those lookups:
  no map/far candidate, reach 0, so vision may start.
- A missing `fix_ts` or bearing uses the position unprojected, as before.
- Logging: `cloudlog.event("ces_icbm_gps", state=proj|stale|raw|none)`, change-only. The fix age is in
  `ces_events` as `icbmGpsAge`, which reads null on the Tesla.

**Timing-equivalence replay.** This is not a log replay: the map polyline is not logged. It is a closed
loop through the real `_icbm_step`:
- All 92 weekend + I-5 map/far ICBM episode starts: logged vEgo, vSet and mapV on a straight approach, with
  mapd's path at 1 Hz and ICBM at 4 Hz.
- Old pipeline (gpsdr2pnw code) calibrated to the measured held-fix age: model p5/50/95 1.10/1.79/2.46 s
  vs measured 1.09/1.79/2.31.
- 72 episodes start in every run; the other 20 don't start in the simulation on their logged numbers.

| start distance to the curve, p10 / p50 / p90 | m |
|---|---|
| old | 69.8 / 168.8 / 326.1 |
| new, truck fix (primary) | 69.0 / 165.7 / 327.6 |
| new, device fix (fallback) | 74.3 / 170.5 / 329.3 |

| start-time shift new − old per episode, p10 / p50 / p90 (s; + = earlier) | truck fix | device fix |
|---|---|---|
| | −0.75 / **0.00** / +0.50 | −0.25 / **+0.25** / +0.75 |

| start-time error vs a perfect receiver (same code, lag exactly 1.59 s), p10 / p50 / p90 | s | within one tick (0.25 s) |
|---|---|---|
| old | −0.75 / −0.25 / +0.25 | 36/72 |
| new, truck fix | −0.50 / −0.25 / 0.00 | 29/72 (the receipt lag; the replay draws it uniform −0.08…+0.62 s per start) |
| new, device fix | 0.00 / 0.00 / 0.00 | 70/72 |

On the truck fix the median start is unchanged (0.00 s). The per-episode shift is the old pipeline's own
spread being removed. Device-fix starts are one tick earlier by design.

**Receiver switch** (Fable, gpssel note 3). A 6 m along-track step plus the 0.2 s receipt lag moves
ICBM's projected position by ≤ 6 m + v·0.2. That is about one tick of travel; the old 1 Hz read stepped
v·1 s (25 m at 25 m/s) every second. Tested: the start moves by at most that step, and a switch inside an
episode neither ends it nor turns it into a restore.

**Tesla**: same 52 s replay through gpsdr2pnw and this commit:
- mapd_configd writes are identical once `fix_ts` is removed (sha256 `f347c36eaa4f6db8`).
- CES `_read_map` + `upcoming_curve` and VTSC `_read_map` + `polyline_curvature` over every write hash
  identically (`b910a46ec0e071b8`; 21 distinct outputs, 11 with a finite candidate).
- `CarGps` is never read; the ICBM projection is unreachable (`ces_shadow` False).

**Tests**: `selfdrive/controls/lib/ces_pnw/tests/test_gpslag_icbm.py`; 16 of 16 mutants killed.

**Stale fix mid-episode (gpsdrgate2pnw, Fable gpslag review).** If both receivers are gone for more than
5 s while a **map/far** cap episode runs, the vanished candidates would read as "curve cleared". The result
would be 3 s silent, then a **restore toward the curve the map had rated**: Fable's probe saw the publish
empty 137 m before it.

Unknown is not clear, so the running cap is **held** (`icbmSrc` `gpsHold`). It is held until the truck has
driven past where its binding candidate was, plus `ICBM_MARGIN_M` (integrated from vEgo). Then the normal
clear → apex-passed (1 s) → restore path takes over, with its in-curve pause.

The hold ends early when:
- the fix comes back (`gpsBack`),
- a vision curve binds (`capBound`), or
- `ICBM_GPS_STALE_HOLD_MAX_S` passes (60 s, `maxTime`).

60 s is a judgement, not a measurement: a held lower set costs speed, not safety. A vision-sourced episode
is never held, because stale GPS does not change what vision sees.

Map provenance is sticky within an episode (Fable B1): vision co-binding the same curve must not cancel the
hold. The provenance resets when a new episode starts.

At `maxTime` the episode **ends without a restore** (Fable B2): the curve is still unlocated. The set stays
where ICBM put it, for the driver to raise.

During a hold ICBM behaves like a live cap:
- a driver SET+ is tapped back down;
- a pedal press, cruise off or the CES button end the hold, like any episode.

Logged as `ces_icbm_stale_hold {hold | release, why, held_s}`. A declined hold is logged once per episode:
`declined, why=notMap|noDistance|noCap`. Mutants 9/9 + 4/4 killed.

The closed-loop harness's stock set now follows the published caps, so a restore is observable. A positive
control proves it, and the in-episode switch test (Fable, point b) uses it.

## 5. truckdecode2pnw — the truck's own dead-reckoning flag, logged next to the inference

**What.** opendbc now decodes `0x463 APIMGPS_Data_Nav_2_FD1.GPS_Actual_vs_Infer_pos` (1 = "Inferred_Position",
0 = "Actual_Postition") into the `CarGps` blob as `dr`, with `drAge` = the Nav_2 frame's own age.
`mapd_configd` reads it as `CarGpsSource.truck_dr` (None when absent, not 0/1, or `drAge` outside
0..`CAR_GPS_MAX_AGE_S`). It is added to every `mapd_configd_gps_source` event as `truck_dr`, next to the inferred
state (`car == "dr"`), and a change in the flag alone also logs a line. `ces_events` carries it inside `car_gps`.
**It selects nothing.**

**Evidence** (all 24 local Lightning rlog segments; `drives/2026-09-12/central-oregon-weekend/TRUCK_DECODE.md`):
- 0x463 is on src 0, the bus the powertrain parser reads, at 1 Hz in every segment.
- Flag 1 on all 243 frames of the Sat 06:24 PT cold start (HDOP 3.8–5.4, `GPS_dimension` 0/1); 0 on all 1,140
  frames of 22 normal segments; one 1 → 0 transition (Sat 14:18:45 PT, parked, HDOP already 1.0 a frame earlier).
- 1,345 publishes through the real opendbc carstate and the real `CarGpsSource`:

  | truck flag | inferred DR | publishes |
  |---|---|---|
  | 0 | no | 1,102 |
  | 1 | yes | 231 |
  | 1 | no | 12 — 10 are the inference's 10 s entry at the cold start, 2 are the flag lagging HDOP's recovery |
  | 0 | yes | **0** |

**Why control stays on the inference.** No publish had the flag saying "actual" while the inference said DR, but
the sample is one cold start and one recovery. The flag was never seen **entering** DR, and never in a tunnel
(the SR 99 drive of 09-08 has no local rlog). The owner's rule — a dead-reckoned truck fix yields to a good comma
fix — is unchanged. Re-evaluate after a drive through the SR 99 tunnel, comparing `truck_dr` with `car == "dr"`.

**Tests**: `system/mapd/tests/test_gps_truck_dr_flag.py` (real `main()` loop; positions byte-identical whatever
the flag says; a cross-repo seam test with real frames through the real publisher). 11 of 11 mutants killed.
