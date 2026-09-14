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
