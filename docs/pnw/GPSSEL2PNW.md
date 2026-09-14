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
