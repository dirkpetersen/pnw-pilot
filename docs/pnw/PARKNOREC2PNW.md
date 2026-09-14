---
updated: 2026-09-14          # git-derived; bump when you edit this file
status: unreviewed     # current | drifted | superseded | unreviewed
---

# PARKNOREC2PNW — the device does not record while the shifter is in Park

Branch `parknorec2pnw` off `origin/3devpnw` `95dd1d968b`. **Not pushed, not deployed, Fable review pending.**

**Requirement (driver, 2026-09-05):** *"it should never be recording when the shifter is on Park, never."*
The owner added on 2026-09-13 that this applies to the **Tesla** as well as the Lightning.

**Baseline to beat.** Measured on the truck 2026-09-13 19:20 PT, parked at a hotel, running `95dd1d968b`:
`IsOnroad=1`, `GearPark=1`, `SkipVideoWhenParked=1`, `ThinRlogWhenParked=1`. It wrote 11 segments in
10 min (newest: qcamera 2.31 MB, qlog 348 KB, rlog 348 KB), about **3 MB/min, ~180 MB/h**. There were
1157 segment dirs, `/data` was 90% full, and uploads were running over the driver's phone.

## Decisions

### What "record" means
**No route segments.** While the gate holds, nothing is written: no rlog, qlog, qcamera, fcamera,
ecamera or dcamera, and no thumbnails. **Out of scope**, because these are not route segments: `bootlog`
(one per manager start), crash logs and tombstones, swaglog (`/data/log`), and
`/data/pnw/ces_events.jsonl`.

### Where the gate lives: loggerd's manager `should_run`
`process_config.logging()` returns False while `ParkRecordGate` holds, so the manager stops **loggerd**
on its own. Nothing else is gated:
- **camerad, modeld, card, controls and selfdrived keep running**, so openpilot is ready the moment the
  shifter leaves Park.
- **encoderd keeps running**, so the video encoders are already warm. Measured from device swaglog
  (n=14 cold onroad starts), encoderd needs **1.0–1.6 s** from process start to `encoder init`. That is
  why it is not restarted.
- **Why not inside loggerd:** that would mean closing and reopening routes, and resyncing encoder
  segment offsets, in boot-critical C++. Stopping the process uses the stock ignition-off/on lifecycle,
  which runs on every drive.

### Truth source: `GearPark`, with two writer changes (`selfdrive/car/gear_park.py`)
The manager must not subscribe to carState (CLAUDE.md Rule 3, memory
`no-carstate-sub-in-background-procs`), so the gate reads the param. The per-car decode was verified
by feeding real CAN frames through the pinned opendbc (`tests/test_gear_park.py`):

| car | message / bus | Park | Drive | SNA / unknown | **never received** |
|---|---|---|---|---|---|
| Tesla Raven HW3 | `DI_torque2.DI_gear`, chassis bus 1 | `1` → park | `4` → drive | `7`/`0` → unknown | **unknown** |
| Lightning (automatic) | `PowertrainData_10.TrnRng_D_Rq`, bus 0 | `0` → park | `3` → drive | `14` → unknown | **unknown** (since gearunknown2pnw) |

Before pnw-opendbc `gearunknown2pnw` the Lightning read **park before its gear message had ever arrived**:
CANParser starts every signal at 0, and `TrnRng_D_Rq` 0 means "Park". That is fixed for the Lightning, but
**74 other platforms in the pinned opendbc still read park from a silent bus** (Hyundai/Kia/Genesis, RAM,
some Toyota/Subaru; measured 2026-09-14). The writer is car-agnostic, so only `canValid` can tell a real
Park from that. The writer rules follow from that:
1. **SET** GearPark only on valid CAN that reads Park. This rule is unchanged.
2. **CLEAR** it on any tick that decodes a **known** non-Park gear, **even when CAN is invalid.**
   Before, invalid ticks were ignored in both directions, so a CAN fault that spanned Park → Drive kept
   GearPark True for the whole drive. That now means an unrecorded drive. An `unknown` gear on
   invalid CAN keeps the last value, because it is not evidence of leaving Park.
3. **Seed False at the top of `Car.__init__`**, before the CAN wait and fingerprinting. A card that
   crash-loops or hangs there can no longer leave a stale True in place.
4. **Log it loudly** (`gear_park_unconfirmed`, `error=True`, once per episode) when the reading has been
   park or unknown for 60 s but GearPark is False.

### Hysteresis
- **Hold** after GearPark has read True continuously for **`PARK_HOLD_S` = 30 s**.
- **Release** on the first manager tick where it reads False.
- A parking-lot shuffle (Park stints under 30 s) neither restarts loggerd nor splits the route.
  Routes are bounded to at most one new route per Park stint of 30 s or longer.
- Cost: the first 30 s of every Park stint are still recorded, thinned by the existing parked gates
  (about 1.5 MB).

### The first seconds after leaving Park
| step | latency | source |
|---|---|---|
| card reads the non-Park gear and writes GearPark False (async) | ~10 ms | code (100 Hz) |
| manager tick evaluates loggerd | ≤ 0.5 s | code (deviceState 2 Hz) |
| loggerd spawn → `logging to …--0` (new route, rlog/qlog start) | **0.60–1.06 s** | measured, n=14 cold starts |
| fcamera/ecamera first keyframe | ≤ 1.5 s more | code (GOP 30 @ 20 fps) |
| qcamera first keyframe | ≤ 0.75 s more | code (GOP 15) |

**rlog starts about 0.6–1.6 s after the shift, and HEVC about 0.6–3.1 s after.** The car is stationary
with the brake held while leaving Park, so what is lost is the shift itself.

Caveats:
- If loggerd was only just stopped, the manager first blocks on the old process exiting (stock join,
  5 s cap, then SIGKILL). Shutdown time was **not measured**: the device dropped off the hotspot.
- There is a stock edge case, now hit once per Park exit instead of once per ignition. If loggerd starts
  within milliseconds of an encoder segment rollover, the first segment can drop video for one encoder
  until the 72 s timed rotation.

## Failure modes (every one records)
| case | what happens | logged |
|---|---|---|
| car never writes GearPark / never decodes a gear | never holds | `gear_park_unconfirmed` (card) |
| Lightning boots into a quiet-CAN charging session (canValid false from the start) | **never holds: known gap**, stays at today's ~180 MB/h. gearunknown2pnw makes a per-car fix possible but did not ship it (owner question in that commit) | `gear_park_unconfirmed` |
| CAN fault spans Park → Drive | a decoded drive gear clears GearPark, and loggerd starts on the next tick | `gear_park` value=False |
| card crashes while holding | releases on the next tick (card not alive) | `park_record_gate` hold=False reason=card_not_running (+ manager "Restarting card") |
| card restarted with a stale True | the seed clears it; otherwise the full 30 s hold starts again | — |
| card alive but hung (not writing) | **not detected**: stale GearPark persists | residual risk |
| `params_pyx.so` older than `params_keys.h` | records | `cloudlog.exception` once |
| owner wants recording in Park | `RecordWhileParked=1` over SSH, effective next tick | `park_record_gate` reason=RecordWhileParked |

## Interactions
- **`SkipVideoWhenParked` / `ThinRlogWhenParked`: not dead, kept.** They still shape the 30 s hold
  window and the quiet-CAN gap above.
- **Pre-existing, NOT changed (Rule 5):** `loggerd.cc` sets `car_parked` from carState with no
  `canValid` check. Its comment says a never-decoded gear reads `unknown`. That was false on the
  Lightning until gearunknown2pnw and is **still false on the 74 platforms above**, where a drive whose
  gear message never arrives would skip video and thin the rlog (only with `SkipVideoWhenParked` /
  `ThinRlogWhenParked` on; both default OFF).
- **Uploader:** `pass2_allowed` stopped consulting `parked` in `uploadanywifi2pnw`, so it is unaffected.
- **Deleter:** unchanged. Fewer new segments means the upload queue can finally drain.
- **Bookmarks** (`userBookmark`) pressed during a hold are not preserved, because loggerd is not running.

## Tests
Both suites use real Params and the real opendbc decode:
- `system/manager/test/test_park_record_gate.py` drives the real `ensure_running` and loggerd's real
  `should_run` tick by tick.
- `selfdrive/car/tests/test_gear_park.py` covers the writer rules and card's call site.

34 tests. **18 mutants, all killed:** every gate branch, the loggerd wiring, `_card_alive`, each writer
rule, the unconfirmed log, the seed position, and the call site (stringified enum, ignored canValid).

## On-car verification (after the owner approves a deploy)
1. **Park 60 s.** Expect `park_record_gate hold=True` in swaglog, no new segment dir, and loggerd absent
   from `ps`.
2. **Shift to Drive.** Expect a new route `--0` within ~2 s.
3. **Repeat on the Tesla.**
