---
updated: 2026-09-19          # git-derived; bump when you edit this file
status: unreviewed     # current | drifted | superseded | unreviewed
---

# PARKGATE2PNW — `ces_events` stops logging a parked truck

Branch `parkgate2pnw` off `origin/3devpnw` `d0a6b08abc`, commit `0e4f446a7c`.
**Not pushed, not deployed.**

## The problem, measured

`/data/pnw/ces_events.jsonl` writes one breadcrumb per second for as long as the device is onroad,
and `IsOnroad` follows **ignition**, not motion (CLAUDE.md Rule 3). On the Lightning, parked +
charging = ignition on, indefinitely, in a parking space.

Six archived generations pulled from S3 — 42,918 records, ~120 MB raw:

| | records | share |
|---|---|---|
| stationary (`vEgo` < 0.5 m/s) | 40,203 | **93.7 %** |
| …of which `{"ev":"tick","mode":"experimental","reason":"stopLatch"}` | 39,983 | 93.2 % |
| driving | 2,715 | **6.3 %** |

The sample is 11.9 h of 1 Hz log containing 45 minutes of driving.

### What that does to retention

`CES_ARCHIVE_MAX_BYTES` = 2 GiB was justified in `ces_pnw.py` as *"~95 days at 21 MB/day"*. Both
halves are true and the conclusion is still wrong, because the 21 MB/day was measured on a **driving
day** (the 2026-08-26 Olympic Peninsula trip) while the archive's steady-state content is 93.7 %
parked. Recomputed from the sample's own bytes (driving records 3.15 KB, parked 2.84 KB):

| | before the gate | after |
|---|---|---|
| records in 2 GiB | 732,000 | ~1,090,000 |
| of which driving | 46,300 | ~500,000 |
| **driving hours retained** | **12.9 h** | **~140–150 h** |
| driving share of the archive | 6.3 % | 74–79 % |

(The "after" column sweeps `E`, the number of separate Park episodes per 12 h sample, because each
costs 30 debounce records: `E`=2 → 149 h, `E`=5 → 145 h, `E`=10 → 140 h, `E`=50 → 107 h. Reproduce
with `_scratch/parkgate/retention.py`.)

The doc's own framing, corrected: 2048 MB ÷ 21 MB/day = 98 calendar days of *bytes*, of which 6.3 %
is driving → **~6 days-equivalent of driving content**, not 95 days of anything useful.

**The structural win is bigger than the multiplier.** After the gate, parked content is capped at
~1 record/minute *however long the truck sits*. Retention stops depending on how much the truck is
parked — the one variable nobody controls. Before, a fortnight of driveway charging could evict a
corridor drive.

**Four places repeat the 95-day figure and should be corrected to the table above** (all left
untouched by this branch — Rule 5; two of them are workbench files outside this repo):

| file | line | what it says |
|---|---|---|
| `~/gh/comma/docs/CURVEDB2PNW.md` | 262 | "`CES_ARCHIVE_MAX_BYTES` = 2 GB ≈ 95 days at the measured 21 MB/day of heavy driving" |
| `~/gh/comma/docs/DEVICE-STATE.md` | 409 | same, in the `/data/pnw/ces_archive/` row |
| `selfdrive/controls/lib/ces_pnw/ces_pnw.py` | 89–90 | the constant's own justification comment (corrected on this branch) |
| `system/loggerd/uploader.py` | 58–59 | "budgeted at 2 GB (~95 days at the measured 21 MB/day)" |

Also `selfdrive/controls/lib/ces_pnw/tests/test_event_log_rotation.py:254` asserts weeks-of-retention
from the same 21 MB/day. The assertion still passes (the constant did not change) but it is measuring
calendar bytes, not driving content, and now understates what the archive is worth.

## The gate

`selfdrive/controls/lib/ces_pnw/park_tick_gate.py`. Suppresses the ~1 Hz breadcrumb **only** while
`carState.gearShifter == park`.

### Truth source: the live gear, compared as an enum

- **Not speed, not standstill, not `IsOnroad`.** A red-light stop is `vEgo` 0 with the shifter in
  **Drive** and keeps logging at full rate — the stop / lurch / green-light analyses live entirely
  in those records, and a speed gate would delete the log's most-used content.
- **Not the `GearPark` param.** Rule 3 says daemons that must not subscribe use the param; *control-
  path* code prefers the live `carState`. `experimental_request()` already receives it, so the param
  would add a file read per tick to fetch a fact already in hand. (Cost: the gate does not inherit
  `gearparkcan2pnw`'s quiet-CAN reasoning. It does not need it — see the interlock below.)
- **Enum comparison, never a string.** `str(x) == "GearShifter.park"` is *always* False and has
  already silently killed one feature here ([[capnp-enum-str-trap]]). Worse, the two forms differ:
  `str(car.CarState.GearShifter.park)` is `'1'` (the schema int) while `str(cs.gearShifter)` on a
  real message is `'park'` (a `_DynamicEnum`). `gear == GearShifter.park` is correct for both, and
  a mutant that swaps in the string form is killed by the test suite.

### Rule 2: it never goes dark and it never goes quiet

| | |
|---|---|
| **Fail open** | Absent carState, `None`/missing `gearShifter`, `unknown` (Tesla `DI_GEAR_SNA`/`INVALID`, Ford `Unknown_Position`), a non-enum gear, an unread kill switch — **all log**. Every ambiguity resolves toward keeping telemetry. |
| **A moving car always logs** | `PARK_RELEASE_V` = 0.5 m/s. CANParser zero-inits every signal and 0 decodes as Park on **74 platforms** in the pinned opendbc (`selfdrive/car/gear_park.py`), so a gear message that never arrived can read `park` for a whole drive. This is an **escape hatch, not the gate**: a finite speed above the threshold can only RELEASE suppression, never cause it. A non-finite or non-numeric reading is no evidence of motion and leaves the gear in charge. |
| **No silent gap** | While suppressing, one full, explicitly-marked record per `PARK_HEARTBEAT_S` (60 s) carries `parkGate:"hold"` and `parkSupp` = records suppressed so far in this hold. The first record after a hold carries `parkGate:"release"` with the total. A gap that reads as "the logger died" is itself a Rule 2 failure, so there is no gap. |
| **It announces itself** | `cloudlog.event("ces_park_gate", …)` on the rising and falling edge **only** — exactly two swaglog lines per Park episode, never per tick. (A per-tick log would be its own regression: this runs in selfdrived's 100 Hz loop.) |
| **It cannot reach control** | Both writers go through one wrapper, `CESController._park_decision()`. The gate is called *outside* `_steer_log_step`'s `except Exception: pass` (so a suppressed tick costs no `_read_map()` work), which would otherwise let a raise reach selfdrived's guard around `experimental_request()` — and **that guard forces CES to Chill**. A telemetry decision must not be able to change what the car does. The wrapper catches, returns "log the record", and says so once per 60 s. Found by the Fable review, not by the tests. |

### Hysteresis

- **Hold** after the gear has read Park continuously for `PARK_HOLD_S` = **30 s**.
- **Release** on the first tick that is not Park (or that trips the moving interlock).
- 30 s is parknorec2pnw's `PARK_HOLD_S`, chosen deliberately rather than independently: the two logs
  then agree about which windows were parked, so a `ces_events` hold and a missing route segment
  describe the same event. A park/unpark shuffle at a rest stop (Park stints under 30 s) keeps its
  surrounding driving context whole, which is the case the debounce exists for.
- Cost: the first 30 records of every Park stint, ~90 KB.

### What is NOT gated

Edge-triggered records — `adopt`, `greenLight`, `steerEvent`, `alert` (Take Control), `mads`, and
`card`'s `accDrop`/`pscmLim` — all still write unconditionally. They are rare (32 `adopt` against
10,568 `tick` on the 2026-07-13 log, 9 of them stationary), and a mode transition while parked is
itself worth having. Gating them would buy ~0.5 % and lose the park/unpark transition detail.

## The other half: the gear is now IN the record

`ces_events` carried **no gear and no park field at all**. That is precisely why 93.7 % parked
content went unnoticed for months: a red-light stop and a truck charging in a driveway were the same
row (`vEgo` 0, `mode` experimental). Every `tick` / `adopt` / `steer` record now carries:

| field | |
|---|---|
| `gear` | the bare enum name (`"park"`, `"drive"`, `"unknown"`, …), `null` when no carState has been seen |
| `park` | the **gate's verdict** — `gear == park` AND not moving. Deliberately not a restatement of `gear`: the two diverge exactly when the gear decode is wrong, which is the case worth being able to see in the log |
| `parkGate` | `"hold"` / `"release"`, present only on the marked records |
| `parkSupp` | records suppressed so far in this hold (cumulative; the release record closes the account) |

## Kill switch

**`RecordWhileParked`** — the existing parknorec2pnw param. No new key, therefore no `params_keys.h`
edit and no `params_pyx.so` rebuild on the car, which is the trap that new params carry.

Its scope widens from "route segments" to "route segments **and** the `ces_events` breadcrumb".
`PARKNOREC2PNW.md` explicitly listed `ces_events.jsonl` as out of scope; that line is updated rather
than left to quietly become false.

Read at ~1 Hz in `_read_params()`, **outside** the `if self._enabled:` block — the CES-off `steer`
breadcrumb is gated too, and runs precisely when CES is off. On a read failure the last known value
is kept and `cloudlog.exception` says so (throttled to once per 60 s with a failure count). Before
the first successful read the gate is **off**, so an unreadable switch can never cost telemetry.

## Failure modes (every one keeps logging)

| case | what happens | logged |
|---|---|---|
| car never decodes a gear (`unknown`) | never suppresses | — (nothing to say; the records carry `gear:"unknown"`) |
| gear message never arrives and decodes as `park` (74 platforms) | suppresses only while stationary; **releases the moment the car moves** | `ces_park_gate hold=False reason=unparked` |
| CAN fault spans Park → Drive | the first decoded drive gear, or any motion, releases on the next tick | `ces_park_gate hold=False` |
| carState absent / `_steer_log_step` called with a stub | `gear` is `None`, never Park | — |
| `RecordWhileParked` unreadable (`UnknownKeyName`, params mismatch) | gate holds its last value; on a first-ever failure that is **off** | `cloudlog.exception`, throttled |
| driver wants the parked breadcrumb back | `RecordWhileParked=1` over SSH, effective within ~1 s | `ces_park_gate hold=False reason=gateOff` |
| the gate itself raises | **not swallowed** — it sits outside `_steer_log_step`'s `except Exception: pass`, so selfdrived's own guard around `experimental_request()` catches and logs it. A gate that quietly stopped gating is the failure this whole file is about. |

### Known, bounded side effect

`_event_record` drains two accumulators: `CurvePeak.take()` (via `_curve_tele`) and the one-shot
`_gl_ev_pending` green-light marker. Suppressing ticks means neither drains for up to 60 s.

- `CurvePeak` keeps accumulating; the next heartbeat or release record drains the whole window and
  reports `kPeakN` ≈ 6,000 instead of ≈ 100. That is **honest, not wrong** — `kPeakN` exists exactly
  so a window that ran can be told from one that did not. The truck is stationary, so the curvature
  peaks it contributes are ~0. Only the single release record is affected; the next one starts clean.
- `_gl_ev_pending` can sit unconsumed for up to 60 s, but `_green_light_step` also writes its **own**
  dedicated `greenLight` record (ungated), so the event itself is never lost.
- On the **CES-off** path `_read_map()` is the only refresher of `_cur_lat/_cur_lon/_cur_bearing`,
  `_speed_limit` and the `sl*`/`lc*`/`vtsc*` fields, and during a hold it runs once per 60 s. The
  ungated `greenLight` / `steerEvent` / `alert` / `madsResume` records and `_curve_peak_step`'s
  disqualifier OR therefore see values up to 60 s stale **while parked**, where the GPS does not move
  and nothing is steering. Documented rather than fixed (Rule 5).
- **A hold ended by ignition-off or a process exit writes no `release` record and no `hold=False`
  swaglog line**, so the final ≤ 60 s of that Park stint go uncounted. Accepted: the next boot starts
  a fresh gate, and the `hold=True` line plus the heartbeats still mark the window.

**Recommendation for `tools/curvedb/`** (not implemented here — that is another effort's file): the
ingest should skip records carrying `parkGate`, or `park == true`. They are real records, but a
heartbeat aggregates up to 60 s of stationary accumulator into one row (`kPeakN` ≈ 6,000), which is
not a per-second sample like every other row.

## Tests

`selfdrive/controls/lib/ces_pnw/tests/test_parkgate2pnw.py` — **77 tests**, organised by what they
cost when they break: a stop is not a park · hysteresis / heartbeat / release · fail open · the kill
switch · it says so · the wiring · the measurement · **the production tick path** · the gate can
never reach control. Every gear value comes from a real `car.CarState` message, so the enum values
under test are genuine `_DynamicEnum`s.

**1,036 tests green** in `selfdrive/controls/lib/ces_pnw/tests` (959 before). Six existing test files
gained the new collaborator in their stubs; no assertion was weakened or removed.

**59 mutants, 59 killed, 0 survivors** (`_scratch/parkgate/mutate.py`). Every mutant is
anchor-checked for a unique match and `compile()`-checked before it counts, and **both** the feature
suite and the wider `ces_pnw` suite are baselined green before any mutant runs. Every one of the 59
is killed by `test_parkgate2pnw.py` alone — no kill depends on the wider suite.

> **This score is the second one. The first was wrong, and the way it was wrong is the point.**
> The harness originally ran pytest with a stripped `env={PYTHONPATH, PATH, HOME}`. In that
> environment the *unmutated* wider suite is red (8 failures + an xdist `INTERNALERROR`), and only
> the feature suite was baselined — so every mutant that survived the feature suite was automatically
> labelled "killed (wider suite)". **The harness structurally could not report a survivor**, and
> reported 50/50. That is exactly the CLAUDE.md Rule 2 pattern: *a uniform 100 % result is grounds to
> suspect the METHOD*. Re-scored honestly, the first version was **46/50 with 4 survivors**, and the
> survivors were all on the `_publish_status` tick branch — **the Lightning's production path**,
> which had no suppression coverage at all (the 39,983 `ev:tick … stopLatch` rows came from there,
> not from `_steer_log_step`). Fable caught it; the tests did not. Fixed by (a) inheriting the real
> environment and baselining both targets, (b) `TestTheProductionTickPath`, (c) collapsing the two
> call sites' duplicated argument surface into the single `_park_decision()` wrapper, which is what
> made the four surviving argument mutants killable at all.

Mutation coverage: the enum comparison including the str-trap form, `None`/`unknown`/foreign gears,
every branch of the moving interlock, the debounce, the heartbeat cadence and its opening record,
the suppressed counters, the release record, both cloudlog edges, `record_fields`, **both call
sites**, all four record fields on **both** record families, the gear sampling, the kill-switch read
and its failure path, the `_park_decision` wrapper's arguments, its fail-open return, its log and
its throttle, and a frozen clock at each call site.

### Replay against real archived logs

`_scratch/parkgate/replay.py` drives the **real** `ParkTickGate` over four archived `ces_events`
files. Those logs predate the gear field, so Park is *labelled* — conservatively, as a contiguous
stationary run of ≥ 5 min (longer than any red light, jam crawl or ferry queue), which makes the
result a **floor** on the saving rather than a ceiling:

```
file                                    recs  parked   kept   supp     MB    MB→    density
ces_events_2026-07-13_full.jsonl       10602    9421   1393   9209   10.4    1.4   11.1% → 84.8%
ces_events_full.jsonl (08-11)           3579    2174   1504   2075    5.0    2.1   39.3% → 93.4%
ces_events_1240-1330PT (09-17)          2764       0   2764      0    8.7    8.7  100.0% →100.0%
ces_events_2026-06-27.jsonl             8127    1148   7127   1000    3.2    2.8   85.9% → 97.9%
TOTAL                                  25072   12743  12788  12284   27.3   15.0   49.2% → 96.4%
```

The 09-17 file is a pure driving capture: **zero records suppressed**, which is the negative control.

## Fable review

Round 1 (2026-09-19) — **HOLD**, on two MAJORs and three MINORs, all applied:

| # | finding | what changed |
|---|---|---|
| 1 | MAJOR — the mutation harness could not report a survivor (stripped env ⇒ red wider baseline ⇒ free kills); the production `_publish_status` tick branch was uncovered | harness fixed (real env, both baselines); `TestTheProductionTickPath` + `TestTheGateCanNeverReachControl` added; `_park_decision` wrapper collapses the duplicated argument surface; score re-run honestly at 59/59 |
| 2 | MAJOR — the commit message claimed a `PARKNOREC2PNW.md` scope update that was not in the commit | the scope note is now actually in `PARKNOREC2PNW.md`; this doc, the `docs/pnw/README.md` row and the workbench `INDEX`/`PENDING-WORK` entries are in the commit |
| 3 | MINOR — `_steer_log_step`'s "any exception here is swallowed" was no longer true; a raise from the gate would reach selfdrived's guard and force CES to Chill | `CESController._park_decision()` wraps it: fail open, log once per 60 s |
| 4 | MINOR — `_read_map()`-refreshed fields go ≤ 60 s stale during a hold | documented above, not fixed (parked, so nothing moves) |
| 5 | MINOR — drain residue is honest but consumers should know | documented, plus the `tools/curvedb/` ingest recommendation above |
| 6 | NIT — the module docstring duplicated the commit message | trimmed to the four invariants |

Fable could not construct a case where a driven car is suppressed, and confirmed the six existing
test-file edits are additive only and that no submodule pin moved.

## On-car verification (after the owner approves a deploy)

1. **Park 60 s with the truck awake.** Expect `ces_park_gate hold=True gear=park` in swaglog once,
   then `tail -f /data/pnw/ces_events.jsonl` showing one `"parkGate":"hold"` record a minute and
   nothing else.
2. **Shift to Drive.** Expect a `"parkGate":"release"` record within ~1 s carrying `parkSupp`, then
   full 1 Hz logging.
3. **Stop at a red light in Drive.** Expect the 1 Hz breadcrumb to continue with `"gear":"drive"`,
   `"park":false` — the single most important thing this change must not break.
4. **Repeat step 1 on the Tesla** (the gate is car-agnostic; no fingerprint or capability branch).
5. `grep -c ces_park_gate` in swaglog after a day: should be 2 per Park episode, not thousands.
