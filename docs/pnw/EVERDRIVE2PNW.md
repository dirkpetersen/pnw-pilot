# EVERDRIVE2PNW.md - EverDrive auxiliary-charger info line

**Status: DEPLOYED AND VERIFIED ON THE TRUCK.** 3devpnw `506b70644d`, opendbc pin `15c52028`, installed
2026-11-02 about 10:30 PT. This file describes the CURRENT design (SoC-based load, generator state).
The first version used motor-2 power; it was dropped on 2026-11-02 (see section 4.1).

**Target:** Ford F-150 Lightning Flash 2025 only. Structurally inert on the Tesla Model S Raven: the
code lives in the Ford car port, which never loads on the Tesla. Display only: no control path, no
panda, no safety, no TX.

**Driver request (2026-09-19):** a one-line EverDrive info box below the CES logging box in the lower-right
corner of the display, showing what the auxiliary bed-port generator puts in and what it buys in range.

> **This is the branch copy.** Links that begin `~/gh/comma/` point into the multi-fork workbench and
> are outside this repository (chiefly the `CANbus/` library, which holds the measured CAN evidence).

## 1. What it displays

One line, right-aligned, bottom-anchored below the CES box. Font size 52, worst box 947 px, 63 px clear of
the screen centre (the green driving path). `->` and all text are ASCII (the font atlas has no U+2192).

**Moving (10 mph or more).** The form never changes while driving. No "@" and no "m/h".

```
generator on:     10.8kw,220m,1mi:215m
generator off:    132m(55.35kwh),1mi:128m
generator idle:   GEN 0kw!,?282m,1mi:?282m        (running, not supplying; no kwh term, for width)
```

- Load P: the measured `grossKw` (SoC window, section 3). Until the window fills (about 25 to 60 s of a
  drive), P is the speed estimate `effWhKm x km/h / 1000 + 1.2 kW` and every figure built from it gets a
  leading "?". The "?" goes away when `grossKw` is published. A figure without "?" is always measured.
- Term 1, generator on: dash range x P/(P-ac), for example dash 141 mi, 30 kW, 10.8 kW in gives `220m`.
  It needs no capacity.
- Term 1, generator off: `energyKwh / P x mph`. No "?" only for a measured P and a result in 1 to 499 mi.
  Regen, coasting, no reading or no energy: the speed estimate, or the dash range, both with "?".
- `>999m`: shown when the measured P is at or below the charge (the generator covers the load) or the result
  is over 999. With the ESTIMATE it is `?>999m`. It is never printed from a missing reading without "?".
- `1mi:` is the AVERAGE of term 1 over the last mile driven (owner 2026-11-02): term 1's formula on the
  producer's distance-weighted mile means of the load (`mileLoadKw`) and of the charge (`mileAcKw`).
  Generator on: dash x mileLoadKw / (mileLoadKw - mileAcKw), `>999m` when mileLoadKw <= mileAcKw.
  Generator off: `energyKwh / mileLoadKw x mph`, `1mi:--` when not usable. "?" when any of the mile used
  the speed estimate. Before the first full mile it repeats term 1. A steady load gives the same figure on
  both terms; a term 1 that moves between 220 and 240 gives about 230.
- `--m` only when the dash range is gone and nothing can stand in. "?" is never put on `--m` or `1mi:--`.

**Moving with the generator charging (everdrivetwoline2pnw, owner 2026-11-02): two lines.** Only while
moving, `genState` is `on` and `acKw` is above 0.05 kW (current flowing). Voltage with 0 kW, generator off,
idle, stopped and parked keep the one-line forms.

```
10.8kw,27kw,59kwh            generator kW (yellow), truck load P, pack energy (white)
227m,1mi:227m,d1m:187m       instant (SoC), last mile (SoC), last mile (dash)
```

- Load P: `grossKw`, else the speed estimate with "?". Rounded to whole kW; the kWh to whole kWh, and
  omitted when `energyKwh` is missing.
- Instant: `energyKwh / (P - acKw) x mph`.
- `1mi:`: `energyKwh / mileNetKwh`. `mileNetKwh` is the net energy per mile over the last mile driven, the
  time integral of (term-1 load - acKw), so at a steady speed and load it equals the instant range. "?"
  when any of the mile used the estimate. Before the first full mile it repeats the instant range.
- `d1m:`: the dash-based mile, the same figure as `1mi:` in the one-line charging form
  (dash x mileLoadKw / (mileLoadKw - mileAcKw); before the first mile dash x P / (P - acKw)).
- Any of the three over 999, or with a net load of 0 or less: a plain `>999m`, never `?>999m` (owner:
  "if any of them reaches >999 just use that number"). Below 999 the "?" rules are as for term 1.
- Width at font 52: worst line 2 `?>999m,1mi:?>999m,d1m:?>999m` 879 px box (131 px clear of the screen
  centre), line 1 647 px. Box height 178 px (113 px for one line); the CES box lifts by it, and by one
  more box during a banner.

**Stopped, parked or below 10 mph.**

```
charging, crawling:  1.4kw,100m(55.35kwh)->104m
charging, stopped:   1.4kw,112m(63.49kwh),+2.7m/h
not charging:        --,112m(63.49kwh)
generator idle:      GEN 0kw!,112m(63.49kwh)
```

The kWh term is omitted (never shown as 0) when capacity or SoC is missing.

**Generator state (colour and banner).** Yellow kW (255, 220, 0) while the generator is on, from the first
frame with voltage, so the figure can start at about 0.3 kW and ramp. Orange-red `GEN 0kw!` (255, 110, 40)
while idle (rules in section 5.1). Green was hard to read in daylight (owner 2026-11-02), and the old amber
was too close to the new yellow. A 5 s banner above the box marks each change between two known states:
`Generator ON 0.3 kW` (yellow, live kW), `Generator OFF` (white), `Generator: NO POWER` (orange-red). The
first state seen in a session, or after the box was hidden (stale payload, key absent, toggle), shows no
banner. A stale payload or one without `acSeen` sets no state, no colour and no banner. Display only: no
alert, no sound. Delay from the first frame with voltage: 0x2A7 is 1 Hz, the producer publishes at 5 Hz
and the UI polls at 5 Hz while the key is present (the module sends 0 V frames while the generator is
off, so the key is present), so the box and banner change within about 1.4 s.

The change of form is the signal. There is no legend and no second line.

**Layout rule (driver, 2026-09-19):** the box must stay out of the green driving path. It is
right-anchored and every term is digit-bounded, so it cannot grow toward the centre.

---

## 2. The evidence base: measured on this truck

Full working: [`../CANbus/ford/f-150/lightning/2024-25/ENERGY-RANGE-SIGNALS.md`](~/gh/comma/CANbus/ford/f-150/lightning/2024-25/ENERGY-RANGE-SIGNALS.md).
Primary capture: route `000001b8--a46fe398b3`, 2026-09-19 12:26–12:40 PT, 16 segments,
**7,240,844 CAN frames**, plus parked-charging routes `000001b6`/`000001b7`.

| Quantity | Signal | State |
|---|---|---|
| EverDrive AC input | `0x2A7` (**not in any DBC** - aftermarket module) | ✅ **verified** |
| Range (the dash number) | `0x442` `VehElRnge_L_Dsply`, 0.1 km/bit | ✅ **verified against the dash** |
| Reference efficiency | `0x36D` `VehElEffAvg_No_Dsply`, 10 Wh/km, **offset −100** | ✅ verified, constant 320 Wh/km |
| Pack SoC (raw) | `0x24C` `BattTracSoc2_Pc_Actl`, 0.01 %/bit | ✅ verified |

### 2.1 `0x2A7` - the EverDrive AC meter

1 Hz on openpilot **bus 0**. Bytes D0–D7, zero-indexed, big-endian:

```
AC current = (D0<<8 | D1) / 160   amperes
AC voltage = (D4<<8 | D5) /   8   volts
```

| State | Payload | Decode |
|---|---|---|
| L1 charging | `07 D0 00 00 03 68 00 00` | 12.50 A × 109.0 V = **1.363 kW** |
| Unplugged | `00 00 00 00 00 00 00 00` × **865 consecutive frames** | a **real** zero |

The all-zero capture is the clean negative control: it proves the meter is live and genuinely reads
zero when nothing is connected, so a zero here is a measurement rather than an absent message.
Cross-checked against the module's own UDS telemetry (`0x6D1`→`0x0FB2`) which read 1.320 kW at the
same time - the small delta is the module's documented `trunc(amps) × volts` truncation.

### 2.2 Range - verified against ground truth

`0x442 VehElRnge_L_Dsply` read **180.2 km = 112.0 mi** at the end of the drive; the driver reported
the dash showing **112 mi**. The other candidates are different quantities and must not be used:

| Signal | Reading | What it actually is |
|---|---|---|
| `0x352 VehElRngeNut_L_Dsply` | 173.5 mi | a second, more optimistic estimate |
| `0x471 RngPerChrgAvg_L_Dsply` | 246.9 mi | **full-charge** range, not current range |
| `0x366 RngPerChrgInst_L_Dsply` | 0 … 409.3 | ⛔ unusable - saturates at raw max under any regen (759 of 1547 samples) |

### 2.3 Charging was real - four independent confirmations

Recorded because "is it actually charging?" is the question this feature exists to answer, and
because the truck's own authorization chain says it is **not**:

1. The module's accumulator climbed **0.082 → 0.431 kWh** across the session.
2. Broadcast `0x2A7` read **1.363 kW**.
3. That meter read exactly zero for 865 frames once unplugged (negative control).
4. **Pack SoC rose monotonically**: 49.88 % (11:56 PT) → 49.96 % (12:15) → 49.99 % (12:26).

⚠️ **But only about a fifth of it reached the battery.** SoC rose **+0.11 points in 30 min** ≈
0.144 kWh at 131 kWh ≈ **0.29 kW net**, against **1.36 kW** going in. The truck's own awake load
(DC-DC to 12 V, electronics, BECM, thermal) consumed the rest. This is a parked-and-awake effect and
**does not** reduce EverDrive's contribution while driving, where that load is drawn regardless, so the 21 % ratio is not applied to the generator's contribution.

---

## 3. The maths

All display in miles. `v_mph = vMs x 2.23694`.

```
energyKwh = socPct x capKwh / 100                 # remaining pack energy; capKwh measured 125.96 kWh
netKw     = -(d energyKwh / dt) over a rolling window    # + = discharging
grossKw   = netKw + acKw                          # the truck load with the generator input added back
```

Why `grossKw`, not `netKw`: the SoC drop is already net of the charger. Using the net value in
`range x P/(P-ac)` would count the charger twice.

**Capacity.** UDS read 2026-09-20: `Energy` 59.350 kWh / `HvbSoc` 47.120 % = 125.96 kWh. The broadcast
`0x24C` SoC equals `HvbSoc` (the true SoC), so no UDS is needed at run time. `capKwh` is derived from
`RngPerChrgAvg x VehElEffAvg` (127.26 kWh, 1.0 % off). If the truck does not send `RngPerChrgAvg`, the
measured 125.96 kWh is used (`capSrc` = measured).

**The window** (`everdrive_pnw._gross_kw`; each constant is justified at its definition):

| | | Why |
|---|---|---|
| accumulate above | 10 mph | below it the truck draws accessories with no distance |
| SoC change floor | 0.15 % (`MIN_DSOC_PCT`) | 0.01 %/bit = 12.6 Wh per bit; 15 bits = +/-6.7 % worst case on the first figure. Below the floor the producer publishes `None`, never a provisional value |
| window | 25 s first, 60 s steady, 180 s max | first figure after 25 s if the floor clears; settles to 60 s; 180 s wall-clock cap so a long stop ages it out. Net drain under about 3.8 kW never clears the floor: the UI shows "?" (honest answer: too close to call) |
| plausibility | 250 kW (`NET_KW_MAX`, magnitude) | the BECM SoC is an estimate and can step; a step up or down is not a measurement |

Short slowdowns keep the history (only its age counts), so the first moving publish after a red light
already has a figure.

**Traps recorded so they are not reintroduced:**

1. The increment is `dSoC x capacity`, never `d(SoC x capacity)`. One bit of `VehElEffAvg` moves the derived
   capacity by about 4 kWh, which would show as about 100 kW of artefact inside the plausibility cap.
2. Shortening the window by discarding history is irreversible. Choose a newer baseline and keep the snapshots
   (replay of route `000001b8--a46fe398b3`: 10 on/off blocks of median 14 s became 5 blocks of median 72 s).
3. The baseline is the NEWEST one at least 60 s old that clears the floor, not the last one before the first
   that fails. Across a sign change (generator surplus, then a climb) the long window can net to under the
   floor while the last 60 s clears it (Gemini 2026-11-02). A last minute of pure regen is therefore
   reported as the negative it is; the UI reads a load <= 0 as no load.

**Last mile (everdrivemile2pnw, owner 2026-11-02).** The owner's rule: the last-mile figure is the average
of the first figure over the last mile. The producer averages term 1's inputs over the last 1 mile driven
above 10 mph, distance-weighted (distance from vEgo, only consecutive samples, so a stop or an unwatched
gap adds nothing): `mileLoadKw` = mean of the load term 1 uses (`grossKw`, else the speed estimate, which
also stands in for a `grossKw` <= 0 with the generator off, as in term 1) and `mileAcKw` = mean `acKw`.
`mileEst` is True while any of the mile used the estimate. The UI applies term 1's formula to the means.
Because `grossKw` is already a 60 s average, the mile mean does not step with the SoC.
Replaced 2026-11-02: the SoC-step-edge mile energy (`mileKwh`, `mileAcKwh`). It measured the SoC drop per
mile, a different quantity from term 1, and read 165 m beside a term 1 of 238 m on the road.

**Speed estimate.** `effWhKm x km/h / 1000 + P_ACC_KW`. `effWhKm` is the truck reference efficiency
(constant 320 Wh/km); `P_ACC_KW` = 1.2 kW is the accessory load (pack minus trip meter, 2026-09-19 drive).
The UI mirrors it as `_P_ACC_KW`. Check on 2026-11-02: 23.6 kW estimated against 22.3 kW measured at
51.5 mph (within 6 %).

### 3.1 Guards: each must fail visibly, never silently

| Condition | Behaviour |
|---|---|
| `acSeen` false | no box. Never print a 0.0 kW that came from a pre-filled CANParser signal |
| `acKw <= 0.05` | "not charging" form: a real measured zero |
| `effOk` false or `effWhKm <= 0` | no speed estimate and no gain rate. No default efficiency is substituted |
| `ts` older than 5 s while onroad | hide the box |
| result out of band | clamped or shown as "?"/`--`, with a comment saying what the clamp protects against |

---

## 4. Signals ruled out: the negative result, recorded in full

Checked over 13 minutes of **real driving**, not just parked. A table of zeros is the classic
artifact of asking the wrong question (Rule 2), so the counts are part of the finding:

| Candidate | Msg | Frames on bus 0 | Verdict |
|---|---|---|---|
| `BattTrac2_Pw_DchrgInst` / `ChrgInst` | `0x24D` | **0** | ⛔ never broadcasts at all - would have been ideal |
| `BattTrac_I_Actl` (pack current) | `0x07A` | 86,505 | ⛔ raw pinned at 0 for **every** frame = encoding floor (−750.000 A) = "not available" |
| `DteVeh_Eff_Actl` / `Pw_Actl` / `E_Actl` | `0x337` | 865 | ⛔ static constants all drive |
| `VehElEff_No_Avg` | `0x480` | 865 | ⛔ raw pinned at 0 (floor) |
| `ConsAvgTrip_Fe_Dsply` | `0x317` | 858 | ⛔ raw pinned at 0 - ICE fuel-economy channel, dead on an EV |
| `EdmCurrent_Fe_Dsply` / `EdmPrev` | `0x44A` | 1,730 | ⛔ same |
| `Inv1Ain_I_ActlMntr` (inverter 1) | `0x442` | 865 | ⛔ constant raw 10000 = 0.0 A (floor) |

### 4.1 Why motor 2 was dropped (2026-11-02)

The Lightning is dual-motor and the DBC names only one motor current (`MtrTrac2`, 0x441). The first
version (`everdriveinstant2pnw`, 2026-10-31) computed `P = k x MtrTrac2_I x MtrTrac2_U + 1.2 kW` with a
self-calibrating k. Earlier measurement (2026-09-19) already showed motor 2 gives only 62 % of wheel
energy. The 2026-11-02 drive disproved it for the highway:

- 5 min at about 53 mph with 10.8 kW in: the motor-2 power had a median of 15.4 kW and was below the
  10.8 kW charge 43 % of the time at more than 50 mph.
- The SoC-based load was 18 to 41 kW (median 25); the 5-minute average was 29 kW (18.3 net + 10.8 AC).
  True range 228 to 277 mi.
- Another 122 s at 52 to 54 mph: SoC 63.99 to 63.68 % = 11.6 kW net, 22.3 kW gross; motor-2 median 0.8 kW.

At cruise the other motor carries the load and the bus does not report it. Motor-2 power is not a proxy for
pack power. The motor-2 path, the k calibration and `/data/pnw/everdrive_cal.json` were removed. A stale
`everdrive_cal.json` on the device is harmless: nothing reads it. The payload keys `pNowKw`, `kCal` and
`kFits` are gone.

---

## 5. Architecture

```
0x2A7, 0x442, 0x36D, 0x24C, 0x471 on bus 0      (0x441 is no longer read)
   -> opendbc Ford CANParser
   -> opendbc/car/ford/everdrive_pnw.py          <- all logic lives here
   -> Params("/dev/shm/params").put_nonblocking("EverDriveStatus", {...})  @ 5 Hz
   -> selfdrive/ui/onroad/everdrive_status.py    <- EverDriveStatusRenderer(Widget)
   -> augmented_road_view.py                     <- drawn after ces_status_renderer
```

This follows the fork's `FordLatStatus` precedent. No process subscribes to the raw `can` socket.

### 5.0 The hazard this feature nearly shipped: read before touching the parser

**A display box came within one design decision of making the truck undriveable.** Recorded in full
because the mistake is invisible, the symptom is catastrophic, and the next person to add a signal
here will be tempted by exactly the same shortcut.

The Ford `CANParser` is constructed with an **empty** message list, and `VLDict.__getitem__` lazily
registers a message the first time you index `cp.vl["SomeMessage"]`. That looks like "just index it,
no registration needed". It is not, because lazy registration passes `freq=None`, and
`opendbc/can/parser.py:179` reads:

```python
ignore_alive = freq is not None and math.isnan(freq)
```

`freq=None` ⇒ `ignore_alive` **False** ⇒ the message is alive-checked, with `timeout_threshold`
falling back to the "assume 1 Hz" branch = **10 s**. So on any Ford that does not transmit that
message - a Lightning with no EverDrive fitted, or any non-EV Ford for the three HEV messages,
`can_valid` goes False, which becomes `ret.canValid` False, which makes **the car undriveable**.

Measured 2026-09-19, and re-confirmed by the Fable review through the **real Lightning
`CarInterface`**: lazy-registered + never received ⇒ `can_valid` False **from the very first update**
(`can_invalid_cnt` initialises at `CAN_INVALID_CNT` and `MessageState.valid()` is False on an empty
deque - it is not a 15 s grace period, it is immediate); `float("nan")`-registered + never received ⇒
`can_valid` True throughout 20 s at 100 Hz, all four `ignore_alive` True, no address lazily added.

**The fix, and the rule:** all the messages are registered explicitly through the `float("nan")`
frequency probe already in `carstate.get_can_parsers` - the same mechanism `GPS_MSGS` and `PPO_MSGS`
use, and for this exact reason. `nan` ⇒ `ignore_alive` True ⇒ exempt from the alive check.
**Any signal added to this feature must go through that probe.** `T1` in the test suite is the
regression guard; it asserts both directions, so it fails if the probe is removed.

Related ordering invariant, marked DO-NOT-REORDER in the code: `cp.ts_nanos` is a **plain dict**
(`parser.py:136`) and raises `KeyError` for an unregistered message, whereas `cp.vl` is the `VLDict`
that silently arms the hazard. The producer therefore touches `ts_nanos` **first**, so a message the
DBC probe skipped fails loudly instead of quietly registering itself alive-checked.
### 5.1 Data contract

Mem param `EverDriveStatus`, registered `{CLEAR_ON_MANAGER_START, JSON}`, published at 5 Hz.

| Key | Meaning |
|---|---|
| `ts` | `time.time()` heartbeat for staleness |
| `acKw`, `acV`, `acSeen` | generator AC power, AC voltage (V), and True only once `0x2A7` was received this session |
| `genState` | `off`, `on` or `idle` (below) |
| `rangeKm`, `effWhKm`, `effOk` | dash range, reference efficiency, False at its encoding floor |
| `socPct`, `capKwh`, `capSrc`, `energyKwh` | SoC, usable capacity, its source (derived or measured), remaining energy (None if an input is None) |
| `grossKw` | load from the SoC window; None until the window clears its floor, and while below 10 mph |
| `moving`, `vMs` | producer moving flag (10 mph or more), speed in m/s |
| `mileNetKwh` | net energy per mile over the last mile, kWh/mi: time integral of (term-1 load - acKw) scaled to one mile; the two-line `1mi:` |
| `mileLoadKw`, `mileAcKw`, `mileEst` | distance-weighted mean of term 1's load and of `acKw` over the last mile (kW); `mileEst` True if any of the mile used the speed estimate. None before the first full mile |

Removed 2026-11-02: `pNowKw`, `kCal`, `kFits`, `mileKwh`, `mileAcKwh`. Added: `acV`, `genState`,
`mileLoadKw`, `mileAcKw`, `mileNetKwh`.

**Generator state** (from `0x2A7`, measured 2026-11-02 09:47 to 09:52 PT): off = 0 V and 0 A. On = 234 V at
once with 1.2 to 3.2 A for about 5 s, then the current ramps about 0.9 A/s to about 46 A (10.8 kW) in
about 50 s. States (owner 2026-11-02, "when I turn on the generator it does not show immediately"):
- `on` as soon as the voltage is present (100 V or more), at the first frame, whatever the power.
- `idle` only when the voltage has been present for more than 60 s without ever reaching 0.5 kW (longer
  than the measured ramp), or when it reached 0.5 kW and then stayed below it for more than 10 s.
- `off` at once when the voltage is gone (below 100 V).
A gap in `0x2A7` (not live) resets the timers, and an out-of-band frame restarts the low-power timer, so
time without a valid reading never counts toward `idle`. Every change is logged once with U, I and kW.

### 5.2 Why `acSeen` exists (Rule 2)

CANParser **pre-fills every signal to 0.0 before the message is ever received.** Without `acSeen`, a
truck with no EverDrive fitted would publish `acKw: 0.0` *indistinguishably from a connected but idle
charger*. Three states must stay distinguishable and the code must never collapse them:

| State | Contract |
|---|---|
| no EverDrive fitted | **key absent entirely** |
| fitted, charger unplugged | key present, `acSeen: true`, `acKw: 0.0` |
| fitted, charging | key present, `acSeen: true`, `acKw > 0` |

### 5.3 Zero cost when absent (driver requirement, 2026-09-19)

> *"if no everdrive is detected, e.g. on the tesla nothing should be displayed and no compute cycles
> should be spent on this"*

- **Tesla:** structurally zero - the Ford car port never loads. Nothing runs at import time or from a
  shared/global registry.
- **Lightning, no module:** the publisher writes **nothing at all** - not even `acSeen: false`. An
  absent key is the cheapest possible signal. Once absence is established it stops building the
  payload, stops calling `time.time()`, stops touching `Params`.
- **UI:** a missing key means hide *and* back the poll off from 0.2 s to a couple of seconds. `_render`
  returns before any string formatting or text measurement. Exemplar measurement happens lazily on
  first actual display, not in `__init__` and not per poll.
- **Not latched.** Absence is re-checked slowly, so plugging the charger in mid-drive still brings the
  line up. A permanent give-up would be its own silent failure.
---

## 6. Files touched

**`pnw/pnw-opendbc` (`master-pnw`):** `opendbc/dbc/ford_lincoln_base_pt.dbc` (message `679` = `0x2A7`, aftermarket),
`opendbc/car/ford/everdrive_pnw.py` (all logic), `opendbc/car/ford/carstate.py` (wiring only).

**`pnw/pnw-pilot` (`3devpnw`):** `common/params_keys.h` (`EverDriveStatus` mem, `DisableEverDrive` persistent bool),
`selfdrive/ui/onroad/everdrive_status.py` (widget), `augmented_road_view.py` (construct + draw),
`ces_status.py` (`_bottom_offset` only: the CES box lifts by the EverDrive box and banner height, 0 when hidden),
`selfdrive/ui/layouts/settings/toggles.py` (toggle row).

### 6.1 Toggle polarity: a deliberate deviation, flagged

New feature toggles default OFF. This one ships shown by default as `DisableEverDrive` (opt-out): it is a
display overlay and does not change what the car does; `DisableLocationServices` and `DisableLaneCentering`
use the same shape.

---

## 7. Open items

1. Verify range and generator indicator on the next drive: expected first number with the generator on at
   about 53 mph is 230 to 300 mi; yellow kW and `Generator ON` banner within about 1.4 s of generator start;
   orange-red `GEN 0kw!` only after 60 s of voltage without power, or 10 s after power stops; `1mi:` close
   to the average of term 1 over the last mile. Tracked in the workbench `WORK-PENDING.md`.
2. Ship `506b70644d` went out without the Gemini pre-push review and without the Rule 9 channel-tip run
   (owner approved the skipped review for that one push). Post-push review and tip run are tracked in
   `WORK-PENDING.md`.
3. The SoC step size on the truck is unknown (63.99 to 63.68 % fits 0.01 % steps).
4. `VehElEffAvg_No_Dsply` read raw 42 (320 Wh/km) in every capture; "slowly moving average" is unverified.
5. Raw to displayed SoC is a one-point fit (48.97 % raw, 51 % dash). Do not build on it.

---

## 8. See also

- [`../CANbus/ford/f-150/lightning/2024-25/ENERGY-RANGE-SIGNALS.md`](~/gh/comma/CANbus/ford/f-150/lightning/2024-25/ENERGY-RANGE-SIGNALS.md): measured evidence for every number here
- [`../CANbus/EverDrive/ANALYSIS_RESULTS.md`](~/gh/comma/CANbus/EverDrive/ANALYSIS_RESULTS.md): the EverDrive sub-project
- `CHANGELOG-2026-10-31.md` (this directory, if present) and the workbench `CHANGELOG-2026-11-02.md`
