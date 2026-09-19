# EVERDRIVE2PNW.md — EverDrive auxiliary-charger info line

**Status: REVIEWED, NOT YET PUSHED, NOT DEPLOYED.**
Branches `everdrive2pnw` in `pnw/pnw-opendbc` (based on `origin/master-pnw` `7ff5d541`) and in
`pnw/pnw-pilot` (based on `origin/3devpnw`). Nothing is on the car. This document is updated
**in place** as the change moves; anything that diverged from the original plan is flagged inline.

**Fable review 2026-09-19 — verdict: SAFE TO PUSH to `3devpnw`. No severe finding.** The reviewer
confirmed the one catastrophic mechanism (a display box taking `canValid` False) end-to-end through
the real Lightning `CarInterface` rather than by reading, and separately raised then **retracted** a
width concern after checking the generated font atlas. Its low/informational findings are all
applied: the liveness block moved inside the `try` (§5.0), a vacuous assertion removed, the toggle's
driver-facing description corrected to the shipped format, and three doc inaccuracies fixed — see
§5.4 for the one place where this doc previously claimed more verification than the tests provide.

**Tests:** 49 opendbc + 24 UI. **Mutation-tested: 38 applied, 38 caught, 0 survivors** — reached
only after two survivors were found and killed (M6b, then M10f, then M11 for the review fix). Suites:
`opendbc/car/ford/tests` + `opendbc/can/tests` 323 passed / 13 skipped / 583 subtests;
`opendbc/car/tests` unchanged at its pre-existing 11 failed / 764 passed.

**Target:** Ford F-150 Lightning Flash 2025 only. Structurally inert on the Tesla Model S Raven —
the code lives in the Ford car port, which never loads on the Tesla. Display-only: no control path,
no panda, no safety, no TX.

**Driver request (2026-09-19):** a one-line EverDrive info box below the CES logging box in the
lower-right corner of the comma display, showing what the auxiliary bed-port charger is putting in
and what it buys in range.

> **Canonical-home note.** Per the 2026-07-18 docs reorganization, `*2pnw` feature docs normally live
> on the branch at `pnw/pnw-pilot/docs/pnw/`. This copy sits in `~/gh/comma/docs/` because the driver
> asked for it here; **mirror it onto the branch in the same commit as the code** so the branch
> carries its own explanation like every other effort.

---

> **This is the branch copy.** The workbench copy lives at `~/gh/comma/docs/EVERDRIVE2PNW.md`;
> keep them in step. Links below that begin `~/gh/comma/` point into the multi-fork workbench
> and are **outside this repository** — chiefly the `CANbus/` reference library, which holds the
> measured CAN evidence every number here rests on.

## 1. What it displays

Four states, all right-aligned and bottom-anchored in the lower-right corner:

```
charging, moving:    ED:1.4kw,112mi->117mi
charging, stopped:   ED:1.4kw,112mi,+2.7mi/h
not charging:        ED:--,112mi
no EverDrive fitted: (nothing at all)
```

**Format decision (driver, 2026-09-19).** The roomy form (`ED: 1.4 kW   112 -> 117 mi`) does not fit
the width budget at the CES box's own font size — measured against the real `Inter-Medium` metrics at
the device's `FONT_SCALE = 1.16`, it is 709.5 px at `_FS = 44` and 769.6 px at 48. The choice was
*roomy text at a smaller font* vs *compact text at the matching font*; the driver picked **compact at
`_FS = 48`**, so the box reads at the same size as the CES box stacked directly above it rather than
noticeably smaller. Both options cleared the driving path; this was legibility, not layout.

`->` is ASCII deliberately. The device font atlas is built by `selfdrive/assets/fonts/process.py`
from `chr(32..126)` plus a short `EXTRA_CHARS` list that does **not** contain U+2192, so `→` would
render as tofu. `ces_status.py` makes the same call (it writes `ICBM 75>62`).

**Hard layout constraint (driver, 2026-09-19):** the box must stay out of the green driving path
drawn down the middle of the screen. It inherits the rule the CES box already states in its own
docstring — *"the box must never grow into the road path while moving."* Right-aligned, anchored to
the right edge, sized from **fixed exemplar strings** rather than live values so it cannot dance
left/right as digits change (the same technique `ces_status.py` uses, and for the same reason).

---

## 2. The evidence base — everything below was measured on this truck

Full working: [`../CANbus/ford/f-150/lightning/2024-25/ENERGY-RANGE-SIGNALS.md`](~/gh/comma/CANbus/ford/f-150/lightning/2024-25/ENERGY-RANGE-SIGNALS.md).
Primary capture: route `000001b8--a46fe398b3`, 2026-09-19 12:26–12:40 PT, 16 segments,
**7,240,844 CAN frames**, plus parked-charging routes `000001b6`/`000001b7`.

| Quantity | Signal | State |
|---|---|---|
| EverDrive AC input | `0x2A7` (**not in any DBC** — aftermarket module) | ✅ **verified** |
| Range (the dash number) | `0x442` `VehElRnge_L_Dsply`, 0.1 km/bit | ✅ **verified against the dash** |
| Reference efficiency | `0x36D` `VehElEffAvg_No_Dsply`, 10 Wh/km, **offset −100** | ✅ verified, constant 320 Wh/km |
| Pack SoC (raw) | `0x24C` `BattTracSoc2_Pc_Actl`, 0.01 %/bit | ✅ verified |

### 2.1 `0x2A7` — the EverDrive AC meter

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
same time — the small delta is the module's documented `trunc(amps) × volts` truncation.

### 2.2 Range — verified against ground truth

`0x442 VehElRnge_L_Dsply` read **180.2 km = 112.0 mi** at the end of the drive; the driver reported
the dash showing **112 mi**. The other candidates are different quantities and must not be used:

| Signal | Reading | What it actually is |
|---|---|---|
| `0x352 VehElRngeNut_L_Dsply` | 173.5 mi | a second, more optimistic estimate |
| `0x471 RngPerChrgAvg_L_Dsply` | 246.9 mi | **full-charge** range, not current range |
| `0x366 RngPerChrgInst_L_Dsply` | 0 … 409.3 | ⛔ unusable — saturates at raw max under any regen (759 of 1547 samples) |

### 2.3 Charging was real — four independent confirmations

Recorded because "is it actually charging?" is the question this feature exists to answer, and
because the truck's own authorization chain says it is **not**:

1. The module's accumulator climbed **0.082 → 0.431 kWh** across the session.
2. Broadcast `0x2A7` read **1.363 kW**.
3. That meter read exactly zero for 865 frames once unplugged (negative control).
4. **Pack SoC rose monotonically**: 49.88 % (11:56 PT) → 49.96 % (12:15) → 49.99 % (12:26).

⚠️ **But only about a fifth of it reached the battery.** SoC rose **+0.11 points in 30 min** ≈
0.144 kWh at 131 kWh ≈ **0.29 kW net**, against **1.36 kW** going in. The truck's own awake load
(DC-DC to 12 V, electronics, BECM, thermal) consumed the rest. This is a parked-and-awake effect and
**does not** reduce EverDrive's contribution while driving, where that load is drawn regardless —
see §3.2, which is the reason the displayed gain rate is *not* scaled by this ratio.

---

## 3. The maths

All display in miles. `rangeMi = rangeKm / 1.60934`, `effWhMi = effWhKm × 1.60934` (≈515 Wh/mi here).

```
v_mph      = vMs × 2.23694
P_assumed  = effWhMi × v_mph / 1000                      # kW the truck's own range model assumes
rangeProj  = rangeMi × P_assumed / (P_assumed − acKw)     # exact
gainMiPerH = acKw / (effWhMi / 1000)                      # EverDrive's contribution per hour
```

### 3.1 Why it uses the truck's own efficiency instead of measured consumption

**There is no usable broadcast source for battery power on this truck** (§4). Rather than ship an
uncalibrated estimate, the projection uses `VehElEffAvg_No_Dsply` — the same reference efficiency the
truck's own range estimate is built on. Two consequences, both good:

- The projected figure stays **internally consistent with the range printed beside it**. A driver
  comparing `112` and `117` is comparing two numbers from the same model.
- It is **conservative**. That constant is ≈515 Wh/mi; the driver actually achieved 263 Wh/mi
  (3.8 mi/kWh) on the measured drive. A pessimistic efficiency makes the projected gain *smaller*,
  which is the safe direction for a number that could otherwise flatter the hardware.

This also unblocked the feature: the alternative — calibrating motor power against the trip meter —
needs a steady-highway run that has not happened yet (§7).

### 3.2 Why the gain rate is NOT scaled by the 21 % net-to-pack ratio

**Corrected during design, recorded so it is not "fixed" back.** The first instinct was to scale
`gainMiPerH` by the measured 0.29/1.36 ≈ 21 % net-to-pack ratio from §2.3. That is wrong. The truck's
~1.0 kW awake load is drawn whether or not EverDrive is connected, so EverDrive's full input
genuinely offsets it. Its **contribution** to range is `1.363 kW ÷ 0.515 kWh/mi ≈ 2.7 mi/h`. The 21 %
figure explains why the *pack SoC* barely moved; it does not mean the contribution is smaller.

### 3.3 Guards — each must fail visibly, never silently

| Condition | Behaviour |
|---|---|
| `acSeen` false | "not charging" form. **Never print a 0.0 kW that came from a pre-filled CANParser signal.** |
| `acKw ≤ 0.05` | "not charging" form — a real measured zero, charger unplugged |
| `effOk` false, or `effWhKm ≤ 0` | show kW and range, **no** projection and **no** gain rate. Do **not** substitute a default efficiency |
| `P_assumed ≤ acKw × 1.05` | crawling; the projection diverges, so use the stopped form. The speed threshold comes from this guard, not a magic number |
| `ts` older than ~5 s while onroad | hide the box (same `_STALE_S` + grace-period pattern as `ces_status.py`) |
| projected range absurd | clamped, with a comment saying what the clamp protects against |

---

## 4. Signals ruled out — the negative result, recorded in full

Checked over 13 minutes of **real driving**, not just parked. A table of zeros is the classic
artifact of asking the wrong question (Rule 2), so the counts are part of the finding:

| Candidate | Msg | Frames on bus 0 | Verdict |
|---|---|---|---|
| `BattTrac2_Pw_DchrgInst` / `ChrgInst` | `0x24D` | **0** | ⛔ never broadcasts at all — would have been ideal |
| `BattTrac_I_Actl` (pack current) | `0x07A` | 86,505 | ⛔ raw pinned at 0 for **every** frame = encoding floor (−750.000 A) = "not available" |
| `DteVeh_Eff_Actl` / `Pw_Actl` / `E_Actl` | `0x337` | 865 | ⛔ static constants all drive |
| `VehElEff_No_Avg` | `0x480` | 865 | ⛔ raw pinned at 0 (floor) |
| `ConsAvgTrip_Fe_Dsply` | `0x317` | 858 | ⛔ raw pinned at 0 — ICE fuel-economy channel, dead on an EV |
| `EdmCurrent_Fe_Dsply` / `EdmPrev` | `0x44A` | 1,730 | ⛔ same |
| `Inv1Ain_I_ActlMntr` (inverter 1) | `0x442` | 865 | ⛔ constant raw 10000 = 0.0 A (floor) |

### 4.1 The dual-motor trap ⚠️

The Lightning is dual-motor (one per axle) and **the DBC names only one motor's current**
(`MtrTrac2`). Integrating the measured drive:

| Path | Net energy |
|---|---|
| Wheels, both axles (`0x167 PrplWhlTot2_Tq_Actl` × mean wheel ω) | **0.963 kWh** |
| Motor 2 mechanical (`0x475` torque × `0x441` speed) | **0.593 kWh** |
| Motor 2 electrical (`0x441` I × U) | 0.760 kWh |
| Truck trip meter (3.8 mi ÷ 3.8 mi/kWh) | ≈**1.09 kWh** |

Motor 2 supplies **62 %** of wheel energy. Its own mech/elec ratio is 0.78 — a sane
motor+inverter+gearbox efficiency, which is what says the decode is sound rather than the ratio being
an artifact. **So `MtrTrac2_I × U` is not vehicle power.** Because the range projection *divides* by
consumption, under-reading it **overstates** the EverDrive benefit — the unsafe direction. This is
why §3.1's approach was chosen instead.

---

## 5. Architecture

Chosen by the driver from two options (the alternative was a new CAN-reading daemon):

```
0x2A7, 0x442, 0x36D, 0x24C on bus 0
   -> opendbc Ford CANParser (dynamic; cp.vl["Name"] lazily registers)
   -> opendbc/car/ford/everdrive_pnw.py          <- all logic lives here
   -> Params("/dev/shm/params").put_nonblocking("EverDriveStatus", {...})  @ 5 Hz
   -> selfdrive/ui/onroad/everdrive_status.py    <- EverDriveStatusRenderer(Widget)
   -> augmented_road_view.py                     <- drawn after ces_status_renderer
```

This mirrors the fork's existing precedent exactly: the Ford carcontroller already publishes
`FordLatStatus` to `/dev/shm/params` and `ces_status.py` already consumes it. No new pattern is
invented, and **no process subscribes to the raw `can` socket** — in this fork `card` is the only
CAN reader, and it stays that way.

### 5.0 🔴 The hazard this feature nearly shipped — READ BEFORE TOUCHING THE PARSER

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
message — a Lightning with no EverDrive fitted, or any non-EV Ford for the three HEV messages —
`can_valid` goes False, which becomes `ret.canValid` False, which makes **the car undriveable**.

Measured 2026-09-19, and re-confirmed by the Fable review through the **real Lightning
`CarInterface`**: lazy-registered + never received ⇒ `can_valid` False **from the very first update**
(`can_invalid_cnt` initialises at `CAN_INVALID_CNT` and `MessageState.valid()` is False on an empty
deque — it is not a 15 s grace period, it is immediate); `float("nan")`-registered + never received ⇒
`can_valid` True throughout 20 s at 100 Hz, all four `ignore_alive` True, no address lazily added.

**The fix, and the rule:** all four messages are registered explicitly through the `float("nan")`
frequency probe already in `carstate.get_can_parsers` — the same mechanism `GPS_MSGS` and `PPO_MSGS`
use, and for this exact reason. `nan` ⇒ `ignore_alive` True ⇒ exempt from the alive check.
**Any signal added to this feature must go through that probe.** `T1` in the test suite is the
regression guard; it asserts both directions, so it fails if the probe is removed.

Related ordering invariant, marked DO-NOT-REORDER in the code: `cp.ts_nanos` is a **plain dict**
(`parser.py:136`) and raises `KeyError` for an unregistered message, whereas `cp.vl` is the `VLDict`
that silently arms the hazard. The producer therefore touches `ts_nanos` **first**, so a message the
DBC probe skipped fails loudly instead of quietly registering itself alive-checked.

### 5.1 Data contract

Mem param `"EverDriveStatus"`, registered `{CLEAR_ON_MANAGER_START, JSON}`, published at 5 Hz:

| Key | Type | Meaning |
|---|---|---|
| `ts` | float | `time.time()` to 2 dp — wall-clock heartbeat for staleness |
| `acKw` | float | EverDrive AC input power, kW |
| `acSeen` | bool | **True only if `0x2A7` has actually been received this session** |
| `rangeKm` | float | `VehElRnge_L_Dsply` |
| `effWhKm` | float | `VehElEffAvg_No_Dsply` |
| `effOk` | bool | False when `effWhKm` sits at its −100 encoding floor |
| `socPct` | float | `BattTracSoc2_Pc_Actl` |
| `vMs` | float | vehicle speed, m/s |

### 5.2 Why `acSeen` exists — the Rule 2 crux

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

- **Tesla:** structurally zero — the Ford car port never loads. Nothing runs at import time or from a
  shared/global registry.
- **Lightning, no module:** the publisher writes **nothing at all** — not even `acSeen: false`. An
  absent key is the cheapest possible signal. Once absence is established it stops building the
  payload, stops calling `time.time()`, stops touching `Params`.
- **UI:** a missing key means hide *and* back the poll off from 0.2 s to a couple of seconds. `_render`
  returns before any string formatting or text measurement. Exemplar measurement happens lazily on
  first actual display, not in `__init__` and not per poll.
- **Not latched.** Absence is re-checked slowly, so plugging the charger in mid-drive still brings the
  line up. A permanent give-up would be its own silent failure.

### 5.4 Stacking below the CES box

`CesStatusRenderer` is bottom-anchored (`by = rect.y + rect.height - box_h - _MARGIN`) in **both**
`_render` and `_render_card`, and its height varies by mode (2 lines moving, up to 5 with an event, a
tall grouped card at standstill). It gains a `_bottom_offset` attribute defaulting to `0`, subtracted
in those two places, which `augmented_road_view` sets each frame from the EverDrive renderer's current
height (0 when hidden). **Regression risk on an in-use overlay:** when EverDrive is hidden the CES box
must render exactly where it does today.

⚠️ **What is and is not verified here (corrected after the Fable review).** The hidden case is safe by
inspection — the offset is `0.0` and `x - 0.0 == x` is bit-identical for every float, which was checked
over 125 values including ±0.0, denormals and ±inf. But the headless UI tests **cannot reach raylib's
draw calls**, so deleting the lift or the wiring altogether still passes all 24 of them. The *shown*
case is therefore unpinned by tests and is cosmetic-on-device only; it is covered by the post-deploy
visual check in §7, not by CI. Do not read the mutation table as covering it.

---

## 6. Files touched

**`pnw/pnw-opendbc`, branch `everdrive2pnw`:**
- `opendbc/dbc/ford_lincoln_base_pt.dbc` — new message `679` (`0x2A7`), flagged as aftermarket
- `opendbc/car/ford/everdrive_pnw.py` — **new**, all logic
- `opendbc/car/ford/carstate.py` — minimal wiring only: import, one construct, one call, plus
  `ENERGY_MSGS` folded into the existing `float("nan")` DBC probe. **`carcontroller.py` is NOT
  touched** (an earlier draft of this doc said it was; the diff never did).

**`pnw/pnw-pilot`, branch `everdrive2pnw`:**
- `common/params_keys.h` — `EverDriveStatus` (mem) + `DisableEverDrive` (persistent bool)
- `selfdrive/ui/onroad/everdrive_status.py` — **new** widget
- `selfdrive/ui/onroad/augmented_road_view.py` — construct + draw
- `selfdrive/ui/onroad/ces_status.py` — `_bottom_offset` only
- `selfdrive/ui/layouts/settings/toggles.py` — the toggle row

Every hunk tagged `# everdrive2pnw:`.

### 6.1 Toggle polarity — a deliberate deviation, flagged

`CLAUDE.md` says new feature toggles default **OFF**. This one ships **shown by default**, as
`DisableEverDrive` (opt-out). Rationale: that rule governs toggles that change what the car *does*;
this is a display overlay, and the closest sibling — `DisableLocationServices`, also a lower-corner
onroad info line — uses exactly this opt-out shape. `DisableLaneCentering` is the same pattern
(`LANECENTER2PNW.md` §8.1 flags its own deviation the same way). Recorded here so it is a decision,
not an oversight.

---

## 7. Open / unverified

1. **Not reviewed, not pushed, not deployed.** The full gate sequence for this change:
   1. Opus verification of the code (Rule 8 — Opus must read and verify what the agents wrote).
   2. **Mutation-test the new logic** (`CODING-POLICY.md` §5: a passing test proves nothing until the
      fix is reverted and the test fails). The guards in §3.3 are the ones that matter — especially
      `acSeen`, which is the difference between "no module fitted" and "0.0 kW".
   3. **Fable review** — mandatory before `git push`; only `.md`/comment/log-string changes are
      exempt and this is not one. Give it the measured evidence, not just the diff.
   4. Push `pnw-opendbc` **`master-pnw` first** (a pin not reachable from the pushed branch makes the
      device's `git submodule update --init` fail mid-update), then the `3devpnw` pin bump via
      `git -C <wt> update-index --cacheinfo 160000,<40-char-sha>,opendbc_repo` — **never** `cd` into
      `opendbc_repo`, and never `git add opendbc_repo` after an `update-index` pin (it silently
      clobbers it).
   5. **Rule 9 — `scripts/check-channel-tip.sh` immediately after the push**, against the channel tip
      rather than the branch. Required here, not optional: this push touches `params_keys.h`, a
      submodule pin *and* `*.py`. Watch `_check_params_so.py` in particular — a `params_pyx.so` that
      predates the two new keys raises `UnknownKeyName` deep in feature code and surfaces as
      *unrelated* tests failing (it manufactured 31 phantom failures on 2026-09-19).
   6. Reboot — **code changes need a device reboot** (the manager preimports every process module at
      boot; an ignition cycle resumes with boot-time code). Gate on a live
      `carState.gearShifter == park`, **not** `IsOnroad`, which reads 1 on this truck whenever the
      ignition line is up — including parked and charging indefinitely.
   7. `SkipWideCameraUpload=1` before the deploy, back to `0` only once health is verified.
2. **The efficiency constant is a single observation.** `VehElEffAvg_No_Dsply` read raw 42
   (320 Wh/km) for the entire drive and never moved. It behaves like a reference constant, but it has
   not been seen to change, so "it is a slowly-moving average" is **unverified**. The `effOk` guard
   handles the floor case; it does not prove the value is right.
3. **Measured consumption is still uncalibrated** — see §4. Closing it needs 5–10 min at steady
   highway speed with trip 1 reset, which would let `k = P_battery / (MtrTrac2_I × U)` fall out. Not
   required for this feature, but it would allow a second, measured projection later.
4. **Raw↔displayed SoC is a one-point fit** (48.97 % raw ↔ 51 % dash, ratio 0.96). Not used by this
   feature. Do not build on it without a second pair at a different SoC.
5. **On-device verification order after deploy:** unplugged first (range-only form, and confirm the
   CES box has not moved), then plugged in (charging form).

---

## 8. See also

- [`../CANbus/ford/f-150/lightning/2024-25/ENERGY-RANGE-SIGNALS.md`](~/gh/comma/CANbus/ford/f-150/lightning/2024-25/ENERGY-RANGE-SIGNALS.md) — ⭐ the measured evidence for every number here, plus `tools/decode-drive-energy.py`
- [`../CANbus/EverDrive/ANALYSIS_RESULTS.md`](~/gh/comma/CANbus/EverDrive/ANALYSIS_RESULTS.md) — the EverDrive sub-project; the `0x167 = VehicleOperatingModes` decode and why `PlgActvArb_B_Actl` = 0 during a live bed-port session
- [`../CANbus/PENDING-WORK.md`](~/gh/comma/CANbus/PENDING-WORK.md) §2.7 — the open consumption item
- [`CODING-POLICY.md`](~/gh/comma/docs/CODING-POLICY.md) — Fable-only review, Opus designs/implements
- `~/.claude/skills/pnw-pilot-deploy` — ship sequence: companion repo `master-pnw` first, then the
  `3devpnw` pin bump; code changes need a **device reboot**, gated on `gearShifter == park`
