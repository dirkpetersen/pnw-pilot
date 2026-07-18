# ONROAD-CHARGING.md — the "parked & charging counts as onroad" problem

**Status:** 🆕 design investigation (2026-07-13). **No code changed, nothing deployed.** This doc
traces exactly how openpilot decides onroad/offroad, why a *stopped, charging* EV wrongly reads as
onroad, what signals can tell "parked & charging" from "driving," and 3 ranked fix options with a
recommendation. Production tree: `pnw/pnw-pilot` @ `3devpnw`; companion `pnw/pnw-opendbc` @
`master-pnw` (submodule → `opendbc_repo/`).

---

## 1. The problem in one paragraph

openpilot equates **ignition == onroad**. On both EVs the ignition line stays energized while the car
sits **parked and charging** (12 V system + CAN buses awake), so `deviceState.started` latches True and
the device runs the full onroad stack for the entire charge session even though nothing is being
driven. Offroad-only work therefore never runs while charging: the pass-2 upload window (rlog + HD
video, gated `only while parked`), any `only_offroad` maintenance, map downloads' offroad assumptions,
etc. — and the device burns onroad CPU/thermal budget for hours. The driver's framing is correct: an
EV is "almost always charging," so it "never gets to the offroad stage."

---

## 2. Why ignition == onroad (the "who invented this" answer)

This is the **comma harness ignition model**, and it is reasonable for the cars comma designed around.
comma's giraffe/harness taps the car's **switched-ignition 12 V line**; on an ICE (and most ICE-style
EV/hybrid) cars that line is live **only when the driver has turned the car "on" to drive** and dies
when they walk away. So "ignition high" is a perfectly good proxy for "a drive is happening → run the
driving stack, record, be ready to actuate." It is cheap, brand-agnostic, and needs no CAN decode or
car-model knowledge — which is exactly why it's the base contract.

It breaks for **battery EVs** because "12 V / HV system awake" and "driver is driving" are decoupled:
the car wakes its 12 V bus (and often the whole CAN network) to **charge**, to precondition, to phone
home — none of which is a drive. The comma model has no concept of "on but not drivable," so charging
looks identical to driving.

---

## 3. The onroad/offroad decision — exact trace (file:line)

### 3.1 Panda: raw ignition (C, safety-sensitive)
- **Ford Lightning (and any harness car):** ignition is the physical SBU harness voltage.
  `pnw/pnw-panda/board/drivers/harness.h:33` `harness_check_ignition()` reads the SBU GPIO; fed into
  the health packet at `pnw/pnw-panda/board/main_comms.h:15` (`health->ignition_line_pkt`). This is
  **12 V-line presence** — it goes high whenever the truck's 12 V system is awake, **including while
  charging.** ← root cause on the Lightning.
- **Tesla Raven:** no harness ignition line; ignition comes from **CAN** in
  `pnw/pnw-panda/board/drivers/can_common.h:214-224` — message `0x348` (`GTW_status`), bit
  `data[0] & 0x1` (`GTW_driveRailReq`), counter-validated → sets `ignition_can`. The bundled reference
  note (`skills/openpilot/references/panda-and-safety.md:140`) claims this bit is "true only when the
  car is genuinely in Drive/READY."
  **⚠️ UNCERTAIN — flag, do not assume:** if that claim holds, the Raven may **not** exhibit the
  charging-onroad bug (drive rail down while charging in Park). If the drive rail is asserted whenever
  the car wakes to charge, it does. This needs a telemetry check (see §7). The Lightning bug is
  confirmed by the harness model; the Tesla one is not yet confirmed either way.

### 3.2 pandad: publish ignition to the rest of the system
`pnw/pnw-pilot/selfdrive/pandad/pandad.cc:144` `ps.setIgnitionLine(health.ignition_line_pkt)` and
`:242` `ignition_local |= ((ignition_line_pkt != 0) || (ignition_can_pkt != 0))`. So openpilot's
"ignition" = **line OR can**, per panda. `pandaState.ignitionLine` / `ignitionCan` land in cereal.

### 3.3 hardwared: ignition → `started` → the onroad params
`pnw/pnw-pilot/system/hardware/hardwared.py`:
- `:230` **`onroad_conditions["ignition"] = any(ps.ignitionLine or ps.ignitionCan …)`** — the ONLY
  driving-related onroad condition. The other two `onroad_conditions` are `not_onroad_cycle` (a manual
  cycle request) and `device_temp_good` (thermal). There is **no gear / motion / drivable input here.**
- `:333` `should_start = all(onroad_conditions.values())` (+ `startup_conditions` on the first
  transition: terms accepted, booted, free space, registered, etc. — `:301-335`).
- `:358-376` `should_start` True → `started_ts` set; False → `started_ts=None`, `off_ts` stamped.
- `:396` **`msg.deviceState.started = started_ts is not None`** → published on `deviceState`.

### 3.4 manager: `started` → `IsOnroad` / `IsOffroad`
`pnw/pnw-pilot/system/manager/helpers.py:48-50` `write_onroad_params(started, params)`:
`params.put_bool("IsOnroad", started)`; `IsOffroad = not started`. These params + `deviceState.started`
are the single source of "are we onroad."

### 3.5 Who consumes "onroad" (why the decision is load-bearing)
`pnw/pnw-pilot/system/manager/process_config.py` gates processes on `started`:
- `only_onroad` (`:59`) → **modeld, card, controlsd, selfdrived, radard, locationd, calibrationd,
  paramsd, sensord, proclogd, encoderd** (`:75,86,92-114`). `encoderd` = **video recording**; `card`
  (`:103`) = **CAN parse + fingerprint**; controls/selfdrived = **actuation**.
- `logging` (`:22-24`) → `loggerd` (records qlog/rlog while `started`).
- `only_offroad` (`:62`) → offroad-only work.
- `pandad.cc:262` also uses `IsOnroad` to decide the safety relay (`should_close_relay = !ignition || !is_onroad`).
- Uploader (`system/loggerd/uploader.py:439-440`) reads `IsOffroad`/`IsOnroad` to gate pass 2.

**Consequence of this list:** anything that flips the *real* onroad decision off during charging would
stop **recording, fingerprinting, controls, and the model** — see §5 option (b) for why that's a trap,
especially the chicken-and-egg with `card`.

---

## 4. Signals that distinguish "parked & charging" from "driving"

### 4.1 Already in `CarState` today (best, no new CAN work)
Both cars already parse these — available **live, onroad** (card is running because ignition is high):
- **`gearShifter`** — Ford `carstate.py:71-73` (`TrnRng_D_Rq` → park/drive/reverse); Tesla
  `carstate.py:114` / `:224` (`DI_gear` via `GEAR_MAP`). **A car being driven is never in Park; a
  charging car is always in Park.** This is the single most discriminating, already-present signal.
- **`vEgo` / `standstill`** — Ford `:35,37`; Tesla `:110` / `:220`. Distinguishes motion, but a
  stoplight is `standstill` **in Drive**, so standstill alone ≠ parked.
- **`cruiseState.available`** — implies the ACC/drive system is up (a "ready to drive" hint), but it's
  also up at a stoplight; not a park discriminator on its own.

**`gearShifter == park` (optionally AND `vEgo < ~0.1`) is the clean predicate.** It's true for
charging (Park) and false for driving *and* for stopped-in-Drive (stoplight, traffic).

### 4.2 Explicit "charging" signals on CAN (available but NOT parsed today)
- **Ford Lightning** — `pnw/pnw-opendbc/opendbc/dbc/ford_lincoln_base_pt.dbc` defines charge/HV
  display signals: `ChrgStat_D2_Dsply` (`ChargingSystemMaintain`, `ChargingInductive`, …),
  `HybMdeStat_D_Dsply` (`EV_Charge`, …), `ChargePortPwr_St` (`HighPower`/`LowPower`/`Null`),
  `EvNowMode`. **⚠️ Not currently in the carstate parser and NOT confirmed to be on the pt bus openpilot
  reads** — adding them means adding messages to `get_can_parsers` + verifying they appear on bus 0.
  Uncertain; treat as a *possible enrichment*, not the primary signal.
- **Tesla Raven** — charge state would live in a charge/BMS message or be inferable from
  `DI_systemState`/`DI_state` (STANDBY/UNAVAILABLE when not driving). Also not parsed today.

**Judgement:** don't chase a car-specific "charging" bit. **`gearShifter == park`** captures the intent
("not drivable right now") on both cars with code that already exists, and is the right abstraction —
we don't actually care *why* ignition is high, only that the car is parked.

### 4.3 At the panda level?
No clean separation. The Lightning's ignition is a **12 V-line voltage** — the panda cannot tell
"charging-awake" from "drive-awake" from that pin. The Tesla's `GTW_driveRailReq` **might** already
encode drive-vs-charge (§3.1, unconfirmed). Either way, teaching the panda about gear/charge would mean
new CAN decode in **safety-critical C** — off the table for a display/upload gating problem.

---

## 5. Fix options (ranked)

### Option A / C (they converge) — a capability-view "parked" predicate consumed by offroad-gated features ✅ RECOMMENDED
Add `is_parked` (and if we ever wire a real charge bit, `is_parked_charging`) to the capability view
`selfdrive/controls/lib/pnw_vehicle.py` — computed from **`CarState.gearShifter == park` AND
`vEgo < eps`**, no `carFingerprint` branch (honors the 2026-07-11 capability-view rule). Offroad-gated
features consume it and treat "onroad-but-parked" as **offroad-equivalent**. The onroad/ignition
contract is **untouched**.

- **Pure Python. No panda-C change.** Safety-inert.
- **Blast radius: only the features that opt in.** Recording, fingerprinting, controls, model, the
  safety relay all keep behaving exactly as today (ignition still == onroad).
- **Works precisely because card is running:** the signal (gear) comes from `carState`, produced by
  `card`, which is `only_onroad`. During charging ignition is high → onroad → card runs → gear is
  available. So the predicate is evaluable **exactly when we need it.** (This is also why Option B is a
  trap — see below.)
- **Tesla-safe:** the Tesla is never *impaired* — the worst case is it keeps behaving as it does today
  (if the Raven doesn't even have the bug, `is_parked` simply also becomes true in Park and lets its
  own uploads run when charging, which is desirable, not harmful).
- **First consumer = the uploader** (see §6): replace the ad-hoc `OnPriorityNetwork + standstill`
  relaxation with `veh.is_parked` — semantically exact (Park, not just "stopped at a light on home
  WiFi").
- **Optional second consumer (bigger, separate decision):** the manager `logging`/`encoderd` gates
  could also consult it to **stop recording video while parked-charging** (the "wastes onroad
  processing/storage" complaint). This has its own risk (you'd stop capturing parked segments / a drive
  that starts mid-charge needs a clean re-arm), so treat it as a **follow-up**, not part of the core
  fix. Requires a debounce so a 2 s Park at a drive-thru doesn't tear down encoderd.
- **Risk:** low. Main care items: (1) **debounce** `is_parked` (e.g. must hold N seconds) so brief Park
  shifts don't thrash a consumer; (2) **fail safe** — if `carState` is invalid/absent, `is_parked` must
  be **False** (behave as onroad), never True. Both are trivial in the predicate.

### Option B — refine the actual onroad decision so park+charging isn't onroad ⛔ NOT RECOMMENDED
Make `hardwared` `should_start` require "drivable," e.g. gate ignition with gear≠park.

- **Chicken-and-egg, fatal:** gear comes from `carState`, produced by **`card`, which only runs when
  already onroad** (`process_config.py:103`, `only_onroad`). You cannot feed gear into the decision that
  gates the process that produces gear without inverting the architecture (run card offroad, restructure
  the manager). hardwared today only sees `pandaStates`, not `carState`.
- **Blast radius: everything.** During charging you'd lose **recording** (encoderd/loggerd),
  **fingerprinting** (card never runs → the shared device can't re-detect the car it was just plugged
  into), the **model/controls**, and you'd flip the **safety relay** (`pandad.cc:262`). A drive that
  begins during a charge session would start **cold** (no warmed stack, delayed engage) — a real safety
  regression.
- **Tesla risk:** if we *also* required gear≠park globally we could strand the Tesla's normal onroad
  (the Raven's whole point). Violates "Tesla must never be impaired."
- Verdict: correct-sounding, but it fights the process model and endangers recording/fingerprinting/
  controls for no benefit that Option A doesn't deliver safely.

### Option D (minor) — enrich with a real charge bit later
Once Option A ships, optionally add the parsed Ford charge signal (`ChrgStat_D2_Dsply`) / Tesla charge
state to *upgrade* `is_parked` → `is_parked_charging` for telemetry/clarity. Pure additive, but gated
on confirming those messages are on the read bus (§4.2). Not needed for the fix; nice-to-have.

---

## 6. Interaction with the just-shipped uploader workaround

The current workaround (branch `firehose2pnw`, in `wt-stophold2pnw`/`wt-firehose`;
`system/loggerd/uploader.py:95-117`, call sites `:482-484`, `:528-529`) relaxes the pass-2 "only while
parked" gate for the charging case by hand:
```
pass2_allowed(..., onroad, at_home, standstill):
    if onroad and not (at_home or standstill): return False
```
where `at_home = OnPriorityNetwork` (home/priority WiFi) and `standstill = carState.standstill`. It
works, but it's a **proxy**: `standstill` also matches stopped-in-Drive, and `at_home` is a location
proxy for "parked long enough to upload." It answers "is it safe/appropriate to burst 75 MB" indirectly.

**Migration path once Option A lands:** the honest predicate is `veh.is_parked` (Park + not moving).
Replace `standstill` with `is_parked` (a Park gate can *never* fire mid-maneuver — strictly safer than
`standstill`, which can fire at a light), and keep `at_home`/metered/network-type gates as the
*network-appropriateness* layer (orthogonal: "should we spend this link" vs "is the car parked"). So:
- `pass2_allowed` becomes: on real un-metered WiFi AND (`at_home` OR `is_parked`) — the EV-charging
  window is now exactly "parked," not "standing still."
- The `_firehose_network_guard` mirror (`:479-484`) uses the same predicate.
- The uploader stops needing to reason about `standstill` semantics; it consumes one clean capability.

No other consumer needs to change for the core fix.

---

## 7. Open items / what to verify before building (Rule 4 honesty)
1. **Tesla `GTW_driveRailReq` during charging** — confirm from telemetry whether `0x348 data[0]&0x1`
   (and thus `pandaState.ignitionCan`) is high while the Raven sits in Park charging. Determines whether
   the Tesla even has this bug. (The Lightning's harness-line bug is confirmed by the model.)
2. **Ford charge messages on the read bus** — only relevant for Option D; verify `ChrgStat_D2_Dsply` /
   `ChargePortPwr_St` actually appear on bus 0 (pt) before parsing them.
3. **Park gear values** — confirm `GearShifter.park` maps correctly for both (`GEAR_MAP` Tesla,
   `parse_gear_shifter` Ford) in a quick offline check; both already produce a park value.
4. **Debounce constant** for `is_parked` (drive-thru / brief-Park immunity) — pick from a real log.

---

## 8. Recommendation (one line)
Add a **debounced, fail-safe `is_parked` capability** (gear==park ∧ vEgo≈0) to `pnw_vehicle.py`, leave
the ignition/onroad contract alone, and have **offroad-gated features consume it** — first the uploader
(swap `standstill` → `is_parked`), optionally later the recording gate. **Key risk to guard:**
`is_parked` must default **False** whenever `carState` is missing/invalid (never treat an unknown state
as parked), so an EV that is actually being driven can never have its onroad-only stack downgraded.
