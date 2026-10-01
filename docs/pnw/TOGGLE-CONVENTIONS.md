---
updated: 2026-10-01
status: current
---

# Toggle conventions (`toggles2pnw`)

Owner rule, 2026-10-01. Applies to every **pnw** toggle in `selfdrive/ui/layouts/settings/toggles.py`.
Stock openpilot toggles keep their stock semantics (see the audit table) and are not changed by this rule.

## The rule

1. **The default operational state of every toggle is OFF.** A toggle that is OFF means "the shipped behaviour".
   Only a **non-default** setting turns a toggle ON. So most pnw toggles are *opt-out* toggles: a feature that runs by
   default gets a toggle named **`Disable X`** or **`No X`**, and ON means "the driver switched the feature off".
2. **The param default in `common/params_keys.h` is `"0"`** for every bool toggle. A param whose default is `"1"`
   is a violation of the rule (the toggle would show ON in the default state).
3. **Status text shows the live state.** If a toggle governs a service with an observable state (Tailscale), the row
   shows that state in words (`connected`, `disconnected`, `unconfigured`, `connecting`, `installing`, `error ...`), not just a switch.
4. **Car-specific toggles are greyed on the other car, never hidden**, with a short reason in the description
   (for example `Ford F-150 Lightning only`). Greying is **by capability** (`PnwVehicle`), never by fingerprint, and it
   is **display only**: the stored param is never written by the greying. The device moves between a Tesla and a
   Lightning; a clamp that persisted a value would silently rewrite the setting for the other car.
5. **Unknown car** (nothing was ever fingerprinted, so there is no last-known `CarParams`): **everything is enabled.**
   The owner must be able to set troubleshooting toggles while the device sits parked at home. See "Unknown car" below.

## How to add a toggle

1. `common/params_keys.h`: `{"DisableX", {PERSISTENT, BOOL, "0"}}`. Rebuild `params_pyx.so`
   (`scons -u -j$(nproc) common/params_pyx.so`); defaults seed only at manager start, and a missing key raises
   `UnknownKeyName` in the UI (the UI hides a toggle whose key is unregistered rather than crash-looping).
2. `DESCRIPTIONS["DisableX"]` in `toggles.py`: say what runs **by default**, then "Turn this ON to disable it".
3. A `self._toggle_defs["DisableX"]` entry `(title, description, icon, needs_restart)`. Title `Disable X` / `No X`.
   A plain `toggle_item` with a description is fine; only an `action_item` combined with a description is
   non-tappable (raylib trap).
4. Car-specific? Add one row to the car-graying table in `_update_toggles` (param, capability predicate, reason).
5. Read the param where the feature runs. Controllers re-read at about 1 Hz so no restart is needed; if a read fails,
   keep the **default** behaviour (the feature runs) **and log** it (Rule 2).
6. A test: default value, the reader, the greying row, and a mutation check of the new branch.

## Caveats

- **Two-param bridge.** `NoFordAngleSteering` is the only param the driver sees. The opendbc side
  (`opendbc/car/pnw_vehicle.py`) still reads the older positive-sense `FordAngleLateral`, so `toggles.py` and
  `system/manager/manager.py` mirror it (`FordAngleLateral = not NoFordAngleSteering`). Do not add new bridges: read the
  new param directly. When a pin bump moves the opendbc reader, delete the mirror.
- **Renaming/inverting a param does not migrate the stored value.** The old key is simply no longer read. State the
  migration decision in the commit and make sure no code path still reads the old key.
- **`needs_restart` toggles** request an onroad cycle; avoid for troubleshooting toggles where possible.
- **Convenience CAN TX rule (Ford).** Anything the comma **writes** to the Ford over CAN that is not driving control
  (Pro Power re-arm today; any future chime, tailgate or body-comfort write) must be gated by
  `DisableFordConvenience`. Details and the inventory: "Disable Ford Convenience Features" below.
- The multi-button selectors (`CESMode`, `RainMode`, `AutoSpeedReduce`, `DmMode`, `LongitudinalPersonality`) are INT
  params, not bool toggles; the polarity rule does not apply to them.

## Car graying and the unknown car (implemented)

- One table, `CAR_GATED` in `selfdrive/ui/layouts/settings/toggles.py`: `param -> (capability predicate on PnwVehicle,
  reason)`. `car_gate(veh, param)` is the pure function behind it; `_update_toggles` applies it. Rows today:
  `DisableCoopSteer` (Tesla), `NoFordAngleSteering`, `DisableFordSignSpeedLimit`, `DisableEverDrive` (Lightning),
  `NudgeForLaneChange` (both cars), `DisengageOnBrake` (both cars, plus its MADS-panda condition). The reason is
  appended to the description (`Greyed out: Ford F-150 Lightning only.`).
- Greying is **display only**. The stored param is never written by it. Two older forced *displays* are kept as they were
  (the angle-steering row paints ON, and clears the inert `FordAngleLateral` mirror, on a known non-Lightning car).
- **Unknown car**: `ui_state.CP` comes from the persistent `CarParamsPersistent` param (not the onroad-cleared
  `CarParams`), so while parked at home it is the **last-fingerprinted** car. `PnwVehicle.car_known` is False when it is
  `None` (never fingerprinted), the fingerprint is empty or the brand is `mock`; then **every row is enabled and no
  display is forced**, so troubleshooting toggles (for example Disable Ford Convenience Features) can always be set.
  A *known* car without the capability (a third car) is greyed.

## Disable Ford Convenience Features (`DisableFordConvenience`)

A troubleshooting switch (default OFF, no restart): ON = the comma transmits **nothing** on the Ford CAN bus that is not
driving control. Greyed on the Tesla (`CAR_GATED`).

**Inventory of non-driving CAN writes the comma makes on the Ford (2026-10-01, pin `c602973c`):**

| What | Where | Gated now |
|---|---|---|
| **Pro Power Onboard re-arm**: the `0x455` "ON" press, at a standstill, once per ignition and again every 15 min, verified, at most 3 presses per window; payload and `!vehicle_moving` also pinned in the panda | `opendbc/car/ford/carcontroller.py` -> `lightning_extra_pnw.ProPowerArmer` | **yes** |

That is the complete list. Searched: the Ford carcontroller, `lightning_extra_pnw.py`, `everdrive_pnw.py` (read only, no TX),
every `sendcan` publisher in the tree. **There is no tailgate or chime CAN write today**: the tailgate/fob chime and the
ajar chime are As-Built (FORScan) configuration, not comma TX (see the separate CANbus effort's notes); the comma's own
chimes (engagement sounds, `madsquiet`) are on-device audio, not CAN. The manual UDS diagnostic tools in the CANbus effort
(As-Built reads) are run by hand with openpilot stopped and are not a background write.

**Not gated, on purpose (driving control, never behind this toggle):** steering/lateral (LKA/LMC frames), ACC and
longitudinal, cruise-button taps (ICBM SET+/-, auto-resume), MADS, and the HUD/alt-experience frames (LKAS-UI, ACC-UI).

**How the flag reaches opendbc:** the persistent param `DisableFordConvenience`, read by `ConvenienceGate`
(`lightning_extra_pnw.py`) at about 1 Hz with `Params().get_bool` (same runtime-guarded `openpilot.common.params` import the
other Ford pnw features use). **Takes effect within about a second, no reboot.** ON: the armer is not constructed (or is
dropped), so nothing is sent and nothing is armed; OFF again: a fresh armer. **Rule 2:** the first time the toggle is seen ON
the controller logs once `Ford convenience features are DISABLED`; a failed read keeps today's behaviour (the features
run) and logs `DisableFordConvenience unreadable ... keep RUNNING` (first failure, then at most once per minute); a gate that
cannot be built logs and leaves the features running.

**Rule for the future:** ANY new convenience write to the Ford over CAN (a chime, a tailgate or body-comfort write, a keep-alive)
MUST be gated by `ConvenienceGate` and listed in the table above. A new convenience TX that ignores this toggle defeats its
purpose, which is to rule the comma out when something odd happens on the truck. Never put driving CAN behind it.

## Audit (2026-10-01, `origin/3devpnw` at `788010a427`)

Bool toggles in `TogglesLayout._toggle_defs`. "Default" is the `params_keys.h` value.

### pnw toggles

| Param | Default | Car | Complies | Note |
|---|---|---|---|---|
| `HideCESDebug` | 0 | both | yes | opt-out (hide) |
| `Ces2Core` | 0 | both | yes | opt-in experiment, OFF = shadow |
| `CESTurns` | 0 | both | yes | opt-in experiment |
| `NudgeForLaneChange` | 0 | Tesla + Lightning (greyed elsewhere) | yes by polarity | ON = non-default (require nudge); name is positive |
| `DisengageOnBrake` | 0 | Lightning + Tesla with MADS panda (greyed otherwise) | yes by polarity | ON = stock behaviour; name is positive |
| `DisableLaneCentering` | 0 | both | yes | |
| `DisableCoopSteer` | 0 | Tesla only (greyed on Lightning) | yes | |
| `NoFordAngleSteering` | 0 | Lightning only (greyed) | yes | two-param bridge |
| `TailscaleEnabled` | 0 | both | **NO** | feature OFF by default; ON = default behaviour wanted. Fixed in phase 1 -> `DisableTailscale` (no migration; old key removed; see TAILSCALE.md) |
| `NoSpeedLimitDisplay` | 0 | both | yes | |
| `FordSignSpeedLimit` | **1** | Lightning only (greyed) | **NO** | default ON. Fixed in phase 1 -> `DisableFordSignSpeedLimit` (default 0; behaviour unchanged by default; description tells the owner to turn it ON in BC / km/h countries until the camera's km/h limit is measured) |
| `RefreshLocationMap` | 0 | both | yes | momentary action, greyed when no map here |
| `DisableLocationServices` | 0 | both | yes | |
| `EvIncludeLevel2` | 0 | both | yes | opt-in sub-option |
| `DisableEverDrive` | 0 | **Lightning only, but NOT greyed on the Tesla before phase 2** | yes | greying added in phase 2 (`PnwVehicle.everdrive`, display only) |
| `DeferHDVideoUpload` | 0 | both | yes | opt-in |
| `DisableFordConvenience` (new, phase 3) | 0 | Lightning only (greyed) | yes | troubleshooting switch, see below |

### Stock openpilot toggles (stock semantics, NOT changed)

| Param | Default |
|---|---|
| `OpenpilotEnabledToggle` | 1 |
| `DisengageOnAccelerator` | 0 |
| `IsLdwEnabled` | unset |
| `AlwaysOnDM` | unset |
| `RecordFront` | unset |
| `RecordAudio` | unset |
| `IsMetric` | unset |

