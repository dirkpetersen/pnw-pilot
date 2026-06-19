# BSM2XNOR — Tesla Model S Raven blind-spot → openpilot

**Branch:** `bsm2xnor` (off `integration2xnor`) · **Status:** code staged, **NOT deployed** — gated on an
on-car CAN check. **Car:** 2021 Tesla Model S, Raven class, HW3 (`TESLA_MODEL_S_HW3`, a `LEGACY_CARS`
platform).

## Goal

Give the Raven a working left/right **blind-spot** signal so lane changes (especially the
`auto2xnor` *nudgeless* ones) refuse to move into an occupied lane. This is milestone 1 toward
fully-automated, non-driver-initiated lane changes — it is the *safety gate*, not the autonomy.

## The sensor-access reality (why this is the tractable path)

- openpilot **cannot tap the Tesla's cameras** on any model — the 8-camera suite and the 12
  ultrasonics feed Tesla's own Autopilot computer (APE) over internal video links that never appear
  on CAN. openpilot perception always runs on the **comma 3X's own forward cameras**, which have **no
  rear/side view**. So the comma can't *see* the blind spot itself.
- What *is* on CAN is the APE's **processed output**: a blind-spot warning derived from the rear
  repeater cameras + ultrasonics. That's what we tap.

## What was already there vs. the Raven gap

openpilot's **modern** Tesla path already reads blind-spot:
```python
ret.leftBlindspot  = cp_ap_party.vl["DAS_status"]["DAS_blindSpotRearLeft"]  != 0
ret.rightBlindspot = cp_ap_party.vl["DAS_status"]["DAS_blindSpotRearRight"] != 0
```
So **every non-legacy Tesla already has it**: Model 3, Model Y, and the refreshed Model S / Model X
(the "party-bus" platforms). The **legacy class** — Model S HW1/HW2/**HW3 (our Raven)**, Model X
HW1/HW2 — returns early through `update_legacy()`, which never reads blind-spot. That's the only
reason the Raven lacked it.

## The change (this branch)

One file: `opendbc_repo/opendbc/car/tesla/carstate.py`, in `update_legacy()`. The legacy DBC
(`tesla_can.dbc`) carries blind-spot in **`AutopilotStatus` (0x399 / 921)**, not `DAS_status`:
```
BO_ 921 AutopilotStatus:
  SG_ DAS_blindSpotRearLeft  : 4|2  # 0 NONE, 1/2 WARNING, 3 SNA
  SG_ DAS_blindSpotRearRight : 6|2
```
Added (read via `cp_chassis`, which parses `tesla_can.dbc`; KeyError-guarded so it can never crash
another legacy variant):
```python
try:
  bsm = cp_chassis.vl["AutopilotStatus"]
  ret.leftBlindspot  = bsm["DAS_blindSpotRearLeft"]  in (1, 2)
  ret.rightBlindspot = bsm["DAS_blindSpotRearRight"] in (1, 2)
except KeyError:
  pass
```
Only an actual warning (1/2) counts as occupied; `0`/`SNA(3)` → clear, so behaviour **degrades
exactly to today's baseline** if the car isn't emitting BSM.

## The lane-change gate is reused, not modified

`selfdrive/controls/lib/desire_helper.py` **already** honours `carstate.left/rightBlindspot` for both
paths (verified, no change needed):
```python
blindspot_detected = ((carstate.leftBlindspot  and direction == left) or
                      (carstate.rightBlindspot and direction == right))
...
if self.nudgeless_lane_change and not blindspot_detected:
    self.auto_lane_change_timer += DT_MDL      # nudgeless hold only accrues when clear
else:
    self.auto_lane_change_timer = 0.0          # reset on any blindspot
...
elif (torque_applied or auto_lane_change) and not blindspot_detected:
    self.lane_change_state = LaneChangeState.laneChangeStarting   # blocked when occupied
```
So **populating the carstate field is the whole wiring** — both the manual-nudge and the nudgeless
auto-start are blocked into an occupied lane, and the nudgeless hold-timer resets if a blind spot
appears mid-hold. Behaviour-neutral until `NudgelessLaneChange` is on and a real warning arrives;
**no panda-safety code touched.**

## PENDING — required before deploy: on-car CAN verification

The assumption "0x399 rides the chassis bus (5) and carries live values while openpilot is engaged"
must be confirmed. Run with the car ON:
```bash
# on the device
source /usr/local/venv/bin/activate
PYTHONPATH=/data/openpilot python /data/dirk/verify-raven-bsm.py            # 20s scan
PYTHONPATH=/data/openpilot python /data/dirk/verify-raven-bsm.py --watch    # live; walk a car up each side
```
Pass criteria:
1. `0x399` is seen, and **on bus 5** (what `cp_chassis` reads). If it's on another bus, repoint the
   read at the matching parser/bus in `get_can_parsers`.
2. `leftBSM`/`rightBSM` actually flip `0 → 1/2` when a vehicle enters each blind spot **while
   openpilot is engaged**. If it's stuck at `NONE/SNA`, the APE isn't producing BSM under openpilot
   control — then BSM via this path isn't viable and we fall back to added rear sensing.

## Deploy plan (after CAN check passes)

`carstate.py` is **pure Python** loaded at runtime — **no scons build needed** (unlike the cesState
schema change). Deploy is a guarded file copy:
1. back up `/data/openpilot/opendbc_repo/opendbc/car/tesla/carstate.py`,
2. md5-guard + replace with this branch's version, clear pyc,
3. activate on next comma boot/drive (or restart). Rollback = restore the backup.
(A `patch-bsm.py` / `update-bsm.sh` pair mirroring the CES deploy can be generated once the CAN check
passes.)

## Roadmap to system-initiated lane changes (honest scope)

- **M1 (this):** BSM as a safety gate. Driver still initiates (blinker), nudgeless is now
  blind-spot-protected. Low risk.
- **M2:** richer rear awareness. The BSM boolean has no closing-speed/distance; the legacy front
  radar is forward-only (and needs a physical CAN tap to bus 1). True "is it safe to merge in N
  seconds" needs rear object data openpilot can't currently get from the Raven over CAN → likely
  **added rear-facing sensing**.
- **M3:** autonomous *initiation* (decide to change lanes with no driver input). This is a planner +
  **panda-safety** change, not a port — comma's openpilot deliberately only *executes* a
  blinker-initiated change. Research-grade; requires M2 + a safety review.

## Files
- `opendbc_repo/opendbc/car/tesla/carstate.py` — the BSM read (only functional change)
- `verify-raven-bsm.py` (repo root) — on-car CAN confirmation (run when powered)
- `selfdrive/controls/lib/desire_helper.py` — **unchanged**; reused gate
