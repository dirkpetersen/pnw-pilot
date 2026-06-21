# TESTING.md — the `testing` branch (full live-device snapshot)

**Purpose:** `testing` is a single branch that contains **all code currently active on the comma 3X**
(`eb1f2f7`, dongle `2fd850c60cc5bfef`) as of 2026-06-21. The device runs `/data/openpilot` as a file
overlay (patched in place), not a git checkout — this branch reassembles that exact live state into one
buildable tree so it can be built, tested, and rolled out as a coherent distribution.

Built on **`integration2xnor`** (the deployed base) + the deltas deployed on top of it.

## What's in it

| Layer | Source | Active-on-device feature |
|-------|--------|--------------------------|
| **Base** | `integration2xnor` | CES (Conditional Experimental Switching) core **+ cesState logging**, VTSC (Vision Turn Speed Control), network2xnor (tethering NAT + perpetual + priority-WiFi + GPS geo-gate + LTE guard), connect2xnor (two-pass WiFi-only upload), light-ces-gentle (CES 3-way Off/Light/Standard + gentle Lightning VTSC), auto2xnor (nudgeless lane change + no-disengage-on-brake), mapd2xnor (OSM speed limits, PNW maps), dmon **param keys** |
| **+ bsm2xnor** | merge | Tesla Raven blind-spot from `AutopilotStatus` (0x399) |
| **+ fingerprint2xnor** | merge | offroad sidebar car label (display-only) + `card.py` never-persist-MOCK durable dashcam fix |
| **+ 2025 Ford fingerprint** | cherry-pick `d7dd108` | F-150 Lightning Flash 2025 (TL38 ABS, RB5T radar) — required or the 2025 truck drops to MOCK/dashcam |
| **+ Ford SecOC fix** | cherry-pick `326d05b` | don't false-flag SecOC dashcam when camera msgs are merely absent at fingerprint |
| **+ relaxed driver monitoring** | cherry-pick `e5d3aa8` + `a02f305` | dmon2xnor-b: relaxed dual-counter DM (`SensitiveDriverMonitoring`, default OFF = relaxed 1h/3h) + software-update gate (`AllowSoftwareUpdates`), behind toggles |

## What's deliberately EXCLUDED (committed but NOT deployed to the device)

- **light2xnor Tier-2** — F-150 radar (camera lead) + opt-in openpilot longitudinal (commit `7576099`).
  Lives on `light2xnor`/`fordsecoc2xnor`; opt-in / default-OFF and **not active on the car**, so it was
  skipped (device-exact). `steerActuatorDelay` stays stock 0.2 here, not Tier-2's 0.22.
- **upload2xnor** — home-WiFi-geofenced uploader (commit `12fe328`). Parked; device uploader is baseline.

## Verification done on assembly

- All active-feature files present (CES/VTSC, networkd arbiter+geo_gate+lte_guard, mapd manager+binary,
  dmon helpers, bsm, fingerprint).
- Ford: 2025 fingerprint + SecOC fix present; **all Tier-2 markers absent** (`0.22`, `STEER_ASSIST_DATA`,
  `alphaLongitudinalAvailable = True`).
- `common/params_keys.h`: **no duplicate keys** (the dmon cherry-pick's redundant
  `SensitiveDriverMonitoring`/`AllowSoftwareUpdates` were de-duped) and the dmon keys carry the
  "REQUIRED or dmonitoringd crashes" warning.
- Touched Python files compile (`helpers.py`, `toggles.py`, `software.py`, `updated.py`, `card.py`,
  ford `interface.py`).

## Before deploying from this branch

Follow the deploy checklist in `~/gh/comma/DEVICE-STATE.md` (diff `params_keys.h` keys to catch any
dropped key, rebuild `common/params_pyx.so` on device, set the persistence guards, confirm
`dmonitoringd` doesn't crash). This branch is intended for **building/testing the whole distribution**;
the surgical on-device deploy scripts still live at the `~/gh/comma` root.
