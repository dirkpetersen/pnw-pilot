# The two cars

One comma device moves between two vehicles and auto-detects which (except the Raven, which uses a
fixed fingerprint). This file covers each car's identity, code location, and the specific quirks that
have caused real failures.

## Table of contents

- [Ford F-150 Lightning Flash 2025](#ford-f-150-lightning-flash-2025)
- [Tesla Model S HW3 (Raven) 2021](#tesla-model-s-hw3-raven-2021)
- [Swapping the device between cars](#swapping-the-device-between-cars)

## Ford F-150 Lightning Flash 2025

| Attribute | Value |
|-----------|-------|
| Platform | `FORD_F_150_LIGHTNING_MK1` |
| VIN | `1FT6W3L78SWG05094` |
| Battery / built | 131 kWh extended range / April 2025 |
| Model-year range | 2022–2025 (one platform) |
| Bus / harness | CAN FD (`FordCANFD` bus), `CarHarness.ford_q4` |
| `CarSpecs` | `mass=2948, wheelbase=3.70, steerRatio=16.9` |
| Code home | bluepilot fork (`bluepilot/opendbc_repo/opendbc/car/ford/`) |

**Where the code lives:** `ford/values.py` (platform, `FW_QUERY_CONFIG`), `ford/fingerprints.py`
(FW strings), `ford/carstate.py`, `ford/carcontroller.py` (4-signal CAN messaging), and bluepilot's
`selfdrive/ui/bp/` for Ford UI. No panda or core-openpilot changes are needed for the Lightning
beyond fingerprinting — which is what makes it a good candidate to port onto the xnor base
(`light2xnor`) without touching the panda.

### The 2025 fingerprint & the EPS quirk (a real gotcha)

The 2025 model added two ECU firmware strings: ABS `0x760 = TL38-2D053-AD` and fwdRadar
`0x764 = RB5T-14D049-AB`. Adding those to the MK1 fingerprint **was not enough on its own**:

- The 2025 Lightning's **EPS at `0x730` only answers Mazda's UDS query, not Ford's** `auxiliary=True`
  (bus 0) query. The matcher tags Ford EPS responses as `logging=True` and excludes them, so MK1's
  exact-match check fails on the EPS line → openpilot falls back to MOCK/dashcam.
- **Fix:** mark EPS non-essential **for MK1 only** — `non_essential_ecus={Ecu.eps: [CAR.FORD_F_150_LIGHTNING_MK1]}`
  in Ford's `FW_QUERY_CONFIG`. Then the 3 cleanly-responding ECUs (abs, fwdCamera, fwdRadar) identify
  the car. This is MK1-scoped, so it can't cause cross-platform misdetection.

This is upstreamed as **BluePilot PR #130** (2 commits: the FW strings + year bump, then the
non-essential-EPS commit). Full detail: `FORD.md`.

## Tesla Model S HW3 (Raven) 2021

| Attribute | Value |
|-----------|-------|
| Platform | `CAR.TESLA_MODEL_S_HW3` |
| Generation | Raven (refreshed Model S, 2019–2024), Autopilot HW3 |
| Harness | `CarHarness.tesla_model_sx_hw3` |
| Panda safety | `tesla_legacy` with `FLAG_HW3` |
| Code home | the xnor base (`xnor/openpilot`); legacy files committed in `sunny/opendbc/` |

The Raven uses Tesla's **older multi-bus CAN topology** (chassis/party/powertrain/radar split),
which modern openpilot Tesla code (Model 3/Y) doesn't support — hence the `tesla_legacy` module from
xnor. **This support is hard to move into other forks; keep xnor as the base** (see
`references/forks-and-porting.md`).

### Radar: AVAILABLE on the xnor base (NOT vision-only) — verified on-device 2026-06-08

**Important correction.** On the **xnor base deployment** (the device's actual config), the HW3 Raven
**uses the Tesla Continental radar**: `tesla/interface.py` `_get_params_sx` sets
`radarUnavailable = candidate in (CAR.TESLA_MODEL_S_HW2,)` → **False for HW3**. Live device cached
`CarParams` confirms `radarUnavailable=False`, `openpilotLongitudinalControl=True`,
`pcmCruise=True`, `carFingerprint=TESLA_MODEL_S_HW3`. So:
- openpilot does **its own longitudinal in all modes** (not stock ACC); the cruise stalk only sets
  *engagement* (`pcmCruise`), while openpilot commands accel via `DAS_control`.
- the **radar feeds the lead in both Chill and Experimental** — radar usage is **not** mode-dependent;
  Chill vs Experimental only changes the longitudinal *planner* (`mpc` vs `min(e2e, mpc)`).
- stock **AEB** is the Tesla's own ECU + sensors, always on, panda-enforced independent of openpilot.

The `radarUnavailable=True` / "vision-only" claim applies **only to the separate sunnypilot
`xnor2sunny` port** (HANDOFF fix #5), where that comma 3X setup couldn't get the radar (bus mapping)
and forced it off. **Do not state "the Raven is vision-only" for the xnor deployment.**

### Known Raven legacy fixes (sunnypilot `xnor2sunny` port — NOT all in the xnor base)

These are the `xnor2sunny` HANDOFF fixes (grafting Raven onto sunnypilot). #1–#4, #6 also matter on
the xnor base; **#5 is sunnypilot-only** (the xnor base keeps radar):

| # | Fix | File | Symptom if missing |
|---|-----|------|--------------------|
| 1 | capnp ordinal hole — add `mg @35;` next to `teslaLegacy @36;` | `car.capnp` | **every** cereal import crashes |
| 2 | Tesla `0x348 GTW_status` ignition (bus 1, `data[0]&0x1`) | `panda` / `opendbc/safety/ignition.h` | never goes onroad in the Tesla |
| 3 | add `RCM_status (529)` message (Raven uses the `NO_SDM1` seatbelt path) | `opendbc/dbc/tesla_can.dbc` | card AssertionError → crash-loop → "Unknown Vehicle Variant" |
| 4 | point pt/ap_pt/chassis parsers at **bus 1** (not 4/5/6) for HW3 | `tesla/carstate.py` `get_can_parsers` | `canBusMissing` (buses 4/5/6 don't exist on a 3-bus comma 3X) |
| 5 | `ret.radarUnavailable = True` (**sunnypilot port only** — xnor base keeps radar) | `tesla/interface.py` `_get_params_sx` | on that port: radard/plannerd stall (`commIssue`) because that harness didn't tap the radar |
| 6 | "Allow auto updates" toggle (inverts `DisableUpdates`, OFF by default) | sunnypilot `software.py` | auto-updater overwrites the deploy |

### Engaging the Raven

- **Cannot auto-fingerprint** — set the fixed `CarPlatformBundle=TESLA_MODEL_S_HW3` param. "Vehicle
  fingerprint selected manually" in the UI is **expected**.
- **Engage via the lower-left cruise stalk** (PCM cruise / TACC) — there's no separate openpilot
  button. **Disable stock Autopilot/Autosteer first.**
- Expect `calibrationIncomplete` on first engage → do a short calibration drive.
- **Radar is used** (xnor base, `radarUnavailable=False`) — not vision-only. openpilot does its own
  longitudinal in all modes; engagement is via the stalk (`pcmCruise`).

The authoritative simpler reference for Raven CAN is xnor itself (`xnor/opendbc`, `origin/master-xnor`),
which reads the whole car from just bus 0 (party) + bus 2 (ap_party). When bus/parsing questions
arise, diff against it. Deep-dives: `RAVEN.md`, `xnor/openpilot/TESLA.md`, `HANDOFF.md`.

## Swapping the device between cars

The device auto-detects the Ford by fingerprint, but the **fixed Raven fingerprint overrides that**.
So when moving the comma device from the Tesla to the Ford:

1. **Clear the fixed fingerprint** (`set-raven-fixed-fingerprint.py --remove`) or the Lightning is
   misdetected as a Raven.
2. The Ford then auto-fingerprints normally.

Moving from the Ford back to the Tesla: re-apply the fixed fingerprint. This swap mistake is easy to
make and produces confusing "wrong car" behavior, so check the fingerprint param first whenever a car
is misidentified.
