# CURVESLOW2PNW — the Lightning slows MORE for curves (per-car curve-speed penalty)

**Status: DEPLOYED on `3devpnw`** — branches `curveslow-lightning` (`175cc19379` → `4ffc737570` →
`19dbd64170`), `descentcurve2pnw` (`14da3d280b`), `icbmalign2pnw` (`de73e7bd1f`) all merged
2026-07-11/12. Field-calibrated across **three same-evening I-90 iterations** plus the 2026-07-11
takeover analysis (`drives/2026-07-11/lightning-icbm-nofire/`). Tesla provably 0.0/neutral at every
call site (pinned by tests); no panda changes, no numpy, no new param keys.

## Why

The Lightning's EPS is physically weaker than the Tesla's it shares the device with: it washes out
of fast highway sweepers the Tesla holds. 2026-07-11 drive: **91 high-speed steering-override
clusters**, VTSC binding caps at takeover 29.1/32.1/33.0 m/s. EPS torque demand scales with v², so
the steering-authority deficit appears **at speed** — slow tight corners have ample authority.

## The penalty (one shared function, both actuation paths)

`PnwVehicle.curve_speed_penalty_ms(v_target, pitch_rad, is_left)` in
`selfdrive/controls/lib/pnw_vehicle.py` (mirrored capability in `opendbc/car/pnw_vehicle.py`)
returns extra m/s to subtract from a binding curve target. Non-Lightning → **0.0**. Applied in:
- **VTSC** (`vtsc_controller.py`, op-long path) — on the finalized curve-safe speed, floored at V_MIN;
- **ICBM** (`ces_pnw.py`, stock-ACC path) — same function, same knobs (`icbmalign2pnw` parity: a
  cross-subsystem test pins VTSC cap == ICBM published target for identical inputs, within 0.02 m/s).

### Field calibration story: the penalty is a HUMP, not a ramp

- **v1** (`175cc19379`): linear ramp 1.5 mph @ ≤30 mph targets → 10 mph @ ≥65 mph.
- **Floor fix** (`4ffc737570`): all 13 takeover moments on the first penalty build showed the cap
  pinned at exactly 29.1 m/s = the 65 mph posted limit — **the highway speed-limit floor (driver
  rule 2026-07-01) ran AFTER the penalty and clamped it away entirely.** Fix: lower the floor by the
  car's own penalty — the Lightning may trim to `limit − penalty(limit)` (~55 in a 65 zone); the
  Tesla's penalty is 0.0 so its floor stays exactly at the posted limit (rule unchanged there).
- **Iteration 3** (`19dbd64170`): driver feedback across three same-evening passes — slow corners
  fine at ~1 mph; the MID-speed washout zone (binding targets 45–62 mph) wants the full cut
  (driver: "right, maybe a touch slow"); long fast gentle sweepers were **too slow** on the
  monotonic ramp ("needs to accelerate more"). Final shape: **piecewise-linear hump over 4 knots**,
  1 mph below 30 → 5 mph across 45–62 → taper to 1.5 mph by 75+. Knots order-sanitized so a bad
  config degrades flat instead of crashing.

### Descent + left-curve factors (`descentcurve2pnw`)

Field evidence: two DOWNHILL LEFT washouts under op-long (18:12:16 @ 77 mph vs cap 71; 19:17–18).
- **Descent guard:** penalty ×= 1 + `descent_gain`·min(|pitch|, `descent_pitch_cap`) when pitch < 0
  (from `carControl.orientationNED[1]`); defaults 8.0 / 0.12 rad → 5% grade = +40%. On a descent
  gravity eats the regen budget, so enter slower instead.
- **Overspeed-into-curve escalation:** when v_ego > applied cap + `overspeed_margin_mph` AND the
  entrance needs more than regen, the rate-limit ceiling escalates to the EXISTING
  `SHARP_A_DECEL_MAX` (no new decel ceiling).
- **Left factor** (default 1.15): US crown drains right → left curves bank adversely; both washouts
  were downhill LEFTS. Sign verified in-tree: `modelV2.orientationRate.z > 0` at apex = LEFT
  (`apex_turn_direction`); map/far candidates use pure `map_turn_direction()` (path-geometry cross
  products, ambiguous → 0 neutral).
- Multipliers clamped ≥ 1 → the penalty only ever GROWS → the target only moves DOWN; total penalty
  hard-capped at `penalty_cap_mph` (15).
- Same commit's ICBM high-speed map fixes (500 m far candidate, firm-decel far ramp, `map_scale`
  0.92) are documented in `docs/ICBM2PNW.md`.

## `/data/pnw/curve.json` — device-local tuning override

Same defensive-loader discipline as `dm.json` (os.stat gate: regular file ≤ 64 KiB; blanket
try/except; per-key clamps chosen so a bad config can only degrade toward neutral, never invert —
e.g. `left_factor` ∈ [1.0, 1.5], `map_scale` ≤ 1.0, penalties ∈ [0, 15] mph). Schema:

```json
{"lightning": {"penalty_min_mph": 1.0, "penalty_max_mph": 5.0, "penalty_taper_mph": 1.5,
               "low_v_mph": 30, "peak_lo_v_mph": 45, "peak_hi_v_mph": 62, "taper_v_mph": 75,
               "descent_gain": 8.0, "descent_pitch_cap": 0.12, "penalty_cap_mph": 15,
               "left_factor": 1.15, "overspeed_margin_mph": 2.0,
               "map_scale": 0.92, "icbm_firm_decel": 1.4}}
```

Read once at construction; unknown keys ignored. **The field-calibrated iteration-3 values are now
the source DEFAULTS** (`_CURVE_DEFAULTS` in `pnw_vehicle.py`) — the device-local `curve.json` used
during live calibration was removed once its values were absorbed, so the source defaults rule.
Note the philosophy difference vs `dm.json`: DM keeps personal values external-only (source stays
strict); curve.json is a *tuning* override whose calibrated end state belongs IN source.

## Washout regression registry (validation-only, never feeds control)

`tools/washouts.py` scans `drives/*/lightning-*/ces_events*.jsonl` for steering-override clusters
>55 mph; checked-in fixture from the 2026-07-11 logs (158 clusters, 27 with a binding cap, incl.
the driver-cited 18:12 washout). Regression test: for every binding washout, penalty + descent(4%)
+ left factor must yield a cap ≥3 mph below the recorded entry speed. Registry sign convention:
raw Ford `StePinComp` `strAng < 0` = left on this truck.

## Telemetry

`vtsctele2pnw` (`610697bd58`): `VTSCStatus` carries `pen` (penalty actually applied, m/s), `pitch`
(rad), `dir` (L/R/""); bridged into ces_events ticks as `vtscPen`/`vtscPitch`/`vtscDir`, plus
explicit `lead`/`gapS`/`dV` lead forensics — added because the 2026-07-12 westbound analysis could
not decompose an over-slow without them.

## Related

`docs/ICBM2PNW.md` (the stock-ACC actuation side) · `FORDSAFETY2PNW.md` (the lateral side of the
same weak-EPS story) · `drives/2026-07-11/lightning-icbm-nofire/DRIVE_REPORT.md` (the motivating
analysis) · `docs/VTSC.md` (base VTSC design).
