# PNW code conventions (driver-directed — follow these when writing ANY pnw feature code)

## 1. Capabilities, never fingerprints (directive 2026-07-11, repeated twice — do not slip again)

Feature code must NEVER branch on `carFingerprint` / brand strings. Ask a **capability view**:

- **openpilot layer**: `openpilot.selfdrive.controls.lib.pnw_vehicle.PnwVehicle(CP)` —
  `op_long`, `stock_acc_buttons`, `ces_shadow`, `ces_capable`, `nudgeless`.
- **opendbc layer** (cannot import openpilot.*): `opendbc.car.pnw_vehicle.PnwVehicle(CP)` —
  `op_long`, `stock_acc_buttons`, `icbm`, `pc_blend`. Keep the two mirrors in sync.

Adding a car = editing the capability class(es), not hunting string comparisons. Adding a feature =
adding a capability attribute there first, then consuming it. The ONLY sanctioned fingerprint
comparisons are INSIDE the capability classes themselves and in upstream-idiom brand code that
predates the rule (grandfathered: `desire_helper`, upstream Ford platform checks like the
Bronco/F150_MK14 anti_overshoot tuple) — do not add new ones.

## 2. Cross-car behavior changes need justification

Default: changes are scoped per vehicle class via capabilities. A change may be cross-car ONLY when
it fixes a bug that exists identically on every car (e.g. the red-light accel-zone fix — driver
explicitly kept it generic). State the justification in the commit message.

## 3. The Tesla must never be impaired by truck work (and vice versa)

Every Ford/Lightning feature ships with an explicit "Tesla bit-identical" argument (capability
gating + review). Gemini reviews must be asked this question for any shared-file change.

## 4. Established fork patterns (follow, don't reinvent)

- Actuation-adjacent config flows via **mem-params** (`/dev/shm/params`), registered in
  `params_keys.h` (JSON keys take DICTS — a pre-serialized string TypeErrors silently).
- Heartbeat + stale-window for any brain→executor channel (executor goes silent on stale).
- Cross-layer imports inside opendbc are RUNTIME-GUARDED (`try/except` → feature off on a bare
  opendbc checkout): `openpilot.common.params`, `cereal.messaging`, model constants.
- New toggles default OFF; safety-critical never weakened; dec-only/reduce-only actuators preferred.
- Every behavior change lands with scenario tests replayed from real telemetry where possible, and
  new telemetry fields so the next drive can measure it.
