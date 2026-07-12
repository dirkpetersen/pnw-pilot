# FORDSAFETY2PNW — BluePilot 4-signal Ford lateral + hardened panda safety (F-150 Lightning)

**Status: DEPLOYED 2026-07-11** (driver-authorized panda reflash; controlled first drive done same
day). Code lives in **pnw-opendbc `master-pnw`** (`14b0caac` port → `19ad2728` latch hardening →
`58ef4868` merge → `deba3bd4` float casts → `723e6b29` crash-proof fallback), consumed by `3devpnw`
via submodule pin bumps (`4353661eee` … `98956ecab6`). Capability-gated to the Lightning
(`pnw_vehicle.four_signal_lat`); **`tesla.h`/`tesla_legacy.h` byte-identical (0-line diff, verified)**.
Fallback build: `backup-20260711-0850-working-preFordSafety` + frozen `4devpnw`.

## What it is

Faithful port of BluePilot's (**alan-polk**, `bluepilotdev/bp-dev`) validated 4-signal Ford lateral
stack: the LMC/LMC2 steer message carries **curvature + curvature_rate + path_offset + path_angle**
instead of curvature-only, with BP's matching custom panda safety. On the Lightning this is the
difference between "holds the lane" and the previous washout-prone curvature-only control.

Preceding steps on the same `fordlat2pnw` arc (all also live, stock-panda-safe, display/curvature-only):
- **predicted-curvature blend** (`2baaeec0`) — turn-exit wind-up fix, 40/60 blend of the model's 0.2 s
  prediction; Gemini-hardened (blend only >9 m/s, suspended in lane changes).
- **human-turn reset** (`0ce08fb6`) — sustained manual turn (|angle|>45° for 1.5 s) flushes desired
  curvature to 0 through the stock rate limiter; release ramps from ~0 (no lurch toward the other lane).
- **BlueCruise cluster** (`ccbce016`, `38979a5d`) — `Tja_D_Stat=7` blue engaged display (CAN-FD) +
  DM-state-driven TJA cluster text; payload-only in TX-allowlisted 0x18A, zero safety surface.

Both blend and reset are now embedded inside `LateralCurvExt` (no double application); the standalone
helpers are bypassed when the 4-signal path owns lateral.

## The safety port (opendbc/safety/modes/ford.h — compiled into the panda → REFLASH to ship)

- BP's looser symmetric curvature ROC tables ({5,16,25 m/s} → {0.0025, 0.0014, 0.00018}) replace the
  stock asymmetric pair; per-signal checks added exactly as BP: path_angle ±0.25 rad, path_offset
  ±1.0 m, curvature_rate (whose bounds exceed the DBC-representable range — **vacuous while steer is
  on, pinned in tests**, not hidden). All three must be exactly 0 when the steer request is off.
- Deliberate deviations: no MADS rx plumbing (not in this tree); `ford_init` keeps the upstream
  non-CANFD longitudinal default.

### ⚠️ The hardened reset latch — the one genuine hole in the faithful port

BP's `reset_bypass_latch_counter`: a neutral frame (curvature==0 && path_angle==0) arms a 60-frame
(~3 s) bypass of ALL violation checks **including `controls_allowed`** — BP's smooth ramp-in after a
human-turn reset. But openpilot streams neutral frames continuously **while disengaged**, so the
latch was permanently armed when off = a full bypass of the panda's disengaged-steering guarantee.
Fix (`19ad2728`): force the latch to 0 whenever `!controls_allowed`, in both LMC and LMC2 handlers.
The ramp-in feature (always engaged) is preserved bit-for-bit; the disengaged block is enforced again.
Doctrine: **"ported safety is not validated safety"** — BP shipped NO tests for its own 4-signal
safety (its stock suite fails against its own ford.h, 8503 subtest failures verified). The suite here
was re-derived for hole-free semantics + dedicated tests that **pin the guarantee, not the hole**
(`test_reset_latch_blocked_when_disengaged`, per-signal limit tests). Ford suite 99 passed / 9000
subtests; full safety suite 0 failed (one pre-existing MISRA-mutation red from tesla_legacy/mg,
present on `master-pnw` base too).

## The numpy engage-crash saga (root cause of the 2026-07-11 card deaths)

card died with exit 1 at ignition-on twice, unreproducible at standstill — looked all day like a
boot-time race. Actual cause (`deba3bd4`): `LateralCurvExt` math returns `numpy.float64`; capnp
setters reject numpy types; **BluePilot's base carcontroller casts `float()` on those assignments,
stock openpilot does not** (its lateral never produces numpy). The port took BP's numpy-producing
lateral but kept stock's uncast assignment → KjException the moment the driver pressed the cruise
button. **Fork-lineage trap:** when porting across openpilot→sunnypilot→BluePilot, diff the BASE
lines at the integration point, not just the feature file.

Forensics chain that caught it (pnw-pilot side):
- `cc8d0c544c` — card logs fatal crashes to swaglog before dying (this manager discards child stderr).
- `47be478339` — manager restarts card on crash (`restart_if_crash=True`, same policy as ui);
  selfdrived alerts during the gap, the crash logger records the traceback.

## Resilience doctrine (mandatory for all Ford lateral changes)

1. **Ported features must fall back to stock on exception, proven by fault injection** — never take
   down card. `723e6b29`: any exception in `lateral_curv_pnw` update → log once, permanently fall
   back to the stock curvature-only path for the drive (steering assist survives).
2. **Engaged-path smoke test** (`test_engaged_smoke_pnw.py`): full engaged loop at real
   speeds/curvatures, asserts the 4-signal path is **ACTIVE** (no silent-fallback false pass) and
   outputs are capnp-safe floats. Crashes here gate on MOTION, not engagement — standstill tests all
   false-pass; this test would have caught the numpy crash on the desk.
3. On a bare opendbc checkout the guarded import falls back to curvature-only (`_latext is None`) —
   verified by smoke test.

## Files

| Where | File | Role |
|---|---|---|
| pnw-opendbc | `opendbc/safety/modes/ford.h` | ported+hardened safety (panda-compiled → reflash) |
| pnw-opendbc | `opendbc/car/ford/lateral_curv_pnw.py` | `LateralCurvExt` 1:1 port (blend, rate deque, path PID, ht-reset, lane-change scaling) |
| pnw-opendbc | `opendbc/car/ford/fordcan_pnw.py` | BP lat-ctl msg builders + `create_acc_ui_msg` (hud) |
| pnw-opendbc | `opendbc/car/ford/values_pnw.py` | `BP_ANGLE_LIMITS` (Python rows stricter than panda tables), `CURVATURE_MAX` |
| pnw-opendbc | `opendbc/car/pnw_vehicle.py` | `four_signal_lat` capability (opendbc-layer view) |
| pnw-pilot | `system/manager/process_config.py` | card `restart_if_crash` |

## Related

Deploy mechanics + the safety-C deploy discipline (protocol-hash/matched-set, never-auto-flash,
controlled first drive): the **`pnw-pilot-deploy` skill** (`ec8cfad29a`, `4a576a0f06`).
`FORDLONG2PNW.md` (the longitudinal counterpart) · `CURVESLOW2PNW.md` (why the Lightning also needs
lower curve speeds) · `CREDITS.md` (attribution: alan-polk / BluePilot).
