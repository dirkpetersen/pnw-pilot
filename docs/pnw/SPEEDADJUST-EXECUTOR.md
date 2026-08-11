# SPEEDADJUST-EXECUTOR — unified stock-ACC button management (icbm2pnw + speedadjust2pnw)

**Status: BUILT, NOT YET DEPLOYED / NOT ROAD-TESTED.** Written 2026-08-11 on branches
`speedadjust-pilot2pnw` (pnw-pilot) + `speedadjust-exec2pnw` (pnw-opendbc). Do not ship without a
Gemini adversarial review pass and an on-road A/B, per this branch's own safety posture below.

## The gap this closes

`AutoSpeedReduce` (speedadjust2pnw: ease-to-limit-ahead on a police report, and/or proportional
trim of your over-limit excess as the posted speed limit drops) was fully implemented, unit-tested,
and reduce-only-safe — but it only ever fed the op-long MPC path
(`selfdrive/controls/lib/longitudinal_planner.py`). On the Ford F-150 Lightning in its **normal
stock-ACC mode** (`AlphaLongitudinalEnabled=0` — the default; openpilot does not own gas/brake), the
`SpeedAdjustController.cap()` function computed nothing at all for that car and returned `v_cruise`
unchanged: a silent no-op. Meanwhile `icbm2pnw` had already solved the identical problem for CURVE
slow-downs on stock ACC, by spoofing the truck's own SET−/SET+ buttons (0x083) — this effort extends
that SAME mechanism to police/limit slow-downs instead of building a second, parallel one.

## Design principle: ONE unified button-management target, not two competing executors

The truck has exactly one set of stock-ACC SET−/SET+ buttons. Two independent "brains" now want to
steer them — `icbm2pnw` (curve apex targets, from `ces_pnw.py`) and `speedadjust2pnw` (police/limit
targets, from `speedadjust_controller.py`) — and a future brain may want to add a third reason. The
architecture keeps these car-agnostic and reduces them to **one** target every poll, rather than
letting two executors fight over the CAN bus:

```
                    op-long path (unchanged)                stock-ACC path (NEW for speedadjust)
                    ────────────────────────                ─────────────────────────────────────
ces_pnw.py (curve)  ──┐                                      ──▶ IcbmTarget (mem-param)      ──┐
                      │  min()-combine, feeds MPC directly                                      │
speedadjust_          │  via longitudinal_planner.py                                             ├─▶ arbitrate()  ─▶ decide_press()
controller.py         │  (v_cruise cap chain, unchanged)                                          │   (icbm_pnw.py)   PressGovernor
(police/limit)      ──┘                                      ──▶ SpeedAdjustTarget (mem-param) ──┘   RestoreGuard    (100 Hz taps)
```

- **Op-long cars (Tesla always; Lightning with Alpha Long ON):** unchanged. Both brains' reduce-only
  caps still compose via `min()` inside `longitudinal_planner.py` and feed the MPC directly. This
  path was not touched.
- **Stock-ACC cars (today: only the Lightning with Alpha Long OFF):** each brain publishes its own
  target as a small JSON mem-param (`IcbmTarget`, `SpeedAdjustTarget`) in the **identical**
  `{target, ceiling, ts, dir?}` shape. `opendbc/car/ford/icbm_pnw.py`'s new `arbitrate(cmds, now)`
  reduces however many are live down to **one** command every poll — DEC always wins over
  restore/inc, and among multiple fresh `dec` commands the LOWEST target (most restrictive) wins.
  `decide_press()` / `PressGovernor` / `RestoreGuard` — the actual CAN-tap machinery — are
  completely unchanged and have no notion of "which brain": they consume one `IcbmCommand` and have
  always worked this way. `arbitrate()` takes a **list**, so a third brain is just another list
  entry at the call site — no signature change needed later.

## Capability gate: `PnwVehicle.button_management`, not a fingerprint check

Both `pnw_vehicle.py` files (`selfdrive/controls/lib/pnw_vehicle.py` — openpilot layer,
`opendbc/car/pnw_vehicle.py` — opendbc layer, kept in sync since opendbc cannot import
`openpilot.*`) now expose:

```python
self.button_management: bool = self.stock_acc_buttons and not self.op_long
self.icbm: bool = self.button_management                 # back-compat alias
self.speedadjust_buttons: bool = self.button_management   # feature-named alias
```

`carcontroller.py`'s executor gate reads `veh.button_management` (previously `veh.icbm`) — same
boolean value today (only the Lightning declares `stock_acc_buttons`), but now named for what it
actually is: the ONE capability that says "this car has a working stock-ACC button-tap executor,
usable by ANY car-agnostic brain." **No brain (`ces_pnw.py`, `speedadjust_controller.py`) checks
`carFingerprint`/`brand` anywhere** — the fingerprint check lives only inside the two
`pnw_vehicle.py` files, as the project convention requires (`docs/pnw/CLAUDE.md` capability-view
rule). A hypothetical second car with its own brand-specific tap executor would get BOTH curve and
speedadjust slowdowns for free the moment it declares `button_management = True`.

## `speedadjust_controller.py`: computing the target regardless of op-long

`plannerd` (and therefore `SpeedAdjustController.cap()`) runs on **every** car — it is not gated on
`openpilotLongitudinalControl`. Before this change, `cap()` short-circuited on
`self._mode == 0 or not self._long_ok` and never computed a target at all for a stock-ACC car. Now
the split is:

- `self._mode == 0` (feature off): still short-circuits fully, publishes `{}` (inactive), same as
  before.
- `not self._long_ok` (stock ACC): the **same** police/limit-drop math now runs as it always did for
  an op-long car — same `_police_cap()`, `_update_baseline()`, `_limit_drop_cap()`, same `CAP_SLEW`-
  limited slew of `_cap_out`. The **return value** of `cap()` is still `v_cruise` unchanged whenever
  `not self._long_ok` — byte-identical to the old behavior on that axis (`test_no_oplong_is_neutral`
  passes unmodified). The only NEW thing is a **side effect**: `self._cap_out` (the exact same
  slewed value an op-long car would have received) is published to the `SpeedAdjustTarget`
  mem-param, gated at a single choke point inside `_publish_target()` (`if self._long_ok: return`).

### The bounded restore

`icbm2pnw`'s curve brain (`ces_pnw.py`, untouched by this effort — off limits, owned by a parallel
port) has a full episode state machine (`idle → CAP → HOLD → RESTORE → idle`) with its own
independent stock-set tracking, apex-timed hold shortening, and a documented ambiguous-tap
absorption fix. `speedadjust2pnw`'s restore is deliberately **simpler**:

- On cap ENGAGE, the driver's raw pre-cap set (`v_cruise_set`) is latched as `_pub_ceiling`.
- When the cap fully releases (after the existing `RELEASE_S` debounce), a bounded restore window
  opens: `_restore_ceiling = _pub_ceiling`, `_restore_deadline = now + RESTORE_WINDOW_S` (45 s,
  matching `icbm2pnw`'s own restore-window convention).
- While the window is open, `SpeedAdjustTarget` publishes `{"target": ceiling, "ceiling": ceiling,
  "dir": "inc"}` every `PUB_THROTTLE_S` (0.25 s, matching `IcbmTarget`'s cadence).
- The restore is canceled instantly (falls back to publishing `{}`) on: window expiry, a NEW cap
  engaging (**DEC ALWAYS WINS** — mirrors `icbm2pnw`'s own new-cap-preempts-restore rule), or
  `sm['carState']` showing `gasPressed`/`brakePressed`/`not cruiseState.enabled` — a
  **defense-in-depth** check only, since the shared executor's `decide_press()`/`RestoreGuard`
  already independently gate every press on the SAME signals read directly off the real CAN state.
  `sm['carState']` is already subscribed by `plannerd` at its normal control rate (not a new
  subscription — the project's "no carState subs in background procs" lesson does not apply to a
  core control process reading a field it already has).
- This brain does **not** track the truck's own reported stock set speed at all (unlike `icbm2pnw`'s
  episode machine). It only decides WHEN to offer a restore and WHAT ceiling to offer; the shared
  executor's `decide_press()`/`RestoreGuard` do all the closed-loop, human-detection work against
  the real stock set — that machinery is untouched and reused as-is.

## `icbm_pnw.py`: `arbitrate()`

```python
def arbitrate(cmds: list[IcbmCommand | None], now: float) -> IcbmCommand | None:
    ...
```

Rule: collect every FRESH (`now - c.ts <= STALE_LIMIT_S`) `dir == "dec"` command; if any exist, the
one with the lowest `target_ms` wins, passed through unchanged (its own `ceiling_ms`/`ts` — no
cross-source merging). Only if NO source wants a `dec` does a fresh `dir == "inc"` get to run (lowest
target/ceiling wins if more than one is offered simultaneously). Otherwise `None` — silence.
Staleness is checked **inside** `arbitrate()`, before the `min()`, so a dead/stale source can never
veto or out-compete a live one by lingering in the list. `decide_press()` independently re-checks
staleness on the winner too (defense in depth, unchanged from before).

`carcontroller.py`'s `_icbm_buttons()` polls both mem-params at 4 Hz (unchanged cadence), parses them
with one shared helper (`_parse_button_cmd`, renamed from `_parse_icbm_cmd` — identical parsing logic
for both since the shapes match), and calls `cmd = arbitrate([self._icbm_cmd, self._sa_cmd], now)`
before `decide_press()`.

## Safety gates carried over unchanged

- **DEC-ONLY for caps**: a `dir` absent/`"dec"` command can only ever lower the stock set
  (`decide_press`, unchanged).
- **Guarded restore**: an `"inc"` command is clamped to `min(target, ceiling)` in `decide_press`, and
  `RestoreGuard` latches BLOCK for the rest of the restore episode the instant the stock set moves in
  a way the executor didn't command (any decrease, or a rise faster than the tap cadence).
- **Acts only while stock ACC is engaged**; never engages/resumes it; driver gas/brake pauses/aborts
  a press mid-tap (`PressGovernor`).
- **Stale heartbeat → silence** (`STALE_LIMIT_S = 2.0 s`, and now also checked pre-arbitration).
- **Reduce-only-to-slow with controlled restore, never above the driver's own pre-cap set** — both
  brains' `ceiling` fields are always ≤ the driver's actual cruise set; `arbitrate()` never invents a
  higher ceiling than either source offered.
- **No panda/safety files touched.** 0x083 was already TX-allowlisted for cancel/resume/`icbm2pnw`;
  nothing about the allowlist or the panda safety model changed.

## What was NOT touched

- `selfdrive/controls/lib/ces_pnw/ces_pnw.py` and `selfdrive/controls/lib/vtsc_pnw/vtsc_constants.py`
  — owned by a parallel curve-ICBM effort; this port coordinates with it only through the shared
  `IcbmTarget`/`SpeedAdjustTarget` mem-param CONTRACT (the `{target, ceiling, ts, dir?}` shape), never
  by editing either file.
- `panda/`, `opendbc/safety/` — no safety-critical file changed.
- The op-long return-value path in `speedadjust_controller.py.cap()` — bit-identical to before.

## Uncertainties / flagged for review before deploy

1. **No on-road validation.** This is a NEW actuator on the stock-ACC bus (a second brain now able to
   press the same buttons `icbm2pnw` presses). It has full unit-test coverage (44 pnw-pilot
   `speedadjust_pnw` tests, 32 pnw-opendbc `icbm_pnw` tests) but has never driven.
2. **The restore's driver-intervention check reads `sm['carState']` inside `plannerd`.** This mirrors
   an existing, already-subscribed field, but it is a code path that did not exist before in this
   file — worth an explicit Gemini pass on the `_driver_intervening()`/`_step_restore()` logic.
3. **Two independent restore mechanisms can both offer `"inc"` in the same tick** (icbm2pnw's episode
   machine and speedadjust2pnw's bounded window) — `arbitrate()` handles this by taking the lower
   target/ceiling, but this specific interleaving (both a curve just cleared AND a police/limit cap
   just cleared, simultaneously) has no dedicated end-to-end test across the two repos (only unit
   tests of `arbitrate()` with synthetic dual-inc inputs). Cross-repo integration testing in one
   Python process was attempted and abandoned — mixing two full monorepo checkouts (`cereal`,
   `opendbc` namespace packages from two different trees) in one interpreter native-aborted via
   capnp; this is not how production ever runs (separate processes, communicating only through the
   mem-param file), so the abandonment does not reflect a real production risk, but it does mean this
   exact interleaving is unverified end-to-end.
4. **`RESTORE_WINDOW_S = 45.0` and `PUB_THROTTLE_S = 0.25`** were chosen to MATCH `icbm2pnw`'s
   existing conventions, not independently field-tuned for the police/limit use case (a limit-drop
   cap and a police cap may plausibly want different restore-window lengths than a curve cap — not
   investigated).
5. **No apex-timed / early restore-hold-shortening** for speedadjust (icbm2pnw has one, tied to curve
   apex-passage geometry that has no speedadjust equivalent — not applicable here, but noting the
   asymmetry).

## Files touched

**pnw-opendbc** (`speedadjust-exec2pnw` branch, off `master-pnw`):
- `opendbc/car/ford/icbm_pnw.py` — added `arbitrate()`
- `opendbc/car/ford/carcontroller.py` — reads both mem-params, calls `arbitrate()`, gate renamed to
  `veh.button_management`
- `opendbc/car/pnw_vehicle.py` — added `button_management` (+ `icbm`/`speedadjust_buttons` aliases)
- `opendbc/car/ford/tests/test_icbm_pnw.py` — 12 new `arbitrate()` tests

**pnw-pilot** (`speedadjust-pilot2pnw` branch, off `steerlimit-log2pnw`):
- `selfdrive/controls/lib/speedadjust_pnw/speedadjust_controller.py` — publish + bounded restore
- `selfdrive/controls/lib/speedadjust_pnw/tests/test_speedadjust.py` — 14 new tests
- `selfdrive/controls/lib/pnw_vehicle.py` — added `button_management` (+ `speedadjust_buttons` alias)
- `common/params_keys.h` — registered `SpeedAdjustTarget` (`CLEAR_ON_MANAGER_START`, `JSON`)
- `PNW-PILOT-FEATURES.md`, `docs/pnw/SPEEDADJUST-EXECUTOR.md` (this file) — docs

## Related

`docs/pnw/ICBM2PNW.md` (the curve-ICBM design this mirrors) · `docs/CES.md` / `docs/VTSC.md` (shared
curve math, untouched) · `docs/pnw/CLAUDE.md` (capability-view rule) · `docs/DEVICE-STATE.md` (param
registry — add `SpeedAdjustTarget` there on deploy).
