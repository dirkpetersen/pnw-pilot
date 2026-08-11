"""
speedadjust2pnw — automatic cruise-speed reduction for lower speed limits and police-ahead warnings.

`SpeedAdjustController.cap(sm, v_cruise_set, v_cruise, v_ego, v_cruise_initialized)` returns a possibly-
lowered cruise speed (m/s) for the longitudinal planner — **reduce-only, NEVER raises above v_cruise**.
The planner MPC bounds the decel rate, so a cap can never slam; it composes with VTSC/MTSC via `min()`.
(See the `cap()` docstring below for why it takes both a raw `v_cruise_set` and an effective `v_cruise`.)

Selector param `AutoSpeedReduce` (INT, default 0):
  0 = Off
  1 = Police         → ease to (posted speed limit + 5 mph) when a police report is ~30 s ahead
  2 = Police+Limits  → ALSO cap proportionally when the posted limit drops (keeps your over-limit ratio)

Works in BOTH Chill and Experimental (it caps `v_cruise`, which both modes honour). The RETURNED cap
only ever feeds the op-long MPC path, so `cap()`'s return value stays exactly what it always was —
`v_cruise` unchanged whenever openpilot does not own longitudinal (op-long off), so this function is
still behaviour-neutral by default on that axis. Inputs are read from params at ~1 Hz — `MapSpeedLimit`
+ `LocationServices` from the /dev/shm mem store (written by the mapd bridge / location_servicesd) and
`AutoSpeedReduce` persistent — with **NO msgq subscriptions** for those reads (the background-
subscriber cascade lesson); `sm['carState']` (already subscribed by plannerd at its normal control
rate — not a new subscription) is read only as a defense-in-depth abort signal for the restore window
below.

Runs inside plannerd (20 Hz) via the planner's cap chain. Pure-ish: only param reads + monotonic time.

speedadjust-exec2pnw (2026-08): the reduce-only cap math ITSELF now runs identically regardless of
op-long — plannerd runs on every car (not gated on openpilotLongitudinalControl), so the target was
always being computed for op-long cars but silently discarded for stock-ACC ones. On a car with NO
op-long (the Lightning in its normal stock-ACC mode), this module ADDITIONALLY publishes its computed
target as a `SpeedAdjustTarget` mem-param (`CLEAR_ON_MANAGER_START`, JSON) in the SAME
`{target, ceiling, ts, dir?}` shape as icbm2pnw's `IcbmTarget` — so a capability-gated, per-brand
stock-ACC button-tap executor (today: `opendbc/car/ford/icbm_pnw.py`, gated on
`PnwVehicle.button_management`) can steer the truck's own SET-/SET+ buttons toward it, exactly the
way icbm2pnw already does for curve slow-downs. Both mem-params share ONE executor; `arbitrate()`
(icbm_pnw.py) reduces whichever brains are live down to a single most-restrictive target every poll —
see `docs/pnw/SPEEDADJUST-EXECUTOR.md` for the full design. This module stays fully car-agnostic: it never
checks carFingerprint/brand, only `self._long_ok` (openpilotLongitudinalControl) — the actual per-car
capability gating lives in `PnwVehicle`/`icbm_pnw.py`, not here.

The mem-param publish also includes a bounded (`RESTORE_WINDOW_S`, matching icbm2pnw's convention) SET+
restore back to the ceiling latched at cap-engage once the cap clears — deliberately SIMPLER than
icbm2pnw's full episode state machine: it does not track the truck's own reported stock set speed at
all (the shared executor's `decide_press`/`RestoreGuard` already do that closed-loop, human-detection
work directly off the real CAN state); this brain only decides WHEN a restore should be offered
(bounded window, canceled instantly by a new cap or `sm['carState']` driver/ACC intervention) and
WHAT ceiling to offer, never above the driver's own pre-cap set.

speedanchor2pnw (2026-07-18, three review-caught fixes to the anchor/seed math — none change the
reduce-only envelope, all conservative):
  F2  cap() now takes the driver's raw PRE-VTSC set (`v_cruise_set`) separately from the current
      EFFECTIVE ceiling (`v_cruise`, after VTSC/etc. have already reduced it). The limit-drop ratio
      anchor and the cap-slew seed use `v_cruise_set` — anchoring/seeding off a VTSC-curve-reduced
      `v_cruise` instead recorded a too-low ratio (could disable the trim) and seeded the slew from
      curve speed, crawling back up at CAP_SLEW after VTSC released instantly. The emitted output is
      still bounded reduce-only against `v_cruise` (the effective ceiling), unchanged.
  F3  the limit-drop baseline (`_sl_ref`/`_ratio`) is now kept current in mode 1 (police-only) too, via
      `_update_baseline()`, not just inside `_limit_drop_cap()` (mode 2 only) — previously flipping
      AutoSpeedReduce 1→2 mid-drive could resume off a baseline captured possibly hours earlier.
  F_uninit  anchoring/seeding is skipped entirely while `v_cruise_initialized` is False (cruise never
      set — `v_cruise`/`v_cruise_set` are the `V_CRUISE_UNSET` sentinel, ~145 km/h) — anchoring off
      that would inflate `_ratio` and seed `_cap_out` far above the real set.
"""
import json
import math
import time

from openpilot.common.swaglog import cloudlog

MPH_TO_MS = 0.44704
MILE_M = 1609.344

# police
POLICE_MARGIN = 5.0 * MPH_TO_MS          # target = posted limit + 5 mph
POLICE_ENGAGE_S = 30.0                   # LATCH on once the report is <= ~30 s of driving ahead
# NO no-limit fallback (driver directive 2026-07-15, city 15-mph hold): without a posted limit there is
# no basis to pick a target — a guessed trim (old: anchor - 10 mph) capped the car to 15 mph in city
# traffic when the map limit dropped out mid-approach. No limit -> no cap.
# limit-drop
MIN_DROP_FRAC = 0.95                     # ignore < 5% limit dips (GPS/OSM noise) — no cap
SANE_MAX_SL = 90.0 * MPH_TO_MS           # reject implausible posted limits (>90 mph) as garbage
# shared
MIN_CAP = 10.0 * MPH_TO_MS               # never auto-slow below ~10 mph via this feature (garbage reject)
READ_S = 1.0                             # param read cadence
# smoothness (driver directive 2026-07-15, the "wild horse" city ride): the map limit flickered
# valid<->unknown at ~1 Hz, so the cap target STEPPED between two values every second and the truck
# surged/braked chasing it. Three guards make the cap a smooth ceiling instead of a square wave:
SL_HOLD_S = 5.0                          # hold the last valid posted limit through brief map dropouts
CAP_SLEW = 1.0                           # m/s per s — emitted cap RAMPS toward its target, never steps
RELEASE_S = 2.0                          # cap sources must stay clear this long before the cap releases
# speedadjust-exec2pnw: the stock-ACC button-management publish (mem-param only; never touches the
# op-long return value)
PUB_THROTTLE_S = 0.25                    # publish cadence — matches icbm2pnw's IcbmTarget cadence,
                                          # well inside the executor's STALE_LIMIT_S=2.0
RESTORE_WINDOW_S = 45.0                  # bounded SET+ walk-back window after a cap clears — matches
                                          # icbm2pnw's own restore-episode window convention


class SpeedAdjustController:
  def __init__(self, CP, params=None):
    import platform
    self.CP = CP
    if params is not None:
      self.params = params
    else:
      from openpilot.common.params import Params
      self.params = Params()
    try:
      from openpilot.common.params import Params as _P
      self.mem_params = _P("/dev/shm/params") if platform.system() != "Darwin" else self.params
    except Exception:
      self.mem_params = None
    self._long_ok = bool(getattr(CP, "openpilotLongitudinalControl", False))
    self._mode = 0
    self._sl = 0.0                # current posted limit (m/s); 0 = unknown
    self._sl_valid_t = -1e9       # monotonic time of the last VALID limit read (for the dropout hold)
    self._sl_ref = 0.0            # baseline limit (the limit we were last uncapped at)
    self._ratio = 0.0            # ANCHORED over-limit ratio (v_set/limit) captured at the baseline —
                                 #   NOT live v_cruise, so re-scrolling the set can't double-reduce
    self._police = None          # last LocationServices["police"] dict
    self._police_latched = False # once within the approach window, hold until the report clears
    self._last_read = -1e9
    self._engaged = False        # for engage/release logging only
    self._cap_out = None         # the SLEWED cap currently emitted (None = not capping)
    self._release_t = None       # when the cap sources first went clear (release debounce)
    self._last_t = None          # last cap() call time (for slew dt)
    # speedadjust-exec2pnw: stock-ACC button-management publish state (SpeedAdjustTarget mem-param).
    # Inert / never touched on any op-long car (self._long_ok True) -- see _publish_target().
    self._pub_ceiling = None     # driver's set latched at cap ENGAGE (icbm2pnw ceiling parity)
    self._restore_ceiling = None # active bounded-restore target (None = no restore in progress)
    self._restore_deadline = None  # monotonic deadline for the bounded restore window
    self._pub_last = -1e9        # publish throttle (monotonic)
    self._pub_active = False     # was the last SpeedAdjustTarget publish non-idle (need one more
                                  # publish to clear it to {} on the transition to idle)

  # ---- input reads (params only; ~1 Hz) -------------------------------------
  def _read_speed_limit(self) -> float:
    """Current posted limit (m/s), 0 = unknown. A limit that was valid within the last SL_HOLD_S is
    HELD through read dropouts — the map limit flickering valid<->unknown at ~1 Hz turned the cap
    target into a square wave (the "wild horse" ride). Brief unknown = continuity, not a change."""
    sl = 0.0
    if self.mem_params is not None:
      try:
        raw = self.mem_params.get("MapSpeedLimit", return_default=True)
        raw = raw.decode() if isinstance(raw, bytes) else raw
        sl = float(raw) if raw else 0.0
      except Exception:
        sl = 0.0
      if not math.isfinite(sl) or sl <= 0.0 or sl > SANE_MAX_SL:   # reject unknown / NaN / garbage-high
        sl = 0.0
    now = time.monotonic()
    if sl > 0.0:
      self._sl_valid_t = now
      return sl
    if now - self._sl_valid_t < SL_HOLD_S and self._sl > 0.0:
      return self._sl                        # brief dropout → hold the last valid limit
    return 0.0

  def _read_police(self):
    if self.mem_params is None:
      return None
    try:
      raw = self.mem_params.get("LocationServices", return_default=True)
      if isinstance(raw, (bytes, str)):
        raw = json.loads(raw)
      if isinstance(raw, dict):
        p = raw.get("police")
        return p if isinstance(p, dict) else None
    except Exception:
      pass
    return None

  def _read_inputs(self):
    try:
      self._mode = int(self.params.get("AutoSpeedReduce", return_default=True) or 0)
    except Exception:
      self._mode = 0
    self._sl = self._read_speed_limit()
    self._police = self._read_police()

  # ---- the two reduce-only sources ------------------------------------------
  def _police_cap(self, v_cruise: float, v_ego: float):
    """Target = posted limit + 5 mph. LATCHES on once the report is ~30 s of driving ahead and HOLDS
    until the report clears — so slowing down (which balloons time-to-report) can't drop the cap and
    surge you back up. Returns the target cruise (m/s) or None. NO posted limit → NO cap (driver
    directive 2026-07-15): without a limit there is no basis for a target — the old guessed trim
    capped the car to 15 mph in city traffic. The latch is kept independent of limit availability,
    so if the limit becomes known mid-approach the cap engages then."""
    p = self._police
    if not isinstance(p, dict) or p.get("state") != "alert":
      self._police_latched = False           # cleared/passed report → release
      return None
    try:
      dist_m = float(p.get("dist_mi", 0.0)) * MILE_M
    except (TypeError, ValueError):
      return None
    if not math.isfinite(dist_m) or dist_m <= 0.0:   # NaN/inf/garbage distance → don't act, don't latch
      return None
    ttr = dist_m / max(v_ego, 1.0)           # time-to-report at current speed
    if ttr <= POLICE_ENGAGE_S and not self._police_latched:
      self._police_latched = True            # entered the approach window → latch on
    if not self._police_latched:
      return None                            # not close enough yet — hold current speed
    if self._sl > 0.0:
      return self._sl + POLICE_MARGIN
    return None                              # no posted limit → no basis to pick a target → no cap

  def _update_baseline(self, v_cruise_set: float):
    """speedanchor2pnw (F3): keep the limit-drop baseline (_sl_ref/_ratio) current whenever the feature
    is ACTIVE (mode 1 or 2) — not just inside _limit_drop_cap(), which only ran in mode 2. Previously a
    police-only (mode 1) drive left _sl_ref/_ratio frozen; flipping AutoSpeedReduce 1→2 mid-drive then
    resumed off a baseline captured possibly hours earlier, producing a surprise cap on the switch tick.

    speedanchor2pnw (F2): anchors off `v_cruise_set` — the driver's raw, PRE-VTSC cruise set — not a
    VTSC-curve-reduced `v_cruise`. A curve happening to be active at the moment of re-anchor must not
    record a bogus-low ratio (which can silently disable the trim on the next real limit drop)."""
    sl = self._sl
    if sl > 0.0 and sl >= self._sl_ref:      # known limit, at/above baseline → uncapped; track it up
      self._sl_ref = sl
      self._ratio = v_cruise_set / sl        # anchor the over-limit ratio HERE (v_set/limit)

  def _limit_drop_cap(self):
    """Trim the driver's OVER-limit excess as the posted limit drops: cap = SL × (v_set/SL_ref), using
    the ratio anchored by _update_baseline() (already refreshed this tick, before this is called).
    Persistent while below the baseline; releases once _update_baseline() re-anchors on a rising limit.
    m/s or None.

    Two guards (Corvallis city fix 2026-07-14): (1) only trim a driver who was ABOVE the baseline limit
    (ratio >= 1) — a limit drop must NOT slow a law-abiding driver (set 30 in a 45 -> ratio 0.65 -> a drop
    to 25 wrongly scaled them to 16 mph); (2) NEVER cap below the posted limit (max floor). So this is a
    'trim your speeding when the limit falls' feature, not a 'slow everyone proportionally' one."""
    sl = self._sl
    if sl <= 0.0:
      return None                            # unknown limit → no cap, preserve the baseline + ratio
    if sl >= self._sl_ref:
      return None                            # at/above baseline → uncapped (already re-anchored above)
    if self._ratio < 1.0:
      return None                            # driver was at/UNDER the limit → a drop shouldn't slow them
    if sl / self._sl_ref > MIN_DROP_FRAC:    # < 5% drop → noise
      return None
    return max(sl, sl * self._ratio)         # proportional trim, but NEVER below the posted limit

  # ---- speedadjust-exec2pnw: stock-ACC button-management publish (mem-param side effect only) ----
  def _publish_target(self, target, ceiling=None, direction="dec") -> None:
    """Publish (or clear) the SpeedAdjustTarget mem-param for the shared stock-ACC button executor.
    Car-agnostic: no fingerprint/brand check here — gated ONLY on self._long_ok (this is the ONE
    choke point every call site funnels through, so a future call site can't forget the gate): a
    no-op on any op-long car, since nothing there ever reads this mem-param — the op-long return
    value from cap() is the only thing that ever steers those cars. target=None -> publish {} exactly
    once on the active->idle transition (never spammed every idle tick); otherwise throttled to
    PUB_THROTTLE_S. Best-effort: NEVER raises into the control path."""
    if self._long_ok or self.mem_params is None:
      return
    now = time.monotonic()
    if target is None:
      if self._pub_active:
        try:
          self.mem_params.put_nonblocking("SpeedAdjustTarget", {})
        except Exception:
          pass
        self._pub_active = False
        self._pub_last = now
      return
    if now - self._pub_last < PUB_THROTTLE_S:
      return
    self._pub_last = now
    self._pub_active = True
    try:
      payload = {"target": round(float(target), 2), "ceiling": round(float(ceiling), 2),
                 "ts": time.time()}  # noqa: TID251 -- wall clock heartbeat shared with the executor
      if direction == "inc":
        payload["dir"] = "inc"
      self.mem_params.put_nonblocking("SpeedAdjustTarget", payload)
    except Exception:
      pass

  @staticmethod
  def _driver_intervening(sm) -> bool:
    """speedadjust-exec2pnw: defense-in-depth abort for the restore window ONLY (mirrors icbm2pnw's
    "driver gas/brake or ACC-off in ANY phase" abort matrix entry). The shared executor's
    decide_press()/RestoreGuard already independently gate EVERY press on the same signals read
    directly off the real stock CAN state at the Ford carcontroller layer — this is a belt-and-
    suspenders check at the brain layer, not the only guard. sm may be None (unit tests / no
    SubMaster) or missing carState -> defensively "not intervening" (never raises)."""
    if sm is None:
      return False
    try:
      cs = sm['carState']
      return bool(cs.gasPressed or cs.brakePressed or not cs.cruiseState.enabled)
    except Exception:
      return False

  def _step_restore(self, now: float, sm) -> None:
    """Bookkeeping + publish for the bounded restore window (see module docstring). Called only from
    inside `cap()` while NOT actively capping. Cancels the moment: the window (RESTORE_WINDOW_S)
    expires, the car is op-long (belt-and-suspenders — callers already gate on `not self._long_ok`
    before starting a restore, so this only matters if that ever changes), or `sm['carState']` shows
    driver/ACC intervention."""
    if self._restore_ceiling is None:
      self._publish_target(None)
      return
    if self._long_ok or now > self._restore_deadline or self._driver_intervening(sm):
      self._restore_ceiling = None
      self._restore_deadline = None
      self._publish_target(None)
      return
    self._publish_target(self._restore_ceiling, self._restore_ceiling, "inc")

  # ---- the cap the planner folds (reduce-only) ------------------------------
  def cap(self, sm, v_cruise_set: float, v_cruise: float, v_ego: float, v_cruise_initialized: bool) -> float:
    """
    v_cruise_set: the driver's raw, PRE-VTSC (pre-any-other-cap) cruise set (m/s) — used ONLY to anchor
      the limit-drop ratio/baseline and to seed the cap slew (speedanchor2pnw F2). Using this instead of
      the already-VTSC-reduced `v_cruise` stops a curve in effect at anchor/engage time from poisoning
      the ratio or the slew seed.
    v_cruise: the EFFECTIVE cruise ceiling after upstream reduce-only caps (VTSC etc.) have already been
      applied — the emitted cap remains reduce-only bounded against THIS value, unchanged from before.
    v_cruise_initialized: False before the driver has ever set cruise (v_cruise*/set are the
      V_CRUISE_UNSET sentinel, ~145 km/h) — anchoring/seeding is skipped entirely in that case
      (speedanchor2pnw F_uninit), same treatment as idle.
    """
    now = time.monotonic()
    dt = min(max(now - self._last_t, 0.0), 0.5) if self._last_t is not None else 0.0
    self._last_t = now
    if now - self._last_read >= READ_S:
      self._last_read = now
      self._read_inputs()

    # speedanchor2pnw (F_uninit, Fable-caught): an uninitialized cruise is not a real driver set —
    # anchoring/seeding off the ~145 km/h sentinel would inflate _ratio (silently no-ops the next real
    # limit drop) and seed _cap_out far above the eventual real set. No bookkeeping, clean passthrough.
    if not v_cruise_initialized:
      self._engaged = False
      self._police_latched = False
      self._cap_out = None
      self._release_t = None
      # speedadjust-exec2pnw: an uninitialized cruise can't be a valid restore ceiling either.
      self._pub_ceiling = None
      self._restore_ceiling = None
      self._restore_deadline = None
      self._publish_target(None)
      return v_cruise

    if self._mode == 0:
      self._engaged = False
      self._police_latched = False
      self._cap_out = None
      self._release_t = None
      self._pub_ceiling = None
      self._restore_ceiling = None
      self._restore_deadline = None
      self._sl_ref = self._sl                # keep baseline current while idle (no stale drop on enable)
      # speedanchor2pnw (F2): anchor off the raw set, not a VTSC-curve-reduced v_cruise.
      self._ratio = (v_cruise_set / self._sl) if self._sl > 0.0 else 0.0
      self._publish_target(None)
      return v_cruise

    # speedadjust-exec2pnw: the cap math below now runs for EVERY mode != 0 car, regardless of
    # self._long_ok — plannerd runs on every car, so the target was always being computed for
    # op-long cars and silently discarded otherwise. The RETURN value at the bottom stays gated on
    # self._long_ok exactly as before (unchanged op-long behavior); the NEW mem-param publish is a
    # pure side effect that only ever fires when not self._long_ok (see _publish_target()).
    caps = []
    pc = self._police_cap(v_cruise_set, v_ego)   # modes 1 and 2
    if pc is not None:
      caps.append(pc)
    # speedanchor2pnw (F3): re-anchor the limit-drop baseline whenever the feature is active (mode 1 or
    # 2) — not just when the drop-cap itself is computed (mode 2 only) — so it never goes stale across
    # an AutoSpeedReduce 1→2 switch.
    self._update_baseline(v_cruise_set)
    if self._mode >= 2:                       # limit-drop cap itself: mode 2 only
      lc = self._limit_drop_cap()
      if lc is not None:
        caps.append(lc)

    if not caps:
      # release DEBOUNCE: sources must stay clear for RELEASE_S before the cap lets go — an
      # engage/release oscillation (flapping source) was half of the "wild horse" ride.
      if self._cap_out is None:
        # not currently capping (and the debounce already ran its course, if any) -- offer/continue
        # any pending bounded restore.
        self._step_restore(now, sm)
        return v_cruise
      if self._release_t is None:
        self._release_t = now
      if now - self._release_t < RELEASE_S:
        self._publish_target(self._cap_out, self._pub_ceiling, "dec")  # still capping through debounce
        return max(0.0, min(v_cruise, self._cap_out))   # hold the last cap through the debounce window
      # debounce elapsed -> the cap fully releases. speedadjust-exec2pnw: hand off to a bounded SET+
      # restore back to the ceiling this cap latched at engage (stock-ACC only -- _step_restore() is a
      # no-op on any op-long car since _pub_ceiling is never consulted there).
      if not self._long_ok and self._pub_ceiling is not None:
        self._restore_ceiling = self._pub_ceiling
        self._restore_deadline = now + RESTORE_WINDOW_S
      self._cap_out = None
      self._release_t = None
      self._pub_ceiling = None
      if self._engaged:
        self._engaged = False
        cloudlog.info("speedadjust: released -> cruise")
      self._step_restore(now, sm)
      return v_cruise

    self._release_t = None
    # speedadjust-exec2pnw: a NEW cap always preempts any in-progress restore (DEC ALWAYS WINS, same
    # principle icbm2pnw's episode machine uses for its own new-cap-vs-restore conflicts).
    self._restore_ceiling = None
    self._restore_deadline = None
    target = max(MIN_CAP, min(caps))          # floor rejects garbage-low targets
    # SLEW: the emitted cap RAMPS toward its target instead of stepping — a step target made the MPC
    # chase a square wave. Seeds at the driver's RAW set on engage (no initial jump — see speedanchor2pnw
    # F2 below) and moves <= CAP_SLEW m/s per s in BOTH directions; the MPC still bounds the actual decel
    # on top of this.
    if self._cap_out is None:
      # speedanchor2pnw (F2): seed from v_cruise_set (the driver's raw, PRE-VTSC set), not v_cruise (the
      # current EFFECTIVE ceiling). Seeding from `v_cruise` meant a VTSC curve cap active exactly at
      # engage time seeded the slew from curve speed; after the curve ended and VTSC released instantly,
      # this cap kept crawling up from curve speed at CAP_SLEW instead of already sitting near the real
      # target ("won't come back up to my set"). `out` below is still bounded by the effective `v_cruise`
      # on every tick, so seeding high here can never cause a jump — the min() catches it immediately.
      self._cap_out = v_cruise_set
      # speedadjust-exec2pnw: latch the ceiling at cap ENGAGE (icbm2pnw ceiling-latch parity) — the
      # value a later bounded restore may walk back up to, never higher.
      self._pub_ceiling = v_cruise_set
    if target < self._cap_out:
      self._cap_out = max(target, self._cap_out - CAP_SLEW * dt)
    else:
      self._cap_out = min(target, self._cap_out + CAP_SLEW * dt)
    out = max(0.0, min(v_cruise, self._cap_out))   # reduce-only — never raise above the driver's set
    # speedadjust-exec2pnw: publish the SAME slewed cap value the op-long path would consume — the
    # stock-ACC executor taps toward the identical target, just via buttons instead of the MPC.
    self._publish_target(self._cap_out, self._pub_ceiling, "dec")
    if not self._engaged:
      self._engaged = True
      cloudlog.info(f"speedadjust: engaged mode={self._mode} cap={out:.1f} (police={pc} sl={self._sl:.1f})")
    # speedadjust-exec2pnw: the RETURN value stays exactly what it always was -- only an op-long car
    # ever gets a non-neutral v_cruise back from this function. Stock-ACC cars are steered solely via
    # the SpeedAdjustTarget mem-param publish above, never through this return path.
    return out if self._long_ok else v_cruise
