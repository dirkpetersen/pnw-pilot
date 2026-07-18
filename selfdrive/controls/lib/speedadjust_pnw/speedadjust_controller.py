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

Works in BOTH Chill and Experimental (it caps `v_cruise`, which both modes honour). Only acts when
openpilot controls longitudinal (op-long); returns v_cruise unchanged otherwise, so it's behaviour-
neutral by default. Inputs are read from params at ~1 Hz — `MapSpeedLimit` + `LocationServices` from
the /dev/shm mem store (written by the mapd bridge / location_servicesd) and `AutoSpeedReduce`
persistent — with **NO msgq subscriptions** (the background-subscriber cascade lesson).

Runs inside plannerd (20 Hz) via the planner's cap chain. Pure-ish: only param reads + monotonic time.

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
      return v_cruise

    if self._mode == 0 or not self._long_ok:
      self._engaged = False
      self._police_latched = False
      self._cap_out = None
      self._release_t = None
      self._sl_ref = self._sl                # keep baseline current while idle (no stale drop on enable)
      # speedanchor2pnw (F2): anchor off the raw set, not a VTSC-curve-reduced v_cruise.
      self._ratio = (v_cruise_set / self._sl) if self._sl > 0.0 else 0.0
      return v_cruise

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
        return v_cruise
      if self._release_t is None:
        self._release_t = now
      if now - self._release_t < RELEASE_S:
        return max(0.0, min(v_cruise, self._cap_out))   # hold the last cap through the debounce window
      self._cap_out = None
      self._release_t = None
      if self._engaged:
        self._engaged = False
        cloudlog.info("speedadjust: released -> cruise")
      return v_cruise

    self._release_t = None
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
    if target < self._cap_out:
      self._cap_out = max(target, self._cap_out - CAP_SLEW * dt)
    else:
      self._cap_out = min(target, self._cap_out + CAP_SLEW * dt)
    out = max(0.0, min(v_cruise, self._cap_out))   # reduce-only — never raise above the driver's set
    if not self._engaged:
      self._engaged = True
      cloudlog.info(f"speedadjust: engaged mode={self._mode} cap={out:.1f} (police={pc} sl={self._sl:.1f})")
    return out
