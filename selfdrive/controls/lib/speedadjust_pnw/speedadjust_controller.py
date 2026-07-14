"""
speedadjust2pnw — automatic cruise-speed reduction for lower speed limits and police-ahead warnings.

`SpeedAdjustController.cap(sm, v_cruise, v_ego)` returns a possibly-lowered cruise speed (m/s) for the
longitudinal planner, exactly like `VTSCController` — **reduce-only, NEVER raises above v_cruise**. The
planner MPC bounds the decel rate, so a cap can never slam; it composes with VTSC/MTSC via `min()`.

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
POLICE_NO_LIMIT_TRIM = 10.0 * MPH_TO_MS  # no posted limit known → trim the set by 10 mph instead
# limit-drop
MIN_DROP_FRAC = 0.95                     # ignore < 5% limit dips (GPS/OSM noise) — no cap
SANE_MAX_SL = 90.0 * MPH_TO_MS           # reject implausible posted limits (>90 mph) as garbage
# shared
MIN_CAP = 10.0 * MPH_TO_MS               # never auto-slow below ~10 mph via this feature (garbage reject)
READ_S = 1.0                             # param read cadence


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
    self._sl_ref = 0.0            # baseline limit (the limit we were last uncapped at)
    self._ratio = 0.0            # ANCHORED over-limit ratio (v_set/limit) captured at the baseline —
                                 #   NOT live v_cruise, so re-scrolling the set can't double-reduce
    self._police = None          # last LocationServices["police"] dict
    self._police_latched = False # once within the approach window, hold until the report clears
    self._police_anchor = 0.0    # v_cruise captured when the police cap latched (for the no-limit trim)
    self._last_read = -1e9
    self._engaged = False        # for engage/release logging only

  # ---- input reads (params only; ~1 Hz) -------------------------------------
  def _read_speed_limit(self) -> float:
    if self.mem_params is None:
      return 0.0
    try:
      raw = self.mem_params.get("MapSpeedLimit", return_default=True)
      raw = raw.decode() if isinstance(raw, bytes) else raw
      sl = float(raw) if raw else 0.0
    except Exception:
      return 0.0
    if not math.isfinite(sl) or sl <= 0.0 or sl > SANE_MAX_SL:   # reject unknown / NaN / garbage-high
      return 0.0
    return sl

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
    surge you back up. Returns the target cruise (m/s) or None. No posted limit → trim the set by 10 mph
    (anchored at latch time so re-scrolling the set can't compound)."""
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
      self._police_anchor = v_cruise         # anchor the set for the no-limit trim
    if not self._police_latched:
      return None                            # not close enough yet — hold current speed
    if self._sl > 0.0:
      return self._sl + POLICE_MARGIN
    return self._police_anchor - POLICE_NO_LIMIT_TRIM

  def _limit_drop_cap(self, v_cruise: float):
    """Hold the driver's over-limit RATIO as the posted limit drops: cap = SL × (v_set/SL_ref), with the
    ratio ANCHORED when last uncapped (not live v_cruise → no double-reduction if the set is re-scrolled).
    Persistent while below the baseline; releases (re-anchors) when the limit rises back. m/s or None."""
    sl = self._sl
    if sl <= 0.0:
      return None                            # unknown limit → no cap, preserve the baseline + ratio
    if sl >= self._sl_ref:
      self._sl_ref = sl                      # at/above baseline → uncapped; track baseline up and
      self._ratio = v_cruise / sl            #   anchor the over-limit ratio HERE (v_set/limit)
      return None
    if sl / self._sl_ref > MIN_DROP_FRAC:    # < 5% drop → noise
      return None
    return sl * self._ratio                  # = v_set_at_baseline × SL/SL_ref

  # ---- the cap the planner folds (reduce-only) ------------------------------
  def cap(self, sm, v_cruise: float, v_ego: float) -> float:
    now = time.monotonic()
    if now - self._last_read >= READ_S:
      self._last_read = now
      self._read_inputs()

    if self._mode == 0 or not self._long_ok:
      self._engaged = False
      self._police_latched = False
      self._sl_ref = self._sl                # keep baseline current while idle (no stale drop on enable)
      self._ratio = (v_cruise / self._sl) if self._sl > 0.0 else 0.0
      return v_cruise

    caps = []
    pc = self._police_cap(v_cruise, v_ego)   # modes 1 and 2
    if pc is not None:
      caps.append(pc)
    if self._mode >= 2:                       # limit-drop: mode 2 only
      lc = self._limit_drop_cap(v_cruise)
      if lc is not None:
        caps.append(lc)

    if not caps:
      if self._engaged:
        self._engaged = False
        cloudlog.info("speedadjust: released -> cruise")
      return v_cruise

    target = max(MIN_CAP, min(caps))          # floor rejects garbage-low targets
    out = max(0.0, min(v_cruise, target))     # reduce-only — never raise above the driver's set
    if not self._engaged:
      self._engaged = True
      cloudlog.info(f"speedadjust: engaged mode={self._mode} cap={out:.1f} (police={pc} sl={self._sl:.1f})")
    return out
