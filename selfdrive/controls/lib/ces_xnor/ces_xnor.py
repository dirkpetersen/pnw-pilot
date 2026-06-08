"""
CES — Conditional Experimental Switching (xnor)  ⚠️ NOT WIRED / NOT DEPLOYED

Decides per-cycle whether the longitudinal planner should run Chill (ACC/MPC) or Experimental
(blended e2e), keeping the car in Chill for steady cruising and flipping to Experimental only for
curves, stop lights/signs, low-speed/complex (incl. city), and closing on a slow/stopped lead.

Design + decisions: see /home/dp/gh/comma/CES.md. Key properties:
  - Default Chill; ANY condition -> Experimental; return to Chill only when ALL clear + sustained +
    min-dwell (hysteresis on every threshold).
  - Per-condition FirstOrderFilter debounce (THRESHOLD ~ 1 s) — no flapping.
  - Tesla-only, longitudinal-only (Experimental does NOT change steering), default OFF.
  - 3-state top-right button override: CES / forced-Chill / forced-Experimental.

SAFETY: this module is PURE DECISION LOGIC. It does not command the car. It must be wired into the
effective-experimental computation (selfdrived) only after review + on-road verification. It never
touches panda safety. The decision core (`decide_active`) takes primitives and is unit-tested.
"""
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.ces_xnor import ces_xnor_constants as C


def vision_curve_lat_accel(orientation_rate_z, velocity_x, timebase, v_ego):
  """FrogPilot-style vision curve detector: predicted lateral accel + time-to-curve over the model
  horizon. Returns (predicted_lat_accel m/s^2, time_to_curve s). Pure; lists must be equal length."""
  if not orientation_rate_z or not velocity_x or not timebase:
    return 0.0, 1.0
  n = min(len(orientation_rate_z), len(velocity_x), len(timebase))
  best_acc, best_t, best_abs = 0.0, 1.0, -1.0
  for i in range(n):
    lat = orientation_rate_z[i] * velocity_x[i]   # yaw_rate * speed = lateral accel
    if abs(lat) > best_abs:
      best_abs, best_acc, best_t = abs(lat), lat, timebase[i]
  return best_acc, max(best_t, 1.0)


class Condition:
  """A debounced boolean signal: raw bool -> filtered -> compared to THRESHOLD."""
  def __init__(self):
    self.f = FirstOrderFilter(0.0, C.FILTER_TAU, DT_MDL)
    self.active = False

  def update(self, raw: bool) -> bool:
    self.f.update(1.0 if raw else 0.0)
    self.active = self.f.x >= C.THRESHOLD
    return self.active

  def reset(self):
    self.f.x = 0.0
    self.active = False


def decide_active(s) -> tuple[bool, str]:
  """PURE decision core (no state, no filtering): given a signals dict-like `s`, return
  (any_condition_active, status). Used by both the live controller (post-filter) and the unit tests.

  Expected keys (all SI, primitives):
    v_ego, has_lead, lead_vlead, lead_drel, blinker,
    curve_lat_accel_map, dist_to_curve, curve_lat_accel_vision, time_to_curve,
    model_should_stop,
    toggles: curves/stops/low_speed/lead (bool enables)
  """
  t = s["toggles"]
  v = s["v_ego"]

  # 1) curve — map (primary, ~10 s) OR vision (fallback, ~3.5 s)
  if t["curves"] and v > C.CRUISING_SPEED:
    map_curve = (abs(s["curve_lat_accel_map"]) > C.CURVE_LAT_ACCEL_ENTER
                 and 0.0 < s["dist_to_curve"] / max(v, 1.0) < C.CURVE_MAP_LOOKAHEAD_S)
    vision_curve = (abs(s["curve_lat_accel_vision"]) > C.CURVE_LAT_ACCEL_ENTER
                    and s["time_to_curve"] < C.CURVE_VISION_LOOKAHEAD_S
                    and not s["blinker"])
    if map_curve or vision_curve:
      return True, "curve"

  # 2) stop light / stop sign — model predicts a stop, not currently following a lead
  if t["stops"] and s["model_should_stop"] and not s["has_lead"]:
    return True, "stop"

  # 3) low speed (city / complex / construction) — lead-aware threshold, NO highway gate
  thr = C.CES_SPEED_LEAD if s["has_lead"] else C.CES_SPEED
  if t["low_speed"] and 1.0 <= v < thr:
    return True, "lowSpeed"

  # 4) slow / stopped lead — closing on a slower/stopped lead -> let e2e do the smooth decel
  if t["lead"] and s["has_lead"]:
    if (v - s["lead_vlead"]) > C.SLOW_LEAD_DV or s["lead_vlead"] < C.STOPPED_LEAD_V:
      return True, "slowLead"

  return False, "chill"


class ConditionalExperimentalSwitching:
  """Live controller. Owns the per-condition filters + the mode state machine (min-dwell + sustained
  clear). `mode()` returns 'experimental'/'chill'; `update(sm, toggles)` is called each cycle."""

  def __init__(self):
    # one debounce filter per condition (entry) + one for the all-clear (exit)
    self._cond = Condition()        # "any condition active" (debounced)
    self._is_experimental = False
    self._dwell = 0.0               # s in current mode
    self._status = "chill"

  def reset(self):
    self._cond.reset()
    self._is_experimental = False
    self._dwell = 0.0
    self._status = "chill"

  def mode(self) -> str:
    return "experimental" if self._is_experimental else "chill"

  def status(self) -> str:
    return self._status

  def update_decision(self, signals: dict) -> str:
    """Advance the state machine one cycle from an extracted `signals` dict (see decide_active).
    Separated from `update(sm)` so it is unit-testable without cereal messages."""
    raw_active, status = decide_active(signals)
    cond_active = self._cond.update(raw_active)   # debounced
    self._dwell += DT_MDL

    if not self._is_experimental:
      # enter Experimental as soon as the debounced condition is active
      if cond_active:
        self._is_experimental = True
        self._status = status
        self._dwell = 0.0
    else:
      # stay Experimental; return to Chill only when condition cleared (sustained via filter)
      # AND min-dwell elapsed
      if status != "chill":
        self._status = status      # keep showing the active reason
      if not cond_active and self._dwell >= C.MIN_DWELL_S:
        self._is_experimental = False
        self._status = "chill"
        self._dwell = 0.0
    return self.mode()


# ---------------------------------------------------------------------------
# Phase 2 — live wiring helpers. Behavior-neutral: the planner only calls
# experimental_request() when enabled() is True (param OFF by default + Tesla-
# gated), so with CES off the planner's experimental flag is IDENTICAL to today.
# ---------------------------------------------------------------------------

def _toggles_from_params(params) -> dict:
  """Per-condition enables; default ON (the master switch is the real gate)."""
  def gb(k):
    try:
      return params.get_bool(k)
    except Exception:
      return True
  return {"curves": gb("CESCurves"), "stops": gb("CESStops"),
          "low_speed": gb("CESLowSpeed"), "lead": gb("CESLead")}


def _signals_from_sm(sm, toggles: dict) -> dict:
  """Extract the decision primitives from STOCK messages only (carState, radarState,
  modelV2). Defensive: any missing/odd data falls back to a 'nothing happening' value so
  the worst case is 'stay Chill'. NOTE: map-half (liveMapDataSP curvature) is NOT on this
  branch — curve_lat_accel_map is fed 0.0, so only the vision fallback fires for now."""
  cs = sm['carState']
  v_ego = float(cs.vEgo)

  lead = sm['radarState'].leadOne
  has_lead = bool(getattr(lead, 'status', False))
  lead_vlead = float(getattr(lead, 'vLead', 0.0)) if has_lead else 0.0
  lead_drel = float(getattr(lead, 'dRel', 0.0)) if has_lead else 0.0

  md = sm['modelV2']
  try:
    orz = list(md.orientationRate.z); vx = list(md.velocity.x); tb = list(md.orientationRate.t)
    vis_acc, ttc = vision_curve_lat_accel(orz, vx, tb, v_ego)
  except Exception:
    vis_acc, ttc = 0.0, 10.0
  try:
    model_should_stop = bool(md.action.shouldStop)
  except Exception:
    model_should_stop = False

  return {
    "v_ego": v_ego, "has_lead": has_lead, "lead_vlead": lead_vlead, "lead_drel": lead_drel,
    "blinker": bool(cs.leftBlinker or cs.rightBlinker),
    "curve_lat_accel_map": 0.0, "dist_to_curve": 0.0,   # map-half pending (separate branch)
    "curve_lat_accel_vision": vis_acc, "time_to_curve": ttc,
    "model_should_stop": model_should_stop, "toggles": toggles,
  }


class CESController:
  """Thin live wrapper used by the planner. Owns the state machine + ~1 Hz param refresh.
  enabled() is the single gate — when it's False the planner must NOT call this (so behavior
  is byte-identical to upstream)."""
  def __init__(self, CP, params=None):
    from openpilot.common.params import Params
    self.CP = CP
    self.params = params or Params()
    self._sm = ConditionalExperimentalSwitching()
    self._enabled = False
    self._toggles = {"curves": True, "stops": True, "low_speed": True, "lead": True}
    self._frame = 0
    self._is_tesla = (getattr(CP, 'brand', '') == 'tesla') and bool(CP.openpilotLongitudinalControl)

  def _read_params(self):
    # refresh ~1 Hz (planner runs at 1/DT_MDL Hz)
    if self._frame % max(1, int(1.0 / DT_MDL)) == 0:
      try:
        self._enabled = self._is_tesla and self.params.get_bool("ConditionalExperimentalSwitching")
      except Exception:
        self._enabled = False
      if self._enabled:
        self._toggles = _toggles_from_params(self.params)
    self._frame += 1

  def enabled(self) -> bool:
    return self._enabled

  def status(self) -> str:
    return self._sm.status()

  def experimental_request(self, sm) -> bool:
    """Returns True if CES wants Experimental this cycle. Only meaningful when enabled().
    Reads params + advances the state machine. Safe to call; returns False if disabled."""
    self._read_params()
    if not self._enabled:
      self._sm.reset()
      return False
    return self._sm.update_decision(_signals_from_sm(sm, self._toggles)) == "experimental"
