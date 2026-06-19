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
import json
import os
import time

from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_CTRL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.ces_xnor import ces_xnor_constants as C

# Persistent, append-only "each adoption" trail. Lives OUTSIDE /data/openpilot so it survives the
# boot overlay-swap AND swaglog rotation (a long drive rotates swaglog and would lose early events).
# One JSON line per CES mode transition, with GPS so we can map where each adoption happened.
CES_EVENT_LOG = "/data/dirk/ces_events.jsonl"


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


def _haversine_m(lat1, lon1, lat2, lon2) -> float:
  """Great-circle distance in metres (pure)."""
  import math
  r = 6371000.0
  p1, p2 = math.radians(lat1), math.radians(lat2)
  dp = math.radians(lat2 - lat1); dl = math.radians(lon2 - lon1)
  a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
  return 2 * r * math.asin(min(1.0, a ** 0.5))


def upcoming_curve(target_velocities, cur_lat, cur_lon, v_ego, lookahead_s) -> tuple[float, float]:
  """From pfeiferj's MapTargetVelocities (list of {latitude, longitude, velocity}) + current
  position, return (min_target_velocity, distance) of the most-binding upcoming curve within the
  lookahead distance (v_ego * lookahead_s). Returns (0.0, inf) if none / no data. Pure & testable."""
  if not target_velocities or cur_lat is None or cur_lon is None:
    return 0.0, float('inf')
  horizon = max(v_ego, 1.0) * lookahead_s
  best_v, best_d = 0.0, float('inf')
  for p in target_velocities:
    try:
      d = _haversine_m(cur_lat, cur_lon, p["latitude"], p["longitude"])
      tv = float(p["velocity"])
    except (KeyError, TypeError, ValueError):
      continue
    if 0.0 < d <= horizon:
      # most-binding = lowest target speed ahead within the horizon
      if best_v == 0.0 or tv < best_v:
        best_v, best_d = tv, d
  return best_v, best_d


class Condition:
  """A debounced boolean signal: raw bool -> filtered -> compared to THRESHOLD. The filter is driven
  by the MEASURED loop dt (selfdrived runs at 100 Hz / DT_CTRL, not the model rate) so FILTER_TAU is a
  real time constant — using a fixed DT_MDL here made the debounce run 5x too fast (instant flapping)."""
  def __init__(self):
    self.f = FirstOrderFilter(0.0, C.FILTER_TAU, DT_CTRL)
    self.active = False

  def update(self, raw: bool, dt: float = DT_CTRL) -> bool:
    if dt != self.f.dt:
      self.f.dt = dt
      self.f.update_alpha(C.FILTER_TAU)
    self.f.update(1.0 if raw else 0.0)
    self.active = self.f.x >= C.THRESHOLD
    return self.active

  def reset(self):
    self.f.x = 0.0
    self.active = False


def _accelerate_zone(s) -> bool:
  """PURE: True when we're slow but should be ACCELERATING into open road, so Experimental's timid
  e2e acceleration would hurt — keep Chill instead. Covers the two cases:
    - highway on-ramp merge (open road ahead, set speed = highway >> ramp speed)
    - stop&go where the lead has pulled away leaving a big gap (catch back up at Chill briskness)
  Requires open road ahead AND a set speed meaningfully above current speed. Only gates `lowSpeed`."""
  open_ahead = (not s["has_lead"]) or (s["lead_drel"] > C.GAP_OPEN_M
                                       and s["lead_vlead"] >= s["v_ego"] - C.LEAD_PULLAWAY_MARGIN)
  want_faster = s["v_set"] > 0.0 and (s["v_set"] - s["v_ego"]) > C.ACCEL_ZONE_DV
  return open_ahead and want_faster


def decide_active(s) -> tuple[bool, str]:
  """PURE decision core (no state, no filtering): given a signals dict-like `s`, return
  (any_condition_active, status). Used by both the live controller (post-filter) and the unit tests.

  Expected keys (all SI, primitives):
    v_ego, has_lead, lead_vlead, lead_drel, blinker,
    map_target_v, map_target_dist, curve_lat_accel_vision, time_to_curve,
    model_should_stop,
    toggles: curves/stops/low_speed/lead (bool enables)
  """
  t = s["toggles"]
  v = s["v_ego"]

  # 1) curve — map (primary, ~10 s) OR vision (fallback, ~3.5 s)
  if t["curves"] and v > C.CRUISING_SPEED:
    # MAP: pfeiferj MapTargetVelocities gives a safe curve speed ahead. Trip when an upcoming
    # target speed within the lookahead is meaningfully (>MIN_SLOWDOWN) below current speed.
    map_curve = (s["map_target_v"] > 0.0
                 and (v - s["map_target_v"]) > C.CURVE_MAP_MIN_SLOWDOWN
                 and 0.0 < s["map_target_dist"] / max(v, 1.0) < C.CURVE_MAP_LOOKAHEAD_S)
    # VISION fallback: predicted lateral accel over the (short) model horizon.
    vision_curve = (abs(s["curve_lat_accel_vision"]) > C.CURVE_LAT_ACCEL_ENTER
                    and s["time_to_curve"] < C.CURVE_VISION_LOOKAHEAD_S
                    and not s["blinker"])
    if map_curve or vision_curve:
      return True, "curve"

  # 2) stop light / stop sign — model predicts a stop, not currently following a lead
  if t["stops"] and s["model_should_stop"] and not s["has_lead"]:
    return True, "stop"

  # 3) low speed (city / complex / construction) — lead-aware threshold. TWO exceptions, both
  #    learned from the drive log (only ever REMOVE Experimental, so safe):
  #    (a) highway gate: skip on a road whose OSM speed limit is high — slow-but-following on a
  #        highway is normal Chill cruising, not a complex zone.
  #    (b) accelerate-zone: skip when we should be accelerating into open road (on-ramp merge /
  #        lead pulled away) — Experimental's timid e2e acceleration is bad there.
  thr = C.CES_SPEED_LEAD if s["has_lead"] else C.CES_SPEED
  on_highway = s["spd_lim"] >= C.LOWSPEED_HWY_GATE
  if t["low_speed"] and 1.0 <= v < thr and not on_highway and not _accelerate_zone(s):
    return True, "lowSpeed"

  # 4) slow / stopped lead — closing on a slower/stopped lead -> let e2e do the smooth decel
  if t["lead"] and s["has_lead"]:
    if (v - s["lead_vlead"]) > C.SLOW_LEAD_DV or s["lead_vlead"] < C.STOPPED_LEAD_V:
      return True, "slowLead"

  return False, "chill"


def _clamp01(x: float) -> float:
  return 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)


def curve_closeness(s) -> tuple[float, str]:
  """PURE, display-only: 'how close are we to tripping Experimental for a curve', 0.0..1.0, plus
  which half drives it ('map' / 'vision' / ''). 1.0 == at/over the entry threshold (switch imminent).
  Mirrors the curve branch of `decide_active` but as a continuous ratio for the on-screen feedback —
  it does NOT make the decision. 0.80 ~= "very close", 0.99 ~= "about to switch", >=1.0 == tripping."""
  t = s["toggles"]
  v = s["v_ego"]
  if not t["curves"] or v <= C.CRUISING_SPEED:
    return 0.0, ""
  # MAP half: how far the upcoming safe curve speed sits below us vs the slowdown that trips it,
  # but only while that curve is within the lookahead time.
  map_close = 0.0
  mv, md = s["map_target_v"], s["map_target_dist"]
  if mv > 0.0 and 0.0 < md / max(v, 1.0) < C.CURVE_MAP_LOOKAHEAD_S:
    map_close = _clamp01((v - mv) / C.CURVE_MAP_MIN_SLOWDOWN)
  # VISION half: predicted lateral accel vs the entry threshold, within the (short) vision horizon.
  vis_close = 0.0
  if s["time_to_curve"] < C.CURVE_VISION_LOOKAHEAD_S and not s["blinker"]:
    vis_close = _clamp01(abs(s["curve_lat_accel_vision"]) / C.CURVE_LAT_ACCEL_ENTER)
  if map_close >= vis_close:
    return map_close, ("map" if map_close > 0.0 else "")
  return vis_close, "vision"


def decision_telemetry(s) -> dict:
  """PURE, display-only: a compact snapshot for the on-screen CES overlay. Reports the binding
  reason, the curve 'closeness' as a 0..100 %, and the upcoming map-curve preview (target speed +
  distance). Built from the SAME signals dict `decide_active` consumes, so the overlay can never
  disagree with the live decision."""
  raw_active, reason = decide_active(s)
  cpct, csrc = curve_closeness(s)
  md = s["map_target_dist"]
  return {
    "rawActive": bool(raw_active),
    "reason": reason,
    "curvePct": int(round(cpct * 100)),
    "curveSrc": csrc,
    "mapV": round(float(s["map_target_v"]), 1),
    "mapDist": round(float(md), 0) if md != float('inf') else 0.0,
    "vEgo": round(float(s["v_ego"]), 1),
    # accelerate-zone + the signals that drive it (also logged per event for later tuning)
    "accelZone": _accelerate_zone(s),
    "hwyGate": s.get("spd_lim", 0.0) >= C.LOWSPEED_HWY_GATE,   # lowSpeed suppressed: on a highway
    "vSet": round(float(s["v_set"]), 1),
    "dRel": round(float(s["lead_drel"]), 0),
    "vLead": round(float(s["lead_vlead"]), 1),
    "aEgo": round(float(s.get("a_ego", 0.0)), 2),
    "gas": bool(s.get("gas", False)),
  }


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

  def update_decision(self, signals: dict, dt: float = DT_CTRL) -> str:
    """Advance the state machine one cycle from an extracted `signals` dict (see decide_active).
    `dt` is the MEASURED loop period (selfdrived runs at 100 Hz) so the dwell/debounce are real
    seconds. Separated from `update(sm)` so it is unit-testable without cereal messages."""
    raw_active, status = decide_active(signals)
    cond_active = self._cond.update(raw_active, dt)   # debounced (real-time)
    self._dwell += dt

    if not self._is_experimental:
      # enter Experimental once the debounced condition is active AND we've been in Chill at least
      # the re-entry cooldown (de-flap: stops the instant snap-back that caused the stop&go sawtooth)
      if cond_active and self._dwell >= C.CHILL_MIN_DWELL_S:
        self._is_experimental = True
        self._status = status
        self._dwell = 0.0
    else:
      # stay Experimental; return to Chill only when the condition cleared (sustained via filter)
      # AND we've held Experimental at least EXP_MIN_DWELL_S
      if status != "chill":
        self._status = status      # keep showing the active reason
      if not cond_active and self._dwell >= C.EXP_MIN_DWELL_S:
        self._is_experimental = False
        self._status = "chill"
        self._dwell = 0.0
    return self.mode()


# ---------------------------------------------------------------------------
# Phase 2/3 — live wiring. Runs in selfdrived (which publishes the effective
# experimentalMode → both the planner AND the top-right icon follow it).
# Behavior-neutral: experimental_request() returns False whenever CES is
# disabled/non-Tesla, so selfdrived's `manual OR request` == manual == upstream.
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


def _signals_from(car_state, lead, model, toggles: dict, map_target_v: float, map_target_dist: float,
                  spd_lim: float = 0.0) -> dict:
  """Build the decision primitives from STOCK messages (carState, radarState.leadOne, modelV2)
  plus the map-curve result (map_target_v/dist) and the OSM speed limit (spd_lim, m/s, for the
  lowSpeed highway gate). Defensive: missing/odd data falls back to 'nothing happening' (stay Chill)."""
  v_ego = float(car_state.vEgo)

  has_lead = bool(getattr(lead, 'status', False))
  lead_vlead = float(getattr(lead, 'vLead', 0.0)) if has_lead else 0.0
  lead_drel = float(getattr(lead, 'dRel', 0.0)) if has_lead else 0.0

  try:
    orz = list(model.orientationRate.z); vx = list(model.velocity.x); tb = list(model.orientationRate.t)
    vis_acc, ttc = vision_curve_lat_accel(orz, vx, tb, v_ego)
  except Exception:
    vis_acc, ttc = 0.0, 10.0
  try:
    model_should_stop = bool(model.action.shouldStop)
  except Exception:
    model_should_stop = False

  # set speed (openpilot's v_cruise, km/h on carState.vCruise) -> m/s; 255 is the unset sentinel.
  v_set_kph = float(getattr(car_state, 'vCruise', 0.0))
  v_set = v_set_kph * CV.KPH_TO_MS if 0.0 < v_set_kph < C.V_SET_MAX_KPH else 0.0

  return {
    "v_ego": v_ego, "has_lead": has_lead, "lead_vlead": lead_vlead, "lead_drel": lead_drel,
    "blinker": bool(car_state.leftBlinker or car_state.rightBlinker),
    "map_target_v": map_target_v, "map_target_dist": map_target_dist,   # map half (MapTargetVelocities)
    "curve_lat_accel_vision": vis_acc, "time_to_curve": ttc,            # vision fallback
    "model_should_stop": model_should_stop, "toggles": toggles,
    "v_set": v_set,                                                     # accelerate-zone (set-speed gap)
    "spd_lim": float(spd_lim),                                         # OSM speed limit (lowSpeed highway gate)
    "a_ego": float(getattr(car_state, 'aEgo', 0.0)),                   # logged for verification
    "gas": bool(getattr(car_state, 'gasPressed', False)),             # logged for verification
  }


class CESController:
  """Live wrapper used by selfdrived. Owns the state machine + ~1 Hz param refresh + the 3-state
  button (CESButtonState: 0=CES, 1=forced Chill, 2=forced Experimental) + the map-curve read.
  Gated on openpilotLongitudinalControl (NOT brand — available on every car, like the stock
  Experimental toggle). experimental_request() returns False when disabled → behavior-neutral."""
  def __init__(self, CP, params=None):
    import platform
    from openpilot.common.params import Params
    self.CP = CP
    self.params = params or Params()
    # pfeiferj mapd writes MapTargetVelocities/LastGPSPosition to the in-memory param store
    try:
      self.mem_params = Params("/dev/shm/params") if platform.system() != "Darwin" else self.params
    except Exception:
      self.mem_params = None
    self._sm = ConditionalExperimentalSwitching()
    self._enabled = False
    self._button = C.BTN_CES
    self._toggles = {"curves": True, "stops": True, "low_speed": True, "lead": True}
    self._map_targets = []          # cached MapTargetVelocities (refreshed ~1 Hz)
    self._cur_lat = self._cur_lon = self._cur_bearing = None
    self._speed_limit = 0.0         # OSM speed limit (m/s, 0 = none) from mapd
    self._frame = 0
    # telemetry / logging (display + diagnostics only — never gates control)
    self.msg = self._disabled_msg()  # latest CesState snapshot (selfdrived publishes it → qlog/rlog)
    self._last_mode = "off"         # last logged mode: off / chill / experimental
    self._tele_last = 0.0           # monotonic stamp of last CESStatus publish
    self._tick_last = 0.0           # monotonic stamp of last breadcrumb tick
    self._last_decide_t = None      # monotonic stamp of last state-machine step (for real dt)
    self._event_log_ok = False      # persistent "each adoption" trail (CES_EVENT_LOG)
    try:
      os.makedirs(os.path.dirname(CES_EVENT_LOG), exist_ok=True)
      self._event_log_ok = True
    except Exception:
      self._event_log_ok = False
    # CES is meaningful only when openpilot owns longitudinal (same gate as ExperimentalMode).
    self._long_ok = bool(getattr(CP, 'openpilotLongitudinalControl', False))

  def _read_params(self):
    if self._frame % max(1, int(1.0 / DT_CTRL)) == 0:   # ~1 Hz (selfdrived steps at 100 Hz / DT_CTRL)
      try:
        self._enabled = self._long_ok and self.params.get_bool("ConditionalExperimentalSwitching")
      except Exception:
        self._enabled = False
      if self._enabled:
        self._toggles = _toggles_from_params(self.params)
        try:
          self._button = int(self.params.get("CESButtonState", return_default=True) or 0)
        except Exception:
          self._button = C.BTN_CES
        self._read_map()
    self._frame += 1

  def _read_map(self):
    """Refresh map-curve inputs + GPS + OSM speed limit from the pfeiferj mem params (defensive — any
    failure => no map curve, vision fallback still works). GPS + speed limit are read regardless of
    the curves toggle because the event log wants them at all times."""
    if self.mem_params is None:
      self._map_targets = []
      return
    # map curve targets — only when the curve condition is enabled
    if self._toggles.get("curves", True):
      try:
        self._map_targets = self.mem_params.get("MapTargetVelocities", return_default=True) or []
      except Exception:
        self._map_targets = []
    else:
      self._map_targets = []
    # GPS (lat/lon/bearing) — always (map-curve distance + logging)
    try:
      pos = self.mem_params.get("LastGPSPosition", return_default=True)
      if isinstance(pos, (bytes, str)):
        pos = json.loads(pos)
      self._cur_lat = float(pos["latitude"]); self._cur_lon = float(pos["longitude"])
      self._cur_bearing = float(pos.get("bearing", 0.0))
    except Exception:
      self._cur_lat = self._cur_lon = self._cur_bearing = None
    # OSM speed limit (m/s; 0 = none) — for the coarse highway guess in the log
    try:
      sl = self.mem_params.get("MapSpeedLimit", return_default=True)
      self._speed_limit = float(sl) if sl not in (None, "", b"") else 0.0
    except Exception:
      self._speed_limit = 0.0

  def enabled(self) -> bool:
    return self._enabled

  def status(self) -> str:
    return self._sm.status()

  def experimental_request(self, car_state, sm) -> bool:
    """True if CES wants Experimental this cycle. Reads params; advances the state machine.
    Safe to call always — returns False whenever CES is disabled (behavior-neutral)."""
    self._read_params()
    if not self._enabled:
      if self._last_mode != "off":
        cloudlog.info("CES disabled (master OFF / no openpilot long) -> Chill baseline")
        self._last_mode = "off"
      self._sm.reset()
      self.msg = self._disabled_msg()
      return False

    # Build the decision signals every cycle while enabled — even in the forced button modes —
    # so the on-screen overlay always reflects what CES sees (curve %, upcoming curve preview).
    sig = None
    try:
      lead = sm['radarState'].leadOne
      model = sm['modelV2']
      v_ego = float(car_state.vEgo)
      mtv, mtd = upcoming_curve(self._map_targets, self._cur_lat, self._cur_lon, v_ego, C.CURVE_MAP_LOOKAHEAD_S)
      sig = _signals_from(car_state, lead, model, self._toggles, mtv, mtd, self._speed_limit)
    except Exception:
      sig = None

    # measured loop period — selfdrived steps at ~100 Hz; never assume a fixed DT (was the 5x bug)
    now_t = time.monotonic()
    dt = (now_t - self._last_decide_t) if self._last_decide_t is not None else DT_CTRL
    self._last_decide_t = now_t
    dt = min(max(dt, 1e-3), 0.5)           # clamp first call / scheduling hiccups

    if self._button == C.BTN_CHILL:        # forced Chill
      self._sm.reset()
      want = False
    elif self._button == C.BTN_EXP:        # forced full Experimental
      want = True
    elif sig is not None:                  # BTN_CES: condition ladder decides
      want = self._sm.update_decision(sig, dt) == "experimental"
    else:
      want = False

    self._publish_status(sig, want)
    return want

  def _publish_status(self, sig, want: bool) -> None:
    """Log mode transitions and publish a throttled CESStatus snapshot to the in-memory param store
    for the on-screen overlay. Display/diagnostics only — never affects the returned decision."""
    mode = "experimental" if want else "chill"
    tele = decision_telemetry(sig) if sig is not None else {
      "reason": "noData", "curvePct": 0, "curveSrc": "", "mapV": 0.0, "mapDist": 0.0, "vEgo": 0.0,
    }
    tele["mode"] = mode
    tele["button"] = int(self._button)
    tele["enabled"] = True
    # mapd diagnostics so the overlay can always show what mapd is up to (curve half is map-driven):
    tele["mapPts"] = len(self._map_targets)                       # MapTargetVelocities points cached
    tele["gps"] = self._cur_lat is not None and self._cur_lon is not None  # LastGPSPosition fix present

    # cesState snapshot for selfdrived to publish into qlog/rlog (set BEFORE the throttled returns
    # below so it always reflects the latest cycle, independent of the /dev/shm publish cadence).
    self.msg = self._build_msg(tele, mode)

    # (a) transition ("adopt") — one record per chill<->experimental change, cloudlog + event file.
    if mode != self._last_mode:
      cloudlog.info("CES %s->%s button=%d reason=%s curve=%d%%(%s) vEgo=%.1f vSet=%.1f az=%s mapV=%.1f",
                    self._last_mode, mode, self._button, tele.get("reason"),
                    tele.get("curvePct", 0), tele.get("curveSrc", ""), tele.get("vEgo", 0.0),
                    tele.get("vSet", 0.0), tele.get("accelZone"), tele.get("mapV", 0.0))
      rec = self._event_record("adopt", tele)
      rec["from"], rec["to"] = self._last_mode, mode
      self._append_event(rec)
      self._last_mode = mode
    else:
      # (b) heartbeat ("tick") — ~1 Hz breadcrumb so the WHOLE drive's GPS track + state is captured
      # (lets us place every adoption on the route and apply the highway / 300 ft buffer in analysis).
      now2 = time.monotonic()
      if now2 - self._tick_last >= C.TICK_S:
        self._tick_last = now2
        self._append_event(self._event_record("tick", tele))

    # ~5 Hz publish to /dev/shm/params (put a dict -> JSON; nonblocking so the safety loop never waits)
    if self.mem_params is None:
      return
    now = time.monotonic()
    if now - self._tele_last < 0.2:
      return
    self._tele_last = now
    try:
      self.mem_params.put_nonblocking("CESStatus", tele)
    except Exception:
      pass

  def _event_record(self, kind: str, tele: dict) -> dict:
    """Build one rich, flat record for the persistent CES_EVENT_LOG. `kind` is "adopt" (a CES mode
    transition) or "tick" (a ~1 Hz breadcrumb). Includes GPS (lat/lon/bearing), OSM speed limit, a
    coarse highway guess, the accelerate-zone decision + its inputs (vSet/dRel/vLead/aEgo/gas), and
    the curve/map diagnostics — everything needed to verify behavior against the route later."""
    vego = float(tele.get("vEgo") or 0.0)
    hwy = (self._speed_limit >= C.HWY_SPEED_LIMIT) or (vego >= C.HWY_VEGO)  # coarse; authoritative = GPS+OSM+300ft in analysis
    return {
      "t": round(time.time(), 1),  # noqa: TID251 -- wall clock, for route/time correlation
      "ev": kind, "mode": tele.get("mode"), "reason": tele.get("reason"), "button": int(self._button),
      "vEgo": tele.get("vEgo"), "vSet": tele.get("vSet"), "aEgo": tele.get("aEgo"), "gas": tele.get("gas"),
      "accelZone": tele.get("accelZone"),
      "curvePct": tele.get("curvePct"), "curveSrc": tele.get("curveSrc"),
      "mapV": tele.get("mapV"), "mapDist": tele.get("mapDist"), "mapPts": tele.get("mapPts"),
      "dRel": tele.get("dRel"), "vLead": tele.get("vLead"),
      "gps": tele.get("gps"), "lat": self._cur_lat, "lon": self._cur_lon, "bearing": self._cur_bearing,
      "spdLim": round(self._speed_limit, 1), "hwy": bool(hwy),
    }

  def _append_event(self, rec: dict) -> None:
    """Append one JSON line to the persistent CES_EVENT_LOG (append-only, outside the overlay so it
    survives reboot + swaglog rotation). Best-effort; never breaks control."""
    if not self._event_log_ok:
      return
    try:
      with open(CES_EVENT_LOG, "a") as f:
        f.write(json.dumps(rec) + "\n")
    except Exception:
      pass

  @staticmethod
  def _disabled_msg() -> dict:
    """CesState snapshot when CES is off / no openpilot longitudinal — every field at its zero so the
    log unambiguously shows 'CES not running' (mode='off', enabled=False)."""
    return {
      "enabled": False, "mode": "off", "button": 0, "reason": "", "rawActive": False,
      "curvePct": 0, "curveSrc": "", "mapV": 0.0, "mapDist": 0.0, "mapPts": 0, "gpsValid": False,
      "vEgo": 0.0, "vSet": 0.0, "aEgo": 0.0, "gas": False, "accelZone": False, "hwyGate": False,
      "dRel": 0.0, "vLead": 0.0, "spdLimit": 0.0, "latitude": 0.0, "longitude": 0.0,
    }

  def _build_msg(self, tele: dict, mode: str) -> dict:
    """Flatten the per-cycle decision telemetry into the CesState fields. Robust with .get defaults
    so the noData fallback tele (sig is None) still produces a valid, fully-populated snapshot."""
    return {
      "enabled": True,
      "mode": mode,
      "button": int(self._button),
      "reason": str(tele.get("reason", "")),
      "rawActive": bool(tele.get("rawActive", False)),
      "curvePct": int(tele.get("curvePct", 0) or 0),
      "curveSrc": str(tele.get("curveSrc", "")),
      "mapV": float(tele.get("mapV", 0.0) or 0.0),
      "mapDist": float(tele.get("mapDist", 0.0) or 0.0),
      "mapPts": int(len(self._map_targets)),
      "gpsValid": self._cur_lat is not None and self._cur_lon is not None,
      "vEgo": float(tele.get("vEgo", 0.0) or 0.0),
      "vSet": float(tele.get("vSet", 0.0) or 0.0),
      "aEgo": float(tele.get("aEgo", 0.0) or 0.0),
      "gas": bool(tele.get("gas", False)),
      "accelZone": bool(tele.get("accelZone", False)),
      "hwyGate": bool(tele.get("hwyGate", False)),
      "dRel": float(tele.get("dRel", 0.0) or 0.0),
      "vLead": float(tele.get("vLead", 0.0) or 0.0),
      "spdLimit": float(self._speed_limit),
      "latitude": float(self._cur_lat) if self._cur_lat is not None else 0.0,
      "longitude": float(self._cur_lon) if self._cur_lon is not None else 0.0,
    }
