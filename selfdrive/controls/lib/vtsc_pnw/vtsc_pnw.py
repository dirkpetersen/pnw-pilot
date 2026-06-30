"""
VTSC — Vision Turn Speed Control (xnor)   ⚠️ PHASE 1: PURE CORE, NOT WIRED

Computes a cruise-speed CAP so the car decelerates to a safe speed for an upcoming curve, derived
from the driving model's predicted-path curvature. This replaces CES's curve trigger, which only
switched to Experimental and never actually braked — on the I-5 Terwilliger curve it held 70 mph
through a 2.6 m/s^2 curve and the driver had to intervene (see /home/dp/gh/comma/VTSC.md).

PURE: no cereal, no car, no control output. `curve_speed_target` is the unit-tested core and only
ever RETURNS a speed cap <= v_cruise. The longitudinal-planner hook (Phase 2) applies it. SI units.
Default OFF; the wrapper/toggle live in Phase 2.
"""
import math

from openpilot.selfdrive.controls.lib.vtsc_pnw import vtsc_constants as C


def v_safe(curvature: float, a_lat: float = C.A_LAT_TARGET) -> float:
  """Safe speed (m/s) that holds lateral accel <= `a_lat` at path `curvature` (1/m). Since
  a_lat = v^2 * curvature, v_safe = sqrt(a_lat / curvature). Straight road -> inf (no limit)."""
  if curvature <= C.MIN_CURVATURE:
    return float('inf')
  return math.sqrt(a_lat / curvature)


def curve_speed_target(curvatures, distances, v_cruise: float,
                       a_lat: float = C.A_LAT_TARGET, a_decel: float = C.A_DECEL,
                       v_min: float = C.V_MIN, min_dist: float = 0.0) -> float:
  """PURE VTSC core. Given per-point predicted-path curvature `curvatures[i]` (1/m) at look-ahead
  distance `distances[i]` (m), return the cruise-speed CAP (m/s) that both:
    (a) holds lateral accel <= a_lat through each curve      -> v_safe(curvature), and
    (b) is reachable from NOW by decel-limited braking       -> envelope sqrt(v_safe^2 + 2*a_decel*d),
  so braking begins ~early instead of AT the curve.

  `min_dist`: points closer than this are COMMITTED (the car is about to be there — braking now can't
  change the speed there) and are skipped. This is the apex-release: brake entrance->apex only; once
  the apex slides inside min_dist the cap relaxes to what the remaining path needs, so the car
  accelerates out of the curve. min_dist=0 keeps the old bind-everything behavior.

  Returns v_cruise when nothing binds; otherwise min(v_cruise, max(cap, v_min)). NEVER raises speed."""
  cap = float(v_cruise)
  for k, d in zip(curvatures, distances, strict=False):
    if d < min_dist:
      continue                                    # committed point (at/behind the apex) -> never brake for it
    vs = v_safe(k, a_lat)
    if vs == float('inf'):
      continue
    v_allow = math.sqrt(vs * vs + 2.0 * a_decel * max(d, 0.0))   # decel envelope from here to the curve
    if v_allow < cap:
      cap = v_allow
  if cap >= v_cruise:
    return float(v_cruise)                        # no curve binds -> no limit
  return min(float(v_cruise), max(cap, v_min))    # floor it, and never above cruise


def apply_limits(prev_applied, target, v_cruise, dt,
                 a_decel_max=C.A_DECEL_MAX, a_relax=C.A_RELAX):
  """PURE: rate-limit the applied cap from `prev_applied` toward `target` (both m/s). Bounds how fast
  the cap may DROP (commanded decel <= a_decel_max) and how fast it EASES back up when the curve
  clears (a_relax). Returns the new applied cap, never above v_cruise. `prev_applied=None` -> start at
  v_cruise. This is the safety rate-limiter on top of the (already smooth) decel-envelope cap."""
  if prev_applied is None:
    prev_applied = v_cruise
  if target < prev_applied:
    applied = max(target, prev_applied - a_decel_max * dt)     # braking: bounded decel
  else:
    applied = min(target, v_cruise, prev_applied + a_relax * dt)   # clearing: ease back gently
  return min(v_cruise, applied)


def sharpest_ahead(curvatures, distances):
  """Return (max_curvature 1/m, distance_m_at_that_point) over the predicted points — i.e. the APEX
  (the tightest part of the upcoming path). (0.0, -1.0) if the path is straight. Pure."""
  best_k, best_d = 0.0, -1.0
  for k, d in zip(curvatures, distances, strict=False):
    if k > best_k:
      best_k, best_d = k, d
  return best_k, best_d


def brake_cap_for_apex(v_curve_safe: float, apex_dist: float, v_ego: float,
                       a_decel: float = C.A_DECEL, finish_s: float = C.APEX_FINISH_S) -> float:
  """Speed cap (m/s) such that decel-limited braking reaches `v_curve_safe` `finish_s` seconds BEFORE
  the apex (so slowing is DONE before the apex and we can accelerate out). As apex_dist shrinks the
  cap falls toward v_curve_safe; the controller's rate-limiter brakes harder near the end if needed.
  Pure. `v_curve_safe`=inf (straight) -> inf."""
  if v_curve_safe == float('inf'):
    return float('inf')
  d_finish = max(apex_dist - max(v_ego, 0.0) * finish_s, 0.0)
  return math.sqrt(v_curve_safe * v_curve_safe + 2.0 * a_decel * d_finish)


def curvatures_from_model(model):
  """Extract (curvatures, distances) from modelV2's predicted path, over the FULL horizon up to
  LOOKAHEAD_MAX_S (NOT CES's 3.5 s gate — that short gate was the 'too late' bug at Terwilliger).
    curvature_i = |orientationRate.z_i| / max(velocity.x_i, eps);  distance_i = position.x_i (m ahead).
  Defensive: missing/odd data -> ([], []). Pure-ish (only reads the message)."""
  try:
    orz = list(model.orientationRate.z)
    vx = list(model.velocity.x)
    px = list(model.position.x)
    tb = list(model.orientationRate.t)
  except Exception:
    return [], []
  curvs, dists = [], []
  n = min(len(orz), len(vx), len(px), len(tb))
  for i in range(n):
    if tb[i] > C.LOOKAHEAD_MAX_S:
      break
    curvs.append(abs(orz[i]) / max(vx[i], 0.1))
    dists.append(max(px[i], 0.0))
  return curvs, dists


def _haversine_m(lat1, lon1, lat2, lon2) -> float:
  """Great-circle distance in metres (pure). Local copy so this module stays self-contained/testable."""
  r = 6371000.0
  p1, p2 = math.radians(lat1), math.radians(lat2)
  dp = math.radians(lat2 - lat1)
  dl = math.radians(lon2 - lon1)
  a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
  return 2 * r * math.asin(min(1.0, a ** 0.5))


def required_decel(v_ego: float, v_target: float, dist: float) -> float:
  """Constant decel (m/s^2) needed to reach v_target by `dist` ahead. 0 if no slowing needed / dist<=0.
  Used to decide when REGEN alone won't make a sharp curve (-> allow last-resort firmer braking). Pure."""
  if dist <= 0.0 or v_target >= v_ego:
    return 0.0
  return (v_ego * v_ego - v_target * v_target) / (2.0 * dist)


def most_binding_map_curve(points, cur_lat, cur_lon, v_ego: float, horizon_m: float,
                           a_decel: float = C.A_DECEL, finish_s: float = C.APEX_FINISH_S,
                           sharp_v: float = C.SHARP_CURVE_V, speed_scale: float = 1.0,
                           v_cruise_cap: float = float('inf')):
  """sharpcurve2pnw: scan pfeiferj map path points {latitude,longitude,velocity} within horizon_m and
  return (v_target, dist, is_sharp) of the curve whose decel-limited brake cap is the LOWEST right now
  — i.e. the one to start slowing for first. This is the distance-based lookahead: a far sharp curve
  has a high (non-binding) envelope until we're close enough, a near curve binds sooner — picking by
  envelope (not nearest / not min-speed) chooses the right one over the FULL ~500 m mapd horizon.
  The MTSC target scale (speed_scale) and clamp (v_cruise_cap) are applied to each point BEFORE the
  envelope so the SELECTED curve matches the value actually used downstream (no selection/use mismatch).
  v_target is the effective (scaled+clamped) target. (0.0, inf, False) if no point / no data. Pure."""
  if not points or cur_lat is None or cur_lon is None:
    return 0.0, float('inf'), False
  best_cap = float('inf')
  best_v = 0.0
  best_d = float('inf')
  best_sharp = False
  for p in points:
    try:
      d = _haversine_m(cur_lat, cur_lon, p["latitude"], p["longitude"])
      tv = float(p["velocity"])
    except (KeyError, TypeError, ValueError):
      continue
    if tv <= 0.0 or not (0.0 < d <= horizon_m):
      continue
    tv_eff = min(tv * speed_scale, v_cruise_cap)   # MTSC scale + clamp applied before selection
    if tv_eff <= 0.0:
      continue
    cap = brake_cap_for_apex(tv_eff, d, v_ego, a_decel, finish_s)
    if cap < best_cap:
      # is_sharp = the RAW (unscaled) target is physically sharp. Classifying on tv_eff would let the
      # 1.12x scale inflate a genuinely sharp 28 m/s curve to 31.4 and DROP its sharp flag -> the
      # last-resort firmer brake would be denied while we target the inflated entrance speed (overshoot).
      best_cap, best_v, best_d, best_sharp = cap, tv_eff, d, (tv < sharp_v)
  return best_v, best_d, best_sharp


def twisty_section_cap(points, cur_lat, cur_lon, v_cruise: float, v_ego: float, horizon_m: float,
                       pitch=None, min_curves: int = C.TWISTY_MIN_CURVES,
                       slowdown: float = C.TWISTY_SLOWDOWN, min_factor: float = C.TWISTY_MIN_FACTOR,
                       descent_pitch: float = C.TWISTY_DESCENT_PITCH) -> float:
  """sharpcurve2pnw ("auto-lower the set on twisty descents"): trim the base cruise ONLY on a winding
  DOWNHILL — both (a) >= min_curves binding curves (target this far below cruise) within horizon_m AND
  (b) the road descending (pitch < descent_pitch rad). Holds a lower base cruise through the section so
  we don't re-accelerate to full set between blind curves. A FLAT twisty section keeps full speed
  (per-curve VTSC handles it) -> no speed lost where it isn't needed. Returns v_cruise unchanged
  otherwise (incl. no pitch data). Bounded by min_factor*v_cruise; only ever <= v_cruise. Pure."""
  if not points or cur_lat is None or cur_lon is None or v_cruise <= 0.0:
    return v_cruise
  if pitch is None or pitch >= descent_pitch:     # not a descent (or no pitch data) -> no trim, keep speed
    return v_cruise
  targets = []
  for p in points:
    try:
      d = _haversine_m(cur_lat, cur_lon, p["latitude"], p["longitude"])
      tv = float(p["velocity"])
    except (KeyError, TypeError, ValueError):
      continue
    if 0.0 < d <= horizon_m and 0.0 < tv < v_cruise - slowdown:
      targets.append(tv)
  if len(targets) < min_curves:
    return v_cruise
  # Use the RAW (unscaled) curve targets for the descent base on purpose: a twisty DOWNHILL is the more
  # dangerous case (gravity fights regen), so we hold a more conservative base than the per-curve scaled
  # targets carry on flat ground — this IS the "lower the set on twisty descents" behavior. The
  # min_factor floor (~0.82) bounds the trim either way, so it stays a modest, capped reduction.
  base = sum(targets) / len(targets)
  return max(v_cruise * min_factor, min(v_cruise, base))


def model_curve_state(model, v_cruise: float, a_lat: float = C.A_LAT_TARGET):
  """Read the model's predicted path and return the curve picture the apex state machine needs:
    (apex_curvature 1/m, apex_dist m, v_curve_safe m/s) where v_curve_safe = sqrt(a_lat/apex_curvature).
  Apex = the sharpest upcoming point. Straight road / bad data -> (0.0, -1.0, inf). Pure-ish."""
  curvs, dists = curvatures_from_model(model)
  if not curvs:
    return 0.0, -1.0, float('inf')
  k_apex, d_apex = sharpest_ahead(curvs, dists)
  return k_apex, d_apex, v_safe(k_apex, a_lat)
