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

from openpilot.selfdrive.controls.lib.vtsc_xnor import vtsc_constants as C


def v_safe(curvature: float, a_lat: float = C.A_LAT_TARGET) -> float:
  """Safe speed (m/s) that holds lateral accel <= `a_lat` at path `curvature` (1/m). Since
  a_lat = v^2 * curvature, v_safe = sqrt(a_lat / curvature). Straight road -> inf (no limit)."""
  if curvature <= C.MIN_CURVATURE:
    return float('inf')
  return math.sqrt(a_lat / curvature)


def curve_speed_target(curvatures, distances, v_cruise: float,
                       a_lat: float = C.A_LAT_TARGET, a_decel: float = C.A_DECEL,
                       v_min: float = C.V_MIN) -> float:
  """PURE VTSC core. Given per-point predicted-path curvature `curvatures[i]` (1/m) at look-ahead
  distance `distances[i]` (m), return the cruise-speed CAP (m/s) that both:
    (a) holds lateral accel <= a_lat through each curve      -> v_safe(curvature), and
    (b) is reachable from NOW by decel-limited braking       -> envelope sqrt(v_safe^2 + 2*a_decel*d),
  so braking begins ~early (≈110 m out at the defaults) instead of AT the curve.

  Returns v_cruise when nothing binds; otherwise min(v_cruise, max(cap, v_min)). NEVER raises speed."""
  cap = float(v_cruise)
  for k, d in zip(curvatures, distances, strict=False):
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
                 a_decel_max=C.A_DECEL_MAX, a_relax=1.5):
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


def vtsc_from_model(model, v_cruise: float, a_lat: float = C.A_LAT_TARGET,
                    a_decel: float = C.A_DECEL, v_min: float = C.V_MIN) -> float:
  """Convenience: curvatures_from_model() -> curve_speed_target(). Returns the cap (m/s), v_cruise
  when no curve or on bad data."""
  curvs, dists = curvatures_from_model(model)
  if not curvs:
    return float(v_cruise)
  return curve_speed_target(curvs, dists, v_cruise, a_lat, a_decel, v_min)
