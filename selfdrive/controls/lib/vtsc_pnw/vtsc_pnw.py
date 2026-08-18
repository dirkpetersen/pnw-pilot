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


def apex_turn_direction(model, lookahead_max_s: float = C.LOOKAHEAD_MAX_S) -> int:
  """descentcurve2pnw: turn DIRECTION of the model path's apex (sharpest upcoming point):
  +1 = LEFT, -1 = right, 0 = straight/unknown. Sign convention verified in-tree: openpilot is
  left-positive (latcontrol_torque "left is positive in this convention"; positive steeringAngleDeg
  and positive desired curvature = left), and modelV2.orientationRate.z is the plan yaw rate with
  z UP, so orientationRate.z > 0 = turning LEFT. model_curve_state/curvatures_from_model take
  abs() and LOSE the sign — this walks the same points keeping it. Direction is only claimed for a
  real bend (apex curvature >= CUE_MIN_CURVATURE, ~R2300 m) so lane noise can't flip it. Pure-ish;
  bad data -> 0."""
  try:
    orz = list(model.orientationRate.z)
    vx = list(model.velocity.x)
    tb = list(model.orientationRate.t)
  except Exception:
    return 0
  best_k, best_z = 0.0, 0.0
  n = min(len(orz), len(vx), len(tb))
  for i in range(n):
    if tb[i] > lookahead_max_s:
      break
    k = abs(orz[i]) / max(vx[i], 0.1)
    if k > best_k:
      best_k, best_z = k, orz[i]
  if best_k < C.CUE_MIN_CURVATURE or best_z == 0.0:
    return 0
  return 1 if best_z > 0.0 else -1


def _haversine_m(lat1, lon1, lat2, lon2) -> float:
  """Great-circle distance in metres (pure). Local copy so this module stays self-contained/testable."""
  r = 6371000.0
  p1, p2 = math.radians(lat1), math.radians(lat2)
  dp = math.radians(lat2 - lat1)
  dl = math.radians(lon2 - lon1)
  a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
  return 2 * r * math.asin(min(1.0, a ** 0.5))


def _rel_bearing_deg(lat1, lon1, lat2, lon2, ref_deg) -> float:
  """|relative bearing| in degrees from a heading of `ref_deg` to the point (lat2, lon2). Local copy
  for the same reason as _haversine_m: this module stays self-contained and unit-testable."""
  dl = math.radians(lon2 - lon1)
  p1, p2 = math.radians(lat1), math.radians(lat2)
  y = math.sin(dl) * math.cos(p2)
  x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
  brg = math.degrees(math.atan2(y, x)) % 360.0
  return abs((brg - ref_deg + 540.0) % 360.0 - 180.0)


def required_decel(v_ego: float, v_target: float, dist: float) -> float:
  """Constant decel (m/s^2) needed to reach v_target by `dist` ahead. 0 if no slowing needed / dist<=0.
  Used to decide when REGEN alone won't make a sharp curve (-> allow last-resort firmer braking). Pure."""
  if dist <= 0.0 or v_target >= v_ego:
    return 0.0
  return (v_ego * v_ego - v_target * v_target) / (2.0 * dist)


# --- mapcurv2pnw: curvature measured from the map POLYLINE (telemetry only, 2026-08-18) ----------
# mapd's `velocity` field is a lossy round-trip of the very thing we need: it emits
# sqrt(a_lat/k) with an unpublished, INCONSISTENT a_lat, which is why a genuine 409 m curve and a
# sweeper the driver takes at 85 mph land in the same 24-27 m/s band and cannot be told apart. The
# lat/lon points in the SAME message carry the real geometry. Applying the already-tuned
# A_LAT_TARGET to a measured radius reproduces all three 2026-08-18 reference cases:
#     Woodland  R=409 m -> 72 mph (measured need ~72)
#     left turn R=208 m -> 51 mph (measured need <63)
#     I-84 sweeper R>=577 m -> 85 mph (must NOT slow)
# Vision cannot substitute: a gradual 90->72 needs ~347 m of runway and LOOKAHEAD_MAX_S=8.0 s gives
# ~322 m at 40 m/s, so vision can only ever produce a firm save.
#
# SPACING IS THE CATCH (measured live on-device: node gaps ran 57 m to 4072 m on one 21-point
# polyline). Menger curvature over raw consecutive triplets is meaningless across a 4 km gap, so
# triplets are ACCEPTED ONLY when every leg falls inside [_K_MIN_LEG_M, _K_MAX_LEG_M]. Sparse or
# bunched stretches simply yield no measurement rather than a fabricated one.
#
# TELEMETRY ONLY for now. Nothing here feeds control until a drive has been replayed against the
# three reference cases above.
_K_MIN_LEG_M = 25.0      # below this, GPS/OSM node jitter dominates the angle
_K_MAX_LEG_M = 300.0     # above this, a triplet no longer describes a local curve
_K_MAX_POINTS = 256      # bound the scan; the polyline is untrusted input


def polyline_curvature(points, cur_lat, cur_lon, horizon_m, a_lat=C.A_LAT_TARGET, cur_bearing=None):
  """Largest reliably-measurable curvature on the map polyline ahead, as (k, dist_m, v_safe).

  Returns (k, dist_m, v_safe, n_ok, ahead). k is 1/m (0.0 = nothing measurable), dist_m the distance
  to that point, v_safe = sqrt(a_lat/k) in m/s (inf when k is 0), n_ok the number of triplets that
  passed the spacing gate, and ahead whether the measured point is in front of the car.

  n_ok exists because k == 0.0 is ambiguous: it means EITHER "the road is straight" OR "the geometry
  was unmeasurable" (a real R=1500 m curve with ~366 m node gaps -- well inside the live-measured
  57-4072 m range -- returns 0.0 exactly like a straight). Replay cannot judge whether this lever is
  trustworthy without knowing which. n_ok == 0 means no measurement was possible.

  `ahead` exists because mapd publishes the whole current way, including nodes BEHIND the car, and
  the distance is unsigned -- so a curve just exited would otherwise keep 'advising' a slowdown while
  receding and contaminate exactly the apex-timing comparisons the replay needs. With no bearing the
  flag is True (unknown) rather than silently dropping data.

  Pure; never raises; always returns finite k/dist.

  PATH ORDER IS TAKEN FROM THE INPUT, never re-derived. mapd emits the polyline in path order (the
  sibling most_binding_map_curve relies on the same thing). An earlier version sorted by straight-line
  distance from the car "to be safe"; that is actively wrong on any road that bends back -- a 267 deg
  switchback reorders to [0,1,2,3,10,4,9,5,8,6,7], and the resulting zig-zag triplets get thrown out
  by the spacing gate, silently UNDER-reporting the sharpest curves on exactly the roads that need
  them most. Filtering by horizon preserves relative order, so the sequence stays intact."""
  best_k, best_d, vs, n_ok, best_ahead = 0.0, 0.0, float('inf'), 0, True
  try:
    if not points or cur_lat is None or cur_lon is None:
      return 0.0, 0.0, float('inf'), 0, True
    pts = []
    for p in points[:_K_MAX_POINTS]:
      try:
        la, lo = float(p["latitude"]), float(p["longitude"])
      except (KeyError, TypeError, ValueError):
        continue
      if not (math.isfinite(la) and math.isfinite(lo)):
        continue
      d = _haversine_m(cur_lat, cur_lon, la, lo)
      if 0.0 <= d <= horizon_m:
        pts.append((d, la, lo))                        # order preserved -- deliberately NOT sorted
    if len(pts) < 3:
      return 0.0, 0.0, float('inf'), 0, True
    # Consecutive leg lengths, computed ONCE each: leg[i] spans pts[i] -> pts[i+1]. Recomputing them
    # per triplet did ~3x the haversine work inside a 20 Hz control-loop caller.
    legs = [_haversine_m(pts[i][1], pts[i][2], pts[i+1][1], pts[i+1][2]) for i in range(len(pts)-1)]
    for i in range(1, len(pts) - 1):
      ab, bc = legs[i-1], legs[i]
      if not (_K_MIN_LEG_M <= ab <= _K_MAX_LEG_M and _K_MIN_LEG_M <= bc <= _K_MAX_LEG_M):
        continue                                       # unusable spacing -> no measurement
      (_, la0, lo0), (db, la1, lo1), (_, la2, lo2) = pts[i-1], pts[i], pts[i+1]
      # n_ok counts triplets that passed the SPACING gate -- it must increment here, before the
      # collinearity check below. A perfectly straight road has zero area and would otherwise
      # `continue` past the counter, reporting n_ok == 0 and making "straight" indistinguishable
      # from "unmeasurable" -- precisely the ambiguity this field exists to resolve.
      n_ok += 1
      ca = _haversine_m(la0, lo0, la2, lo2)
      if ab * bc * ca <= 0.0:
        continue
      # Menger curvature: k = 4*area / (|ab| |bc| |ca|); area from the cross product about b.
      cosb = math.cos(math.radians(la1))
      ax, ay = (lo0 - lo1) * 111320.0 * cosb, (la0 - la1) * 111320.0
      cx, cy = (lo2 - lo1) * 111320.0 * cosb, (la2 - la1) * 111320.0
      area2 = abs(ax * cy - ay * cx)                   # 2 * triangle area
      if area2 <= 0.0:
        continue
      k = 2.0 * area2 / (ab * bc * ca)
      if k > best_k:
        best_k, best_d = k, db
        best_ahead = True
        if cur_bearing is not None:
          try:
            best_ahead = _rel_bearing_deg(cur_lat, cur_lon, la1, lo1, float(cur_bearing)) <= 90.0
          except (TypeError, ValueError):
            best_ahead = True
    # INSIDE the try: a_lat comes from the tune dict, so a bad value must not escape as an exception
    # into plannerd. (Left outside previously -- a negative or non-numeric a_lat would have raised
    # straight through the guard and taken lateral control down with it.)
    if best_k > 1e-9 and math.isfinite(a_lat) and a_lat > 0.0:
      vs = math.sqrt(a_lat / best_k)
  except Exception:
    return 0.0, 0.0, float('inf'), 0, True             # telemetry must never break the control loop
  if not (math.isfinite(best_k) and math.isfinite(best_d)):
    return 0.0, 0.0, float('inf'), 0, True
  return best_k, best_d, vs, n_ok, best_ahead


def most_binding_map_curve(points, cur_lat, cur_lon, v_ego: float, horizon_m: float,
                           a_decel: float = C.A_DECEL, finish_s: float = C.APEX_FINISH_S,
                           sharp_v: float = C.SHARP_CURVE_V, speed_scale: float = 1.0,
                           v_cruise_cap: float = float('inf'),
                           min_slowdown: float = C.MAP_MIN_SLOWDOWN,
                           floor_limit: float = 0.0, floor_depth: float = C.MAP_FLOOR_DEPTH):
  """sharpcurve2pnw: scan pfeiferj map path points {latitude,longitude,velocity} within horizon_m and
  return (v_target, dist, is_sharp) of the curve whose decel-limited brake cap is the LOWEST right now
  — i.e. the one to start slowing for first. This is the distance-based lookahead: a far sharp curve
  has a high (non-binding) envelope until we're close enough, a near curve binds sooner — picking by
  envelope (not nearest / not min-speed) chooses the right one over the FULL ~500 m mapd horizon.
  The MTSC target scale (speed_scale) and clamp (v_cruise_cap) are applied to each point BEFORE the
  envelope so the SELECTED curve matches the value actually used downstream (no selection/use mismatch).
  Returns (v_target, dist, is_sharp, v_raw, floored): v_target is the effective (scaled+clamped, and
  possibly floored -- see below) target, v_raw is the UNSCALED mapd target for that same curve, and
  floored says the curvefloor2pnw minimum-slowdown floor was applied to at least one candidate.
  (0.0, inf, False, 0.0, False) if no point / no data. Pure."""
  if not points or cur_lat is None or cur_lon is None:
    return 0.0, float('inf'), False, 0.0, False
  best_cap = float('inf')
  best_v = 0.0
  best_d = float('inf')
  best_sharp = False
  best_raw = 0.0
  best_floored = False
  for p in points:
    try:
      d = _haversine_m(cur_lat, cur_lon, p["latitude"], p["longitude"])
      tv = float(p["velocity"])
    except (KeyError, TypeError, ValueError):
      continue
    if tv <= 0.0 or not (0.0 < d <= horizon_m):
      continue
    # sharpcurve2pnw iter2 (2026-07-08): TIERED scale — trust mapd on tight curves (raw target low),
    # override it on sweepers (raw target high, where mapd runs absurdly conservative). The shared
    # helper keeps this fold and CES's sharp-curve classification in agreement. Tiering engages only
    # on the production call (speed_scale == MAP_SPEED_SCALE); explicit/test callers keep flat scaling.
    scale = C.tiered_map_scale(tv) if math.isclose(speed_scale, C.MAP_SPEED_SCALE) else speed_scale
    tv_eff = min(tv * scale, v_cruise_cap)   # MTSC scale + clamp applied before selection
    # curvefloor2pnw (2026-08-18, I-5 Woodland takeover): the MTSC scale must never ERASE a curve mapd
    # actually flagged. Measured: raw 27.0 m/s (60 mph) * tiered scale 1.742 = 47 m/s (105 mph),
    # clamped to the 40.2 m/s set speed, then discarded downstream as "not a meaningful slowdown" ->
    # VTSC commanded nothing and the car entered a 409 m curve at 90 mph.
    # This MUST happen per-point HERE, before the envelope, not after selection: the envelope ranks on
    # tv_eff, so once several points clamp to v_cruise_cap they tie and the NEAREST wins. mapd emits a
    # target for every node with finite curvature, so an ordinary gentle node in front of the real
    # curve would be selected instead and its (high) raw would suppress a post-selection floor
    # entirely -- the fix would silently not fire on exactly the road that motivated it.
    # Keeping it here also preserves this function's documented invariant: scale+clamp applied INSIDE
    # the selection so the SELECTED curve matches the value used downstream.
    floored_pt = False
    shallow_floor = False
    if math.isfinite(v_cruise_cap):
      notch = v_cruise_cap - min_slowdown
      if tv < notch <= tv_eff:               # raw says slow down, scaled says don't
        # Target mapd's OWN raw advisory, but bounded on BOTH sides:
        #   never SHALLOWER than the minimum notch (else it wouldn't clear the gate downstream), and
        #   never DEEPER than floor_limit -- the posted speed limit on a freeway. That deeper bound is
        #   not new authority: the controller ALREADY refuses to trim below the posted limit
        #   (see the SPEED-LIMIT FLOOR in vtsc_controller.cap()), so this can never ask for more
        #   slowdown than the system already permits. With no limit known, floor_limit is 0.0 and the
        #   behaviour degrades to the flat minimum notch.
        # 2026-08-18 second event: a flat notch measured from the SET speed is far too weak whenever
        # the driver is already below set -- at 63 mph with a 70 set it bought 3.1 mph, against a
        # steering-rate saturation that scales with speed. mapd said 51 and the limit was 50.
        if floor_limit > 0.0 and floor_depth > 0.0:
          deep = min(max(tv, floor_limit), notch)          # mapd's advisory, never below the limit
          tv_eff = notch + (deep - notch) * min(max(floor_depth, 0.0), 1.0)
        else:
          tv_eff = notch                                    # flat minimum notch (MAP_FLOOR_DEPTH=0)
        floored_pt = True
        # Only the SHALLOW target (still at the minimum notch) is a synthetic trim that must stay
        # regen-only. Once MAP_FLOOR_DEPTH deepens it toward mapd's advisory it is a real slowdown
        # and MUST keep its firm-braking flag -- otherwise raising the knob hands the car a deep
        # target while denying it the authority to reach it, which is worse than not deepening at
        # all. (A raw target of 25-29 m/s is both "sharp" and floorable, so this is reachable.)
        shallow_floor = tv_eff >= notch - 1e-6
    if tv_eff <= 0.0:
      continue
    cap = brake_cap_for_apex(tv_eff, d, v_ego, a_decel, finish_s)
    if cap < best_cap:
      # is_sharp = the RAW (unscaled) target is physically sharp. Classifying on tv_eff would let the
      # 1.12x scale inflate a genuinely sharp 28 m/s curve to 31.4 and DROP its sharp flag -> the
      # last-resort firmer brake would be denied while we target the inflated entrance speed (overshoot).
      # is_sharp unlocks SHARP_A_DECEL_MAX (2.8 m/s^2 friction braking) downstream. A SHALLOW floored
      # target is a synthetic ~10 mph trim and must be reached with regen alone -- driver directive
      # 2026-08-18: "we only want very very gradual slowdowns". A DEEPER floored target keeps the
      # flag (see shallow_floor above).
      best_cap, best_v, best_d, best_sharp = cap, tv_eff, d, (tv < sharp_v and not shallow_floor)
      # F4: report whether the SELECTED curve was floored, not whether any candidate was -- a global
      # accumulator would mislabel the telemetry whenever a non-selected point happened to be floored.
      best_floored = floored_pt
      best_raw = tv                            # UNSCALED mapd target for the chosen curve
  return best_v, best_d, best_sharp, best_raw, best_floored


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
