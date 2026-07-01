"""
Unit tests for the VTSC core — pure math, calibrated to the drive-#3 I-5 Terwilliger log:
the car held 70 mph through a ~415 m-radius curve (2.6 m/s^2 lateral) and did NOT slow, so the
driver intervened. VTSC must instead command a slowdown to a safe speed (~57 mph at A_LAT_TARGET=1.5)
and START braking ~100 m before the apex, not at it.
"""
import math

from openpilot.selfdrive.controls.lib.vtsc_pnw.vtsc_pnw import (
  v_safe, curve_speed_target, apply_limits, sharpest_ahead, brake_cap_for_apex,
  required_decel, most_binding_map_curve, twisty_section_cap)
from openpilot.selfdrive.controls.lib.vtsc_pnw import vtsc_constants as C

MPH = 0.44704
def mph(v):
  return v / MPH

TERW_KAPPA = 1.0 / 415   # Terwilliger apex curvature (radius ~415 m)
V70 = 70 * MPH


# ---- v_safe ----------------------------------------------------------------
def test_v_safe_terwilliger_apex():
  vs = v_safe(TERW_KAPPA, 1.5)
  assert 23.0 < vs < 27.0                          # ~52-60 mph
  assert abs(vs * vs * TERW_KAPPA - 1.5) < 1e-6    # holds a_lat = 1.5 by construction


def test_v_safe_straight_is_unlimited():
  assert v_safe(0.0) == float('inf')
  assert v_safe(1e-6) == float('inf')


# ---- core cap --------------------------------------------------------------
def test_straight_road_no_cap():
  curvs = [0.0] * 10
  dists = [i * 15.0 for i in range(10)]
  assert curve_speed_target(curvs, dists, v_cruise=V70) == V70


def test_terwilliger_apex_commands_slowdown():
  # at the apex (d=0) it must command ~54-58 mph, NOT 70 (the failure)
  cap = curve_speed_target([TERW_KAPPA], [0.0], v_cruise=V70, a_lat=1.5)
  assert 50 < mph(cap) < 60


def test_brakes_before_the_curve_not_at_it():
  # the cap must be below cruise while the curve is still ahead (early braking) and ramp DOWN as you
  # approach. Uses explicit firmer params so the distances are unambiguous (mechanism, not the default).
  kw = dict(v_cruise=V70, a_lat=1.5, a_decel=1.5)
  cap_far = curve_speed_target([TERW_KAPPA], [150.0], **kw)   # far: little/no cap yet
  cap_near = curve_speed_target([TERW_KAPPA], [80.0], **kw)   # near: braking
  cap_apex = curve_speed_target([TERW_KAPPA], [0.0], **kw)
  assert cap_near < V70                 # already braking before the curve
  assert cap_near < cap_far             # ramps down as distance shrinks
  assert cap_near > cap_apex            # but not yet at the apex target


def test_aggressiveness_knob_lower_alat_is_slower():
  a = curve_speed_target([TERW_KAPPA], [0.0], v_cruise=V70, a_lat=2.0)
  b = curve_speed_target([TERW_KAPPA], [0.0], v_cruise=V70, a_lat=1.5)
  c = curve_speed_target([TERW_KAPPA], [0.0], v_cruise=V70, a_lat=1.2)
  assert mph(a) > mph(b) > mph(c)


def test_only_reduces_never_raises():
  # already slower than the curve's safe speed -> unchanged (VTSC can never speed up)
  cap = curve_speed_target([TERW_KAPPA], [0.0], v_cruise=40 * MPH)
  assert cap <= 40 * MPH


def test_v_min_floor():
  # a hairpin (very high curvature) is floored, not commanded toward 0
  cap = curve_speed_target([1.0 / 30], [0.0], v_cruise=V70, v_min=6.7)
  assert cap >= 6.7


def test_binding_curve_is_the_sharpest_nearest():
  # with several points the cap is set by the most-binding curve, not a gentle far one. (Uses a tighter
  # R250 apex so it binds at 70 under the deployed A_LAT_TARGET=2.2 — Terwilliger R415 no longer binds @70.)
  curvs = [1 / 250, 1 / 2000, 1 / 3000]
  dists = [40.0, 60.0, 100.0]
  cap = curve_speed_target(curvs, dists, v_cruise=V70)
  assert mph(cap) < 70
  # equals what the R250 point alone (the binding one, at 40 m) would give
  assert abs(cap - curve_speed_target([1 / 250], [40.0], v_cruise=V70)) < 1e-6


# ---- rate limiter (apply_limits) -------------------------------------------
def test_apply_limits_none_starts_at_cruise():
  # no prior state + no curve (target == cruise) -> stays at cruise
  assert apply_limits(None, 31.3, 31.3, dt=0.05) == 31.3
  # no prior state + a curve target -> begins from cruise and steps down by one decel step
  assert abs(apply_limits(None, 20.0, 31.3, dt=0.05, a_decel_max=3.0) - (31.3 - 0.15)) < 1e-6


def test_apply_limits_bounds_decel_rate():
  # target far below current; one 50 ms step may drop by at most A_DECEL_MAX(3.0)*dt = 0.15 m/s
  out = apply_limits(31.3, 20.0, 31.3, dt=0.05, a_decel_max=3.0)
  assert abs(out - (31.3 - 0.15)) < 1e-6


def test_apply_limits_never_above_cruise():
  out = apply_limits(31.0, 35.0, 31.3, dt=0.05)   # target above cruise -> clamp to cruise
  assert out <= 31.3


def test_apply_limits_eases_back_up():
  # curve cleared (target=cruise): cap rises gently, not instantly
  out = apply_limits(20.0, 31.3, 31.3, dt=0.05, a_relax=1.5)
  assert 20.0 < out < 20.0 + 1.5 * 0.05 + 1e-6


def test_decel_envelope_matches_calibration():
  # to slow 70 -> ~57 mph (apex) at a_decel=1.5, braking should engage ~100-115 m out
  vc = V70
  d_engage = next(d for d in range(160, 0, -1)
                  if curve_speed_target([TERW_KAPPA], [float(d)], v_cruise=vc, a_lat=1.5, a_decel=1.5) < vc)
  assert 95 < d_engage < 125


def test_default_apex_target_terwilliger():
  # deployed A_LAT_TARGET=2.2 targets ~67-68 mph at the Terwilliger apex (raised 1.9->2.2 to carry more
  # speed; only curves tighter than ~R550 bind at 70 at all)
  cap = curve_speed_target([TERW_KAPPA], [0.0], v_cruise=V70)
  assert 65 <= mph(cap) <= 70


def test_gentler_curves_allow_higher_speeds():
  # v_safe scales with sqrt(R): tighter curves get proportionally LOWER caps, and beyond ~R326 the
  # 70 mph cruise doesn't bind at all (A_LAT_TARGET=3.0 -> Terwilliger R415 is safe at ~79 mph now).
  v_tight = curve_speed_target([1 / 250], [0.0], v_cruise=V70)
  v_gentle = curve_speed_target([1 / 300], [0.0], v_cruise=V70)
  v_sweeping = curve_speed_target([1 / 415], [0.0], v_cruise=V70)
  assert v_tight < v_gentle <= V70
  assert v_sweeping == V70                      # R415 at 70 -> no cap at all under A_LAT=3.0


# ---- apex release (brake entrance->apex, accelerate out) --------------------
def test_apex_release_committed_points_dont_bind():
  # the apex point inside min_dist is committed -> no cap from it; a curve still AHEAD binds. (R250 so it
  # binds at 70 under A_LAT_TARGET=2.2.)
  K = 1 / 250
  assert curve_speed_target([K], [3.0], v_cruise=V70, min_dist=8.0) == V70   # at the apex -> release
  assert curve_speed_target([K], [40.0], v_cruise=V70, min_dist=8.0) < V70   # still ahead -> brake


def test_apex_release_cap_rises_as_apex_passes():
  # same curve: cap while approaching < cap once the apex zone has slid inside the commit window
  approaching = curve_speed_target([1 / 250, 1 / 250], [10.0, 30.0], v_cruise=V70, min_dist=8.0)
  passing = curve_speed_target([1 / 250, 1 / 250], [2.0, 6.0], v_cruise=V70, min_dist=8.0)
  assert approaching < V70                      # entrance: braking (R250 binds under A_LAT=3.0)
  assert passing == V70                         # apex under the car: released -> MPC accelerates out


def test_apex_release_second_curve_still_binds():
  # passing curve #1's apex must NOT release a second curve further ahead
  cap = curve_speed_target([1 / 250, 1 / 220], [2.0, 90.0], v_cruise=V70, min_dist=8.0)
  assert cap < V70                              # the R220 curve 90 m out still caps (binds under A_LAT=3.0)
  assert abs(cap - curve_speed_target([1 / 220], [90.0], v_cruise=V70)) < 1e-6


def test_min_dist_zero_keeps_old_behavior():
  apex = curve_speed_target([1 / 250], [0.0], v_cruise=V70, min_dist=0.0)
  assert apex < V70                             # min_dist=0 -> apex point still binds (R250 under A_LAT=3.0)


# ---- controller apex state machine (drive #4: confidence cut, finish-before-apex, accelerate out) ----
import time
import types


def _fake_model(apex_k, apex_d, v_ego):
  vx = max(v_ego, 1.0)
  return types.SimpleNamespace(
    orientationRate=types.SimpleNamespace(z=[0.0, apex_k * vx, 0.0],
                                          t=[0.0, apex_d / vx, (apex_d + 30.0) / vx]),
    velocity=types.SimpleNamespace(x=[vx, vx, vx]),
    position=types.SimpleNamespace(x=[0.0, apex_d, apex_d + 30.0]))


def _make_ctrl():
  from openpilot.selfdrive.controls.lib.vtsc_pnw.vtsc_controller import VTSCController
  cp = types.SimpleNamespace(openpilotLongitudinalControl=True)
  c = VTSCController(cp, params=types.SimpleNamespace(get_bool=lambda k: True))
  c.mem_params = None              # no overlay publish in the test
  return c


def _step(c, apex_d, apex_k=TERW_KAPPA, v_cruise=V70, v_ego=28.0):
  c._last_t = time.monotonic() - 0.05   # force dt ~ 50 ms (deterministic rate-limit step)
  c.cap({'modelV2': _fake_model(apex_k, apex_d, v_ego)}, v_cruise, v_ego)
  return c.msg["vTarget"]


def test_state_machine_confidence_cut_then_brake_hold_release():
  c = _make_ctrl()
  # 1) approach (apex far) -> after debounce, BRAKE with an immediate >=1mph cut
  for _ in range(5):
    _step(c, apex_d=200.0)
  assert c._state == "brake"
  cut = c.msg["vTarget"]
  assert cut <= V70 - C.CONFIDENCE_CUT + 1e-3            # instant confidence cut (>=~1 mph)

  # 2) keep braking with the apex closer -> slows further (below the mere cut)
  for _ in range(40):
    _step(c, apex_d=60.0)
  braked = c.msg["vTarget"]
  assert braked < cut - 0.2                              # actually reduced speed for the curve

  # 3) HOLD zone (close/uncertain) -> must NOT reduce further
  hold_start = braked
  for _ in range(20):
    v = _step(c, apex_d=25.0)
    assert v >= hold_start - 1e-3                         # never reduces in hold
  assert c._state == "hold"

  # 4) at the apex -> RELEASE: accelerate back toward cruise, never reduce
  rel = [_step(c, apex_d=8.0) for _ in range(20)]
  assert c._state == "release"
  assert rel[-1] > rel[0] + 0.1                           # accelerating out
  for a, b in zip(rel, rel[1:], strict=False):
    assert b >= a - 1e-6                                  # monotonic non-decrease past the apex

  # 5) road clears -> back to idle at full cruise
  for _ in range(C.CLEAR_CYCLES + 60):
    _step(c, apex_d=-1.0, apex_k=0.0)
  assert c._state == "idle"
  assert abs(c.msg["vTarget"] - V70) < 1e-6


def test_state_machine_disabled_is_neutral():
  from openpilot.selfdrive.controls.lib.vtsc_pnw.vtsc_controller import VTSCController
  cp = types.SimpleNamespace(openpilotLongitudinalControl=True)
  c = VTSCController(cp, params=types.SimpleNamespace(get_bool=lambda k: False))  # CES off
  c.mem_params = None
  c._last_t = time.monotonic() - 0.05
  out = c.cap({'modelV2': _fake_model(TERW_KAPPA, 50.0, 28.0)}, V70, 28.0)
  assert out == V70 and not c.msg["enabled"]              # disabled -> byte-identical passthrough


def test_decel_ceiling_bounded_but_firm_enough():
  # the rate limiter bounds one step to A_DECEL_MAX*dt (no slam), but the ceiling is now firm enough
  # (drive #4: brake harder if needed to finish before the apex) — yet still well below an emergency stop
  step = apply_limits(V70, 10.0, V70, dt=0.05)          # huge target drop
  assert (V70 - step) <= C.A_DECEL_MAX * 0.05 + 1e-9     # bounded by the ceiling
  assert 1.5 <= C.A_DECEL_MAX <= 3.0                     # firm enough to finish before apex, not a slam


# ---- sharpest_ahead (apex finder) ------------------------------------------
def test_sharpest_ahead_picks_max_curvature():
  k, d = sharpest_ahead([1/2000, TERW_KAPPA, 1/3000], [40.0, 80.0, 120.0])
  assert abs(k - TERW_KAPPA) < 1e-9 and d == 80.0       # the apex = the sharpest point


def test_sharpest_ahead_straight_is_none():
  k, d = sharpest_ahead([0.0, 0.0], [10.0, 40.0])
  assert k == 0.0 and d == -1.0


# ---- brake_cap_for_apex (finish before the apex) ---------------------------
def test_brake_cap_falls_to_curve_speed_at_finish_point():
  vcs = v_safe(TERW_KAPPA)                               # default a_lat
  # distances chosen so the finish point (v_ego*APEX_FINISH_S=2.5 -> 70 m) sits inside them
  far = brake_cap_for_apex(vcs, apex_dist=300.0, v_ego=28.0)
  near = brake_cap_for_apex(vcs, apex_dist=120.0, v_ego=28.0)
  at = brake_cap_for_apex(vcs, apex_dist=28.0 * C.APEX_FINISH_S, v_ego=28.0)
  assert far > near > at                                 # cap falls as the apex approaches
  assert abs(at - vcs) < 1e-6                            # at the finish point -> exactly curve speed


def test_brake_cap_finishes_before_apex():
  # at apex_dist == v_ego*APEX_FINISH_S the target is already curve speed (slowing done before apex)
  vcs = v_safe(TERW_KAPPA)
  d_finish = 28.0 * C.APEX_FINISH_S
  assert abs(brake_cap_for_apex(vcs, apex_dist=d_finish, v_ego=28.0) - vcs) < 1e-6


def test_brake_cap_straight_unlimited():
  assert brake_cap_for_apex(float('inf'), 100.0, 28.0) == float('inf')


# ---- sharpcurve2pnw: distance-based lookahead + regen-coast + twisty descent ----
LAT0, LON0 = 47.6, -122.3
def _pt(d_m, v):
  """A map point d_m metres due-east of (LAT0,LON0) with target velocity v (haversine ~= d_m)."""
  dlon = d_m / (111320.0 * math.cos(math.radians(LAT0)))
  return {"latitude": LAT0, "longitude": LON0 + dlon, "velocity": v}


def test_required_decel():
  assert abs(required_decel(31.3, 13.4, 200.0) - (31.3**2 - 13.4**2) / 400.0) < 1e-6
  assert required_decel(20.0, 25.0, 100.0) == 0.0    # no slowing needed
  assert required_decel(20.0, 10.0, 0.0) == 0.0      # no distance -> 0


def test_most_binding_picks_lowest_envelope_not_nearest():
  # a NEAR sharp curve (13 m/s @ 60 m) must out-bind a FAR gentle one (25 m/s @ 300 m)
  v, d, sharp = most_binding_map_curve([_pt(300, 25.0), _pt(60, 13.0)], LAT0, LON0, 28.0, 500.0)
  assert abs(v - 13.0) < 0.5 and 40 < d < 90 and sharp is True


def test_most_binding_respects_horizon():
  # the binding curve sits beyond a short horizon -> not considered
  v, d, _ = most_binding_map_curve([_pt(300, 13.0)], LAT0, LON0, 28.0, 100.0)
  assert v == 0.0 and d == float('inf')


def test_most_binding_far_curve_seen_over_full_500m():
  # the whole point of the change: a sharp curve at 450 m IS now found (old ~370 m horizon missed it)
  v, d, sharp = most_binding_map_curve([_pt(450, 18.0)], LAT0, LON0, 31.0, 500.0)
  assert abs(v - 18.0) < 0.5 and 400 < d < 500 and sharp is True


def test_most_binding_empty_or_no_gps():
  assert most_binding_map_curve([], LAT0, LON0, 28.0, 500.0) == (0.0, float('inf'), False)
  assert most_binding_map_curve([_pt(60, 13.0)], None, None, 28.0, 500.0) == (0.0, float('inf'), False)


def test_most_binding_applies_scale_and_clamp():
  # the MTSC scale + clamp are applied INSIDE selection so the returned target matches what's used
  v, _, _ = most_binding_map_curve([_pt(80, 20.0)], LAT0, LON0, 28.0, 500.0, speed_scale=1.12)
  assert abs(v - 20.0 * 1.12) < 0.5                       # scaled up
  v2, _, _ = most_binding_map_curve([_pt(80, 25.0)], LAT0, LON0, 28.0, 500.0,
                                    speed_scale=1.12, v_cruise_cap=26.0)
  assert v2 <= 26.0 + 1e-6                                # clamped to the set cruise


def test_sharp_classification_on_raw_target_not_scaled():
  # raw 28 m/s is physically sharp (<30); the 1.12x scale -> 31.4 must NOT lose the sharp flag,
  # else the last-resort firmer brake would be denied while targeting the inflated entrance speed
  _, _, sharp = most_binding_map_curve([_pt(120, 28.0)], LAT0, LON0, 31.0, 500.0,
                                       speed_scale=1.12, v_cruise_cap=40.0)
  assert sharp is True


def test_twisty_trim_only_on_descent():
  pts = [_pt(80, 20.0), _pt(160, 21.0), _pt(240, 22.0)]   # 3 packed curves
  vc = 31.3
  # flat (no pitch data) -> NO trim, full speed kept
  assert twisty_section_cap(pts, LAT0, LON0, vc, 28.0, 500.0, pitch=None) == vc
  # flat road (pitch ~0) -> NO trim
  assert twisty_section_cap(pts, LAT0, LON0, vc, 28.0, 500.0, pitch=0.0) == vc
  # descending + 3 curves -> trims, bounded by the floor, never below it, never above cruise
  out = twisty_section_cap(pts, LAT0, LON0, vc, 28.0, 500.0, pitch=-0.05)
  assert vc * C.TWISTY_MIN_FACTOR - 1e-6 <= out < vc


def test_twisty_needs_enough_curves():
  pts = [_pt(80, 20.0), _pt(160, 21.0)]                   # only 2 curves
  vc = 31.3
  assert twisty_section_cap(pts, LAT0, LON0, vc, 28.0, 500.0, pitch=-0.05) == vc   # < min_curves -> no trim


def test_twisty_only_reduces():
  pts = [_pt(80, 20.0), _pt(160, 21.0), _pt(240, 22.0)]
  vc = 31.3
  assert twisty_section_cap(pts, LAT0, LON0, vc, 28.0, 500.0, pitch=-0.05) <= vc
