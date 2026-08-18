"""
Unit tests for the VTSC core — pure math, calibrated to the drive-#3 I-5 Terwilliger log:
the car held 70 mph through a ~415 m-radius curve (2.6 m/s^2 lateral) and did NOT slow, so the
driver intervened. VTSC must instead command a slowdown to a safe speed (~57 mph at A_LAT_TARGET=1.5)
and START braking ~100 m before the apex, not at it.
"""
import math

from openpilot.selfdrive.controls.lib.vtsc_pnw.vtsc_pnw import (
  polyline_curvature,
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

import pytest
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
  v, d, sharp, _raw, _fl = most_binding_map_curve([_pt(300, 25.0), _pt(60, 13.0)], LAT0, LON0, 28.0, 500.0)
  assert abs(v - 13.0) < 0.5 and 40 < d < 90 and sharp is True


def test_most_binding_respects_horizon():
  # the binding curve sits beyond a short horizon -> not considered
  v, d, _, _raw, _fl = most_binding_map_curve([_pt(300, 13.0)], LAT0, LON0, 28.0, 100.0)
  assert v == 0.0 and d == float('inf')


def test_most_binding_far_curve_seen_over_full_500m():
  # the whole point of the change: a sharp curve at 450 m IS now found (old ~370 m horizon missed it)
  v, d, sharp, _raw, _fl = most_binding_map_curve([_pt(450, 18.0)], LAT0, LON0, 31.0, 500.0)
  assert abs(v - 18.0) < 0.5 and 400 < d < 500 and sharp is True


def test_most_binding_empty_or_no_gps():
  assert most_binding_map_curve([], LAT0, LON0, 28.0, 500.0) == (0.0, float('inf'), False, 0.0, False)
  assert most_binding_map_curve([_pt(60, 13.0)], None, None, 28.0, 500.0) == (0.0, float('inf'), False, 0.0, False)


def test_most_binding_applies_scale_and_clamp():
  # the MTSC scale + clamp are applied INSIDE selection so the returned target matches what's used
  v, _, _, _raw, _fl = most_binding_map_curve([_pt(80, 20.0)], LAT0, LON0, 28.0, 500.0, speed_scale=1.12)
  assert abs(v - 20.0 * 1.12) < 0.5                       # scaled up
  v2, _, _, _raw2, _fl2 = most_binding_map_curve([_pt(80, 25.0)], LAT0, LON0, 28.0, 500.0,
                                    speed_scale=1.12, v_cruise_cap=26.0)
  assert v2 <= 26.0 + 1e-6                                # clamped to the set cruise


def test_sharp_classification_on_raw_target_not_scaled():
  # raw 28 m/s is physically sharp (<30); the 1.12x scale -> 31.4 must NOT lose the sharp flag,
  # else the last-resort firmer brake would be denied while targeting the inflated entrance speed
  _, _, sharp, _raw, _fl = most_binding_map_curve([_pt(120, 28.0)], LAT0, LON0, 31.0, 500.0,
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


# --- curvefloor2pnw: the MTSC scale must never erase a curve mapd DID flag ------------------------
# 2026-08-18, I-5 Woodland: raw map target 27.0 m/s (60 mph) -> tiered scale 1.742 -> 47 m/s (105 mph)
# -> clamped to the 40.2 m/s set speed -> failed the MAP_MIN_SLOWDOWN gate -> "no map curve" -> VTSC
# commanded NOTHING and the car entered a 409 m-radius curve at 90 mph until the driver took over.

def test_scale_can_inflate_a_real_target_past_the_set_speed():
    """The mechanism itself, pinned so it can't silently change."""
    scale = C.tiered_map_scale(27.0)
    assert 1.7 < scale < 1.8
    assert 27.0 * scale > 40.2, "the scaled target exceeded the driver's set speed"


def test_selector_returns_the_raw_target_alongside_the_scaled_one():
    v, d, _sharp, raw, _fl = most_binding_map_curve([_pt(300, 27.0)], LAT0, LON0, 40.0, 500.0,
                                               speed_scale=C.MAP_SPEED_SCALE, v_cruise_cap=40.2)
    assert raw == 27.0, "the caller must be able to see what mapd actually said"
    assert v > raw, "and that the scale inflated it"


def _fold(ctrl, raw_target, v_cruise_set, dist_m=300.0):
    ctrl._map_targets = [_pt(dist_m, raw_target)]
    ctrl._cur_lat, ctrl._cur_lon = LAT0, LON0
    return ctrl._fold_map_curve(0.0, -1.0, float('inf'), v_cruise_set, v_cruise_set, 500.0)


def test_inflated_target_now_yields_the_minimum_slowdown_instead_of_nothing():
    c = _make_ctrl()
    c._map_curves = True
    _k, d, v, _sharp = _fold(c, 27.0, 40.2)
    assert v != float('inf'), "the curve must not be discarded"
    assert v == pytest.approx(40.2 - C.MAP_MIN_SLOWDOWN, abs=1e-6)
    assert d > 0.0
    assert c._tele_map_floored is True


def test_the_floor_is_small_and_never_over_brakes():
    c = _make_ctrl()
    c._map_curves = True
    _k, _d, v, _sharp = _fold(c, 27.0, 40.2)
    # ~10 mph off a 90 mph set speed -- a trim, not a stab
    assert (40.2 - v) * 2.237 == pytest.approx(10.0, abs=0.5)


def test_an_already_binding_map_curve_is_untouched():
    """A curve whose SCALED target is already meaningfully below cruise must keep its own value --
    the tuned sweeper behaviour must not change."""
    c = _make_ctrl()
    c._map_curves = True
    _k, _d, v, _sharp = _fold(c, 13.0, 40.2)          # tight curve: scale 1.35 -> ~17.6 m/s
    assert v == pytest.approx(13.0 * C.tiered_map_scale(13.0), rel=1e-6)
    assert c._tele_map_floored is False


def test_no_map_curve_still_means_no_cap():
    c = _make_ctrl()
    c._map_curves = True
    c._map_targets = []
    c._cur_lat, c._cur_lon = LAT0, LON0
    _k, _d, v, _sharp = c._fold_map_curve(0.0, -1.0, float('inf'), 40.2, 40.2, 500.0)
    assert v == float('inf'), "no map data must never synthesise a slowdown"
    assert c._tele_map_floored is False


def test_a_trivial_target_above_cruise_is_still_ignored():
    c = _make_ctrl()
    c._map_curves = True
    _k, _d, v, _sharp = _fold(c, 45.0, 40.2)          # mapd says faster than we're going
    assert v == float('inf')
    assert c._tele_map_floored is False


# --- curvefloor2pnw: the floor must survive SHADOWING by ordinary multi-point map data -------------
# A review caught that applying the floor AFTER selection was provably suppressed: the envelope ranks
# on the scaled+clamped target, so once several points clamp to the set speed they tie and the NEAREST
# wins. mapd emits a target for EVERY node with finite curvature, so an ordinary gentle node in front
# of the real curve was selected instead and its high raw blocked the floor -- the fix would not have
# fired on the very road that motivated it. Every test below uses MULTIPLE points for that reason;
# the original single-point tests could not see this class of bug at all.

_SET = 40.2


def _sharp_plus_shadows(sharp_d, sharp_v=27.0, shadow_v=39.2, shadows=(50, 120, 220)):
    return [_pt(sharp_d, sharp_v)] + [_pt(x, shadow_v) for x in shadows if x < sharp_d]


def _select(points, v_ego=_SET):
    return most_binding_map_curve(points, LAT0, LON0, v_ego, 500.0, C.A_DECEL, C.APEX_FINISH_S,
                                  C.SHARP_CURVE_V, C.MAP_SPEED_SCALE, _SET, C.MAP_MIN_SLOWDOWN)


def test_floor_survives_a_nearer_gentle_node_shadowing_the_curve():
    v, d, _s, _raw, floored = _select(_sharp_plus_shadows(200))
    assert floored is True
    assert v == pytest.approx(_SET - C.MAP_MIN_SLOWDOWN, abs=1e-6)
    assert d == pytest.approx(200, abs=5), "the SHARP curve must be the selected one, not the shadow"


def test_floor_binds_with_useful_warning_distance():
    """It must fire far enough out to be a trim rather than a stab."""
    bound_at = None
    for dist in (500, 400, 300, 250, 200, 150, 100):
        v, d, _s, _raw, _f = _select(_sharp_plus_shadows(dist))
        if 0.0 < v < _SET - C.MAP_MIN_SLOWDOWN + 1e-6 and bound_at is None:
            bound_at = dist
    assert bound_at is not None, "never binds during the whole approach"
    assert bound_at >= 150, f"only {bound_at} m of warning — too late to be a gentle trim"
    # and the decel that trim implies is mild
    req = (_SET ** 2 - (_SET - C.MAP_MIN_SLOWDOWN) ** 2) / (2 * bound_at)
    assert req < 1.5, f"required decel {req:.2f} m/s^2 is not a gentle trim"


def test_dense_shadowing_still_selects_the_real_curve_in_time():
    """Worst realistic case: mapd always emits a node a few metres ahead, so there is ALWAYS a near
    shadow. The floored curve legitimately loses the envelope until it is close enough to actually
    need braking -- what matters is that it wins EARLY ENOUGH for the trim to stay gentle."""
    bound_at = None
    for dist in (400, 300, 250, 200, 170, 150, 120, 100):
        pts = [_pt(dist, 27.0)] + [_pt(x, 39.2) for x in (24, 60, 100, 150, 250) if x < dist]
        v, d, _s, _raw, _f = _select(pts)
        if 0.0 < v < _SET - C.MAP_MIN_SLOWDOWN + 1e-6:
            bound_at = dist
            assert d == pytest.approx(dist, abs=5), "must select the SHARP curve, not a shadow"
            break
    assert bound_at is not None, "dense shadowing suppressed the curve for the whole approach"
    req = (_SET ** 2 - (_SET - C.MAP_MIN_SLOWDOWN) ** 2) / (2 * bound_at)
    assert req < 1.5, f"binds at {bound_at} m -> {req:.2f} m/s^2, no longer a gentle trim"


def test_gentle_nodes_alone_never_synthesise_a_slowdown():
    v, _d, _s, _raw, floored = _select([_pt(x, 39.2) for x in (24, 100, 300)])
    assert floored is False
    assert v >= _SET - C.MAP_MIN_SLOWDOWN, "no real curve -> no cap"


def test_an_already_binding_curve_is_not_raised_by_the_floor():
    # a genuinely tight curve keeps its own (deeper) target even with shadows present
    pts = [_pt(200, 13.0)] + [_pt(x, 39.2) for x in (50, 120)]
    v, _d, _s, _raw, floored = _select(pts)
    assert v == pytest.approx(13.0 * C.tiered_map_scale(13.0), rel=1e-6)
    assert floored is False, "a curve that already binds must not be floored upward"


def test_floor_never_targets_below_the_raw_advisory():
    """The floored target is always ABOVE what mapd asked for -- it can never over-brake."""
    for raw in (20.0, 27.0, 30.0, 34.0):
        v, _d, _s, _r, floored = _select(_sharp_plus_shadows(200, sharp_v=raw))
        if floored:
            assert v > raw, f"floored target {v} undercut mapd's own advisory {raw}"


# --- MAP_FLOOR_DEPTH: the knob must default to the cheapest possible safety net -------------------
# Corridor directive 2026-08-18: "very very gradual slowdowns... the system is working very well...
# I want to go as smooth and fast as possible." The map path cannot tell a real curve from a mapd
# artifact, so its default must cost as little speed as possible.

def test_default_depth_is_the_flat_notch():
    assert C.MAP_FLOOR_DEPTH == 0.0, "the shipped default must not deepen the trim"
    v, _d, _s, _raw, floored = _select(_sharp_plus_shadows(200))
    assert floored is True
    assert v == pytest.approx(_SET - C.MAP_MIN_SLOWDOWN, abs=1e-6)


def _select_depth(points, depth, limit):
    return most_binding_map_curve(points, LAT0, LON0, _SET, 500.0, C.A_DECEL, C.APEX_FINISH_S,
                                  C.SHARP_CURVE_V, C.MAP_SPEED_SCALE, _SET, C.MAP_MIN_SLOWDOWN,
                                  limit, depth)


def test_depth_interpolates_between_notch_and_posted_limit():
    LIMIT = 31.3                                   # 70 mph posted
    notch = _SET - C.MAP_MIN_SLOWDOWN
    pts = _sharp_plus_shadows(200, sharp_v=27.0)
    v0 = _select_depth(pts, 0.0, LIMIT)[0]
    v5 = _select_depth(pts, 0.5, LIMIT)[0]
    v1 = _select_depth(pts, 1.0, LIMIT)[0]
    assert v0 == pytest.approx(notch, abs=1e-6)
    assert v1 == pytest.approx(LIMIT, abs=1e-6)
    assert v5 == pytest.approx((notch + LIMIT) / 2.0, abs=1e-6)


def test_floor_never_goes_below_the_posted_limit_at_any_depth():
    """The controller already refuses to trim below the posted limit; the knob must not out-run it.
    Only applies to FLOORED curves -- a curve whose scaled target already binds keeps its own (deeper)
    value here and is floored by the controller's own SPEED-LIMIT FLOOR downstream, not by this."""
    LIMIT = 31.3
    for depth in (0.0, 0.25, 0.5, 0.75, 1.0):
        v, _d, _s, _r, floored = _select_depth(_sharp_plus_shadows(200, sharp_v=23.0), depth, LIMIT)
        assert floored is True, "test must exercise the floor path"
        assert v >= LIMIT - 1e-6, f"depth {depth} trimmed below the posted limit"


def test_depth_is_clamped_against_a_bad_value():
    LIMIT = 31.3
    notch = _SET - C.MAP_MIN_SLOWDOWN
    pts = _sharp_plus_shadows(200, sharp_v=27.0)
    assert _select_depth(pts, -5.0, LIMIT)[0] == pytest.approx(notch, abs=1e-6)
    assert _select_depth(pts, 99.0, LIMIT)[0] == pytest.approx(LIMIT, abs=1e-6)


def test_a_floored_curve_never_earns_the_friction_brake():
    """is_sharp unlocks SHARP_A_DECEL_MAX (2.8 m/s^2). A synthetic trim must be regen-only -- this is
    what keeps the slowdown gradual rather than a stab."""
    # raws whose SCALED value saturates at/above the notch (so the floor actually fires) AND which
    # are below SHARP_CURVE_V, i.e. would normally be flagged sharp. raw 20 scales to only 30.8,
    # already below the notch, so it binds normally and never reaches the floor.
    for raw in (23.0, 25.0, 27.0):
        _v, _d, sharp, _r, floored = _select(_sharp_plus_shadows(200, sharp_v=raw))
        assert floored is True
        assert sharp is False, f"floored curve at raw {raw} would have unlocked friction braking"


def test_a_genuinely_sharp_unfloored_curve_keeps_its_sharp_flag():
    # the guard must not disarm firm braking for curves that legitimately need it
    pts = [_pt(200, 13.0)] + [_pt(x, 39.2) for x in (50, 120)]
    _v, _d, sharp, _r, floored = _select(pts)
    assert floored is False and sharp is True


def test_a_DEEPENED_floored_curve_keeps_its_firm_braking():
    """Review CRITICAL: suppressing is_sharp for ANY floored curve meant that raising MAP_FLOOR_DEPTH
    handed the car a deep target while denying it SHARP_A_DECEL_MAX to reach it -- worse than not
    deepening at all. A raw target of 25-29 m/s is both 'sharp' (< SHARP_CURVE_V) and floorable."""
    LIMIT = 22.35                                  # 50 mph posted
    pts = _sharp_plus_shadows(200, sharp_v=25.0)   # raw 25 -> scaled 42.1 -> saturates -> floorable
    v0, _d, sharp0, _r, fl0 = _select_depth(pts, 0.0, LIMIT)
    v1, _d, sharp1, _r, fl1 = _select_depth(pts, 1.0, LIMIT)
    assert fl0 is True and fl1 is True
    assert v0 == pytest.approx(_SET - C.MAP_MIN_SLOWDOWN, abs=1e-6)
    assert sharp0 is False, "the shallow synthetic notch must stay regen-only (gradual)"
    assert v1 < v0 - 1.0, "depth 1.0 must actually deepen the target"
    assert sharp1 is True, "a DEEPENED target must keep the authority to reach it"


def test_floored_flag_describes_the_SELECTED_curve_not_any_candidate():
    """Review finding: floored was a global accumulator, so telemetry could report a floor that did
    not apply to the curve actually chosen."""
    # a floorable point far away, and a genuinely-binding tight curve that wins selection
    pts = [_pt(450, 27.0)] + [_pt(150, 13.0)] + [_pt(x, 39.2) for x in (50, 100)]
    v, _d, _s, _raw, floored = _select(pts)
    assert v == pytest.approx(13.0 * C.tiered_map_scale(13.0), rel=1e-6), "tight curve should win"
    assert floored is False, "the SELECTED curve was not floored, so the flag must be False"


# --- mapcurv2pnw: curvature measured from the map polyline (TELEMETRY ONLY) -----------------------
# mapd's velocity field cannot tell a real 409 m curve from a sweeper the driver takes at 85 mph
# (both land in the same 24-27 m/s band). The lat/lon points in the SAME message can. These tests
# pin the three 2026-08-18 reference cases and the live-measured spacing hazard.

def _arc(radius_m, n=9, step_m=40.0, lat0=LAT0, lon0=LON0):
    """n points along a circular arc of the given radius, `step_m` apart, starting at (lat0, lon0)."""
    pts, ang = [], 0.0
    for _ in range(n):
        x, y = radius_m * math.sin(ang), radius_m * (1.0 - math.cos(ang))
        pts.append({"latitude": lat0 + y / 111320.0,
                    "longitude": lon0 + x / (111320.0 * math.cos(math.radians(lat0))),
                    "velocity": 30.0})
        ang += step_m / radius_m
    return pts


def _straight(n=9, step_m=100.0, lat0=LAT0, lon0=LON0):
    return [{"latitude": lat0 + (i * step_m) / 111320.0, "longitude": lon0, "velocity": 30.0}
            for i in range(n)]


@pytest.mark.parametrize("radius,expect_mph,tol", [
    (409.0, 72.0, 6.0),      # Woodland curve  -> measured need ~72 mph
    (208.0, 51.0, 5.0),      # left turn       -> measured need <63 mph
    (577.0, 85.0, 8.0),      # I-84 sweeper    -> must NOT slow (driver took it at 85)
])
def test_reference_curves_reproduce_their_measured_targets(radius, expect_mph, tol):
    k, d, v, _n, _ah = polyline_curvature(_arc(radius), LAT0, LON0, 500.0, a_lat=2.5)
    assert k > 0.0, "a real arc must yield a measurement"
    assert 1.0 / k == pytest.approx(radius, rel=0.25), "recovered radius should track the true one"
    assert v * 2.237 == pytest.approx(expect_mph, abs=tol)
    assert d > 0.0


def test_the_sweeper_and_the_takeover_curve_are_now_distinguishable():
    """The whole point: mapd's velocity band cannot separate these two, geometry can."""
    _k1, _d1, v_takeover, _n, _ah = polyline_curvature(_arc(409.0), LAT0, LON0, 500.0, a_lat=2.5)
    _k2, _d2, v_sweeper, _n, _ah = polyline_curvature(_arc(577.0), LAT0, LON0, 500.0, a_lat=2.5)
    assert v_sweeper - v_takeover > 4.0, "must separate a takeover curve from a fine sweeper"


def test_a_straight_road_yields_no_meaningful_curvature():
    k, _d, v, _n, _ah = polyline_curvature(_straight(), LAT0, LON0, 500.0, a_lat=2.5)
    assert k < 1e-4, "a straight must not manufacture a curve"
    assert v * 2.237 > 200.0, "no curve -> no advisory"


def test_sparse_polyline_yields_no_measurement_rather_than_garbage():
    """Measured live on-device: node gaps ran 57 m to 4072 m. A triplet spanning a 4 km gap must be
    REJECTED, not turned into a fabricated curvature."""
    far = [{"latitude": LAT0 + (i * 4000.0) / 111320.0, "longitude": LON0 + (0.0005 * i),
            "velocity": 30.0} for i in range(5)]
    k, _d, v, _n, _ah = polyline_curvature(far, LAT0, LON0, 100000.0, a_lat=2.5)
    assert k == 0.0, "over-long legs must produce no measurement"
    assert v == float('inf')


def test_bunched_points_are_rejected_as_jitter():
    tight = [{"latitude": LAT0 + (i * 3.0) / 111320.0, "longitude": LON0 + 0.00002 * (i % 2),
              "velocity": 30.0} for i in range(9)]
    k, _d, _v, _n, _ah = polyline_curvature(tight, LAT0, LON0, 500.0, a_lat=2.5)
    assert k == 0.0, "sub-25 m legs are GPS/OSM jitter, not curvature"


def _hairpin():
    """Straight 300 m, then a tight 120 m-radius hairpin that loops back toward the car. Unlike a
    constant-radius arc (where ANY three points give the same curvature, so ordering cannot show a
    difference), curvature VARIES here, so triplet adjacency actually matters."""
    def pt(x, y):
        return {"latitude": LAT0 + y / 111320.0,
                "longitude": LON0 + x / (111320.0 * math.cos(math.radians(LAT0))),
                "velocity": 30.0}
    pts = [pt(0.0, i * 60.0) for i in range(6)]
    R, ang, by = 120.0, 0.0, 300.0
    for _ in range(9):
        ang += 60.0 / R
        pts.append(pt(R * (1.0 - math.cos(ang)), by + R * math.sin(ang)))
    return pts


def _chord(p):
    return math.hypot((p["latitude"] - LAT0) * 111320.0,
                      (p["longitude"] - LON0) * 111320.0 * math.cos(math.radians(LAT0)))


def test_path_order_is_taken_from_the_input_not_re_derived():
    """Review CRITICAL: an earlier version sorted points by straight-line distance from the car. On
    any road that bends back that destroys path topology and FABRICATES geometry. Measured here:
    path order recovers the true R=120 m (39 mph); distance-ordered invents R=78 m (31 mph).
    The previous ordering test could not catch this -- it used a constant-radius arc, where every
    triplet yields the same curvature no matter how the points are permuted."""
    pts = _hairpin()
    d = [_chord(p) for p in pts]
    assert d != sorted(d), "geometry must actually bend back, or this proves nothing"

    k_path, _dd, v_path, _n, _ah = polyline_curvature(pts, LAT0, LON0, 5000.0, a_lat=2.5)
    assert k_path > 0.0
    assert 1.0 / k_path == pytest.approx(120.0, rel=0.15), "path order must recover the TRUE radius"

    k_sorted, _d2, _v2, _n, _ah = polyline_curvature(sorted(pts, key=_chord), LAT0, LON0, 5000.0, a_lat=2.5)
    assert abs(k_sorted - k_path) / k_path > 1e-3, \
        "distance-ordered input must measurably differ -- otherwise the fix is untested"
    assert v_path * 2.237 == pytest.approx(39.0, abs=3.0)


def test_reversing_the_path_does_not_change_the_measurement():
    """Traversal direction must not matter -- curvature is a property of the geometry."""
    pts = _hairpin()
    k_fwd, _d, _v, _n, _ah = polyline_curvature(pts, LAT0, LON0, 5000.0, a_lat=2.5)
    k_rev, _d2, _v2, _n, _ah = polyline_curvature(list(reversed(pts)), LAT0, LON0, 5000.0, a_lat=2.5)
    assert k_rev == pytest.approx(k_fwd, rel=0.05)


@pytest.mark.parametrize("bad", [None, [], [{}], [{"latitude": "x", "longitude": None}],
                                 [{"latitude": float('nan'), "longitude": 0.0}] * 5,
                                 [{"latitude": float('inf'), "longitude": 0.0}] * 5])
def test_garbage_input_never_raises(bad):
    k, d, v, _n, _ah = polyline_curvature(bad, LAT0, LON0, 500.0, a_lat=2.5)
    assert k == 0.0 and d == 0.0 and v == float('inf')


def test_missing_gps_yields_nothing():
    assert polyline_curvature(_arc(400.0), None, None, 500.0)[0] == 0.0


def test_beyond_horizon_points_are_ignored():
    k, _d, _v, _n, _ah = polyline_curvature(_arc(300.0), LAT0, LON0, 10.0, a_lat=2.5)
    assert k == 0.0, "horizon must actually bound the scan"


@pytest.mark.parametrize("a_lat", [0.0, -1.0, float('nan'), float('inf')])
def test_a_bad_a_lat_cannot_raise_into_the_control_loop(a_lat):
    """Review HIGH: the sqrt was outside the try, so a negative/NaN a_lat from the tune dict would
    have raised straight through the guard and taken plannerd (and lateral control) down."""
    k, d, v, _n, _ah = polyline_curvature(_arc(400.0), LAT0, LON0, 500.0, a_lat=a_lat)
    assert math.isfinite(k) and math.isfinite(d)
    assert v == float('inf'), "an unusable a_lat must yield no advisory, not a crash"


def test_returned_values_are_always_json_safe():
    """mapK/mapKD are JSON-serialised into /dev/shm; a NaN there breaks strict consumers."""
    for pts in (_arc(300.0), _straight(), [], _arc(300.0)[:2]):
        k, d, _v, _n, _ah = polyline_curvature(pts, LAT0, LON0, 500.0, a_lat=2.5)
        assert math.isfinite(k) and math.isfinite(d)


# --- review findings: validity count and ahead/behind -------------------------------------------

def test_zero_curvature_is_distinguishable_from_unmeasurable():
    """k == 0.0 alone is ambiguous: 'straight' vs 'geometry unmeasurable'. A real R=1500 m curve with
    ~366 m node gaps (inside the live-measured 57-4072 m range) returns 0.0 exactly like a straight,
    so replay cannot judge whether this lever is trustworthy without a validity count."""
    k_str, _d, _v, n_str, _a = polyline_curvature(_straight(), LAT0, LON0, 5000.0, a_lat=2.5)
    assert k_str < 1e-4 and n_str > 0, "a straight IS measurable -- just not curved"

    sparse = _arc(1500.0, n=6, step_m=366.0)          # legs beyond _K_MAX_LEG_M
    k_sp, _d2, _v2, n_sp, _a2 = polyline_curvature(sparse, LAT0, LON0, 5000.0, a_lat=2.5)
    assert k_sp == 0.0 and n_sp == 0, "unmeasurable must report zero accepted triplets"


def test_a_curve_behind_the_car_is_flagged_not_reported_as_upcoming():
    """mapd publishes the whole current way including nodes BEHIND the car, and the distance is
    unsigned -- so a curve just exited would keep 'advising' a slowdown while receding."""
    ahead_pts = _arc(400.0)
    behind = [{"latitude": 2 * LAT0 - p["latitude"], "longitude": p["longitude"], "velocity": 30.0}
              for p in ahead_pts]                     # mirrored to the south
    _k, _d, _v, n, is_ahead = polyline_curvature(behind, LAT0, LON0, 5000.0, a_lat=2.5,
                                                 cur_bearing=0.0)   # heading north
    assert n > 0, "the geometry is still measurable"
    assert is_ahead is False, "a curve behind us must be flagged"

    _k2, _d2, _v2, _n2, is_ahead2 = polyline_curvature(ahead_pts, LAT0, LON0, 5000.0, a_lat=2.5,
                                                       cur_bearing=0.0)
    assert is_ahead2 is True


def test_unknown_bearing_does_not_silently_drop_data():
    _k, _d, _v, n, is_ahead = polyline_curvature(_arc(400.0), LAT0, LON0, 5000.0, a_lat=2.5,
                                                 cur_bearing=None)
    assert n > 0 and is_ahead is True, "no bearing -> keep the measurement, flagged as unknown/ahead"


def test_mixed_spacing_recovers_a_dense_curve_after_a_huge_gap():
    """The live polyline had a 4072 m gap next to 57 m ones. Per-triplet gating must let the dense
    part through rather than discarding the whole polyline."""
    far = [{"latitude": LAT0 + (i * 4000.0) / 111320.0, "longitude": LON0, "velocity": 30.0}
           for i in range(2)]
    dense = _arc(200.0, n=8, step_m=40.0, lat0=LAT0 + 8000.0 / 111320.0)
    k, _d, _v, n, _a = polyline_curvature(far + dense, LAT0, LON0, 100000.0, a_lat=2.5)
    assert n > 0, "the dense section must still be measured"
    assert 1.0 / k == pytest.approx(200.0, rel=0.25)
