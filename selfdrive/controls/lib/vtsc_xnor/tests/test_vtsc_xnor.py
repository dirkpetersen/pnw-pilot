"""
Unit tests for the VTSC core — pure math, calibrated to the drive-#3 I-5 Terwilliger log:
the car held 70 mph through a ~415 m-radius curve (2.6 m/s^2 lateral) and did NOT slow, so the
driver intervened. VTSC must instead command a slowdown to a safe speed (~57 mph at A_LAT_TARGET=1.5)
and START braking ~100 m before the apex, not at it.
"""
from openpilot.selfdrive.controls.lib.vtsc_xnor.vtsc_xnor import (
  v_safe, curve_speed_target, apply_limits, sharpest_ahead, brake_cap_for_apex)
from openpilot.selfdrive.controls.lib.vtsc_xnor import vtsc_constants as C

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
  # with several points the cap is set by the most-binding curve, not a gentle far one
  curvs = [TERW_KAPPA, 1 / 2000, 1 / 3000]
  dists = [40.0, 60.0, 100.0]
  cap = curve_speed_target(curvs, dists, v_cruise=V70)
  assert mph(cap) < 70
  # equals what the Terwilliger point alone (the binding one, at 40 m) would give
  assert abs(cap - curve_speed_target([TERW_KAPPA], [40.0], v_cruise=V70)) < 1e-6


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
  # default A_LAT_TARGET targets ~62 mph at the Terwilliger apex — a slight ~8 mph trim
  # (driver feedback after drive #4: 57 was too aggressive, only a slight adjustment is wanted)
  cap = curve_speed_target([TERW_KAPPA], [0.0], v_cruise=V70)
  assert 60 <= mph(cap) <= 65


def test_gentler_curves_allow_higher_speeds():
  # v_safe scales with sqrt(R): gentler-than-Terwilliger curves get proportionally higher caps,
  # and beyond ~R550 the 70 mph cruise doesn't bind at all
  v_terw = curve_speed_target([1 / 415], [0.0], v_cruise=V70)
  v_gentle = curve_speed_target([1 / 550], [0.0], v_cruise=V70)
  v_sweeping = curve_speed_target([1 / 700], [0.0], v_cruise=V70)
  assert v_terw < v_gentle <= V70
  assert v_sweeping == V70                      # sweeping curve at 70 -> no cap at all


# ---- apex release (brake entrance->apex, accelerate out) --------------------
def test_apex_release_committed_points_dont_bind():
  # the apex point inside min_dist is committed -> no cap from it; a curve still AHEAD binds
  assert curve_speed_target([TERW_KAPPA], [3.0], v_cruise=V70, min_dist=8.0) == V70   # at the apex -> release
  assert curve_speed_target([TERW_KAPPA], [40.0], v_cruise=V70, min_dist=8.0) < V70   # still ahead -> brake


def test_apex_release_cap_rises_as_apex_passes():
  # same curve: cap while approaching < cap once the apex zone has slid inside the commit window
  approaching = curve_speed_target([TERW_KAPPA, TERW_KAPPA], [10.0, 30.0], v_cruise=V70, min_dist=8.0)
  passing = curve_speed_target([TERW_KAPPA, TERW_KAPPA], [2.0, 6.0], v_cruise=V70, min_dist=8.0)
  assert approaching < V70                      # entrance: braking
  assert passing == V70                         # apex under the car: released -> MPC accelerates out


def test_apex_release_second_curve_still_binds():
  # passing curve #1's apex must NOT release a second curve further ahead
  cap = curve_speed_target([TERW_KAPPA, 1 / 300], [2.0, 90.0], v_cruise=V70, min_dist=8.0)
  assert cap < V70                              # the R300 curve 90 m out still caps
  assert abs(cap - curve_speed_target([1 / 300], [90.0], v_cruise=V70)) < 1e-6


def test_min_dist_zero_keeps_old_behavior():
  apex = curve_speed_target([TERW_KAPPA], [0.0], v_cruise=V70, min_dist=0.0)
  assert apex < V70                             # min_dist=0 -> apex point still binds (old behavior)


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
  from openpilot.selfdrive.controls.lib.vtsc_xnor.vtsc_controller import VTSCController
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
  from openpilot.selfdrive.controls.lib.vtsc_xnor.vtsc_controller import VTSCController
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
  far = brake_cap_for_apex(vcs, apex_dist=200.0, v_ego=28.0)
  near = brake_cap_for_apex(vcs, apex_dist=40.0, v_ego=28.0)
  at = brake_cap_for_apex(vcs, apex_dist=0.0, v_ego=28.0)
  assert far > near > at                                 # cap falls as the apex approaches
  assert abs(at - vcs) < 1e-6                            # at/after the finish point -> exactly curve speed


def test_brake_cap_finishes_before_apex():
  # at apex_dist == v_ego*APEX_FINISH_S the target is already curve speed (slowing done before apex)
  vcs = v_safe(TERW_KAPPA)
  d_finish = 28.0 * C.APEX_FINISH_S
  assert abs(brake_cap_for_apex(vcs, apex_dist=d_finish, v_ego=28.0) - vcs) < 1e-6


def test_brake_cap_straight_unlimited():
  assert brake_cap_for_apex(float('inf'), 100.0, 28.0) == float('inf')
