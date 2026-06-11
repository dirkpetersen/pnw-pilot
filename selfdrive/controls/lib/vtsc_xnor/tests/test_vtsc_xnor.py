"""
Unit tests for the VTSC core — pure math, calibrated to the drive-#3 I-5 Terwilliger log:
the car held 70 mph through a ~415 m-radius curve (2.6 m/s^2 lateral) and did NOT slow, so the
driver intervened. VTSC must instead command a slowdown to a safe speed (~57 mph at A_LAT_TARGET=1.5)
and START braking ~100 m before the apex, not at it.
"""
from openpilot.selfdrive.controls.lib.vtsc_xnor.vtsc_xnor import v_safe, curve_speed_target, apply_limits

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


def test_default_decel_ceiling_never_slams():
  # the rate limiter at the default ceiling must keep one step tiny (no hard braking)
  from openpilot.selfdrive.controls.lib.vtsc_xnor import vtsc_constants as C
  step = apply_limits(V70, 10.0, V70, dt=0.05)          # huge target drop
  assert (V70 - step) <= C.A_DECEL_MAX * 0.05 + 1e-9     # bounded by the ceiling
  assert C.A_DECEL_MAX <= 1.5                            # ceiling itself is gentle (<=0.15 g)
