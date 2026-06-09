"""
Unit tests for the VTSC core — pure math, calibrated to the drive-#3 I-5 Terwilliger log:
the car held 70 mph through a ~415 m-radius curve (2.6 m/s^2 lateral) and did NOT slow, so the
driver intervened. VTSC must instead command a slowdown to a safe speed (~57 mph at A_LAT_TARGET=1.5)
and START braking ~100 m before the apex, not at it.
"""
from openpilot.selfdrive.controls.lib.vtsc_xnor.vtsc_xnor import v_safe, curve_speed_target

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
  # the cap must be below cruise while the curve is still ~80 m ahead (early braking),
  # and ramp DOWN as you approach
  cap_far = curve_speed_target([TERW_KAPPA], [150.0], v_cruise=V70)   # far: little/no cap yet
  cap_near = curve_speed_target([TERW_KAPPA], [80.0], v_cruise=V70)   # near: braking
  cap_apex = curve_speed_target([TERW_KAPPA], [0.0], v_cruise=V70)
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
  curvs = [1 / 2000, TERW_KAPPA, 1 / 3000]
  dists = [40.0, 80.0, 120.0]
  cap = curve_speed_target(curvs, dists, v_cruise=V70)
  assert mph(cap) < 70
  # equals what the Terwilliger point alone would give (it's the binding one)
  assert abs(cap - curve_speed_target([TERW_KAPPA], [80.0], v_cruise=V70)) < 1e-6


def test_decel_envelope_matches_calibration():
  # to slow 70 -> ~57 mph (apex) at a_decel=1.5, braking should engage ~100-115 m out
  vc = V70
  d_engage = next(d for d in range(160, 0, -1)
                  if curve_speed_target([TERW_KAPPA], [float(d)], v_cruise=vc) < vc)
  assert 95 < d_engage < 125
