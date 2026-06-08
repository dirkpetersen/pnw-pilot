"""
Unit tests for the CES decision core. Pure logic — no cereal, no car. Validates the calibration
anchors from CES.md (I-5 Terwilliger / Marquam / Wilsonville) and the lead/speed/stop logic.

Run:  pytest selfdrive/controls/lib/ces/tests/test_ces.py
"""
from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.ces_xnor import ces_xnor_constants as C
from openpilot.selfdrive.controls.lib.ces_xnor.ces_xnor import (
  decide_active, vision_curve_lat_accel, curve_closeness, decision_telemetry,
)


ALL_ON = {"curves": True, "stops": True, "low_speed": True, "lead": True}


def base(**kw):
  """A 'cruising, nothing happening' signal set; override fields per test."""
  s = {
    "v_ego": 30.0, "has_lead": False, "lead_vlead": 0.0, "lead_drel": 0.0, "blinker": False,
    "map_target_v": 0.0, "map_target_dist": float('inf'),
    "curve_lat_accel_vision": 0.0, "time_to_curve": 10.0,
    "model_should_stop": False, "toggles": ALL_ON,
  }
  s.update(kw)
  return s


def lat_accel(radius_m, mph):
  return (mph * CV.MPH_TO_MS) ** 2 / radius_m


# ---- curve anchors via the VISION half (lat-accel threshold = 1.9 m/s^2) ----
# These pin CURVE_LAT_ACCEL_ENTER against the real I-5 curves (within the vision horizon).
def test_terwilliger_trips_at_50():
  # R~250 m @ 50 mph -> ~2.0 m/s^2 > 1.9 -> trip
  s = base(v_ego=50 * CV.MPH_TO_MS, curve_lat_accel_vision=lat_accel(250, 50), time_to_curve=2.0)
  active, status = decide_active(s)
  assert active and status == "curve"


def test_wilsonville_easy_at_70_stays_chill():
  # R~550 m @ 70 mph -> ~1.78 m/s^2 < 1.9 -> NOT trip
  s = base(v_ego=70 * CV.MPH_TO_MS, curve_lat_accel_vision=lat_accel(550, 70), time_to_curve=2.0)
  assert not decide_active(s)[0]


def test_wilsonville_hard_at_90_trips():
  # R~550 m @ 90 mph -> ~2.94 m/s^2 > 1.9 -> trip
  s = base(v_ego=90 * CV.MPH_TO_MS, curve_lat_accel_vision=lat_accel(550, 90), time_to_curve=2.0)
  active, status = decide_active(s)
  assert active and status == "curve"


def test_s_marquam_gentle_at_50_chill_but_70_trips():
  # R~335 m: 50 mph ~1.36 (chill), 70 mph ~2.66 (trip)
  assert not decide_active(base(v_ego=50 * CV.MPH_TO_MS, curve_lat_accel_vision=lat_accel(335, 50), time_to_curve=2.0))[0]
  assert decide_active(base(v_ego=70 * CV.MPH_TO_MS, curve_lat_accel_vision=lat_accel(335, 70), time_to_curve=2.0))[0]


def test_vision_curve_beyond_horizon_does_not_trip():
  # tight curve but time_to_curve 5 s > 3.5 s vision horizon -> no vision trip
  s = base(v_ego=25.0, curve_lat_accel_vision=3.0, time_to_curve=5.0)
  assert not decide_active(s)[0]


def test_vision_curve_ignored_with_blinker():
  s = base(v_ego=22.0, curve_lat_accel_vision=2.5, time_to_curve=3.0, blinker=True)
  assert not decide_active(s)[0]


# ---- curve via the MAP half (MapTargetVelocities target-speed) -------------
def test_map_curve_trips_on_big_slowdown():
  # 60 mph (26.8), upcoming target 40 mph (17.9) -> slowdown ~9 m/s > 3, 200 m / 26.8 = 7.5 s < 10
  s = base(v_ego=60 * CV.MPH_TO_MS, map_target_v=40 * CV.MPH_TO_MS, map_target_dist=200.0)
  active, status = decide_active(s)
  assert active and status == "curve"


def test_map_curve_small_slowdown_stays_chill():
  # only ~1.5 m/s slowdown (< MIN_SLOWDOWN 3) -> not a real curve
  s = base(v_ego=27.0, map_target_v=25.5, map_target_dist=150.0)
  assert not decide_active(s)[0]


def test_map_curve_beyond_lookahead_stays_chill():
  # big slowdown but 400 m ahead at 27 m/s -> 14.8 s > 10 s -> too far yet
  s = base(v_ego=27.0, map_target_v=12.0, map_target_dist=400.0)
  assert not decide_active(s)[0]


def test_map_and_vision_both_quiet_is_chill():
  assert not decide_active(base(v_ego=27.0))[0]


# ---- upcoming_curve helper (MapTargetVelocities parsing + distance) --------
def test_upcoming_curve_picks_binding_point_within_horizon():
  from openpilot.selfdrive.controls.lib.ces_xnor.ces_xnor import upcoming_curve
  # ~111 m north (0.001 deg lat) target 15; ~5.5 km north target 5 (beyond horizon)
  tv = [{"latitude": 45.001, "longitude": -122.0, "velocity": 15.0},
        {"latitude": 45.05, "longitude": -122.0, "velocity": 5.0}]
  mtv, mtd = upcoming_curve(tv, 45.0, -122.0, v_ego=20.0, lookahead_s=10.0)  # horizon 200 m
  assert abs(mtv - 15.0) < 1e-6 and 100 < mtd < 125


def test_upcoming_curve_empty_returns_none():
  from openpilot.selfdrive.controls.lib.ces_xnor.ces_xnor import upcoming_curve
  mtv, mtd = upcoming_curve([], 45.0, -122.0, 20.0, 10.0)
  assert mtv == 0.0 and mtd == float('inf')


# ---- low speed / city ------------------------------------------------------
def test_low_speed_city_no_lead_trips():
  # 30 mph, no lead, < CES_SPEED (40) -> Experimental (city; NO highway gate)
  s = base(v_ego=30 * CV.MPH_TO_MS, has_lead=False)
  active, status = decide_active(s)
  assert active and status == "lowSpeed"


def test_highway_cruise_no_lead_chill():
  s = base(v_ego=65 * CV.MPH_TO_MS, has_lead=False)
  assert not decide_active(s)[0]


# ---- lead-aware thresholds (the high-speed-lead fix) -----------------------
def test_high_speed_following_lead_stays_chill():
  # 60 mph following a matched-speed lead, above CES_SPEED_LEAD (55) -> Chill (ACC closes gap)
  s = base(v_ego=60 * CV.MPH_TO_MS, has_lead=True, lead_vlead=60 * CV.MPH_TO_MS)
  assert not decide_active(s)[0]


def test_low_speed_following_lead_trips():
  # 45 mph following a lead, below CES_SPEED_LEAD (55) -> Experimental
  s = base(v_ego=45 * CV.MPH_TO_MS, has_lead=True, lead_vlead=45 * CV.MPH_TO_MS)
  active, status = decide_active(s)
  assert active and status == "lowSpeed"


def test_closing_on_slow_lead_trips():
  # 65 mph, lead doing 50 mph (15 mph slower > 5 m/s) -> slowLead even above thresholds
  s = base(v_ego=65 * CV.MPH_TO_MS, has_lead=True, lead_vlead=50 * CV.MPH_TO_MS)
  active, status = decide_active(s)
  assert active and status == "slowLead"


def test_lead_pulls_away_returns_chill_path():
  # at 48 mph the lead is gone (has_lead False) -> threshold drops 55->40, 48>40 -> chill
  s = base(v_ego=48 * CV.MPH_TO_MS, has_lead=False)
  assert not decide_active(s)[0]


# ---- stop light ------------------------------------------------------------
def test_stop_light_trips_without_lead():
  s = base(v_ego=40.0, model_should_stop=True, has_lead=False)
  active, status = decide_active(s)
  assert active and status == "stop"


def test_stop_ignored_when_following_lead():
  # following a lead: the lead path handles the stop, not the stop condition
  s = base(v_ego=40.0, model_should_stop=True, has_lead=True, lead_vlead=40.0)
  _, status = decide_active(s)
  assert status != "stop"


# ---- toggles ---------------------------------------------------------------
def test_curve_toggle_off_disables_curve():
  # both map + vision curve present, but curves toggle off -> no curve trip
  s = base(v_ego=50 * CV.MPH_TO_MS, curve_lat_accel_vision=lat_accel(250, 50), time_to_curve=2.0,
           map_target_v=40 * CV.MPH_TO_MS, map_target_dist=200.0,
           toggles={"curves": False, "stops": True, "low_speed": True, "lead": True})
  assert not decide_active(s)[0]


# ---- vision helper ---------------------------------------------------------
def test_vision_curve_lat_accel_picks_max():
  acc, t = vision_curve_lat_accel([0.0, 0.05, 0.1], [20.0, 20.0, 20.0], [0.0, 1.0, 2.0], 20.0)
  assert abs(acc - 2.0) < 1e-6 and t == 2.0


# ---- regression: the planner's experimental flag with CES DISABLED ----------
# This mirrors the planner's exact expression:
#     use_experimental = manual OR (ces.enabled() and ces_request)
# With CES disabled, ces.enabled() is False, so the short-circuit guarantees
# use_experimental == manual for ALL inputs — byte-identical to upstream.
def _planner_flag(manual, ces_enabled, ces_request):
  ces_experimental = ces_request if ces_enabled else False
  return manual or ces_experimental


def test_regression_ces_disabled_is_identical_to_manual():
  for manual in (False, True):
    for ces_request in (False, True):   # whatever CES *would* say is irrelevant when disabled
      assert _planner_flag(manual, ces_enabled=False, ces_request=ces_request) == manual


def test_ces_enabled_only_adds_never_removes_manual():
  # manual ON always stays experimental regardless of CES
  assert _planner_flag(True, ces_enabled=True, ces_request=False) is True
  # manual OFF + CES wants experimental -> experimental
  assert _planner_flag(False, ces_enabled=True, ces_request=True) is True
  # manual OFF + CES wants chill -> chill
  assert _planner_flag(False, ces_enabled=True, ces_request=False) is False


# ---- telemetry / overlay closeness (display-only; must track the decision) ----
def test_curve_closeness_zero_when_cruising_straight():
  pct, src = curve_closeness(base())
  assert pct == 0.0 and src == ""


def test_curve_closeness_ramps_with_vision_lat_accel():
  # half the entry threshold -> ~50% closeness; the source is the vision half
  s = base(v_ego=50 * CV.MPH_TO_MS, curve_lat_accel_vision=C.CURVE_LAT_ACCEL_ENTER * 0.5, time_to_curve=2.0)
  pct, src = curve_closeness(s)
  assert src == "vision"
  assert 0.45 < pct < 0.55


def test_curve_closeness_caps_at_one_when_tripping():
  # well over threshold -> clamped to 1.0, and decide_active agrees it's a curve
  s = base(v_ego=50 * CV.MPH_TO_MS, curve_lat_accel_vision=C.CURVE_LAT_ACCEL_ENTER * 3.0, time_to_curve=2.0)
  pct, _ = curve_closeness(s)
  assert pct == 1.0
  assert decide_active(s) == (True, "curve")


def test_curve_closeness_map_half():
  # upcoming map target speed is MIN_SLOWDOWN below us, within lookahead -> ~100%, source 'map'
  v = 30.0
  s = base(v_ego=v, map_target_v=v - C.CURVE_MAP_MIN_SLOWDOWN, map_target_dist=v * 2.0)
  pct, src = curve_closeness(s)
  assert src == "map"
  assert pct == 1.0


def test_curve_closeness_ignored_when_curves_toggle_off():
  s = base(v_ego=50 * CV.MPH_TO_MS, curve_lat_accel_vision=C.CURVE_LAT_ACCEL_ENTER * 2,
           time_to_curve=2.0, toggles={"curves": False, "stops": True, "low_speed": True, "lead": True})
  pct, src = curve_closeness(s)
  assert pct == 0.0 and src == ""


def test_decision_telemetry_shape_and_consistency():
  s = base(v_ego=50 * CV.MPH_TO_MS, curve_lat_accel_vision=lat_accel(250, 50), time_to_curve=2.0)
  t = decision_telemetry(s)
  assert t["reason"] == "curve" and t["rawActive"] is True
  assert isinstance(t["curvePct"], int) and t["curvePct"] >= 100
  assert t["curveSrc"] == "vision"
  # mapDist must be a finite number (never inf) so it JSON-serializes cleanly for the overlay
  assert t["mapDist"] == 0.0
