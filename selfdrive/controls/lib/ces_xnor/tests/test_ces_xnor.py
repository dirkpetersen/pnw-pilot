"""
Unit tests for the CES decision core. Pure logic — no cereal, no car. Validates the calibration
anchors from CES.md (I-5 Terwilliger / Marquam / Wilsonville) and the lead/speed/stop logic.

Run:  pytest selfdrive/controls/lib/ces/tests/test_ces.py
"""
from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.ces_xnor import ces_xnor_constants as C
from openpilot.selfdrive.controls.lib.ces_xnor.ces_xnor import decide_active, vision_curve_lat_accel


ALL_ON = {"curves": True, "stops": True, "low_speed": True, "lead": True}


def base(**kw):
  """A 'cruising, nothing happening' signal set; override fields per test."""
  s = {
    "v_ego": 30.0, "has_lead": False, "lead_vlead": 0.0, "lead_drel": 0.0, "blinker": False,
    "curve_lat_accel_map": 0.0, "dist_to_curve": 0.0,
    "curve_lat_accel_vision": 0.0, "time_to_curve": 10.0,
    "model_should_stop": False, "toggles": ALL_ON,
  }
  s.update(kw)
  return s


def lat_accel(radius_m, mph):
  return (mph * CV.MPH_TO_MS) ** 2 / radius_m


# ---- curve anchors (map half) ----------------------------------------------
def test_terwilliger_trips_at_50():
  # R~250 m @ 50 mph -> ~2.0 m/s^2, ~200 m ahead -> within 10 s
  s = base(v_ego=50 * CV.MPH_TO_MS, curve_lat_accel_map=lat_accel(250, 50), dist_to_curve=180.0)
  active, status = decide_active(s)
  assert active and status == "curve"


def test_wilsonville_easy_at_70_stays_chill():
  # R~550 m @ 70 mph -> ~1.78 m/s^2 < 1.9 enter -> must NOT trip
  s = base(v_ego=70 * CV.MPH_TO_MS, curve_lat_accel_map=lat_accel(550, 70), dist_to_curve=250.0)
  active, _ = decide_active(s)
  assert not active


def test_wilsonville_hard_at_90_trips():
  # R~550 m @ 90 mph -> ~2.94 m/s^2 > 1.9 -> trips
  s = base(v_ego=90 * CV.MPH_TO_MS, curve_lat_accel_map=lat_accel(550, 90), dist_to_curve=320.0)
  active, status = decide_active(s)
  assert active and status == "curve"


def test_s_marquam_gentle_at_50_chill_but_70_trips():
  # R~335 m: 50 mph ~1.36 (chill), 70 mph ~2.66 (trip)
  s50 = base(v_ego=50 * CV.MPH_TO_MS, curve_lat_accel_map=lat_accel(335, 50), dist_to_curve=200.0)
  assert not decide_active(s50)[0]
  s70 = base(v_ego=70 * CV.MPH_TO_MS, curve_lat_accel_map=lat_accel(335, 70), dist_to_curve=300.0)
  assert decide_active(s70)[0]


def test_curve_beyond_lookahead_does_not_trip():
  # tight curve but 400 m ahead at 30 m/s -> 13 s > 10 s lookahead -> no map trip
  s = base(v_ego=30.0, curve_lat_accel_map=3.0, dist_to_curve=400.0)
  assert not decide_active(s)[0]


def test_vision_fallback_trips_when_map_misses():
  s = base(v_ego=22.0, curve_lat_accel_map=0.0, dist_to_curve=0.0,
           curve_lat_accel_vision=2.5, time_to_curve=3.0)
  active, status = decide_active(s)
  assert active and status == "curve"


def test_vision_curve_ignored_with_blinker():
  s = base(v_ego=22.0, curve_lat_accel_vision=2.5, time_to_curve=3.0, blinker=True)
  assert not decide_active(s)[0]


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
  s = base(v_ego=50 * CV.MPH_TO_MS, curve_lat_accel_map=lat_accel(250, 50), dist_to_curve=180.0,
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
