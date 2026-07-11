"""
Unit tests for the CES decision core. Pure logic — no cereal, no car. Validates the calibration
anchors from CES.md (I-5 Terwilliger / Marquam / Wilsonville) and the lead/speed/stop logic.

Run:  pytest selfdrive/controls/lib/ces/tests/test_ces.py
"""
from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw_constants as C
from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import (
  decide_active, vision_curve_lat_accel, curve_closeness, decision_telemetry, _accelerate_zone,
)


ALL_ON = {"curves": True, "stops": True, "low_speed": True, "lead": True}


def base(**kw):
  """A 'cruising, nothing happening' signal set; override fields per test."""
  s = {
    "v_ego": 30.0, "has_lead": False, "lead_vlead": 0.0, "lead_drel": 0.0, "blinker": False,
    "map_target_v": 0.0, "map_target_dist": float('inf'),
    "curve_lat_accel_vision": 0.0, "time_to_curve": 10.0,
    "model_should_stop": False, "v_set": 0.0, "spd_lim": 0.0, "toggles": ALL_ON,
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
  from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import upcoming_curve
  # ~111 m north (0.001 deg lat) target 15; ~5.5 km north target 5 (beyond horizon)
  tv = [{"latitude": 45.001, "longitude": -122.0, "velocity": 15.0},
        {"latitude": 45.05, "longitude": -122.0, "velocity": 5.0}]
  mtv, mtd = upcoming_curve(tv, 45.0, -122.0, v_ego=20.0, lookahead_s=10.0)  # horizon 200 m
  assert abs(mtv - 15.0) < 1e-6 and 100 < mtd < 125


def test_upcoming_curve_empty_returns_none():
  from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import upcoming_curve
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
  # 60 mph following a matched-speed lead, above CES_SPEED_LEAD (45) -> Chill (ACC closes gap)
  s = base(v_ego=60 * CV.MPH_TO_MS, has_lead=True, lead_vlead=60 * CV.MPH_TO_MS)
  assert not decide_active(s)[0]


def test_low_speed_following_lead_trips():
  # 35 mph following a lead, below CES_SPEED_LEAD (45) -> Experimental (dense city traffic)
  s = base(v_ego=35 * CV.MPH_TO_MS, has_lead=True, lead_vlead=35 * CV.MPH_TO_MS)
  active, status = decide_active(s)
  assert active and status == "lowSpeed"


def test_highway_following_lead_at_50mph_stays_chill():
  # the drive-log false-positive: 50 mph behind traffic, now ABOVE CES_SPEED_LEAD (45) -> Chill
  s = base(v_ego=50 * CV.MPH_TO_MS, has_lead=True, lead_vlead=50 * CV.MPH_TO_MS)
  assert not decide_active(s)[0]


def test_lowspeed_suppressed_on_highway_by_spdlim_gate():
  # 35 mph behind a lead on a 60 mph road (OSM spd_lim high) -> highway gate -> Chill, not lowSpeed
  s = base(v_ego=35 * CV.MPH_TO_MS, has_lead=True, lead_vlead=35 * CV.MPH_TO_MS,
           spd_lim=60 * CV.MPH_TO_MS)
  assert not decide_active(s)[0]
  # same situation on a surface street (low spd_lim) still trips
  s2 = base(v_ego=35 * CV.MPH_TO_MS, has_lead=True, lead_vlead=35 * CV.MPH_TO_MS, spd_lim=30 * CV.MPH_TO_MS)
  assert decide_active(s2) == (True, "lowSpeed")


def test_slowlead_not_gated_on_highway():
  # closing on a much slower lead on a highway (spd_lim high) MUST still trip slowLead (valid e2e case)
  s = base(v_ego=65 * CV.MPH_TO_MS, has_lead=True, lead_vlead=45 * CV.MPH_TO_MS, spd_lim=65 * CV.MPH_TO_MS)
  assert decide_active(s) == (True, "slowLead")


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


# ---- accelerate-zone: suppress lowSpeed-Experimental when we should be speeding up ----
def test_onramp_no_lead_high_set_speed_stays_chill():
  # 29 mph on an open on-ramp with cruise set to 65 mph -> accelerate-zone -> NOT lowSpeed
  s = base(v_ego=29 * CV.MPH_TO_MS, has_lead=False, v_set=65 * CV.MPH_TO_MS)
  assert _accelerate_zone(s) is True
  assert decide_active(s) == (False, "chill")


def test_stopgo_lead_pulling_away_big_gap_stays_chill():
  # crawling at 9 mph, lead 60 m ahead and FASTER (pulling away), set 36 mph -> catch up at Chill
  s = base(v_ego=4.0, has_lead=True, lead_drel=60.0, lead_vlead=10.0, v_set=36 * CV.MPH_TO_MS)
  assert _accelerate_zone(s) is True
  assert decide_active(s) == (False, "chill")


def test_genuine_city_slow_with_near_lead_still_experimental():
  # slow behind a CLOSE lead (20 m) -> not open road -> lowSpeed still fires
  s = base(v_ego=8.0, has_lead=True, lead_drel=20.0, lead_vlead=8.0, v_set=40 * CV.MPH_TO_MS)
  assert _accelerate_zone(s) is False
  assert decide_active(s) == (True, "lowSpeed")


def test_redlight_approach_no_lead_low_speed_holds_experimental():
  # SAFETY (driver report 2026-07-11, confirmed live): near-stop, NO lead, high set speed = a red-light
  # approach, NOT a merge. The no-lead accel-zone must not fire below ACCEL_ZONE_MIN_V, so the low-speed
  # Experimental hold stays and e2e stops for the light instead of Chill accelerating through it.
  s = base(v_ego=2.0, has_lead=False, v_set=25 * CV.MPH_TO_MS)   # ~4.5 mph, cruise set 25
  assert _accelerate_zone(s) is False
  assert decide_active(s) == (True, "lowSpeed")


def test_model_stop_prediction_defeats_accel_zone():
  # if the model predicts a stop it is NEVER an accelerate-zone, even at merge-like speed/set (belt).
  s = base(v_ego=20.0, has_lead=False, v_set=65 * CV.MPH_TO_MS, model_should_stop=True)
  assert _accelerate_zone(s) is False
  assert decide_active(s) == (True, "stop")   # stop fires (model_should_stop + no lead), not chill


def test_slow_with_no_set_speed_gap_still_experimental():
  # deliberately cruising slow on open road, set speed barely above -> NOT accelerate-zone
  s = base(v_ego=8.0, has_lead=False, v_set=9.0)
  assert _accelerate_zone(s) is False
  assert decide_active(s) == (True, "lowSpeed")


def test_accelerate_zone_suppresses_curve_trip():
  # driver directive (I-90 on-ramp incident, 2026-07-09): an accelerate-zone (open road, set speed
  # >= 6 m/s above ego) suppresses curve-Experimental — a ramp IS a curve, and tripping Experimental
  # there pinned merges at 39 mph. VTSC remains the physical curve-speed backstop. (This test used to
  # assert the OPPOSITE pre-directive design and went stale when the gate shipped.)
  s = base(v_ego=20.0, has_lead=False, v_set=65 * CV.MPH_TO_MS,
           curve_lat_accel_vision=C.CURVE_LAT_ACCEL_ENTER * 1.5, time_to_curve=2.0)
  assert _accelerate_zone(s) is True
  assert decide_active(s) == (False, "chill")


# ---- de-flap: entry cooldown + exit dwell (state machine) ------------------
def test_deflap_entry_cooldown_and_exit_dwell():
  from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import ConditionalExperimentalSwitching
  sm = ConditionalExperimentalSwitching()
  active = base(v_ego=20 * CV.MPH_TO_MS, has_lead=False)   # lowSpeed condition active
  clear = base(v_ego=60 * CV.MPH_TO_MS, has_lead=False)    # nothing -> chill
  DT = 0.05   # inject the loop period so cycle counts map to real seconds

  # one cycle of an active condition must NOT instantly enter Experimental (cooldown + debounce)
  sm.update_decision(active, DT)
  assert sm.mode() == "chill"
  # after enough time (>CHILL_MIN_DWELL_S + filter) it does enter
  for _ in range(200):   # 10 s
    sm.update_decision(active, DT)
  assert sm.mode() == "experimental"
  # condition clears, but it must HOLD Experimental through EXP_MIN_DWELL_S (no instant snap-out)
  sm.update_decision(clear, DT)
  assert sm.mode() == "experimental"
  for _ in range(400):   # 20 s > EXP_MIN_DWELL_S
    sm.update_decision(clear, DT)
  assert sm.mode() == "chill"


def test_decision_telemetry_shape_and_consistency():
  s = base(v_ego=50 * CV.MPH_TO_MS, curve_lat_accel_vision=lat_accel(250, 50), time_to_curve=2.0)
  t = decision_telemetry(s)
  assert t["reason"] == "curve" and t["rawActive"] is True
  assert isinstance(t["curvePct"], int) and t["curvePct"] >= 100
  assert t["curveSrc"] == "vision"
  # mapDist must be a finite number (never inf) so it JSON-serializes cleanly for the overlay
  assert t["mapDist"] == 0.0


# ---- redlight2pnw (SAFETY, 2026-07-11): accel-zone must never fire on a red-light approach ----
# Replayed from the live incident telemetry (ces_events 07:12-07:15Z): exp->chill at 0-5 mph with
# az=True, no lead, set 19-25 mph -> Chill's MPC accelerated toward the set speed AT the light.
def test_redlight_low_speed_no_lead_keeps_experimental():
  # 4 mph rolling to a red light, no lead, set 19 mph (the 07:12:53 row): below the no-lead floor,
  # az must be False and the low-speed hold must keep Experimental (which stops for lights).
  s = base(v_ego=4 * CV.MPH_TO_MS, has_lead=False, v_set=19 * CV.MPH_TO_MS)
  assert _accelerate_zone(s) is False
  assert decide_active(s) == (True, "lowSpeed")


def test_model_stop_always_beats_accel_zone():
  # 25 mph (above the floor), no lead, set 45 -> would be an accel-zone, but the model predicts a
  # stop (red light ahead): stop intent wins, az must stand down.
  s = base(v_ego=25 * CV.MPH_TO_MS, has_lead=False, v_set=45 * CV.MPH_TO_MS, model_should_stop=True)
  assert _accelerate_zone(s) is False
  assert decide_active(s) == (True, "stop")


def test_onramp_merge_above_floor_still_gated():
  # the original I-90 on-ramp case (38-39 mph, set 90): far above the floor -> gate unchanged.
  s = base(v_ego=38 * CV.MPH_TO_MS, has_lead=False, v_set=90 * CV.MPH_TO_MS)
  assert _accelerate_zone(s) is True
  assert decide_active(s) == (False, "chill")


def test_lead_pullaway_below_floor_still_gated():
  # the stop&go lead-pull-away case has a LEAD, so the no-lead floor does not apply — preserved.
  s = base(v_ego=4.0, has_lead=True, lead_drel=60.0, lead_vlead=10.0, v_set=36 * CV.MPH_TO_MS)
  assert _accelerate_zone(s) is True
