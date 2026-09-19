"""curvedbtel2pnw — CURVEDB2PNW.md PHASE 1 telemetry. TELEMETRY ONLY; nothing here changes control.

What each section of the design this file pins, and the mutation each test exists to kill
(tools/../_scratch/mutate_curvedbtel.py applies them by literal source substitution):

  3.1 P1-A  achieved curvature for BOTH cars from livePose, because CS.yawRate is dead on the Tesla
            (measured: slKActl exactly 0.0 on 7,218 of 7,221 moving ticks, 2026-09-03 corpus).
  3.2 P1-B  an exactly-zero curvature/lateral field logs as null -- M3 "zero logged as 0.0".
  3.3 P1-C  kPeak is the per-second MAX of max(|achieved|, |commanded|) -- M1 "instantaneous sample
            instead of the peak", M2 "achieved only instead of max(cmd, actl)".
  3.4 P1-D  mapLat/mapLon are the CANDIDATE's coordinates -- M4 "taken from the truck".
  3.5 P1-E  one per-second OR of the disqualifiers -- M5 "the OR is dropped".
  3.6 P1-F  driver steering torque, and ONLY that (wiper/ambient are explicitly out of scope).
(Section 3.8's archive path is the OTHER half of Phase 1 and is tested next to the rotation it
changes, in test_event_log_rotation.py -- it retains bytes rather than writing fields, and it ships
as its own commit.)

The integration test at the bottom drives the REAL CESController.experimental_request at 100 Hz, so
a mutation that survives the pure-unit tests by breaking the WIRING still dies: that is the class of
defect (a field that is logged but never computed) section 3.7 exists to prevent, and which visK and
icbmKVis both belonged to.
"""
import copy
import json
import math
import types

import pytest

from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw_constants as C
from openpilot.selfdrive.controls.lib.ces_pnw import park_tick_gate
from openpilot.selfdrive.controls.lib import pnw_vehicle as pv
from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_curvelead2pnw import FakeCP, LAT0, LON0, _model, _scene
from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_ces_mode_read_failure_logged import (
  LIGHTNING, MPH, TESLA, _P,
)
# The _icbm_step stub pattern, reused rather than re-grown: the two tests that need a vis/restore
# source cannot get one out of _drive (its truck never moves, so no episode ever clears).
from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_icbm_mapfirst import (
  _icbm_stub, _pt_north, _run, _sig,
)

NS = types.SimpleNamespace


def _icbm_stub_at(monkeypatch, tmp_path, targets=None, dist=300.0, map_v=8.0):
  """An _icbm_step stub sitting at LAT0/LON0 with (by default) a binding far candidate ~300 m north."""
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(tmp_path / "absent.json"))
  monkeypatch.setattr(pv, "RAIN_CONFIG_PATH", str(tmp_path / "absent-rain.json"))
  mgr, step = _icbm_stub()
  mgr._cur_lat, mgr._cur_lon = LAT0, LON0
  mgr._map_targets = ([_pt_north(LAT0, LON0, d, map_v) for d in (dist, dist + 40.0)]
                      if targets is None else targets)
  return mgr, step


# =====================================================================================================
# 3.2 -- exact zero is null, never a measurement
# =====================================================================================================
class TestZeroIsNull:
  def test_exact_zero_becomes_null(self):
    assert m._zero_is_null(0.0) is None
    assert m._zero_is_null(-0.0) is None          # -0.0 == 0.0: the dead-sensor case, not a reading

  def test_a_real_reading_survives_unchanged(self):
    assert m._zero_is_null(0.0021) == 0.0021
    assert m._zero_is_null(-3.5) == -3.5
    assert m._zero_is_null(1e-6) == 1e-6          # the smallest value the 1e-6 rounding can carry

  def test_none_stays_none(self):
    assert m._zero_is_null(None) is None


# =====================================================================================================
# 3.1 / 3.3 -- the curvature sources
# =====================================================================================================
class TestCurvatureFromYaw:
  def test_it_is_yaw_over_speed(self):
    assert m._curvature_from_yaw(0.2, 20.0) == pytest.approx(0.01)
    assert m._curvature_from_yaw(-0.2, 20.0) == pytest.approx(-0.01)   # signed: direction is data

  def test_the_speed_floor_matches_controlsd(self):
    """CURVE_MIN_SPEED must stay equal to the constant controlsd divides by, or kPeak's achieved half
    stops being comparable to the slKActl/achLat pair section 3.7's invariant checks it against."""
    from openpilot.selfdrive.controls.lib.drive_helpers import MIN_SPEED
    assert m.CURVE_MIN_SPEED == MIN_SPEED
    assert m._curvature_from_yaw(0.2, 0.05) == pytest.approx(0.2 / m.CURVE_MIN_SPEED)

  def test_bad_input_is_null_never_zero_and_never_raises(self):
    assert m._curvature_from_yaw(None, 20.0) is None
    assert m._curvature_from_yaw(0.2, None) is None
    assert m._curvature_from_yaw(float("nan"), 20.0) is None
    assert m._curvature_from_yaw(float("inf"), 20.0) is None
    assert m._curvature_from_yaw("x", 20.0) is None


class TestPoseCurvature:
  """3.1 (P1-A): THE fix for defect D1 -- the Tesla has no yaw-rate CAN signal at all, so the only
  achieved curvature it can ever have comes from the localizer."""

  def test_it_reads_angular_velocity_device_z_AND_NEGATES_IT(self):
    """THE SIGN IS THE POINT (measured on the truck 2026-09-17; this shipped inverted for one day).

    The device frame's z runs opposite to the vehicle convention `carState.yawRate` uses
    (positive = left), so the raw reading describes every bend as the mirror of the one driven.
    openpilot negates the same quantity itself at paramsd.py:98. On the first real corpus, 285 of
    292 real bends disagreed in sign with slKActl while |ratio| was 0.992 -- right magnitude, wrong
    direction -- and against the steering wheel as a third witness kPose agreed 0.5 % of the time
    where slKActl agreed 95.3 %.

    This test previously asserted the UNNEGATED value, i.e. it pinned the bug. A test can hold a
    defect in place as firmly as it can hold a contract."""
    lp = NS(angularVelocityDevice=NS(x=0.0, y=0.0, z=0.25, valid=True))
    assert m._pose_curvature(lp, 25.0) == pytest.approx(-0.01)
    # ...and it must agree in SIGN with the CAN-derived pair, which is the convention of record.
    assert m._pose_curvature(NS(angularVelocityDevice=NS(z=-0.25, valid=True)), 25.0) > 0
    assert m._curvature_from_yaw(0.25, 25.0) > 0

  def test_it_works_where_CS_yawRate_is_dead(self):
    """The Tesla case, stated as an invariant: CAN says 0.0 (a lie), livePose says the truth."""
    lp = NS(angularVelocityDevice=NS(z=0.25, valid=True))
    assert m._curvature_from_yaw(0.0, 25.0) == 0.0             # what carState offers on the Raven
    assert m._pose_curvature(lp, 25.0) == pytest.approx(-0.01)  # what livePose offers instead

  def test_an_invalid_pose_is_null_not_zero(self):
    """An uninitialised/diverged localizer must NOT read as a perfectly straight road -- that is
    exactly the reading that voided the design's v1 proof of concept."""
    assert m._pose_curvature(NS(angularVelocityDevice=NS(z=0.25, valid=False)), 25.0) is None

  def test_a_missing_message_is_null_and_never_raises(self):
    assert m._pose_curvature(None, 25.0) is None
    assert m._pose_curvature(NS(), 25.0) is None


class TestCurvePeak:
  """3.3 (P1-C). The record samples at ~1 Hz while control runs at 100 Hz -- 99 of every 100 readings
  were being thrown away, and the ones that matter are the peaks."""

  def test_it_reports_the_peak_not_the_last_sample(self):
    """M1. A curve entered and exited inside one second must still be visible in that second."""
    p = m.CurvePeak()
    for k in (0.001, 0.004, 0.012, 0.004, 0.0005):     # tightens, then straightens again
      p.step(k, None, None)
    k_peak, _, n, _ = p.take()
    assert k_peak == pytest.approx(0.012), "the per-second MAX, not the 0.0005 final sample"
    assert n == 5

  def test_max_of_commanded_and_achieved_not_achieved_alone(self):
    """M2 / defect D2. Achieved is bounded by steering authority: on the 2026-09-08 19:44 PSCM
    LimitReached event the REQUEST went 0.1225 -> 0.162 rad with the wheel flat. Where the truck
    cannot follow, achieved under-reads the road -- precisely on the curves that matter."""
    p = m.CurvePeak()
    p.step(0.004, 0.011, None)                          # saturated: asked for much more than achieved
    k_peak, _, _, _ = p.take()
    assert k_peak == pytest.approx(0.011), "commanded must win when the truck could not follow"

  def test_achieved_wins_when_it_exceeds_the_command(self):
    p = m.CurvePeak()
    p.step(0.011, 0.004, None)
    assert p.take()[0] == pytest.approx(0.011)

  def test_it_is_a_magnitude_so_a_right_hand_curve_counts(self):
    p = m.CurvePeak()
    p.step(-0.012, -0.004, None)
    assert p.take()[0] == pytest.approx(0.012)

  def test_the_pose_peak_is_accumulated_separately(self):
    """3.1 at peak resolution. On the Tesla this is the ONLY achieved curvature there is, so it
    cannot be folded into kPeak's CAN-derived half."""
    p = m.CurvePeak()
    p.step(0.0, 0.002, 0.009)                           # CAN dead (Tesla), localizer alive
    k_peak, k_pose_peak, _, _ = p.take()
    assert k_pose_peak == pytest.approx(0.009)
    assert k_peak == pytest.approx(0.002), "kPeak stays the CAN+commanded pair (section 3.3's formula)"

  def test_none_inputs_do_not_contribute_but_still_count_the_tick(self):
    p = m.CurvePeak()
    for _ in range(7):
      p.step(None, None, None)
    k_peak, k_pose_peak, n, _ = p.take()
    assert (k_peak, k_pose_peak) == (0.0, 0.0)
    assert n == 7, "a window that ran and saw nothing must be distinguishable from one that never ran"

  def test_take_drains_so_the_next_window_starts_clean(self):
    p = m.CurvePeak()
    p.step(0.02, None, None)
    assert p.take()[0] == pytest.approx(0.02)
    assert p.take() == (0.0, 0.0, 0, 0), "a stale peak must never bleed into the next record"

  def test_disqualifier_bits_or_over_the_window(self):
    """3.5 (P1-E): the OR is over the whole window -- a 30 ms blinker flick inside the second still
    disqualifies it, which a 1 Hz sample of the flag would miss."""
    p = m.CurvePeak()
    p.step(0.01, 0.01, 0.01, 0)
    p.step(0.01, 0.01, 0.01, m.DQ_BLINKER)
    p.step(0.01, 0.01, 0.01, 0)
    assert p.take()[3] == m.DQ_BLINKER

  def test_it_never_raises_on_garbage(self):
    p = m.CurvePeak()
    with pytest.raises(TypeError):
      abs("x")                              # sanity: the thing we are asserting is NOT swallowed here
    p.step(None, None, None, 0)             # the caller (_curve_peak_step) owns the guarding
    assert p.n == 1


class TestDqNames:
  def test_each_bit_renders_and_the_order_is_stable(self):
    assert m._dq_names(0) == ""
    assert m._dq_names(m.DQ_SAT) == "sat"
    assert m._dq_names(m.DQ_DRIVER | m.DQ_BLINKER) == "drv,blnk"
    assert m._dq_names(m.DQ_SAT | m.DQ_DRIVER | m.DQ_LANECHG | m.DQ_BLINKER) == "sat,drv,lc,blnk"

  def test_garbage_is_empty_never_an_exception(self):
    assert m._dq_names(None) == ""
    assert m._dq_names("x") == ""


# =====================================================================================================
# 3.4 -- where the CURVE is, not where the truck is
# =====================================================================================================
def _pt(north_m, east_m=0.0, v=0.0):
  return {"latitude": LAT0 + north_m / 111320.0,
          "longitude": LON0 + east_m / (111320.0 * math.cos(math.radians(LAT0))),
          "velocity": v}


class TestMapCandidatePoint:
  def test_it_returns_the_candidate_not_the_truck(self):
    """M4. Map/far ticks sit median 49 m / p90 129 m BEFORE the candidate, which is why clustering
    truck positions smeared one 2026-09-08 episode across four or five "sites"."""
    pts = [_pt(0.0), _pt(100.0), _pt(300.0), _pt(500.0)]
    lat, lon = m.map_candidate_point(pts, LAT0, LON0, 300.0)
    assert (lat, lon) != (None, None)
    assert lat != LAT0, "the truck's own latitude is not an answer"
    assert m._haversine_m(LAT0, LON0, lat, lon) == pytest.approx(300.0, abs=1.0)

  def test_the_match_is_exact_for_a_distance_taken_from_the_same_list(self):
    pts = [_pt(0.0), _pt(137.0), _pt(412.0)]
    d = m._haversine_m(LAT0, LON0, pts[1]["latitude"], pts[1]["longitude"])
    lat, lon = m.map_candidate_point(pts, LAT0, LON0, d)
    assert (lat, lon) == pytest.approx((pts[1]["latitude"], pts[1]["longitude"]))

  def test_no_candidate_this_tick_is_null(self):
    """decision_telemetry renders an infinite mapDist as 0.0, so 0.0 means "no candidate"."""
    pts = [_pt(100.0), _pt(300.0)]
    assert m.map_candidate_point(pts, LAT0, LON0, 0.0) == (None, None)
    assert m.map_candidate_point(pts, LAT0, LON0, None) == (None, None)
    assert m.map_candidate_point(pts, LAT0, LON0, float("inf")) == (None, None)
    assert m.map_candidate_point([], LAT0, LON0, 300.0) == (None, None)
    assert m.map_candidate_point(pts, None, None, 300.0) == (None, None)

  def test_a_match_outside_the_tolerance_is_null_not_a_wrong_coordinate(self):
    """The point list changed underneath us -> say nothing, rather than name a point 300 m away."""
    pts = [_pt(100.0)]
    assert m.map_candidate_point(pts, LAT0, LON0, 400.0) == (None, None)
    assert m.CURVE_CAND_TOL_M == 30.0     # section 3.7's own invariant bound

  def test_malformed_points_are_skipped_and_it_never_raises(self):
    pts = [{"latitude": "x", "longitude": 0.0}, {}, _pt(float("nan")), _pt(200.0)]
    lat, lon = m.map_candidate_point(pts, LAT0, LON0, 200.0)
    assert m._haversine_m(LAT0, LON0, lat, lon) == pytest.approx(200.0, abs=1.0)


# =====================================================================================================
# the record: does the value actually reach ces_events.jsonl
# =====================================================================================================
def _rec(tele=None, **over):
  """Drive the REAL _event_record("tick", ...) with the permissive stub test_ces_record_fields
  established (a `__getattr__` returning None, so unrelated fields this record grows over time do
  not have to be tracked here)."""
  class Stub:
    def __getattr__(self, n):
      return None

  g = Stub()
  g._vtsc_tele = {}
  g._sa_tele = {}
  g._map_targets = []
  g._speed_limit = 11.2
  g._button = C.BTN_CES
  g._ces2_urg = 0.0
  g._ces2_div = type("D", (), {"count": 0})()
  g._gl = type("G", (), {"state": None, "status": lambda s: None})()
  g._sm = type("S", (), {"state": None, "status": lambda s: None})()
  g._icbm_k_at = g._icbm_k_at_d = g._icbm_k_at_n = g._icbm_k_at_gap = 0.0
  g._icbm_k = g._icbm_k_dist = g._icbm_k_v = g._icbm_k_n = 0.0
  g._icbm_k_ahead = True
  g._icbm_floor_lim = 0.0
  g._icbm_floor_hit = False
  g._cur_lat, g._cur_lon = LAT0, LON0
  g._curve_peak = m.CurvePeak()
  g._gear_name = "drive"                                 # parkgate2pnw
  g._park_gate = park_tick_gate.ParkTickGate()           # parkgate2pnw (real gate -- `park` is a bool)
  for k, v in over.items():
    setattr(g, k, v)
  return m.CESController._event_record.__get__(g)("tick", tele or {"vEgo": 25.0})


class TestRecordFields:
  def test_every_declared_key_is_emitted(self):
    """The CURVE_TELE_KEYS contract: a key declared but never emitted is a null column that reads as
    "the feature did not trigger" -- this file's history has four of those (visK, icbmKVis,
    waysel2pnw's eight, lcSpdA)."""
    rec = _rec()
    for k in m.CURVE_TELE_KEYS:
      assert k in rec, f"{k} is declared in CURVE_TELE_KEYS but never reaches the record"

  def test_the_peak_reaches_the_record_and_is_the_peak(self):
    """M1 at the record level: the wiring must carry the accumulator's MAX, not a fresh sample."""
    peak = m.CurvePeak()
    for k in (0.001, 0.013, 0.002):
      peak.step(k, None, None)
    rec = _rec(_curve_peak=peak)
    assert rec["kPeak"] == pytest.approx(0.013)
    assert rec["kPeakN"] == 3

  def test_the_record_drains_the_accumulator(self):
    """Two records in a row must not both carry the first one's peak."""
    peak = m.CurvePeak()
    peak.step(0.02, None, None)
    assert _rec(_curve_peak=peak)["kPeak"] == pytest.approx(0.02)
    assert _rec(_curve_peak=peak)["kPeak"] is None, "a stale peak would fake a curve on straight road"

  def test_a_measured_zero_logs_as_null_with_the_tick_count_intact(self):
    """M3. kPeakN is what separates "measured, and it was zero/unmeasurable" from "never ran"."""
    peak = m.CurvePeak()
    for _ in range(100):
      peak.step(0.0, 0.0, 0.0)
    rec = _rec(_curve_peak=peak)
    assert rec["kPeak"] is None and rec["kPoseP"] is None
    assert rec["kPeakN"] == 100

  def test_an_accumulator_that_never_ran_is_null_everywhere(self):
    rec = _rec(_curve_peak=m.CurvePeak())
    assert rec["kPeakN"] == 0
    assert rec["kPeak"] is None and rec["kPoseP"] is None
    assert rec["dq"] is None, "no ticks is not the same claim as 'clean second'"

  def test_pose_curvature_and_its_lateral_accel_reach_the_record(self):
    rec = _rec(_pose_k=0.008, tele={"vEgo": 25.0})
    assert rec["kPose"] == pytest.approx(0.008)
    assert rec["achLatPose"] == pytest.approx(0.008 * 25.0 ** 2, abs=0.01)   # k * v^2

  def test_pose_fields_are_null_when_the_localizer_had_nothing(self):
    rec = _rec(_pose_k=None)
    assert rec["kPose"] is None and rec["achLatPose"] is None

  def test_the_existing_achieved_pair_now_nulls_on_exact_zero(self):
    """3.2 applied to the two fields defect D1 is actually about."""
    rec = _rec(_sl_k_actl=0.0, tele={"vEgo": 25.0})
    assert rec["slKActl"] is None, "0.0 reads as a perfectly straight road; it is a dead sensor"
    assert rec["achLat"] is None
    alive = _rec(_sl_k_actl=0.004, tele={"vEgo": 25.0})
    assert alive["slKActl"] == pytest.approx(0.004) and alive["achLat"] == pytest.approx(2.5, abs=0.01)

  def test_commanded_curvature_is_deliberately_left_alone(self):
    """Scope discipline: slKCmd 0.0 is a genuine command (and slLatAct already says whether lateral
    was active), so it must NOT be swept into the zero->null rule."""
    assert _rec(_sl_k_cmd=0.0)["slKCmd"] == 0.0

  def test_the_disqualifier_roll_up_reaches_the_record_with_its_reason(self):
    """M5."""
    peak = m.CurvePeak()
    peak.step(0.01, 0.01, 0.01, m.DQ_SAT | m.DQ_DRIVER)
    rec = _rec(_curve_peak=peak)
    assert rec["dq"] is True
    assert rec["dqWhy"] == "sat,drv"

  def test_a_clean_second_is_false_not_null(self):
    peak = m.CurvePeak()
    peak.step(0.01, 0.01, 0.01, 0)
    rec = _rec(_curve_peak=peak)
    assert rec["dq"] is False and rec["dqWhy"] is None

  def test_driver_steering_torque_reaches_the_record(self):
    """3.6 (P1-F). Kept per-car: Ford SteeringColumnTorque is +-8 Nm, Tesla EPAS +-20.5 Nm."""
    assert _rec(_str_tq=1.75)["strTq"] == pytest.approx(1.75)
    assert _rec(_str_tq=-6.5)["strTq"] == pytest.approx(-6.5)
    assert _rec(_str_tq=0.0)["strTq"] is None       # 3.2 again: zero torque or a dead signal?
    assert _rec(_str_tq=None)["strTq"] is None

  def test_the_candidate_position_reaches_the_record_and_is_not_the_truck(self):
    """M4 at the record level."""
    pts = [_pt(0.0), _pt(150.0), _pt(332.0)]
    d = round(m._haversine_m(LAT0, LON0, pts[2]["latitude"], pts[2]["longitude"]), 0)
    rec = _rec(_map_targets=pts, tele={"vEgo": 25.0, "mapDist": d})
    assert rec["mapLat"] is not None and rec["mapLon"] is not None
    assert rec["mapLat"] != rec["lat"], "logging the truck's own position answers nothing"
    assert m._haversine_m(rec["lat"], rec["lon"], rec["mapLat"], rec["mapLon"]) == pytest.approx(d, abs=30.0)

  def test_no_map_candidate_means_null_coordinates(self):
    pts = [_pt(0.0), _pt(150.0)]
    rec = _rec(_map_targets=pts, tele={"vEgo": 25.0, "mapDist": 0.0})
    assert rec["mapLat"] is None and rec["mapLon"] is None

  def test_a_point_with_no_fix_to_measure_it_from_nulls_BOTH(self):
    """Fable round 3. ICBM latches a point up to 250 ms before the record is built, and _read_map can
    null the fix in between -- _publish_status runs BEFORE _icbm_step in the same 100 Hz cycle. The
    point branch used to emit coordinates with a null mapCandD, which is precisely the orphan this
    feature's own checker (I3c) FAILS on: a false alarm from a GPS blip, in the acceptance instrument
    for its first drive. Null both, exactly as map_candidate_point does with no fix."""
    class Stub:
      def __getattr__(self, n):
        return None
    frag = m._curve_tele(Stub(), 25.0, 320.0, (47.00269, -122.0))
    assert (frag["mapLat"], frag["mapLon"], frag["mapCandD"]) == (None, None, None)

  def test_a_FAILED_latch_with_a_good_fix_also_nulls_rather_than_raising(self):
    """The other half of the same guard (Fable round 4, F2). If the latch could not resolve a point
    -- `map_candidate_point` returning (None, None) because the path list changed underneath it --
    the record has a fix but no point. Guarding on the fix alone would then hand None to the
    haversine and RAISE, out of a function whose callers do not wrap it: `_publish_status` is called
    unwrapped from experimental_request, and selfdrived's own backstop would force chill for that
    cycle and skip that cycle's ICBM publish. Near-unreachable by construction; still guarded."""
    class Stub:
      _cur_lat, _cur_lon = 47.0, -122.0

      def __getattr__(self, n):
        return None
    frag = m._curve_tele(Stub(), 25.0, 320.0, (None, None))
    assert (frag["mapLat"], frag["mapLon"], frag["mapCandD"]) == (None, None, None)

  def test_the_record_never_raises_without_an_accumulator(self):
    """A controller built before this feature (or the permissive stub) must degrade, not explode."""
    rec = _rec(_curve_peak=None)
    assert rec["kPeakN"] == 0 and rec["kPeak"] is None and rec["dq"] is None


# =====================================================================================================
# 3.7 -- the wiring, through the REAL 100 Hz call path
# =====================================================================================================
def _drive(monkeypatch, tmp_path, fp, brand, op_long, yaw_of, cmd_of, pose_of=None, ticks=250,
           str_tq=0.0, blinker=False, steer_pressed=False, saturated=False, lane_change="off",
           pose_valid=True, curve_at=None, curve_v=20.0):
  """Run the REAL CESController.experimental_request at 100 Hz and return every appended record.

  This is the anti-visK test: it proves the value is COMPUTED on the shipped call path, not merely
  that a key exists in a dict a unit test handed to _event_record."""
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(tmp_path / "absent.json"))
  monkeypatch.setattr(pv, "RAIN_CONFIG_PATH", str(tmp_path / "absent-rain.json"))
  clock = [5000.0]
  ns = types.SimpleNamespace(monotonic=lambda: clock[0], time=lambda: clock[0])
  for mod in (C, m):
    monkeypatch.setattr(mod, "time", ns)
  C._ces_mode_hold_st.clear()

  class Mem:
    def __init__(self):
      self.puts = []

    def get(self, k, return_default=False):
      if k == "MapTargetVelocities":
        return _scene(curve_at, 180.0, curve_v) if curve_at is not None else []
      if k == "LastGPSPosition":
        return json.dumps({"latitude": LAT0, "longitude": LON0, "bearing": 0.0, "src": "device",
                           "ts": clock[0], "fix_ts": clock[0] - 0.3})
      if k == "MapSpeedLimit":
        return str(60 * MPH)
      return None

    def put_nonblocking(self, k, v):
      self.puts.append((k, copy.deepcopy(v)))

  params = _P(clock, mode=lambda t: 2, extra={"CESButtonState": "0"})
  c = m.CESController(FakeCP(fp, brand, op_long), params=params)
  c.mem_params = Mem()
  recs = []
  c._event_log_ok = True
  c._append_event = lambda rec: recs.append(copy.deepcopy(rec))
  v_ego, stock = 25.0, 60 * MPH
  for i in range(ticks):
    clock[0] = 5000.0 + (i + 1) * 0.01
    orz, vx, px, ts = _model(v_ego, curve_at if curve_at is not None else 1e9, 180.0)
    model = NS(orientationRate=NS(z=orz, t=ts), velocity=NS(x=vx), position=NS(x=px),
               action=NS(shouldStop=False), meta=NS(laneChangeState=lane_change))
    yaw = yaw_of(i)
    pose_z = pose_of(i) if pose_of is not None else yaw
    sm = {"radarState": NS(leadOne=NS(status=False, vLead=0.0, dRel=0.0, aLeadK=0.0, vLeadK=0.0)),
          "modelV2": model, "carControl": NS(orientationNED=[0.0, 0.0, 0.0]),
          "livePose": NS(angularVelocityDevice=NS(x=0.0, y=0.0, z=pose_z, valid=pose_valid)),
          "controlsState": NS(desiredCurvature=cmd_of(i),
                              lateralControlState=NS(which=lambda: "angleState",
                                                     angleState=NS(saturated=saturated)))}
    cs = NS(vEgo=v_ego, aEgo=0.0, gasPressed=False, brakePressed=False,
            leftBlinker=blinker, rightBlinker=False, vCruise=stock * 3.6, standstill=False,
            steeringAngleDeg=0.0, steeringPressed=steer_pressed, leftBlindspot=False,
            rightBlindspot=False, cruiseState=NS(speed=stock, enabled=True),
            yawRate=yaw, steeringTorque=str_tq)
    c.experimental_request(cs, sm)
  return [r for r in recs if r.get("ev") in ("tick", "adopt")]


class TestOnTheRealCallPath:
  def test_the_accumulator_runs_every_tick_and_the_record_carries_the_peak(self, monkeypatch, tmp_path):
    """One 20 ms spike inside the second. A 1 Hz sample would miss it 98 times out of 100; kPeak
    must not. (yawRate 0.3 rad/s at 25 m/s = 0.012 1/m = 7.5 m/s^2 -- a genuinely tight moment.)"""
    recs = _drive(monkeypatch, tmp_path, LIGHTNING, "ford", True,
                  yaw_of=lambda i: 0.3 if i % 100 == 50 else 0.02, cmd_of=lambda i: 0.0)
    assert recs, "no tick/adopt records at all -- the harness, not the feature, is broken"
    last = recs[-1]
    assert last["kPeakN"] > 50, f"the accumulator is not running at control rate (kPeakN={last['kPeakN']})"
    assert last["kPeak"] == pytest.approx(0.3 / 25.0, rel=0.02), "the spike must survive into the record"

  def test_a_commanded_curvature_the_truck_cannot_follow_still_reaches_kPeak(self, monkeypatch, tmp_path):
    """Defect D2 end to end: the wheel is flat (yaw ~0) while the planner asks for a real curve."""
    recs = _drive(monkeypatch, tmp_path, LIGHTNING, "ford", True,
                  yaw_of=lambda i: 0.0, cmd_of=lambda i: -0.009)
    assert recs[-1]["kPeak"] == pytest.approx(0.009, rel=0.02)

  def test_the_tesla_gets_an_achieved_curvature_where_CAN_gives_it_none(self, monkeypatch, tmp_path):
    """THE point of P1-A / defect D1. carState.yawRate is 0.0 on every Raven tick (the party DBC has
    no yaw signal at all), so slKActl/achLat read as a perfectly straight road forever. kPose/kPoseP
    come from the localizer and must be alive on exactly the same drive."""
    recs = _drive(monkeypatch, tmp_path, TESLA, "tesla", True,
                  yaw_of=lambda i: 0.0,                      # what the Raven's CAN offers: nothing
                  cmd_of=lambda i: 0.0,
                  pose_of=lambda i: 0.25)                    # what the localizer measured: a real bend
    last = recs[-1]
    # NEGATED (2026-09-17): device-frame z runs opposite the vehicle convention, so a +0.25 rad/s
    # reading is a bend to the OTHER side. Only the SIGNED fields move -- kPoseP is a peak of
    # abs(k_pose) (CurvePeak.step), unsigned by design exactly like kPeak, so the fix cannot touch
    # it. That asymmetry is why checker I5 (kPoseP >= |kPose|) still holds either way, and why the
    # sign defect was invisible to every "is the field alive / is it plausible" check.
    assert last["kPose"] == pytest.approx(-0.01, rel=0.02)
    assert last["kPoseP"] == pytest.approx(0.01, rel=0.02)              # magnitude, unsigned
    assert last["achLatPose"] == pytest.approx(-0.25 * 25.0, rel=0.02)   # -yaw * v = -6.25 m/s^2
    assert last["kPeak"] is None, "the CAN-derived pair is still dead -- that is the fact being worked around"

  def test_an_invalid_localizer_nulls_rather_than_claiming_a_straight_road(self, monkeypatch, tmp_path):
    recs = _drive(monkeypatch, tmp_path, TESLA, "tesla", True, yaw_of=lambda i: 0.0,
                  cmd_of=lambda i: 0.0, pose_of=lambda i: 0.25, pose_valid=False)
    assert recs[-1]["kPose"] is None and recs[-1]["kPoseP"] is None

  @pytest.mark.parametrize("kw, want", [
    ({"steer_pressed": True}, "drv"),
    ({"blinker": True}, "blnk"),
    ({"lane_change": "laneChangeStarting"}, "lc"),
    ({"saturated": True}, "sat"),
  ])
  def test_each_disqualifier_is_seen_on_the_real_call_path(self, monkeypatch, tmp_path, kw, want):
    """M5 end to end -- and specifically that `sat` comes from controlsState at 100 Hz, not only
    from the ~1 Hz SteerLimitStatus sample that section 3.3 shows aliases."""
    recs = _drive(monkeypatch, tmp_path, LIGHTNING, "ford", True,
                  yaw_of=lambda i: 0.02, cmd_of=lambda i: 0.0, **kw)
    last = recs[-1]
    assert last["dq"] is True
    assert want in (last["dqWhy"] or "")

  def test_a_clean_drive_disqualifies_nothing(self, monkeypatch, tmp_path):
    """The negative control for the test above: without it, `dq = True` would also pass."""
    recs = _drive(monkeypatch, tmp_path, LIGHTNING, "ford", True,
                  yaw_of=lambda i: 0.02, cmd_of=lambda i: 0.0)
    assert recs[-1]["dq"] is False and recs[-1]["dqWhy"] is None

  def test_driver_torque_reaches_the_record_from_carState(self, monkeypatch, tmp_path):
    recs = _drive(monkeypatch, tmp_path, LIGHTNING, "ford", True, yaw_of=lambda i: 0.02,
                  cmd_of=lambda i: 0.0, str_tq=2.5)
    assert recs[-1]["strTq"] == pytest.approx(2.5)

  def test_the_map_candidate_position_is_logged_on_a_real_map_tick(self, monkeypatch, tmp_path):
    """M4 end to end, and section 3.7's invariant: mapLat/mapLon must sit mapDist away from the truck."""
    # 150 m ahead: inside CES's own map horizon (v_ego * CURVE_MAP_LOOKAHEAD_S = 25 * 10 = 250 m),
    # so upcoming_curve actually produces a candidate and mapDist is populated.
    recs = _drive(monkeypatch, tmp_path, LIGHTNING, "ford", True, yaw_of=lambda i: 0.02,
                  cmd_of=lambda i: 0.0, curve_at=150.0)
    withmap = [r for r in recs if r.get("mapDist")]
    assert withmap, "the harness produced no map candidate -- nothing was actually tested"
    for r in withmap:
      assert r["mapLat"] is not None and r["mapLon"] is not None, "a candidate with no position logged"
      d = m._haversine_m(r["lat"], r["lon"], r["mapLat"], r["mapLon"])
      assert abs(d - r["mapDist"]) <= 30.0, f"mapLat/mapLon is {d:.0f} m out vs mapDist {r['mapDist']:.0f} m"

  def test_a_FAR_candidate_past_CES_own_horizon_is_still_located(self, monkeypatch, tmp_path):
    """Fable I1 end to end -- the defect this fix exists for.

    A 8 m/s curve whose binding point sits ~330 m out. CES's own map window is
    v_ego * CURVE_MAP_LOOKAHEAD_S = 25 * 10 = 250 m, so `mapDist` is 0.0 on every record: CES cannot
    see this curve at all. ICBM's FAR source reaches ICBM_MAP_HORIZON_M (500 m) and does, and it is
    the source ICBM acts on (icbmSrc == "far"). Keying the coordinates off mapDist therefore logged
    mapLat/mapLon as NULL on exactly the far-map records section 3.4 was added to locate -- and a
    null reads as "there was no candidate", not as "the wrong distance was asked for".

    op_long=False because ICBM is the stock-ACC path: with openpilot longitudinal ON there is no
    ICBM episode at all and icbmSrc is None on every record (verified -- the first draft of this
    test passed `True` and measured nothing)."""
    recs = _drive(monkeypatch, tmp_path, LIGHTNING, "ford", False, yaw_of=lambda i: 0.02,
                  cmd_of=lambda i: 0.0, curve_at=300.0, curve_v=8.0)
    far = [r for r in recs if r.get("icbmSrc") == "far"]
    assert far, "the harness produced no far-source record -- nothing was actually tested"
    assert all(not r.get("mapDist") for r in far), \
      "CES saw this curve after all, so the far-only case is not being exercised"
    for r in far:
      assert r["mapLat"] is not None and r["mapLon"] is not None, \
        "a far candidate with no position logged -- this is the Fable I1 defect"
      assert r["mapCandD"] > 250.0, f"mapCandD {r['mapCandD']} is inside CES's own horizon"
      d = m._haversine_m(r["lat"], r["lon"], r["mapLat"], r["mapLon"])
      assert abs(d - r["mapCandD"]) <= 30.0, f"coordinates {d:.0f} m out vs mapCandD {r['mapCandD']:.0f} m"
      # THE check, and the one the first version of this test did not make (Fable, round 2). The two
      # assertions above are both measured against mapCandD, so they pass together even when the
      # logged node is the WRONG one -- which is exactly what happened: ICBM measures src_dist from
      # its PROJECTED position (here 32 m behind the fix, gpsAge 0.3), so re-matching 332 m against
      # the raw fix returned the node at 339 m instead of the arc start `_scene` puts at 300 m.
      # Pin the ABSOLUTE position of the candidate, which no origin mix-up can satisfy by accident.
      assert m._haversine_m(LAT0, LON0, r["mapLat"], r["mapLon"]) == pytest.approx(300.0, abs=2.0), \
        "the logged node is not the one ICBM bound to -- the distance was matched from the wrong origin"
      # mapCandD is BY CONSTRUCTION the rounded fix->node haversine, so pin it to 1 m, not 30. At
      # 30 m the check passes on ICBM's own (projected-origin) distance too -- it cleared by 2.3 m
      # here, which is luck, not a pin (Fable X1).
      assert abs(r["mapCandD"] - d) <= 1.0, f"mapCandD {r['mapCandD']} is not the logged node's distance {d:.1f}"

  def test_a_source_with_no_map_point_logs_no_candidate_distance(self, monkeypatch, tmp_path):
    """The negative control: _icbm_cand_d must be None for every non-map source, or mapLat/mapLon
    would be resolved at a vision/restore distance and land on a map point by coincidence.

    ⚠️ This drive only ever produces `far` and `None` records -- the harness truck's GPS does not
    move, so no episode can clear and no vis/gpsHold/restore record exists. It is therefore NOT
    sufficient on its own (Fable, round 2: mutating the latch condition to `is not None` survived
    it). The two tests below cover the sources it cannot reach."""
    recs = _drive(monkeypatch, tmp_path, LIGHTNING, "ford", False, yaw_of=lambda i: 0.02,
                  cmd_of=lambda i: 0.0, curve_at=300.0, curve_v=8.0)
    assert any(r.get("icbmSrc") not in ("map", "far") for r in recs), "no non-map record to check"
    for r in recs:
      if r.get("icbmSrc") not in ("map", "far"):
        assert r.get("mapCandD") in (None, r.get("mapDist") or None), \
          f"icbmSrc={r.get('icbmSrc')} carried a map candidate distance {r.get('mapCandD')}"

  def test_a_VISION_tick_carries_no_candidate(self, monkeypatch, tmp_path):
    """Pins the latch CONDITION. `src_dist` is a real number for a vision source too (it is
    vis_dist), so latching on "is the source set" rather than "is the source a map point" attaches a
    polyline node to a record whose candidate never came from the polyline. The drive harness cannot
    reach this: its truck never moves, so no episode clears and every record is far/None."""
    mgr, step = _icbm_stub_at(monkeypatch, tmp_path, targets=[])       # map blind; only vision
    _run(mgr, step, _sig(29.0, 29.0, vis_lat=3.474, ttc=4.0))
    assert mgr._icbm_src == "vis", f"expected a vision-sourced tick, got {mgr._icbm_src}"
    assert mgr._icbm_cand_d is None and mgr._icbm_cand_pt == (None, None)

  def test_a_RESTORE_tick_carries_no_candidate_even_though_the_map_one_was_binding(self, monkeypatch, tmp_path):
    """Pins the latch POSITION. During a restore the source label is "restore" and nothing is being
    approached, but `src_dist` still holds the map distance from the decision earlier in the SAME
    tick -- so latching before the relabel attaches a real node to a record that is giving speed
    back. Also unreachable from the drive harness."""
    mgr, step = _icbm_stub_at(monkeypatch, tmp_path)
    _run(mgr, step, _sig(25.0, 60 * MPH))
    assert mgr._icbm_src in ("map", "far") and mgr._icbm_cand_d is not None, \
      f"no map candidate bound (src={mgr._icbm_src}) -- this test would prove nothing"
    monkeypatch.setattr(mgr._icbm_ep, "step", lambda *a, **k: (20.0, "inc"))
    _run(mgr, step, _sig(25.0, 60 * MPH))
    assert mgr._icbm_src == "restore"
    assert mgr._icbm_cand_d is None and mgr._icbm_cand_pt == (None, None)

  def test_the_latched_point_is_the_RE_DECIDED_candidate_after_a_passed_point_is_dropped(self, monkeypatch, tmp_path):
    """Fable X8. behindgate/behindrun remove a map point the truck has already driven past and
    RE-DECIDE on what is left, so `far_dist` at the latch is not the one the first scan produced.
    Latching the pre-gate number would put the record's coordinates on the curve ICBM explicitly
    refused to act on -- and that curve is BEHIND the truck, which is the most misleading place for a
    'where is the curve' field to point.

    The road: a tight 5 m/s curve 60-100 m BEHIND (far more binding) and an 8 m/s one ~300 m ahead."""
    mgr, step = _icbm_stub_at(monkeypatch, tmp_path, targets=[])
    mgr._cur_bearing = 0.0                                    # heading north; the gate needs a heading
    mgr._map_targets = [_pt_north(LAT0, LON0, -100.0, 5.0), _pt_north(LAT0, LON0, -60.0, 5.0),
                        _pt_north(LAT0, LON0, 300.0, 8.0), _pt_north(LAT0, LON0, 340.0, 8.0)]
    _run(mgr, step, _sig(25.0, 60 * MPH))
    assert mgr._icbm_passed_state and mgr._icbm_passed_state[0] == "passed", \
      f"the behind point was not dropped ({mgr._icbm_passed_state}) -- this test proves nothing"
    assert mgr._icbm_src == "far" and mgr._icbm_cand_pt[0] is not None
    assert mgr._icbm_cand_pt[0] > LAT0, "the latched point is BEHIND the truck -- the pre-gate candidate"
    assert m._haversine_m(LAT0, LON0, *mgr._icbm_cand_pt) == pytest.approx(300.0, abs=2.0)

  def test_a_tick_that_BLEW_UP_does_not_leave_the_previous_candidate_behind(self, monkeypatch, tmp_path):
    """Why the latch is also CLEARED at the top of the tick. _icbm_step swallows (loudly) into its
    own except, so a raise between the clear and the latch would otherwise pair THIS tick's icbmSrc
    with the PREVIOUS publish's coordinates -- a confidently wrong position, which is worse than the
    null this whole feature replaced."""
    mgr, step = _icbm_stub_at(monkeypatch, tmp_path)
    _run(mgr, step, _sig(25.0, 60 * MPH))
    assert mgr._icbm_cand_d is not None, "no candidate latched -- this test would prove nothing"
    said = []
    monkeypatch.setattr(m.cloudlog, "exception", lambda msg, *a, **k: said.append(msg))

    def boom(*a, **k):
      raise RuntimeError("far scanner down")
    monkeypatch.setattr(m, "icbm_far_map_candidate", boom)
    _run(mgr, step, _sig(25.0, 60 * MPH))
    assert said, "Rule 2: the failure must be loud"
    assert mgr._icbm_cand_d is None and mgr._icbm_cand_pt == (None, None), \
      "the previous tick's candidate survived a failed tick"

  def test_the_vision_curvature_is_logged_on_ticks_ICBM_DID_NOT_ACT_ON(self, monkeypatch, tmp_path):
    """viskvis2pnw, and the whole reason the field was added.

    `icbmKVis` carries the same quantity, but it is written by `_curvelead_note`, which runs only
    inside `_icbm_step`'s lead-pacing block -- i.e. only where ICBM ALREADY HAS A TARGET. So the one
    reading that could say whether vision agreed with a map slowdown was recorded only where ICBM had
    already decided to slow. Measured 2026-09-19 across every corpus: a correctly-computed vision
    curvature coexists with an ICBM decision on ONE drive, 56 ticks.

    This drives a road with NO map candidate at all, so ICBM never acts -- and asserts the reading is
    there anyway. A field present only when the feature fires cannot answer whether the feature
    should have fired."""
    recs = _drive(monkeypatch, tmp_path, LIGHTNING, "ford", False,
                  yaw_of=lambda i: 0.02, cmd_of=lambda i: 0.0, curve_at=None)
    quiet = [r for r in recs if r.get("icbmSrc") is None]
    assert quiet, "every record had an ICBM source -- this test proves nothing"
    assert all("visKMax" in r for r in quiet), "the key is missing on ticks ICBM did not act on"
    live = [r for r in quiet if r.get("visKMax") is not None]
    assert live, "visKMax is null on every ICBM-idle tick -- logged but never computed (the visK failure)"
    assert all(r.get("visKRch") is not None for r in live), "a curvature with no horizon reach cannot be judged"

  def test_the_vision_curvature_TRACKS_THE_MODEL_and_is_not_a_constant(self, monkeypatch, tmp_path):
    """Fable's optional, taken: the test above only ever sees exact-0.0 readings, because its fixture
    road is straight. A field hard-wired to 0.0 would pass it. Drive a real curve and require the
    reading to move -- otherwise "the value is present" says nothing about whether it is the model's."""
    recs = _drive(monkeypatch, tmp_path, LIGHTNING, "ford", False, yaw_of=lambda i: 0.02,
                  cmd_of=lambda i: 0.0, curve_at=150.0)
    live = [r for r in recs if r.get("visKMax") is not None]
    assert live, "no vision reading at all -- the harness, not the field, is broken"
    assert any(r["visKMax"] > 0 for r in live), \
      "visKMax is 0.0 on every tick of a curved road -- it is a constant, not the model's reading"
    for r in live:
      assert r.get("visKRch") and r["visKRch"] > 0, \
        "a curvature with a 0.0 horizon reach is a hiccup's signature, not a reading"

  def test_the_vision_curvature_is_null_where_nothing_computes_it(self, monkeypatch, tmp_path):
    """The negative control. `vis_k_max` is computed only on the ICBM shadow path (`veh.ces_shadow`),
    so with openpilot longitudinal ON it is never produced -- and must read NULL, not 0.0. Without
    this, a field that is always 0.0 would pass the test above and look alive on every car."""
    recs = _drive(monkeypatch, tmp_path, LIGHTNING, "ford", True,
                  yaw_of=lambda i: 0.02, cmd_of=lambda i: 0.0, curve_at=None)
    assert recs, "no records"
    assert all(r.get("visKMax") is None for r in recs), \
      "visKMax is populated where nothing computes it -- that is a fabricated reading"

  def test_a_broken_accumulator_is_logged_and_never_reaches_the_control_loop(self, monkeypatch, tmp_path):
    """Rule 2. A silently dead accumulator is the visK failure; kPeakN going to 0 in the record and
    the swaglog line must agree that it is dead."""
    said = []
    monkeypatch.setattr(m.cloudlog, "exception", lambda msg, *a, **k: said.append(msg))

    def boom(*a, **k):
      raise RuntimeError("accumulator down")
    monkeypatch.setattr(m, "_pose_curvature", boom)
    recs = _drive(monkeypatch, tmp_path, LIGHTNING, "ford", True,
                  yaw_of=lambda i: 0.3, cmd_of=lambda i: 0.0, ticks=150)
    assert any("_curve_peak_step FAILED" in s and "RuntimeError" in s for s in said)
    assert recs and recs[-1]["kPeakN"] == 0, "the record must agree with the log that nothing was measured"
    assert recs[-1]["kPeak"] is None


# =====================================================================================================
# 3.7 -- the CHECKER itself. A verification gate that cannot fail verifies nothing, which is the same
# class of defect (something that looks like evidence and is not) as visK.
# =====================================================================================================
def _good_row(i, **over):
  """One synthetic post-curvedbtel2pnw Lightning tick that satisfies every invariant."""
  cand = _pt(200.0)
  d = round(m._haversine_m(LAT0, LON0, cand["latitude"], cand["longitude"]), 0)
  r = {"t": 1788000000.0 + i, "ev": "tick", "car": LIGHTNING, "vEgo": 25.0,
       "lat": LAT0, "lon": LON0, "mapDist": d,
       # mapCandD is what I3 measures the coordinates against; here the source is "map", so it and
       # mapDist agree. The far-candidate row below is the case where they do NOT.
       "mapCandD": d, "icbmSrc": "map",
       "mapLat": cand["latitude"], "mapLon": cand["longitude"],
       "kPeak": 0.0052, "kPeakN": 100, "kPoseP": 0.0051, "kPose": 0.005,
       "achLatPose": 3.125, "achLat": 3.0, "slKActl": 0.0048, "strTq": 0.5,
       "dq": False, "dqWhy": None, "strPrs": False, "blnk": False,
       "slAngSat": False, "slSat": False, "slCurvLim": False, "lcGate": "ok"}
  r.update(over)
  return r


def _corpus(tmp_path, rows, name="ces_events.jsonl"):
  p = tmp_path / name
  p.write_text("\n".join(json.dumps(r) for r in rows) + "\n")
  return str(p)


def _run_check(path, *extra):
  from openpilot.tools import curvedb_telemetry_check as chk
  return chk.main([path, "--quiet", *extra])


class TestTheCheckerItself:
  def test_a_healthy_corpus_passes(self, tmp_path):
    assert _run_check(_corpus(tmp_path, [_good_row(i) for i in range(60)])) == 0

  def test_a_pre_feature_corpus_fails_rather_than_passing_vacuously(self, tmp_path):
    """ABSENT is not "nothing to check": a pre-feature corpus and a dead writer look identical."""
    rows = [{k: v for k, v in _good_row(i).items() if k not in m.CURVE_TELE_KEYS} for i in range(60)]
    path = _corpus(tmp_path, rows)
    assert _run_check(path) == 1
    assert _run_check(path, "--allow-absent") == 0     # the documented escape hatch, and only that

  def test_mapLat_taken_from_the_truck_is_caught(self, tmp_path):
    """M4. The prototype that keyed on truck positions smeared one episode across 4-5 "sites"."""
    rows = [_good_row(i, mapLat=LAT0, mapLon=LON0) for i in range(60)]
    assert _run_check(_corpus(tmp_path, rows)) == 1

  def test_a_truck_driving_OVER_a_map_node_is_not_a_defect(self, tmp_path):
    """I3b's false positive, found on the first real corpus (2026-09-17).

    The truck passes directly over map nodes constantly. One record in 46 minutes had mapDist 1.0 m
    with the node resolved 0.71 m away -- correct, and I3b failed it for being close. As originally
    written this check would have gone red on essentially every drive, and an acceptance gate that
    always fails is one a human learns to ignore -- a worse failure than not having the gate.

    The arbiter is `mapDist`, CES's own INDEPENDENT distance: on the truck while mapDist says metres
    away is the defect; on the truck while mapDist agrees is a node underfoot."""
    near = _pt(0.8)                                    # a node the truck is almost exactly on
    rows = [_good_row(i, mapDist=1.0, mapCandD=1.0,
                      mapLat=near["latitude"], mapLon=near["longitude"]) for i in range(60)]
    assert _run_check(_corpus(tmp_path, rows)) == 0, "a node underfoot must not fail the gate"

  def test_but_coordinates_copied_from_the_truck_are_STILL_caught(self, tmp_path):
    """The negative control for the relaxation above -- otherwise I3b would pass anything close.
    This is the M4 defect proper: the coordinates ARE the truck's while mapDist says 200 m."""
    rows = [_good_row(i, mapLat=LAT0, mapLon=LON0, mapCandD=0.0) for i in range(60)]
    assert _run_check(_corpus(tmp_path, rows)) == 1

  def test_a_far_candidate_beyond_CES_own_horizon_PASSES(self, tmp_path):
    """Fable I1, the checker half. A far candidate at 400 m is past CES's 10 s window, so mapDist is
    0.0 while the coordinates are genuinely 400 m out. Measured against mapDist (the old I3) this
    correct record was either skipped or failed; measured against mapCandD it passes."""
    cand = _pt(400.0)
    d = round(m._haversine_m(LAT0, LON0, cand["latitude"], cand["longitude"]), 0)
    # mapDist 150, NOT 0: the record names a DIFFERENT, nearer curve that CES can see, which is the
    # real shape of this case. With mapDist 0 the old I3's `and r.get("mapDist")` filter drops the
    # row and the check SKIPs, so this test passed against the old checker too and proved nothing
    # (Fable, round 2).
    rows = [_good_row(i, icbmSrc="far", mapDist=150.0, mapCandD=d,
                      mapLat=cand["latitude"], mapLon=cand["longitude"]) for i in range(60)]
    assert _run_check(_corpus(tmp_path, rows)) == 0

  def test_coordinates_resolved_at_the_WRONG_candidate_are_still_caught(self, tmp_path):
    """The negative control for the test above -- otherwise "check against mapCandD" would pass
    anything. Coordinates 400 m out while mapCandD says 200 m is the defect I3 exists to find."""
    cand = _pt(400.0)
    rows = [_good_row(i, icbmSrc="far", mapCandD=200.0,
                      mapLat=cand["latitude"], mapLon=cand["longitude"]) for i in range(60)]
    assert _run_check(_corpus(tmp_path, rows)) == 1

  def test_a_far_only_drive_MEASURES_the_candidate_position_instead_of_excusing_it(self, tmp_path, capsys):
    """Fable X10, and the assertion is on the REPORT, not the exit code -- which is the whole point.

    mapLat/mapLon are null by design on most records, so they are only judged within the population
    where a value is due. That population used to be keyed on `mapDist`, which is 0.0 on EVERY far
    record: a far-only drive printed "no records with a map candidate on this drive -- nothing to
    expect", and a human reading the acceptance output would conclude the field was fine when it had
    never been exercised. The exit code cannot see the difference (a healthy corpus passes either
    way); a reader can, and this is the instrument they read."""
    cand = _pt(400.0)
    d = round(m._haversine_m(LAT0, LON0, cand["latitude"], cand["longitude"]), 0)
    rows = [_good_row(i, icbmSrc="far", mapDist=0.0, mapCandD=d,
                      mapLat=cand["latitude"], mapLon=cand["longitude"]) for i in range(60)]
    from openpilot.tools import curvedb_telemetry_check as chk
    assert chk.main([_corpus(tmp_path, rows)]) == 0
    # "records carry it" disambiguates the PRESENCE row from the WRITER PATHS row, which also names
    # mapLat -- matching on the field name alone silently picked the wrong line.
    line = next(ln for ln in capsys.readouterr().out.splitlines()
                if " mapLat " in ln and "records carry it" in ln)
    assert "nothing to expect" not in line, f"the far population was excused, not measured: {line}"
    assert "60/60 non-null" in line, line

  def test_coordinates_with_no_distance_to_check_them_against_are_caught(self, tmp_path):
    """Rule 2 (I3c): dropping the un-checkable records would shrink n silently and still print PASS.
    This is exactly how the old I3 hid every far-candidate record -- `and r.get("mapDist")`.

    ONE orphan among 59 good rows, deliberately: nulling all 60 makes the field ALWAYS-NULL, the
    presence check fails first, and I3c is never the deciding check (Fable, round 2).

    The orphan also carries mapDist 0.0 (round 4). With a non-zero mapDist it was the row that killed
    the "I3 filters on mapDist again" mutant -- but by CRASHING the checker (`float - NoneType`),
    not by detecting anything. A crash is not a detection; the mutant is pinned properly by
    test_a_far_only_drive_is_actually_CHECKED_not_skipped below."""
    rows = [_good_row(i) for i in range(59)] + [_good_row(59, mapCandD=None, mapDist=0.0)]
    assert _run_check(_corpus(tmp_path, rows)) == 1

  def test_a_far_only_drive_is_actually_CHECKED_not_skipped(self, tmp_path):
    """I3's own row must be a PASS over all 60 records, not a SKIP.

    This is the failure the exit code cannot see (Fable round 4): filtering `geo` on mapDist -- 0.0 on
    every far record -- empties it, I3 reports SKIP, and the run still exits 0. A verification gate
    that silently verified nothing is the same defect class as the field it is checking for."""
    cand = _pt(400.0)
    d = round(m._haversine_m(LAT0, LON0, cand["latitude"], cand["longitude"]), 0)
    rows = [_good_row(i, icbmSrc="far", mapDist=0.0, mapCandD=d,
                      mapLat=cand["latitude"], mapLon=cand["longitude"]) for i in range(60)]
    from openpilot.tools import curvedb_telemetry_check as chk
    assert chk.main([_corpus(tmp_path, rows), "--quiet"]) == 0
    i3 = [r for r in chk.main.last_report.rows if r[1].startswith("I3 ")]
    assert len(i3) == 1, i3
    assert i3[0][0] == "PASS", f"I3 did not check the far records: {i3[0]}"
    assert "n=60" in i3[0][2], f"I3 checked only some of them: {i3[0]}"

  def test_a_peak_that_under_reads_the_instantaneous_sample_is_caught(self, tmp_path):
    """M1. If kPeak were the last sample rather than the window max it would routinely sit below the
    achLat measured at the record instant; I1 is what notices."""
    rows = [_good_row(i, kPeak=0.0001) for i in range(60)]
    assert _run_check(_corpus(tmp_path, rows)) == 1

  def test_a_disqualifier_the_roll_up_missed_is_caught(self, tmp_path):
    """M5. A sampled flag true while dq is false means the OR is not running."""
    rows = [_good_row(i, strPrs=True) for i in range(60)]
    assert _run_check(_corpus(tmp_path, rows)) == 1

  def test_an_exact_zero_is_caught(self, tmp_path):
    """M3, and this is the check that FAILS on today's real Tesla corpus -- by design."""
    rows = [_good_row(i, kPose=0.0) for i in range(60)]
    assert _run_check(_corpus(tmp_path, rows)) == 1

  def test_a_sign_flipped_pose_source_is_caught(self, tmp_path):
    """Non-zero and WRONG -- the units2pnw / capnp-enum class of defect that "is it non-zero" misses."""
    rows = [_good_row(i, kPose=-0.005) for i in range(60)]
    assert _run_check(_corpus(tmp_path, rows)) == 1

  def test_a_ford_only_field_leaking_onto_a_tesla_is_caught(self, tmp_path):
    rows = [_good_row(i, car=TESLA, slKActl=None, achLat=None,
                      car_gps={"lat": LAT0, "lon": LON0}) for i in range(60)]
    assert _run_check(_corpus(tmp_path, rows)) == 1

  def test_a_tesla_whose_pose_curvature_is_dead_is_caught(self, tmp_path):
    """The positive half of the negative control: if P1-A did not actually fix D1, say so."""
    rows = [_good_row(i, car=TESLA, slKActl=None, achLat=None, kPose=None, kPoseP=None,
                      achLatPose=None, kPeak=None) for i in range(60)]
    assert _run_check(_corpus(tmp_path, rows)) == 1

  def test_the_writer_check_names_the_real_emit_sites(self, tmp_path):
    """Section 3.7 item 1: every field in the checker's table must be traceable to live source."""
    from openpilot.tools import curvedb_telemetry_check as chk
    assert set(chk.WRITERS) == set(m.CURVE_TELE_KEYS), \
      "the checker's writer table and CURVE_TELE_KEYS must describe the same field set"
