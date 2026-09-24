"""curvefix2pnw -- ICBM curve-target fixes for the F-150 Lightning (2026-09-24, OR-34 Corvallis -> I-5).

Part A: the VISION turn direction was inverted. `icbm_penalise` read `curve_lat_accel_vision > 0` as LEFT, but that
value is modelV2.orientationRate.z * v and orientationRate.z is RIGHT-positive in this tree. Every right-hand
vision curve got the Lightning left factor (1.15x the hump) and every left-hand one lost it. It hid because the
direction was never logged. MEASURED, not assumed: drives/2026-09-24/vision-left-flag-check.md (147 / 147 in-curve
ticks, three independent witnesses) and the real modelV2 frames in vtsc_pnw/tests/measured_turn_frames.py.
"""
import math

import pytest

from openpilot.selfdrive.controls.lib import pnw_vehicle as pv
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
from openpilot.selfdrive.controls.lib.pnw_vehicle import PnwVehicle
from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_icbm_bridge import (
  FakeCPA, LIGHTNING, _icbm_stub, _path, _published_target,
)
from openpilot.selfdrive.controls.lib.vtsc_pnw.tests import measured_turn_frames as mtf

MPH = 0.44704
LAT, LON = 47.0, -122.0


@pytest.fixture(autouse=True)
def shipped_cfg(tmp_path, monkeypatch):
  """No /data/pnw/curve.json -> the shipped source defaults."""
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(tmp_path / "nope.json"))


def _veh(fp=LIGHTNING, brand="ford", **cfg):
  v = PnwVehicle(FakeCPA(fp, brand))
  if cfg:
    v._curve_cfg = dict(v._curve_cfg, **cfg)
  return v


def _frame_vis(frame):
  """(visLat, ttc) exactly as the car computes it from this modelV2 frame."""
  return m.vision_curve_lat_accel(list(frame["z"]), list(frame["vx"]), list(mtf.T), frame["v_ego"])


def _vis_sig(v_ego, lat, ttc=4.0, pitch=None):
  return {"v_ego": v_ego, "v_set": 40.0, "map_target_v": 0.0, "map_target_dist": float("inf"),
          "curve_lat_accel_vision": lat, "time_to_curve": ttc, "pitch": pitch}


# --------------------------------------------------------------------------------------------------------------------
# Part A: the vision direction, on the road's own frames
# --------------------------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("frame,left", [(mtf.LEFT_HANDER, True), (mtf.RIGHT_HANDER, False)])
def test_vision_direction_matches_the_measured_road(frame, left):
  """The frame's three witnesses (steering angle, yaw rate, GPS bearing) say which way the road turned; the
  direction ICBM uses for the left factor must say the same."""
  assert mtf.witnesses_say_left(frame) is left
  lat, _ = _frame_vis(frame)
  assert abs(lat) > m.ICBM_VISION_ENTER, "the frame must be a real vision candidate or it proves nothing"
  apex = frame["v_ego"] * math.sqrt(m.VTSC_A_LAT / abs(lat))
  _, _, _, is_left, src = m.icbm_penalise(_veh(), [], apex, "vis", _vis_sig(frame["v_ego"], lat), LAT, LON,
                                          float("inf"), 0.0)
  assert is_left is left
  assert src == "vis"


def test_the_or34_right_hander_gets_no_left_factor():
  """The 11:19 over-slow curve: vision's target now carries at most the plain hump, never hump x left_factor."""
  fr = mtf.RIGHT_HANDER
  lat, _ = _frame_vis(fr)
  assert lat > 0.0                                   # the road's sign: + = RIGHT
  veh = _veh()
  apex = fr["v_ego"] * math.sqrt(m.VTSC_A_LAT / lat)
  t, *_ = m.icbm_penalise(veh, [], apex, "vis", _vis_sig(fr["v_ego"], lat), LAT, LON, float("inf"), 0.0)
  with_left = apex - veh.curve_speed_penalty_ms(apex, is_left=True)
  assert t > with_left + 0.1, "the right-hander still pays the left factor"


def test_the_measured_left_hander_gets_the_left_factor_through_icbm_step():
  """End to end through the real _icbm_step: the left frame's vision candidate is published LOWER than the same
  magnitude on the right, and the controller records the direction it used."""
  veh = _veh()
  lat, _ = _frame_vis(mtf.LEFT_HANDER)
  mgr, step = _icbm_stub(veh)
  t_left = _published_target(step, mgr, _vis_sig(33.0, lat))
  assert (mgr._icbm_left, mgr._icbm_left_src) == (True, "vis")
  t_right = _published_target(step, mgr, _vis_sig(33.0, -lat))
  assert (mgr._icbm_left, mgr._icbm_left_src) == (False, "vis")
  assert t_left is not None and t_right is not None and t_left < t_right


def test_map_direction_is_unchanged():
  """The map/far path was verified correct (Terwilliger's logged floor extras, 25 / 25 ticks): a left-bending
  path is still LEFT, a right-bending one still right."""
  veh = _veh()
  for bend, left in ((+4.0, True), (-4.0, False)):
    sig = {"v_ego": 26.0, "v_set": 26.0, "map_target_v": 20.0, "map_target_dist": 205.0,
           "curve_lat_accel_vision": 0.0, "time_to_curve": 10.0, "pitch": None}
    _, _, _, is_left, src = m.icbm_penalise(veh, _path(LAT, LON, bend), 20.0, "map", sig, LAT, LON,
                                            float("inf"), 0.0)
    assert (is_left, src) == (left, "map")
  # no geometry to call -> "unknown", no left factor
  _, _, _, is_left, src = m.icbm_penalise(veh, [], 20.0, "map", sig, LAT, LON, float("inf"), 0.0)
  assert (is_left, src) == (False, "unknown")


def test_a_direction_failure_is_logged_rate_limited_and_neutral(monkeypatch):
  """Rule 2: the old `except Exception: is_left = False` was silent, and on a right curve a failure looks exactly
  like a correct "right". Now it logs (throttled), reports "err", and still falls back to no left factor."""
  calls = []

  def boom(*a, **k):
    raise TypeError("map_target_dist arrived as None")
  monkeypatch.setattr(m, "map_turn_direction", boom)
  monkeypatch.setattr(m.cloudlog, "exception", lambda msg, *a, **k: calls.append(msg))
  monkeypatch.setitem(m._ICBM_DIR_ERR, "t", -1e9)
  monkeypatch.setitem(m._ICBM_DIR_ERR, "n", 0)
  veh = _veh()
  sig = {"v_ego": 26.0, "v_set": 26.0, "map_target_v": 20.0, "map_target_dist": 205.0,
         "curve_lat_accel_vision": 0.0, "time_to_curve": 10.0, "pitch": None}
  for _ in range(5):
    t, _, _, is_left, src = m.icbm_penalise(veh, [], 20.0, "map", sig, LAT, LON, float("inf"), 0.0)
    assert (is_left, src) == (False, "err")
    assert t == pytest.approx(max(20.0 - veh.curve_speed_penalty_ms(20.0), veh.icbm_map_floor_ms(20.0)))
  assert len(calls) == 1, f"expected one throttled log line, got {len(calls)}"
  assert "left-curve factor is NOT applied" in calls[0]
  assert m._ICBM_DIR_ERR["n"] == 4                   # the four suppressed failures are counted for the next line


def test_non_lightning_direction_changes_nothing():
  """Tesla (and every other car): the penalty is 0.0, so the direction cannot move the target."""
  veh = _veh("TESLA_MODEL_S_HW3", "tesla")
  for lat in (+2.8, -2.8):
    t, *_ = m.icbm_penalise(veh, [], 30.0, "vis", _vis_sig(33.0, lat), LAT, LON, float("inf"), 0.0)
    assert t == 30.0


def test_direction_telemetry_reaches_the_record():
  from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_ces_record_fields import _record
  rec = _record(_icbm_left=True, _icbm_left_src="vis")
  assert rec["icbmLeft"] is True and rec["icbmLeftSrc"] == "vis"
  rec = _record(_icbm_left=False, _icbm_left_src="err")
  assert rec["icbmLeft"] is False and rec["icbmLeftSrc"] == "err"
  rec = _record(_icbm_left=None, _icbm_left_src=None)
  assert rec["icbmLeft"] is None and rec["icbmLeftSrc"] is None


def test_direction_clears_on_a_tick_without_a_target():
  """Never publish the previous tick's direction beside icbmT=None (the floor fields' staleness rule)."""
  veh = _veh()
  mgr, step = _icbm_stub(veh)
  # a sharper right-hander than the fixture (+3.5), 3 s out: inside the brake envelope, past the 2.75 s start gate
  assert _published_target(step, mgr, _vis_sig(33.0, 3.5, ttc=3.0)) is not None
  assert mgr._icbm_left_src == "vis"
  assert _published_target(step, mgr, _vis_sig(33.0, 0.0)) is None
  assert (mgr._icbm_left, mgr._icbm_left_src) == (None, None)
