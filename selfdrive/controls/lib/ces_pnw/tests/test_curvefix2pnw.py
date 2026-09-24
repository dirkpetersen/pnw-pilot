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
  """The 11:19 over-slow curve: vision's target now carries at most the plain hump, never hump x left_factor.
  (left_factor is neutral by default since 2026-09-24; set here so the direction still has something to gate.)"""
  fr = mtf.RIGHT_HANDER
  lat, _ = _frame_vis(fr)
  assert lat > 0.0                                   # the road's sign: + = RIGHT
  veh = _veh(left_factor=1.15)
  apex = fr["v_ego"] * math.sqrt(m.VTSC_A_LAT / lat)
  t, *_ = m.icbm_penalise(veh, [], apex, "vis", _vis_sig(fr["v_ego"], lat), LAT, LON, float("inf"), 0.0)
  with_left = apex - veh.curve_speed_penalty_ms(apex, is_left=True)
  assert t > with_left + 0.1, "the right-hander still pays the left factor"


def test_the_measured_left_hander_gets_the_left_factor_through_icbm_step():
  """End to end through the real _icbm_step, and the controller records the direction it used. At the shipped
  defaults (left_factor 1.0 since 2026-09-24) left and right publish the SAME target; with a curve.json left_factor
  the left frame's vision candidate is published LOWER than the same magnitude on the right."""
  lat, _ = _frame_vis(mtf.LEFT_HANDER)
  for veh, left_lower in ((_veh(), False), (_veh(left_factor=1.15), True)):
    mgr, step = _icbm_stub(veh)
    t_left = _published_target(step, mgr, _vis_sig(33.0, lat))
    assert (mgr._icbm_left, mgr._icbm_left_src) == (True, "vis")
    t_right = _published_target(step, mgr, _vis_sig(33.0, -lat))
    assert (mgr._icbm_left, mgr._icbm_left_src) == (False, "vis")
    assert t_left is not None and t_right is not None
    if left_lower:
      assert t_left < t_right
    else:
      assert t_left == t_right


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


# --------------------------------------------------------------------------------------------------------------------
# Part B: a vision candidate may not be penalised below vision's own 2.5 m/s^2 speed for the curve
# --------------------------------------------------------------------------------------------------------------------
# The three icbmSrc=vis ticks of OR-34 2026-09-24 (ces_events, PT): (time, vEgo m/s, visLat m/s^2, logged icbmT m/s).
OR34_VIS_TICKS = [("11:19:12.2", 32.7, 2.79, 29.63), ("11:19:13.2", 32.3, 2.72, 29.52), ("11:19:14.2", 31.6, 2.59, 29.64)]
OR34_NEED_MPH = 69.9      # the curve's measured k 0.00256 at 2.5 m/s^2


@pytest.mark.parametrize("pt,v,lat,logged", OR34_VIS_TICKS)
def test_or34_vision_target_is_vision_own_speed(pt, v, lat, logged):
  """The complaint: set 80 -> 66 on a curve that needed 69.9. Vision's own 2.5 m/s^2 speed was ~69.2 mph; the
  hump (x the wrong left factor) took it to ~66. Now: exactly vision's own speed (a right-hander, pitch uphill ->
  no left, no descent extra)."""
  apex = v * math.sqrt(m.VTSC_A_LAT / lat)
  sig = _vis_sig(v, lat, pitch=0.015)                 # measured pitch +0.010 .. +0.021 (uphill)
  new, flr, hit, is_left, _ = m.icbm_penalise(_veh(), [], apex, "vis", sig, LAT, LON, float("inf"), 0.0)
  assert is_left is False
  assert new == pytest.approx(apex, abs=1e-6)
  assert flr == pytest.approx(apex) and hit is True
  assert 69.0 < new / MPH < OR34_NEED_MPH + 0.5
  # the pre-curvefix model (the then-default 1.15 left factor on this right-hander, no floor) reproduces the logged
  # ~66 mph. left_factor is pinned to the value the truck ran, since the default is 1.0 since 2026-09-24.
  veh0 = _veh(left_factor=1.15)
  old = apex - veh0.curve_speed_penalty_ms(apex, pitch_rad=0.015, is_left=True)
  assert old / MPH == pytest.approx(logged / MPH, abs=0.6), f"{pt}: model {old / MPH:.2f} vs logged {logged / MPH:.2f}"


def test_vis_floor_frac_zero_is_the_off_switch():
  for lat in (2.2, 2.6, 3.0, 3.5, -2.6, -3.5):
    for pitch in (None, 0.0, -0.05):
      v = 30.0
      apex = v * math.sqrt(m.VTSC_A_LAT / abs(lat))
      veh = _veh(icbm_vis_floor_frac=0.0)
      t, flr, hit, is_left, _ = m.icbm_penalise(veh, [], apex, "vis", _vis_sig(v, lat, pitch=pitch), LAT, LON,
                                                float("inf"), 0.0)
      assert flr == 0.0 and hit is False
      assert t == pytest.approx(max(apex - veh.curve_speed_penalty_ms(apex, pitch_rad=pitch, is_left=is_left), 0.0))


def test_left_and_descent_extras_still_bite_below_the_vision_floor():
  """The floor bounds the BASE hump only (as the map floor does): a left curve on a descent still comes in below
  vision's own speed, by exactly the multiplier extras. (left_factor is neutral by default since 2026-09-24; set
  here so the left extra is non-zero.)"""
  veh = _veh(left_factor=1.15)
  v, lat, pitch = 30.0, -3.0, -0.05                   # LEFT (lat < 0), 5 % downhill
  apex = v * math.sqrt(m.VTSC_A_LAT / abs(lat))
  t, *_ = m.icbm_penalise(veh, [], apex, "vis", _vis_sig(v, lat, pitch=pitch), LAT, LON, float("inf"), 0.0)
  extra = veh.curve_speed_penalty_ms(apex, pitch_rad=pitch, is_left=True) - veh.curve_speed_penalty_ms(apex)
  assert extra > 1.0
  assert t == pytest.approx(apex - extra, abs=1e-6)


def test_vision_floor_is_reduce_only_and_never_changes_candidacy():
  """Through the real _icbm_step: the floored target is never above the candidate or the set, never below the
  unfloored one, and binds exactly when the unfloored one binds."""
  on, off = _veh(), _veh(icbm_vis_floor_frac=0.0)
  checked = 0
  for v_set in (25.0, 30.0, 35.0):
    for lat in (2.0, 2.5, 3.0, 4.0, -2.5, -4.0):
      for ttc in (3.0, 5.0, 8.0):
        sig = {**_vis_sig(v_set, lat, ttc=ttc), "v_set": v_set}
        mgr1, step1 = _icbm_stub(on)
        mgr0, step0 = _icbm_stub(off)
        t1, t0 = _published_target(step1, mgr1, sig), _published_target(step0, mgr0, sig)
        assert (t1 is None) == (t0 is None), "the floor changed whether a vision curve binds"
        if t1 is None:
          continue
        apex = v_set * math.sqrt(m.VTSC_A_LAT / abs(lat))
        assert t0 - 0.006 <= t1 <= min(apex, v_set) + 0.006      # IcbmTarget is published rounded to 0.01
        checked += 1
  assert checked >= 15, f"only {checked} binding cases -- the sweep proves little"


def test_vision_floor_is_lightning_only():
  tesla = _veh("TESLA_MODEL_S_HW3", "tesla")
  assert tesla.icbm_vis_floor_ms(30.0) == 0.0
  assert _veh().icbm_vis_floor_ms(30.0) == pytest.approx(30.0)
  for bad in (None, float("nan"), float("inf"), -5.0, 0.0, "x"):
    assert _veh().icbm_vis_floor_ms(bad) == 0.0, bad


def test_curve_json_clamps_the_vis_fraction(tmp_path, monkeypatch):
  import json
  p = tmp_path / "curve.json"
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(p))
  for written, expect in ((2.0, 1.0), (-1.0, 0.0), (0.5, 0.5), (float("nan"), 1.0)):
    p.write_text(json.dumps({"lightning": {"icbm_vis_floor_frac": written}}))
    assert PnwVehicle(FakeCPA(LIGHTNING, "ford"))._curve_cfg["icbm_vis_floor_frac"] == pytest.approx(expect), written


def test_map_floor_unchanged_by_the_vision_floor():
  """A map candidate is still floored at mapd's own rating, not at its (scaled) candidate speed."""
  veh = _veh()
  sig = {"v_ego": 26.0, "v_set": 26.0, "map_target_v": 20.0, "map_target_dist": 205.0,
         "curve_lat_accel_vision": 0.0, "time_to_curve": 10.0, "pitch": None}
  _, flr, *_ = m.icbm_penalise(veh, [], 22.0, "map", sig, LAT, LON, float("inf"), 0.0)
  assert flr == pytest.approx(20.0)
