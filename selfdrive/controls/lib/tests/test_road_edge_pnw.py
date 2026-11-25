"""roadedge2pnw: the road-edge detector (road_edge_pnw.py) and its use in DesireHelper.

The block holds off ONLY the auto (nudgeless) lane change toward the edge side; a torque nudge still starts it, the
blind-spot block is unchanged, and the other side is not affected.
"""
from types import SimpleNamespace

from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.desire_helper import AUTO_LANE_CHANGE_DELAY, LaneChangeState
from openpilot.selfdrive.controls.lib.road_edge_pnw import EDGE_CLEAR_TIME, EDGE_REACTION_TIME, RoadEdgeLaneChange
from openpilot.selfdrive.controls.lib.tests.test_desire_helper import V35, V50, _CS, _dh

V15 = 15 * CV.MPH_TO_MS


def _model(left_edge=False, right_edge=False, edge_y=3.0):
  """A model frame. An edge side = confident edge (std 0.2), no outer lane line (prob 0.05), edge `edge_y` m away."""
  return SimpleNamespace(
    roadEdgeStds=[0.2 if left_edge else 2.0, 0.2 if right_edge else 2.0],
    laneLineProbs=[0.05 if left_edge else 0.9, 0.9, 0.9, 0.05 if right_edge else 0.9],
    roadEdges=[SimpleNamespace(y=[-edge_y]), SimpleNamespace(y=[edge_y])])


def _steps(seconds):
  return int(round(seconds / DT_MDL))


def test_edge_needs_reaction_time_and_clears():
  r = RoadEdgeLaneChange()
  for _ in range(_steps(EDGE_REACTION_TIME) - 1):
    assert r.update(_model(left_edge=True), V50) == (False, False)
  r.update(_model(left_edge=True), V50)  # float sum of DT_MDL steps lands on either side of 1.0 here
  assert r.update(_model(left_edge=True), V50) == (True, False)
  for _ in range(_steps(EDGE_CLEAR_TIME)):
    r.update(_model(), V50)
  assert r.left  # still held for EDGE_CLEAR_TIME
  r.update(_model(), V50)
  assert r.left is False


def test_far_edge_or_outer_lane_line_is_not_an_edge():
  r = RoadEdgeLaneChange()
  for _ in range(_steps(EDGE_REACTION_TIME + 1)):
    r.update(_model(left_edge=True, edge_y=5.0), V50)  # 5.0 - 1.08 m margin > 3.7 m clearance: room for a lane
  assert r.left is False
  m = _model(left_edge=True)
  m.laneLineProbs[0] = 0.9  # a lane line beyond ours on the left: there is a lane there
  for _ in range(_steps(EDGE_REACTION_TIME + 1)):
    r.update(m, V50)
  assert r.left is False


def test_low_speed_resets():
  r = RoadEdgeLaneChange()
  for _ in range(_steps(EDGE_REACTION_TIME + 1)):
    r.update(_model(right_edge=True), V50)
  assert r.right
  assert r.update(_model(right_edge=True), V15) == (False, False)


def _run_edge(d, cs, seconds, left_edge=False, right_edge=False):
  for _ in range(_steps(seconds)):
    d.update(cs, True, 0.0, left_edge, right_edge)
  return d.lane_change_state


def test_edge_blocks_auto_lane_change_toward_it(monkeypatch):
  d = _dh(monkeypatch, highway_class="motorway")
  cs = _CS(V50, left_blinker=True)
  assert _run_edge(d, cs, AUTO_LANE_CHANGE_DELAY + 1.0, left_edge=True) == LaneChangeState.preLaneChange
  assert d.edge_block_logged


def test_edge_on_other_side_does_not_block(monkeypatch):
  d = _dh(monkeypatch, highway_class="motorway")
  cs = _CS(V50, left_blinker=True)
  assert _run_edge(d, cs, AUTO_LANE_CHANGE_DELAY + 0.5, right_edge=True) == LaneChangeState.laneChangeStarting


def test_nudge_overrides_edge(monkeypatch):
  d = _dh(monkeypatch, highway_class="motorway")
  cs = _CS(V50, left_blinker=True, steering_pressed=True, steering_torque=1.0)
  d.update(cs, True, 0.0, True, False)
  d.update(cs, True, 0.0, True, False)
  assert d.lane_change_state == LaneChangeState.laneChangeStarting


def test_blindspot_still_blocks_nudge_with_edge(monkeypatch):
  d = _dh(monkeypatch, highway_class="motorway")
  cs = _CS(V50, left_blinker=True, steering_pressed=True, steering_torque=1.0, left_blindspot=True)
  assert _run_edge(d, cs, 1.0, left_edge=True) == LaneChangeState.preLaneChange


def test_full_delay_needed_after_edge_clears(monkeypatch):
  d = _dh(monkeypatch, highway_class="motorway")
  cs = _CS(V50, left_blinker=True)
  _run_edge(d, cs, AUTO_LANE_CHANGE_DELAY + 1.0, left_edge=True)
  assert _run_edge(d, cs, AUTO_LANE_CHANGE_DELAY - 0.1) == LaneChangeState.preLaneChange
  assert _run_edge(d, cs, 0.3) == LaneChangeState.laneChangeStarting


# ---------------------------------------------------------------------------------------------------------------- selfdrived alert
def _sd(brand, fp):
  from opendbc.car import structs
  from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD
  cp = structs.CarParams.new_message()
  cp.brand, cp.carFingerprint = brand, fp
  return SelfdriveD(CP=cp)


def test_alert_is_a_plain_warning():
  from openpilot.selfdrive.selfdrived.events import EVENTS, ET, EventNamePnw
  assert set(EVENTS[EventNamePnw.laneChangeRoadEdge]) == {ET.WARNING}


def test_selfdrived_detector_on_both_cars_and_fails_loudly(monkeypatch):
  from openpilot.common.prefix import OpenpilotPrefix
  from openpilot.selfdrive.selfdrived import selfdrived as sd_mod
  with OpenpilotPrefix():
    for brand, fp in (("ford", "FORD_F_150_LIGHTNING_MK1"), ("tesla", "TESLA_MODEL_S_HW3")):
      sd = _sd(brand, fp)
      assert sd.road_edge is not None, f"{fp}: no road-edge alert on a nudgeless car"
      sd.sm.data['modelV2'] = _model(left_edge=True)
      sd.sm.updated['modelV2'] = True
      for _ in range(_steps(EDGE_REACTION_TIME) + 2):
        left, right = sd._road_edge_step(SimpleNamespace(vEgo=V50))
      assert (left, right) == (True, False)

    logged = []
    monkeypatch.setattr(sd_mod.cloudlog, "exception", lambda msg, *a, **k: logged.append(msg))
    sd.sm.data['modelV2'] = SimpleNamespace(roadEdgeStds=[], laneLineProbs=[], roadEdges=[])  # malformed
    assert sd._road_edge_step(SimpleNamespace(vEgo=V50)) == (False, False)
    assert len(logged) == 1 and "NOT deciding" in logged[0]


def _edge_logs(monkeypatch):
  import openpilot.selfdrive.controls.lib.desire_helper as dh_mod
  logs = []
  monkeypatch.setattr(dh_mod.cloudlog, "event", lambda name, **kw: logs.append(name) if name == "lane_change_edge_block" else None)
  return logs


def test_edge_log_only_when_the_edge_is_what_blocks(monkeypatch):
  logs = _edge_logs(monkeypatch)
  d = _dh(monkeypatch, highway_class="residential")       # off highway: the highway gate blocks, not the edge
  _run_edge(d, _CS(V35, left_blinker=True), 1.0, left_edge=True)
  d = _dh(monkeypatch, highway_class="motorway")           # blind spot blocks
  _run_edge(d, _CS(V50, left_blinker=True, left_blindspot=True), 1.0, left_edge=True)
  d = _dh(monkeypatch, highway_class="motorway", nudge_required=True)  # no auto change at all
  _run_edge(d, _CS(V50, left_blinker=True), 1.0, left_edge=True)
  assert logs == []
  d = _dh(monkeypatch, highway_class="motorway")
  _run_edge(d, _CS(V50, left_blinker=True), 1.0, left_edge=True)
  assert logs == ["lane_change_edge_block"]


def _finish(d, cs):
  """Drive a started lane change to its end with the blinker still held (the model reports the change done)."""
  for _ in range(_steps(3.0)):
    d.update(cs, True, 0.0)
    if d.lane_change_state == LaneChangeState.preLaneChange:
      return
  raise AssertionError(f"lane change did not finish: {d.lane_change_state}")


def test_held_blinker_second_change_is_a_new_episode(monkeypatch):
  logs = _edge_logs(monkeypatch)
  d = _dh(monkeypatch, highway_class="motorway")
  cs = _CS(V50, left_blinker=True)
  _run_edge(d, cs, 0.4, left_edge=True)                    # edge seen and logged early in the first episode
  assert logs == ["lane_change_edge_block"]
  assert _run_edge(d, cs, AUTO_LANE_CHANGE_DELAY + 0.3) == LaneChangeState.laneChangeStarting
  _finish(d, cs)
  assert d.auto_lane_change_timer == 0.0 and not d.edge_block_logged
  assert _run_edge(d, cs, 1.0, left_edge=True) == LaneChangeState.preLaneChange  # the next lane ends at an edge
  assert logs == ["lane_change_edge_block"] * 2


def test_selfdrived_alert_only_when_the_auto_change_would_run(monkeypatch):
  from openpilot.common.prefix import OpenpilotPrefix
  with OpenpilotPrefix():
    sd = _sd("ford", "FORD_F_150_LIGHTNING_MK1")
    sd.sm.data['modelV2'] = _model(left_edge=True)
    sd.sm.updated['modelV2'] = True
    for _ in range(_steps(EDGE_REACTION_TIME) + 2):
      sd._road_edge_step(SimpleNamespace(vEgo=V50))
    assert sd._road_edge_step(SimpleNamespace(vEgo=V50)) == (True, False)
    assert sd._road_edge_step(SimpleNamespace(vEgo=V35)) == (False, False)  # below the highway speed: no auto change to block
    sd.nudge_required = True
    assert sd._road_edge_step(SimpleNamespace(vEgo=V50)) == (False, False)  # NudgeForLaneChange on: nothing is blocked
