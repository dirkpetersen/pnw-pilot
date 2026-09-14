"""lcabort2pnw: the driver steering against an openpilot lane change (docs/PENDING-WORK.md, MADS item (c)).

This round is a SHADOW: DesireHelper counts consecutive laneChangeStarting ticks where carstate's debounced
steeringPressed has its torque opposite the change direction, and logs `lane_change_abort_shadow` once when the
count reaches LANE_CHANGE_ABORT_TICKS. Nothing the car does may change, so the replays below also assert the
laneChangeState trace DesireHelper produced on the car, tick for tick.

Replays use tests/data/lcabort_2026.json (see its _about): the 2026-08-31 Raven fight that ended in
steerDisengage, and a 2026-09-12 Lightning change the driver helped with same-direction torque.
"""
import json
import os

import openpilot.selfdrive.controls.lib.desire_helper as dh_module
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.desire_helper import LANE_CHANGE_ABORT_TICKS, LaneChangeState
from openpilot.selfdrive.controls.lib.tests.test_desire_helper import _CS, _dh

FIXTURE = os.path.join(os.path.dirname(__file__), "data", "lcabort_2026.json")
V30 = 30.0  # m/s: above HIGHWAY_MIN_SPEED, so nudgeless is allowed on speed alone
STATE_NAMES = {LaneChangeState.off: "off", LaneChangeState.preLaneChange: "preLaneChange",
               LaneChangeState.laneChangeStarting: "laneChangeStarting", LaneChangeState.laneChangeFinishing: "laneChangeFinishing"}


class _Log:
  def __init__(self):
    self.events = []

  def event(self, name, **kw):
    self.events.append((name, kw))


def _logged(monkeypatch):
  log = _Log()
  monkeypatch.setattr(dh_module, "cloudlog", log)
  return log


def _replay(monkeypatch, window, nudge_required):
  with open(FIXTURE) as f:
    fx = json.load(f)
  cols = fx["_cols"]
  d = _dh(monkeypatch, nudge_required=nudge_required)
  log = _logged(monkeypatch)
  trace = []
  for row in fx[window]:
    r = dict(zip(cols, row, strict=True))
    cs = _CS(r["vEgo"], left_blinker=r["leftBlinker"], right_blinker=r["rightBlinker"], steering_pressed=r["steeringPressed"],
             steering_torque=r["steeringTorque"], left_blindspot=r["leftBlindspot"], right_blindspot=r["rightBlindspot"])
    n_before = len(log.events)
    d.update(cs, r["latActive"], r["laneChangeProb"])
    trace.append((r, STATE_NAMES[d.lane_change_state], len(log.events) > n_before))
  return d, log, trace


def _start_change(d, torque=0.0, pressed=False):
  """Blinker left at V30, nudgeless: off -> preLaneChange -> laneChangeStarting. Returns the held CarState."""
  cs = _CS(V30, left_blinker=True, steering_pressed=pressed, steering_torque=torque)
  for _ in range(100):
    d.update(cs, True, 1.0)
    if d.lane_change_state == LaneChangeState.laneChangeStarting:
      return cs
  raise AssertionError(f"lane change never started, state={d.lane_change_state}")


def _hold(d, cs, ticks, prob=1.0):
  for _ in range(ticks):
    d.update(cs, True, prob)


AGAINST_LEFT = _CS(V30, left_blinker=True, steering_pressed=True, steering_torque=-2.0)


# --- real telemetry ----------------------------------------------------------------------------------------------

def test_raven_fight_logs_once_at_the_sixth_tick_and_changes_nothing(monkeypatch):
  d, log, trace = _replay(monkeypatch, "tesla_0831_i5_fight", nudge_required=False)
  assert [s for _, s, _ in trace] == [r["laneChangeState"] for r, _, _ in trace]  # bit-identical to the car
  fired = [r["t"] for r, _, f in trace if f]
  assert fired == [51.097], fired  # not the 3-tick blip at 49.75-49.85; 6th tick of the fight, 0.46 s before disengage
  (name, kw), = log.events
  assert name == "lane_change_abort_shadow"
  assert kw["direction"] == "left" and kw["torque"] == -1.85 and kw["v_ego"] == 33.21
  assert kw["error"] is False
  assert 0.0 <= kw["lane_change_ll_prob"] < 0.01 and kw["lane_change_prob"] == 0.996
  assert kw["t_in_change"] == round(30 * DT_MDL, 2)  # entered laneChangeStarting at 49.597, 30 ticks (1.5 s) earlier


def test_lightning_same_direction_help_never_logs(monkeypatch):
  _, log, trace = _replay(monkeypatch, "lightning_0912_same_dir", nudge_required=True)
  assert [s for _, s, _ in trace] == [r["laneChangeState"] for r, _, _ in trace]
  assert sum(1 for r, s, _ in trace if s == "laneChangeStarting" and r["steeringPressed"]) == 54  # the help is really there
  assert log.events == []


# --- synthetic boundaries ----------------------------------------------------------------------------------------

def test_one_tick_short_does_not_log_then_the_next_does(monkeypatch):
  d = _dh(monkeypatch)
  _start_change(d)
  log = _logged(monkeypatch)
  _hold(d, AGAINST_LEFT, LANE_CHANGE_ABORT_TICKS - 1)
  assert log.events == []
  _hold(d, AGAINST_LEFT, 1)
  assert len(log.events) == 1
  _hold(d, AGAINST_LEFT, 40)  # still fighting: one event per sustained override, not one per tick
  assert len(log.events) == 1
  assert d.lane_change_state == LaneChangeState.laneChangeStarting  # shadow: the change goes on


def test_steering_pressed_is_the_gate_not_raw_torque(monkeypatch):
  # A big opposite torque that carstate has not (yet) debounced into steeringPressed breaks the run.
  d = _dh(monkeypatch)
  _start_change(d)
  log = _logged(monkeypatch)
  unpressed = _CS(V30, left_blinker=True, steering_pressed=False, steering_torque=-6.0)
  _hold(d, AGAINST_LEFT, LANE_CHANGE_ABORT_TICKS - 1)
  _hold(d, unpressed, 1)
  _hold(d, AGAINST_LEFT, LANE_CHANGE_ABORT_TICKS - 1)
  assert log.events == []
  _hold(d, AGAINST_LEFT, 1)
  assert len(log.events) == 1


def test_pressed_with_zero_torque_is_not_against(monkeypatch):
  for blinker in ("left", "right"):
    d = _dh(monkeypatch)
    cs = _CS(V30, left_blinker=blinker == "left", right_blinker=blinker == "right")
    _hold(d, cs, 20)
    assert d.lane_change_state == LaneChangeState.laneChangeStarting
    log = _logged(monkeypatch)
    _hold(d, _CS(V30, left_blinker=blinker == "left", right_blinker=blinker == "right", steering_pressed=True, steering_torque=0.0),
          3 * LANE_CHANGE_ABORT_TICKS)
    assert log.events == [], blinker


def test_same_direction_torque_never_logs(monkeypatch):
  d = _dh(monkeypatch)
  _start_change(d)
  log = _logged(monkeypatch)
  _hold(d, _CS(V30, left_blinker=True, steering_pressed=True, steering_torque=2.0), 3 * LANE_CHANGE_ABORT_TICKS)
  assert log.events == []


def test_right_change_against_is_positive_torque(monkeypatch):
  d = _dh(monkeypatch)
  cs = _CS(V30, right_blinker=True)
  for _ in range(100):
    d.update(cs, True, 1.0)
    if d.lane_change_state == LaneChangeState.laneChangeStarting:
      break
  assert d.lane_change_state == LaneChangeState.laneChangeStarting
  log = _logged(monkeypatch)
  _hold(d, _CS(V30, right_blinker=True, steering_pressed=True, steering_torque=-2.0), 3 * LANE_CHANGE_ABORT_TICKS)
  assert log.events == []
  _hold(d, _CS(V30, right_blinker=True, steering_pressed=True, steering_torque=2.0), LANE_CHANGE_ABORT_TICKS)
  assert len(log.events) == 1 and log.events[0][1]["direction"] == "right"


def test_count_restarts_with_each_lane_change(monkeypatch):
  # A change that ends (lateral drops) mid-run must not hand its count to the next change.
  d = _dh(monkeypatch)
  _start_change(d)
  log = _logged(monkeypatch)
  _hold(d, AGAINST_LEFT, LANE_CHANGE_ABORT_TICKS - 1)
  d.update(_CS(V30), False, 0.0)  # lateral inactive -> off
  d.update(_CS(V30), True, 0.0)   # blinker released
  _start_change(d)
  _hold(d, AGAINST_LEFT, 1)
  assert log.events == []


def test_counts_only_in_lane_change_starting(monkeypatch):
  d = _dh(monkeypatch, nudge_required=True)  # no auto start: stays in preLaneChange
  log = _logged(monkeypatch)
  _hold(d, AGAINST_LEFT, 3 * LANE_CHANGE_ABORT_TICKS)
  assert d.lane_change_state == LaneChangeState.preLaneChange
  assert d.against_ticks == 0 and log.events == []
