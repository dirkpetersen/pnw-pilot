"""handsready2pnw: "Keep hands ready: sharp turn" (selfdrive/selfdrived/sharp_turn_pnw.py + SelfdriveD._sharp_turn_step).
Evidence: drives/2026-10-21/seattle-left-curve-overshoot (22:39 PT: plan curvature 0.02-0.038 1/m, laneLineProbs <= 0.07, 7.4-8.9 m/s)."""
import time
from types import SimpleNamespace

import pytest

from cereal import log
from opendbc.car import structs
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import CESController
from openpilot.selfdrive.controls.lib.pnw_vehicle import PnwVehicle
from openpilot.selfdrive.selfdrived import sharp_turn_pnw as M
from openpilot.selfdrive.selfdrived.events import EVENTS, ET, AudibleAlert, EventNamePnw, Events
from openpilot.selfdrive.selfdrived.mads_pnw import has_blocking_event
from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD

LIGHTNING = "FORD_F_150_LIGHTNING_MK1"
TESLA = "TESLA_MODEL_S_HW3"
DT = 0.01


def make_cp(brand, fp):
  cp = structs.CarParams.new_message()
  cp.brand, cp.carFingerprint = brand, fp
  return cp


def model(k_now=0.0, k_ahead=0.0, lane=0.05, v=8.0):
  """modelV2 with desiredCurvature k_now and a plan whose curvature is k_ahead at t = 1.5 s (0 elsewhere)."""
  m = log.ModelDataV2.new_message()
  m.action.desiredCurvature = k_now
  ts = [0.0, 0.5, 1.0, 1.5, 2.5, 4.0]
  m.orientationRate.t = ts
  m.orientationRate.z = [k_ahead * v if t == 1.5 else 0.0 for t in ts]
  m.velocity.x = [v] * len(ts)
  m.laneLineProbs = [0.1, lane, lane, 0.1]
  return m


# ---------------------------------------------------------------------------------------------------------------- pure inputs
def test_plan_curvature_takes_the_largest_of_now_and_the_next_two_seconds():
  assert M.plan_curvature(model(k_now=0.005)) == pytest.approx(0.005)
  assert M.plan_curvature(model(k_now=0.005, k_ahead=-0.025)) == pytest.approx(0.025)   # left (negative) counts too
  m = model()
  m.orientationRate.z = [0.0, 0.0, 0.0, 0.0, 0.5, 0.5]   # sharp only beyond 2 s: not yet
  assert M.plan_curvature(m) == 0.0


def test_inputs_raise_on_a_malformed_message_never_read_as_straight_or_confident():
  m = log.ModelDataV2.new_message()          # no laneLineProbs at all
  with pytest.raises(IndexError):
    M.ego_lane_prob(m)
  assert M.ego_lane_prob(model(lane=0.05)) == pytest.approx(0.05)
  bad = model()
  bad.action.desiredCurvature = float("nan")
  with pytest.raises(ValueError):
    M.plan_curvature(bad)


def test_lane_threshold_is_the_lane_centering_one():
  from openpilot.selfdrive.controls.lib.lane_centering import DEFAULT_TUNING
  assert M.LANE_PROB_LOW == DEFAULT_TUNING["min_lane_prob"] == 0.6


# ---------------------------------------------------------------------------------------------------------------- pure state machine
def run(st, t0, secs, **kw):
  """Feed the detector for `secs` at 100 Hz from t0. Returns (fires, list of active per frame, end time)."""
  args = dict(armed=True, v_ego=8.0, k_plan=0.025, k_meas=0.02, lane_prob=0.05) | kw
  fires, act, t = 0, [], t0
  for _ in range(round(secs / DT)):
    fires += st.update(t, **args)
    act.append(st.active)
    t += DT
  return fires, act, t


def test_fires_on_the_2239_pattern_after_the_persist_time_only():
  st = M.SharpTurnHands()
  fires, act, _ = run(st, 0.0, 0.29)
  assert fires == 0 and not any(act)
  fires, act, _ = run(st, 0.29, 0.05)
  assert fires == 1 and act[-1]


@pytest.mark.parametrize("kw", [dict(lane_prob=0.9), dict(k_plan=0.015), dict(v_ego=2.5), dict(v_ego=16.0), dict(armed=False)])
def test_never_fires_when_a_gate_fails(kw):
  """confident lanes / gentle curve / too slow / too fast / not armed (Tesla, toggle off, not steering, not angle mode)"""
  fires, act, _ = run(M.SharpTurnHands(), 0.0, 5.0, **kw)
  assert fires == 0 and not any(act)


def test_a_gap_in_the_condition_restarts_the_persist_timer():
  st = M.SharpTurnHands()
  t = 0.0
  for _ in range(10):   # 0.2 s on, one frame off: never 0.3 s continuous
    _, _, t = run(st, t, 0.2)
    _, _, t = run(st, t, DT, lane_prob=0.9)
  assert not st.active


def test_holds_at_least_two_seconds_then_until_the_turn_ends_with_no_flicker():
  st = M.SharpTurnHands()
  _, _, t = run(st, 0.0, 0.31)
  assert st.active
  fire_t = t - DT
  # plan straightens at once and lane lines come back: still held for 2 s
  _, act, t = run(st, t, 1.5, k_plan=0.0, k_meas=0.0, lane_prob=0.9)
  assert all(act)
  # the truck is still turning (PSCM unwind): the measured curvature keeps it up past 2 s
  _, act, t = run(st, t, 1.0, k_plan=0.0, k_meas=0.015, lane_prob=0.9)
  assert all(act) and t - fire_t > M.HOLD_S
  # turn ended: clears, and stays clear (no flicker) while straight
  _, act, t = run(st, t, 1.0, k_plan=0.005, k_meas=-0.005, lane_prob=0.9)
  assert act[0] is False and not any(act)


def test_rate_limited_to_one_fire_per_ten_seconds():
  st = M.SharpTurnHands()
  fires, _, t = run(st, 0.0, 0.31)
  assert fires == 1
  _, _, t = run(st, t, 2.5, k_plan=0.0, k_meas=0.0)
  assert not st.active
  fires, act, t = run(st, t, 6.0)               # sharp again at ~2.8..8.8 s: inside the 10 s window
  assert fires == 0 and not any(act)
  fires, act, _ = run(st, t, 2.0)               # past 10 s since the first fire
  assert fires == 1


def test_clears_at_once_when_steering_stops():
  st = M.SharpTurnHands()
  _, _, t = run(st, 0.0, 0.31)
  assert st.active
  st.update(t, False, 8.0, 0.025, 0.02, 0.05)
  assert not st.active


# ---------------------------------------------------------------------------------------------------------------- capability + event
def test_capability_is_lightning_only():
  assert PnwVehicle(make_cp("ford", LIGHTNING)).sharp_turn_hands_alert is True
  assert PnwVehicle(make_cp("tesla", TESLA)).sharp_turn_hands_alert is False
  assert PnwVehicle(None).sharp_turn_hands_alert is False


def test_event_is_a_warning_only_soft_prompt_that_cannot_disengage():
  ev = EVENTS[EventNamePnw.sharpTurnHands]
  assert set(ev) == {ET.WARNING}
  a = ev[ET.WARNING]
  assert a.alert_text_1 == "Keep hands ready: sharp turn" and a.audible_alert == AudibleAlert.prompt
  e = Events()
  e.add(EventNamePnw.sharpTurnHands)
  assert not has_blocking_event(e)
  assert not e.contains(ET.NO_ENTRY) and not e.contains(ET.SOFT_DISABLE) and not e.contains(ET.IMMEDIATE_DISABLE)


# ---------------------------------------------------------------------------------------------------------------- selfdrived wiring
class Recorder:
  def __init__(self):
    self.records = []

  def log_take_control_alert(self, payload):
    self.records.append(payload)


def drive(sd, monkeypatch, secs, m, v=8.0, lat_active=True, k_meas=0.02, t0=1000.0):
  """Run SelfdriveD._sharp_turn_step at 100 Hz with a fixed modelV2. Returns the frames the event was raised."""
  sd.sm.data['modelV2'] = m
  clock = {"t": t0}
  monkeypatch.setattr(time, "monotonic", lambda: clock["t"])
  shown = 0
  for _ in range(round(secs / DT)):
    sd.events = Events()
    sd._sharp_turn_step(SimpleNamespace(vEgo=v), lat_active, k_meas)
    shown += EventNamePnw.sharpTurnHands in sd.events.names
    clock["t"] += DT
  return shown


def lightning(mode="angle"):
  sd = SelfdriveD(CP=make_cp("ford", LIGHTNING))
  sd.latstat_mem.put("FordLatStatus", {"mode": mode, "ts": time.time()})  # noqa: TID251 -- carcontroller stamps wall time
  sd.sm.frame = 0
  sd.ces_pnw = Recorder()
  return sd


def test_selfdrived_fires_logs_and_shows_on_the_lightning(monkeypatch):
  with OpenpilotPrefix():
    sd = lightning()
    assert sd.sharp_turn is not None and sd.disable_sharp_turn is False   # param default "0": ON by default
    assert drive(sd, monkeypatch, 1.0, model(k_now=0.025)) > 0
    assert len(sd.ces_pnw.records) == 1
    r = sd.ces_pnw.records[0]
    assert r["name"] == "sharpTurnHands" and r["phase"] == "fire"
    assert r["k"] == pytest.approx(0.025) and r["laneConf"] == pytest.approx(0.05) and r["vEgo"] == 8.0 and r["kMeas"] == 0.02


def test_selfdrived_never_on_a_confident_lane(monkeypatch):
  with OpenpilotPrefix():
    sd = lightning()
    assert drive(sd, monkeypatch, 3.0, model(k_now=0.03, lane=0.8)) == 0 and sd.ces_pnw.records == []


def test_selfdrived_toggle_off_never(monkeypatch):
  with OpenpilotPrefix():
    sd = lightning()
    sd.disable_sharp_turn = True
    assert drive(sd, monkeypatch, 3.0, model(k_now=0.025)) == 0 and sd.ces_pnw.records == []


def test_selfdrived_not_in_curvature_mode_and_not_when_not_steering(monkeypatch):
  with OpenpilotPrefix():
    sd = lightning(mode="curvature")
    assert drive(sd, monkeypatch, 3.0, model(k_now=0.025)) == 0
    sd = lightning()
    assert drive(sd, monkeypatch, 3.0, model(k_now=0.025), lat_active=False) == 0


def test_selfdrived_tesla_never(monkeypatch):
  with OpenpilotPrefix():
    sd = SelfdriveD(CP=make_cp("tesla", TESLA))
    sd.ces_pnw = Recorder()
    assert sd.sharp_turn is None
    assert drive(sd, monkeypatch, 3.0, model(k_now=0.025)) == 0 and sd.ces_pnw.records == []


def test_selfdrived_malformed_model_is_logged_not_silent(monkeypatch):
  logged = []
  import openpilot.selfdrive.selfdrived.selfdrived as SD
  monkeypatch.setattr(SD.cloudlog, "exception", lambda msg, *a, **k: logged.append(msg))
  with OpenpilotPrefix():
    sd = lightning()
    assert drive(sd, monkeypatch, 0.5, log.ModelDataV2.new_message()) == 0
    assert any("NOT deciding" in m for m in logged)


def test_ces_record_carries_curvature_lane_conf_speed_and_position():
  h = SimpleNamespace(_car="LIGHTNING", _mode=0, _cur_lat=47.6679, _cur_lon=-122.3616, _cur_bearing=331.0,
                      _sl_ang_des=None, _sl_ang_act=None, _sl_ang_err=None, _sl_lat_dem=None, _sl_lat_max=None, recs=[])
  h._append_event = h.recs.append
  CESController.log_take_control_alert(h, {"name": "sharpTurnHands", "phase": "fire", "vEgo": 8.0, "k": 0.025, "kMeas": 0.02,
                                           "laneConf": 0.05})
  r = h.recs[0]
  assert (r["ev"], r["name"], r["k"], r["kMeas"], r["laneConf"], r["vEgo"], r["lat"], r["lon"]) == \
         ("alert", "sharpTurnHands", 0.025, 0.02, 0.05, 8.0, 47.6679, -122.3616)
