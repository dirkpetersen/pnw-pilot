"""steersatfloor2pnw: Ford angle-mode "Take Control" demand floor (2.2 m/s2) + the never-silent suppression log.
Fixture = the 56 steerSaturated episodes of the 10-09..10-11 trip (drives/2026-10-11/canyon-steersat)."""
import inspect
import json
import os
import textwrap
from types import SimpleNamespace

from cereal import car
from openpilot.selfdrive.controls.lib.pnw_vehicle import PnwVehicle, STEER_SAT_ANGLE_FLOOR
from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD, steer_sat_verdict

ANGLE = car.CarParams.SteerControlType.angle
TORQUE = car.CarParams.SteerControlType.torque
with open(os.path.join(os.path.dirname(__file__), "data", "steersat_episodes_2026-10-11.json")) as _f:
  EPISODES = json.load(_f)


def cp(fp, ctype=ANGLE):
  return SimpleNamespace(carFingerprint=fp, brand="ford" if "FORD" in fp else "tesla", steerControlType=ctype)


def verdict(demand, floor):
  # actual lateral accel 0.8 * desired: undershooting ratio 1.25 > 1.2, as in the trip data
  return steer_sat_verdict(demand, 0.8 * demand, True, floor)


def test_capability_lightning_angle_only():
  assert PnwVehicle(cp("FORD_F_150_LIGHTNING_MK1")).steer_sat_demand_floor == STEER_SAT_ANGLE_FLOOR == 2.2
  assert PnwVehicle(cp("FORD_F_150_LIGHTNING_MK1", TORQUE)).steer_sat_demand_floor == 1.0  # curvature mode: stock
  # Tesla Raven is angle-steered too and must stay exactly stock
  assert PnwVehicle(cp("TESLA_MODEL_S_HW3")).steer_sat_demand_floor == 1.0
  assert PnwVehicle(None).steer_sat_demand_floor == 1.0


def test_verdict_below_and_above_floor():
  assert verdict(1.8, 2.2) == "suppressed"
  assert verdict(2.5, 2.2) == "fire"
  assert verdict(0.9, 2.2) is None            # below the stock gate: never an alert, never a suppression
  assert steer_sat_verdict(2.5, 0.8 * 2.5, False, 2.2) is None   # not saturated
  assert steer_sat_verdict(2.5, 2.4, True, 2.2) is None          # not undershooting


def test_stock_floor_is_identical_to_old_logic():
  for d in (1.01, 1.5, 3.0):
    assert verdict(d, 1.0) == "fire"
  assert verdict(1.0, 1.0) is None


def test_replay_56_episodes():
  assert len(EPISODES) == 56
  genuine = [e for e in EPISODES if e["genuine"]]
  assert len(genuine) == 7
  assert all(verdict(e["demand"], 2.2) == "fire" for e in genuine)       # every genuine case still alerts
  kept = [e for e in EPISODES if verdict(e["demand"], 2.2) == "fire"]
  assert len(kept) == 14
  assert all(verdict(e["demand"], 1.0) == "fire" for e in EPISODES if e["demand"] > 1.0)


def test_update_events_uses_floor_and_logs_suppression():
  src = textwrap.dedent(inspect.getsource(SelfdriveD.update_events))
  assert "self.steer_sat_floor" in src and "_note_steer_sat_suppressed" in src


def _stub():
  calls = []
  s = object.__new__(SelfdriveD)
  s.sm = SimpleNamespace(frame=1000)
  s.steer_sat_floor = 2.2
  s.steer_sat_sup_count = 0
  s.last_steer_sat_sup_frame = None
  s.ces_pnw = SimpleNamespace(log_take_control_alert=calls.append)
  return s, calls


def test_suppression_is_logged_once_per_episode():
  s, calls = _stub()
  lac = SimpleNamespace(steeringAngleDeg=10.0, steeringAngleDesiredDeg=14.0)
  cs = SimpleNamespace(vEgo=15.0)
  for fr in range(1000, 1100):             # 1 s of continuous suppression -> one record
    s.sm.frame = fr
    s._note_steer_sat_suppressed(cs, 1.8, lac)
  assert len(calls) == 1 and s.steer_sat_sup_count == 1
  r = calls[0]
  assert r["phase"] == "suppressed" and r["demand"] == 1.8 and r["floor"] == 2.2 and r["angErr"] == 4.0 and r["supCount"] == 1
  s.sm.frame = 1400                        # a new episode
  s._note_steer_sat_suppressed(cs, 1.5, lac)
  assert len(calls) == 2 and calls[1]["supCount"] == 2
