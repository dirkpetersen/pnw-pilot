"""steersatfloor2pnw: Lightning angle-mode "Take Control" demand floor (2.2 m/s2), its escapes, and the never-silent suppression log.
Replay fixture = 100 Hz rlog rows (modelV2 desiredCurvature, controlsState curvature, lac saturated, wheel error) around the 21 stock
steerSaturated episodes of Sunday 2026-10-11 (drives/2026-10-11/canyon-steersat). Saturday has no rlog."""
import json
import os
import time
from types import SimpleNamespace

from cereal import log
from openpilot.common.prefix import OpenpilotPrefix
from opendbc.car import structs
from openpilot.selfdrive.controls.lib.pnw_vehicle import PnwVehicle, STEER_SAT_ANGLE_FLOOR
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD, steer_sat_verdict

EventName = log.OnroadEvent.EventName
LIGHTNING = "FORD_F_150_LIGHTNING_MK1"
TESLA = "TESLA_MODEL_S_HW3"
with open(os.path.join(os.path.dirname(__file__), "data", "steersat_rlog_runtime_2026-10-11.json")) as _f:
  REPLAY = json.load(_f)


def make_cp(brand, fp):
  cp = structs.CarParams.new_message()
  cp.brand = brand
  cp.carFingerprint = fp
  return cp


def lightning_sd():
  return SelfdriveD(CP=make_cp("ford", LIGHTNING))


def live_mode(sd, mode, ts=None):
  sd.latstat_mem.put("FordLatStatus", {"mode": mode, "ts": time.time() if ts is None else ts})  # noqa: TID251 -- carcontroller stamps wall time


def prime(sd, mode="angle"):
  """Start a fresh drive state: carcontroller says `mode`, selfdrived has polled it."""
  live_mode(sd, mode)
  sd.sm.frame = 0
  sd.events = Events()
  sd._refresh_steer_sat_mode()


class Recorder:
  def __init__(self):
    self.records = []

  def log_take_control_alert(self, payload):
    self.records.append(payload)


def run_frame(sd, frame, desired, err, saturated=True, v=15.0):
  """One selfdrived tick of the steerSaturated decision; returns True if the alert fired."""
  sd.sm.frame = frame
  sd.events = Events()
  lac = SimpleNamespace(saturated=saturated, steeringAngleDesiredDeg=err, steeringAngleDeg=0.0)
  sd._steer_sat_step(SimpleNamespace(vEgo=v), desired, 0.8 * desired, lac)   # actual = 0.8 desired -> ratio 1.25 > 1.2
  return EventName.steerSaturated in sd.events.names


def test_capability_lightning_only():
  assert PnwVehicle(make_cp("ford", LIGHTNING)).steer_sat_demand_floor == STEER_SAT_ANGLE_FLOOR == 2.2
  assert PnwVehicle(make_cp("tesla", TESLA)).steer_sat_demand_floor == 1.0
  assert PnwVehicle(None).steer_sat_demand_floor == 1.0


def test_real_init_carries_floor_to_the_decision():
  """B1: the capability must survive SelfdriveD.__init__ and reach the decision (a later reset to 1.0 once disabled the feature)."""
  with OpenpilotPrefix():
    sd = lightning_sd()
    assert sd.steer_sat_floor == 2.2 and sd.latstat_mem is not None
    prime(sd, "angle")
    assert sd.steer_sat_floor_now == 2.2
    assert not run_frame(sd, 1, 1.8, 5.0)       # below the floor, ordinary lag: quiet
    assert run_frame(sd, 2, 2.5, 5.0)           # above: alert as today
    tesla = SelfdriveD(CP=make_cp("tesla", TESLA))
    assert tesla.steer_sat_floor == 1.0 and tesla.latstat_mem is None
    tesla.sm.frame = 1
    assert run_frame(tesla, 3, 1.8, 5.0)        # Tesla identical to stock at 1.0
    assert tesla.steer_sat_sup_count == 0


def test_floor_follows_the_lateral_mode_actually_in_use():
  """B2: floor only while the carcontroller reports the angle path; unknown / stale / curvature fall back to stock 1.0."""
  with OpenpilotPrefix():
    sd = lightning_sd()
    prime(sd, "angle")
    assert sd.steer_sat_floor_now == 2.2
    live_mode(sd, "stock")                      # LateralAngleExt fell back to curvature mid-drive
    sd.sm.frame = 25
    sd._refresh_steer_sat_mode()
    assert sd.steer_sat_floor_now == 1.0 and sd.steer_sat_mode == "stock"
    assert run_frame(sd, 26, 1.8, 5.0)          # stock behaviour again
    live_mode(sd, "angle", ts=time.time() - 60)  # noqa: TID251  # stale status
    sd.sm.frame = 50
    sd._refresh_steer_sat_mode()
    assert sd.steer_sat_floor_now == 1.0 and sd.steer_sat_mode is None
    live_mode(sd, "angle")
    sd.sm.frame = 75
    sd._refresh_steer_sat_mode()
    assert sd.steer_sat_floor_now == 2.2


def test_verdict_stock_floor_identical_to_old_logic():
  for d in (1.01, 1.5, 3.0):
    assert steer_sat_verdict(d, 0.8 * d, True, 1.0) == "fire"
  assert steer_sat_verdict(1.0, 0.8, True, 1.0) is None
  assert steer_sat_verdict(0.9, 0.5, True, 2.2) is None                  # under the stock gate: no alert, no suppression
  assert steer_sat_verdict(2.5, 0.8 * 2.5, False, 2.2) is None           # not saturated
  assert steer_sat_verdict(2.5, 2.4, True, 2.2) is None                  # not undershooting


def test_escape_large_wheel_error_keeps_firing():
  with OpenpilotPrefix():
    sd = lightning_sd()
    prime(sd)
    assert not run_frame(sd, 1, 1.8, 9.9)
    assert run_frame(sd, 2, 1.8, 10.0)          # >= 10 deg behind: not ordinary lag
    assert sd.steer_sat_sup_escaped


def test_escape_long_suppression_keeps_firing():
  with OpenpilotPrefix():
    sd = lightning_sd()
    prime(sd)
    fired = [run_frame(sd, f, 1.8, 5.0) for f in range(1, 260)]
    assert not any(fired[:195])                 # < 2 s: quiet
    assert fired[-1] and sd.steer_sat_sup_escaped   # >= 2 s in one episode: alert returns


def test_suppression_is_logged_with_distinct_name_and_close_summary():
  with OpenpilotPrefix():
    sd = lightning_sd()
    rec = Recorder()
    sd.ces_pnw = rec
    prime(sd)
    for f in range(1, 101):
      run_frame(sd, f, 1.5 + 0.005 * f, 3.0 + 0.04 * f)
    assert [r["phase"] for r in rec.records] == ["start"]
    assert rec.records[0]["name"] == "steerSaturatedSuppressed" and rec.records[0]["supCount"] == 1
    sd._steer_sat_sup_close_check(SimpleNamespace(vEgo=15.0))   # not yet: gap < hold-off
    assert len(rec.records) == 1
    sd.sm.frame = 400
    sd._steer_sat_sup_close_check(SimpleNamespace(vEgo=15.0))
    end = rec.records[1]
    assert end["phase"] == "end" and end["name"] == "steerSaturatedSuppressed"
    assert end["peakDemand"] == 2.0 and end["peakAngErr"] == 7.0 and end["durationS"] == 0.99 and end["escaped"] is False
    assert run_frame(sd, 500, 1.8, 5.0) is False and len(rec.records) == 3   # next episode: new start record
    assert rec.records[2]["supCount"] == 2


def test_replay_sunday_rlog_runtime_quantity():
  """21 stock episodes replayed at 100 Hz through the real decision path: the 4 with runtime peak >= 2.4 m/s2 must still alert."""
  assert len(REPLAY["episodes"]) == 21
  kept, genuine_kept, genuine = 0, 0, 0
  with OpenpilotPrefix():
    sd = lightning_sd()
    for ep in REPLAY["episodes"]:
      prime(sd)
      sd.steer_sat_sup_start = sd.steer_sat_sup_last = None
      fired, peak = False, 0.0
      for i, (v, k_mdl, k_meas, sat, err, gate) in enumerate(ep["rows"]):
        if not gate:
          continue
        vv = max(v, 0.3)
        des, act = k_mdl * vv * vv, k_meas * vv * vv
        sd.sm.frame = i
        sd.events = Events()
        lac = SimpleNamespace(saturated=bool(sat), steeringAngleDesiredDeg=err, steeringAngleDeg=0.0)
        sd._steer_sat_step(SimpleNamespace(vEgo=v), des, act, lac)
        if steer_sat_verdict(des, act, bool(sat), 1.0):
          peak = max(peak, abs(des))
        fired = fired or EventName.steerSaturated in sd.events.names
      kept += fired
      if peak >= 2.4:
        genuine += 1
        genuine_kept += fired
  assert genuine == 4 and genuine_kept == 4
  assert kept == 9   # per replay window (adjacent episodes share a window; 8 distinct episodes in the offline replay) -- 12 windows go quiet


def test_replay_stock_floor_keeps_all():
  with OpenpilotPrefix():
    sd = SelfdriveD(CP=make_cp("tesla", TESLA))
    n = 0
    for ep in REPLAY["episodes"]:
      sd.sm.frame = 0
      for v, k_mdl, k_meas, sat, _err, gate in ep["rows"]:
        vv = max(v, 0.3)
        if gate and steer_sat_verdict(k_mdl * vv * vv, k_meas * vv * vv, bool(sat), 1.0):
          n += 1
          break
    assert n == 21
