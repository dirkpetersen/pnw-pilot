"""wheelweight2pnw: steering-wheel weight detector (selfdrive/selfdrived/wheel_weight_pnw.py + SelfdriveD._wheel_weight_step).
Evidence: drives/2026-10-26/tesla-i5-olympia-grand-mound (offset 4.1 deg, straight-road torque +0.16 Nm with the weight on the
left; earlier drives offset about 2.3 deg, torque about -0.2 Nm)."""
import time
from types import SimpleNamespace

import pytest

from cereal import log
from opendbc.car import structs
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_parkgate2pnw import (_make_controller, _publish_controller,
                                                                               _run_publish, _run_steer_log_step)
from openpilot.selfdrive.controls.lib.pnw_vehicle import PnwVehicle
from openpilot.selfdrive.selfdrived import wheel_weight_pnw as M
from openpilot.selfdrive.selfdrived.events import EVENTS, ET, AudibleAlert, EventNamePnw, Events
from openpilot.selfdrive.selfdrived.mads_pnw import has_blocking_event
from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD, WHEEL_WEIGHT_SHOW_S

LIGHTNING = "FORD_F_150_LIGHTNING_MK1"
TESLA = "TESLA_MODEL_S_HW3"
DT = 0.01


def make_cp(brand, fp):
  cp = structs.CarParams.new_message()
  cp.brand, cp.carFingerprint = brand, fp
  return cp


def run(ww, t0, secs, off=4.1, tq=0.16, v=30.0, lat=0.0, pressed=False, valid=True):
  """Feed the detector at 100 Hz. Returns (number of True returns, end time)."""
  n, t = 0, t0
  for _ in range(round(secs / DT)):
    n += ww.update(t, valid, off, v, lat, tq, pressed)
    t += DT
  return n, t


# ---------------------------------------------------------------------------------------------------------------- pure detector
def test_todays_numbers_read_left():
  ww = M.WheelWeight()
  run(ww, 0.0, 20.0)
  assert ww.side == M.LEFT
  assert ww.tele() == {"wwSide": "left", "wwOffSide": "left", "wwOff": 4.1, "wwTq": 0.16}


def test_earlier_drives_read_none():
  ww = M.WheelWeight()
  run(ww, 0.0, 20.0, off=2.3, tq=-0.2)
  assert ww.side == M.NONE


def test_mirror_reads_right():
  ww = M.WheelWeight()
  run(ww, 0.0, 20.0, off=-4.1, tq=-0.16)
  assert ww.side == M.RIGHT


@pytest.mark.parametrize("off,tq", [(4.1, 0.0), (2.5, 0.16), (4.1, -0.16), (-4.1, 0.16)])
def test_both_signs_must_agree(off, tq):
  ww = M.WheelWeight()
  run(ww, 0.0, 20.0, off=off, tq=tq)
  assert ww.side == M.NONE


@pytest.mark.parametrize("kw", [dict(valid=False), dict(off=float("nan")), dict(tq=float("nan"))])
def test_missing_input_is_unknown_and_reported_once(kw):
  ww = M.WheelWeight()
  run(ww, 0.0, 20.0)
  assert ww.side == M.LEFT
  n, t = run(ww, 20.0, 5.0, **kw)
  assert ww.side == M.UNKNOWN and n == 1 and ww.why
  n, _ = run(ww, t, 0.5)   # inputs come back: one report, and the samples kept decide at once
  assert n == 1 and ww.side == M.LEFT


def test_missing_from_the_start_is_reported_once_not_silent():
  ww = M.WheelWeight()
  n, _ = run(ww, 0.0, 5.0, valid=False)
  assert n == 1 and ww.side == M.UNKNOWN and "liveParameters" in ww.why


def test_waits_for_ten_seconds_of_straight_road():
  ww = M.WheelWeight()
  run(ww, 0.0, 9.0)
  assert ww.side == M.UNKNOWN and ww.tq is None
  run(ww, 9.0, 1.5)
  assert ww.side == M.LEFT


@pytest.mark.parametrize("kw", [dict(v=19.0), dict(lat=0.2), dict(lat=-0.2), dict(pressed=True)])
def test_only_straight_road_samples_count(kw):
  ww = M.WheelWeight()
  run(ww, 0.0, 30.0, **kw)
  assert ww.side == M.UNKNOWN and ww.tq is None


def test_median_is_over_the_last_60_seconds_only():
  ww = M.WheelWeight()
  _, t = run(ww, 0.0, 120.0, off=4.1, tq=-0.2)   # old drive data: no weight
  assert ww.side == M.NONE
  _, t = run(ww, t, 25.0, tq=0.16)                 # weight on: 25 of the last 60 s positive, median still negative
  assert ww.side == M.NONE and ww.tq == pytest.approx(-0.2)
  _, t = run(ww, t, 10.0, tq=0.16)                 # 35 of 60 s positive: median flips
  assert ww.side == M.LEFT and ww.tq == pytest.approx(0.16)


def test_median_ignores_a_short_spike():
  ww = M.WheelWeight()
  _, t = run(ww, 0.0, 50.0, off=4.1, tq=0.0)
  _, t = run(ww, t, 5.0, off=4.1, tq=3.0)   # a 5 s push of the wheel
  assert ww.side == M.NONE and ww.tq == pytest.approx(0.0)


def test_hysteresis_holds_left_until_well_back():
  ww = M.WheelWeight()
  _, t = run(ww, 0.0, 20.0)
  assert ww.side == M.LEFT
  _, t = run(ww, t, 120.0, off=2.5, tq=0.02)    # below the "on" thresholds, above the "off" ones
  assert ww.side == M.LEFT
  _, t = run(ww, t, 1.0, off=1.9, tq=0.02)      # offset below 3.0 - 1.0
  assert ww.side == M.NONE
  _, t = run(ww, t, 120.0, off=2.5, tq=0.02)    # same values from "none" do not decide left
  assert ww.side == M.NONE


def test_hysteresis_on_torque():
  ww = M.WheelWeight()
  _, t = run(ww, 0.0, 20.0)
  _, t = run(ww, t, 120.0, off=4.1, tq=0.01)
  assert ww.side == M.LEFT
  _, t = run(ww, t, 120.0, off=4.1, tq=-0.01)
  assert ww.side == M.NONE


def test_no_flicker_near_the_threshold():
  ww = M.WheelWeight()
  sides, t = [], 0.0
  for i in range(60):
    _, t = run(ww, t, 2.0, off=3.0 + (0.3 if i % 2 else -0.3), tq=0.16)
    sides.append(ww.side)
  first_left = sides.index(M.LEFT)
  assert all(s == M.LEFT for s in sides[first_left:])   # once left, the 2.7 deg dips never clear it


# ---------------------------------------------------------------------------------------------------------------- offset side (pricing)
def test_offset_side_follows_the_offset_alone_10_27():
  """10-27: weight right (torque -0.19 Nm) but the offset stayed +4.8 to +5.3 deg: the clamp loses room on the LEFT."""
  ww = M.WheelWeight()
  n, _ = run(ww, 0.0, 0.05, off=5.0, tq=-0.19)
  assert ww.off_side == M.LEFT and n == 1 and ww.tele()["wwOffSide"] == "left"   # at once, no torque samples needed
  run(ww, 0.0, 0.05, off=-4.8, tq=0.16)
  assert ww.off_side == M.RIGHT


def test_offset_side_hysteresis():
  ww = M.WheelWeight()
  for off, want in [(2.9, M.NONE), (3.1, M.LEFT), (2.1, M.LEFT), (1.9, M.NONE), (2.5, M.NONE), (-3.1, M.RIGHT), (-2.1, M.RIGHT),
                    (-1.9, M.NONE)]:
    run(ww, 0.0, 0.05, off=off)
    assert ww.off_side == want, off


def test_offset_side_is_unknown_while_live_parameters_is_invalid_and_reported_once():
  ww = M.WheelWeight()
  run(ww, 0.0, 1.0, off=5.0)
  n, _ = run(ww, 1.0, 5.0, off=5.0, valid=False)
  assert ww.off_side == M.UNKNOWN and n == 1
  n, _ = run(ww, 6.0, 0.05, off=5.0, tq=float("nan"))   # carState bad, offset valid: the offset side is still known
  assert ww.off_side == M.LEFT


# ---------------------------------------------------------------------------------------------------------------- capability + event
def test_capability_is_tesla_only():
  assert PnwVehicle(make_cp("tesla", TESLA)).wheel_weight_detect is True
  assert PnwVehicle(make_cp("ford", LIGHTNING)).wheel_weight_detect is False
  assert PnwVehicle(None).wheel_weight_detect is False


@pytest.mark.parametrize("name,side", [("wheelWeightLeft", "left"), ("wheelWeightRight", "right")])
def test_event_is_a_silent_warning_that_cannot_disengage(name, side):
  ev = EVENTS[getattr(EventNamePnw, name)]
  assert set(ev) == {ET.WARNING}
  a = ev[ET.WARNING]
  assert a.alert_text_1 == f"Wheel weight detected: {side}" and a.audible_alert == AudibleAlert.none
  e = Events()
  e.add(getattr(EventNamePnw, name))
  assert not has_blocking_event(e)
  assert not e.contains(ET.NO_ENTRY) and not e.contains(ET.SOFT_DISABLE) and not e.contains(ET.IMMEDIATE_DISABLE)


# ---------------------------------------------------------------------------------------------------------------- selfdrived wiring
def lp_msg(off=4.1, valid=True):
  m = log.Event.new_message().init('liveParameters')
  m.valid, m.angleOffsetValid, m.angleOffsetDeg = valid, True, off
  return m


def tesla(off=4.1):
  sd = SelfdriveD(CP=make_cp("tesla", TESLA))
  sd.sm.data['liveParameters'] = lp_msg(off)
  sd.sm.valid['liveParameters'] = True
  return sd


def drive(sd, monkeypatch, secs, lat_active=True, tq=0.16, v=30.0, t0=1000.0):
  """Run SelfdriveD._wheel_weight_step at 100 Hz. Returns the frames each side's event was raised."""
  clock = {"t": t0}
  monkeypatch.setattr(time, "monotonic", lambda: clock["t"])
  shown = {"left": 0, "right": 0}
  for _ in range(round(secs / DT)):
    sd.events = Events()
    sd._wheel_weight_step(SimpleNamespace(vEgo=v, steeringTorque=tq, steeringPressed=False), lat_active, 0.0)
    shown["left"] += EventNamePnw.wheelWeightLeft in sd.events.names
    shown["right"] += EventNamePnw.wheelWeightRight in sd.events.names
    clock["t"] += DT
  return shown


def test_selfdrived_shows_left_once_per_drive_and_logs_the_change(monkeypatch):
  logged = []
  import openpilot.selfdrive.selfdrived.selfdrived as SD
  monkeypatch.setattr(SD.cloudlog, "warning", lambda msg, *a, **k: logged.append(msg % a))
  with OpenpilotPrefix():
    sd = tesla()
    assert sd.wheel_weight is not None and sd.ces_pnw.wheel_weight is sd.wheel_weight
    shown = drive(sd, monkeypatch, 30.0)
    assert sd.wheel_weight.side == "left"
    assert shown["left"] == pytest.approx(WHEEL_WEIGHT_SHOW_S / DT, abs=2) and shown["right"] == 0
    assert any("wheel weight now left" in m for m in logged)
    assert drive(sd, monkeypatch, 30.0, t0=2000.0)["left"] == 0   # once per drive, not again


def test_selfdrived_waits_for_steering_to_show(monkeypatch):
  with OpenpilotPrefix():
    sd = tesla()
    assert drive(sd, monkeypatch, 30.0, lat_active=False)["left"] == 0
    assert sd.wheel_weight.side == "left"   # decided while not engaged
    assert drive(sd, monkeypatch, 10.0, t0=2000.0)["left"] > 0


def test_selfdrived_no_weight_shows_nothing(monkeypatch):
  with OpenpilotPrefix():
    sd = tesla(off=2.3)
    assert drive(sd, monkeypatch, 30.0, tq=-0.2) == {"left": 0, "right": 0}
    assert sd.wheel_weight.side == "none"


def test_selfdrived_invalid_live_parameters_is_unknown_and_logged(monkeypatch):
  logged = []
  import openpilot.selfdrive.selfdrived.selfdrived as SD
  monkeypatch.setattr(SD.cloudlog, "warning", lambda msg, *a, **k: logged.append(msg % a))
  with OpenpilotPrefix():
    sd = tesla()
    sd.sm.valid['liveParameters'] = False
    assert drive(sd, monkeypatch, 30.0) == {"left": 0, "right": 0}
    assert sd.wheel_weight.side == "unknown"
    ww_logs = [m for m in logged if "wheelweight2pnw" in m]
    assert len(ww_logs) == 1 and "unknown" in ww_logs[0] and "liveParameters not valid" in ww_logs[0]


def test_selfdrived_lightning_has_no_detector(monkeypatch):
  with OpenpilotPrefix():
    sd = SelfdriveD(CP=make_cp("ford", LIGHTNING))
    assert sd.wheel_weight is None and sd.ces_pnw.wheel_weight is None
    sd.sm.data['liveParameters'] = lp_msg()
    sd.sm.valid['liveParameters'] = True
    assert drive(sd, monkeypatch, 30.0) == {"left": 0, "right": 0}


# ---------------------------------------------------------------------------------------------------------------- ces_events
def _left():
  ww = M.WheelWeight()
  run(ww, 0.0, 20.0)
  return ww


def test_the_ces_tick_record_carries_the_ww_fields(monkeypatch):
  c = _publish_controller(monkeypatch)
  c.wheel_weight = _left()
  caps = _run_publish(monkeypatch, c, 2, g="drive", v_ego=30.0)
  assert caps and all((r["wwSide"], r["wwOffSide"], r["wwOff"], r["wwTq"]) == ("left", "left", 4.1, 0.16) for r in caps)


def test_the_ces_off_steer_record_carries_the_ww_fields(monkeypatch):
  c = _make_controller(monkeypatch, wheel_weight=_left())
  caps = _run_steer_log_step(monkeypatch, c=c)
  assert caps and (caps[0]["wwSide"], caps[0]["wwOffSide"], caps[0]["wwOff"], caps[0]["wwTq"]) == ("left", "left", 4.1, 0.16)


def test_the_ces_records_have_no_ww_fields_without_the_capability(monkeypatch):
  det = ("wwSide", "wwOffSide", "wwOff", "wwTq")   # the detector's own fields (the stage-2 price fields are nulled columns on every car)
  caps = _run_publish(monkeypatch, _publish_controller(monkeypatch), 2, g="drive", v_ego=30.0)
  assert caps and not any(k in r for r in caps for k in det)
  caps = _run_steer_log_step(monkeypatch)
  assert caps and not any(k in caps[0] for k in det)


class _BadCS:
  vEgo, steeringPressed = 30.0, False

  @property
  def steeringTorque(self):
    raise ValueError("bad carState")


def test_selfdrived_a_raising_input_does_not_propagate_and_is_logged_once(monkeypatch):
  """Gemini 2026-10-27: selfdrived is restart_if_crash=False, so the display-only detector must never raise out of its step."""
  excs, warns = [], []
  import openpilot.selfdrive.selfdrived.selfdrived as SD
  monkeypatch.setattr(SD.cloudlog, "exception", lambda msg, *a, **k: excs.append(msg))
  monkeypatch.setattr(SD.cloudlog, "warning", lambda msg, *a, **k: warns.append(msg % a))
  with OpenpilotPrefix():
    sd = tesla()
    drive(sd, monkeypatch, 30.0)
    assert sd.wheel_weight.side == "left"
    clock = {"t": 2000.0}
    monkeypatch.setattr(time, "monotonic", lambda: clock["t"])
    for _ in range(300):
      sd.events = Events()
      sd._wheel_weight_step(_BadCS(), True, 0.0)   # must not raise
      clock["t"] += DT
    assert sd.wheel_weight.side == "unknown" and "ValueError" in sd.wheel_weight.why
    assert len([m for m in excs if "wheelweight2pnw" in m]) == 1
    warns.clear()
    drive(sd, monkeypatch, 15.0, t0=3000.0)           # good input again (old samples are past the 60 s window): recovers, says so
    assert sd.wheel_weight.side == "left" and any("wheel weight now left" in m for m in warns)
