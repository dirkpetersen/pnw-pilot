"""rule2fixes2pnw -- a failing apex turn-direction read must SAY SO (Rule 2) and fall back exactly as before.

cap() wrapped apex_turn_direction() in `except Exception: turn_dir = 0`, so a failure silently dropped the direction
(no left-curve factor on the Lightning, telemetry dir ""). The fallback is kept -- this path runs on the Tesla too --
and it is now logged: the first failure at once, then at most one line per TWISTY_ERR_LOG_S with a count.
"""
import logging
import types

import pytest

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.vtsc_pnw import vtsc_controller as vc

V = 29.0
K = 2.5 / (24.6 * 24.6)     # a curve that binds below the set speed (see test_vtsc_controller_penalty)
CARS = [("TESLA_MODEL_S_HW3", "tesla"), ("FORD_F_150_LIGHTNING_MK1", "ford")]


class _CP:
  def __init__(self, fp, brand):
    self.carFingerprint, self.brand, self.openpilotLongitudinalControl = fp, brand, True


class _Params:
  def get(self, k, return_default=False):
    return "2" if k == "CESMode" else None

  def get_bool(self, k):
    return False


class _NS:
  pass


def _sm(sign):
  m = _NS()
  m.orientationRate, m.velocity, m.position, m.action = _NS(), _NS(), _NS(), _NS()
  m.orientationRate.z = [sign * K * V] * 20
  m.orientationRate.t = [i * 0.25 for i in range(20)]
  m.velocity.x = [V] * 20
  m.position.x = [V * i * 0.25 for i in range(20)]
  m.action.shouldStop = False
  cc = _NS()
  cc.orientationNED = [0.0, 0.0, 0.0]
  return {"modelV2": m, "carControl": cc}


class _Records(logging.Handler):
  def __init__(self):
    super().__init__(level=logging.DEBUG)
    self.records = []

  def emit(self, record):
    self.records.append(record)

  def dir_lines(self):
    return [r for r in self.records if isinstance(r.msg, str) and "apex turn direction FAILED" in r.msg]


@pytest.fixture
def logs():
  h = _Records()
  cloudlog.addHandler(h)
  yield h
  cloudlog.removeHandler(h)


@pytest.fixture
def clock(monkeypatch):
  t = [1000.0]
  monkeypatch.setattr(vc, "time", types.SimpleNamespace(monotonic=lambda: t[0]))
  return t


def _drive(clock, car, sign, ticks=20):
  c = vc.VTSCController(_CP(*car), params=_Params())
  c.mem_params = None
  out, dirs = [], []
  for _ in range(ticks):
    clock[0] += 0.05
    out.append(c.cap(_sm(sign), V, V))
    dirs.append(c._tele_dir)
  return out, dict(c.msg), dirs


def _raise(exc):
  def f(*a, **k):
    raise exc
  return f


@pytest.mark.parametrize("sign", [1.0, -1.0])
def test_positive_control_direction_is_really_computed(clock, sign):
  """Without this the fallback equality below could pass because the direction branch never ran."""
  _, _, dirs = _drive(clock, CARS[1], sign)
  assert dirs[-1] in ("L", "R")


@pytest.mark.parametrize("car", CARS)
@pytest.mark.parametrize("sign", [1.0, -1.0])
def test_failure_is_logged_and_falls_back_to_straight(clock, logs, monkeypatch, car, sign):
  monkeypatch.setattr(vc, "apex_turn_direction", _raise(TypeError("bad")))
  got = _drive(clock, car, sign)
  lines = logs.dir_lines()
  assert len(lines) == 1
  assert lines[0].levelno >= logging.ERROR and lines[0].exc_info is not None
  assert lines[0].exc_info[0] is TypeError and "(TypeError)" in lines[0].msg
  assert "(1 failure(s)" in lines[0].msg

  monkeypatch.setattr(vc, "apex_turn_direction", lambda *a, **k: 0)
  want = _drive(clock, car, sign)
  assert got == want                      # the fallback is exactly "direction 0", on both cars
  assert got[2][-1] == ""


def test_failure_log_is_rate_limited_to_once_a_minute(clock, logs, monkeypatch):
  monkeypatch.setattr(vc, "apex_turn_direction", _raise(RuntimeError("code defect")))
  c = vc.VTSCController(_CP(*CARS[0]), params=_Params())
  c.mem_params = None
  start = clock[0]
  for i in range(1201):                   # 60 s at 20 Hz, plus the tick at exactly +60 s
    clock[0] = start + i / 20.0
    c.cap(_sm(1.0), V, V)
    if i == 1199:
      assert len(logs.dir_lines()) == 1
  lines = logs.dir_lines()
  assert len(lines) == 2
  assert "(1 failure(s)" in lines[0].msg and "(1200 failure(s)" in lines[1].msg
