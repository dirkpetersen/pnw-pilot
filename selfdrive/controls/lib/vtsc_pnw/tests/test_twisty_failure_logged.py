"""twistyr2pnw -- a failing twisty-descent cap must SAY SO (Rule 2) and must fall back exactly as before.

The call used to be wrapped in `except Exception: pass`: if twisty_section_cap() raised, the working cruise was
silently left untrimmed on a winding descent. The fallback is kept (no trim, the rest of cap() runs), and it is now
logged: the first failure at once, then at most one line per TWISTY_ERR_LOG_S.
"""
import logging
import math
import types

import pytest

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.vtsc_pnw import vtsc_controller as vc

LAT0, LON0 = 47.6, -122.3
V_SET = 31.3
DESCENT = -0.05          # rad, below TWISTY_DESCENT_PITCH


def _pt(d_m, v):
  """A map point d_m meters due east of (LAT0, LON0) with target velocity v."""
  return {"latitude": LAT0, "longitude": LON0 + d_m / (111320.0 * math.cos(math.radians(LAT0))), "velocity": v}


TWISTY = [_pt(80, 20.0), _pt(160, 21.0), _pt(240, 22.0)]    # 3 binding curves: a twisty section


class _CP:
  carFingerprint = "TESLA_MODEL_S_HW3"
  brand = "tesla"
  openpilotLongitudinalControl = True


class _Params:
  """CESMode=Standard so VTSC runs, VtscMapCurves on so the map fold and the twisty trim run."""
  def get(self, k, return_default=False):
    return "2" if k == "CESMode" else None

  def get_bool(self, k):
    return k == "VtscMapCurves"


class _Mem:
  def __init__(self, targets):
    self.targets = targets

  def get(self, k, return_default=False):
    if k == "MapTargetVelocities":
      return self.targets
    if k == "LastGPSPosition":
      return {"latitude": LAT0, "longitude": LON0, "bearing": 90.0}
    return None

  def put_nonblocking(self, k, v):
    pass


class _NS:
  pass


def _sm():
  m = _NS()
  m.orientationRate, m.velocity, m.position, m.action = _NS(), _NS(), _NS(), _NS()
  m.orientationRate.z = [0.0] * 20                  # straight ahead for vision: only the map sees the curves
  m.orientationRate.t = [i * 0.25 for i in range(20)]
  m.velocity.x = [V_SET] * 20
  m.position.x = [V_SET * i * 0.25 for i in range(20)]
  m.action.shouldStop = False
  cc = _NS()
  cc.orientationNED = [0.0, DESCENT, 0.0]
  return {"modelV2": m, "carControl": cc}


class _Records(logging.Handler):
  def __init__(self):
    super().__init__(level=logging.DEBUG)
    self.records = []

  def emit(self, record):
    self.records.append(record)

  def twisty(self):
    return [r for r in self.records if isinstance(r.msg, str) and "twisty-descent cap FAILED" in r.msg]


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


def _drive(clock, targets, ticks=40):
  """Run a fresh controller at 20 Hz; return every cap() output and the final vtscState dict."""
  c = vc.VTSCController(_CP(), params=_Params())
  c.mem_params = _Mem(targets)
  out = []
  for _ in range(ticks):
    clock[0] += 0.05
    out.append(c.cap(_sm(), V_SET, V_SET))
  return out, dict(c.msg)


def _no_trim(points, cur_lat, cur_lon, v_cruise, *a, **k):
  return v_cruise


def test_positive_control_the_trim_reaches_the_cap(clock, monkeypatch):
  """Without this, an equality below could pass because the twisty branch never ran at all."""
  trimmed, _ = _drive(clock, TWISTY)
  monkeypatch.setattr(vc, "twisty_section_cap", _no_trim)
  untrimmed, _ = _drive(clock, TWISTY)
  assert trimmed[0] < untrimmed[0] - 1.0


def test_a_malformed_map_payload_is_logged_and_left_untrimmed(clock, logs, monkeypatch):
  """A real failure, not a stub: MapTargetVelocities arrives as a bare number, so iterating it raises TypeError."""
  got, got_msg = _drive(clock, 5)
  lines = logs.twisty()
  assert len(lines) == 1
  assert lines[0].levelno >= logging.ERROR and lines[0].exc_info is not None
  assert lines[0].exc_info[0] is TypeError
  assert "(1 failure(s)" in lines[0].msg

  monkeypatch.setattr(vc, "twisty_section_cap", _no_trim)
  want, want_msg = _drive(clock, 5)
  assert got == want and got_msg == want_msg      # today's fallback: exactly the untrimmed result


@pytest.mark.parametrize("exc", [TypeError("bad"), ValueError("bad"), OverflowError("bad")])
def test_each_caught_error_falls_back_to_no_trim(clock, logs, monkeypatch, exc):
  def _raise(*a, **k):
    raise exc
  monkeypatch.setattr(vc, "twisty_section_cap", _raise)
  got, got_msg = _drive(clock, TWISTY)
  assert len(logs.twisty()) == 1

  monkeypatch.setattr(vc, "twisty_section_cap", _no_trim)
  want, want_msg = _drive(clock, TWISTY)
  assert got == want and got_msg == want_msg


def test_failure_log_is_rate_limited_to_once_a_minute(clock, logs, monkeypatch):
  def _raise(*a, **k):
    raise TypeError("bad")
  monkeypatch.setattr(vc, "twisty_section_cap", _raise)
  c = vc.VTSCController(_CP(), params=_Params())
  c.mem_params = _Mem(TWISTY)
  start = clock[0]
  for i in range(1201):                              # 60 s at 20 Hz, plus the tick at exactly +60 s
    clock[0] = start + i / 20.0
    c.cap(_sm(), V_SET, V_SET)
    if i == 1199:
      assert len(logs.twisty()) == 1                 # still inside the minute
  lines = logs.twisty()
  assert len(lines) == 2
  assert "(1 failure(s)" in lines[0].msg and "(1200 failure(s)" in lines[1].msg


def test_any_other_error_is_also_caught_logged_with_its_type_and_left_untrimmed(clock, logs, monkeypatch):
  """Fable: plannerd does not restart after a crash, so a code defect must degrade to the logged fallback."""
  def _raise(*a, **k):
    raise RuntimeError("code defect")
  monkeypatch.setattr(vc, "twisty_section_cap", _raise)
  c = vc.VTSCController(_CP(), params=_Params())
  c.mem_params = _Mem(TWISTY)
  c.cap(_sm(), V_SET, V_SET)
  lines = logs.twisty()
  assert len(lines) == 1 and "(RuntimeError)" in lines[0].msg
