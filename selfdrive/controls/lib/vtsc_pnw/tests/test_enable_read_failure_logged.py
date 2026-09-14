"""silentexc2pnw -- a failing VTSC enable/mode read must SAY SO (Rule 2) and must fall back exactly as before.

_read_enabled() ended in a bare `except Exception: self._enabled = False`: any read error switched VTSC off with no
trace, which looks exactly like CESMode=Off. The fallback is kept (VTSC off until a read succeeds) and it is now logged:
the first failure at once, then at most one line per TWISTY_ERR_LOG_S (the twistyr2pnw pattern).
"""
import logging
import types

import pytest

from openpilot.common.params import UnknownKeyName
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.vtsc_pnw import vtsc_controller as vc

V_SET = 31.3
VISION_K = 2.5 / 15.0 ** 2      # a sharp vision curve ~125 m ahead: VTSC brakes for it when enabled


class _CP:
  carFingerprint = "TESLA_MODEL_S_HW3"
  brand = "tesla"
  openpilotLongitudinalControl = True


class _Params:
  """CESMode from `mode(t)`; VtscMapCurves read from `curves(t)`: a bool, or an exception instance to raise."""
  def __init__(self, mode=lambda t: 2, curves=lambda t: True):
    self.mode, self.curves = mode, curves

  def get(self, k, return_default=False):
    return str(self.mode(_t())) if k == "CESMode" else None

  def get_bool(self, k):
    if k != "VtscMapCurves":
      return False
    v = self.curves(_t())
    if isinstance(v, BaseException):
      raise v
    return v


class _Mem:
  def get(self, k, return_default=False):
    return None

  def put_nonblocking(self, k, v):
    self.last = v


class _NS:
  pass


def _sm():
  m = _NS()
  m.orientationRate, m.velocity, m.position, m.action = _NS(), _NS(), _NS(), _NS()
  m.orientationRate.z = [0.0] * 20
  m.orientationRate.z[16] = VISION_K * V_SET        # t = 4 s -> 125 m ahead
  m.orientationRate.t = [i * 0.25 for i in range(20)]
  m.velocity.x = [V_SET] * 20
  m.position.x = [V_SET * i * 0.25 for i in range(20)]
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

  def enable(self):
    return [r for r in self.records if isinstance(r.msg, str) and "enable/mode read FAILED" in r.msg]


_CLOCK = [1000.0]


def _t():
  return _CLOCK[0] - 1000.0


@pytest.fixture
def logs():
  h = _Records()
  cloudlog.addHandler(h)
  yield h
  cloudlog.removeHandler(h)


@pytest.fixture
def clock(monkeypatch):
  _CLOCK[0] = 1000.0
  monkeypatch.setattr(vc, "time", types.SimpleNamespace(monotonic=lambda: _CLOCK[0]))
  return _CLOCK


def _controller(params):
  c = vc.VTSCController(_CP(), params=params)
  c.mem_params = _Mem()
  return c


def _drive(params, ticks=200):
  """A fresh controller at exactly 20 Hz; per tick (cap() output, vtscState dict, VTSCStatus payload)."""
  _CLOCK[0] = 1000.0
  c = _controller(params)
  out = []
  for i in range(ticks):
    _CLOCK[0] = 1000.0 + i / 20.0
    cap = c.cap(_sm(), V_SET, V_SET)
    out.append((cap, dict(c.msg), c.overlay_payload()))
  return out


def test_positive_control_an_enabled_read_brakes_for_the_curve(clock):
  """Without this, the equalities below could pass because VTSC never capped at all."""
  on = _drive(_Params())
  assert min(cap for cap, _, _ in on) < V_SET - 1.0
  assert all(p["enabled"] for _, _, p in on)
  off = _drive(_Params(mode=lambda t: 0))
  assert all(cap == V_SET for cap, _, _ in off)


def test_an_unknown_key_is_logged_and_drives_exactly_like_ces_mode_off(clock, logs):
  """A real exception class from a params_keys.h / params_pyx.so mismatch."""
  got = _drive(_Params(curves=lambda t: UnknownKeyName(b"VtscMapCurves")))
  lines = logs.enable()
  assert len(lines) == 1
  assert lines[0].levelno >= logging.ERROR and lines[0].exc_info is not None
  assert lines[0].exc_info[0] is UnknownKeyName
  assert "(UnknownKeyName)" in lines[0].msg and "(1 failure(s)" in lines[0].msg
  want = _drive(_Params(mode=lambda t: 0))
  assert got == want


def test_a_failure_mid_curve_releases_exactly_like_switching_ces_off(clock, logs):
  """Braking for the curve, then the read breaks at 3 s: tick for tick what CESMode 2 -> 0 at 3 s does."""
  got = _drive(_Params(curves=lambda t: True if t < 3.0 else UnknownKeyName(b"VtscMapCurves")))
  want = _drive(_Params(mode=lambda t: 2 if t < 3.0 else 0))
  assert min(cap for cap, _, _ in got[:60]) < V_SET - 1.0     # VTSC really was braking before the failure
  assert got == want
  assert len(logs.enable()) == 1


def test_the_first_good_read_re_enables(clock, logs):
  got = _drive(_Params(curves=lambda t: UnknownKeyName(b"VtscMapCurves") if t < 2.0 else True))
  want = _drive(_Params(mode=lambda t: 0 if t < 2.0 else 2))
  assert got == want
  assert got[-1][2]["enabled"]


@pytest.mark.parametrize("exc", [TypeError("bad"), KeyError("CESMode"), RuntimeError("code defect")])
def test_any_error_is_caught_logged_with_its_type_and_falls_back(clock, logs, monkeypatch, exc):
  """plannerd does not restart after a crash, so a code defect must degrade to the logged fallback, not escape."""
  real, failing = vc.CES.read_ces_mode, [True]

  def _read(params, **kw):             # silentexc3pnw: VTSC now passes who="VTSC"
    if failing[0]:
      raise exc
    return real(params, **kw)
  monkeypatch.setattr(vc.CES, "read_ces_mode", _read)
  got = _drive(_Params())
  lines = logs.enable()
  assert len(lines) == 1 and f"({type(exc).__name__})" in lines[0].msg
  failing[0] = False
  want = _drive(_Params(mode=lambda t: 0))
  assert got == want


def test_failure_log_is_rate_limited_to_once_a_minute(clock, logs):
  c = _controller(_Params(curves=lambda t: UnknownKeyName(b"VtscMapCurves")))
  for i in range(1201):                              # 60 s at 20 Hz, plus the tick at exactly +60 s
    _CLOCK[0] = 1000.0 + i / 20.0
    c.cap(_sm(), V_SET, V_SET)
    if i == 1199:
      assert len(logs.enable()) == 1                 # still inside the minute
  lines = logs.enable()
  assert len(lines) == 2
  # the read runs at ~1 Hz: 60 failed reads at +1 s .. +60 s since the first line
  assert "(1 failure(s)" in lines[0].msg and "(60 failure(s)" in lines[1].msg
