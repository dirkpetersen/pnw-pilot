"""silentexc3pnw -- unreadable freeway-floor inputs must SAY SO (Rule 2) and must fall back exactly as before.

_read_enabled() read MapSpeedLimit and RoadContext under a silent `except Exception:` -> no limit, not a freeway. With
no freeway floor VTSC may brake a freeway curve below the posted limit (the I-90 over-brake class), on both op-long
cars. The fallback is kept; the failure is now logged -- the first at once, then at most one line per TWISTY_ERR_LOG_S,
counting the failures since the previous line -- with its own state. Unset inputs are not failures and do not log.
"""
import logging
import types

import pytest

from openpilot.common.params import Params, UnknownKeyName
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib import pnw_vehicle as pv
from openpilot.selfdrive.controls.lib.vtsc_pnw import vtsc_controller as vc

V_SET = 31.3                    # 70 mph set
LIMIT = 29.06                   # 65 mph freeway
VISION_K = 2.5 / 15.0 ** 2      # a sharp vision curve: without the floor VTSC brakes far below the limit
CARS = [pytest.param("TESLA_MODEL_S_HW3", "tesla", id="tesla"),
        pytest.param("FORD_F_150_LIGHTNING_MK1", "ford", id="lightning-oplong")]


class _CP:
  def __init__(self, fp, brand):
    self.carFingerprint, self.brand, self.openpilotLongitudinalControl = fp, brand, True


class _Params:
  def __init__(self, map_curves=False):
    self.map_curves = map_curves

  def get(self, k, return_default=False):
    return "2" if k == "CESMode" else None

  def get_bool(self, k):
    return self.map_curves if k == "VtscMapCurves" else False


_CLOCK = [1000.0]


def _t():
  return _CLOCK[0] - 1000.0


class _Mem:
  """sl(t) / ctx(t) / targets(t) -> a value, or an exception instance to raise."""
  def __init__(self, sl=lambda t: str(LIMIT), ctx=lambda t: "freeway", targets=lambda t: []):
    self.fns = {"MapSpeedLimit": sl, "RoadContext": ctx, "MapTargetVelocities": targets}

  def get(self, k, return_default=False):
    fn = self.fns.get(k)
    v = fn(_t()) if fn else None
    if isinstance(v, BaseException):
      raise v
    return v

  def put_nonblocking(self, k, v):
    pass


class _NS:
  pass


def _sm():
  m = _NS()
  m.orientationRate, m.velocity, m.position, m.action = _NS(), _NS(), _NS(), _NS()
  m.orientationRate.z = [0.0] * 20
  m.orientationRate.z[16] = VISION_K * V_SET
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

  def floor(self):
    return [r for r in self.records if isinstance(r.msg, str) and "MapSpeedLimit/RoadContext unreadable" in r.msg]

  def errors(self):
    return [r for r in self.records if r.levelno >= logging.WARNING]


@pytest.fixture
def logs():
  h = _Records()
  cloudlog.addHandler(h)
  yield h
  cloudlog.removeHandler(h)


@pytest.fixture(autouse=True)
def clock(monkeypatch, tmp_path):
  _CLOCK[0] = 1000.0
  monkeypatch.setattr(vc, "time", types.SimpleNamespace(monotonic=lambda: _CLOCK[0]))
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(tmp_path / "absent.json"))   # Lightning: built-in curve defaults
  monkeypatch.setattr(pv, "RAIN_CONFIG_PATH", str(tmp_path / "absent-rain.json"))
  return _CLOCK


def _controller(fp, brand, mem, params=None):
  c = vc.VTSCController(_CP(fp, brand), params=params or _Params())
  c.mem_params = mem
  return c


def _drive(fp, brand, mem, ticks=200, params=None):
  """A fresh controller at exactly 20 Hz; per tick (cap, vtscState dict, VTSCStatus payload, floor inputs, enabled)."""
  _CLOCK[0] = 1000.0
  c = _controller(fp, brand, mem, params)
  out = []
  for i in range(ticks):
    _CLOCK[0] = 1000.0 + i / 20.0
    cap = c.cap(_sm(), V_SET, V_SET)
    out.append((cap, dict(c.msg), c.overlay_payload(), c._speed_limit, c._is_freeway, c._enabled))
  return out


def _none(t):
  return None


def _min_cap(run):
  return min(r[0] for r in run)


@pytest.mark.parametrize("fp, brand", CARS)
def test_positive_control_the_floor_holds_a_freeway_curve_near_the_limit(fp, brand):
  """Without this, the equalities below could pass because the floor never bound at all."""
  floored = _drive(fp, brand, _Mem())
  bare = _drive(fp, brand, _Mem(sl=_none, ctx=_none))
  assert _min_cap(floored) > _min_cap(bare) + 1.0
  assert _min_cap(bare) < LIMIT - 5.0


@pytest.mark.parametrize("fp, brand", CARS)
@pytest.mark.parametrize("which", ["sl", "ctx"])
def test_an_unknown_key_is_logged_and_drives_exactly_like_no_floor_inputs(logs, fp, brand, which):
  bad = {which: lambda t: UnknownKeyName(b"MapSpeedLimit" if which == "sl" else b"RoadContext")}
  got = _drive(fp, brand, _Mem(**bad))
  lines = logs.floor()
  assert len(lines) == 1
  assert lines[0].levelno >= logging.ERROR and lines[0].exc_info is not None and lines[0].exc_info[0] is UnknownKeyName
  assert "(UnknownKeyName)" in lines[0].msg and "(1 failure(s)" in lines[0].msg
  assert got == _drive(fp, brand, _Mem(sl=_none, ctx=_none))     # BOTH inputs dropped, as before, VTSC still enabled
  assert all(r[5] for r in got) and [r for r in logs.errors() if "enable/mode read FAILED" in str(r.msg)] == []


@pytest.mark.parametrize("fp, brand", CARS)
def test_a_failure_mid_curve_is_exactly_the_floor_inputs_vanishing(logs, fp, brand):
  got = _drive(fp, brand, _Mem(sl=lambda t: str(LIMIT) if t < 3.0 else UnknownKeyName(b"MapSpeedLimit")))
  want = _drive(fp, brand, _Mem(sl=lambda t: str(LIMIT) if t < 3.0 else None, ctx=lambda t: "freeway" if t < 3.0 else None))
  assert got == want
  assert _min_cap(got[60:]) < _min_cap(got[:60]) - 1.0       # the floor really was holding, and then it was gone
  assert len(logs.floor()) == 1


@pytest.mark.parametrize("sl, ctx, exc_type", [
  pytest.param("abc", "freeway", ValueError, id="non-numeric limit"),
  pytest.param(str(LIMIT), b"\x80\x81", UnicodeDecodeError, id="RoadContext bytes not UTF-8"),
  pytest.param(RuntimeError("code defect"), "freeway", RuntimeError, id="RuntimeError"),
])
def test_any_read_error_is_caught_and_logged_with_its_type(logs, sl, ctx, exc_type):
  got = _drive("TESLA_MODEL_S_HW3", "tesla", _Mem(sl=lambda t: sl, ctx=lambda t: ctx), ticks=40)
  lines = logs.floor()
  assert len(lines) == 1 and issubclass(lines[0].exc_info[0], exc_type) and f"({lines[0].exc_info[0].__name__})" in lines[0].msg
  assert got == _drive("TESLA_MODEL_S_HW3", "tesla", _Mem(sl=_none, ctx=_none), ticks=40)


@pytest.mark.parametrize("sl, ctx, floored", [
  pytest.param(None, None, False, id="both unset"),
  pytest.param(str(LIMIT), None, False, id="no road context"),
  pytest.param(None, "freeway", False, id="no limit"),
  pytest.param(str(LIMIT), "city", False, id="city"),
  pytest.param(str(LIMIT), b"freeway", True, id="freeway as bytes"),
  pytest.param("0.0", "freeway", False, id="limit 0 (mapd: none)"),
])
def test_unset_or_ordinary_inputs_do_not_log(logs, sl, ctx, floored):
  run = _drive("TESLA_MODEL_S_HW3", "tesla", _Mem(sl=lambda t: sl, ctx=lambda t: ctx), ticks=40)
  assert logs.errors() == []
  assert all((r[4] and r[3] > 0.0) == floored for r in run)


def test_a_real_params_store_unset_inputs_do_not_log(logs, tmp_path):
  p = Params(str(tmp_path))
  assert p.get("MapSpeedLimit") is None and p.get("RoadContext") is None
  c = _controller("TESLA_MODEL_S_HW3", "tesla", p)
  c.cap(_sm(), V_SET, V_SET)
  assert (c._speed_limit, c._is_freeway) == (0.0, False) and logs.errors() == []
  p.put("MapSpeedLimit", str(LIMIT))
  p.put("RoadContext", "freeway")
  _CLOCK[0] += 1.0
  c.cap(_sm(), V_SET, V_SET)
  assert (c._speed_limit, c._is_freeway) == (LIMIT, True) and logs.errors() == []     # the real read path works
  p.put("MapSpeedLimit", "abc")
  _CLOCK[0] += 1.0
  c.cap(_sm(), V_SET, V_SET)
  assert (c._speed_limit, c._is_freeway) == (0.0, False)
  assert len(logs.floor()) == 1 and logs.floor()[0].exc_info[0] is ValueError


def test_failure_log_is_rate_limited_to_once_a_minute(logs):
  c = _controller("TESLA_MODEL_S_HW3", "tesla", _Mem(sl=lambda t: UnknownKeyName(b"MapSpeedLimit")))
  for i in range(1201):                              # 60 s at 20 Hz, plus the tick at exactly +60 s
    _CLOCK[0] = 1000.0 + i / 20.0
    c.cap(_sm(), V_SET, V_SET)
    if i == 1199:
      assert len(logs.floor()) == 1
  lines = logs.floor()
  assert len(lines) == 2
  assert "(1 failure(s)" in lines[0].msg and "(60 failure(s)" in lines[1].msg   # the read runs at ~1 Hz


def test_floor_and_map_read_failures_log_independently(logs):
  mem = _Mem(sl=lambda t: UnknownKeyName(b"MapSpeedLimit"), targets=lambda t: UnknownKeyName(b"MapTargetVelocities"))
  _drive("TESLA_MODEL_S_HW3", "tesla", mem, ticks=40, params=_Params(map_curves=True))
  assert len(logs.floor()) == 1
  assert sum("MapTargetVelocities read FAILED" in str(r.msg) for r in logs.errors()) == 1
