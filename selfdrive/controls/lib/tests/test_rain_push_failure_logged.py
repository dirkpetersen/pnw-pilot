"""silentexc3pnw -- a failing RainMode push must SAY SO (Rule 2) and must fall back exactly as before.

VTSC (_read_enabled, plannerd) and CES (_read_params, selfdrived) each push RainMode into their capability view once a
second, under `except Exception: pass`: on a failure the view keeps its last rain tier, so the wet-weather margin in
VTSC's curve targets / freeway floor and in ICBM's curve targets silently stays whatever it was. The fallback is kept
(the last tier is held); each site now logs -- the first failure at once, then at most one line per minute, counting
the failures since the previous line -- with its own state. An unset RainMode is not a failure and does not log.
"""
import copy
import json
import logging
import types

import pytest

from openpilot.common.params import Params, UnknownKeyName
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib import pnw_vehicle as pv
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_curvelead2pnw import FakeCP, LAT0, LON0, _model, _scene
from openpilot.selfdrive.controls.lib.vtsc_pnw import vtsc_controller as vc

MPH = 0.44704
TESLA, LIGHTNING = "TESLA_MODEL_S_HW3", "FORD_F_150_LIGHTNING_MK1"
UKN = UnknownKeyName(b"RainMode")
_CLOCK = [1000.0]


def _t():
  return _CLOCK[0] - 1000.0


class _P:
  """CESMode 2; RainMode from rain(t): a value, or an exception instance to raise."""
  def __init__(self, rain=lambda t: 0, extra=None):
    self.rain, self.extra = rain, extra or {}

  def get(self, k, return_default=False):
    if k == "RainMode":
      v = self.rain(_t())
      if isinstance(v, BaseException):
        raise v
      return v
    if k == "CESMode":
      return "2"
    return self.extra.get(k)

  def get_bool(self, k):
    return k in ("CESCurves", "CESStops", "CESLowSpeed", "CESLead")


class _Records(logging.Handler):
  def __init__(self):
    super().__init__(level=logging.DEBUG)
    self.records = []

  def emit(self, record):
    self.records.append(record)

  def rain(self, who):
    return [r for r in self.records if isinstance(r.msg, str) and r.msg.startswith(f"{who}: RainMode unreadable")]

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
  ns = types.SimpleNamespace(monotonic=lambda: _CLOCK[0], time=lambda: _CLOCK[0])
  monkeypatch.setattr(vc, "time", ns)
  monkeypatch.setattr(m, "time", ns)
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(tmp_path / "absent.json"))   # built-in curve / rain defaults
  monkeypatch.setattr(pv, "RAIN_CONFIG_PATH", str(tmp_path / "absent-rain.json"))
  return _CLOCK


# ---------------------------------------------------------------- VTSC (plannerd), both op-long cars
V_SET = 31.3
VISION_K = 2.5 / 22.0 ** 2          # a curve that binds ~22 m/s: the rain margin shows in the cap
CARS = [pytest.param(TESLA, "tesla", id="tesla"), pytest.param(LIGHTNING, "ford", id="lightning-oplong")]


class _VMem:
  def get(self, k, return_default=False):
    return None

  def put_nonblocking(self, k, v):
    pass


def _vsm():
  o = types.SimpleNamespace
  z = [0.0] * 20
  z[16] = VISION_K * V_SET
  model = o(orientationRate=o(z=z, t=[i * 0.25 for i in range(20)]), velocity=o(x=[V_SET] * 20),
            position=o(x=[V_SET * i * 0.25 for i in range(20)]), action=o(shouldStop=False))
  return {"modelV2": model, "carControl": o(orientationNED=[0.0, 0.0, 0.0])}


def _vtsc(fp, brand, params):
  c = vc.VTSCController(FakeCP(fp, brand, True), params=params)
  c.mem_params = _VMem()
  return c


def _vdrive(fp, brand, rain, ticks=200):
  _CLOCK[0] = 1000.0
  c = _vtsc(fp, brand, _P(rain))
  out = []
  for i in range(ticks):
    _CLOCK[0] = 1000.0 + i / 20.0
    cap = c.cap(_vsm(), V_SET, V_SET)
    out.append((cap, dict(c.msg), c.overlay_payload(), c.veh._rain_tier, c._enabled))
  return out


@pytest.mark.parametrize("fp, brand", CARS)
def test_vtsc_positive_control_the_rain_tier_reaches_the_cap(fp, brand):
  """Without this, the equalities below could pass because the rain tier never changed anything."""
  dry, wet = _vdrive(fp, brand, lambda t: 0), _vdrive(fp, brand, lambda t: 2)
  assert min(r[0] for r in wet) < min(r[0] for r in dry) - 1.0


@pytest.mark.parametrize("fp, brand", CARS)
def test_vtsc_a_failure_holds_the_last_tier_and_says_so(logs, fp, brand):
  got = _vdrive(fp, brand, lambda t: 2 if t < 3.0 else UKN)
  assert got == _vdrive(fp, brand, lambda t: 2)                       # the Heavy tier is held, tick for tick ...
  assert got != _vdrive(fp, brand, lambda t: 2 if t < 3.0 else 0)     # ... which is NOT what switching it off does
  lines = logs.rain("VTSC")
  assert len(lines) == 1
  assert lines[0].levelno >= logging.ERROR and lines[0].exc_info is not None and lines[0].exc_info[0] is UnknownKeyName
  assert "(UnknownKeyName)" in lines[0].msg and "(1 failure(s)" in lines[0].msg
  assert all(r[4] for r in got) and [r for r in logs.errors() if "enable/mode read FAILED" in str(r.msg)] == []


@pytest.mark.parametrize("fp, brand", CARS)
def test_vtsc_a_failure_from_the_start_holds_tier_none(logs, fp, brand):
  got = _vdrive(fp, brand, lambda t: UKN)
  assert got == _vdrive(fp, brand, lambda t: 0) and len(logs.rain("VTSC")) == 1


def test_vtsc_any_push_error_is_caught_and_logged_with_its_type(logs, monkeypatch):
  c = _vtsc(TESLA, "tesla", _P(lambda t: 2))

  def boom(tier):
    raise RuntimeError("code defect")
  monkeypatch.setattr(c.veh, "set_rain_tier", boom)
  c.cap(_vsm(), V_SET, V_SET)
  lines = logs.rain("VTSC")
  assert len(lines) == 1 and lines[0].exc_info[0] is RuntimeError and "(RuntimeError)" in lines[0].msg
  assert c._enabled


def test_vtsc_failure_log_is_rate_limited_to_once_a_minute(logs):
  c = _vtsc(TESLA, "tesla", _P(lambda t: UKN))
  for i in range(1201):                              # 60 s at 20 Hz, plus the tick at exactly +60 s
    _CLOCK[0] = 1000.0 + i / 20.0
    c.cap(_vsm(), V_SET, V_SET)
    if i == 1199:
      assert len(logs.rain("VTSC")) == 1
  lines = logs.rain("VTSC")
  assert len(lines) == 2 and "(1 failure(s)" in lines[0].msg and "(60 failure(s)" in lines[1].msg


def test_vtsc_rain_and_enable_read_failures_log_independently(logs):
  class _BadCurves(_P):
    def get_bool(self, k):
      if k == "VtscMapCurves":
        raise UnknownKeyName(b"VtscMapCurves")
      return super().get_bool(k)
  c = vc.VTSCController(FakeCP(TESLA, "tesla", True), params=_BadCurves(lambda t: UKN))
  c.mem_params = _VMem()
  c.cap(_vsm(), V_SET, V_SET)
  assert len(logs.rain("VTSC")) == 1
  assert sum("enable/mode read FAILED" in str(r.msg) for r in logs.errors()) == 1


# ---------------------------------------------------------------- CES (selfdrived): ICBM on the Lightning's stock ACC
def _ces_run(rain, fp=LIGHTNING, brand="ford", op_long=False, T=14.0):
  """The REAL CESController.experimental_request at 100 Hz on a curve (the silentexc2pnw CES identity harness scene)."""
  _CLOCK[0] = 1000.0
  st = {"curve_at": 150.0, "v": 26.0, "stock": 60 * MPH}

  class Mem:
    def __init__(self):
      self.puts = []

    def get(self, k, return_default=False):
      if k == "MapTargetVelocities":
        return _scene(st["curve_at"], 180.0, 20.0)
      if k == "LastGPSPosition":
        return json.dumps({"latitude": LAT0, "longitude": LON0, "bearing": 0.0, "src": "car", "ts": _CLOCK[0],
                           "fix_ts": _CLOCK[0] - 0.3})
      if k == "MapSpeedLimit":
        return str(60 * MPH)
      return None

    def put_nonblocking(self, k, v):
      v = copy.deepcopy(v)
      if isinstance(v, dict):
        v.pop("ts", None)
      self.puts.append((round(_CLOCK[0], 3), k, v))

  c = m.CESController(FakeCP(fp, brand, op_long), params=_P(rain, extra={"CESButtonState": "0"}))
  c.mem_params = Mem()
  records, decisions, tiers = [], [], []
  c._event_log_ok = True
  c._append_event = lambda rec: records.append(copy.deepcopy(rec))
  NS = types.SimpleNamespace
  i = 0
  while _CLOCK[0] - 1000.0 < T:
    i += 1
    _CLOCK[0] = 1000.0 + i * 0.01
    v = st["v"]
    st["curve_at"] -= v * 0.01
    orz, vx, px, ts = _model(v, st["curve_at"], 180.0)
    model = NS(orientationRate=NS(z=orz, t=ts), velocity=NS(x=vx), position=NS(x=px), action=NS(shouldStop=False),
               meta=NS(laneChangeState="off"))
    sm = {"radarState": NS(leadOne=NS(status=False, vLead=0.0, dRel=0.0, aLeadK=0.0, vLeadK=0.0)),
          "modelV2": model, "carControl": NS(orientationNED=[0.0, 0.0, 0.0])}
    tgt = next((p for _, k, p in reversed(c.mem_params.puts) if k == "IcbmTarget"), None)
    if tgt and "target" in tgt and i % 30 == 0 and abs(tgt["target"] - st["stock"]) > 0.3:
      st["stock"] += MPH if tgt["target"] > st["stock"] else -MPH
    cs = NS(vEgo=v, aEgo=0.0, gasPressed=False, brakePressed=False, leftBlinker=False, rightBlinker=False,
            vCruise=st["stock"] * 3.6, standstill=False, steeringAngleDeg=0.0, steeringPressed=False,
            leftBlindspot=False, rightBlindspot=False, cruiseState=NS(speed=st["stock"], enabled=True))
    decisions.append(c.experimental_request(cs, sm))
    tiers.append(c._veh._rain_tier)
    st["v"] = max(min(st["v"] + (min(st["stock"], 40.0) - st["v"]) * 0.002, 40.0), 5.0)
  return {"dec": decisions, "puts": c.mem_params.puts, "rec": records, "tier": tiers}


def _targets(run):
  return [p["target"] for _, k, p in run["puts"] if k == "IcbmTarget" and p.get("target") is not None]


def test_ces_positive_control_the_rain_tier_reaches_the_icbm_target():
  dry, wet = _targets(_ces_run(lambda t: 0)), _targets(_ces_run(lambda t: 2))
  assert len(dry) > 20 and len(wet) == len(dry)
  assert min(wet) < min(dry) - 1.0


def test_ces_a_failure_holds_the_last_tier_and_says_so(logs):
  got = _ces_run(lambda t: 2 if t < 2.0 else UKN)
  assert got == _ces_run(lambda t: 2)                                  # decisions, IcbmTarget puts, records, tier
  assert got["puts"] != _ces_run(lambda t: 2 if t < 2.0 else 0)["puts"]
  lines = logs.rain("ces_pnw")
  assert len(lines) == 1
  assert lines[0].levelno >= logging.ERROR and lines[0].exc_info is not None and lines[0].exc_info[0] is UnknownKeyName
  assert "(UnknownKeyName)" in lines[0].msg and "(1 failure(s)" in lines[0].msg


@pytest.mark.parametrize("fp, brand, op_long", [pytest.param(TESLA, "tesla", True, id="tesla"),
                                                pytest.param(LIGHTNING, "ford", False, id="lightning-stock-acc")])
def test_ces_a_failure_from_the_start_holds_tier_none(logs, fp, brand, op_long):
  got = _ces_run(lambda t: UKN, fp, brand, op_long, T=4.0)
  assert got == _ces_run(lambda t: 0, fp, brand, op_long, T=4.0) and len(logs.rain("ces_pnw")) == 1


def test_ces_failure_log_is_rate_limited_to_once_a_minute(logs):
  c = m.CESController(FakeCP(LIGHTNING, "ford", False), params=_P(lambda t: UKN))
  c.mem_params = None
  n = int(1.0 / m.DT_CTRL)
  for s in range(61):                                # _read_params reads on every n-th frame: once per second
    _CLOCK[0] = 1000.0 + s
    c._frame = n * s
    c._read_params()
    if s == 59:
      assert len(logs.rain("ces_pnw")) == 1
  lines = logs.rain("ces_pnw")
  assert len(lines) == 2 and "(1 failure(s)" in lines[0].msg and "(60 failure(s)" in lines[1].msg


# ---------------------------------------------------------------- unset is not a failure (both sites)
def test_unset_rain_mode_is_not_a_failure(logs, tmp_path):
  p = Params(str(tmp_path))
  assert p.get("RainMode") is None and p.get("RainMode", return_default=True) == 0
  v = _vtsc(TESLA, "tesla", p)
  v.cap(_vsm(), V_SET, V_SET)
  c = m.CESController(FakeCP(LIGHTNING, "ford", False), params=p)
  c.mem_params = None
  c._frame = 0
  c._read_params()
  assert v.veh._rain_tier == 0 and c._veh._rain_tier == 0
  p.put("RainMode", 2)
  _CLOCK[0] += 1.0
  v.cap(_vsm(), V_SET, V_SET)
  c._frame = 0
  c._read_params()
  assert v.veh._rain_tier == 2 and c._veh._rain_tier == 2               # the real read path does push a tier
  # a None from a non-Params source is mapped to 0 inside set_rain_tier and never reaches either except
  _vdrive(TESLA, "tesla", lambda t: None, ticks=40)
  assert logs.errors() == []
