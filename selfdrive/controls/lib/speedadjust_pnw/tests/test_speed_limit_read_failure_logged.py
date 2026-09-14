"""silentexc3pnw -- an unreadable MapSpeedLimit must SAY SO (Rule 2) and must fall back exactly as before.

_read_speed_limit() read the posted limit under `except Exception: sl = 0.0` with no log. An unreadable limit is
"unknown": the last valid limit is held for SL_HOLD_S, then the limit-drop cap AND the police cap (both need a posted
limit) stop, on both cars. The fallback is kept; the failure is now logged -- the first at once, then at most one line
per POLICE_READ_ERR_LOG_S, counting the failed reads since the previous line -- with its own state. An unset limit is
not a failure and does not log.
"""
import json
import logging
import types

import pytest

from openpilot.common.params import Params, UnknownKeyName
from openpilot.common.swaglog import cloudlog
import openpilot.selfdrive.controls.lib.speedadjust_pnw.speedadjust_controller as sa

MPH = sa.MPH_TO_MS
V70 = 70 * MPH
ALERT = json.dumps({"police": {"state": "alert", "dist_mi": 0.1, "key": "k1"}})   # ~5 s ahead at 70 mph


class _Clock:
  t = 1000.0

  @classmethod
  def monotonic(cls):
    return cls.t

  @classmethod
  def time(cls):
    return cls.t


def _t():
  return _Clock.t - 1000.0


class _Params:
  def __init__(self, mode):
    self.mode = mode

  def get(self, k, return_default=False):
    return str(self.mode) if k == "AutoSpeedReduce" else None

  def get_bool(self, k):
    return False


def _zone(t):
  """60 mph, then a 45 mph zone from 10 s: a 70 mph driver is trimmed (mode 2)."""
  return str(60 * MPH) if t < 10.0 else str(45 * MPH)


class _Mem:
  """MapSpeedLimit from sl(t) and LocationServices from loc(t): a value, or an exception instance to raise."""
  def __init__(self, sl=_zone, loc=lambda t: "{}"):
    self.sl, self.loc, self.puts = sl, loc, []

  def get(self, k, return_default=False):
    fn = {"MapSpeedLimit": self.sl, "LocationServices": self.loc}.get(k)
    v = fn(_t()) if fn else None
    if isinstance(v, BaseException):
      raise v
    return v

  def put_nonblocking(self, k, v):
    self.puts.append((round(_Clock.t, 3), k, v))


class _Records(logging.Handler):
  def __init__(self):
    super().__init__(level=logging.DEBUG)
    self.records = []

  def emit(self, record):
    self.records.append(record)

  def sl(self):
    return [r for r in self.records if isinstance(r.msg, str) and "MapSpeedLimit unreadable" in r.msg]

  def errors(self):
    return [r for r in self.records if r.levelno >= logging.WARNING]


@pytest.fixture
def logs():
  h = _Records()
  cloudlog.addHandler(h)
  yield h
  cloudlog.removeHandler(h)


@pytest.fixture
def clock(monkeypatch):
  _Clock.t = 1000.0
  monkeypatch.setattr(sa, "time", _Clock)
  return _Clock


def _ctrl(op_long, mode, mem):
  c = sa.SpeedAdjustController(types.SimpleNamespace(openpilotLongitudinalControl=op_long), params=_Params(mode))
  c.mem_params = mem
  return c


def _drive(op_long, mem, mode=2, seconds=60.0):
  """Exactly 20 Hz through the real 1 Hz reader. Per tick: the cap() output and the limit/publish state; plus every put."""
  _Clock.t = 1000.0
  c = _ctrl(op_long, mode, mem)
  out = []
  for i in range(int(seconds * 20)):
    _Clock.t = 1000.0 + i / 20.0
    cap = c.cap(None, V70, V70, V70, True)
    out.append((cap, c._sl, c._pub_active, c._pub_last, c._cap_out, c._restore_ceiling))
  return out, mem.puts


def _slowed(op_long, run, t0, t1):
  ticks, puts = run
  if op_long:
    return min(cap for cap, *_ in ticks[int(t0 * 20):int(t1 * 20)]) < V70 - 1.0
  return any(k == "SpeedAdjustTarget" and "target" in p and p.get("dir") != "inc" and p["target"] < V70 - 1.0
             and t0 <= ts - 1000.0 < t1 for ts, k, p in puts)


CARS = [pytest.param(True, id="op-long"), pytest.param(False, id="stock-ACC")]


@pytest.mark.parametrize("op_long", CARS)
def test_positive_control_a_readable_limit_drop_slows(clock, op_long):
  """Without this, the equalities below could pass because nothing ever slowed."""
  assert _slowed(op_long, _drive(op_long, _Mem()), 12.0, 60.0)
  assert not _slowed(op_long, _drive(op_long, _Mem(sl=lambda t: None)), 0.0, 60.0)


@pytest.mark.parametrize("op_long", CARS)
def test_an_unknown_key_is_logged_and_drives_exactly_like_no_limit(clock, logs, op_long):
  got = _drive(op_long, _Mem(sl=lambda t: UnknownKeyName(b"MapSpeedLimit")))
  lines = logs.sl()
  assert len(lines) == 1
  assert lines[0].levelno >= logging.ERROR and lines[0].exc_info is not None and lines[0].exc_info[0] is UnknownKeyName
  assert "(UnknownKeyName)" in lines[0].msg and "(1 failed read(s)" in lines[0].msg
  assert got == _drive(op_long, _Mem(sl=lambda t: None))


@pytest.mark.parametrize("op_long", CARS)
def test_a_failure_in_the_zone_is_exactly_the_limit_vanishing(clock, logs, op_long):
  """Trimmed in the 45 zone, then the read breaks at 20 s: tick for tick what the limit going unknown at 20 s does --
  held SL_HOLD_S, then the trim releases."""
  got = _drive(op_long, _Mem(sl=lambda t: _zone(t) if t < 20.0 else UnknownKeyName(b"MapSpeedLimit")))
  want = _drive(op_long, _Mem(sl=lambda t: _zone(t) if t < 20.0 else None))
  assert _slowed(op_long, got, 12.0, 20.0)                  # the trim really was engaged before the failure
  assert got == want
  ticks = got[0]
  # the last valid read was at 19 s: held through the failed reads at 20..23 s, unknown from the read at 24 s
  assert all(sl == pytest.approx(45 * MPH) for _, sl, *_ in ticks[20 * 20:24 * 20])
  assert all(sl == 0.0 for _, sl, *_ in ticks[24 * 20:])
  if op_long:
    assert ticks[-1][0] == V70                              # and the slowdown is gone
  assert len(logs.sl()) == 1                                # 40 failed 1 Hz reads, inside the minute


@pytest.mark.parametrize("op_long", CARS)
def test_an_unreadable_limit_also_stops_the_police_slowdown(clock, logs, op_long):
  """The police cap is limit + 5 mph: with no limit there is no police slowdown -- exactly as before."""
  police = dict(loc=lambda t: ALERT)
  assert _slowed(op_long, _drive(op_long, _Mem(sl=lambda t: str(60 * MPH), **police), mode=1, seconds=20.0), 0.0, 20.0)
  got = _drive(op_long, _Mem(sl=lambda t: UnknownKeyName(b"MapSpeedLimit"), **police), mode=1, seconds=20.0)
  assert got == _drive(op_long, _Mem(sl=lambda t: None, **police), mode=1, seconds=20.0)
  assert not _slowed(op_long, got, 0.0, 20.0) and len(logs.sl()) == 1


@pytest.mark.parametrize("bad, exc_type", [
  pytest.param("abc", ValueError, id="non-numeric string"),
  pytest.param(b"\x80\x81", UnicodeDecodeError, id="bad UTF-8 bytes"),
  pytest.param(RuntimeError("code defect"), RuntimeError, id="RuntimeError"),
])
def test_any_read_error_is_caught_and_logged_with_its_type(clock, logs, bad, exc_type):
  c = _ctrl(True, 2, _Mem(sl=lambda t: bad))
  assert c._read_speed_limit() == 0.0
  lines = logs.sl()
  assert len(lines) == 1 and issubclass(lines[0].exc_info[0], exc_type) and f"({lines[0].exc_info[0].__name__})" in lines[0].msg


@pytest.mark.parametrize("raw", [None, "", "nan", "inf", "-5", "0", str(sa.SANE_MAX_SL + 1.0)])
def test_unset_or_rejected_values_read_unknown_without_logging(clock, logs, raw):
  """Not failures: an unset / empty limit is simply unknown, and non-finite / non-positive / garbage-high values are
  rejected by the existing sanity check. None of them raises, so none logs."""
  c = _ctrl(True, 2, _Mem(sl=lambda t: raw))
  assert c._read_speed_limit() == 0.0
  assert logs.errors() == []


def test_a_real_params_store_unset_limit_reads_unknown_and_does_not_log(clock, logs, tmp_path):
  p = Params(str(tmp_path))
  c = _ctrl(True, 2, p)
  assert p.get("MapSpeedLimit") is None and c._read_speed_limit() == 0.0
  p.put("MapSpeedLimit", str(26.8))
  assert c._read_speed_limit() == pytest.approx(26.8)       # the real read path does read a value
  assert logs.errors() == []
  p.put("MapSpeedLimit", "abc")
  c._sl_valid_t = -1e9
  assert c._read_speed_limit() == 0.0 and len(logs.sl()) == 1 and logs.sl()[0].exc_info[0] is ValueError


def test_failure_log_is_rate_limited_to_once_a_minute(clock, logs):
  c = _ctrl(True, 2, _Mem(sl=lambda t: UnknownKeyName(b"MapSpeedLimit")))
  for i in range(1201):                              # 60 s at 20 Hz, plus the tick at exactly +60 s
    _Clock.t = 1000.0 + i / 20.0
    c.cap(None, V70, V70, V70, True)
    if i == 1199:
      assert len(logs.sl()) == 1
  lines = logs.sl()
  assert len(lines) == 2
  assert "(1 failed read(s)" in lines[0].msg and "(60 failed read(s)" in lines[1].msg   # the read runs at 1 Hz


def test_limit_mode_and_police_failures_log_independently(clock, logs):
  class _BadMode(_Params):
    def get(self, k, return_default=False):
      if k == "AutoSpeedReduce":
        raise UnknownKeyName(b"AutoSpeedReduce")
      return None
  c = sa.SpeedAdjustController(types.SimpleNamespace(openpilotLongitudinalControl=True), params=_BadMode(2))
  c.mem_params = _Mem(sl=lambda t: UnknownKeyName(b"MapSpeedLimit"), loc=lambda t: "{not json")
  for i in range(3):
    _Clock.t = 1000.0 + i
    c._read_inputs()
  msgs = [r.msg for r in logs.errors()]
  assert len(logs.sl()) == 1
  assert sum("AutoSpeedReduce unreadable" in m for m in msgs) == 1
  assert sum("LocationServices police input unreadable" in m for m in msgs) == 1
