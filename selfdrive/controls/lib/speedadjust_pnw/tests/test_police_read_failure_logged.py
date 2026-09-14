"""policer2pnw -- an unreadable police input must SAY SO (Rule 2) and must fall back exactly as before.

`_read_police()` used to end in `except Exception: pass`: a malformed LocationServices payload read as "no police
report", so police slowdowns silently stopped. The fallback is kept (no report -> no police cap), and it is now
logged: the first failure at once, then at most one line per POLICE_READ_ERR_LOG_S.
"""
import json
import logging
import types

import pytest

from openpilot.common.swaglog import cloudlog
import openpilot.selfdrive.controls.lib.speedadjust_pnw.speedadjust_controller as sa

MPH = sa.MPH_TO_MS
V70 = 70 * MPH
ALERT = json.dumps({"police": {"state": "alert", "dist_mi": 0.1, "key": "k1"}})   # ~5 s ahead at 70 mph
NO_POLICE = "{}"


class _Clock:
  t = 1000.0

  @classmethod
  def monotonic(cls):
    return cls.t

  @classmethod
  def time(cls):
    return cls.t


class _Params:
  """AutoSpeedReduce = 1 (police only)."""
  def get(self, k, return_default=False):
    return "1" if k == "AutoSpeedReduce" else None

  def get_bool(self, k):
    return False


class _Mem:
  """LocationServices comes from `loc(t)`: a payload, or an exception instance to raise."""
  def __init__(self, loc):
    self.loc = loc

  def get(self, k, return_default=False):
    if k == "MapSpeedLimit":
      return str(60 * MPH)
    if k == "LocationServices":
      v = self.loc(_Clock.t - 1000.0)
      if isinstance(v, BaseException):
        raise v
      return v
    return None

  def put_nonblocking(self, k, v):
    pass


class _Records(logging.Handler):
  def __init__(self):
    super().__init__(level=logging.DEBUG)
    self.records = []

  def emit(self, record):
    self.records.append(record)

  def police(self):
    return [r for r in self.records if isinstance(r.msg, str) and "police input unreadable" in r.msg]


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


def _ctrl(loc):
  c = sa.SpeedAdjustController(types.SimpleNamespace(openpilotLongitudinalControl=True), params=_Params())
  c.mem_params = _Mem(loc)
  return c


def _drive(loc, seconds=20.0):
  """20 Hz through the real 1 Hz reader; returns every cap() output."""
  _Clock.t = 1000.0
  c = _ctrl(loc)
  out = []
  while _Clock.t - 1000.0 < seconds:
    _Clock.t += 0.05
    out.append(c.cap(None, V70, V70, V70, True))
  return out


BAD = [
  pytest.param("{not json", id="bad JSON string"),
  pytest.param(b"\x80\x81{", id="bad UTF-8 bytes"),
  pytest.param(TypeError("wrong type"), id="TypeError from the read"),
]


def test_positive_control_a_readable_alert_caps(clock):
  """Without this, the equalities below could pass because the police path never capped at all."""
  out = _drive(lambda t: ALERT)
  assert min(out) < V70 - 1.0
  assert all(v == V70 for v in _drive(lambda t: NO_POLICE))


@pytest.mark.parametrize("bad", BAD)
def test_unreadable_input_is_logged_and_reads_as_no_report(clock, logs, bad):
  c = _ctrl(lambda t: bad)
  assert c._read_police() is None
  lines = logs.police()
  assert len(lines) == 1
  assert lines[0].levelno >= logging.ERROR and lines[0].exc_info is not None
  assert "(1 failed read(s)" in lines[0].msg


@pytest.mark.parametrize("bad", BAD)
def test_the_fallback_matches_a_no_report_read_tick_for_tick(clock, logs, bad):
  """Alert for 8 s, then the input breaks: the car must do exactly what it does when the report simply clears."""
  got = _drive(lambda t: ALERT if t < 8.0 else bad)
  want = _drive(lambda t: ALERT if t < 8.0 else NO_POLICE)
  assert min(got[:160]) < V70 - 1.0                 # the cap really was engaged before the failure
  assert got == want
  assert len(logs.police()) == 1                    # 12 s of failed 1 Hz reads: one line, inside the minute


def test_failure_log_is_rate_limited_to_once_a_minute(clock, logs):
  c = _ctrl(lambda t: "{not json")
  for t in (1000.0, 1030.0, 1059.9):
    _Clock.t = t
    c._read_police()
  assert len(logs.police()) == 1
  _Clock.t = 1060.0                                 # exactly 60 s after the first line
  c._read_police()
  lines = logs.police()
  assert len(lines) == 2 and "(3 failed read(s)" in lines[1].msg


def test_any_other_error_is_also_caught_and_logged_with_its_type(clock, logs):
  """Fable: e.g. UnknownKeyName from a params build mismatch. plannerd does not restart after a crash."""
  c = _ctrl(lambda t: RuntimeError("code defect"))
  assert c._read_police() is None
  lines = logs.police()
  assert len(lines) == 1 and "(RuntimeError)" in lines[0].msg
