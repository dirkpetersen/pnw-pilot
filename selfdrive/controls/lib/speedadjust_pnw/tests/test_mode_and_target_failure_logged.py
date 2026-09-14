"""silentexc2pnw -- an unreadable AutoSpeedReduce selector and a failing SpeedAdjustTarget publish must SAY SO (Rule 2)
and must fall back exactly as before.

_read_inputs() read the selector under `except Exception: self._mode = 0` (police and limit slowdowns silently Off),
and both SpeedAdjustTarget publishes (the target and the {} clear) ended in `except Exception: pass` (the stock-ACC
executor silently got nothing). The fallbacks are kept; each failure is now logged -- the first at once, then at most one
line per POLICE_READ_ERR_LOG_S (the policer2pnw pattern), with its own state.
"""
import json
import logging
import types

import pytest

from openpilot.common.params import UnknownKeyName
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


def _t():
  return _Clock.t - 1000.0


class _Params:
  """AutoSpeedReduce from `mode(t)`: a value, or an exception instance to raise."""
  def __init__(self, mode=lambda t: "1"):
    self.mode = mode

  def get(self, k, return_default=False):
    if k != "AutoSpeedReduce":
      return None
    v = self.mode(_t())
    if isinstance(v, BaseException):
      raise v
    return v

  def get_bool(self, k):
    return False


class _Mem:
  """LocationServices from `loc(t)`. A SpeedAdjustTarget put consults `target(t, payload)`: "ok" stores it, "drop"
  loses it silently (the reference for the pre-change `except: pass`), an exception instance is raised."""
  def __init__(self, loc=lambda t: ALERT, target=lambda t, p: "ok"):
    self.loc, self.target, self.puts = loc, target, []

  def get(self, k, return_default=False):
    if k == "MapSpeedLimit":
      return str(60 * MPH)
    if k == "LocationServices":
      return self.loc(_t())
    return None

  def put_nonblocking(self, k, v):
    if k == "SpeedAdjustTarget":
      how = self.target(_t(), v)
      if isinstance(how, BaseException):
        raise how
      if how == "drop":
        return
    self.puts.append((round(_Clock.t, 3), k, v))


class _Records(logging.Handler):
  def __init__(self):
    super().__init__(level=logging.DEBUG)
    self.records = []

  def emit(self, record):
    self.records.append(record)

  def _match(self, s):
    return [r for r in self.records if isinstance(r.msg, str) and s in r.msg]

  def mode(self):
    return self._match("AutoSpeedReduce unreadable")

  def target(self):
    return self._match("SpeedAdjustTarget publish FAILED")

  def clear(self):
    return self._match("SpeedAdjustTarget clear FAILED")


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


def _ctrl(op_long, params, mem):
  c = sa.SpeedAdjustController(types.SimpleNamespace(openpilotLongitudinalControl=op_long), params=params)
  c.mem_params = mem
  return c


def _drive(op_long, params, mem, seconds=60.0):
  """Exactly 20 Hz through the real 1 Hz reader. Per tick: the cap() output and the publish state; plus every put."""
  _Clock.t = 1000.0
  c = _ctrl(op_long, params, mem)
  out = []
  for i in range(int(seconds * 20)):
    _Clock.t = 1000.0 + i / 20.0
    cap = c.cap(None, V70, V70, V70, True)
    out.append((cap, c._mode, c._pub_active, c._pub_last, c._cap_out, c._restore_ceiling))
  return out, mem.puts


def _police_then_clear(t):
  return ALERT if t < 8.0 else NO_POLICE


# ---------------------------------------------------------------- the AutoSpeedReduce read
@pytest.mark.parametrize("op_long", [True, False], ids=["op-long", "stock-ACC"])
def test_positive_control_a_readable_mode_slows_for_police(clock, op_long):
  """Without this, the equalities below could pass because nothing ever slowed."""
  on, puts = _drive(op_long, _Params(), _Mem(), seconds=20.0)
  if op_long:
    assert min(cap for cap, *_ in on) < V70 - 1.0
  else:
    assert any(k == "SpeedAdjustTarget" and p.get("target", V70) < V70 - 1.0 for _, k, p in puts)
  off, puts = _drive(op_long, _Params(mode=lambda t: "0"), _Mem(), seconds=20.0)
  assert all(cap == V70 for cap, *_ in off) and not any(k == "SpeedAdjustTarget" for _, k, _ in puts)


@pytest.mark.parametrize("op_long", [True, False], ids=["op-long", "stock-ACC"])
def test_an_unknown_key_is_logged_and_drives_exactly_like_mode_off(clock, logs, op_long):
  got = _drive(op_long, _Params(mode=lambda t: UnknownKeyName(b"AutoSpeedReduce")), _Mem(), seconds=20.0)
  lines = logs.mode()
  assert len(lines) == 1
  assert lines[0].levelno >= logging.ERROR and lines[0].exc_info is not None
  assert lines[0].exc_info[0] is UnknownKeyName
  assert "(UnknownKeyName)" in lines[0].msg and "(1 failed read(s)" in lines[0].msg
  want = _drive(op_long, _Params(mode=lambda t: "0"), _Mem(), seconds=20.0)
  assert got == want


@pytest.mark.parametrize("op_long", [True, False], ids=["op-long", "stock-ACC"])
def test_a_failure_mid_slowdown_is_exactly_switching_the_mode_off(clock, logs, op_long):
  """Slowing for police, then the selector read breaks at 6 s: tick for tick what AutoSpeedReduce 1 -> 0 at 6 s does,
  including every SpeedAdjustStatus / SpeedAdjustTarget publish."""
  got = _drive(op_long, _Params(mode=lambda t: "1" if t < 6.0 else UnknownKeyName(b"AutoSpeedReduce")), _Mem())
  want = _drive(op_long, _Params(mode=lambda t: "1" if t < 6.0 else "0"), _Mem())
  if op_long:
    assert min(cap for cap, *_ in got[0][:120]) < V70 - 1.0          # the cap really was engaged before the failure
  else:
    assert any(k == "SpeedAdjustTarget" and "target" in p for _, k, p in got[1])
  assert got == want
  assert len(logs.mode()) == 1                                      # 54 failed 1 Hz reads, inside the minute


@pytest.mark.parametrize("bad", [ValueError("invalid literal"), RuntimeError("code defect")], ids=lambda e: type(e).__name__)
def test_any_mode_read_error_is_caught_and_logged_with_its_type(clock, logs, bad):
  got = _drive(True, _Params(mode=lambda t: bad), _Mem(), seconds=5.0)
  lines = logs.mode()
  assert len(lines) == 1 and f"({type(bad).__name__})" in lines[0].msg
  assert got == _drive(True, _Params(mode=lambda t: "0"), _Mem(), seconds=5.0)


def test_a_malformed_selector_string_is_logged(clock, logs):
  """int("abc") in this module (a real Params returns the default for a malformed INT and warns itself)."""
  c = _ctrl(True, _Params(mode=lambda t: "abc"), _Mem())
  c._read_inputs()
  assert c._mode == 0
  lines = logs.mode()
  assert len(lines) == 1 and lines[0].exc_info[0] is ValueError and "(ValueError)" in lines[0].msg


def test_mode_read_failure_log_is_rate_limited_to_once_a_minute(clock, logs):
  c = _ctrl(True, _Params(mode=lambda t: UnknownKeyName(b"AutoSpeedReduce")), _Mem())
  for i in range(1201):                              # 60 s at 20 Hz, plus the tick at exactly +60 s
    _Clock.t = 1000.0 + i / 20.0
    c.cap(None, V70, V70, V70, True)
    if i == 1199:
      assert len(logs.mode()) == 1
  lines = logs.mode()
  assert len(lines) == 2
  assert "(1 failed read(s)" in lines[0].msg and "(60 failed read(s)" in lines[1].msg   # the read runs at 1 Hz


# ---------------------------------------------------------------- the SpeedAdjustTarget publishes
def test_positive_control_the_stock_acc_brain_publishes_a_target_and_a_clear(clock):
  _, puts = _drive(False, _Params(), _Mem(loc=_police_then_clear))
  tp = [p for _, k, p in puts if k == "SpeedAdjustTarget"]
  assert any("target" in p and p.get("dir") != "inc" for p in tp)      # the police slowdown
  assert any(p.get("dir") == "inc" for p in tp)                        # the bounded restore after it
  assert {} in tp                                                      # and the clear when that window ends


def _only_status(puts):
  return [x for x in puts if x[1] != "SpeedAdjustTarget"]


@pytest.mark.parametrize("which", ["target", "clear", "both"])
def test_a_failing_publish_is_logged_and_is_exactly_a_dropped_publish(clock, logs, which):
  """The pre-change `except: pass` behaved as if the put went nowhere: same cap() outputs, same publish state, same
  SpeedAdjustStatus stream, tick for tick."""
  def fail(t, p):
    hit = which == "both" or (which == "clear") == (p == {})
    return UnknownKeyName(b"SpeedAdjustTarget") if hit else "ok"

  def drop(t, p):
    hit = which == "both" or (which == "clear") == (p == {})
    return "drop" if hit else "ok"
  got_ticks, got_puts = _drive(False, _Params(), _Mem(loc=_police_then_clear, target=fail))
  want_ticks, want_puts = _drive(False, _Params(), _Mem(loc=_police_then_clear, target=drop))
  assert got_ticks == want_ticks
  assert got_puts == want_puts                       # includes the publishes that did NOT fail
  assert len(_only_status(got_puts)) > 100           # the status stream really ran
  if which in ("target", "both"):
    lines = logs.target()
    assert len(lines) == 1 and lines[0].levelno >= logging.ERROR and lines[0].exc_info[0] is UnknownKeyName
    assert "(UnknownKeyName)" in lines[0].msg and "(1 failure(s)" in lines[0].msg
  else:
    assert logs.target() == []
  if which in ("clear", "both"):
    lines = logs.clear()
    assert len(lines) == 1 and lines[0].levelno >= logging.ERROR and lines[0].exc_info[0] is UnknownKeyName
    assert "(UnknownKeyName)" in lines[0].msg and "(1 failure(s)" in lines[0].msg
  else:
    assert logs.clear() == []


def test_the_op_long_car_never_touches_the_target_channel(clock, logs):
  """The publish is a no-op on op-long: a raising put there must not even be reached."""
  got = _drive(True, _Params(), _Mem(loc=_police_then_clear, target=lambda t, p: RuntimeError("never")))
  assert logs.target() == [] and logs.clear() == []
  assert got == _drive(True, _Params(), _Mem(loc=_police_then_clear))


@pytest.mark.parametrize("exc", [RuntimeError("code defect"), TypeError("bad")], ids=lambda e: type(e).__name__)
def test_any_put_error_is_caught_and_logged_with_its_type(clock, logs, exc):
  c = _ctrl(False, _Params(), _Mem(target=lambda t, p: exc))
  c._publish_target(20.0, 25.0)
  c._publish_target(None)
  assert len(logs.target()) == 1 and f"({type(exc).__name__})" in logs.target()[0].msg
  assert len(logs.clear()) == 1 and f"({type(exc).__name__})" in logs.clear()[0].msg


def test_a_non_numeric_target_is_logged(clock, logs):
  c = _ctrl(False, _Params(), _Mem())
  c._publish_target("x", 25.0)
  lines = logs.target()
  assert len(lines) == 1 and lines[0].exc_info[0] is ValueError
  assert not any(k == "SpeedAdjustTarget" for _, k, _ in c.mem_params.puts)


def test_target_publish_failure_log_is_rate_limited_to_once_a_minute(clock, logs):
  c = _ctrl(False, _Params(), _Mem(target=lambda t, p: UnknownKeyName(b"SpeedAdjustTarget")))
  for i in range(1201):
    _Clock.t = 1000.0 + i / 20.0
    c._publish_target(20.0, 25.0)                    # throttled to one attempt per PUB_THROTTLE_S (0.25 s)
    if i == 1199:
      assert len(logs.target()) == 1
  lines = logs.target()
  assert len(lines) == 2
  assert "(1 failure(s)" in lines[0].msg and "(240 failure(s)" in lines[1].msg


def test_clear_failure_log_is_rate_limited_to_once_a_minute(clock, logs):
  c = _ctrl(False, _Params(), _Mem(target=lambda t, p: UnknownKeyName(b"SpeedAdjustTarget") if p == {} else "ok"))
  for i in range(1201):
    _Clock.t = 1000.0 + i / 20.0
    c._publish_target(20.0, 25.0)                    # a real publish every 0.25 s ...
    c._publish_target(None)                          # ... and a failing clear right behind each one
    if i == 1199:
      assert len(logs.clear()) == 1
  lines = logs.clear()
  assert len(lines) == 2 and logs.target() == []
  assert "(1 failure(s)" in lines[0].msg and "(240 failure(s)" in lines[1].msg
