"""silentexc2pnw -- VTSC's map inputs and its VTSCStatus publish must SAY SO when they fail (Rule 2), and must fall back
exactly as before.

_read_map() read MapTargetVelocities and LastGPSPosition under silent `except Exception:` handlers (no map points / no
position -> map-curve anticipation silently off), and _publish_overlay() ended in `except Exception: pass` (the overlay
and the ces_events VTSC columns silently frozen). The fallbacks are kept; each failure is now logged -- the first at
once, then at most one line per TWISTY_ERR_LOG_S (the twistyr2pnw pattern), with its own state. A missing GPS fix is
NORMAL and is deliberately not logged.
"""
import json
import logging
import math
import types

import pytest

from openpilot.common.params import UnknownKeyName
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.vtsc_pnw import vtsc_controller as vc

LAT0, LON0 = 47.6, -122.3
V_SET = 31.3


def _pt(d_m, v):
  """A map point d_m meters due east of (LAT0, LON0) with target velocity v."""
  return {"latitude": LAT0, "longitude": LON0 + d_m / (111320.0 * math.cos(math.radians(LAT0))), "velocity": v}


CURVE = [_pt(200, 13.0)]                                   # one binding map curve that vision cannot see
FIX = {"latitude": LAT0, "longitude": LON0, "bearing": 90.0}


class _CP:
  carFingerprint = "TESLA_MODEL_S_HW3"
  brand = "tesla"
  openpilotLongitudinalControl = True


class _Params:
  """CESMode=Standard so VTSC runs, VtscMapCurves on so _read_map runs."""
  def get(self, k, return_default=False):
    return "2" if k == "CESMode" else None

  def get_bool(self, k):
    return k == "VtscMapCurves"


def _t():
  return _CLOCK[0] - 1000.0


class _Mem:
  """targets(t) / gps(t) -> a value, or an exception instance to raise. put(t) -> "ok" stores the VTSCStatus
  payload, "drop" loses it silently (the reference for the pre-change `except: pass`), an exception is raised."""
  def __init__(self, targets=lambda t: CURVE, gps=lambda t: FIX, put=lambda t: "ok"):
    self.targets, self.gps, self.put = targets, gps, put
    self.payloads, self.attempts = [], []

  def get(self, k, return_default=False):
    v = self.targets(_t()) if k == "MapTargetVelocities" else self.gps(_t()) if k == "LastGPSPosition" else None
    if isinstance(v, BaseException):
      raise v
    return v

  def put_nonblocking(self, k, v):
    self.attempts.append(_CLOCK[0])
    how = self.put(_t())
    if isinstance(how, BaseException):
      raise how
    if how == "ok":
      self.payloads.append(v)


class _NS:
  pass


def _sm():
  m = _NS()
  m.orientationRate, m.velocity, m.position, m.action = _NS(), _NS(), _NS(), _NS()
  m.orientationRate.z = [0.0] * 20                        # straight ahead for vision: only the map sees the curve
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

  def _match(self, s):
    return [r for r in self.records if isinstance(r.msg, str) and s in r.msg]

  def targets(self):
    return self._match("MapTargetVelocities read FAILED")

  def gps(self):
    return self._match("LastGPSPosition unreadable")

  def overlay(self):
    return self._match("VTSCStatus publish FAILED")

  def errors(self):
    return [r for r in self.records if r.levelno >= logging.WARNING]


_CLOCK = [1000.0]


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


def _controller(mem):
  c = vc.VTSCController(_CP(), params=_Params())
  c.mem_params = mem
  return c


def _drive(mem, ticks=160):
  """A fresh controller at exactly 20 Hz; per tick (cap() output, vtscState dict, VTSCStatus payload, position)."""
  _CLOCK[0] = 1000.0
  c = _controller(mem)
  out = []
  for i in range(ticks):
    _CLOCK[0] = 1000.0 + i / 20.0
    cap = c.cap(_sm(), V_SET, V_SET)
    out.append((cap, dict(c.msg), c.overlay_payload(), (c._cur_lat, c._cur_lon, c._cur_bearing), list(c._map_targets)))
  return out


def _caps(run):
  return [r[0] for r in run]


# ---------------------------------------------------------------- MapTargetVelocities
def test_positive_control_the_map_curve_binds_the_cap(clock):
  """Without this, the equalities below could pass because the map curve never bound at all."""
  with_map = _drive(_Mem())
  assert any(p["curveWin"] == "map" for _, _, p, _, _ in with_map)
  assert min(_caps(with_map)) < V_SET - 1.0
  assert all(cap == V_SET for cap in _caps(_drive(_Mem(targets=lambda t: []))))
  assert all(cap == V_SET for cap in _caps(_drive(_Mem(gps=lambda t: None))))


def test_an_unreadable_map_is_logged_and_drives_exactly_like_no_map_data(clock, logs):
  got = _drive(_Mem(targets=lambda t: UnknownKeyName(b"MapTargetVelocities")))
  lines = logs.targets()
  assert len(lines) == 1
  assert lines[0].levelno >= logging.ERROR and lines[0].exc_info is not None
  assert lines[0].exc_info[0] is UnknownKeyName
  assert "(UnknownKeyName)" in lines[0].msg and "(1 failure(s)" in lines[0].msg
  assert got == _drive(_Mem(targets=lambda t: []))


def test_a_map_read_failure_mid_curve_is_exactly_the_map_data_vanishing(clock, logs):
  got = _drive(_Mem(targets=lambda t: CURVE if t < 3.0 else UnknownKeyName(b"MapTargetVelocities")))
  want = _drive(_Mem(targets=lambda t: CURVE if t < 3.0 else []))
  assert min(_caps(got[:60])) < V_SET - 1.0                # the map curve really was braking before the failure
  assert got == want
  assert len(logs.targets()) == 1


@pytest.mark.parametrize("exc", [TypeError("bad"), RuntimeError("code defect")], ids=lambda e: type(e).__name__)
def test_any_map_read_error_is_caught_logged_with_its_type(clock, logs, exc):
  got = _drive(_Mem(targets=lambda t: exc), ticks=40)
  lines = logs.targets()
  assert len(lines) == 1 and f"({type(exc).__name__})" in lines[0].msg
  assert got == _drive(_Mem(targets=lambda t: []), ticks=40)


# ---------------------------------------------------------------- LastGPSPosition
def test_no_fix_is_normal_and_is_not_logged(clock, logs):
  """mapd_configd writes LastGPSPosition only with a fix, so None is the ordinary state before the first fix."""
  got = _drive(_Mem(gps=lambda t: None))
  assert logs.gps() == [] and logs.errors() == []
  assert all(pos == (None, None, None) for *_, pos, _ in got)
  # ... and it is the same no-position fallback a real read failure produces (the pre-change path for None was a
  # TypeError into that same except)
  assert got == _drive(_Mem(gps=lambda t: UnknownKeyName(b"LastGPSPosition")))


def test_losing_the_fix_mid_curve_is_not_logged_and_releases_like_a_failure(clock, logs):
  got = _drive(_Mem(gps=lambda t: FIX if t < 3.0 else None))
  assert min(_caps(got[:60])) < V_SET - 1.0
  assert logs.gps() == [] and logs.errors() == []
  assert got == _drive(_Mem(gps=lambda t: FIX if t < 3.0 else RuntimeError("x")))


BAD_GPS = [
  pytest.param("{not json", json.JSONDecodeError, id="bad JSON string"),
  pytest.param(b"\x80\x81{", UnicodeDecodeError, id="bad UTF-8 bytes"),
  pytest.param({"longitude": LON0, "bearing": 90.0}, KeyError, id="no latitude"),
  pytest.param({"latitude": "north", "longitude": LON0}, ValueError, id="non-numeric latitude"),
  pytest.param(5, TypeError, id="a bare number"),
  pytest.param(UnknownKeyName(b"LastGPSPosition"), UnknownKeyName, id="UnknownKeyName"),
]


@pytest.mark.parametrize("bad, exc_type", BAD_GPS)
def test_an_unreadable_position_is_logged_and_drives_exactly_like_no_fix(clock, logs, bad, exc_type):
  got = _drive(_Mem(gps=lambda t: bad))
  lines = logs.gps()
  assert len(lines) == 1
  assert lines[0].levelno >= logging.ERROR and lines[0].exc_info is not None
  assert issubclass(lines[0].exc_info[0], exc_type) and f"({lines[0].exc_info[0].__name__})" in lines[0].msg
  assert "(1 failure(s)" in lines[0].msg
  assert got == _drive(_Mem(gps=lambda t: None))


def test_a_good_fix_as_json_text_still_reads(clock, logs):
  got = _drive(_Mem(gps=lambda t: json.dumps(FIX)))
  assert logs.errors() == [] and got == _drive(_Mem())


def test_map_and_gps_failures_log_independently(clock, logs):
  _drive(_Mem(targets=lambda t: UnknownKeyName(b"MapTargetVelocities"), gps=lambda t: "{not json"), ticks=40)
  assert len(logs.targets()) == 1 and len(logs.gps()) == 1


@pytest.mark.parametrize("which", ["targets", "gps"])
def test_map_input_failure_logs_are_rate_limited_to_once_a_minute(clock, logs, which):
  mem = _Mem(**{which: lambda t: RuntimeError("bad")})
  c = _controller(mem)
  for i in range(1201):                                    # 60 s at 20 Hz, plus the tick at exactly +60 s
    _CLOCK[0] = 1000.0 + i / 20.0
    c.cap(_sm(), V_SET, V_SET)
    if i == 1199:
      assert len(getattr(logs, which)()) == 1
  lines = getattr(logs, which)()
  assert len(lines) == 2
  assert "(1 failure(s)" in lines[0].msg and "(60 failure(s)" in lines[1].msg   # _read_map runs at 1 Hz


# ---------------------------------------------------------------- VTSCStatus publish
def test_positive_control_the_overlay_publishes(clock):
  mem = _Mem()
  _drive(mem, ticks=40)
  assert len(mem.payloads) >= 9 and any(p["curveWin"] == "map" for p in mem.payloads)


def test_a_failing_overlay_publish_is_logged_and_is_exactly_a_dropped_publish(clock, logs):
  bad, drop = _Mem(put=lambda t: UnknownKeyName(b"VTSCStatus")), _Mem(put=lambda t: "drop")
  got, want = _drive(bad), _drive(drop)
  assert got == want
  assert bad.attempts == drop.attempts and len(bad.attempts) >= 30     # same ~5 Hz throttle, nothing retried
  lines = logs.overlay()
  assert len(lines) == 1 and lines[0].levelno >= logging.ERROR and lines[0].exc_info[0] is UnknownKeyName
  assert "(UnknownKeyName)" in lines[0].msg and "(1 failure(s)" in lines[0].msg


def test_a_failure_building_the_payload_is_logged_with_its_type(clock, logs, monkeypatch):
  def _raise(self):
    raise KeyError("vTarget")
  mem = _Mem()
  monkeypatch.setattr(vc.VTSCController, "overlay_payload", _raise)
  c = _controller(mem)
  c.cap(_sm(), V_SET, V_SET)
  lines = logs.overlay()
  assert len(lines) == 1 and "(KeyError)" in lines[0].msg and mem.attempts == []


def test_overlay_failure_log_is_rate_limited_to_once_a_minute(clock, logs):
  """Driven through _publish_overlay at exact 0.25 s steps (cap() calls it with the same `now`), so one attempt lands
  at exactly +60 s -- float 20 Hz stamps may miss it, which would hide a `>` for `>=`."""
  mem = _Mem(put=lambda t: RuntimeError("bad"))
  c = _controller(mem)
  for k in range(241):                                      # 60 s at 4 Hz, plus the attempt at exactly +60 s
    c._publish_overlay(1000.0 + k * 0.25)
    if k == 239:
      assert len(logs.overlay()) == 1                       # still inside the minute
  assert len(mem.attempts) == 241                           # every call cleared the 0.2 s throttle and failed
  lines = logs.overlay()
  assert len(lines) == 2
  assert "(1 failure(s)" in lines[0].msg and "(240 failure(s)" in lines[1].msg
