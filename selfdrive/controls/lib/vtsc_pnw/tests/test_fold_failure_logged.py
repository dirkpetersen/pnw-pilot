"""foldlog2pnw -- a failing map-curve fold must SAY SO (Rule 2) and must fall back exactly as before.

_fold_map_curve() wrapped most_binding_map_curve() in a bare `except Exception: return <no map curve>`: if it raised,
map-curve anticipation silently stopped and only the vision cap was left. The fallback is kept, and it is now logged
(the first failure at once, then at most one line per TWISTY_ERR_LOG_S, the twistyr2pnw pattern) and reported per
tick in the VTSCStatus `mapErr` field, which ces_pnw copies into ces_events.
"""
import inspect
import json
import logging
import math
import types
from collections import deque

import pytest

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw
from openpilot.selfdrive.controls.lib.vtsc_pnw import vtsc_controller as vc

LAT0, LON0 = 47.6, -122.3
V_SET = 31.3
FLAT = 0.0               # rad: not a descent, so the twisty trim stays out of these runs
DESCENT = -0.05          # rad, below TWISTY_DESCENT_PITCH
# A sharp VISION curve 125 m ahead (curve-safe ~15 m/s at A_LAT 2.5), close enough that reaching it needs more than
# regen decel -- so a fallback that wrongly flagged it sharp would change the rate limit and show up tick for tick.
VISION_K = 2.5 / 15.0 ** 2


def _pt(d_m, v):
  """A map point d_m meters due east of (LAT0, LON0) with target velocity v."""
  return {"latitude": LAT0, "longitude": LON0 + d_m / (111320.0 * math.cos(math.radians(LAT0))), "velocity": v}


CURVE = [_pt(200, 13.0)]                          # one binding map curve that vision cannot see
# A real malformed payload: one point's velocity is an integer too large for a float (json.loads of a long integer
# literal gives exactly this). The helper skips KeyError/TypeError/ValueError per point, but float() raises
# OverflowError, which ends the whole scan -- so one bad node hides the good curve in front of it.
CURVE_OVERFLOW = CURVE + [{"latitude": LAT0, "longitude": LON0 + 0.004, "velocity": 10 ** 400}]


class _CP:
  carFingerprint = "TESLA_MODEL_S_HW3"
  brand = "tesla"
  openpilotLongitudinalControl = True


class _Params:
  """CESMode=Standard so VTSC runs, VtscMapCurves on so the map fold runs."""
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


def _sm(pitch=FLAT, vision_k=0.0):
  m = _NS()
  m.orientationRate, m.velocity, m.position, m.action = _NS(), _NS(), _NS(), _NS()
  m.orientationRate.z = [0.0] * 20                  # straight ahead for vision by default: only the map sees the curve
  m.orientationRate.z[16] = vision_k * V_SET        # t = 4 s -> 125 m ahead
  m.orientationRate.t = [i * 0.25 for i in range(20)]
  m.velocity.x = [V_SET] * 20
  m.position.x = [V_SET * i * 0.25 for i in range(20)]
  m.action.shouldStop = False
  cc = _NS()
  cc.orientationNED = [0.0, pitch, 0.0]
  return {"modelV2": m, "carControl": cc}


class _Records(logging.Handler):
  def __init__(self):
    super().__init__(level=logging.DEBUG)
    self.records = []

  def emit(self, record):
    self.records.append(record)

  def fold(self):
    return [r for r in self.records if isinstance(r.msg, str) and "map-curve fold FAILED" in r.msg]

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


def _controller(targets):
  c = vc.VTSCController(_CP(), params=_Params())
  c.mem_params = _Mem(targets)
  return c


def _drive(clock, targets, ticks=60, pitch=FLAT, vision_k=0.0):
  """Run a fresh controller at 20 Hz; per tick return (cap() output, vtscState dict, VTSCStatus payload)."""
  c = _controller(targets)
  out = []
  for _ in range(ticks):
    clock[0] += 0.05
    cap = c.cap(_sm(pitch, vision_k), V_SET, V_SET)
    out.append((cap, dict(c.msg), c.overlay_payload()))
  return out


def _no_fold(self, k_apex, d_apex, v_curve, *a, **k):
  """The fold contributing no map curve: vision's picture passes straight through."""
  return k_apex, d_apex, v_curve, False


def _without(payload, *keys):
  return {k: v for k, v in payload.items() if k not in keys}


def test_positive_control_the_fold_binds_the_cap(clock, monkeypatch):
  """Without this, the equalities below could pass because the map curve never bound at all."""
  folded = _drive(clock, CURVE)
  assert any(p["curveWin"] == "map" for _, _, p in folded)
  monkeypatch.setattr(vc.VTSCController, "_fold_map_curve", _no_fold)
  unfolded = _drive(clock, CURVE)
  assert min(cap for cap, _, _ in folded) < min(cap for cap, _, _ in unfolded) - 1.0
  assert all(cap == V_SET for cap, _, _ in unfolded)          # vision alone sees a straight road


@pytest.mark.parametrize("vision_k", [0.0, VISION_K], ids=["vision-straight", "vision-sharp-curve"])
def test_a_malformed_map_payload_is_logged_and_falls_back_tick_for_tick(clock, logs, monkeypatch, vision_k):
  """A real failure, not a stub: the OverflowError point hides the binding curve in front of it."""
  got = _drive(clock, CURVE_OVERFLOW, vision_k=vision_k)
  lines = logs.fold()
  assert len(lines) == 1
  assert lines[0].levelno >= logging.ERROR and lines[0].exc_info is not None
  assert lines[0].exc_info[0] is OverflowError
  assert "(OverflowError)" in lines[0].msg and "(1 failure(s)" in lines[0].msg
  assert all(p["mapErr"] == "OverflowError" for _, _, p in got)

  monkeypatch.setattr(vc.VTSCController, "_fold_map_curve", _no_fold)
  want = _drive(clock, CURVE_OVERFLOW, vision_k=vision_k)
  assert all(p["mapErr"] == "" for _, _, p in want)
  if vision_k:
    assert min(cap for cap, _, _ in want) < V_SET - 1.0     # the vision curve really is braking in this variant
  assert [cap for cap, _, _ in got] == [cap for cap, _, _ in want]
  assert [m for _, m, _ in got] == [m for _, m, _ in want]
  assert [_without(p, "mapErr") for _, _, p in got] == [_without(p, "mapErr") for _, _, p in want]


@pytest.mark.parametrize("vision_k", [0.0, VISION_K], ids=["vision-straight", "vision-sharp-curve"])
def test_the_fallback_drives_exactly_like_no_map_data(clock, logs, vision_k):
  """Against the fold's OWN no-map-curve path (no map data at all), not a stub: every control output is identical.
  Only the map-selection telemetry differs -- mapRaw/mapEff/mapD/mapFlr are written by a fold that ran."""
  got = _drive(clock, CURVE_OVERFLOW, vision_k=vision_k)
  want = _drive(clock, [], vision_k=vision_k)
  assert [cap for cap, _, _ in got] == [cap for cap, _, _ in want]
  tele = ("mapRaw", "mapEff", "mapD", "mapFlr")
  assert [_without(m, *tele) for _, m, _ in got] == [_without(m, *tele) for _, m, _ in want]
  assert [_without(p, *tele, "mapErr") for _, _, p in got] == [_without(p, *tele, "mapErr") for _, _, p in want]


def test_a_non_list_payload_is_logged_as_a_type_error(clock, logs, monkeypatch):
  got = _drive(clock, 5)
  lines = logs.fold()
  assert len(lines) == 1 and lines[0].exc_info[0] is TypeError and "(TypeError)" in lines[0].msg
  monkeypatch.setattr(vc.VTSCController, "_fold_map_curve", _no_fold)
  want = _drive(clock, 5)
  assert [(cap, m) for cap, m, _ in got] == [(cap, m) for cap, m, _ in want]


@pytest.mark.parametrize("exc", [TypeError("bad"), OverflowError("bad"), KeyError("A_DECEL"), RuntimeError("code defect")])
def test_any_error_is_caught_logged_with_its_type_and_falls_back(clock, logs, monkeypatch, exc):
  """Fable (twistyr2pnw): plannerd does not restart after a crash, so a code defect must degrade to the logged
  fallback rather than escape."""
  def _raise(*a, **k):
    raise exc
  monkeypatch.setattr(vc, "most_binding_map_curve", _raise)
  got = _drive(clock, CURVE)
  lines = logs.fold()
  assert len(lines) == 1 and f"({type(exc).__name__})" in lines[0].msg
  assert got[-1][2]["mapErr"] == type(exc).__name__

  monkeypatch.setattr(vc.VTSCController, "_fold_map_curve", _no_fold)
  want = _drive(clock, CURVE)
  assert [(cap, m) for cap, m, _ in got] == [(cap, m) for cap, m, _ in want]


def test_failure_log_is_rate_limited_to_once_a_minute(clock, logs):
  c = _controller(CURVE_OVERFLOW)
  start = clock[0]
  for i in range(1201):                              # 60 s at 20 Hz, plus the tick at exactly +60 s
    clock[0] = start + i / 20.0
    c.cap(_sm(), V_SET, V_SET)
    if i == 1199:
      assert len(logs.fold()) == 1                   # still inside the minute
  lines = logs.fold()
  assert len(lines) == 2
  assert "(1 failure(s)" in lines[0].msg and "(1200 failure(s)" in lines[1].msg


def test_the_fold_and_the_twisty_cap_log_independently(clock, logs):
  """On a descent the same bad payload breaks both; neither log line may suppress the other."""
  _drive(clock, 5, ticks=5, pitch=DESCENT)
  assert len(logs.fold()) == 1 and len(logs.twisty()) == 1


def test_map_err_clears_on_the_first_good_tick(clock):
  c = _controller(CURVE_OVERFLOW)
  for _ in range(5):
    clock[0] += 0.05
    c.cap(_sm(), V_SET, V_SET)
  assert c.overlay_payload()["mapErr"] == "OverflowError"
  c.mem_params.targets = CURVE                       # mapd republishes a clean path
  c._last_read = -1e9                                # the next tick re-reads the map (normally ~1 Hz)
  clock[0] += 0.05
  c.cap(_sm(), V_SET, V_SET)
  p = c.overlay_payload()
  assert p["mapErr"] == "" and p["curveWin"] == "map"


def test_map_err_reaches_the_ces_events_tick(clock):
  """The last mile: ces_pnw lifts VTSCStatus keys by an explicit list; a key missing there reads null forever."""
  c = _controller(CURVE_OVERFLOW)
  for _ in range(3):
    clock[0] += 0.05
    c.cap(_sm(), V_SET, V_SET)
  blob = json.dumps(c.overlay_payload())            # what Params(JSON) stores in /dev/shm

  cls = next(o for o in vars(ces_pnw).values() if inspect.isclass(o) and hasattr(o, "_read_map")
             and hasattr(o, "_event_record"))

  class Stub:
    def __getattr__(self, n):
      return None

  g = Stub()
  g.mem_params = types.SimpleNamespace(get=lambda k, return_default=False: blob if k == "VTSCStatus" else None)
  g._toggles, g._vtsc_tele, g._bearing_hist = {"curves": True}, {}, deque(maxlen=8)
  cls._read_map.__get__(g)()
  assert g._vtsc_tele.get("mapErr") == "OverflowError"
