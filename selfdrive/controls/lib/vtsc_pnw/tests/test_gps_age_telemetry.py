"""vtscgpsage2pnw -- VTSCStatus `gpsAge`: how old the GPS fix behind VTSC's map-curve fold is. TELEMETRY ONLY.

VTSC's map fold reads LastGPSPosition from /dev/shm with no freshness check (ces_pnw's ICBM has one). In the SR 99
tunnel the device fix froze for 126 s (drives/2026-09-12/central-oregon-weekend/GPS_TRUCK_VS_COMMA.md), so MTSC can
fold a map curve for a place the car has already left. Before any freshness check is built, the drives have to show
what the age looks like; this field is that measurement. It must change nothing else: the position, the cap, the
state machine and every other VTSCStatus key are the same with and without a fix_ts.

gpsAge = time.monotonic() - LastGPSPosition["fix_ts"] -- the clock and timestamp ces_pnw's icbm_project_position uses
for icbmGpsAge -- rounded to 0.1 s; null when the fold uses no position (no fix, no usable fix_ts, map curves off or
VTSC disabled).
"""
import inspect
import json
import logging
import math
import types
from collections import deque

import pytest

from openpilot.common.params import UnknownKeyName
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw
from openpilot.selfdrive.controls.lib.vtsc_pnw import vtsc_controller as vc

LAT0, LON0 = 47.6, -122.3
V_SET = 31.3
T0 = 1000.0
LATENCY = 0.57          # mapd_configd DEVICE_GPS_FIX_LATENCY_S: fix_ts = arrival - 0.57 s
TUNNEL = (5.0, 131.0)   # 126 s of frozen device GPS, as in the SR 99 tunnel


def _pt(d_m, v):
  return {"latitude": LAT0, "longitude": LON0 + d_m / (111320.0 * math.cos(math.radians(LAT0))), "velocity": v}


CURVE = [_pt(200, 13.0)]                     # one binding map curve that vision cannot see


def _blob(arrival, **extra):
  """A LastGPSPosition exactly as mapd_configd's device branch writes it, for a fix that arrived at `arrival`."""
  b = {"latitude": LAT0, "longitude": LON0, "bearing": 90.0, "speed": 20.0, "src": "device", "ts": arrival,
       "fix_ts": arrival - LATENCY}
  b.update(extra)
  return b


_CLOCK = [T0]


def _t():
  return _CLOCK[0] - T0


def writer_1hz(t):
  """mapd_configd writing a new device fix once a second."""
  return _blob(T0 + math.floor(t))


def tunnel(t):
  """The same writer, silent for TUNNEL: the last pre-tunnel fix stays in /dev/shm."""
  return _blob(T0 + math.floor(TUNNEL[0] - 1e-9)) if TUNNEL[0] <= t < TUNNEL[1] else writer_1hz(t)


def no_fix_ts(gps):
  """The same positions without the fix_ts key."""
  def f(t):
    b = gps(t)
    return {k: v for k, v in b.items() if k != "fix_ts"} if isinstance(b, dict) else b
  return f


class _CP:
  def __init__(self, op_long=True, fp="TESLA_MODEL_S_HW3", brand="tesla"):
    self.carFingerprint, self.brand, self.openpilotLongitudinalControl = fp, brand, op_long


TESLA = {}
LIGHTNING_OPLONG = {"fp": "FORD_F_150_LIGHTNING_MK1", "brand": "ford"}
LIGHTNING_STOCK = {"op_long": False, "fp": "FORD_F_150_LIGHTNING_MK1", "brand": "ford"}


class _Params:
  def __init__(self, mode=lambda t: 2, map_curves=True):
    self.mode, self.map_curves = mode, map_curves

  def get(self, k, return_default=False):
    return str(self.mode(_t())) if k == "CESMode" else None

  def get_bool(self, k):
    return self.map_curves if k == "VtscMapCurves" else False


class _Mem:
  """gps(t) -> the LastGPSPosition value, or an exception instance to raise. Remembers the fix_ts it last handed out."""
  def __init__(self, gps):
    self.gps, self.reads, self.last_fix_ts = gps, 0, None

  def get(self, k, return_default=False):
    if k == "MapTargetVelocities":
      return CURVE
    if k != "LastGPSPosition":
      return None
    self.reads += 1
    v = self.gps(_t())
    if isinstance(v, BaseException):
      raise v
    self.last_fix_ts = v.get("fix_ts") if isinstance(v, dict) else None
    return v

  def put_nonblocking(self, k, v):
    pass


class _NS:
  pass


def _sm():
  m = _NS()
  m.orientationRate, m.velocity, m.position, m.action = _NS(), _NS(), _NS(), _NS()
  m.orientationRate.z = [0.0] * 20
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

  def fix_ts(self):
    return self._match("fix_ts unusable")

  def gps(self):
    return self._match("LastGPSPosition unreadable")

  def errors(self):
    return [r for r in self.records if r.levelno >= logging.WARNING]


@pytest.fixture
def logs():
  h = _Records()
  cloudlog.addHandler(h)
  yield h
  cloudlog.removeHandler(h)


@pytest.fixture(autouse=True)
def clock(monkeypatch):
  _CLOCK[0] = T0
  monkeypatch.setattr(vc, "time", types.SimpleNamespace(monotonic=lambda: _CLOCK[0]))
  return _CLOCK


def _controller(gps, cp=TESLA, **params):
  c = vc.VTSCController(_CP(**cp), params=_Params(**params))
  c.mem_params = _Mem(gps)
  return c


def _drive(gps, ticks=120, cp=TESLA, **params):
  """A fresh controller at exactly 20 Hz from T0. Per tick: (cap, vtscState dict, VTSCStatus payload, position, raw
  age, the fix_ts the mem handed out last)."""
  c = _controller(gps, cp, **params)
  out = []
  for i in range(ticks):
    _CLOCK[0] = T0 + i / 20.0
    cap = c.cap(_sm(), V_SET, V_SET)
    out.append((cap, dict(c.msg), c.overlay_payload(), (c._cur_lat, c._cur_lon, c._cur_bearing), c._tele_gps_age,
                c.mem_params.last_fix_ts))
  return out


def _ages(run):
  return [r[2]["gpsAge"] for r in run]


def _tick(t):
  return int(round(t * 20))


def _without_age(run):
  """Everything the change must leave alone: cap, vtscState, the payload minus gpsAge, and the position."""
  return [(cap, msg, {k: v for k, v in p.items() if k != "gpsAge"}, pos) for cap, msg, p, pos, *_ in run]


# ---------------------------------------------------------------- positive control
def test_positive_control_the_position_drives_the_map_cap():
  """Without this, 'nothing but gpsAge changed' could hold because the fold never used the position at all."""
  run = _drive(writer_1hz)
  assert any(p["curveWin"] == "map" for _, _, p, *_ in run)
  assert min(r[0] for r in run) < V_SET - 1.0
  assert all(r[0] == V_SET for r in _drive(lambda t: None))


# ---------------------------------------------------------------- fresh
def test_a_fresh_fix_reports_its_age_on_every_tick():
  run = _drive(writer_1hz, ticks=200)
  # hand-computed: a fix arriving at T0 has fix_ts T0 - 0.57, and VTSC re-reads at T0 + 1.0
  assert _ages(run)[0] == 0.6            # T0:        0.57 s
  assert _ages(run)[10] == 1.1           # T0 + 0.5:  1.07 s (the held fix ages between the 1 Hz reads)
  assert _ages(run)[19] == 1.5           # T0 + 0.95: 1.52 s
  assert _ages(run)[20] == 0.6           # T0 + 1.0:  the next fix is read
  # every tick: now minus the fix_ts of the fix VTSC last read
  for i, (_, _, p, _, raw, fts) in enumerate(run):
    assert raw == pytest.approx(T0 + i / 20.0 - fts, abs=1e-9)
    assert p["gpsAge"] == round(raw, 1)
  assert 0.6 <= min(_ages(run)) and max(_ages(run)) <= 1.6


def test_the_age_uses_fix_ts_not_ts():
  """ts is the arrival time; fix_ts is when the fix was valid. They differ by 0.57 s on the device fix."""
  run = _drive(lambda t: _blob(T0, ts=T0 + 7.0), ticks=1)
  assert _ages(run) == [0.6]


def test_a_fix_ts_ahead_of_the_clock_is_reported_negative_not_hidden():
  """ces_pnw reports this case too (icbm_project_position: 'a negative age is a fix_ts from a previous boot')."""
  run = _drive(lambda t: _blob(T0 + 5.0 + LATENCY), ticks=1)
  assert _ages(run) == [-5.0]


def test_rounded_to_a_tenth_of_a_second():
  run = _drive(lambda t: _blob(T0 - 3.0 + LATENCY - 0.04), ticks=1)    # age 3.04 s
  assert _ages(run) == [3.0] and run[0][4] == pytest.approx(3.04)


# ---------------------------------------------------------------- stale (the tunnel)
def test_a_frozen_fix_ages_through_the_tunnel_and_recovers_at_the_exit():
  run = _drive(tunnel, ticks=140 * 20)
  age, raw = _ages(run), [r[4] for r in run]
  assert age[_tick(4.95)] <= 1.6                                          # before the tunnel: fresh
  assert age[_tick(35.0)] == 31.6                                         # the last fix arrived at T0 + 4 (31.57 s)
  assert age[_tick(65.0)] == 61.6
  assert age[_tick(130.95)] == 127.5                                      # just before the exit
  assert age[_tick(131.0)] == 0.6                                         # the first read after the exit
  in_tunnel = raw[_tick(TUNNEL[0]):_tick(TUNNEL[1])]
  assert all(b > a for a, b in zip(in_tunnel, in_tunnel[1:], strict=False))    # grows every tick while frozen
  over = [k for k, a in enumerate(raw) if a > 30.0]                       # a 30 s check would fire from T0 + 33.45 ...
  assert over == list(range(_tick(33.45), _tick(131.0)))                  # ... until the exit, and nowhere else


@pytest.mark.parametrize("gps", [writer_1hz, tunnel], ids=["fresh", "tunnel"])
def test_fix_ts_changes_nothing_but_gps_age(gps):
  """The whole point of TELEMETRY ONLY: the same positions with and without fix_ts drive tick-for-tick the same."""
  ticks = 140 * 20 if gps is tunnel else 200
  got, want = _drive(gps, ticks=ticks), _drive(no_fix_ts(gps), ticks=ticks)
  assert _without_age(got) == _without_age(want)
  assert all(a is not None for a in _ages(got)) and all(a is None for a in _ages(want))


@pytest.mark.parametrize("cp", [TESLA, LIGHTNING_OPLONG], ids=["tesla", "lightning-oplong"])
def test_both_op_long_cars_report_it(cp):
  assert _ages(_drive(writer_1hz, ticks=1, cp=cp)) == [0.6]


# ---------------------------------------------------------------- missing
@pytest.mark.parametrize("gps", [
  pytest.param(lambda t: None, id="no fix yet"),
  pytest.param(no_fix_ts(writer_1hz), id="a fix without fix_ts"),
  pytest.param(lambda t: _blob(T0, fix_ts=None), id="fix_ts null"),
])
def test_no_fix_or_no_fix_time_is_null_and_not_logged(logs, gps):
  run = _drive(gps, ticks=60)
  assert all(a is None for a in _ages(run))
  assert logs.errors() == []


@pytest.mark.parametrize("bad", [UnknownKeyName(b"LastGPSPosition"), "{not json"], ids=["UnknownKeyName", "bad JSON"])
def test_an_unreadable_position_is_null(logs, bad):
  run = _drive(lambda t: bad, ticks=20)
  assert all(a is None for a in _ages(run))
  assert len(logs.gps()) == 1 and logs.fix_ts() == []


@pytest.mark.parametrize("after", [
  pytest.param(lambda t: None, id="the fix is lost"),
  pytest.param(no_fix_ts(writer_1hz), id="the position stays but fix_ts goes"),
  pytest.param(lambda t: RuntimeError("x"), id="the position becomes unreadable"),
])
def test_the_age_goes_null_at_the_first_read_after_the_fix_time_is_gone(after):
  run = _drive(lambda t: writer_1hz(t) if t < 3.0 else after(t), ticks=120)
  age = _ages(run)
  assert all(a is not None for a in age[:60])
  assert all(a is None for a in age[60:])                  # the read at T0 + 3.0 drops it; no stale fix_ts survives


# ---------------------------------------------------------------- the fold uses no position
@pytest.mark.parametrize("cp, params", [
  pytest.param(TESLA, dict(mode=lambda t: 0), id="CESMode Off"),
  pytest.param(TESLA, dict(map_curves=False), id="VtscMapCurves off"),
  pytest.param(LIGHTNING_STOCK, dict(), id="Lightning on stock ACC"),
])
def test_null_where_vtsc_folds_no_map_curve(cp, params):
  """VTSC reads LastGPSPosition only for the map fold, which needs op-long, CESMode > 0 and VtscMapCurves. Where it
  does not run, there is no age to report -- the Lightning on stock ACC reports null on every tick (ICBM's own
  icbmGpsAge covers that car)."""
  run = _drive(writer_1hz, ticks=60, cp=cp, **params)
  assert all(a is None for a in _ages(run))


def test_turning_vtsc_off_nulls_the_age_although_the_last_fix_time_is_still_held():
  c = _controller(writer_1hz, mode=lambda t: 2 if t < 3.0 else 0)
  ages = []
  for i in range(100):
    _CLOCK[0] = T0 + i / 20.0
    c.cap(_sm(), V_SET, V_SET)
    ages.append(c.overlay_payload()["gpsAge"])
  assert all(a is not None for a in ages[:60]) and all(a is None for a in ages[60:])
  assert c._gps_fix_ts is not None                       # not refreshed while off, so it must not be reported


# ---------------------------------------------------------------- an unusable fix_ts
BAD_FIX_TS = [
  pytest.param("soon", ValueError, id="a string"),
  pytest.param([T0], TypeError, id="a list"),
  pytest.param(10 ** 400, OverflowError, id="an integer too large for a float"),
  pytest.param(float("nan"), ValueError, id="NaN"),
  pytest.param(float("inf"), ValueError, id="Infinity"),
]


@pytest.mark.parametrize("bad, exc_type", BAD_FIX_TS)
def test_an_unusable_fix_ts_is_logged_nulls_only_the_age_and_keeps_the_position(logs, bad, exc_type):
  got = [r[:5] for r in _drive(lambda t: _blob(T0 + math.floor(t), fix_ts=bad), ticks=60)]
  want = [r[:5] for r in _drive(no_fix_ts(writer_1hz), ticks=60)]
  assert got == want                                         # gpsAge null in both; position, cap, payload identical
  assert all(r[3][0] == LAT0 for r in got)                   # the position really is still used
  lines = logs.fix_ts()
  assert len(lines) == 1 and logs.gps() == []
  assert lines[0].levelno >= logging.ERROR and lines[0].exc_info is not None
  assert lines[0].exc_info[0] is exc_type and f"({exc_type.__name__})" in lines[0].msg
  assert "(1 failure(s)" in lines[0].msg


def test_a_nan_fix_ts_from_json_text_is_logged_and_never_reaches_the_payload(logs):
  """Python's json writes and reads a bare NaN, so a NaN fix_ts can arrive as text too."""
  run = _drive(lambda t: json.dumps(_blob(T0, fix_ts=float("nan"))), ticks=5)
  assert all(a is None for a in _ages(run)) and len(logs.fix_ts()) == 1
  assert "NaN" not in json.dumps(run[-1][2])


def test_fix_ts_log_is_rate_limited_to_once_a_minute(logs):
  c = _controller(lambda t: _blob(T0, fix_ts="soon"))
  for i in range(1201):                                      # 60 s at 20 Hz, plus the tick at exactly +60 s
    _CLOCK[0] = T0 + i / 20.0
    c.cap(_sm(), V_SET, V_SET)
    if i == 1199:
      assert len(logs.fix_ts()) == 1
  lines = logs.fix_ts()
  assert len(lines) == 2
  assert "(1 failure(s)" in lines[0].msg and "(60 failure(s)" in lines[1].msg   # _read_map runs at 1 Hz


@pytest.mark.parametrize("first", ["fix_ts", "gps"])
def test_fix_ts_and_position_failures_log_independently(logs, first):
  bad = {"fix_ts": _blob(T0, fix_ts="soon"), "gps": RuntimeError("unreadable")}
  second = "gps" if first == "fix_ts" else "fix_ts"
  _drive(lambda t: bad[first] if t < 2.0 else bad[second], ticks=60)
  assert len(logs.fix_ts()) == 1 and len(logs.gps()) == 1


# ---------------------------------------------------------------- comparable with ces_pnw, and reaches ces_events
def _ces_read(blob_by_key):
  cls = next(o for o in vars(ces_pnw).values() if inspect.isclass(o) and hasattr(o, "_read_map")
             and hasattr(o, "_event_record"))

  class Stub:
    def __getattr__(self, n):
      return None

  g = Stub()
  g.mem_params = types.SimpleNamespace(get=lambda k, return_default=False: blob_by_key.get(k))
  g._toggles, g._vtsc_tele, g._bearing_hist = {"curves": True}, {}, deque(maxlen=8)
  cls._read_map.__get__(g)()
  return g


@pytest.mark.parametrize("gps, t", [(writer_1hz, 0.95), (tunnel, 65.0)], ids=["fresh", "tunnel"])
def test_the_same_age_ces_pnw_computes_for_icbm(gps, t):
  """Same fix, same instant: VTSC's raw gpsAge equals the age icbm_project_position derives from what ces_pnw read."""
  run = _drive(gps, ticks=int(round(t * 20)) + 1)
  blob = json.dumps(gps(t - (t % 1.0)))                       # the fix VTSC read at its last 1 Hz read
  g = _ces_read({"LastGPSPosition": blob})
  _, _, ces_age, _ = ces_pnw.icbm_project_position(g._cur_lat, g._cur_lon, g._cur_bearing, g._gps_fix_ts, 20.0,
                                                    T0 + t)
  assert run[-1][4] == pytest.approx(ces_age, abs=1e-9) and ces_age > 0.0


def test_gps_age_reaches_the_ces_events_tick():
  """The last mile: ces_pnw lifts VTSCStatus keys by an explicit list; a key missing there reads null forever."""
  run = _drive(tunnel, ticks=65 * 20 + 1)
  g = _ces_read({"VTSCStatus": json.dumps(run[-1][2])})
  assert g._vtsc_tele.get("gpsAge") == 61.6
