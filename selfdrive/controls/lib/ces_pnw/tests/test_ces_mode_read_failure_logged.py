"""silentexc3pnw -- an unreadable CES master selector must SAY SO (Rule 2) and must fall back exactly as before.

read_ces_mode() (ces_pnw_constants) read CESMode under `except Exception: mode = 0` and the legacy
ConditionalExperimentalSwitching bool under `except Exception: pass`. Its callers are CES (selfdrived: CES decisions and
the Lightning's ICBM curve slowdowns), VTSC (plannerd: op-long curve slowdowns) and the CES overlay (ui: the overlay and
its NO-SIGNAL dead-man). An unreadable CESMode therefore switched all of that off with no trace. The fallbacks are kept
(CESMode -> 0, then the legacy bool still applies; legacy -> skipped); each failure is now logged -- the first at once,
then at most one line per CES_MODE_READ_ERR_LOG_S, counting the failures since the previous line -- with its own state
per caller and per read. An unset param is not a failure and does not log.

cesmodehold2pnw: that fallback is now reached only after the caller has held its last good mode for CES_MODE_HOLD_S
(10 s) -- the tests below therefore read the fallback out of a caller with no good read behind it, or past the window.
The hold itself lives in test_ces_mode_hold.py.
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
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw_constants as C
from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_curvelead2pnw import FakeCP, LAT0, LON0, _model, _scene
from openpilot.selfdrive.controls.lib.vtsc_pnw import vtsc_controller as vc

MPH = 0.44704
LIGHTNING = "FORD_F_150_LIGHTNING_MK1"
TESLA = "TESLA_MODEL_S_HW3"


class _Records(logging.Handler):
  def __init__(self):
    super().__init__(level=logging.DEBUG)
    self.records = []

  def emit(self, record):
    self.records.append(record)

  def mode(self, who=None, key="CESMode"):
    head = f"read_ces_mode ({who}): {key} unreadable" if who else f": {key} unreadable"
    return [r for r in self.records if isinstance(r.msg, str) and r.msg.startswith("read_ces_mode") and head in r.msg]

  def errors(self):
    return [r for r in self.records if r.levelno >= logging.WARNING]


@pytest.fixture
def logs():
  h = _Records()
  cloudlog.addHandler(h)
  yield h
  cloudlog.removeHandler(h)


@pytest.fixture(autouse=True)
def fresh_state(monkeypatch, tmp_path):
  """Module-level log state: each test starts with no failure history. Lightnings read the built-in curve defaults."""
  monkeypatch.setattr(C, "_ces_mode_read_err", {})
  monkeypatch.setattr(C, "_ces_mode_hold_st", {})   # cesmodehold2pnw: no last good mode carried in from another test
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(tmp_path / "absent.json"))
  monkeypatch.setattr(pv, "RAIN_CONFIG_PATH", str(tmp_path / "absent-rain.json"))


@pytest.fixture
def clock(monkeypatch):
  t = [1000.0]
  ns = types.SimpleNamespace(monotonic=lambda: t[0], time=lambda: t[0])
  monkeypatch.setattr(C, "time", ns)
  monkeypatch.setattr(vc, "time", ns)
  monkeypatch.setattr(m, "time", ns)
  return t


def _t0(clock):
  return clock[0] - 1000.0


class _P:
  """CESMode from mode(t) and the legacy bool from legacy(t): a value, or an exception instance to raise."""
  def __init__(self, clock, mode=lambda t: 2, legacy=lambda t: False, extra=None):
    self.clock, self.mode, self.legacy, self.extra = clock, mode, legacy, extra or {}

  def get(self, k, return_default=False):
    if k == "CESMode":
      v = self.mode(_t0(self.clock))
      if isinstance(v, BaseException):
        raise v
      return None if v is None else str(v)
    return self.extra.get(k)

  def get_bool(self, k):
    if k == "ConditionalExperimentalSwitching":
      v = self.legacy(_t0(self.clock))
      if isinstance(v, BaseException):
        raise v
      return v
    return k in ("CESCurves", "CESStops", "CESLowSpeed", "CESLead", "VtscMapCurves")


UKN = UnknownKeyName(b"CESMode")
UKN_LEGACY = UnknownKeyName(b"ConditionalExperimentalSwitching")


# ---------------------------------------------------------------- read_ces_mode itself
def test_positive_control_modes_and_the_legacy_bool_read_as_before(clock, logs):
  for v, want in ((0, 0), (1, 1), (2, 2), (None, 0)):
    assert C.read_ces_mode(_P(clock, mode=lambda t, v=v: v), who="VTSC") == want
  assert C.read_ces_mode(_P(clock, mode=lambda t: 0, legacy=lambda t: True), who="VTSC") == 2
  assert C.read_ces_mode(_P(clock, mode=lambda t: 1, legacy=lambda t: True), who="VTSC") == 1
  assert logs.errors() == []


def test_unset_params_read_their_defaults_and_do_not_log(clock, logs, tmp_path):
  """A REAL Params on an empty store: CESMode reads its "0" default and the legacy bool False -- no exception, no log."""
  p = Params(str(tmp_path))
  assert p.get("CESMode") is None                                # really unset
  assert C.read_ces_mode(p, who="CES") == 0
  p.put("CESMode", 2)
  assert C.read_ces_mode(p, who="CES") == 2                      # ... and the real Params path does read a value
  assert logs.errors() == []


def test_an_unknown_key_is_logged_and_reads_off(clock, logs):
  assert C.read_ces_mode(_P(clock, mode=lambda t: UKN), who="VTSC") == 0
  lines = logs.mode("VTSC")
  assert len(lines) == 1
  assert lines[0].levelno >= logging.ERROR and lines[0].exc_info is not None and lines[0].exc_info[0] is UnknownKeyName
  assert "(UnknownKeyName)" in lines[0].msg and "(1 failure(s)" in lines[0].msg
  assert "VTSC keeps its last good mode for up to 10 s" in lines[0].msg   # cesmodehold2pnw
  assert logs.mode(key="ConditionalExperimentalSwitching") == []


def test_the_legacy_bool_is_not_consulted_after_a_cesmode_failure(clock, logs):
  """cesmodehold2pnw Q2: it used to be -- an unreadable CESMode was 0, and 0 + the bool set meant Standard. The bool
  cannot tell Light from Standard (toggles.py writes it as `CESMode > 0`), so it must not stand in for a failed read:
  a Light driver would silently get the Standard tune. It is not even READ."""
  class _Counting(_P):
    reads = 0

    def get_bool(self, k):
      if k == "ConditionalExperimentalSwitching":
        _Counting.reads += 1
      return super().get_bool(k)

  assert _Counting(clock, mode=lambda t: 0, legacy=lambda t: True).get_bool("ConditionalExperimentalSwitching") is True
  _Counting.reads = 0                                   # positive control: this stub does report a set legacy bool
  assert C.read_ces_mode(_Counting(clock, mode=lambda t: UKN, legacy=lambda t: True), who="CES") == 0
  assert _Counting.reads == 0                           # ... and read_ces_mode never asked for it
  assert len(logs.mode("CES")) == 1
  assert "NOT consulted on a failed read" in logs.mode("CES")[0].msg


def test_the_legacy_bool_still_migrates_a_device_that_only_has_it(clock, logs, tmp_path):
  """The migration itself is untouched: on a REAL Params, an UNSET CESMode reads its "0" default -- a genuine 0 -- so
  an old device carrying only the bool still comes up Standard."""
  p = Params(str(tmp_path))
  p.put_bool("ConditionalExperimentalSwitching", True)
  assert p.get("CESMode") is None                       # really unset (it reads the "0" default, it does not raise)
  assert C.read_ces_mode(p, who="CES") == 2             # ... and the legacy bool still migrates it to Standard
  p.put("CESMode", 1)                                   # once the driver picks a mode, CESMode wins outright
  assert C.read_ces_mode(p, who="CES") == 1
  assert logs.errors() == []


def test_an_unreadable_legacy_bool_is_logged_and_the_mode_stays_off(clock, logs):
  assert C.read_ces_mode(_P(clock, mode=lambda t: 0, legacy=lambda t: UKN_LEGACY), who="CES overlay") == 0
  lines = logs.mode("CES overlay", "ConditionalExperimentalSwitching")
  assert len(lines) == 1 and lines[0].levelno >= logging.ERROR and lines[0].exc_info[0] is UnknownKeyName
  assert "(UnknownKeyName)" in lines[0].msg and "(1 failure(s)" in lines[0].msg
  assert logs.mode(key="CESMode") == []
  # a non-Off CESMode never reads the legacy bool, so a broken legacy key cannot log there
  assert C.read_ces_mode(_P(clock, mode=lambda t: 1, legacy=lambda t: UKN_LEGACY), who="VTSC") == 1
  assert len(logs.mode(key="ConditionalExperimentalSwitching")) == 1 and logs.mode(key="CESMode") == []


@pytest.mark.parametrize("bad, exc_type", [("abc", ValueError), (TypeError("bad"), TypeError),
                                           (RuntimeError("code defect"), RuntimeError)], ids=["abc", "TypeError", "RuntimeError"])
def test_any_cesmode_error_is_caught_and_logged_with_its_type(clock, logs, bad, exc_type):
  assert C.read_ces_mode(_P(clock, mode=lambda t: bad), who="VTSC") == 0
  lines = logs.mode("VTSC")
  assert len(lines) == 1 and lines[0].exc_info[0] is exc_type and f"({exc_type.__name__})" in lines[0].msg


@pytest.mark.parametrize("key", ["CESMode", "ConditionalExperimentalSwitching"])
def test_failure_log_is_rate_limited_to_once_a_minute(clock, logs, key):
  p = _P(clock, mode=(lambda t: UKN) if key == "CESMode" else (lambda t: 0), legacy=lambda t: UKN_LEGACY)
  for i in range(61):                                   # the callers read at ~1 Hz: +0 s .. +60 s
    clock[0] = 1000.0 + i
    C.read_ces_mode(p, who="VTSC")
    if i == 59:
      assert len(logs.mode("VTSC", key)) == 1           # still inside the minute
  lines = logs.mode("VTSC", key)
  assert len(lines) == 2
  assert "(1 failure(s)" in lines[0].msg and "(60 failure(s)" in lines[1].msg
  assert C.CES_MODE_READ_ERR_LOG_S == 60.0


def test_each_caller_and_each_read_has_its_own_log_state(clock, logs):
  # cesmodehold2pnw Q2: the legacy read is reached only when CESMode itself reads a genuine 0, so the two failing reads
  # now need their own params (one stub failing both would only ever log the CESMode one).
  bad_mode = _P(clock, mode=lambda t: UKN)
  bad_legacy = _P(clock, mode=lambda t: 0, legacy=lambda t: UKN_LEGACY)
  for who in ("VTSC", "CES", "CES overlay"):
    for p in (bad_mode, bad_legacy):
      C.read_ces_mode(p, who=who)
      C.read_ces_mode(p, who=who)
  for who in ("VTSC", "CES", "CES overlay"):
    assert len(logs.mode(who, "CESMode")) == 1 and len(logs.mode(who, "ConditionalExperimentalSwitching")) == 1


def test_a_raising_logger_cannot_change_the_result_or_escape(clock, logs, monkeypatch):
  """The UI overlay calls read_ces_mode with no try of its own."""
  def boom(*a, **k):
    raise RuntimeError("logger down")
  for level in ("exception", "error", "warning"):   # cesmodehold2pnw: the hold's change-only lines too
    monkeypatch.setattr(C.cloudlog, level, boom)
  assert C.read_ces_mode(_P(clock, mode=lambda t: UKN), who="CES overlay") == 0
  assert C.read_ces_mode(_P(clock, mode=lambda t: UKN, legacy=lambda t: True), who="CES overlay") == 0   # Q2
  assert C.read_ces_mode(_P(clock, mode=lambda t: 0, legacy=lambda t: UKN_LEGACY), who="CES overlay") == 0


# ---------------------------------------------------------------- through VTSC (plannerd), both cars
V_SET = 31.3
VISION_K = 2.5 / 15.0 ** 2


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


def _vtsc_drive(clock, fp, brand, mode, ticks=200):
  clock[0] = 1000.0
  C._ces_mode_hold_st.clear()   # cesmodehold2pnw: each drive is a fresh plannerd
  c = vc.VTSCController(FakeCP(fp, brand, True), params=_P(clock, mode=mode))
  c.mem_params = _VMem()
  out = []
  for i in range(ticks):
    clock[0] = 1000.0 + i / 20.0
    cap = c.cap(_vsm(), V_SET, V_SET)
    out.append((cap, dict(c.msg), c.overlay_payload()))
  return out


CARS = [(TESLA, "tesla"), (LIGHTNING, "ford")]


@pytest.mark.parametrize("fp, brand", CARS, ids=["tesla", "lightning-oplong"])
def test_vtsc_an_unreadable_cesmode_drives_exactly_like_ces_off_and_says_so(clock, logs, fp, brand):
  on = _vtsc_drive(clock, fp, brand, lambda t: 2)
  assert min(cap for cap, _, _ in on) < V_SET - 1.0                      # positive control: VTSC brakes when on
  assert logs.errors() == []
  got = _vtsc_drive(clock, fp, brand, lambda t: UKN)
  assert got == _vtsc_drive(clock, fp, brand, lambda t: 0)
  assert len(logs.mode("VTSC")) == 1
  assert [r for r in logs.errors() if "enable/mode read FAILED" in str(r.msg)] == []   # read_ces_mode handled it
  # cesmodehold2pnw: mid-drive, VTSC's reads land on whole seconds, so the 3 s failure holds Standard to exactly 13 s
  mid = _vtsc_drive(clock, fp, brand, lambda t: 2 if t < 3.0 else UKN, ticks=400)
  assert min(cap for cap, _, _ in mid[:60]) < V_SET - 1.0
  assert mid == _vtsc_drive(clock, fp, brand, lambda t: 2 if t < 13.0 else 0, ticks=400)
  assert mid != _vtsc_drive(clock, fp, brand, lambda t: 2 if t < 3.0 else 0, ticks=400)   # the hold is what differs


# ---------------------------------------------------------------- through CES (selfdrived), all three cars
def _ces_run(clock, fp, brand, op_long, mode, lead, curve0, v0, T=12.0):
  """The REAL CESController.experimental_request at 100 Hz (the silentexc2pnw CES identity harness, shortened)."""
  clock[0] = 5000.0
  C._ces_mode_hold_st.clear()   # cesmodehold2pnw: each run is a fresh selfdrived
  st = {"curve_at": curve0, "v": v0, "stock": 60 * MPH}

  class Mem:
    def __init__(self):
      self.puts = []

    def get(self, k, return_default=False):
      if k == "MapTargetVelocities":
        return _scene(st["curve_at"], 180.0, 20.0) if st["curve_at"] is not None else []
      if k == "LastGPSPosition":
        return json.dumps({"latitude": LAT0, "longitude": LON0, "bearing": 0.0, "src": "car", "ts": clock[0],
                           "fix_ts": clock[0] - 0.3})
      if k == "MapSpeedLimit":
        return str(60 * MPH)
      return None

    def put_nonblocking(self, k, v):
      v = copy.deepcopy(v)
      if isinstance(v, dict):
        v.pop("ts", None)
      self.puts.append((round(clock[0], 3), k, v))

  params = _P(clock, mode=lambda t: mode(t - 4000.0), extra={"CESButtonState": "0"})
  c = m.CESController(FakeCP(fp, brand, op_long), params=params)
  c.mem_params = Mem()
  records, decisions = [], []
  c._event_log_ok = True
  c._append_event = lambda rec: records.append(copy.deepcopy(rec))
  NS = types.SimpleNamespace
  i = 0
  while clock[0] - 5000.0 < T:
    i += 1
    clock[0] = 5000.0 + i * 0.01
    v, ca = st["v"], st["curve_at"]
    if ca is not None:
      st["curve_at"] = ca - v * 0.01
    orz, vx, px, ts = _model(v, st["curve_at"] if ca is not None else 1e9, 180.0)
    model = NS(orientationRate=NS(z=orz, t=ts), velocity=NS(x=vx), position=NS(x=px), action=NS(shouldStop=False),
               meta=NS(laneChangeState="off"))
    has = lead is not None
    sm = {"radarState": NS(leadOne=NS(status=has, vLead=lead[1] if has else 0.0, dRel=lead[0] if has else 0.0,
                                      aLeadK=0.0, vLeadK=lead[1] if has else 0.0)),
          "modelV2": model, "carControl": NS(orientationNED=[0.0, 0.0, 0.0])}
    tgt = next((p for _, k, p in reversed(c.mem_params.puts) if k == "IcbmTarget"), None)
    if tgt and "target" in tgt and i % 30 == 0 and abs(tgt["target"] - st["stock"]) > 0.3:
      st["stock"] += MPH if tgt["target"] > st["stock"] else -MPH
    cs = NS(vEgo=v, aEgo=0.0, gasPressed=False, brakePressed=False, leftBlinker=False, rightBlinker=False,
            vCruise=st["stock"] * 3.6, standstill=False, steeringAngleDeg=0.0, steeringPressed=False,
            leftBlindspot=False, rightBlindspot=False, cruiseState=NS(speed=st["stock"], enabled=True))
    decisions.append(c.experimental_request(cs, sm))
    st["v"] = max(min(st["v"] + (min(st["stock"], 40.0) - st["v"]) * 0.002, 40.0), 5.0)
  return {"dec": decisions, "puts": c.mem_params.puts, "rec": records}


SCENES = [
  pytest.param(TESLA, "tesla", True, (15.0, 3.0), None, 12.0, "dec", id="tesla-slow-lead"),
  pytest.param(LIGHTNING, "ford", True, (15.0, 3.0), None, 12.0, "dec", id="lightning-oplong-slow-lead"),
  pytest.param(LIGHTNING, "ford", False, None, 150.0, 26.0, "icbm", id="lightning-stock-acc-curve"),
]


def _acted(run, what):
  if what == "dec":
    return sum(run["dec"])
  return sum(1 for _, k, p in run["puts"] if k == "IcbmTarget" and p.get("target") is not None)


@pytest.mark.parametrize("fp, brand, op_long, lead, curve0, v0, what", SCENES)
def test_ces_an_unreadable_cesmode_runs_exactly_like_ces_off_and_says_so(clock, logs, fp, brand, op_long, lead, curve0,
                                                                          v0, what):
  on = _ces_run(clock, fp, brand, op_long, lambda t: 2, lead, curve0, v0)
  assert _acted(on, what) > 20                        # positive control: Experimental / ICBM curve targets when on
  assert logs.mode() == []
  got = _ces_run(clock, fp, brand, op_long, lambda t: UKN, lead, curve0, v0)
  want = _ces_run(clock, fp, brand, op_long, lambda t: 0, lead, curve0, v0)
  assert _acted(want, what) == 0 and got == want
  assert len(logs.mode("CES")) == 1 and "CES keeps its last good mode for up to 10 s" in logs.mode("CES")[0].msg


@pytest.mark.parametrize("fp, brand, op_long, lead, curve0, v0, what", SCENES)
def test_ces_a_failure_mid_drive_holds_standard_then_is_exactly_switching_ces_off(clock, logs, monkeypatch, fp, brand,
                                                                                    op_long, lead, curve0, v0, what):
  """cesmodehold2pnw: CES reads at ~1 Hz on 0.01 s float steps (x.01 s), so a 10 s hold lands ON a read and float
  rounding picks the side. 9.5 s puts the expiry unambiguously on the 10th read after the first failure (16.01 s); the
  10.0 s boundary itself is pinned with an exact clock in test_ces_mode_hold.py."""
  monkeypatch.setattr(C, "CES_MODE_HOLD_S", 9.5)
  got = _ces_run(clock, fp, brand, op_long, lambda t: 2 if t < 6.0 else UKN, lead, curve0, v0, T=20.0)
  want = _ces_run(clock, fp, brand, op_long, lambda t: 2 if t < 16.0 else 0, lead, curve0, v0, T=20.0)
  assert _acted({"dec": got["dec"][:600], "puts": [p for p in got["puts"] if p[0] < 5006.0]}, what) > 0
  assert got == want
  assert got != _ces_run(clock, fp, brand, op_long, lambda t: 2 if t < 6.0 else 0, lead, curve0, v0, T=20.0)
  assert len(logs.mode("CES")) == 1


# ---------------------------------------------------------------- through the CES overlay (ui)
def _overlay(monkeypatch, params):
  from openpilot.selfdrive.ui.onroad import ces_status as cs
  monkeypatch.setattr(cs, "ui_state", types.SimpleNamespace(params=params, started=False, is_metric=False))
  r = object.__new__(cs.CesStatusRenderer)
  r._last_poll, r._mem, r._onroad_t0, r._ces_enabled = -1e9, None, None, None
  C._ces_mode_hold_st.clear()   # cesmodehold2pnw: each call is a fresh ui (these cases are about the read, not the hold)
  r._update_state()
  return r._ces_enabled


def test_ui_overlay_an_unreadable_cesmode_hides_it_like_ces_off_and_says_so(clock, logs, monkeypatch):
  assert _overlay(monkeypatch, _P(clock, mode=lambda t: 2)) is True       # positive control
  assert _overlay(monkeypatch, _P(clock, mode=lambda t: 0)) is False
  assert logs.errors() == []
  assert _overlay(monkeypatch, _P(clock, mode=lambda t: UKN)) is False
  assert len(logs.mode("CES overlay")) == 1
