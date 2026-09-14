"""pscmlimlog2pnw: the PSCM lateral-limit logger (selfdrive/car/pscmlim_pnw.py) and its guard.

Every CAN input here goes through the REAL Lightning CarInterface: real 0x3CC payloads are fed to CarInterface.update(),
so opendbc's CANParser decodes them and opendbc's Ford carstate registers the message exactly as on the truck.

The guard (test_logging_a_real_limit_frame_leaves_the_angle_clamp_input_at_zero) is the point of the whole change.
opendbc's lateral_angle_pnw.py reads getattr(CS, 'lat_ctl_lim_stat', 0), and its PSCM clamp is dead code because nothing
sets that name. Feeding it the real signal would have frozen the 2026-09-08 19:44 PT command below what the truck was
still delivering (drives/2026-09-12/central-oregon-weekend/PSCM_LIMITREACHED.md s5, option H1). Logging must not switch
it on.
"""
import inspect
import json
import math
import time
from types import SimpleNamespace

import pytest

from opendbc.can.dbc import DBC
from opendbc.can.packer import CANPacker
from opendbc.can.parser import get_raw_value
from opendbc.car import structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.ford.values import CAR, FordFlags

from openpilot.selfdrive.car import accdrop_pnw as adp
from openpilot.selfdrive.car import pscmlim_pnw as pl
from openpilot.selfdrive.car.card import Car
from openpilot.selfdrive.controls.lib.pnw_vehicle import PnwVehicle

LIGHTNING = CAR.FORD_F_150_LIGHTNING_MK1
RAVEN = "TESLA_MODEL_S_HW3"
MSG = "Lane_Assist_Data3_FD1"
ADDR = 0x3CC

# REAL Lane_Assist_Data3_FD1 payloads, bus 0, from the Lightning rlogs in drives/2026-09-12/central-oregon-weekend/rlogs/
F_NOT_REACHED = "2800028088f90000"    # 00000131--4c7547abb4--13, Sat 2026-09-12 12:28:43.645 PDT: lim 0, cpb 2, ste 2
F_LIMIT_CLOSE = "28000280a5f10000"    # same segment, 12:28:43.195 PDT: lim 1 LimitClose, cpb 2, ste 2
F_DRIVER_ACTIVE = "08000280b3ec0000"  # 0000012e--6e434c48e6--157, Fri 2026-09-11 22:07:17.261 PDT: lim 3, cpb 2, ste 2
_DBC = DBC("ford_lincoln_base_pt")


def _repack(base_hex: str, **changes) -> str:
  """A real payload with some signals rewritten by the real packer. Used ONLY where no raw frame is on this host: the
  one hands-off LimitReached (2) episode is segment 00000110--0138170496--62, whose rlog is not local."""
  m = _DBC.name_to_msg[MSG]
  dat = bytes.fromhex(base_hex)
  vals = {name: get_raw_value(dat, s) * s.factor + s.offset for name, s in m.sigs.items()}
  vals.update(changes)
  return bytes(CANPacker("ford_lincoln_base_pt").make_can_msg(MSG, 0, vals)[1]).hex()


F_LIMIT_REACHED = _repack(F_LIMIT_CLOSE, LatCtlLim_D_Stat=2)
F_CLOSE_CPB1 = _repack(F_LIMIT_CLOSE, LatCtlCpblty_D_Stat=1)


class FakeLog:
  def __init__(self):
    self.calls = []

  def _rec(self, kind):
    return lambda msg, *a, **k: self.calls.append((kind, msg % a if a else msg))

  def __getattr__(self, kind):
    return self._rec(kind)

  def of(self, kind):
    return [m for k, m in self.calls if k == kind]


def _lightning_ci():
  CI_cls = interfaces[LIGHTNING]
  return CI_cls(CI_cls.get_non_essential_params(LIGHTNING))


class Rig:
  """A Lightning card as the logger sees it. Built BEFORE the first CarInterface.update(), in card's order."""

  def __init__(self, cc_obj="real", write_fn=None, route_fn=None):
    self.CI = _lightning_ci()
    assert self.CI.CP.flags & FordFlags.CANFD, "fixture: the Lightning must be a CAN-FD Ford"
    self.records: list = []
    report = PnwVehicle(SimpleNamespace(carFingerprint=LIGHTNING, brand="ford", openpilotLongitudinalControl=False)).pscm_limit_report
    self.log = pl.PscmLimitLogger(self.CI.can_parsers, report, self.CI.CC if cc_obj == "real" else cc_obj,
                                  write_fn=write_fn or self.records.append,
                                  route_fn=route_fn or (lambda wall: {"route": "00000131--4c7547abb4", "seg": 13}),
                                  threaded=False)
    self.cc = structs.CarControl()
    self.t = 1000.0
    # card's first tick: opendbc carstate registers the message during this update, AFTER the parser ran, so a 0x3CC
    # frame in this very first batch would be dropped by the parser itself (as on the truck).
    self.tick()

  def tick(self, *frames, cs=None):
    self.t += 0.01
    CS = self.CI.update([(int(self.t * 1e9), [(ADDR, bytes.fromhex(h), 0) for h in frames])])
    for k, v in (cs or {}).items():
      setattr(CS, k, v)
    self.log.update(CS, self.cc, self.t)
    return CS

  def parsed(self):
    return [json.loads(r) for r in self.records]

  def edges(self):
    return [(r["from"], r["to"]) for r in self.parsed()]


# ---------------------------------------------------------------- capability view / Tesla

def test_capability_view_names_the_message_only_for_the_lightning():
  raven = PnwVehicle(SimpleNamespace(carFingerprint=RAVEN, brand="tesla", openpilotLongitudinalControl=True))
  assert raven.pscm_limit_report == ()
  assert PnwVehicle(None).pscm_limit_report == ()
  report = PnwVehicle(SimpleNamespace(carFingerprint=LIGHTNING, brand="ford", openpilotLongitudinalControl=False)).pscm_limit_report
  assert report == ("pt", MSG, "LatCtlLim_D_Stat", "LatCtlCpblty_D_Stat")
  # what it names really is registered by the pinned opendbc carstate after one update, on that bus, with those signals
  rig = Rig()
  rig.tick()
  p = rig.CI.can_parsers[report[0]]
  assert dict.__contains__(p.vl, MSG)
  assert {report[2], report[3]} <= set(p.dbc.name_to_msg[MSG].sigs)


class _StubCar:
  """Just enough of card's Car for the two pscmlimlog2pnw methods -- the REAL methods run."""
  def __init__(self, CP, CI):
    self.CP, self.CI = CP, CI
    self.sm = {"carControl": structs.CarControl()}
    self._pscmlim_err = 0

  _pscm_limit_logger = Car._pscm_limit_logger
  _log_pscm_limit = Car._log_pscm_limit


class _Untouchable:
  def __getattr__(self, name):
    raise AssertionError(f"the Tesla path touched CarInterface.{name}")


def test_card_builds_nothing_for_the_tesla(monkeypatch):
  log = FakeLog()
  monkeypatch.setattr("openpilot.selfdrive.car.card.cloudlog", log)
  CI_cls = interfaces[RAVEN]
  CP = CI_cls.get_non_essential_params(RAVEN)
  car = _StubCar(CP, _Untouchable())
  assert car._pscm_limit_logger() is None
  assert log.calls == []   # not a swallowed failure: the capability said no before CarInterface was touched


def test_card_builds_the_logger_for_the_lightning_and_a_failure_is_loud(monkeypatch):
  log = FakeLog()
  monkeypatch.setattr("openpilot.selfdrive.car.card.cloudlog", log)
  CI = _lightning_ci()
  lg = _StubCar(CI.CP, CI)._pscm_limit_logger()
  assert isinstance(lg, pl.PscmLimitLogger) and lg._parsers is CI.can_parsers and lg._cc_obj is CI.CC
  assert log.calls == []
  assert _StubCar(CI.CP, SimpleNamespace(CC=None))._pscm_limit_logger() is None   # no can_parsers -> raises inside
  assert len(log.of("exception")) == 1 and "construction FAILED" in log.of("exception")[0]


def test_card_update_failure_is_logged_first_then_rate_limited_and_never_escapes(monkeypatch):
  log = FakeLog()
  monkeypatch.setattr("openpilot.selfdrive.car.card.cloudlog", log)
  car = _StubCar(None, None)
  car._pscmlim = SimpleNamespace(update=lambda *a: 1 / 0)
  for _ in range(6000):
    car._log_pscm_limit(structs.CarState())
  assert car._pscmlim_err == 6000
  assert len(log.of("exception")) == 2 and "logging is DARK" in log.of("exception")[0]


class _Seen(dict):
  seen = {"onroadEvents": False}


class _StepCar(_StubCar):
  """card's REAL step() with the CAN/publish plumbing stubbed out, to pin the pscmlim call site."""
  step = Car.step

  def __init__(self, pscmlim):
    super().__init__(SimpleNamespace(passive=True), None)
    self.sm = _Seen(carControl=structs.CarControl(), onroadEvents=[])
    self._cs = structs.CarState()
    self._accdrop = None
    self._pscmlim = pscmlim
    self.calls: list = []

  def state_update(self):
    return self._cs, None

  def state_publish(self, CS, RD):
    pass


def test_card_step_ticks_the_logger_with_this_ticks_carstate_and_skips_it_when_absent():
  lg = SimpleNamespace(calls=[])
  lg.update = lambda cs, cc, now: lg.calls.append((cs, cc, now))
  car = _StepCar(lg)
  car.step()
  assert len(lg.calls) == 1 and lg.calls[0][0] is car._cs and lg.calls[0][1] is car.sm["carControl"]
  car = _StepCar(None)                                      # the Tesla: no logger, and step never calls into one
  car._log_pscm_limit = lambda CS: (_ for _ in ()).throw(AssertionError("called with no logger"))
  car.step()


# ---------------------------------------------------------------- decode + edges

def test_real_frames_every_change_is_exactly_one_record():
  rig = Rig()
  rig.tick()
  assert rig.records == []                                  # nothing received yet: no record, no fake 0
  for _ in range(10):
    rig.tick(F_NOT_REACHED)
  for _ in range(5):
    rig.tick()                                              # ticks without a 0x3CC frame are not changes
  for _ in range(30):
    rig.tick(F_LIMIT_CLOSE)
  for _ in range(50):
    rig.tick(F_LIMIT_REACHED)
  rig.tick(F_DRIVER_ACTIVE)
  rig.tick(F_NOT_REACHED)
  recs = rig.parsed()
  assert rig.edges() == [(None, 0), (0, 1), (1, 2), (2, 3), (3, 0)]
  assert recs[0]["heldS"] is None and recs[0]["fromWhy"].startswith("first frame this card session")
  assert [r["heldS"] for r in recs[1:]] == [0.15, 0.3, 0.5, 0.01]
  assert all(r["cpb"] == 2 for r in recs)
  assert all("fromWhy" not in r for r in recs[1:])


def test_two_changes_in_one_can_batch_are_both_recorded_with_their_own_capability():
  rig = Rig()
  rig.tick(F_NOT_REACHED)
  rig.tick(F_CLOSE_CPB1, F_NOT_REACHED)                     # a 1-frame LimitClose and the clear, parsed in ONE update
  assert rig.edges() == [(None, 0), (0, 1), (1, 0)]
  assert [r["cpb"] for r in rig.parsed()] == [2, 1, 2]


def test_record_is_ces_events_shaped_and_carries_the_context():
  rig = Rig(cc_obj=SimpleNamespace(_latext_angle=SimpleNamespace(path_angle_last=0.1225)))
  rig.cc.latActive = True
  rig.cc.actuators.curvature = 0.00188
  ctx = dict(vEgo=25.8, steeringAngleDeg=-24.03, steeringTorque=-1.4, steeringPressed=True, yawRate=-0.1123)
  rig.tick(F_NOT_REACHED, cs=ctx)
  rig.tick(F_LIMIT_REACHED, cs=ctx)
  line = rig.records[-1]
  assert "NaN" not in line and line == line.strip()
  r = json.loads(line)
  assert set(r) == {"t", "ev", "v", "from", "to", "heldS", "cpb", "latActive", "kDes", "vEgo", "ang", "tq", "sp", "yaw", "pa",
                    "route", "seg"}
  assert (r["ev"], r["v"], r["from"], r["to"]) == ("pscmLim", 1, 0, 2)
  assert (r["latActive"], r["kDes"], r["vEgo"], r["ang"], r["tq"], r["sp"], r["yaw"], r["pa"]) == \
         (True, 0.00188, 25.8, -24.0, -1.4, True, -0.1123, 0.1225)
  assert (r["route"], r["seg"]) == ("00000131--4c7547abb4", 13)
  assert abs(r["t"] - time.time()) < 5  # noqa: TID251 -- the record stamps wall time
  assert {"fromWhy"} <= set(rig.parsed()[0])


def test_records_go_to_the_ces_events_log():
  from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import CES_EVENT_LOG
  params = inspect.signature(pl.PscmLimitLogger).parameters
  assert params["write_fn"].default is adp.append_to_ces_log and adp.CES_EVENT_LOG == CES_EVENT_LOG
  assert params["route_fn"].default is adp.current_route_segment
  assert params["threaded"].default is True                  # the file append never runs in card's loop


@pytest.mark.parametrize("cc_obj,why", [
  (None, "no carcontroller"),
  (SimpleNamespace(), "this carcontroller has no angle-mode strategy"),
  ("real", "angle mode is not running"),                    # the real Ford carcontroller with FordAngleLateral off
])
def test_path_angle_is_null_with_a_reason_never_zero(cc_obj, why):
  rig = Rig(cc_obj=cc_obj)
  rig.tick(F_LIMIT_CLOSE)
  r = rig.parsed()[0]
  assert r["pa"] is None and r["paWhy"].startswith(why)


# ---------------------------------------------------------------- Rule 2

def test_unregistered_message_is_never_registered_and_reported_once(monkeypatch):
  log = FakeLog()
  monkeypatch.setattr(pl, "cloudlog", log)
  CI = _lightning_ci()                                      # carstate never ran: nothing registered
  report = PnwVehicle(SimpleNamespace(carFingerprint=LIGHTNING, brand="ford")).pscm_limit_report
  records: list = []
  lg = pl.PscmLimitLogger(CI.can_parsers, report, None, write_fn=records.append, route_fn=lambda w: {}, threaded=False)
  t = 50.0
  for i in range(2000):                                     # 20 s at 100 Hz, retrying the binding once a second
    lg.update(structs.CarState(), structs.CarControl(), t + i * 0.01)
  assert not dict.__contains__(CI.can_parsers["pt"].vl, MSG)
  assert len(records) == 1
  r = json.loads(records[0])
  assert r["to"] is None and r["from"] is None and "not registered by opendbc carstate" in r["why"]
  assert len(log.of("error")) == 1 and "NOT being logged" in log.of("error")[0]


def test_no_parser_names_the_reason(monkeypatch):
  monkeypatch.setattr(pl, "cloudlog", FakeLog())
  records: list = []
  lg = pl.PscmLimitLogger({}, ("pt", MSG, "LatCtlLim_D_Stat", "LatCtlCpblty_D_Stat"), None,
                          write_fn=records.append, route_fn=lambda w: {}, threaded=False)
  for i in range(1100):
    lg.update(structs.CarState(), structs.CarControl(), i * 0.01)
  assert len(records) == 1 and "no 'pt' CAN parser" in json.loads(records[0])["why"]


def test_registered_but_silent_is_reported_once_and_a_late_first_frame_is_announced(monkeypatch):
  log = FakeLog()
  monkeypatch.setattr(pl, "cloudlog", log)
  rig = Rig()
  for _ in range(round(pl.UNSEEN_S * 100) - 5):
    rig.tick()
  assert rig.records == []                                  # not before UNSEEN_S
  for _ in range(300):
    rig.tick()
  assert len(rig.records) == 1 and "registered but no frame was received" in rig.parsed()[0]["why"]
  rig.tick(F_NOT_REACHED)
  assert rig.edges() == [(None, None), (None, 0)]
  assert len(log.of("warning")) == 1 and "first Lane_Assist_Data3_FD1 frame" in log.of("warning")[0]


def test_a_live_frame_stream_never_reports_unseen_and_neither_does_a_later_silence():
  rig = Rig()
  for _ in range(round(pl.UNSEEN_S * 100) + 500):
    rig.tick(F_NOT_REACHED)
  for _ in range(round(pl.UNSEEN_S * 100) + 500):
    rig.tick()                                              # a message that goes quiet is canValid's job, not this log's
  assert rig.edges() == [(None, 0)]


def test_a_message_registered_after_the_first_tick_is_still_bound():
  CI = _lightning_ci()
  report = PnwVehicle(SimpleNamespace(carFingerprint=LIGHTNING, brand="ford")).pscm_limit_report
  records: list = []
  lg = pl.PscmLimitLogger(CI.can_parsers, report, None, write_fn=records.append, route_fn=lambda w: {}, threaded=False)
  t = 10.0
  for _ in range(50):                                       # card ticks before carstate has registered anything
    t += 0.01
    lg.update(structs.CarState(), structs.CarControl(), t)
  for _ in range(150):
    t += 0.01
    CS = CI.update([(int(t * 1e9), [(ADDR, bytes.fromhex(F_LIMIT_CLOSE), 0)])])
    lg.update(CS, structs.CarControl(), t)
  assert [(json.loads(r)["from"], json.loads(r)["to"]) for r in records] == [(None, 1)]


def test_flapping_is_rate_limited_counted_and_warned_once(monkeypatch):
  log = FakeLog()
  monkeypatch.setattr(pl, "cloudlog", log)
  rig = Rig()
  for i in range(50):                                        # 50 changes in 0.5 s (the first frame counts as one)
    rig.tick(F_NOT_REACHED if i % 2 == 0 else F_LIMIT_CLOSE)
  assert len(rig.records) == pl.MAX_RECORDS_PER_MIN
  assert len(log.of("warning")) == 1 and "suppressing" in log.of("warning")[0]
  rig.t += 61.0
  rig.tick(F_NOT_REACHED)
  last = rig.parsed()[-1]
  assert (last["from"], last["to"], last["suppressed"]) == (1, 0, 30)
  rig.tick(F_LIMIT_CLOSE)
  assert "suppressed" not in rig.parsed()[-1]


def test_write_failure_is_loud_and_does_not_escape(monkeypatch):
  log = FakeLog()
  monkeypatch.setattr(pl, "cloudlog", log)
  outcomes = iter([False, False, True, False])
  written: list = []

  def write(line):
    if not next(outcomes):
      raise OSError("disk full")
    written.append(line)

  rig = Rig(write_fn=write)
  for frame in (F_NOT_REACHED, F_LIMIT_CLOSE, F_NOT_REACHED, F_LIMIT_CLOSE):
    rig.tick(frame)
  msgs = log.of("exception")
  assert len(msgs) == 3 and len(written) == 1
  assert "FAILED to write the 0->1 record (2 in a row)" in msgs[1]
  assert "FAILED to write the 0->1 record (1 in a row)" in msgs[2]   # the success in between reset the streak


def test_the_file_append_runs_off_the_card_thread():
  import threading
  done = threading.Event()
  where: list = []

  def write(line):
    where.append(threading.current_thread().name)
    done.set()

  rig = Rig(write_fn=write)
  rig.log._threaded = True
  rig.tick(F_NOT_REACHED)
  assert done.wait(5.0)
  assert where == ["pscmlimlog"]


def test_a_non_finite_value_is_null_not_nan_and_not_zero():
  rig = Rig()
  rig.tick(F_LIMIT_CLOSE, cs=dict(yawRate=float("nan"), vEgo=float("inf")))
  line = rig.records[0]
  assert "NaN" not in line and "Infinity" not in line
  r = json.loads(line)
  assert r["yaw"] is None and r["vEgo"] is None


def test_update_cost_is_tiny():
  rig = Rig()
  rig.tick(F_NOT_REACHED)
  CS, cc, lg = rig.CI.CS.out, rig.cc, rig.log
  n = 20000
  start = time.perf_counter()
  for i in range(n):
    lg.update(CS, cc, rig.t + i * 0.01)
  per_tick = (time.perf_counter() - start) / n
  print(f"pscmlim update(): {per_tick * 1e6:.2f} us/tick on this host")
  assert per_tick < 20e-6 and math.isfinite(per_tick)


# ---------------------------------------------------------------- the guard

def _angle_run(limit_frame: str, feed_attr: bool = False):
  """Real decode -> logger -> opendbc's LateralAngleExt on the SAME real CarState object carcontroller hands it.
  20 frames of LimitNotReached while a curve builds, then 20 frames of `limit_frame` while it keeps building. The car
  tracks the command (measured curvature == commanded), so no other limiter binds. feed_attr sets the name the clamp
  reads -- the control case that proves this fixture reaches the clamp at all."""
  from opendbc.car.ford.lateral_angle_pnw import LateralAngleExt
  rig = Rig()
  ext = LateralAngleExt(rig.CI.CP)
  ext.path_angle_blend_ratio = 0.0     # no modelV2 in a test (see test_blipguard_pnw.py)
  ext._apply_tuning = lambda: None
  v = 25.0
  path_angles = []
  for i in range(40):
    rig.tick(F_NOT_REACHED if i < 20 else limit_frame)
    CS = rig.CI.CS
    if feed_attr:
      CS.lat_ctl_lim_stat = int(dict.get(rig.CI.can_parsers["pt"].vl, MSG)["LatCtlLim_D_Stat"])
    kappa = 0.0002 * i
    out = structs.CarState()
    out.vEgo = out.vEgoRaw = v
    out.yawRate = -kappa * v
    CS.out = out
    path_angles.append(ext.update(SimpleNamespace(latActive=True), CS, SimpleNamespace(curvature=kappa), rig.CI.CP).path_angle)
  return rig, path_angles


@pytest.mark.parametrize("limit_frame,value", [(F_LIMIT_CLOSE, 1), (F_LIMIT_REACHED, 2)])
def test_logging_a_real_limit_frame_leaves_the_angle_clamp_input_at_zero(limit_frame, value):
  _, baseline = _angle_run(F_NOT_REACHED)
  rig, logged = _angle_run(limit_frame)
  assert rig.edges() == [(None, 0), (0, value)]            # the limit really was decoded and logged ...
  assert not hasattr(rig.CI.CS, "lat_ctl_lim_stat")         # ... under no name the clamp reads
  assert logged == baseline                                 # so the command is exactly what it was without it

  _, fed = _angle_run(limit_frame, feed_attr=True)
  assert fed[:20] == baseline[:20]
  assert max(abs(x) for x in fed[20:]) < abs(baseline[-1]) - 0.05, \
    "control case: feeding the clamp must freeze the growing command, else this guard proves nothing"
