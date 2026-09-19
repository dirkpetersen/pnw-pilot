"""parkgate2pnw — the ces_events breadcrumb stops logging a PARKED truck, and only a parked truck.

The facts this file defends, in order of how much they cost when they break:
  1. **A red-light stop still logs at full rate.** vEgo 0 in DRIVE is the single most analysed state
     in this log (stop / lurch / green-light). Gating on speed or standstill would delete it.
  2. **Everything ambiguous keeps logging.** Unknown gear, no carState, a gear that is not the enum,
     an unread kill switch, a moving car whose gear decodes as park -- all log.
  3. **A suppressed window is never a silent gap.** One marked heartbeat per minute, carrying how
     many records it stands for, plus exactly two swaglog lines per Park episode.
  4. **The gear is now IN the record.** The whole reason 93.7% parked content hid for months.

Every gear value here comes from a REAL `car.CarState` message, so the values under test are genuine
`_DynamicEnum`s and not hand-made ints -- which is what makes the capnp-enum-str-trap tests real.
"""
import pytest

from cereal import car
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw_constants as C
from openpilot.selfdrive.controls.lib.ces_pnw import park_tick_gate as pg
from openpilot.selfdrive.controls.lib.ces_pnw.park_tick_gate import ParkTickGate, LOG, HOLD, RELEASE, SUPPRESS

GearShifter = car.CarState.GearShifter


def gear(name):
  """The gear value a real carState carries -- a _DynamicEnum, not the int schema constant."""
  cs = car.CarState.new_message()
  cs.gearShifter = name
  return cs.gearShifter


def run(g, secs, v_ego=0.0, gate_on=True, hz=1.0, gate=None, t0=1000.0):
  """Drive the gate for `secs` seconds of ~1 Hz ticks. Returns (gate, [decisions])."""
  gate = gate or ParkTickGate()
  out = []
  for i in range(int(secs * hz)):
    out.append(gate.update(g, v_ego, t0 + i / hz, gate_on))
  return gate, out


@pytest.fixture(autouse=True)
def _quiet_cloudlog(monkeypatch):
  """Capture cloudlog.event instead of writing it, and expose the calls to the tests."""
  calls = []
  monkeypatch.setattr(pg.cloudlog, "event", lambda name, **kw: calls.append((name, kw)))
  return calls


# =====================================================================================================
# 1. the thing that must never break: a stop is not a park
# =====================================================================================================
class TestAStopIsNotAPark:
  def test_an_hour_stopped_in_drive_logs_every_single_tick(self):
    """A red light, a traffic jam, a drive-through: vEgo 0, gear DRIVE. 3600/3600 records."""
    gate, out = run(gear("drive"), 3600)
    assert out.count(SUPPRESS) == 0
    assert set(out) == {LOG}
    assert gate.parked is False

  @pytest.mark.parametrize("g", ["drive", "reverse", "neutral", "low", "sport", "eco", "brake", "manumatic"])
  def test_no_gear_other_than_park_ever_suppresses(self, g):
    _, out = run(gear(g), 600)
    assert SUPPRESS not in out

  def test_standstill_is_not_consulted_at_all(self):
    """The gate takes no standstill/IsOnroad input -- it cannot regress into one."""
    import inspect
    params = inspect.signature(ParkTickGate.update).parameters
    assert set(params) == {"self", "gear", "v_ego", "now", "gate_on"}


# =====================================================================================================
# 2. the gate proper: hysteresis, heartbeat, release
# =====================================================================================================
class TestHysteresis:
  def test_the_first_29_seconds_of_park_are_logged_in_full(self):
    gate, out = run(gear("park"), 29)
    assert out == [LOG] * 29
    assert gate.holding is False and gate.parked is True

  def test_suppression_starts_at_the_30_second_mark_with_a_marked_record(self):
    gate, out = run(gear("park"), 31)
    assert out[:30] == [LOG] * 30                     # t0..t0+29 -> still inside the debounce
    assert out[30] == HOLD                            # t0+30 -> the hold's own opening record
    assert gate.holding is True

  def test_a_rest_stop_shuffle_never_suppresses(self):
    """Park 25 s, Drive 10 s, Park 25 s, Drive on. The surrounding driving context is kept whole."""
    gate = ParkTickGate()
    out = []
    for g, secs in (("drive", 20), ("park", 25), ("drive", 10), ("park", 25), ("drive", 60)):
      gate, o = run(gear(g), secs, gate=gate, t0=1000.0 + len(out))
      out += o
    assert SUPPRESS not in out and HOLD not in out and RELEASE not in out
    assert len(out) == 140

  def test_the_debounce_restarts_after_any_non_park_tick(self):
    gate = ParkTickGate()
    gate, _ = run(gear("park"), 29, gate=gate, t0=1000.0)
    gate, _ = run(gear("drive"), 1, gate=gate, t0=1029.0)     # one tick out of Park
    gate, out = run(gear("park"), 29, gate=gate, t0=1030.0)   # the clock starts over
    assert SUPPRESS not in out and gate.holding is False


class TestHeartbeat:
  def test_an_hour_parked_keeps_one_marked_record_per_minute(self):
    gate, out = run(gear("park"), 3600)
    assert out.count(LOG) == 30                         # the debounce window
    assert out.count(HOLD) == 60                        # the opening record + 59 more, one a minute
    assert out.count(SUPPRESS) == 3600 - 30 - 60
    assert gate.hold_total == out.count(SUPPRESS)

  def test_the_heartbeat_cadence_is_exactly_PARK_HEARTBEAT_S(self):
    _, out = run(gear("park"), 600)
    beats = [i for i, d in enumerate(out) if d == HOLD]
    assert beats[0] == 30
    assert all(b - a == pg.PARK_HEARTBEAT_S for a, b in zip(beats, beats[1:], strict=False))

  def test_a_heartbeat_says_how_many_records_it_stands_for(self):
    """Rule 2: a suppressed window must never be confusable with a dead logger."""
    gate = ParkTickGate()
    fields = []
    for i in range(200):
      d = gate.update(gear("park"), 0.0, 1000.0 + i, True)
      if d == HOLD:
        fields.append(gate.record_fields(d))
    # parkSupp is CUMULATIVE within the hold: 0 at the opening record, then +59 each minute (the
    # heartbeat tick itself is written, not suppressed). The release record closes the account.
    assert fields[0] == {"parkGate": "hold", "parkSupp": 0}
    assert fields[1] == {"parkGate": "hold", "parkSupp": 59}
    assert fields[2] == {"parkGate": "hold", "parkSupp": 118}

  def test_a_plain_record_carries_no_gate_fields_at_all(self):
    """99% of records keep their exact shape -- nothing downstream sees a new key on a normal tick."""
    gate = ParkTickGate()
    assert gate.record_fields(gate.update(gear("drive"), 20.0, 1000.0, True)) == {}


class TestRelease:
  def test_leaving_park_releases_on_the_very_next_tick_and_reports_the_loss(self):
    gate, _ = run(gear("park"), 600)
    d = gate.update(gear("drive"), 0.0, 1600.0, True)
    assert d == RELEASE
    assert gate.record_fields(d) == {"parkGate": "release", "parkSupp": 600 - 30 - 10}
    assert gate.holding is False and gate.suppressed == 0

  def test_the_tick_after_a_release_is_a_plain_record(self):
    gate, _ = run(gear("park"), 600)
    gate.update(gear("drive"), 0.0, 1600.0, True)
    assert gate.update(gear("drive"), 0.0, 1601.0, True) == LOG

  def test_a_release_with_no_hold_never_happens(self):
    gate, out = run(gear("park"), 10)
    assert gate.update(gear("drive"), 0.0, 1010.0, True) == LOG
    assert RELEASE not in out


# =====================================================================================================
# 3. fail open -- every ambiguity keeps logging
# =====================================================================================================
class TestFailsOpen:
  def test_an_unknown_gear_never_suppresses(self):
    """Tesla DI_GEAR_SNA/INVALID, Ford Unknown_Position, and every never-received gear message."""
    _, out = run(gear("unknown"), 3600)
    assert set(out) == {LOG}

  def test_no_carstate_at_all_never_suppresses(self):
    _, out = run(None, 3600)
    assert set(out) == {LOG}

  def test_a_stringified_gear_never_suppresses(self):
    """capnp-enum-str-trap, in the direction that matters here: the gate compares ENUMS. A caller
    that hands it a string must fail OPEN (keep logging), not accidentally match."""
    for s in ("park", "GearShifter.park", "PARK"):
      _, out = run(s, 3600)
      assert set(out) == {LOG}, s

  def test_the_gate_does_not_key_off_the_schema_int(self):
    """`GearShifter.park` is the int 1 and `str()` of it is '1', not 'park' -- so a str-based gate
    would behave differently for the constant and for a message value. Both must suppress."""
    assert str(GearShifter.park) == "1" and str(gear("park")) == "park"
    for g in (GearShifter.park, gear("park")):
      _, out = run(g, 40)
      assert HOLD in out, g

  def test_a_moving_car_logs_however_the_gear_decodes(self):
    """CANParser zero-inits every signal and 0 decodes as Park on 74 platforms in the pinned opendbc
    (selfdrive/car/gear_park.py). A gear message that never arrived must not cost a whole drive."""
    _, out = run(gear("park"), 3600, v_ego=25.0)
    assert set(out) == {LOG}

  @pytest.mark.parametrize("v", [0.0, 0.5, -0.4, None, "fast", float("nan"), float("inf")])
  def test_a_speed_that_is_not_evidence_of_motion_leaves_the_gear_in_charge(self, v):
    """The interlock is an escape hatch, not the gate: only a finite reading above the threshold
    releases. Unreadable/absent/non-finite speed must not stop the gear gate from working."""
    _, out = run(gear("park"), 40, v_ego=v)
    assert HOLD in out, v

  @pytest.mark.parametrize("v", [0.51, 1.0, -30.0])
  def test_any_finite_speed_over_the_threshold_releases(self, v):
    _, out = run(gear("park"), 40, v_ego=v)
    assert set(out) == {LOG}, v

  def test_the_interlock_breaks_an_existing_hold(self):
    """A truck that starts rolling while the gear still reads park resumes logging immediately."""
    gate, _ = run(gear("park"), 600)
    assert gate.holding is True
    assert gate.update(gear("park"), 30.0, 1600.0, True) == RELEASE


class TestKillSwitch:
  def test_gate_off_never_suppresses(self):
    _, out = run(gear("park"), 3600, gate_on=False)
    assert set(out) == {LOG}

  def test_gate_off_still_reports_the_gear(self):
    gate, _ = run(gear("park"), 5, gate_on=False)
    assert gate.parked is True     # the record still says the truck is parked; it just keeps logging

  def test_flipping_the_switch_on_mid_park_closes_the_hold_through_the_release_path(self):
    gate, _ = run(gear("park"), 600)
    d = gate.update(gear("park"), 0.0, 1600.0, False)
    assert d == RELEASE and gate.record_fields(d)["parkGate"] == "release"

  def test_flipping_the_switch_off_mid_park_needs_the_full_debounce_again(self):
    gate, _ = run(gear("park"), 600, gate_on=False)
    gate, out = run(gear("park"), 31, gate=gate, t0=1600.0)
    assert out[:30] == [LOG] * 30 and out[30] == HOLD


# =====================================================================================================
# 4. it announces itself -- and never per tick
# =====================================================================================================
class TestItSaysSo:
  def test_exactly_two_swaglog_lines_per_park_episode(self, _quiet_cloudlog):
    gate, _ = run(gear("park"), 3600)
    gate.update(gear("drive"), 0.0, 4600.0, True)
    assert len(_quiet_cloudlog) == 2, "the gate must log its edges, and ONLY its edges"
    assert [c[0] for c in _quiet_cloudlog] == ["ces_park_gate", "ces_park_gate"]
    assert _quiet_cloudlog[0][1]["hold"] is True
    assert _quiet_cloudlog[0][1]["gear"] == "park"        # str() of a _DynamicEnum: the bare name
    assert _quiet_cloudlog[1][1]["hold"] is False
    assert _quiet_cloudlog[1][1]["suppressed"] == 3600 - 30 - 60
    assert _quiet_cloudlog[1][1]["reason"] == "unparked"

  def test_nothing_is_logged_while_merely_driving(self, _quiet_cloudlog):
    run(gear("drive"), 3600)
    assert _quiet_cloudlog == []

  def test_nothing_is_logged_during_the_debounce(self, _quiet_cloudlog):
    run(gear("park"), 29)
    assert _quiet_cloudlog == []

  def test_the_kill_switch_release_names_itself(self, _quiet_cloudlog):
    gate, _ = run(gear("park"), 600)
    gate.update(gear("park"), 0.0, 1600.0, False)
    assert _quiet_cloudlog[-1][1]["reason"] == "gateOff"

  def test_the_cumulative_total_survives_across_episodes(self, _quiet_cloudlog):
    gate = ParkTickGate()
    for k in range(3):
      gate, _ = run(gear("park"), 600, gate=gate, t0=10000.0 * k)
      gate.update(gear("drive"), 0.0, 10000.0 * k + 700.0, True)
    assert gate.hold_total == 3 * (600 - 30 - 10)
    assert _quiet_cloudlog[-1][1]["hold_total"] == gate.hold_total


# =====================================================================================================
# 5. the wiring: does any of this reach the real record
# =====================================================================================================
def _tick_record(**over):
  from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_ces_record_fields import _record
  return _record(**over)


class TestTheRecordCarriesTheGear:
  def test_the_tick_record_names_the_gear(self):
    rec = _tick_record(_gear_name="drive")
    assert rec["gear"] == "drive"
    assert rec["park"] is False

  def test_the_tick_record_marks_a_parked_truck(self):
    g = ParkTickGate()
    g.update(gear("park"), 0.0, 1.0, True)
    rec = _tick_record(_gear_name="park", _park_gate=g)
    assert rec["gear"] == "park" and rec["park"] is True

  def test_an_unread_gear_reaches_the_record_as_null_not_as_park(self):
    rec = _tick_record(_gear_name=None)
    assert rec["gear"] is None and rec["park"] is False

  def test_park_is_the_gates_verdict_and_not_a_restatement_of_gear(self):
    """A moving truck whose gear decodes as park: gear says 'park', park says False. That divergence
    is the whole point of carrying both -- it is the only way to see a bad gear decode in the log."""
    g = ParkTickGate()
    g.update(gear("park"), 30.0, 1.0, True)
    rec = _tick_record(_gear_name="park", _park_gate=g)
    assert rec["gear"] == "park" and rec["park"] is False

  def test_the_ces_off_steer_record_carries_the_same_two_keys(self, monkeypatch):
    caps = _run_steer_log_step(monkeypatch)
    assert caps[0]["gear"] == "drive" and caps[0]["park"] is False


def _make_controller(monkeypatch, **over):
  """A CESController with __init__ bypassed, carrying only what the methods under test touch."""
  c = m.CESController.__new__(m.CESController)
  c._park_gate = ParkTickGate()
  c._park_gate_on = True
  c._park_err_t, c._park_err_n = None, 0
  c._gear = c._gear_name = c._v_ego_raw = None
  c._enabled = False
  c._steer_tick_last = -1e9
  c._tick_last = -1e9
  c._mode = 1
  c._car = "TESTCAR"
  c._cur_lat = c._cur_lon = c._cur_bearing = None
  c._speed_limit = 0.0
  c._car_gps = c._gps_src = None
  c._vtsc_cap = c._vtsc_state = None
  c._vtsc_tele = {}
  c._sa_tele = {}
  c._lc_corr = c._lc_act = c._lc_gate = c._lc_err = c._lc_lim_n = c._lc_spd_a = None
  c._sl_curv_lim = c._sl_safe_lim = c._sl_sat = c._sl_lat_active = c._sl_ang_sat = False
  c._sl_ang_des = c._sl_ang_act = c._sl_ang_err = 0.0
  c._sl_lat_dem = c._sl_lat_max = c._sl_curv_max = 0.0
  c._sl_k_cmd = c._sl_k_actl = c._sl_k_err = 0.02
  c._cp_off = c._cp_tgt = c._cp_cap = c._cp_why = c._cp_tq = c._cp_rate = c._cp_cmd = None
  c._curve_peak = m.CurvePeak()
  c._map_targets = []
  c.captured = []
  c._append_event = c.captured.append
  monkeypatch.setattr(c, "_read_map", lambda: c.__dict__.__setitem__("read_map_calls",
                                                                     c.__dict__.get("read_map_calls", 0) + 1),
                      raising=False)
  for k, v in over.items():
    setattr(c, k, v)
  return c


def _run_steer_log_step(monkeypatch, secs=1, g="drive", v_ego=20.0, c=None, t0=1000.0, hz=1.0):
  c = c or _make_controller(monkeypatch)
  cs = car.CarState.new_message()
  cs.gearShifter = g
  cs.vEgo = v_ego
  sm = {"radarState": type("RS", (), {"leadOne": type("L", (), {"status": False})()})()}
  for i in range(int(secs * hz)):
    monkeypatch.setattr(m.time, "monotonic", lambda t=t0 + i / hz: t)
    c._gear, c._gear_name, c._v_ego_raw = cs.gearShifter, str(cs.gearShifter), v_ego
    c._steer_log_step(cs, sm)
  return c.captured


class TestTheStepsHonourTheGate:
  def test_the_ces_off_breadcrumb_is_suppressed_while_parked(self, monkeypatch):
    c = _make_controller(monkeypatch)
    caps = _run_steer_log_step(monkeypatch, secs=600, g="park", v_ego=0.0, c=c)
    assert len(caps) == 30 + 10                       # 30 s of debounce + 10 minute-heartbeats
    assert [r for r in caps if r.get("parkGate") == "hold"]

  def test_a_suppressed_breadcrumb_costs_nothing(self, monkeypatch):
    """The gate runs BEFORE _read_map(), so a suppressed tick does no mem-param work at all."""
    c = _make_controller(monkeypatch)
    _run_steer_log_step(monkeypatch, secs=600, g="park", v_ego=0.0, c=c)
    assert c.__dict__["read_map_calls"] == 40         # once per record written, never per tick

  def test_the_ces_off_breadcrumb_survives_an_hour_at_a_red_light(self, monkeypatch):
    c = _make_controller(monkeypatch)
    caps = _run_steer_log_step(monkeypatch, secs=3600, g="drive", v_ego=0.0, c=c)
    assert len(caps) == 3600
    assert all("parkGate" not in r for r in caps)

  def test_the_steer_record_is_marked_parked_while_holding(self, monkeypatch):
    """Fable finding 1 (C09): `park` must be a real reading on the CES-off record too, not a
    constant False that happens to match every driving case."""
    c = _make_controller(monkeypatch)
    caps = _run_steer_log_step(monkeypatch, secs=600, g="park", v_ego=0.0, c=c)
    assert caps and all(r["gear"] == "park" and r["park"] is True for r in caps)

  def test_the_release_record_reaches_the_log(self, monkeypatch):
    c = _make_controller(monkeypatch)
    _run_steer_log_step(monkeypatch, secs=600, g="park", v_ego=0.0, c=c)
    caps = _run_steer_log_step(monkeypatch, secs=1, g="drive", v_ego=0.0, c=c, t0=2000.0)
    assert caps[-1]["parkGate"] == "release" and caps[-1]["parkSupp"] == 600 - 30 - 10
    assert caps[-1]["gear"] == "drive"


class TestTheGearIsSampled:
  def test_experimental_request_samples_the_gear_and_the_raw_speed(self, monkeypatch):
    c = _make_controller(monkeypatch)
    c.params = None
    cs = car.CarState.new_message()
    cs.gearShifter = "park"
    cs.vEgo = 0.0
    monkeypatch.setattr(m.time, "monotonic", lambda: 1000.0)
    monkeypatch.setattr(c, "_read_params", lambda: None, raising=False)
    monkeypatch.setattr(c, "_curve_peak_step", lambda *a: None, raising=False)
    monkeypatch.setattr(c, "_green_light_step", lambda *a: None, raising=False)
    monkeypatch.setattr(c, "_steer_event_step", lambda: None, raising=False)
    c._last_mode = "off"
    c._sm = type("S", (), {"reset": lambda s: None})()
    c._ces2 = type("S", (), {"reset": lambda s: None})()
    c.experimental_request(cs, {})
    assert c._gear == GearShifter.park
    assert c._gear_name == "park"
    assert c._v_ego_raw == 0.0

  def test_an_object_with_no_gearshifter_samples_as_none(self, monkeypatch):
    c = _make_controller(monkeypatch)
    monkeypatch.setattr(m.time, "monotonic", lambda: 1000.0)
    for name in ("_read_params", "_curve_peak_step", "_green_light_step", "_steer_event_step"):
      monkeypatch.setattr(c, name, lambda *a: None, raising=False)
    c._last_mode = "off"
    c._sm = type("S", (), {"reset": lambda s: None})()
    c._ces2 = type("S", (), {"reset": lambda s: None})()
    c.experimental_request(object(), {})
    assert c._gear is None and c._gear_name is None and c._v_ego_raw is None


class TestTheKillSwitchIsRead:
  class _P:
    def __init__(self, val=False, boom=None):
      self.val, self.boom, self.reads = val, boom, 0

    def get_bool(self, k):
      assert k == "RecordWhileParked"
      self.reads += 1
      if self.boom:
        raise self.boom
      return self.val

    def get(self, *a, **kw):
      return None

  def _read(self, monkeypatch, params):
    c = _make_controller(monkeypatch)
    c.params = params
    c._frame = 0
    c._veh = type("V", (), {"set_rain_tier": lambda s, v: None})()
    c._long_ok = c._shadow = False
    c._rain_err_t, c._rain_err_n = None, 0
    c._rwp_err_t, c._rwp_err_n = None, 0
    c._park_gate_on = False      # as a real __init__ seeds it: never read the switch yet -> gate open
    monkeypatch.setattr(m.C, "read_ces_mode", lambda *a, **kw: 0)
    monkeypatch.setattr(c, "_set_mode", lambda v: None, raising=False)
    c._read_params()
    return c

  def test_the_gate_is_on_when_the_switch_is_off(self, monkeypatch):
    c = self._read(monkeypatch, self._P(False))
    assert c._park_gate_on is True

  def test_the_gate_is_off_when_the_driver_asks_to_record_in_park(self, monkeypatch):
    c = self._read(monkeypatch, self._P(True))
    assert c._park_gate_on is False

  def test_the_switch_is_read_even_when_ces_is_off(self, monkeypatch):
    """The CES-off "steer" breadcrumb is gated too, so the switch cannot live behind `if _enabled`."""
    p = self._P(False)
    self._read(monkeypatch, p)
    assert p.reads == 1

  def test_an_unreadable_switch_keeps_logging_and_says_so(self, monkeypatch):
    said = []
    monkeypatch.setattr(m.cloudlog, "exception", lambda msg: said.append(msg))
    c = self._read(monkeypatch, self._P(boom=RuntimeError("UnknownKeyName")))
    assert c._park_gate_on is False, "an unread kill switch must not cost telemetry"
    assert len(said) == 1 and "RecordWhileParked" in said[0]

  def test_an_unreadable_switch_holds_the_last_good_value(self, monkeypatch):
    monkeypatch.setattr(m.cloudlog, "exception", lambda msg: None)
    p = self._P(False)
    c = self._read(monkeypatch, p)
    assert c._park_gate_on is True
    p.boom = RuntimeError("boom")
    c._frame = 0
    c._read_params()
    assert c._park_gate_on is True


# =====================================================================================================
# 6. the measurement this feature exists for
# =====================================================================================================
def test_the_gate_turns_a_parked_day_into_a_handful_of_records():
  """24 h of parked-and-charging at 1 Hz: 86,400 records become 1,470 -- a 98.3% cut of the parked
  content, with the truck's presence still visible once a minute."""
  gate, out = run(gear("park"), 24 * 3600)
  written = len(out) - out.count(SUPPRESS)
  assert written == 30 + 24 * 60
  assert written / len(out) < 0.018


def test_a_full_day_of_driving_loses_nothing():
  _, out = run(gear("drive"), 24 * 3600, v_ego=25.0)
  assert out.count(SUPPRESS) == 0


def test_a_fresh_controller_starts_with_the_gate_open():
  """Rule 2: until RecordWhileParked has actually been read, the log behaves exactly as before."""
  c = m.CESController.__new__(m.CESController)
  src = m.CESController.__init__
  import inspect
  body = inspect.getsource(src)
  assert "self._park_gate_on = False" in body, "the gate must not default to ON before the first read"
  assert "self._park_gate = ParkTickGate()" in body
  del c


# =====================================================================================================
# 7. the PRODUCTION call site: _publish_status's CES-on "tick" record
#
# Fable review 2026-09-19 (finding 1): the first version of this file covered only _steer_log_step.
# On the Lightning CES runs in shadow, so `_enabled` is True and the ~1 Hz breadcrumb comes from
# _publish_status -- the 39,983 `ev:tick … stopLatch` rows in the archive were written HERE. That
# branch had no suppression coverage at all, and a broken harness was reporting its mutants as killed.
# =====================================================================================================
def _publish_controller(monkeypatch):
  """A controller stub that drives the REAL _publish_status tick branch and captures what it appends."""
  cls = m.CESController

  class Stub:
    def __getattr__(self, n):
      return None

  c = Stub()
  c.mem_params = None                     # skip the overlay publish; the record path is what is under test
  c._shadow = False                       # keep the icbm overlay block out of this test's way
  c._last_mode = "chill"                  # equal to the mode _publish_status computes -> the TICK branch
  c._tick_last = -1e9
  c._vtsc_tele = {}
  c._sa_tele = {}
  c._map_targets = []
  c._speed_limit = 11.2
  c._button = C.BTN_CES
  c._ces2_urg = 0.0
  c._ces2_div = type("D", (), {"count": 0})()
  c._gl = type("G", (), {"state": None, "status": lambda s: None})()
  c._sm = type("S", (), {"state": None, "status": lambda s: None})()
  c._icbm_floor_lim = 0.0
  c._icbm_floor_hit = False
  # _event_record float()s these unconditionally (the permissive __getattr__ -> None would blow up
  # inside the record, which is exactly the failure test_ces_record_fields' stub exists to avoid).
  c._icbm_k = c._icbm_k_dist = c._icbm_k_v = 0.0
  c._icbm_k_n = 0
  c._icbm_k_ahead = False
  c._icbm_k_at = c._icbm_k_at_d = c._icbm_k_at_gap = 0.0
  c._icbm_k_at_n = 0
  c._curve_peak = m.CurvePeak()
  c._gear_name = None
  c._gear = c._v_ego_raw = None
  c._park_gate = ParkTickGate()
  c._park_gate_on = True
  c._park_err_t, c._park_err_n = None, 0
  c.captured = []
  c._append_event = c.captured.append
  c._event_record = cls._event_record.__get__(c)
  c._park_decision = cls._park_decision.__get__(c)
  c._publish = cls._publish_status.__get__(c)
  return c


def _run_publish(monkeypatch, c, secs, g="drive", v_ego=0.0, t0=1000.0, hz=1.0):
  cs = car.CarState.new_message()
  cs.gearShifter = g
  for i in range(int(secs * hz)):
    monkeypatch.setattr(m.time, "monotonic", lambda t=t0 + i / hz: t)
    c._gear, c._gear_name, c._v_ego_raw = cs.gearShifter, str(cs.gearShifter), v_ego
    c._publish(None, False)
  return c.captured


class TestTheProductionTickPath:
  def test_an_hour_at_a_red_light_logs_every_tick(self, monkeypatch):
    c = _publish_controller(monkeypatch)
    caps = _run_publish(monkeypatch, c, 3600, g="drive", v_ego=0.0)
    assert len(caps) == 3600
    assert all(r["ev"] == "tick" and r["gear"] == "drive" and r["park"] is False for r in caps)
    assert all("parkGate" not in r for r in caps)

  def test_ten_minutes_parked_collapses_to_the_debounce_plus_one_per_minute(self, monkeypatch):
    c = _publish_controller(monkeypatch)
    caps = _run_publish(monkeypatch, c, 600, g="park", v_ego=0.0)
    assert len(caps) == 30 + 10
    assert sum(1 for r in caps if r.get("parkGate") == "hold") == 10
    assert caps[30]["parkGate"] == "hold" and caps[30]["parkSupp"] == 0
    assert caps[-1]["parkSupp"] == 60 * 9 - 9

  def test_every_suppressed_record_is_accounted_for_in_the_log_itself(self, monkeypatch):
    """Rule 2 end-to-end: reading ONLY the jsonl, the 600 s window reconstructs to 600 seconds."""
    c = _publish_controller(monkeypatch)
    n_parked = len(_run_publish(monkeypatch, c, 600, g="park", v_ego=0.0))
    _run_publish(monkeypatch, c, 1, g="drive", v_ego=0.0, t0=1600.0)
    rel = c.captured[-1]
    assert rel["parkGate"] == "release"
    assert n_parked == 40                                 # 30 debounce + 10 heartbeats
    assert n_parked + rel["parkSupp"] + 1 == 601          # every one of the 601 ticks is accounted for

  def test_the_record_is_marked_parked_while_holding(self, monkeypatch):
    c = _publish_controller(monkeypatch)
    caps = _run_publish(monkeypatch, c, 600, g="park", v_ego=0.0)
    assert all(r["gear"] == "park" and r["park"] is True for r in caps)

  def test_the_kill_switch_restores_full_rate_on_the_production_path(self, monkeypatch):
    c = _publish_controller(monkeypatch)
    c._park_gate_on = False
    caps = _run_publish(monkeypatch, c, 600, g="park", v_ego=0.0)
    assert len(caps) == 600
    assert all(r["park"] is True for r in caps)      # still says the truck is parked, just keeps logging

  def test_a_moving_truck_whose_gear_reads_park_is_never_suppressed_here_either(self, monkeypatch):
    c = _publish_controller(monkeypatch)
    caps = _run_publish(monkeypatch, c, 600, g="park", v_ego=25.0)
    assert len(caps) == 600
    assert all(r["gear"] == "park" and r["park"] is False for r in caps)

  def test_an_adopt_record_is_never_gated(self, monkeypatch):
    """A mode transition during a hold still writes -- edge records are deliberately ungated."""
    c = _publish_controller(monkeypatch)
    _run_publish(monkeypatch, c, 600, g="park", v_ego=0.0)
    n = len(c.captured)
    monkeypatch.setattr(m.time, "monotonic", lambda: 1600.0)
    c._publish(None, True)                            # chill -> experimental while parked
    assert len(c.captured) == n + 1
    assert c.captured[-1]["ev"] == "adopt" and c.captured[-1]["park"] is True
    assert "parkGate" not in c.captured[-1]


class TestTheGateCanNeverReachControl:
  """Fable finding 3: _park_decision is called OUTSIDE _steer_log_step's try, so a raise would hit
  selfdrived's guard around experimental_request() -- which forces CES to Chill. A logging decision
  must not be able to change what the car does."""

  def _boom(self, monkeypatch, c):
    monkeypatch.setattr(c._park_gate, "update",
                        lambda *a, **kw: (_ for _ in ()).throw(RuntimeError("gate exploded")))

  def test_a_raising_gate_fails_open_and_never_propagates(self, monkeypatch):
    c = _publish_controller(monkeypatch)
    self._boom(monkeypatch, c)
    said = []
    monkeypatch.setattr(m.cloudlog, "exception", lambda msg: said.append(msg))
    caps = _run_publish(monkeypatch, c, 120, g="park", v_ego=0.0)
    assert len(caps) == 120, "a broken gate must keep the full-rate log, not suppress it"

  def test_a_raising_gate_says_so_but_does_not_flood(self, monkeypatch):
    c = _publish_controller(monkeypatch)
    self._boom(monkeypatch, c)
    said = []
    monkeypatch.setattr(m.cloudlog, "exception", lambda msg: said.append(msg))
    _run_publish(monkeypatch, c, 600, g="park", v_ego=0.0)
    assert 1 <= len(said) <= 12, f"expected ~1 line per 60 s over 600 s, got {len(said)}"
    assert "park gate raised" in said[0]

  def test_the_steer_path_is_wrapped_too(self, monkeypatch):
    c = _make_controller(monkeypatch)
    monkeypatch.setattr(c._park_gate, "update",
                        lambda *a, **kw: (_ for _ in ()).throw(RuntimeError("gate exploded")))
    monkeypatch.setattr(m.cloudlog, "exception", lambda msg: None)
    caps = _run_steer_log_step(monkeypatch, secs=120, g="park", v_ego=0.0, c=c)
    assert len(caps) == 120

  def test_both_writers_go_through_the_one_wrapper(self):
    """Structural: neither call site may talk to the gate directly (that is what made the argument
    mutants survive at one site while the other was covered)."""
    import inspect
    for meth in (m.CESController._steer_log_step, m.CESController._publish_status):
      src = inspect.getsource(meth)
      assert "_park_decision(" in src
      assert "_park_gate.update(" not in src


# =====================================================================================================
# 9. the gate lives INSIDE each writer's 1 Hz throttle, not above it
#
# Fable round 2 (finding 2): both harnesses above drive the writers at exactly 1 Hz, so nothing
# pinned that `_park_decision` is evaluated once per RECORD rather than once per 100 Hz control tick.
# Two mutants that moved it outside the throttle survived all 1,036 tests. They are not defects in the
# shipped code -- the gate is correctly inside both throttles -- but a refactor making either mistake
# would inflate parkSupp ~100x and silently break the jsonl accounting invariant in section 7.
# These two run the REAL writers at the REAL 100 Hz selfdrived rate.
# =====================================================================================================
class TestTheGateRunsOncePerRecordNotOncePerTick:
  def test_the_tick_writer_counts_records_not_control_ticks(self, monkeypatch):
    c = _publish_controller(monkeypatch)
    caps = _run_publish(monkeypatch, c, 600, g="park", v_ego=0.0, hz=100.0)
    assert len(caps) == 40                                  # 30 debounce + 10 heartbeats, as at 1 Hz
    _run_publish(monkeypatch, c, 1, g="drive", v_ego=0.0, t0=1600.0, hz=100.0)
    rel = c.captured[-1]
    assert rel["parkGate"] == "release"
    assert rel["parkSupp"] == 560, "parkSupp must count SUPPRESSED RECORDS, not 100 Hz control ticks"

  def test_the_steer_writer_counts_records_not_control_ticks(self, monkeypatch):
    c = _make_controller(monkeypatch)
    caps = _run_steer_log_step(monkeypatch, secs=600, g="park", v_ego=0.0, c=c, hz=100.0)
    assert len(caps) == 40
    _run_steer_log_step(monkeypatch, secs=1, g="drive", v_ego=0.0, c=c, t0=1600.0, hz=100.0)
    assert caps[-1]["parkGate"] == "release" and caps[-1]["parkSupp"] == 560
