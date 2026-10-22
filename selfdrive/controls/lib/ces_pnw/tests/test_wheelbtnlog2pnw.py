"""wheelbtnlog2pnw: the driver's steering-wheel buttons reach the ces_events tick (wb* fields).

drives/2026-10-21/corvallis-portland-i5: the driver held SET+ for 4.4 s (85 mph in a 65), and no tick field said so.
Drives the REAL tick writer (_publish_status) at 100 Hz with real capnp carStates.
"""
import math
from types import SimpleNamespace

from cereal import car
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_parkgate2pnw import _publish_controller
from openpilot.selfdrive.controls.lib.wheel_buttons_pnw import WheelButtons

BT = car.CarState.ButtonEvent.Type


def _cs(*events):
  cs = car.CarState.new_message()
  cs.gearShifter = "drive"
  cs.buttonEvents = [{"type": t, "pressed": p} for t, p in events]
  return cs.as_reader()


def _controller(monkeypatch, rocker=()):
  c = _publish_controller(monkeypatch)
  c._wb = WheelButtons()
  c._wb_cs = None
  c._wb_since = 1000.0
  c._wb_rock_t = -math.inf
  c._wb_err = 0
  c._veh = SimpleNamespace(wheel_rocker=rocker)
  c._gear = c._gear_name = "drive"
  c._tick_last = 1000.0          # first tick ~1 s in, like a drive already under way
  return c


def _drive(monkeypatch, c, frames, t0=1000.0):
  """frames: one carState per 10 ms; the tick writer runs every frame and writes a record ~1 Hz."""
  for i, cs in enumerate(frames):
    t = t0 + (i + 1) * 0.01
    monkeypatch.setattr(m.time, "monotonic", lambda t=t: t)
    m._wb_feed(c, cs, t)
    c._publish(None, False)
  return [r for r in c.captured if r["ev"] == "tick"]


def _set_plus_held(seconds=4.0):
  idle = _cs()
  return [idle] * 50 + [_cs((BT.accelCruise, True))] + [idle] * int(seconds * 100 - 1) + [_cs((BT.accelCruise, False))] + [idle] * 150


class TestTheTickNamesTheDriversButtons:
  def test_set_plus_held_four_seconds(self, monkeypatch):
    c = _controller(monkeypatch)
    ticks = _drive(monkeypatch, c, _set_plus_held())
    first = ticks[0]
    assert first["wbN"] == 1 and first["wbLast"] == "accelCruise" and first["wbHeld"] == "accelCruise"
    held = [r for r in ticks if r["wbHeld"] == "accelCruise"]
    assert len(held) == 4, "SET+ held 4 s must read as held on 4 consecutive ~1 Hz ticks"
    assert all(r["wbN"] == 0 for r in held[1:]), "a held button is ONE press, not one per tick"
    assert held[-1]["wbAge"] > 3.0
    assert ticks[-1]["wbHeld"] is None and ticks[-1]["wbLast"] == "accelCruise"

  def test_a_stale_carstate_is_not_counted_twice(self, monkeypatch):
    c = _controller(monkeypatch)
    press = _cs((BT.decelCruise, True))
    ticks = _drive(monkeypatch, c, [press] * 5 + [_cs((BT.decelCruise, False))] + [_cs()] * 100)
    assert ticks[0]["wbN"] == 1 and ticks[0]["wbLast"] == "decelCruise"

  def test_no_press_is_none_not_a_fake_button(self, monkeypatch):
    c = _controller(monkeypatch)
    ticks = _drive(monkeypatch, c, [_cs()] * 150)
    assert ticks[0]["wbLast"] is None and ticks[0]["wbN"] == 0 and ticks[0]["wbHeld"] is None
    assert "wbRockWhy" not in ticks[0], "a car with no rocker capability must not report a missing rocker"

  def test_ces_button_is_named_cesBtn(self, monkeypatch):
    ticks = _drive(monkeypatch, _controller(monkeypatch), [_cs()] * 110)
    assert ticks[0]["cesBtn"] == ticks[0]["button"] == 0


class _Mem:
  def __init__(self, value):
    self.value = value

  def get(self, k, return_default=False):
    assert k == "WheelRocker"
    if isinstance(self.value, Exception):
      raise self.value
    return self.value


ROCKER = (("pt", "Steering_Data_FD1", "CcAslButtnCnclResPress", "cnclRes"),)


class TestTheRockerFromCard:
  def _ctl(self, monkeypatch, value):
    c = _controller(monkeypatch, rocker=ROCKER)
    c.mem_params = _Mem(value)
    c._tele_last = math.inf                              # keep the 5 Hz overlay publish out of the way
    return c

  def test_a_rocker_press_counts_and_is_named(self, monkeypatch):
    c = self._ctl(monkeypatch, {"edges": [[1000.3, "cnclRes"]], "held": [], "ts": 1000.35})
    ticks = _drive(monkeypatch, c, [_cs()] * 110)
    assert ticks[0]["wbN"] == 1 and ticks[0]["wbLast"] == "cnclRes"
    assert abs(ticks[0]["wbAge"] - 0.7) < 0.02
    assert "wbRockWhy" not in ticks[0]

  def test_a_rocker_edge_published_after_the_tick_is_counted_on_the_next(self, monkeypatch):
    c = self._ctl(monkeypatch, {"edges": [], "held": [], "ts": 0.0})
    _drive(monkeypatch, c, [_cs()] * 100)                                 # tick at 1001.0 saw nothing
    c.mem_params.value = {"edges": [[1000.99, "cnclRes"]], "held": [], "ts": 1001.02}
    ticks = _drive(monkeypatch, c, [_cs()] * 100, t0=1001.0)
    assert ticks[-1]["wbN"] == 1, "a press timed before the previous tick but published after it was lost"

  def test_no_rocker_state_says_why(self, monkeypatch):
    ticks = _drive(monkeypatch, self._ctl(monkeypatch, None), [_cs()] * 110)
    assert "card's rocker reader is not running" in ticks[0]["wbRockWhy"]

  def test_a_failing_read_says_why(self, monkeypatch):
    ticks = _drive(monkeypatch, self._ctl(monkeypatch, OSError("boom")), [_cs()] * 110)
    assert ticks[0]["wbRockWhy"].startswith("WheelRocker read failed: OSError")


def test_the_real_controller_feeds_buttons_from_experimental_request():
  """The wiring, not just the helper: selfdrived's one call reaches the tracker, once per carState message."""
  from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_curvelead2pnw import FakeCP
  c = m.CESController(FakeCP("TOYOTA_COROLLA", "toyota", False))
  cs = _cs((BT.accelCruise, True))
  c.experimental_request(cs, {})
  c.experimental_request(cs, {})          # selfdrived's CS_prev re-pass after a carState timeout
  assert [n for _, n in c._wb.edges] == ["accelCruise"] and c._wb_err == 0


def test_the_ces_off_steer_breadcrumb_carries_them_too(monkeypatch):
  from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_parkgate2pnw import _make_controller
  c = _make_controller(monkeypatch, _wb=WheelButtons(), _wb_cs=None, _wb_since=999.0, _wb_rock_t=-math.inf,
                       _wb_err=0, _veh=SimpleNamespace(wheel_rocker=()))
  m._wb_feed(c, _cs((BT.resumeCruise, True)), 999.5)
  monkeypatch.setattr(m.time, "monotonic", lambda: 1000.0)
  c._gear, c._gear_name, c._v_ego_raw = "drive", "drive", 20.0
  c._steer_log_step(_cs(), {"radarState": SimpleNamespace(leadOne=SimpleNamespace(status=False))})
  (rec,) = c.captured
  assert rec["ev"] == "steer" and rec["wbLast"] == "resumeCruise" and rec["wbN"] == 1 and rec["wbHeld"] == "resumeCruise"


def test_an_adopt_record_carries_the_fields_and_drains_the_window(monkeypatch):
  """Review fix 1: a press just before a chill<->experimental change lands in the adopt record, once."""
  c = _controller(monkeypatch)
  c._last_mode = "experimental"           # _publish computes "chill" -> the next record is an adopt
  m._wb_feed(c, _cs((BT.decelCruise, True)), 1000.5)
  monkeypatch.setattr(m.time, "monotonic", lambda: 1000.6)
  c._publish(None, False)
  (adopt,) = c.captured
  assert adopt["ev"] == "adopt" and adopt["wbN"] == 1 and adopt["wbLast"] == "decelCruise"
  ticks = _drive(monkeypatch, c, [_cs((BT.decelCruise, False))] + [_cs()] * 150, t0=1000.6)
  assert ticks and ticks[0]["wbN"] == 0, "the adopt record already reported that press"


def test_held_is_the_newest_press_across_both_sources(monkeypatch):
  """Review fix 2: SET+ held since 1000.2, the rocker pressed at 1000.5 and still down -> wbHeld is the rocker."""
  c = _controller(monkeypatch, rocker=ROCKER)
  c.mem_params = _Mem({"edges": [[1000.5, "cnclRes"]], "held": ["cnclRes"], "ts": 1000.5})
  c._tele_last = math.inf
  m._wb_feed(c, _cs((BT.accelCruise, True)), 1000.2)
  ticks = _drive(monkeypatch, c, [_cs()] * 110)
  assert ticks[0]["wbHeld"] == "cnclRes" and ticks[0]["wbN"] == 2


def test_held_unit_rocker_without_an_edge_still_counts():
  wb = WheelButtons()
  wb.feed(10.0, [("accelCruise", True)])
  assert wb.fields(11.0, 9.0, ["cnclRes"])["wbHeld"] == "accelCruise"   # undated extra loses to a dated press
  assert WheelButtons().fields(11.0, 9.0, ["cnclRes"])["wbHeld"] == "cnclRes"
