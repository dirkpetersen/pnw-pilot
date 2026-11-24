"""fsdguard2pnw: on the Tesla Raven the comma engages only with stock Autosteer/FSD off (TACC only), unless the
TeslaCommaOverridesFsd toggle is ON. carState.teslaAutopilot (opendbc carstate) -> selfdrived.fsd_guard_events -> the real
StateMachine and the real alert texts."""
import pytest

from cereal import car
from openpilot.selfdrive.selfdrived.events import ET, EventName, EventNamePnw, Events
from openpilot.selfdrive.selfdrived.mads_pnw import has_blocking_event
from openpilot.selfdrive.selfdrived.selfdrived import fsd_guard_events
from openpilot.selfdrive.selfdrived.state import State, StateMachine

TA = car.CarState.TeslaAutopilot
BLOCKED = [TA.selected, TA.stockSteering, TA.missing]


def events_for(state, overrides=False, extra=()):
  ev = Events()
  for e in fsd_guard_events(state, overrides) + list(extra):
    ev.add(e)
  return ev


def try_engage(state, overrides=False):
  sm = StateMachine()
  sm.update(events_for(state, overrides, [EventName.pcmEnable]))
  return sm


def alert_texts(ev, types, state):
  cs = car.CarState.new_message(teslaAutopilot=state)
  return [(a.alert_text_1, a.alert_text_2) for a in ev.create_alerts(types, [None, cs, None, False, 0, None])]


def test_tacc_only_engages():
  assert try_engage(TA.taccOnly).state == State.enabled


@pytest.mark.parametrize("state", BLOCKED)
def test_anything_else_is_no_entry(state):
  sm = try_engage(state)
  assert sm.state == State.disabled
  assert ET.NO_ENTRY in sm.current_alert_types


@pytest.mark.parametrize("state", BLOCKED + [TA.taccOnly, TA.unavailable])
def test_toggle_on_is_the_old_behaviour(state):
  assert fsd_guard_events(state, True) == []
  assert try_engage(state, overrides=True).state == State.enabled


def test_other_cars_are_untouched():
  assert fsd_guard_events(TA.unavailable, False) == []


def test_owner_wording_and_missing_says_missing():
  ev = events_for(TA.selected)
  assert alert_texts(ev, [ET.NO_ENTRY], TA.selected) == [
    ("FSD/Autosteer selected", "Unplug comma to use FSD, or select TACC only for comma")]
  assert alert_texts(events_for(TA.missing), [ET.NO_ENTRY], TA.missing) == [("FSD/Autosteer selected", "Autopilot status missing")]


def test_engaged_then_stock_starts_warns_and_keeps_steering():
  sm = try_engage(TA.taccOnly)
  ev = events_for(TA.stockSteering)
  for _ in range(500):                       # 5 s of stock steering while engaged: no disengage, no soft-disable
    enabled, active = sm.update(ev)
    assert (enabled, active) == (True, True) and sm.state == State.enabled
  assert ET.WARNING in sm.current_alert_types
  texts = alert_texts(ev, sm.current_alert_types, TA.stockSteering)
  assert ("The comma is steering, not FSD", "Stock Autopilot started; its steering is blocked") in texts
  assert not has_blocking_event(ev), "MADS lateral-only must not be dropped either"


def test_not_latched():
  sm = try_engage(TA.taccOnly)
  sm.update(events_for(TA.stockSteering))
  assert fsd_guard_events(TA.taccOnly, False) == []
  sm.state = State.disabled
  sm.update(events_for(TA.taccOnly, extra=[EventName.pcmEnable]))
  assert sm.state == State.enabled


def test_stock_warning_only_while_stock_steers():
  assert EventNamePnw.teslaStockSteering in fsd_guard_events(TA.stockSteering, False)
  for s in (TA.selected, TA.missing):
    assert fsd_guard_events(s, False) == [EventNamePnw.teslaFsdSelected]


@pytest.mark.parametrize("overrides, expected", [(False, True), (True, False)])
def test_selfdrived_wiring(overrides, expected):
  """The real SelfdriveD: the toggle is read at start and the guard event reaches self.events from update_events."""
  from opendbc.car import structs
  from openpilot.common.params import Params
  from openpilot.common.prefix import OpenpilotPrefix
  from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD
  with OpenpilotPrefix():
    Params().put_bool("TeslaCommaOverridesFsd", overrides)
    cp = structs.CarParams.new_message(brand="tesla", carFingerprint="TESLA_MODEL_S_HW3")
    sd = SelfdriveD(CP=cp)
    assert sd.comma_overrides_fsd is overrides
    sd.initialized = True
    sd.update_events(car.CarState.new_message(teslaAutopilot=TA.selected, canValid=False))
    assert (EventNamePnw.teslaFsdSelected in sd.events.names) is expected
