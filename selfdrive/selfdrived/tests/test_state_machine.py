import pytest

from cereal import log
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.selfdrived.state import StateMachine, SOFT_DISABLE_TIME
from openpilot.selfdrive.selfdrived.events import Events, ET, EVENTS, NormalPermanentAlert

State = log.SelfdriveState.OpenpilotState

# The event types that maintain the current state
MAINTAIN_STATES = {State.enabled: (None,), State.disabled: (None,), State.softDisabling: (ET.SOFT_DISABLE,),
                   State.preEnabled: (ET.PRE_ENABLE,), State.overriding: (ET.OVERRIDE_LATERAL, ET.OVERRIDE_LONGITUDINAL)}
ALL_STATES = tuple(State.schema.enumerants.values())
# The event types checked in DISABLED section of state machine
ENABLE_EVENT_TYPES = (ET.ENABLE, ET.PRE_ENABLE, ET.OVERRIDE_LATERAL, ET.OVERRIDE_LONGITUDINAL)


# `EVENTS[0]` -- which this used to use -- IS a real event: EventName.canError == 0. Registering the
# synthetic event there permanently overwrote canError's real definition for the rest of the process,
# so any later test in the same worker that exercised canError got this stub instead. Our
# test_mads_pnw.py parametrises over EventName.canError, so it failed only when xdist happened to put
# the two files in the same worker -- intermittently, and pointing at MADS rather than at here.
#
# Found 2026-09-19 by scripts/check-channel-tip.sh (CLAUDE.md Rule 9); upstream file, upstream bug.
# Use a key that cannot be an EventName, and clean it up so nothing leaks past this module.
SYNTHETIC_EVENT = max(EVENTS) + 1000


def make_event(event_types):
  event = {}
  for ev in event_types:
    event[ev] = NormalPermanentAlert("alert")
  EVENTS[SYNTHETIC_EVENT] = event
  return SYNTHETIC_EVENT


@pytest.fixture(autouse=True)
def _drop_synthetic_event():
  """Remove the synthetic entry after every test, so EVENTS is exactly as this module found it.

  Without this the table stays mutated for the rest of the worker -- harmless at a key nothing else
  reads, but the whole point of the fix is that "nothing else reads it" was the assumption that was
  false the first time."""
  yield
  EVENTS.pop(SYNTHETIC_EVENT, None)


class TestStateMachine:
  def setup_method(self):
    self.events = Events()
    self.state_machine = StateMachine()
    self.state_machine.soft_disable_timer = int(SOFT_DISABLE_TIME / DT_CTRL)

  def test_immediate_disable(self):
    for state in ALL_STATES:
      for et in MAINTAIN_STATES[state]:
        self.events.add(make_event([et, ET.IMMEDIATE_DISABLE]))
        self.state_machine.state = state
        self.state_machine.update(self.events)
        assert State.disabled == self.state_machine.state
        self.events.clear()

  def test_user_disable(self):
    for state in ALL_STATES:
      for et in MAINTAIN_STATES[state]:
        self.events.add(make_event([et, ET.USER_DISABLE]))
        self.state_machine.state = state
        self.state_machine.update(self.events)
        assert State.disabled == self.state_machine.state
        self.events.clear()

  def test_soft_disable(self):
    for state in ALL_STATES:
      if state == State.preEnabled:  # preEnabled considers NO_ENTRY instead
        continue
      for et in MAINTAIN_STATES[state]:
        self.events.add(make_event([et, ET.SOFT_DISABLE]))
        self.state_machine.state = state
        self.state_machine.update(self.events)
        assert self.state_machine.state == State.disabled if state == State.disabled else State.softDisabling
        self.events.clear()

  def test_soft_disable_timer(self):
    self.state_machine.state = State.enabled
    self.events.add(make_event([ET.SOFT_DISABLE]))
    self.state_machine.update(self.events)
    for _ in range(int(SOFT_DISABLE_TIME / DT_CTRL)):
      assert self.state_machine.state == State.softDisabling
      self.state_machine.update(self.events)

    assert self.state_machine.state == State.disabled

  def test_no_entry(self):
    # Make sure noEntry keeps us disabled
    for et in ENABLE_EVENT_TYPES:
      self.events.add(make_event([ET.NO_ENTRY, et]))
      self.state_machine.update(self.events)
      assert self.state_machine.state == State.disabled
      self.events.clear()

  def test_no_entry_pre_enable(self):
    # preEnabled with noEntry event
    self.state_machine.state = State.preEnabled
    self.events.add(make_event([ET.NO_ENTRY, ET.PRE_ENABLE]))
    self.state_machine.update(self.events)
    assert self.state_machine.state == State.preEnabled

  def test_maintain_states(self):
    # Given current state's event type, we should maintain state
    for state in ALL_STATES:
      for et in MAINTAIN_STATES[state]:
        self.state_machine.state = state
        self.events.add(make_event([et]))
        self.state_machine.update(self.events)
        assert self.state_machine.state == state
        self.events.clear()
