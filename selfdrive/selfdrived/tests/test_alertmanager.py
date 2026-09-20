import copy
import random

import pytest

from openpilot.selfdrive.selfdrived.events import Alert, EmptyAlert, EVENTS
from openpilot.selfdrive.selfdrived.alertmanager import AlertManager


@pytest.fixture(autouse=True)
def _events_table_must_be_left_alone():
  """Fail HERE if this file ever mutates the shared EVENTS alerts again.

  The bug this guards was silent by construction: the damage landed on a random OTHER test file in
  whichever xdist worker happened to run this one first, so the failure always named an innocent
  test. Same shape, and same remedy, as `_drop_synthetic_event` in test_state_machine.py.
  """
  before = {id(a): a.duration for d in EVENTS.values() for a in d.values() if isinstance(a, Alert)}
  yield
  after = {id(a): a.duration for d in EVENTS.values() for a in d.values() if isinstance(a, Alert)}
  changed = [k for k, v in after.items() if before.get(k) != v]
  assert not changed, f"{len(changed)} shared Alert(s) in EVENTS were mutated by this test. That poisons every later test in this worker process -- copy the alert before writing to it."  # noqa: E501


class TestAlertManager:

  def test_duration(self):
    """
      Enforce that an alert lasts for max(alert duration, duration the alert is added)
    """
    for duration in range(1, 100):
      alert = None
      while not isinstance(alert, Alert):
        event = random.choice([e for e in EVENTS.values() if len(e)])
        alert = random.choice(list(event.values()))

      # COPY FIRST -- EVENTS hands out SHARED Alert singletons. `Events.create_alerts()` returns
      # `EVENTS[e][et]` itself, so writing `.duration` on the object drawn above used to rewrite the
      # REAL alert table for the rest of this worker process, and nothing ever restored it. Measured
      # 2026-09-20: one run of this test clobbers ~61 of the 123 alerts in EVENTS with a random value
      # in 1..99. Which ones is random, so it poisons a DIFFERENT set of later tests every run -- the
      # exact profile of the flake this file caused (1 red, then 11 red, then seven greens on
      # identical code). The confirmed casualty was
      # test_mads_pnw.py::TestLateralMismatchDetector::test_event_actually_reaches_the_driver, which
      # checks that alert's real 400-frame duration and failed with "assert 73 >= 100".
      # Deterministic reproducer (before this line existed): running
      #   test_alertmanager.py test_mads_pnw.py
      # in one process failed while either file alone passed, and the reversed order passed.
      # AlertManager only READS the alert, so a shallow copy exercises byte-identical code here.
      alert = copy.copy(alert)
      alert.duration = duration

      # check two cases:
      # - alert is added to AM for <= the alert's duration
      # - alert is added to AM for > alert's duration
      for greater in (True, False):
        if greater:
          add_duration = duration + random.randint(1, 10)
        else:
          add_duration = random.randint(1, duration)
        show_duration = max(duration, add_duration)

        AM = AlertManager()
        for frame in range(duration+10):
          if frame < add_duration:
            AM.add_many(frame, [alert, ])
          AM.process_alerts(frame, set())

          shown = AM.current_alert != EmptyAlert
          should_show = frame <= show_duration
          assert shown == should_show, f"{frame=} {add_duration=} {duration=}"

      # check one case:
      # - if alert is re-added to AM before it ends the duration is extended
      if duration > 1:
        AM = AlertManager()
        show_duration = duration * 2
        for frame in range(duration * 2 + 10):
          if frame == 0:
            AM.add_many(frame, [alert, ])

          if frame == duration:
            # add alert one frame before it ends
            assert AM.current_alert == alert
            AM.add_many(frame, [alert, ])
          AM.process_alerts(frame, set())

          shown = AM.current_alert != EmptyAlert
          should_show = frame <= show_duration
          assert shown == should_show, f"{frame=} {duration=}"
