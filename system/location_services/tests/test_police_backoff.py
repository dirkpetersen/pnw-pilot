"""policebackoff2pnw: the police-poll backoff escalation rule.

The behaviour under test is deliberately asymmetric: a transient failure (tunnel / LTE handover /
momentary no-signal) must NOT cost the driver the fast poll cadence on the highway, but a policy
denial (402 budget / 429 daily cap) must stop hammering a cap that is already spent.
"""
from openpilot.system.location_services.location_servicesd import (
  POLICE_MAX_BACKOFF_S,
  POLICE_POLL_S,
  POLICE_TRANSIENT_FAILS_BEFORE_BACKOFF as THRESH,
  next_police_backoff,
)


def test_isolated_blips_never_slow_the_cadence():
  """The whole point: a single dropped poll, then a success, must leave the interval untouched."""
  backoff, consec = POLICE_POLL_S, 0
  for _ in range(20):
    backoff, consec = next_police_backoff(backoff, consec, False)   # blip
    assert backoff == POLICE_POLL_S
    backoff, consec = POLICE_POLL_S, 0                              # success resets both


def test_tolerates_a_short_run_then_escalates():
  backoff, consec = POLICE_POLL_S, 0
  for i in range(1, THRESH):
    backoff, consec = next_police_backoff(backoff, consec, False)
    assert backoff == POLICE_POLL_S, f"escalated too early at failure {i}"
    assert consec == i
  # the Nth consecutive failure is the one that concedes the service is down
  backoff, consec = next_police_backoff(backoff, consec, False)
  assert consec == THRESH
  assert backoff == POLICE_POLL_S * 2


def test_sustained_outage_reaches_and_holds_the_ceiling():
  backoff, consec = POLICE_POLL_S, 0
  for _ in range(50):
    backoff, consec = next_police_backoff(backoff, consec, False)
  assert backoff == POLICE_MAX_BACKOFF_S
  backoff, consec = next_police_backoff(backoff, consec, False)
  assert backoff == POLICE_MAX_BACKOFF_S, "backoff grew past the ceiling"


def test_policy_denial_goes_straight_to_max_on_the_first_one():
  backoff, consec = next_police_backoff(POLICE_POLL_S, 0, True)
  assert backoff == POLICE_MAX_BACKOFF_S
  assert consec == THRESH, "a denial must not leave the streak short of the threshold"


def test_denial_then_transient_does_not_drop_back_to_fast_polling():
  """Regression guard: after a cap is hit, a following ordinary failure must not reset the cadence
  to 60s just because its own streak looks short."""
  backoff, consec = next_police_backoff(POLICE_POLL_S, 0, True)
  backoff, consec = next_police_backoff(backoff, consec, False)
  assert backoff == POLICE_MAX_BACKOFF_S


def test_success_after_escalation_returns_to_normal():
  backoff, consec = POLICE_POLL_S, 0
  for _ in range(10):
    backoff, consec = next_police_backoff(backoff, consec, False)
  assert backoff > POLICE_POLL_S
  backoff, consec = POLICE_POLL_S, 0          # what run() does on a successful poll
  backoff, consec = next_police_backoff(backoff, consec, False)
  assert backoff == POLICE_POLL_S, "a recovered service did not get the fast cadence back"


def test_never_returns_a_sub_normal_interval():
  """Defensive: even fed a nonsense low current interval, the rule never polls faster than the
  designed 1/min ceiling on a PAID upstream call."""
  for cur in (0.0, 1.0, POLICE_POLL_S / 2):
    for consec in range(THRESH + 3):
      backoff, _ = next_police_backoff(cur, consec, False)
      assert backoff >= POLICE_POLL_S, f"would poll faster than 1/min: {backoff}"
