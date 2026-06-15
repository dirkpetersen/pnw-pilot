"""Unit tests for the pure LTE PDN-throttle backoff guard (network2xnor)."""
from openpilot.system.networkd.lte_guard import decide_lte_guard, next_backoff_s, BACKOFF_SCHEDULE_S


def test_backoff_schedule_escalates_and_caps():
  assert next_backoff_s(0) == BACKOFF_SCHEDULE_S[0]
  assert next_backoff_s(1) == BACKOFF_SCHEDULE_S[1]
  # beyond the schedule clamps at the longest step
  assert next_backoff_s(99) == BACKOFF_SCHEDULE_S[-1]
  # monotonic non-decreasing
  vals = [next_backoff_s(i) for i in range(6)]
  assert vals == sorted(vals)


def test_healthy_lte_is_noop_and_resets_count():
  # has IP, never throttled -> behavior-neutral, count reset to 0
  action, parked, until, count = decide_lte_guard(
    now=100.0, throttled=False, lte_has_ip=True, parked=False, parked_until=0.0, throttle_count=3)
  assert action == "noop"
  assert parked is False
  assert count == 0


def test_no_ip_no_throttle_does_not_interfere():
  # no IP but no throttle string -> let NM do its thing, don't park
  action, parked, _until, count = decide_lte_guard(
    now=100.0, throttled=False, lte_has_ip=False, parked=False, parked_until=0.0, throttle_count=0)
  assert action == "noop"
  assert parked is False
  assert count == 0


def test_throttle_trips_park_with_first_backoff():
  action, parked, until, count = decide_lte_guard(
    now=1000.0, throttled=True, lte_has_ip=False, parked=False, parked_until=0.0, throttle_count=0)
  assert action == "park"
  assert parked is True
  assert until == 1000.0 + BACKOFF_SCHEDULE_S[0]
  assert count == 1


def test_parked_stays_parked_until_deadline():
  action, parked, until, count = decide_lte_guard(
    now=1010.0, throttled=False, lte_has_ip=False, parked=True, parked_until=1030.0, throttle_count=1)
  assert action == "noop"
  assert parked is True
  assert until == 1030.0
  assert count == 1  # preserved


def test_parked_unparks_at_deadline():
  action, parked, _until, count = decide_lte_guard(
    now=1030.0, throttled=False, lte_has_ip=False, parked=True, parked_until=1030.0, throttle_count=1)
  assert action == "unpark"
  assert parked is False
  assert count == 1  # kept so a re-throttle escalates


def test_re_throttle_after_unpark_escalates_backoff():
  # came back, still throttled (count carried from a prior park) -> longer backoff
  action, parked, until, count = decide_lte_guard(
    now=2000.0, throttled=True, lte_has_ip=False, parked=False, parked_until=0.0, throttle_count=1)
  assert action == "park"
  assert until == 2000.0 + BACKOFF_SCHEDULE_S[1]  # escalated to the 2nd step
  assert count == 2


def test_recovery_after_unpark_resets_count():
  # unparked, then got an IP -> healthy -> count resets so the next incident starts fresh
  action, parked, _until, count = decide_lte_guard(
    now=2100.0, throttled=False, lte_has_ip=True, parked=False, parked_until=0.0, throttle_count=2)
  assert action == "noop"
  assert count == 0
