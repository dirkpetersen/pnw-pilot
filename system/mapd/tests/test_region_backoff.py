"""mapdgate2pnw: the whole-STATE download retry must be bounded.

mapd has no incremental fetch, so every retry re-downloads an entire US state. Before this, a region
that stayed uncovered was re-requested every REGION_RESEND_INTERVAL_S (60 s) with no attempt counter
and no cap. The real-world rate was bounded by the pull duration D rather than by any logic --
requests are suppressed while downloadProgress.active -- so it was ~3600/D pulls an hour, back to
back with no idle gap, up to a ceiling of 60/h when D < 60 s. The fix inserts real idle time.

The pure interval helper is the easy half. The bugs live in the state machine, so most of this file
tests RegionRetry against the three scenarios that actually defeated the first version:
  * a MOVING car (coverage returns between bad areas -- tileLoaded is per 0.25-degree area)
  * a pull that runs LONGER than the current interval
  * a region that alternates on a bbox edge (WA/OR through NE Portland)
"""
from openpilot.system.mapd.mapd_configd import (
  REGION_RESEND_INTERVAL_S,
  REGION_MAX_RESEND_INTERVAL_S,
  RegionRetry,
  next_region_interval,
)


class TestNextRegionInterval:
  def test_first_attempt_keeps_the_plain_interval(self):
    # The benign case: mapd missed the message because its mapdIn socket wasn't up. Penalising it
    # would make a cold start wait minutes for maps.
    assert next_region_interval(0) == REGION_RESEND_INTERVAL_S
    assert next_region_interval(1) == REGION_RESEND_INTERVAL_S

  def test_escalates_geometrically(self):
    assert next_region_interval(2) == 120.0
    assert next_region_interval(3) == 240.0
    assert next_region_interval(4) == 480.0
    assert next_region_interval(5) == 960.0

  def test_caps(self):
    assert next_region_interval(6) == REGION_MAX_RESEND_INTERVAL_S
    for attempts in range(6, 60):
      assert next_region_interval(attempts) == REGION_MAX_RESEND_INTERVAL_S

  def test_never_exceeds_cap_and_never_below_base(self):
    for attempts in range(200):
      assert REGION_RESEND_INTERVAL_S <= next_region_interval(attempts) <= REGION_MAX_RESEND_INTERVAL_S

  def test_monotonic_non_decreasing(self):
    prev = 0.0
    for attempts in range(60):
      interval = next_region_interval(attempts)
      assert interval >= prev
      prev = interval

  def test_negative_attempts_are_safe(self):
    assert next_region_interval(-1) == REGION_RESEND_INTERVAL_S
    assert next_region_interval(-100) == REGION_RESEND_INTERVAL_S

  def test_huge_attempt_count_does_not_overflow(self):
    # 2.0 ** 1024 raises OverflowError; the exponent is clamped. ~21 days uncovered reaches this.
    for attempts in (1024, 1025, 10_000, 10**6):
      assert next_region_interval(attempts) == REGION_MAX_RESEND_INTERVAL_S


class TestRegionRetry:
  def test_unseen_region_is_requested_immediately(self):
    r = RegionRetry()
    assert r.due("WA", 0.0)
    assert r.attempts("WA") == 0

  def test_record_counts_per_region(self):
    r = RegionRetry()
    assert r.record("WA", 0.0) == 1
    assert r.record("WA", 100.0) == 2
    assert r.attempts("WA") == 2
    assert r.attempts("OR") == 0          # independent

  def test_moving_car_regaining_coverage_does_not_reset_the_backoff(self):
    """THE bug in the first version. tileLoaded is per 0.25-degree area, so a commute crossing one
    bad area went uncovered -> pull -> covered -> counter wiped -> next trip, immediate pull again."""
    r = RegionRetry()
    r.record("WA", 0.0)                   # hit the bad area, requested
    # ... drive out of it, regain coverage for an hour, drive back in. Nothing calls forget().
    assert not r.due("WA", 30.0), "re-requested while the interval was still running"
    assert r.due("WA", 61.0)              # the interval, not the episode, decides
    r.record("WA", 61.0)
    assert not r.due("WA", 100.0)         # 2nd attempt now owes 120 s
    assert r.due("WA", 182.0)

  def test_pull_longer_than_the_interval_still_gets_a_real_gap(self):
    """The interval must measure idle time AFTER the pull ends, not time since the request was sent
    -- otherwise a 5-minute pull has already outlasted the first several tiers when it finishes."""
    r = RegionRetry()
    r.record("WA", 0.0)
    for t in range(0, 301, 10):           # pull runs 0..300 s; loop holds the clock while active
      r.hold("WA", float(t))
    assert not r.due("WA", 300.0), "fired the instant the pull ended"
    assert not r.due("WA", 359.0)
    assert r.due("WA", 361.0)             # a genuine 60 s of idle after the pull

  def test_border_ping_pong_is_bounded(self):
    """A car on a road along a state-bbox edge alternates WA/OR. With one shared counter every flip
    looked new and re-requested; per-region state bounds each independently."""
    r = RegionRetry()
    pulls = 0
    for i in range(400):                  # 400 loops alternating, 1 s apart
      region = "WA" if i % 2 == 0 else "OR"
      now = float(i)
      if r.due(region, now):
        r.record(region, now)
        pulls += 1
    # 2 immediate (one per unseen region) + a bounded few as each region's own interval expires
    assert pulls <= 12, f"{pulls} pulls in 400 s of border alternation"

  def test_forget_forces_an_immediate_request(self):
    """"Refresh this location map" deletes the tiles on purpose and must re-fetch now."""
    r = RegionRetry()
    for _ in range(6):
      r.record("WA", 0.0)                 # deep into the backoff
    assert not r.due("WA", 100.0)
    r.forget("WA")
    assert r.due("WA", 100.0)
    assert r.attempts("WA") == 0

  def test_hold_on_an_unknown_region_is_a_noop(self):
    r = RegionRetry()
    r.hold("WA", 5.0)                     # must not create state or raise
    assert r.attempts("WA") == 0
    assert r.due("WA", 5.0)

  def test_every_retry_gets_real_idle_time_not_just_a_bounded_count(self):
    """The property that actually distinguishes fixed from broken.

    A pull-count bound alone does NOT: with a 5-minute pull, measuring the interval from send time
    and measuring it from pull end BOTH yield 6 pulls in the first hour. What differs is the gaps --
    measuring from send makes the first several retries fire the instant the pull ends, back to back
    with zero idle time, because the interval expired while the pull was still running.
    """
    D = 300.0                             # whole-state pull duration
    r = RegionRetry()
    now, starts, gaps = 0.0, [], []
    prev_end = None
    while now < 3600.0:
      if r.due("WA", now):
        if prev_end is not None:
          gaps.append(now - prev_end)     # idle time between the last pull ENDING and this one starting
        starts.append(now)
        r.record("WA", now)
        for t in range(0, int(D) + 1, 10):   # pull runs; the loop holds the clock while active
          r.hold("WA", now + t)
        now += D
        prev_end = now
      else:
        now += 1.0
    assert len(starts) <= 6, f"{len(starts)} whole-state pulls in the first hour"
    assert gaps, "expected at least one retry to measure a gap for"
    # every retry must wait at least the base interval AFTER the previous pull finished
    assert min(gaps) >= REGION_RESEND_INTERVAL_S, f"gapless retry: gaps={[round(g) for g in gaps]}"
    # and the gaps must actually escalate, not sit flat at the base
    assert gaps[-1] > gaps[0], f"backoff did not escalate: gaps={[round(g) for g in gaps]}"
