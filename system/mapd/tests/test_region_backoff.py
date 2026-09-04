"""mapdgate2pnw: the whole-STATE download retry must be bounded.

mapd has no incremental fetch, so every retry re-downloads the entire region. Before this, a region
that stayed uncovered was re-requested every REGION_RESEND_INTERVAL_S (60 s) forever -- ~60 whole-state
downloads an hour for as long as the car sat there.
"""
from openpilot.system.mapd.mapd_configd import (
  REGION_RESEND_INTERVAL_S,
  REGION_MAX_RESEND_INTERVAL_S,
  next_region_interval,
)


class TestNextRegionInterval:
  def test_first_attempt_keeps_the_plain_interval(self):
    # attempts<=1 is the benign case: mapd missed the message because its mapdIn socket wasn't up.
    # It must NOT be penalised, or a cold start would wait minutes for maps.
    assert next_region_interval(0) == REGION_RESEND_INTERVAL_S
    assert next_region_interval(1) == REGION_RESEND_INTERVAL_S

  def test_escalates_geometrically(self):
    assert next_region_interval(2) == 120.0
    assert next_region_interval(3) == 240.0
    assert next_region_interval(4) == 480.0
    assert next_region_interval(5) == 960.0

  def test_caps(self):
    # 60 * 2**5 = 1920 would exceed the cap
    assert next_region_interval(6) == REGION_MAX_RESEND_INTERVAL_S
    for attempts in range(6, 60):
      assert next_region_interval(attempts) == REGION_MAX_RESEND_INTERVAL_S

  def test_never_exceeds_cap_and_never_below_base(self):
    for attempts in range(200):
      interval = next_region_interval(attempts)
      assert REGION_RESEND_INTERVAL_S <= interval <= REGION_MAX_RESEND_INTERVAL_S

  def test_monotonic_non_decreasing(self):
    prev = 0.0
    for attempts in range(60):
      interval = next_region_interval(attempts)
      assert interval >= prev
      prev = interval

  def test_negative_attempts_are_safe(self):
    # defensive: a caller bug must not produce a tiny interval and reinstate the hammering
    assert next_region_interval(-1) == REGION_RESEND_INTERVAL_S
    assert next_region_interval(-100) == REGION_RESEND_INTERVAL_S

  def test_bounds_the_hourly_download_count(self):
    """The whole point: cap the number of whole-state pulls per hour of sitting uncovered."""
    elapsed, attempts, pulls = 0.0, 0, 0
    while elapsed < 3600.0:
      elapsed += next_region_interval(attempts)
      attempts += 1
      pulls += 1
    # old behaviour was a flat 60 s => 60 pulls/hour. Anything near that is a regression.
    assert pulls <= 8, f"{pulls} whole-state downloads in the first hour"
