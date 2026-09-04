"""policeretain2pnw: keep a police report after it ages out of the aggregator feed.

Driver report 2026-09-04: Waze showed police at 0.1 mi; our overlay showed nothing. The forensics
log ruled out every filter -- all reports that tick were verdict "kept", nearest 6.6 mi. The report
HAD been in our feed and expired before we reached it:
    A  first seen 24.2 mi / age 14m   last seen 12.6 mi / age 29m   passed at 0.12 mi
    B  first seen 24.1 mi / age 21m   last seen 19.6 mi / age 29m   passed at 0.02 mi
Across 208 reports watched from young, 30% last appear in the 25-35 min band, only 4% survive past
40 min. At 60 mph a report 24 mi ahead takes ~24 min to reach -- comparable to its whole lifetime.
"""
from openpilot.system.location_services.location_servicesd import (
  POLICE_RETAIN_MAX,
  POLICE_RETAIN_S,
  _tier_of,
  merge_retained_police,
)


def _al(uuid, **kw):
  d = {"uuid": uuid, "lat": 47.5, "lon": -122.3, "ts": 0, "thumbs": 0}
  d.update(kw)
  return d


class TestMergeRetainedPolice:
  def test_a_report_that_leaves_the_feed_is_still_published(self):
    """THE case this exists for."""
    out, cache = merge_retained_police({}, [_al("A")], 1000.0)
    assert [a["uuid"] for a in out] == ["A"] and not out[0].get("retained")
    out, cache = merge_retained_police(cache, [], 1060.0)          # A vanished from the feed
    assert [a["uuid"] for a in out] == ["A"], "the report was lost the moment the feed dropped it"
    assert out[0]["retained"] is True

  def test_a_retained_report_is_forced_unconfirmed(self):
    """The safety property. `confirmed` is what licenses a slowdown; a retained report must never
    reach it -- and age alone does NOT guarantee that, since effective life is BASE + BONUS*thumbs."""
    _, cache = merge_retained_police({}, [_al("A", thumbs=6)], 1000.0)
    out, _ = merge_retained_police(cache, [], 1060.0)
    assert _tier_of(out[0], now=1060.0, base=10.0, bonus=10.0) == "unconfirmed"

  def test_a_live_report_keeps_its_real_tier(self):
    """Retention must not downgrade reports the feed still vouches for."""
    live = _al("A", ts=int(1060_000 - 60_000), thumbs=6)
    out, _ = merge_retained_police({}, [live], 1060.0)
    assert not out[0].get("retained")

  def test_returning_to_the_feed_clears_the_retained_flag(self):
    _, cache = merge_retained_police({}, [_al("A")], 1000.0)
    out, cache = merge_retained_police(cache, [], 1060.0)
    assert out[0]["retained"] is True
    out, cache = merge_retained_police(cache, [_al("A")], 1120.0)   # re-reported by another driver
    assert not out[0].get("retained"), "a report back in the feed is not retained"

  def test_retention_expires(self):
    _, cache = merge_retained_police({}, [_al("A")], 1000.0)
    out, cache = merge_retained_police(cache, [], 1000.0 + POLICE_RETAIN_S - 1)
    assert out and out[0]["retained"]
    out, cache = merge_retained_police(cache, [], 1000.0 + POLICE_RETAIN_S + 1)
    assert out == [] and cache == {}, "an expired report must be dropped, not carried forever"

  def test_live_reports_are_ordered_before_retained_ones(self):
    """Downstream selection is nearest-first; on a tie the feed-backed report should win."""
    _, cache = merge_retained_police({}, [_al("OLD")], 1000.0)
    out, _ = merge_retained_police(cache, [_al("NEW")], 1060.0)
    assert [a["uuid"] for a in out] == ["NEW", "OLD"]

  def test_no_duplicates_when_a_report_is_both_live_and_cached(self):
    _, cache = merge_retained_police({}, [_al("A")], 1000.0)
    out, _ = merge_retained_police(cache, [_al("A")], 1060.0)
    assert len(out) == 1

  def test_alerts_without_a_uuid_are_published_but_never_retained(self):
    out, cache = merge_retained_police({}, [{"lat": 1, "lon": 2}], 1000.0)
    assert len(out) == 1 and cache == {}
    out, _ = merge_retained_police(cache, [], 1060.0)
    assert out == []

  def test_the_cache_is_bounded(self):
    cache = {}
    for i in range(POLICE_RETAIN_MAX + 50):
      _, cache = merge_retained_police(cache, [_al(f"U{i}")], 1000.0 + i)
    assert len(cache) <= POLICE_RETAIN_MAX
    out, cache = merge_retained_police(cache, [], 1000.0 + POLICE_RETAIN_MAX + 60)
    assert all(a.get("uuid") in cache for a in out), "published a report evicted from the cache"

  def test_the_input_cache_is_not_mutated(self):
    """The poller holds the cache across ticks; an in-place edit would be a hidden aliasing bug."""
    _, cache = merge_retained_police({}, [_al("A")], 1000.0)
    before = {u: dict(v) for u, v in cache.items()}
    merge_retained_police(cache, [], 1060.0)
    assert cache == before

  def test_the_published_alert_is_a_copy(self):
    """Downstream mutates alerts (recede tracking); it must not corrupt the cache."""
    _, cache = merge_retained_police({}, [_al("A")], 1000.0)
    out, cache = merge_retained_police(cache, [], 1060.0)
    out[0]["lat"] = 99.0
    assert cache["A"]["al"]["lat"] == 47.5
