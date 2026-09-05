"""policeretain2pnw: keep a police report after it ages out of the aggregator feed.

Driver report 2026-09-04: Waze showed police at 0.1 mi; our overlay showed nothing. The forensics
log ruled out every filter -- all reports that tick were verdict "kept", nearest 6.6 mi. The report
HAD been in our feed and expired before we reached it:
    A  first seen 24.2 mi / age 14m   last seen 12.6 mi / age 29m   passed at 0.12 mi
    B  first seen 24.1 mi / age 21m   last seen 19.6 mi / age 29m   passed at 0.02 mi
Across 208 reports watched from young, 30% last appear in the 25-35 min band, only 4% survive past
40 min. At 60 mph a report 24 mi ahead takes ~24 min to reach -- comparable to its whole lifetime.
"""
from openpilot.system.location_services import location_servicesd as lsd
from openpilot.system.location_services.location_servicesd import (
  POLICE_RETAIN_MAX,
  POLICE_RETAIN_S,
  _line_police,
  _now_epoch,
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


def _retained_al(uuid, mi_ahead, age_min=2, thumbs=0):
  """A report as it is PUBLISHED after merge_retained_police held it: flagged, positioned ahead."""
  al = _line_alert(uuid, mi_ahead, age_min, thumbs)
  al["retained"] = True
  return al


def _line_alert(uuid, mi_ahead, age_min=2, thumbs=0):
  return {"lat": 47.0 + mi_ahead / 69.0, "lon": -122.0, "magvar": None, "uuid": uuid,
          "street": "", "town": "T", "thumbs": thumbs,
          "ts": (_now_epoch() - age_min * 60.0) * 1000.0}


def _line(alerts):
  recede = lsd._PoliceRecede(lsd.POLICE_RECEDE_MI)
  return _line_police(alerts, "ok", "", 47.0, -122.0, 0.0, [], recede)


class TestRetainedOnTheOverlayLine:
  """END-TO-END through the real _line_police. The isolation tests above prove _tier_of returns the
  right string; only these prove the CONTROL channel actually consults it. Fable review 2026-09-05
  showed a mutant reverting the `confirmed` list-comp to the pre-commit _police_tier() call passed
  all 109 tests while letting a retained report command a slowdown -- the safety property was
  asserted but untested."""

  def test_a_retained_report_never_reaches_the_control_channel(self):
    """THE safety property, at the only place it matters. 6 thumbs + 2 min old grades `confirmed`
    on age alone, so this fails the moment the retained override stops being consulted."""
    out = _line([_retained_al("A", 0.5, age_min=2, thumbs=6)])
    assert out["state"] == "alert", "a retained report must still be DISPLAYED"
    assert out["tier"] == "unconfirmed"
    assert "cap" not in out, "a retained report reached the channel that licenses a slowdown"

  def test_the_same_report_unretained_does_reach_it(self):
    """Control for the test above: proves the assertion is about `retained`, not about the fixture
    being too old/far to ever produce a cap."""
    al = _retained_al("A", 0.5, age_min=2, thumbs=6)
    del al["retained"]
    out = _line([al])
    assert out["tier"] == "confirmed" and "cap" in out

  def test_cap_prefers_a_live_report_over_a_nearer_retained_one(self):
    """The display may pick the nearest (retained) report; the control channel must skip it and
    take the live one behind it, not fall through to nothing."""
    out = _line([_retained_al("R", 0.5, age_min=2, thumbs=6), _line_alert("L", 3.0, age_min=2)])
    assert out["uuid"] == "R", "display is proximity-first"
    assert out["cap"]["uuid"] == "L", "control must skip the retained report, not the near one"

  def test_the_display_line_marks_a_retained_report(self):
    """soundd reads this field to suppress the siren, and it is the only way the driver can tell a
    retained amber from a merely-aged amber."""
    assert _line([_retained_al("A", 0.5)])["retained"] is True
    assert _line([_line_alert("A", 0.5)])["retained"] is False


class TestRetentionEdges:
  def test_seen_is_refreshed_on_every_appearance(self):
    """Retention is 45 min since the LAST appearance, not since the first. A report live for 40 min
    then vanishing must still be held ~45 min, not ~5."""
    _, cache = merge_retained_police({}, [_al("A")], 1000.0)
    for t in range(1060, 3400, 60):                       # stays in the feed for ~40 min
      _, cache = merge_retained_police(cache, [_al("A")], float(t))
    out, _ = merge_retained_police(cache, [], 3340.0 + POLICE_RETAIN_S - 60.0)
    assert out and out[0]["retained"], "retention was measured from FIRST seen, not last"

  def test_an_evicted_retained_report_is_not_published(self):
    """The cache bound and the published list must agree -- publishing a report we no longer hold
    would make it reappear-then-vanish tick to tick."""
    cache = {}
    for i in range(POLICE_RETAIN_MAX):                    # fill with retainable reports
      _, cache = merge_retained_police(cache, [_al(f"R{i}")], 1000.0 + i)
    live = [_al(f"L{i}") for i in range(5)]               # 5 live reports force 5 evictions
    out, cache = merge_retained_police(cache, live, 2000.0)
    assert len(cache) == POLICE_RETAIN_MAX
    published_retained = {a["uuid"] for a in out if a.get("retained")}
    assert published_retained <= set(cache), "published a retained report evicted from the cache"
    assert {a["uuid"] for a in out if not a.get("retained")} == {f"L{i}" for i in range(5)}

  def test_a_report_is_still_held_exactly_at_the_ttl(self):
    """Boundary: expiry is strictly `>` retain_s, so the TTL is inclusive."""
    _, cache = merge_retained_police({}, [_al("A")], 1000.0)
    out, _ = merge_retained_police(cache, [], 1000.0 + POLICE_RETAIN_S)
    assert out and out[0]["retained"], "dropped one tick early at the boundary"
