"""policetier2pnw — confirmed/unconfirmed grading from age + Waze upvotes.

Waze publishes no report lifetime. Their Help says only that reports "appear on the map for a certain
amount of time and this time changes according to the number of Wazers who react to a report", and a
Waze Team Admin stated: "duration for any report depends on number of upvotes and 'Not there's -- the
more upvoted the report is, the more time it lasts on the map." So we model that rule with the same
input the app expires on (num_thumbs_up):  life = BASE + BONUS * thumbs.

Driver directives this encodes:
  * never drop an old report ("I still do want to see it" when nothing else is in range)
  * "slow down only for the ones that show up on Waze" -> only `confirmed` may command a cap
"""
import json

import pytest

from openpilot.system.location_services import location_servicesd as lsd
from openpilot.system.location_services.location_servicesd import _line_police, _police_tier, _now_epoch

BASE = lsd.POLICE_TIER_BASE_MIN
BONUS = lsd.POLICE_TIER_BONUS_MIN


@pytest.fixture(autouse=True)
def _reset_knob_cache():
  """_police_tier_knobs() memoises in module state (a long-lived daemon reads one file); reset it so
  a test that points POLICE_TIER_PATH at a tmp file cannot leak its knobs into the next test."""
  lsd._police_tier_cache.update({"t": -1e9, "base": lsd.POLICE_TIER_BASE_MIN,
                                 "bonus": lsd.POLICE_TIER_BONUS_MIN, "id": None})
  yield
  lsd._police_tier_cache.update({"t": -1e9, "base": lsd.POLICE_TIER_BASE_MIN,
                                 "bonus": lsd.POLICE_TIER_BONUS_MIN, "id": None})


class TestTierModel:
  def test_fresh_unconfirmed_report_is_confirmed(self):
    assert _police_tier(BASE - 1, 0, BASE, BONUS) == "confirmed"

  def test_aged_report_with_no_upvotes_is_unconfirmed(self):
    assert _police_tier(BASE + 1, 0, BASE, BONUS) == "unconfirmed"

  def test_upvotes_extend_the_lifetime(self):
    # the live case that motivated this: 33.5 min old with 4 thumbs-up was the most reliable report in
    # the pull, while a 20 min / 0 thumbs one was not being shown by the driver's app.
    assert _police_tier(33.5, 4, BASE, BONUS) == "confirmed"
    assert _police_tier(20.3, 0, BASE, BONUS) == "unconfirmed"

  def test_boundary_is_inclusive(self):
    assert _police_tier(BASE, 0, BASE, BONUS) == "confirmed"
    assert _police_tier(BASE + 0.01, 0, BASE, BONUS) == "unconfirmed"

  def test_unknown_age_grades_confirmed(self):
    # a warning system must fail TOWARD warning; no timestamp is not evidence of staleness
    assert _police_tier(None, 0, BASE, BONUS) == "confirmed"
    assert _police_tier(None, None, BASE, BONUS) == "confirmed"

  @pytest.mark.parametrize("thumbs", ["", "x", float("nan"), -5, [1]])
  def test_garbage_thumbs_never_raises_and_counts_as_zero(self, thumbs):
    # None is NOT in this list: it means "field not carried" and deliberately grades confirmed
    # (fail toward warning) -- see TestUnknownInputsFailTowardWarning.
    assert _police_tier(BASE + 1, thumbs, BASE, BONUS) == "unconfirmed"

  def test_absurd_upvote_count_is_bounded(self):
    # a huge count must not make an arbitrarily old report "confirmed" forever
    life_cap = BASE + BONUS * lsd._POLICE_TIER_MAX_THUMBS
    assert _police_tier(life_cap + 1, 10 ** 9, BASE, BONUS) == "unconfirmed"

  def test_knob_loader_never_raises_and_returns_finite_defaults(self, tmp_path, monkeypatch):
    monkeypatch.setattr(lsd, "POLICE_TIER_PATH", str(tmp_path / "missing.json"))
    lsd._police_tier_cache["t"] = -1e9
    base, bonus = lsd._police_tier_knobs()
    assert (base, bonus) == (lsd.POLICE_TIER_BASE_MIN, lsd.POLICE_TIER_BONUS_MIN)

  def test_malformed_knob_file_falls_back_to_defaults(self, tmp_path, monkeypatch):
    f = tmp_path / "police_tiers.json"
    f.write_text("{not json at all")
    monkeypatch.setattr(lsd, "POLICE_TIER_PATH", str(f))
    lsd._police_tier_cache["t"] = -1e9
    assert lsd._police_tier_knobs() == (lsd.POLICE_TIER_BASE_MIN, lsd.POLICE_TIER_BONUS_MIN)

  def test_out_of_range_knob_is_rejected(self, tmp_path, monkeypatch):
    f = tmp_path / "police_tiers.json"
    f.write_text('{"base_min": 99999, "bonus_min": 5}')
    monkeypatch.setattr(lsd, "POLICE_TIER_PATH", str(f))
    lsd._police_tier_cache["t"] = -1e9
    base, bonus = lsd._police_tier_knobs()
    assert base == lsd.POLICE_TIER_BASE_MIN, "an out-of-range base must not make everything confirmed"
    assert bonus == 5.0, "a valid sibling knob is still honoured"


def _alert(uuid, lat, age_min=5, thumbs=0):
  return {"lat": lat, "lon": -122.0, "magvar": None, "uuid": uuid, "street": "", "town": "T",
          "thumbs": thumbs, "ts": None if age_min is None else (_now_epoch() - age_min * 60.0) * 1000.0}


def _line(alerts):
  recede = lsd._PoliceRecede(lsd.POLICE_RECEDE_MI)
  return _line_police(alerts, "ok", "", 47.0, -122.0, 0.0, [], recede)


class TestTierOnTheOverlayLine:
  def test_confirmed_report_is_tagged(self):
    out = _line([_alert("u", 47.0 + 3.0 / 69.0, age_min=2, thumbs=0)])
    assert out["state"] == "alert" and out["tier"] == "confirmed"

  def test_unconfirmed_report_is_still_shown(self):
    out = _line([_alert("u", 47.0 + 3.0 / 69.0, age_min=120, thumbs=0)])
    assert out["state"] == "alert", "an aged report must still reach the driver"
    assert out["tier"] == "unconfirmed"

  def test_thumbs_are_surfaced(self):
    out = _line([_alert("u", 47.0 + 3.0 / 69.0, age_min=33.5, thumbs=4)])
    assert out["thumbs"] == 4 and out["tier"] == "confirmed"

  def test_missing_thumbs_field_does_not_break_the_line(self):
    al = _alert("u", 47.0 + 3.0 / 69.0, age_min=2)
    del al["thumbs"]                       # pre-tier proxy body / cached response
    out = _line([al])
    assert out["state"] == "alert" and out["tier"] == "confirmed"


class TestDisplayControlSplit:
  """Two review rounds shaped this contract:

  Round 1 (BLOCKER): only ONE report was published and the controller acted on ITS tier, so a nearer
  UNCONFIRMED report masked a CONFIRMED one behind it and killed the banner AND the slowdown.
  Round 2 (BLOCKER): fixing that by giving the DISPLAY pick to the confirmed subset reintroduced the
  original flapping -- a confirmed report beyond POLICE_NEAR_MI goes through nearest_ahead's unstable
  path projection, so it alternated tick-to-tick with a near unconfirmed one.

  Contract: `dist_mi`/`tier`/`uuid` = the DISPLAY pick (proximity-first over all tiers, stable
  near-lock). `cap` = the CONTROL pick (nearest CONFIRMED report), the only thing allowed to slow the
  car or raise the banner. Neither can suppress the other.
  """

  def test_amber_is_displayed_while_the_confirmed_report_drives_control(self):
    out = _line([_alert("amber_near", 47.0 + 0.4 / 69.0, age_min=40, thumbs=0),
                 _alert("live_far", 47.0 + 0.8 / 69.0, age_min=2, thumbs=5)])
    assert out["uuid"] == "amber_near", "display stays proximity-first (no cross-tier flapping)"
    assert out["tier"] == "unconfirmed"
    assert out["cap"]["uuid"] == "live_far", "the live report must still reach the controller"

  def test_unconfirmed_shown_when_it_is_the_only_thing_in_range(self):
    # the driver's explicit rule: "if the last update was 64 minutes and there's no other police in
    # range then I do want to see it"
    out = _line([_alert("only_old", 47.0 + 0.4 / 69.0, age_min=64, thumbs=0)])
    assert out["state"] == "alert" and out["uuid"] == "only_old"
    assert out["tier"] == "unconfirmed"
    assert "cap" not in out, "nothing confirmed ahead -> no control channel -> no slowdown"

  def test_control_channel_picks_the_nearest_confirmed_report(self):
    out = _line([_alert("c_far", 47.0 + 0.9 / 69.0, age_min=2, thumbs=0),
                 _alert("c_near", 47.0 + 0.3 / 69.0, age_min=2, thumbs=0),
                 _alert("amber", 47.0 + 0.1 / 69.0, age_min=99, thumbs=0)])
    assert out["uuid"] == "amber"                 # closest overall
    assert out["cap"]["uuid"] == "c_near"         # closest CONFIRMED

  def test_confirmed_outside_the_near_ring_still_reaches_control(self):
    out = _line([_alert("amber_near", 47.0 + 0.3 / 69.0, age_min=99, thumbs=0),
                 _alert("live_5mi", 47.0 + 5.0 / 69.0, age_min=1, thumbs=2)])
    assert out["uuid"] == "amber_near"
    assert out["cap"]["uuid"] == "live_5mi"
    assert out["cap"]["dist_mi"] > 1.0, "distance is the CONFIRMED one's, not the displayed one's"

  def test_near_lock_still_holds_on_the_display_line(self):
    # the original flapping fix must survive: a distant report cannot preempt a near one on screen
    out = _line([_alert("near", 47.0 + 0.6 / 69.0, age_min=3, thumbs=0),
                 _alert("far", 47.0 + 6.0 / 69.0, age_min=3, thumbs=0)])
    assert out["uuid"] == "near" and out["cap"]["uuid"] == "near"

  def test_confirmed_report_is_both_display_and_control_when_it_is_closest(self):
    out = _line([_alert("live_near", 47.0 + 0.3 / 69.0, age_min=2, thumbs=0),
                 _alert("amber_far", 47.0 + 4.0 / 69.0, age_min=99, thumbs=0)])
    assert out["uuid"] == "live_near" and out["tier"] == "confirmed"
    assert out["cap"]["uuid"] == "live_near"


class TestProxyChainCarriesThumbs:
  """Fable review CRITICAL (F1). `thumbs` was added to the Lambda and to the legacy direct-key parse,
  but _parse_proxy_body -- the PRODUCTION path -- rebuilt each alert dict and dropped it. Every report
  then graded at thumbs=0, i.e. a flat BASE-minute lifetime, so heavily-upvoted live sightings lost
  their banner and their slowdown. Two review rounds missed it because the tier tests inject alerts
  straight into _line_police and the proxy tests predate the field. This closes that gap end-to-end."""

  @staticmethod
  def _proxy_body(**over):
    a = {"lat": 47.05, "lon": -122.0, "magvar": None, "ts": (_now_epoch() - 33.5 * 60) * 1000.0,
         "uuid": "u1", "street": "I-5", "town": "Lacey", "thumbs": 4}
    a.update(over)
    return json.dumps({"generated_at": _now_epoch(), "ttl_s": 180, "alerts": [a]})

  def test_thumbs_survives_the_proxy_parse(self):
    out = lsd.PoliceUpdater._parse_proxy_body(self._proxy_body())
    assert out[0]["thumbs"] == 4, "the production path must not drop the field the tiers key on"

  def test_upvoted_old_report_stays_confirmed_end_to_end(self):
    # the exact live case: 33.5 min old, 4 upvotes -> life = 10 + 40 = 50 min -> CONFIRMED
    alerts = lsd.PoliceUpdater._parse_proxy_body(self._proxy_body())
    out = _line(alerts)
    assert out["tier"] == "confirmed"
    assert "cap" in out, "an upvoted live report must still reach the control channel"

  def test_zero_upvotes_still_grades_by_age_end_to_end(self):
    alerts = lsd.PoliceUpdater._parse_proxy_body(self._proxy_body(thumbs=0))
    assert _line(alerts)["tier"] == "unconfirmed"


class TestUnknownInputsFailTowardWarning:
  """Fable F2: unknown AGE graded confirmed but unknown THUMBS graded 0 -- the suppressive direction.
  `thumbs` is None whenever the proxy body predates the field (older Lambda, or a cached response), so
  that asymmetry turned a deploy-order mismatch into silently suppressed slowdowns rather than a
  merely degraded model. Upstream sends an explicit 0 for a genuine zero, so None is unambiguous."""

  def test_missing_thumbs_grades_confirmed_not_zero(self):
    assert _police_tier(600, None, BASE, BONUS) == "confirmed"

  def test_explicit_zero_is_still_a_real_zero(self):
    assert _police_tier(BASE + 1, 0, BASE, BONUS) == "unconfirmed"

  def test_pre_tier_proxy_body_keeps_the_slowdown(self):
    al = _alert("legacy", 47.0 + 0.4 / 69.0, age_min=90)
    del al["thumbs"]                      # older Lambda / cached body
    out = _line([al])
    assert out["tier"] == "confirmed" and "cap" in out


class TestKnobFileCannotSuppressEverything:
  """Fable F6: bool is an int subclass, so {"base_min": true} set a 1-minute lifetime; and a base_min
  of 0 would mark every report unconfirmed. Both are the suppressive direction, one typo away."""

  def _knobs(self, tmp_path, monkeypatch, text):
    f = tmp_path / "police_tiers.json"
    f.write_text(text)
    monkeypatch.setattr(lsd, "POLICE_TIER_PATH", str(f))
    lsd._police_tier_cache["t"] = -1e9
    lsd._police_tier_cache["id"] = None
    return lsd._police_tier_knobs()

  def test_boolean_base_min_is_rejected(self, tmp_path, monkeypatch):
    assert self._knobs(tmp_path, monkeypatch, '{"base_min": true}')[0] == lsd.POLICE_TIER_BASE_MIN

  def test_zero_base_min_is_rejected(self, tmp_path, monkeypatch):
    assert self._knobs(tmp_path, monkeypatch, '{"base_min": 0}')[0] == lsd.POLICE_TIER_BASE_MIN

  def test_zero_bonus_is_allowed_age_only_mode(self, tmp_path, monkeypatch):
    assert self._knobs(tmp_path, monkeypatch, '{"bonus_min": 0}')[1] == 0.0

  def test_valid_knobs_apply(self, tmp_path, monkeypatch):
    assert self._knobs(tmp_path, monkeypatch, '{"base_min": 15, "bonus_min": 5}') == (15.0, 5.0)
