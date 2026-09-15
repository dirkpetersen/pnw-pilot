"""netscanpin2pnw — the pure half: parsing the driver's pick, judging whether it is still in force,
and deciding when a tier-1/2 link may scan for something cheaper. The loop-level behaviour is pinned
in test_network_arbiter_sequences.py; these pin each decision directly."""
import math

import pytest

from openpilot.system.networkd.geo_gate import HOME_GEOFENCE_M
from openpilot.system.networkd.network_arbiter import (COST_METERED, COST_UNKNOWN, COST_UNMETERED,
                                                       PIN_HOME_FAR_M, PIN_JOIN_WINDOW_S, UPGRADE_SCAN_S, arrival_candidates,
                                                       cost_class, home_to_yield_to, judge_pin, on_priority_network,
                                                       parse_manual_pick, unmetered_to_yield_to, update_home_arrival,
                                                       upgrade_scan_due)

STAR, PHONE = "KarlMoik", "Dirk’s iPhone 13"


class TestParseManualPick:
  def test_a_well_formed_pick(self):
    assert parse_manual_pick({"ssid": STAR, "ts": 12.5}) == ((STAR, 12.5), "")

  def test_ABSENT_is_the_only_silent_no_pin(self):
    """None = never written or cleared on boot. That is a real negative and needs no log."""
    assert parse_manual_pick(None) == (None, "")

  @pytest.mark.parametrize("raw, why", [
    ("KarlMoik", "not an object"),
    ({"ts": 1.0}, "ssid"),
    ({"ssid": "   ", "ts": 1.0}, "ssid"),
    ({"ssid": STAR}, "ts"),
    ({"ssid": STAR, "ts": "yesterday"}, "ts"),
    ({"ssid": STAR, "ts": True}, "ts"),
  ])
  def test_a_DAMAGED_pick_says_why(self, raw, why):
    """Present-but-unusable is not 'no pin'. The caller logs the reason."""
    pick, problem = parse_manual_pick(raw)
    assert pick is None and why in problem


class TestJudgePin:
  def test_no_pick_no_pin(self):
    assert judge_pin("", 0.0, False, STAR, False, 10.0) == ("", False, "")

  def test_on_the_pinned_network_it_holds_and_is_marked_seen(self):
    assert judge_pin(STAR, 0.0, False, STAR, False, 10.0) == (STAR, True, "")

  def test_it_is_case_insensitive(self):
    assert judge_pin("karlmoik", 0.0, False, STAR, False, 10.0).pinned_ssid == "karlmoik"

  def test_during_the_join_it_holds(self):
    """Not yet active, inside the window: the UI is still joining. Hold, do not end."""
    assert judge_pin(STAR, 0.0, False, "", False, PIN_JOIN_WINDOW_S - 1) == (STAR, False, "")

  def test_a_join_that_never_lands_times_out(self):
    assert judge_pin(STAR, 0.0, False, "", False, PIN_JOIN_WINDOW_S).ended == "join_timeout"

  def test_once_seen_a_different_active_network_means_dropped(self):
    assert judge_pin(STAR, 0.0, True, PHONE, False, 5.0) == ("", True, "dropped")

  def test_once_seen_nothing_active_means_dropped(self):
    assert judge_pin(STAR, 0.0, True, "", False, 5.0).ended == "dropped"

  def test_a_BLAMED_pinned_network_ends_the_pin(self):
    """A pin must never hold the device offline."""
    assert judge_pin(STAR, 0.0, True, STAR, True, 5.0) == ("", True, "failed")

  def test_an_UNREADABLE_active_connection_is_no_evidence(self):
    """None = nmcli failed. Must neither end the pin nor advance its state -- including past the
    join window and after the network was seen."""
    assert judge_pin(STAR, 0.0, True, None, False, 5.0) == (STAR, True, "")
    assert judge_pin(STAR, 0.0, False, None, False, PIN_JOIN_WINDOW_S * 10) == (STAR, False, "")

  def test_unreadable_does_not_mask_a_real_failure_verdict(self):
    """judge_link can only blame from a successful read, so a blame with an unreadable active read
    cannot happen in the loop. The ORDER still matters: no evidence wins over a stale blame."""
    assert judge_pin(STAR, 0.0, True, None, True, 5.0).ended == ""


class TestUpgradeScanDue:
  """netrank2pnw: `on_tier0` became `active_unmetered` (the scan is due on ANY link not explicitly
  unmetered, configured or not), and `link_settling` was added (never scan inside a DHCP window).
  pinunmetered2pnw: `pinned` became `pin_joining` -- a pin suppresses the scan only while its network is not the
  active link yet; once on it, an arriving unmetered network can end the pin, and only a scan can see it arrive."""
  BASE = dict(ladder_on=True, on_client_wifi=True, active_unmetered=False, pin_joining=False, link_settling=False,
              now=500.0, last_scan=0.0)

  def test_due_on_a_link_that_is_not_explicitly_unmetered(self):
    assert upgrade_scan_due(**self.BASE) is True

  @pytest.mark.parametrize("field, value", [
    ("ladder_on", False),         # kill switch = pre-ladder behaviour
    ("on_client_wifi", False),    # not needed: off client wifi the geo-gate already allows scanning
    ("active_unmetered", True),   # nothing can be cheaper
    ("pin_joining", True),        # the UI is still joining the driver's pick
    ("link_settling", True),      # a bring-up may still be inside its DHCP window
  ])
  def test_each_guard_suppresses_it(self, field, value):
    assert upgrade_scan_due(**{**self.BASE, field: value}) is False

  def test_it_is_throttled(self):
    assert upgrade_scan_due(**{**self.BASE, "now": UPGRADE_SCAN_S - 1, "last_scan": 0.0}) is False
    assert upgrade_scan_due(**{**self.BASE, "now": UPGRADE_SCAN_S, "last_scan": 0.0}) is True

  def test_the_first_scan_is_immediate(self):
    assert upgrade_scan_due(**{**self.BASE, "now": 0.0, "last_scan": -math.inf}) is True


class TestCostClass:
  def test_the_three_classes_in_order(self):
    assert COST_UNMETERED < COST_UNKNOWN < COST_METERED
    assert cost_class("Phone", set(), {"phone"}) == COST_UNMETERED
    assert cost_class("Visitor", set(), set()) == COST_UNKNOWN
    assert cost_class("KarlMoik", {"KARLMOIK"}, set()) == COST_METERED

  def test_both_sets_is_treated_as_metered(self):
    """Impossible from NM; if it ever happens, the conservative reading."""
    assert cost_class("x", {"x"}, {"x"}) == COST_METERED


HOME = "Hannelore"
SAVED = ["Hotspot", "lte", "openpilot connection Hannelore", "openpilot connection Visitor",
         f"openpilot connection {PHONE}", f"openpilot connection {STAR}"]


class TestHomeToYieldTo:
  """Fable D2: a pin yields to a STATIONARY, EXPLICITLY UNMETERED configured network in range."""
  STAT = [HOME, "Visitor"]

  def test_the_home_network_qualifies_once_it_has_ARRIVED(self):
    """netrank2pnw correction: qualifying is not enough -- it must have been genuinely absent since the pick
    (update_home_arrival). Without `arrived` nothing yields, so a pick made at home sticks."""
    assert home_to_yield_to(self.STAT, [HOME, STAR], SAVED, {HOME}, set(), STAR, arrived={HOME.lower()}) == HOME
    assert home_to_yield_to(self.STAT, [HOME, STAR], SAVED, {HOME}, set(), STAR) == ""
    assert home_to_yield_to(self.STAT, [HOME, STAR], SAVED, {HOME}, set(), STAR, arrived=set()) == ""

  def test_unknown_cost_does_NOT_qualify(self):
    """Explicitly unmetered only. Visitor is `unknown`."""
    assert home_to_yield_to(self.STAT, ["Visitor"], SAVED, set(), set(), STAR, arrived={"visitor"}) == ""

  def test_a_MOBILE_entry_is_never_home(self):
    """The phone is a mobile priority entry and explicitly unmetered. If it ended pins, picking Starlink
    with the phone in range -- the driver's own measured case -- could never stick."""
    assert home_to_yield_to(self.STAT, [PHONE], SAVED, {PHONE}, set(), STAR, arrived={PHONE.lower()}) == ""

  def test_no_real_scan_is_no_evidence(self):
    assert home_to_yield_to(self.STAT, None, SAVED, {HOME}, set(), STAR, arrived={HOME.lower()}) == ""

  def test_a_BLOCKED_home_router_does_not_end_the_pin(self):
    """A dead home router in range must not end a working pin, only for the ladder to fail on it."""
    assert home_to_yield_to(self.STAT, [HOME], SAVED, {HOME}, {HOME.lower()}, STAR, arrived={HOME.lower()}) == ""

  def test_an_unsaved_home_does_not_qualify(self):
    assert home_to_yield_to(self.STAT, [HOME], ["Hotspot"], {HOME}, set(), STAR, arrived={HOME.lower()}) == ""

  def test_the_pinned_network_does_not_yield_to_itself(self):
    assert home_to_yield_to(self.STAT, [HOME], SAVED, {HOME}, set(), HOME, arrived={HOME.lower()}) == ""

  def test_it_is_case_insensitive(self):
    assert home_to_yield_to(["hannelore"], ["HANNELORE"], SAVED, {"Hannelore"}, set(), STAR,
                            arrived={"HanneLore"}) == "hannelore"


class TestJudgePinYieldsToHome:
  def test_home_in_range_ends_the_pin(self):
    assert judge_pin(STAR, 0.0, True, STAR, False, 5.0, home_ssid=HOME) == ("", True, "home")

  def test_a_failure_verdict_takes_precedence_over_home(self):
    """Both end the pin; `failed` is the more important thing for a human to read in the log."""
    assert judge_pin(STAR, 0.0, True, STAR, True, 5.0, home_ssid=HOME).ended == "failed"

  def test_an_unreadable_active_read_still_wins(self):
    assert judge_pin(STAR, 0.0, True, None, False, 5.0, home_ssid=HOME) == (STAR, True, "")


class TestOnPriorityNetwork:
  """What the uploader's `at_home` means after netrank2pnw."""
  CONFIGURED = [HOME, "Visitor", PHONE]

  def test_a_configured_network_not_explicitly_metered_qualifies(self):
    assert on_priority_network(HOME, self.CONFIGURED, "no") is True
    assert on_priority_network("Visitor", self.CONFIGURED, "unknown") is True

  def test_an_EXPLICITLY_METERED_configured_network_does_not(self):
    """THE CHANGE. Membership alone used to authorise 75 MB uploads over a link the driver had marked
    metered -- the 2026-09-10 shape: 2,642 MB over metered Starlink while it was a configured entry."""
    assert on_priority_network(STAR, [*self.CONFIGURED, STAR], "yes") is False

  def test_a_failed_read_with_nothing_cached_is_not_evidence_of_cost(self):
    assert on_priority_network("Visitor", self.CONFIGURED, None) is True

  def test_an_unconfigured_network_never_qualifies(self):
    assert on_priority_network(STAR, self.CONFIGURED, "no") is False

  def test_membership_is_case_insensitive(self):
    """Was exact case -- a configured "visitor" never set OnPriorityNetwork on the AP's "Visitor"."""
    assert on_priority_network("Visitor", ["visitor"], "unknown") is True

  def test_nothing_active_is_not_a_priority_network(self):
    assert on_priority_network("", self.CONFIGURED, None) is False


class TestUpdateHomeArrival:
  """netrank2pnw (D2 correction): has a home network been GENUINELY absent since the pick?"""
  LOC = (HOME, 47.0, -122.0)
  AT_HOME, NEAR, FAR = (47.0, -122.0), (47.0036, -122.0), (47.0063, -122.0)   # 0 m, ~400 m, ~700 m

  def step(self, state, scan, gps=AT_HOME):
    return update_home_arrival(state, [self.LOC], scan, gps)

  def gone(self, state):
    return state.get(HOME.lower(), (0, False))[1]

  def test_consecutive_REAL_scans_missing_it_establish_absence_WITHOUT_GPS(self):
    """Fable D2: scan-count absence applies only where GPS cannot place the truck. This test used to run
    with GPS at home; under the corrected rule that is precisely the case where misses do NOT count."""
    st = {}
    for _ in range(2):                      # concrete: two misses are not absence, the third is
      st = self.step(st, [STAR], gps=None)
    assert not self.gone(st), "two missing scans must not be absence"
    st = self.step(st, [STAR], gps=None)
    assert self.gone(st)

  def test_missing_scans_do_NOT_count_while_GPS_says_the_truck_is_still_there(self):
    """At the learned location, and inside twice the geofence: the AP went quiet, the truck did not leave."""
    for gps in (self.AT_HOME, self.NEAR):
      st = {}
      for _ in range(10):
        st = self.step(st, [STAR], gps=gps)
      assert not self.gone(st), f"ten misses at {gps} were read as the truck leaving"

  def test_with_no_LEARNED_location_scan_misses_still_count(self):
    st = {}
    for _ in range(3):
      st = update_home_arrival(st, [(HOME, None, None)], [STAR], self.AT_HOME)
    assert self.gone(st)

  def test_a_scan_that_LISTS_it_resets_the_count(self):
    """Flicker never adds up. Run WITHOUT GPS: since Fable's D2 fix, misses don't count at all while GPS
    places the truck at home, so with GPS at home this test could no longer tell whether presence resets
    the count -- mutation A3 survived exactly that way."""
    st = {}
    for _ in range(10):
      st = self.step(st, [STAR], gps=None)            # missing
      st = self.step(st, [STAR], gps=None)            # missing
      st = self.step(st, [STAR, HOME], gps=None)      # present again
    assert not self.gone(st)

  def test_a_scan_that_did_not_RUN_is_no_evidence(self):
    """WITHOUT GPS (mutation A4 survived with GPS at home, where D2 already refuses to count any miss)."""
    st = {}
    for _ in range(20):
      st = self.step(st, None, gps=None)
    assert st.get(HOME.lower(), (0, False)) == (0, False)

  def test_GPS_confidently_far_establishes_absence_without_any_scan(self):
    """How "not visible at pick time" is established on the road, where no scans run."""
    assert self.gone(self.step({}, None, gps=self.FAR))

  def test_GPS_inside_twice_the_geofence_does_NOT(self):
    """The 2x margin: a truck parked near the edge of a learned location is not 'away'."""
    assert HOME_GEOFENCE_M < 400 < PIN_HOME_FAR_M
    assert not self.gone(self.step({}, None, gps=self.NEAR))

  def test_no_GPS_or_no_learned_location_is_no_evidence(self):
    assert not self.gone(self.step({}, None, gps=None))
    assert not self.gone(update_home_arrival({}, [(HOME, None, None)], None, self.FAR))

  def test_a_scan_that_LISTS_it_beats_a_far_GPS_reading(self):
    """The learned location may be stale; the radio is the direct observation."""
    assert not self.gone(self.step({}, [HOME], gps=self.FAR))

  def test_absence_once_established_persists_through_presence(self):
    """Arriving home, the network is present from then on -- it must still be able to end the pin, e.g.
    once a failure backoff on it expires."""
    st = self.step({}, None, gps=self.FAR)
    for _ in range(5):
      st = self.step(st, [HOME])
    assert self.gone(st)

  def test_it_is_pure(self):
    st = {}
    self.step(st, [STAR])
    assert st == {}


# ---- pinunmetered2pnw: a pin ends when an explicitly unmetered network ARRIVES (owner decision 2026-09-14) ----

CAFE = "CafeFree"
SAVED2 = [*SAVED, f"openpilot connection {CAFE}"]


TRUCK_CONFIGURED = (HOME, "Visitor", PHONE)   # the truck's TetheringPriorityNetworks: 2 stationary + the mobile phone


class TestUnmeteredToYieldTo:
  """A pin ends for a saved, CONFIGURED, explicitly unmetered network that is in the real scan, ARRIVED, not in backoff,
  not the pinned network -- and only when the pinned network's cost was read and is not `no`."""

  @staticmethod
  def y(scan=(PHONE, STAR), saved=SAVED2, configured=TRUCK_CONFIGURED, unmetered=(PHONE,), blocked=(), pinned=STAR,
        pinned_metered="yes", arrived=(PHONE,)):
    return unmetered_to_yield_to(None if scan is None else list(scan), list(saved), list(configured), set(unmetered),
                                 set(blocked), pinned, pinned_metered, None if arrived is None else {a.lower() for a in arrived})

  def test_an_ARRIVED_unmetered_network_ends_a_METERED_pin(self):
    assert self.y() == (PHONE, "", "")

  def test_and_a_DEFAULT_cost_pin_too(self):
    """Strictly cheaper means `no` beats `unknown` as well as `yes`."""
    assert self.y(pinned_metered="unknown").ends_by == PHONE

  def test_the_MOBILE_phone_qualifies_mobility_is_not_an_input(self):
    """netrank2pnw exempted mobile entries; the owner's rule does not. Mobility only changes arrival EVIDENCE, which
    the caller folds into `arrived`; this function takes the configured SSIDs with no mobile flag at all."""
    assert self.y().ends_by == PHONE

  def test_a_configured_STATIONARY_entry_qualifies_too(self):
    """In the daemon home_to_yield_to is judged first and names it `home`; this rule alone would end it as well."""
    assert self.y(scan=(HOME, STAR), unmetered=(HOME,), arrived=(HOME,)) == (HOME, "", "")

  def test_a_network_visible_since_the_pick_does_not_and_is_named(self):
    assert self.y(arrived=()) == ("", PHONE, "visible_since_pick")
    assert self.y(arrived=None) == ("", PHONE, "visible_since_pick")

  def test_an_EXPLICITLY_UNMETERED_pin_is_never_ended_here(self):
    assert self.y(pinned_metered="no") == ("", "", "")
    assert self.y(pinned_metered=" No ") == ("", "", "")

  def test_a_pinned_cost_that_was_NEVER_READ_holds_and_says_so(self):
    """None is a failed read with nothing cached -- not `unknown`. A cost move needs a cost that was read."""
    assert self.y(pinned_metered=None) == ("", PHONE, "pinned_cost_unread")

  def test_DEFAULT_cost_does_not_qualify(self):
    assert self.y(unmetered=()) == ("", "", "")

  def test_a_network_IN_BACKOFF_does_not_qualify(self):
    assert self.y(blocked=(PHONE.lower(),)) == ("", "", "")

  def test_an_UNSAVED_network_does_not_qualify(self):
    assert self.y(saved=["Hotspot", f"openpilot connection {STAR}"]) == ("", "", "")

  def test_it_must_be_in_THIS_TICKS_REAL_scan(self):
    assert self.y(scan=None) == ("", "", "")
    assert self.y(scan=(STAR,)) == ("", "", "")

  def test_the_pinned_network_never_yields_to_itself(self):
    """Its cost comes from a different read than the candidates', so they can disagree (e.g. marked mid-tick)."""
    assert self.y(scan=(STAR,), unmetered=(STAR,), arrived=(STAR,)) == ("", "", "")

  def test_no_pin_nothing_to_end(self):
    assert self.y(pinned="  ") == ("", "", "")

  def test_the_comma_hotspot_and_lte_profiles_are_never_candidates(self):
    assert self.y(scan=("Hotspot", "lte"), unmetered=("Hotspot", "lte"), arrived=("Hotspot", "lte")) == ("", "", "")

  def test_a_NON_configured_saved_profile_does_NOT_qualify_and_is_named_as_such(self):
    """pinconfigured2pnw -- OWNER DECISION 2026-09-14 ~21:30 PT, verbatim "(no just the configued ones)". A saved cafe
    the driver marked unmetered, arrived, in range: it does not end the pin, and the log can say why."""
    assert self.y(scan=(CAFE, STAR), unmetered=(CAFE,), arrived=(CAFE,)) == ("", CAFE, "not_configured")
    assert self.y(scan=(CAFE, STAR), unmetered=(CAFE,), arrived=()) == ("", CAFE, "not_configured")
    assert self.y(scan=(CAFE, STAR), unmetered=(CAFE,), arrived=(CAFE,), pinned_metered=None) == ("", CAFE, "not_configured")

  def test_the_same_network_ends_it_once_it_IS_configured(self):
    """The list is what decides, nothing else about the network."""
    assert self.y(scan=(CAFE, STAR), configured=(*TRUCK_CONFIGURED, CAFE), unmetered=(CAFE,), arrived=(CAFE,)) == \
      (CAFE, "", "")

  def test_an_unconfigured_network_is_not_reported_when_nothing_else_would_qualify(self):
    """`not_configured` is named only for a network that passes the scan, cost and backoff guards."""
    assert self.y(scan=(CAFE, STAR), unmetered=(), arrived=(CAFE,)) == ("", "", "")
    assert self.y(scan=(CAFE, STAR), unmetered=(CAFE,), blocked=(CAFE.lower(),), arrived=(CAFE,)) == ("", "", "")
    assert self.y(scan=(STAR,), unmetered=(CAFE,), arrived=(CAFE,)) == ("", "", "")

  def test_an_unconfigured_network_never_HIDES_a_configured_one(self):
    """CafeFree sorts before the phone. Its arrival must not stand in for the phone's (the phone did not arrive), and its
    `not_configured` must not replace the phone's own reason in the log."""
    assert self.y(scan=(CAFE, PHONE), unmetered=(CAFE, PHONE), arrived=(CAFE,)) == ("", PHONE, "visible_since_pick")
    assert self.y(scan=(CAFE, PHONE), unmetered=(CAFE, PHONE), arrived=(CAFE, PHONE)) == (PHONE, "", "")
    assert self.y(scan=(CAFE, PHONE), unmetered=(CAFE, PHONE), arrived=(CAFE, PHONE), pinned_metered=None) == \
      ("", PHONE, "pinned_cost_unread")

  def test_the_first_unconfigured_network_is_named_stably(self):
    saved = [f"openpilot connection {STAR}", "openpilot connection zed", f"openpilot connection {CAFE}"]
    assert self.y(scan=("zed", CAFE), saved=saved, unmetered=("zed", CAFE), arrived=()) == ("", CAFE, "not_configured")

  def test_it_is_case_insensitive(self):
    """Called directly: the helper above folds `arrived` itself, which hid a case-sensitive `arrived` (mutation). The
    configured spelling may differ in case and surrounding space from the saved profile's."""
    assert unmetered_to_yield_to([PHONE.upper(), STAR], SAVED2, [f" {PHONE.swapcase()} "], {PHONE.lower()}, set(),
                                 STAR.lower(), "yes", {PHONE.swapcase()}).ends_by == PHONE

  def test_an_arrival_wins_over_an_EARLIER_network_that_was_visible_since_the_pick(self):
    """Order only picks WHICH network is named; any arrived one ends the pin. CafeFree sorts first and did not arrive."""
    configured = (*TRUCK_CONFIGURED, CAFE)
    assert self.y(scan=(CAFE, PHONE), configured=configured, unmetered=(CAFE, PHONE), arrived=(PHONE,)) == (PHONE, "", "")

  def test_the_named_network_is_STABLE_first_in_case_folded_order(self):
    saved = [f"openpilot connection {PHONE}", f"openpilot connection {CAFE}"]
    configured = (PHONE, CAFE)
    assert self.y(scan=(PHONE, CAFE), saved=saved, configured=configured, unmetered=(PHONE, CAFE),
                  arrived=(PHONE, CAFE)).ends_by == CAFE
    assert self.y(scan=(PHONE, CAFE), saved=saved, configured=configured, unmetered=(PHONE, CAFE), arrived=()).kept_by == CAFE


class TestArrivalCandidates:
  NETS = [{"ssid": HOME, "lat": 47.0, "lon": -122.0, "mobile": False},
          {"ssid": PHONE, "lat": 45.0, "lon": -123.0, "mobile": True},     # "Add Network Here" stored a spot
          {"ssid": "visitor", "lat": None, "lon": None, "mobile": False}]

  def test_stationary_entries_keep_their_learned_location(self):
    assert (HOME, 47.0, -122.0) in arrival_candidates(self.NETS)

  def test_a_MOBILE_entry_has_NO_location_even_when_one_is_stored(self):
    """Otherwise GPS "far from where the phone was added" would read as the phone being absent, and "near it" would
    veto its scan misses."""
    assert (PHONE, None, None) in arrival_candidates(self.NETS)

  def test_exactly_the_configured_entries_are_tracked(self):
    """pinconfigured2pnw: an unconfigured saved profile cannot end a pin, so its arrival is not tracked."""
    assert arrival_candidates(self.NETS) == [(HOME, 47.0, -122.0), (PHONE, None, None), ("visitor", None, None)]

  def test_one_entry_per_network_case_insensitively(self):
    """update_home_arrival counts one miss per entry; a duplicate would count every miss twice."""
    nets = [*self.NETS, {"ssid": HOME.upper(), "lat": 1.0, "lon": 1.0, "mobile": False}]
    got = arrival_candidates(nets)
    assert [s.lower() for s, _a, _b in got] == [HOME.lower(), PHONE.lower(), "visitor"]
    assert got[0] == (HOME, 47.0, -122.0), "the first entry's spelling and location win"

  def test_blank_entries_are_not_tracked(self):
    assert arrival_candidates([{"ssid": " ", "mobile": False}, {"mobile": True}]) == []


class TestJudgePinUnmetered:
  def test_an_unmetered_arrival_ends_the_pin(self):
    assert judge_pin(STAR, 0.0, True, STAR, False, 5.0, unmetered_ssid=PHONE) == ("", True, "unmetered")

  def test_home_keeps_its_own_reason_when_both_apply(self):
    assert judge_pin(STAR, 0.0, True, STAR, False, 5.0, home_ssid=HOME, unmetered_ssid=HOME).ended == "home"

  def test_a_failure_still_comes_first(self):
    assert judge_pin(STAR, 0.0, True, STAR, True, 5.0, unmetered_ssid=PHONE).ended == "failed"

  def test_an_unreadable_active_read_still_wins(self):
    assert judge_pin(STAR, 0.0, True, None, False, 5.0, unmetered_ssid=PHONE) == (STAR, True, "")
