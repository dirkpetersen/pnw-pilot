"""netscanpin2pnw — the pure half: parsing the driver's pick, judging whether it is still in force,
and deciding when a tier-1/2 link may scan for something cheaper. The loop-level behaviour is pinned
in test_network_arbiter_sequences.py; these pin each decision directly."""
import math

import pytest

from openpilot.system.networkd.network_arbiter import (COST_METERED, COST_UNKNOWN, COST_UNMETERED, PIN_JOIN_WINDOW_S,
                                                       UPGRADE_SCAN_S, cost_class, home_to_yield_to, judge_pin,
                                                       on_priority_network, parse_manual_pick, upgrade_scan_due)

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
  unmetered, configured or not), and `link_settling` was added (never scan inside a DHCP window)."""
  BASE = dict(ladder_on=True, on_client_wifi=True, active_unmetered=False, pinned=False, link_settling=False,
              now=500.0, last_scan=0.0)

  def test_due_on_a_link_that_is_not_explicitly_unmetered(self):
    assert upgrade_scan_due(**self.BASE) is True

  @pytest.mark.parametrize("field, value", [
    ("ladder_on", False),         # kill switch = pre-ladder behaviour
    ("on_client_wifi", False),    # not needed: off client wifi the geo-gate already allows scanning
    ("active_unmetered", True),   # nothing can be cheaper
    ("pinned", True),             # the driver chose
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

  def test_the_home_network_qualifies(self):
    assert home_to_yield_to(self.STAT, [HOME, STAR], SAVED, {HOME}, set(), STAR) == HOME

  def test_unknown_cost_does_NOT_qualify(self):
    """Explicitly unmetered only. Visitor is `unknown`."""
    assert home_to_yield_to(self.STAT, ["Visitor"], SAVED, set(), set(), STAR) == ""

  def test_a_MOBILE_entry_is_never_home(self):
    """The phone is a mobile priority entry and explicitly unmetered. If it ended pins, picking Starlink
    with the phone in range -- the driver's own measured case -- could never stick."""
    assert home_to_yield_to(self.STAT, [PHONE], SAVED, {PHONE}, set(), STAR) == ""

  def test_no_real_scan_is_no_evidence(self):
    assert home_to_yield_to(self.STAT, None, SAVED, {HOME}, set(), STAR) == ""

  def test_a_BLOCKED_home_router_does_not_end_the_pin(self):
    """A dead home router in range must not end a working pin, only for the ladder to fail on it."""
    assert home_to_yield_to(self.STAT, [HOME], SAVED, {HOME}, {HOME.lower()}, STAR) == ""

  def test_an_unsaved_home_does_not_qualify(self):
    assert home_to_yield_to(self.STAT, [HOME], ["Hotspot"], {HOME}, set(), STAR) == ""

  def test_the_pinned_network_does_not_yield_to_itself(self):
    assert home_to_yield_to(self.STAT, [HOME], SAVED, {HOME}, set(), HOME) == ""

  def test_it_is_case_insensitive(self):
    assert home_to_yield_to(["hannelore"], ["HANNELORE"], SAVED, {"Hannelore"}, set(), STAR) == "hannelore"


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
