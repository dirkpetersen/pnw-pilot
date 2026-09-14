"""netscanpin2pnw — the pure half: parsing the driver's pick, judging whether it is still in force,
and deciding when a tier-1/2 link may scan for something cheaper. The loop-level behaviour is pinned
in test_network_arbiter_sequences.py; these pin each decision directly."""
import math

import pytest

from openpilot.system.networkd.network_arbiter import (PIN_JOIN_WINDOW_S, UPGRADE_SCAN_S, judge_pin,
                                                       parse_manual_pick, upgrade_scan_due)

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
  BASE = dict(ladder_on=True, on_client_wifi=True, on_tier0=False, pinned=False, now=500.0, last_scan=0.0)

  def test_due_on_a_tier_1_2_link(self):
    assert upgrade_scan_due(**self.BASE) is True

  @pytest.mark.parametrize("field, value", [
    ("ladder_on", False),       # kill switch = pre-ladder behaviour
    ("on_client_wifi", False),  # not needed: off client wifi the geo-gate already allows scanning
    ("on_tier0", True),         # already cheapest
    ("pinned", True),           # the driver chose
  ])
  def test_each_guard_suppresses_it(self, field, value):
    assert upgrade_scan_due(**{**self.BASE, field: value}) is False

  def test_it_is_throttled(self):
    assert upgrade_scan_due(**{**self.BASE, "now": UPGRADE_SCAN_S - 1, "last_scan": 0.0}) is False
    assert upgrade_scan_due(**{**self.BASE, "now": UPGRADE_SCAN_S, "last_scan": 0.0}) is True

  def test_the_first_scan_is_immediate(self):
    assert upgrade_scan_due(**{**self.BASE, "now": 0.0, "last_scan": -math.inf}) is True
