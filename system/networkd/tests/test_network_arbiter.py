"""Unit tests for the pure network2xnor arbitration decision function."""
from openpilot.system.networkd.network_arbiter import (
  HOTSPOT_CONNECTION_ID,
  decide,
  priority_connection_id,
)

PRIORITY_SSID = "home_wifi"
PRIORITY_ID = priority_connection_id(PRIORITY_SSID)  # "openpilot connection home_wifi"


class TestPriorityConnectionId:
  def test_format(self):
    assert priority_connection_id("foo") == "openpilot connection foo"


class TestDecideTetheringOff:
  def test_off_hotspot_up_tears_it_down(self):
    # tethering disabled but the hotspot is somehow active -> bring it down
    assert decide(False, "", [], [], HOTSPOT_CONNECTION_ID) == "down_hotspot"

  def test_off_nothing_active_noop(self):
    assert decide(False, "", [], [], None) == "noop"

  def test_off_never_touches_client_wifi(self):
    # connected to a client network while tethering off -> leave it alone
    assert decide(False, PRIORITY_SSID, [PRIORITY_SSID], [PRIORITY_ID], PRIORITY_ID) == "noop"

  def test_off_ignores_priority_config(self):
    # even with a priority SSID in range, tethering-off only ever ensures hotspot down
    assert decide(False, PRIORITY_SSID, [PRIORITY_SSID], [PRIORITY_ID], None) == "noop"


class TestDecideTetheringOnPriority:
  def test_priority_in_range_and_saved_switches_to_wifi(self):
    # hotspot currently up, priority appears in range and is saved -> switch to it
    assert decide(True, PRIORITY_SSID, [PRIORITY_SSID, "other"], [PRIORITY_ID], HOTSPOT_CONNECTION_ID) == "up_priority"

  def test_priority_already_active_noop(self):
    # already on the priority network -> idempotent, do nothing
    assert decide(True, PRIORITY_SSID, [PRIORITY_SSID], [PRIORITY_ID], PRIORITY_ID) == "noop"

  def test_priority_absent_brings_hotspot_up(self):
    # tethering on, priority configured but not in range -> hotspot should be up
    assert decide(True, PRIORITY_SSID, ["other"], [PRIORITY_ID], None) == "up_hotspot"

  def test_priority_absent_hotspot_already_up_noop(self):
    assert decide(True, PRIORITY_SSID, ["other"], [PRIORITY_ID], HOTSPOT_CONNECTION_ID) == "noop"

  def test_priority_in_range_but_not_saved_brings_hotspot_up(self):
    # SSID visible but no saved "openpilot connection <ssid>" -> can't switch, keep hotspot
    assert decide(True, PRIORITY_SSID, [PRIORITY_SSID], [], None) == "up_hotspot"

  def test_priority_not_saved_hotspot_already_up_noop(self):
    assert decide(True, PRIORITY_SSID, [PRIORITY_SSID], [], HOTSPOT_CONNECTION_ID) == "noop"


class TestDecideTetheringOnNoPriority:
  def test_blank_priority_brings_hotspot_up(self):
    # tethering on, no priority configured -> just run the hotspot
    assert decide(True, "", ["whatever"], [], None) == "up_hotspot"

  def test_blank_priority_hotspot_already_up_noop(self):
    assert decide(True, "", ["whatever"], [], HOTSPOT_CONNECTION_ID) == "noop"

  def test_whitespace_priority_treated_as_blank(self):
    # a priority of only spaces must not match any SSID
    assert decide(True, "   ", ["   "], [], None) == "up_hotspot"


class TestDecideEdgeCases:
  def test_priority_only_named_ssid_interrupts_hotspot(self):
    # a DIFFERENT saved+in-range network must NOT drag the radio off the hotspot
    other_id = priority_connection_id("cafe")
    assert decide(True, PRIORITY_SSID, ["cafe"], [other_id], HOTSPOT_CONNECTION_ID) == "noop"

  def test_switch_from_wrong_client_to_priority(self):
    # currently on some other client connection; priority appears -> switch to priority
    other_id = priority_connection_id("cafe")
    assert decide(True, PRIORITY_SSID, [PRIORITY_SSID, "cafe"], [PRIORITY_ID, other_id], other_id) == "up_priority"
