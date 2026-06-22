"""Unit tests for the pure multi-location priority-network model (network2xnor)."""
import json

from openpilot.system.networkd import priority_networks as pn
from openpilot.system.networkd.network_arbiter import priority_connection_id
from openpilot.system.networkd.geo_gate import near_any_home


class TestParseAndMigrate:
  def test_empty(self):
    assert pn.parse(None) == []
    assert pn.parse("") == []
    assert pn.parse("not json") == []

  def test_parse_list(self):
    raw = json.dumps([{"label": "Home", "ssid": "MyWifi", "lat": 45.5, "lon": -122.6, "portal": None},
                      {"ssid": "Visitor", "lat": None, "lon": None, "portal": "peak"}])
    nets = pn.parse(raw)
    assert len(nets) == 2
    assert nets[0]["label"] == "Home" and nets[0]["ssid"] == "MyWifi"
    assert nets[1]["label"] == "Visitor"          # label defaults to ssid
    assert nets[1]["portal"] == "peak"
    assert nets[1]["lat"] is None

  def test_dedupe_by_ssid(self):
    raw = json.dumps([{"ssid": "A"}, {"ssid": "A", "label": "dup"}])
    assert len(pn.parse(raw)) == 1

  def test_drops_entry_without_ssid(self):
    raw = json.dumps([{"label": "nossid"}, {"ssid": "ok"}])
    nets = pn.parse(raw)
    assert [e["ssid"] for e in nets] == ["ok"]

  def test_legacy_migration(self):
    # new list empty -> synthesize from the old single-home params
    nets = pn.parse(None, legacy_ssid="OldWifi", legacy_home_raw=json.dumps([45.1, -122.2]))
    assert len(nets) == 1
    assert nets[0]["ssid"] == "OldWifi"
    assert nets[0]["lat"] == 45.1 and nets[0]["lon"] == -122.2

  def test_legacy_migration_no_home(self):
    nets = pn.parse(None, legacy_ssid="OldWifi", legacy_home_raw=None)
    assert len(nets) == 1 and nets[0]["lat"] is None

  def test_list_wins_over_legacy(self):
    raw = json.dumps([{"ssid": "New"}])
    nets = pn.parse(raw, legacy_ssid="Old", legacy_home_raw=None)
    assert [e["ssid"] for e in nets] == ["New"]

  def test_empty_list_is_respected_not_resurrected(self):
    # user deleted all networks via the UI -> param is "[]". Must NOT resurrect the legacy network,
    # else a deleted legacy network is un-removable (Gemini increment-C finding #3).
    nets = pn.parse("[]", legacy_ssid="OldWifi", legacy_home_raw=json.dumps([45.1, -122.2]))
    assert nets == []

  def test_migration_still_runs_when_param_unset(self):
    # None/"" (never set) still migrates — distinct from an explicit empty list.
    assert len(pn.parse(None, legacy_ssid="OldWifi")) == 1
    assert len(pn.parse("", legacy_ssid="OldWifi")) == 1

  def test_roundtrip(self):
    raw = json.dumps([{"label": "Home", "ssid": "W", "lat": 1.0, "lon": 2.0, "portal": None}])
    assert pn.parse(pn.dumps(pn.parse(raw))) == pn.parse(raw)


class TestSelection:
  def setup_method(self):
    self.nets = pn.parse(json.dumps([
      {"ssid": "Home", "lat": 45.5, "lon": -122.6},
      {"ssid": "Visitor", "lat": 45.0, "lon": -123.0, "portal": "peak"},
    ]))

  def test_selects_in_range_and_saved(self):
    saved = [priority_connection_id("Visitor")]
    chosen = pn.select_available(self.nets, ["Visitor", "Other"], saved, priority_connection_id)
    assert chosen["ssid"] == "Visitor"

  def test_none_when_not_saved(self):
    chosen = pn.select_available(self.nets, ["Visitor"], [], priority_connection_id)
    assert chosen is None

  def test_none_when_not_in_range(self):
    saved = [priority_connection_id("Home")]
    assert pn.select_available(self.nets, ["Elsewhere"], saved, priority_connection_id) is None

  def test_first_match_wins(self):
    saved = [priority_connection_id("Home"), priority_connection_id("Visitor")]
    chosen = pn.select_available(self.nets, ["Visitor", "Home"], saved, priority_connection_id)
    assert chosen["ssid"] == "Home"   # order follows the configured list, not scan order

  def test_entry_for_ssid(self):
    assert pn.entry_for_ssid(self.nets, "Visitor")["portal"] == "peak"
    assert pn.entry_for_ssid(self.nets, "nope") is None
    assert pn.entry_for_ssid(self.nets, "") is None

  def test_locations_skips_unlearned(self):
    nets = pn.parse(json.dumps([{"ssid": "A", "lat": 1.0, "lon": 2.0}, {"ssid": "B"}]))
    assert pn.locations(nets) == [(1.0, 2.0)]


class TestNearAnyHome:
  def test_within_one(self):
    homes = [(45.5, -122.6), (45.0, -123.0)]
    assert near_any_home(homes, (45.0001, -123.0001)) is True   # ~15 m from 2nd

  def test_far_from_all(self):
    homes = [(45.5, -122.6), (45.0, -123.0)]
    assert near_any_home(homes, (40.0, -120.0)) is False

  def test_fail_open_no_homes(self):
    assert near_any_home([], (45.0, -123.0)) is True

  def test_fail_open_no_gps(self):
    assert near_any_home([(45.5, -122.6)], None) is True
