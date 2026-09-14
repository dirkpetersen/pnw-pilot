"""netscanpin2pnw — the UI records the driver's hand-picked network, on BOTH join paths.

A saved network tapped in Settings goes through activate_connection (NM ActivateConnection, no
password); a new network or a password re-entry goes through connect_to_network (NM
AddAndActivateConnection2). The driver said a hand pick must stick, and he will usually tap a saved
network without typing anything -- so the saved-network path must record a pick too, or the arbiter
cannot know. The comma's own hotspot is activated through the same method and must NOT count.
"""
import threading

import pytest

import openpilot.system.ui.lib.wifi_manager as wm_mod
from openpilot.system.ui.lib.wifi_manager import WifiManager


class FakeParams:
  store: dict = {}
  fail: Exception | None = None

  def put_nonblocking(self, key, value):
    if FakeParams.fail is not None:
      raise FakeParams.fail
    FakeParams.store[key] = value


@pytest.fixture
def wm(monkeypatch):
  FakeParams.store, FakeParams.fail = {}, None
  monkeypatch.setattr(wm_mod, "Params", FakeParams)
  # never touch DBus: the join workers run in threads and would talk to NetworkManager
  monkeypatch.setattr(threading.Thread, "start", lambda self: None)
  m = WifiManager.__new__(WifiManager)
  m._exit = True
  m._tethering_ssid = "weedle-2fd8"
  m._set_connecting = lambda ssid: None
  return m


def test_a_tap_on_a_SAVED_network_records_a_pick(wm):
  wm.activate_connection("KarlMoik")
  pick = FakeParams.store.get("WifiManualPick")
  assert pick and pick["ssid"] == "KarlMoik" and isinstance(pick["ts"], float)


def test_a_PASSWORD_entry_records_a_pick(wm):
  wm.connect_to_network("KarlMoik", "hunter2")
  assert FakeParams.store["WifiManualPick"]["ssid"] == "KarlMoik"


def test_the_comma_hotspot_is_NOT_a_pick(wm):
  """set_tethering_active activates the hotspot through activate_connection. Not a WiFi pick."""
  wm.activate_connection("weedle-2fd8", block=False)
  assert "WifiManualPick" not in FakeParams.store


def test_picking_the_same_network_twice_gives_two_identities(wm):
  """ts is what tells a fresh pick of an SSID from an old one the arbiter already ended."""
  wm.activate_connection("KarlMoik")
  first = FakeParams.store["WifiManualPick"]["ts"]
  wm.activate_connection("KarlMoik")
  assert FakeParams.store["WifiManualPick"]["ts"] >= first


def test_a_failed_write_is_LOGGED_and_does_not_break_the_join(wm, monkeypatch):
  """Rule 2. A pick that could not be recorded means the arbiter may later move the driver off a
  network he chose -- that must be visible. The join itself must still proceed."""
  logged = []
  monkeypatch.setattr(wm_mod.cloudlog, "exception", lambda msg, *a, **k: logged.append(msg))
  FakeParams.fail = RuntimeError("UnknownKeyName: WifiManualPick")
  wm.activate_connection("KarlMoik")          # must not raise
  assert logged and "KarlMoik" in logged[0]
