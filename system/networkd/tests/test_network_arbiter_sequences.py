"""netcosttier2pnw — LOOP-LEVEL sequence tests: main() driven tick by tick against a fake NM.

WHY THIS FILE EXISTS, and it is not a style preference. Every pure function in this feature is
pinned by unit tests, and that was not enough twice:

  * a revision of this change shipped with 100 green tests while `_metered_states` had no `return`
    statement -- it returned None, every tick raised inside the loop's `except Exception`, and the
    arbiter was alive and doing NOTHING. No unit test touched the daemon.
  * a later revision reintroduced a defect a reviewer had already found and I had already fixed --
    an outright `con up` failure going unrecorded, so the hotspot was never re-raised -- with 118
    green tests. The regression was entirely in the loop's glue between tested functions.

So these tests drive `network_arbiterd.main()` itself, with nmcli, params, the clock, the geo-gate
and the modem all faked, and assert on the SEQUENCE of connections the arbiter actually brings up.
The scenarios are the measured failure modes from review; each one is a bug that shipped in some
revision of this branch. Adapted from the review harness.
"""
import json
import subprocess

import pytest

import openpilot.system.networkd.network_arbiterd as d
from openpilot.system.networkd.network_arbiter import PIN_JOIN_WINDOW_S

PHONE, STAR, HOME = "Dirk's iPhone 13", "KarlMoik", "Hannelore"
HOTSPOT = "Hotspot"


class FakeNM:
  """Just enough NetworkManager to drive the loop. `behave` scripts how each profile responds to
  `con up`: ok / refuse (association fails, nothing active) / no_dhcp (associates, no address)."""

  def __init__(self):
    self.active: str | None = None
    self.saved = [HOTSPOT, d.priority_connection_id(HOME), d.priority_connection_id(PHONE),
                  d.priority_connection_id(STAR), "lte"]
    self.scan: list[str] = []
    self.metered: dict[str, str] = {}
    self.ip: dict[str, str | None] = {}
    self.conn: dict[str, str] = {}
    self.behave: dict[str, object] = {}
    self.fail_reads: set[str] = set()
    self.t = 0.0
    self.ups: list[str] = []
    self.scans = 0                              # netscanpin2pnw: `dev wifi list` calls that returned data
    self.scan_times: list[float] = []           # ...and when
    self.up_log: list[tuple[float, str]] = []   # (t, connection) for every `con up`

  def nmcli(self, args):
    a = " ".join(args)
    if any(k in a for k in self.fail_reads):
      return None
    if "--active" in a:
      return f"{self.active}:802-11-wireless:wlan0\n" if self.active else ""
    if "dev wifi list" in a:
      self.scans += 1
      self.scan_times.append(self.t)
      return "\n".join(self.scan) + "\n"
    if a == "-t -f NAME con show":
      return "\n".join(self.saved) + "\n"
    if "connection.metered" in a:
      return f"connection.metered:{self.metered.get(d.ssid_of(args[-1]), 'unknown')}\n"
    if "IP4.ADDRESS" in a:
      c = args[-1]
      return f"IP4.ADDRESS[1]:{self.ip[c]}/24\n" if self.active == c and self.ip.get(c) else ""
    if "GENERAL.IP4-CONNECTIVITY" in a:
      return self.conn.get(self.active, "4 (full)") + "\n"
    if "CONNECTIVITY" in a:
      return "full\n"
    if args[:2] == ["con", "up"]:
      c = args[2]
      self.ups.append(c)
      self.up_log.append((self.t, c))
      if c == HOTSPOT:
        self.active = HOTSPOT
        return ""
      b = self.behave.get(c, "ok")
      if callable(b):
        b = b(self.t)
      if b == "refuse":
        self.active = None
        return None
      self.active = c
      self.ip[c] = "10.0.0.2" if b == "ok" else None
      return ""
    if args[:2] == ["con", "down"]:
      if self.active == args[2]:
        self.active = None
      return ""
    if args[:3] == ["-t", "-f", "NAME,TYPE,AUTOCONNECT,AUTOCONNECT-PRIORITY"]:
      return ""
    return ""


def run_loop(monkeypatch, nm, ticks=8, near_home=True, hooks=(), priority=(HOME,), ladder=True,
             params=None, mobile=(PHONE,), gps=(47.0, -122.0)):
  """Drive main() for `ticks` polls. Returns the list of connections it brought up, in order.

  `params` (netscanpin2pnw): a dict consulted by the fake Params.get for any other key -- hooks may
  mutate it mid-run (the driver picking a network). A value that is an Exception instance is RAISED,
  to model a read failure such as UnknownKeyName."""
  params = {} if params is None else params
  # netrank2pnw: `mobile` mirrors the truck's REAL TetheringPriorityNetworks, where the iPhone is
  # "mobile": true and Hannelore/Visitor are stationary. The harness used to build every entry as
  # stationary, which is harmless until stationarity MEANS something -- a pin now yields to a stationary,
  # explicitly unmetered entry in range, and a stationary phone would wrongly qualify as "home".
  state = {"n": 0}
  monkeypatch.setattr(d, "_nmcli", nm.nmcli)
  monkeypatch.setattr(d, "_run", lambda args: subprocess.CompletedProcess(args, 0, "", ""))
  monkeypatch.setattr(d, "_modem_index", lambda: None)
  monkeypatch.setattr(d, "_lte_throttled_recently", lambda: False)
  monkeypatch.setattr(d, "_lte_has_ip", lambda: False)
  # netrank2pnw: gps may be a callable, so a sequence can DRIVE (a pin's arrival evidence reads it)
  monkeypatch.setattr(d, "_read_gps", lambda p, m: gps() if callable(gps) else gps)
  # netrank2pnw: near_home may be a callable, so a sequence can ARRIVE home mid-run
  monkeypatch.setattr(d, "near_any_home", lambda locs, gps: near_home() if callable(near_home) else near_home)
  monkeypatch.setattr(d, "_usable_cache", {})

  nets = [{"label": s, "ssid": s, "lat": None if s in mobile else 47.0, "lon": None if s in mobile else -122.0,
           "mobile": s in mobile} for s in priority]

  class P:
    def get_bool(self, k):
      if k == "DisableNetworkCostLadder":
        return not ladder
      return k == "TetheringEnabled"

    def get(self, k):
      if k == "TetheringPriorityNetworks":
        return json.dumps(nets)
      v = params.get(k)
      if isinstance(v, Exception):
        raise v
      return v

    def put(self, *a):
      pass

    def put_bool(self, k, v):
      params[k] = v          # netrank2pnw: record, so OnPriorityNetwork (the uploader's at_home) is assertable

  monkeypatch.setattr(d, "Params", lambda *a, **k: P())

  def fake_sleep(sec):
    nm.t += sec
    state["n"] += 1
    for h in hooks:
      h(nm, state["n"])
    if state["n"] >= ticks:
      raise SystemExit

  monkeypatch.setattr(d.time, "sleep", fake_sleep)
  monkeypatch.setattr(d.time, "monotonic", lambda: float(nm.t))
  with pytest.raises(SystemExit):
    d.main()
  return nm.ups


class TestTheArbiterAlwaysHasAnUplink:
  """The property that matters more than which tier wins: the device must never end up with no
  working connection and no plan to get one."""

  def test_a_network_that_REFUSES_to_associate_falls_back_to_the_hotspot(self, monkeypatch):
    """`up_fallback` drops the hotspot BEFORE raising the client, and a refused association leaves
    NOTHING active -- so this failure is invisible unless the pending bring-up is judged. A revision
    of this branch shipped without that and retried forever with the hotspot down, tethered clients
    dark. Measured: 5 attempts, 0 hotspot re-raises."""
    nm = FakeNM()
    nm.active, nm.scan = HOTSPOT, [PHONE]
    nm.metered[PHONE] = "no"
    nm.behave[d.priority_connection_id(PHONE)] = "refuse"
    ups = run_loop(monkeypatch, nm, ticks=8, priority=())
    assert HOTSPOT in ups, f"never re-raised the hotspot after a refused association: {ups}"

  def test_and_it_backs_off_instead_of_hammering(self, monkeypatch):
    nm = FakeNM()
    nm.active, nm.scan = HOTSPOT, [PHONE]
    nm.metered[PHONE] = "no"
    nm.behave[d.priority_connection_id(PHONE)] = "refuse"
    ups = run_loop(monkeypatch, nm, ticks=8, priority=())
    client_ups = [u for u in ups if u != HOTSPOT]
    assert len(client_ups) <= 3, f"retried a refusing network every tick: {ups}"


class TestItDoesNotTearDownWorkingLinks:
  """Every one of these dropped a perfectly good connection in some revision."""

  def test_a_link_still_doing_DHCP_is_left_alone(self, monkeypatch):
    """NM's DHCP timeout is 45 s, the poll is 20 s, so the first look lands mid-activation."""
    nm = FakeNM()
    nm.active, nm.scan = HOTSPOT, [PHONE]
    nm.metered[PHONE] = "no"
    nm.behave[d.priority_connection_id(PHONE)] = "no_dhcp"
    hooks = [lambda nm, tk: nm.ip.__setitem__(d.priority_connection_id(PHONE), "10.0.0.9")
             if tk == 3 and nm.active == d.priority_connection_id(PHONE) else None]
    ups = run_loop(monkeypatch, nm, ticks=6, near_home=False, hooks=hooks, priority=())
    assert ups.count(HOTSPOT) == 0, f"tore down a link that was mid-DHCP: {ups}"

  def test_one_unreadable_nmcli_call_does_not_cost_us_the_radio(self, monkeypatch):
    """An ERROR is not a NEGATIVE RESULT. A revision folded a read timeout into 'dead' and handed
    the radio to the hotspot on the next tick."""
    nm = FakeNM()
    nm.active = d.priority_connection_id(PHONE)
    nm.ip[nm.active] = "10.0.0.2"
    nm.scan = []
    hooks = [lambda nm, tk: nm.fail_reads.add("IP4.ADDRESS") if tk == 2
             else nm.fail_reads.discard("IP4.ADDRESS")]
    ups = run_loop(monkeypatch, nm, ticks=6, near_home=False, hooks=hooks, priority=())
    assert HOTSPOT not in ups, f"one flaky read cost a working link: {ups}"

  def test_a_stale_cached_reading_does_not_either(self, monkeypatch):
    """The cache is a memory of the LAST time we could read a network, not a current fact."""
    nm = FakeNM()
    nm.active, nm.scan = HOTSPOT, [PHONE]
    nm.metered[PHONE] = "no"
    monkeypatch.setattr(d, "_usable_cache", {PHONE.lower(): False})
    hooks = [lambda nm, tk: nm.fail_reads.add("IP4.ADDRESS") if tk == 1
             else nm.fail_reads.discard("IP4.ADDRESS")]
    ups = run_loop(monkeypatch, nm, ticks=6, near_home=False, hooks=hooks, priority=())
    assert ups.count(HOTSPOT) == 0, f"a stale cache entry tore down a good link: {ups}"

  def test_a_network_the_driver_joined_himself_is_not_dropped_mid_DHCP(self, monkeypatch):
    nm = FakeNM()
    nm.active = d.priority_connection_id(STAR)
    nm.ip[nm.active] = None
    nm.scan = []
    nm.metered[STAR] = "yes"
    hooks = [lambda nm, tk: nm.ip.__setitem__(d.priority_connection_id(STAR), "10.0.0.5")
             if tk == 1 and nm.active == d.priority_connection_id(STAR) else None]
    ups = run_loop(monkeypatch, nm, ticks=6, near_home=False, hooks=hooks, priority=())
    assert HOTSPOT not in ups, f"dropped the driver's own manual join: {ups}"


class TestItDoesNotParkOnADeadLink:
  def test_a_captive_portal_with_no_handler_does_not_hold_the_radio(self, monkeypatch):
    """A hotel WiFi joined once is still saved, so the ladder can pick it. With an address but no
    upstream it wins the default route (wlan0 metric 600 vs wwan0 1000) and black-holes the DEVICE's
    own traffic. Measured in a revision: parked on it indefinitely with no log line at all."""
    nm = FakeNM()
    nm.active = HOTSPOT
    nm.saved.append(d.priority_connection_id("HotelWifi"))
    nm.scan = ["HotelWifi"]
    nm.conn[d.priority_connection_id("HotelWifi")] = "3 (portal)"
    ups = run_loop(monkeypatch, nm, ticks=6, near_home=False, priority=())
    assert ups.count(HOTSPOT) >= 1, f"parked on a portal network with no handler: {ups}"


class TestTheKillSwitch:
  def test_the_ladder_runs_by_default(self, monkeypatch):
    nm = FakeNM()
    nm.active, nm.scan = HOTSPOT, [PHONE]
    nm.metered[PHONE] = "no"
    ups = run_loop(monkeypatch, nm, ticks=3, near_home=False, priority=(), ladder=True)
    assert d.priority_connection_id(PHONE) in ups

  def test_and_the_param_turns_it_off_without_a_deploy(self, monkeypatch):
    """A feature that can hold the radio needs a revert that does not need the radio."""
    nm = FakeNM()
    nm.active, nm.scan = HOTSPOT, [PHONE]
    nm.metered[PHONE] = "no"
    ups = run_loop(monkeypatch, nm, ticks=3, near_home=False, priority=(), ladder=False)
    assert d.priority_connection_id(PHONE) not in ups, f"kill switch ignored: {ups}"


# ================================================================================================
# netscanpin2pnw — (1) a tier-1/2 link keeps looking for something cheaper; (2) a manual pick sticks.
#
# Measured 2026-09-13: 66 minutes on metered Starlink ("KarlMoik") with the unmetered iPhone never
# considered, because the geo-gate stops scanning on ANY client WiFi away from a learned location.
# Driver, on the fix: "Yes if you can identify that a handpicked Wi-fi was selected then we want that to
# stick." Every scenario below starts AWAY from home (near_home=False), which is where the bug lived.
# ================================================================================================

ID_PHONE, ID_STAR = d.priority_connection_id(PHONE), d.priority_connection_id(STAR)
POLL = d.POLL_INTERVAL_S


@pytest.fixture
def events(monkeypatch):
  """Every cloudlog.event the daemon emits, as (name, kwargs). Rule 2: the pin and the upgrade scan
  must be visible where a human would look, so the tests assert on them."""
  got: list[tuple[str, dict]] = []
  monkeypatch.setattr(d.cloudlog, "event", lambda name, **kw: got.append((name, kw)))
  return got


def names(events):
  return [n for n, _ in events]


def _away(nm, active, *, scan=(STAR, PHONE)):
  """Both networks physically in range: Starlink asserted metered, the iPhone asserted unmetered."""
  nm.scan = list(scan)
  nm.metered = {STAR: "yes", PHONE: "no"}
  nm.active = active
  if active and active != HOTSPOT:
    nm.ip[active] = "10.0.0.2"


def _pick(ssid, ts=1.0):
  return {"ssid": ssid, "ts": ts}


class TestStarlinkToPhone:
  """Part 1 — on a non-tier-0 client link, scan for something cheaper, throttled."""

  def test_the_phone_takes_over_from_metered_starlink_within_one_scan_interval(self, monkeypatch, events):
    """THE REPORT. On KarlMoik (usable, metered, not a priority entry); the iPhone appears LATER, after
    an upgrade scan has already run, so the switch must come from the NEXT throttled scan."""
    nm = FakeNM()
    _away(nm, ID_STAR, scan=(STAR,))
    appear = 2 * POLL
    hooks = [lambda nm, tk: nm.scan.append(PHONE) if tk == 2 else None]
    run_loop(monkeypatch, nm, ticks=12, near_home=False, hooks=hooks, priority=(HOME, PHONE))
    switch = [t for t, c in nm.up_log if c == ID_PHONE]
    assert switch, f"stayed on metered Starlink with the phone in range; ups={nm.up_log} scans={nm.scan_times}"
    assert switch[0] - appear <= d.UPGRADE_SCAN_S + POLL, f"took {switch[0] - appear:.0f} s after the phone appeared"
    assert "netcosttier_upgrade_scan" in names(events)

  def test_the_upgrade_scan_is_throttled(self, monkeypatch, events):
    """A scan takes the single radio off-channel. Nothing cheaper around -> at most one per interval."""
    nm = FakeNM()
    _away(nm, ID_STAR, scan=(STAR,))
    run_loop(monkeypatch, nm, ticks=16, near_home=False, priority=(HOME, PHONE))   # 320 s
    assert nm.ups == [], f"moved off Starlink with nothing cheaper in range: {nm.ups}"
    gaps = [b - a for a, b in zip(nm.scan_times, nm.scan_times[1:], strict=False)]
    assert nm.scans >= 2, f"expected repeated upgrade scans over 320 s, got {nm.scan_times}"
    assert all(g >= d.UPGRADE_SCAN_S for g in gaps), f"scans closer than {d.UPGRADE_SCAN_S} s: {nm.scan_times}"
    assert names(events).count("netcosttier_upgrade_scan") == nm.scans

  def test_no_scans_at_all_while_on_a_priority_network(self, monkeypatch, events):
    """The geo-gate's purpose survives: already on the cheapest tier, away from home -> zero scans."""
    nm = FakeNM()
    _away(nm, ID_PHONE)
    run_loop(monkeypatch, nm, ticks=10, near_home=False, priority=(HOME, PHONE))
    assert nm.ups == [] and nm.scans == 0, f"ups={nm.ups} scans={nm.scan_times}"
    assert "netcosttier_upgrade_scan" not in names(events)

  def test_a_METERED_priority_entry_is_not_tier_0_and_keeps_looking(self, monkeypatch, events):
    """choose_wifi demotes a metered priority entry into the ladder, so being on one is not 'done'."""
    nm = FakeNM()
    _away(nm, ID_STAR, scan=(STAR,))
    run_loop(monkeypatch, nm, ticks=4, near_home=False, priority=(HOME, STAR, PHONE))
    assert nm.scans >= 1, "a metered priority entry was treated as the cheapest tier"

  def test_after_the_upgrade_it_does_not_flap_back(self, monkeypatch, events):
    nm = FakeNM()
    _away(nm, ID_STAR)
    run_loop(monkeypatch, nm, ticks=14, near_home=False, priority=(HOME, PHONE))
    assert nm.ups == [ID_PHONE], f"expected exactly one switch to the phone: {nm.ups}"
    assert nm.active == ID_PHONE


class TestManualPickSticks:
  """Part 2 — when the driver picks a network in Settings, the ladder does not move him off it."""

  def test_a_manual_pick_of_starlink_is_not_overridden_by_the_phone(self, monkeypatch, events):
    """THE DRIVER'S DECISION. On the phone; he picks KarlMoik in Settings (UI writes WifiManualPick, NM
    joins it). The phone is still in range and cheaper. The arbiter must leave him on KarlMoik."""
    nm = FakeNM()
    _away(nm, ID_PHONE)
    params = {}
    def pick_starlink(nm, tk):
      if tk == 2:
        params["WifiManualPick"] = _pick(STAR)
        nm.active = ID_STAR
        nm.ip[ID_STAR] = "10.0.0.3"
    run_loop(monkeypatch, nm, ticks=16, near_home=False, hooks=[pick_starlink], priority=(HOME, PHONE),
             params=params)
    assert ID_PHONE not in nm.ups, f"the arbiter overrode the driver's manual pick: {nm.up_log}"
    assert nm.active == ID_STAR
    assert "netcosttier_pin_set" in names(events)
    assert "netcosttier_upgrade_scan" not in names(events), "scanned for something cheaper while pinned"

  def test_the_pin_holds_the_radio_DURING_the_join(self, monkeypatch, events):
    """The UI's password path deletes and re-adds the profile, so for a moment NOTHING is active. The
    arbiter must not use that moment to raise the phone over the driver's in-progress choice."""
    nm = FakeNM()
    _away(nm, None)
    params = {"WifiManualPick": _pick(STAR)}
    def join_lands(nm, tk):
      if tk == 3:
        nm.active = ID_STAR
        nm.ip[ID_STAR] = "10.0.0.3"
    run_loop(monkeypatch, nm, ticks=10, near_home=False, hooks=[join_lands], priority=(HOME, PHONE),
             params=params)
    assert nm.ups == [], f"fought the UI's join: {nm.up_log}"
    assert "netcosttier_pin_held" in names(events)

  def test_the_pin_ends_when_the_pinned_network_DROPS_and_the_ladder_resumes(self, monkeypatch, events):
    nm = FakeNM()
    _away(nm, ID_STAR)
    params = {"WifiManualPick": _pick(STAR)}
    def starlink_gone(nm, tk):
      if tk == 3:
        nm.active = None
        nm.scan = [PHONE]
    run_loop(monkeypatch, nm, ticks=8, near_home=False, hooks=[starlink_gone], priority=(HOME, PHONE),
             params=params)
    assert ID_PHONE in nm.ups, f"the ladder did not resume after the pinned network dropped: {nm.up_log}"
    assert ("netcosttier_pin_cleared", {"ssid": STAR, "reason": "dropped"}) in events

  def test_a_pick_that_never_joins_ends_on_the_join_window(self, monkeypatch, events):
    """Wrong password / AP gone: the pick never becomes active. Held for PIN_JOIN_WINDOW_S, no longer."""
    nm = FakeNM()
    _away(nm, HOTSPOT, scan=(PHONE,))
    params = {"WifiManualPick": _pick(STAR)}
    run_loop(monkeypatch, nm, ticks=9, near_home=False, priority=(HOME, PHONE), params=params)
    ups = [t for t, c in nm.up_log if c == ID_PHONE]
    assert ups, f"the pin held the radio forever after a failed join: {nm.up_log}"
    assert ups[0] >= PIN_JOIN_WINDOW_S, f"gave up on the join after only {ups[0]:.0f} s"
    assert ("netcosttier_pin_cleared", {"ssid": STAR, "reason": "join_timeout"}) in events

  def test_a_pinned_network_that_FAILS_does_not_hold_the_device_offline(self, monkeypatch, events):
    """Associated to the pinned network, no address. After the DHCP grace it is blamed, the pin ends,
    the failure ledger records it, and the device moves on -- a pin must never mean 'offline'."""
    nm = FakeNM()
    _away(nm, None)
    nm.active = ID_STAR
    nm.ip[ID_STAR] = None
    params = {"WifiManualPick": _pick(STAR)}
    run_loop(monkeypatch, nm, ticks=10, near_home=False, priority=(HOME, PHONE), params=params)
    assert ID_PHONE in nm.ups or HOTSPOT in nm.ups, f"held on a dead pinned link: {nm.up_log}"
    assert ("netcosttier_pin_cleared", {"ssid": STAR, "reason": "failed"}) in events
    assert any(n == "netcosttier_assoc_failed" and kw.get("ssid") == STAR.lower() for n, kw in events)

  def test_a_newer_pick_supersedes_the_old_one(self, monkeypatch, events):
    nm = FakeNM()
    _away(nm, ID_PHONE)
    params = {"WifiManualPick": _pick(PHONE, ts=1.0)}
    def repick(nm, tk):
      if tk == 3:
        params["WifiManualPick"] = _pick(STAR, ts=2.0)
        nm.active = ID_STAR
        nm.ip[ID_STAR] = "10.0.0.3"
    run_loop(monkeypatch, nm, ticks=12, near_home=False, hooks=[repick], priority=(HOME, PHONE), params=params)
    assert ID_PHONE not in nm.ups and nm.active == ID_STAR, f"new pick not honoured: {nm.up_log}"
    assert ("netcosttier_pin_cleared", {"ssid": PHONE, "reason": "superseded", "by": STAR}) in events


class TestAnEndedPinStaysEnded:
  """Found by mutation: two ways a pin's lifecycle could go wrong without any earlier test noticing."""

  def test_an_ended_pin_does_not_come_back_when_its_network_returns(self, monkeypatch, events):
    """The pinned network drops (pin ends), then NM autoconnects it again with NO new pick. That is
    not the driver choosing it -- the ladder must treat it like any other network and move to the
    cheaper phone. Had the ended pin not been marked ended, it would silently resurrect here."""
    nm = FakeNM()
    _away(nm, ID_STAR, scan=(STAR,))
    params = {"WifiManualPick": _pick(STAR)}
    def drop_then_return(nm, tk):
      if tk == 2:
        nm.active = None                       # Starlink drops: pin ends
      if tk == 3:
        nm.active = ID_STAR                    # NM autoconnects it back -- no pick written
        nm.ip[ID_STAR] = "10.0.0.3"
        nm.scan = [STAR, PHONE]
    run_loop(monkeypatch, nm, ticks=14, near_home=False, hooks=[drop_then_return], priority=(HOME, PHONE),
             params=params)
    assert ID_PHONE in nm.ups, f"an ended pin resurrected and held a metered link: {nm.up_log}"
    assert names(events).count("netcosttier_pin_cleared") == 1, \
      f"pin_cleared must be logged once, not every tick: {names(events)}"

  def test_a_REPICK_whose_join_takes_a_moment_is_still_honoured(self, monkeypatch, events):
    """On a pinned phone, the driver picks KarlMoik. The phone is still the active link for a tick or
    two while NM switches. The NEW pin must start fresh -- carrying the old pick's 'seen active' would
    call it 'dropped' before the join lands, and the ladder would then pull him back to the phone."""
    nm = FakeNM()
    _away(nm, ID_PHONE)
    params = {"WifiManualPick": _pick(PHONE, ts=1.0)}
    def repick_slow_join(nm, tk):
      if tk == 3:
        params["WifiManualPick"] = _pick(STAR, ts=2.0)   # phone still active this tick
      if tk == 5:
        nm.active = ID_STAR
        nm.ip[ID_STAR] = "10.0.0.3"
    run_loop(monkeypatch, nm, ticks=16, near_home=False, hooks=[repick_slow_join], priority=(HOME, PHONE),
             params=params)
    later_phone = [t for t, c in nm.up_log if c == ID_PHONE]
    assert not later_phone and nm.active == ID_STAR, f"the new pick was overridden: {nm.up_log}"


class TestReleasingAPinByHand:
  def test_removing_WifiManualPick_releases_the_pin_and_the_ladder_resumes(self, monkeypatch, events):
    """The escape hatch from an SSH session: delete the param. (A reboot does the same through
    CLEAR_ON_MANAGER_START.) An ABSENT param is a real reading, unlike a failed one."""
    nm = FakeNM()
    _away(nm, ID_STAR)
    params = {"WifiManualPick": _pick(STAR)}
    def release(nm, tk):
      if tk == 3:
        params.pop("WifiManualPick")
    run_loop(monkeypatch, nm, ticks=14, near_home=False, hooks=[release], priority=(HOME, PHONE), params=params)
    assert ID_PHONE in nm.ups, f"a removed pick still held the radio: {nm.up_log}"
    cleared = [kw for n, kw in events if n == "netcosttier_pin_cleared"]
    assert cleared and cleared[0]["ssid"] == STAR and cleared[0]["reason"] == "param_removed"
    # netrank2pnw: params_pyx returns None for an absent key AND for undecodable JSON, so the event says so
    assert "not decodable" in cleared[0]["note"]


class TestPinAbsenceOfEvidence:
  def test_an_unreadable_ACTIVE_read_does_not_end_the_pin(self, monkeypatch, events):
    """nmcli failing to list active connections is not 'the driver's network dropped'."""
    nm = FakeNM()
    _away(nm, ID_STAR)
    params = {"WifiManualPick": _pick(STAR)}
    hooks = [lambda nm, tk: nm.fail_reads.add("--active") if tk == 2 else nm.fail_reads.discard("--active")]
    run_loop(monkeypatch, nm, ticks=10, near_home=False, hooks=hooks, priority=(HOME, PHONE), params=params)
    assert ID_PHONE not in nm.ups, f"one unreadable tick ended the pin: {nm.up_log}"
    assert not any(n == "netcosttier_pin_cleared" for n in names(events))

  def test_an_unreadable_PICK_param_is_logged_and_changes_nothing(self, monkeypatch, events):
    nm = FakeNM()
    _away(nm, ID_STAR)
    params = {"WifiManualPick": _pick(STAR)}
    def breaks(nm, tk):
      if tk == 3:
        params["WifiManualPick"] = RuntimeError("UnknownKeyName")
    run_loop(monkeypatch, nm, ticks=10, near_home=False, hooks=[breaks], priority=(HOME, PHONE), params=params)
    assert ID_PHONE not in nm.ups, f"a failed param read ended the pin: {nm.up_log}"
    assert "netcosttier_pin_unreadable" in names(events)
    assert not any(n == "netcosttier_pin_cleared" for n in names(events))

  def test_a_DAMAGED_pick_is_logged_not_honoured_silently(self, monkeypatch, events):
    nm = FakeNM()
    _away(nm, ID_STAR)
    run_loop(monkeypatch, nm, ticks=3, near_home=False, priority=(HOME, PHONE),
             params={"WifiManualPick": {"ts": 1.0}})
    unreadable = [kw for n, kw in events if n == "netcosttier_pin_unreadable"]
    assert unreadable and "ssid" in unreadable[0]["problem"]


class TestKillSwitchKeepsItsMeaning:
  def test_with_the_ladder_disabled_the_behaviour_is_pre_ladder_binary(self, monkeypatch, events):
    """DisableNetworkCostLadder = the old arbiter: a configured priority network, or our own hotspot.
    It never knew about manual picks, so a pick is NOT honoured and no upgrade scans run."""
    nm = FakeNM()
    _away(nm, ID_STAR)
    run_loop(monkeypatch, nm, ticks=4, near_home=False, priority=(HOME, PHONE), ladder=False,
             params={"WifiManualPick": _pick(STAR)})
    assert nm.ups and nm.ups[0] == HOTSPOT, f"kill switch did not give the pre-ladder binary behaviour: {nm.ups}"
    assert "netcosttier_upgrade_scan" not in names(events)
    assert "netcosttier_pin_held" not in names(events)


# ================================================================================================
# netrank2pnw — the driver's GENERIC ranking rule, the pin yielding to home, and the hardening Fable
# asked for after netscanpin2pnw.
#
# Driver, verbatim: "we don't want a solution where it just scans the Starlink SSIDs that I have
# configured; we need a generic solution where an unmetered network is always prioritized over a metered
# network or a default setting ... if I clearly have an unmetered network and the other network is set to
# either metered or default, then I made a conscious choice that this network is a priority if it's
# available."
# ================================================================================================

VISITOR, CAFE = "Visitor", "CafeFree"
ID_VISITOR, ID_CAFE, ID_HOME = (d.priority_connection_id(x) for x in (VISITOR, CAFE, HOME))


def _with(nm, *ssids):
  for x in ssids:
    cid = d.priority_connection_id(x)
    if cid not in nm.saved:
      nm.saved.append(cid)


class TestGenericRanking:
  def test_an_explicitly_unmetered_NON_member_beats_a_DEFAULT_configured_network(self, monkeypatch, events):
    """THE DRIVER'S RULE at loop level. On Visitor (configured, stationary, `unknown`) away from home. His
    phone -- NOT in the priority list in this scenario -- is explicitly unmetered and in range. The old
    rule kept him on Visitor forever (a configured entry was tier 0, and no upgrade scan ran on it)."""
    nm = FakeNM()
    _with(nm, VISITOR)
    nm.active, nm.ip[ID_VISITOR] = ID_VISITOR, "10.0.0.4"
    nm.scan, nm.metered = [VISITOR, PHONE], {PHONE: "no"}
    run_loop(monkeypatch, nm, ticks=6, near_home=False, priority=(HOME, VISITOR), mobile=())
    assert nm.ups[:1] == [ID_PHONE], f"stayed on a default network with an explicitly unmetered one in range: {nm.up_log}"
    assert "netcosttier_upgrade_scan" in names(events), "no upgrade scan ran on an `unknown` configured network"

  def test_membership_still_decides_WITHIN_a_cost_class(self, monkeypatch, events):
    """Visitor (configured) and a cafe (not configured) are both `unknown`: the configured one wins."""
    nm = FakeNM()
    _with(nm, VISITOR, CAFE)
    nm.active, nm.scan = HOTSPOT, [CAFE, VISITOR]
    run_loop(monkeypatch, nm, ticks=3, near_home=False, priority=(HOME, VISITOR), mobile=())
    assert nm.ups[:1] == [ID_VISITOR], f"membership did not break the tie: {nm.up_log}"

  def test_list_order_breaks_a_tie_between_two_members_of_one_class(self, monkeypatch, events):
    nm = FakeNM()
    _with(nm, VISITOR)
    nm.active, nm.scan = HOTSPOT, [VISITOR, HOME]
    run_loop(monkeypatch, nm, ticks=3, near_home=False, priority=(VISITOR, HOME), mobile=())
    assert nm.ups[:1] == [ID_VISITOR], f"the driver's list order was not the tiebreak: {nm.up_log}"

  def test_no_upgrade_scans_on_an_EXPLICITLY_unmetered_network_even_if_unconfigured(self, monkeypatch, events):
    """The generic scan rule, from the other side: nothing can beat an explicitly unmetered link on cost,
    configured or not."""
    nm = FakeNM()
    _with(nm, CAFE)
    nm.active, nm.ip[ID_CAFE] = ID_CAFE, "10.0.0.5"
    nm.scan, nm.metered = [CAFE, VISITOR], {CAFE: "no"}
    run_loop(monkeypatch, nm, ticks=10, near_home=False, priority=(HOME,), mobile=())
    assert nm.ups == [] and nm.scans == 0, f"ups={nm.up_log} scans={nm.scan_times}"


class TestOnPriorityNetworkMeaning:
  """OnPriorityNetwork is the uploader's `at_home`: on it, a metered link may still carry drive files."""

  def test_a_configured_but_EXPLICITLY_METERED_network_is_not_at_home(self, monkeypatch, events):
    nm = FakeNM()
    nm.active, nm.ip[ID_STAR] = ID_STAR, "10.0.0.3"
    nm.scan, nm.metered = [STAR], {STAR: "yes"}
    params = {}
    run_loop(monkeypatch, nm, ticks=3, near_home=False, priority=(HOME, STAR), mobile=(), params=params)
    assert params.get("OnPriorityNetwork") is False, "a configured metered link would authorise 75 MB uploads"

  def test_a_configured_DEFAULT_network_still_is(self, monkeypatch, events):
    nm = FakeNM()
    _with(nm, VISITOR)
    nm.active, nm.ip[ID_VISITOR] = ID_VISITOR, "10.0.0.4"
    nm.scan = [VISITOR]
    params = {}
    run_loop(monkeypatch, nm, ticks=3, near_home=False, priority=(VISITOR,), mobile=(), params=params)
    assert params.get("OnPriorityNetwork") is True

  def test_the_kill_switch_does_not_reopen_metered_uploads(self, monkeypatch, events):
    """Deliberate: DisableNetworkCostLadder reverts RADIO selection. It must not quietly re-authorise
    uploads over a link the driver marked metered."""
    nm = FakeNM()
    nm.active, nm.ip[ID_STAR] = ID_STAR, "10.0.0.3"
    nm.scan, nm.metered = [STAR], {STAR: "yes"}
    params = {}
    run_loop(monkeypatch, nm, ticks=3, near_home=True, priority=(STAR,), mobile=(), ladder=False, params=params)
    assert params.get("OnPriorityNetwork") is False


class TestPinYieldsToHome:
  """Fable D2, proposed to the driver, no veto: a pick holds until a STATIONARY configured network that is
  EXPLICITLY UNMETERED is in range."""

  def test_arriving_HOME_ends_a_pin_made_on_the_road_and_the_ladder_takes_home(self, monkeypatch, events):
    nm = FakeNM()
    _away(nm, ID_STAR, scan=(STAR,))
    nm.metered[HOME] = "no"
    # netrank2pnw: the truck must genuinely BE away when the pick is made. The first version of this test
    # left GPS on the home location the whole time, which the corrected rule rightly reads as "home was
    # visible at pick time" -- no scan ran and GPS said home -- so the pin stuck.
    near = {"home": False, "gps": (47.05, -122.0)}          # ~5.6 km north of the learned home location
    def arrive(nm, tk):
      if tk == 4:
        near["home"], near["gps"] = True, (47.0, -122.0)
        nm.scan = [STAR, HOME]
    run_loop(monkeypatch, nm, ticks=10, near_home=lambda: near["home"], gps=lambda: near["gps"],
             hooks=[arrive], priority=(HOME, PHONE),
             params={"WifiManualPick": _pick(STAR)})
    assert ID_HOME in nm.ups, f"a road pin kept the truck off the home network: {nm.up_log}"
    assert ("netcosttier_pin_cleared", {"ssid": STAR, "reason": "home", "to": HOME}) in events

  def test_the_MOBILE_phone_in_range_does_NOT_end_the_pin(self, monkeypatch, events):
    """Scans running (near a learned location), the explicitly unmetered phone in range, Starlink pinned.
    The phone is a mobile priority entry: exempting it would reopen the driver's measured case."""
    nm = FakeNM()
    _away(nm, ID_STAR)
    run_loop(monkeypatch, nm, ticks=8, near_home=True, priority=(HOME, PHONE),
             params={"WifiManualPick": _pick(STAR)})
    assert nm.scans > 0, "precondition: scans must run for this test to mean anything"
    assert ID_PHONE not in nm.ups, f"a mobile entry ended the pin: {nm.up_log}"
    assert not any(n == "netcosttier_pin_cleared" for n in names(events))

  def test_a_DEFAULT_cost_stationary_network_does_NOT_end_the_pin(self, monkeypatch, events):
    """Explicitly unmetered only. Visitor is configured, stationary, `unknown`."""
    nm = FakeNM()
    _with(nm, VISITOR)
    _away(nm, ID_STAR, scan=(STAR, VISITOR))
    run_loop(monkeypatch, nm, ticks=8, near_home=True, priority=(VISITOR, PHONE),
             params={"WifiManualPick": _pick(STAR)})
    assert nm.scans > 0 and ID_VISITOR not in nm.ups, f"an unknown-cost network ended the pin: {nm.up_log}"


class TestHoldSuppressesTheHotspotToo:
  """Fable D3: mutating the hold to drop `up_hotspot` survived all 208 netscanpin2pnw tests. This is the
  sequence that needs it."""

  def test_a_HIDDEN_ssid_pick_is_not_trampled_by_the_hotspot_during_its_join(self, monkeypatch, events):
    """A hidden network never appears in a scan. The driver enters it by name in Settings; the UI deletes
    and re-adds the profile, so for a moment NOTHING is active and nothing is in range. With no candidate,
    decide() returns up_hotspot -- and without the hold it would raise the comma's own AP over the join."""
    nm = FakeNM()
    hidden = "BackyardHidden"
    _with(nm, hidden)
    nm.active, nm.scan = None, []
    params = {"WifiManualPick": _pick(hidden)}
    def join_lands(nm, tk):
      if tk == 3:
        nm.active = d.priority_connection_id(hidden)
        nm.ip[nm.active] = "10.0.0.9"
    run_loop(monkeypatch, nm, ticks=8, near_home=False, hooks=[join_lands], priority=(HOME,), mobile=(),
             params=params)
    assert HOTSPOT not in nm.ups, f"raised the hotspot over the driver's hidden-network join: {nm.up_log}"
    assert any(n == "netcosttier_pin_held" and kw["suppressed"] == "up_hotspot" for n, kw in events)
    assert nm.active == d.priority_connection_id(hidden)


class TestNoScanInsideADhcpWindow:
  def test_the_immediate_first_upgrade_scan_waits_for_a_new_link_to_settle(self, monkeypatch, events):
    """Fable, netscanpin2pnw review: the first upgrade scan is immediate, so it could land inside the DHCP
    window of a link just brought up, taking the single radio off-channel while it negotiates."""
    nm = FakeNM()
    nm.active, nm.scan, nm.metered = HOTSPOT, [STAR], {STAR: "yes"}
    nm.behave[ID_STAR] = "no_dhcp"
    hooks = [lambda nm, tk: nm.ip.__setitem__(ID_STAR, "10.0.0.3") if tk == 3 and nm.active == ID_STAR else None]
    run_loop(monkeypatch, nm, ticks=8, near_home=False, hooks=hooks, priority=(HOME,), mobile=())
    raised = next(t for t, c in nm.up_log if c == ID_STAR)
    usable_at = 3 * POLL
    inside = [t for t in nm.scan_times if raised < t <= usable_at]
    assert not inside, f"scanned at {inside} while the link raised at {raised} was still getting an address"
    assert any(t > usable_at for t in nm.scan_times), "and the upgrade scan must still run once it settles"


class TestKillSwitchIsPreLadderBinary:
  def test_binary_mode_takes_the_first_configured_entry_and_ignores_cost(self, monkeypatch, events):
    """DisableNetworkCostLadder = the arbiter before cost existed: the first reachable configured entry,
    in list order -- even a default-cost one with an explicitly unmetered non-member in range."""
    nm = FakeNM()
    _with(nm, VISITOR, CAFE)
    nm.active, nm.scan, nm.metered = HOTSPOT, [CAFE, VISITOR], {CAFE: "no"}
    run_loop(monkeypatch, nm, ticks=3, near_home=True, priority=(VISITOR,), mobile=(), ladder=False)
    assert nm.ups[:1] == [ID_VISITOR], f"the kill switch still ranked by cost: {nm.up_log}"
    assert "netcosttier_upgrade_scan" not in names(events)

  def test_binary_mode_takes_a_METERED_configured_entry_too(self, monkeypatch, events):
    nm = FakeNM()
    nm.active, nm.scan, nm.metered = HOTSPOT, [STAR], {STAR: "yes"}
    run_loop(monkeypatch, nm, ticks=3, near_home=True, priority=(STAR,), mobile=(), ladder=False)
    assert nm.ups[:1] == [ID_STAR], f"pre-ladder binary has no notion of cost: {nm.up_log}"


class TestEveryMemberCountsNotJustTheFirstReachable:
  def test_a_second_member_beats_a_non_member_of_the_same_class(self, monkeypatch, events):
    """The daemon must pass EVERY configured entry to decide(). With only the first reachable one
    (select_available), HOME would count as a non-member and lose the unknown-class tie to CafeFree on
    ssid order -- the list would silently stop meaning anything past its first reachable entry."""
    nm = FakeNM()
    _with(nm, CAFE)
    nm.active, nm.scan, nm.metered = HOTSPOT, [STAR, HOME, CAFE], {STAR: "yes"}
    run_loop(monkeypatch, nm, ticks=3, near_home=False, priority=(STAR, HOME), mobile=())
    assert nm.ups[:1] == [ID_HOME], f"only the first reachable entry was treated as a member: {nm.up_log}"


class TestTheActiveLinkCompetesAtItsTrueCost:
  def test_on_metered_starlink_a_DEFAULT_network_later_in_the_alphabet_still_wins(self, monkeypatch, events):
    """The active profile's cost is read at the top of the tick and excluded from the later bulk read, so
    it must be MERGED back. Without the merge the active Starlink ranks as `unknown`, ties with ZedCafe,
    and wins on ssid order -- a metered link kept over a default one."""
    nm = FakeNM()
    zed = "ZedCafe"
    _with(nm, zed)
    nm.active, nm.ip[ID_STAR] = ID_STAR, "10.0.0.3"
    nm.scan, nm.metered = [STAR, zed], {STAR: "yes"}
    run_loop(monkeypatch, nm, ticks=4, near_home=False, priority=(HOME,), mobile=())
    assert nm.ups[:1] == [d.priority_connection_id(zed)], f"the active metered link was ranked as unknown: {nm.up_log}"


class TestAMobileNetworkNeverEndsAPinEvenWhenItArrives:
  def test_the_phone_going_away_and_coming_back_does_not_end_a_pin(self, monkeypatch, events):
    """Found by mutation: once a pin could only yield to a network that ARRIVED, the earlier "mobile phone in
    range does not end a pin" test stopped exercising the mobile exclusion -- the phone sat continuously in
    range, so it never arrived, and treating mobile entries as homes survived. Here the explicitly unmetered
    phone genuinely leaves (four real scans) and returns while Starlink is pinned. It is still not home."""
    nm = FakeNM()
    _away(nm, ID_STAR)
    def phone_leaves_and_returns(nm, tk):
      nm.scan = [STAR] if 2 <= tk < 6 else [STAR, PHONE]
    run_loop(monkeypatch, nm, ticks=14, near_home=True, hooks=[phone_leaves_and_returns], priority=(HOME, PHONE),
             params={"WifiManualPick": _pick(STAR)})
    assert ID_PHONE not in nm.ups, f"a mobile entry ended the pin after 'arriving': {nm.up_log}"
    assert not any(n == "netcosttier_pin_cleared" for n in names(events))


class TestAPickMadeAtHomeSticks:
  """netrank2pnw D2 correction. The case approved was "pick Starlink, drive home, and the truck stays on
  paid Starlink in the driveway". A pick made deliberately WHILE home is visible must stick -- otherwise a
  manual pick at home reverts on the next tick and is useless there."""

  @staticmethod
  def _at_home(nm):
    _away(nm, ID_STAR, scan=(STAR, HOME))
    nm.metered[HOME] = "no"

  def test_a_pick_made_while_home_is_visible_sticks_with_home_continuously_in_range(self, monkeypatch, events):
    nm = FakeNM()
    self._at_home(nm)
    run_loop(monkeypatch, nm, ticks=25, near_home=True, priority=(HOME, PHONE),
             params={"WifiManualPick": _pick(STAR)})
    assert nm.scans >= 20, "precondition: home must be seen by real scans throughout"
    assert ID_HOME not in nm.ups, f"a pick made at home reverted: {nm.up_log}"
    assert not any(n == "netcosttier_pin_cleared" for n in names(events))

  def test_home_FLICKERING_in_scan_results_does_not_end_it(self, monkeypatch, events):
    """Present, missing, present, missing ... and runs shorter than the absence threshold."""
    nm = FakeNM()
    self._at_home(nm)
    pattern = [1, 0, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 0, 1]
    def flicker(nm, tk):
      nm.scan = [STAR, HOME] if pattern[tk % len(pattern)] else [STAR]
    # concrete numbers, deliberately not the constant: runs of at most TWO missing scans. A test sized from
    # PIN_HOME_ABSENT_SCANS would move with any mutation of it and pin nothing.
    assert "000" not in "".join(map(str, pattern * 2)), "precondition: never three misses in a row"
    run_loop(monkeypatch, nm, ticks=40, near_home=True, hooks=[flicker], priority=(HOME, PHONE),
             params={"WifiManualPick": _pick(STAR)})
    assert ID_HOME not in nm.ups, f"a flickering home network ended the pick: {nm.up_log}"

  def test_but_a_home_that_is_GENUINELY_absent_and_then_returns_does_end_it(self, monkeypatch, events):
    """The other side of the flicker rule: absence across enough REAL scans is genuine."""
    nm = FakeNM()
    self._at_home(nm)
    def outage(nm, tk):
      nm.scan = [STAR] if 2 <= tk < 6 else [STAR, HOME]     # FOUR consecutive real scans without home
    run_loop(monkeypatch, nm, ticks=12, near_home=True, hooks=[outage], priority=(HOME, PHONE),
             params={"WifiManualPick": _pick(STAR)})
    assert ID_HOME in nm.ups, f"a genuinely returning home network did not end the pin: {nm.up_log}"
    assert ("netcosttier_pin_cleared", {"ssid": STAR, "reason": "home", "to": HOME}) in events

  def test_SCANS_THAT_FAIL_at_home_are_not_absence(self, monkeypatch, events):
    """nmcli failing to scan for several ticks says nothing about whether home left."""
    nm = FakeNM()
    self._at_home(nm)
    hooks = [lambda nm, tk: nm.fail_reads.add("dev wifi list") if 2 <= tk < 8 else nm.fail_reads.discard("dev wifi list")]
    run_loop(monkeypatch, nm, ticks=14, near_home=True, hooks=hooks, priority=(HOME, PHONE),
             params={"WifiManualPick": _pick(STAR)})
    assert ID_HOME not in nm.ups, f"failed scans were read as home leaving: {nm.up_log}"

  def test_a_REPICK_at_home_after_a_road_pick_sticks(self, monkeypatch, events):
    """Arrival evidence belongs to ONE pick. A road pick establishes that home was away; a NEW pick made
    once home must start from nothing, or it would inherit that and revert."""
    nm = FakeNM()
    _away(nm, ID_STAR, scan=(STAR,))
    nm.metered[HOME] = "no"
    params = {"WifiManualPick": _pick(STAR, ts=1.0)}
    where = {"gps": (47.05, -122.0), "near": False}
    def trip(nm, tk):
      if tk == 3:                                      # still away, while pinned: re-pick after arriving
        where["gps"], where["near"] = (47.0, -122.0), True
        nm.scan = [STAR, HOME]
        params["WifiManualPick"] = _pick(STAR, ts=2.0)
    # ordering matters: arrival and the new pick land on the same tick, so no tick sees the OLD pin at home
    run_loop(monkeypatch, nm, ticks=16, near_home=lambda: where["near"], gps=lambda: where["gps"],
             hooks=[trip], priority=(HOME, PHONE), params=params)
    assert ID_HOME not in nm.ups, f"the new pick inherited the old pick's arrival evidence: {nm.up_log}"
