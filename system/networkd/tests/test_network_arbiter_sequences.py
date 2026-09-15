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
    # hotspotretry2pnw: connection id -> the stderr nmcli prints when `con up` on it fails. Only read
    # for a `refuse`; unset means a bare failure with no text, which classifies as `real` (i.e. exactly
    # the pre-change behaviour, which is why no existing scenario had to change).
    self.up_error: dict[str, str] = {}
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
      if b == "late":
        # hotspotretry2pnw: nmcli returned an error (our own NMCLI_TIMEOUT_S can kill it at 15 s) but
        # NetworkManager carried the activation through anyway -- a failing rc with a link that is up.
        self.active, self.ip[c] = c, "10.0.0.2"
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

  def _proc(args):
    """hotspotretry2pnw: `_con_up` needs the return code and stderr, so it goes through `_nmcli_run`
    rather than `_nmcli`. Delegate to the SAME FakeNM state machine so every scenario still drives it,
    and render a failure as rc=4 plus whatever nm.up_error scripts for that connection."""
    out = nm.nmcli(args)
    if out is None:
      return subprocess.CompletedProcess(args, 4, "", nm.up_error.get(args[-1], ""))
    return subprocess.CompletedProcess(args, 0, out, "")

  monkeypatch.setattr(d, "_nmcli_run", _proc)
  monkeypatch.setattr(d, "_run", lambda args: subprocess.CompletedProcess(args, 0, "", ""))
  monkeypatch.setattr(d, "_modem_index", lambda: None)
  monkeypatch.setattr(d, "_lte_throttled_recently", lambda: False)
  monkeypatch.setattr(d, "_lte_has_ip", lambda: False)
  # netrank2pnw: gps may be a callable, so a sequence can DRIVE (a pin's arrival evidence reads it)
  monkeypatch.setattr(d, "_read_gps", lambda p, m: gps() if callable(gps) else gps)
  # netrank2pnw: near_home may be a callable, so a sequence can ARRIVE home mid-run
  # gpscarry2pnw: "real" keeps the real geo-gate, so a test can see which GPS reached it
  if near_home != "real":
    monkeypatch.setattr(d, "near_any_home", lambda locs, gps: near_home() if callable(near_home) else near_home)
  monkeypatch.setattr(d, "_usable_cache", {})
  # netrank2pnw: _metered_cache is a module global too. It was never reset here, so a value cached by one
  # test leaked into the next -- a "failed read with nothing cached" could not be reproduced reliably and
  # results could depend on test order (Fable had to reset it by hand in its probe).
  monkeypatch.setattr(d, "_metered_cache", {})

  nets = [{"label": s, "ssid": s, "lat": None if s in mobile else 47.0, "lon": None if s in mobile else -122.0,
           "mobile": s in mobile} for s in priority]

  class P:
    def get_bool(self, k):
      if k == "DisableNetworkCostLadder":
        return not ladder
      if k == "IsOnroad":            # gpscarry2pnw: offroad unless a test says otherwise (hooks may flip it)
        return bool(params.get("IsOnroad", False))
      return k == "TetheringEnabled"

    def get(self, k):
      if k == "TetheringPriorityNetworks":
        return json.dumps(nets)
      v = params.get(k)
      if isinstance(v, Exception):
        raise v
      return v

    def put(self, k, v):
      params[k] = v          # gpscarry2pnw: record, so a location write is assertable (get() above ignores it)

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
    joins it). The phone is still in range and cheaper. The arbiter must leave him on KarlMoik.

    pinunmetered2pnw CHANGED THE LAST ASSERTION: it said no upgrade scan runs while pinned. Now the scan runs on a
    pinned link that is not explicitly unmetered -- an unmetered network ARRIVING ends the pin (owner, 2026-09-14),
    and away from home that scan is the only way to see one arrive. The phone here was in range when the pick was
    made and stays in range, so it never arrives, and the pick still sticks -- and the log says why."""
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
    assert names(events).count("netcosttier_upgrade_scan") >= 2, "no upgrade scan while pinned on metered Starlink"
    kept = [kw for n, kw in events if n == "netcosttier_pin_kept"]
    assert [(k["ssid"], k["by"], k["reason"]) for k in kept] == [(STAR, PHONE, "visible_since_pick")], kept
    assert "out of range since the pick" in kept[0]["rule"], kept

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
    assert ("netcosttier_pin_cleared", {"ssid": STAR, "reason": "home", "trigger": HOME}) in events

  def test_the_MOBILE_phone_in_range_does_NOT_end_the_pin(self, monkeypatch, events):
    """Scans running (near a learned location), the explicitly unmetered phone in range, Starlink pinned.
    The phone is a mobile priority entry: exempting it would reopen the driver's measured case.
    pinunmetered2pnw: the phone CAN now end a metered pin, but only by arriving -- it is in range at the pick and
    never leaves, so this pick still sticks (the owner's 2026-09-13 decision)."""
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


class TestAMobileNetworkThatArrivesIsNotHomeButIsUnmetered:
  def test_the_phone_going_away_and_coming_back_ends_a_METERED_pin_as_unmetered_not_as_home(self, monkeypatch, events):
    """Found by mutation (netrank2pnw): once a pin could only yield to a network that ARRIVED, the earlier "mobile
    phone in range does not end a pin" test stopped exercising the mobile exclusion. Here the explicitly unmetered
    phone genuinely leaves (four real scans) and returns while Starlink is pinned.

    INVERTED by pinunmetered2pnw. This asserted the pin HOLDS. The owner decided on 2026-09-14 ("when an unmetered
    network appears !") that an unmetered network arriving ends a pin on a network that is not unmetered, mobile or
    not. What survives of the netrank2pnw rule is that the phone is still not HOME: the reason is `unmetered`."""
    nm = FakeNM()
    _away(nm, ID_STAR)
    def phone_leaves_and_returns(nm, tk):
      nm.scan = [STAR] if 2 <= tk < 6 else [STAR, PHONE]
    run_loop(monkeypatch, nm, ticks=14, near_home=True, hooks=[phone_leaves_and_returns], priority=(HOME, PHONE),
             params={"WifiManualPick": _pick(STAR)})
    assert nm.up_log == [(6 * POLL, ID_PHONE)], f"the returning phone did not end the metered pin: {nm.up_log}"
    cleared = [kw for n, kw in events if n == "netcosttier_pin_cleared"]
    assert cleared == [{"ssid": STAR, "reason": "unmetered", "by": PHONE}], cleared


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

  def test_home_missing_from_scans_while_GPS_says_the_truck_is_STILL_HOME_does_not_end_it(self, monkeypatch, events):
    """INVERTED by Fable's netrank2pnw review (D2). This test used to assert the opposite -- that four
    consecutive real scans without home, then its return, ENDED an at-home pin. Parked at home that is one
    minute of silence from the AP: a router reboot, a 5 GHz DFS channel check, or a weak signal from the
    garage -- and a weak home AP is the likeliest reason to pick Starlink at home at all. With GPS placing
    the truck at the learned home location throughout, the truck never left, so the pick stands."""
    nm = FakeNM()
    self._at_home(nm)
    def outage(nm, tk):
      nm.scan = [STAR] if 2 <= tk < 6 else [STAR, HOME]     # FOUR consecutive real scans without home
    run_loop(monkeypatch, nm, ticks=14, near_home=True, hooks=[outage], priority=(HOME, PHONE),
             gps=(47.0, -122.0), params={"WifiManualPick": _pick(STAR)})
    assert ID_HOME not in nm.ups, f"an AP outage ended an at-home pick while GPS said we never moved: {nm.up_log}"
    assert not any(n == "netcosttier_pin_cleared" for n in names(events))

  def test_WITHOUT_GPS_a_genuine_scan_absence_still_ends_it(self, monkeypatch, events):
    """Scan-count absence survives exactly where GPS cannot speak: no fix. Four real scans without home,
    then back -> the pin ends and the ladder takes home."""
    nm = FakeNM()
    self._at_home(nm)
    def outage(nm, tk):
      nm.scan = [STAR] if 2 <= tk < 6 else [STAR, HOME]
    run_loop(monkeypatch, nm, ticks=12, near_home=True, hooks=[outage], priority=(HOME, PHONE),
             gps=None, params={"WifiManualPick": _pick(STAR)})
    assert ID_HOME in nm.ups, f"with no GPS, a genuine absence did not end the pin: {nm.up_log}"
    assert ("netcosttier_pin_cleared", {"ssid": STAR, "reason": "home", "trigger": HOME}) in events

  def test_home_flickering_WITHOUT_GPS_does_not_end_it_either(self, monkeypatch, events):
    """The flicker guard where it still carries weight: no fix, so scan misses are the only evidence, and a
    presence must reset them. (With GPS at home misses do not count at all, which hid mutation A3.)"""
    nm = FakeNM()
    self._at_home(nm)
    pattern = [1, 0, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0]
    assert "000" not in "".join(map(str, pattern * 2)), "precondition: never three misses in a row"
    def flicker(nm, tk):
      nm.scan = [STAR, HOME] if pattern[tk % len(pattern)] else [STAR]
    run_loop(monkeypatch, nm, ticks=36, near_home=True, hooks=[flicker], priority=(HOME, PHONE),
             gps=None, params={"WifiManualPick": _pick(STAR)})
    assert ID_HOME not in nm.ups, f"flicker without GPS ended the pick: {nm.up_log}"

  def test_SCANS_THAT_FAIL_at_home_are_not_absence(self, monkeypatch, events):
    """nmcli failing to scan for several ticks says nothing about whether home left. Run WITHOUT GPS: with
    GPS at home the D2 guard refuses to count misses anyway, which hid mutation A4."""
    nm = FakeNM()
    self._at_home(nm)
    hooks = [lambda nm, tk: nm.fail_reads.add("dev wifi list") if 2 <= tk < 8 else nm.fail_reads.discard("dev wifi list")]
    run_loop(monkeypatch, nm, ticks=14, near_home=True, hooks=hooks, priority=(HOME, PHONE),
             gps=None, params={"WifiManualPick": _pick(STAR)})
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


class TestACostSwitchNeedsACostThatWasRead:
  """Fable D1 (netrank2pnw review), measured: when the ACTIVE profile's connection.metered read fails and
  nothing is cached, choose_wifi ranked the active link as unknown and an explicitly unmetered candidate
  outranked it -- home torn down for the phone at boot, when NM is slowest, and restored 20 s later."""

  @staticmethod
  def _flaky_home_read(nm, fail_until):
    orig = nm.nmcli
    def flaky(args):
      if "connection.metered" in " ".join(args) and args[-1] == ID_HOME and nm.t < fail_until:
        return None
      return orig(args)
    nm.nmcli = flaky

  def test_one_failed_first_read_of_the_active_cost_does_not_tear_down_home(self, monkeypatch, events):
    """Fable's harness, verbatim scenario: at home on Hannelore (`no`), phone (`no`) in range, the FIRST
    read of Hannelore's cost fails. Before the fix: [(0 s, iPhone), (20 s, Hannelore)]."""
    nm = FakeNM()
    nm.active, nm.ip[ID_HOME] = ID_HOME, "10.0.0.7"
    nm.scan, nm.metered = [HOME, PHONE], {HOME: "no", PHONE: "no"}
    self._flaky_home_read(nm, fail_until=1.0)
    run_loop(monkeypatch, nm, ticks=4, near_home=True, priority=(HOME, PHONE))
    assert nm.up_log == [], f"one unreadable first read tore down home for the phone: {nm.up_log}"
    held = [kw for n, kw in events if n == "netcosttier_active_cost_unreadable"]
    assert held and held[0]["active"] == HOME, "the hold must be LOUD, not silent"

  def test_a_persistently_unreadable_cost_holds_and_logs_ONCE(self, monkeypatch, events):
    nm = FakeNM()
    nm.active, nm.ip[ID_HOME] = ID_HOME, "10.0.0.7"
    nm.scan, nm.metered = [HOME, PHONE], {HOME: "no", PHONE: "no"}
    self._flaky_home_read(nm, fail_until=1e9)
    run_loop(monkeypatch, nm, ticks=8, near_home=True, priority=(HOME, PHONE))
    assert nm.up_log == []
    assert names(events).count("netcosttier_active_cost_unreadable") == 1, "change-only, not every tick"

  def test_an_unreadable_cost_does_NOT_hold_a_DEAD_link(self, monkeypatch, events):
    """The exemption: moving off a link that has failed its usability check is failure-driven, not
    cost-driven. A hold must never keep the device on a dead link."""
    nm = FakeNM()
    nm.active, nm.ip[ID_HOME] = ID_HOME, None               # associated, never gets an address
    nm.scan, nm.metered = [HOME, PHONE], {HOME: "no", PHONE: "no"}
    self._flaky_home_read(nm, fail_until=1e9)
    run_loop(monkeypatch, nm, ticks=8, near_home=True, priority=(HOME, PHONE))
    assert ID_PHONE in nm.ups, f"held on a dead link because its cost was unreadable: {nm.up_log}"

  def test_a_successful_read_of_UNKNOWN_is_not_a_failed_read(self, monkeypatch, events):
    """The distinction the fix rests on: `unknown` is an answer. A default-cost active link with an
    explicitly unmetered candidate in range is displaced, exactly as the generic rule says."""
    nm = FakeNM()
    _with(nm, VISITOR)
    nm.active, nm.ip[ID_VISITOR] = ID_VISITOR, "10.0.0.4"
    nm.scan, nm.metered = [VISITOR, PHONE], {PHONE: "no"}   # Visitor reads `unknown` successfully
    run_loop(monkeypatch, nm, ticks=4, near_home=True, priority=(HOME, VISITOR), mobile=())
    assert nm.ups[:1] == [ID_PHONE], f"a successful `unknown` was treated as unreadable: {nm.up_log}"
    assert "netcosttier_active_cost_unreadable" not in names(events)

  def test_a_SECOND_unreadable_episode_is_logged_again(self, monkeypatch, events):
    """Change-only must reset when a read succeeds -- otherwise the second hold is silent."""
    nm = FakeNM()
    nm.active, nm.ip[ID_HOME] = ID_HOME, "10.0.0.7"
    nm.scan, nm.metered = [HOME, PHONE], {HOME: "no", PHONE: "no"}
    orig = nm.nmcli
    def flaky(args):
      bad = nm.t < 30.0 or 60.0 <= nm.t < 90.0              # fail, recover, fail again
      if "connection.metered" in " ".join(args) and args[-1] == ID_HOME and bad:
        return None
      return orig(args)
    nm.nmcli = flaky
    # the cache must not paper over the second failure: clear it once recovery has been observed
    hooks = [lambda nm, tk: d._metered_cache.clear() if tk == 3 else None]
    run_loop(monkeypatch, nm, ticks=6, near_home=True, hooks=hooks, priority=(HOME, PHONE))
    assert nm.up_log == []
    assert names(events).count("netcosttier_active_cost_unreadable") == 2, names(events)

  def test_the_KILL_SWITCH_is_exempt(self, monkeypatch, events):
    """Binary mode has no notion of cost, so there is nothing cost-driven to hold: on a non-priority link
    it still tears down to the hotspot exactly as before the ladder, unreadable cost or not."""
    nm = FakeNM()
    nm.active, nm.ip[ID_STAR] = ID_STAR, "10.0.0.3"
    nm.scan = [STAR]
    orig = nm.nmcli
    nm.nmcli = lambda args: None if ("connection.metered" in " ".join(args) and args[-1] == ID_STAR) else orig(args)
    run_loop(monkeypatch, nm, ticks=3, near_home=False, priority=(HOME,), mobile=(), ladder=False)
    assert nm.ups[:1] == [HOTSPOT], f"the cost-unreadable hold leaked into binary mode: {nm.up_log}"


# ================================================================================================
# gpscarry2pnw — while the ignition is off, the last fresh fix stands in for GPS (Fable, netrank2pnw re-review).
#
# qcomgpsd runs only while `started`, so parked with the device awake _read_gps reads None 10 s after the
# ignition goes off: the GPS veto on a scan gap disappeared (an at-home pin ended by a 60 s home-AP gap) and
# the geo-gate failed open (a scan every 20 s on client WiFi away from any learned location). `gps` in this
# harness is what _read_gps returns, i.e. a FRESH fix; the carry itself runs for real inside main().
# ================================================================================================

AT_HOME_FIX, FAR_FIX = (47.0, -122.0), (47.05, -122.0)   # the harness's learned home, and ~5.6 km north of it


def _car(params, car, schedule):
  """Hook: at tick tk in `schedule`, set (IsOnroad, the fresh fix _read_gps returns)."""
  def hook(nm, tk):
    if tk in schedule:
      params["IsOnroad"], car["gps"] = schedule[tk]
  return hook


def _home_gap(first, last):
  """Hook: home missing from the real scans for ticks first..last-1 (4 ticks = past the 3-scan threshold)."""
  def hook(nm, tk):
    nm.scan = [STAR] if first <= tk < last else [STAR, HOME]
  return hook


def _carry_events(events, which):
  return [kw for n, kw in events if n == f"network_arbiterd_gps_carry_{which}"]


class TestAParkedDeviceKeepsItsLastFix:
  @staticmethod
  def _at_home_pinned(nm):
    _away(nm, ID_STAR, scan=(STAR, HOME))
    nm.metered[HOME] = "no"

  def test_ignition_off_at_home_then_10_MIN_then_a_scan_gap_does_not_end_the_pin(self, monkeypatch, events):
    """Fable's (a): parked at home, device awake, Starlink picked by hand. Ten minutes after the ignition went
    off, the home AP drops out of four scans. GPS said the truck never left before the ignition went off, and a
    parked truck does not move, so the pick stands -- as it does with the ignition on."""
    nm = FakeNM()
    self._at_home_pinned(nm)
    params, car = {"WifiManualPick": _pick(STAR), "IsOnroad": True}, {"gps": AT_HOME_FIX}
    off = 3
    gap = off + int(600 / POLL)                                  # 10 min after the ignition went off
    run_loop(monkeypatch, nm, ticks=gap + 8, near_home=True, gps=lambda: car["gps"], priority=(HOME, PHONE),
             hooks=[_car(params, car, {off: (False, None)}), _home_gap(gap, gap + 4)], params=params)
    assert nm.scans >= gap, "precondition: real scans must run throughout"
    assert ID_HOME not in nm.ups, f"a scan gap ended an at-home pin while the truck was parked: {nm.up_log}"
    assert not any(n == "netcosttier_pin_cleared" for n in names(events))
    started = _carry_events(events, "started")
    assert len(started) == 1, f"the carry must be logged ONCE when it starts: {started}"
    assert 0 < started[0]["fix_age_s"] <= POLL, started
    assert _carry_events(events, "ended") == []

  def test_with_NO_fix_seen_this_boot_it_is_todays_behaviour(self, monkeypatch, events):
    """Onroad without ever getting a fresh fix (no sky view), then the ignition goes off: nothing to carry, so
    the same gap ends the pin exactly as it does today without GPS."""
    nm = FakeNM()
    self._at_home_pinned(nm)
    params, car = {"WifiManualPick": _pick(STAR), "IsOnroad": True}, {"gps": None}
    off, gap = 3, 33
    run_loop(monkeypatch, nm, ticks=gap + 8, near_home=True, gps=lambda: car["gps"], priority=(HOME, PHONE),
             hooks=[_car(params, car, {off: (False, None)}), _home_gap(gap, gap + 4)], params=params)
    assert ID_HOME in nm.ups, f"with no fix to carry, a genuine absence did not end the pin: {nm.up_log}"
    assert ("netcosttier_pin_cleared", {"ssid": STAR, "reason": "home", "trigger": HOME}) in events
    assert _carry_events(events, "started") == [] and _carry_events(events, "ended") == []

  def test_ignition_ON_without_fresh_GPS_drops_the_carried_fix_and_fails_open(self, monkeypatch, events):
    """The carry ends the moment IsOnroad turns on, fresh GPS or not. From then today's rule applies: a truck
    that drives away with GPS not (yet) fresh has no GPS, its scan misses count, and home ends the pin when it
    comes back. Carrying on would reopen 'GPS frozen at home while the truck drives away'."""
    nm = FakeNM()
    self._at_home_pinned(nm)
    params, car = {"WifiManualPick": _pick(STAR), "IsOnroad": True}, {"gps": AT_HOME_FIX}
    run_loop(monkeypatch, nm, ticks=20, near_home=True, gps=lambda: car["gps"], priority=(HOME, PHONE),
             hooks=[_car(params, car, {3: (False, None), 10: (True, None)}), _home_gap(11, 15)], params=params)
    ended = _carry_events(events, "ended")
    assert len(ended) == 1 and ended[0]["reason"].startswith("IsOnroad=1"), ended
    assert ID_HOME in nm.ups, f"the carried fix outlived the ignition turning on: {nm.up_log}"
    assert ("netcosttier_pin_cleared", {"ssid": STAR, "reason": "home", "trigger": HOME}) in events

  def test_the_fix_is_DROPPED_on_ignition_on_not_suspended(self, monkeypatch, events):
    """On, drive with GPS dead, off again somewhere else: the pre-drive position must not come back."""
    nm = FakeNM()
    self._at_home_pinned(nm)
    params, car = {"WifiManualPick": _pick(STAR), "IsOnroad": True}, {"gps": AT_HOME_FIX}
    schedule = {3: (False, None), 10: (True, None), 14: (False, None)}
    run_loop(monkeypatch, nm, ticks=25, near_home=True, gps=lambda: car["gps"], priority=(HOME, PHONE),
             hooks=[_car(params, car, schedule), _home_gap(16, 20)], params=params)
    assert len(_carry_events(events, "started")) == 1, "the second ignition-off had no fresh fix to carry"
    assert ID_HOME in nm.ups, f"a fix from before the drive came back after it: {nm.up_log}"

  def test_a_DAEMON_RESTART_carries_nothing(self, monkeypatch, events):
    """The carry lives in main()'s locals. A new process (a crash restart, a manager restart, a reboot -- or the
    device carried to the other car) starts with no fix, even though the ignition is still off."""
    nm = FakeNM()
    self._at_home_pinned(nm)
    params, car = {"WifiManualPick": _pick(STAR), "IsOnroad": True}, {"gps": AT_HOME_FIX}
    run_loop(monkeypatch, nm, ticks=6, near_home=True, gps=lambda: car["gps"], priority=(HOME, PHONE),
             hooks=[_car(params, car, {3: (False, None)})], params=params)
    assert len(_carry_events(events, "started")) == 1, "precondition: the first process was carrying"
    seen = len(events)
    run_loop(monkeypatch, nm, ticks=10, near_home=True, gps=None, priority=(HOME, PHONE),
             hooks=[_home_gap(2, 6)], params=params)
    assert _carry_events(events[seen:], "started") == [], "a restarted daemon carried a fix it never saw"
    assert ID_HOME in nm.ups, f"the restarted daemon behaved as if it had GPS: {nm.up_log}"

  def test_parked_AWAY_on_client_wifi_the_geo_gate_stays_shut(self, monkeypatch, events):
    """Fable's (b): parked ~5.6 km from the only learned location, on the (explicitly unmetered) phone. With
    the ignition off the geo-gate used to fail open and scan every 20 s (~2.8 s latency bump each)."""
    def scans(gps_while_on):
      nm = FakeNM()
      nm.active, nm.ip[ID_PHONE], nm.scan, nm.metered = ID_PHONE, "10.0.0.2", [PHONE], {PHONE: "no"}
      params, car = {"IsOnroad": True}, {"gps": gps_while_on}
      run_loop(monkeypatch, nm, ticks=20, near_home="real", gps=lambda: car["gps"], priority=(HOME, PHONE),
               hooks=[_car(params, car, {3: (False, None)})], params=params)
      return nm.scans
    assert scans(None) > 10, "precondition: with no fix the gate fails open and scans"
    assert scans(FAR_FIX) == 0, "the parked device's last fix did not reach the geo-gate"

  def test_a_carried_fix_is_never_LEARNED_as_a_location(self, monkeypatch, events):
    """Learning persists a location, so it stays on fresh GPS. IsOnroad=0 is not proof the truck stands still
    (startup blocked, thermal offroad): here it reaches home WiFi while still carrying a fix from 5.6 km away,
    which must not overwrite home's learned location."""
    nm = FakeNM()
    nm.active, nm.ip[ID_PHONE], nm.scan, nm.metered = ID_PHONE, "10.0.0.2", [PHONE], {PHONE: "no", HOME: "no"}
    params, car = {"IsOnroad": True}, {"gps": FAR_FIX}
    def reach_home(nm, tk):
      if tk == 6:
        nm.active, nm.ip[ID_HOME], nm.scan = ID_HOME, "10.0.0.7", [HOME, PHONE]
    run_loop(monkeypatch, nm, ticks=10, near_home=True, gps=lambda: car["gps"], priority=(HOME, PHONE),
             hooks=[_car(params, car, {3: (False, None)}), reach_home], params=params)
    assert _carry_events(events, "started"), "precondition: the fix was being carried"
    assert "TetheringPriorityNetworks" not in params, f"learned a carried fix: {params.get('TetheringPriorityNetworks')}"


# ================================================================================================
# smallfix0914pnw — every change of the active wifi connection is LOGGED, including ones the arbiter did
# not make. Measured 2026-09-13 22:01 PT on the truck: the phone dropped, NetworkManager autoconnected
# KarlMoik by itself (its profile has autoconnect=yes), and the arbiter's log said nothing at all -- the
# switch was only visible as the local IP moving from 172.20.10.10 to 192.168.1.79 in the cloud log.
# ================================================================================================

@pytest.fixture
def infos(monkeypatch):
  got: list[str] = []
  monkeypatch.setattr(d.cloudlog, "info", lambda msg, *a, **k: got.append(str(msg)))
  return got


def _changes(events):
  return [kw for n, kw in events if n == "network_arbiter_active_changed"]


class TestActiveConnectionChangesAreLogged:
  @staticmethod
  def _phone_drops_and_NM_joins_karlmoik(nm, tk):
    if tk == 2:
      nm.active, nm.ip[ID_STAR], nm.scan = ID_STAR, "10.0.0.3", [STAR]

  def test_NM_moving_the_phone_to_KarlMoik_by_itself_is_logged_as_external(self, monkeypatch, events):
    """THE 2026-09-13 22:01 PT CASE. Logged exactly once (change-only, and nothing at startup), the arbiter
    does not fight NM's choice, and its next decisions are made from KarlMoik -- on a metered link it runs
    an upgrade scan for something cheaper, and it does not treat the link as gone."""
    nm = FakeNM()
    _away(nm, ID_PHONE, scan=(PHONE, STAR))
    run_loop(monkeypatch, nm, ticks=9, near_home=False, hooks=[self._phone_drops_and_NM_joins_karlmoik],
             priority=(HOME, PHONE))
    assert _changes(events) == [{"from": ID_PHONE, "to": ID_STAR, "by": "external"}], _changes(events)
    assert nm.ups == [], f"fought NetworkManager's own choice: {nm.up_log}"
    scans = [kw for n, kw in events if n == "netcosttier_upgrade_scan"]
    assert scans and scans[0]["active"] == ID_STAR, f"the next decisions did not see KarlMoik: {scans}"

  def test_the_arbiters_own_switch_back_is_logged_as_the_arbiters_and_says_what_it_leaves(self, monkeypatch, events, infos):
    """The phone comes back; the ladder (unchanged) moves off metered KarlMoik. The bring-up line used to
    say '(dropping hotspot)' here although the device was leaving a client WiFi."""
    nm = FakeNM()
    _away(nm, ID_PHONE, scan=(PHONE, STAR))
    def phone_returns(nm, tk):
      if tk == 6:
        nm.scan = [STAR, PHONE]
    run_loop(monkeypatch, nm, ticks=14, near_home=False, hooks=[self._phone_drops_and_NM_joins_karlmoik, phone_returns],
             priority=(HOME, PHONE))
    assert nm.ups == [ID_PHONE], nm.up_log
    assert _changes(events) == [{"from": ID_PHONE, "to": ID_STAR, "by": "external"},
                                {"from": ID_STAR, "to": ID_PHONE, "by": "arbiter"}], _changes(events)
    ups = [m for m in infos if "in range ->" in m]
    assert len(ups) == 1 and f"(leaving {ID_STAR})" in ups[0], ups

  def test_leaving_the_hotspot_still_says_so(self, monkeypatch, events, infos):
    nm = FakeNM()
    _away(nm, HOTSPOT, scan=(PHONE,))
    run_loop(monkeypatch, nm, ticks=3, near_home=False, priority=(HOME, PHONE))
    assert _changes(events) == [{"from": HOTSPOT, "to": ID_PHONE, "by": "arbiter"}], _changes(events)
    ups = [m for m in infos if "in range ->" in m]
    assert len(ups) == 1 and ups[0].endswith("(dropping hotspot)"), ups

  def test_a_REFUSED_bring_up_is_not_blamed_on_someone_else(self, monkeypatch, events, infos):
    """The arbiter asked for the phone and got nothing: that is its own request not landing, not an
    external change -- the event says what it had asked for."""
    nm = FakeNM()
    nm.active, nm.scan, nm.metered = HOTSPOT, [PHONE], {PHONE: "no"}
    nm.behave[ID_PHONE] = "refuse"
    run_loop(monkeypatch, nm, ticks=3, near_home=False, priority=())
    assert _changes(events)[:2] == [
      {"from": HOTSPOT, "to": None, "by": "not_as_requested", "arbiter_requested": ID_PHONE},
      {"from": None, "to": HOTSPOT, "by": "arbiter"},
    ], _changes(events)
    fallback = [m for m in infos if "falling back to saved wifi" in m]
    assert fallback and fallback[0].endswith("(dropping hotspot)"), fallback

  def test_an_UNREADABLE_active_read_is_not_a_change(self, monkeypatch, events, infos):
    """An nmcli failure is no evidence the connection changed -- it must not log 'phone -> nothing' and then
    'nothing -> phone' around one flaky call.
    unreadhold2pnw: nor does it re-run `con up` on the phone, which is in the scan. Before the hold it did, and on
    real NM that re-activates a working link (Fable). The '(active connection unreadable)' wording is still
    reachable after the bound -- see TestAnUnreadableActiveReadHoldsTheRadio."""
    nm = FakeNM()
    _away(nm, ID_PHONE, scan=(PHONE,))
    hooks = [lambda nm, tk: nm.fail_reads.add("--active") if tk == 2 else nm.fail_reads.discard("--active")]
    run_loop(monkeypatch, nm, ticks=6, near_home=False, hooks=hooks, priority=(HOME, PHONE))
    assert _changes(events) == [], _changes(events)
    assert nm.active == ID_PHONE
    assert nm.ups == [], f"re-upped the already-active link on an unreadable tick: {nm.up_log}"
    assert [m for m in infos if "in range ->" in m] == []

  def test_an_EARLIER_arbiter_switch_does_not_colour_a_later_external_one(self, monkeypatch, events):
    """Found by mutation: the arbiter raises the phone from the hotspot, then NM moves it to KarlMoik on its
    own. A request that already landed must not make the later change read as the arbiter's (or as its
    request failing)."""
    nm = FakeNM()
    _away(nm, HOTSPOT, scan=(PHONE,))
    run_loop(monkeypatch, nm, ticks=6, near_home=False, hooks=[self._phone_drops_and_NM_joins_karlmoik],
             priority=(HOME, PHONE))
    assert _changes(events) == [{"from": HOTSPOT, "to": ID_PHONE, "by": "arbiter"},
                                {"from": ID_PHONE, "to": ID_STAR, "by": "external"}], _changes(events)


# ================================================================================================
# unreadhold2pnw — an UNREADABLE active connection does not move the radio, for a bounded time (Fable, confirmed).
#
# A failed `nmcli con show --active` reads as current_active None: on_client_wifi goes False, the sticky seeding
# never runs, and the scan commonly omits the connected AP -- so decide() raised the hotspot on a working link.
# Fable's loop harness, on the phone: [(40 s, Hotspot), (60 s, iPhone)] from ONE nmcli hiccup.
# ================================================================================================

HOLD_S = d.ACTIVE_UNREADABLE_HOLD_S


def _unreadable(ticks, *, scan_then=None):
  """Hook: `con show --active` fails on the ticks in `ticks`. `scan_then`, if given, is the scan on those ticks
  (the scan on the others is [PHONE])."""
  def hook(nm, tk):
    bad = tk in ticks
    (nm.fail_reads.add if bad else nm.fail_reads.discard)("--active")
    if scan_then is not None:
      nm.scan = list(scan_then) if bad else [PHONE]
  return hook


def _holds(events):
  return [kw for n, kw in events if n == "network_arbiter_active_unreadable_hold"]


def _releases(events):
  return [kw for n, kw in events if n == "network_arbiter_active_unreadable_released"]


class TestAnUnreadableActiveReadHoldsTheRadio:
  def test_FABLES_SCENARIO_one_hiccup_with_the_AP_missing_from_the_scan_raises_no_hotspot(self, monkeypatch, events):
    """On the phone; one tick where the active read fails AND the scan omits the phone. Before: [(40, Hotspot),
    (60, iPhone)] -- a 20-40 s bounce on a working link plus hotspot NAT churn. The hold is logged, once."""
    nm = FakeNM()
    _away(nm, ID_PHONE, scan=(PHONE,))
    run_loop(monkeypatch, nm, ticks=6, near_home=False, hooks=[_unreadable({2}, scan_then=())], priority=(HOME, PHONE))
    assert nm.up_log == [], f"one unreadable tick bounced a working link: {nm.up_log}"
    assert _holds(events) == [{"suppressed": "up_hotspot", "target": "", "unreadable_s": 0.0, "hold_s": HOLD_S}], events
    assert _releases(events) == []

  @pytest.mark.parametrize("scan_then, first_up", [((), HOTSPOT), ((PHONE,), ID_PHONE)])
  def test_a_PERSISTENT_failure_acts_as_before_once_the_bound_passes_and_says_so_loudly(self, monkeypatch, events, infos,
                                                                                       scan_then, first_up):
    """nmcli broken for minutes must not strand the device on a link nobody can see. The failure starts at 40 s;
    the hold covers every tick while under HOLD_S, then today's action goes through. Logged change-only: one hold
    event, one release at error level -- not one per tick."""
    nm = FakeNM()
    _away(nm, ID_PHONE, scan=(PHONE,))
    run_loop(monkeypatch, nm, ticks=10, near_home=False, hooks=[_unreadable(range(2, 10**6), scan_then=scan_then)],
             priority=(HOME, PHONE))
    released_at = 2 * POLL + HOLD_S          # first tick whose unreadable run is >= HOLD_S old
    assert [u for u in nm.up_log if u[0] < released_at] == [], f"acted inside the hold: {nm.up_log}"
    assert nm.up_log[:1] == [(released_at, first_up)], f"did not fall back once the bound passed: {nm.up_log}"
    assert len(_holds(events)) == 1, f"the hold must be logged change-only: {_holds(events)}"
    rel = _releases(events)
    assert len(rel) == 1 and rel[0]["unreadable_s"] == HOLD_S and rel[0]["action"] == ("up_hotspot" if first_up == HOTSPOT
                                                                                      else "up_priority"), rel
    assert "error" in rel[0], "the release must be logged at ERROR level (cloudlog.event error=)"
    if first_up == ID_PHONE:
      ups = [m for m in infos if "in range ->" in m]
      assert ups[:1] and ups[0].endswith("(active connection unreadable)"), ups

  @pytest.mark.parametrize("scan, first_up", [((PHONE,), ID_PHONE), ((), HOTSPOT)])
  def test_at_BOOT_with_nothing_to_hold_an_unreadable_read_does_not_delay_the_first_connection(self, monkeypatch, events,
                                                                                               scan, first_up):
    """Fable re-review: nothing active and `--active` failing from tick 0 (boot, nmcli slow under build-on-boot).
    The hold has nothing to protect, so the first action must go through on tick 0 -- not at HOLD_S."""
    nm = FakeNM()
    _away(nm, None, scan=scan)
    nm.fail_reads.add("--active")            # failing BEFORE the first tick's read, not only from the first hook on
    run_loop(monkeypatch, nm, ticks=3, near_home=False, hooks=[_unreadable(range(0, 10**6))], priority=(HOME, PHONE))
    assert nm.up_log[:1] == [(0.0, first_up)], f"boot connection delayed by the unreadable hold: {nm.up_log}"
    # Once our own bring-up is requested, the next unreadable ticks DO hold (Fable's scenario-E protection):
    # the link we just raised is not bounced while nobody can read it.
    assert nm.up_log == [(0.0, first_up)], f"the fresh bring-up was bounced while unreadable: {nm.up_log}"
    assert all(h["unreadable_s"] < HOLD_S for h in _holds(events)), _holds(events)

  def test_a_good_read_RESETS_the_bound_and_afterwards_the_arbiter_acts_normally(self, monkeypatch, events):
    """Two episodes of 5 unreadable ticks (80 s each) with one good read between them: neither reaches the bound,
    because the run restarts on a good read -- without the reset the second one would release at 160 s. Each is
    logged. Then the phone genuinely goes away, read successfully, and the hotspot comes up on that very tick."""
    nm = FakeNM()
    _away(nm, ID_PHONE, scan=())
    def phone_gone(nm, tk):
      if tk == 14:
        nm.active = None
    run_loop(monkeypatch, nm, ticks=16, near_home=False,
             hooks=[_unreadable(set(range(2, 7)) | set(range(8, 13))), phone_gone], priority=(HOME, PHONE))
    assert nm.up_log == [(14 * POLL, HOTSPOT)], nm.up_log
    assert len(_holds(events)) == 2, f"a new episode must be logged again: {_holds(events)}"
    assert _releases(events) == []


# ================================================================================================
# arbiterfu2pnw (1) — an UNREADABLE verification read is not a failed bring-up (Fable follow-up to unreadhold2pnw).
#
# After `con up`, the next tick judges the pending bring-up. A failed `nmcli con show --active` read as "nothing
# active", so judge_link concluded "the bring-up never took", blamed the network and started its retry backoff --
# on no evidence at all. Inside ACTIVE_UNREADABLE_HOLD_S the judgement now waits for the next good read; past the
# bound it is made as before, and said at ERROR level.
# ================================================================================================

@pytest.fixture
def timed_events(monkeypatch):
  """(t, name, kwargs) for every cloudlog.event -- the sequences below are about WHEN a network is blamed."""
  got: list[tuple[float, str, dict]] = []
  clock = {"nm": None}
  monkeypatch.setattr(d.cloudlog, "event", lambda name, **kw: got.append((clock["nm"].t, name, kw)))
  return got, clock


def _named(timed, name):
  return [(t, kw) for t, n, kw in timed if n == name]


class TestAnUnreadableVerificationReadIsNotAFailure:
  def test_ONE_unreadable_read_right_after_a_bring_up_blames_nobody(self, monkeypatch, timed_events):
    """The phone comes up fine at 0 s; the read at 20 s fails. Before: netcosttier_assoc_failed for the phone (a
    60 s backoff) and netcosttier_recovered at 40 s. Now: nothing in the ledger, and the deferral is logged once."""
    timed, clock = timed_events
    nm = FakeNM()
    clock["nm"] = nm
    _away(nm, HOTSPOT, scan=(PHONE,))
    run_loop(monkeypatch, nm, ticks=6, near_home=False, hooks=[_unreadable({1})], priority=(HOME, PHONE))
    assert nm.up_log == [(0.0, ID_PHONE)], nm.up_log
    assert _named(timed, "netcosttier_assoc_failed") == [], _named(timed, "netcosttier_assoc_failed")
    assert _named(timed, "netcosttier_recovered") == []
    deferred = _named(timed, "netcosttier_verify_deferred")
    assert [(t, kw["ssid"]) for t, kw in deferred] == [(POLL, PHONE)], deferred

  def test_a_bring_up_that_REALLY_failed_is_blamed_on_the_next_GOOD_read(self, monkeypatch, timed_events):
    """The phone refuses at 0 s, the read at 20 s fails, the read at 40 s succeeds and shows nothing active: that
    is the evidence, so the phone is blamed at 40 s and the hotspot comes back on that same tick."""
    timed, clock = timed_events
    nm = FakeNM()
    clock["nm"] = nm
    _away(nm, HOTSPOT, scan=(PHONE,))
    nm.behave[ID_PHONE] = "refuse"
    run_loop(monkeypatch, nm, ticks=4, near_home=False, hooks=[_unreadable({1})], priority=(HOME, PHONE))
    assert nm.up_log == [(0.0, ID_PHONE), (2 * POLL, HOTSPOT)], nm.up_log
    failed = _named(timed, "netcosttier_assoc_failed")
    assert [(t, kw["ssid"], kw["consecutive_failures"]) for t, kw in failed] == [(2 * POLL, PHONE.lower(), 1)], failed
    assert _named(timed, "netcosttier_blamed_unverified") == []

  def test_the_DHCP_grace_still_counts_from_the_bring_up_and_a_dead_link_is_blamed_once(self, monkeypatch, timed_events):
    """The phone associates at 0 s but never gets an address; the read at 20 s fails. Before: blamed at 20 s
    (failure 1), its grace RESTARTED at 40 s as a 'new' link, blamed again at 100 s (failure 2, a 5 min backoff),
    hotspot at 100 s. Now: one failure, at the end of the grace from the bring-up, hotspot at 60 s."""
    timed, clock = timed_events
    nm = FakeNM()
    clock["nm"] = nm
    _away(nm, HOTSPOT, scan=(PHONE,))
    nm.behave[ID_PHONE] = "no_dhcp"
    run_loop(monkeypatch, nm, ticks=6, near_home=False, hooks=[_unreadable({1})], priority=(HOME, PHONE))
    assert nm.up_log == [(0.0, ID_PHONE), (d.DHCP_GRACE_S, HOTSPOT)], nm.up_log
    failed = _named(timed, "netcosttier_assoc_failed")
    assert [(t, kw["consecutive_failures"]) for t, kw in failed] == [(d.DHCP_GRACE_S, 1)], failed

  def test_a_SECOND_unreadable_episode_is_logged_again_and_does_not_stretch_the_grace(self, monkeypatch, timed_events):
    """No address ever; reads fail at 20 s and 60 s. Each episode logs its deferral (re-armed by the good read at 40 s),
    and the dead link is blamed on the first GOOD read past the grace (80 s) -- not held off by the second episode."""
    timed, clock = timed_events
    nm = FakeNM()
    clock["nm"] = nm
    _away(nm, HOTSPOT, scan=(PHONE,))
    nm.behave[ID_PHONE] = "no_dhcp"
    run_loop(monkeypatch, nm, ticks=6, near_home=False, hooks=[_unreadable({1, 3})], priority=(HOME, PHONE))
    assert [t for t, _kw in _named(timed, "netcosttier_verify_deferred")] == [POLL, 3 * POLL], timed
    assert nm.up_log == [(0.0, ID_PHONE), (4 * POLL, HOTSPOT)], nm.up_log
    assert [t for t, _kw in _named(timed, "netcosttier_assoc_failed")] == [4 * POLL]

  def test_a_PERSISTENTLY_unreadable_nmcli_blames_at_the_bound_as_before_and_says_so_loudly(self, monkeypatch, timed_events):
    """Reads fail from 20 s on, forever. The bring-up is judged when the unreadable run reaches HOLD_S (at 140 s,
    the tick the hold releases): blamed as before, with an ERROR-level event saying no verification read was
    possible -- so the released action moves on to the hotspot instead of re-upping the phone blind."""
    timed, clock = timed_events
    nm = FakeNM()
    clock["nm"] = nm
    _away(nm, HOTSPOT, scan=(PHONE,))
    run_loop(monkeypatch, nm, ticks=8, near_home=False, hooks=[_unreadable(range(1, 10**6))], priority=(HOME, PHONE))
    bound = POLL + HOLD_S
    assert nm.up_log == [(0.0, ID_PHONE), (bound, HOTSPOT)], nm.up_log
    failed = _named(timed, "netcosttier_assoc_failed")
    assert [(t, kw["ssid"]) for t, kw in failed] == [(bound, PHONE.lower())], failed
    loud = _named(timed, "netcosttier_blamed_unverified")
    assert [(t, kw["ssid"], kw["unreadable_s"]) for t, kw in loud] == [(bound, PHONE.lower(), HOLD_S)], loud
    assert "error" in loud[0][1], "the unverified blame must be logged at ERROR level (cloudlog.event error=)"
    assert len(_named(timed, "netcosttier_verify_deferred")) == 1, "the deferral is logged once, not every tick"

  def test_at_BOOT_the_first_connection_is_not_delayed_and_not_blamed(self, monkeypatch, timed_events):
    """unreadhold2pnw's boot rule, kept: reads failing from before tick 0, the phone is raised at 0 s. And the
    unreadable ticks that follow no longer put it in backoff."""
    timed, clock = timed_events
    nm = FakeNM()
    clock["nm"] = nm
    _away(nm, None, scan=(PHONE,))
    nm.fail_reads.add("--active")
    run_loop(monkeypatch, nm, ticks=4, near_home=False, hooks=[_unreadable(range(10**6))], priority=(HOME, PHONE))
    assert nm.up_log == [(0.0, ID_PHONE)], f"boot connection delayed or bounced: {nm.up_log}"
    assert _named(timed, "netcosttier_assoc_failed") == [], _named(timed, "netcosttier_assoc_failed")

  def test_a_deferral_never_happens_without_the_hold__double_fault(self, monkeypatch, timed_events):
    """Fable's probe. The phone refuses at 0 s. At 20 s a GOOD read shows nothing active, so seen_active is '' and
    nothing is requested, but the loop raises before judging, so the pending bring-up survives. Reads fail from 40 s.
    Without the gate the judgement was deferred with no hold: the refusing phone was re-upped blind at 40 s and the
    hotspot waited until 160 s. With it: there is nothing to hold, so the phone is blamed at 40 s and the hotspot
    comes up on that tick."""
    timed, clock = timed_events
    nm = FakeNM()
    clock["nm"] = nm
    _away(nm, HOTSPOT, scan=(PHONE,))
    nm.behave[ID_PHONE] = "refuse"
    real_saved = d._saved_connections

    def saved_raising_on_the_judging_tick():
      if nm.t == POLL:
        raise RuntimeError("synthetic nmcli failure on the judging tick")
      return real_saved()
    monkeypatch.setattr(d, "_saved_connections", saved_raising_on_the_judging_tick)
    monkeypatch.setattr(d.cloudlog, "exception", lambda *a, **k: None)
    run_loop(monkeypatch, nm, ticks=4, near_home=False, hooks=[_unreadable(range(2, 10**6))], priority=(HOME, PHONE))
    assert nm.up_log == [(0.0, ID_PHONE), (2 * POLL, HOTSPOT)], nm.up_log
    assert _named(timed, "netcosttier_verify_deferred") == []


# ================================================================================================
# arbiterfu2pnw (2) — the fallback line says what happened. It said "no priority network in range", which netrank2pnw
# made false: a configured network can be in range and lose on cost, or be excluded by a backoff or a missing profile.
# LOG TEXT ONLY: every test also pins the bring-up, and a broken explanation must not stop it.
# ================================================================================================

def _fallback_lines(infos):
  return [m for m in infos if "falling back to saved wifi" in m]


class TestTheFallbackLineTellsTheTruth:
  def test_a_configured_network_IN_RANGE_that_loses_on_cost_is_named_as_outranked(self, monkeypatch, infos):
    """The phone is configured, in range and explicitly metered; CafeFree is not configured and explicitly unmetered.
    Cost dominates membership, so CafeFree wins -- with the phone in range, which the old line denied."""
    nm = FakeNM()
    _with(nm, CAFE)
    nm.active, nm.scan, nm.metered = HOTSPOT, [PHONE, CAFE], {PHONE: "yes", CAFE: "no"}
    run_loop(monkeypatch, nm, ticks=2, near_home=False, priority=(HOME, PHONE))
    assert nm.up_log == [(0.0, ID_CAFE)], nm.up_log
    lines = _fallback_lines(infos)
    assert len(lines) == 1, lines
    assert "no priority network in range" not in lines[0], lines[0]
    want = f"[unmetered, not a configured priority network -- '{HOME}': not in the scan; '{PHONE}': in range, metered, outranked]"
    assert want in lines[0], lines[0]
    assert lines[0].endswith("(dropping hotspot)"), lines[0]

  def test_a_configured_network_in_FAILURE_BACKOFF_is_named_as_such(self, monkeypatch, infos):
    """The phone (configured, unmetered) refuses at 0 s and serves a backoff; CafeFree (cost unknown) is joined at 20 s."""
    nm = FakeNM()
    _with(nm, CAFE)
    nm.active, nm.scan, nm.metered = HOTSPOT, [PHONE, CAFE], {PHONE: "no"}
    nm.behave[ID_PHONE] = "refuse"
    run_loop(monkeypatch, nm, ticks=3, near_home=False, priority=(PHONE,))
    assert nm.up_log == [(0.0, ID_PHONE), (POLL, ID_CAFE)], nm.up_log
    lines = _fallback_lines(infos)
    assert len(lines) == 1 and f"[cost unknown, not a configured priority network -- '{PHONE}': in range, in failure backoff]" \
      in lines[0], lines

  def test_a_configured_network_in_range_WITHOUT_a_saved_profile_is_named_as_such(self, monkeypatch, infos):
    nm = FakeNM()
    _with(nm, CAFE)
    nm.active, nm.scan = HOTSPOT, [VISITOR, CAFE]
    run_loop(monkeypatch, nm, ticks=2, near_home=False, priority=(VISITOR,))
    assert nm.up_log == [(0.0, ID_CAFE)], nm.up_log
    lines = _fallback_lines(infos)
    assert len(lines) == 1 and f"-- '{VISITOR}': in range, no saved profile]" in lines[0], lines

  def test_with_no_priority_networks_configured_it_says_that(self, monkeypatch, infos):
    nm = FakeNM()
    nm.active, nm.scan, nm.metered = HOTSPOT, [PHONE], {PHONE: "no"}
    run_loop(monkeypatch, nm, ticks=2, near_home=False, priority=())
    lines = _fallback_lines(infos)
    assert len(lines) == 1 and "[unmetered -- no priority networks configured]" in lines[0], lines

  def test_still_logged_only_when_the_radio_moves_not_every_tick(self, monkeypatch, infos):
    nm = FakeNM()
    _with(nm, CAFE)
    nm.active, nm.scan, nm.metered = HOTSPOT, [PHONE, CAFE], {PHONE: "yes", CAFE: "no"}
    run_loop(monkeypatch, nm, ticks=8, near_home=False, priority=(HOME, PHONE))
    assert nm.up_log == [(0.0, ID_CAFE)], nm.up_log
    assert len(_fallback_lines(infos)) == 1, _fallback_lines(infos)

  def test_the_explanation_is_computed_from_the_inputs_decide_RANKED(self, monkeypatch, infos):
    """Wiring: on metered KarlMoik (configured, active, usable) an upgrade scan finds CafeFree, unmetered and not
    configured. The explanation must see what decide() saw -- including the active link -- or it can contradict it."""
    nm = FakeNM()
    _with(nm, CAFE)
    _away(nm, ID_STAR, scan=(STAR, CAFE))
    nm.metered[CAFE] = "no"
    ranked, explained = [], []
    real_decide, real_explain = d.decide, d.explain_fallback
    def spy_decide(**kw):
      out = real_decide(**kw)
      ranked.append((kw, out))
      return out
    def spy_explain(*a, **kw):
      explained.append((a, kw))
      return real_explain(*a, **kw)
    monkeypatch.setattr(d, "decide", spy_decide)
    monkeypatch.setattr(d, "explain_fallback", spy_explain)
    run_loop(monkeypatch, nm, ticks=3, near_home=False, priority=(HOME, STAR))
    assert nm.up_log == [(POLL, ID_CAFE)], nm.up_log      # the first upgrade scan waits one tick for the link to settle
    (kw, out), = [r for r in ranked if r[1][0] == "up_fallback"]
    (args, ekw), = explained
    assert args == (kw["priority_ssids"], kw["scan_ssids"], kw["saved_connections"], out[1], kw["metered_ssids"],
                    kw["unmetered_ssids"], kw["blocked_ssids"], kw["active_ssid"]), (args, kw)
    assert kw["active_ssid"] == STAR and ekw == {"scanned": True}, (kw["active_ssid"], ekw)
    assert f"'{STAR}': in range, metered, outranked]" in _fallback_lines(infos)[0], infos

  def test_a_BROKEN_explanation_never_stops_the_bring_up_it_describes(self, monkeypatch, infos):
    """Log text must not be able to alter a decision. An exception in it would escape to the loop's `except
    Exception` and skip _apply -- the device would stay on the hotspot. It is caught, logged, and the join happens."""
    nm = FakeNM()
    _with(nm, CAFE)
    nm.active, nm.scan, nm.metered = HOTSPOT, [PHONE, CAFE], {PHONE: "yes", CAFE: "no"}
    def boom(*a, **k):
      raise ValueError("synthetic")
    monkeypatch.setattr(d, "explain_fallback", boom)
    exceptions: list[str] = []
    monkeypatch.setattr(d.cloudlog, "exception", lambda msg, *a, **k: exceptions.append(str(msg)))
    run_loop(monkeypatch, nm, ticks=2, near_home=False, priority=(HOME, PHONE))
    assert nm.up_log == [(0.0, ID_CAFE)], f"a log-text failure changed the decision: {nm.up_log}"
    assert len(exceptions) == 1 and "explain" in exceptions[0], f"explained only for the up_fallback tick: {exceptions}"
    lines = _fallback_lines(infos)
    assert len(lines) == 1 and "[reason unavailable: ValueError]" in lines[0], lines


# ================================================================================================
# pinunmetered2pnw — a manual pick ENDS when an explicitly unmetered network ARRIVES (owner decision 2026-09-14).
#
# Asked "should a manual WiFi pick end on its own when you mark that network metered, or when an unmetered network
# appears?", the owner answered, verbatim: "when an unmetered network appears !". The case that night: KarlMoik (the
# mobile Starlink) picked by hand and marked metered; the iPhone hotspot turned on; the truck stayed on KarlMoik,
# because only a STATIONARY home could end a pin and no upgrade scan ran while pinned. The 2026-09-13 decisions still
# hold: a hand pick sticks, and a pick made while the other network is ALREADY visible sticks.
# ================================================================================================

def _cleared(events):
  return [kw for n, kw in events if n == "netcosttier_pin_cleared"]


def _kept(events):
  return [(kw["ssid"], kw["by"], kw["reason"]) for n, kw in events if n == "netcosttier_pin_kept"]


def _scan_by_tick(present, absent_ticks, base=(STAR,)):
  """Hook: the scan lists `base`, plus `present` on every tick NOT in `absent_ticks`."""
  def hook(nm, tk):
    nm.scan = list(base) if tk in absent_ticks else [*base, present]
  return hook


class TestAnUnmeteredNetworkThatArrivesEndsThePin:
  def test_TONIGHT_karlmoik_pinned_and_marked_metered_then_the_iPhone_hotspot_turns_on(self, monkeypatch, events):
    """THE 2026-09-14 CASE, away from home (the geo-gate shut, so the 120 s upgrade scan is the only scan). KarlMoik
    is picked while still `unknown`, marked metered in Settings a few ticks later (never a pick), and the iPhone
    hotspot, off all along, turns on at 400 s. The next upgrade scan sees it arrive: the pin ends and the ladder
    joins the phone."""
    nm = FakeNM()
    _away(nm, ID_STAR, scan=(STAR,))
    nm.metered[STAR] = "unknown"
    on_at = 20 * POLL
    def evening(nm, tk):
      if tk == 5:
        nm.metered[STAR] = "yes"
      if tk == 20:
        nm.scan = [STAR, PHONE]
    run_loop(monkeypatch, nm, ticks=40, near_home=False, gps=FAR_FIX, hooks=[evening], priority=(HOME, PHONE),
             params={"WifiManualPick": _pick(STAR)})
    assert [c for _t, c in nm.up_log] == [ID_PHONE], f"the iPhone did not take over from the pin: {nm.up_log}"
    took = nm.up_log[0][0]
    assert on_at < took <= on_at + d.UPGRADE_SCAN_S, f"took {took - on_at:.0f} s after the hotspot turned on"
    assert _cleared(events) == [{"ssid": STAR, "reason": "unmetered", "by": PHONE}], _cleared(events)
    assert _kept(events) == [], "a network that arrived must not be reported as kept"
    before = [t for t in nm.scan_times if t < took]
    assert len(before) >= 4 and all(b - a >= d.UPGRADE_SCAN_S for a, b in zip(before, before[1:], strict=False)), \
      f"expected throttled upgrade scans while pinned: {nm.scan_times}"

  def test_a_pick_made_while_the_iPhone_is_ALREADY_visible_sticks_and_the_log_says_why(self, monkeypatch, events):
    """The 2026-09-13 decision, with scans now running while pinned: ten minutes, the phone in every scan, never
    joined. The reason is logged ONCE, not per scan."""
    nm = FakeNM()
    _away(nm, ID_STAR)
    run_loop(monkeypatch, nm, ticks=31, near_home=False, gps=FAR_FIX, priority=(HOME, PHONE),
             params={"WifiManualPick": _pick(STAR)})
    assert nm.scans >= 4, f"precondition: the phone must be seen by real scans while pinned: {nm.scan_times}"
    assert nm.ups == [], f"a pick made with the phone in range was overridden: {nm.up_log}"
    assert _cleared(events) == []
    assert _kept(events) == [(STAR, PHONE, "visible_since_pick")], _kept(events)

  def test_a_NON_configured_saved_network_marked_unmetered_does_NOT_end_it_and_the_log_says_why(self, monkeypatch, events):
    """pinconfigured2pnw -- INVERTED. pinunmetered2pnw let any saved profile end a pin. Asked to confirm "any saved
    network marked unmetered counts, not just your configured ones", the owner answered on 2026-09-14 ~21:30 PT,
    verbatim: "(no just the configued ones)". A saved cafe the driver marked unmetered, NOT in the list, genuinely
    leaves and comes back while metered Starlink is pinned. The ladder would take it; the pin holds; the log says why,
    once."""
    nm = FakeNM()
    _with(nm, CAFE)
    _away(nm, ID_STAR, scan=(STAR,))
    nm.metered[CAFE] = "no"
    run_loop(monkeypatch, nm, ticks=14, near_home=True, hooks=[_scan_by_tick(CAFE, range(6))], priority=(HOME, PHONE),
             params={"WifiManualPick": _pick(STAR)})
    assert nm.ups == [] and nm.active == ID_STAR, f"an unconfigured unmetered network ended the pin: {nm.up_log}"
    assert _cleared(events) == [], _cleared(events)
    assert any(n == "netcosttier_pin_held" and kw["target"] == CAFE for n, kw in events), \
      "precondition: the ladder must want the cafe, or this test proves nothing"
    assert _kept(events) == [(STAR, CAFE, "not_configured")], _kept(events)
    assert "TetheringPriorityNetworks" in next(kw["rule"] for n, kw in events if n == "netcosttier_pin_kept")

  def test_the_same_network_CONFIGURED_as_a_MOBILE_entry_ends_it(self, monkeypatch, events):
    """The identical sequence with the cafe added to the list as a mobile entry (as the iPhone is): the list is the
    only difference, and the pin ends as `unmetered`."""
    nm = FakeNM()
    _with(nm, CAFE)
    _away(nm, ID_STAR, scan=(STAR,))
    nm.metered[CAFE] = "no"
    run_loop(monkeypatch, nm, ticks=10, near_home=True, hooks=[_scan_by_tick(CAFE, range(6))],
             priority=(HOME, PHONE, CAFE), mobile=(PHONE, CAFE), params={"WifiManualPick": _pick(STAR)})
    assert nm.up_log == [(6 * POLL, ID_CAFE)], nm.up_log
    assert _cleared(events) == [{"ssid": STAR, "reason": "unmetered", "by": CAFE}], _cleared(events)
    assert _kept(events) == [], _kept(events)

  def test_the_same_network_CONFIGURED_as_a_STATIONARY_entry_ends_it_as_home(self, monkeypatch, events):
    """And as a stationary entry. Every stationary network the `unmetered` rule accepts, the home rule accepts too, and
    home is judged first -- so the reason is `home` (away from its learned location, then in range)."""
    nm = FakeNM()
    _with(nm, CAFE)
    _away(nm, ID_STAR, scan=(STAR,))
    nm.metered[CAFE] = "no"
    run_loop(monkeypatch, nm, ticks=10, near_home=True, gps=FAR_FIX, hooks=[_scan_by_tick(CAFE, range(6))],
             priority=(HOME, PHONE, CAFE), mobile=(PHONE,), params={"WifiManualPick": _pick(STAR)})
    assert nm.up_log == [(6 * POLL, ID_CAFE)], nm.up_log
    assert _cleared(events) == [{"ssid": STAR, "reason": "home", "trigger": CAFE}], _cleared(events)

  def test_a_STATIONARY_configured_network_visible_at_the_pick_is_logged_as_such_not_as_unconfigured(self, monkeypatch,
                                                                                                     events):
    """Found by mutation: the daemon must hand the rule EVERY configured entry, not only mobile ones. Starlink picked at
    home with Hannelore (stationary, unmetered) in range: the pin sticks, and the kept line must say why truthfully --
    Hannelore IS configured."""
    nm = FakeNM()
    _away(nm, ID_STAR, scan=(STAR, HOME))
    nm.metered[HOME] = "no"
    run_loop(monkeypatch, nm, ticks=6, near_home=True, priority=(HOME, PHONE), params={"WifiManualPick": _pick(STAR)})
    assert nm.ups == [] and _cleared(events) == [], nm.up_log
    assert _kept(events) == [(STAR, HOME, "visible_since_pick")], _kept(events)

  def test_a_REPICK_starts_its_log_afresh(self, monkeypatch, events):
    """Found by mutation: the kept line is once per network PER PICK. A second pick of the same network with the
    phone still in range is a new decision, and the log says so again."""
    nm = FakeNM()
    _away(nm, ID_STAR)
    params = {"WifiManualPick": _pick(STAR, ts=1.0)}
    def repick(nm, tk):
      if tk == 10:
        params["WifiManualPick"] = _pick(STAR, ts=2.0)
    run_loop(monkeypatch, nm, ticks=20, near_home=True, hooks=[repick], priority=(HOME, PHONE), params=params)
    assert nm.ups == []
    assert _kept(events) == [(STAR, PHONE, "visible_since_pick")] * 2, _kept(events)


class TestWhatDoesNotEndThePin:
  def test_a_pin_on_an_EXPLICITLY_UNMETERED_network_is_not_ended_by_another_one(self, monkeypatch, events):
    """Nothing is cheaper than an explicitly unmetered pin. The driver picked a saved cafe (`no`, not configured);
    his phone (`no`, configured) leaves and comes back. Without the pin the ladder WOULD take the phone -- a member
    beats a non-member inside one cost class -- so the hold is the rule's doing."""
    nm = FakeNM()
    _with(nm, CAFE)
    nm.active, nm.ip[ID_CAFE] = ID_CAFE, "10.0.0.5"
    nm.metered = {CAFE: "no", PHONE: "no", STAR: "yes"}
    run_loop(monkeypatch, nm, ticks=14, near_home=True, hooks=[_scan_by_tick(PHONE, range(2, 6), base=(CAFE,))],
             priority=(HOME, PHONE), params={"WifiManualPick": _pick(CAFE)})
    assert nm.ups == [], f"an unmetered pin was ended by another unmetered network: {nm.up_log}"
    assert _cleared(events) == [] and _kept(events) == []
    assert any(n == "netcosttier_pin_held" and kw["target"] == PHONE for n, kw in events), \
      "precondition: the ladder must want the phone, or this test proves nothing"

  def test_a_DEFAULT_cost_network_that_arrives_does_not_end_it(self, monkeypatch, events):
    """`unknown` is not unmetered. Visitor (saved, cost never set) leaves and returns while metered Starlink is
    pinned; the ladder would prefer it (unknown < metered), the pin does not yield to it."""
    nm = FakeNM()
    _with(nm, VISITOR)
    _away(nm, ID_STAR, scan=(STAR,))
    run_loop(monkeypatch, nm, ticks=14, near_home=True, hooks=[_scan_by_tick(VISITOR, range(6))],
             priority=(HOME, PHONE), params={"WifiManualPick": _pick(STAR)})
    assert ID_VISITOR not in nm.ups and _cleared(events) == [], f"an unknown-cost arrival ended the pin: {nm.up_log}"
    assert any(n == "netcosttier_pin_held" and kw["target"] == VISITOR for n, kw in events), "precondition"

  def test_an_unmetered_network_IN_BACKOFF_does_not_end_it(self, monkeypatch, events):
    """A network that will not come up must not end a working pin only for the ladder to fail on it. (With the real
    ledger an arriving network's backoff is usually already cleared on reappearance -- _forget_on_reappearance -- so
    this guard is defence in depth; the ledger is faked here to reach it.)"""
    nm = FakeNM()
    _away(nm, ID_STAR, scan=(STAR,))
    monkeypatch.setattr(d, "_blocked", lambda ledger, now: {PHONE.lower()})
    run_loop(monkeypatch, nm, ticks=14, near_home=True, hooks=[_scan_by_tick(PHONE, range(2, 6))],
             priority=(HOME, PHONE), params={"WifiManualPick": _pick(STAR)})
    assert nm.ups == [] and _cleared(events) == [], f"a backed-off network ended the pin: {nm.up_log}"

  def test_a_pinned_network_whose_COST_WAS_NEVER_READ_is_not_assumed_expensive(self, monkeypatch, events):
    """Ending a pin on cost needs a cost that was read (netrank2pnw D1). Every read of KarlMoik's connection.metered
    fails; the phone genuinely arrives. The pin holds, and the log says it is the unread cost holding it."""
    nm = FakeNM()
    _away(nm, ID_STAR, scan=(STAR,))
    orig = nm.nmcli
    nm.nmcli = lambda args: None if ("connection.metered" in " ".join(args) and args[-1] == ID_STAR) else orig(args)
    run_loop(monkeypatch, nm, ticks=14, near_home=True, hooks=[_scan_by_tick(PHONE, range(6))],
             priority=(HOME, PHONE), params={"WifiManualPick": _pick(STAR)})
    assert nm.ups == [] and _cleared(events) == [], nm.up_log
    assert _kept(events) == [(STAR, PHONE, "pinned_cost_unread")], _kept(events)
    assert "could not be read" in next(kw["rule"] for n, kw in events if n == "netcosttier_pin_kept")


  def test_a_tick_with_NO_REAL_SCAN_never_ends_it_even_with_a_qualifying_network_active(self, monkeypatch, events):
    """Found by mutation: the rule must read this tick's REAL scan, not the list the daemon seeds with the active
    configured network. The phone (unmetered) is still the active link while the UI joins the pick; three dense
    scans omit it (scans often leave out the connected AP), which is "absence"; then the geo-gate shuts and no scan
    runs. The seeded list names the phone -- and ended the pin under the join."""
    nm = FakeNM()
    _away(nm, ID_PHONE, scan=(STAR,))
    where = {"near": True}
    def gate_shuts(nm, tk):
      if tk == 3:
        where["near"] = False
    run_loop(monkeypatch, nm, ticks=4, near_home=lambda: where["near"], gps=FAR_FIX, hooks=[gate_shuts],
             priority=(HOME, PHONE), params={"WifiManualPick": _pick(STAR)})
    assert nm.scan_times == [0.0, POLL, 2 * POLL], f"precondition: real scans on ticks 0-2 only: {nm.scan_times}"
    assert _cleared(events) == [], _cleared(events)


class TestAFlappingHotspotDoesNotThrash:
  def test_a_phone_that_FLICKERS_after_the_pick_never_counts_as_arriving(self, monkeypatch, events):
    """In range at the pick, then in and out of the scans (an iPhone hotspot comes and goes with the phone's screen):
    runs of at most two missing real scans. Dense scanning, 40 ticks. The pin holds and is reported once."""
    nm = FakeNM()
    _away(nm, ID_STAR)
    pattern = [1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 0, 1]
    assert "000" not in "".join(map(str, pattern * 2)), "precondition: never three misses in a row"
    def flicker(nm, tk):
      nm.scan = [STAR, PHONE] if pattern[tk % len(pattern)] else [STAR]
    run_loop(monkeypatch, nm, ticks=40, near_home=True, hooks=[flicker], priority=(HOME, PHONE),
             params={"WifiManualPick": _pick(STAR)})
    assert nm.ups == [] and _cleared(events) == [], f"a flickering phone ended the pin: {nm.up_log}"
    assert _kept(events) == [(STAR, PHONE, "visible_since_pick")], f"logged per reappearance: {_kept(events)}"

  def test_a_ONE_SCAN_appearance_ends_the_pin_once_and_a_failed_join_costs_one_tick(self, monkeypatch, events):
    """The worst case of ending on a single appearance, and why no presence confirmation was added. Absent since the
    pick, the hotspot shows in ONE scan and is gone before the join (refused). The pin ends -- once, it cannot come
    back to end again -- the phone is blamed, and Starlink is back on the very next tick. When the hotspot really
    turns on later, the ladder takes it. No stranding, no repeated switching."""
    nm = FakeNM()
    _away(nm, ID_STAR, scan=(STAR,))
    real_on = 12
    nm.behave[ID_PHONE] = lambda t: "ok" if t >= real_on * POLL else "refuse"
    run_loop(monkeypatch, nm, ticks=20, near_home=True, hooks=[_scan_by_tick(PHONE, set(range(20)) - {5} - set(range(real_on, 20)))],
             priority=(HOME, PHONE), params={"WifiManualPick": _pick(STAR)})
    assert nm.up_log == [(5 * POLL, ID_PHONE), (6 * POLL, ID_STAR), (real_on * POLL, ID_PHONE)], nm.up_log
    assert _cleared(events) == [{"ssid": STAR, "reason": "unmetered", "by": PHONE}], _cleared(events)
    assert nm.active == ID_PHONE


class TestUpgradeScansWhilePinned:
  def test_they_run_every_UPGRADE_SCAN_S_on_a_pinned_METERED_link(self, monkeypatch, events):
    """Away from home, pinned KarlMoik, nothing else around: 320 s. The first scan waits one tick for the link to
    settle, then exactly one per 120 s."""
    nm = FakeNM()
    _away(nm, ID_STAR, scan=(STAR,))
    run_loop(monkeypatch, nm, ticks=16, near_home=False, gps=FAR_FIX, priority=(HOME, PHONE),
             params={"WifiManualPick": _pick(STAR)})
    assert nm.scan_times == [POLL, POLL + 120.0, POLL + 240.0], nm.scan_times
    assert nm.ups == []

  def test_they_do_not_run_on_a_pinned_UNMETERED_link(self, monkeypatch, events):
    nm = FakeNM()
    _away(nm, ID_PHONE, scan=(PHONE,))
    run_loop(monkeypatch, nm, ticks=16, near_home=False, gps=FAR_FIX, priority=(HOME, PHONE),
             params={"WifiManualPick": _pick(PHONE)})
    assert nm.scans == 0, nm.scan_times

  def test_they_do_not_run_while_the_UI_is_still_JOINING_the_pick(self, monkeypatch, events):
    """On Visitor (default cost, so an unpinned link WOULD scan), the driver picks KarlMoik; NM takes 80 s to switch.
    No scan may take the radio off-channel under that join. Once KarlMoik is up and settled, they resume."""
    nm = FakeNM()
    _with(nm, VISITOR)
    nm.active, nm.ip[ID_VISITOR] = ID_VISITOR, "10.0.0.4"
    nm.scan, nm.metered = [VISITOR, STAR], {STAR: "yes"}
    lands = 4
    def join_lands(nm, tk):
      if tk == lands:
        nm.active, nm.ip[ID_STAR] = ID_STAR, "10.0.0.3"
    run_loop(monkeypatch, nm, ticks=8, near_home=False, gps=FAR_FIX, hooks=[join_lands], priority=(HOME, PHONE),
             params={"WifiManualPick": _pick(STAR)})
    assert [t for t in nm.scan_times if t < lands * POLL] == [], f"scanned under the UI's join: {nm.scan_times}"
    assert nm.scan_times[:1] == [(lands + 1) * POLL], f"scans did not resume once the pick was up: {nm.scan_times}"
    assert nm.ups == []


class TestTheHomeRuleIsUnchanged:
  def test_home_still_ends_a_pin_on_the_UNMETERED_phone(self, monkeypatch, events):
    """netrank2pnw D2's own case, which the new rule alone would NOT cover (nothing is cheaper than the unmetered
    phone): the phone picked on the road, then home arrives. The home rule was kept as it was, not folded into the
    strictly-cheaper rule -- so this still ends as `home` and the ladder takes home."""
    nm = FakeNM()
    _away(nm, ID_PHONE, scan=(PHONE,))
    nm.metered[HOME] = "no"
    where = {"near": False, "gps": FAR_FIX}
    def arrive(nm, tk):
      if tk == 4:
        where["near"], where["gps"] = True, AT_HOME_FIX
        nm.scan = [PHONE, HOME]
    run_loop(monkeypatch, nm, ticks=8, near_home=lambda: where["near"], gps=lambda: where["gps"], hooks=[arrive],
             priority=(HOME, PHONE), params={"WifiManualPick": _pick(PHONE)})
    assert nm.up_log == [(4 * POLL, ID_HOME)], nm.up_log
    assert _cleared(events) == [{"ssid": PHONE, "reason": "home", "trigger": HOME}], _cleared(events)


# ================================================================================================
# hotspotretry2pnw — a TRANSIENT join failure costs one poll, not 5-15 minutes of backoff.
#
# MEASURED 2026-09-14 18:55:12 PT: an upgrade scan found "Dirk's iPhone 13", the arbiter ran `nmcli con
# up` on it, wpa_supplicant reported `CTRL-EVENT-SSID-TEMP-DISABLED ... reason=WRONG_KEY`, NM asked for
# secrets, no secret agent is available for the arbiter's nmcli, and it exited 4 with "Secrets were
# required, but not provided". The retry at 18:57:20 failed differently (NM `ssid-not-found`). Each was
# blamed and the ledger exiled the phone for 60 s, then 300 s. The password was never wrong: the SAME
# profile joined the SAME phone at 21:50, 21:53, 22:13 and on 09-15 at 07:39:43, 22 s after the hotspot
# was switched on. An iPhone hotspot that is not fully awake fails the 4-way handshake.
# ================================================================================================

# nmcli's own wording for the two measured failures, and one that must stay `real`.
SECRETS_ERR = "Error: Connection activation failed: (7) Secrets were required, but not provided."
NOT_FOUND_ERR = "Error: Connection activation failed: (53) The Wi-Fi network could not be found."
REAL_ERR = "Error: Connection activation failed: (1) Unknown reason."


def _classified(events):
  return [kw for n, kw in events if n == "netcosttier_join_classified"]


def _blamed(timed):
  return [(t, kw["ssid"], kw["consecutive_failures"]) for t, kw in _named(timed, "netcosttier_assoc_failed")]


class TestATransientJoinFailureIsRetriedOnce:
  def test_the_1855_case_a_secrets_failure_then_success_costs_ONE_TICK_and_no_backoff(self, monkeypatch, timed_events):
    """THE REPORT. The phone refuses at 0 s with the secrets error and is awake by the next poll. The
    arbiter must retry it at 20 s and join -- with nothing in the ledger and no hotspot blip. Before this
    change: blamed at 20 s, hotspot raised at 20 s, phone not retried until ~80 s."""
    timed, clock = timed_events
    nm = FakeNM()
    clock["nm"] = nm
    _away(nm, HOTSPOT, scan=(PHONE,))
    nm.behave[ID_PHONE] = lambda t: "refuse" if t < POLL else "ok"
    nm.up_error[ID_PHONE] = SECRETS_ERR
    run_loop(monkeypatch, nm, ticks=6, near_home=False, priority=(HOME, PHONE))
    assert nm.up_log == [(0.0, ID_PHONE), (POLL, ID_PHONE)], nm.up_log
    assert _blamed(timed) == [], f"a transient failure went into the ledger: {_blamed(timed)}"
    assert nm.active == ID_PHONE

  def test_and_the_log_says_WHY_it_was_retried_with_the_rc_and_NM_s_own_words(self, monkeypatch, events):
    """Rule 2: a retry that happens for a reason nobody can see is the silence this repo bans."""
    nm = FakeNM()
    _away(nm, HOTSPOT, scan=(PHONE,))
    nm.behave[ID_PHONE] = lambda t: "refuse" if t < POLL else "ok"
    nm.up_error[ID_PHONE] = SECRETS_ERR
    run_loop(monkeypatch, nm, ticks=6, near_home=False, priority=(HOME, PHONE))
    got = _classified(events)
    assert len(got) == 1, got
    assert got[0]["ssid"] == PHONE.lower() and got[0]["classification"] == "transient"
    assert got[0]["retrying"] is True and got[0]["mobile"] is True
    assert got[0]["rc"] == 4 and got[0]["error"] == SECRETS_ERR
    assert got[0]["retry_in_s"] == POLL

  def test_the_OTHER_measured_failure_ssid_not_found_is_transient_too(self, monkeypatch, timed_events):
    """18:57:20 PT: NM `ssid-not-found`, 'association took too long' -- the phone had gone back to sleep."""
    timed, clock = timed_events
    nm = FakeNM()
    clock["nm"] = nm
    _away(nm, HOTSPOT, scan=(PHONE,))
    nm.behave[ID_PHONE] = lambda t: "refuse" if t < POLL else "ok"
    nm.up_error[ID_PHONE] = NOT_FOUND_ERR
    run_loop(monkeypatch, nm, ticks=6, near_home=False, priority=(HOME, PHONE))
    assert nm.up_log == [(0.0, ID_PHONE), (POLL, ID_PHONE)], nm.up_log
    assert _blamed(timed) == []


class TestTheRetryIsBoundedAndTheLedgerStillWorks:
  def test_TWO_transient_failures_in_a_row_land_in_the_ledger(self, monkeypatch, timed_events):
    """The bound. One free retry per network, then the ordinary escalating backoff -- so a phone that
    always fails cannot hold the radio in a 20 s loop."""
    timed, clock = timed_events
    nm = FakeNM()
    clock["nm"] = nm
    _away(nm, HOTSPOT, scan=(PHONE,))
    nm.behave[ID_PHONE] = "refuse"
    nm.up_error[ID_PHONE] = SECRETS_ERR
    run_loop(monkeypatch, nm, ticks=4, near_home=False, priority=(HOME, PHONE))
    assert nm.up_log == [(0.0, ID_PHONE), (POLL, ID_PHONE), (2 * POLL, HOTSPOT)], nm.up_log
    assert _blamed(timed) == [(2 * POLL, PHONE.lower(), 1)], _blamed(timed)

  def test_a_PERMANENTLY_broken_hotspot_still_escalates_60_300_900(self, monkeypatch, timed_events):
    """The worst-case attempt rate. Against a phone that never joins the change costs exactly ONE extra
    `con up` per episode; the escalation and its 15-minute steady state are untouched."""
    timed, clock = timed_events
    nm = FakeNM()
    clock["nm"] = nm
    _away(nm, HOTSPOT, scan=(PHONE,))
    nm.behave[ID_PHONE] = "refuse"
    nm.up_error[ID_PHONE] = SECRETS_ERR
    run_loop(monkeypatch, nm, ticks=40, near_home=False, priority=(HOME, PHONE))
    attempts = [t for t, c in nm.up_log if c == ID_PHONE]
    assert attempts == [0.0, POLL, 100.0, 420.0], attempts   # free retry, then 60 s, 300 s, 900 s
    assert [(t, f) for t, _s, f in _blamed(timed)] == [(40.0, 1), (120.0, 2), (440.0, 3)], _blamed(timed)

  def test_the_free_retry_comes_back_ONLY_after_a_successful_join(self, monkeypatch, timed_events):
    """It is spent per network and returned by a join, not by time -- so a morning hotspot that wakes up
    slowly is forgiven again tomorrow, while one that never works is never forgiven twice."""
    timed, clock = timed_events
    nm = FakeNM()
    clock["nm"] = nm
    _away(nm, HOTSPOT, scan=(PHONE,))
    # fails at 0 s (retry spent), joins at 20 s (retry returned), fails again at 200 s, joins at 220 s
    nm.behave[ID_PHONE] = lambda t: "refuse" if t in (0.0, 200.0) else "ok"
    nm.up_error[ID_PHONE] = SECRETS_ERR
    hooks = [lambda nm, tk: nm.__setattr__("active", None) if tk == 10 else None]   # the phone drops at 200 s
    run_loop(monkeypatch, nm, ticks=14, near_home=False, hooks=hooks, priority=(HOME, PHONE))
    assert [t for t, c in nm.up_log if c == ID_PHONE] == [0.0, POLL, 200.0, 220.0], nm.up_log
    assert _blamed(timed) == [], f"the second episode was not forgiven: {_blamed(timed)}"


class TestWhatIsNOTRetried:
  def test_a_REAL_failure_is_blamed_immediately_exactly_as_before(self, monkeypatch, timed_events):
    """An error string the classifier does not recognise keeps today's behaviour, and the log shows the
    raw text so a wording this list gets wrong is visible rather than silently swallowed."""
    timed, clock = timed_events
    nm = FakeNM()
    clock["nm"] = nm
    _away(nm, HOTSPOT, scan=(PHONE,))
    nm.behave[ID_PHONE] = "refuse"
    nm.up_error[ID_PHONE] = REAL_ERR
    run_loop(monkeypatch, nm, ticks=4, near_home=False, priority=(HOME, PHONE))
    assert nm.up_log == [(0.0, ID_PHONE), (POLL, HOTSPOT)], nm.up_log
    assert _blamed(timed) == [(POLL, PHONE.lower(), 1)], _blamed(timed)
    cls = [kw for _t, n, kw in timed if n == "netcosttier_join_classified"]
    assert [(c["classification"], c["retrying"], c["error"]) for c in cls] == [("real", False, REAL_ERR)], cls

  def test_a_STATIONARY_configured_network_is_not_retried_and_the_log_says_why(self, monkeypatch, timed_events):
    """A router that refuses the stored PSK is a wrong password until proven otherwise; only a hotspot
    that travels with the car gets the benefit of the doubt."""
    timed, clock = timed_events
    nm = FakeNM()
    clock["nm"] = nm
    nm.active, nm.scan, nm.metered = HOTSPOT, [HOME], {HOME: "no"}
    nm.behave[ID_HOME] = "refuse"
    nm.up_error[ID_HOME] = SECRETS_ERR
    run_loop(monkeypatch, nm, ticks=4, near_home=True, priority=(HOME, PHONE))
    assert nm.up_log == [(0.0, ID_HOME), (POLL, HOTSPOT)], nm.up_log
    assert _blamed(timed) == [(POLL, HOME.lower(), 1)], _blamed(timed)
    cls = [kw for _t, n, kw in timed if n == "netcosttier_join_classified"]
    assert [(c["classification"], c["mobile"], c["retrying"]) for c in cls] == [("transient", False, False)], cls

  def test_a_link_that_came_UP_and_then_died_is_not_a_transient_JOIN_failure(self, monkeypatch, timed_events):
    """`con up` succeeded (rc 0) and DHCP never completed -- a different failure, judged by the grace as
    before. This pins that the carried classification is overwritten by every bring-up, success included."""
    timed, clock = timed_events
    nm = FakeNM()
    clock["nm"] = nm
    _away(nm, HOTSPOT, scan=(PHONE,))
    nm.behave[ID_PHONE] = lambda t: "refuse" if t < POLL else "no_dhcp"
    nm.up_error[ID_PHONE] = SECRETS_ERR
    run_loop(monkeypatch, nm, ticks=6, near_home=False, priority=(HOME, PHONE))
    assert nm.up_log == [(0.0, ID_PHONE), (POLL, ID_PHONE), (POLL + d.DHCP_GRACE_S, HOTSPOT)], nm.up_log
    assert _blamed(timed) == [(POLL + d.DHCP_GRACE_S, PHONE.lower(), 1)], _blamed(timed)
    # ...and it is not dressed up as one in the log either: only the 0 s `con up` gets a classification.
    cls = [(t, kw["classification"]) for t, kw in _named(timed, "netcosttier_join_classified")]
    assert cls == [(POLL, "transient")], cls

  def test_a_join_that_nmcli_GAVE_UP_ON_but_NM_completed_is_a_recovery_on_that_tick(self, monkeypatch, timed_events):
    """Our own NMCLI_TIMEOUT_S (15 s) can kill `nmcli con up` while NetworkManager carries the activation
    through, so a failing rc can be followed by the link genuinely coming up. That is a SUCCESS: it clears
    the ledger and logs netcosttier_recovered on THAT tick, and it does not spend the free retry. Here the
    phone is blamed for a real failure first (so there IS a ledger entry), then comes up this way."""
    timed, clock = timed_events
    nm = FakeNM()
    clock["nm"] = nm
    _away(nm, HOTSPOT, scan=(PHONE,))
    nm.behave[ID_PHONE] = "refuse"
    nm.up_error[ID_PHONE] = REAL_ERR

    def nm_finishes_the_join_itself(fake, tk):
      if tk == 2:                       # before the 60 s backoff expires at 80 s
        fake.behave[ID_PHONE], fake.up_error[ID_PHONE] = "late", SECRETS_ERR

    run_loop(monkeypatch, nm, ticks=8, near_home=False, hooks=[nm_finishes_the_join_itself],
             priority=(HOME, PHONE))
    assert _blamed(timed) == [(POLL, PHONE.lower(), 1)], _blamed(timed)
    rec = [(t, kw["after_failures"]) for t, kw in _named(timed, "netcosttier_recovered")]
    assert rec == [(5 * POLL, 1)], f"the recovery was not recorded on the tick the link was seen up: {rec}"
    # ...and the failing rc of a join that WORKED is not logged as a classified failure.
    cls = [(t, kw["classification"]) for t, kw in _named(timed, "netcosttier_join_classified")]
    assert cls == [(POLL, "real")], cls


  def test_a_transient_failure_on_ONE_network_does_not_forgive_ANOTHER(self, monkeypatch, timed_events):
    """The phone refuses transiently at 0 s; before the next tick NetworkManager autoconnects KarlMoik on
    its own (measured 2026-09-13 22:01 PT, logged `by=external`) and KarlMoik never gets an address. The
    carried classification belongs to the PHONE, so KarlMoik is blamed on the grace as usual -- even with
    KarlMoik configured as a mobile entry here, so the only thing keeping it out of the retry is the
    ssid check."""
    timed, clock = timed_events
    nm = FakeNM()
    clock["nm"] = nm
    _away(nm, HOTSPOT, scan=(PHONE,))
    nm.behave[ID_PHONE] = "refuse"
    nm.up_error[ID_PHONE] = SECRETS_ERR

    def nm_grabs_karlmoik(fake, tk):
      if tk == 1:
        fake.active, fake.ip[ID_STAR], fake.scan = ID_STAR, None, [STAR]

    run_loop(monkeypatch, nm, ticks=6, near_home=False, hooks=[nm_grabs_karlmoik],
             priority=(HOME, PHONE, STAR), mobile=(PHONE, STAR))
    assert nm.up_log == [(0.0, ID_PHONE), (POLL + d.DHCP_GRACE_S, HOTSPOT)], nm.up_log
    assert _blamed(timed) == [(POLL + d.DHCP_GRACE_S, STAR.lower(), 1)], _blamed(timed)


class TestAHotspotThatIsNeverVisibleIsNeverTried:
  def test_no_con_up_at_all_on_a_phone_that_is_in_no_scan(self, monkeypatch, events):
    """Part of the brief: `ssid-not-found` attempts are avoided because the ladder already requires the
    network to be in THIS tick's real scan (choose_wifi's `low not in scan` -> continue). Nothing here is
    new -- the test exists so that property cannot be lost while relaxing the ledger."""
    nm = FakeNM()
    nm.active, nm.scan, nm.metered = None, [], {PHONE: "no"}
    run_loop(monkeypatch, nm, ticks=20, near_home=False, priority=(HOME, PHONE))
    assert [c for _t, c in nm.up_log if c != HOTSPOT] == [], nm.up_log
    assert nm.up_log == [(0.0, HOTSPOT)], f"the hotspot was re-raised or the phone was tried blind: {nm.up_log}"
    assert _classified(events) == []


class TestTheClassificationLogIsChangeOnly:
  def test_the_same_failure_over_and_over_is_logged_once_per_distinct_reason(self, monkeypatch, events):
    """Change-only, like every other hold/kept line in this daemon -- but a DIFFERENT error string is
    news and is logged again."""
    nm = FakeNM()
    _away(nm, HOTSPOT, scan=(PHONE,))
    nm.behave[ID_PHONE] = "refuse"
    nm.up_error[ID_PHONE] = SECRETS_ERR
    hooks = [lambda nm, tk: nm.up_error.__setitem__(ID_PHONE, NOT_FOUND_ERR) if tk == 10 else None]
    run_loop(monkeypatch, nm, ticks=40, near_home=False, hooks=hooks, priority=(HOME, PHONE))
    got = [(c["classification"], c["retrying"], c["error"]) for c in _classified(events)]
    assert got == [("transient", True, SECRETS_ERR), ("transient", False, SECRETS_ERR),
                   ("transient", False, NOT_FOUND_ERR)], got

  def test_a_CONSUMED_classification_is_not_inherited_by_a_LATER_blame(self, monkeypatch, timed_events):
    """Fable (hotspotretry2pnw review, scratch S1): the carried classification must be CLEARED once its blame is
    decided, or a LATER, unrelated failure of the same ssid inherits it and is forgiven a second time.
    Sequence: the phone fails transiently at 0 s; at 20 s it is out of the scan, so the failure is judged (retry
    granted, not blamed) and never retried; at 60 s NetworkManager autoconnects it ITSELF and the link works, which
    returns the free retry; at 100 s that link dies in DHCP. That death is its own failure and must be blamed on the
    tick the grace expires. Without the fix the stale "transient" verdict is reused, the blame is swallowed, and the
    truck sits 20 s longer on a dead link."""
    timed, clock = timed_events
    nm = FakeNM()
    clock["nm"] = nm
    _away(nm, HOTSPOT, scan=(PHONE,))
    nm.behave[ID_PHONE] = "refuse"
    nm.up_error[ID_PHONE] = SECRETS_ERR

    def gone_then_nm_joins_it_then_it_dies(fake, tk):
      if tk == 1:                                     # out of the scan: judged, retry granted, never attempted
        fake.scan = ()
      if tk == 3:                                     # NM autoconnects it and it WORKS -> the free retry comes back
        fake.scan = (PHONE,)
        fake.behave[ID_PHONE], fake.up_error[ID_PHONE] = "ok", None
        fake.active, fake.ip[ID_PHONE] = ID_PHONE, "172.20.10.5"
      if tk == 5:                                     # ...and then NM's own link dies in DHCP
        fake.ip[ID_PHONE], fake.behave[ID_PHONE] = None, "no_dhcp"

    run_loop(monkeypatch, nm, ticks=10, near_home=False, hooks=[gone_then_nm_joins_it_then_it_dies],
             priority=(HOME, PHONE))
    assert _blamed(timed) == [(5 * POLL, PHONE.lower(), 1)], \
      f"the DHCP death was not blamed on its own grace tick -- a consumed classification was inherited: {_blamed(timed)}"
    cls = [(t, kw["classification"], kw["retrying"]) for t, kw in _named(timed, "netcosttier_join_classified")]
    assert cls == [(POLL, "transient", True)], f"a later blame was dressed up as the old join failure: {cls}"

  def test_a_retry_AFTER_a_successful_join_is_logged_again(self, monkeypatch, timed_events):
    """Fable (Rule 2): the classification log is change-only, so a second transient failure with byte-identical
    fields would be suppressed unless the successful join in between re-arms it. Without the re-arm the evening
    retry of a phone that already failed in the morning happens silently."""
    timed, clock = timed_events
    nm = FakeNM()
    clock["nm"] = nm
    _away(nm, HOTSPOT, scan=(PHONE,))
    nm.behave[ID_PHONE] = "refuse"
    nm.up_error[ID_PHONE] = SECRETS_ERR

    def the_retry_works_then_it_fails_the_same_way_again(fake, tk):
      if tk == 1:                                     # the granted retry joins -> the free retry is returned
        fake.behave[ID_PHONE], fake.up_error[ID_PHONE] = "ok", None
      if tk == 4:                                     # the phone drops and refuses exactly as it did at 0 s
        fake.active = None
        fake.behave[ID_PHONE], fake.up_error[ID_PHONE] = "refuse", SECRETS_ERR

    run_loop(monkeypatch, nm, ticks=10, near_home=False, hooks=[the_retry_works_then_it_fails_the_same_way_again],
             priority=(HOME, PHONE))
    cls = [(t, kw["classification"], kw["retrying"]) for t, kw in _named(timed, "netcosttier_join_classified")]
    assert cls == [(POLL, "transient", True),          # the morning failure: retried
                   (5 * POLL, "transient", True),      # the evening one, after a success returned the retry
                   (6 * POLL, "transient", False)], \
      f"a retry after a successful join went unlogged (change-only not re-armed): {cls}"
