#!/usr/bin/env python3
"""
network2xnor: arbitration supervisor for perpetual tethering + priority WiFi.

The comma 3X has ONE WiFi radio: it can run the hotspot (AP) OR connect to a client network, never
both. NetworkManager scans even while in AP mode (it sees other SSIDs) but will NOT auto-switch off
an active hotspot to a higher-priority client. This always-on process closes that gap:

  - When tethering is enabled (param TetheringEnabled), the hotspot is kept up so it survives reboot
    ("perpetual tethering") and re-asserts itself if knocked down. It ALSO installs the NAT (ip_forward
    + masquerade out LTE) whenever it raises the hotspot, so tethered clients actually get internet —
    the UI toggle isn't the only path that raises the AP.
  - When a single named "priority" SSID (param TetheringPriorityWifi) comes into range AND we have a
    saved connection for it, we switch the radio over to that client network (dropping the hotspot).
    When it leaves range, we bring the hotspot back.

GEO-GATED SCANNING: while tethering, we'd otherwise scan for the priority SSID every cycle — but on a
single radio a WiFi scan forces the radio off-channel (NetworkManager's own maintainer documents that
scans induce lag/drops), competing with the hotspot. There's no point scanning for home WiFi when
you're nowhere near home. So we record the home location (GPS, where the priority WiFi lives) and only
scan/switch when within HOME_GEOFENCE_M of it. Fail-open: if home isn't learned yet or GPS is missing,
we scan as before.

The *decisions* live in the pure, unit-tested `decide(...)` (network_arbiter.py) + `near_home(...)`
(geo_gate.py). This module is the thin I/O shell: it polls NM via `nmcli`, feeds snapshots to the pure
deciders, and runs the chosen action. Robust to nmcli failures (log + continue), idempotent.

IMPORTANT — netplan/keyfile crash: on this AGNOS the gsm profiles (lte/esim) are NETPLAN-managed.
`nmcli con modify` on a gsm profile triggers NM's keyfile-writer assertion -> NetworkManager ABRT
(crash loop) -> hardwared's network read goes blank. So we NEVER `nmcli con modify` a gsm profile here.
Persistence of the LTE profile (blank APN, autoconnect) is handled in /data/etc/netplan/ + the GsmApn
param, NOT by this daemon. The throttle guard parks LTE via the MODEM (mmcli --disable), not nmcli.

Behavior-neutral when TetheringEnabled is unset/0 (its default).
"""
import json
import subprocess
import threading
import time
from typing import NoReturn

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.system.networkd.network_arbiter import (
  HOTSPOT_CONNECTION_ID,
  UPGRADE_SCAN_S,
  UnmeteredYield,
  arrival_candidates,
  classify_join_failure,
  decide,
  explain_fallback,
  home_to_yield_to,
  unmetered_to_yield_to,
  update_home_arrival,
  judge_link,
  judge_pin,
  on_priority_network,
  parse_manual_pick,
  pending_for_new_link,
  upgrade_scan_due,
  priority_connection_id,
  ssid_of,
)
from openpilot.system.networkd.lte_guard import decide_lte_guard
from openpilot.system.networkd.geo_gate import near_any_home, haversine_m
from openpilot.system.networkd import priority_networks as pn
from openpilot.system.networkd import captive_portal

POLL_INTERVAL_S = 20.0
NMCLI_TIMEOUT_S = 15.0
SIGNAL_EVERY_N = 3   # run LTE signal logging only every Nth loop (~60s) so its mmcli calls can't repeatedly stall WiFi recovery
LEARN_MIN_MOVE_M = 50.0   # only re-write a learned home location if GPS moved more than this (flash-wear guard)
PORTAL_MAX_TRIES = 6   # max captive-portal accept() attempts per SSID session (bounds loop time; resets on leaving)

LTE_CONNECTION_ID = "lte"
HOTSPOT_SUBNET = "192.168.43.0/24"   # AP client subnet, masqueraded out the LTE uplink
# the carrier/modem PDN throttle string NM logs when a speed test (or any rapid PDN churn) gets the
# IMEI flagged. NM then hammers retries ~1/s, which keeps the carrier timer from ever aging out.
PDN_THROTTLE_TOKEN = "pdn-ipv4-call-throttled"


def _nmcli_run(args: list[str]) -> subprocess.CompletedProcess | None:
  """Run nmcli and return the RAW CompletedProcess, or None if it could not be run at all.

  hotspotretry2pnw: `_nmcli` folds every failure into None, which is all its callers need -- but a join
  that failed because a phone hotspot's 4-way handshake did not complete is a different fact from a
  wrong password or a dead router, and only the return code and stderr can tell them apart. Extracted
  so `_con_up` can see them, without changing what `_nmcli` returns to everything else."""
  try:
    return subprocess.run(["nmcli", *args], capture_output=True, text=True,
                          timeout=NMCLI_TIMEOUT_S, check=False)
  except (OSError, subprocess.TimeoutExpired):
    cloudlog.exception(f"network_arbiterd: nmcli {args} failed to run")
    return None


def _nmcli(args: list[str]) -> str | None:
  """Run an nmcli query/command. Returns stdout on success, None on any failure (logged)."""
  proc = _nmcli_run(args)
  if proc is None:
    return None
  if proc.returncode != 0:
    cloudlog.warning(f"network_arbiterd: nmcli {args} rc={proc.returncode} err={proc.stderr.strip()}")
    return None
  return proc.stdout


def _con_up(conn_id: str) -> tuple[str, int | None, str]:
  """`nmcli con up <conn_id>`, reporting HOW it failed: (classification, rc, error text).

  ("", 0, "") on success. `classification` is classify_join_failure's "transient"/"real" on a failure,
  and "real" when nmcli could not be run at all (our own NMCLI_TIMEOUT_S, or an exec error) -- we learnt
  nothing there, so it keeps the pre-change behaviour. The caller decides what to do with it; this
  function only reads the evidence. The rc/stderr warning mirrors `_nmcli`'s, so nothing that used to be
  visible in the log stops being visible."""
  proc = _nmcli_run(["con", "up", conn_id])
  if proc is None:
    return "real", None, "nmcli could not be run"
  if proc.returncode == 0:
    return "", 0, ""
  err = " ".join(proc.stderr.split())
  cloudlog.warning(f"network_arbiterd: nmcli ['con', 'up', {conn_id!r}] rc={proc.returncode} err={err}")
  return classify_join_failure(err), proc.returncode, err


def _run(args: list[str]) -> subprocess.CompletedProcess | None:
  """Run an arbitrary command, swallow exec errors. Returns the CompletedProcess or None."""
  try:
    return subprocess.run(args, capture_output=True, text=True, timeout=NMCLI_TIMEOUT_S, check=False)
  except (OSError, subprocess.TimeoutExpired):
    cloudlog.exception(f"network_arbiterd: {args[:2]} failed to run")
    return None


def _wifi_autoconnect_repair(nets: list[dict]) -> None:
  """wifirepair2pnw: restore `autoconnect` on saved client wifi, and make selection deterministic.

  THE BUG THIS FIXES. The hotspot and a client connection cannot share the single radio, so
  wifi_manager._set_others_autoconnect(False) turns autoconnect OFF on every saved wifi network when
  tethering starts, and only turns it back ON in set_tethering_active(False). That restore is reachable
  ONLY through the UI toggle, runs on a daemon thread that dies with the UI process, and the
  `autoconnect=no` it leaves behind is PERSISTED in NM config across reboots. Meanwhile decide() here
  can independently return "down_hotspot" and tear the hotspot down without any of that running.

  Net effect (driver report 2026-08-22, "sometimes it does not auto connect to a saved network, but
  not always", and once "could not see ANY wifi networks until I toggled tethering"): the device is
  left with autoconnect disabled on every saved network and silently never rejoins anything. Toggling
  tethering on and off by hand is what repairs it today, which is exactly the reported workaround.

  So: whenever tethering is NOT active, assert the invariant every tick. Idempotent -- it only issues
  an nmcli write when a connection is actually in the wrong state, so the steady-state cost is one
  read. Deliberately does nothing while tethering IS active: there the UI's suppression is correct and
  fighting it would let a client steal the radio back from the AP.

  Also sets connection.autoconnect-priority from the driver's OWN priority-list order (first entry
  wins). Without it every saved network sits at priority 0 and NM picks among in-range candidates
  arbitrarily -- the second half of "sometimes it does not auto connect to [the right] network".
  """
  out = _nmcli(["-t", "-f", "NAME,TYPE,AUTOCONNECT,AUTOCONNECT-PRIORITY", "con", "show"])
  if out is None:
    return
  # highest priority to the first list entry; anything not in the list keeps a neutral 0
  want_prio = {priority_connection_id(e["ssid"]): len(nets) - i for i, e in enumerate(nets)}
  for line in out.splitlines():
    parts = line.rsplit(":", 3)
    if len(parts) != 4:
      continue
    name, ctype, autoconn, prio = parts
    if "wireless" not in ctype or name == HOTSPOT_CONNECTION_ID:
      continue
    if autoconn.lower() != "yes":
      cloudlog.warning(f"network_arbiterd: repairing autoconnect on '{name}' (was {autoconn!r}) -- left disabled by a hotspot session that never restored it")
      _nmcli(["con", "modify", name, "connection.autoconnect", "yes"])
    target = want_prio.get(name)
    if target is not None and prio.strip() != str(target):
      _nmcli(["con", "modify", name, "connection.autoconnect-priority", str(target)])


def _scan_ssids() -> list[str] | None:
  """SSIDs currently visible to NM, or None if the scan COULD NOT BE TAKEN.

  netcosttier2pnw: None and [] are different facts. [] means "we looked and saw nothing"; None means
  nmcli failed and we know nothing. The ledger's reappearance logic must not read a failed scan as
  "every network went out of range" -- see _forget_on_reappearance."""
  out = _nmcli(["-t", "-f", "SSID", "dev", "wifi", "list"])
  if out is None:
    return None
  return [line for line in (raw.strip() for raw in out.splitlines()) if line]


def _saved_connections() -> list[str]:
  """All NM connection ids that exist. `nmcli -t -f NAME con show`."""
  out = _nmcli(["-t", "-f", "NAME", "con", "show"])
  if out is None:
    return []
  return [line for line in (raw.strip() for raw in out.splitlines()) if line]


_metered_cache: dict[str, str] = {}   # ssid -> last SUCCESSFULLY read connection.metered value


def _metered_states(saved: list[str], of_interest: set[str]) -> tuple[set[str], set[str]]:
  """(metered, unmetered) SSIDs, from NM's `connection.metered` on each saved client profile.

  netcosttier2pnw. Returns TWO sets because the field has THREE states and the third is not a
  synonym for either:

      yes      -> in `metered`     asserted expensive
      no       -> in `unmetered`   asserted cheap
      unknown  -> in NEITHER       nobody ever said -- ranked between them by choose_wifi

  Measured on the 3X 2026-09-10: of four saved client profiles only the iPhone and the home WiFi
  carry an explicit `no`; the driver's mobile Starlink and "Visitor" are both `unknown`. Collapsing
  unknown into `no` (the first cut of this file) put the Starlink level with the unmetered iPhone and
  left an alphabetical tiebreak to decide -- see choose_wifi for why that is a Rule 2 failure.

  `of_interest` bounds the work to SSIDs that could actually be chosen this tick (in the scan, or the
  one we are on). Without it this walked every saved profile ever created, one nmcli each, every 20 s
  forever. nmcli has no way to dump connection.metered for all profiles in a single call -- the
  field only appears in a per-connection `con show` -- so the fix is to ask about fewer of them, not
  to ask once.

  An nmcli failure puts the profile in NEITHER set, i.e. it is treated as `unknown`, and it is
  LOGGED. That is the honest reading: a read that failed tells us nothing, and must not be recorded
  as an assertion in either direction."""
  metered: set[str] = set()
  unmetered: set[str] = set()
  want = {s.lower() for s in of_interest}
  for conn in saved:
    ssid = ssid_of(conn)
    if not ssid or ssid.lower() not in want:
      continue
    raw = _nmcli(["-t", "-f", "connection.metered", "con", "show", conn])
    if raw is None:
      # NO NEW INFORMATION -- reuse the last value we successfully read, if any. Dropping to
      # "unknown" here was a real defect (Gemini review 2026-09-11): a metered PRIORITY network is
      # demoted out of tier 0 only while we can see that it is metered, so one timed-out nmcli call
      # promoted it straight back to tier 0, tore down the cheaper link the ladder had chosen, and
      # the next successful read demoted it again -- a flap driven purely by nmcli flakiness.
      cached = _metered_cache.get(ssid)
      if cached == "yes":
        metered.add(ssid)
      elif cached == "no":
        unmetered.add(ssid)
      cloudlog.warning(f"network_arbiterd: could not read connection.metered for {conn!r} -- reusing last known value {cached!r}")
      continue
    val = raw.strip().lower().rsplit(":", 1)[-1]
    _metered_cache[ssid] = val
    if val == "yes":
      metered.add(ssid)
    elif val == "no":
      unmetered.add(ssid)
    # anything else (notably "unknown") deliberately lands in neither set
  return metered, unmetered


# --- netcosttier2pnw: association-failure ledger -----------------------------------------------
# Bringing a client network up DROPS THE HOTSPOT FIRST. If the association then fails -- AP in range
# but refusing, wrong PSK, DHCP dead -- the device is left with no uplink at all, and next tick it
# picks the same network again (still in the scan) and repeats. That is an indefinite offline loop,
# and it is newly reachable because the cost ladder makes EVERY saved network a candidate, where
# before only a geo-gated priority network was. So a network that will not come up has to earn its
# way out of the running for a while.
_WLAN_DEV = "wlan0"

DHCP_GRACE_S = 60.0   # NM's DHCP timeout is 45 s and the poll is 20 s, so the first look at a link
                      # we just raised can legitimately land mid-activation. Judging it there was
                      # measured to tear down links that would have completed seconds later.
_usable_cache: dict[str, bool] = {}   # ssid -> last SUCCESSFULLY determined link usability

FAIL_BACKOFF_S = (60.0, 300.0, 900.0)   # escalating, then held at the last value (15 min cap)
# The cap is deliberately NOT an hour. The costs are asymmetric: retrying a genuinely dead network
# costs one hotspot blip every 15 min (we detect the failure and go back within one 20 s tick),
# while exiling a network that has come back costs the driver an hour of LTE in his own driveway.
# A rebooting router is also handled directly -- see _forget_on_reappearance.

# unreadhold2pnw: how long a run of FAILED `nmcli con show --active` reads may hold the radio still (see the hold
# in main()). In seconds, not ticks: a wedged NM makes nmcli calls run to NMCLI_TIMEOUT_S, which stretches a tick
# well past POLL_INTERVAL_S, so a tick count would not bound the time. 120 s = 6 polls = twice DHCP_GRACE_S (the
# longest the arbiter already waits on a link it cannot confirm), so an NM slow to answer at boot or mid-activation
# (its own DHCP timeout is 45 s) is ridden out with margin. Past that the failure is persistent, the link may
# genuinely be gone, and the hotspot must be allowed back as the recovery path. Worst case this adds the hold plus
# one poll (plus nmcli timeouts) before the arbiter acts on a link that really did die during the failure.
ACTIVE_UNREADABLE_HOLD_S = 120.0


def _client_link_usable(conn_id: str, has_portal_handler: bool = False) -> bool | None:
  """TRI-STATE. True = has an IPv4 address, False = associated with none, None = COULD NOT TELL.

  NetworkManager reports a wifi 'activated' as soon as it associates, so association alone is worth
  nothing: a dead DHCP server, or an AP that accepts the association and routes nowhere, looks
  connected forever. An IPv4 address is the cheapest honest evidence that the link is carrying
  anything.

  The None case is not pedantry. An earlier cut returned a plain bool and folded an nmcli timeout
  into False, so one flaky call recorded a failure against a working link, un-stuck it, and -- with
  the scan empty under the geo-gate -- handed the radio to the hotspot on the next tick. A test of
  mine asserted that folding as correct. It is the parent CLAUDE.md's "an ERROR is not a NEGATIVE
  RESULT" rule, which this file had already been bitten by once."""
  out = _nmcli(["-t", "-f", "IP4.ADDRESS", "con", "show", conn_id])
  if out is None:
    cloudlog.warning(f"network_arbiterd: could not read IP4.ADDRESS for {conn_id!r} -- link state UNKNOWN this tick, not assumed dead")
    return None
  if not any(line.split(":", 1)[-1].strip() for line in out.splitlines() if line.strip()):
    return False

  # An address is necessary but not sufficient. A saved cafe network whose portal we never accepted,
  # a home router whose ISP is down, an obstructed Starlink: all hold an IPv4 address and would pass
  # the check above, hold the radio, and BLACK-HOLE THE DEVICE'S OWN TRAFFIC -- measured on the 3X,
  # wlan0's default route has metric 600 against wwan0's 1000, so WiFi wins even when it goes
  # nowhere. NM's own connectivity check is enabled here (verified: `nmcli -t -f CONNECTIVITY
  # general` -> full) and reports per-device, which is what we need: the global value is masked by
  # LTE still working.
  #
  # `none` and `limited` are always dead. `portal` is usable ONLY when this SSID actually has an
  # accept handler configured -- that is the case the exemption exists for, because the auto-accept
  # path has to be ON the network to POST the form, and demoting `portal` there would tear the link
  # down before the handler could ever run.
  #
  # For any OTHER portal network the exemption is a black hole (Fable): a hotel WiFi joined once,
  # still saved, is now a ladder candidate. Measured -- the device parked on it sticky and "usable",
  # hotspot down, wlan0 holding the default route at metric 600, indefinitely, with no log line at
  # all. That is precisely the failure this connectivity check was added to close, re-opened by its
  # own exemption. An unreadable value is UNKNOWN, not dead.
  conn = _nmcli(["-g", "GENERAL.IP4-CONNECTIVITY", "dev", "show", _WLAN_DEV])
  if conn is None:
    return None
  state = conn.strip().lower()
  dead = state.startswith(("1 ", "2 ", "none", "limited")) or state in ("1", "2")
  portal = state.startswith("3 ") or state == "portal"
  if dead or (portal and not has_portal_handler):
    cloudlog.event("netcosttier_link_no_upstream", conn_id=conn_id, ip4_connectivity=conn.strip(),
                   portal_handler=has_portal_handler)
    return False
  return True


ABSENT_SCANS_FOR_FRESH_START = 2


def _forget_on_reappearance(ledger: dict[str, tuple[int, float]], absent: dict[str, int],
                            scan: set[str] | None) -> None:
  """Clear the ledger for an SSID that has been genuinely out of range and has come back.

  A backoff is a statement about a network that FAILED while reachable. An SSID that vanished and
  returned is new information -- most often a router that was rebooting, which is what an escalating
  backoff punishes hardest (radio up before DHCP, fail, 60 s, fail, 5 min, and the car sits in the
  driveway on LTE long after the router is healthy).

  TWO GUARDS, both from measured failures of the first version:
    * `scan is None` means NO SCAN RAN this tick -- the geo-gate suppresses scanning whenever we are
      already on client WiFi away from a learned location, which is normal operation, not absence.
      Treating a suppressed scan as "the network is gone" wiped the ledger every other tick and
      collapsed the whole backoff to the scan-flicker rate: measured 3 clears in 7 ticks, with
      `consecutive_failures` never getting past 1 and the hotspot dropping every other tick.
    * a single missing scan result is not absence either -- APs drop out of one scan routinely -- so
      an SSID must be missing from ABSENT_SCANS_FOR_FRESH_START consecutive REAL scans before its
      return counts as news."""
  if scan is None:
    return                                   # no evidence either way; leave every counter alone
  for ssid in list(absent):
    if ssid in scan:
      if absent.get(ssid, 0) >= ABSENT_SCANS_FOR_FRESH_START and ssid in ledger:
        cloudlog.event("netcosttier_ledger_cleared", ssid=ssid,
                       reason=f"back in range after {absent[ssid]} scans absent")
        ledger.pop(ssid, None)
      absent[ssid] = 0
    else:
      absent[ssid] = absent.get(ssid, 0) + 1
  for ssid in ledger:
    absent.setdefault(ssid, 0 if ssid in scan else 1)


def _note_attempt(ledger: dict[str, tuple[int, float]], ssid: str, ok: bool, now: float) -> None:
  """Update the per-SSID failure ledger. Rule 2: every transition is logged as a cloudlog EVENT, not
  a debug line -- a network quietly dropping out of the ladder is exactly the kind of silence that
  gets trusted."""
  ssid = ssid.lower()   # the SAME network reaches here as the configured case ("visitor") from a
                        # tier-0 pending target and as the AP's case ("Visitor") from ssid_of() --
                        # keyed literally, one network held TWO ledger entries and a success on one
                        # never cleared the other (Fable). One key per network.
  fails, _until = ledger.get(ssid, (0, 0.0))
  if ok:
    if fails:
      cloudlog.event("netcosttier_recovered", ssid=ssid, after_failures=fails)
    ledger.pop(ssid, None)
    return
  fails += 1
  backoff = FAIL_BACKOFF_S[min(fails, len(FAIL_BACKOFF_S)) - 1]
  ledger[ssid] = (fails, now + backoff)
  cloudlog.event("netcosttier_assoc_failed", ssid=ssid, consecutive_failures=fails,
                 backoff_s=backoff, error="con up did not yield an active wifi with an IPv4 address")


def _blocked(ledger: dict[str, tuple[int, float]], now: float) -> set[str]:
  """Lowercase SSIDs currently serving a backoff. choose_wifi folds case on its side too."""
  return {ssid for ssid, (_f, until) in ledger.items() if until > now}


def _active_wifi_read() -> tuple[str | None, bool]:
  """(active wifi connection id or None, whether the read SUCCEEDED).

  netscanpin2pnw: `_active_wifi_connection()` returns None for two different facts -- "nothing is
  active" and "nmcli failed". The manual-pick pin needs them apart: an unreadable tick must not look
  like "the driver's network dropped" and end his pin. Everything else keeps the old contract."""
  out = _nmcli(["-t", "-f", "NAME,TYPE,DEVICE", "con", "show", "--active"])
  if out is None:
    return None, False
  for raw in out.splitlines():
    parts = raw.split(":")
    if len(parts) < 2:
      continue
    name, conn_type = parts[0], parts[1]
    if "wireless" in conn_type:
      return name, True
  return None, True


def _active_wifi_connection() -> str | None:
  """The NM connection id currently active on the wlan device, or None."""
  return _active_wifi_read()[0]


# --- hotspot NAT (so the arbiter-raised AP actually passes traffic, like the UI toggle does) --------

def _set_hotspot_nat(enabled: bool) -> None:
  """Forward + masquerade the AP subnet out the LTE uplink so tethered clients get internet. Mirrors
  wifi_manager._set_tethering_nat — the arbiter raises the hotspot on boot / after a WiFi-drop, paths
  the UI toggle never runs, so without this the hotspot is up but 'nothing happens' for clients.
  AGNOS uses iptables-LEGACY (the nft binary lacks the MASQUERADE module). Idempotent."""
  _run(["sudo", "sysctl", "-w", f"net.ipv4.ip_forward={1 if enabled else 0}"])
  rules = [
    ["-t", "nat", "POSTROUTING", "-s", HOTSPOT_SUBNET, "!", "-d", HOTSPOT_SUBNET, "-j", "MASQUERADE"],
    ["FORWARD", "-s", HOTSPOT_SUBNET, "-j", "ACCEPT"],
    ["FORWARD", "-d", HOTSPOT_SUBNET, "-j", "ACCEPT"],
  ]
  for rule in rules:
    pre = rule[:2] if rule[0] == "-t" else []
    chain = rule[2] if rule[0] == "-t" else rule[0]
    rest = rule[3:] if rule[0] == "-t" else rule[1:]
    base = ["sudo", "iptables-legacy", *pre]
    _run([*base, "-D", chain, *rest])           # delete first (idempotent)
    if enabled:
      _run([*base, "-I", chain, *rest])


def _leaving(current_active: str | None, active_read_ok: bool) -> str:
  """smallfix0914pnw: what a client bring-up takes the single radio away from, for its log line.

  The line used to say "(dropping hotspot)" unconditionally, which was false whenever the device was on a
  client WiFi -- e.g. leaving KarlMoik for the phone. An unreadable active connection is said as such, not
  guessed."""
  if not active_read_ok:
    return "active connection unreadable"
  if current_active == HOTSPOT_CONNECTION_ID:
    return "dropping hotspot"
  if current_active:
    return f"leaving {current_active}"
  return "nothing was active"


def _expected_active(action: str, ssid: str) -> str | None:
  """smallfix0914pnw: the wifi connection `action` should leave active ("" = none), or None for noop/unknown.
  Mirrors the connection ids _apply brings up, so network_arbiter_active_changed can say whether a change
  was the arbiter's own."""
  if action in ("up_priority", "up_fallback") and ssid.strip():
    return priority_connection_id(ssid.strip())
  if action == "up_hotspot":
    return HOTSPOT_CONNECTION_ID
  if action == "down_hotspot":
    return ""
  return None


def _apply(action: str, ssid: str, current_active: str | None = None, active_read_ok: bool = True,
           why: str = "") -> tuple[str, int | None, str]:
  """Run the one action chosen by decide(). All failures are logged, never raised.
  `current_active`/`active_read_ok` are this tick's read, used only to say in the log what is being left.
  `why` (arbiterfu2pnw) is explain_fallback's text for an up_fallback, used only in its log line.

  hotspotretry2pnw: returns `_con_up`'s (classification, rc, error) for a CLIENT bring-up, and
  ("", 0, "") for everything else. The caller carries it to the next tick, where judge_link decides
  whether the bring-up is to be blamed -- a failed `con up` leaves nothing active, so the failure is
  only ever seen one poll later."""
  if action == "noop":
    return "", 0, ""
  if action == "up_priority":
    conn_id = priority_connection_id(ssid.strip())
    cloudlog.info(f"network_arbiterd: priority wifi '{ssid}' in range -> {conn_id} ({_leaving(current_active, active_read_ok)})")
    _set_hotspot_nat(False)                       # hotspot going away -> tear down its NAT
    return _con_up(conn_id)
  elif action == "up_fallback":
    # netcosttier2pnw: tier 1/2 -- some other saved wifi, cheaper than our own LTE.
    conn_id = priority_connection_id(ssid.strip())
    leaving = _leaving(current_active, active_read_ok)
    cloudlog.info(f"network_arbiterd: falling back to saved wifi '{ssid}' -> {conn_id} [{why}] ({leaving})")
    _set_hotspot_nat(False)
    return _con_up(conn_id)
  elif action == "up_hotspot":
    cloudlog.info("network_arbiterd: bringing hotspot up (+NAT)")
    _set_hotspot_nat(True)                         # install NAT BEFORE the AP so the first client packet routes
    _nmcli(["con", "up", HOTSPOT_CONNECTION_ID])
  elif action == "down_hotspot":
    cloudlog.info("network_arbiterd: tethering disabled -> bringing hotspot down (-NAT)")
    _nmcli(["con", "down", HOTSPOT_CONNECTION_ID])
    _set_hotspot_nat(False)
  else:
    cloudlog.error(f"network_arbiterd: unknown action {action!r}")
  return "", 0, ""


# --- GPS / home-location for the geo-gate ------------------------------------------------------------

GPS_MAX_AGE_S = 10.0   # netrank2pnw: reject a LastGPSPosition older than this (same window location_servicesd uses)
_gps_stale_logged = ""  # change-only log state for the stale-GPS event


def _read_gps(params: Params, mem_params: Params | None) -> tuple[float, float] | None:
  """Current (lat, lon) from LastGPSPosition (JSON {latitude, longitude, ..., ts}), or None.

  The only writer in this tree is system/mapd/mapd_configd.py, which bridges the GPS service into the
  IN-MEMORY store at ~1 Hz with `ts` = time.monotonic() (system-wide, comparable across processes). The
  previous docstring said "locationd writes the persistent one" -- nothing in this tree does. The
  persistent key still exists, so an old value can sit there; it is still consulted, and the freshness
  check below rejects it.

  netrank2pnw FRESHNESS (Fable N2): mapd_configd stops rewriting the param when GPS dies, so a blob can
  describe where the truck WAS. Consumers here are the geo-gate, geo-learning, and a pin's arrival
  evidence, which reads "more than 500 m from home" as proof the truck left -- a stale reading is exactly
  the wrong input for that. A reading older than GPS_MAX_AGE_S, from the future (a `ts` from a previous
  boot, since monotonic restarts at boot), or without a `ts` (not from the bridge) is treated as NO GPS,
  and logged once per distinct reason. No GPS is already the fail-safe case everywhere downstream: the
  geo-gate fails open and a pin gets no arrival evidence."""
  global _gps_stale_logged
  problem = "no LastGPSPosition in either store"
  for store in (mem_params, params):
    if store is None:
      continue
    try:
      raw = store.get("LastGPSPosition")
      if not raw:
        continue
      d = json.loads(raw) if isinstance(raw, (str, bytes, bytearray)) else raw
      lat, lon = float(d["latitude"]), float(d["longitude"])
      ts = d.get("ts")
      if ts is None:
        problem = "LastGPSPosition has no ts (not written by the mapd_configd bridge)"
        continue
      age = time.monotonic() - float(ts)
      if age < 0 or age > GPS_MAX_AGE_S:
        problem = f"LastGPSPosition is stale (age {age:.0f} s, limit {GPS_MAX_AGE_S:.0f} s)"
        continue
      _gps_stale_logged = ""
      return lat, lon
    except Exception as e:
      problem = f"LastGPSPosition unreadable: {type(e).__name__}"
      continue
  kind = problem.split(" (")[0]
  if kind != _gps_stale_logged and problem != "no LastGPSPosition in either store":
    cloudlog.event("network_arbiterd_gps_unusable", problem=problem, treated_as="no GPS")
    _gps_stale_logged = kind
  return None


def _carry_parked_fix(fresh: tuple[float, float] | None, onroad: bool,
                      last_fix: tuple[float, float, float] | None, carrying: bool
                      ) -> tuple[tuple[float, float] | None, tuple[float, float, float] | None, bool]:
  """gpscarry2pnw (Fable, netrank2pnw re-review): while the ignition is off, keep using the last fresh fix.

  qcomgpsd runs only while `started` (process_config.qcomgps), so with the ignition off nothing feeds the
  mapd_configd bridge and _read_gps reads None 10 s later. Parked with the device awake that cost two
  things: the GPS veto on a scan gap (an at-home pin ended by a 60 s home-AP gap), and the geo-gate, which
  failed open and scanned every 20 s on client WiFi away from any learned location. A parked device does
  not move.

  `onroad` is the IsOnroad param, i.e. deviceState.started -- the SAME condition that schedules qcomgpsd,
  so a fix is carried exactly while the GPS producer is not scheduled. A param, not a msgq subscription
  (background-process rule). NOT the same as ignition: see the residual risks in the gpscarry2pnw doc.

  Returns (gps for this tick's decisions, new last_fix, carrying). last_fix = (lat, lon, monotonic time it
  was last read FRESH), and lives only in main()'s locals: a daemon restart or a reboot carries nothing,
  and _read_gps's previous-boot check is untouched.
    * a fresh fix is used as is, and becomes last_fix;
    * onroad without a fresh fix: no GPS, and last_fix is DROPPED, not suspended -- a truck that drove with
      GPS dead and parked again must not get its pre-drive position back;
    * offroad without a fresh fix: last_fix, if this process saw one; otherwise no GPS, as before.
  Logged once when a carry starts (with the fix's age) and once when it ends (with why). fix_age_s counts
  from the last fresh read; the fix itself was up to GPS_MAX_AGE_S older than that."""
  now = time.monotonic()
  if fresh is None and not onroad and last_fix is not None:
    if not carrying:
      cloudlog.event("network_arbiterd_gps_carry_started", fix_age_s=round(now - last_fix[2], 1),
                     reason="IsOnroad=0 and no fresh GPS: the device is parked, keep its last fresh fix",
                     replaces="no GPS")
    return (last_fix[0], last_fix[1]), last_fix, True
  if carrying and last_fix is not None:
    cloudlog.event("network_arbiterd_gps_carry_ended", fix_age_s=round(now - last_fix[2], 1),
                   reason="IsOnroad=1: ignition on, the carried fix is dropped" if onroad else "fresh GPS fix")
  return fresh, (None if fresh is None else (fresh[0], fresh[1], now)), False


# NOTE: the single-home _read_home/_save_home helpers were removed when this daemon moved to the
# multi-location model — per-entry locations now live in TetheringPriorityNetworks (priority_networks
# .py), auto-learned inline in main(). The legacy TetheringHomeLocation param is still read (only) by
# priority_networks.parse() for one-time migration of an old single-home setup.


# --- LTE PDN-throttle park/unpark (MODEM-level — never nmcli con modify a netplan gsm profile) -------

def _modem_index() -> str | None:
  """The modem's current ModemManager index (it RE-ENUMERATES on reset, so resolve it live)."""
  p = _run(["mmcli", "-L"])
  if p is None or p.returncode != 0:
    return None
  import re
  m = re.search(r"Modem/(\d+)", p.stdout)
  return m.group(1) if m else None


def _lte_throttled_recently() -> bool:
  p = _run(["sudo", "journalctl", "-u", "NetworkManager", "--no-pager", "--since", "-60s"])
  return p is not None and p.returncode == 0 and PDN_THROTTLE_TOKEN in p.stdout


def _lte_has_ip() -> bool:
  p = _run(["ip", "-4", "-o", "addr", "show", "wwan0"])
  return p is not None and p.returncode == 0 and "inet " in p.stdout


def _park_lte() -> None:
  """Stop the modem hammering the throttled PDN: disable the modem RF (mmcli) so it stops requesting a
  bearer and the carrier's tower-side timer can age out. NM can't activate a disabled modem, so the
  ~1/s retry loop stops — WITHOUT any nmcli con modify (which would crash NM on a netplan gsm profile)."""
  idx = _modem_index()
  cloudlog.warning(f"network_arbiterd: LTE PDN-throttled -> disabling modem {idx} to let carrier timer clear")
  _nmcli(["con", "down", LTE_CONNECTION_ID])
  if idx is not None:
    _run(["mmcli", "-m", idx, "--disable"])


def _unpark_lte() -> None:
  """Backoff elapsed: re-enable the modem RF; NM autoconnect (set in netplan) brings lte back up."""
  idx = _modem_index()
  cloudlog.info(f"network_arbiterd: LTE backoff elapsed -> re-enabling modem {idx}")
  if idx is not None:
    _run(["mmcli", "-m", idx, "--enable"])
  _nmcli(["con", "up", LTE_CONNECTION_ID])


# --- LTE signal-strength logging (network2xnor: log to qlog on change so slow spots are visible) -----

# coarse "bars" buckets from RSSI dBm (matches the spirit of deviceState.networkStrength on screen).
def _bars_from_dbm(dbm: float | None) -> int | None:
  if dbm is None:
    return None
  if dbm >= -75:
    return 4
  if dbm >= -85:
    return 3
  if dbm >= -95:
    return 2
  if dbm >= -105:
    return 1
  return 0


def _read_lte_operator(idx: str | None) -> str | None:
  """Carrier/provider name via `mmcli -m <idx> -J` (modem.3gpp.operator-name). None on failure.
  Changes rarely (roaming / tower handoff), so the caller caches it and only re-reads occasionally."""
  if idx is None:
    return None
  p = _run(["mmcli", "-m", idx, "-J"])
  if p is None or p.returncode != 0 or not p.stdout.strip():
    return None
  try:
    tgpp = json.loads(p.stdout).get("modem", {}).get("3gpp", {})
    op = (tgpp.get("operator-name") or "").strip()
    return op or None
  except (ValueError, AttributeError):
    return None


_signal_setup_done_for: str | None = None   # modem idx we've already enabled signal polling on


def _ensure_signal_setup(idx: str | None) -> None:
  """Enable the modem's periodic signal sampling ONCE per modem (not every loop).

  Hammering `mmcli --signal-setup` every cycle can wedge ModemManager during the initial carrier
  attach / PDN bearer activation (and blocks the loop up to NMCLI_TIMEOUT_S each time). The modem
  re-enumerates on reset, so we re-run setup only when the index changes. Mark done ONLY on success,
  so a failed setup during the busy boot/attach window is retried next time (else signal-get would
  silently return nothing forever for this session)."""
  global _signal_setup_done_for
  if idx is None or idx == _signal_setup_done_for:
    return
  p = _run(["mmcli", "-m", idx, "--signal-setup", "30"])   # 30 s polling; we only read on change anyway
  if p is not None and p.returncode == 0:
    _signal_setup_done_for = idx


def _read_lte_signal(idx: str | None) -> dict | None:
  """Modem signal metrics via `mmcli -m <idx> --signal-get -J` (RSSI/RSRP/RSRQ/SNR dBm). None on fail.
  Assumes signal polling was already enabled once via _ensure_signal_setup()."""
  if idx is None:
    return None
  p = _run(["mmcli", "-m", idx, "--signal-get", "-J"])
  if p is None or p.returncode != 0 or not p.stdout.strip():
    return None
  try:
    data = json.loads(p.stdout).get("modem", {}).get("signal", {})
  except (ValueError, AttributeError):
    return None

  def _g(*path):
    cur = data
    for k in path:
      if not isinstance(cur, dict):
        return None
      cur = cur.get(k)
    try:
      return round(float(cur), 1)
    except (TypeError, ValueError):
      return None

  # try the common access techs in order (lte, then 5g, then umts), first with a value wins.
  out: dict = {"access_tech": None}
  for tech in ("lte", "5g", "umts", "gsm"):
    rssi = _g(tech, "rssi")
    rsrp = _g(tech, "rsrp")
    if rssi is not None or rsrp is not None:
      out = {"access_tech": tech, "rssi": rssi, "rsrp": rsrp,
             "rsrq": _g(tech, "rsrq"), "snr": _g(tech, "snr") or _g(tech, "sinr")}
      break
  if out["access_tech"] is None:
    return None
  out["bars"] = _bars_from_dbm(out.get("rssi") if out.get("rssi") is not None else out.get("rsrp"))
  return out


def _log_signal_if_changed(idx: str | None, last: dict | None, operator: str | None) -> dict | None:
  """Read the modem signal and emit a qlog event ONLY when bars/dBm/operator changed since last tick.
  Returns the new reading (or `last` unchanged if nothing to report). `operator` = carrier name."""
  sig = _read_lte_signal(idx)
  if sig is None:
    return last
  sig["operator"] = operator
  # consider it "changed" if operator/access-tech changes, or any dBm moved >= 2. NOTE: we deliberately
  # do NOT treat a `bars` flip on its own as a change — bars is a bucketed dBm, so a 0.1 dBm jitter
  # across a bucket boundary (e.g. -75.0 -> -75.1) would flip bars every tick and spam qlog. Gating on
  # the >= 2 dBm move gives bars hysteresis for free: bars only effectively re-logs once the underlying
  # dBm has genuinely moved past the threshold.
  def _moved(a, b, key, thresh=2.0):
    va, vb = (a or {}).get(key), (b or {}).get(key)
    if va is None or vb is None:
      return va is not vb
    return abs(va - vb) >= thresh
  changed = (last is None
             or sig.get("access_tech") != (last or {}).get("access_tech")
             or sig.get("operator") != (last or {}).get("operator")
             or any(_moved(sig, last, k) for k in ("rssi", "rsrp", "rsrq", "snr")))
  if changed:
    cloudlog.event("network2xnor_lte_signal", operator=sig.get("operator"), bars=sig.get("bars"),
                   access_tech=sig.get("access_tech"), rssi=sig.get("rssi"), rsrp=sig.get("rsrp"),
                   rsrq=sig.get("rsrq"), snr=sig.get("snr"))
    return sig
  return last


def _has_connectivity() -> bool:
  """NM's view of internet reachability (used to decide whether a captive portal still needs poking)."""
  out = _nmcli(["-t", "-f", "CONNECTIVITY", "general"])
  return out is not None and out.strip() == "full"


def main() -> NoReturn:
  import platform
  params = Params()
  try:
    mem_params = Params("/dev/shm/params") if platform.system() != "Darwin" else None
  except Exception:
    mem_params = None
  cloudlog.info("network_arbiterd: started")

  lte_parked = False
  lte_parked_until = 0.0
  lte_throttle_count = 0
  last_signal: dict | None = None
  operator: str | None = None
  signal_tick = 0
  portal_done_for: str | None = None   # ssid we've already satisfied a captive portal for this session
  portal_tries: dict[str, int] = {}    # ssid -> accept() attempts this session (bounded by PORTAL_MAX_TRIES)
  portal_result: dict[str, bool] = {}  # ssid -> last accept() online result (set by the worker thread)
  portal_thread: threading.Thread | None = None  # captive-portal accept() runs here so it NEVER blocks this loop
  prev_on_priority: bool | None = None  # firehose2pnw: change-only publish of OnPriorityNetwork (flash-wear guard)
  assoc_fail: dict[str, tuple[int, float]] = {}  # netcosttier2pnw: ssid -> (consecutive failures, blocked-until)
  pending_up: tuple[str, float] | None = None   # netcosttier2pnw: (ssid, raised_at) awaiting judgement
  # hotspotretry2pnw: how the LAST client bring-up went, as (ssid_lower, classification, rc, error).
  # Overwritten on every up_priority/up_fallback, success included, so a later failure can never
  # inherit an older classification; consulted one tick later, when judge_link blames the bring-up.
  last_join: tuple[str, str, int | None, str] = ("", "", None, "")
  transient_used: set[str] = set()              # hotspotretry2pnw: ssids whose ONE free retry is spent
  join_class_logged: tuple | None = None        # hotspotretry2pnw: last logged classification (change-only)
  absent_scans: dict[str, int] = {}             # netcosttier2pnw: ssid -> consecutive REAL scans missing it
  prev_active_ssid = ""                          # netcosttier2pnw: to spot a link appearing that we did not raise
  # netscanpin2pnw: the driver's manual pick. The arbiter never WRITES WifiManualPick; it tracks the
  # (ssid, ts) it last saw and ends a pin by marking that identity ended, so a new pick written by the UI
  # while an old pin is ending can never be clobbered.
  pin_key: tuple[str, float] | None = None      # the pick identity currently tracked
  pin_first_seen = 0.0                          # monotonic time the arbiter first saw pin_key
  pin_seen_active = False                       # pin_key's network has been the active link since then
  pin_ended_key: tuple[str, float] | None = None  # pin_key once it has ended (dropped/failed/...)
  pin_problem = ""                              # last logged pick-read problem (change-only log)
  pin_held_logged: tuple | None = None          # last logged hold (change-only log)
  pin_home_state: dict[str, tuple[int, bool]] = {}  # netrank2pnw: absence since the pick (pinunmetered2pnw: every candidate)
  pin_kept_logged: set[tuple[str, str]] = set()  # pinunmetered2pnw: (network, why) already logged as not ending this pin
  cost_unread_logged: tuple | None = None       # netrank2pnw: last logged hold for an unreadable active cost
  last_upgrade_scan = float("-inf")             # netscanpin2pnw: last cost-upgrade scan issued
  last_fix: tuple[float, float, float] | None = None  # gpscarry2pnw: last fresh fix THIS PROCESS saw (never persisted)
  carrying_fix = False                          # gpscarry2pnw: last_fix is standing in for GPS (ignition off)
  seen_active: str | None = None                # smallfix0914pnw: active wifi at the last SUCCESSFUL read ("" = none; None = no read yet)
  requested_active: str | None = None           # smallfix0914pnw: what the arbiter's own action since then should leave active
  unread_since: float | None = None             # unreadhold2pnw: start of the current run of FAILED active reads (None = last read ok)
  unread_hold_logged: tuple | None = None       # unreadhold2pnw: last logged hold (change-only log)
  unread_release_logged = False                 # unreadhold2pnw: this run's release past the bound is logged
  verify_deferred_logged: tuple | None = None   # arbiterfu2pnw: the pending bring-up whose deferred judgement is logged

  while True:
    try:
      tethering_enabled = params.get_bool("TetheringEnabled")
      # netcosttier2pnw: re-read every tick so the kill switch takes effect without a restart.
      fallback_enabled = not params.get_bool("DisableNetworkCostLadder")
      # network2xnor (multi-location): the list param is authoritative; fall back to the legacy single
      # params so existing setups keep working (parse() migrates them transparently).
      nets = pn.parse(params.get("TetheringPriorityNetworks"),
                      legacy_ssid=(params.get("TetheringPriorityWifi") or ""),
                      legacy_home_raw=params.get("TetheringHomeLocation"))
      net_ssids = pn.ssids(nets)
      # hotspotretry2pnw: the configured entries that TRAVEL (the driver's iPhone). Only these get the
      # one-shot retry below: a phone hotspot that is not fully awake is the measured transient case,
      # while a stationary AP that refuses the stored PSK is a wrong password until proven otherwise.
      mobile_ssids = {(e.get("ssid") or "").strip().lower() for e in nets if e.get("mobile")}
      current_active, active_read_ok = _active_wifi_read()
      if active_read_ok:                        # unreadhold2pnw: a good read ends the run (and re-arms its logs)
        unread_since, unread_hold_logged, unread_release_logged = None, None, False
      elif unread_since is None:
        unread_since = time.monotonic()
      # smallfix0914pnw: log EVERY change of the active wifi connection, including the ones the arbiter did not
      # make. Measured 2026-09-13 22:01 PT: the phone dropped, NetworkManager itself autoconnected KarlMoik
      # (its profile has autoconnect=yes), and the arbiter logged nothing -- the switch was only visible as a
      # local-IP change in the cloud log. LOG ONLY: NM's choice is not fought here; everything below already
      # decides from this tick's read. An unreadable tick is no evidence of a change and moves nothing.
      if active_read_ok:
        now_active = current_active or ""
        if seen_active is not None and now_active != seen_active:
          if requested_active == now_active:
            by, extra = "arbiter", {}
          elif requested_active is not None:   # the arbiter asked for something else, e.g. a refused association
            by, extra = "not_as_requested", {"arbiter_requested": requested_active or None}
          else:
            by, extra = "external", {}
          cloudlog.event("network_arbiter_active_changed", **{"from": seen_active or None, "to": now_active or None},
                         by=by, **extra)
        seen_active, requested_active = now_active, None
      # netrank2pnw: the ACTIVE profile's connection.metered, read every tick while on client WiFi (one
      # nmcli). Both OnPriorityNetwork and the upgrade-scan decision depend on it, and both run before the
      # ladder's full cost read further down. A failed read reuses the last value (_metered_states); with
      # nothing cached it is None = unknown -- never assumed metered, never assumed unmetered.
      active_ssid_now = ssid_of(current_active or "")
      active_metered: str | None = None
      if active_ssid_now and current_active:
        _metered_states([current_active], {active_ssid_now})
        active_metered = _metered_cache.get(active_ssid_now)

      # netscanpin2pnw: the driver's manual pick. Read failures and damaged values are logged and change
      # NOTHING -- they are not evidence that a pin ended, nor that a new one began.
      try:
        pick, problem = parse_manual_pick(params.get("WifiManualPick"))
      except Exception as e:                    # e.g. UnknownKeyName on a device whose params_pyx.so was not rebuilt
        pick, problem = None, f"read failed: {type(e).__name__}: {e}"
      if problem != pin_problem:
        if problem:
          cloudlog.event("netcosttier_pin_unreadable", problem=problem, keeping=pin_key[0] if pin_key else None)
        pin_problem = problem
      if not problem:
        if pick is not None and pick != pin_key:
          if pin_key is not None and pin_key != pin_ended_key:
            cloudlog.event("netcosttier_pin_cleared", ssid=pin_key[0], reason="superseded", by=pick[0])
          pin_key, pin_first_seen, pin_seen_active, pin_ended_key = pick, time.monotonic(), False, None
          pin_home_state = {}                   # netrank2pnw: arrival evidence belongs to ONE pick
          pin_kept_logged = set()               # pinunmetered2pnw: ...and so do its "kept" log lines
          cloudlog.event("netcosttier_pin_set", ssid=pick[0])
        elif pick is None and pin_key is not None and pin_key != pin_ended_key:
          # present last tick, absent now: only a manager start (CLEAR_ON_MANAGER_START) or a hand removal
          # netrank2pnw: params_pyx returns None BOTH for an absent key and for a value it cannot decode
          # as JSON (it logs a cast warning and falls back to the default), so this reason covers both.
          cloudlog.event("netcosttier_pin_cleared", ssid=pin_key[0], reason="param_removed",
                         note="absent, or present but not decodable JSON (params_pyx returns None for both)")
          pin_ended_key = pin_key
      pin_in_force = pin_key is not None and pin_key != pin_ended_key
      gps_fresh = _read_gps(params, mem_params)
      # gpscarry2pnw: the geo-gate and a pin's arrival evidence use `gps`, which is the last fresh fix while
      # the ignition is off (see _carry_parked_fix). Learning below does NOT: it PERSISTS a location, so it
      # stays on fresh GPS only -- a carried fix is an inference, and IsOnroad=0 is not proof of standing still.
      gps, last_fix, carrying_fix = _carry_parked_fix(gps_fresh, params.get_bool("IsOnroad"), last_fix, carrying_fix)

      # auto-learn each network's location: if we're connected to one of OUR priority SSIDs right now,
      # this spot IS that network's geofence center -> update only that entry.
      # FLASH-WEAR GUARD: GPS jitters every read, so only WRITE the param when the fix has actually
      # moved meaningfully (> LEARN_MIN_MOVE_M) from the stored location, or it was never learned.
      # Otherwise this would params.put() a new JSON blob every 20 s forever, wearing the flash.
      if gps_fresh is not None and current_active:
        for e in nets:
          # uploadgate2pnw2: never learn a location for a mobile (hotspot) entry — it travels with the
          # car, so every LEARN_MIN_MOVE_M of driving would trigger a param write.
          if e.get("mobile"):
            continue
          if current_active == priority_connection_id(e["ssid"]):
            old = (e.get("lat"), e.get("lon"))
            moved = old[0] is None or old[1] is None or \
              haversine_m(old[0], old[1], gps_fresh[0], gps_fresh[1]) > LEARN_MIN_MOVE_M
            if moved:
              e["lat"], e["lon"] = round(gps_fresh[0], 6), round(gps_fresh[1], 6)
              params.put("TetheringPriorityNetworks", pn.dumps(nets))
            break

      # geo-gate: scan only when near ANY learned location (a scan competes with the hotspot on the
      # single radio). Fail-open when no locations learned yet or GPS missing.
      # ESCAPE HATCH (fix for the stale-GPS lockout): scan whenever we are NOT on a real client WiFi —
      # disconnected (current_active is None) OR sitting on our own hotspot. A stale "far from home"
      # GPS fix must never suppress the scan in those states, or the device can never find its home
      # WiFi to recover and sits stranded forever. Geo-gating only avoids needless scans once we're
      # already on a real client network; otherwise finding WiFi wins.
      on_client_wifi = current_active is not None and current_active != HOTSPOT_CONNECTION_ID
      # netscanpin2pnw: the geo-gate below was written when the only client WiFi the arbiter could be on
      # was a priority network, i.e. already the cheapest. On any other link a cheaper network can come
      # into range and can only be seen by scanning -- see network_arbiter.upgrade_scan_due for the
      # measured 66-minute case.
      # netrank2pnw: GENERIC -- due whenever the active network is not EXPLICITLY unmetered (see
      # upgrade_scan_due), and never while the link is settling: a bring-up awaiting judgement, or a link
      # that appeared since last tick, may still be inside its DHCP window.
      link_settling = pending_up is not None or (bool(active_ssid_now) and active_ssid_now.lower() != prev_active_ssid)
      # pinunmetered2pnw: a pin suppresses the upgrade scan only while its network is not the active link yet (the
      # join). Once on it, the scan runs like on any link not explicitly unmetered: an unmetered network arriving
      # now ends the pin, and on client WiFi away from a learned location this scan is the only way to see it.
      pin_joining = pin_in_force and pin_key is not None and active_ssid_now.lower() != pin_key[0].lower()
      upgrade_due = upgrade_scan_due(tethering_enabled and fallback_enabled, on_client_wifi,
                                     active_metered == "no", pin_joining, link_settling,
                                     time.monotonic(), last_upgrade_scan)
      if upgrade_due:
        last_upgrade_scan = time.monotonic()   # throttle the ATTEMPT, so a failing scan cannot hammer
        cloudlog.event("netcosttier_upgrade_scan", active=current_active, interval_s=UPGRADE_SCAN_S)
      allow_scan = (not on_client_wifi) or near_any_home(pn.locations(nets), gps) or upgrade_due
      # netcosttier2pnw: `net_ssids` alone is no longer the right precondition. That gate exists so
      # we do not burn the single radio scanning for nothing -- but with the cost ladder on there IS
      # something to look for even when the driver has configured no priority networks at all: every
      # other saved client profile is a candidate. Left as-is, removing the last priority entry
      # silently disabled the whole ladder (no scan -> no candidates -> straight to the hotspot),
      # with nothing to indicate why.
      scan_raw = _scan_ssids() if (tethering_enabled and (net_ssids or fallback_enabled) and allow_scan) else None
      scan = scan_raw if scan_raw is not None else []

      # STICKY ACTIVE CONNECTION: if we're ALREADY connected to one of our priority SSIDs, keep it —
      # do NOT require it to re-appear in this tick's scan. Otherwise, when the geo-gate pauses
      # scanning (or a scan simply omits the connected AP, which is common), chosen_ssid would go ""
      # and decide() would tear down a perfectly good client WiFi to raise the hotspot. We seed the
      # scan+chosen with the active network so decide() sees it as available and returns noop.
      active_entry = None
      if on_client_wifi:
        for e in nets:
          if current_active == priority_connection_id(e["ssid"]):
            active_entry = e
            break
      if active_entry is not None and active_entry["ssid"] not in scan:
        scan = [*scan, active_entry["ssid"]]

      # firehose2pnw: expose "connected to a priority (home / geo-gated) WiFi" so the uploader can run
      # pass-2 (rlog/HD) even while onroad. An EV (Lightning / Tesla) parked and CHARGING keeps
      # ignitionLine on -> onroad True, which would otherwise forbid the best upload window (parked for
      # hours on home WiFi). active_entry is non-None only when we're actually joined to one of our
      # priority SSIDs, so this is a strong SSID-match signal (no GPS-cold-start dependency).
      # netrank2pnw: a configured entry that is NOT explicitly metered (see on_priority_network). Was:
      # membership alone, exact case -- which let a configured, explicitly metered link authorise 75 MB
      # uploads over it. The uploader consumes this as `at_home`.
      on_priority = on_priority_network(active_ssid_now, net_ssids, active_metered)
      if on_priority != prev_on_priority:
        params.put_bool("OnPriorityNetwork", on_priority)
        prev_on_priority = on_priority

      # pick the first configured network that is both in range and has a saved NM connection.
      saved = _saved_connections()      # netcosttier2pnw: ONE read per tick, shared below
      chosen = pn.select_available(nets, scan, saved, priority_connection_id)
      chosen_ssid = chosen["ssid"] if chosen else ""

      # wifirepair2pnw: assert the autoconnect invariant while tethering is off (see the helper).
      if not tethering_enabled and current_active != HOTSPOT_CONNECTION_ID:
        _wifi_autoconnect_repair(nets)

      # netcosttier2pnw / netrank2pnw: the cost ladder. decide() ranks every saved network in range by cost
      # class first (explicitly unmetered < default < explicitly metered) and configured membership second
      # (net_ssids, in list order), so we only reach our own hotspot/LTE when nothing is usable.
      # `chosen_ssid` (the first reachable configured entry) is used only by the binary kill-switch mode.
      # decide() returns the SSID it picked, so the bring-up below cannot act on a different network than
      # the one that was ranked.
      now = time.monotonic()

      # One pure function decides what the client link's state MEANS -- whether it is sticky, and
      # what (if anything) goes in the ledger. It is in network_arbiter.judge_link and is tested
      # directly, because four separate review defects lived in this glue when it was inline:
      # an outright `con up` failure was never recorded (nothing is active to inspect, so only the
      # PENDING bring-up can see it); a flaky nmcli read was treated as a dead link and tore a
      # working one down; a link still doing DHCP was judged before NM's 45 s timeout; and a
      # network the driver joined by hand got blamed for the one we had raised.
      raw_active_ssid = ssid_of(current_active or "")

      # A link we did NOT raise still deserves the DHCP grace: the driver joining a network from the
      # UI, or NM autoconnecting one at boot, is caught mid-activation if the tick lands in that
      # window. Without this the first look blamed his manual join, un-stuck it and handed the radio
      # to the hotspot (Fable). Treat a newly-seen active link as a bring-up we are awaiting, judged
      # by the same three-way split.
      pending_up = pending_for_new_link(raw_active_ssid, pending_up, prev_active_ssid, now)
      prev_active_ssid = raw_active_ssid.lower()

      portal_entry = pn.entry_for_ssid(nets, raw_active_ssid) if raw_active_ssid else None
      usable = _client_link_usable(current_active, bool(portal_entry and portal_entry.get("portal"))) \
        if raw_active_ssid else None
      # arbiterfu2pnw: a failed active read is NO EVIDENCE about a pending bring-up (judge_link: None) -- for the same
      # bound as the unreadable hold below, and releasing on the same tick. Past it, judged as before: nothing readable
      # counts as nothing active, and an unverified blame is logged at ERROR level just below.
      # Fable: gated on the same "something to hold onto" as the hold below, so a deferral can never happen without
      # the hold also blocking radio actions -- structurally, not just because pending_up implies it. (A loop exception
      # on the judging tick could otherwise leave a pending bring-up with nothing seen or requested.)
      defer_judgement = unread_since is not None and bool(seen_active or requested_active) \
        and now - unread_since < ACTIVE_UNREADABLE_HOLD_S
      if not defer_judgement:
        verify_deferred_logged = None           # re-armed, so the next unreadable episode is logged again
      elif pending_up is not None and pending_up != verify_deferred_logged:
        cloudlog.event("netcosttier_verify_deferred", ssid=pending_up[0], unreadable_s=round(now - unread_since, 1),
                       hold_s=ACTIVE_UNREADABLE_HOLD_S, reason="nmcli con show --active failed; judged on the next good read")
        verify_deferred_logged = pending_up
      verdict = judge_link(None if defer_judgement else raw_active_ssid, usable, pending_up, now, DHCP_GRACE_S,
                           last_known_usable=_usable_cache.get(raw_active_ssid.lower()))
      if usable is not None and raw_active_ssid:
        _usable_cache[raw_active_ssid.lower()] = usable
      active_ssid = verdict.sticky_ssid
      pending_up = verdict.pending
      if verdict.blame and not verdict.blame_ok and unread_since is not None:
        cloudlog.event("netcosttier_blamed_unverified", ssid=verdict.blame.lower(), unreadable_s=round(now - unread_since, 1),
                       hold_s=ACTIVE_UNREADABLE_HOLD_S,
                       error="nmcli con show --active kept failing past the hold; counting the bring-up as failed without a verification read")
      if verdict.blame:
        # hotspotretry2pnw: A TRANSIENT JOIN FAILURE ON THE PHONE COSTS ONE POLL, NOT 5-15 MINUTES.
        # Measured 2026-09-14 18:55 PT: `con up` on the iPhone hotspot failed with "Secrets were
        # required, but not provided" (the supplicant had reported WRONG_KEY) -- and the SAME profile
        # joined the SAME phone cleanly three hours later and again the next morning, so the password
        # was never wrong; the phone was not awake. Today that failure is blamed immediately and the
        # ledger exiles the phone for 60 s, then 300 s, then 900 s.
        # So: the FIRST such failure on a configured MOBILE entry is not blamed. Nothing else changes --
        # the network is simply not in `blocked` below, so this same tick's ladder raises it again, one
        # poll (20 s) after it failed. The retry is spent per network and is returned ONLY by a
        # successful join, so a phone that always fails is blamed on its second try and lands in the
        # ordinary escalating backoff: one extra `con up` per episode, and the 15-minute steady state
        # is untouched. `blame_ok` (the link came up) always goes to _note_attempt, which is what logs
        # netcosttier_recovered and clears the ledger.
        blamed = verdict.blame.lower()
        cls = last_join[1] if last_join[0] == blamed else ""
        retry = (not verdict.blame_ok and cls == "transient" and blamed in mobile_ssids
                 and blamed not in transient_used)
        if cls and not verdict.blame_ok:
          # Rule 2: say which way this failure was read, with the rc and NM's own words, so an error
          # string classify_join_failure does not recognise is visible instead of silently blamed.
          entry = (blamed, last_join[2], last_join[3], cls, retry)
          if entry != join_class_logged:
            rule = ("a transient join failure on a configured mobile entry is retried once before the ledger blames it"
                    if retry else
                    "not a transient failure -- blamed as usual" if cls != "transient" else
                    "not a configured mobile entry -- blamed as usual" if blamed not in mobile_ssids else
                    "its one retry is already spent; only a successful join returns it")
            cloudlog.event("netcosttier_join_classified", ssid=blamed, classification=cls,
                           rc=last_join[2], error=last_join[3], mobile=blamed in mobile_ssids,
                           retrying=retry, retry_in_s=POLL_INTERVAL_S if retry else None, rule=rule)
            join_class_logged = entry
        if retry:
          transient_used.add(blamed)
        else:
          if verdict.blame_ok:
            transient_used.discard(blamed)
            # Fable: a classified failure AFTER a successful join is news -- re-arm the change-only log, or the
            # evening retry of an ssid that already failed this morning goes unreported (Rule 2).
            join_class_logged = None
          _note_attempt(assoc_fail, verdict.blame, verdict.blame_ok, now)
        if last_join[0] == blamed:
          # Fable: the classification is CONSUMED here. Without this, `last_join` lingers and a LATER, unrelated
          # blame of the same ssid (an external NM autoconnect that then dies in DHCP) inherits the old "transient"
          # verdict and is swallowed once.
          last_join = ("", "", None, "")

      # A network that has been genuinely out of range and has come back gets a clean slate.
      _forget_on_reappearance(assoc_fail, absent_scans,
                              None if scan_raw is None else {s.lower() for s in scan_raw})
      blocked = _blocked(assoc_fail, now)
      # cost is only needed when the ladder can actually run, and only for networks that could be
      # chosen this tick -- one nmcli per candidate, not per saved profile ever created.
      metered_ssids: set[str] = set()
      unmetered_ssids: set[str] = set()
      if tethering_enabled and fallback_enabled:
        # the active profile was already read at the top of this tick -- do not pay for it twice
        of_interest = {x for x in scan if x.lower() != raw_active_ssid.lower()}
        metered_ssids, unmetered_ssids = _metered_states(saved, of_interest)
        if raw_active_ssid and active_metered == "yes":
          metered_ssids.add(raw_active_ssid)
        elif raw_active_ssid and active_metered == "no":
          unmetered_ssids.add(raw_active_ssid)

      # netscanpin2pnw: is the driver's manual pick still in force? Judged AFTER judge_link, so a pinned
      # network that has failed its usability check (and was just blamed for it) ends the pin THIS tick --
      # a pin must never hold the device offline. active_ssid None = the read failed = no evidence.
      # netrank2pnw: judged after the cost read and the ledger too, because a pin now also ends when a
      # stationary, explicitly unmetered configured network is in range (home_to_yield_to, Fable D2).
      pin_ssid = pin_key[0] if pin_in_force and pin_key else ""
      home = ""
      unmet = UnmeteredYield("", "", "")
      if pin_ssid:
        # netrank2pnw: a pin gives way only to a home network that has ARRIVED since the pick -- genuinely
        # absent (consecutive real scans, or GPS confidently far) and then back. A pick made while home is
        # visible therefore sticks. See update_home_arrival for what counts as evidence.
        # pinunmetered2pnw: arrival is tracked for EVERY configured entry, mobile ones included (arrival_candidates),
        # because any explicitly unmetered one can now end a pin; stationary entries keep their GPS evidence.
        # pinconfigured2pnw: configured entries only -- an unconfigured saved profile cannot end a pin.
        stationary = [e for e in nets if not e.get("mobile")]
        pin_home_state = update_home_arrival(pin_home_state, arrival_candidates(nets), scan_raw, gps)
        arrived = {k for k, (_m, gone) in pin_home_state.items() if gone}
        home = home_to_yield_to([e["ssid"] for e in stationary], scan_raw, saved, unmetered_ssids, blocked,
                                pin_ssid, arrived=arrived)
        # pinunmetered2pnw (owner decision 2026-09-14): "when an unmetered network appears !" -- an explicitly
        # unmetered saved network that ARRIVED ends a pin on a network that is not explicitly unmetered. The pinned
        # network's cost is the last one READ (the active link's is read every tick; None = never read).
        # pinconfigured2pnw (owner decision 2026-09-14 ~21:30 PT): "(no just the configued ones)" -- only a
        # TetheringPriorityNetworks entry (net_ssids, stationary or mobile) can end it.
        pinned_metered = next((v for k, v in _metered_cache.items() if k.lower() == pin_ssid.lower()), None)
        unmet = unmetered_to_yield_to(scan_raw, saved, net_ssids, unmetered_ssids, blocked, pin_ssid, pinned_metered,
                                      arrived)
      pv = judge_pin(pin_ssid, pin_first_seen, pin_seen_active,
                     raw_active_ssid if active_read_ok else None,
                     bool(pin_ssid) and verdict.blame.lower() == pin_ssid.lower() and not verdict.blame_ok,
                     now, home_ssid=home, unmetered_ssid=unmet.ends_by)
      pin_seen_active = pv.seen_active
      if pv.ended:
        # `trigger` is the home network whose arrival ended the pin -- NOT necessarily what the ladder
        # joins next (an explicitly unmetered member earlier in the list would win). The bring-up that
        # follows logs what is actually joined. (Was `to=`, which claimed the latter.) `by` for the unmetered
        # reason means the same: the network whose arrival ended it.
        cloudlog.event("netcosttier_pin_cleared", ssid=pin_ssid, reason=pv.ended,
                       **({"trigger": home} if pv.ended == "home" else
                          {"by": unmet.ends_by} if pv.ended == "unmetered" else {}))
        pin_ended_key = pin_key
      elif unmet.kept_by and (unmet.kept_by.lower(), unmet.kept_why) not in pin_kept_logged:
        # pinunmetered2pnw: say why a cheaper, explicitly unmetered network in range did NOT end the pin -- otherwise
        # "the phone is on and nothing happens" is indistinguishable from a broken rule. Once per network per pick.
        cloudlog.event("netcosttier_pin_kept", ssid=pin_ssid, by=unmet.kept_by, reason=unmet.kept_why,
                       rule="an unmetered network ends a pin only after it has been out of range since the pick"
                       if unmet.kept_why == "visible_since_pick" else
                       "only a network in TetheringPriorityNetworks ends a pin; this one is saved and unmetered but not configured"
                       if unmet.kept_why == "not_configured" else
                       "the pinned network's connection.metered could not be read, so nothing is known to be cheaper")
        pin_kept_logged.add((unmet.kept_by.lower(), unmet.kept_why))
      pinned = bool(pv.pinned_ssid)

      action, target_ssid = decide(
        tethering_enabled=tethering_enabled,
        priority_ssid=chosen_ssid,
        scan_ssids=scan,
        saved_connections=saved,
        current_active=current_active,
        metered_ssids=metered_ssids,
        fallback_enabled=fallback_enabled,
        blocked_ssids=blocked,
        unmetered_ssids=unmetered_ssids,
        active_ssid=active_ssid,
        priority_ssids=net_ssids,          # netrank2pnw: membership = a tiebreak inside a cost class
      )
      # netrank2pnw (Fable D1): A COST-DRIVEN SWITCH NEEDS A COST THAT WAS ACTUALLY READ. When the ACTIVE
      # profile's connection.metered read FAILED and nothing is cached (active_metered None -- distinct from
      # a successful read of "unknown"), choose_wifi ranks the active link as unknown and an explicitly
      # unmetered candidate outranks it. Measured in Fable's loop harness: at home on Hannelore (`no`) with
      # the phone (`no`) in range, one failed first read -> [(0 s, iPhone), (20 s, Hannelore)] -- home torn
      # down and restored, at boot, which is exactly when NM is slowest to answer. So: while the active
      # link is USABLE and its cost unread, the ladder may not displace it. Hold, and say so. A link that is
      # NOT usable is exempt: that move is failure-driven, and a hold must never keep the device on a dead
      # link. The kill switch's binary mode has no notion of cost and is exempt too.
      if (tethering_enabled and fallback_enabled and raw_active_ssid and active_metered is None
          and active_ssid and active_ssid.lower() == raw_active_ssid.lower()
          and action in ("up_priority", "up_fallback", "up_hotspot")):
        held = (raw_active_ssid, action, target_ssid)
        if held != cost_unread_logged:
          cloudlog.event("netcosttier_active_cost_unreadable", active=raw_active_ssid, suppressed=action,
                         target=target_ssid, error="connection.metered read failed and nothing is cached")
          cost_unread_logged = held
        action, target_ssid = "noop", ""
      elif active_metered is not None:
        cost_unread_logged = None

      # netscanpin2pnw: HOLD. While the driver's pick is in force the arbiter takes no radio action for
      # cost reasons -- neither during the join (so it never fights the UI's own activation) nor after.
      # Pins are a cost-ladder feature: with DisableNetworkCostLadder set the behaviour is the pre-ladder
      # binary one, which never knew about manual picks. down_hotspot is never suppressed.
      if pinned and tethering_enabled and fallback_enabled and action in ("up_priority", "up_fallback", "up_hotspot"):
        held = (pin_key, action, target_ssid)
        if held != pin_held_logged:
          cloudlog.event("netcosttier_pin_held", pinned=pv.pinned_ssid, suppressed=action, target=target_ssid)
          pin_held_logged = held
        action, target_ssid = "noop", ""

      # unreadhold2pnw (Fable, confirmed): A FAILED READ OF THE ACTIVE CONNECTION NEVER MOVES THE RADIO -- the same
      # rule as the cost hold above (D1), for the active read itself. A failed `con show --active` reads as
      # current_active None, so on_client_wifi is False and the sticky seeding never runs; the scan commonly omits
      # the connected AP, so decide() found nothing usable and raised the hotspot on a working link. Fable's loop
      # harness, on the phone: [(40 s, Hotspot), (60 s, iPhone)] from one nmcli hiccup. With the AP in the scan it
      # re-ran `con up` on the already-active link instead, which re-activates it on real NM. down_hotspot needs
      # current_active == Hotspot, so it cannot fire on an unreadable tick and needs no guard.
      # BOUNDED, unlike D1: D1 only holds a link whose usability was just READ as good, so it cannot hold a dead
      # one. Here nothing about the link could be read and it may genuinely be gone, so after
      # ACTIVE_UNREADABLE_HOLD_S of consecutive failed reads the action goes through as before, at ERROR level.
      # Fable (re-review): hold only if there IS something to hold onto -- a link seen active at the last good read,
      # or our own unresolved bring-up. At boot (no read yet) or after a good read of "nothing active", an
      # unreadable tick must not delay the first connection by up to ACTIVE_UNREADABLE_HOLD_S.
      if unread_since is not None and (seen_active or requested_active) and action in ("up_priority", "up_fallback", "up_hotspot"):
        unread_s = round(now - unread_since, 1)
        if now - unread_since < ACTIVE_UNREADABLE_HOLD_S:
          held = (action, target_ssid)
          if held != unread_hold_logged:
            cloudlog.event("network_arbiter_active_unreadable_hold", suppressed=action, target=target_ssid,
                           unreadable_s=unread_s, hold_s=ACTIVE_UNREADABLE_HOLD_S)
            unread_hold_logged = held
          action, target_ssid = "noop", ""
        elif not unread_release_logged:
          cloudlog.event("network_arbiter_active_unreadable_released", action=action, target=target_ssid,
                         unreadable_s=unread_s, hold_s=ACTIVE_UNREADABLE_HOLD_S,
                         error="nmcli con show --active kept failing past the hold; acting without knowing the active link")
          unread_release_logged = True
      # arbiterfu2pnw: the fallback line's reason, from decide()'s own inputs. Log text must never alter a decision, and an
      # exception here would escape to the loop's handler and skip the bring-up -- so it is caught, logged, and replaced.
      why = ""
      if action == "up_fallback":
        try:
          why = explain_fallback(net_ssids, scan, saved, target_ssid, metered_ssids, unmetered_ssids, blocked,
                                 active_ssid or "", scanned=scan_raw is not None)
        except Exception as e:
          cloudlog.exception("network_arbiterd: could not explain the fallback choice; joining it anyway")
          why = f"reason unavailable: {type(e).__name__}"
      join_cls, join_rc, join_err = _apply(action, target_ssid, current_active, active_read_ok, why)
      expected = _expected_active(action, target_ssid)
      if expected is not None:
        requested_active = expected
      if action in ("up_priority", "up_fallback") and target_ssid:
        # Remember what we raised. A `con up` that fails outright leaves NOTHING active, so this is
        # the only way that failure is ever visible -- judging the active link alone cannot see it.
        pending_up = (target_ssid, now)
        # hotspotretry2pnw: ...and HOW it went, for the blame one tick from now.
        last_join = (target_ssid.lower(), join_cls, join_rc, join_err)
        prev_active_ssid = target_ssid.lower()
        # ...and DROP any cached usability for it. DEFENCE IN DEPTH, not the load-bearing fix: with
        # judge_link keeping a pending link sticky through an unreadable tick regardless of the
        # cache, nothing consults a stale entry between the raise and its resolution. This keeps a
        # stale False from mattering if that ever changes. Untested wiring, deliberately kept.
        _usable_cache.pop(target_ssid.lower(), None)

      # ...and VERIFY it. `up_priority`/`up_fallback` drop the hotspot BEFORE raising the client, so a
      # failed association leaves the device with no uplink at all. Unverified, the same network would
      # be picked again next tick (it is still in the scan) and the device would sit offline forever.
      # Rule 2: the failure is recorded as an EVENT and the network serves an escalating backoff, so
      # the ladder moves on to the next tier instead of silently retrying the thing that does not work.

      # captive-portal auto-accept: when we're sitting on one of OUR SSIDs that declares a portal
      # handler and we don't yet have full connectivity, POST its accept form (once per session).
      active_portal_entry = pn.entry_for_ssid(
        nets, current_active.replace("openpilot connection ", "")) if current_active else None
      if active_portal_entry and active_portal_entry.get("portal"):
        ssid = active_portal_entry["ssid"]
        # Poke the portal even when NM reports global connectivity "full": the comma keeps LTE up
        # alongside WiFi, so LTE's connectivity masks a captive WiFi (the default route) and the old
        # `if not _has_connectivity()` guard never fired here. accept() is HTTP-ONLY — it never touches
        # NM, the LTE connection, the hotspot, or routing.
        #
        # CRITICAL: run accept() in a DAEMON THREAD, never inline. A DNS-blocking portal can make its
        # GETs time out (~tens of seconds), and blocking THIS loop would delay LTE-signal logging and
        # tethering/hotspot NAT upkeep. The thread stores its result in portal_result; we read it next
        # loop. One thread at a time; bounded to PORTAL_MAX_TRIES per session.
        if portal_result.get(ssid):                # worker confirmed we're actually online -> done
          portal_done_for = ssid
        if (portal_done_for != ssid and portal_tries.get(ssid, 0) < PORTAL_MAX_TRIES
            and (portal_thread is None or not portal_thread.is_alive())):
          portal_tries[ssid] = portal_tries.get(ssid, 0) + 1
          cloudlog.event("network2xnor_portal_try", ssid=ssid, portal=active_portal_entry["portal"],
                         current_active=current_active, connectivity_full=_has_connectivity(),
                         attempt=portal_tries[ssid])

          def _poke(handler=active_portal_entry["portal"], sid=ssid):
            try:
              portal_result[sid] = bool(captive_portal.accept(handler, already_online=False))
            except Exception:
              portal_result[sid] = False

          portal_thread = threading.Thread(target=_poke, name="captive_portal", daemon=True)
          portal_thread.start()
      else:
        # not on a configured portal SSID -> reset session state so re-entry re-tries cleanly. A still-
        # running daemon thread just finishes its HTTP harmlessly (it only writes its result file).
        # Mutate (.clear()) rather than rebind so the worker thread's closure always shares this object.
        if portal_done_for is not None or portal_tries or portal_result:
          portal_done_for = None
          portal_tries.clear()
          portal_result.clear()
    except Exception:
      cloudlog.exception("network_arbiterd: unhandled error in loop")

    # LTE signal-strength logging (operator + bars + dBm, only on change -> qlog timeline of slow
    # spots). Runs on a SLOWER cadence than the arbiter loop (every SIGNAL_EVERY_N ticks ~= 60 s), so
    # the up-to-4 sequential mmcli calls here can't repeatedly stall the WiFi-recovery logic that runs
    # earlier in the loop (each _run has a 15 s timeout). Operator name changes rarely -> re-read it
    # only occasionally. Skipped entirely while LTE is parked (modem RF off) — reads would just block.
    signal_tick += 1
    if not lte_parked and signal_tick % SIGNAL_EVERY_N == 0:
      try:
        idx = _modem_index()
        _ensure_signal_setup(idx)   # once per modem, NOT every loop (avoid wedging ModemManager)
        if operator is None or signal_tick % (SIGNAL_EVERY_N * 10) == 0:
          operator = _read_lte_operator(idx)
        last_signal = _log_signal_if_changed(idx, last_signal, operator)
      except Exception:
        cloudlog.exception("network_arbiterd: unhandled error in signal logging")

    # LTE PDN-throttle backoff guard
    try:
      lte_action, lte_parked, lte_parked_until, lte_throttle_count = decide_lte_guard(
        now=time.monotonic(),
        throttled=_lte_throttled_recently() if not lte_parked else False,
        lte_has_ip=_lte_has_ip() if not lte_parked else False,
        parked=lte_parked,
        parked_until=lte_parked_until,
        throttle_count=lte_throttle_count,
      )
      if lte_action == "park":
        _park_lte()
      elif lte_action == "unpark":
        _unpark_lte()
    except Exception:
      cloudlog.exception("network_arbiterd: unhandled error in lte guard")

    time.sleep(POLL_INTERVAL_S)


if __name__ == "__main__":
  main()
