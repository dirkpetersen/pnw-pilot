"""
network2xnor: pure arbitration logic for perpetual tethering + priority WiFi.

This module contains ONLY the decision function `decide(...)`. It performs no I/O — it takes a
snapshot of the world (params + NetworkManager scan/state) and returns exactly one action string.
The supervisor process (`network_arbiterd.py`) wraps it with the nmcli I/O.

Keeping the decision pure makes it trivially unit-testable and keeps the dangerous part (running
nmcli, dropping the hotspot) thin and obvious.

Actions:
  'up_priority'  -> bring the saved priority-wifi client connection up (drops the hotspot)
  'up_fallback'  -> bring some OTHER saved client wifi up (netcosttier2pnw; drops the hotspot)
  'up_hotspot'   -> bring the Hotspot AP up
  'down_hotspot' -> tear the Hotspot AP down (tethering disabled but AP still up)
  'noop'         -> nothing to do; current state already matches the desired state

netcosttier2pnw (2026-09-10, driver spec) — THE COST LADDER. Until now this module was BINARY: a
configured priority SSID, or else the hotspot. Any other saved WiFi in range was never considered,
so the device fell straight to its own LTE — the most expensive link it has — while cheaper WiFi sat
in range. Driver, verbatim: "the tethering network should be the lowest priority if another wifi
connection is available because the tethering network is the most expensive network".

netrank2pnw (2026-09-13, driver spec) — THE RANKING IS GENERIC. netcosttier2pnw's tier ladder (a configured entry
= tier 0 unless explicitly metered) is SUPERSEDED. Driver, verbatim: "we need a generic solution where an
unmetered network is always prioritized over a metered network or a default setting". Every saved network
in range is ranked by one key:

    1. cost class       explicitly unmetered (NM `no`) < default/unknown < explicitly metered (`yes`)
    2. membership       a configured TetheringPriorityNetworks entry beats a non-member; members keep the
                        driver's list order among themselves
    3. ssid             stable tiebreak, so equal networks never flap
    and last, the comma's OWN hotspot + LTE when nothing is usable.

What that means for the two rankings netcosttier2pnw rejected:
  * LIST POSITION is still never a COST ranking -- it only orders members inside one cost class, where
    the driver's own order is the only preference left to express.
  * the `mobile` FLAG still does not rank anything. It does decide one thing now: a manual pick yields
    to a STATIONARY, explicitly unmetered configured network in range (home_to_yield_to) -- a mobile
    entry such as the iPhone never ends a pin.
"""
from __future__ import annotations

from typing import NamedTuple

from openpilot.system.networkd.geo_gate import HOME_GEOFENCE_M, haversine_m

# NM connection ids. The Hotspot connection is always named "Hotspot"; saved client networks are
# created by wifi_manager.connect_to_network as "openpilot connection <SSID>".
HOTSPOT_CONNECTION_ID = "Hotspot"


def priority_connection_id(ssid: str) -> str:
  """The NM connection id wifi_manager uses for a saved client network."""
  return f"openpilot connection {ssid}"


def ssid_of(connection_id: str) -> str:
  """The SSID inside a saved client connection id, or "" if it is not one of ours.

  wifi_manager names saved client networks "openpilot connection <SSID>"; anything else (the Hotspot,
  the LTE gsm profile, a hand-made profile) is not a candidate and must not be guessed at."""
  prefix = "openpilot connection "
  return connection_id[len(prefix):].strip() if connection_id.startswith(prefix) else ""


class LinkVerdict(NamedTuple):
  """What this tick concluded about the client link. All fields are decisions, never I/O."""
  sticky_ssid: str                       # "" -> the active link must NOT be treated as a candidate
  blame: str                             # "" -> record nothing in the ledger this tick
  blame_ok: bool                         # only meaningful when `blame` is set
  pending: tuple[str, float] | None      # a bring-up still awaiting judgement (ssid, raised_at)


# --- netscanpin2pnw: the driver's manual pick sticks, and a tier-1/2 link keeps looking for cheaper ---

PIN_JOIN_WINDOW_S = 90.0   # a pick that has not become the active network this long after the arbiter
                           # first saw it has failed to join (wrong password, AP gone). NM's DHCP timeout
                           # is 45 s; the UI's password path also deletes and re-adds the profile first.
UPGRADE_SCAN_S = 120.0     # at most one cost-upgrade scan per this, on a client link not explicitly unmetered


def parse_manual_pick(raw: object) -> tuple[tuple[str, float] | None, str]:
  """WifiManualPick -> ((ssid, ts), "") or (None, why). `why` is "" ONLY for a genuinely absent pick.

  Absence and damage are different facts and must stay different: None (never written, or cleared on
  boot) is a real "no pin"; a value that is present but unusable is a problem the caller logs, because
  it means the driver may have picked a network that the arbiter cannot honour."""
  if raw is None:
    return None, ""
  if not isinstance(raw, dict):
    return None, f"not an object: {type(raw).__name__}"
  ssid, ts = raw.get("ssid"), raw.get("ts")
  if not isinstance(ssid, str) or not ssid.strip():
    return None, "missing or empty ssid"
  if isinstance(ts, bool) or not isinstance(ts, (int, float)):
    return None, "missing or non-numeric ts"
  return (ssid.strip(), float(ts)), ""


class PinVerdict(NamedTuple):
  pinned_ssid: str    # "" -> no pin in force this tick
  seen_active: bool   # the pinned network has been the active link at least once since this pick
  ended: str          # why the pin ended THIS tick ("" if it did not)


def judge_pin(pick_ssid: str, first_seen: float, seen_active: bool, active_ssid: str | None,
              blamed_failed: bool, now: float, join_window_s: float = PIN_JOIN_WINDOW_S,
              home_ssid: str = "") -> PinVerdict:
  """Is the driver's manual pick still in force?

  A pin HOLDS the radio: while it is in force the arbiter takes no cost-driven action at all. That
  covers both the join itself (so the arbiter never fights the UI's own activation) and the time the
  driver spends on the network he chose.

  It ENDS -- and the cost ladder resumes -- when:
    * failed        judge_link blamed the pinned network (associated, no usable link, past the DHCP
                    grace). A pin must never hold the device offline; the failure ledger then applies.
    * dropped       it WAS the active link and no longer is (out of range, or the driver joined
                    something else by a path that did not record a pick).
    * join_timeout  it never became the active link within join_window_s (wrong password, AP gone).
    * home          netrank2pnw (Fable D2): a stationary, explicitly unmetered configured network is in
                    range (home_ssid, from home_to_yield_to). The ladder then takes it.
    * superseded    a newer pick replaced it -- decided by the caller, which sees the new (ssid, ts).
    * reboot        WifiManualPick is CLEAR_ON_MANAGER_START.

  `active_ssid` None means the active connection COULD NOT BE READ (nmcli failed). That is no evidence,
  so the pin neither ends nor advances -- an unreadable tick must not look like "the network dropped".
  An empty string is a real reading: nothing is active."""
  if not pick_ssid:
    return PinVerdict("", False, "")
  if active_ssid is None:
    return PinVerdict(pick_ssid, seen_active, "")
  if blamed_failed:
    return PinVerdict("", seen_active, "failed")
  if home_ssid:
    return PinVerdict("", seen_active, "home")
  if active_ssid.lower() == pick_ssid.lower():
    return PinVerdict(pick_ssid, True, "")
  if seen_active:
    return PinVerdict("", True, "dropped")
  if now - first_seen >= join_window_s:
    return PinVerdict("", False, "join_timeout")
  return PinVerdict(pick_ssid, False, "")


def upgrade_scan_due(ladder_on: bool, on_client_wifi: bool, active_unmetered: bool, pinned: bool,
                     link_settling: bool, now: float, last_scan: float, interval_s: float = UPGRADE_SCAN_S) -> bool:
  """Should the arbiter scan for a CHEAPER network although the geo-gate would not?

  The geo-gate stops scanning once we are on client WiFi away from a learned location -- a rule written
  when the only client WiFi the arbiter could join was a priority network, i.e. already the cheapest.
  Measured 2026-09-13: 66 min on metered Starlink with the unmetered iPhone never considered, and NM 1.46
  does not rescan on its own while associated.

  netrank2pnw -- GENERIC: due whenever the active network is not EXPLICITLY UNMETERED. The first cut
  (netscanpin2pnw) exempted every configured priority entry not known to be metered, which matched the one
  measured case but not the driver's rule: "we don't want a solution where it just scans the Starlink
  SSIDs that I have configured; we need a generic solution". Under the generic ranking a configured
  `unknown` network can be beaten by an explicitly unmetered one, so being on it is not "done" either, and
  only a scan can find the better network. On an explicitly unmetered link nothing can outrank it on cost,
  so no scan.

  Suppressed:
    * ladder off         -- kill switch = pre-ladder behaviour
    * not on client wifi -- off client wifi the geo-gate already lets scans through
    * active unmetered   -- nothing is cheaper
    * pinned             -- the driver chose this network
    * link_settling      -- a bring-up is awaiting judgement, or the link appeared this tick. A scan takes
                            the single radio off-channel, and the FIRST upgrade scan is immediate, so without
                            this it could land inside the link's DHCP window (Fable, netscanpin2pnw review).
    * throttled          -- at most one per interval_s
  """
  return (ladder_on and on_client_wifi and not active_unmetered and not pinned and not link_settling
          and now - last_scan >= interval_s)


PIN_HOME_ABSENT_SCANS = 3                  # consecutive REAL scans missing a home network = genuinely absent
PIN_HOME_FAR_M = 2.0 * HOME_GEOFENCE_M      # confidently far from a home network's learned location


def update_home_arrival(state: dict[str, tuple[int, bool]],
                        stationary: list[tuple[str, float | None, float | None]],
                        scan_ssids: list[str] | None,
                        gps: tuple[float, float] | None) -> dict[str, tuple[int, bool]]:
  """netrank2pnw (Fable D2): has each home network been GENUINELY ABSENT since the current pick was made?

  Returns the new state, {ssid_lower: (consecutive_missing_real_scans, absent_since_pick)}. The caller
  starts it empty when a pick is first seen and drops it when the pin ends. A pin may yield to a home
  network only once that network has been established absent since the pick and then appears -- so a pick
  made deliberately WHILE home is visible sticks, and a pick made on the road still gives way on arrival.
  The case the driver approved: "pick Starlink, drive home, and the truck stays on paid Starlink in the
  driveway" must not happen; a pick made at home must not revert on the next tick either.

  ABSENCE IS ESTABLISHED ONLY BY EVIDENCE, two kinds:

    * PIN_HOME_ABSENT_SCANS consecutive REAL scans that do not list it -- ONLY while GPS cannot place the
      truck within PIN_HOME_FAR_M of the network's learned location (no fix, or no learned location). With
      GPS saying the truck is still there, a missing result is the AP, not the truck: a router reboot, a
      DFS check, a weak signal from the garage. A scan that did not run (scan_ssids None) is no evidence
      and changes nothing. A single missing result is not absence -- APs drop out of scans -- and any scan
      that DOES list it resets the count, so flicker never adds up.

    * GPS: the truck is more than PIN_HOME_FAR_M from that network's LEARNED location, on a tick where no
      real scan listed it. This is how "not visible at pick time" is established when no scan ran around
      the pick -- which is the NORMAL case on the road, where the geo-gate suppresses scanning on client
      WiFi and the first real scan happens only on arriving near home. Without it, a road pick could
      never yield. Twice the geofence, not the geofence itself: parked at home a truck can sit near the
      edge of a location learned where it first connected, and GPS jitters.

  Everything else is NOT evidence of absence and leaves a network visible-at-pick: no GPS fix, no
  learned location (a stationary entry never connected to), a far GPS reading on a tick whose real scan
  still lists the network (the scan wins -- the learned location may be stale). If neither kind of
  evidence ever appears after the pick, the network counts as visible when the pick was made, and the pin
  sticks. Once established, absence persists for the life of the pin: arriving home, the network is
  present from then on, and it must still be allowed to end the pin (e.g. once a failure backoff expires).
  """
  scan = None if scan_ssids is None else {x.lower() for x in scan_ssids}
  out = dict(state)
  for ssid, lat, lon in stationary:
    low = (ssid or "").strip().lower()
    if not low:
      continue
    missing, absent = out.get(low, (0, False))
    if absent:
      continue
    if scan is not None and low in scan:
      out[low] = (0, False)              # seen: present now, and any run of misses is broken
      continue
    dist = (haversine_m(lat, lon, gps[0], gps[1])
            if gps is not None and lat is not None and lon is not None else None)
    # A missing scan result counts ONLY when GPS cannot say the truck is still at that network's learned
    # location (Fable, netrank2pnw review). Parked at home, three misses at the 20 s poll is one minute:
    # a router reboot, a 5 GHz DFS channel-availability check, or a weak AP heard from the garage -- and a
    # weak home AP is the likeliest reason to pick Starlink at home in the first place. Counting those
    # ended an at-home pin while GPS said the truck never moved.
    if scan is not None and not (dist is not None and dist <= PIN_HOME_FAR_M):
      missing += 1
    out[low] = (missing, missing >= PIN_HOME_ABSENT_SCANS or (dist is not None and dist > PIN_HOME_FAR_M))
  return out


def home_to_yield_to(stationary_ssids: list[str], scan_ssids: list[str] | None, saved_connections: list[str],
                     unmetered_ssids: set[str] | None, blocked_ssids: set[str] | None, pinned_ssid: str,
                     arrived: set[str] | None = None) -> str:
  """netrank2pnw (Fable D2): the home network a manual pick must give way to, or "" if none.

  A pin used to hold until its network failed or dropped -- so a pick made on the road (the phone, say)
  kept the truck off the home WiFi after arriving home, indefinitely. Proposed to the driver, no veto:
  a pin holds UNTIL a network that is ALL of
    * a configured priority entry that is STATIONARY (not `mobile`),
    * EXPLICITLY UNMETERED (NM `no`, not merely unknown),
    * in this tick's REAL scan (None = no scan ran = no evidence),
    * a saved profile, and not serving a failure backoff (a dead home router must not end the pin),
    * not the pinned network itself,
    * ARRIVED: established genuinely absent since the pick (update_home_arrival), then back,
  is in range. Then the pin ends and the ladder takes that network.

  Mobile entries are NOT exempt from being pinned over, and they do not end a pin: the iPhone is a mobile
  priority entry, and "any up_priority ends the pin" would reopen exactly the measured case of the driver
  picking Starlink while his phone is in range.

  A pick made while the home network is ALREADY visible sticks: `arrived` excludes it until it has been
  genuinely absent. (The first cut of this function implemented the rule literally -- any qualifying home
  in range ended the pin -- which made a manual pick at home revert on the next tick. The driver approved
  the D2 fix for "pick Starlink, drive home, stay on paid Starlink in the driveway", not for that.)
  `arrived` None means no arrival evidence at all, i.e. nothing qualifies."""
  if not scan_ssids:
    return ""
  scan = {x.lower() for x in scan_ssids}
  saved = {ssid_of(c).lower() for c in saved_connections if ssid_of(c)}
  unmetered = {u.lower() for u in (unmetered_ssids or set())}
  blocked = {b.lower() for b in (blocked_ssids or set())}
  pinned = (pinned_ssid or "").lower()
  came = {a.lower() for a in (arrived or set())}
  for ssid in stationary_ssids:
    low = (ssid or "").strip().lower()
    if (low and low != pinned and low in came and low in scan and low in saved and low in unmetered
        and low not in blocked):
      return ssid
  return ""


def on_priority_network(active_ssid: str, configured_ssids: list[str], active_metered: str | None) -> bool:
  """The OnPriorityNetwork param, which the UPLOADER consumes as `at_home` (uploader.effective_metered):
  on a qualifying network, a metered link may still carry drive files.

  netrank2pnw DEFINITION: the active client network is a configured TetheringPriorityNetworks entry
  (case-insensitive) AND its NM connection.metered is not EXPLICITLY `yes`.

  Before: membership alone, exact case. That let a configured network the driver had explicitly marked
  metered authorise 75 MB uploads over it -- the very shape of the 2026-09-10 incident (2,642 MB over
  metered Starlink while it was a configured entry). Under the generic ranking an explicit `metered=yes` is
  the driver's statement that the link costs money, and list membership does not override that for
  uploads any more than it does for ranking. `unknown` still qualifies (unchanged for Visitor-like
  networks), and so does a failed read with nothing cached (`active_metered` None): no evidence of cost
  is not evidence of cost, and that was the previous behaviour.

  Independent of DisableNetworkCostLadder, deliberately: the kill switch reverts RADIO SELECTION. It must
  not re-open metered uploads as a side effect of someone reverting a radio problem."""
  low = (active_ssid or "").strip().lower()
  if not low:
    return False
  if not any((c or "").strip().lower() == low for c in configured_ssids):
    return False
  return (active_metered or "").strip().lower() != "yes"


def pending_for_new_link(active_ssid: str, pending: tuple[str, float] | None,
                        prev_active_ssid: str, now: float) -> tuple[str, float] | None:
  """A client link that APPEARS without us raising it still needs the DHCP grace.

  The driver joining a network from the UI, or NM autoconnecting one at boot, produces an active
  connection the arbiter never asked for. judge_link's no-pending branch blames `usable is False`
  immediately, so if the first tick lands inside that link's DHCP window it was blamed, un-stuck and
  replaced by the hotspot on its very first tick (measured, Fable). Synthesising a pending entry the
  first tick we see it gives it the same three-way judgement -- and the same grace -- as one we
  raised ourselves.

  Pure and separate from the daemon loop on purpose: this exact kind of one-line glue is where four
  earlier defects in this change lived, untested because the loop could not be driven.
  """
  if active_ssid and pending is None and active_ssid.lower() != (prev_active_ssid or "").lower():
    return (active_ssid, now)
  return pending


def judge_link(active_ssid: str | None, usable: bool | None, pending: tuple[str, float] | None,
               now: float, grace_s: float, last_known_usable: bool | None = None) -> LinkVerdict:
  """Decide what the client link's state means: is it sticky, and does anything go in the ledger?

  This is the tick logic that four separate review defects lived in, so it is pure and tested
  directly rather than being glue inside the daemon loop.

  `usable` is TRI-STATE and that is the point:
      True   the link has an IPv4 address
      False  it is associated and has none
      None   WE COULD NOT TELL (nmcli timed out or errored)

  None must never collapse to False. An earlier cut returned a plain bool, so a single flaky nmcli
  call recorded a failure against a perfectly good link, un-stuck it, and -- with the scan empty
  under the geo-gate -- tore it down for the hotspot on the very next tick. That is the parent
  CLAUDE.md's "an ERROR is not a NEGATIVE RESULT" rule, and a test of mine had pinned the wrong
  behaviour as intended.

  The three cases this has to separate, which one signal cannot:

  * A BRING-UP THAT NEVER TOOK. We raised X last tick and nothing is active now -> X failed. This is
    the only way an outright `con up` failure (wrong PSK, AP refusing) is ever seen, because a failed
    activation leaves no active connection to inspect. Judging only the ACTIVE link -- which is what
    the previous cut did after deleting the pending mechanism -- reintroduced exactly this: the
    hotspot came down, the association failed, nothing was recorded, and the same network was chosen
    again forever.
  * SOMEONE ELSE'S CHOICE. We raised X and something else is active -> the driver joined it from the
    UI. Drop the pending judgement and blame nobody; X did not fail, it was overruled.
  * SLOW DHCP. Associated to the network we raised, no address YET. NM's DHCP timeout is 45 s and the
    poll is 20 s, so the first look can legally land mid-activation. Inside `grace_s` this is not a
    failure and the link stays sticky -- tearing it down here was measured to drop a link that would
    have completed a few seconds later.

  `active_ssid` None means the active connection COULD NOT BE READ (arbiterfu2pnw; judge_pin's convention).
  "" is a real reading: nothing is active. Only a pending bring-up treats them differently -- see below.
  """
  act = (active_ssid or "").strip()

  if pending is not None:
    ssid_p, raised_at = pending
    if active_ssid is None:
      # arbiterfu2pnw (Fable, unreadhold2pnw follow-up): AN UNREADABLE READ IS NOT A BRING-UP THAT NEVER TOOK. A failed
      # `con show --active` read as "nothing active" here, so the tick after a `con up` blamed the network and started
      # its backoff on no evidence; a link still in DHCP was then re-seen as new, its grace restarted, and it was
      # blamed twice. Same rule as `usable is None` below: keep waiting, grace still counted from `raised_at`, and
      # the next GOOD read judges it. The caller bounds how long (ACTIVE_UNREADABLE_HOLD_S) by passing "" after it.
      return LinkVerdict(ssid_p, "", False, pending)
    if act.lower() != ssid_p.lower():
      if not act:
        return LinkVerdict("", ssid_p, False, None)      # the bring-up never took -> blame it
      # OVERRULED: the driver joined something else from the UI. Blame nobody -- and hand the
      # newcomer the same grace a link we raised would get, by making IT the pending one. Without
      # that, a manual join landing mid-DHCP read as an unusable non-sticky link and the hotspot
      # took the radio out from under it (measured, Fable): pending_for_new_link had already run
      # this tick with the OLD pending set, so the newcomer never got a synthesised one.
      if usable is True:
        return LinkVerdict(act, "", False, None)
      return LinkVerdict(act, "", False, (act, now))
    if usable is True:
      return LinkVerdict(ssid_p, ssid_p, True, None)
    if usable is None:
      # STICKY UNCONDITIONALLY, and deliberately NOT consulting last_known_usable. A link we raised
      # ourselves is inside its own judgement window: the grace/False path below still catches one
      # that is genuinely dead, so nothing is lost by holding on through a tick we could not read.
      # Consulting the cache here was a measured defect (Fable): a link doing DHCP legitimately reads
      # False, that False is cached, and then ONE unreadable tick inside the grace window made it
      # non-sticky -- with the scan suppressed by the geo-gate the hotspot took the radio, and the
      # next tick blamed the target for "never taking". A stale False from an earlier visit to the
      # same network did the same thing, because the cache was never invalidated on a bring-up.
      return LinkVerdict(ssid_p, "", False, pending)     # no new information -- keep waiting
    if now - raised_at < grace_s:
      return LinkVerdict(ssid_p, "", False, pending)     # still inside DHCP's window
    return LinkVerdict("", ssid_p, False, None)

  if not act:
    return LinkVerdict("", "", False, None)

  if usable is True:
    return LinkVerdict(act, act, True, None)
  if usable is None:
    return LinkVerdict(act if last_known_usable is not False else "", "", False, None)
  return LinkVerdict("", act, False, None)


# netrank2pnw: cost CLASS dominates, for every saved network. Lower is cheaper.
COST_UNMETERED = 0   # NM connection.metered = no   -- the driver (or the OS) asserted it is free
COST_UNKNOWN = 1     # NM connection.metered = unknown -- the default; nobody ever said
COST_METERED = 2     # NM connection.metered = yes  -- asserted expensive


def cost_class(ssid: str, metered_ssids: set[str] | None, unmetered_ssids: set[str] | None) -> int:
  """The cost class of one SSID from NM's three-state connection.metered. Case-insensitive. An SSID
  in BOTH sets (impossible from NM) is treated as metered -- the conservative reading."""
  low = (ssid or "").lower()
  if low in {m.lower() for m in (metered_ssids or set())}:
    return COST_METERED
  if low in {u.lower() for u in (unmetered_ssids or set())}:
    return COST_UNMETERED
  return COST_UNKNOWN


def choose_wifi(priority_ssids: str | list[str], scan_ssids: list[str], saved_connections: list[str],
                metered_ssids: set[str] | None = None, active_ssid: str = "",
                blocked_ssids: set[str] | None = None,
                unmetered_ssids: set[str] | None = None) -> tuple[int, str] | None:
  """Pick the cheapest usable WiFi. Returns (cost_class, ssid) or None if no saved wifi is usable.

  netrank2pnw -- THE DRIVER'S GENERIC RULE (2026-09-13, verbatim): "we don't want a solution where it
  just scans the Starlink SSIDs that I have configured; we need a generic solution where an unmetered
  network is always prioritized over a metered network or a default setting ... if I clearly have an
  unmetered network and the other network is set to either metered or default, then I made a conscious
  choice that this network is a priority if it's available."

  So the sort key, for EVERY saved client profile in range, is:

      1. cost class       COST_UNMETERED (no) < COST_UNKNOWN (default) < COST_METERED (yes)
      2. priority member  a configured TetheringPriorityNetworks entry beats a non-member, and members
                          keep the driver's own list order among themselves
      3. ssid             so equal networks resolve STABLY and never flap

  Cost dominates membership. This REVERSES netcosttier2pnw, where a configured entry was tier 0 unless
  explicitly metered -- i.e. "unknown does not demote a priority entry". Under that rule an unknown
  (default) configured network outranked a network the driver had explicitly marked unmetered, which is
  exactly what he says is wrong: marking a network unmetered IS his conscious choice. Membership still
  matters, but only between networks that cost the same.

  UNKNOWN IS NOT UNMETERED (kept from netcosttier2pnw): measured on the 3X, most saved profiles carry
  `unknown`. Folding it into unmetered once let an alphabetical tiebreak pick between the driver's
  metered Starlink and his unmetered iPhone. Absence of evidence ranks as absence of evidence -- after
  everything known-cheap, before everything known-expensive.

  `priority_ssids` -- the configured entries in list order (a bare str is accepted as a one-entry list).
  Membership only; reachability is decided here like any other network: a saved profile that is in the
  scan (or is the active link).

  `active_ssid` -- the USABLE-active SSID (see judge_link). Seeded into the candidates even when this
  tick's scan does not list it, and never blocked: the geo-gate suppresses scanning on client WiFi, and
  without this the arbiter tore down a working link every tick away from home. It competes at its TRUE
  cost, so anything cheaper that appears in a scan still wins.

  `blocked_ssids` -- SSIDs serving an association-failure backoff; excluded so a network that is in
  range but will not come up cannot hold the device offline.

  The returned ssid is the SAVED PROFILE's own spelling (ssid_of), never the configured entry's -- `nmcli
  con up` matches connection ids case-sensitively, so bringing up "openpilot connection visitor" when the
  profile is "openpilot connection Visitor" would fail.
  """
  members = [priority_ssids] if isinstance(priority_ssids, str) else list(priority_ssids or [])
  member_rank: dict[str, int] = {}
  for i, m in enumerate(members):
    low = (m or "").strip().lower()
    if low and low not in member_rank:
      member_rank[low] = i
  blocked = {b.lower() for b in (blocked_ssids or set())}
  scan = {x.lower() for x in scan_ssids}
  active_ssid = (active_ssid or "").strip()
  if active_ssid:
    scan.add(active_ssid.lower())          # sticky: associated-and-usable beats "not in this tick's scan"
    blocked.discard(active_ssid.lower())   # ...and a VERIFIED-usable link is never in backoff

  candidates = []
  for conn in saved_connections:
    ssid = ssid_of(conn)
    low = ssid.lower()
    if not ssid or low not in scan or low in blocked:
      continue
    candidates.append((cost_class(ssid, metered_ssids, unmetered_ssids),
                       member_rank.get(low, len(members)), low, ssid))
  if not candidates:
    return None
  candidates.sort()
  klass, _member, _low, ssid = candidates[0]
  return (klass, ssid)


_COST_NAMES = {COST_UNMETERED: "unmetered", COST_UNKNOWN: "cost unknown", COST_METERED: "metered"}


def explain_fallback(priority_ssids: list[str], scan_ssids: list[str], saved_connections: list[str], chosen_ssid: str,
                     metered_ssids: set[str] | None = None, unmetered_ssids: set[str] | None = None,
                     blocked_ssids: set[str] | None = None, active_ssid: str = "", scanned: bool = True) -> str:
  """arbiterfu2pnw: why decide() joined a NON-member (`up_fallback`), for its log line. TEXT ONLY -- no decision reads it.

  The line said "no priority network in range", which netrank2pnw made false: a configured network can be in range
  and lose on cost (an explicitly unmetered non-member outranks a default or metered member), or be excluded by a
  failure backoff or a missing saved profile. Takes choose_wifi's own inputs and mirrors its eligibility rules, and
  says for each configured network which rule excluded it. `scanned` False = no scan ran this tick (geo-gate)."""
  chosen_cost = cost_class(chosen_ssid, metered_ssids, unmetered_ssids)
  members = [m.strip() for m in priority_ssids if m and m.strip()]
  if not members:
    return f"{_COST_NAMES[chosen_cost]} -- no priority networks configured"
  active = (active_ssid or "").strip().lower()
  scan = {x.lower() for x in scan_ssids} | ({active} if active else set())
  blocked = {b.lower() for b in (blocked_ssids or set())} - {active}
  saved = {ssid_of(c).lower() for c in saved_connections if ssid_of(c)}
  parts = []
  for m in members:
    low = m.lower()
    cost = cost_class(m, metered_ssids, unmetered_ssids)
    if low not in scan:
      why = "not in the scan" if scanned else "not seen, no scan ran this tick"
    elif low not in saved:
      why = "in range, no saved profile"
    elif low in blocked:
      why = "in range, in failure backoff"
    elif cost > chosen_cost:
      why = f"in range, {_COST_NAMES[cost]}, outranked"
    else:   # cannot happen from decide()'s own inputs -- a member wins a tie -- so say so rather than invent a reason
      why = f"in range, {_COST_NAMES[cost]}, NOT outranked: explanation disagrees with the ranking"
    parts.append(f"'{m}': {why}")
  return f"{_COST_NAMES[chosen_cost]}, not a configured priority network -- " + "; ".join(parts)


def decide(
  tethering_enabled: bool,
  priority_ssid: str,
  scan_ssids: list[str],
  saved_connections: list[str],
  current_active: str | None,
  metered_ssids: set[str] | None = None,
  fallback_enabled: bool = False,
  blocked_ssids: set[str] | None = None,
  unmetered_ssids: set[str] | None = None,
  active_ssid: str | None = None,
  priority_ssids: list[str] | None = None,
) -> tuple[str, str]:
  """
  Decide the single nmcli action to take this tick, and which SSID it applies to.

  Args:
    tethering_enabled: value of the TetheringEnabled param.
    priority_ssid: the FIRST reachable configured entry (priority_networks.select_available). Used by
                   the binary mode only.
    scan_ssids: SSIDs currently visible to NM.
    saved_connections: NM connection ids that exist.
    current_active: NM connection id currently active on wlan0 (the Hotspot, an "openpilot connection
                    <ssid>", or None). Idempotence: we never re-`up` what is already active.
    metered_ssids / unmetered_ssids: SSIDs whose saved profile reports connection.metered = yes / no.
    fallback_enabled: the cost ladder. False = DisableNetworkCostLadder = the pre-ladder binary arbiter.
    blocked_ssids: SSIDs serving an association-failure backoff; never offered as a choice.
    active_ssid: the usable-active SSID (judge_link); None derives it from current_active.
    priority_ssids: every configured entry, in the driver's list order (netrank2pnw). Membership is a
                    tiebreak inside a cost class. None falls back to [priority_ssid].

  Returns (action, ssid). `ssid` is "" for the hotspot actions and for noop-on-hotspot.

  THE LADDER (fallback_enabled): every saved network in range is ranked by choose_wifi -- cost class
  first, configured membership second. The action is `up_priority` when the winner is a configured entry
  and `up_fallback` otherwise; that name is about MEMBERSHIP, not cost (both bring the network up the same
  way). Nothing usable -> our own hotspot + LTE, the last resort.

  THE BINARY MODE (not fallback_enabled): the arbiter as it was before the ladder existed -- the first
  reachable configured entry, or the hotspot, and nothing in between. It has no notion of cost at all,
  deliberately: that is what the kill switch is for. The ONE retention is the failure ledger (a blocked
  entry is not reachable), because without it a dead-but-in-range router holds the device offline forever,
  and a kill switch that can strand the device is not a safe kill switch.
  """
  # --- Tethering OFF: only ever ensure the hotspot is DOWN. Never touch client wifi. ---
  if not tethering_enabled:
    if current_active == HOTSPOT_CONNECTION_ID:
      return ("down_hotspot", "")
    return ("noop", "")

  if active_ssid is None:
    active_ssid = ssid_of(current_active or "")
  blocked_low = {b.lower() for b in (blocked_ssids or set())}
  if active_ssid:
    blocked_low.discard(active_ssid.lower())

  if fallback_enabled:
    members = priority_ssids if priority_ssids is not None else ([priority_ssid] if priority_ssid else [])
    choice = choose_wifi(members, scan_ssids, saved_connections, metered_ssids,
                         active_ssid=active_ssid, blocked_ssids=blocked_ssids,
                         unmetered_ssids=unmetered_ssids)
    if choice is not None:
      _klass, ssid = choice
      if current_active == priority_connection_id(ssid):
        return ("noop", ssid)                    # already on it -- never re-`up` (idempotence)
      is_member = ssid.lower() in {(m or "").strip().lower() for m in members}
      return ("up_priority" if is_member else "up_fallback", ssid)
  else:
    # binary: the first reachable configured entry (the caller's select_available, which is in list order)
    priority_ssid = (priority_ssid or "").strip()
    if priority_ssid and priority_ssid.lower() not in blocked_low:
      in_range = priority_ssid.lower() in {x.lower() for x in scan_ssids} or \
        priority_ssid.lower() == (active_ssid or "").lower()
      conn = next((c for c in saved_connections
                   if c.lower() == priority_connection_id(priority_ssid).lower()), None)
      if in_range and conn is not None:
        ssid = ssid_of(conn)
        if current_active == conn:
          return ("noop", ssid)
        return ("up_priority", ssid)

  # Nothing usable -> our own hotspot + LTE.
  if current_active == HOTSPOT_CONNECTION_ID:
    return ("noop", "")
  return ("up_hotspot", "")
