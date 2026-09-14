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

The ladder, cheapest first (driver's own numbering):

    tier 0   a configured PRIORITY network            (stationary, geo-gated; always unmetered)
    tier 1   any other saved wifi in range, UNMETERED (e.g. the driver's iPhone hotspot)
    tier 2   any other saved wifi in range, METERED   (e.g. mobile Starlink)
    tier 3   the comma's OWN hotspot + LTE            LAST RESORT

Two things this deliberately does NOT do:
  * it does not rank by list position. The old priority list was ordered, so a phone-hotspot entry
    could outrank a cheaper stationary network purely by sitting earlier in the JSON.
  * it does not rank by the `mobile` flag. Mobile says "this network travels with the car", which is
    about geofence learning, NOT about cost -- the driver's iPhone hotspot is mobile AND unmetered,
    while his Starlink is stationary-ish AND metered. COST is the axis; `metered` is how the OS
    reports it, and it self-maintains: mark a network metered and it demotes with no list to edit.
"""
from __future__ import annotations

from typing import NamedTuple

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
UPGRADE_SCAN_S = 120.0     # at most one cost-upgrade scan per this, on a non-tier-0 client link


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
              blamed_failed: bool, now: float, join_window_s: float = PIN_JOIN_WINDOW_S) -> PinVerdict:
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
  if active_ssid.lower() == pick_ssid.lower():
    return PinVerdict(pick_ssid, True, "")
  if seen_active:
    return PinVerdict("", True, "dropped")
  if now - first_seen >= join_window_s:
    return PinVerdict("", False, "join_timeout")
  return PinVerdict(pick_ssid, False, "")


def upgrade_scan_due(ladder_on: bool, on_client_wifi: bool, on_tier0: bool, pinned: bool,
                     now: float, last_scan: float, interval_s: float = UPGRADE_SCAN_S) -> bool:
  """Should the arbiter scan for a CHEAPER network although the geo-gate would not?

  The geo-gate stops scanning once we are on client WiFi away from a learned location. That rule was
  written when the only client WiFi the arbiter could be on was a PRIORITY network, i.e. already the
  cheapest. The cost ladder broke the assumption: on a tier-1/2 link -- the driver's metered Starlink --
  a cheaper network can come into range and can only be SEEN by scanning. Measured 2026-09-13: 66 min on
  metered KarlMoik with the unmetered iPhone never considered, and NM 1.46 does not rescan on its own
  while associated (LastScan unchanged across 75 s; the --rescan no cache pruned to the current AP).

  Not on tier 0 (nothing to gain), not while a manual pick holds the radio (the driver chose), not with
  the ladder disabled (kill switch = pre-ladder behaviour), and at most once per interval, because a scan
  takes the single radio off-channel. The gate's own worry -- disturbing tethered clients -- does not
  arise on a client link: the comma's hotspot is down whenever wlan0 is a client."""
  return (ladder_on and on_client_wifi and not on_tier0 and not pinned
          and now - last_scan >= interval_s)


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


def judge_link(active_ssid: str, usable: bool | None, pending: tuple[str, float] | None,
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
  """
  act = (active_ssid or "").strip()

  if pending is not None:
    ssid_p, raised_at = pending
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


def choose_wifi(priority_ssid: str, scan_ssids: list[str], saved_connections: list[str],
                metered_ssids: set[str] | None = None, active_ssid: str = "",
                blocked_ssids: set[str] | None = None,
                unmetered_ssids: set[str] | None = None) -> tuple[int, str] | None:
  """Pick the cheapest usable WiFi. Returns (tier, ssid) or None if no saved wifi is usable.

  netcosttier2pnw. PURE. The cost signal is NetworkManager's `connection.metered` on the saved
  profile, which has THREE values, not two:

      yes       somebody asserted this network costs money      -> tier 2
      no        somebody asserted it does not                   -> tier 1
      unknown   NOBODY EVER SAID                                -> between them (see below)

  UNKNOWN IS NOT THE SAME AS UNMETERED, and the first cut of this file treated them as identical.
  That was measured wrong on the truck 2026-09-10: of the four saved client profiles, only the
  iPhone and the home WiFi carry an explicit `no`; the driver's mobile Starlink ("KarlMoik") and the
  "Visitor" network are both `unknown`. Folding unknown into tier 1 put the Starlink -- the network
  the driver NAMED as the metered one -- level with his unmetered iPhone, and the tie then fell to
  the alphabetical tiebreak. The iPhone won by starting with a "D". Rename the phone and the ladder
  silently inverts. A cost ladder whose decisive input is a coin flip is the Rule-2 failure mode
  exactly: it looks like it is working.

  So `unknown` sorts BETWEEN the two assertions -- after everything known-cheap, before anything
  known-expensive. Absence of evidence orders as absence of evidence, and no reading of an unset
  field is invented in either direction. It stays inside the driver's tier 1 (it is still ordinary
  WiFi, and still beats tier 3, our own LTE, which is the expensive thing this ladder exists to
  avoid) -- it just never outranks a network somebody actually vouched for.

  `active_ssid` (Fable review) -- the SSID we are ASSOCIATED WITH right now. Seeded into the
  candidate set even when this tick's scan does not list it, and never blocked. NM's association
  state, not a scan, is the truth for "am I on this network": the geo-gate deliberately suppresses
  scanning once we are on client WiFi, so without this the candidate set goes empty every tick away
  from home and the arbiter tears down a working link to raise its own hotspot. Seeded at its TRUE
  cost (not tier 0), so anything genuinely cheaper that does appear still wins.

  `blocked_ssids` (Fable review) -- SSIDs serving an association-failure backoff. Excluded so a
  network that is in range but will not associate cannot hold the device offline forever. The ACTIVE
  ssid is never excluded: it is demonstrably working.

  Case-insensitive throughout, matching select_available()'s reasoning: users type "visitor" while
  the AP advertises "Visitor".
  """
  metered = {s.lower() for s in (metered_ssids or set())}
  unmetered = {s.lower() for s in (unmetered_ssids or set())}
  blocked = {s.lower() for s in (blocked_ssids or set())}
  scan = {s.lower() for s in scan_ssids}

  active_ssid = (active_ssid or "").strip()
  if active_ssid:
    scan.add(active_ssid.lower())          # sticky: associated beats "not in this tick's scan"
    blocked.discard(active_ssid.lower())   # ...and a VERIFIED-usable link is never in backoff

  # tier 0 -- the configured priority network. Its availability is decided by the caller
  # (priority_networks.select_available), which already applied the geo-gate and saved-connection check.
  #
  # EXCEPT when it is known METERED. The driver's own definition is "tier 0 is priority network,
  # those are always unmetered" -- so a configured entry NM reports as metered is a contradiction, and
  # resolving it by list position is the ranking this change exists to reject (his mobile Starlink
  # outranking his unmetered iPhone purely by sitting earlier in the JSON). A metered priority entry
  # is DEMOTED into the ladder below and competes at tier 2 -- still far ahead of tier 3, our own LTE.
  # `unknown` does NOT demote a priority entry: the driver curated that list by hand, which is a
  # stronger statement about it than an unset NM field is.
  priority_ssid = (priority_ssid or "").strip()
  if priority_ssid and priority_ssid.lower() not in metered and priority_ssid.lower() not in blocked:
    return (0, priority_ssid)

  # tiers 1 and 2 -- any OTHER saved client wifi in range (plus a demoted priority entry, which
  # reaches here through its own saved connection). `rank` is the sort key; `tier` is the driver's
  # own numbering, reported out for the log so a decision can be read back in his vocabulary.
  candidates = []
  for conn in saved_connections:
    ssid = ssid_of(conn)
    low = ssid.lower()
    if not ssid or low not in scan or low in blocked:
      continue
    if low in metered:
      rank, tier = 2, 2          # asserted expensive
    elif low in unmetered:
      rank, tier = 0, 1          # asserted cheap
    else:
      rank, tier = 1, 1          # nobody said -- between the two assertions
    candidates.append((rank, ssid.lower(), tier, ssid))
  if not candidates:
    return None
  # cheapest rank wins; ties broken by ssid so the choice is STABLE -- an unstable pick would drop and
  # re-raise the radio every cycle between two genuinely equal-cost networks.
  candidates.sort()
  _rank, _low, tier, ssid = candidates[0]
  return (tier, ssid)


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
) -> tuple[str, str]:
  """
  Decide the single nmcli action to take this tick, and which SSID it applies to.

  Args:
    tethering_enabled: value of the TetheringEnabled param.
    priority_ssid: value of the TetheringPriorityWifi param (blank if unset). Only THIS ssid may
                   interrupt the hotspot.
    scan_ssids: SSIDs currently visible to NM (`nmcli -t -f SSID dev wifi list`). NM scans even
                while in AP mode.
    saved_connections: NM connection ids that exist (`nmcli -t -f NAME con show`). Used to confirm
                       a saved client connection exists for the priority SSID before trying to
                       bring it up.
    current_active: NM connection id that is currently active on wlan0 (the Hotspot, an
                    "openpilot connection <ssid>", or None). Used to stay idempotent — we never
                    re-`up` what is already active — and, netcosttier2pnw2, as the STICKY signal:
                    whatever we are associated with stays a candidate even when this tick's scan
                    does not list it.
    metered_ssids: SSIDs whose saved profile reports connection.metered = yes (the cost signal).
    fallback_enabled: opt-in for tiers 1 and 2. Off = the old binary tier-0-or-hotspot behaviour.
    blocked_ssids: SSIDs serving an association-failure backoff; never offered as a choice.

  Returns (action, ssid). `ssid` is the network the action applies to, and is "" for the hotspot
  actions and for noop. Returning it here rather than recomputing it in the caller is deliberate:
  the daemon previously ran choose_wifi a SECOND time to work out what to bring up, and two
  independently-argued calls can disagree -- which would `nmcli con up` a different network than the
  one the ladder actually ranked.
  """
  # --- Tethering OFF: only ever ensure the hotspot is DOWN. Never touch client wifi. ---
  if not tethering_enabled:
    if current_active == HOTSPOT_CONNECTION_ID:
      return ("down_hotspot", "")
    return ("noop", "")

  # --- Tethering ON ---
  # netcosttier2pnw2: the USABLE-active network is sticky (see choose_wifi). The caller supplies it,
  # because only the caller can tell "associated" from "carrying traffic" -- that needs an nmcli read
  # and this function is pure. Passing None falls back to deriving it from current_active, which is
  # what the pre-existing callers (and the tests) do; the daemon passes "" when the active link is
  # associated but has no IPv4 address, so a dead link is neither sticky nor exempt from its backoff.
  if active_ssid is None:
    active_ssid = ssid_of(current_active or "")

  priority_ssid = (priority_ssid or "").strip()
  priority_id = priority_connection_id(priority_ssid) if priority_ssid else None

  # A named priority SSID is a TIER-0 CANDIDATE when it is reachable (in range, or the one we are
  # already on) and has a saved connection. Whether it actually WINS is choose_wifi's decision, not
  # ours -- it is the only place that knows about cost and about the failure ledger. Deciding it
  # here as well is how the first cut ended up with the demotion rule written twice, in two functions
  # that could drift apart; choose_wifi's copy was then dead code, because this function only ever
  # called it with an empty priority_ssid.
  # Case-INSENSITIVE, matching select_available and entry_for_ssid. A configured "visitor" against
  # an AP advertising "Visitor" was selected as tier 0 by select_available and then rejected here,
  # so it connected as an ordinary tier-1-unknown network and lost to anything explicitly unmetered
  # (Fable review). The Peak 'Visitor' SSID is a real network in the driver's list and has already
  # cost a captive-portal bug for exactly this reason.
  scan_low = {x.lower() for x in scan_ssids}
  saved_low = {c.lower() for c in saved_connections}
  reachable = bool(
    priority_ssid
    and (priority_ssid.lower() in scan_low or priority_ssid.lower() == active_ssid.lower())
    and priority_id and priority_id.lower() in saved_low
  )

  choice = choose_wifi(priority_ssid if reachable else "", scan_ssids, saved_connections,
                       metered_ssids, active_ssid=active_ssid, blocked_ssids=blocked_ssids,
                       unmetered_ssids=unmetered_ssids)

  if choice is not None:
    tier, ssid = choice
    # `fallback_enabled` gates tiers 1 and 2 ONLY, so the pre-existing callers (and the deployed
    # default, until the daemon turns it on) keep the exact old binary semantics: tier 0, or the
    # hotspot, and nothing in between.
    if tier == 0 or fallback_enabled:
      conn_id = priority_connection_id(ssid)
      if current_active == conn_id:
        return ("noop", ssid)                    # already on it -- never re-`up` (idempotence)
      return ("up_priority" if tier == 0 else "up_fallback", ssid)

  # Nothing cheaper is usable -> tier 3, our own hotspot + LTE.
  if current_active == HOTSPOT_CONNECTION_ID:
    return ("noop", "")
  return ("up_hotspot", "")
