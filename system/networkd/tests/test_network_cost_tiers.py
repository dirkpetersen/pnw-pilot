"""netcosttier2pnw — the network COST LADDER: never fall to our own LTE while cheaper WiFi is in range.

THE DEFECT, reported by the driver 2026-09-10 and confirmed in the code: `decide()` was BINARY. A
configured priority SSID won the radio; anything else raised the comma's own hotspot. There was no
tier in between, so the device went from tier 0 straight to tier 3 — its own LTE, the most expensive
link it has — while saved, in-range, cheaper WiFi sat there unused.

Driver, verbatim: "the tethering network should be the lowest priority if another wifi connection is
available because the tethering network is the most expensive network".

THE LADDER (the driver's own numbering):

    tier 0   configured PRIORITY network            stationary, geo-gated, always unmetered
    tier 1   other saved wifi in range, UNMETERED   his iPhone hotspot
    tier 2   other saved wifi in range, METERED     his mobile Starlink ("KarlMoik")
    tier 3   the comma's own hotspot + LTE          LAST RESORT

netrank2pnw (2026-09-13) SUPERSEDED THE TIER NUMBERING ABOVE. Cost class now dominates membership for every
saved network -- explicitly unmetered < default/unknown < explicitly metered -- and a configured entry is
only a tiebreak inside its class. choose_wifi returns the COST CLASS (COST_UNMETERED/UNKNOWN/METERED), not
a tier, and decide()'s up_priority/up_fallback name reports MEMBERSHIP, not cost. Tests below that
changed say why in their docstrings.

TWO RANKINGS THAT WERE REJECTED, and the tests that pin them out:
  * by LIST POSITION — the old priority list is ordered, so a phone-hotspot entry could outrank a
    cheaper network purely by sitting earlier in the JSON.
  * by the `mobile` FLAG — mobile means "travels with the car" (a geofence-learning concern), not
    "expensive". The driver's iPhone hotspot is mobile AND unmetered; his Starlink is metered. Cost
    is the axis, and `metered` is how the OS reports it.
"""
import pytest

from openpilot.system.networkd.network_arbiter import (COST_UNKNOWN, COST_UNMETERED, HOTSPOT_CONNECTION_ID, choose_wifi, decide,
                                                       explain_fallback, priority_connection_id, ssid_of)


def act(*a, **k):
  """decide() returns (action, ssid); most assertions here only care about the action."""
  return decide(*a, **k)[0]

HOME, PHONE, STARLINK = "Hannelore", "Dirk's iPhone 13", "KarlMoik"
ID_HOME, ID_PHONE, ID_STAR = (priority_connection_id(s) for s in (HOME, PHONE, STARLINK))
ALL_SAVED = [HOTSPOT_CONNECTION_ID, ID_HOME, ID_PHONE, ID_STAR, "lte"]


class TestSsidOf:
  def test_it_extracts_our_own_saved_client_profiles(self):
    assert ssid_of(ID_STAR) == STARLINK

  @pytest.mark.parametrize("other", [HOTSPOT_CONNECTION_ID, "lte", "", "some hand-made profile"])
  def test_it_refuses_anything_that_is_not_ours(self, other):
    """The Hotspot and the LTE profile must never be mistaken for candidate client WiFi — picking
    either would be the arbiter 'falling back' to the thing it is trying to avoid."""
    assert ssid_of(other) == ""


class TestTheLadder:
  def test_a_priority_member_wins_WITHIN_its_cost_class(self):
    """netrank2pnw (was test_tier0_priority_beats_everything): membership no longer beats everything,
    it breaks ties inside a cost class. HOME and PHONE are both `unknown` here, so the configured HOME
    wins; the returned class is COST_UNKNOWN, not the old tier 0."""
    assert choose_wifi(HOME, [HOME, PHONE, STARLINK], ALL_SAVED, {STARLINK}) == (COST_UNKNOWN, HOME)

  def test_an_explicitly_UNMETERED_network_beats_a_DEFAULT_priority_member(self):
    """THE DRIVER'S RULE, netrank2pnw, verbatim: "if I clearly have an unmetered network and the other
    network is set to either metered or default, then I made a conscious choice that this network is a
    priority if it's available." HOME is configured but `unknown`; PHONE is not configured but `no`."""
    assert choose_wifi(HOME, [HOME, PHONE], ALL_SAVED, set(), unmetered_ssids={PHONE}) == (COST_UNMETERED, PHONE)

  def test_tier1_unmetered_beats_tier2_metered(self):
    """THE CASE THE DRIVER DESCRIBED. No priority network in range; his unmetered iPhone and his
    metered Starlink both are. The iPhone must win."""
    assert choose_wifi("", [PHONE, STARLINK], ALL_SAVED, {STARLINK}) == (1, PHONE)

  def test_tier2_metered_wifi_is_still_chosen_over_nothing(self):
    """Metered WiFi is tier 2, NOT excluded — it is still far cheaper than our own LTE. This is the
    'KarlMoik is metered but cheaper than tethering' case, verbatim from the driver."""
    assert choose_wifi("", [STARLINK], ALL_SAVED, {STARLINK}) == (2, STARLINK)

  def test_nothing_in_range_yields_no_choice(self):
    assert choose_wifi("", [], ALL_SAVED, set()) is None

  def test_a_saved_network_that_is_not_in_range_is_not_chosen(self):
    assert choose_wifi("", ["SomeoneElsesWifi"], ALL_SAVED, set()) is None

  def test_unknown_metered_state_is_NOT_treated_as_metered(self):
    """netrank2pnw rename (the old name said "counts as unmetered", which stopped being true when
    unknown got its own class): NM reports `unknown` for a profile nobody marked. It is its own class,
    between the two assertions -- never folded into metered, and never into unmetered."""
    assert choose_wifi("", [PHONE], ALL_SAVED, set()) == (COST_UNKNOWN, PHONE)

  def test_the_choice_is_stable_between_equal_cost_networks(self):
    """An unstable pick would drop and re-raise the radio every cycle between two equal-cost
    networks -- worse than either choice.

    The first version of this test called choose_wifi five times with IDENTICAL arguments and
    asserted one distinct answer. choose_wifi is a pure deterministic function, so that could not
    fail for any implementation whatsoever -- it was vacuous (Fable review). Stability that MEANS
    something is invariance under the orderings the caller cannot control: nmcli returns saved
    profiles and scan results in whatever order it likes, and that order must not decide."""
    import itertools
    answers = set()
    for scan in itertools.permutations([PHONE, HOME, STARLINK]):
      for saved in itertools.permutations([ID_PHONE, ID_HOME, ID_STAR]):
        answers.add(choose_wifi("", list(scan), list(saved), set(), unmetered_ssids={PHONE, HOME}))
    assert len(answers) == 1, f"the pick depends on nmcli's ordering: {answers}"

  def test_ranking_is_by_cost_not_by_list_position(self):
    """REJECTED RANKING #1. Reverse the saved order and the answer must not move."""
    fwd = choose_wifi("", [PHONE, STARLINK], [ID_PHONE, ID_STAR], {STARLINK})
    rev = choose_wifi("", [STARLINK, PHONE], [ID_STAR, ID_PHONE], {STARLINK})
    assert fwd == rev == (1, PHONE)

  def test_case_insensitive_like_the_rest_of_the_module(self):
    assert choose_wifi("", ["karlmoik"], [priority_connection_id("KarlMoik")], {"KARLMOIK"}) == (2, "KarlMoik")


class TestDecideUsesTheLadder:
  def test_it_no_longer_jumps_from_tier0_to_tier3(self):
    """THE REGRESSION THIS FIXES. No priority network, but the iPhone is in range and saved: the old
    code returned 'up_hotspot' (our own LTE). It must now take the WiFi."""
    action = decide(True, "", [PHONE], ALL_SAVED, HOTSPOT_CONNECTION_ID,
                 metered_ssids=set(), fallback_enabled=True)
    assert action[0] == "up_fallback"

  def test_metered_wifi_still_beats_our_own_lte(self):
    action = decide(True, "", [STARLINK], ALL_SAVED, HOTSPOT_CONNECTION_ID,
                 metered_ssids={STARLINK}, fallback_enabled=True)
    assert action[0] == "up_fallback"

  def test_hotspot_is_still_the_last_resort_when_nothing_is_in_range(self):
    action = decide(True, "", [], ALL_SAVED, None, metered_ssids=set(), fallback_enabled=True)
    assert action[0] == "up_hotspot"

  def test_it_is_idempotent_once_connected(self):
    """Never re-`up` what is already active — a re-up drops and re-raises the radio."""
    action = decide(True, "", [PHONE], ALL_SAVED, ID_PHONE, metered_ssids=set(), fallback_enabled=True)
    assert action[0] == "noop"

  def test_priority_still_outranks_the_fallback(self):
    action = decide(True, HOME, [HOME, PHONE], ALL_SAVED, HOTSPOT_CONNECTION_ID,
                 metered_ssids=set(), fallback_enabled=True)
    assert action[0] == "up_priority"

  def test_tethering_off_still_never_touches_client_wifi(self):
    """The oldest invariant in this module: with tethering off we only ever ensure the AP is down.
    The new tier must not create a path that grabs the radio in that state."""
    assert act(False, "", [PHONE, STARLINK], ALL_SAVED, HOTSPOT_CONNECTION_ID,
                  metered_ssids=set(), fallback_enabled=True) == "down_hotspot"
    assert act(False, "", [PHONE, STARLINK], ALL_SAVED, None,
                  metered_ssids=set(), fallback_enabled=True) == "noop"

  def test_the_old_behaviour_is_preserved_when_the_flag_is_off(self):
    """16 pre-existing callers pass 5 positional args. They must keep the exact old semantics."""
    assert act(True, "", [PHONE], ALL_SAVED, HOTSPOT_CONNECTION_ID) == "noop"
    assert act(True, "", [PHONE], ALL_SAVED, None) == "up_hotspot"

  def test_the_hotspot_profile_can_never_be_chosen_as_a_fallback(self):
    """If `Hotspot` or `lte` were ever treated as a candidate client network, the arbiter would
    'fall back' to precisely the tier-3 link the ladder exists to avoid — while reporting tier 1."""
    action = decide(True, "", ["Hotspot", "lte"], ALL_SAVED, None,
                 metered_ssids=set(), fallback_enabled=True)
    assert action[0] == "up_hotspot"


class TestUnknownIsNotUnmetered:
  """THE THIRD DEFECT, found by the driver's own question ("what's the difference between unknown
  and unmetered?") and not by either reviewer.

  Measured on the 3X 2026-09-10, `nmcli -t -f connection.metered con show <profile>`:

      openpilot connection Dirk's iPhone 13   no
      openpilot connection Hannelore          no
      openpilot connection KarlMoik           unknown      <-- the driver's METERED Starlink
      openpilot connection Visitor            unknown

  The first cut folded `unknown` into unmetered. That put KarlMoik level with the unmetered iPhone at
  tier 1, and the tie fell through to the alphabetical tiebreak -- the iPhone won because it starts
  with a "D". A ladder whose decisive input is the driver's choice of phone name is not a ladder."""

  def test_an_asserted_unmetered_network_beats_an_unknown_one(self):
    assert choose_wifi("", [PHONE, STARLINK], ALL_SAVED, set(), unmetered_ssids={PHONE}) == (COST_UNMETERED, PHONE)

  def test_and_it_still_wins_when_the_alphabet_is_against_it(self):
    """THE REGRESSION TEST. Rename the phone so it sorts AFTER the Starlink; the answer must not
    move. Under the old unknown==unmetered rule this returned KarlMoik."""
    zed = "Zed's iPhone"
    saved = [priority_connection_id(zed), ID_STAR]
    assert choose_wifi("", [zed, STARLINK], saved, set(), unmetered_ssids={zed}) == (COST_UNMETERED, zed)

  def test_an_unknown_network_still_beats_an_asserted_metered_one(self):
    """Unknown sits BETWEEN the two assertions -- it is not a synonym for either."""
    assert choose_wifi("", [STARLINK, "Visitor"], [ID_STAR, priority_connection_id("Visitor")],
                       {STARLINK}) == (1, "Visitor")

  def test_an_unknown_network_is_still_chosen_over_our_own_lte(self):
    """Being unranked must never demote a network below tier 3. Every WiFi beats the modem."""
    assert act(True, "", [STARLINK], ALL_SAVED, None, fallback_enabled=True) == "up_fallback"

  def test_a_failed_metered_read_is_recorded_as_unknown_not_as_cheap(self):
    """_metered_states puts an nmcli failure in NEITHER set. A read that failed tells us nothing and
    must not be laundered into an assertion -- so it ranks below anything actually vouched for."""
    assert choose_wifi("", [PHONE, STARLINK], ALL_SAVED, set(), unmetered_ssids={PHONE}) == (COST_UNMETERED, PHONE)


class TestStickyActiveConnection:
  """FABLE FINDING 1: a radio flap every 20 s away from home.

  The geo-gate deliberately stops scanning once we are on client WiFi, so away from any learned
  location `scan` is EMPTY. With no candidates the arbiter raised its own hotspot -- tearing down a
  working link. Next tick it was on the hotspot, so scanning resumed, it found the WiFi and went
  back. Flap, every poll interval, forever.

  NM's association state, not a scan, is the truth for "am I on this network"."""

  def test_an_empty_scan_does_not_tear_down_a_working_link(self):
    assert act(True, "", [], ALL_SAVED, ID_PHONE, fallback_enabled=True) == "noop"

  def test_the_full_flap_trace_does_not_repeat(self):
    """TRACE A, end to end: on the phone, away from home, scan suppressed by the geo-gate."""
    active = ID_PHONE
    actions = []
    for _ in range(5):
      a, ssid = decide(True, "", [], ALL_SAVED, active, fallback_enabled=True)
      actions.append(a)
      if a == "up_hotspot":
        active = HOTSPOT_CONNECTION_ID
      elif a == "up_fallback":
        active = priority_connection_id(ssid)
    assert actions == ["noop"] * 5, f"radio flapped: {actions}"

  def test_stickiness_does_not_pin_us_to_an_expensive_link(self):
    """Sticky must not mean stuck: if something genuinely cheaper appears in a scan, take it."""
    a, ssid = decide(True, "", [PHONE], ALL_SAVED, ID_STAR,
                     metered_ssids={STARLINK}, unmetered_ssids={PHONE}, fallback_enabled=True)
    assert (a, ssid) == ("up_fallback", PHONE)

  def test_the_hotspot_is_not_sticky(self):
    """Being on our own hotspot is the thing we are trying to escape -- it must never be seeded as a
    candidate, or the ladder could never climb back down to real WiFi."""
    assert act(True, "", [PHONE], ALL_SAVED, HOTSPOT_CONNECTION_ID, fallback_enabled=True) == "up_fallback"


class TestAssociationFailureLedger:
  """FABLE FINDING 2: an indefinite hotspot-down when a network will not associate.

  `up_fallback` drops the hotspot BEFORE raising the client. If the client then fails -- AP in range
  but refusing, wrong PSK, dead DHCP -- the device has no uplink at all, and next tick it picks the
  same network again because it is still in the scan. Offline forever, silently."""

  def test_a_blocked_network_is_not_chosen(self):
    assert choose_wifi("", [PHONE], ALL_SAVED, set(), blocked_ssids={PHONE}) is None

  def test_the_ladder_falls_through_to_the_next_tier_instead_of_retrying(self):
    """TRACE B: the unmetered network will not come up, so the metered one gets a turn -- and that
    is still far better than our own LTE."""
    a, ssid = decide(True, "", [PHONE, STARLINK], ALL_SAVED, None,
                     metered_ssids={STARLINK}, unmetered_ssids={PHONE},
                     blocked_ssids={PHONE}, fallback_enabled=True)
    assert (a, ssid) == ("up_fallback", STARLINK)

  def test_and_reaches_the_hotspot_when_everything_is_blocked(self):
    """TRACE C: nothing usable is left -> tier 3. This is the case that was unreachable before, and
    its absence is what left the device offline indefinitely."""
    assert act(True, "", [PHONE, STARLINK], ALL_SAVED, None,
               blocked_ssids={PHONE, STARLINK}, fallback_enabled=True) == "up_hotspot"

  def test_a_blocked_network_we_are_CONNECTED_to_is_never_dropped(self):
    """A backoff is about bringing a link UP. If we are on it, it demonstrably works -- tearing it
    down over a stale ledger entry would be the flap bug wearing a different hat."""
    assert act(True, "", [], ALL_SAVED, ID_PHONE, blocked_ssids={PHONE}, fallback_enabled=True) == "noop"

  def test_a_blocked_PRIORITY_network_falls_through_too(self):
    """The ledger applies at tier 0 as well: up_priority drops the hotspot exactly like up_fallback,
    so a dead-but-in-range home router would otherwise loop offline at 20 s intervals forever."""
    a, ssid = decide(True, HOME, [HOME, PHONE], ALL_SAVED, None,
                     unmetered_ssids={PHONE}, blocked_ssids={HOME}, fallback_enabled=True)
    assert (a, ssid) == ("up_fallback", PHONE)


class TestMeteredPriorityIsDemoted:
  """FABLE FINDING 3: configured entries were resolved by LIST POSITION, so the ladder never saw
  them. The driver defines tier 0 as "priority network, those are always unmetered" -- so a
  configured entry NM reports as metered is a contradiction, and list position must not settle it."""

  def test_a_metered_priority_entry_does_not_win_by_being_in_the_list(self):
    a, ssid = decide(True, STARLINK, [STARLINK, PHONE], ALL_SAVED, None,
                     metered_ssids={STARLINK}, unmetered_ssids={PHONE}, fallback_enabled=True)
    assert (a, ssid) == ("up_fallback", PHONE)

  def test_but_it_is_demoted_not_excluded(self):
    """A metered network still beats our own LTE. It is the right answer when it is the only WiFi.
    netrank2pnw: the action is now `up_priority`, not `up_fallback` -- the action NAME reports whether the
    winner is a configured entry (STARLINK is, here); the COST lives in its class. Both bring the network
    up identically."""
    a, ssid = decide(True, STARLINK, [STARLINK], ALL_SAVED, None,
                     metered_ssids={STARLINK}, fallback_enabled=True)
    assert (a, ssid) == ("up_priority", STARLINK)

  def test_an_unknown_priority_entry_is_still_chosen_when_it_is_the_best_there_is(self):
    """netrank2pnw REVERSED the principle this test used to state ("an unknown priority entry is NOT
    demoted -- only an explicit metered=yes contradicts tier 0"). Unknown IS now below explicitly
    unmetered, for members and non-members alike -- see the test below. What survives: alone in range,
    an unknown member is still chosen, and as a priority entry."""
    assert act(True, "Visitor", ["Visitor"], [priority_connection_id("Visitor")], None,
               fallback_enabled=True) == "up_priority"

  def test_an_unknown_priority_entry_IS_outranked_by_an_explicitly_unmetered_network(self):
    """THE REVERSAL, as a decision. Visitor is configured and `unknown`; the phone is `no`. The driver
    marking a network unmetered is his conscious choice, so the phone wins even though it is not a
    member of the list passed here."""
    visitor = priority_connection_id("Visitor")
    assert decide(True, "Visitor", ["Visitor", PHONE], [visitor, ID_PHONE], None,
                  unmetered_ssids={PHONE}, fallback_enabled=True) == ("up_fallback", PHONE)


class TestDecideReturnsWhatItRanked:
  """FABLE FINDING: the daemon used to run choose_wifi a SECOND time to work out what to bring up.
  Two independently-argued calls can disagree, and the failure mode is `nmcli con up` on a different
  network than the one the ladder ranked -- silently, since both look like a successful fallback."""

  def test_the_ssid_comes_back_with_the_action(self):
    assert decide(True, "", [STARLINK], ALL_SAVED, None,
                  metered_ssids={STARLINK}, fallback_enabled=True) == ("up_fallback", STARLINK)

  def test_hotspot_actions_carry_no_ssid(self):
    assert decide(True, "", [], ALL_SAVED, None, fallback_enabled=True) == ("up_hotspot", "")
    assert decide(False, "", [], ALL_SAVED, HOTSPOT_CONNECTION_ID) == ("down_hotspot", "")

  def test_up_priority_carries_the_priority_ssid(self):
    assert decide(True, HOME, [HOME], ALL_SAVED, None) == ("up_priority", HOME)


class TestAssociatedIsNotUsable:
  """GEMINI FINDING 1 — the severe one: the stickiness fix created a PERMANENT OFFLINE state.

  Trace, exactly as reported:
    tick N    up_fallback to a network whose DHCP is dead
    tick N+1  it is `active` (NM reports activated on association), verification finds no IPv4,
              a failure is recorded... and then the stickiness rule unblocked it again for being
              active, choose_wifi returned it as the winner, decide() said noop because we were
              already on it, and pending_up was not set because the action was noop.
    tick N+2+ nothing ever re-checks. Associated, no address, forever, hotspot never raised.

  The bug was a comment of mine that read "a working link is never in backoff" while the code only
  ever established that the link was ASSOCIATED. The daemon now verifies the ACTIVE connection every
  tick and passes active_ssid="" when it has no IPv4, so a dead link is neither sticky nor exempt."""

  def test_a_dead_link_is_not_sticky_and_we_move_off_it(self):
    a, ssid = decide(True, "", [PHONE], ALL_SAVED, ID_STAR, active_ssid="",
                     blocked_ssids={STARLINK}, unmetered_ssids={PHONE}, fallback_enabled=True)
    assert (a, ssid) == ("up_fallback", PHONE)

  def test_a_dead_link_with_nothing_else_around_reaches_the_hotspot(self):
    """THE STATE THAT WAS UNREACHABLE. Associated to a dead network, nothing else in range: the
    device must raise its own hotspot, not noop forever."""
    assert act(True, "", [], ALL_SAVED, ID_STAR, active_ssid="",
               blocked_ssids={STARLINK}, fallback_enabled=True) == "up_hotspot"

  def test_a_VERIFIED_link_is_still_sticky(self):
    """...and the fix must not undo Fable's finding 1: a link that IS carrying traffic still
    survives an empty scan."""
    assert act(True, "", [], ALL_SAVED, ID_PHONE, active_ssid=PHONE,
               blocked_ssids={PHONE}, fallback_enabled=True) == "noop"

  def test_the_full_dead_dhcp_trace_terminates(self):
    """Drive the loop the way the daemon does. It must reach the hotspot and STAY there, rather
    than nooping on a dead link."""
    active, blocked, actions = ID_STAR, set(), []
    for _ in range(6):
      usable = active not in (ID_STAR,)          # the Starlink profile never gets an address
      a_ssid = ssid_of(active or "") if usable else ""
      if not usable and active:
        blocked.add(ssid_of(active))
      a, ssid = decide(True, "", [STARLINK], ALL_SAVED, active, active_ssid=a_ssid,
                       blocked_ssids=blocked, fallback_enabled=True)
      actions.append(a)
      if a == "up_hotspot":
        active = HOTSPOT_CONNECTION_ID
      elif a in ("up_fallback", "up_priority"):
        active = priority_connection_id(ssid)
    assert actions[0] == "up_hotspot", f"never escaped the dead link: {actions}"
    assert set(actions[1:]) == {"noop"}, f"did not settle on the hotspot: {actions}"


class TestNoBlameForSomeoneElsesChoice:
  """GEMINI FINDING 4. The old code remembered the ssid it had raised and judged THAT one a tick
  later. If the driver joined a different network from the UI in between, the network we raised was
  blamed and backed off for something it never did. Judging whatever is ACTIVE removes the failure
  mode rather than compensating for it: we can only ever penalise the network actually in use."""

  def test_the_network_the_driver_picked_is_the_one_judged(self):
    """He joined PHONE by hand after we raised STARLINK. PHONE is active and usable, so the ladder
    leaves it alone -- and STARLINK carries no new failure."""
    assert act(True, "", [PHONE, STARLINK], ALL_SAVED, ID_PHONE, active_ssid=PHONE,
               unmetered_ssids={PHONE}, fallback_enabled=True) == "noop"


class TestCaseInsensitivity:
  """FABLE: `reachable` compared exact case while select_available is case-insensitive, so a
  configured "visitor" against an AP advertising "Visitor" was picked as tier 0 by one and rejected
  by the other -- it connected as an ordinary unknown-cost network and lost to anything explicitly
  unmetered. That SSID is in the driver's real priority list and has already cost one captive-portal
  bug for the same reason (users type "visitor", the AP advertises "Visitor")."""

  def test_a_priority_entry_still_wins_when_the_case_differs(self):
    """Membership must match case-insensitively. netrank2pnw: the phone is no longer asserted unmetered
    in this test -- under the generic rule an explicitly unmetered phone would (correctly) beat an unknown
    Visitor on COST, which would stop this test being about case at all. Both are `unknown` now, so only
    membership can decide, and it has to match "visitor" to "Visitor"."""
    a, ssid = decide(True, "visitor", ["Visitor", PHONE], [priority_connection_id("visitor"), ID_PHONE],
                     None, fallback_enabled=True)
    assert (a, ssid) == ("up_priority", "visitor")

  def test_and_the_saved_connection_match_is_case_insensitive_too(self):
    a, _ = decide(True, "Visitor", ["Visitor"], [priority_connection_id("visitor")], None,
                  fallback_enabled=True)
    assert a == "up_priority"


class TestBinaryModeIsPreLadder:
  """netrank2pnw: with fallback_enabled False (DisableNetworkCostLadder) decide() is the arbiter from
  before cost existed -- the first reachable configured entry, or the hotspot -- plus the failure ledger."""

  def test_it_ignores_cost_entirely(self):
    assert decide(True, STARLINK, [STARLINK, PHONE], ALL_SAVED, None, metered_ssids={STARLINK},
                  unmetered_ssids={PHONE}) == ("up_priority", STARLINK)

  def test_it_still_honours_the_failure_ledger(self):
    """The one retention: a dead-but-in-range router must not hold the device offline under the kill switch."""
    assert decide(True, HOME, [HOME], ALL_SAVED, None, blocked_ssids={HOME.lower()}) == ("up_hotspot", "")

  def test_it_brings_up_the_SAVED_PROFILE_spelling(self):
    """`nmcli con up` matches ids case-sensitively: configured "visitor", profile "Visitor"."""
    assert decide(True, "visitor", ["Visitor"], [priority_connection_id("Visitor")], None) == ("up_priority", "Visitor")

  def test_an_active_priority_entry_absent_from_the_scan_stays_put(self):
    """The geo-gate suppresses scanning on client WiFi; being ON the entry is reach enough."""
    assert decide(True, HOME, [], ALL_SAVED, ID_HOME) == ("noop", HOME)


class TestExplainFallback:
  """arbiterfu2pnw: the reason text on the up_fallback log line. The loop tests pin the common cases; these pin the ones
  the loop cannot reach -- and every one of them is text only."""

  def test_a_scan_that_did_not_RUN_is_not_called_out_of_range(self):
    """The geo-gate suppresses scans on client WiFi: an empty scan there is no statement about range."""
    txt = explain_fallback([HOME], [], ALL_SAVED, STARLINK, scanned=False)
    assert "'Hannelore': not seen, no scan ran this tick" in txt, txt
    assert "not in the scan" not in txt, txt

  def test_the_usable_active_link_counts_as_in_range_and_is_never_in_backoff(self):
    """Mirrors choose_wifi: the sticky active link is a candidate without a scan result, and a verified link is never
    blocked. Here it is metered and loses to an unmetered non-member."""
    txt = explain_fallback([STARLINK], [PHONE], ALL_SAVED, PHONE, metered_ssids={STARLINK}, unmetered_ssids={PHONE},
                           blocked_ssids={STARLINK.lower()}, active_ssid=STARLINK)
    assert txt == "unmetered, not a configured priority network -- 'KarlMoik': in range, metered, outranked", txt

  def test_a_member_that_should_have_WON_is_reported_as_a_disagreement_not_given_a_made_up_reason(self):
    txt = explain_fallback([HOME], [HOME, PHONE], ALL_SAVED, PHONE, unmetered_ssids={HOME, PHONE})
    assert "'Hannelore': in range, unmetered, NOT outranked: explanation disagrees with the ranking" in txt, txt

  def test_blank_entries_are_not_members(self):
    assert explain_fallback(["", "  "], [PHONE], ALL_SAVED, PHONE) == "cost unknown -- no priority networks configured"

  def test_case_is_folded_like_the_ranking_does(self):
    txt = explain_fallback(["hannelore"], ["HANNELORE"], ALL_SAVED, PHONE, metered_ssids={"HANNELORE"},
                           unmetered_ssids={PHONE}, blocked_ssids=set())
    assert "'hannelore': in range, metered, outranked" in txt, txt
