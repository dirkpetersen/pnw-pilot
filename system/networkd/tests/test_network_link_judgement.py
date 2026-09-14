"""netcosttier2pnw — judge_link: what a tick concludes about the client link.

This is the tick logic that FOUR separate review defects lived in while it was glue inside the
daemon loop. It is pure now precisely so the four measured failure sequences can be tests.

The sequences, by the names the review gave them:

  refuse           `con up` fails outright. Nothing is active afterwards, so judging the ACTIVE link
                   cannot see it -- the failure is invisible, the SSID stays unblocked, and the
                   arbiter drops the hotspot and retries it forever. Measured: 5 attempts, 0 hotspot
                   re-raises, tethered clients dark indefinitely.
  flicker          the scan omits an SSID (or the geo-gate suppresses scanning entirely), the ledger
                   is wiped as if the network had gone away and come back, and the backoff collapses
                   to the flicker rate. Measured: cleared at ticks 3, 5, 7; failures never got past 1.
  ip_read_timeout  one nmcli read of IP4.ADDRESS times out, the link is recorded as dead, un-stuck,
                   and with the scan empty the radio goes to the hotspot. Measured: a WORKING iPhone
                   link blamed and torn down on tick 2.
  slow_dhcp        the first look lands mid-activation (NM's DHCP timeout is 45 s, the poll is 20 s).
                   Measured: torn down on tick 2, and it only recovered because `flicker` happened to
                   clear the ledger -- two bugs cancelling.
"""
from openpilot.system.networkd.network_arbiter import judge_link, pending_for_new_link

PHONE, STAR = "Dirk's iPhone 13", "KarlMoik"
GRACE = 60.0


class TestRefuse:
  def test_a_bring_up_that_never_took_is_blamed(self):
    """The ONLY way an outright association failure is ever seen: we raised it, and nothing is
    active now. An earlier cut deleted this mechanism in favour of judging the active link, which
    made this failure invisible again."""
    v = judge_link("", None, (STAR, 90.0), 100.0, GRACE)
    assert (v.blame, v.blame_ok, v.pending) == (STAR, False, None)
    assert v.sticky_ssid == ""

  def test_and_the_blame_does_not_linger(self):
    v = judge_link("", None, (STAR, 90.0), 100.0, GRACE)
    assert v.pending is None, "a judged bring-up must not be judged twice"


class TestUnreadableActiveRead:
  """arbiterfu2pnw: active_ssid None = `con show --active` failed. That is not "nothing active", so it is no
  evidence that a pending bring-up never took."""

  def test_an_unreadable_read_does_not_blame_a_pending_bring_up(self):
    v = judge_link(None, None, (STAR, 90.0), 100.0, GRACE)
    assert v == (STAR, "", False, (STAR, 90.0)), "blamed a bring-up on a read that failed"

  def test_not_even_past_the_grace__the_caller_owns_that_bound(self):
    """The DHCP grace judges a link we can SEE without an address; an unreadable tick sees nothing."""
    v = judge_link(None, None, (STAR, 10.0), 100.0, GRACE)
    assert v == (STAR, "", False, (STAR, 10.0))

  def test_with_nothing_pending_it_concludes_nothing__as_before(self):
    assert judge_link(None, None, None, 100.0, GRACE) == ("", "", False, None)


class TestSomeoneElsesChoice:
  def test_a_network_the_driver_joined_by_hand_takes_over_without_blame(self):
    """We raised STAR; the driver picked PHONE from the UI. STAR did not fail, it was overruled."""
    v = judge_link(PHONE, True, (STAR, 90.0), 100.0, GRACE)
    assert v.blame == "" and v.pending is None
    assert v.sticky_ssid == PHONE

  def test_a_manual_join_that_lands_mid_DHCP_gets_the_grace_too(self):
    """The last variant of the no-grace defect. pending_for_new_link cannot help here: it runs
    earlier in the tick, sees the OLD pending still set, and so never synthesises one for the
    newcomer. Measured -- the hotspot took the radio out from under the driver's own join."""
    v = judge_link(PHONE, False, (STAR, 95.0), 100.0, GRACE)
    assert v.sticky_ssid == PHONE, "the driver's manual join was dropped mid-DHCP"
    assert v.pending == (PHONE, 100.0), "the newcomer must become the pending one"
    assert v.blame == "", "and nobody is blamed for being overruled"

  def test_an_unreadable_manual_join_also_gets_it(self):
    v = judge_link(PHONE, None, (STAR, 95.0), 100.0, GRACE)
    assert v.sticky_ssid == PHONE and v.pending == (PHONE, 100.0)


class TestSlowDhcp:
  def test_inside_the_grace_window_it_is_not_a_failure(self):
    v = judge_link(PHONE, False, (PHONE, 90.0), 100.0, GRACE)
    assert v.blame == "" and v.pending == (PHONE, 90.0)
    assert v.sticky_ssid == PHONE, "a link still doing DHCP must not be torn down"

  def test_past_the_grace_window_it_is(self):
    v = judge_link(PHONE, False, (PHONE, 20.0), 100.0, GRACE)
    assert (v.blame, v.blame_ok) == (PHONE, False)
    assert v.sticky_ssid == ""

  def test_an_address_arriving_inside_the_window_settles_it_immediately(self):
    v = judge_link(PHONE, True, (PHONE, 90.0), 100.0, GRACE)
    assert (v.blame, v.blame_ok, v.pending) == (PHONE, True, None)


class TestIpReadTimeout:
  def test_an_unreadable_link_is_neither_blamed_nor_un_stuck(self):
    """THE DEFECT. One flaky nmcli call tore down a working link and blamed it."""
    v = judge_link(PHONE, None, None, 100.0, GRACE, last_known_usable=True)
    assert v.blame == ""
    assert v.sticky_ssid == PHONE

  def test_it_also_does_not_resolve_a_pending_judgement(self):
    """No information means keep waiting, not decide -- and this must hold OUTSIDE the DHCP grace
    window too. An earlier version of this test used a timestamp inside the grace window, where the
    slow-DHCP branch produces the same answer for the wrong reason: it passed even with the unknown
    handling deleted. Caught by mutation."""
    v = judge_link(PHONE, None, (PHONE, 10.0), 100.0, GRACE, last_known_usable=True)
    assert 100.0 - 10.0 > GRACE, "the point of this test is to be PAST the grace window"
    assert v.pending == (PHONE, 10.0), "an unreadable tick resolved a pending judgement"
    assert v.blame == "", "an unreadable tick blamed a link it could not read"
    assert v.sticky_ssid == PHONE

  def test_but_a_link_last_seen_DEAD_is_not_made_sticky_by_an_unreadable_tick(self):
    """Unknown must not launder a known-bad link into a good one either -- for a link we did NOT
    raise. (With a pending bring-up the cache is deliberately ignored; see below.)"""
    v = judge_link(PHONE, None, None, 100.0, GRACE, last_known_usable=False)
    assert v.sticky_ssid == ""

  def test_a_PENDING_link_stays_sticky_even_with_a_cached_dead_reading(self):
    """FABLE, measured: a link doing DHCP legitimately reads False, main() cached that False, and
    then ONE unreadable tick inside the grace window made it non-sticky -- with the scan suppressed
    by the geo-gate the hotspot took the radio, and the next tick blamed the target for 'never
    taking'. A stale False from an EARLIER visit to the same network did the same, because the cache
    was never invalidated on a bring-up. A link we raised ourselves is inside its own judgement
    window; the grace/False path still catches one that is genuinely dead."""
    v = judge_link(PHONE, None, (PHONE, 90.0), 100.0, GRACE, last_known_usable=False)
    assert v.sticky_ssid == PHONE, "a cached False tore down a link mid-bring-up"
    assert v.blame == "" and v.pending == (PHONE, 90.0)

  def test_and_that_holds_past_the_grace_window_too(self):
    v = judge_link(PHONE, None, (PHONE, 10.0), 100.0, GRACE, last_known_usable=False)
    assert v.sticky_ssid == PHONE and v.blame == ""


class TestTheOrdinaryCases:
  def test_a_healthy_link_is_sticky_and_clears_its_ledger_entry(self):
    v = judge_link(PHONE, True, None, 100.0, GRACE)
    assert (v.sticky_ssid, v.blame, v.blame_ok) == (PHONE, PHONE, True)

  def test_a_dead_link_is_blamed_and_not_sticky(self):
    v = judge_link(PHONE, False, None, 100.0, GRACE)
    assert (v.sticky_ssid, v.blame, v.blame_ok) == ("", PHONE, False)

  def test_nothing_active_and_nothing_pending_concludes_nothing(self):
    v = judge_link("", None, None, 100.0, GRACE)
    assert v == ("", "", False, None)


class TestALinkWeDidNotRaise:
  """FABLE #3: the no-pending branch blamed `usable is False` immediately, so a network the driver
  joined from the UI -- or one NM autoconnected at boot -- was caught mid-DHCP if the tick landed in
  that window, blamed, un-stuck, and replaced by the hotspot on its very first tick.

  The daemon now synthesises a pending entry the first tick it sees an active link it did not raise,
  so the same three-way split (and the same DHCP grace) applies to it. These pin the behaviour
  judge_link must provide for that to work."""

  def test_a_freshly_seen_link_gets_the_same_grace(self):
    v = judge_link(PHONE, False, (PHONE, 95.0), 100.0, GRACE)
    assert v.blame == "", "a link joined from the UI was blamed mid-DHCP"
    assert v.sticky_ssid == PHONE

  def test_and_is_still_blamed_if_it_never_comes_up(self):
    v = judge_link(PHONE, False, (PHONE, 10.0), 100.0, GRACE)
    assert (v.blame, v.blame_ok) == (PHONE, False)


class TestPendingForNewLink:
  """The glue that gives an unraised link its DHCP grace -- pure, so it is actually testable."""

  def test_a_newly_seen_link_becomes_pending(self):
    assert pending_for_new_link(PHONE, None, "", 100.0) == (PHONE, 100.0)

  def test_a_link_we_already_knew_about_does_not_restart_its_grace(self):
    """Otherwise the grace would renew every tick and a dead link would never be blamed."""
    assert pending_for_new_link(PHONE, None, PHONE.lower(), 100.0) is None

  def test_an_existing_pending_is_never_overwritten(self):
    assert pending_for_new_link(PHONE, (STAR, 50.0), "", 100.0) == (STAR, 50.0)

  def test_nothing_active_creates_nothing(self):
    assert pending_for_new_link("", None, PHONE.lower(), 100.0) is None

  def test_the_comparison_is_case_insensitive(self):
    assert pending_for_new_link("Visitor", None, "visitor", 100.0) is None
