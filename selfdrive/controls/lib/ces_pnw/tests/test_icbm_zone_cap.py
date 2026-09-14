"""sazoneset2pnw — a curve restore returns to the pre-curve set, capped ONLY when the limit dropped during it.

Driver directive 2026-09-13: "Then comes a curve and you reduce for the curve, remember what the speed was
before the curve and then you go back to the speed after the curve." Under his zone model the pre-curve set
already belongs to the current road, so a restore goes all the way back -- even when that is more than
5 mph over the limit. The one exception is a ceiling that became STALE because the limit dropped while the
curve was running (the 15:46 incident: latched at 60 on a 45 road, restored to 60 inside a 25 zone). Then
the restore is capped at the zone speed the driver would have got entering that zone, and the cap is sticky.
"""
import json
import math
import os

import pytest

from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import IcbmEpisode, ICBM_RESTORE_DELAY_S, icbm_note_speedadjust
from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw_constants import (
  ICBM_RESTORE_LIMIT_MARGIN_MS, icbm_restore_limit, icbm_stale_zone_cap)

MPH = 0.44704
FIXTURE = os.path.join(os.path.dirname(__file__), "data", "restore_25zone_2026-09-13.json")


class TestStaleZoneCap:
  @pytest.mark.parametrize("limit_now", [45, 55])
  def test_same_or_higher_limit_is_NOT_stale(self, limit_now):
    assert icbm_stale_zone_cap(60 * MPH, 45 * MPH, limit_now * MPH, True) == (None, None)

  @pytest.mark.parametrize("limit_now", [0.0, None, float("nan"), "x"])
  def test_no_limit_now_is_no_evidence_of_a_new_zone(self, limit_now):
    assert icbm_stale_zone_cap(60 * MPH, 45 * MPH, limit_now, True) == (None, None)

  def test_a_drop_with_zone_speeds_on_is_the_drivers_own_percentage(self):
    cap, why = icbm_stale_zone_cap(60 * MPH, 45 * MPH, 25 * MPH, True)
    assert why == "prop" and math.isclose(cap, 25 * MPH * 60 / 45)

  def test_a_drop_with_zone_speeds_off_is_limit_plus_five(self):
    cap, why = icbm_stale_zone_cap(60 * MPH, 45 * MPH, 25 * MPH, False)
    assert why == "limit5" and math.isclose(cap, 25 * MPH + ICBM_RESTORE_LIMIT_MARGIN_MS)

  def test_limit_unknown_at_latch_and_known_now_is_stale_with_the_backstop(self):
    cap, why = icbm_stale_zone_cap(60 * MPH, None, 25 * MPH, True)
    assert why == "limit5" and math.isclose(cap, 25 * MPH + ICBM_RESTORE_LIMIT_MARGIN_MS)

  def test_a_driver_who_was_UNDER_the_limit_gets_the_backstop_not_a_trim_below_it(self):
    cap, why = icbm_stale_zone_cap(40 * MPH, 45 * MPH, 25 * MPH, True)
    assert why == "limit5"

  def test_one_mph_of_rounding_is_not_a_drop(self):
    assert icbm_stale_zone_cap(60 * MPH, 45 * MPH, 45 * MPH - 0.5, True) == (None, None)

  def test_the_proportional_cap_is_always_below_the_ceiling(self):
    for lat in (30, 45, 65):
      for now in (25, 35, 50):
        cap, _ = icbm_stale_zone_cap(70 * MPH, lat * MPH, now * MPH, True)
        if cap is not None and now < lat:
          assert cap < 70 * MPH


class TestTheEpisodeKeepsTheCapSticky:
  @staticmethod
  def _cap(ep, t=100.0, limit=45 * MPH):
    ep.step(t, 30 * MPH, 60 * MPH, 60 * MPH, True, False, limit_now=limit)
    return t

  def test_the_latch_records_the_limit(self):
    ep = IcbmEpisode()
    self._cap(ep)
    assert math.isclose(ep.latch_limit, 45 * MPH)

  def test_note_limit_never_touches_the_ceiling(self):
    """SAFETY. The ceiling is the reference a curve's binding is judged against during the cap phase;
    lowering it there could make a real curve stop binding mid-approach. Only the restore is capped."""
    ep = IcbmEpisode()
    self._cap(ep)
    ep.note_limit(25 * MPH, True)
    assert math.isclose(ep.ceiling, 60 * MPH), "note_limit edited the cap-phase ceiling"
    assert ep.zone_cap is not None

  def test_the_cap_only_ever_lowers(self):
    ep = IcbmEpisode()
    self._cap(ep)
    ep.note_limit(35 * MPH, False)
    first = ep.zone_cap
    ep.note_limit(45 * MPH, False)                       # the limit rises back: no evidence to raise it
    ep.note_limit(55 * MPH, False)
    assert ep.zone_cap == first
    ep.note_limit(25 * MPH, False)
    assert ep.zone_cap < first

  def test_idle_ignores_limits(self):
    ep = IcbmEpisode()
    ep.note_limit(25 * MPH, True)
    assert ep.zone_cap is None

  def test_a_new_curve_during_restore_relatches_and_clears_the_cap(self):
    ep = IcbmEpisode()
    t = self._cap(ep)
    ep.note_limit(25 * MPH, False)
    ep.step(t + 1, None, 30 * MPH, 30 * MPH, True, False, restore_cap=ep.zone_cap, limit_now=25 * MPH)
    ep.step(t + 1 + ICBM_RESTORE_DELAY_S + 0.1, None, 30 * MPH, 30 * MPH, True, False,
            restore_cap=ep.zone_cap, limit_now=25 * MPH)
    ep.step(t + 10, 20 * MPH, 30 * MPH, 30 * MPH, True, False, limit_now=25 * MPH)   # DEC ALWAYS WINS
    assert ep.phase == "cap" and math.isclose(ep.latch_limit, 25 * MPH) and ep.zone_cap is None


class TestSpeedadjustsZoneBoundsTheRestore:
  """Fable review, measured: a curve overlapping the zone entry latched the PRE-zone set (75) as its ceiling with the
  limit already reading 45, so nothing looked stale and the restore went back to 75 in the 45 zone -- permanently.
  The restore is now also bounded by speedadjust's zone speed whenever a zone set ran during the episode."""

  @staticmethod
  def _episode(n_idle=3):
    ep = IcbmEpisode()
    ep.note_sa_zone(None, n_idle, 50 * MPH)              # idle reads before the curve
    ep.step(100.0, 35 * MPH, 75 * MPH, 75 * MPH, True, False, limit_now=45 * MPH)
    assert ep.phase == "cap"
    return ep

  def test_a_zone_in_progress_bounds_the_restore_and_never_the_ceiling(self):
    ep = self._episode()
    ep.note_sa_zone(56.25 * MPH, 4, 56.25 * MPH)
    assert ep.zone_why == "saZone" and math.isclose(ep.zone_cap, 56.25 * MPH)
    assert math.isclose(ep.ceiling, 75 * MPH), "SAFETY: the zone bound edited the cap-phase ceiling"

  def test_a_zone_that_BEGAN_and_completed_between_two_reads_still_bounds_it(self):
    """Our own taps had the set below the zone speed before the drop confirmed, so the zone opened and completed on
    one speedadjust tick: zoneTgt was never published. Only the count says a zone happened."""
    ep = self._episode(n_idle=3)
    ep.note_sa_zone(None, 4, 56.25 * MPH)
    assert ep.zone_why == "saZone" and math.isclose(ep.zone_cap, 56.25 * MPH)

  def test_the_count_at_the_START_is_the_last_idle_read(self):
    """ICBM reads the status at ~1 Hz; the first read inside the episode may already include a zone that began
    after the latch. The reference is the last read before the episode, not the first one inside it."""
    ep = self._episode(n_idle=3)
    ep.note_sa_zone(None, 4, 56.25 * MPH)
    assert ep.zone_n0 == 3 and ep.zone_cap is not None

  def test_a_zone_completed_BEFORE_the_curve_does_not_bound_it(self):
    """S1L: the zone set 56 long ago, the curve latched 56 (or the driver has since raised the set to 65) --
    the pre-curve set already belongs to this road."""
    ep = self._episode(n_idle=3)
    for _ in range(5):
      ep.note_sa_zone(None, 3, 50 * MPH)
    assert ep.zone_cap is None and ep.zone_why is None

  def test_the_NEXT_episode_takes_a_fresh_reference_count(self):
    """A zone that completed between two curves happened before the second one -- it must not bound its restore."""
    ep = self._episode(n_idle=3)
    ep.note_sa_zone(None, 3, 50 * MPH)                   # episode 1 takes its reference count
    assert ep.zone_n0 == 3
    ep.reset()                                           # episode 1 ends
    ep.note_sa_zone(None, 4, 50 * MPH)                   # a zone opens and completes while idle
    ep.step(200.0, 35 * MPH, 56 * MPH, 56 * MPH, True, False, limit_now=45 * MPH)
    ep.note_sa_zone(None, 4, 50 * MPH)
    assert ep.zone_n0 == 4 and ep.zone_cap is None

  @staticmethod
  def _restarted(ep, zone_tgt=None, limit=45 * MPH, inst=2.0, n=0):
    ep.note_sa_zone(zone_tgt, n, None, limit, inst)

  def _ep_inst(self, inst_idle=1.0, n_idle=1):
    ep = IcbmEpisode()
    ep.note_sa_zone(None, n_idle, 56.25 * MPH, 45 * MPH, inst_idle)
    ep.step(100.0, 35 * MPH, 75 * MPH, 75 * MPH, True, False, limit_now=45 * MPH)
    return ep

  def test_a_speedadjust_RESTART_mid_episode_falls_back_to_limit_plus_five_and_says_so(self, monkeypatch):
    """zonefollow2pnw (Fable A3d): plannerd restarted inside the ~1 s between the latch and ICBM's next status read,
    so the zone it was setting vanished from its status -- 73-75 mph in a 45 zone."""
    import openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw as m
    logged = []
    monkeypatch.setattr(m.cloudlog, "error", lambda msg, *a, **k: logged.append(msg))
    ep = self._ep_inst()
    ep.note_sa_zone(None, 1, 56.25 * MPH, 45 * MPH, 1.0)   # same instance: nothing to bound
    assert ep.zone_cap is None
    for _ in range(3):
      self._restarted(ep)
    assert ep.zone_why == "saRestart" and math.isclose(ep.zone_cap, 45 * MPH + ICBM_RESTORE_LIMIT_MARGIN_MS)
    assert len([x for x in logged if "restarted" in x]) == 1
    assert math.isclose(ep.ceiling, 75 * MPH), "SAFETY: the restart bound edited the cap-phase ceiling"

  def test_a_zone_speed_seen_before_the_restart_stays_the_bound(self):
    ep = self._ep_inst()
    ep.note_sa_zone(56.25 * MPH, 2, 56.25 * MPH, 45 * MPH, 1.0)
    self._restarted(ep)
    assert ep.zone_why == "saZone" and math.isclose(ep.zone_cap, 56.25 * MPH)

  def test_a_LOWER_zone_from_the_restarted_instance_still_binds(self):
    ep = self._ep_inst()
    self._restarted(ep, zone_tgt=48 * MPH)
    assert math.isclose(ep.zone_cap, 48 * MPH) and ep.zone_why == "saZone"

  def test_a_restart_with_the_limit_unknown_bounds_nothing_but_is_logged(self, monkeypatch):
    import openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw as m
    logged = []
    monkeypatch.setattr(m.cloudlog, "error", lambda msg, *a, **k: logged.append(msg))
    ep = self._ep_inst()
    self._restarted(ep, limit=None)
    assert ep.zone_cap is None and any("NOTHING" in x for x in logged)

  @pytest.mark.parametrize("inst", [None])
  def test_an_unreadable_instance_is_not_a_restart(self, inst):
    ep = self._ep_inst()
    ep.note_sa_zone(None, 1, 56.25 * MPH, 45 * MPH, inst)
    assert ep.zone_cap is None

  def test_the_instance_at_the_START_is_the_last_idle_read_and_reset_clears_it(self):
    """A restart between the latch and the first in-episode read must still count."""
    ep = self._ep_inst(inst_idle=1.0)
    self._restarted(ep, inst=2.0)
    assert ep._sa_inst0 == 1.0 and ep.zone_why == "saRestart"
    ep.reset()
    ep.note_sa_zone(None, 0, None, 45 * MPH, 2.0)          # idle again, on the new instance
    ep.step(200.0, 35 * MPH, 50 * MPH, 50 * MPH, True, False, limit_now=45 * MPH)
    ep.note_sa_zone(None, 0, None, 45 * MPH, 2.0)
    assert ep._sa_inst0 == 2.0 and ep.zone_cap is None, "the next episode inherited the old restart"

  def test_the_bound_only_ever_lowers(self):
    ep = self._episode()
    ep.note_sa_zone(50 * MPH, 4, 50 * MPH)
    ep.note_sa_zone(60 * MPH, 5, 60 * MPH)
    assert math.isclose(ep.zone_cap, 50 * MPH)

  @pytest.mark.parametrize("tgt,n,last", [("x", 4, None), (float("nan"), 3, None), (-1.0, 3, None),
                                          (None, "junk", 56 * MPH), (None, None, 56 * MPH)])
  def test_garbage_and_unreadable_status_bound_nothing(self, tgt, n, last):
    ep = self._episode()
    ep.note_sa_zone(tgt, n, last)
    assert ep.zone_cap is None

  def test_idle_notes_bound_nothing_and_reset_keeps_the_idle_count(self):
    ep = IcbmEpisode()
    ep.note_sa_zone(40 * MPH, 7, 40 * MPH)
    assert ep.zone_cap is None and ep.zone_n0 is None
    ep.reset()
    assert ep._sa_zone_n_idle == 7

  def test_the_wiring_helper_feeds_both_bounds(self):
    ep = self._episode()
    icbm_note_speedadjust(ep, {"saMode": 2, "saZoneTgt": None, "saZoneN": 3, "saZoneLast": None}, 25 * MPH)
    assert ep.zone_why == "prop" and math.isclose(ep.zone_cap, 25 * MPH * 75 / 45)
    icbm_note_speedadjust(ep, {"saMode": 2, "saZoneTgt": 30 * MPH, "saZoneN": 4, "saZoneLast": 30 * MPH}, 25 * MPH)
    assert ep.zone_why == "saZone" and math.isclose(ep.zone_cap, 30 * MPH)

  @pytest.mark.parametrize("tele", [None, {}, {"saMode": "x"}, {"saMode": 1}])
  def test_the_wiring_helper_without_zone_speeds_uses_the_backstop(self, tele):
    ep = IcbmEpisode()
    ep.step(100.0, 30 * MPH, 60 * MPH, 60 * MPH, True, False, limit_now=45 * MPH)
    icbm_note_speedadjust(ep, tele, 25 * MPH)
    assert ep.zone_why == "limit5"


class TestTheRealIncident:
  """2026-09-13 15:46 PT, replayed through the new decision: it must never restore toward 60."""

  @pytest.mark.parametrize("zone_speeds_on", [True, False])
  def test_it_never_restores_above_the_zone_speed(self, zone_speeds_on):
    with open(FIXTURE) as f:
      ticks = json.load(f)
    ep, st, worst, caps, phases = IcbmEpisode(), None, None, set(), set()
    for r in ticks:
      lim, st = icbm_restore_limit(r["spdLim"], st, r["t"])
      ep.note_limit(lim if lim > 0 else None, zone_speeds_on)
      cap_t = r["icbmT"] if r["icbmSrc"] in ("map", "far", "vis") else None
      tgt, d = ep.step(r["t"], cap_t, r["stockSet"], r["stockSet"], bool(r["stockOn"]), bool(r["gas"]),
                       restore_cap=ep.zone_cap, limit_now=lim if lim > 0 else None)
      phases.add(ep.phase)
      if ep.zone_cap is not None:
        caps.add(ep.zone_cap)
      if d == "inc" and abs((r["spdLim"] or 0) - 25 * MPH) < 0.2:
        worst = tgt if worst is None else max(worst, tgt)
    # the path must actually have run: a stale cap was formed and the episode reached its restore. (With zone
    # speeds on the cap is 26.75 mph and the recorded set was already 27, so the restore rightly HOLDS without
    # publishing anything -- a first version of this test demanded a publish and failed on correct behaviour.)
    assert caps and "restore" in phases, "the replay never formed a stale cap and restored -- proves nothing"
    cap = min(caps)
    want = 25.05 * MPH * (48.0 / 45.0) if zone_speeds_on else 25.05 * MPH + ICBM_RESTORE_LIMIT_MARGIN_MS
    assert math.isclose(cap, want, abs_tol=0.05), f"stale cap {cap / MPH:.2f}, want {want / MPH:.2f}"
    assert worst is None or worst <= cap + 1e-6, f"restored toward {worst / MPH:.1f} mph past the zone cap"
    assert worst is None or worst < 40 * MPH
