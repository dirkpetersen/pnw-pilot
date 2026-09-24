"""restorehold2pnw -- an ICBM restore may not rise above the curve ahead's safe speed.

THE EVENT (2026-09-21 21:21 PT, I-5 S, Tumwater S-bend; tests/data/tumwater_sbend_2026-09-21.json, the logged
ces_events ticks): ICBM capped the S-bend's right half 75 -> 64. At 21:21:49 the curve cleared and the RESTORE
walked the set 64 -> 75; the truck accelerated 64 -> 73 mph into the LEFT half and openpilot's steering saturated
(21:22:01, demand 3.47 vs its own 3.21 ceiling) before the driver steered in. At the restore's first tick the map
polyline already measured the left curve: icbmK 0.00256 at 393 m, ahead -- 2.87 m/s^2 at the 75 mph ceiling.

THE OWNER'S CONSTRAINTS: "I do not want any more slowdowns" (2026-09-23) and "resume speed as fast as possible"
(2026-09-24). These tests pin that the hold (a) never taps SET-, never lowers a target and never changes how a cap
binds, (b) still restores up to the curve's safe speed and never parks the set below the truck's current speed, (c)
triggers on the polyline only at 2.8 and on vision at 2.5, (d) resumes once the curve is clear, (e) is bounded by the
restore window, (f) never leaves the restore stuck below the driver's original set because a curve bound during it,
(g) survives our own late SET+ taps (Fable F1), (h) does nothing on a car without the capability.
"""
import json
import math
import os
import random

import pytest

from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import (
  IcbmEpisode, icbm_restore_ahead_hold, ICBM_RESTORE_DELAY_S, ICBM_RESTORE_HOLD_CLEAR_S, ICBM_RESTORE_WINDOW_S)
from openpilot.selfdrive.controls.lib import pnw_vehicle as pv
from openpilot.selfdrive.controls.lib.pnw_vehicle import PnwVehicle

MPH = 0.44704
DT = 0.25                      # the brain's cadence
TAP_S = 0.4                    # the executor's tap cadence (icbm_pnw PRESS+GAP)
DEADBAND = 0.6 * MPH           # icbm_pnw DEADBAND_MS
A_POLY, A_VIS = 2.8, 2.5       # the Lightning defaults: polyline bar, vision bar (= the safe-speed target)
FIXTURE = os.path.join(os.path.dirname(__file__), "data", "tumwater_sbend_2026-09-21.json")


def hold_fn(v, kp, kn, ahead, kv, a_poly=A_POLY, a_vis=A_VIS, a_safe=A_VIS):
  return icbm_restore_ahead_hold(v, kp, kn, ahead, kv, a_poly, a_vis, a_safe)


class FakeCP:
  def __init__(self, fp, brand, oplong=False):
    self.carFingerprint, self.brand, self.openpilotLongitudinalControl = fp, brand, oplong


@pytest.fixture
def default_curve_cfg(tmp_path, monkeypatch):
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(tmp_path / "absent.json"))


def _lightning():
  return PnwVehicle(FakeCP("FORD_F_150_LIGHTNING_MK1", "ford"))


# ---------------------------------------------------------------------------------------------------------------
# The predicate
# ---------------------------------------------------------------------------------------------------------------
class TestThePredicate:
  def test_tumwater_first_restore_tick_holds_on_the_polyline_and_gives_back_up_to_70(self):
    hold, a, src, v_safe = hold_fn(33.53, 0.002557, 12, True, 0.00169)
    assert hold and src == "poly" and a == pytest.approx(2.875, abs=0.01)
    assert v_safe == pytest.approx(math.sqrt(2.5 / 0.002557)) and v_safe / MPH == pytest.approx(69.9, abs=0.1)

  def test_the_polyline_alone_needs_2_8_not_2_5(self):
    """(b): 2.6 m/s^2 on the polyline alone no longer holds; it did at the first cut's 2.5."""
    k = 2.6 / 33.53 ** 2
    assert hold_fn(33.53, k, 12, True, None)[0] is False
    assert hold_fn(33.53, 2.8 / 33.53 ** 2 * 1.001, 12, True, None)[0] is True

  def test_vision_alone_holds_at_2_5(self):
    hold, a, src, v_safe = hold_fn(33.53, 0.0, 12, True, 0.00226)   # 21:21:51's vision reading
    assert hold and src == "vis" and a == pytest.approx(2.54, abs=0.01)
    assert v_safe == pytest.approx(math.sqrt(2.5 / 0.00226))

  def test_both_firing_is_reported_and_the_safe_speed_uses_the_tighter_curvature(self):
    hold, a, src, v_safe = hold_fn(33.53, 0.0030, 12, True, 0.0026)
    assert hold and src == "poly+vis" and v_safe == pytest.approx(math.sqrt(2.5 / 0.0030))

  def test_vision_can_set_the_safe_speed_when_only_the_polyline_fired(self):
    hold, _, src, v_safe = hold_fn(33.53, 0.0026, 12, True, 0.0022)
    assert hold and src == "poly" and v_safe == pytest.approx(math.sqrt(2.5 / 0.0026))
    hold, _, src, v_safe = hold_fn(33.53, 0.0026, 12, True, 0.0021 + 0.0009)    # vis 0.0030 fires too, tighter
    assert src == "poly+vis" and v_safe == pytest.approx(math.sqrt(2.5 / 0.0030))

  def test_a_gentle_road_does_not_hold(self):
    hold, a, _, v_safe = hold_fn(33.53, 0.001, 12, True, 0.0012)
    assert not hold and a < 2.5 and v_safe is None

  def test_a_polyline_max_BEHIND_the_truck_is_ignored(self):
    """The curve the restore follows is behind the truck; it must not hold its own restore."""
    assert hold_fn(33.53, 0.005, 12, False, None) == (False, None, "noGeom", None)

  def test_an_unmeasurable_polyline_is_not_used(self):
    assert hold_fn(33.53, 0.005, 0, True, None) == (False, None, "noGeom", None)

  def test_the_polyline_bar_at_zero_turns_only_the_polyline_off(self):
    assert hold_fn(33.53, 0.005, 12, True, None, a_poly=0.0) == (False, None, "noGeom", None)
    assert hold_fn(33.53, 0.005, 12, True, 0.003, a_poly=0.0)[:3] == (True, pytest.approx(0.003 * 33.53 ** 2), "vis")

  @pytest.mark.parametrize("a_hold", [0.0, -1.0, None, float("nan"), "x"])
  def test_off(self, a_hold):
    assert hold_fn(33.53, 0.005, 12, True, 0.005, a_poly=a_hold, a_vis=a_hold) == (False, None, "off", None)

  @pytest.mark.parametrize("v", [0.0, -3.0, None, float("inf"), "x"])
  def test_bad_target_speed(self, v):
    assert hold_fn(v, 0.005, 12, True, 0.005) == (False, None, "badInput", None)

  @pytest.mark.parametrize("junk", [None, float("nan"), float("inf"), "x", True])
  def test_junk_curvature_never_raises_and_never_holds_by_itself(self, junk):
    assert hold_fn(33.53, junk, 12, True, junk)[0] is False

  @pytest.mark.parametrize("a_safe", [0.0, None, float("nan")])
  def test_an_unusable_safe_speed_target_holds_at_the_current_speed(self, a_safe):
    hold, _, _, v_safe = hold_fn(33.53, 0.003, 12, True, None, a_safe=a_safe)
    assert hold and v_safe is None

  @pytest.mark.parametrize("src,bar", [("poly", A_POLY), ("vis", A_VIS)])
  def test_each_bar_is_inclusive_and_absolute(self, src, bar):
    """Mutation guard: pins the comparison and the v^2 of each source, not just 'some' threshold."""
    k = bar / 30.0 ** 2
    def f(v, kk):
      return hold_fn(v, kk, 3, True, None)[0] if src == "poly" else hold_fn(v, None, 0, True, kk)[0]
    assert f(30.0, k * 1.001) is True
    assert f(30.0, k * 0.999) is False
    assert f(15.0, k * 1.001) is False       # a quarter of the load at half the speed


# ---------------------------------------------------------------------------------------------------------------
# Closed-loop harness: brain -> executor (1 mph taps, 0.4 s cadence, 0.6 mph deadband) -> stock ACC (set follows
# taps, speed follows set). Counts SET- taps, which is the owner's "slowdown" currency. `lag_s` delays the set the
# truck REPORTS (to brain and executor alike), as the real one does by ~1-1.25 s.
# ---------------------------------------------------------------------------------------------------------------
def simulate(plan, set0=75 * MPH, v0=75 * MPH, window_s=ICBM_RESTORE_WINDOW_S, a_up=0.6, a_dn=0.8, lag_s=0.0,
             lag_from=0):
  """plan: per 0.25 s tick, (cap m/s or None, hold_ahead) or (cap, hold_ahead, v_safe m/s or None). A planned cap only
  reaches the episode when it would BIND -- target at least ICBM_MIN_DROP_MS below the reference the brain judges it
  against (the episode's bind_ceiling while capping, else the current set), exactly as icbm_curve_target's reduce-only
  test does -- so a plan can never feed the machine a cap the real brain could not produce. The reported set lags by
  lag_s from tick `lag_from` on (Fable's inflight_sim.py lags only the restore)."""
  ep = IcbmEpisode(window_s=window_s)
  real, v, t, next_tap = set0, v0, 0.0, 0.0
  hist = []
  out = {"dec_taps": 0, "inc_taps": 0, "set": [], "v": [], "pub": [], "phase": [], "ceiling": [], "ep": ep}
  for step in plan:
    cap, hold = step[0], step[1]
    vsafe = step[2] if len(step) > 2 else None
    lagged = lag_s > 0 and len(hist) >= lag_from
    rep = next((x for (tt, x) in reversed(hist) if tt <= t - lag_s), real) if lagged else real
    if cap is not None:
      ref = ep.bind_ceiling if (ep.phase == "cap" and ep.ceiling is not None) else rep
      if not cap < ref - m.ICBM_MIN_DROP_MS:
        cap = None
    pub, d = ep.step(t, cap, rep, rep, True, False, v_ego=v, hold_ahead=hold, hold_vsafe=vsafe)
    out["pub"].append((pub, d))
    out["phase"].append(ep.phase)
    out["ceiling"].append(ep.ceiling)
    if pub is not None and t >= next_tap:
      tgt = min(pub, ep.ceiling) if (d == "inc" and ep.ceiling is not None) else pub
      if d == "dec" and rep > tgt + DEADBAND:
        real -= MPH
        out["dec_taps"] += 1
        next_tap = t + TAP_S
      elif d == "inc" and rep < tgt - DEADBAND:
        real += MPH
        out["inc_taps"] += 1
        next_tap = t + TAP_S
    v = min(v + a_up * DT, real) if v < real else max(v - a_dn * DT, real)
    hist.append((t, real))
    out["set"].append(real)
    out["v"].append(v)
    t += DT
  return out


def _sbend_plan(hold_ticks, clear_ticks=60, cap_mph=64.0, vsafe=None):
  """75 mph cruise, a curve caps the set to 64 for 8 s, then it clears; hold_ticks of 'curve ahead' follow."""
  plan = [(cap_mph * MPH, False)] * 32
  plan += [(None, True, vsafe)] * hold_ticks
  plan += [(None, False)] * clear_ticks
  return plan


class TestTheSbend:
  def test_without_the_hold_the_restore_accelerates_into_the_next_curve(self):
    tr = simulate(_sbend_plan(0))
    i = 32 + 40                                   # 10 s after the clear (3 s restore delay, then the climb)
    assert tr["v"][i] > 70 * MPH, "the pre-fix restore must be reproduced by the harness"

  def test_the_hold_keeps_the_truck_at_the_speed_it_had(self):
    base, held = simulate(_sbend_plan(0)), simulate(_sbend_plan(60))
    span = range(32 + int(ICBM_RESTORE_DELAY_S / DT) + 2, 32 + 60)
    assert max(held["v"][i] for i in span) <= 64.5 * MPH
    assert max(base["v"][i] for i in span) > 72 * MPH

  def test_zero_new_slowdowns(self):
    """THE OWNER'S CONSTRAINT: identical SET- taps and identical minimum speed, hold or not."""
    base, held = simulate(_sbend_plan(0)), simulate(_sbend_plan(60))
    assert held["dec_taps"] == base["dec_taps"] == 11
    assert min(held["v"]) == pytest.approx(min(base["v"]))
    assert min(held["set"]) == pytest.approx(min(base["set"]))

  def test_the_restore_resumes_after_the_curve(self):
    held = simulate(_sbend_plan(60, clear_ticks=120))
    assert held["set"][-1] == pytest.approx(75 * MPH)
    first_inc = next(i for i, (p, d) in enumerate(held["pub"]) if d == "inc")
    # the hold ended at tick 32+60; the resume waits ICBM_RESTORE_HOLD_CLEAR_S, not the full restore delay again
    assert first_inc >= 32 + 60 + int(ICBM_RESTORE_HOLD_CLEAR_S / DT) - 1
    assert first_inc <= 32 + 60 + int(ICBM_RESTORE_HOLD_CLEAR_S / DT) + 2

  def test_a_one_tick_clear_does_not_release_the_hold(self):
    plan = [(64 * MPH, False)] * 32 + [(None, True)] * 30 + [(None, False)] + [(None, True)] * 30 + [(None, False)] * 4
    tr = simulate(plan)
    assert tr["inc_taps"] == 0 and max(tr["set"][32:]) == pytest.approx(64 * MPH)

  def test_the_hold_is_bounded_by_the_restore_window_and_leaves_the_set_where_it_held(self):
    tr = simulate(_sbend_plan(400, clear_ticks=40))            # 100 s of "curve ahead"
    t_end = next(i for i in range(32, len(tr["phase"])) if i > 40 and tr["phase"][i] == "idle")
    assert (t_end - 32) * DT <= ICBM_RESTORE_DELAY_S + ICBM_RESTORE_WINDOW_S + 1.0
    assert tr["inc_taps"] == 0 and tr["dec_taps"] == 11
    assert tr["set"][-1] == pytest.approx(64 * MPH)            # nothing restored after expiry, nothing lowered

  def test_a_truck_still_slowing_is_held_at_its_speed_not_dragged_to_the_set(self):
    """Hold = the CURRENT speed. A plain 'stay silent' would let the ACC finish slowing to the tapped-down set --
    a decel the restore would not have caused."""
    ep = IcbmEpisode()
    ep.step(0.0, 60 * MPH, 75 * MPH, 75 * MPH, True, False)
    ep.step(1.0, None, 60 * MPH, 60 * MPH, True, False, v_ego=66 * MPH)
    pub, d = ep.step(1.0 + ICBM_RESTORE_DELAY_S + 0.1, None, 60 * MPH, 60 * MPH, True, False,
                     v_ego=66 * MPH, hold_ahead=True)
    assert d == "inc" and pub == pytest.approx(66 * MPH) and ep.ahead_cap == pytest.approx(66 * MPH)

  def test_the_latch_rounds_up_to_the_tap_grid_so_the_truck_never_settles_lower(self):
    """Replay 2026-09-17 18:31:53: set 51, vEgo 51.2. A latch of exactly 51.2 sits inside the executor's 0.6 mph
    deadband, no tap happens, and the ACC settles to 51.0 -- 0.2 mph of decel the restore would not have caused.
    The latch rounds up to 52 instead."""
    plan = [(51 * MPH, False)] * 40 + [(None, True)] * 60
    tr = simulate(plan, set0=65 * MPH, v0=65 * MPH)
    i_entry = next(i for i, ph in enumerate(tr["phase"]) if ph == "restore")
    v_entry = tr["v"][i_entry - 1]
    assert min(tr["v"][i_entry:]) >= v_entry - 1e-9, "the hold let the truck slow below its speed at restore entry"
    ep = IcbmEpisode()
    ep.step(0.0, 51 * MPH, 65 * MPH, 65 * MPH, True, False)
    ep.step(1.0, None, 51 * MPH, 51 * MPH, True, False)
    pub, d = ep.step(1.0 + ICBM_RESTORE_DELAY_S + 0.1, None, 51 * MPH, 51 * MPH, True, False,
                     v_ego=51.2 * MPH, hold_ahead=True)
    assert ep.ahead_cap == pytest.approx(52 * MPH) and d == "inc" and pub == pytest.approx(52 * MPH)

  def test_the_hold_latch_never_creeps_with_speed(self):
    ep = IcbmEpisode()
    ep.step(0.0, 60 * MPH, 75 * MPH, 75 * MPH, True, False)
    ep.step(1.0, None, 60 * MPH, 60 * MPH, True, False)
    t = 1.0 + ICBM_RESTORE_DELAY_S + 0.1
    assert ep.step(t, None, 60 * MPH, 60 * MPH, True, False, v_ego=60 * MPH, hold_ahead=True) == (None, None)
    for i in range(1, 20):                                     # vEgo drifts up downhill: the latch must not follow
      assert ep.step(t + i * DT, None, 60 * MPH, 60 * MPH, True, False, v_ego=(60 + i) * MPH,
                     hold_ahead=True) == (None, None)
    assert ep.ahead_cap == pytest.approx(60 * MPH)

  def test_a_new_cap_during_the_hold_is_untouched(self):
    """DEC always wins, exactly as before: a curve that binds during the hold caps with the SAME target on the SAME
    ticks. It needs FEWER SET- taps to get there, because the held set never climbed (the 09-23 21:10 replay case),
    and the truck ends at the same set."""
    plan = [(64 * MPH, False)] * 32 + [(None, True)] * 30 + [(55 * MPH, True)] * 60
    base = simulate([(c, False) for c, _ in plan])
    held = simulate(plan)
    assert held["pub"][-60:] == base["pub"][-60:]
    assert held["dec_taps"] < base["dec_taps"]
    assert held["set"][-1] == pytest.approx(base["set"][-1]) == pytest.approx(55 * MPH)
    assert min(held["v"]) >= min(base["v"]) - 1e-9

  def test_the_hold_latches_and_debounces_during_in_curve_ticks_too(self):
    """Mutation M9: the hold's bookkeeping must run on the ticks the in-curve pause swallows. Otherwise the latch
    waits for the first unloaded tick -- by which time the truck may have sped up -- and the clear debounce has not
    started when the load ends, so the restore resumes later than designed."""
    ep = IcbmEpisode()
    ep.step(0.0, 60 * MPH, 75 * MPH, 75 * MPH, True, False)
    ep.step(1.0, None, 60 * MPH, 60 * MPH, True, False)
    t = 1.0 + ICBM_RESTORE_DELAY_S + 0.1
    assert ep.step(t, None, 60 * MPH, 60 * MPH, True, False, v_ego=60 * MPH)[1] == "inc"     # restore running
    t += DT
    assert ep.step(t, None, 60 * MPH, 60 * MPH, True, False, v_ego=61 * MPH, in_curve=True, hold_ahead=True) \
      == (None, None)
    assert ep.ahead_cap == pytest.approx(61 * MPH), "latched on the in-curve tick itself"
    t += DT
    ep.step(t, None, 60 * MPH, 60 * MPH, True, False, v_ego=61 * MPH, in_curve=True, hold_ahead=False)
    t += ICBM_RESTORE_HOLD_CLEAR_S + 0.01                   # still loaded: the clear debounce ran meanwhile
    ep.step(t, None, 60 * MPH, 60 * MPH, True, False, v_ego=61 * MPH, in_curve=True, hold_ahead=False)
    assert ep.ahead_cap is None
    pub, d = ep.step(t + DT, None, 60 * MPH, 60 * MPH, True, False, v_ego=61 * MPH)
    assert d == "inc" and pub == pytest.approx(75 * MPH)

  def test_a_ahead_the_restore_still_rises_to_the_curves_safe_speed(self):
    """(a) owner 2026-09-24, "resume as fast as possible": held at 64 with the curve's safe speed at 70, the restore
    gives back 64 -> 70 before the curve, holds there, and the rest after it."""
    tr = simulate(_sbend_plan(80, clear_ticks=80, vsafe=70 * MPH))
    span = range(32 + int(ICBM_RESTORE_DELAY_S / DT) + 2, 32 + 80)
    assert max(tr["set"][i] for i in span) == pytest.approx(70 * MPH)
    assert tr["set"][32 + 79] == pytest.approx(70 * MPH)
    assert tr["set"][-1] == pytest.approx(75 * MPH) and tr["dec_taps"] == 11

  def test_a_safe_speed_below_the_current_speed_never_parks_the_set_lower(self):
    tr = simulate(_sbend_plan(60, vsafe=58 * MPH))
    i_entry = next(i for i, ph in enumerate(tr["phase"]) if ph == "restore")
    assert min(tr["set"][i_entry:]) == pytest.approx(64 * MPH) and min(tr["v"][i_entry:]) >= tr["v"][i_entry - 1] - 1e-9
    assert tr["dec_taps"] == 11

  def test_a_safe_speed_below_a_still_slowing_truck_holds_the_truck_not_the_curve(self):
    """Set 60 but the truck still at 66, curve safe at 58: hold at 66 (one tap grid), never let the ACC finish slowing
    to 60 because the curve's safe speed is lower -- that would be a decel, and the owner forbids any."""
    ep = IcbmEpisode()
    ep.step(0.0, 60 * MPH, 75 * MPH, 75 * MPH, True, False)
    ep.step(1.0, None, 60 * MPH, 60 * MPH, True, False, v_ego=66 * MPH)
    pub, d = ep.step(1.0 + ICBM_RESTORE_DELAY_S + 0.1, None, 60 * MPH, 60 * MPH, True, False,
                     v_ego=66 * MPH, hold_ahead=True, hold_vsafe=58 * MPH)
    assert d == "inc" and pub == pytest.approx(66 * MPH)

  def test_the_safe_speed_only_ever_tightens_during_a_hold(self):
    """A noisy reading cannot raise the cap tick by tick: the hold keeps the LOWEST safe speed it has seen."""
    ep = IcbmEpisode()
    ep.step(0.0, 60 * MPH, 75 * MPH, 75 * MPH, True, False)
    ep.step(1.0, None, 60 * MPH, 60 * MPH, True, False)
    t = 1.0 + ICBM_RESTORE_DELAY_S + 0.1
    ep.step(t, None, 60 * MPH, 60 * MPH, True, False, v_ego=60 * MPH, hold_ahead=True, hold_vsafe=66 * MPH)
    assert ep.ahead_limit() == pytest.approx(66 * MPH)
    pub, d = ep.step(t + DT, None, 60 * MPH, 60 * MPH, True, False, v_ego=60 * MPH, hold_ahead=True,
                     hold_vsafe=72 * MPH)
    assert ep.ahead_limit() == pytest.approx(66 * MPH) and d == "inc" and pub == pytest.approx(66 * MPH)

  def test_c_a_curve_binding_during_a_hold_does_not_stick_the_ceiling(self):
    """(c) and Fable point 4: a cap that interrupts a HELD restore used to re-latch the ceiling at the held set (64),
    so after that curve the set stayed at 64 for good. It must go back to the driver's 75. Fails on 7aade6a326."""
    plan = [(64 * MPH, False)] * 32 + [(None, True)] * 30 + [(60 * MPH, True)] * 40 + [(None, False)] * 120
    tr = simulate(plan)
    assert tr["set"][-1] == pytest.approx(75 * MPH), "the restore stuck below the driver's original set"
    i_cap2 = 32 + 30
    assert tr["ceiling"][i_cap2] == pytest.approx(75 * MPH)
    assert tr["ep"].ceiling is None       # completed, not abandoned

  def test_c_the_carried_cap_binds_exactly_as_the_relatched_one_did(self):
    """The carry keeps the ceiling for the RESTORE only; candidates are judged against the interrupted set, so which
    curves bind -- and so every SET- -- is what it was."""
    ep = IcbmEpisode()
    ep.step(0.0, 64 * MPH, 75 * MPH, 75 * MPH, True, False)
    ep.step(1.0, None, 64 * MPH, 64 * MPH, True, False)
    t = 1.0 + ICBM_RESTORE_DELAY_S + 0.1
    ep.step(t, None, 64 * MPH, 64 * MPH, True, False, v_ego=64 * MPH, hold_ahead=True)
    pub, d = ep.step(t + DT, 60 * MPH, 64 * MPH, 64 * MPH, True, False, v_ego=64 * MPH, hold_ahead=True)
    assert d == "dec" and pub == pytest.approx(60 * MPH)
    assert ep.ceiling == pytest.approx(75 * MPH) and ep.bind_ceiling == pytest.approx(64 * MPH)

  def test_c_the_driver_moving_the_set_during_the_hold_still_wins(self):
    ep = IcbmEpisode()
    ep.step(0.0, 64 * MPH, 75 * MPH, 75 * MPH, True, False)
    ep.step(1.0, None, 64 * MPH, 64 * MPH, True, False)
    t = 1.0 + ICBM_RESTORE_DELAY_S + 0.1
    ep.step(t, None, 64 * MPH, 64 * MPH, True, False, v_ego=64 * MPH, hold_ahead=True)
    ep.step(t + DT, 58 * MPH, 62 * MPH, 62 * MPH, True, False, v_ego=64 * MPH, hold_ahead=True)   # driver SET- x2
    assert ep.ceiling == pytest.approx(62 * MPH) and ep.bind_ceiling == pytest.approx(62 * MPH)

  def test_c_an_unheld_restore_keeps_its_documented_relatch(self):
    """Scope: only a HELD restore carries its ceiling. A cap during an ordinary restore re-latches at the current set,
    as icbmrestore2pnw has always done (test_icbm_restore.test_dec_beats_inc_new_cap_cancels_restore_and_relatches_lower)."""
    ep = IcbmEpisode()
    ep.step(0.0, 64 * MPH, 75 * MPH, 75 * MPH, True, False)
    ep.step(1.0, None, 64 * MPH, 64 * MPH, True, False)
    t = 1.0 + ICBM_RESTORE_DELAY_S + 0.1
    ep.step(t, None, 64 * MPH, 64 * MPH, True, False, v_ego=64 * MPH)
    ep.step(t + DT, 60 * MPH, 65 * MPH, 65 * MPH, True, False, v_ego=64 * MPH)
    assert ep.ceiling == pytest.approx(65 * MPH)

  def test_f1_a_hold_starting_mid_climb_survives_our_own_late_taps(self):
    """Fable F1 (its inflight_sim.py): 1.25 s set-report lag, the hold arriving 4 s into the climb. Our in-flight SET+
    taps land after the at-cap snapshot; the old one-tap tolerance read them as the driver and killed the restore at
    +7.75 s, stuck above the latch until the driver raised the set. It must hold, then complete."""
    plan = [(64 * MPH, False)] * 32 + [(None, False)] * 28 + [(None, True)] * 40 + [(None, False)] * 100
    tr = simulate(plan, lag_s=1.25, lag_from=32)
    assert all(ph != "idle" for ph in tr["phase"][32 + 28:32 + 68]), "the late taps aborted the held restore"
    # it completes exactly as an unheld restore does under the same lag (this harness's lagged executor overshoots
    # every restore by ~2 taps, hold or not -- see TUMWATER-RESTORE-FIX.md, "not sure")
    base = simulate([(c, False) for c, *_ in plan], lag_s=1.25, lag_from=32)
    assert tr["set"][-1] == pytest.approx(base["set"][-1]) and tr["dec_taps"] == base["dec_taps"] == 11

  def test_f1_more_than_our_in_flight_taps_is_still_the_driver(self):
    ep = IcbmEpisode()
    ep.step(0.0, 60 * MPH, 75 * MPH, 75 * MPH, True, False)
    ep.step(1.0, None, 60 * MPH, 60 * MPH, True, False)
    t = 1.0 + ICBM_RESTORE_DELAY_S + 0.1
    assert ep.step(t, None, 60 * MPH, 60 * MPH, True, False, v_ego=60 * MPH)[1] == "inc"
    assert ep.step(t + DT, None, 61 * MPH, 61 * MPH, True, False, v_ego=60 * MPH, hold_ahead=True) == (None, None)
    ep.step(t + 2 * DT, None, 63 * MPH, 63 * MPH, True, False, v_ego=60 * MPH, hold_ahead=True)
    assert ep.phase == "restore"                               # late taps of ours
    ep.step(t + 3 * DT, None, 65 * MPH, 65 * MPH, True, False, v_ego=60 * MPH, hold_ahead=True)
    assert ep.phase == "restore"                               # 4 over the snapshot: still within our in-flight taps
    # +2 per tick never trips the fast-rise guard (2.2 taps per 0.25 s); only the late-tap band can catch this
    ep.step(t + 4 * DT, None, 67 * MPH, 67 * MPH, True, False, v_ego=60 * MPH, hold_ahead=True)
    assert ep.phase == "idle"                                  # 6 over the snapshot, inside the grace: the driver

  def test_f1_after_the_grace_the_strict_tolerance_is_back(self):
    ep = IcbmEpisode()
    ep.step(0.0, 60 * MPH, 75 * MPH, 75 * MPH, True, False)
    ep.step(1.0, None, 60 * MPH, 60 * MPH, True, False)
    t = 1.0 + ICBM_RESTORE_DELAY_S + 0.1
    ep.step(t, None, 60 * MPH, 60 * MPH, True, False, v_ego=60 * MPH)
    ep.step(t + DT, None, 61 * MPH, 61 * MPH, True, False, v_ego=60 * MPH, hold_ahead=True)
    ep.step(t + 2.0, None, 63 * MPH, 63 * MPH, True, False, v_ego=60 * MPH, hold_ahead=True)   # re-anchored at 63
    assert ep.phase == "restore"
    ep.step(t + 3.0, None, 65 * MPH, 65 * MPH, True, False, v_ego=60 * MPH, hold_ahead=True)   # 2 taps later: driver
    assert ep.phase == "idle"

  def test_the_zone_cap_still_bounds_the_hold_latch(self):
    """Mutation M10: a truck held at 66 in a zone capped at 62 must not be raised to 66."""
    ep = IcbmEpisode()
    ep.step(0.0, 55 * MPH, 75 * MPH, 75 * MPH, True, False)
    ep.step(1.0, None, 60 * MPH, 60 * MPH, True, False)
    pub, d = ep.step(1.0 + ICBM_RESTORE_DELAY_S + 0.1, None, 60 * MPH, 60 * MPH, True, False,
                     v_ego=66 * MPH, hold_ahead=True, restore_cap=62 * MPH)
    assert d == "inc" and pub == pytest.approx(62 * MPH)

  def test_the_zone_cap_still_bounds_a_resumed_restore(self):
    ep = IcbmEpisode()
    ep.step(0.0, 40 * MPH, 60 * MPH, 60 * MPH, True, False)
    ep.step(1.0, None, 40 * MPH, 40 * MPH, True, False)
    t = 1.0 + ICBM_RESTORE_DELAY_S + 0.1
    ep.step(t, None, 40 * MPH, 40 * MPH, True, False, v_ego=40 * MPH, hold_ahead=True, restore_cap=50 * MPH)
    t += DT
    assert ep.step(t, None, 40 * MPH, 40 * MPH, True, False, v_ego=40 * MPH, restore_cap=50 * MPH) == (None, None)
    pub, d = ep.step(t + ICBM_RESTORE_HOLD_CLEAR_S + 0.01, None, 40 * MPH, 40 * MPH, True, False, v_ego=40 * MPH,
                     restore_cap=50 * MPH)
    assert d == "inc" and pub == pytest.approx(50 * MPH)


class TestNoNewSlowdownsFuzz:
  """Decision level, identical inputs to both machines: every dec either machine publishes, the other publishes
  identically -- the hold (and the ceiling it carries) never changes a cap. Every inc the held machine publishes is
  at or below its ceiling, which is always a set the driver had. Closed loop: the set only ever falls on a tick fed a
  cap, never below that cap's target, and every cap takes the truck exactly as low as it does without the hold."""
  @pytest.mark.parametrize("seed", range(40))
  def test_open_loop_decisions(self, seed):
    rnd = random.Random(seed)
    a, b = IcbmEpisode(), IcbmEpisode()
    stock, t, driver_sets = 75 * MPH, 0.0, {75 * MPH}
    for _ in range(600):
      r = rnd.random()
      cap = rnd.uniform(35, 74) * MPH if r < 0.25 else None
      stock = min(max(stock + rnd.choice((0.0, 0.0, 0.0, MPH, -MPH)), 20 * MPH), 80 * MPH)
      driver_sets.add(stock)
      pedal = rnd.random() < 0.01
      v = stock + rnd.uniform(-3, 3) * MPH
      vs = rnd.choice((None, rnd.uniform(30, 80) * MPH))
      pa = a.step(t, cap, stock, stock, True, pedal, v_ego=v)
      pb = b.step(t, cap, stock, stock, True, pedal, v_ego=v, hold_ahead=rnd.random() < 0.5, hold_vsafe=vs)
      if pb[1] == "dec" or pa[1] == "dec":
        assert pb == pa, f"seed {seed} t {t}: {pa} vs {pb}"
      if pb[1] == "inc":                    # F2: the old `A and B or C` was vacuous whenever pa was idle
        assert b.ceiling is not None and pb[0] <= b.ceiling + 1e-9
        assert any(abs(b.ceiling - x) < 1e-9 for x in driver_sets), "restored toward a set the driver never had"
      t += DT

  @pytest.mark.parametrize("seed", range(40))
  def test_closed_loop_no_new_slowdown(self, seed):
    rnd = random.Random(1000 + seed)
    plan, segs = [], []
    for _ in range(12):
      cap = rnd.uniform(40, 72) * MPH
      n = rnd.randint(8, 40)
      segs.append((len(plan), len(plan) + n, cap))
      plan += [(cap, False)] * n
      h = rnd.randint(0, 120)
      vs = rnd.choice((None, rnd.uniform(40, 80) * MPH))
      plan += [(None, True, vs)] * h + [(None, False)] * rnd.randint(4, 80)
    base = simulate([(c, False) for c, *_ in plan])
    held = simulate(plan)
    for run in (base, held):              # the same rule for both: the hold keeps a property the brain already had
      for i in range(1, len(plan)):
        if run["set"][i] < run["set"][i - 1] - 1e-9:
          assert plan[i][0] is not None, f"seed {seed} tick {i}: the set fell with no cap fed"
          assert run["set"][i] >= plan[i][0] - DEADBAND - 1e-9
    # NOT asserted: an equal per-cap floor. A held truck starts a cap LOWER, so within a short cap it can reach the
    # cap's own target where the unheld one, tapping down from higher, runs out of cap first (seed 0). It never goes
    # below the target -- asserted per tick above -- which is what the brain commanded in both runs.
    assert segs

  @pytest.mark.parametrize("seed", range(40))
  def test_the_restore_never_sticks_below_the_original_ceiling(self, seed):
    """Curves that bind DURING held restores, over and over: once the road finally clears, the set is the driver's
    original 75 -- never the speed some hold happened to be at when the next curve bound."""
    rnd = random.Random(5000 + seed)
    plan = [(rnd.uniform(45, 70) * MPH, False)] * rnd.randint(16, 40)
    for _ in range(rnd.randint(1, 5)):
      plan += [(None, False)] * (int(ICBM_RESTORE_DELAY_S / DT) + 2)
      plan += [(None, True, rnd.choice((None, rnd.uniform(50, 74) * MPH)))] * rnd.randint(4, 40)
      plan += [(rnd.uniform(45, 70) * MPH, True)] * rnd.randint(8, 40)
    plan += [(None, False)] * 240
    tr = simulate(plan)
    assert tr["set"][-1] == pytest.approx(75 * MPH), f"seed {seed}: stuck at {tr['set'][-1] / MPH:.1f}"


# ---------------------------------------------------------------------------------------------------------------
# The logged Tumwater ticks
# ---------------------------------------------------------------------------------------------------------------
class TestTheLoggedTumwaterTicks:
  @pytest.fixture(scope="class")
  def ticks(self):
    with open(FIXTURE) as f:
      return json.load(f)["ticks"]

  def test_every_logged_restore_tick_would_have_held(self, ticks):
    restore = [r for r in ticks if r["icbmPhase"] == "restore"]
    assert len(restore) == 6, "fixture drifted: 21:21:49-54 are the six restore ticks"
    ceiling = max(r["icbmT"] for r in restore if r["icbmDir"] == "inc")
    assert ceiling == pytest.approx(75 * MPH, abs=0.1)
    for r in restore:
      hold, a, src, _ = hold_fn(ceiling, r["icbmK"], r["icbmKN"], r["icbmKAhead"], r["visKMax"])
      assert hold, f"{r['t']}: a={a} src={src}"

  def test_the_polyline_catches_the_first_tick_alone_and_vision_joins_two_seconds_later(self, ticks):
    """(b)'s thin margin, stated plainly: at 21:21:49 only the polyline fires (2.87 vs its 2.8 bar); vision reads 1.90.
    Vision first clears its 2.5 bar at 21:21:51 (2.54), with the set then at 67."""
    restore = [r for r in ticks if r["icbmPhase"] == "restore"]
    srcs = [hold_fn(33.53, r["icbmK"], r["icbmKN"], r["icbmKAhead"], r["visKMax"])[2] for r in restore]
    assert srcs == ["poly", "poly", "poly+vis", "poly", "poly", "poly"]

  def test_replayed_through_the_episode_the_restore_stops_at_70(self, ticks):
    """Feed the machine the logged state at restore entry (set 64.0, vEgo 64.4, ceiling 75) and the logged
    predicate on each tick. With the hold nothing above the left curve's safe speed at 2.5 m/s^2 on the polyline
    (69.9 mph) is ever published, instead of the 75 the logged restore walked to."""
    ep = IcbmEpisode()
    ep.step(0.0, 64 * MPH, 75 * MPH, 75 * MPH, True, False)
    ep.step(1.0, None, 64 * MPH, 64 * MPH, True, False)
    t = 1.0 + ICBM_RESTORE_DELAY_S + 0.1
    restore = [r for r in ticks if r["icbmPhase"] == "restore"]
    for r in restore:
      hold, _, _, vs = hold_fn(ep.ceiling, r["icbmK"], r["icbmKN"], r["icbmKAhead"], r["visKMax"])
      pub, d = ep.step(t, None, 64 * MPH, 64 * MPH, True, False, v_ego=r["vEgo"], hold_ahead=hold, hold_vsafe=vs)
      assert d is None or pub <= 69.95 * MPH
      t += 1.0


# ---------------------------------------------------------------------------------------------------------------
# Capability view: the Lightning has it, nothing else does, and a bad config is clamped
# ---------------------------------------------------------------------------------------------------------------
class TestTheCapability:
  def test_lightning_on_every_other_car_off(self, default_curve_cfg):
    assert _lightning().icbm_restore_hold_lat_accel == 2.5
    assert _lightning().icbm_restore_hold_poly_lat_accel == 2.8
    tesla = PnwVehicle(FakeCP("TESLA_MODEL_S_HW3", "tesla", True))
    assert tesla.icbm_restore_hold_lat_accel == 0.0 and tesla.icbm_restore_hold_poly_lat_accel == 0.0
    assert PnwVehicle(None).icbm_restore_hold_lat_accel == 0.0
    assert PnwVehicle(None).icbm_restore_hold_poly_lat_accel == 0.0
    assert icbm_restore_ahead_hold(33.5, 0.01, 9, True, 0.01, tesla.icbm_restore_hold_poly_lat_accel,
                                   tesla.icbm_restore_hold_lat_accel, tesla.icbm_restore_hold_lat_accel) \
      == (False, None, "off", None)

  def test_a_bad_config_is_clamped(self, tmp_path, monkeypatch):
    cfg = tmp_path / "curve.json"
    cfg.write_text(json.dumps({"lightning": {"icbm_restore_hold_lat_accel": 9.0,
                                             "icbm_restore_hold_poly_lat_accel": -4.0}}))
    monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(cfg))
    assert _lightning().icbm_restore_hold_lat_accel == 3.2
    assert _lightning().icbm_restore_hold_poly_lat_accel == 0.0

  def test_default_input_is_byte_identical(self):
    """hold_ahead defaults to False: every existing caller keeps its exact behavior."""
    plan = _sbend_plan(0)
    a = simulate(plan)
    b = simulate([(c, False) for c, *_ in plan])
    for k in ("dec_taps", "inc_taps", "set", "v", "pub", "phase", "ceiling"):
      assert a[k] == b[k]


# ---------------------------------------------------------------------------------------------------------------
# _icbm_step plumbing: the controller wires the predicate in, logs, and the telemetry reaches the record
# ---------------------------------------------------------------------------------------------------------------
def _stub_mgr(tmp_path, monkeypatch, fp="FORD_F_150_LIGHTNING_MK1", brand="ford"):
  import inspect
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(tmp_path / "nope.json"))
  cls = next(o for o in vars(m).values() if inspect.isclass(o) and hasattr(o, "_icbm_step"))

  class FakeMem:
    def put_nonblocking(self, k, v): self.last = v

  class Stub:
    pass
  mgr = Stub()
  mgr.mem_params = FakeMem()
  mgr._icbm_ceiling = None
  mgr._veh = PnwVehicle(FakeCP(fp, brand))
  mgr._map_targets = []
  mgr._cur_lat = mgr._cur_lon = mgr._cur_bearing = None
  mgr._icbm_ep = m.IcbmEpisode(clear_delay_s=0.0)
  mgr._icbm_dir = None
  mgr._icbm_floor_lim, mgr._icbm_floor_pend, mgr._icbm_floor_hit = 0.0, None, False
  mgr._icbm_k, mgr._icbm_k_n, mgr._icbm_k_ahead = 0.0, 0, False
  mgr._icbm_k_at, mgr._icbm_k_at_d, mgr._icbm_k_at_n, mgr._icbm_k_at_gap = 0.0, 0.0, 0, 0.0
  return mgr, cls._icbm_step.__get__(mgr)


def test_icbm_step_holds_the_restore_and_logs_it(tmp_path, monkeypatch):
  import time as _t
  events = []
  monkeypatch.setattr(m.cloudlog, "event", lambda name, **kw: events.append((name, kw)))
  mgr, step = _stub_mgr(tmp_path, monkeypatch)

  def run(sig, stock_set):
    mgr._stock_set, mgr._stock_on = stock_set, True
    mgr._icbm_last_pub = _t.monotonic() - 1.0
    step(sig, active=True)
    return mgr.mem_params.last

  cap_sig = {"v_ego": 75 * MPH, "v_set": 75 * MPH, "map_target_v": 40 * MPH, "map_target_dist": 60.0,
             "curve_lat_accel_vision": 0.0, "time_to_curve": 10.0, "gas": False, "brake": False}
  clear_sig = {"v_ego": 64 * MPH, "v_set": 64 * MPH, "map_target_v": 0.0, "map_target_dist": float("inf"),
               "curve_lat_accel_vision": 0.0, "time_to_curve": 10.0, "gas": False, "brake": False}
  assert run(cap_sig, 75 * MPH).get("target") is not None and mgr._icbm_dir == "dec"
  mgr._icbm_ep._engage_t0 -= (m.ICBM_RATCHET_CONFIRM_S + 0.1)
  # the left curve on the polyline, ahead: held -- the restore gives back only up to the curve's safe speed (69.9)
  mgr._icbm_k, mgr._icbm_k_n, mgr._icbm_k_ahead = 0.002557, 12, True
  out = run(clear_sig, 64 * MPH)
  assert out.get("dir") == "inc" and out["target"] == pytest.approx(round(math.sqrt(2.5 / 0.002557), 2))
  assert mgr._icbm_ep.phase == "restore" and mgr._icbm_ep.ahead_cap == pytest.approx(64 * MPH)
  assert m._rhold_tele(mgr) == {"icbmRHold": "poly", "icbmRHoldA": pytest.approx(2.88, abs=0.01),
                                "icbmRHoldV": pytest.approx(math.sqrt(2.5 / 0.002557), abs=0.01)}
  assert [e for e in events if e[0] == "ces_icbm_restore_hold"][-1][1]["state"] == "hold"
  # the curve passes: clear for the debounce, then the restore resumes toward 75
  mgr._icbm_k, mgr._icbm_k_ahead = 0.0005, True
  run(clear_sig, 64 * MPH)
  mgr._icbm_ep._ahead_clear_t0 -= ICBM_RESTORE_HOLD_CLEAR_S + 0.1
  out = run(clear_sig, 64 * MPH)
  assert out.get("dir") == "inc" and out["target"] == pytest.approx(round(75 * MPH, 2))
  assert [e for e in events if e[0] == "ces_icbm_restore_hold"][-1][1]["state"] == "release"
  assert m._rhold_tele(mgr)["icbmRHold"] is None


def test_icbm_step_hold_cut_short_is_logged_as_ended(tmp_path, monkeypatch):
  import time as _t
  events = []
  monkeypatch.setattr(m.cloudlog, "event", lambda name, **kw: events.append((name, kw)))
  mgr, step = _stub_mgr(tmp_path, monkeypatch)
  mgr._stock_set, mgr._stock_on = 75 * MPH, True
  sig = {"v_ego": 75 * MPH, "v_set": 75 * MPH, "map_target_v": 40 * MPH, "map_target_dist": 60.0,
         "curve_lat_accel_vision": 0.0, "time_to_curve": 10.0, "gas": False, "brake": False}
  mgr._icbm_last_pub = _t.monotonic() - 1.0
  step(sig, active=True)
  mgr._icbm_ep._engage_t0 -= (m.ICBM_RATCHET_CONFIRM_S + 0.1)
  mgr._icbm_k, mgr._icbm_k_n, mgr._icbm_k_ahead = 0.003, 12, True
  mgr._stock_set = 64 * MPH
  mgr._icbm_last_pub = _t.monotonic() - 1.0
  step({**sig, "v_ego": 64 * MPH, "v_set": 64 * MPH, "map_target_v": 0.0, "map_target_dist": float("inf")}, active=True)
  assert mgr._icbm_rhold_on
  mgr._icbm_last_pub = _t.monotonic() - 1.0
  step({**sig, "gas": True, "map_target_v": 0.0, "map_target_dist": float("inf")}, active=True)
  last = [e for e in events if e[0] == "ces_icbm_restore_hold"][-1][1]
  assert last["state"] == "ended" and last["phase"] == "idle"


def _to_restore_via_step(mgr, step, events=None):
  import time as _t

  def run(sig, stock_set):
    mgr._stock_set, mgr._stock_on = stock_set, True
    mgr._icbm_last_pub = _t.monotonic() - 1.0
    step(sig, active=True)
    return mgr.mem_params.last
  cap_sig = {"v_ego": 75 * MPH, "v_set": 75 * MPH, "map_target_v": 40 * MPH, "map_target_dist": 60.0,
             "curve_lat_accel_vision": 0.0, "time_to_curve": 10.0, "gas": False, "brake": False}
  clear_sig = {**cap_sig, "v_ego": 64 * MPH, "v_set": 64 * MPH, "map_target_v": 0.0, "map_target_dist": float("inf")}
  run(cap_sig, 75 * MPH)
  mgr._icbm_ep._engage_t0 -= (m.ICBM_RATCHET_CONFIRM_S + 0.1)
  return run, cap_sig, clear_sig


def test_icbm_step_the_polyline_bar_is_2_8_not_the_vision_bar(tmp_path, monkeypatch):
  """(b) at the controller: 2.64 m/s^2 on the polyline alone (above vision's 2.5, below its own 2.8) must not hold."""
  mgr, step = _stub_mgr(tmp_path, monkeypatch)
  run, _, clear_sig = _to_restore_via_step(mgr, step)
  mgr._icbm_k, mgr._icbm_k_n, mgr._icbm_k_ahead = 2.64 / (75 * MPH) ** 2, 12, True
  out = run(clear_sig, 64 * MPH)
  assert mgr._icbm_ep.phase == "restore" and mgr._icbm_ep.ahead_cap is None
  assert out.get("dir") == "inc" and out["target"] == pytest.approx(round(75 * MPH, 2))


def test_icbm_step_a_carried_cap_judges_candidates_against_the_interrupted_set(tmp_path, monkeypatch):
  """(c) at the controller: while a cap carries the original 75 ceiling, a curve that binds against 75 -- here a 50 mph
  curve 300 m out, inside the tracking window only for the higher reference -- but not yet against the 64 the hold was
  at must NOT bind (with the old re-latch it could not have). Otherwise the carry would move SET- taps earlier."""
  mgr, step = _stub_mgr(tmp_path, monkeypatch)
  run, _, _ = _to_restore_via_step(mgr, step)
  ep = mgr._icbm_ep
  ep.phase, ep.ceiling, ep._bind_ref = "cap", 75 * MPH, 64 * MPH       # a carried cap, mid-episode
  ep._clear_t0, ep._engage_t0 = None, -1e9
  sig = {"v_ego": 64 * MPH, "v_set": 64 * MPH, "map_target_v": 50 * MPH, "map_target_dist": 300.0,
         "curve_lat_accel_vision": 0.0, "time_to_curve": 10.0, "gas": False, "brake": False}
  # sanity: the same curve against a 75 reference DOES bind (so the test measures the reference, not the curve)
  t75, _, _ = m.icbm_curve_target(64 * MPH, 64 * MPH, 50 * MPH, 300.0, 75 * MPH, m.icbm_map_eff_scale,
                                  map_scale=mgr._veh.icbm_map_scale, firm_decel=mgr._veh.icbm_firm_decel, track=True)
  t64, _, _ = m.icbm_curve_target(64 * MPH, 64 * MPH, 50 * MPH, 300.0, 64 * MPH, m.icbm_map_eff_scale,
                                  map_scale=mgr._veh.icbm_map_scale, firm_decel=mgr._veh.icbm_firm_decel, track=True)
  assert t75 is not None and t64 is None
  run(sig, 64 * MPH)
  assert mgr._icbm_dir != "dec"


def test_icbm_step_says_so_when_a_running_restore_is_blind(tmp_path, monkeypatch):
  """Fable F3 (Rule 2): a restore running with no measurable polyline ahead and no vision logs "blind" ONCE."""
  import time as _t
  events = []
  monkeypatch.setattr(m.cloudlog, "event", lambda name, **kw: events.append((name, kw)))
  mgr, step = _stub_mgr(tmp_path, monkeypatch)

  def run(sig, stock_set):
    mgr._stock_set, mgr._stock_on = stock_set, True
    mgr._icbm_last_pub = _t.monotonic() - 1.0
    step(sig, active=True)

  cap_sig = {"v_ego": 75 * MPH, "v_set": 75 * MPH, "map_target_v": 40 * MPH, "map_target_dist": 60.0,
             "curve_lat_accel_vision": 0.0, "time_to_curve": 10.0, "gas": False, "brake": False}
  clear_sig = {**cap_sig, "v_ego": 64 * MPH, "v_set": 64 * MPH, "map_target_v": 0.0, "map_target_dist": float("inf")}
  run(cap_sig, 75 * MPH)
  blind = [e for e in events if e[0] == "ces_icbm_restore_hold" and e[1].get("state") == "blind"]
  assert blind == [], "blind is about a RUNNING restore, not a cap"
  mgr._icbm_ep._engage_t0 -= (m.ICBM_RATCHET_CONFIRM_S + 0.1)
  run(clear_sig, 64 * MPH)
  run(clear_sig, 64 * MPH)
  assert mgr._icbm_ep.phase == "restore"
  blind = [e for e in events if e[0] == "ces_icbm_restore_hold" and e[1].get("state") == "blind"]
  assert len(blind) == 1 and blind[0][1]["why"] == "noGeom"


def test_the_telemetry_reaches_the_real_record():
  """The last mile (test_ces_record_fields.py's lesson): the fields are on the ces_events tick record."""
  from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_ces_record_fields import _record
  ep = IcbmEpisode()
  ep.ahead_cap, ep.ahead_vsafe = 28.0, 31.0
  rec = _record(_icbm_rhold_on=True, _icbm_rhold_src="vis", _icbm_rhold_a=2.713, _icbm_ep=ep)
  assert rec["icbmRHold"] == "vis" and rec["icbmRHoldA"] == 2.71 and rec["icbmRHoldV"] == 31.0
  rec = _record(_icbm_rhold_on=False, _icbm_rhold_src="vis", _icbm_rhold_a=None)
  assert rec["icbmRHold"] is None and rec["icbmRHoldA"] is None and rec["icbmRHoldV"] is None


def test_the_hold_constant_is_the_fast_restore_debounce():
  assert math.isclose(ICBM_RESTORE_HOLD_CLEAR_S, m.ICBM_RESTORE_DELAY_FAST_S)


# ---------------------------------------------------------------------------------------------------------------
# restorehold3pnw: Fable's two residuals on 0a5cb972dc. Both LOST a restore (never slowed the truck).
# ---------------------------------------------------------------------------------------------------------------
def _restore_running(v=60):
  """An episode whose restore just published an inc at set `v` toward 75. Returns (ep, t of that tick)."""
  ep = IcbmEpisode()
  ep.step(0.0, v * MPH, 75 * MPH, 75 * MPH, True, False)
  ep.step(1.0, None, v * MPH, v * MPH, True, False)
  t = 1.0 + ICBM_RESTORE_DELAY_S + 0.1
  assert ep.step(t, None, v * MPH, v * MPH, True, False, v_ego=v * MPH)[1] == "inc"
  return ep, t


class TestFableResiduals:
  def test_s2_an_in_curve_tick_between_the_press_and_the_hold_keeps_the_late_tap_grace(self):
    """Fable sim S2 -- the 21:21:50 shape: an inc tick, then ONE in-curve pause tick, then the hold binds. Our taps
    pressed on the inc tick are still in flight; the pause tick used to wipe "we were pressing", so the hold opened no
    grace and 2 late taps reset the episode (restore lost, driver has to raise the set)."""
    ep, t = _restore_running()
    assert ep.step(t + DT, None, 60 * MPH, 60 * MPH, True, False, v_ego=60 * MPH, in_curve=True) == (None, None)
    assert ep.step(t + 2 * DT, None, 61 * MPH, 61 * MPH, True, False, v_ego=60 * MPH, hold_ahead=True) == (None, None)
    ep.step(t + 3 * DT, None, 63 * MPH, 63 * MPH, True, False, v_ego=60 * MPH, hold_ahead=True)
    assert ep.phase == "restore", "our own late taps after an in-curve tick reset the held restore"
    ep.step(t + 3 * DT + 1.5, None, 63 * MPH, 63 * MPH, True, False, v_ego=60 * MPH)    # clears; debounce runs
    pub, d = ep.step(t + 3 * DT + 1.5 + ICBM_RESTORE_HOLD_CLEAR_S + 0.01, None, 63 * MPH, 63 * MPH, True, False,
                     v_ego=60 * MPH)
    assert d == "inc" and pub == pytest.approx(75 * MPH)

  def test_s2_the_grace_is_bounded_in_time_not_by_ticks(self):
    """The press must be RECENT: a hold that binds more than ICBM_LATE_TAP_GRACE_S after the last inc publishes
    opens no grace (nothing of ours can still be in flight), so 2 extra taps are the driver."""
    ep, t = _restore_running()
    for i in range(1, 8):                                         # 1.75 s of in-curve pause after the last press
      ep.step(t + i * DT, None, 60 * MPH, 60 * MPH, True, False, v_ego=60 * MPH, in_curve=True)
    t2 = t + 8 * DT
    ep.step(t2, None, 60 * MPH, 60 * MPH, True, False, v_ego=60 * MPH, hold_ahead=True)
    ep.step(t2 + DT, None, 62 * MPH, 62 * MPH, True, False, v_ego=60 * MPH, hold_ahead=True)
    assert ep.phase == "idle"

  def test_s3_a_curve_during_the_late_tap_grace_still_carries_the_ceiling(self):
    """Fable sim S3: a mid-climb hold opens the grace, 2+ of our late taps land, and a curve binds inside the 1.5 s.
    _carry_on_cap used the strict one-tap tolerance, read our taps as the driver's, and re-latched the ceiling at the
    held set (75 -> 63): after that curve the restore stopped at 63."""
    ep, t = _restore_running()
    ep.step(t + DT, None, 61 * MPH, 61 * MPH, True, False, v_ego=60 * MPH, hold_ahead=True)     # snapshot 61, grace
    ep.step(t + 2 * DT, None, 62 * MPH, 62 * MPH, True, False, v_ego=60 * MPH, hold_ahead=True)
    pub, d = ep.step(t + 3 * DT, 55 * MPH, 63 * MPH, 63 * MPH, True, False, v_ego=60 * MPH, hold_ahead=True)
    assert d == "dec" and ep.ceiling == pytest.approx(75 * MPH), "the late taps made the carry re-latch low"
    assert ep.bind_ceiling == pytest.approx(63 * MPH)

  def test_s3_beyond_our_in_flight_taps_the_driver_still_wins(self):
    ep, t = _restore_running()
    ep.step(t + DT, None, 61 * MPH, 61 * MPH, True, False, v_ego=60 * MPH, hold_ahead=True)
    ep.step(t + 2 * DT, None, 63 * MPH, 63 * MPH, True, False, v_ego=60 * MPH, hold_ahead=True)
    ep.step(t + 3 * DT, 55 * MPH, 65 * MPH, 65 * MPH, True, False, v_ego=60 * MPH, hold_ahead=True)   # +4, the band
    assert ep.ceiling == pytest.approx(75 * MPH)
    ep2, t = _restore_running()
    ep2.step(t + DT, None, 61 * MPH, 61 * MPH, True, False, v_ego=60 * MPH, hold_ahead=True)
    ep2.step(t + 2 * DT, None, 63 * MPH, 63 * MPH, True, False, v_ego=60 * MPH, hold_ahead=True)
    ep2.step(t + 3 * DT, 55 * MPH, 65.8 * MPH, 65.8 * MPH, True, False, v_ego=60 * MPH, hold_ahead=True)
    assert ep2.ceiling == pytest.approx(65.8 * MPH), "more than our in-flight taps is the driver: he wins"

  def test_s3_after_the_grace_the_carry_is_strict_again(self):
    ep, t = _restore_running()
    ep.step(t + DT, None, 61 * MPH, 61 * MPH, True, False, v_ego=60 * MPH, hold_ahead=True)
    ep.step(t + 2.0, None, 63 * MPH, 63 * MPH, True, False, v_ego=60 * MPH, hold_ahead=True)    # re-anchored at 63
    ep.step(t + 2.25, 55 * MPH, 65 * MPH, 65 * MPH, True, False, v_ego=60 * MPH, hold_ahead=True)
    assert ep.ceiling == pytest.approx(65 * MPH)

  def test_s3_the_band_itself_catches_a_driver_who_climbs_at_cadence(self):
    """Mutation s3c: the driver pressing SET+ at our own cadence never trips the rise check, so only the late-tap band
    can tell him apart: 5.5 taps over the snapshot, 1.5 over the previous tick, inside the grace -> his value wins."""
    ep, t = _restore_running()
    ep.step(t + DT, None, 61 * MPH, 61 * MPH, True, False, v_ego=60 * MPH, hold_ahead=True)     # snapshot 61
    ep.step(t + 2 * DT, None, 63 * MPH, 63 * MPH, True, False, v_ego=60 * MPH, hold_ahead=True)
    ep.step(t + 3 * DT, None, 65 * MPH, 65 * MPH, True, False, v_ego=60 * MPH, hold_ahead=True)
    assert ep.phase == "restore"                                  # +4: still ours
    ep.step(t + 4 * DT, 55 * MPH, 66.5 * MPH, 66.5 * MPH, True, False, v_ego=60 * MPH, hold_ahead=True)
    assert ep.ceiling == pytest.approx(66.5 * MPH)

  def test_s3_the_grace_band_never_carries_a_ceiling_below_the_set(self):
    """Fable review of restorehold3pnw, F1: ceiling one tap above the hold (63 over a 62 snapshot), and two taps land on
    the SAME tick a curve binds (62 -> 64: inside the cadence check and the grace band). The carry must not keep the
    63 ceiling -- the restore after that curve would stop a tap BELOW the driver's 64. The set has reached the
    ceiling, so there is nothing left to carry: re-latch at the set, as 0a5cb972dc did."""
    ep = IcbmEpisode()
    ep.step(0.0, 60 * MPH, 63 * MPH, 63 * MPH, True, False)
    ep.step(1.0, None, 60 * MPH, 60 * MPH, True, False)
    t = 1.0 + ICBM_RESTORE_DELAY_S + 0.1
    assert ep.step(t, None, 60 * MPH, 60 * MPH, True, False, v_ego=60 * MPH)[1] == "inc"
    ep.step(t + DT, None, 62 * MPH, 62 * MPH, True, False, v_ego=60 * MPH, hold_ahead=True)     # snapshot 62, grace
    pub, d = ep.step(t + 2 * DT, 55 * MPH, 64 * MPH, 64 * MPH, True, False, v_ego=60 * MPH, hold_ahead=True)
    assert d == "dec"
    assert ep.ceiling >= 64 * MPH - 1e-6, "the grace band carried a ceiling below the driver's set"

  def test_s3_a_cap_on_the_first_tick_after_the_grace_is_strict(self):
    """Mutation s3b: the grace is a time window. A cap arriving on the first tick AFTER it closed -- before any restore
    tick re-anchored the snapshot -- is judged with the strict one-tap tolerance again."""
    ep, t = _restore_running()
    ep.step(t + DT, None, 61 * MPH, 61 * MPH, True, False, v_ego=60 * MPH, hold_ahead=True)     # grace to t+DT+1.5
    ep.step(t + DT + m.ICBM_LATE_TAP_GRACE_S + 0.1, 55 * MPH, 63 * MPH, 63 * MPH, True, False, v_ego=60 * MPH,
            hold_ahead=True)
    assert ep.ceiling == pytest.approx(63 * MPH)
