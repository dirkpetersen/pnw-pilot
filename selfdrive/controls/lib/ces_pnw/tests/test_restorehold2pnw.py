"""restorehold2pnw -- an ICBM restore is HELD while the curve ahead is too sharp for the speed it would give back.

THE EVENT (2026-09-21 21:21 PT, I-5 S, Tumwater S-bend; tests/data/tumwater_sbend_2026-09-21.json, the logged
ces_events ticks): ICBM capped the S-bend's right half 75 -> 64. At 21:21:49 the curve cleared and the RESTORE
walked the set 64 -> 75; the truck accelerated 64 -> 73 mph into the LEFT half and openpilot's steering saturated
(21:22:01, demand 3.47 vs its own 3.21 ceiling) before the driver steered in. At the restore's first tick the map
polyline already measured the left curve: icbmK 0.00256 at 393 m, ahead -- 2.87 m/s^2 at the 75 mph ceiling.

THE OWNER'S CONSTRAINT: "I do not want any more slowdowns." So the only lever is withholding acceleration. These
tests pin that the hold (a) never taps SET-, never lowers a target and never touches a cap, (b) holds the truck's
CURRENT speed, (c) resumes once the curve is clear, (d) is bounded by the restore window, (e) does nothing on a car
without the capability.
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
FIXTURE = os.path.join(os.path.dirname(__file__), "data", "tumwater_sbend_2026-09-21.json")


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
  def test_tumwater_first_restore_tick_holds_on_the_polyline(self):
    hold, a, src = icbm_restore_ahead_hold(33.53, 0.002557, 12, True, 0.00169, 2.5)
    assert hold and src == "poly" and a == pytest.approx(2.875, abs=0.01)

  def test_vision_alone_can_hold(self):
    hold, a, src = icbm_restore_ahead_hold(33.53, 0.0, 12, True, 0.00226, 2.5)   # 21:21:51's vision reading
    assert hold and src == "vis" and a == pytest.approx(2.54, abs=0.01)

  def test_the_tighter_source_wins(self):
    assert icbm_restore_ahead_hold(30.0, 0.002, 5, True, 0.003, 2.5)[2] == "vis"
    assert icbm_restore_ahead_hold(30.0, 0.003, 5, True, 0.002, 2.5)[2] == "poly"

  def test_a_gentle_road_does_not_hold(self):
    hold, a, _ = icbm_restore_ahead_hold(33.53, 0.001, 12, True, 0.0012, 2.5)
    assert not hold and a < 2.5

  def test_a_polyline_max_BEHIND_the_truck_is_ignored(self):
    """The curve the restore follows is behind the truck; it must not hold its own restore."""
    hold, a, src = icbm_restore_ahead_hold(33.53, 0.005, 12, False, None, 2.5)
    assert (hold, a, src) == (False, None, "noGeom")

  def test_an_unmeasurable_polyline_is_not_used(self):
    assert icbm_restore_ahead_hold(33.53, 0.005, 0, True, None, 2.5) == (False, None, "noGeom")

  @pytest.mark.parametrize("a_hold", [0.0, -1.0, None, float("nan"), "x"])
  def test_off(self, a_hold):
    assert icbm_restore_ahead_hold(33.53, 0.005, 12, True, 0.005, a_hold) == (False, None, "off")

  @pytest.mark.parametrize("v", [0.0, -3.0, None, float("inf"), "x"])
  def test_bad_target_speed(self, v):
    assert icbm_restore_ahead_hold(v, 0.005, 12, True, 0.005, 2.5) == (False, None, "badInput")

  @pytest.mark.parametrize("junk", [None, float("nan"), float("inf"), "x", True])
  def test_junk_curvature_never_raises_and_never_holds_by_itself(self, junk):
    assert icbm_restore_ahead_hold(33.53, junk, 12, True, junk, 2.5)[0] is False

  def test_the_threshold_is_inclusive_and_absolute(self):
    """Mutation guard: pins the comparison and the v^2, not just 'some' threshold."""
    k = 2.5 / 30.0 ** 2
    assert icbm_restore_ahead_hold(30.0, k * 1.001, 3, True, None, 2.5)[0] is True
    assert icbm_restore_ahead_hold(30.0, k * 0.999, 3, True, None, 2.5)[0] is False
    assert icbm_restore_ahead_hold(15.0, k * 1.001, 3, True, None, 2.5)[0] is False   # quarter the load at half speed


# ---------------------------------------------------------------------------------------------------------------
# Closed-loop harness: brain -> executor (1 mph taps, 0.4 s cadence, 0.6 mph deadband) -> stock ACC (set follows
# taps, speed follows set). Counts SET- taps, which is the owner's "slowdown" currency.
# ---------------------------------------------------------------------------------------------------------------
def simulate(plan, set0=75 * MPH, v0=75 * MPH, window_s=ICBM_RESTORE_WINDOW_S, a_up=0.6, a_dn=0.8):
  """plan: list of (cap_target m/s or None, hold_ahead bool) per 0.25 s tick. Returns the trace dict."""
  ep = IcbmEpisode(window_s=window_s)
  stock, v, t, next_tap = set0, v0, 0.0, 0.0
  out = {"dec_taps": 0, "inc_taps": 0, "set": [], "v": [], "pub": [], "phase": []}
  for cap, hold in plan:
    pub, d = ep.step(t, cap, stock, stock, True, False, v_ego=v, hold_ahead=hold)
    out["pub"].append((pub, d))
    out["phase"].append(ep.phase)
    if pub is not None and t >= next_tap:
      tgt = min(pub, ep.ceiling) if (d == "inc" and ep.ceiling is not None) else pub
      if d == "dec" and stock > tgt + DEADBAND:
        stock -= MPH
        out["dec_taps"] += 1
        next_tap = t + TAP_S
      elif d == "inc" and stock < tgt - DEADBAND:
        stock += MPH
        out["inc_taps"] += 1
        next_tap = t + TAP_S
    v = min(v + a_up * DT, stock) if v < stock else max(v - a_dn * DT, stock)
    out["set"].append(stock)
    out["v"].append(v)
    t += DT
  return out


def _sbend_plan(hold_ticks, clear_ticks=60, cap_mph=64.0):
  """75 mph cruise, a curve caps the set to 64 for 8 s, then it clears; hold_ticks of 'curve ahead' follow."""
  plan = [(cap_mph * MPH, False)] * 32
  plan += [(None, True)] * hold_ticks
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
  """Decision level, identical inputs to both machines: wherever the held machine publishes a dec, the unheld one
  publishes the SAME dec; the held machine never publishes a dec the other does not, and every inc it publishes is
  at or below the unheld one's. Closed loop: never more SET- taps, never a higher set."""
  @pytest.mark.parametrize("seed", range(40))
  def test_open_loop_decisions(self, seed):
    rnd = random.Random(seed)
    a, b = IcbmEpisode(), IcbmEpisode()
    stock, t = 75 * MPH, 0.0
    for _ in range(600):
      r = rnd.random()
      cap = rnd.uniform(35, 74) * MPH if r < 0.25 else None
      stock = min(max(stock + rnd.choice((0.0, 0.0, 0.0, MPH, -MPH)), 20 * MPH), 80 * MPH)
      pedal = rnd.random() < 0.01
      v = stock + rnd.uniform(-3, 3) * MPH
      pa = a.step(t, cap, stock, stock, True, pedal, v_ego=v)
      pb = b.step(t, cap, stock, stock, True, pedal, v_ego=v, hold_ahead=rnd.random() < 0.5)
      if pb[1] == "dec" or pa[1] == "dec":
        assert pb == pa, f"seed {seed} t {t}: {pa} vs {pb}"
      if pb[1] == "inc":
        assert pa[1] == "inc" and pb[0] <= pa[0] + 1e-9 or pa[1] is None
      t += DT

  @pytest.mark.parametrize("seed", range(40))
  def test_closed_loop_taps_and_set(self, seed):
    rnd = random.Random(1000 + seed)
    plan = []
    for _ in range(12):
      cap = rnd.uniform(40, 72) * MPH
      plan += [(cap, False)] * rnd.randint(8, 40)
      h = rnd.randint(0, 120)
      plan += [(None, True)] * h + [(None, False)] * rnd.randint(4, 80)
    base = simulate([(c, False) for c, _ in plan])
    held = simulate(plan)
    assert held["dec_taps"] <= base["dec_taps"]
    # one tap of slack, never more: when a cap follows a resumed restore, the held run's executor can be mid-cadence
    # after its last inc tap, so its first dec lands one tick later than the unheld run's (seeds 12 and 27).
    assert all(s_h <= s_b + MPH + 1e-9 for s_h, s_b in zip(held["set"], base["set"], strict=True))


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
      hold, a, src = icbm_restore_ahead_hold(ceiling, r["icbmK"], r["icbmKN"], r["icbmKAhead"], r["visKMax"], 2.5)
      assert hold, f"{r['t']}: a={a} src={src}"

  def test_replayed_through_the_episode_the_set_stays_at_64(self, ticks):
    """Feed the machine the logged state at restore entry (set 64.0, vEgo 64.4, ceiling 75) and the logged
    predicate on each tick. With the hold nothing above the truck's speed, rounded up to the next 1 mph tap, is
    ever published: 65 instead of the 75 the logged restore walked to."""
    ep = IcbmEpisode()
    ep.step(0.0, 64 * MPH, 75 * MPH, 75 * MPH, True, False)
    ep.step(1.0, None, 64 * MPH, 64 * MPH, True, False)
    t = 1.0 + ICBM_RESTORE_DELAY_S + 0.1
    restore = [r for r in ticks if r["icbmPhase"] == "restore"]
    for r in restore:
      hold = icbm_restore_ahead_hold(ep.ceiling, r["icbmK"], r["icbmKN"], r["icbmKAhead"], r["visKMax"], 2.5)[0]
      pub, d = ep.step(t, None, 64 * MPH, 64 * MPH, True, False, v_ego=r["vEgo"], hold_ahead=hold)
      assert d is None or pub <= 65 * MPH + 1e-6
      t += 1.0


# ---------------------------------------------------------------------------------------------------------------
# Capability view: the Lightning has it, nothing else does, and a bad config is clamped
# ---------------------------------------------------------------------------------------------------------------
class TestTheCapability:
  def test_lightning_on_every_other_car_off(self, default_curve_cfg):
    assert _lightning().icbm_restore_hold_lat_accel == 2.5
    tesla = PnwVehicle(FakeCP("TESLA_MODEL_S_HW3", "tesla", True))
    assert tesla.icbm_restore_hold_lat_accel == 0.0
    assert PnwVehicle(None).icbm_restore_hold_lat_accel == 0.0
    assert icbm_restore_ahead_hold(33.5, 0.01, 9, True, 0.01, tesla.icbm_restore_hold_lat_accel) == (False, None, "off")

  def test_a_bad_config_is_clamped(self, tmp_path, monkeypatch):
    cfg = tmp_path / "curve.json"
    cfg.write_text(json.dumps({"lightning": {"icbm_restore_hold_lat_accel": 9.0}}))
    monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(cfg))
    assert _lightning().icbm_restore_hold_lat_accel == 3.2

  def test_default_input_is_byte_identical(self):
    """hold_ahead defaults to False: every existing caller keeps its exact behavior."""
    plan = _sbend_plan(0)
    a = simulate(plan)
    b = simulate([(c, False) for c, _ in plan])
    assert a == b


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
  # the left curve on the polyline, ahead: held -- nothing is published, the set stays at 64
  mgr._icbm_k, mgr._icbm_k_n, mgr._icbm_k_ahead = 0.002557, 12, True
  assert run(clear_sig, 64 * MPH) == {}
  assert mgr._icbm_ep.phase == "restore" and mgr._icbm_ep.ahead_cap == pytest.approx(64 * MPH)
  assert m._rhold_tele(mgr) == {"icbmRHold": "poly", "icbmRHoldA": pytest.approx(2.88, abs=0.01)}
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


def test_the_telemetry_reaches_the_real_record():
  """The last mile (test_ces_record_fields.py's lesson): both fields are on the ces_events tick record."""
  from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_ces_record_fields import _record
  rec = _record(_icbm_rhold_on=True, _icbm_rhold_src="vis", _icbm_rhold_a=2.713)
  assert rec["icbmRHold"] == "vis" and rec["icbmRHoldA"] == 2.71
  rec = _record(_icbm_rhold_on=False, _icbm_rhold_src="vis", _icbm_rhold_a=None)
  assert rec["icbmRHold"] is None and rec["icbmRHoldA"] is None


def test_the_hold_constant_is_the_fast_restore_debounce():
  assert math.isclose(ICBM_RESTORE_HOLD_CLEAR_S, m.ICBM_RESTORE_DELAY_FAST_S)
