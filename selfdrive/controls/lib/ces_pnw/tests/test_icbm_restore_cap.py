"""icbmrestorecap2pnw — a RESTORE may not raise the set above the posted limit + the driver's margin.

DRIVER REPORT, live, 2026-09-13 15:46 PT: "I'm in a 25 mph zone and I was properly slowed down, but
then ICBM puts the target speed to 60, which is wrong -- it should not do that in a 25 zone."

What the truck logged (tests/data/restore_25zone_2026-09-13.json):

    15:46:17-30  a curve episode taps the stock set 45 -> 27     latched ceiling = 60
    15:46:30     the posted limit drops to 25
    15:46:48     the curve clears -> RESTORE, target 60           limit still 25
    15:46:49-47:01  set taps 27 -> 60 over 13 s                   limit 25 the entire time
    15:47:34     limit rises to 45

Restore exists to "give back what the curve took", and it gave it back on a road that had changed
under it. Only a slower lead car held the truck near 30 mph.

The margin is the driver's own habit, not a chosen number: 9,063 weekend ticks where HE set the speed
(stock ACC on, ICBM silent, limit known) sit at the limit to +5 mph.
"""
import json
import math
import os

import pytest

from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import IcbmEpisode, ICBM_RESTORE_DELAY_S
from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw_constants import (
  ICBM_RESTORE_LIMIT_MARGIN_MS, ICBM_RESTORE_LIMIT_RISE_HOLD_S, ICBM_RESTORE_LIMIT_STALE_S, icbm_restore_limit)

MPH = 0.44704
T0 = 100.0
FIXTURE = os.path.join(os.path.dirname(__file__), "data", "restore_25zone_2026-09-13.json")


def _to_restore(ep, ceiling=60 * MPH, target=27 * MPH, t=T0, restore_cap=None):
  ep.step(t, target, ceiling, ceiling, True, False)                   # cap engages, latches ceiling
  t += 1.0
  ep.step(t, None, target, target, True, False, restore_cap=restore_cap)
  t += ICBM_RESTORE_DELAY_S + 0.1
  out = ep.step(t, None, target, target, True, False, restore_cap=restore_cap)
  return t, out


class TestTheDebounce:
  def test_a_lower_limit_is_adopted_immediately(self):
    lim, st = icbm_restore_limit(60 * MPH, None, 0.0)
    lim, st = icbm_restore_limit(25 * MPH, st, 1.0)
    assert math.isclose(lim, 25 * MPH)

  def test_a_higher_limit_must_persist(self):
    lim, st = icbm_restore_limit(25 * MPH, None, 0.0)
    lim, st = icbm_restore_limit(45 * MPH, st, 1.0)
    assert math.isclose(lim, 25 * MPH), "a one-tick rise released the cap"
    lim, st = icbm_restore_limit(45 * MPH, st, 1.0 + ICBM_RESTORE_LIMIT_RISE_HOLD_S - 0.1)
    assert math.isclose(lim, 25 * MPH)
    lim, st = icbm_restore_limit(45 * MPH, st, 1.0 + ICBM_RESTORE_LIMIT_RISE_HOLD_S + 0.1)
    assert math.isclose(lim, 45 * MPH)

  def test_the_rise_hold_is_real_not_just_proportional_to_its_constant(self):
    """Fable mutation M5: the test above is written in terms of ICBM_RESTORE_LIMIT_RISE_HOLD_S, so
    setting that constant to 0.0 left it green. Pin an absolute behaviour: a higher limit that has
    stood for 2 s is still not adopted."""
    lim, st = icbm_restore_limit(25 * MPH, None, 0.0)
    for t in (1.0, 1.5, 2.0, 2.5, 3.0):
      lim, st = icbm_restore_limit(45 * MPH, st, t)
    assert math.isclose(lim, 25 * MPH), "a 2 s old rise was adopted -- the persistence hold is gone"

  def test_flicker_up_never_releases_the_cap(self):
    """25 <-> 45 alternating each tick is the observed mapd flicker shape; it must hold at 25."""
    lim, st = icbm_restore_limit(25 * MPH, None, 0.0)
    for i in range(1, 20):
      lim, st = icbm_restore_limit((45 if i % 2 else 25) * MPH, st, float(i))
      assert math.isclose(lim, 25 * MPH), f"flicker released the cap at tick {i}"

  @pytest.mark.parametrize("dropout", [0.0, -1.0, float("nan"), float("inf"), None, "x"])
  def test_an_unknown_reading_does_NOT_lift_the_cap(self, dropout):
    """THE LESSON OF THE DAY. A missing limit is not evidence that the limit went up. Letting a
    mapd dropout inside a real 25 zone release the restore toward the old set is the very bug."""
    lim, st = icbm_restore_limit(25 * MPH, None, 0.0)
    lim, st = icbm_restore_limit(dropout, st, 5.0)
    assert math.isclose(lim, 25 * MPH)

  def test_but_the_held_limit_expires(self):
    """...so a town's 25 cannot follow the truck down an unmapped highway and cap a restore there."""
    lim, st = icbm_restore_limit(25 * MPH, None, 0.0)
    lim, st = icbm_restore_limit(0.0, st, ICBM_RESTORE_LIMIT_STALE_S - 1.0)
    assert math.isclose(lim, 25 * MPH)
    lim, st = icbm_restore_limit(0.0, st, ICBM_RESTORE_LIMIT_STALE_S + 1.0)
    assert lim == 0.0 and st is None

  def test_a_NEW_pending_rise_counts_as_the_limit_being_seen(self):
    """While a rise is settling the road HAS a known limit; the stale clock must not run out from under
    it. Two code paths refresh `seen` during a rise and each can hide a bug in the other, so each gets
    its own sequence. This one isolates the NEW-CANDIDATE path: a rise arrives just before the stale
    window would end, then the limit drops out. (Fable mutation M1; my first version fed two known
    readings in a row, so the second path refreshed `seen` and masked it.)"""
    lim, st = icbm_restore_limit(25 * MPH, None, 0.0)
    lim, st = icbm_restore_limit(45 * MPH, st, ICBM_RESTORE_LIMIT_STALE_S + 1.0)      # new candidate
    lim, st = icbm_restore_limit(0.0, st, ICBM_RESTORE_LIMIT_STALE_S + 2.0)           # dropout 1 s later
    assert math.isclose(lim, 25 * MPH), "the cap expired 1 s after a KNOWN reading -- `seen` not refreshed"

  def test_a_SETTLING_pending_rise_counts_as_the_limit_being_seen(self):
    """The STILL-SETTLING path, isolated (Fable mutation M2): the candidate is first seen at +1, seen
    again at +2, and the limit then drops out 29.5 s after that second reading -- inside the window if
    `seen` was refreshed at +2, outside it if it was left at +1."""
    lim, st = icbm_restore_limit(25 * MPH, None, 0.0)
    lim, st = icbm_restore_limit(45 * MPH, st, 1.0)                                    # new candidate
    lim, st = icbm_restore_limit(45 * MPH, st, 2.0)                                    # still settling
    lim, st = icbm_restore_limit(0.0, st, 2.0 + ICBM_RESTORE_LIMIT_STALE_S - 0.5)      # 29.5 s after
    assert math.isclose(lim, 25 * MPH), "the settling reading did not refresh `seen`"

  @pytest.mark.parametrize("state", [
    ("junk", None, None),                          # wrong arity
    (25 * MPH, 100.0, "garbage", 100.0),           # the case review found: bad PENDING entry
    (25 * MPH, 100.0, 45 * MPH, None),             # pending without its timestamp
    (25 * MPH, "x", None, None),
    (float("nan"), 100.0, None, None),
    42, "state", object(),
  ])
  def test_a_malformed_carry_over_never_raises_and_is_discarded(self, state):
    """The state is CARRIED across ticks and _icbm_step swallows exceptions, so a state that raised
    once would raise on every tick after and silently disable ICBM for the rest of the drive. Gemini
    review: the first version validated only the limit and raised on a bad pending entry."""
    lim, st = icbm_restore_limit(45 * MPH, state, 200.0)
    assert math.isclose(lim, 45 * MPH), "a malformed history was trusted instead of discarded"
    lim2, _ = icbm_restore_limit(45 * MPH, st, 201.0)
    assert math.isclose(lim2, 45 * MPH), "the state it returned is not itself usable"

  def test_an_internal_failure_is_LOGGED_not_silent(self, monkeypatch):
    """Rule 2 (Fable review): falling back to "no cap" is right, but doing it silently would switch the
    fix off on every tick with nothing in the log."""
    import openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw_constants as C
    logged = []
    monkeypatch.setattr(C, "_icbm_restore_limit", lambda *a: (_ for _ in ()).throw(RuntimeError("boom")))
    monkeypatch.setattr(C.cloudlog, "exception", lambda msg, *a, **k: logged.append(msg))
    monkeypatch.setattr(C, "_icbm_rcap_err_last", -1e9)
    assert C.icbm_restore_limit(25 * MPH, None, 0.0) == (0.0, None)
    assert logged and "DISABLED" in logged[0], "the cap switched itself off without a word"
    C.icbm_restore_limit(25 * MPH, None, 0.0)
    assert len(logged) == 1, "the failure log is not throttled -- it would flood at ~4 Hz"

  def test_a_bad_clock_never_raises(self):
    assert icbm_restore_limit(25 * MPH, None, "not-a-time") == (0.0, None)
    assert icbm_restore_limit(25 * MPH, None, float("nan")) == (0.0, None)


class TestTheEpisodeUnderACap:
  def test_restore_target_is_capped(self):
    ep = IcbmEpisode()
    _, (tgt, d) = _to_restore(ep, restore_cap=30 * MPH)
    assert d == "inc" and math.isclose(tgt, 30 * MPH), f"restored toward {tgt / MPH:.1f} mph, not 30"
    assert ep.phase == "restore" and math.isclose(ep.ceiling, 60 * MPH), "the ceiling itself is untouched"

  def test_at_the_cap_it_HOLDS_rather_than_ending(self):
    ep = IcbmEpisode()
    t, _ = _to_restore(ep, restore_cap=30 * MPH)
    assert ep.step(t + 1.0, None, 29.9 * MPH, 29.9 * MPH, True, False, restore_cap=30 * MPH) == (None, None)
    assert ep.phase == "restore", "reaching the cap ended the episode -- a rising limit could never resume it"

  def test_and_follows_a_rising_limit_back_up_to_the_ceiling_never_past_it(self):
    ep = IcbmEpisode()
    t, _ = _to_restore(ep, restore_cap=30 * MPH)
    ep.step(t + 1.0, None, 30 * MPH, 30 * MPH, True, False, restore_cap=30 * MPH)
    tgt, d = ep.step(t + 2.0, None, 30 * MPH, 30 * MPH, True, False, restore_cap=50 * MPH)
    assert d == "inc" and math.isclose(tgt, 50 * MPH)
    tgt, d = ep.step(t + 3.0, None, 31 * MPH, 31 * MPH, True, False, restore_cap=80 * MPH)
    assert d == "inc" and math.isclose(tgt, 60 * MPH), "a cap above the ceiling must not lift the ceiling"

  def test_a_driver_SET_plus_during_the_hold_ends_the_episode(self):
    """Fable: while holding, the brain publishes nothing, so a rise beyond one in-flight tap is a human.
    A single SET+ tap used to slip under the fast-rise detector and be absorbed as if it were ours --
    and the hold stretched that window from ~13 s to the full 45 s restore window."""
    ep = IcbmEpisode()
    t, _ = _to_restore(ep, restore_cap=30 * MPH)
    for i, stock in enumerate((28.5, 30.0, 30.0)):                   # climb to the cap, then hold
      ep.step(t + 1.0 + i, None, stock * MPH, stock * MPH, True, False, restore_cap=30 * MPH)
    assert ep.phase == "restore"
    ep.step(t + 5.0, None, 32.0 * MPH, 32.0 * MPH, True, False, restore_cap=30 * MPH)   # driver: SET+ x2
    assert ep.phase == "idle", "a driver SET+ during the hold was absorbed as ours"

  def test_one_late_in_flight_tap_after_the_hold_began_is_not_a_human(self):
    ep = IcbmEpisode()
    t, _ = _to_restore(ep, restore_cap=30 * MPH)
    for i, stock in enumerate((28.5, 30.0)):
      ep.step(t + 1.0 + i, None, stock * MPH, stock * MPH, True, False, restore_cap=30 * MPH)
    ep.step(t + 3.0, None, 30.6 * MPH, 30.6 * MPH, True, False, restore_cap=30 * MPH)   # our last tap lands
    assert ep.phase == "restore", "our own in-flight tap was mistaken for the driver"

  def test_no_cap_is_exactly_the_old_behaviour(self):
    ep = IcbmEpisode()
    _, (tgt, d) = _to_restore(ep, restore_cap=None)
    assert d == "inc" and math.isclose(tgt, 60 * MPH)

  @pytest.mark.parametrize("bad", [0.0, -5.0, float("nan"), "x"])
  def test_a_nonsense_cap_is_ignored_not_obeyed(self, bad):
    ep = IcbmEpisode()
    _, (tgt, d) = _to_restore(ep, restore_cap=bad)
    assert d == "inc" and math.isclose(tgt, 60 * MPH)

  def test_the_cap_never_turns_a_restore_into_a_decrease(self):
    """Set already ABOVE the cap (e.g. curve cleared at 40 in a 25 zone): restore must go silent,
    never publish a lower target -- an 'inc' command below the set would be meaningless at best."""
    ep = IcbmEpisode()
    ep.step(T0, 40 * MPH, 60 * MPH, 60 * MPH, True, False)
    ep.step(T0 + 1.0, None, 40 * MPH, 40 * MPH, True, False, restore_cap=30 * MPH)
    out = ep.step(T0 + 1.0 + ICBM_RESTORE_DELAY_S + 0.1, None, 40 * MPH, 40 * MPH, True, False, restore_cap=30 * MPH)
    assert out == (None, None)


class TestTheRealIncident:
  """Replay 2026-09-13 15:46:15-15:47:05 PT through the episode machine and the limit debounce.

  Note the fixture window opens after the truck's real 60 mph ceiling was latched, so the replayed
  episode latches the set it sees first (48 mph). The cap (30) is below either, so the test exercises
  the defect the same way; it does not claim to reproduce the 60 itself."""

  @staticmethod
  def _load():
    with open(FIXTURE) as f:
      return json.load(f)

  def test_the_fixture_is_the_incident(self):
    ticks = self._load()
    assert any(r["icbmSrc"] == "restore" for r in ticks), "fixture no longer contains the restore"
    assert any(abs((r["spdLim"] or 0) - 25 * MPH) < 0.2 for r in ticks)
    assert max(r["stockSet"] for r in ticks) > 59 * MPH, "fixture no longer shows the set reaching 60"

  def test_OPEN_LOOP_no_published_restore_exceeds_limit_plus_margin(self):
    """Recorded inputs, fixed brain. The recorded set kept rising (it was driven by the OLD taps),
    so this checks only what the brain would have COMMANDED -- which is the defect."""
    ticks = self._load()
    ep, st, worst = IcbmEpisode(), None, None
    for r in ticks:
      lim, st = icbm_restore_limit(r["spdLim"], st, r["t"])
      cap = lim + ICBM_RESTORE_LIMIT_MARGIN_MS if lim > 0 else None
      cap_t = r["icbmT"] if r["icbmSrc"] in ("map", "far", "vis") else None
      tgt, d = ep.step(r["t"], cap_t, r["stockSet"], r["stockSet"], bool(r["stockOn"]), bool(r["gas"]),
                       restore_cap=cap)
      if d == "inc" and abs((r["spdLim"] or 0) - 25 * MPH) < 0.2:
        worst = tgt if worst is None else max(worst, tgt)
    assert worst is not None, "the replay never reached a restore in the 25 zone -- the test proves nothing"
    # compare against the limit AS RECORDED (11.2 m/s = 25.05 mph), not an idealised 25.000 mph:
    # the first version of this assertion used 25 * MPH and failed on the 0.05 mph rounding alone.
    zone_lim = max(r["spdLim"] for r in ticks if abs((r["spdLim"] or 0) - 25 * MPH) < 0.2)
    assert worst <= zone_lim + ICBM_RESTORE_LIMIT_MARGIN_MS + 1e-6, \
      f"commanded a restore to {worst / MPH:.2f} mph in the 25 zone"
    # and for contrast, what the truck actually did with the OLD brain in the same window:
    assert max(r["stockSet"] for r in ticks if abs((r["spdLim"] or 0) - 25 * MPH) < 0.2) > 59 * MPH

  def test_CLOSED_LOOP_the_set_settles_at_the_cap(self):
    """Same curve, same limit sequence, but the set now responds to OUR taps (1 mph per 0.4 s, the
    executor cadence) instead of the recorded ones."""
    ticks = self._load()
    ep, st, stock = IcbmEpisode(), None, ticks[0]["stockSet"]
    in_zone = []
    for r in ticks:
      lim, st = icbm_restore_limit(r["spdLim"], st, r["t"])
      cap = lim + ICBM_RESTORE_LIMIT_MARGIN_MS if lim > 0 else None
      cap_t = r["icbmT"] if r["icbmSrc"] in ("map", "far", "vis") else None
      tgt, d = ep.step(r["t"], cap_t, stock, stock, True, False, restore_cap=cap)
      if tgt is not None:
        step = 2.5 * MPH                                   # ~1 s of taps at the executor cadence
        stock = max(stock - step, tgt) if d == "dec" else min(stock + step, tgt)
      if abs((r["spdLim"] or 0) - 25 * MPH) < 0.2:
        in_zone.append(stock)
    assert in_zone, "never simulated inside the 25 zone"
    zone_lim = max(r["spdLim"] for r in ticks if abs((r["spdLim"] or 0) - 25 * MPH) < 0.2)
    assert max(in_zone) <= zone_lim + ICBM_RESTORE_LIMIT_MARGIN_MS + 1e-6, \
      f"set reached {max(in_zone) / MPH:.2f} mph in the 25 zone"


class TestThroughTheControllerStep:
  """The WIRING, driven through CESController._icbm_step exactly as the car runs it.

  Every test above calls IcbmEpisode or icbm_restore_limit directly. Mutation testing showed that was
  not enough: deleting the margin, or never passing the cap into the episode at all, left all of them
  green -- the fix would have been inert on the truck with a fully passing suite. This class is here
  because the glue between two tested functions is exactly where that kind of defect lives."""

  @staticmethod
  def _controller(monkeypatch, clock):
    import inspect
    from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
    from openpilot.selfdrive.controls.lib.pnw_vehicle import PnwVehicle
    monkeypatch.setattr(m.time, "monotonic", lambda: clock[0])
    cls = next(o for o in vars(m).values() if inspect.isclass(o) and hasattr(o, "_icbm_step"))

    class FakeMem:
      def __init__(self):
        self.log = []

      def put_nonblocking(self, k, v):
        self.log.append(v)

    class Stub:
      pass
    c = Stub()
    c.mem_params = FakeMem()
    c._veh = PnwVehicle(None)
    c._icbm_ep = IcbmEpisode()
    for k, v in dict(_icbm_ceiling=None, _icbm_dir=None, _map_targets=[], _cur_lat=None, _cur_lon=None,
                     _cur_bearing=None, _icbm_floor_lim=0.0, _icbm_floor_pend=None, _icbm_floor_hit=False,
                     _icbm_k=0.0, _icbm_k_n=0, _icbm_k_ahead=False, _icbm_k_at=0.0, _icbm_k_at_d=0.0,
                     _icbm_k_at_n=0, _icbm_k_at_gap=0.0, _stock_set=0.0, _stock_on=True,
                     _icbm_last_pub=-1e9).items():
      setattr(c, k, v)
    return c, cls._icbm_step.__get__(c)

  def _run(self, monkeypatch, spd_lim_mph, new_curve_at=None):
    """Drive the controller through a curve and its restore. The simulated stock set RESPONDS to the
    controller's own commands at the executor's tap cadence -- the first version held the set fixed,
    so the hold-at-cap branch never executed through the controller at all (Gemini review)."""
    clock = [1000.0]
    c, step = self._controller(monkeypatch, clock)
    set_mph = 60.0
    # a SHARP curve, so the tapped-down set lands clearly below the 30 mph cap. (With a 27 mph map
    # target the posted-limit floor held the set at 29.7 -- already at the cap -- and restore correctly
    # held silent, which made the first version of this test assert on nothing.)
    curve = {"v_ego": 45 * MPH, "v_set": set_mph * MPH, "map_target_v": 12 * MPH, "map_target_dist": 15.0,
             "spd_lim": spd_lim_mph * MPH}
    c._stock_set = set_mph * MPH
    for _ in range(6):                                   # curve binds: cap episode, ceiling 60
      step(curve, active=True)
      clock[0] += 0.5
    assert c._icbm_ep.phase == "cap", "the curve never started an episode -- the test proves nothing"
    tapped = min(p["target"] for p in c.mem_params.log if p and p.get("dir") != "inc")
    assert tapped < 25 * MPH + ICBM_RESTORE_LIMIT_MARGIN_MS - 1.0, \
      f"set only tapped to {tapped / MPH:.1f} mph -- too close to the cap to test a restore"
    c._stock_set = tapped                                # the executor's taps landed
    clear = dict(curve, map_target_v=0.0, map_target_dist=float("inf"))
    incs, phases, published = [], [], []
    for i in range(40):                                  # curve clears -> restore
      sig = dict(clear, v_set=c._stock_set)
      if new_curve_at is not None and i == new_curve_at:
        sig.update(map_target_v=12 * MPH, map_target_dist=15.0)
      n_before = len(c.mem_params.log)
      step(sig, active=True)
      new = c.mem_params.log[n_before:]
      last = new[-1] if new else None
      published.append(last)
      if last and last.get("dir") == "inc":
        incs.append(last["target"])
        c._stock_set = min(c._stock_set + 1.25 * MPH, last["target"])   # 1 mph / 0.4 s over a 0.5 s tick
      phases.append(c._icbm_ep.phase)
      clock[0] += 0.5
    return c, incs, phases, published

  def test_in_a_25_zone_the_published_restore_stops_at_limit_plus_margin(self, monkeypatch):
    c, incs, phases, _ = self._run(monkeypatch, 25)
    assert incs, "no restore was ever published -- the test proves nothing"
    assert max(incs) <= 25 * MPH + ICBM_RESTORE_LIMIT_MARGIN_MS + 1e-6, \
      f"the controller published a restore to {max(incs) / MPH:.1f} mph in a 25 zone"
    assert max(incs) > 25 * MPH + 1e-6, "the margin was not applied -- restore capped at the bare limit"

  def test_with_no_known_limit_it_restores_to_the_old_set_as_before(self, monkeypatch):
    c, incs, phases, _ = self._run(monkeypatch, 0)
    assert incs and math.isclose(max(incs), 60 * MPH, abs_tol=0.05)

  def test_the_set_settles_AT_the_cap_and_the_episode_HOLDS_there(self, monkeypatch):
    """The hold branch, executed through the controller: the set reaches the cap, the controller goes
    silent, and the episode stays alive in `restore` rather than ending."""
    c, incs, phases, published = self._run(monkeypatch, 25)
    cap = 25 * MPH + ICBM_RESTORE_LIMIT_MARGIN_MS
    assert c._stock_set >= cap - 0.3, f"the set never reached the cap ({c._stock_set / MPH:.1f} mph)"
    assert c._stock_set <= cap + 1e-6
    held = [i for i, p in enumerate(published) if i > 0 and phases[i] == "restore" and not (p and p.get("dir") == "inc")]
    assert held, "the controller never held silently at the cap -- the hold branch did not run"

  def test_a_new_curve_during_the_hold_still_slows_the_truck(self, monkeypatch):
    """Review claimed a held restore would ignore a new curve and let the truck 'blow through' it.
    Refuted in the episode machine (DEC ALWAYS WINS during restore) -- pinned here through the
    controller, so it stays refuted."""
    c, incs, phases, published = self._run(monkeypatch, 25, new_curve_at=30)
    assert phases[29] == "restore", "the test did not reach the hold before the new curve"
    after = [p for p in published[30:34] if p]
    assert any(p.get("dir") != "inc" and p["target"] < 25 * MPH + ICBM_RESTORE_LIMIT_MARGIN_MS for p in after), \
      f"a curve binding during the hold published no decrease: {after}"
    assert "cap" in phases[30:34]
