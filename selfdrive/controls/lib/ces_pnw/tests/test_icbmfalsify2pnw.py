"""icbmfalsify2pnw: END a RUNNING map-sourced curve slowdown once the truck has DRIVEN PAST the point the map
claimed the curve was and MEASURED the road to be far gentler than the cap demands.

    k_map  = A_LAT / target^2     what the cap being commanded asks the road to be
    k_meas = |ach_lat| / v_ego^2  what the truck drove   (== |kActl|, the yaw-rate-derived curvature)
    end the cap when  k_meas * 2.0 < k_map,  sustained 2 s, after arriving at the candidate

Curvature is speed-independent, so "I measured a gentle curve" cannot be explained away by "the slowdown worked" --
the confounder that makes measured lateral ACCELERATION useless here. The rule only ever ENDS a cap.

WHAT THIS SUITE PINS, and the one thing a reader must not miss:

  * (§8) the 2026-09-08 20:28 phantom aborts at exactly 20:29:04 PT through the REAL _icbm_step -- and none of the
    three real-curve control windows is touched;
  * (§5) ending the cap does not blind the truck to a node still AHEAD: arrival is PER CANDIDATE, so a two-node
    curve whose first node is falsified stays capped on the second, and the tick after an abort is free to cap again;
  * Rule 2: an unreadable v_ego, an unreadable lateral accel, a FROZEN one, an unusable distance and wall time nobody
    measured through all reset the hold. A missing input can never buy a cap removal;
  * a vision-sourced cap, a START, and the Tesla are untouched;
  * >>> TestPreemptedByBehindrun: ON THE SHIPPED CODE THIS RULE NEVER FIRES. behindrun2pnw (2026-09-15) removes a
    passed map point as soon as it is 5 m behind along the path, which is strictly EARLIER than this rule can arm
    (receded PASS/PAST from the closest approach, then 2 s). The design's 35-abort / 759 s replay corpus was recorded
    2026-09-11..13, BEFORE behindrun2pnw existed. What is left is the case behindrun2pnw declines: it cannot tell the
    point is passed (icbm_passed_points -> None). Those tests deliberately fail if that ever stops being true.
"""
import json
import os

import pytest

from openpilot.selfdrive.controls.lib import pnw_vehicle as pv
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_behindgate2pnw import (
  V, VSET, _controller, _gate_off, _ll, _road, _starts,
)
from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_ces_record_fields import _record

FIXTURE = os.path.join(os.path.dirname(__file__), "data", "icbmfalsify_2026-09.json")
GENTLE = 0.0008      # 1/m -- a straight-ish road (R = 1250 m); against a 15 m/s map claim this is a falsification
REAL = 0.012         # 1/m -- a real R = 83 m bend; the map claim is then consistent and the cap must stand


# ---------------------------------------------------------------------------------------------------------
# the rule itself (pure)
# ---------------------------------------------------------------------------------------------------------
class TestMeasuredCurvature:
  def test_it_is_a_lat_over_v_squared(self):
    assert m.icbm_measured_curvature(2.5, 25.0) == pytest.approx(2.5 / 625.0)

  def test_a_RIGHT_hand_curve_measures_the_same_as_a_left(self):
    """Fable 2026-09-17. achLat is SIGNED -- negative is a right-hand bend on both cars. Without the
    abs() every right-hand curve reads a negative k_meas, `k_meas * RATIO < k_map` is then true for
    any map claim at all, and the rule falsifies EVERY right-hand curve after its 2 s hold: it would
    cancel real slowdowns, on one side of the road only, which is the single worst thing this rule
    could do. The abs() was the only line preventing it and nothing pinned it -- every synthetic
    test used positive curvature, and no fixture window arrives-and-measures on a right-hander."""
    assert m.icbm_measured_curvature(-2.5, 25.0) == pytest.approx(2.5 / 625.0)
    assert m.icbm_measured_curvature(-2.5, 25.0) == m.icbm_measured_curvature(2.5, 25.0)

  def test_it_is_speed_independent_which_is_the_whole_argument(self):
    """The same bend taken at 69 mph and at 44 mph reads as the same curvature -- which is why "I measured a gentle
    curve" cannot be explained away by "the slowdown worked"."""
    fast, slow = 31.0, 19.7
    assert m.icbm_measured_curvature(0.004 * fast ** 2, fast) == pytest.approx(
      m.icbm_measured_curvature(0.004 * slow ** 2, slow))

  @pytest.mark.parametrize("a_lat,v", [(None, 25.0), (0.5, None), (float("nan"), 25.0), (0.5, float("inf")),
                                       ("x", 25.0), (0.5, "x"), (0.5, m.ICBM_FALSIFY_MIN_V), (0.5, 0.0)])
  def test_anything_unreadable_is_none_and_never_zero(self, a_lat, v):
    """None is NOT zero. A zero reads as "perfectly straight road", which is the strongest possible evidence FOR
    ending a cap -- so a failed read must never produce one."""
    assert m.icbm_measured_curvature(a_lat, v) is None


class TestArrival:
  """The truck has reached the candidate only when it came within PASS_M and has SINCE receded PAST_M."""

  def _run(self, dists, **kw):
    """-> [(why, falsified)] for a sequence of candidate distances at 0.25 s / V m/s."""
    st = (None, None, 0.0)
    out = []
    for d in dists:
      min_d, prev_d, held, fired, why, _, _ = m.icbm_falsify_tick(
        st[0], st[1], st[2], kw.get("dt", 0.25), d, kw.get("target", 15.0),
        kw.get("k", GENTLE) * kw.get("v", V) ** 2, kw.get("v", V), kw.get("age", 0.0))
      st = (min_d, prev_d, held)
      out.append((why, fired))
    return out

  def test_within_60_m_but_still_approaching_does_not_count_as_arrived(self):
    """The FIRST arrival test accepted "within 60 m", which at 30 mph is still ~2 s BEFORE the curve: the truck was
    measuring the straight it had not yet left, and that produced both of the replay's false aborts."""
    got = self._run([60.0, 55.0, 50.0, 45.0, 40.0, 35.0, 30.0, 27.0])
    assert [w for w, _ in got] == ["start"] + ["approaching"] * 7, got
    assert not any(f for _, f in got)

  def test_close_enough_is_not_enough_either_it_must_then_recede(self):
    got = self._run([30.0, 24.0, 20.0, 16.0, 12.0, 8.0, 4.0, 2.0])
    assert not any(f for _, f in got), got
    assert [w for w, _ in got][-1] == "approaching"

  def test_passed_then_receded_arrives_and_a_sustained_gap_ends_the_cap(self):
    got = self._run([30.0, 20.0, 10.0, 2.0] + [5.0 + 2.0 * i for i in range(16)])
    assert any(f for _, f in got), got
    first = next(i for i, (_, f) in enumerate(got) if f)
    assert got[first][0] == "falsified"
    assert got[first - 1] == ("falsified", False), "the hold completed in fewer than HOLD_S"

  def test_a_candidate_that_never_came_close_can_never_be_falsified(self):
    """PASS_M: a point 300 m away that recedes is a point the truck never reached."""
    got = self._run([300.0, 303.0, 306.0, 309.0, 312.0, 315.0, 318.0])
    assert [w for w, _ in got] == ["start"] + ["approaching"] * 6, got


class TestPerCandidateArrivalIsSection5:
  """ICBMFALSIFY2PNW.md §5. The replay tracked ONE min_d per episode, so once any node had been passed it read
  "arrived" for every later candidate too -- including a node still AHEAD, whose target would then be judged against
  curvature measured back at the node already behind. A fixed point's distance can change by at most v_ego*dt between
  ticks, so anything larger is mapd picking a DIFFERENT point and the track restarts on it."""

  def test_a_jump_beyond_what_driving_can_explain_restarts_the_track(self):
    st = (2.0, 2.0, 0.0)     # the truck is 2 m from the candidate it has been tracking
    min_d, prev_d, held, fired, why, _, _ = m.icbm_falsify_tick(
      st[0], st[1], st[2], 0.25, 300.0, 15.0, GENTLE * V ** 2, V, 0.0)
    assert (why, fired) == ("newPoint", False)
    assert min_d == 300.0 and prev_d == 300.0 and held == 0.0, "the passed point's min_d survived into a new one"

  def test_the_09_08_trace_126_to_300_m_jump_is_not_an_arrival(self):
    """The real re-pick from the 2026-09-08 log. A bare "d grew" would read as arrival one second into a slowdown for
    a point 300 m away."""
    got = TestArrival()._run([126.0, 300.0, 268.0, 237.0, 205.0], dt=1.0, v=30.0)
    assert [w for w, _ in got] == ["start", "newPoint", "approaching", "approaching", "approaching"], got

  def test_driving_past_a_point_is_within_the_physics_and_does_not_restart(self):
    """v_ego*dt at 25 m/s over 0.25 s is 6.25 m; the recession that means "passed" is smaller than that per tick, so
    the legitimate case must never look like a re-pick."""
    got = TestArrival()._run([12.0, 6.0, 1.0, 4.0, 9.0, 12.0, 15.0])
    assert "newPoint" not in [w for w, _ in got], got

  def test_a_gap_nobody_measured_through_restarts_it_too(self):
    st = (2.0, 2.0, 1.75)
    min_d, _, held, fired, why, _, _ = m.icbm_falsify_tick(
      st[0], st[1], st[2], m.ICBM_FALSIFY_MAX_DT_S + 0.1, 14.0, 15.0, GENTLE * V ** 2, V, 0.0)
    assert (why, fired, held) == ("gap", False, 0.0)
    assert min_d == 14.0


class TestTheHoldMustBeConsecutive:
  def _seq(self, ks):
    """arrive first (ending on a REAL tick so the hold starts at zero), then feed measured curvatures `ks` --
    returns the (why, falsified, held) of each ks tick."""
    st, out = (None, None, 0.0), []
    for d in (30.0, 20.0, 10.0, 2.0, 6.0, 10.0, 14.0):     # arrive
      st = m.icbm_falsify_tick(st[0], st[1], st[2], 0.25, d, 15.0, REAL * V ** 2, V, 0.0)[:3]
    assert st[2] == 0.0
    d = 14.0
    for k in ks:
      d += 1.0
      min_d, prev_d, held, fired, why, _, _ = m.icbm_falsify_tick(
        st[0], st[1], st[2], 0.25, d, 15.0, k * V ** 2, V, 0.0)
      st = (min_d, prev_d, held)
      out.append((why, fired, round(held, 2)))
    return out

  def test_two_seconds_of_evidence_ends_the_cap(self):
    got = self._seq([GENTLE] * 8)
    assert [f for _, f, _ in got] == [False] * 7 + [True], got
    assert got[-1][2] == pytest.approx(m.ICBM_FALSIFY_HOLD_S)

  def test_one_consistent_tick_resets_the_hold_so_the_two_seconds_must_be_consecutive(self):
    got = self._seq([GENTLE] * 7 + [REAL] + [GENTLE] * 7)
    assert got[7] == ("consistent", False, 0.0), got[7]
    assert not any(f for _, f, _ in got), "the hold survived a tick where the road matched the claim"
    assert self._seq([GENTLE] * 7 + [REAL] + [GENTLE] * 8)[-1][1] is True, "it never recovers -- proves nothing"

  def test_a_road_that_matches_the_claim_never_ends_the_cap(self):
    assert [f for _, f, _ in self._seq([REAL] * 40)] == [False] * 40

  def test_exactly_2x_is_not_more_than_2x(self):
    """RATIO is a strict >: the map must have asked for MORE than 2x what the road delivered."""
    k_map = m.ICBM_FALSIFY_A_LAT / 15.0 ** 2
    assert [f for _, f, _ in self._seq([k_map / m.ICBM_FALSIFY_RATIO] * 20)] == [False] * 20
    assert self._seq([k_map / m.ICBM_FALSIFY_RATIO * 0.99] * 20)[-1][1] is True


class TestRule2AMissingInputCanNeverBuyACapRemoval:
  def _arrived(self):
    st = (None, None, 0.0)
    for d in (30.0, 20.0, 10.0, 2.0, 6.0, 10.0, 14.0):
      st = m.icbm_falsify_tick(st[0], st[1], st[2], 0.25, d, 15.0, GENTLE * V ** 2, V, 0.0)[:3]
    return st

  @pytest.mark.parametrize("kw,why", [
    ({"ach_lat": None}, "unreadable"),
    ({"ach_lat": float("nan")}, "unreadable"),
    ({"v_ego": None}, "gap"),
    ({"v_ego": m.ICBM_FALSIFY_MIN_V - 0.1}, "unreadable"),
    ({"target": None}, "noTarget"),
    ({"target": 0.0}, "noTarget"),
    ({"cand_dist": None}, "noDistance"),
    ({"cand_dist": float("inf")}, "noDistance"),
    ({"meas_age": m.ICBM_FALSIFY_STALE_S}, "stale"),
    ({"meas_age": None}, "stale"),
  ])
  def test_it_resets_the_hold_and_names_why(self, kw, why):
    st = self._arrived()
    assert st[2] > 0.0, "the setup never accrued a hold -- proves nothing"
    args = {"cand_dist": 15.0, "target": 15.0, "ach_lat": GENTLE * V ** 2, "v_ego": V, "meas_age": 0.0}
    args.update(kw)
    _, _, held, fired, got, _, _ = m.icbm_falsify_tick(st[0], st[1], st[2], 0.25, **args)
    assert (got, fired, held) == (why, False, 0.0)

  def test_a_frozen_measurement_can_never_complete_a_hold(self):
    """SteerLimitStatus carries NO timestamp: if controlsd stops publishing, kActl freezes at its last value and
    stays perfectly readable -- and a frozen small curvature would BUY A CAP REMOVAL. STALE_S is tied to HOLD_S and
    the test is `age + dt`, so the hold provably cannot complete without the reading changing: the worst case here is
    a value that froze at the very tick the hold began, which is the strongest case a freeze can make."""
    assert m.ICBM_FALSIFY_STALE_S <= m.ICBM_FALSIFY_HOLD_S
    st, age = self._arrived(), 0.0                      # held is already accruing when the publisher freezes
    assert st[2] > 0.0
    d = 14.0
    for _ in range(40):
      d, age = d + 1.0, age + 0.25                      # the reading never changes: its age only grows
      min_d, prev_d, held, fired, _, _, _ = m.icbm_falsify_tick(
        st[0], st[1], st[2], 0.25, d, 15.0, GENTLE * V ** 2, V, age)
      st = (min_d, prev_d, held)
      assert not fired, "a frozen kActl bought a cap removal"
    assert st[2] == 0.0, "the hold never reset -- the staleness test is not what stopped the abort"

  def test_a_healthy_one_hz_refresh_is_never_called_stale(self):
    """_read_map refreshes _sl_k_actl at ~1 Hz while the gate runs at ~4 Hz, so the SAME reading is seen about four
    ticks running in normal operation. That must not read as a freeze, or the rule is inert on healthy data."""
    st, age = self._arrived(), 0.0
    d, whys = 14.0, []
    for i in range(40):
      d, age = d + 1.0, (0.0 if i % 4 == 0 else age + 0.25)     # a fresh publish every 4th tick
      min_d, prev_d, held, fired, why, _, _ = m.icbm_falsify_tick(
        st[0], st[1], st[2], 0.25, d, 15.0, GENTLE * V ** 2, V, age)
      st, whys = (min_d, prev_d, held), whys + [why]
      if fired:
        break
    assert "stale" not in whys, whys
    assert fired, "a healthy 1 Hz refresh never reached an abort"

  def test_the_speed_floor_is_where_the_design_put_it(self):
    """Literal speeds, not the constant: |a_lat|/v^2 is dominated by yaw noise at crawl speed, and the rule is only
    validated above MIN_V. A test written against the constant moves with it and pins nothing."""
    assert m.ICBM_FALSIFY_MIN_V == 8.0
    assert m.icbm_measured_curvature(0.5, 5.0) is None
    assert m.icbm_measured_curvature(0.5, 7.9) is None
    assert m.icbm_measured_curvature(0.5, 9.0) is not None

  def test_the_pure_rule_never_raises_on_anything(self):
    for bad in (None, "x", float("nan"), float("inf"), -1.0, object()):
      m.icbm_falsify_tick(bad, bad, bad, bad, bad, bad, bad, bad, bad)


# ---------------------------------------------------------------------------------------------------------
# through the real CESController._icbm_step
# ---------------------------------------------------------------------------------------------------------
@pytest.fixture(autouse=True)
def _default_curve_cfg(tmp_path, monkeypatch):
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(tmp_path / "absent.json"))
  monkeypatch.setattr(pv, "RAIN_CONFIG_PATH", str(tmp_path / "absent-rain.json"))


def _drive_k(c, step, pubs, clock, pts, y0, seconds, k_actl, v=V, v_set=VSET, vis=None, freeze=False):
  """test_behindgate2pnw._drive with the MEASURED curvature added: `k_actl` is a constant or a f(y) -> 1/m, written
  where controlsd's SteerLimitStatus read puts it. The reading is dithered in its last (6th) decimal unless
  freeze=True, because a real yaw-rate-derived kActl on a moving truck does not repeat -- a bit-identical one is what
  the staleness guard exists to catch. -> [(t, y, published, src, icbmGate, phase)]"""
  out, t0, n_tick = [], clock[0], 0
  while clock[0] - t0 < seconds - 1e-9:
    y = y0 + v * (clock[0] - t0)
    c._map_targets = pts
    c._cur_lat, c._cur_lon = _ll(0.0, y)
    c._cur_bearing = 0.0
    c._gps_fix_ts = clock[0] - m.ICBM_GPS_LAG_KEEP_S
    k = k_actl(y) if callable(k_actl) else k_actl
    c._sl_k_actl = round(k if freeze else k + 1e-6 * (n_tick % 5), 6)
    n_tick += 1
    mtv, mtd = m.upcoming_curve(pts, c._cur_lat, c._cur_lon, v, m.C.CURVE_MAP_LOOKAHEAD_S)
    va, ttc = vis(y) if vis is not None else (0.0, 10.0)
    sig = {"v_ego": v, "v_set": v_set, "map_target_v": mtv, "map_target_dist": mtd,
           "curve_lat_accel_vision": va, "time_to_curve": ttc, "lat_accel_now": 0.0, "has_lead": False,
           "lead_drel": 0.0, "lead_vlead": 0.0, "gas": False, "brake": False, "spd_lim": 0.0, "pitch": None,
           "vis_k_max": None, "vis_reach": 0.0}
    n = len(pubs)
    c._icbm_last_pub = -1e9
    step(sig, active=True)
    p = pubs[-1][1] if len(pubs) > n else {}
    out.append((round(clock[0] - t0, 3), round(y, 2), p, c._icbm_src, c._icbm_gate, c._icbm_ep.phase))
    clock[0] = round(clock[0] + 0.25, 6)
  return out


def _scene(pts, seconds, k_actl, t0=100.0, falsify=True, behindrun=False, k_freeze=False, **kw):
  """One drive north from y=0. behindrun=False makes icbm_passed_points say "cannot tell", which is the ONLY state in
  which this rule can act on the shipped code (see TestPreemptedByBehindrun). -> (trace, events, controller)"""
  mp = pytest.MonkeyPatch()
  try:
    mp.setattr(pv, "CURVE_CONFIG_PATH", "/nonexistent/curve.json")
    mp.setattr(pv, "RAIN_CONFIG_PATH", "/nonexistent/rain.json")
    if not behindrun:
      _gate_off(mp)
    if not falsify:
      mp.setattr(m, "_icbm_falsify_gate", lambda ctl, now, target, *a, **k: target)
    clock, ev = [t0], []
    c, step, pubs = _controller(mp, clock, ev)
    return _drive_k(c, step, pubs, clock, pts, 0.0, seconds, k_actl, freeze=k_freeze, **kw), ev, c
  finally:
    mp.undo()


def _capped(trace):
  return [t for t, _, p, *_ in trace if p.get("target") is not None and p.get("dir", "dec") == "dec"]


def _falsify_ev(ev, state=None):
  return [kw for _, n, kw in ev if n == "ces_icbm_falsify" and (state is None or kw.get("state") == state)]


ONE_NODE = _road(passed_curve=False, ahead_curve_at=100.0, v_curve=15.0, back=200.0)


class TestItEndsACapTheTruckHasMeasuredWrong:
  def test_a_map_claim_the_road_does_not_support_ends_the_cap(self):
    base, _, _ = _scene(ONE_NODE, 14.0, GENTLE, falsify=False)
    gated, ev, _ = _scene(ONE_NODE, 14.0, GENTLE)
    assert _capped(base), "nothing capped at all -- proves nothing"
    assert len(_capped(gated)) < len(_capped(base)), \
      f"the cap did not end (base {len(_capped(base))} ticks, gated {len(_capped(gated))})"
    assert any(g == "mapFalsified" for *_, g, _ in gated)
    end = _falsify_ev(ev, "end")
    assert len(end) == 1, _falsify_ev(ev)
    assert end[0]["kMeas"] * m.ICBM_FALSIFY_RATIO < end[0]["kMap"]
    assert end[0]["held"] == pytest.approx(m.ICBM_FALSIFY_HOLD_S)
    assert end[0]["src"] == "map"

  def test_the_approach_and_the_curve_itself_are_untouched(self):
    """Nothing may change until the truck is past the node: the rule is a MEASUREMENT, and before the node there is
    nothing to measure. A start is never gated."""
    base, _, _ = _scene(ONE_NODE, 14.0, GENTLE, falsify=False)
    gated, _, _ = _scene(ONE_NODE, 14.0, GENTLE)
    k = next(i for i, r in enumerate(base) if r[1] > 100.0 + m.ICBM_FALSIFY_PAST_M)
    assert any(r[2].get("target") is not None for r in base[:k]), "no cap before the node -- proves nothing"
    assert [r[2] for r in gated[:k]] == [r[2] for r in base[:k]]

  @pytest.mark.parametrize("k", [REAL, -REAL], ids=["left", "right"])
  def test_a_real_curve_at_the_same_node_is_never_aborted(self, k):
    """The same road, the same map claim, the same drive -- only the truck actually measures the bend. Identical.

    BOTH DIRECTIONS (Fable 2026-09-17): achLat is signed, and a rule that cancelled real slowdowns on
    right-handers only would be a one-sided failure nobody would think to look for. The `right` case
    fails if the abs() in icbm_measured_curvature is ever dropped."""
    base, _, _ = _scene(ONE_NODE, 14.0, k, falsify=False)
    gated, ev, _ = _scene(ONE_NODE, 14.0, k)
    assert _capped(base), "nothing capped -- proves nothing"
    assert [r[2] for r in gated] == [r[2] for r in base]
    assert _falsify_ev(ev, "end") == []

  def test_it_only_ever_ends_a_cap_and_never_lowers_one(self):
    base, _, _ = _scene(ONE_NODE, 14.0, GENTLE, falsify=False)
    gated, _, _ = _scene(ONE_NODE, 14.0, GENTLE)
    for (_, _, pb, *_), (_, _, pg, *_) in zip(base, gated, strict=True):
      if pb != pg:
        assert pg.get("target") is None or pg["target"] >= pb.get("target", 0.0), (pb, pg)

  def test_the_same_falsified_node_may_restart_the_episode_and_that_is_the_stated_limit(self):
    """KNOWN LIMIT, pinned so it cannot change silently. The abort ends THIS cap; it does not suppress the feature
    (§5), and nothing remembers which node was falsified across an episode boundary. So once the episode leaves its
    cap phase the SAME node -- still binding, because nothing else removes it in this scene -- starts a new one, and
    the rule cannot re-arm on it (its distance only grows now, so min_d never reaches PASS_M again). The cap is
    therefore cut once, not permanently. On the truck behindgate2pnw normally blocks that restart; this scene is the
    case where it cannot tell, which is the only case this rule is reachable in at all."""
    gated, _, _ = _scene(ONE_NODE, 14.0, GENTLE)
    after = [(t, p.get("target")) for t, _, p, *_ in gated if t > 9.5]
    assert any(tgt is not None for _, tgt in after), "the cap never came back -- the limit has changed, re-read §5"
    assert [t for t, _, p, *_ in gated if p.get("target") is None], "the cap never ended -- proves nothing"


def _two_node_road():
  """node A at 100 m (the map claims 15 m/s; the road is straight) and node B at 400 m (12 m/s, a REAL bend). B is
  outside the 250 m horizon at the start, so A binds first; B comes into range while the truck is past A."""
  pts = _road(passed_curve=False, ahead_curve_at=100.0, v_curve=15.0, back=200.0)
  far = _road(passed_curve=False, ahead_curve_at=400.0, v_curve=12.0, back=200.0)
  return [dict(p, velocity=(f["velocity"] or p["velocity"])) for p, f in zip(pts, far, strict=True)]


class TestSection5ANodeStillAheadKeepsBinding:
  """The requirement the design told the implementer to settle and TEST EXPLICITLY: ending the cap must not blind the
  truck to a node still ahead."""

  def test_a_two_node_curve_whose_first_node_is_falsified_stays_capped_on_the_second(self):
    one, _, _ = _scene(ONE_NODE, 10.0, GENTLE)
    two, ev, _ = _scene(_two_node_road(), 10.0, GENTLE)
    assert [t for t, _, p, *_ in one if p.get("target") is None], \
      "the single-node control never lost its cap -- the two-node case proves nothing"
    assert all(p.get("target") is not None for _, _, p, *_ in two), \
      f"a node still AHEAD lost its cap: {[(t, p.get('target')) for t, _, p, *_ in two if p.get('target') is None]}"
    assert _falsify_ev(ev, "end") == [], "the second node was falsified on curvature measured at the first"

  def test_the_second_node_is_what_the_cap_comes_from_once_it_binds(self):
    two, _, _ = _scene(_two_node_road(), 10.0, GENTLE)
    first = two[0][2]["target"]
    assert two[-1][2]["target"] < first - 1.0, \
      f"the tighter node ahead never took over the cap ({first} -> {two[-1][2]['target']})"

  def test_the_tick_after_an_abort_is_free_to_cap_again_from_a_candidate_still_ahead(self):
    """The abort ends a cap, it does not disable icbm_curve_target: with a real bend appearing behind the falsified
    node the cap comes straight back, lower than the one that was ended."""
    gated, ev, _ = _scene(ONE_NODE, 14.0, GENTLE)
    end_t = next(t for t, _, _, _, g, _ in gated if g == "mapFalsified")
    back = [(t, p["target"]) for t, _, p, *_ in gated if t > end_t and p.get("target") is not None]
    assert back, "nothing ever capped again after the abort -- the rule suppressed the feature"
    assert _falsify_ev(ev, "end")


class TestTheResetClearsEverything:
  """_icbm_falsify_reset runs from __init__, from EVERY tick with no map/far cap running, and when ICBM goes
  inactive. Every piece of arrival evidence has to go: a min_d carried into the next candidate would call a curve
  "passed" that is still ahead, which is the §5 failure in its most direct form."""

  STATE = ("_icbm_falsify_min_d", "_icbm_falsify_prev_d", "_icbm_falsify_held", "_icbm_falsify_t",
           "_icbm_falsify_k", "_icbm_falsify_k_t", "_icbm_falsify_on", "_icbm_falsify_state")

  def test_every_field_the_gate_writes_is_cleared(self):
    from types import SimpleNamespace
    ctl = SimpleNamespace()
    m._icbm_falsify_reset(ctl)
    assert set(self.STATE) <= set(vars(ctl)), \
      f"the reset does not touch {sorted(set(self.STATE) - set(vars(ctl)))}"
    for k in self.STATE:                       # dirty every field, then reset again
      setattr(ctl, k, 123.0)
    m._icbm_falsify_reset(ctl)
    assert vars(ctl) == {"_icbm_falsify_min_d": None, "_icbm_falsify_prev_d": None, "_icbm_falsify_held": 0.0,
                         "_icbm_falsify_t": None, "_icbm_falsify_k": None, "_icbm_falsify_k_t": None,
                         "_icbm_falsify_on": False, "_icbm_falsify_state": None}, vars(ctl)

  def test_a_real_controller_starts_with_the_state_already_cleared(self):
    """The REAL class, built the way test_behindgate2pnw/test_curvelead2pnw build one. The shared _controller()
    harness is a SimpleNamespace that never runs CESController.__init__, so it cannot see this property at all: an
    unseeded field would leave the first tick reading getattr() fallbacks instead of state something cleared."""
    from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_behindgate2pnw import LIGHTNING
    from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_curvelead2pnw import FakeCP

    class P:
      def get(self, k, return_default=False):
        return {"CESMode": "2", "CESButtonState": "0"}.get(k)

      def get_bool(self, k):
        return k == "CESCurves"

    c = m.CESController(FakeCP(LIGHTNING, "ford", False), params=P())
    assert [getattr(c, k, "MISSING") for k in self.STATE] == [None, None, 0.0, None, None, None, False, None]
    assert getattr(c, "_icbm_falsify_err", "MISSING") is None, "the fail-open log's throttle is unseeded"

  def test_a_tick_with_no_running_map_cap_leaves_no_evidence_behind(self):
    """§5 in its most direct form, and the bug this file's first version shipped with. The arrival evidence belongs
    to the map/far cap that is RUNNING. The moment there is not one it has to go, or the next candidate -- which
    can be a DIFFERENT node, one the truck has NOT driven to -- inherits a min_d measured back at a node already
    behind and reads "arrived" immediately.

    The first implementation reset only when the EPISODE left its cap phase, which left exactly this gap open: a
    cap can stop binding, or be bound by vision / the stale-GPS hold, for a few ticks while the phase is held by
    the S-gap clear debounce. Nothing inside icbm_falsify_tick covers a SHORT gap: the wall-time re-anchor needs
    ICBM_FALSIFY_MAX_DT_S (> 4 ticks) and the newPoint jump test allows v_ego*dt + JUMP_M, which is 28 m of slack
    across the two-tick interlude used here."""
    snaps, calls, tick = [], [], [0]
    mp = pytest.MonkeyPatch()
    try:
      mp.setattr(pv, "CURVE_CONFIG_PATH", "/nonexistent/curve.json")
      mp.setattr(pv, "RAIN_CONFIG_PATH", "/nonexistent/rain.json")
      _gate_off(mp)
      real = m._icbm_falsify_gate
      mp.setattr(m, "_icbm_falsify_gate", lambda *a, **k: (calls.append(1), real(*a, **k))[1])
      clock, ev = [1900.0], []
      c, step, pubs = _controller(mp, clock, ev)

      def spy(sig, active=True):
        n = len(calls)
        out = step(sig, active=active)
        snaps.append((len(calls) > n, c._icbm_ep.phase, c._icbm_src, getattr(c, "_icbm_falsify_min_d", "MISSING")))
        return out

      def vis(_y):                                  # ticks 21-22 only: a tighter vision curve takes the cap
        tick[0] += 1
        return (12.0, 3.0) if 21 <= tick[0] <= 22 else (0.0, 10.0)

      _drive_k(c, spy, pubs, clock, ONE_NODE, 0.0, 9.0, GENTLE, vis=vis)
    finally:
      mp.undo()
    assert any(g for g, *_ in snaps), "the rule never ran at all -- proves nothing"
    assert [i for i, (g, ph, src, _) in enumerate(snaps) if not g and ph == "cap" and src == "vis"], \
      "no vision-bound tick inside a running cap episode -- the gap this test exists for never opened"
    left = [(i, ph, src, d) for i, (g, ph, src, d) in enumerate(snaps) if not g and d is not None]
    assert left == [], f"arrival evidence survived ticks with no running map/far cap: {left}"


class TestWhatIsUntouched:
  def test_a_vision_bound_cap_never_reaches_the_rule(self):
    """§6: a vision-sourced cap is not falsifiable this way and must be left alone."""
    calls = []
    mp = pytest.MonkeyPatch()
    try:
      mp.setattr(pv, "CURVE_CONFIG_PATH", "/nonexistent/curve.json")
      mp.setattr(pv, "RAIN_CONFIG_PATH", "/nonexistent/rain.json")
      _gate_off(mp)
      real = m._icbm_falsify_gate
      mp.setattr(m, "_icbm_falsify_gate", lambda *a, **k: (calls.append(a[3]), real(*a, **k))[1])
      clock, ev = [1200.0], []
      c, step, pubs = _controller(mp, clock, ev)
      tr = _drive_k(c, step, pubs, clock, _road(passed_curve=False, back=100.0, fwd=60.0), 0.0, 6.0, GENTLE,
                    vis=lambda y: (4.0, 5.0))
    finally:
      mp.undo()
    assert _starts(tr), "vision never bound -- proves nothing"
    assert all(r[3] == "vis" for r in tr if r[2].get("target") is not None), [r[3] for r in tr]
    assert calls == [], f"the rule ran on a vision-bound cap: {calls}"
    assert all(r[4] != "mapFalsified" for r in tr)

  def test_a_start_is_never_gated(self):
    """The rule may only ever END a running slowdown -- it must not be able to stop one from beginning. Every call it
    makes has to come from a tick whose episode was already in its cap phase."""
    phases = []
    mp = pytest.MonkeyPatch()
    try:
      mp.setattr(pv, "CURVE_CONFIG_PATH", "/nonexistent/curve.json")
      mp.setattr(pv, "RAIN_CONFIG_PATH", "/nonexistent/rain.json")
      _gate_off(mp)
      real = m._icbm_falsify_gate
      mp.setattr(m, "_icbm_falsify_gate", lambda ctl, *a, **k: (phases.append(ctl._icbm_ep.phase), real(ctl, *a, **k))[1])
      clock, ev = [1300.0], []
      c, step, pubs = _controller(mp, clock, ev)
      _drive_k(c, step, pubs, clock, ONE_NODE, 0.0, 14.0, GENTLE)
    finally:
      mp.undo()
    assert phases, "the rule never ran -- proves nothing"
    assert set(phases) == {"cap"}, set(phases)

  def test_the_tesla_never_reaches_the_rule_because_icbm_is_a_lightning_capability(self):
    """Capability, never a fingerprint: the Tesla has no button_management/ICBM, so _icbm_step publishes nothing and
    the rule is unreachable. (The Tesla CES+VTSC consumer hash is checked separately, outside pytest.)"""
    from types import SimpleNamespace

    from openpilot.selfdrive.controls.lib.pnw_vehicle import PnwVehicle
    tesla = PnwVehicle(SimpleNamespace(carFingerprint="TESLA_MODEL_S_RAVEN", brand="tesla",
                                       openpilotLongitudinalControl=True, dashcamOnly=False))
    assert not tesla.button_management, "the Tesla grew ICBM -- this rule's gating must be re-examined"


class TestFailOpen:
  def test_a_crash_keeps_the_running_cap_logs_once_and_names_the_running_case(self):
    """_icbm_step runs in selfdrived (restart_if_crash=False). A defect must cost the rule, never the cap and never
    the IcbmTarget publish -- and fail-open for THIS rule means KEEPING the slowdown."""
    logged = []
    mp = pytest.MonkeyPatch()
    try:
      mp.setattr(pv, "CURVE_CONFIG_PATH", "/nonexistent/curve.json")
      mp.setattr(pv, "RAIN_CONFIG_PATH", "/nonexistent/rain.json")
      _gate_off(mp)
      mp.setattr(m.cloudlog, "exception", lambda msg, *a, **k: logged.append(msg))
      clock, ev = [1600.0], []
      c, step, pubs = _controller(mp, clock, ev)
      ok = _drive_k(c, step, pubs, clock, ONE_NODE, 0.0, 1.0, GENTLE)
      assert ok[0][2].get("target") is not None and not logged, ok[0]
      mp.setattr(m, "icbm_falsify_tick", lambda *a, **k: (_ for _ in ()).throw(RuntimeError("boom")))
      broke = _drive_k(c, step, pubs, clock, ONE_NODE, 25.0, 12.0, GENTLE)
    finally:
      mp.undo()
    assert all(r[2].get("target") is not None for r in broke), "a crash in the rule ended or stopped the cap"
    assert sum("icbmfalsify2pnw" in s for s in logged) == 1, logged
    assert "CONTINUES" in logged[0], logged[0]
    assert not any("_icbm_step FAILED" in s for s in logged), "the crash escaped into _icbm_step's except"


class TestTelemetry:
  def test_icbmGate_mapFalsified_reaches_the_record(self):
    assert _record(_icbm_gate="mapFalsified")["icbmGate"] == "mapFalsified"

  def test_the_end_record_carries_the_numbers_that_justify_it(self):
    """§7: a cap that silently disappears is exactly the failure this project forbids -- the driver feels the truck
    stop slowing and nothing explains it."""
    _, ev, _ = _scene(ONE_NODE, 14.0, GENTLE)
    end = _falsify_ev(ev, "end")
    assert len(end) == 1, _falsify_ev(ev)
    assert set(end[0]) == {"state", "why", "src", "kMap", "kMeas", "ratio", "dist", "minD", "target", "held"}
    assert end[0]["ratio"] >= m.ICBM_FALSIFY_RATIO
    assert end[0]["minD"] <= m.ICBM_FALSIFY_PASS_M and end[0]["dist"] > end[0]["minD"] + m.ICBM_FALSIFY_PAST_M

  def test_it_is_change_only_and_says_when_it_is_blind(self):
    """Rule 2: "arrived but cannot measure" is its own fact and is logged once, not silently treated as "no curve"."""
    _, ev, _ = _scene(ONE_NODE, 14.0, GENTLE, k_freeze=True)
    assert _falsify_ev(ev, "end") == [], "a frozen reading ended a cap"
    blind = _falsify_ev(ev, "blind")
    assert len(blind) == 1 and blind[0]["why"] == "stale", _falsify_ev(ev)


# ---------------------------------------------------------------------------------------------------------
# real telemetry windows -- the same seven drives behindrun2pnw was validated on, with the MEASURED curvature
# (kActl) joined on from the raw ces_events (builder:
# drives/2026-09-12/central-oregon-weekend/icbmfalsify_make_fixture.py)
# ---------------------------------------------------------------------------------------------------------
def _windows():
  with open(FIXTURE) as f:
    return json.load(f)["windows"]


def _replay(w, falsify=True, behindrun=True):
  """the real _icbm_step at 4 Hz over a logged window (the harness test_behindrun2pnw._replay uses, plus kActl).
  -> ([(t, published, src, icbmGate, phase, wall_t)], ces_icbm_falsify events)"""
  from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_behindrun2pnw import _run_gate_off
  mp = pytest.MonkeyPatch()
  try:
    mp.setattr(pv, "CURVE_CONFIG_PATH", "/nonexistent/curve.json")
    mp.setattr(pv, "RAIN_CONFIG_PATH", "/nonexistent/rain.json")
    if not behindrun:
      _run_gate_off(mp)
    if not falsify:
      mp.setattr(m, "_icbm_falsify_gate", lambda ctl, now, target, *a, **k: target)
    clock, ev = [w["records"][0]["t"]], []
    c, step, pubs = _controller(mp, clock, ev)
    trace, recs = [], w["records"]
    for n, r in enumerate(recs):
      t_next = recs[n + 1]["t"] if n + 1 < len(recs) and recs[n + 1]["t"] - r["t"] <= 2.0 else r["t"] + 1.0
      pts = [{"latitude": a, "longitude": b, "velocity": v} for a, b, v in w["paths"][r["path"]]]
      while clock[0] < t_next - 1e-6:
        c._map_targets = pts
        c._cur_lat, c._cur_lon, c._cur_bearing = r["lat"], r["lon"], r["bearing"] or 0.0
        c._gps_fix_ts = r["t"] - m.ICBM_GPS_LAG_KEEP_S
        c._stock_set, c._stock_on = r["stockSet"], bool(r["stockOn"])
        c._sl_k_actl = r["kActl"]
        v = r["vEgo"]
        mtv, mtd = m.upcoming_curve(pts, r["lat"], r["lon"], v, m.C.CURVE_MAP_LOOKAHEAD_S)
        sig = {"v_ego": v, "v_set": r["vSet"], "map_target_v": mtv, "map_target_dist": mtd,
               "curve_lat_accel_vision": r["visLat"] or 0.0, "time_to_curve": r["visTtc"] or 10.0,
               "lat_accel_now": 2.0 if r["icbmGate"] == "inCurve" else 0.0, "has_lead": bool(r["lead"]),
               "lead_drel": r["dRel"] or 0.0, "lead_vlead": r["vLead"] or 0.0, "gas": bool(r["gas"]),
               "brake": False, "spd_lim": r["spdLim"] or 0.0, "pitch": None, "vis_k_max": None, "vis_reach": 0.0}
        k = len(pubs)
        c._icbm_last_pub = -1e9
        step(sig, active=True)
        p = pubs[-1][1] if len(pubs) > k else {}
        trace.append((round(clock[0] - w["t_start"], 3), p, c._icbm_src, c._icbm_gate, c._icbm_ep.phase, r["t"]))
        clock[0] = round(clock[0] + 0.25, 6)
    return trace, ev
  finally:
    mp.undo()


def _rcapped(trace):
  return [t for t, p, *_ in trace if p.get("target") is not None and p.get("dir", "dec") == "dec"]


class TestPreemptedByBehindrun:
  """THE FINDING A READER MUST NOT MISS. behindrun2pnw shipped 2026-09-15 and removes a passed map point as soon as
  it is ICBM_PASSED_TOL_M (5 m) behind along mapd's path. This rule cannot arm until the candidate has receded
  ICBM_FALSIFY_PAST_M (10 m) from its closest approach -- geometrically ~12-25 m past the point -- and then needs
  HOLD_S more. behindrun's condition is therefore ALWAYS strictly earlier, so wherever icbm_passed_points can tell,
  this rule is dead code. It measured 'ok' on 23,826 of 23,828 moving weekend ticks (behindgate's
  result_osm_predicates.txt: {'ok': 23826, 'reversed': 2, 'slow': 1866}), i.e. it can tell 99.99 % of the time.

  The design's 35-abort / 759 s replay corpus was recorded 2026-09-11..13, BEFORE behindrun2pnw existed."""

  @pytest.mark.parametrize("w", _windows(), ids=lambda w: w["name"])
  def test_on_the_shipped_code_the_rule_changes_nothing(self, w, monkeypatch):
    base, _ = _replay(w, falsify=False)
    # Count the ticks the rule actually EVALUATED. Without this the class passes vacuously the day a
    # change makes the gate unreachable -- "it changed nothing" would then be true for the wrong
    # reason and the tripwire would have stopped being one without saying so (Fable 2026-09-17).
    # It fails in the safe direction, but a tripwire that cannot trip is not a tripwire.
    reached = []
    real_tick = m.icbm_falsify_tick
    monkeypatch.setattr(m, "icbm_falsify_tick",
                        lambda *a, **k: (reached.append(1), real_tick(*a, **k))[1])
    gated, ev = _replay(w, falsify=True)
    assert _rcapped(base), f'{w["when_pt"]}: nothing caps here -- proves nothing'
    assert reached, f'{w["when_pt"]}: the rule was never evaluated -- this window proves nothing'
    assert [(t, p) for t, p, *_ in gated] == [(t, p) for t, p, *_ in base], \
      f'{w["when_pt"]}: the rule acted with behindrun2pnw live -- re-read this class and re-measure the overlap'
    assert _falsify_ev(ev, "end") == []


class TestTheRuleItselfOnRealDrives:
  """...and with behindrun2pnw's RUNNING half switched off -- which is both the pre-2026-09-15 code the design was
  measured on AND the live case it declines (icbm_passed_points cannot tell) -- the rule does exactly what the design
  claims, through the real _icbm_step."""

  def test_the_2026_09_08_phantom_ends_at_20_29_04_PT(self):
    """§8, and the one OUT-OF-SAMPLE case: mapd claimed a curve on a straight 60 mph motorway and ICBM dragged the
    set 70 -> 44 mph. The design predicts the abort at 20:29:04; this is the real _icbm_step agreeing."""
    import datetime
    import zoneinfo
    w = next(x for x in _windows() if x["name"] == "sep08_2028_run")
    base, _ = _replay(w, falsify=False, behindrun=False)
    gated, ev = _replay(w, falsify=True, behindrun=False)
    assert _rcapped(base), "the phantom does not cap here -- proves nothing"
    end = _falsify_ev(ev, "end")
    assert len(end) == 1, _falsify_ev(ev)
    first = next(t for (t, pb, *_), (_, pg, *_) in zip(base, gated, strict=True) if pb != pg)
    wall = next(r[5] for r in gated if r[0] == first)
    pt = datetime.datetime.fromtimestamp(wall, zoneinfo.ZoneInfo("America/Los_Angeles"))
    assert pt.strftime("%H:%M:%S") == "20:29:04", pt
    assert len(_rcapped(gated)) < len(_rcapped(base))
    assert end[0]["kMap"] > end[0]["kMeas"] * m.ICBM_FALSIFY_RATIO

  @pytest.mark.parametrize("w", [x for x in _windows() if x["kind"] == "unchanged"], ids=lambda w: w["name"])
  def test_a_real_curve_is_never_aborted(self, w):
    """The three control windows: a real curve still ahead, capped for good reason. Zero false aborts -- the number
    the whole design rests on."""
    base, _ = _replay(w, falsify=False, behindrun=False)
    gated, ev = _replay(w, falsify=True, behindrun=False)
    assert _rcapped(base), f'{w["when_pt"]}: nothing caps -- proves nothing'
    assert [(t, p) for t, p, *_ in gated] == [(t, p) for t, p, *_ in base], f'{w["when_pt"]} ({w["why"]})'
    assert _falsify_ev(ev, "end") == []

  def test_it_acts_on_the_run_windows_and_only_after_the_start(self):
    acted = 0
    for w in [x for x in _windows() if x["kind"] == "run"]:
      base, _ = _replay(w, falsify=False, behindrun=False)
      gated, ev = _replay(w, falsify=True, behindrun=False)
      if not _falsify_ev(ev, "end"):
        continue
      acted += 1
      first = next(t for (t, pb, *_), (_, pg, *_) in zip(base, gated, strict=True) if pb != pg)
      assert first > 0.0, f'{w["when_pt"]}: the rule changed the START -- a start is never gated'
      assert len(_rcapped(gated)) < len(_rcapped(base)), w["when_pt"]
    assert acted >= 3, f"the rule acted on only {acted} of the run windows -- the fixture proves too little"
