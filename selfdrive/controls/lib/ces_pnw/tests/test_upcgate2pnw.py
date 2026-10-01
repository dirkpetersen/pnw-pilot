"""upcgate2pnw: the CES curve scan (`upcoming_curve`) no longer binds to a map point the truck has already driven past
on a car with no ICBM gate (the Tesla), and the curve-DB candidate pool no longer gets an unfiltered far/map candidate
(the cross-source hand-off).

WHY. mapd publishes its current way from the way's FIRST node, so the stretch just driven stays in MapTargetVelocities,
and the scanners' distance is unsigned. The Lightning's ICBM is protected by `_icbm_passed_gate` (behindgate2pnw /
behindrun2pnw); the Tesla's CES curve trip was not. Tesla 2026-09-30 22:36:18-23, the OR-34 -> I-5 loop ramp: mode
experimental / reason "curve" / curveSrc "map" on a 21 mph node 160-170 deg BEHIND the truck, 223 -> 289 m and receding
at ~28 m/s (the real record is the fixture tests below). The same signature on 09-29 08:02 and 14:10.

What is pinned: the helper (drops passed points, same list when nothing is passed, fail-open + loud when it cannot tell
or throws, cached across the 100 Hz cycle), the Tesla end to end (a passed curve does not bind, a curve ahead still
does, no passed points -> identical output), the Lightning's raw CES scan untouched, and the cross-source case through
the real curve DB.
"""
from __future__ import annotations

import json
import os
import types

import pytest

from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
from openpilot.selfdrive.controls.lib.ces_pnw import curvedb_live as cl
from openpilot.selfdrive.controls.lib.ces_pnw.tests import test_curvedblive2pnw as cdb
from openpilot.selfdrive.controls.lib.ces_pnw.tests import test_behindgate2pnw as bg
# (not the Test* classes by name: pytest would collect them here too)
from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_behindgate2pnw import LIGHTNING, _ll, _pts, _road

TESLA = "TESLA_MODEL_S_HW3"
FIXTURE = os.path.join(os.path.dirname(__file__), "data", "upcgate_2026-09-30_tesla_loop_ramp.json")


@pytest.fixture(autouse=True)
def _default_curve_cfg(tmp_path, monkeypatch):
  from openpilot.selfdrive.controls.lib import pnw_vehicle as pv
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(tmp_path / "absent.json"))
  monkeypatch.setattr(pv, "RAIN_CONFIG_PATH", str(tmp_path / "absent-rain.json"))


@pytest.fixture
def events(monkeypatch):
  ev, exc = [], []
  monkeypatch.setattr(m.cloudlog, "event", lambda name, **kw: ev.append((name, kw)))
  monkeypatch.setattr(m.cloudlog, "exception", lambda msg, *a, **k: exc.append(msg))
  return ev, exc


def _ctl(bearing=0.0):
  return types.SimpleNamespace(_cur_bearing=bearing)


def _at(y):
  return _ll(0.0, y)


def _line():
  """A straight path north, a node every 20 m from 300 m behind to 400 m ahead; passed curve 100-60 m behind (6 m/s),
  curve ahead 200-240 m (9 m/s)."""
  return _pts([(0.0, float(y), 6.0 if -100 <= y <= -60 else 9.0 if 200 <= y <= 240 else 0.0) for y in range(-300, 401, 20)])


# ---------------------------------------------------------------------------------------------------------
# the helper
# ---------------------------------------------------------------------------------------------------------
class TestFilter:
  def test_passed_points_are_dropped_and_ahead_ones_stay(self, events):
    pts = _line()
    la, lo = _at(0.0)
    out = m._curve_passed_filter(_ctl(), pts, la, lo, 20.0)
    ys = [round((p["latitude"] - _at(0.0)[0]) * 111320.0) for p in out]
    assert len(out) < len(pts) and min(ys) >= -5, ys          # nothing more than the 5 m tolerance behind remains
    assert max(ys) == 400 and 200 in ys, "points ahead were dropped"
    assert all(any(p is q for q in pts) for p in out), "a point was rebuilt rather than kept"
    assert events[0] == [], "a working mask must log nothing"

  def test_nothing_passed_returns_the_very_same_list(self, events):
    pts = _pts([(0.0, float(y), 0.0) for y in range(0, 401, 20)])    # the path starts at the truck
    la, lo = _at(0.0)
    assert m._curve_passed_filter(_ctl(), pts, la, lo, 20.0) is pts

  def test_a_passed_curve_does_not_bind_the_scan_and_one_ahead_does(self, events):
    la, lo = _at(0.0)
    pts = _line()
    raw = m.upcoming_curve(pts, la, lo, 20.0, 12.0)
    flt = m.upcoming_curve(m._curve_passed_filter(_ctl(), pts, la, lo, 20.0), la, lo, 20.0, 12.0)
    assert raw[0] == 6.0 and raw[1] < 110.0, f"the unfiltered scan must bind the passed curve (control): {raw}"
    assert flt[0] == 9.0 and 195.0 < flt[1] < 215.0, flt

  @pytest.mark.parametrize("kw,why", [({"v": m.ICBM_PASSED_MIN_V - 0.1}, "slow"), ({"bearing": None}, "noHeading"),
                                      ({"bearing": 180.0}, "reversed"), ({"at": 45.0}, "offPath")])
  def test_cannot_tell_is_todays_scan_and_says_so_once(self, events, kw, why):
    ev, _ = events
    pts = _line()
    la, lo = _ll(kw.get("at", 0.0), 0.0)
    ctl = _ctl(kw.get("bearing", 0.0))
    for _ in range(3):
      assert m._curve_passed_filter(ctl, pts, la, lo, kw.get("v", 20.0)) is pts
    assert [(n, k["slot"], k["why"]) for n, k in ev] == [("ces_passed_mask", "ces", why)], ev

  def test_too_many_points_is_unfiltered_and_says_so(self, events):
    ev, _ = events
    many = _pts([(0.0, float(y), 0.0) for y in range(m.ICBM_PASSED_MAX_POINTS + 1)])
    la, lo = _at(500.0)
    assert m._curve_passed_filter(_ctl(), many, la, lo, 20.0) is many
    assert [k["why"] for _, k in ev] == ["tooMany"]

  def test_an_unusable_path_with_points_says_noPath(self, events):
    ev, _ = events
    pts = _pts([(0.0, 0.0, 0.0)])
    la, lo = _at(0.0)
    assert m._curve_passed_filter(_ctl(), pts, la, lo, 20.0) is pts
    assert [k["why"] for _, k in ev] == ["noPath"]

  def test_no_map_or_no_fix_is_not_a_failure_and_logs_nothing(self, events):
    ev, exc = events
    la, lo = _at(0.0)
    assert m._curve_passed_filter(_ctl(), [], la, lo, 20.0) == []
    assert m._curve_passed_filter(_ctl(), _line(), None, None, 20.0) == _line()
    assert ev == [] and exc == []

  def test_the_log_is_change_only_across_a_recovery(self, events):
    ev, _ = events
    pts = _line()
    la, lo = _at(0.0)
    ctl = _ctl()
    for v in (3.0, 3.1, 20.0, 20.5, 3.0):
      m._curve_passed_filter(ctl, pts, la, lo, v)
    assert [k["why"] for _, k in ev] == ["slow", "ok", "slow"]

  def test_a_crash_runs_the_scan_unfiltered_and_logs_loudly_throttled(self, monkeypatch, events):
    _, exc = events
    monkeypatch.setattr(m, "icbm_passed_points", lambda *a, **k: (_ for _ in ()).throw(RuntimeError("boom")))
    clock = [100.0]
    monkeypatch.setattr(m.time, "monotonic", lambda: clock[0])
    pts = _line()
    la, lo = _at(0.0)
    ctl = _ctl()
    for _ in range(5):
      assert m._curve_passed_filter(ctl, pts, la, lo, 20.0) is pts
    assert len(exc) == 1 and "upcgate2pnw" in exc[0] and "WITHOUT" in exc[0]
    clock[0] += m.ICBM_ERR_LOG_S + 1
    m._curve_passed_filter(ctl, pts, la, lo, 20.0)
    assert len(exc) == 2

  def test_the_mask_is_computed_once_per_fix_not_once_per_cycle(self, monkeypatch, events):
    calls = []
    real = m.icbm_passed_points
    monkeypatch.setattr(m, "icbm_passed_points", lambda *a, **k: (calls.append(1), real(*a, **k))[1])
    pts = _line()
    ctl = _ctl()
    la, lo = _at(0.0)
    for _ in range(100):                                     # 100 Hz cycles on one ~1 Hz fix
      m._curve_passed_filter(ctl, pts, la, lo, 20.0)
    assert len(calls) == 1
    la2, lo2 = _at(30.0)                                     # a new fix recomputes
    m._curve_passed_filter(ctl, pts, la2, lo2, 20.0)
    m._curve_passed_filter(ctl, list(pts), la2, lo2, 20.0)   # a new list (the next map read) recomputes
    m._curve_passed_filter(ctl, pts, la2, lo2, 4.0)          # crossing the speed floor changes the answer
    assert len(calls) == 4


# ---------------------------------------------------------------------------------------------------------
# the real record: Tesla 2026-09-30 22:36 PT loop ramp
# ---------------------------------------------------------------------------------------------------------
class TestRealLoopRamp:
  """mapdPath record and the logged position/heading/speed of the Tesla's ticks on 2026-09-30 (the loop ramp at the
  OR-34 -> I-5 interchange; every coordinate shifted by a constant, the geometry intact). The logged tick said
  experimental / curve / map at mapV 9.4 with the point 160-170 deg BEHIND the truck, 223 -> 289 m and receding."""

  @staticmethod
  def _scenes():
    with open(FIXTURE) as f:
      return {s["name"]: s for s in json.load(f)["scenes"]}

  @staticmethod
  def _scan(scene, t, filtered):
    pts = [{"latitude": a, "longitude": b, "velocity": float("nan") if v is None else v} for a, b, v in scene["path"]]
    if filtered:
      pts = m._curve_passed_filter(_ctl(t["bearing"]), pts, t["lat"], t["lon"], t["vEgo"])
    return m.upcoming_curve(pts, t["lat"], t["lon"], t["vEgo"], m.C.CURVE_MAP_LOOKAHEAD_S)

  def test_the_replay_reproduces_what_the_tesla_logged(self):
    """Fidelity first: the unfiltered scan on the logged geometry gives the logged mapV and mapDist on every tick, and
    those ticks are the bug (Experimental / curve / map). mapDist to 20 m: the logged distance comes from the control
    cycle's own GPS read, the record's position from the writer's (22:36:21 differs by 14 m); mapV is exact."""
    for scene in self._scenes().values():
      for t in scene["ticks"]:
        v, d = self._scan(scene, t, filtered=False)
        assert v == pytest.approx(t["mapV"], abs=0.05) and d == pytest.approx(t["mapDist"], abs=20.0), (scene["name"], t, v, d)
      assert any((t["mode"], t["reason"], t["curveSrc"]) == ("experimental", "curve", "map") for t in scene["ticks"]), scene["name"]

  def test_the_passed_ramp_node_no_longer_binds(self):
    sc = self._scenes()["loop_ramp_2236"]
    got = [self._scan(sc, t, filtered=True) for t in sc["ticks"]]
    assert got == [(0.0, float("inf"))] * len(sc["ticks"]), got          # nothing ahead on that path: no curve trip

  def test_the_passed_loop_node_no_longer_binds_and_what_replaces_it_is_not_a_curve(self):
    sc = self._scenes()["loop_2227"]
    for t in sc["ticks"]:
      v, d = self._scan(sc, t, filtered=True)
      assert v > 25.0 and d > 5.0, (t["hms"], v, d)       # the logged 19.2 m/s node (a curve to the trip) is gone


# ---------------------------------------------------------------------------------------------------------
# the Tesla end to end (CESController.experimental_request -> the CESStatus the overlay and the log read)
# ---------------------------------------------------------------------------------------------------------
def _tesla(monkeypatch, road, filter_on=True):
  if not filter_on:
    monkeypatch.setattr(m, "icbm_passed_points", lambda *a, **k: (None, "off"))
  d, puts, _ = bg.TestTheTeslaIsUntouched._run(monkeypatch, TESLA, "tesla", True, poison=False, road=road)
  st = [v for k, v in puts if k == "CESStatus"]
  return d, puts, st


class TestTeslaEndToEnd:
  def test_a_curve_just_driven_does_not_bind(self, monkeypatch):
    mp = pytest.MonkeyPatch()
    try:
      _, _, off = _tesla(mp, _road(), filter_on=False)
    finally:
      mp.undo()
    _, _, on = _tesla(monkeypatch, _road())
    assert off and off[-1]["mapV"] == 15.0, f"without the filter the passed curve must bind (control): {off[-1]}"
    assert on and all(s["mapV"] == 0.0 and s["mapDist"] == 0.0 for s in on), [(s["mapV"], s["mapDist"]) for s in on]

  def test_a_curve_ahead_still_binds_exactly_as_before(self, monkeypatch):
    road = _road(passed_curve=False, ahead_curve_at=200.0)
    mp = pytest.MonkeyPatch()
    try:
      d_off, puts_off, off = _tesla(mp, road, filter_on=False)
    finally:
      mp.undo()
    d_on, puts_on, on = _tesla(monkeypatch, road)
    assert off[-1]["mapV"] == 15.0, "the curve ahead is not seen at all -- the scene proves nothing"
    assert (d_on, puts_on) == (d_off, puts_off), "a curve ahead with nothing binding behind changed the Tesla's output"

  def test_a_passed_curve_and_one_ahead_the_one_ahead_binds(self, monkeypatch):
    on = _tesla(monkeypatch, _road(passed_curve=True, ahead_curve_at=200.0, v_curve=15.0))[2]
    assert on[-1]["mapV"] == 15.0 and on[-1]["mapDist"] > 190.0, on[-1]

  def test_the_lightning_scan_is_the_raw_one_its_gate_owns_the_decision(self, monkeypatch):
    """The ICBM car (veh.ces_shadow) does NOT filter at this site: its CES near candidate, its telemetry and the ICBM
    gate's `clear`/`passed` verdicts stay byte for byte what they were."""
    calls = []
    real = m._curve_passed_filter
    spy = pytest.MonkeyPatch()               # _run() undoes the monkeypatch it is given; the spy must outlive both runs
    try:
      spy.setattr(m, "_curve_passed_filter", lambda *a, **k: (calls.append(1), real(*a, **k))[1])
      bg.TestTheTeslaIsUntouched._run(monkeypatch, LIGHTNING, "ford", False, poison=False)
      assert calls == [], "the Lightning's CES scan ran the Tesla's filter"
      bg.TestTheTeslaIsUntouched._run(pytest.MonkeyPatch(), TESLA, "tesla", True, poison=False)
      assert calls, "the spy cannot see the Tesla's call -- the Lightning check above is blind"
    finally:
      spy.undo()


# ---------------------------------------------------------------------------------------------------------
# the curve-DB candidate pool: the cross-source hand-off (Lightning) and the cannot-tell log
# ---------------------------------------------------------------------------------------------------------
class TestCrossSource:
  """_icbm_passed_gate re-derives only the source that bound. With src=map and the gate `clear` (the near point is
  ahead), the far candidate -- the lowest decel cap over 500 m, which includes the stretch just driven -- reached the
  curve-DB pool unfiltered, and a row at that passed node lowered it. Scene: a 13 m/s node 100 m behind (its row says
  8.6 m/s) and a 12 m/s curve 100 m ahead. The truck sits at y=0; ICBM projects its position ~40 m ahead of that."""

  ROW = 0.03                                    # v_db = sqrt(2.2 / 0.03) = 8.56 m/s: a LOWERING if the row is found

  def _run(self, monkeypatch, tmp_path, points, spy=None):
    anchors = [cdb.FAR_ANCHOR, cdb._anchor(-100.0, self.ROW, branches=[(0.0, 50.0, self.ROW)])]
    if spy is not None:
      real = cl.CurveDbLive._decide

      def wrapped(self_, **kw):
        cands, _ = kw["cands_fn"]()
        spy.append((kw["src"], cands))
        return real(self_, **kw)
      monkeypatch.setattr(cl.CurveDbLive, "_decide", wrapped)
    icbm, recs, c = cdb._drive(monkeypatch, tmp_path, points=points, anchors=anchors, ticks=300)
    return [p.get("target") for p in icbm], recs, c

  def test_a_row_at_a_passed_far_point_does_not_lower_the_target(self, monkeypatch, tmp_path):
    scene = cdb._path({-100.0: 13.0, 100.0: 12.0})
    ahead_only = cdb._path({100.0: 12.0}, y0=-20.0)                   # the same road with the passed stretch removed
    spy = []
    t_on, recs_on, c = self._run(monkeypatch, tmp_path / "on", scene, spy)
    t_ref, recs_ref, _ = self._run(monkeypatch, tmp_path / "ref", ahead_only)
    # the scene exercises the hand-off: the map point binds, the gate found it clear
    assert spy and spy[-1][0] == "map" and c._icbm_gate is None, (spy[-1][0] if spy else None, c._icbm_gate)
    assert t_ref[-1] == pytest.approx(12.0, abs=0.01), "the reference run must be the plain map curve"
    assert t_on[-1] == pytest.approx(t_ref[-1], abs=0.01), f"a row at a passed node lowered ICBM: {t_on[-1]}"
    assert recs_on[-1]["cdb2Dir"] == recs_ref[-1]["cdb2Dir"] and recs_on[-1]["cdb2Why"] == recs_ref[-1]["cdb2Why"]

  def test_the_control_without_a_mask_shows_the_failure(self, monkeypatch, tmp_path):
    """Same scene with the mask unable to tell (gate and pool unfiltered): the row at the passed node binds. Without
    this the test above could pass on a scene where the hand-off never mattered."""
    monkeypatch.setattr(m, "icbm_passed_points", lambda *a, **k: (None, "off"))
    t, recs, _ = self._run(monkeypatch, tmp_path / "off", cdb._path({-100.0: 13.0, 100.0: 12.0}))
    assert t[-1] == pytest.approx((2.2 / self.ROW) ** 0.5, abs=0.02) and recs[-1]["cdb2Dir"] == "lower", (t[-1], recs[-1]["cdb2Dir"])

  def test_the_pool_is_handed_candidates_derived_on_the_points_ahead(self, monkeypatch, tmp_path):
    spy = []
    self._run(monkeypatch, tmp_path / "on", cdb._path({-100.0: 13.0, 100.0: 12.0}), spy)
    ref = []
    self._run(monkeypatch, tmp_path / "ref", cdb._path({100.0: 12.0}, y0=-20.0), ref)
    assert spy and ref and spy[-1][1] == ref[-1][1], (spy[-1][1], ref[-1][1])

  def test_with_nothing_passed_the_pool_is_exactly_todays(self, monkeypatch, tmp_path):
    road = cdb._path({100.0: 12.0}, y0=-20.0)
    on, ref = [], []
    t_on, _, _ = self._run(monkeypatch, tmp_path / "a", road, on)
    monkeypatch.setattr(m, "icbm_passed_points", lambda *a, **k: (None, "off"))
    t_off, _, _ = self._run(monkeypatch, tmp_path / "b", road, ref)
    assert t_on == t_off and on[-1] == ref[-1]

  def test_a_slow_truck_cannot_tell_and_the_pool_log_says_so_once(self, monkeypatch, tmp_path):
    ev = []
    monkeypatch.setattr(m.cloudlog, "event", lambda name, **kw: ev.append((name, kw)))
    icbm, _, c = cdb._drive(monkeypatch, tmp_path, points=cdb._path({100.0: 12.0}), anchors=[cdb.FAR_ANCHOR],
                            v_ego=m.ICBM_PASSED_MIN_V - 1.0, ticks=50)
    assert c._roaddb.enabled
    got = [(n, k["slot"], k["why"]) for n, k in ev if n == "ces_passed_mask"]
    assert got == [("ces_passed_mask", "roaddb", "slow")], got

  def test_a_mask_crash_in_the_pool_is_logged_by_the_dbs_own_guard(self, monkeypatch, tmp_path):
    logged = []
    monkeypatch.setattr(m.cloudlog, "exception", lambda msg, *a, **k: logged.append(msg))
    monkeypatch.setattr(m, "icbm_passed_points", lambda *a, **k: (_ for _ in ()).throw(RuntimeError("boom")))
    icbm, _, _ = cdb._drive(monkeypatch, tmp_path, points=cdb._path({100.0: 12.0}), anchors=[cdb.FAR_ANCHOR], ticks=50)
    assert any("curvedb_v2: decision FAILED" in s for s in logged), logged

  def test_the_reverse_the_near_candidate_is_masked_when_far_or_vision_bound(self):
    """The mirror hand-off, at the pool itself: src=far (the gate, which only re-derives the source that bound, ran or
    not), and the NEAR candidate sig carries a passed 6 m/s node. The pool's map candidate must be what the road ahead
    gives -- the same as the run on the road with the passed stretch removed."""
    import inspect

    from openpilot.selfdrive.controls.lib.pnw_vehicle import PnwVehicle

    cls = next(o for o in vars(m).values() if inspect.isclass(o) and hasattr(o, "_icbm_roaddb"))
    veh = PnwVehicle(types.SimpleNamespace(carFingerprint=LIGHTNING, brand="ford", openpilotLongitudinalControl=False, dashcamOnly=False))

    def pool(points, src):
      la, lo = _at(0.0)
      got = []
      c = types.SimpleNamespace(_veh=veh, _map_targets=points, _cur_bearing=0.0, _icbm_src=src, _icbm_ep=m.IcbmEpisode(),
                                _icbm_cap_src=None, _way_sel="current", _hwy_class="motorway", _icbm_eff_fn=None,
                                _roaddb=types.SimpleNamespace(decide=lambda **kw: (got.append(kw["cands_fn"]()), (kw["today"], kw["src"], None))[1]))
      v = 33.0
      mv, md = m.upcoming_curve(points, la, lo, v, m.C.CURVE_MAP_LOOKAHEAD_S)
      fv, fd, fr = m.icbm_far_map_candidate(points, la, lo, v, 33.5, m.icbm_map_eff_scale, veh.icbm_map_scale, veh.icbm_firm_decel)
      sig = {"v_ego": v, "v_set": 33.5, "map_target_v": mv, "map_target_dist": md, "pitch": None}
      cls._icbm_roaddb(c, None, sig, la, lo, 33.5, None, True, False, 0.0, float("inf"), 10.0, fv, fd, fr)
      return got[-1][0]

    scene = _pts([(0.0, float(y), 6.0 if y == -100 else 12.0 if y == 100 else 0.0) for y in range(-180, 901, 40)])
    ahead_only = _pts([(0.0, float(y), 12.0 if y == 100 else 0.0) for y in range(-20, 901, 40)])
    for src in ("far", "vis"):
      assert pool(scene, src) == pool(ahead_only, src), src
    raw = _pts([(0.0, float(y), 6.0 if y == -100 else 0.0) for y in range(-180, 901, 40)])
    assert pool(raw, "far")["map"] is None, "the passed node still reached the pool's map candidate"


# ---------------------------------------------------------------------------------------------------------
# Rule 2 for the ICBM gate's own fail-open reasons (already logged by behindgate2pnw; pinned here for every one)
# ---------------------------------------------------------------------------------------------------------
class TestGateFailOpenIsLoud:
  @staticmethod
  def _scene(why):
    kw = {"bearing": 0.0}
    if why == "slow":
      return _road(v_curve=3.0), dict(kw, v=m.ICBM_PASSED_MIN_V - 0.5, v_set=10.0)
    if why == "noHeading":
      return _road(v_curve=3.0), dict(kw, bearing=None)
    if why == "reversed":
      return _road(v_curve=3.0), dict(kw, bearing=180.0)
    if why == "offPath":
      return _pts([(60.0, float(y), 3.0 if 60 <= y <= 100 else 0.0) for y in range(-400, 901, 20)]), kw
    if why == "tooMany":
      return _road(v_curve=3.0, fwd=22000.0), kw
    return _pts([(0.0, 100.0, 3.0), (0.0, 100.0, 3.0)]), kw         # noPath: no segment to project onto

  @pytest.mark.parametrize("why", ["slow", "noHeading", "reversed", "offPath", "tooMany", "noPath"])
  def test_each_reason_is_logged_once_and_the_decision_is_ungated(self, monkeypatch, why):
    clock, ev = [900.0], []
    c, step, pubs = bg._controller(monkeypatch, clock, ev)
    pts, kw = self._scene(why)
    tr = bg._drive(c, step, pubs, clock, pts, 0.0, 2.0, **kw)
    unknown = [k for _, name, k in ev if name == "ces_icbm_passed" and "phase" not in k and k["state"] == "unknown"]
    assert [u["why"] for u in unknown] == [why], (why, [(n, k) for _, n, k in ev])
    assert any(row[2].get("target") is not None for row in tr), f"{why}: no slowdown started, so the fail-open path was not exercised"
    assert all(row[4] != "mapPassed" for row in tr), f"{why}: the gate acted although it could not tell"
