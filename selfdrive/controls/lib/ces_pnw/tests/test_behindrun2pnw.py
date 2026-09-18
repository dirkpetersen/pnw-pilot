"""behindrun2pnw: a curve the truck has already driven past may not lower a RUNNING ICBM slowdown, nor hold back the
restore that gives the speed back after the curve.

OWNER, 2026-09-14 ~21:40 PT, asked "should a curve the truck has already passed also stop lowering a slowdown that's
already running, and delaying the speed coming back after the curve?" -- answer: "Behind-curve gate: yes".

behindgate2pnw (365d287034) stopped a passed map point from STARTING an episode and left the running case as its known
limit: "a passed point still lowers a RUNNING slowdown and delays the speed coming back after a curve". That is what
this closes, with the SAME test (icbm_passed_points), the same cannot-tell -> no gate rule, and the same fail-open
crash behaviour -- only the label differs ("mapPassedRun", phase="run"), so the two are countable apart in ces_events.

BASE in every comparison below is the SHIPPED code: _icbm_passed_gate(running=True) short-circuited back to the
ungated decision. So each test measures exactly what this commit changes and nothing else.

What is pinned here:
  * (a) a passed point no longer LOWERS a running episode's target -- and the result is exactly the run where that
    point never existed;
  * (b) a passed point no longer HOLDS the episode bound, so the restore begins earlier and the set comes back;
  * a curve still AHEAD keeps full authority: identical tick for tick, including the whole approach of a real curve;
  * START behaviour is untouched (ceiling None, vision's start rule, the "mapPassed" label);
  * vision candidates are untouched;
  * cannot-tell -> no gate, and it says so (phase="run"); a crash keeps the running episode and names the running case;
  * the real weekend + 09-08 telemetry (tests/data/behindrun_2026-09.json, 45 s past each logged start): the four
    windows where a passed point held a running episode release earlier, the three control windows are identical, and
    a closed loop through the real Ford executor gives the set back earlier.
"""
import json
import os

import pytest

from opendbc.car.ford.icbm_pnw import IcbmCommand, PressGovernor, STEP_MS, arbitrate, decide_press
from openpilot.selfdrive.controls.lib import pnw_vehicle as pv
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_behindgate2pnw import (
  VSET, _controller, _drive, _road, _starts,
)
from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_ces_record_fields import _record

FIXTURE = os.path.join(os.path.dirname(__file__), "data", "behindrun_2026-09.json")
MPH = 0.44704


@pytest.fixture(autouse=True)
def _default_curve_cfg(tmp_path, monkeypatch):
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(tmp_path / "absent.json"))
  monkeypatch.setattr(pv, "RAIN_CONFIG_PATH", str(tmp_path / "absent-rain.json"))


def _run_gate_off(mp):
  """the SHIPPED behindgate2pnw code: the passed-point gate on STARTS only, a running episode ungated."""
  real = m._icbm_passed_gate

  def start_only(ctl, now, target, sig, plat, plon, ref, far_v, far_dist, far_raw, vis,
                 ceiling=None, running=False):
    # icbmslow2pnw threaded far_raw through the gate; this stand-in mirrors the real signature.
    if running:
      return target, sig, far_v, far_dist, far_raw
    return real(ctl, now, target, sig, plat, plon, ref, far_v, far_dist, far_raw, vis,
                ceiling=ceiling, running=running)
  mp.setattr(m, "_icbm_passed_gate", start_only)


def _scene(gate_running, pts, seconds, t0=100.0, **kw):
  """one drive north from y=0 through `pts`. gate_running False = the shipped code. -> (trace, events, controller)"""
  mp = pytest.MonkeyPatch()
  try:
    if not gate_running:
      _run_gate_off(mp)
    clock, ev = [t0], []
    c, step, pubs = _controller(mp, clock, ev)
    return _drive(c, step, pubs, clock, pts, 0.0, seconds, **kw), ev, c
  finally:
    mp.undo()


# ---------------------------------------------------------------------------------------------------------
# (a) a passed point may not LOWER a running episode
# ---------------------------------------------------------------------------------------------------------
def _two_curve_road():
  """the curve AHEAD at 220 m rated 20 m/s, the curve just DRIVEN (100-60 m back) rated 12 -- the passed one is the
  more binding of the two, so with the shipped code it drags the running episode's target down."""
  pts = _road(passed_curve=False, ahead_curve_at=220.0, v_curve=20.0)
  for i, p in enumerate(_road(v_curve=12.0)):
    if p["velocity"]:
      pts[i] = dict(pts[i], velocity=12.0)
  return pts


class TestARunningEpisodeIsNotLowered:
  def test_the_passed_curve_no_longer_drags_the_running_target_down(self):
    base, _, _ = _scene(False, _two_curve_road(), 2.0)
    gated, ev, _ = _scene(True, _two_curve_road(), 2.0)
    assert base[0][2].get("target") is not None, "the curve ahead never started an episode -- proves nothing"
    assert base[-1][2]["target"] < base[0][2]["target"] - 1.0, \
      "the shipped code does not lower the running target here -- the test proves nothing"
    assert gated[-1][2]["target"] == pytest.approx(gated[0][2]["target"], abs=0.35), \
      f"the passed curve still lowered the running target: {gated[0][2]['target']} -> {gated[-1][2]['target']}"
    assert [r[4] for r in gated[1:]] == ["mapPassedRun"] * (len(gated) - 1)
    ran = [kw for _, name, kw in ev if name == "ces_icbm_passed" and kw.get("phase") == "run"]
    assert [(k["state"], k["started"]) for k in ran] == [("passed", "map")], ran

  def test_it_is_exactly_the_drive_where_the_passed_curve_never_existed(self):
    """Not merely "higher": a curve still ahead keeps FULL authority, so the running target must equal the one the
    episode would have had on the same road without the passed curve -- tick for tick."""
    gated, _, _ = _scene(True, _two_curve_road(), 3.0)
    clean, _, _ = _scene(True, _road(passed_curve=False, ahead_curve_at=220.0, v_curve=20.0), 3.0)
    assert [r[2] for r in clean if r[2].get("target") is not None], "the clean road never binds -- proves nothing"
    assert [r[2] for r in gated] == [r[2] for r in clean]

  def test_the_start_of_that_very_episode_is_untouched(self):
    """START is behindgate2pnw's, and this commit must not move it: the first tick still reports the START label."""
    gated, _, _ = _scene(True, _two_curve_road(), 2.0)
    assert gated[0][4] == "mapPassed", gated[0]


# ---------------------------------------------------------------------------------------------------------
# (b) a passed point may not HOLD the episode / delay the restore
# ---------------------------------------------------------------------------------------------------------
class TestTheRestoreIsNotDelayed:
  PTS = _road(passed_curve=False, ahead_curve_at=400.0)

  def _both(self, seconds=30.0):
    return (_scene(False, self.PTS, seconds, t0=400.0, follow=True),
            _scene(True, self.PTS, seconds, t0=400.0, follow=True))

  def test_the_approach_and_the_curve_itself_are_identical(self):
    """Nothing changes until the truck is actually past the curve: the whole approach, the start and the in-curve
    stretch must be tick-for-tick identical, or this gate is touching a curve that is still ahead."""
    (base, _, _), (gated, _, _) = self._both()
    past = 400.0 + 40.0 + m.ICBM_PASSED_TOL_M
    k = next(i for i, r in enumerate(base) if r[1] > past)
    assert any(r[2].get("target") is not None for r in base[:k]), "no episode before the curve -- proves nothing"
    assert [r[2] for r in gated[:k]] == [r[2] for r in base[:k]]

  def test_the_cap_ends_and_the_restore_begins_earlier(self):
    (base, _, _), (gated, ev, _) = self._both()
    last_cap = lambda tr: max(i for i, r in enumerate(tr) if r[2].get("target") is not None   # noqa: E731
                              and r[2].get("dir", "dec") == "dec")
    assert last_cap(gated) < last_cap(base), \
      f"the cap did not end earlier (base tick {last_cap(base)}, gated {last_cap(gated)})"
    first_inc = lambda tr: next((i for i, r in enumerate(tr) if r[2].get("dir") == "inc"), None)   # noqa: E731
    assert first_inc(gated) is not None, "the gated run never restored -- proves nothing"
    assert first_inc(base) is None or first_inc(gated) < first_inc(base), \
      f"the restore did not begin earlier (base {first_inc(base)}, gated {first_inc(gated)})"
    assert any(kw.get("phase") == "run" and kw["state"] == "passed" for _, n, kw in ev if n == "ces_icbm_passed")

  def test_the_set_speed_is_back_sooner(self):
    """Through the follow loop (the stock set walks toward the published target): the driver gets the speed back."""
    (_, _, c_base), (_, _, c_gate) = self._both()
    assert c_gate._stock_set > c_base._stock_set + STEP_MS, \
      f"the set did not come back sooner: base {c_base._stock_set / MPH:.1f} mph, gated {c_gate._stock_set / MPH:.1f}"


# ---------------------------------------------------------------------------------------------------------
# what the gate must NOT touch
# ---------------------------------------------------------------------------------------------------------
class TestWhatIsUntouched:
  def test_a_vision_bound_running_episode_never_calls_the_gate(self):
    """Vision candidates are out of scope (owner: start behaviour, vision and curve timing unchanged). A running
    episode bound by vision -- here a curve 125 m out, beyond the 60 m the map path reaches, so vision may start --
    must never reach the gate."""
    calls = []
    mp = pytest.MonkeyPatch()
    try:
      real = m._icbm_passed_gate
      mp.setattr(m, "_icbm_passed_gate", lambda *a, **k: (calls.append(k.get("running")), real(*a, **k))[1])
      clock, ev = [1200.0], []
      c, step, pubs = _controller(mp, clock, ev)
      tr = _drive(c, step, pubs, clock, _road(passed_curve=False, back=100.0, fwd=60.0), 0.0, 3.0,
                  vis=lambda y: (4.0, 5.0))
    finally:
      mp.undo()
    assert [r[3] for r in tr if r[2].get("target") is not None] == ["vis"] * sum(
      1 for r in tr if r[2].get("target") is not None), [r[3] for r in tr]
    assert _starts(tr), "vision never bound -- proves nothing"
    assert calls == [], f"the gate ran on a vision-bound episode: {calls}"
    assert all(r[4] is None for r in tr)

  def test_a_start_is_still_handed_ceiling_none_and_visions_start_rule(self):
    """byte-identical START path: on a starting tick the gate is called with running=False and ceiling=None (there is
    no episode ceiling yet), which is what behindgate2pnw passed."""
    seen = []
    mp = pytest.MonkeyPatch()
    try:
      real = m._icbm_passed_gate
      mp.setattr(m, "_icbm_passed_gate",
                 lambda *a, **k: (seen.append((k.get("running"), k.get("ceiling"))), real(*a, **k))[1])
      clock, ev = [1300.0], []
      c, step, pubs = _controller(mp, clock, ev)
      _drive(c, step, pubs, clock, _road(), 0.0, 1.0)
    finally:
      mp.undo()
    assert seen and all(s == (False, None) for s in seen), seen

  def test_a_running_tick_is_handed_the_episodes_own_latched_ceiling(self):
    """...and a RUNNING tick gets the episode's latched ceiling, so the re-decision is judged against the same
    reference the ungated decision used (v_set follows the tapped-down stock set while capped)."""
    seen = []
    mp = pytest.MonkeyPatch()
    try:
      real = m._icbm_passed_gate
      mp.setattr(m, "_icbm_passed_gate",
                 lambda *a, **k: (seen.append((k.get("running"), k.get("ceiling"))), real(*a, **k))[1])
      clock, ev = [1400.0], []
      c, step, pubs = _controller(mp, clock, ev)
      _drive(c, step, pubs, clock, _two_curve_road(), 0.0, 2.0, follow=True)
    finally:
      mp.undo()
    running = [s for s in seen if s[0]]
    assert running, "no running tick reached the gate -- proves nothing"
    assert all(s[1] == pytest.approx(VSET) for s in running), running


# ---------------------------------------------------------------------------------------------------------
# cannot tell / crash -- the running episode keeps the ungated decision, and says so
# ---------------------------------------------------------------------------------------------------------
class TestFailOpen:
  def test_below_the_speed_floor_the_running_episode_is_unchanged_and_says_so(self):
    pts = _road(v_curve=3.0)
    base, _, _ = _scene(False, pts, 1.5, t0=1500.0, v=m.ICBM_PASSED_MIN_V - 0.5, v_set=10.0)
    gated, ev, _ = _scene(True, pts, 1.5, t0=1500.0, v=m.ICBM_PASSED_MIN_V - 0.5, v_set=10.0)
    assert _starts(gated), "nothing runs below the floor -- the fail-open path is not exercised"
    assert [r[2] for r in gated] == [r[2] for r in base]
    ran = [kw for _, n, kw in ev if n == "ces_icbm_passed" and kw.get("phase") == "run"]
    assert [(k["state"], k["why"]) for k in ran] == [("unknown", "slow")], ran

  def test_a_crash_on_a_running_tick_keeps_the_episode_logs_once_and_names_the_running_case(self):
    """The gate runs in selfdrived (restart_if_crash=False). A defect must cost the gate, never the episode and never
    the IcbmTarget publish -- and it must say which case it dropped."""
    logged = []
    mp = pytest.MonkeyPatch()
    try:
      mp.setattr(m.cloudlog, "exception", lambda msg, *a, **k: logged.append(msg))
      clock, ev = [1600.0], []
      c, step, pubs = _controller(mp, clock, ev)
      pts = _road(passed_curve=False, ahead_curve_at=100.0)
      ok = _drive(c, step, pubs, clock, pts, 0.0, 1.0)                    # a clean start: the gate works
      assert ok[0][2].get("target") is not None and not logged, ok[0]
      mp.setattr(m, "icbm_passed_points", lambda *a, **k: (_ for _ in ()).throw(RuntimeError("boom")))
      broke = _drive(c, step, pubs, clock, pts, 25.0, 3.0)                # ...then it throws while the episode runs
    finally:
      mp.undo()
    assert broke[0][2].get("target") is not None, "a gate crash stopped ICBM publishing"
    assert sum("behindgate2pnw" in s for s in logged) == 1, logged
    assert "RUNNING" in logged[0], logged
    assert not any("_icbm_step FAILED" in s for s in logged), "the crash escaped into _icbm_step's except"


# ---------------------------------------------------------------------------------------------------------
# telemetry
# ---------------------------------------------------------------------------------------------------------
class TestTelemetry:
  def test_icbmGate_mapPassedRun_reaches_the_record(self):
    assert _record(_icbm_gate="mapPassedRun")["icbmGate"] == "mapPassedRun"

  def test_the_start_and_running_verdicts_are_logged_separately_and_change_only(self):
    """A START verdict and a RUNNING verdict about the same road are different facts: both are logged, each once."""
    _, ev, _ = _scene(True, _road(), 3.0, t0=1700.0)
    got = [(kw["state"], kw.get("phase"), kw.get("started")) for _, n, kw in ev if n == "ces_icbm_passed"]
    assert got == [("passed", None, None)], got     # the start is suppressed: no episode ever runs
    _, ev2, _ = _scene(True, _two_curve_road(), 2.0, t0=1800.0)
    got2 = [(kw["state"], kw.get("phase"), kw.get("started")) for _, n, kw in ev2 if n == "ces_icbm_passed"]
    assert got2 == [("passed", None, "map"), ("passed", "run", "map")], got2


# ---------------------------------------------------------------------------------------------------------
# real telemetry windows -- 45 s past each logged start
# ---------------------------------------------------------------------------------------------------------
def _windows():
  with open(FIXTURE) as f:
    return json.load(f)["windows"]


def _replay(w, gate_running, executor=False):
  """the real _icbm_step at 4 Hz over a logged window (same harness as test_behindgate2pnw._replay, run long past the
  start). -> [(t, published, src, icbmGate, phase)], ces_icbm_passed events, stock set per record"""
  mp = pytest.MonkeyPatch()
  try:
    if not gate_running:
      _run_gate_off(mp)
    clock, ev = [w["records"][0]["t"]], []
    c, step, pubs = _controller(mp, clock, ev)
    stock, gov, frame, held = None, PressGovernor(), 0, None
    trace, sets, recs = [], [], w["records"]
    for n, r in enumerate(recs):
      t_next = recs[n + 1]["t"] if n + 1 < len(recs) and recs[n + 1]["t"] - r["t"] <= 2.0 else r["t"] + 1.0
      pts = [{"latitude": a, "longitude": b, "velocity": v} for a, b, v in w["paths"][r["path"]]]
      if stock is None:
        stock = r["stockSet"]
      while clock[0] < t_next - 1e-6:
        c._map_targets = pts
        c._cur_lat, c._cur_lon, c._cur_bearing = r["lat"], r["lon"], r["bearing"] or 0.0
        c._gps_fix_ts = r["t"] - m.ICBM_GPS_LAG_KEEP_S
        c._stock_set = stock if executor else r["stockSet"]
        c._stock_on = bool(r["stockOn"])
        v = r["vEgo"]
        mtv, mtd = m.upcoming_curve(pts, r["lat"], r["lon"], v, m.C.CURVE_MAP_LOOKAHEAD_S)
        sig = {"v_ego": v, "v_set": c._stock_set if executor else r["vSet"], "map_target_v": mtv, "map_target_dist": mtd,
               "curve_lat_accel_vision": r["visLat"] or 0.0, "time_to_curve": r["visTtc"] or 10.0,
               "lat_accel_now": 2.0 if r["icbmGate"] == "inCurve" else 0.0, "has_lead": bool(r["lead"]),
               "lead_drel": r["dRel"] or 0.0, "lead_vlead": r["vLead"] or 0.0, "gas": bool(r["gas"]), "brake": False,
               "spd_lim": r["spdLim"] or 0.0, "pitch": None, "vis_k_max": None, "vis_reach": 0.0}
        k = len(pubs)
        c._icbm_last_pub = -1e9
        step(sig, active=True)
        p = pubs[-1][1] if len(pubs) > k else {}
        trace.append((round(clock[0] - w["t_start"], 3), p, c._icbm_src, c._icbm_gate, c._icbm_ep.phase))
        if executor:
          cmd = IcbmCommand(target_ms=p["target"], ceiling_ms=p["ceiling"], ts=p["ts"], dir=p.get("dir", "dec")) \
            if p.get("target") is not None else None
          for _ in range(25):                           # 0.25 s of executor frames; a tap completes on release
            btn = gov.update(frame, decide_press(stock, arbitrate([cmd], clock[0]), clock[0], True, False))
            if held is not None and btn is None:
              stock += -STEP_MS if held == "dec" else STEP_MS
            held = btn
            frame += 1
            clock[0] = round(clock[0] + 0.01, 6)
          sets.append(stock)
        else:
          clock[0] = round(clock[0] + 0.25, 6)
    return trace, [kw for _, name, kw in ev if name == "ces_icbm_passed"], sets
  finally:
    mp.undo()


def _capping(trace):
  return [t for t, p, *_ in trace if p.get("target") is not None and p.get("dir", "dec") == "dec"]


class TestRealWindows:
  @pytest.mark.parametrize("w", _windows(), ids=lambda w: w["name"])
  def test_the_window(self, w):
    base, _, _ = _replay(w, gate_running=False)
    gated, ev, _ = _replay(w, gate_running=True)
    assert _capping(base), f"{w['when_pt']}: the shipped code does not cap at all here -- proves nothing"
    if w["kind"] == "unchanged":
      assert [(t, p) for t, p, *_ in gated] == [(t, p) for t, p, *_ in base], \
        f"{w['when_pt']} ({w['why']}): a curve still ahead was changed"
      assert not [e for e in ev if e.get("phase") == "run" and e["state"] == "passed"]
    else:
      first_diff = next(t for (t, pb, *_), (_, pg, *_) in zip(base, gated, strict=True) if pb != pg)
      assert first_diff > 0.0, f"{w['when_pt']}: the gate changed the START (t={first_diff}) -- START is untouched"
      assert max(_capping(gated), default=-1.0) < max(_capping(base)), \
        f"{w['when_pt']} ({w['why']}): the cap did not end earlier"
      assert any(e.get("phase") == "run" and e["state"] == "passed" for e in ev), ev

  def test_the_fixture_covers_both_directions(self):
    kinds = [w["kind"] for w in _windows()]
    assert kinds.count("run") >= 4 and kinds.count("unchanged") >= 3

  def test_the_gate_only_ever_acts_behind_the_truck(self):
    """Across every window: on every tick the running gate acted, the point it removed was behind ICBM's own position
    -- icbm_passed_points said so. Nothing here may fire on a curve still ahead."""
    acted = sum(1 for w in _windows() for _t, _p, _s, g, _ph in _replay(w, gate_running=True)[0] if g == "mapPassedRun")
    assert acted > 0, "the running gate never acted on the real corpus -- the fixture proves nothing"


class TestClosedLoopThroughTheFordExecutor:
  def test_the_set_comes_back_earlier_after_the_curve(self):
    w = next(x for x in _windows() if x["name"] == "sun_1546_run")
    _, _, base = _replay(w, gate_running=False, executor=True)
    _, _, gated = _replay(w, gate_running=True, executor=True)
    start_set = w["records"][0]["stockSet"]
    assert min(base) < start_set - STEP_MS, "the shipped code did not tap the set down -- proves nothing"
    assert min(gated) == pytest.approx(min(base)), "the slowdown itself changed -- only the restore may"
    assert gated[-1] > base[-1] + STEP_MS, \
      f"the set did not come back sooner: base {base[-1] / MPH:.1f} mph, gated {gated[-1] / MPH:.1f} mph"
