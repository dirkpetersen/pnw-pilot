"""behindgate2pnw: ICBM may not START a curve slowdown for a map point the truck has already driven past.

THE OWNER'S MOST FREQUENT COMPLAINT: "going like 50+ on a country road, the car suddenly loses 20 mph, sometimes 10 below
the speed limit." mapd publishes its current way from the way's first node, so the curve just driven stays in the path
with an unsigned distance. On the weekend the curve-exit shape repeats: in-curve holds every start through the curve,
and the moment it releases, the curve just driven starts a new episode (Sun 12:05:28, set 60 -> 51 mph).

What is pinned here:
  * icbm_passed_points, one test per guard, on synthetic geometry AND the Sat 12:47:21 geometry from OSM;
  * the gate through the real CESController._icbm_step: suppressed, reselected onto a curve still ahead, START only,
    fail-open paths (slow / crash) that say so, vision's own start rule kept, the change-only log;
  * real telemetry windows (tests/data/behindgate_2026-09.json): six passed-point starts no longer start, six real
    starts (incl. the 09-08 20:28 far point, which was AHEAD) start identically;
  * a closed loop through the real Ford executor: the curve-exit set taps disappear, a real curve's are unchanged;
  * the Tesla never runs any of it.
"""
import inspect
import json
import math
import os
import types
from types import SimpleNamespace

import pytest

from opendbc.car.ford.icbm_pnw import IcbmCommand, PressGovernor, STEP_MS, arbitrate, decide_press
from openpilot.selfdrive.controls.lib import pnw_vehicle as pv
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import icbm_passed_points, icbm_path_behind
from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_ces_record_fields import _record
from openpilot.selfdrive.controls.lib.pnw_vehicle import PnwVehicle

LAT0, LON0 = 44.0, -121.3
LIGHTNING = "FORD_F_150_LIGHTNING_MK1"
FIXTURE = os.path.join(os.path.dirname(__file__), "data", "behindgate_2026-09.json")
MPH = 0.44704


@pytest.fixture(autouse=True)
def _default_curve_cfg(tmp_path, monkeypatch):
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(tmp_path / "absent.json"))
  monkeypatch.setattr(pv, "RAIN_CONFIG_PATH", str(tmp_path / "absent-rain.json"))


def _ll(x, y):
  return LAT0 + y / 111320.0, LON0 + x / (111320.0 * math.cos(math.radians(LAT0)))


def _pts(xyv):
  out = []
  for p in xyv:
    la, lo = _ll(p[0], p[1])
    out.append({"latitude": la, "longitude": lo, "velocity": p[2] if len(p) > 2 else 0.0})
  return out


def _passed(pts, bearing=0.0, v=20.0, at=(0.0, 0.0)):
  la, lo = _ll(*at)
  return icbm_passed_points(pts, la, lo, bearing, v)


# ---------------------------------------------------------------------------------------------------------
# icbm_passed_points
# ---------------------------------------------------------------------------------------------------------
class TestPassedPoints:
  def test_a_straight_road_points_behind_are_passed_and_ahead_are_not(self):
    ys = list(range(-200, 201, 20))
    mask, why = _passed(_pts([(0.0, y) for y in ys]))
    assert why == "ok"
    assert [y for y, gone in zip(ys, mask, strict=True) if gone] == [y for y in ys if y < 0]

  def test_the_along_path_tolerance(self):
    assert m.ICBM_PASSED_TOL_M == 5.0 and m.ICBM_PASSED_MIN_V == 5.0
    pts = _pts([(0.0, -100.0), (0.0, -6.0), (0.0, -4.0), (0.0, 100.0)])
    assert _passed(pts)[0] == [True, True, False, False]

  def test_the_heading_is_required_too(self):
    """Round a loop: the approach is behind ALONG the path, but after > 180 deg of turn it lies ahead of the heading.
    Only one reading says passed -> not passed (fail open)."""
    r = 40.0
    loop = [(r - r * math.cos(math.radians(a)), r * math.sin(math.radians(a))) for a in range(0, 360, 15)]
    pts = _pts([(0.0, -60.0), (0.0, -30.0)] + loop)
    truck = (r - r * math.cos(math.radians(270)), r * math.sin(math.radians(270)))   # 3/4 round, heading west
    mask, why = _passed(pts, bearing=270.0, at=truck)
    assert why == "ok" and mask[1] is False, "the approach point ahead of the heading was called passed"
    assert any(mask[2:]), "nothing on the loop behind the truck was passed -- the geometry proves nothing"

  def test_the_far_side_of_a_loop_ramp_is_ahead_though_its_bearing_points_back(self):
    r = 60.0
    loop = [(r * (1 - math.cos(a)), r * math.sin(a)) for a in [math.radians(i * 30) for i in range(10)]]
    pts = _pts([(0.0, -80.0), (0.0, -40.0)] + [(x, y) for x, y in loop])
    la, lo = LAT0, LON0
    back = [i for i, p in enumerate(pts[2:], 2) if (p["latitude"] - la) < 0 and abs(math.degrees(math.atan2(
      (p["longitude"] - lo) * math.cos(math.radians(la)), p["latitude"] - la))) > 90.0]
    assert back, "no loop point reads behind by bearing -- the geometry proves nothing"
    mask, _ = _passed(pts)
    assert mask == [True, True] + [False] * (len(pts) - 2)

  def test_a_switchback_leg_behind_by_bearing_is_not_passed(self):
    """Real roads: 1.8 % of points still ahead read as behind by bearing, on switchbacks. Leg 3 runs north like the
    truck, west and south of it -- behind by bearing, 300+ m ahead along the road."""
    leg1 = [(0.0, y) for y in range(-100, 101, 20)]
    leg2 = [(-40.0, y) for y in range(100, -101, -20)]
    leg3 = [(-80.0, y) for y in range(-100, 101, 20)]
    pts = _pts(leg1 + leg2 + leg3)
    mask, why = _passed(pts, at=(0.0, 50.0))
    assert why == "ok"
    la, lo = _ll(0.0, 50.0)
    leg3_behind = [i for i in range(len(leg1) + len(leg2), len(pts)) if pts[i]["latitude"] < la]
    assert leg3_behind and not any(mask[i] for i in leg3_behind)
    assert all(mask[i] for i, (_, y) in enumerate(leg1) if y < 40)

  def test_the_sat_1247_way_switch_geometry_from_osm(self):
    """Sat 2026-09-12 12:47:21 PT, the OSM nodes mapd's new path was built from, relative to the logged fix (m east,
    m north), heading 2 deg: the curve point (mapV 11.0) sits 7.8 m back with the truck 10.7 m off the centreline.
    The telemetry projection (15 m tolerance, every near segment treated as ambiguous) calls it not behind."""
    xy = [(-178.9, -39.5), (-169.2, -38.1), (-139.2, -36.6), (-91.4, -37.8), (-71.2, -38.6), (-10.0, -37.1),
          (-10.7, -7.8), (-8.5, 17.8), (-4.9, 42.5), (3.1, 84.4), (8.2, 101.6), (23.3, 145.9), (28.4, 158.4)]
    pts = _pts([(x, y, 11.0 if i == 6 else 0.0) for i, (x, y) in enumerate(xy)])
    mask, why = icbm_passed_points(pts, LAT0, LON0, 2.0, 19.5)
    assert why == "ok" and mask[6] is True and not any(mask[7:])
    curve = pts[6]
    assert icbm_path_behind(pts, LAT0, LON0, m._haversine_m(LAT0, LON0, curve["latitude"], curve["longitude"])) is False

  def test_a_different_part_of_the_path_passing_close_resolves_toward_ahead(self):
    """A hairpin whose legs pass 6 m apart: the truck projects onto both. The later leg must not make the approach
    passed."""
    pts = _pts([(0, -115), (0, -75), (0, -35), (0, 5), (0, 45), (3, 65), (6, 42), (6, 2), (6, -38), (6, -78)])
    mask, _ = _passed(pts)
    assert mask[:3] == [True, True, True] and mask[3:] == [False] * 7

  def test_a_loop_ramp_passing_under_the_road_does_not_pull_the_truck_forward(self):
    """A cloverleaf: the loop comes back under the approach road heading west. The fix sits 0.8 m off the approach and
    0.1 m off the leg below, so the NEAREST segment is 700 m further along the path. Taking it would call the whole
    loop -- still ahead, and behind the truck by bearing on its southern half -- passed."""
    xy = [(0, -200), (0, -100), (0, 0), (0, 50), (30, 100), (90, 100), (130, 50), (130, -20), (90, -60), (40, -40),
          (30, -1), (-30, -1), (-100, -1)]
    mask, why = _passed(_pts(xy), at=(0.8, -0.9))
    assert why == "ok"
    loop_behind_by_bearing = [i for i in range(4, 11) if xy[i][1] < -0.9]
    assert loop_behind_by_bearing, "no loop point behind by bearing -- proves nothing"
    assert mask[:2] == [True, True] and not any(mask[2:]), mask

  def test_one_leg_the_truck_drives_beside_is_the_same_place_not_an_ambiguity(self):
    """Two consecutive segments of the corner the truck just drove, both within the ambiguity band: their projections
    are closer along the path than the straight line through the truck allows for two different places."""
    pts = _pts([(-10.0, -60.0), (-10.0, -8.0), (-10.0, 40.0), (-10.0, 90.0)])
    mask, _ = _passed(pts)
    assert mask == [True, True, False, False]

  @pytest.mark.parametrize("kw,why", [({"v": m.ICBM_PASSED_MIN_V - 0.1}, "slow"), ({"bearing": None}, "noHeading"),
                                      ({"bearing": float("nan")}, "noHeading"), ({"bearing": 180.0}, "reversed"),
                                      ({"at": (45.0, 0.0)}, "offPath")])
  def test_cannot_tell(self, kw, why):
    mask, got = _passed(_pts([(0.0, y) for y in range(-200, 201, 20)]), **kw)
    assert mask is None and got == why

  def test_the_speed_floor_is_inclusive(self):
    mask, why = _passed(_pts([(0.0, -100.0), (0.0, 100.0)]), v=m.ICBM_PASSED_MIN_V)
    assert why == "ok" and mask == [True, False]

  def test_no_path_too_many_and_garbage(self):
    assert icbm_passed_points(None, LAT0, LON0, 0.0, 20.0) == (None, "noPath")
    assert icbm_passed_points(_pts([(0, 0)]), LAT0, LON0, 0.0, 20.0) == (None, "noPath")
    assert icbm_passed_points(_pts([(0.0, -10.0), (0.0, 10.0)]), None, LON0, 0.0, 20.0) == (None, "noPath")
    many = _pts([(0.0, float(y)) for y in range(m.ICBM_PASSED_MAX_POINTS + 1)])
    assert icbm_passed_points(many, LAT0, LON0, 0.0, 20.0) == (None, "tooMany")
    pts = _pts([(0.0, -100.0)]) + [{"latitude": "x"}, {"longitude": 1.0}, {"latitude": float("nan"), "longitude": LON0}] \
      + _pts([(0.0, -50.0), (0.0, 50.0)])
    mask, why = icbm_passed_points(pts, LAT0, LON0, 0.0, 20.0)
    assert why == "ok" and mask == [True, False, False, False, True, False], "a bad point shifted the mask"


# ---------------------------------------------------------------------------------------------------------
# Through CESController._icbm_step
# ---------------------------------------------------------------------------------------------------------
V, VSET = 25.0, 30.0


def _controller(monkeypatch, clock, events, stock=VSET):
  monkeypatch.setattr(m.time, "monotonic", lambda: clock[0])
  monkeypatch.setattr(m.time, "time", lambda: clock[0])
  monkeypatch.setattr(m.cloudlog, "event", lambda name, **kw: events.append((clock[0], name, kw)))
  cls = next(o for o in vars(m).values() if inspect.isclass(o) and hasattr(o, "_icbm_step"))
  pubs = []
  c = SimpleNamespace(mem_params=SimpleNamespace(put_nonblocking=lambda k, v: pubs.append((clock[0], v)) if k == "IcbmTarget" else None))
  c._veh = PnwVehicle(SimpleNamespace(carFingerprint=LIGHTNING, brand="ford", openpilotLongitudinalControl=False, dashcamOnly=False))
  c._icbm_ep = m.IcbmEpisode()
  for k, v in dict(_icbm_ceiling=None, _icbm_dir=None, _map_targets=[], _cur_lat=None, _cur_lon=None, _cur_bearing=0.0,
                   _gps_fix_ts=None, _gps_src="car", _icbm_floor_lim=0.0, _icbm_floor_pend=None, _icbm_floor_hit=False,
                   _icbm_k=0.0, _icbm_k_n=0, _icbm_k_ahead=True, _icbm_k_at=0.0, _icbm_k_at_d=0.0, _icbm_k_at_n=0,
                   _icbm_k_at_gap=0.0, _stock_set=stock, _stock_on=True, _icbm_last_pub=-1e9).items():
    setattr(c, k, v)
  return c, cls._icbm_step.__get__(c), pubs


def _gate_off(monkeypatch):
  monkeypatch.setattr(m, "icbm_passed_points", lambda *a, **k: (None, "off"))


def _drive(c, step, pubs, clock, pts, y0, seconds, v=V, v_set=VSET, bearing=0.0, vis=None, follow=False):
  """Drive north from y0 at v; the fix is re-read every tick with fix_ts = now - keep, so ICBM's projected position is
  the true one. vis(y) -> (curve_lat_accel_vision, time_to_curve) or None. Returns [(t, y, published)]."""
  out = []
  t0 = clock[0]
  while clock[0] - t0 < seconds - 1e-9:
    y = y0 + v * (clock[0] - t0)
    c._map_targets = pts
    c._cur_lat, c._cur_lon = _ll(0.0, y)
    c._cur_bearing = bearing
    c._gps_fix_ts = clock[0] - m.ICBM_GPS_LAG_KEEP_S
    mtv, mtd = m.upcoming_curve(pts, c._cur_lat, c._cur_lon, v, m.C.CURVE_MAP_LOOKAHEAD_S)
    va, ttc = vis(y) if vis is not None else (0.0, 10.0)
    sig = {"v_ego": v, "v_set": c._stock_set if follow else v_set, "map_target_v": mtv, "map_target_dist": mtd,
           "curve_lat_accel_vision": va, "time_to_curve": ttc, "lat_accel_now": 0.0, "has_lead": False,
           "lead_drel": 0.0, "lead_vlead": 0.0, "gas": False, "brake": False, "spd_lim": 0.0, "pitch": None,
           "vis_k_max": None, "vis_reach": 0.0}
    n = len(pubs)
    c._icbm_last_pub = -1e9
    step(sig, active=True)
    p = pubs[-1][1] if len(pubs) > n else {}
    out.append((round(clock[0] - t0, 3), round(y, 2), p, c._icbm_src, c._icbm_gate, c._icbm_ep.phase))
    if follow and p.get("target") is not None:
      if p.get("dir", "dec") == "dec" and c._stock_set > p["target"] + 0.2:
        c._stock_set -= min(c._stock_set - p["target"], STEP_MS)
      elif p.get("dir") == "inc" and c._stock_set < p["target"] - 0.2:
        c._stock_set += min(p["target"] - c._stock_set, STEP_MS)
    clock[0] = round(clock[0] + 0.25, 6)
  return out


def _road(passed_curve=True, ahead_curve_at=None, v_curve=15.0, back=400.0, fwd=900.0):
  """mapd-style path in travel order from its node 0 `back` m behind the truck's start (y=0), 20 m spacing. A passed
  curve: velocity v_curve on the points 100-60 m behind. An ahead curve: 3 points from ahead_curve_at."""
  xyv, y = [], -back
  while y <= fwd:
    v = 0.0
    if passed_curve and -100.0 <= y <= -60.0:
      v = v_curve
    if ahead_curve_at is not None and ahead_curve_at <= y <= ahead_curve_at + 40.0:
      v = v_curve
    xyv.append((0.0, y, v))
    y += 20.0
  return _pts(xyv)


def _starts(trace):
  return [(t, y, p["target"], src) for t, y, p, src, _, _ in trace if p.get("target") is not None]


class TestTheGate:
  def test_the_curve_just_driven_no_longer_starts_a_slowdown(self, monkeypatch):
    mp = pytest.MonkeyPatch()
    try:
      _gate_off(mp)
      clock, ev = [100.0], []
      c, step, pubs = _controller(mp, clock, ev)
      base = _drive(c, step, pubs, clock, _road(), 0.0, 2.0)
    finally:
      mp.undo()
    assert _starts(base), "the base code does not start on the passed curve -- the test proves nothing"
    clock, ev = [100.0], []
    c, step, pubs = _controller(monkeypatch, clock, ev)
    gated = _drive(c, step, pubs, clock, _road(), 0.0, 2.0)
    assert not _starts(gated), f"a passed curve started a slowdown: {_starts(gated)[:2]}"
    assert all(g == "mapPassed" for *_, g, _ in gated)
    passed = [kw for _, name, kw in ev if name == "ces_icbm_passed"]
    assert passed == [{"state": "passed", "src": "map", "dist": passed[0]["dist"], "passed": passed[0]["passed"],
                       "started": None}], f"logged {passed} (must be once, change-only)"
    assert passed[0]["dist"] == pytest.approx(100.0, abs=1.0)

  def test_with_the_gate_off_the_harness_starts_where_the_old_code_did(self, monkeypatch):
    """The control for the test above, and for every 'kept' assertion below."""
    _gate_off(monkeypatch)
    clock, ev = [100.0], []
    c, step, pubs = _controller(monkeypatch, clock, ev)
    tr = _drive(c, step, pubs, clock, _road(), 0.0, 1.0)
    assert _starts(tr)[0][0] == 0.0
    assert not [e for e in ev if e[1] == "ces_icbm_passed" and e[2]["state"] == "passed"]

  def test_a_curve_still_ahead_starts_instead_exactly_as_if_the_passed_one_were_not_there(self, monkeypatch):
    clock, ev = [200.0], []
    c, step, pubs = _controller(monkeypatch, clock, ev)
    both = _drive(c, step, pubs, clock, _road(passed_curve=True, ahead_curve_at=220.0), 0.0, 3.0)
    clock, ev2 = [200.0], []
    c2, step2, pubs2 = _controller(monkeypatch, clock, ev2)
    clean = _drive(c2, step2, pubs2, clock, _road(passed_curve=False, ahead_curve_at=220.0), 0.0, 3.0)
    assert _starts(clean), "the ahead curve never binds -- proves nothing"
    assert _starts(both)[:1] == _starts(clean)[:1]
    assert c._icbm_ep._last_cap_dist is not None
    # the start tick hands the episode the curve AHEAD, not the passed one: replay just that tick in both scenes
    for scene, passed in ((c, True), (c2, False)):
      clock[0] = 900.0
      scene._icbm_ep = m.IcbmEpisode()
      _drive(scene, step if passed else step2, pubs if passed else pubs2, clock,
             _road(passed_curve=passed, ahead_curve_at=220.0), 0.0, 0.25)
    assert c._icbm_ep._last_cap_dist == pytest.approx(c2._icbm_ep._last_cap_dist, abs=1e-6), \
      "the episode got the passed curve's distance, not the one it started for"
    assert c._icbm_ep._last_cap_dist == pytest.approx(220.0, abs=1.0)
    assert _starts(both) == _starts(clean)
    assert both[0][4] == "mapPassed" and clean[0][4] is None
    started = [kw["started"] for _, name, kw in ev if name == "ces_icbm_passed"]
    assert started[:1] == ["map"], ev

  def test_real_start_timing_is_unchanged_with_node0_paths(self, monkeypatch):
    """A curve 600 m ahead approached at 25 m/s, mapd's path from 400 m behind: clean vs. a passed curve behind the
    start. The passed curve must not move the real start by a tick; with the gate off it starts at once (control)."""
    seen = {}

    def first(passed, gate):
      mp = pytest.MonkeyPatch()
      try:
        if not gate:
          _gate_off(mp)
        clock, ev = [300.0], []
        c, step, pubs = _controller(mp, clock, ev)
        tr = _drive(c, step, pubs, clock, _road(passed_curve=passed, ahead_curve_at=600.0), 0.0, 24.0)
        seen[(passed, gate)] = ([row[4] for row in tr], [kw["state"] for _, name, kw in ev if name == "ces_icbm_passed"])
        return _starts(tr)[:1]
      finally:
        mp.undo()
    clean = first(False, True)
    assert "mapPassed" not in seen[(False, True)][0] and seen[(False, True)][1] == ["clear"], \
      f"the gate acted on a clean approach: {seen[(False, True)][1]}"
    assert clean and clean[0][1] > 200.0, f"the real curve did not start on its approach: {clean}"
    assert first(True, True) == clean
    assert first(False, False) == clean
    assert first(True, False)[0][0] == 0.0, "with the gate off the passed curve should start at once (control)"

  def test_the_gate_judges_from_icbms_projected_position(self, monkeypatch):
    """The fix is 1 s older than the keep, so ICBM projects it 25 m forward. A curve 15 m AHEAD of the raw fix is 10 m
    BEHIND ICBM's own position -- the one every distance in the start decision was measured from."""
    clock, ev = [950.0], []
    c, step, pubs = _controller(monkeypatch, clock, ev)
    pts = _pts([(0.0, float(y), 15.0 if y == 15 else 0.0) for y in range(-400, 901, 5)])
    c._map_targets = pts
    c._cur_lat, c._cur_lon, c._cur_bearing = _ll(0.0, 0.0) + (0.0,)
    c._gps_fix_ts = clock[0] - m.ICBM_GPS_LAG_KEEP_S - 1.0
    sig = {"v_ego": V, "v_set": VSET, "map_target_v": 0.0, "map_target_dist": float("inf"), "curve_lat_accel_vision": 0.0,
           "time_to_curve": 10.0, "lat_accel_now": 0.0, "has_lead": False, "lead_drel": 0.0, "lead_vlead": 0.0,
           "gas": False, "brake": False, "spd_lim": 0.0, "pitch": None, "vis_k_max": None, "vis_reach": 0.0}
    step(sig, active=True)
    assert c._icbm_gate == "mapPassed" and (not pubs or pubs[-1][1].get("target") is None), (c._icbm_gate, pubs[-1:])

  def test_start_only_a_running_episode_is_untouched(self, monkeypatch):
    """An episode that started on the approach keeps its full candidate set after the truck passes the curve. The
    comparison runs until the base episode leaves its cap phase, and must include ticks where the binding point was
    already passed (else it proves nothing)."""
    def run(gate):
      mp = pytest.MonkeyPatch()
      try:
        if not gate:
          _gate_off(mp)
        clock, ev = [400.0], []
        c, step, pubs = _controller(mp, clock, ev)
        return _drive(c, step, pubs, clock, _road(passed_curve=False, ahead_curve_at=400.0), 0.0, 30.0, follow=True)
      finally:
        mp.undo()
    base, gated = run(False), run(True)
    first = next(i for i, row in enumerate(base) if row[2].get("target") is not None)
    end = next((i for i in range(first, len(base)) if base[i][5] != "cap"), len(base))
    past = [row for row in base[first:end] if row[1] > 400.0 + 40.0 + m.ICBM_PASSED_TOL_M]
    assert past, "the episode ended before the truck passed the curve -- proves nothing"
    assert [r[2] for r in gated[:end]] == [r[2] for r in base[:end]]

  def test_known_limit_a_running_episode_still_takes_a_lower_passed_point(self, monkeypatch):
    """START only, as specified (Fable: do not end a real episode on noise). An episode started for a curve AHEAD keeps
    the full candidate set, so a passed point rated slower still lowers its target. Pinned so a change is deliberate."""
    clock, ev = [450.0], []
    c, step, pubs = _controller(monkeypatch, clock, ev)
    pts = _road(passed_curve=False, ahead_curve_at=220.0, v_curve=20.0)
    for i, p in enumerate(_road(v_curve=12.0)):
      if p["velocity"]:
        pts[i] = dict(pts[i], velocity=12.0)                 # the passed curve rated 12, the one ahead 20
    tr = _drive(c, step, pubs, clock, pts, 0.0, 2.0)
    assert tr[0][4] == "mapPassed" and tr[0][2].get("target") is not None, "the start was not reselected -- proves nothing"
    assert tr[-1][2]["target"] < tr[0][2]["target"] - 1.0, "the running episode ignored the slower passed point"

  def test_below_the_speed_floor_it_cannot_tell_starts_as_before_and_says_so(self, monkeypatch):
    clock, ev = [500.0], []
    c, step, pubs = _controller(monkeypatch, clock, ev)
    tr = _drive(c, step, pubs, clock, _road(v_curve=3.0), 0.0, 1.5, v=m.ICBM_PASSED_MIN_V - 0.5, v_set=10.0)
    assert _starts(tr), "no start below the floor -- the fail-open path is not exercised"
    unknown = [kw for _, name, kw in ev if name == "ces_icbm_passed"]
    assert [(u["state"], u["why"]) for u in unknown] == [("unknown", "slow")], unknown

  def test_a_crash_in_the_gate_starts_as_before_and_is_logged_once(self, monkeypatch):
    logged = []
    monkeypatch.setattr(m.cloudlog, "exception", lambda msg, *a, **k: logged.append(msg))
    monkeypatch.setattr(m, "icbm_passed_points", lambda *a, **k: (_ for _ in ()).throw(RuntimeError("boom")))
    clock, ev = [600.0], []
    c, step, pubs = _controller(monkeypatch, clock, ev)
    tr = _drive(c, step, pubs, clock, _road(), 0.0, 3.0)
    assert _starts(tr) and tr[0][2].get("target") is not None, "a gate crash stopped ICBM publishing"
    assert sum("behindgate2pnw" in s for s in logged) == 1
    assert not any("_icbm_step FAILED" in s for s in logged), "the crash escaped into _icbm_step's except"

  def test_the_reselection_keeps_visions_own_start_rule(self, monkeypatch):
    """A passed map curve plus a vision curve the map covers (visCovered): removing the map curve must not let vision
    start through the back door. Vision beyond the map's reach with time to act may start."""
    def run(vis_at, ttc, back=400.0):
      mp = pytest.MonkeyPatch()
      try:
        clock, ev = [700.0], []
        c, step, pubs = _controller(mp, clock, ev)
        vis = lambda y: (4.0, ttc)   # noqa: E731
        return _drive(c, step, pubs, clock, _road(fwd=vis_at, back=back), 0.0, 0.25, vis=vis)[0]
      finally:
        mp.undo()
    covered = run(900.0, 3.5)
    assert covered[2].get("target") is None and covered[4] == "mapPassed", covered
    blind = run(60.0, 5.0, back=100.0)    # the path spans -100..+60 m: vision's curve 125 m out is beyond its reach
    assert blind[3] == "vis" and blind[2].get("target") is not None, blind

  def test_chill_resets_the_change_only_log(self, monkeypatch):
    clock, ev = [800.0], []
    c, step, pubs = _controller(monkeypatch, clock, ev)
    _drive(c, step, pubs, clock, _road(), 0.0, 1.0)
    step({"v_ego": V}, active=False)
    clock[0] += 0.25
    _drive(c, step, pubs, clock, _road(), 0.0, 1.0)
    assert [kw["state"] for _, name, kw in ev if name == "ces_icbm_passed"] == ["passed", "passed"]

  def test_icbmGate_mapPassed_reaches_the_record(self):
    assert _record(_icbm_gate="mapPassed")["icbmGate"] == "mapPassed"


# ---------------------------------------------------------------------------------------------------------
# Real telemetry windows
# ---------------------------------------------------------------------------------------------------------
def _windows():
  with open(FIXTURE) as f:
    return json.load(f)["windows"]


def _replay(monkeypatch, w, gate, executor=False):
  """The real _icbm_step at 4 Hz over a logged window: the record's position (ICBM's projected position equals it at
  the record time), bearing, vEgo, vSet, stock set/ACC, gas, vision, posted limit and the OSM path; in-curve held on
  exactly the records the LOG held it (icbmGate inCurve). executor: the Ford executor taps the set at 100 Hz."""
  mp = pytest.MonkeyPatch()
  try:
    if not gate:
      _gate_off(mp)
    clock, ev = [w["records"][0]["t"]], []
    c, step, pubs = _controller(mp, clock, ev)
    stock, gov, frame, held = None, PressGovernor(), 0, None
    starts, last_on, sets = [], None, []
    recs = w["records"]
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
        on = p.get("target") is not None and p.get("dir", "dec") == "dec"
        if on and (last_on is None or clock[0] - last_on > 3.0):
          starts.append((round(clock[0] - w["t_start"], 2), c._icbm_src, round(p["target"], 2)))
        if on:
          last_on = clock[0]
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
    return starts, [kw for _, name, kw in ev if name == "ces_icbm_passed"], sets
  finally:
    mp.undo()


def _at_start(starts):
  return [s for s in starts if abs(s[0]) <= 1.5]


class TestRealWindows:
  @pytest.mark.parametrize("w", _windows(), ids=lambda w: w["name"])
  def test_the_window(self, monkeypatch, w):
    base, _, _ = _replay(monkeypatch, w, gate=False)
    gated, ev, _ = _replay(monkeypatch, w, gate=True)
    assert _at_start(base), f"{w['when_pt']}: the base code does not reproduce the logged start {base} -- proves nothing"
    if w["kind"] == "phantom":
      assert not _at_start(gated), f"{w['when_pt']} ({w['why']}): still starts {gated}"
      assert any(e["state"] == "passed" for e in ev)
    else:
      assert gated == base, f"{w['when_pt']} ({w['why']}): a real start changed {base} -> {gated}"

  def test_the_fixture_covers_both_directions(self):
    kinds = [w["kind"] for w in _windows()]
    assert kinds.count("phantom") >= 6 and kinds.count("real") >= 6


class TestClosedLoopThroughTheFordExecutor:
  def _window(self, name):
    return next(w for w in _windows() if w["name"] == name)

  def test_the_curve_exit_set_taps_disappear(self, monkeypatch):
    w = self._window("sun_1205_curve_exit")
    _, _, base = _replay(monkeypatch, w, gate=False, executor=True)
    _, _, gated = _replay(monkeypatch, w, gate=True, executor=True)
    start_set = w["records"][0]["stockSet"]
    assert min(base) <= start_set - 3 * MPH, f"the base code did not tap the set down ({min(base) / MPH:.1f} mph) -- proves nothing"
    assert gated == [start_set] * len(gated), f"the gate still tapped: {min(gated) / MPH:.1f} mph"

  def test_a_real_curve_is_tapped_identically(self, monkeypatch):
    w = self._window("sun_1209_real")
    _, _, base = _replay(monkeypatch, w, gate=False, executor=True)
    _, _, gated = _replay(monkeypatch, w, gate=True, executor=True)
    assert min(base) < w["records"][0]["stockSet"] - STEP_MS, "no tap on the real curve -- proves nothing"
    assert gated == base


# ---------------------------------------------------------------------------------------------------------
# The Tesla
# ---------------------------------------------------------------------------------------------------------
class TestTheTeslaIsUntouched:
  """End to end through CESController.experimental_request, with a mapd path whose curve is behind the car: the Tesla
  (op-long, no ICBM) never calls the gate, and its decisions and overlay feed are identical with the gate poisoned.
  The Lightning run is the control that shows the spy can see a call."""

  @staticmethod
  def _run(monkeypatch, fp, brand, op_long, poison):
    NS = types.SimpleNamespace
    la, lo = _ll(0.0, 0.0)
    path = [{"latitude": p["latitude"], "longitude": p["longitude"], "velocity": p["velocity"]} for p in _road()]
    blob = json.dumps({"latitude": la, "longitude": lo, "bearing": 0.0, "src": "car", "fix_ts": 7000.0 - m.ICBM_GPS_LAG_KEEP_S})

    class P:
      def get(self, k, return_default=False):
        return {"CESMode": "2", "CESButtonState": "0"}.get(k)

      def get_bool(self, k):
        return k == "CESCurves"                            # map curves on, so the Lightning's ICBM sees the path

    class Mem:
      def __init__(self):
        self.puts = []

      def get(self, k, return_default=False):
        return {"MapTargetVelocities": path, "LastGPSPosition": blob}.get(k)

      def put_nonblocking(self, k, v):
        self.puts.append((k, {kk: vv for kk, vv in v.items() if kk != "ts"} if isinstance(v, dict) else v))

    calls = {"gate": 0}
    real = m.icbm_passed_points
    if poison:
      def boom(*a, **k):
        raise AssertionError("behindgate helper called")
      monkeypatch.setattr(m, "icbm_passed_points", boom)
      monkeypatch.setattr(m, "_icbm_passed_gate", boom)
    else:
      monkeypatch.setattr(m, "icbm_passed_points", lambda *a, **k: (calls.__setitem__("gate", calls["gate"] + 1), real(*a, **k))[1])
    clock = [7000.0]
    monkeypatch.setattr(m.time, "monotonic", lambda: clock[0])
    from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_curvelead2pnw import FakeCP
    c = m.CESController(FakeCP(fp, brand, op_long), params=P())
    c.mem_params = Mem()
    c._event_log_ok = False
    n = 33
    model = NS(orientationRate=NS(z=[0.0] * n, t=[i * 0.3 for i in range(n)]), velocity=NS(x=[25.0] * n),
               position=NS(x=[25.0 * i * 0.3 for i in range(n)]), action=NS(shouldStop=False), meta=NS(laneChangeState="off"))
    sm = {"radarState": NS(leadOne=NS(status=False, vLead=0.0, dRel=0.0)), "modelV2": model,
          "carControl": NS(orientationNED=[0.0, 0.0, 0.0])}
    cs = NS(vEgo=25.0, aEgo=0.0, gasPressed=False, brakePressed=False, leftBlinker=False, rightBlinker=False, vCruise=108.0,
            standstill=False, steeringAngleDeg=0.0, steeringPressed=False, leftBlindspot=False, rightBlindspot=False,
            cruiseState=NS(speed=30.0, enabled=True))
    decisions = []
    for _ in range(12):
      decisions.append(c.experimental_request(cs, sm))
      clock[0] += 0.3
    monkeypatch.undo()
    return decisions, c.mem_params.puts, calls

  def test_tesla_decisions_and_overlay_are_identical_with_the_gate_poisoned(self, monkeypatch):
    d_real, puts_real, calls = self._run(monkeypatch, "TESLA_MODEL_S_HW3", "tesla", True, poison=False)
    d_poison, puts_poison, _ = self._run(monkeypatch, "TESLA_MODEL_S_HW3", "tesla", True, poison=True)
    assert calls == {"gate": 0}, f"the Tesla ran the gate: {calls}"
    assert not any(k == "IcbmTarget" for k, _ in puts_real)
    assert d_real == d_poison and puts_real == puts_poison

  def test_the_lightning_control_run_does_call_it(self, monkeypatch):
    _, puts, calls = self._run(monkeypatch, LIGHTNING, "ford", False, poison=False)
    assert calls["gate"] > 0, f"the spy cannot see the Lightning either -- test is blind: {calls}"
