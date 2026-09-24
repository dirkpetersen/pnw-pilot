"""curvedblive2pnw: the learned curve database sets ICBM's curve target LIVE.

What is pinned here, each by a test that fails if the rule is broken:
  * the target rule: a raise is exactly sqrt(A / k) within the +15 mph / posted + 10 mph caps (terwilliger2pnw removed
    the 1.25x curvature margin); a lowering is exactly sqrt(A / k); never above the set;
  * an unknown, ambiguous or refused branch is NO effect, in both directions;
  * a missing / corrupt / foreign file is DB OFF, loudly, never partial; an unreadable A is DB OFF for the decision;
  * the kill switch and the Tesla: byte-identical ICBM output;
  * the cdb2* fields reach the ces_events line on disk;
  * the build report's replay table, reproduced from its numbers (the positions stay private; the full geometric
    replay is test_curvedblive2pnw_replay.py, on the private fixture).
"""
from __future__ import annotations

import builtins
import copy
import json
import math
import types

import pytest

from openpilot.selfdrive.controls.lib import pnw_vehicle as pv
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw_constants as C
from openpilot.selfdrive.controls.lib.ces_pnw import curvedb_live as cl
from openpilot.selfdrive.controls.lib.ces_pnw.tests.roaddb_fixture import FAR_ANCHOR, SHIPPED_LAT_A, reader, write_db
from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_ces_mode_read_failure_logged import LIGHTNING, _P
from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_curvelead2pnw import LAT0, LON0, FakeCP, _model

NS = types.SimpleNamespace
MPH = cl.MPH
TESLA = "TESLA_MODEL_S_HW3"
A = 2.2


def _ll(x, y):
  return LAT0 + y / 111320.0, LON0 + x / (111320.0 * math.cos(math.radians(LAT0)))


def _path(curve=None, y0=-180.0, y1=900.0, step=40.0):
  """mapd-style path due north through the truck (which sits at y = 0), a node every 40 m from -180 m (so 300 and
  460 are nodes); `curve` = {y: mapd velocity}."""
  pts, y = [], y0
  while y <= y1:
    la, lo = _ll(0.0, y)
    pts.append({"latitude": la, "longitude": lo, "velocity": (curve or {}).get(y, 0.0)})
    y += step
  return pts


def _anchor(y, k, branches=None, brg=0.0):
  la, lo = _ll(0.0, y)
  brs = branches if branches is not None else [(0.0, y + 150.0, k)]
  return [la, lo, brg, [[*_ll(bx, by), bk, 3] for bx, by, bk in brs]]


class Logs:
  def __init__(self):
    self.errors, self.events, self.exceptions = [], [], []

  def error(self, msg, *a, **k):
    self.errors.append(msg)

  def event(self, name, *a, **k):
    self.events.append((name, k))

  def exception(self, msg, *a, **k):
    self.exceptions.append(msg)

  def warning(self, *a, **k):
    pass

  def info(self, *a, **k):
    pass


@pytest.fixture
def logs(monkeypatch):
  lg = Logs()
  monkeypatch.setattr(cl, "cloudlog", lg)
  return lg


# =====================================================================================================
# the target rule
# =====================================================================================================
class TestTargetRule:
  def test_a_raise_is_exactly_the_row_speed_no_margin(self):
    """terwilliger2pnw: a raise uses v_db itself, the same speed a lowering uses (was sqrt(A / (1.25 k)))."""
    k, today = 0.001, 44.0                                  # v_db 46.90 m/s, inside the +15 mph cap (50.7)
    v, d = cl.db_target(today, k, A, ref=60.0, posted=None)
    assert d == "raise" and v == pytest.approx(cl.v_db(A, k), abs=1e-9)
    assert v > math.sqrt(A / (1.25 * k)) + 4.0, "the raise carries the old 1.25x curvature margin again"

  def test_a_raise_is_capped_at_plus_15_mph(self):
    v, d = cl.db_target(20.0, 0.0004, A, ref=60.0, posted=None)   # margin'd 66 m/s
    assert d == "raise" and v == pytest.approx(20.0 + 15 * MPH, abs=1e-9)

  def test_a_raise_is_capped_at_posted_plus_10_mph(self):
    v, _ = cl.db_target(20.0, 0.0004, A, ref=60.0, posted=20.0)
    assert v == pytest.approx(20.0 + 10 * MPH, abs=1e-9)

  def test_a_raise_never_exceeds_the_set(self):
    v, _ = cl.db_target(20.0, 0.0004, A, ref=22.0, posted=None)
    assert v == pytest.approx(22.0)

  def test_terwilliger_I_is_now_raised_to_the_row_speed(self):
    """Terwilliger I (09-21 23:39): v_db 49.1 mph over ICBM's 44.0. The old 1.25 margin gave 43.96 and ICBM's target
    stood; without it the raise goes to 49.1 (held-out pass 2.21 m/s^2 there, test_the_replay_table_...)."""
    today, k = 44.045275590551185 * MPH, 0.004557081583558678
    v, d = cl.db_target(today, k, A, ref=59 * MPH, posted=22.4)
    assert d == "raise" and v == pytest.approx(cl.v_db(A, k), abs=1e-9) and v / MPH == pytest.approx(49.15, abs=0.01)

  def test_a_raise_the_posted_cap_withholds_is_held(self):
    """posted + 10 mph at or below ICBM's target: nothing changes, and the direction says why ("held")."""
    v, d = cl.db_target(20.0, 0.001, A, ref=60.0, posted=20.0 - 10 * MPH)
    assert d == "held" and v == 20.0
    v, d = cl.db_target(20.0, 0.001, A, ref=20.0, posted=None)        # the set itself
    assert d == "held" and v == 20.0

  def test_a_lowering_is_exactly_the_row_speed_no_margin(self):
    k = 0.004
    v, d = cl.db_target(30.0, k, A, ref=33.0, posted=31.0)
    assert d == "lower" and v == pytest.approx(math.sqrt(A / k), abs=1e-12)

  def test_same_speed_is_no_change(self):
    k = 0.004
    v, d = cl.db_target(cl.v_db(A, k), k, A, ref=33.0, posted=None)
    assert d == "none"

  def test_bad_inputs_raise_rather_than_guess(self):
    with pytest.raises(ValueError):
      cl.db_target(30.0, 0.0, A, ref=33.0, posted=None)
    with pytest.raises(ValueError):
      cl.db_target(float("nan"), 0.001, A, ref=33.0, posted=None)


# =====================================================================================================
# the build report's replay table, from its numbers (docs/CURVEDB-V2-BUILD.md s4.2 episodes, A = 2.2), through the
# CURRENT rule -- no raise margin since terwilliger2pnw. The report's own column (margin 1.25) kept I and J at ICBM's
# 44.0 / 49.8; without the margin they are raised to their row speeds, still below 2.5 on the held-out pass.
# =====================================================================================================
# (PT, ref mph, ICBM mph, posted m/s, LODO k_row, own-pass k_truth, owner's class) -- no positions: those are private
REPLAY = [
  ("09-08 20:28:51 Olympia phantom", 70.00000202067, 44.11238367931281, 26.8, 0.0018152179572738076, 0.0017012949959797, "unwanted"),
  ("09-17 18:39:24 SR 99 far", 55.00000226491014, 21.340372226198998, 22.4, 0.0004019119165926268, 0.00038886622822021977, "unwanted"),
  ("09-21 21:20:23 B Tumwater", 74.99999877879931, 48.38493199713672, 26.8, 0.0019092734894621817, 0.0018023576525494234, "unwanted"),
  ("09-21 21:21:41 C Tumwater S", 74.99999877879931, 63.57372942018612, 26.8, 0.0018631597168344883, 0.0019263556727453454, "unwanted"),
  ("09-21 23:38:12 H I-405 jct", 55.00000226491014, 48.273085182534, 24.6, 0.0030237375121294407, 0.0030611011383598523, "unwanted"),
  ("09-21 23:39:14 I Terwilliger", 58.999998723276384, 44.045275590551185, 22.4, 0.004557081583558678, 0.004579531400473212, "wanted"),
  ("09-21 23:40:01 J Terwilliger", 58.999998723276384, 49.838940586972086, 22.4, 0.004018049922301351, 0.004008636677515643, "marginal"),
]
# the raises B/C/H/Olympia/SR99 land where the build report's v2 column put them, except where the old margin bound
# (C: +15 mph cap 69.95 instead of the margin'd 68.75; H: v_db 60.3 is at/above the 55 set -> REMOVED); I and J go to
# their row speeds (terwilliger2pnw)
EXPECTED_MPH = [59.11238367931281, 36.340372226198994, 63.38493199713672, 69.95, 55.0, 49.15, 52.34]
MIN_RED_MS = 1.0     # v2_replay.MIN_RED_MS: within this of the reference = no slowdown left
REAL = 2.5


def replay_outcome(ref, icbm, posted, k_row, k_truth, cls, a=A):
  tgt, _ = cl.db_target(icbm * MPH, k_row, a, ref=ref * MPH, posted=posted)
  a_t = k_truth * tgt ** 2
  raised, gone = tgt > icbm * MPH + 0.5, tgt >= ref * MPH - MIN_RED_MS
  if cls == "unwanted":
    return tgt, a_t, "REMOVED" if gone else "reduced" if raised else "unchanged"
  if not raised:
    return tgt, a_t, "kept"
  return tgt, a_t, ("LOST" if gone else "WEAKENED") if a_t >= REAL else "raised, < 2.5"


def test_the_replay_table_is_reproduced_from_its_numbers():
  rows = [replay_outcome(*r[1:]) for r in REPLAY]
  for (name, *_), (tgt, _a, _o), want in zip(REPLAY, rows, EXPECTED_MPH, strict=True):
    assert tgt / MPH == pytest.approx(want, abs=0.01), name
  outcomes = [o for _t, _a, o in rows]
  unwanted = [o for r, o in zip(REPLAY, outcomes, strict=True) if r[-1] == "unwanted"]
  assert sorted(unwanted) == ["REMOVED", "reduced", "reduced", "reduced", "reduced"], "5 of 5 unwanted removed/reduced"
  real = [(r, t, a, o) for r, (t, a, o) in zip(REPLAY, rows, strict=True) if r[5] * (r[1] * MPH) ** 2 >= REAL]
  assert len(real) == 2
  assert all(o == "raised, < 2.5" for _r, _t, _a, o in real), "a real curve was weakened to >= 2.5, or not raised"
  assert not [r for r, _t, a, o in real if o in ("LOST", "WEAKENED")], "0 wanted weakened to >= 2.5"


# the same seven at mapd's A on the truck (2.0): mph at the candidate, no raise margin
EXPECTED_MPH_A20 = [59.1, 36.3, 63.4, 69.95, 55.0, 46.86, 49.84]


def test_the_replay_table_at_the_trucks_A_2_0():
  rows = [replay_outcome(*r[1:], a=2.0) for r in REPLAY]
  for (name, *_), (tgt, _a, _o), want in zip(REPLAY, rows, EXPECTED_MPH_A20, strict=True):
    assert tgt / MPH == pytest.approx(want, abs=0.06), name
  assert sorted(o for r, (_t, _a, o) in zip(REPLAY, rows, strict=True) if r[-1] == "unwanted") == \
    ["REMOVED", "reduced", "reduced", "reduced", "reduced"]
  real = [(a, o) for r, (_t, a, o) in zip(REPLAY, rows, strict=True) if r[5] * (r[1] * MPH) ** 2 >= REAL]
  assert len(real) == 2 and all(o in ("kept", "raised, < 2.5") and a < REAL for a, o in real)


# the shipped A (2.5, curve.json): the DB aims at exactly 2.5 on its row, so a real curve lands AT the owner's line
EXPECTED_MPH_A25 = [59.1, 36.3, 63.4, 69.95, 55.0, 52.39, 55.80]


def test_the_replay_table_at_the_shipped_A_2_5():
  """terwilliger2pnw: at A = 2.5 with no margin, Terwilliger I is raised 44.0 -> 52.4 mph and its held-out pass
  measures 2.51 m/s^2 there; J 49.8 -> 55.8 at 2.49. "Only to the level required": the row speed IS the 2.5 line,
  so a real curve sits on it -- never meaningfully past it. Bounded here at 2.6 (the rows' p50 spread)."""
  rows = [replay_outcome(*r[1:], a=2.5) for r in REPLAY]
  for (name, *_), (tgt, _a, _o), want in zip(REPLAY, rows, EXPECTED_MPH_A25, strict=True):
    assert tgt / MPH == pytest.approx(want, abs=0.06), name
  assert sorted(o for r, (_t, _a, o) in zip(REPLAY, rows, strict=True) if r[-1] == "unwanted") == \
    ["REMOVED", "reduced", "reduced", "reduced", "reduced"]
  real = [a for r, (_t, a, _o) in zip(REPLAY, rows, strict=True) if r[5] * (r[1] * MPH) ** 2 >= REAL]
  assert len(real) == 2 and max(real) < 2.6, real


# 2026-09-24 12:35 PT Terwilliger Curves, I-5 north (drives/2026-09-24/terwilliger-too-slow): posted 50, set 60 (zone
# set 1.2 x 50). mapd rated curve A 47.4 / curve B 49.0 and ICBM's post-penalty base was ~47.0 / ~47.2 mph. The
# measured curvature (3 s mean of |yaw|/v) was 0.00391 (A, right) and 0.00429 (B, left): 56.6 / 54.0 mph at 2.5.
TERWILLIGER = [("A right-hander", 47.0, 0.00391, 56.6), ("B left-hander", 47.2, 0.00429, 54.0)]


@pytest.mark.parametrize("name,base_mph,k,need_mph", TERWILLIGER)
def test_terwilliger_is_raised_to_what_the_curve_needs_at_2_5(name, base_mph, k, need_mph):
  """The drive's root cause: the 1.25 margin made every raise sqrt(2.0 / k) -- 50.6 / 48.3 mph here, mapd's own A.
  Without it the raise reaches the curve's 2.5 m/s^2 speed, inside every cap (+15 mph: 62; posted + 10: 60; set 60)."""
  v, d = cl.db_target(base_mph * MPH, k, 2.5, ref=60 * MPH, posted=50 * MPH)
  assert d == "raise", name
  assert v / MPH == pytest.approx(need_mph, abs=0.1), name
  assert v * v * k == pytest.approx(2.5, abs=1e-6), name      # exactly the owner's line on the measured curve


def test_the_tumwater_left_curve_is_added_at_about_69_mph():
  """09-21 21:21:58: ICBM never slowed (mapd's 59.9 was inflated past the set and discarded); the row says 69.3."""
  k_row, k_truth, v_app = 0.002293353371888105, 0.0025637414026552746, 73.91508589835361 * MPH
  v = cl.v_db(A, k_row)                      # an add is a lowering: no margin
  assert v / MPH == pytest.approx(69.28, abs=0.01)
  assert v < v_app - MIN_RED_MS
  assert k_truth * v_app ** 2 >= REAL        # it is a real curve at the approach speed (2.80 m/s^2)


# =====================================================================================================
# geometry: the car's keying is the offline keying
# =====================================================================================================
def test_the_geometry_formulas_are_the_stores():
  from openpilot.tools.curvedb import store
  for a in ((47.0, -122.0, 47.001, -122.002), (45.0, -121.0, 45.0015, -120.9975)):
    assert cl.haversine_m(*a) == store.haversine_m(*a)
    assert cl.initial_bearing_deg(*a) == store.initial_bearing_deg(*a)
  for x, y in ((10.0, 350.0), (180.0, 0.0), (359.0, 1.0)):
    assert cl.bearing_diff_deg(x, y) == store.bearing_diff_deg(x, y)


def test_nearest_anchor_and_branch_end_agree_with_roadtable_on_a_curved_pass():
  """Same pass, same anchor: the live Polyline's branch end is where roadtable.branch_point puts it."""
  from openpilot.tools.curvedb import roadtable as rt
  R, pts = 400.0, []
  for n in range(60):                              # 25 m points: 300 m straight, then a right-hand arc
    d = n * 25.0
    if d <= 300.0:
      x, y = 0.0, d
    else:
      th = (d - 300.0) / R
      x, y = R * (1 - math.cos(th)), 300.0 + R * math.sin(th)
    la, lo = _ll(x, y)
    pts.append(rt.Point(s=d, t=float(n), lat=la, lon=lo, brg=0.0, v=30.0, k=0.0, fix_ok=True, drv=False,
                        lat_active=True, way=0, hwy="motorway", road="", spl=0.0, mcs=0.0))
  for i, p in enumerate(pts):
    a, b = pts[max(i - 1, 0)], pts[min(i + 1, len(pts) - 1)]
    p.brg = cl.initial_bearing_deg(a.lat, a.lon, b.lat, b.lon)
  alat, alon = _ll(8.0, 290.0)                     # anchor 8 m off the line, 10 m before the pass point
  anchor = rt.Anchor(alat, alon, 0.0)
  i = 12                                           # the pass point at 300 m
  want = rt.branch_point(pts, i, anchor, 150.0)
  poly = cl.Polyline([{"latitude": p.lat, "longitude": p.lon} for p in pts])
  idx = cl.RowIndex([[alat, alon, 0.0, [[want[0], want[1], 0.002, 3]]]], cl.EXPECTED_PARAMS)
  s_q, _ = poly.project(pts[i].lat, pts[i].lon)
  mt = cl.match_at(idx, poly, s_q, pts[i].lat, pts[i].lon)
  assert mt.why == "ok" and mt.k == 0.002
  got = poly.at(mt.s_anchor + 150.0)
  assert cl.haversine_m(*got, *want) < 1.0


class TestBranchGate:
  def _m(self, branches, path_x_at_end=0.0):
    idx = cl.RowIndex([_anchor(300.0, None, branches)], cl.EXPECTED_PARAMS)
    poly = cl.Polyline(_path())
    la, lo = _ll(0.0, 300.0)
    s_q, _ = poly.project(la, lo)
    return cl.match_at(idx, poly, s_q, la, lo)

  def test_known_branch(self):
    assert self._m([(0.0, 450.0, 0.002)]).why == "ok"

  def test_unknown_branch_the_row_goes_elsewhere(self):
    assert self._m([(40.0, 450.0, 0.002)]).why == "branchUnknown"

  def test_ambiguous_two_branches_both_near(self):
    assert self._m([(0.0, 450.0, 0.002), (10.0, 450.0, 0.001)]).why == "branchAmbiguous"

  def test_the_path_takes_a_refused_branch(self):
    assert self._m([(0.0, 450.0, None), (20.0, 450.0, 0.002)]).why == "noAuthority"

  def test_the_path_is_too_short_to_know(self):
    idx = cl.RowIndex([_anchor(300.0, 0.002)], cl.EXPECTED_PARAMS)
    poly = cl.Polyline(_path(y1=400.0))
    la, lo = _ll(0.0, 300.0)
    assert cl.match_at(idx, poly, poly.project(la, lo)[0], la, lo).why == "branchUnknown"

  def test_the_wrong_direction_finds_no_anchor(self):
    idx = cl.RowIndex([_anchor(300.0, 0.002, brg=180.0)], cl.EXPECTED_PARAMS)
    poly = cl.Polyline(_path())
    la, lo = _ll(0.0, 300.0)
    assert cl.match_at(idx, poly, poly.project(la, lo)[0], la, lo).why == "noAnchor"


# =====================================================================================================
# the file: verified, whole, or OFF -- loudly
# =====================================================================================================
class TestLoader:
  def _live(self, d, logs, read=None):
    db = cl.CurveDbLive(True, data_dir=str(d), read_params=read or reader(), start=False)
    db.load()
    return db

  def test_a_good_file_loads(self, tmp_path, logs):
    db = self._live(write_db(str(tmp_path), [FAR_ANCHOR, _anchor(300.0, 0.002)]), logs)
    assert db.state == "ok" and db.index.n_rows == 2 and logs.errors == []
    assert [n for n, _ in logs.events] == ["curvedb_v2_loaded"]

  @pytest.mark.parametrize("case", ["missing", "tamper", "swapped", "format", "lodo", "params", "count", "k", "dates", "nozstd"])
  def test_a_bad_file_is_OFF_loud_and_never_partial(self, tmp_path, logs, monkeypatch, case):
    d = str(tmp_path)
    good = [FAR_ANCHOR, _anchor(300.0, 0.002)]
    if case == "missing":
      pass
    elif case == "tamper":
      write_db(d, good, tamper=lambda b: b[:-3] + b"xyz")
    elif case == "swapped":          # a VALID file, just not the one the manifest vouches for: only the hash sees it
      other = str(tmp_path / "other")
      write_db(other, [FAR_ANCHOR, _anchor(300.0, 0.009)])
      write_db(d, good, tamper=lambda b: (tmp_path / "other" / cl.ROWS_NAME).read_bytes())
    elif case == "format":
      write_db(d, good, fmt="curvedb-v2-live/0")
    elif case == "lodo":
      write_db(d, good, exclude_date="2026-09-21")
    elif case == "params":
      write_db(d, good, params={**cl.EXPECTED_PARAMS, "branch_radius_m": 30.0})
    elif case == "count":
      write_db(d, good)
      man = json.loads((tmp_path / cl.MANIFEST_NAME).read_text())
      man["rows_with_authority"] = 3
      (tmp_path / cl.MANIFEST_NAME).write_text(json.dumps(man))
    elif case == "k":
      write_db(d, [FAR_ANCHOR, _anchor(300.0, -0.002)])
    elif case == "dates":
      bad = _anchor(300.0, 0.002)
      bad[3][0][3] = 1
      write_db(d, [FAR_ANCHOR, bad])
    elif case == "nozstd":
      write_db(d, good)
      real_import = builtins.__import__

      def no_zstd(name, *a, **k):
        if name == "zstandard":
          raise ImportError("No module named 'zstandard'")
        return real_import(name, *a, **k)
      monkeypatch.setattr(builtins, "__import__", no_zstd)
    db = self._live(d, logs)
    assert db.state == "err" and db.index is None and db.err
    assert len(logs.errors) == 1 and "LOAD FAILED" in logs.errors[0] and "OFF" in logs.errors[0]
    if case == "nozstd":
      assert "zstandard is not importable" in logs.errors[0]
    if case in ("tamper", "swapped"):
      assert "sha256" in logs.errors[0]
    assert db.gate(way_sel="current", hwy="motorway", posted=30.0, plat=LAT0, plon=LON0, allow=True) == "dbErr"

  def test_the_real_export_format_loads(self, tmp_path, logs):
    """tools/curvedb/v2_live_export.py writes what the car reads (same format string, names, hash)."""
    from openpilot.tools.curvedb import roadtable as rt
    from openpilot.tools.curvedb import v2_live_export as ex
    idx = rt.AnchorIndex(rt.PROVISIONAL_V2)
    a = idx.anchors[idx.add(LAT0, LON0, 0.0)]
    for n, date in enumerate(("2026-09-01", "2026-09-02", "2026-09-03")):
      a.obs.append(rt.PassObs(date=date, drive=f"d{n}", car=LIGHTNING, t=float(n), k_ext=0.002, k_loc=0.002, way=0,
                              hwy="motorway", road="I 5|", spl=30.0, mode="op", d_m=1.0, end_lat=LAT0 + 0.00135,
                              end_lon=LON0))
    doc, counts = ex.export_doc(idx)
    src = tmp_path / "table.json.gz"
    src.write_bytes(b"x")
    ex.write(doc, counts, str(tmp_path), str(src))
    assert ex.ROWS_NAME == cl.ROWS_NAME and ex.FORMAT == cl.FORMAT and ex.MANIFEST_NAME == cl.MANIFEST_NAME
    db = self._live(tmp_path, logs)
    assert db.state == "ok" and db.index.n_rows == 1


class TestA:
  @pytest.mark.parametrize("settings,pers,want", [
    (None, 1, None), ("{not json", 1, None), ({"personalities": {}}, 1, None), (b'{"personalities": 3}', 1, None),
    ({"personalities": {"standard": {"map_curve_target_lat_a": 9.0}}}, 1, None),
    ({"personalities": {"standard": {"map_curve_target_lat_a": float("nan")}}}, 1, None),
    ({"personalities": {"standard": {"map_curve_target_lat_a": 2.2}}}, 7, None),
    ({"personalities": {"standard": {"map_curve_target_lat_a": 2.2}}}, None, None),
    ({"personalities": {"standard": {"map_curve_target_lat_a": 2.2}}}, 1, 2.2),
    (json.dumps({"personalities": {"relaxed": {"map_curve_target_lat_a": 1.9}}}), b"2", 1.9),
    ({"map_curve_target_lat_a": 2}, None, 2.0),          # top-level layout: the personality is not needed
    ({"map_curve_target_lat_a": "x"}, 0, None), ({"map_curve_target_lat_a": 0.5}, 0, None), ([], 0, None),
    ({"settings_version": 1}, 0, None),                  # neither layout
  ])
  def test_parse(self, settings, pers, want):
    assert cl.parse_a(settings, pers)[0] == want

  def test_the_trucks_own_mapd_settings_read_2_0(self):
    """DEVICE-VERIFIED 2026-09-24 (read-only probe of the truck, custom mapd 77bad867): MapdSettings has NO
    `personalities`; map_curve_target_lat_a is TOP-LEVEL, the int 2; LongitudinalPersonality is the int 0; Params
    returns the param already parsed. The v2-layout-only first cut read this as unreadable -> DB OFF on every
    decision."""
    truck = {"accept_speed_limit_timeout": 0, "adjust_set_speed_to_accept_speed_limit": False,
             "conditional_speed_limit_control_enabled": False, "default_lane_width": 3.7, "enable_speed": 0,
             "external_speed_limit_control_enabled": False, "hold_last_seen_speed_limit": False,
             "hold_speed_limit_while_changing_set_speed": True, "log_json": True, "log_level": "error",
             "log_source": True, "map_curve_speed_control_enabled": False, "map_curve_target_lat_a": 2,
             "map_curve_use_enable_speed": False, "press_gas_to_accept_speed_limit": False,
             "press_gas_to_override_speed_limit": False, "settings_version": 1,
             "slow_down_for_next_speed_limit": True, "speed_limit_change_requires_accept": False,
             "speed_limit_control_enabled": False, "speed_limit_offset": 0, "speed_limit_priority": "map",
             "speed_limit_use_enable_speed": False, "speed_up_for_next_speed_limit": False, "target_speed_accel": 1.2}
    for raw in (truck, json.dumps(truck), json.dumps(truck).encode()):
      a, src, why = cl.parse_a(raw, 0)
      assert (a, src, why) == (2.0, "mapd:top", "ok") and isinstance(a, float)

  def test_curve_json_overrides_mapd(self, tmp_path, logs):
    db = cl.CurveDbLive(True, data_dir=write_db(str(tmp_path), [FAR_ANCHOR]), read_params=lambda: (None, 1),
                        start=False, a_override=2.5)
    db.load()
    db.poll_a()
    assert db.a_lat() == (2.5, "curve.json", "ok") and logs.errors == []   # mapd is not even read

  @pytest.mark.parametrize("cfg,want", [(None, 2.5), (0, None), (2.2, 2.2), (0.4, 1.0), (9.0, 3.5)])   # default 2.5 (owner 2026-09-24 11:25 PT)
  def test_the_curve_json_knob(self, tmp_path, monkeypatch, cfg, want):
    monkeypatch.setitem(pv._CURVE_DEFAULTS, "curvedb_v2_lat_a", SHIPPED_LAT_A)   # undo the fixture pin: test the real default
    f = tmp_path / "curve.json"
    if cfg is not None:
      f.write_text(json.dumps({"lightning": {"curvedb_v2_lat_a": cfg}}))
    monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(f))
    assert pv.PnwVehicle(FakeCP(LIGHTNING, "ford", False)).curvedb_v2_lat_a == want
    assert pv.PnwVehicle(FakeCP(TESLA, "tesla", True)).curvedb_v2_lat_a is None

  def test_unreadable_A_is_OFF_and_logged_once(self, tmp_path, logs):
    db = cl.CurveDbLive(True, data_dir=write_db(str(tmp_path), [FAR_ANCHOR]), read_params=lambda: (None, 1), start=False)
    db.load()
    db.poll_a()
    db.poll_a()
    assert db.gate(way_sel="current", hwy="motorway", posted=30.0, plat=LAT0, plon=LON0, allow=True) == "noA"
    assert len(logs.errors) == 1 and "UNREADABLE" in logs.errors[0]

  def test_a_stale_reading_is_no_reading(self, tmp_path, logs, monkeypatch):
    db = cl.CurveDbLive(True, data_dir=write_db(str(tmp_path), [FAR_ANCHOR]), read_params=reader(), start=False)
    db.load()
    db.poll_a()
    assert db.a_lat()[0] == 2.2
    monkeypatch.setattr(cl, "A_MAX_AGE_S", -1.0)
    assert db.a_lat()[0] is None


# =====================================================================================================
# the real controller, end to end
# =====================================================================================================
PHANTOM = _path({300.0: 20.0})                    # mapd: a 45 mph curve on a straight road
STRAIGHT = _path()                                # mapd: nothing to slow for
MAPD_LOW = _path({300.0: 24.0})                   # mapd: a curve, but not as sharp as the road


def _drive(monkeypatch, tmp_path, *, points, anchors, fp=LIGHTNING, brand="ford", op_long=False, switch=None,
           read=None, way_sel="current", stock_mph=75.0, posted_mph=70.0, v_ego=33.0, ticks=400,
           log_file=None):
  tmp_path.mkdir(parents=True, exist_ok=True)
  cfg = tmp_path / "curve.json"
  if switch is not None:
    cfg.write_text(json.dumps({"lightning": {"curvedb_v2_live": switch}}))
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(cfg))
  monkeypatch.setattr(pv, "RAIN_CONFIG_PATH", str(tmp_path / "absent-rain.json"))
  d = tmp_path / "roaddb"
  d.mkdir(exist_ok=True)
  if anchors is not None:
    write_db(str(d), anchors)
  monkeypatch.setattr(cl, "DATA_DIR", str(d))
  monkeypatch.setattr(cl, "READ_PARAMS", [read or reader()])

  clock = [5000.0]
  ns = types.SimpleNamespace(monotonic=lambda: clock[0], time=lambda: clock[0])
  for mod in (C, m, cl):             # cl too: the waySel hold (terwilliger2pnw) runs on the decision clock
    monkeypatch.setattr(mod, "time", ns)
  C._ces_mode_hold_st.clear()

  class Mem:
    def __init__(self):
      self.puts = []

    def get(self, k, return_default=False):
      return {"MapTargetVelocities": points, "MapSpeedLimit": str(posted_mph * MPH), "MapHighwayClass": "motorway",
              "MapWaySel": way_sel(clock[0] - 5000.0) if callable(way_sel) else way_sel,
              "LastGPSPosition": json.dumps({"latitude": LAT0, "longitude": LON0, "bearing": 0.0, "src": "device",
                                             "ts": clock[0], "fix_ts": clock[0] - 0.3})}.get(k)

    def put_nonblocking(self, k, v):
      self.puts.append((k, copy.deepcopy(v)))

  params = _P(clock, mode=lambda t: 2, extra={"CESButtonState": "0"})
  c = m.CESController(FakeCP(fp, brand, op_long), params=params)
  c.mem_params = Mem()
  recs = []
  c._event_log_ok = True
  if log_file is not None:
    monkeypatch.setattr(m, "CES_EVENT_LOG", str(log_file))
  else:
    c._append_event = lambda rec: recs.append(copy.deepcopy(rec))
  stock = stock_mph * MPH
  for i in range(ticks):
    clock[0] = 5000.0 + (i + 1) * 0.01
    orz, vx, px, ts = _model(v_ego, 5000.0, 1e6)          # vision: a straight road
    model = NS(orientationRate=NS(z=orz, t=ts), velocity=NS(x=vx), position=NS(x=px),
               action=NS(shouldStop=False), meta=NS(laneChangeState="off"))
    sm = {"radarState": NS(leadOne=NS(status=False, vLead=0.0, dRel=0.0, aLeadK=0.0, vLeadK=0.0)),
          "modelV2": model, "carControl": NS(orientationNED=[0.0, 0.0, 0.0]),
          "livePose": NS(angularVelocityDevice=NS(x=0.0, y=0.0, z=0.0, valid=True)),
          "controlsState": NS(desiredCurvature=0.0,
                              lateralControlState=NS(which=lambda: "angleState", angleState=NS(saturated=False)))}
    cstate = NS(vEgo=v_ego, aEgo=0.0, gasPressed=False, brakePressed=False, leftBlinker=False, rightBlinker=False,
                vCruise=stock * 3.6, standstill=False, steeringAngleDeg=0.0, steeringPressed=False,
                leftBlindspot=False, rightBlindspot=False, cruiseState=NS(speed=stock, enabled=True),
                yawRate=0.0, steeringTorque=0.0)
    c.experimental_request(cstate, sm)
  icbm = [v for k, v in c.mem_params.puts if k == "IcbmTarget"]
  return icbm, recs, c


def _targets(icbm):
  return [p.get("target") for p in icbm]


RAISE_ROW = [_anchor(300.0, 0.0004)]              # the road there is nearly straight (v_db 74 m/s)
ADD_ROW = [_anchor(150.0, 0.004)]                 # a real curve mapd did not rate (v_db 23.45 m/s)
LOWER_ROW = [_anchor(300.0, 0.006)]               # sharper than mapd claims (v_db 19.15 m/s)


class TestController:
  def test_off_and_no_row_are_todays_icbm(self, monkeypatch, tmp_path):
    off, _, c0 = _drive(monkeypatch, tmp_path / "off", points=PHANTOM, anchors=None, switch=0)
    norow, recs, _ = _drive(monkeypatch, tmp_path / "norow", points=PHANTOM, anchors=[FAR_ANCHOR])
    assert not c0._roaddb.enabled
    assert any(t is not None for t in _targets(off)), "ICBM never slowed: the scenario proves nothing"
    assert norow == off
    assert {r["cdb2Why"] for r in recs if r["icbmT"] is not None} == {"noAnchor"}

  def test_a_phantom_is_raised_by_the_capped_amount(self, monkeypatch, tmp_path):
    off, _, _ = _drive(monkeypatch, tmp_path / "off", points=PHANTOM, anchors=None, switch=0)
    on, recs, _ = _drive(monkeypatch, tmp_path / "on", points=PHANTOM, anchors=RAISE_ROW)
    t_off = [t for t in _targets(off) if t is not None][-1]
    t_on = [t for t in _targets(on) if t is not None][-1]
    raised = [r for r in recs if r["cdb2Dir"] == "raise"]
    assert raised, {r["cdb2Why"] for r in recs}
    r = raised[-1]
    assert r["cdb2Tgt"] == pytest.approx(r["cdb2Base"] + 15 * MPH, abs=0.011)      # the +15 mph cap binds
    assert r["cdb2Row"] == "0:0" and r["cdb2K"] == 0.0004 and r["cdb2A"] == 2.2
    assert t_on == pytest.approx(t_off + 15 * MPH, abs=0.02)

  def test_a_missed_curve_is_added_at_exactly_v_db(self, monkeypatch, tmp_path):
    off, _, _ = _drive(monkeypatch, tmp_path / "off", points=STRAIGHT, anchors=None, switch=0)
    on, recs, _ = _drive(monkeypatch, tmp_path / "on", points=STRAIGHT, anchors=ADD_ROW)
    assert all(t is None for t in _targets(off))
    got = [t for t in _targets(on) if t is not None]
    assert got and got[-1] == pytest.approx(math.sqrt(A / 0.004), abs=0.01)    # no margin, no penalty on top
    add = [r for r in recs if r["cdb2Dir"] == "add"]
    assert add and add[-1]["cdb2Base"] is None and add[-1]["icbmSrc"] == "far"

  def test_a_sharper_road_lowers_to_exactly_v_db(self, monkeypatch, tmp_path):
    off, _, _ = _drive(monkeypatch, tmp_path / "off", points=MAPD_LOW, anchors=None, switch=0)
    on, recs, _ = _drive(monkeypatch, tmp_path / "on", points=MAPD_LOW, anchors=LOWER_ROW)
    t_off = [t for t in _targets(off) if t is not None][-1]
    t_on = [t for t in _targets(on) if t is not None][-1]
    assert t_on == pytest.approx(math.sqrt(A / 0.006), abs=0.01) and t_on < t_off - 1.0
    assert [r for r in recs if r["cdb2Dir"] == "lower"]

  @pytest.mark.parametrize("branches", [[(40.0, 450.0, 0.0004)],                       # unknown
                                        [(0.0, 450.0, 0.0004), (10.0, 450.0, 0.0004)],  # ambiguous
                                        [(0.0, 450.0, None), (20.0, 450.0, 0.0004)]])   # refused under the path
  def test_no_known_branch_is_no_effect_in_either_direction(self, monkeypatch, tmp_path, branches):
    for name, pts, y in (("raise", PHANTOM, 300.0), ("add", STRAIGHT, 150.0)):
      off, _, _ = _drive(monkeypatch, tmp_path / f"off{name}", points=pts, anchors=None, switch=0)
      brs = [(x, yy - 300.0 + y, k) for x, yy, k in branches]
      if name == "add":
        brs = [(x, yy, 0.004 if k else None) for x, yy, k in brs]
      on, _, _ = _drive(monkeypatch, tmp_path / f"on{name}", points=pts, anchors=[_anchor(y, None, brs)])
      assert on == off, name

  def test_the_kill_switch_is_todays_icbm(self, monkeypatch, tmp_path):
    base, _, _ = _drive(monkeypatch, tmp_path / "base", points=PHANTOM, anchors=[FAR_ANCHOR])
    killed, recs, c = _drive(monkeypatch, tmp_path / "killed", points=PHANTOM, anchors=RAISE_ROW, switch=0)
    assert not c._roaddb.enabled and killed == base
    assert {r["cdb2On"] for r in recs} == {"off"}

  def test_the_switch_on_explicitly(self, monkeypatch, tmp_path):
    _, _, c = _drive(monkeypatch, tmp_path / "on", points=PHANTOM, anchors=RAISE_ROW, switch=1, ticks=2)
    assert c._roaddb.enabled

  def test_a_missing_file_is_todays_icbm_and_says_so(self, monkeypatch, tmp_path, logs):
    base, _, _ = _drive(monkeypatch, tmp_path / "base", points=PHANTOM, anchors=[FAR_ANCHOR])
    missing, recs, _ = _drive(monkeypatch, tmp_path / "missing", points=PHANTOM, anchors=None)
    assert missing == base
    assert {r["cdb2On"] for r in recs} == {"err"} and all("missing" in r["cdb2Err"] for r in recs)
    assert any("LOAD FAILED" in e for e in logs.errors)

  def test_an_unreadable_A_is_todays_icbm_and_says_so(self, monkeypatch, tmp_path, logs):
    base, _, _ = _drive(monkeypatch, tmp_path / "base", points=PHANTOM, anchors=[FAR_ANCHOR])
    noa, recs, _ = _drive(monkeypatch, tmp_path / "noa", points=PHANTOM, anchors=RAISE_ROW, read=lambda: (None, 1))
    assert noa == base
    assert {r["cdb2Why"] for r in recs if r["icbmT"] is not None} == {"noA"}
    assert any("UNREADABLE" in e for e in logs.errors)

  def test_way_selection_not_current_is_no_effect(self, monkeypatch, tmp_path):
    base, _, _ = _drive(monkeypatch, tmp_path / "base", points=PHANTOM, anchors=[FAR_ANCHOR], way_sel="predicted")
    got, recs, _ = _drive(monkeypatch, tmp_path / "ws", points=PHANTOM, anchors=RAISE_ROW, way_sel="predicted")
    assert got == base and {r["cdb2Why"] for r in recs if r["icbmT"] is not None} == {"waySel"}

  def test_a_short_way_selection_flicker_is_ridden_through(self, monkeypatch, tmp_path, logs):
    """terwilliger2pnw: 2026-09-24 12:35:35 PT, one ~1 s waySel != current tick mid-curve switched the DB off; the target
    fell to mapd's number and the running cap walked the set down to it. A flicker shorter than WAYSEL_HOLD_S must
    leave the DB's decision exactly as if waySel had stayed current -- and say it rode through (cdb2WayHold)."""
    def flicker(t):
      return "predicted" if 1.5 <= t < 2.6 else "current"      # 1.1 s, spanning the 1 Hz record at t = 2.0
    steady, _, _ = _drive(monkeypatch, tmp_path / "steady", points=PHANTOM, anchors=RAISE_ROW)
    got, recs, _ = _drive(monkeypatch, tmp_path / "flick", points=PHANTOM, anchors=RAISE_ROW, way_sel=flicker)
    no_db, _, _ = _drive(monkeypatch, tmp_path / "nodb", points=PHANTOM, anchors=[FAR_ANCHOR])
    assert _targets(steady) != _targets(no_db), "the DB must change the target here, or this proves nothing"
    assert _targets(got) == _targets(steady)
    during = [r for r in recs if 1.9 <= r["t"] - 5000.0 <= 2.1 and r["icbmT"] is not None]
    assert during and all(r["cdb2WayHold"] is True and r["cdb2Why"] != "waySel" for r in during), \
      [(r["t"], r["cdb2Why"], r["cdb2WayHold"]) for r in during]
    assert not [r for r in recs if r["cdb2Why"] == "waySel"]
    assert all(r["cdb2WayHold"] is False for r in recs if r["icbmT"] is not None and not 1.9 <= r["t"] - 5000.0 <= 2.1)
    starts = [k for n, k in logs.events if n == "curvedb_v2_waysel_hold"]
    assert len(starts) == 1 and starts[0]["way_sel"] == "predicted", logs.events   # change-only, not per tick

  def test_a_sustained_way_change_still_gates_within_the_hold(self, monkeypatch, tmp_path, logs):
    """A real exit onto a ramp or another road (waySel stays non-current) must switch the DB off no later than
    WAYSEL_HOLD_S after the last current decision -- exactly as without the hold from then on."""
    t_off = 1.5
    got, recs, _ = _drive(monkeypatch, tmp_path / "exit", points=PHANTOM, anchors=RAISE_ROW,
                          way_sel=lambda t: "current" if t < t_off else "possible", ticks=800)
    live = [r for r in recs if r["icbmT"] is not None]
    gated = [r["t"] - 5000.0 for r in live if r["cdb2Why"] == "waySel"]
    assert gated, {r["cdb2Why"] for r in live}
    # the first gated record: within the hold + one ICBM decision period + the ~1 Hz record cadence
    assert gated[0] <= t_off + cl.WAYSEL_HOLD_S + 1.3, gated[0]
    assert all(r["cdb2Why"] == "waySel" and r["cdb2WayHold"] is False
               for r in live if r["t"] - 5000.0 > t_off + cl.WAYSEL_HOLD_S + 1.3)
    assert [n for n, _k in logs.events if n == "curvedb_v2_waysel_hold_end"], "the end of the ride-through is not logged"

  def test_the_hold_boundary(self):
    db = cl.CurveDbLive(False)
    assert db.way_sel_held("current", 100.0) == ("current", False)
    assert db.way_sel_held("predicted", 101.0) == ("current", True)
    assert db.way_sel_held(None, 102.0) == ("current", True)             # exactly WAYSEL_HOLD_S: still held
    assert db.way_sel_held("predicted", 102.01) == ("predicted", False)
    assert db.way_sel_held("predicted", 103.0) == ("predicted", False)
    fresh = cl.CurveDbLive(False)                                         # never saw current: no hold at all
    assert fresh.way_sel_held("predicted", 5.0) == ("predicted", False)

  def test_the_tesla_is_unaffected(self, monkeypatch, tmp_path):
    assert not pv.PnwVehicle(FakeCP(TESLA, "tesla", True)).curvedb_v2_live
    off, _, _ = _drive(monkeypatch, tmp_path / "off", points=PHANTOM, anchors=None, fp=TESLA, brand="tesla",
                       op_long=True, switch=1)
    on, recs, c = _drive(monkeypatch, tmp_path / "on", points=PHANTOM, anchors=RAISE_ROW, fp=TESLA, brand="tesla",
                         op_long=True, switch=1)
    assert not c._roaddb.enabled and on == off
    assert {r["cdb2On"] for r in recs} == {"off"} and not any(r["cdb2Dir"] for r in recs)

  def test_the_cdb2_fields_reach_the_ces_events_line_on_disk(self, monkeypatch, tmp_path):
    """Not the dict: the JSON line _append_event writes. New fields have evaporated before (cherry-picked keys)."""
    log = tmp_path / "ces_events.jsonl"
    _drive(monkeypatch, tmp_path / "rec", points=STRAIGHT, anchors=ADD_ROW, log_file=log)
    lines = [json.loads(ln) for ln in log.read_text().splitlines()]
    assert lines, "nothing was written"
    for rec in lines:
      assert set(cl.TELE_KEYS) <= set(rec), set(cl.TELE_KEYS) - set(rec)
    add = [r for r in lines if r["cdb2Dir"] == "add"]
    assert add, {r["cdb2Why"] for r in lines}
    r = add[-1]
    assert r["cdb2On"] == "ok" and r["cdb2Rows"] == 1 and r["cdb2A"] == 2.2 and r["cdb2ASrc"] == "mapd:standard"
    assert r["cdb2Row"] == "0:0" and r["cdb2K"] == 0.004 and r["cdb2VDb"] == pytest.approx(23.45, abs=0.01)
    assert r["cdb2Base"] is None and r["cdb2Tgt"] == pytest.approx(23.45, abs=0.01) and r["cdb2Why"] == "ok"
    assert r["cdb2Lat"] is not None and r["cdb2D"] is not None and r["cdb2N"] > 0 and r["cdb2NL"] > 0

  def test_a_crashing_db_costs_the_db_not_icbm(self, monkeypatch, tmp_path):
    base, _, _ = _drive(monkeypatch, tmp_path / "base", points=PHANTOM, anchors=[FAR_ANCHOR])

    def boom(self, **kw):
      raise RuntimeError("db on fire")
    monkeypatch.setattr(cl.CurveDbLive, "_decide", boom)
    got, recs, _ = _drive(monkeypatch, tmp_path / "boom", points=PHANTOM, anchors=RAISE_ROW)
    assert got == base
    # Fable F1: the telemetry must say the DB crashed, never replay an earlier decision as if live
    whys = {r.get("cdb2Why") for r in recs}
    assert "crash" in whys and not whys & {"ok", "notMin"}, whys


# =====================================================================================================
# hidden curves: a raised phantom must not hide the next mapd-rated curve
# =====================================================================================================
def test_a_raised_phantom_does_not_hide_the_next_rated_curve(monkeypatch, tmp_path):
    """mapd rates a phantom at 220 m (45 mph) and a real curve at 380 m (34 mph), outside the phantom row's measured
    stretch [195, 370] m. ICBM keeps ONE far candidate, the most binding -- the phantom -- and the DB raises it. The
    real curve, which has no row of its own, must still bind at exactly what ICBM gives it alone."""
    both = _path({220.0: 20.0, 380.0: 15.0})
    real_only, _, _ = _drive(monkeypatch, tmp_path / "real", points=_path({380.0: 15.0}), anchors=None, switch=0)
    both_off, _, _ = _drive(monkeypatch, tmp_path / "off", points=both, anchors=None, switch=0)
    on, recs, _ = _drive(monkeypatch, tmp_path / "on", points=both, anchors=[_anchor(220.0, 0.0004)])
    t_real = [t for t in _targets(real_only) if t is not None]
    t_off = [t for t in _targets(both_off) if t is not None]
    t_on = [t for t in _targets(on) if t is not None]
    assert t_real and t_off and t_on
    # without the DB ICBM names the phantom (the lowest decel cap) and publishes ITS number, not the real curve's
    assert abs(t_off[-1] - t_real[-1]) > 0.5, "the phantom must be what binds without the DB, or this proves nothing"
    assert t_on[-1] == pytest.approx(t_real[-1], abs=0.02), "the second curve was hidden behind the raised phantom"
    # the phantom's 20.0 was replaced; what binds now is the real curve, below it -- logged as a lowering
    assert [r for r in recs if r["cdb2Base"] == pytest.approx(t_off[-1], abs=0.01) and r["cdb2Dir"] == "lower"
            and r["cdb2Tgt"] == pytest.approx(t_real[-1], abs=0.01)]


def test_a_row_found_by_the_scan_raises_to_its_row_speed_and_binds(monkeypatch, tmp_path):
    """The phantom's row lets ICBM's 20 m/s go up to its cap (26.7), but another row 120 m further on measures a
    curve whose v_db is 25.0. That second row is ABOVE ICBM's target, so it is a raise too, through the same rule:
    its row speed 25.0 (no margin since terwilliger2pnw), and the minimum wins over the phantom's 26.7."""
    k2 = A / 25.0 ** 2
    on, recs, _ = _drive(monkeypatch, tmp_path / "on", points=_path({220.0: 20.0}),
                         anchors=[_anchor(220.0, 0.0004), _anchor(340.0, k2)])
    got = [t for t in _targets(on) if t is not None]
    assert got and got[-1] == pytest.approx(25.0, abs=0.01)


def test_a_curve_json_A_sets_the_speed_and_is_logged_as_its_source(monkeypatch, tmp_path):
    cfg_dir = tmp_path / "ov"
    cfg_dir.mkdir()
    (cfg_dir / "curve.json").write_text(json.dumps({"lightning": {"curvedb_v2_lat_a": 2.5}}))
    monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(cfg_dir / "curve.json"))
    on, recs, c = _drive(monkeypatch, cfg_dir, points=STRAIGHT, anchors=ADD_ROW, read=lambda: (None, 1))
    got = [t for t in _targets(on) if t is not None]
    assert got and got[-1] == pytest.approx(math.sqrt(2.5 / 0.004), abs=0.01)
    assert {(r["cdb2A"], r["cdb2ASrc"]) for r in recs} == {(2.5, "curve.json")}


def test_the_shipped_default_is_2_5():
  """Owner decision 2026-09-24 11:25 PT: the Lightning's curve DB turns curvature into speed at a fixed 2.5 m/s^2."""
  from openpilot.selfdrive.controls.lib.ces_pnw.tests.roaddb_fixture import SHIPPED_LAT_A as shipped
  assert shipped == 2.5
