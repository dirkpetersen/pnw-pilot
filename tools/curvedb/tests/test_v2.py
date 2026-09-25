"""curvedb v2: the pure functions (roadtable, v2_replay's target/verdict, v2_validate's pairing)."""
import gzip
import json
import math
import os
from collections import Counter
from dataclasses import replace

import pytest

from openpilot.tools.curvedb import roadtable as rt
from openpilot.tools.curvedb import v2_build
from openpilot.tools.curvedb.v2_replay import MPH, classify, outcome, row_accuracy, v2_target
from openpilot.tools.curvedb.v2_validate import interp, ratio_stats, window_peak

P = rt.PROVISIONAL_V2
LIGHTNING, TESLA = "FORD_F_150_LIGHTNING_MK1", "TESLA_MODEL_S_HW3"


# ---------------------------------------------------------------------------------------------
# numeric helpers
# ---------------------------------------------------------------------------------------------

def test_percentile_matches_linear_interpolation():
  assert rt.percentile([1, 2, 3, 4], 50) == 2.5
  assert rt.percentile([5], 90) == 5.0
  assert rt.percentile([0, 10], 25) == 2.5
  with pytest.raises(ValueError):
    rt.percentile([], 50)


def test_curvature_refuses_low_speed_and_non_finite():
  assert rt.curvature(0.1, 20.0, 8.0) == pytest.approx(0.005)
  assert rt.curvature(0.1, 7.9, 8.0) is None
  assert rt.curvature(float("nan"), 20.0, 8.0) is None
  assert rt.curvature(None, 20.0, 8.0) is None


def test_smooth_signed_cancels_wiggle_but_keeps_a_bend():
  ts = [i * 0.2 for i in range(50)]
  wiggle = [0.002 * (-1) ** i for i in range(50)]
  sm = rt.smooth_signed(ts, wiggle, 1.0)
  assert max(abs(x) for x in sm[5:-5]) < 0.0005          # alternating sign averages out
  bend = [0.003] * 50
  assert all(x == pytest.approx(0.003) for x in rt.smooth_signed(ts, bend, 1.0))


def test_smooth_signed_skips_none_and_keeps_alignment():
  ts = [0.0, 0.2, 0.4, 0.6]
  out = rt.smooth_signed(ts, [0.001, None, 0.003, 0.002], 1.0)
  assert out[1] is None and out[0] is not None and len(out) == 4


# ---------------------------------------------------------------------------------------------
# fusion + resampling on a synthetic drive
# ---------------------------------------------------------------------------------------------

def _synthetic_doc(k=0.002, v=25.0, n_s=40, lat0=45.0, lon0=-122.7, bacc=2.0):
  """A straight-then-constant-curvature drive heading north, streams in v2_extract's format."""
  pose, cs, cc, gps, mapd = [], [], [], [], []
  lat, lon, hdg = lat0, lon0, 0.0
  for i in range(n_s * 5 + 1):
    t = i * 0.2
    yaw = v * k
    pose.append([t, -yaw])                      # device z is the opposite sign of vehicle yaw
    cs.append([t, v, 0, 0, 0, 0, yaw])
    cc.append([t, 1])
    if i % 5 == 0:
      gps.append([t, 1, lat, lon, math.degrees(hdg) % 360, bacc, v])
      mapd.append([t, 42, "motorway", 26.8, 0.0, "Some Name", "I 5"])
    hdg += yaw * 0.2
    lat += v * 0.2 * math.cos(hdg) / 111194.0
    lon += v * 0.2 * math.sin(hdg) / (111194.0 * math.cos(math.radians(lat)))
  return {"pose": pose, "cs": cs, "cc": cc, "gps": gps, "mapd": mapd}


def test_fuse_recovers_curvature_sign_and_position():
  s = rt.fuse(_synthetic_doc(), 1000.0, P)
  mid = s[len(s) // 2]
  assert mid.k == pytest.approx(0.002, rel=1e-6)          # sign flipped back to vehicle convention
  assert mid.t == pytest.approx(1000.0 + 20.0, abs=0.3)
  assert mid.lat is not None and mid.fix_ok and mid.way == 42 and mid.road == "I 5|Some Name"


def test_fuse_marks_bad_bearing_accuracy():
  s = rt.fuse(_synthetic_doc(bacc=15.0), 0.0, P)
  assert not any(x.fix_ok for x in s if x.lat is not None)


def test_resample_spacing_and_extent_peak():
  s = rt.fuse(_synthetic_doc(), 0.0, P)
  passes = rt.resample(s, P)
  assert len(passes) == 1
  pts = passes[0]
  gaps = [b.s - a.s for a, b in zip(pts, pts[1:], strict=False)]
  assert all(20.0 <= g <= 30.0 for g in gaps)        # step +- one tick of travel (5 m)
  k, why = rt.extent_peak(pts, len(pts) // 2, P.extent_back_m, P.extent_fwd_m)
  assert why == "ok" and k == pytest.approx(0.002, rel=1e-3)


def test_extent_peak_refuses_partial_extent_and_bad_gps():
  pts = rt.resample(rt.fuse(_synthetic_doc(), 0.0, P), P)[0]
  assert rt.extent_peak(pts, 0, 25, 150)[0] is None                       # starts before the pass
  assert rt.extent_peak(pts, len(pts) - 2, 25, 150)[0] is None            # runs past the end
  pts[len(pts) // 2 + 2].fix_ok = False
  assert rt.extent_peak(pts, len(pts) // 2, 25, 150) == (None, "gps")


# ---------------------------------------------------------------------------------------------
# the build scope: class + speed, no geography (owner 2026-09-24)
# ---------------------------------------------------------------------------------------------

def _pt(hwy, v):
  return rt.Point(s=0.0, t=0.0, lat=45.0, lon=-122.7, brg=0.0, v=v, k=0.001, fix_ok=True, drv=False,
                  lat_active=True, way=1, hwy=hwy, road="", spl=0.0, mcs=0.0)


def test_scope_is_class_and_speed_not_geography():
  fast, slow = 40 * MPH + 0.01, 40 * MPH - 0.01
  for hwy in ("motorway", "trunk", "primary"):
    assert v2_build.in_scope(_pt(hwy, fast))
    assert not v2_build.in_scope(_pt(hwy, slow))                  # a 39 mph crawl does not seed anchors
  for hwy in ("motorwayLink", "trunkLink", "primaryLink", "secondary", "tertiary", "residential", "",
              "unknown", "None"):
    assert not v2_build.in_scope(_pt(hwy, 30.0))                   # ramps / unknown / minor roads never
  assert not hasattr(v2_build, "SEATTLE_BOX") and not hasattr(v2_build, "region")


def test_seed_anchors_creates_what_add_pass_creates_without_observations():
  pts = rt.resample(rt.fuse(_synthetic_doc(), 0.0, P), P)[0]
  a, b = rt.AnchorIndex(P), rt.AnchorIndex(P)
  n = rt.seed_anchors(a, pts, in_scope=lambda p: True)
  rt.add_pass(b, pts, date="2026-09-01", drive="r", car=LIGHTNING, in_scope=lambda p: True, tally=Counter())
  assert n == len(a.anchors) == len(b.anchors) > 0
  assert [(x.lat, x.lon, x.brg) for x in a.anchors] == [(x.lat, x.lon, x.brg) for x in b.anchors]
  assert all(not x.obs for x in a.anchors)
  assert rt.seed_anchors(a, pts, in_scope=lambda p: True) == 0      # the same road reuses its anchors
  assert rt.seed_anchors(rt.AnchorIndex(P), pts, in_scope=lambda p: False) == 0


def _write_route(d, route, doc, off_s, fp=LIGHTNING):
  with gzip.open(os.path.join(d, f"{route}--0.qlog.json.gz"), "wt") as f:
    json.dump(dict(doc, off_s=off_s, fp=fp), f)


def test_build_attaches_an_unclassified_earlier_pass_to_anchors_a_later_pass_creates(tmp_path):
  """MEASURED: no point before 2026-08-15 has a highwayClass. The early pass cannot CREATE anchors under a
  class scope, but it must still count as a date once a classified pass has created them."""
  early = _synthetic_doc()
  early["mapd"] = [[r[0], r[1], "", *r[3:]] for r in early["mapd"]]
  _write_route(tmp_path, "00000001--aaaa", early, 1_788_000_000.0)             # an earlier PT date, no class
  _write_route(tmp_path, "00000002--bbbb", _synthetic_doc(), 1_789_000_000.0)  # later: motorway, 56 mph
  idx = v2_build.build(str(tmp_path), P, Counter())
  assert idx.anchors
  assert len({o.date for a in idx.anchors for o in a.obs}) == 2               # the early date is kept
  verdicts = [rt.row_verdict(a, P, branch=br) for a in idx.anchors for br in rt.branches(a, P.branch_radius_m)]
  assert any(v.granted and v.n_dates == 2 for v in verdicts)


def test_build_creates_no_anchor_on_a_minor_or_slow_road(tmp_path):
  _write_route(tmp_path, "00000001--aaaa", _synthetic_doc(v=15.0), 1_788_000_000.0)   # motorway at 34 mph
  minor = _synthetic_doc()
  minor["mapd"] = [[r[0], r[1], "secondary", *r[3:]] for r in minor["mapd"]]
  _write_route(tmp_path, "00000002--bbbb", minor, 1_789_000_000.0)
  assert not v2_build.build(str(tmp_path), P, Counter()).anchors


# ---------------------------------------------------------------------------------------------
# anchors and the row verdict (the owner's rules)
# ---------------------------------------------------------------------------------------------

def _obs(date, k, car=LIGHTNING, way=42, hwy="motorway", drive=None, end=(45.00135, -122.7)):
  return rt.PassObs(date=date, drive=drive or f"r{date}", car=car, t=0.0, k_ext=k, k_loc=k, way=way, hwy=hwy,
                    road="I 5", spl=26.8, mode="op", d_m=0.0, end_lat=end[0], end_lon=end[1])


def _anchor(*obs):
  a = rt.Anchor(45.0, -122.7, 0.0)
  a.obs = list(obs)
  return a


def test_row_needs_two_dates_never_one_pass():
  v = rt.row_verdict(_anchor(_obs("2026-09-01", 0.002), _obs("2026-09-01", 0.002, drive="x")), P)
  assert not v.granted and "date" in v.reason
  assert rt.row_verdict(_anchor(_obs("2026-09-01", 0.002), _obs("2026-09-02", 0.002)), P).granted


def test_row_value_is_the_middle_percentile_across_dates_not_a_single_pass():
  a = _anchor(_obs("2026-09-01", 0.0020), _obs("2026-09-02", 0.0022), _obs("2026-09-03", 0.0024))
  v = rt.row_verdict(a, P)
  assert v.granted and v.k == pytest.approx(0.0022)
  # one date with many passes does not outvote the others: per-date median first
  a.obs += [_obs("2026-09-03", 0.0024, drive=f"d{i}") for i in range(10)]
  assert rt.row_verdict(a, P).k == pytest.approx(0.0022)


def test_tesla_is_never_sole_evidence_but_may_contribute():
  only_t = _anchor(_obs("2026-08-01", 0.002, car=TESLA), _obs("2026-08-02", 0.002, car=TESLA))
  assert rt.row_verdict(only_t, P).reason == "tesla-only evidence"
  unknown = _anchor(_obs("2026-08-01", 0.002, car=TESLA), _obs("2026-08-02", 0.002, car="unknown"))
  assert rt.row_verdict(unknown, P).reason == "tesla-only evidence"
  mixed = _anchor(_obs("2026-08-01", 0.002, car=TESLA), _obs("2026-09-02", 0.002))
  assert rt.row_verdict(mixed, P).granted


@pytest.mark.parametrize("hwy,why", [("motorwayLink", "ramp"), ("unknown", "unknown"), ("", "unknown")])
def test_r8_ramps_and_unknown_class_are_refused(hwy, why):
  v = rt.row_verdict(_anchor(_obs("2026-09-01", 0.002, hwy=hwy), _obs("2026-09-02", 0.002, hwy=hwy)), P)
  assert not v.granted and why in v.reason


def test_dates_that_disagree_are_refused_but_straight_road_noise_is_not():
  v = rt.row_verdict(_anchor(_obs("2026-09-01", 0.001), _obs("2026-09-02", 0.003)), P)
  assert not v.granted and "disagree" in v.reason
  # 0.0003 vs 0.0006 is a 2x ratio but 3e-4 absolute: straight-road noise, not two different roads
  assert rt.row_verdict(_anchor(_obs("2026-09-01", 0.0003), _obs("2026-09-02", 0.0006)), P).granted
  assert not rt.row_verdict(_anchor(_obs("2026-09-01", 0.0003), _obs("2026-09-02", 0.0009)), P).granted


def test_lodo_excludes_the_held_out_date():
  a = _anchor(_obs("2026-09-01", 0.002), _obs("2026-09-02", 0.002), _obs("2026-09-03", 0.004))
  assert rt.row_verdict(a, P, exclude_date="2026-09-01").k == pytest.approx(0.003)
  assert not rt.row_verdict(_anchor(_obs("2026-09-01", 0.002), _obs("2026-09-02", 0.002)), P,
                            exclude_date="2026-09-02").granted


def test_way_check_drops_the_parallel_road():
  a = _anchor(_obs("2026-09-01", 0.001, way=1), _obs("2026-09-02", 0.001, way=1), _obs("2026-09-03", 0.009, way=2))
  assert rt.row_verdict(a, replace(P, way_check=True)).k == pytest.approx(0.001)
  assert rt.row_verdict(a, P).granted is False            # way_check is OFF by default: the spread check refuses


def test_anchor_index_respects_radius_and_heading():
  idx = rt.AnchorIndex(P)
  idx.add(45.0, -122.7, 0.0)
  assert idx.nearest(45.0002, -122.7, 10.0)[0] == 0              # ~22 m, 10 deg
  assert idx.nearest(45.0002, -122.7, 180.0)[0] is None          # opposite carriageway
  assert idx.nearest(45.001, -122.7, 0.0)[0] is None             # ~111 m
  assert idx.nearest(45.0, -122.7, float("nan"))[0] is None


def test_add_pass_one_obs_per_pass_and_scope_only_creates_in_scope_anchors():
  pts = rt.resample(rt.fuse(_synthetic_doc(), 0.0, P), P)[0]
  idx = rt.AnchorIndex(P)
  t = Counter()
  rt.add_pass(idx, pts, date="2026-09-01", drive="r1", car=LIGHTNING, in_scope=lambda p: False, tally=t)
  assert not idx.anchors
  rt.add_pass(idx, pts, date="2026-09-01", drive="r1", car=LIGHTNING, in_scope=lambda p: True, tally=t)
  assert idx.anchors and all(len(a.obs) <= 1 for a in idx.anchors)
  n = len(idx.anchors)
  rt.add_pass(idx, pts, date="2026-09-02", drive="r2", car=LIGHTNING, in_scope=lambda p: True, tally=t)
  assert len(idx.anchors) == n                                    # the same road reuses its anchors
  assert t["pass admitted"] > 0 and any(k.startswith("pass refused") for k in t)


# ---------------------------------------------------------------------------------------------
# the replay's target and verdict
# ---------------------------------------------------------------------------------------------

def test_v2_target_uses_mapd_lateral_target_with_measured_curvature():
  # k = 0.002, A = 2.2 -> 33.2 m/s; bounded by ref
  assert v2_target(k_row=0.002, a_mapd=2.2, icbm_ms=20.0, ref_ms=40.0, posted_ms=None,
                   raise_cap_ms=1e9) == pytest.approx(math.sqrt(1100))
  assert v2_target(k_row=0.002, a_mapd=2.2, icbm_ms=20.0, ref_ms=30.0, posted_ms=None, raise_cap_ms=1e9) == 30.0


def test_v2_target_raise_is_bounded_lowering_is_free():
  up = v2_target(k_row=0.0005, a_mapd=2.2, icbm_ms=20.0, ref_ms=40.0, posted_ms=None)
  assert up == pytest.approx(20.0 + 15 * MPH)                               # +15 mph cap
  up2 = v2_target(k_row=0.0005, a_mapd=2.2, icbm_ms=20.0, ref_ms=40.0, posted_ms=21.0)
  assert up2 == pytest.approx(21.0 + 10 * MPH)                              # posted + 10 mph binds first
  assert v2_target(k_row=0.01, a_mapd=2.2, icbm_ms=20.0, ref_ms=40.0, posted_ms=24.0) == pytest.approx(math.sqrt(220))
  # a posted cap below ICBM's own target never pushes BELOW ICBM on a raise
  assert v2_target(k_row=0.0005, a_mapd=2.2, icbm_ms=20.0, ref_ms=40.0, posted_ms=10.0) == 20.0
  with pytest.raises(ValueError):
    v2_target(k_row=0.0, a_mapd=2.2, icbm_ms=20.0, ref_ms=40.0, posted_ms=None)


def test_classify_uses_the_owner_cutoffs():
  assert classify(2.2) == "unwanted" and classify(2.5) == "marginal" and classify(2.8) == "wanted"


def test_outcome_flags_a_lost_real_slowdown():
  assert outcome("wanted", ref=33.0, icbm=20.0, target=33.0, a_target=3.5) == "LOST"
  assert outcome("wanted", ref=33.0, icbm=20.0, target=25.0, a_target=2.6) == "WEAKENED"
  assert outcome("wanted", ref=33.0, icbm=20.0, target=22.0, a_target=2.1) == "raised, still < 2.5 at target"
  assert outcome("wanted", ref=33.0, icbm=20.0, target=20.2, a_target=2.1) == "kept"
  assert outcome("wanted", ref=33.0, icbm=20.0, target=18.0, a_target=2.1) == "kept (stronger)"


def test_outcome_for_unwanted():
  assert outcome("unwanted", ref=33.0, icbm=22.0, target=32.5, a_target=1.0) == "REMOVED"
  assert outcome("unwanted", ref=33.0, icbm=22.0, target=27.0, a_target=1.0) == "reduced"
  assert outcome("unwanted", ref=33.0, icbm=22.0, target=20.0, a_target=1.0) == "WORSENED"
  assert outcome("unwanted", ref=33.0, icbm=22.0, target=22.2, a_target=1.0) == "unchanged"


# ---------------------------------------------------------------------------------------------
# validation pairing
# ---------------------------------------------------------------------------------------------

def test_window_peak_and_interp():
  ts, vs = [0.0, 0.5, 1.0, 1.5], [0.001, -0.004, None, 0.002]
  assert window_peak(ts, vs, 0.0, 1.5) == 0.004              # (t0, t1]: excludes t=0
  assert window_peak(ts, vs, 2.0, 3.0) is None
  assert interp([0.0, 1.0], [0.0, 2.0], 0.25) == 0.5
  assert interp([0.0, 1.0], [0.0, None], 0.5) is None


def test_ratio_stats_reports_bias_and_low_share():
  s = ratio_stats([1.0, 1.0, 1.0, 1.0], [1.0, 0.9, 0.7, 1.1])
  assert s["n"] == 4 and s["bias"] == pytest.approx(0.95) and s["low20"] == 0.25
  assert ratio_stats([], []) == {"n": 0}


def test_row_accuracy_is_leave_one_date_out_and_flags_the_tight_pass():
  idx = rt.AnchorIndex(P)
  idx.add(45.0, -122.7, 0.0)
  idx.anchors[0].obs = [_obs("2026-09-01", 0.002), _obs("2026-09-02", 0.002), _obs("2026-09-03", 0.0029)]
  acc, tally = row_accuracy(idx, k_min=0.001, a_mapd=2.2)
  by = {x["date"]: x for x in acc}
  assert set(by) == {"2026-09-01", "2026-09-02", "2026-09-03"}
  assert by["2026-09-03"]["k_row"] == pytest.approx(0.002)            # its own date is excluded
  assert by["2026-09-03"]["a_at_vdb"] == pytest.approx(2.2 * 0.0029 / 0.002)
  assert by["2026-09-03"]["a_at_vdb"] >= 2.5 and by["2026-09-01"]["a_at_vdb"] < 2.5
  acc2, tally2 = row_accuracy(idx, k_min=0.01, a_mapd=2.2)
  assert not acc2 and tally2["row k below k_min"] == 3


RAMP_END = (45.00135, -122.6995)          # ~40 m east of the mainline end point: the other branch


def test_branch_keeps_a_split_from_pooling_the_ramp_with_the_mainline():
  a = _anchor(_obs("2026-09-01", 0.0010), _obs("2026-09-02", 0.0010), _obs("2026-09-03", 0.0060, end=RAMP_END),
              _obs("2026-09-04", 0.0062, end=RAMP_END))
  assert len(rt.branches(a, P.branch_radius_m)) == 2
  main = rt.row_verdict(a, P, branch=(45.00135, -122.7))
  ramp = rt.row_verdict(a, P, branch=RAMP_END)
  assert main.granted and main.k == pytest.approx(0.0010)
  assert ramp.granted and ramp.k == pytest.approx(0.0061)
  # unbranched, the split cannot hide: the dates disagree and the row is refused
  assert not rt.row_verdict(a, P).granted
  # a query on a branch nobody else took has no row
  assert rt.row_verdict(a, P, branch=(45.002, -122.698)).reason == "no passes on this branch"


def test_extent_end_is_the_pass_position_150m_on():
  pts = rt.resample(rt.fuse(_synthetic_doc(), 0.0, P), P)[0]
  i = 4
  end = rt.extent_end(pts, i, 150.0)
  from openpilot.tools.curvedb.store import haversine_m
  assert 147.0 <= haversine_m(pts[i].lat, pts[i].lon, *end) <= 153.0
  assert rt.extent_end(pts, len(pts) - 2, 150.0) is None


def _line(n, step=10.0, lat0=45.0, lon0=-122.7):
  """n Points due north every `step` m, constant curvature 0.002, clean GPS."""
  return [rt.Point(s=i * step, t=float(i), lat=lat0 + i * step / 111194.0, lon=lon0, brg=0.0, v=25.0, k=0.002,
                   fix_ok=True, drv=False, lat_active=True, way=1, hwy="motorway", road="I 5|I 5", spl=26.8,
                   mcs=0.0) for i in range(n)]


def test_add_pass_survives_a_projected_end_past_the_pass():
  """Regression: an anchor AHEAD of the pass's nearest point pushes the branch end past the pass end; that
  used to raise TypeError. It must be admitted without a branch point, and counted."""
  pts = _line(20)                                  # s = 0 .. 190 m
  idx = rt.AnchorIndex(P)
  ai = idx.add(45.0 + 34.0 / 111194.0, -122.7, 0.0)  # nearest point s=30 (4 m behind); 34 + 150 = 184 ...
  pts = pts[:19]                                   # ... and the pass ends at s=180
  t = Counter()
  rt.add_pass(idx, pts, date="2026-09-01", drive="r", car=LIGHTNING, in_scope=lambda p: False, tally=t)
  assert t["pass admitted without a branch point (projected end past the pass)"] == 1
  assert len(idx.anchors[ai].obs) == 1 and not math.isfinite(idx.anchors[ai].obs[0].end_lat)


def test_branch_point_is_measured_from_the_anchor_not_the_nearest_point():
  pts = rt.resample(rt.fuse(_synthetic_doc(k=0.0), 0.0, P), P)[0]
  i = 6
  q = pts[i]
  ahead = rt.Anchor(q.lat + 20 / 111194.0, q.lon, 0.0)          # 20 m further along (heading north)
  from openpilot.tools.curvedb.store import haversine_m
  bp = rt.branch_point(pts, i, ahead, 150.0)
  assert haversine_m(ahead.lat, ahead.lon, *bp) == pytest.approx(150.0, abs=2.0)


def test_raise_margin_applies_to_raises_only():
  # a raise: k=0.001, A=2.2 -> 46.9 m/s unmargined; 1.25x -> 41.95; capped at icbm + 15 mph = 26.7
  assert v2_target(k_row=0.001, a_mapd=2.2, icbm_ms=30.0, ref_ms=60.0, posted_ms=None, raise_cap_ms=1e9,
                   raise_margin=1.25) == pytest.approx(math.sqrt(2.2 / 0.00125))
  # a lowering is untouched by the margin
  assert v2_target(k_row=0.01, a_mapd=2.2, icbm_ms=30.0, ref_ms=60.0, posted_ms=None,
                   raise_margin=1.25) == pytest.approx(math.sqrt(220))
  # a margin never pushes a raise below ICBM's own target
  assert v2_target(k_row=0.0024, a_mapd=2.2, icbm_ms=30.0, ref_ms=60.0, posted_ms=None, raise_margin=1.25) == 30.0
  with pytest.raises(ValueError):
    v2_target(k_row=0.001, a_mapd=2.2, icbm_ms=30.0, ref_ms=60.0, posted_ms=None, raise_margin=0.9)


def test_r8_applies_over_the_extent_not_just_the_anchor():
  """Regression: a mainline anchor whose extent runs into the exit ramp must not have authority."""
  exitb = [replace(_obs("2026-09-01", 0.004), ext_ramp=True), replace(_obs("2026-09-02", 0.004), ext_ramp=True)]
  v = rt.row_verdict(_anchor(*exitb), P)
  assert not v.granted and v.reason == "extent reaches a ramp"
  # a pre-mapd-v2.2 pass (no class anywhere) is not class evidence, but does not refuse a row either
  old = [replace(_obs("2026-08-01", 0.004, hwy="unknown"), ext_known=False), _obs("2026-09-02", 0.004)]
  assert rt.row_verdict(_anchor(*old), P).granted
  only_old = [replace(_obs(d, 0.004), ext_known=False) for d in ("2026-08-01", "2026-08-02")]
  assert rt.row_verdict(_anchor(*only_old), P).reason == "no pass with a fully classified extent"


def test_extent_classes_sees_the_ramp_ahead():
  pts = _line(30)
  for q in pts[20:]:
    q.hwy = "motorwayLink"
  assert rt.extent_classes(pts, 10, 25, 150) == (True, True)      # s=100..250 reaches s=200 (ramp)
  assert rt.extent_classes(pts, 2, 25, 150) == (False, True)      # s=0..170 stays on the mainline
  pts[5].hwy = "unknown"
  assert rt.extent_classes(pts, 2, 25, 150) == (False, False)
