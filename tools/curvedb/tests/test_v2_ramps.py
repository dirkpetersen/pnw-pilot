"""curvedbramps2pnw (owner 2026-10-19, "ramp option A"): with V2Params.allow_ramps a ramp/link branch is treated like any other road;
EVERY other refusal stays, and a build without the flag is unchanged. Synthetic geometry only (the real rows are private)."""
import gzip
import math
import json
from collections import Counter
from dataclasses import asdict, replace

import pytest

from openpilot.selfdrive.controls.lib.ces_pnw import curvedb_live as cl
from openpilot.tools.curvedb import roadtable as rt
from openpilot.tools.curvedb import v2_build
from openpilot.tools.curvedb import v2_live_export as ex
from openpilot.tools.curvedb.tests.test_v2 import MPH, P, TESLA, _anchor, _obs, _pt, _synthetic_doc, _write_route

PR = replace(P, allow_ramps=True)


def _link_pair(**kw):
  return _anchor(_obs("2026-09-01", 0.004, hwy="motorwayLink", **kw), _obs("2026-09-02", 0.004, hwy="motorwayLink", **kw))


def test_the_flag_is_off_in_code():
  assert rt.PROVISIONAL_V2.allow_ramps is False


def test_a_ramp_branch_is_granted_with_the_flag_and_refused_without():
  assert rt.row_verdict(_link_pair(), P).reason == "ramp (motorwayLink)"
  v = rt.row_verdict(_link_pair(), PR)
  assert v.granted and v.k == pytest.approx(0.004) and v.hwy == "motorwayLink"


def test_a_mainline_extent_reaching_a_ramp_is_granted_with_the_flag_and_refused_without():
  a = _anchor(*[replace(_obs(d, 0.004), ext_ramp=True) for d in ("2026-09-01", "2026-09-02")])
  assert rt.row_verdict(a, P).reason == "extent reaches a ramp" and rt.row_verdict(a, PR).granted


@pytest.mark.parametrize("name, anchor, why", [
  ("one date", lambda: _anchor(_obs("2026-09-01", 0.004, hwy="motorwayLink"), _obs("2026-09-01", 0.004, hwy="motorwayLink", drive="x")), "date"),
  ("tesla only", lambda: _anchor(*[_obs(d, 0.004, car=TESLA, hwy="motorwayLink") for d in ("2026-09-01", "2026-09-02")]), "tesla-only"),
  ("unknown class", lambda: _anchor(*[_obs(d, 0.004, hwy="unknown") for d in ("2026-09-01", "2026-09-02")]), "unknown"),
  ("dates disagree", lambda: _anchor(_obs("2026-09-01", 0.004, hwy="motorwayLink"), _obs("2026-09-02", 0.009, hwy="motorwayLink")), "disagree"),
  ("no classified extent", lambda: _anchor(*[replace(_obs(d, 0.004, hwy="motorwayLink"), ext_known=False) for d in ("2026-09-01", "2026-09-02")]),
   "classified"),
])
def test_every_other_refusal_stays_with_the_flag(name, anchor, why):
  v = rt.row_verdict(anchor(), PR)
  assert not v.granted and why in v.reason, name


def test_non_ramp_verdicts_are_identical_with_and_without_the_flag():
  for obs in ([_obs("2026-09-01", 0.002), _obs("2026-09-02", 0.002)], [_obs("2026-09-01", 0.002)],
              [_obs("2026-09-01", 0.002, hwy="unknown"), _obs("2026-09-02", 0.002, hwy="unknown")]):
    assert rt.row_verdict(_anchor(*obs), P) == rt.row_verdict(_anchor(*obs), PR)


def test_links_seed_anchors_only_with_the_flag_and_only_the_three_scope_links():
  fast = 40 * MPH + 0.01
  for hwy in ("motorwayLink", "trunkLink", "primaryLink"):
    assert not v2_build.in_scope(_pt(hwy, fast)) and v2_build.in_scope(_pt(hwy, fast), True)
    assert not v2_build.in_scope(_pt(hwy, 40 * MPH - 0.01), True)
  for hwy in ("secondaryLink", "tertiaryLink", "secondary", "unknown", ""):
    assert not v2_build.in_scope(_pt(hwy, fast), True)


def _link_doc():
  d = _synthetic_doc()
  d["mapd"] = [[r[0], r[1], "motorwayLink", *r[3:]] for r in d["mapd"]]
  return d


def _build_link_table(tmp_path, params):
  _write_route(tmp_path, "00000001--aaaa", _link_doc(), 1_788_000_000.0)
  _write_route(tmp_path, "00000002--bbbb", _link_doc(), 1_789_000_000.0)
  return v2_build.build(str(tmp_path), params, Counter())


def test_a_ramp_driven_on_two_dates_gets_a_row_only_with_the_flag(tmp_path):
  assert not _build_link_table(tmp_path, P).anchors
  idx = _build_link_table(tmp_path, PR)
  rows = [rt.row_verdict(a, PR, branch=br) for a in idx.anchors for br in rt.branches(a, PR.branch_radius_m)]
  assert any(v.granted and v.n_dates == 2 and v.hwy == "motorwayLink" for v in rows)


def test_params_doc_is_byte_identical_without_the_flag():
  assert rt.params_doc(P) == {k: v for k, v in asdict(P).items() if k != "allow_ramps"}
  assert "allow_ramps" not in rt.params_doc(P) and rt.params_doc(PR)["allow_ramps"] is True


def test_the_build_dump_without_the_flag_has_no_trace_of_it(tmp_path):
  src = tmp_path / "tr"
  src.mkdir()
  _write_route(src, "00000001--aaaa", _synthetic_doc(), 1_788_000_000.0)
  _write_route(src, "00000002--bbbb", _synthetic_doc(), 1_789_000_000.0)
  idx = v2_build.build(str(src), P, Counter())
  v2_build.dump(idx, P, str(tmp_path / "o"), Counter())
  assert "allow_ramps" not in gzip.open(tmp_path / "o" / "table.json.gz", "rt").read()
  assert "allow_ramps" not in (tmp_path / "o" / "rows.json").read_text()


def _export(tmp_path, p):
  idx = rt.AnchorIndex(p)
  a = idx.anchors[idx.add(45.0, -122.7, 0.0)]
  a.obs = [_obs(d, 0.004, hwy="motorwayLink") for d in ("2026-09-01", "2026-09-02")]
  doc, counts = ex.export_doc(idx)
  src = tmp_path / "table.json.gz"
  src.write_bytes(b"x")
  ex.write(doc, counts, str(tmp_path), str(src))
  return doc, counts, json.loads((tmp_path / "manifest.json").read_text())


def test_the_export_without_the_flag_refuses_the_ramp_and_declares_nothing(tmp_path):
  doc, counts, man = _export(tmp_path, P)
  assert doc["anchors"][0][3][0][2] is None and "flags" not in man and "ramp_rows" not in man and "ramps" not in counts


def test_the_export_with_the_flag_grants_the_ramp_declares_it_and_the_car_loader_reads_it(tmp_path):
  doc, counts, man = _export(tmp_path, PR)
  assert doc["anchors"][0][3][0][2] == pytest.approx(0.004) and man["flags"] == "ramps" and man["ramp_rows"] == 1
  idx, _ = cl.load_rows(str(tmp_path))
  assert idx.has_ramps and idx.n_rows == 1


# ---- sharpest-pass pricing of link rows ----
def _export_k(p, hwy, ks, exclude_date=None):
  idx = rt.AnchorIndex(p)
  a = idx.anchors[idx.add(45.0, -122.7, 0.0)]
  a.obs = [_obs(f"2026-09-0{n + 1}", k, hwy=hwy) for n, k in enumerate(ks)]
  doc, _ = ex.export_doc(idx, exclude_date)
  return doc["anchors"][0][3][0][2]


def _passes(hwy, ks, ext=False):
  return [replace(_obs(f"2026-09-0{n + 1}", k, hwy=hwy), ext_ramp=ext) for n, k in enumerate(ks)]


def test_a_link_row_is_exported_at_its_sharpest_pass_with_the_flag():
  assert _export_k(PR, "motorwayLink", [0.010, 0.012, 0.015]) == pytest.approx(0.015)
  # the verdict itself carries it, so the replay and rows.json (which read v.k) see the same k as the export
  assert rt.row_verdict(_anchor(*_passes("motorwayLink", [0.010, 0.012, 0.015])), PR).k == pytest.approx(0.015)


def test_a_mainline_anchor_whose_extent_reaches_a_ramp_is_priced_on_the_sharpest_pass_too():
  a = _anchor(*_passes("motorway", [0.010, 0.012, 0.015], ext=True))
  assert rt.row_verdict(a, PR).granted and rt.row_verdict(a, PR).k == pytest.approx(0.015)
  idx = rt.AnchorIndex(PR)
  idx.anchors[idx.add(45.0, -122.7, 0.0)].obs = a.obs
  assert ex.export_doc(idx)[0]["anchors"][0][3][0][2] == pytest.approx(0.015)


def test_build_rows_json_uses_the_same_k(tmp_path):
  idx = rt.AnchorIndex(PR)
  idx.anchors[idx.add(45.0, -122.7, 0.0)].obs = _passes("motorwayLink", [0.010, 0.012, 0.015])
  v2_build.dump(idx, PR, str(tmp_path), Counter())
  assert json.loads((tmp_path / "rows.json").read_text())["rows"][0]["k"] == pytest.approx(0.015)


@pytest.mark.parametrize("hwy", ["trunkLink", "primaryLink"])
def test_every_link_class_is_priced_on_the_sharpest_pass(hwy):
  assert _export_k(PR, hwy, [0.010, 0.012, 0.015]) == pytest.approx(0.015)


def test_a_mainline_row_that_touches_no_ramp_keeps_the_median_with_the_flag():
  assert _export_k(PR, "motorway", [0.010, 0.012, 0.015]) == pytest.approx(0.012)


def test_without_the_flag_nothing_changes():
  assert _export_k(P, "motorway", [0.010, 0.012, 0.015]) == pytest.approx(0.012)
  assert _export_k(P, "motorwayLink", [0.010, 0.012, 0.015]) is None     # refused, as before
  assert rt.row_verdict(_anchor(*_passes("motorway", [0.010, 0.012, 0.015], ext=True)), P).reason == "extent reaches a ramp"


def test_the_sharpest_pass_honours_exclude_date():
  assert _export_k(PR, "motorwayLink", [0.010, 0.012, 0.015], exclude_date="2026-09-03") == pytest.approx(0.012)


# ---- ramp end vs mainline end separation ----
def _two_branch_anchor(sep_m, ramp_hwy="motorwayLink", ramp_ext=False):
  """One anchor, a mainline branch (3 dates) and a ramp branch whose end is sep_m metres north of the mainline end."""
  main_end = (45.00135, -122.7)
  ramp_end = (main_end[0] + sep_m / 111194.0, main_end[1])
  obs = [_obs(f"2026-09-0{n + 1}", 0.004, end=main_end) for n in range(3)]
  obs += [replace(_obs(f"2026-09-0{n + 1}", 0.007, hwy=ramp_hwy, drive=f"ramp{n}", end=ramp_end), ext_ramp=ramp_ext) for n in range(3)]
  return _anchor(*obs), main_end, ramp_end


def test_a_ramp_end_within_25_m_of_a_mainline_end_is_refused_beyond_it_granted():
  a, main_end, ramp_end = _two_branch_anchor(19.0)
  v = rt.row_verdict(a, PR, branch=ramp_end)
  assert not v.granted and v.reason.startswith("ramp end 19 m from a mainline branch end")
  a, main_end, ramp_end = _two_branch_anchor(40.0)
  v = rt.row_verdict(a, PR, branch=ramp_end)
  assert v.granted and v.k == pytest.approx(0.007)


def test_the_mainline_branch_is_unaffected_by_a_close_ramp():
  a, main_end, ramp_end = _two_branch_anchor(19.0)
  v = rt.row_verdict(a, PR, branch=main_end)
  assert v.granted and v.k == pytest.approx(0.004)


def test_an_ext_ramp_priced_branch_is_covered_too_and_the_flag_off_is_unchanged():
  a, main_end, ramp_end = _two_branch_anchor(19.0, ramp_hwy="motorway", ramp_ext=True)
  assert rt.row_verdict(a, PR, branch=ramp_end).reason.startswith("ramp end")
  assert rt.row_verdict(a, P, branch=ramp_end).reason == "extent reaches a ramp"      # off: the old refusal, not the new one


def test_a_ramp_next_to_another_ramp_is_not_refused_by_the_rule():
  a, main_end, ramp_end = _two_branch_anchor(19.0)
  second = (ramp_end[0] + 20.0 / 111194.0, ramp_end[1])                          # a second ramp 20 m past the first (39 m from the mainline)
  a.obs += [replace(_obs(f"2026-09-0{n + 1}", 0.007, hwy="motorwayLink", drive=f"r2{n}", end=second), ext_ramp=True) for n in range(3)]
  assert rt.ramp_near_mainline(a, PR, None, second) is None


def test_the_export_writes_the_refused_ramp_branch_as_null():
  a, main_end, ramp_end = _two_branch_anchor(19.0)
  idx = rt.AnchorIndex(PR)
  idx.anchors[idx.add(a.lat, a.lon, a.brg)].obs = a.obs
  doc, _ = ex.export_doc(idx)
  ks = sorted((b[2] is None, round(b[2] or 0, 4)) for b in doc["anchors"][0][3])
  assert ks == [(False, 0.004), (True, 0)]


# ---- review fixes: canonical centres, mainline flicker, cross-anchor ----
def _north(end, m):
  return (end[0] + m / 111194.0, end[1])


def test_a_raw_query_end_point_is_snapped_to_its_branch_centre_before_the_test():
  """LODO / replay queries pass a pass end point, not a centre. M-R = 16 m; a ramp query 4 m toward M (12 m from M, 4 m from R) used to skip BOTH."""
  a, main_end, ramp_end = _two_branch_anchor(16.0)
  query = _north(ramp_end, -4.0)
  assert rt.ramp_near_mainline(a, PR, None, query) == pytest.approx(16.0, abs=0.5)
  a, main_end, ramp_end = _two_branch_anchor(23.0)
  assert rt.ramp_near_mainline(a, PR, None, _north(ramp_end, 4.0)) == pytest.approx(23.0, abs=0.5)      # snapped to R: still 23 m from M, not granted by drift


def test_one_flickering_pass_does_not_turn_a_mainline_branch_into_a_ramp():
  a, main_end, ramp_end = _two_branch_anchor(19.0)
  a.obs += [replace(_obs(f"2026-08-{n + 10}", 0.004, end=main_end, drive=f"m{n}"), ext_ramp=(n == 0)) for n in range(5)]    # 1 of 8 mainline passes flickered
  assert rt.row_verdict(a, PR, branch=ramp_end).reason.startswith("ramp end")


def _gore_index(ramp_gap_m, mainline_ext=False):
  """A0: mainline passes. A1 (40 m on): ONLY ramp passes, ending ramp_gap_m from A0's mainline end. Mainline passes never attached to A1."""
  idx = rt.AnchorIndex(PR)
  main_end = (45.00135, -122.7)
  a0 = idx.anchors[idx.add(45.0, -122.7, 0.0)]
  a0.obs = [_obs(f"2026-09-0{n + 1}", 0.004, end=main_end) for n in range(3)]
  a1 = idx.anchors[idx.add(45.00036, -122.7, 0.0)]
  ramp_end = _north(main_end, ramp_gap_m)
  a1.obs = [replace(_obs(f"2026-09-0{n + 1}", 0.007, hwy="motorwayLink", drive=f"r{n}", end=ramp_end), ext_ramp=True) for n in range(3)]
  return idx, a1, ramp_end


def test_a_ramp_only_anchor_past_the_gore_is_refused_when_a_mainline_end_is_within_25_m():
  idx, a1, ramp_end = _gore_index(19.0)
  assert rt.row_verdict(a1, PR, branch=ramp_end).granted                                  # the same-anchor test alone cannot see the mainline
  v = rt.row_verdict(a1, PR, branch=ramp_end, index=idx)
  assert not v.granted and v.reason.startswith("ramp end 19 m")
  doc, _ = ex.export_doc(idx)
  assert [b[2] for b in doc["anchors"][1][3]] == [None] and doc["anchors"][0][3][0][2] == pytest.approx(0.004)      # the export passes the index


def test_a_ramp_only_anchor_with_a_mainline_end_70_m_away_is_granted():
  idx, a1, ramp_end = _gore_index(70.0)
  assert rt.row_verdict(a1, PR, branch=ramp_end, index=idx).granted


def test_the_cross_anchor_test_ignores_ramp_ends_and_other_headings():
  idx, a1, ramp_end = _gore_index(19.0)
  idx.anchors[0].obs = [replace(o, hwy="motorwayLink", ext_ramp=True) for o in idx.anchors[0].obs]       # A0 is a ramp too: nothing mainline nearby
  assert rt.row_verdict(a1, PR, branch=ramp_end, index=idx).granted
  idx, a1, ramp_end = _gore_index(19.0)
  idx.anchors[0].brg = 180.0                                                                # a crossing road, not the same direction
  assert rt.row_verdict(a1, PR, branch=ramp_end, index=idx).granted


def _chain_gap_index(lateral_m, with_rule_index):
  """Mainline anchors A0 and A2 80 m apart (first-come anchors can be that far apart); a RAMP-ONLY anchor A1 midway whose ramp end is lateral_m beside
  where a mainline path ends 150 m past A1. Returns (rows dir loaded by the car, mainline polyline)."""
  idx = rt.AnchorIndex(PR)
  lon = -122.7
  for lat in (45.0, 45.00072):
    idx.anchors[idx.add(lat, lon, 0.0)].obs = [_obs(f"2026-09-0{n + 1}", 0.004, end=(lat + 150.0 / 111194.0, lon)) for n in range(3)]
  a1 = idx.anchors[idx.add(45.00036, lon, 0.0)]
  ramp_end = (45.00036 + 150.0 / 111194.0, lon + lateral_m / (111194.0 * math.cos(math.radians(45.0))))
  a1.obs = [replace(_obs(f"2026-09-0{n + 1}", 0.007, hwy="motorwayLink", drive=f"r{n}", end=ramp_end), ext_ramp=True) for n in range(3)]
  return idx


def _car_match_at_a1(idx, tmp_path, index_rule):
  import types
  if not index_rule:                                   # the same-anchor-only rule of e0059cc187
    real = rt.mainline_ends
    rt.mainline_ends = lambda index, exclude_date: types.SimpleNamespace(nearest=lambda *a: None)
  try:
    d, c = ex.export_doc(idx)
  finally:
    if not index_rule:
      rt.mainline_ends = real
  (tmp_path / "t").write_bytes(b"x")
  ex.write(d, c, str(tmp_path), str(tmp_path / "t"))
  rows, _ = cl.load_rows(str(tmp_path))
  pts = [{"latitude": 44.9990 + n * 10.0 / 111194.0, "longitude": -122.7} for n in range(520)]
  poly = cl.Polyline(pts)
  s = (45.00036 - 44.9990) * 111194.0
  la, lo = poly.at(s)
  return cl.match_at(rows, poly, s, la, lo)


def test_a_mainline_chain_gap_lets_the_ramp_row_be_matched_by_a_mainline_car_without_the_cross_anchor_rule(tmp_path):
  m = _car_match_at_a1(_chain_gap_index(8.0, False), tmp_path, index_rule=False)
  assert m.why == "ok" and m.k == pytest.approx(0.007)         # the phantom: the mainline car is handed the ramp's row


def test_the_cross_anchor_rule_closes_the_chain_gap(tmp_path):
  m = _car_match_at_a1(_chain_gap_index(8.0, True), tmp_path, index_rule=True)
  assert m.why == "noAuthority"                                   # the branch is exported null: no effect on the mainline car


@pytest.mark.parametrize("lat0, lon0", [(44.55632, -123.26681), (47.97923, -122.19012), (45.52, -122.66), (58.0, -122.7), (64.0, -122.7), (75.0, -122.7)])
@pytest.mark.parametrize("bearing_deg", [0, 45, 90, 135, 180, 225, 270, 315])
def test_the_cross_anchor_search_finds_a_mainline_end_just_inside_65_m_in_every_direction_and_cell_alignment(lat0, lon0, bearing_deg):
  """Negative longitudes, several latitudes (cos(lat) scaling), diagonal and east-west offsets, many alignments to the grid edge."""
  kx = 111194.0 * math.cos(math.radians(lat0))
  for step in range(40):
    la = lat0 + step * 1.3e-5
    lo = lon0 + step * 2.9e-5
    idx = rt.AnchorIndex(PR)
    idx.anchors[idx.add(la, lo, 0.0)].obs = [_obs(f"2026-09-0{n + 1}", 0.004, end=(la, lo)) for n in range(3)]
    for dist, expect in ((64.0, True), (66.0, False)):
      q = (la + dist * math.cos(math.radians(bearing_deg)) / 111194.0, lo + dist * math.sin(math.radians(bearing_deg)) / kx)
      got = rt.mainline_ends(idx, None).nearest(q[0], q[1], 0.0, -1)
      assert (got is not None) == expect, (step, dist, got)


def _spread_ramp_anchor(sep_m):
  """A mainline branch (3 dates, k 0.004) and a ramp branch sep_m north of it whose dates DISAGREE (0.004 / 0.007), pass speeds recorded."""
  main_end = (45.00135, -122.7)
  ramp_end = _north(main_end, sep_m)
  obs = [replace(_obs(f"2026-09-0{n + 1}", 0.004, end=main_end), v_ms=20.0) for n in range(3)]
  ks = (0.004, 0.007, 0.007)
  obs += [replace(_obs(f"2026-09-0{n + 1}", ks[n], hwy="motorwayLink", drive=f"ramp{n}", end=ramp_end), ext_ramp=True, v_ms=15.0) for n in range(3)]
  idx = rt.AnchorIndex(PR)
  idx.anchors[idx.add(45.0, -122.7, 0.0)].obs = obs
  return idx


def test_the_separation_refusal_wins_over_raise_only():
  doc, counts = ex.export_doc(_spread_ramp_anchor(19.0), raise_only=True)
  assert counts["raise_only_rows"] == 0
  assert sorted(len(b) for b in doc["anchors"][0][3]) == [4, 4] and sorted(b[2] is None for b in doc["anchors"][0][3]) == [False, True]


def test_a_far_spread_refused_ramp_branch_is_still_a_raise_only_row():
  doc, counts = ex.export_doc(_spread_ramp_anchor(70.0), raise_only=True)
  assert counts["raise_only_rows"] == 1
