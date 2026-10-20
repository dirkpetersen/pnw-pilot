"""curvedbramps2pnw (owner 2026-10-19, "ramp option A"): with V2Params.allow_ramps a ramp/link branch is treated like any other road;
EVERY other refusal stays, and a build without the flag is unchanged. Synthetic geometry only (the real rows are private)."""
import gzip
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
