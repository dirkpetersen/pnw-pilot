"""dbfirst2pnw -- the curve brain's COVERAGE of mapd's path (ces_pnw/curve_brain.py), the input of VTSC's DB-first rule.

On a synthetic table (no private data): a vertex whose keyed row has authority is COVERED (1); no anchor / a refused branch / a branch the path
takes that was never recorded is UNCOVERED (0); a path too short to key the branch, or an uncovered vertex inside a per-curve override, is UNKNOWN
(omitted -- VTSC then keeps today's notch). Coverage is published only while curve.json tesla.vtsc_db_first is on, the brain ACTS (lower/raise)
and every gate passed; an exception never publishes it. parse_coverage is VTSC's side of the contract."""
import json

import pytest

from openpilot.selfdrive.controls.lib import pnw_vehicle as pv
from openpilot.selfdrive.controls.lib.ces_pnw import curve_brain as cb
from openpilot.selfdrive.controls.lib.ces_pnw import curvedb_live as cl
from openpilot.selfdrive.controls.lib.ces_pnw.tests.roaddb_fixture import write_db
from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_curvebrain_need_pnw import (   # fixtures + helpers of the brain's own tests
  TESLA, _step, cfgpath, log, schedule, tesla, write_overrides)
from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_curvedblive2pnw import LAT0, _anchor, _ll, _path

__all__ = ["cfgpath", "log", "schedule"]     # pytest fixtures re-exported from the brain's tests

M_PER_DEG = 111320.0

def _flag(anchor):
  """The same anchor with every branch flagged lowerBound (5th element 1)."""
  return [*anchor[:3], [[*b, 1] if b[2] is not None else b for b in anchor[3]]]


# a path due north through the car at y = 0, a node every 40 m from -180 m
TABLE = [
  _anchor(300.0, 0.004),                                       # a real row: nodes 260..340 are COVERED
  _anchor(140.0, None, branches=[(0.0, 290.0, None)]),         # an anchor whose branch was refused: nodes 100..180 are UNCOVERED (noAuthority)
  _anchor(420.0, None, branches=[(60.0, 570.0, 0.002)]),       # driven, but the branch THIS path takes (y = 570) was never recorded: UNCOVERED
]


def _brain(tmp_path, monkeypatch, anchors=None, flags=True):
  """A Tesla brain over a table that DECLARES lowerBound flags (flags=False: a table with none)."""
  d = tmp_path / "db"
  write_db(str(d), anchors, flags=flags)
  monkeypatch.setattr(cl, "DATA_DIR", str(d))
  return cb.CurveBrain(tesla())


def _y(p):
  return round((p["latitude"] - LAT0) * M_PER_DEG)


def _cov(out):
  return {_y({"latitude": la}): c for la, lo, c in out["cov"]}


def test_the_verdicts(tmp_path, monkeypatch, cfgpath, schedule, log):
  b = _brain(tmp_path, monkeypatch, anchors=TABLE)
  out = _step(b)
  cov = _cov(out)
  assert cov[300] == 1                                              # a row with authority
  assert cov[60] == 0 and cov[220] == 0                              # no anchor: never driven
  assert cov[140] == 3                                              # an anchor whose row was refused: DRIVEN, not measured (owner: not 'never driven')
  assert cov[420] == 4                                              # driven, but not along this branch: also not 'never driven'
  n1, n2 = sum(1 for c in cov.values() if c == 1), sum(1 for c in cov.values() if c == 2)
  assert out["v"] is not None and b.tele(100.0)["cbCov"] == f"{n1}/{sum(1 for c in cov.values() if c == 0)}/{n2}/{sum(1 for c in cov.values() if c in (3, 4))}"
  assert min(cov) >= -20 and max(cov) <= 500                        # from just behind the car to mapd's horizon
  assert log.errors == [] and log.exceptions == []


@pytest.mark.parametrize("flag_y, want", [(60.0, 2), (340.0, 2), (380.0, 1)])
def test_a_covered_vertex_is_unreliable_when_a_lower_bound_row_lies_between_the_car_and_it_or_just_past_it(
    tmp_path, monkeypatch, cfgpath, schedule, flag_y, want):
  """The notch a covered vertex replaces is a cap over the brake envelope from the car to that vertex (and the apex just past it): the bend
  whose saturated APEX rows lie before the covered node must keep its notch. Vertex 100: a flagged row at 60 (between the car at 0 and it) or at
  340 (240 m past it) makes it unreliable; at 380 (280 m past) it stays reliable."""
  b = _brain(tmp_path, monkeypatch, anchors=[_anchor(100.0, 0.001), _flag(_anchor(flag_y, 0.003))])
  cov = _cov(_step(b))
  assert cov[100] == want
  assert cov[flag_y] == 2                                           # the flagged row's own vertex is unreliable either way


def test_a_flagged_row_behind_the_car_does_not_count_for_other_vertices(tmp_path, monkeypatch, cfgpath, schedule):
  """The flagged anchor at -50 is behind the car's window (s_ego - 25): vertex -20 is keyed to it (so that vertex itself is UNRELIABLE), but the
  vertex at 300 is not made unreliable by a row the car has already passed."""
  b = _brain(tmp_path, monkeypatch, anchors=[_flag(_anchor(-50.0, 0.003)), _anchor(300.0, 0.001)])
  cov = _cov(_step(b))
  assert cov[-20] == 2 and cov[300] == 1


def test_a_table_without_flags_makes_every_covered_row_unreliable_and_says_so_once(tmp_path, monkeypatch, cfgpath, schedule, log):
  warnings = []
  monkeypatch.setattr(cb.cloudlog, "warning", lambda msg, *a, **k: warnings.append(msg))
  b = _brain(tmp_path, monkeypatch, anchors=[_anchor(300.0, 0.004)], flags=False)
  out = _step(b)
  assert _cov(out)[300] == 2 and 1 not in set(_cov(out).values())
  _step(b, now=100.25)
  assert sum("declares no `lowerBound`" in w for w in warnings) == 1
  assert out["v"] is not None                                        # ... the row is still PRICED (cbV), only the notch is kept


def test_the_keys_survive_json_exactly(tmp_path, monkeypatch, cfgpath, schedule):
  """VTSC looks a point up by the exact (lat, lon) floats of mapd's list: they must round-trip through the param's JSON."""
  b = _brain(tmp_path, monkeypatch, anchors=TABLE)
  path = _path()
  out = _step(b, points=path)
  parsed, why = cb.parse_coverage(json.dumps(out), out["ts"])
  assert why == "ok" and len(parsed) == len(out["cov"])
  for p in path:
    if -25.0 <= _y(p) <= 500.0:
      assert (p["latitude"], p["longitude"]) in parsed


def test_a_path_too_short_to_key_the_branch_is_unknown_not_uncovered(tmp_path, monkeypatch, cfgpath, schedule):
  b = _brain(tmp_path, monkeypatch, anchors=[_anchor(460.0, 0.004)])
  out = _step(b, points=_path(y1=520.0))                             # the anchor's +150 m lies past the path's end
  assert 460 in {_y(p) for p in _path(y1=520.0)}                     # ... and 460 IS a vertex of this path (else this test proves nothing)
  assert 460 not in _cov(out)                                       # omitted: VTSC keeps the notch there
  assert _cov(out)[380] == 0                                        # ... while a vertex far from any anchor is plainly uncovered


def test_an_uncovered_vertex_inside_a_per_curve_override_is_unknown(tmp_path, monkeypatch, cfgpath, schedule):
  la, lo = _ll(0.0, 60.0)
  write_overrides(None, raw=json.dumps({"version": 2, "overrides": [
    {"lat": la, "lon": lo, "radius_m": 60.0, "heading_deg": 0.0, "heading_tol_deg": 30.0,
     "cars": {TESLA: {"a_max": 2.5}}, "note": "known bad curve"}]}))
  b = _brain(tmp_path, monkeypatch, anchors=TABLE)
  cov = _cov(_step(b))
  assert 60 not in cov and 20 not in cov                            # inside the circle, no row to price: today's notch
  assert cov[220] == 0                                              # outside it: plainly uncovered


def test_a_covered_vertexs_row_is_priced_even_when_the_scan_stepped_over_it(tmp_path, monkeypatch, cfgpath, schedule):
  b = _brain(tmp_path, monkeypatch, anchors=[TABLE[0]])
  poly = b.db.polyline(_path())
  _, extra = b._coverage(poly, _path(), 0.0, [])                     # the 25 m scan found nothing
  assert [m.row_id for m in extra] == ["0:0"] and extra[0].k == 0.004


@pytest.mark.parametrize("cfg, why", [
  ({"tesla": {"vtsc_db_first": False}}, "the switch is off"),
  ({"tesla": {"curve_brain": "shadow"}}, "the brain does not act"),
  ({"tesla": {"curve_brain": "off"}}, "the brain is off"),
])
def test_no_coverage_unless_the_switch_is_on_and_the_brain_acts(tmp_path, monkeypatch, cfgpath, schedule, cfg, why):
  cfgpath.write_text(json.dumps(cfg))
  b = _brain(tmp_path, monkeypatch, anchors=TABLE)
  out = _step(b)
  assert "cov" not in out, why
  assert b.tele(100.0)["cbCov"] is None


@pytest.mark.parametrize("kw", [dict(way="predicted"), dict(hwy="motorwayLink"), dict(gps="stale"), dict(points=[])])
def test_a_closed_gate_publishes_no_coverage(tmp_path, monkeypatch, cfgpath, schedule, kw):
  b = _brain(tmp_path, monkeypatch, anchors=TABLE)
  assert "cov" not in _step(b, **kw)


def test_a_covered_vertexs_row_is_priced_in_the_step_itself(tmp_path, monkeypatch, cfgpath, schedule):
  """The scan finds nothing (stubbed); the covered vertex's row must still give the need."""
  monkeypatch.setattr(cb, "scan_ahead", lambda *a, **k: [])
  b = _brain(tmp_path, monkeypatch, anchors=[TABLE[0]])
  out = _step(b)
  assert out["v"] is not None and out["row"] == "0:0" and b.tele(100.0)["cbN"] == 1


def test_an_exception_after_coverage_was_computed_still_publishes_none(tmp_path, monkeypatch, cfgpath, schedule, log):
  b = _brain(tmp_path, monkeypatch, anchors=TABLE)
  monkeypatch.setattr(cb, "most_binding_row", lambda *a, **k: 1 / 0)
  out = _step(b)
  assert "cov" not in out and b.tele(100.0)["cbWhy"] == "err"


def test_an_exception_never_publishes_coverage_and_says_so(tmp_path, monkeypatch, cfgpath, schedule, log):
  b = _brain(tmp_path, monkeypatch, anchors=TABLE)
  monkeypatch.setattr(b, "_coverage", lambda *a, **k: 1 / 0)
  out = _step(b)
  assert "cov" not in out and out["v"] is None and b.tele(100.0)["cbWhy"] == "err" and b.tele(100.0)["cbCov"] is None
  assert any("need layer FAILED" in e for e in log.exceptions)


def test_the_vertex_count_is_bounded(tmp_path, monkeypatch, cfgpath, schedule):
  monkeypatch.setattr(cb, "COV_MAX_POINTS", 4)
  b = _brain(tmp_path, monkeypatch, anchors=TABLE)
  assert len(_step(b)["cov"]) == 4


def test_a_table_without_rows_ahead_still_publishes_coverage_uncovered(tmp_path, monkeypatch, cfgpath, schedule):
  """noRow (no row anywhere ahead) is the COMMON uncovered case: the entry has no need but must still say 'uncovered'."""
  b = _brain(tmp_path, monkeypatch, anchors=[[0.5, 0.5, 0.0, [[0.5013, 0.5, 0.001, 3]]]])
  out = _step(b)
  assert out["v"] is None and b.tele(100.0)["cbWhy"] == "noRow"
  assert out["cov"] and set(_cov(out).values()) == {0}


# ---------------------------------------------------------------- parse_coverage (VTSC's side)

def _entry(**kw):
  e = {"ts": 100.0, "mode": "lower", "v": None, "cov": [[44.5, -123.1, 1], [44.501, -123.1, 0]]}
  e.update(kw)
  return e


def test_parse_accepts_every_verdict():
  assert cb.parse_coverage(_entry(cov=[[1.0, 2.0, 2], [1.0, 3.0, 3], [1.0, 4.0, 4]]), 100.0) == ({(1.0, 2.0): 2, (1.0, 3.0): 3, (1.0, 4.0): 4}, "ok")


def test_parse_ok_returns_exact_keys():
  assert cb.parse_coverage(json.dumps(_entry()), 100.2) == ({(44.5, -123.1): 1, (44.501, -123.1): 0}, "ok")
  assert cb.parse_coverage(_entry(mode="raise"), 100.2)[1] == "ok"             # a dict is accepted like the param's decoded value


@pytest.mark.parametrize("raw, now, why", [
  (None, 100.0, "absent"), ("", 100.0, "absent"), ({}, 100.0, "absent"), (_entry(cov=None), 100.0, "absent"),
  (_entry(), 101.5, "stale"), (_entry(), 99.0, "stale"),
  (_entry(mode="shadow"), 100.0, "mode"), (_entry(mode="off"), 100.0, "mode"),
  ("{nope", 100.0, "bad"), ([], 100.0, "bad"), (_entry(ts=None), 100.0, "bad"), (_entry(ts=float("nan")), 100.0, "bad"),
  (_entry(cov="x"), 100.0, "bad"), (_entry(cov=[[1, 2]]), 100.0, "bad"), (_entry(cov=[[1, 2, 5]]), 100.0, "bad"),
  (_entry(cov=[[1, 2, True]]), 100.0, "bad"), (_entry(cov=[[float("inf"), 2, 1]]), 100.0, "bad"),
  (_entry(cov=[["a", 2, 1]]), 100.0, "bad"), (_entry(cov=[[1, 2, 1]] * (2 * cb.COV_MAX_POINTS + 1)), 100.0, "bad"),
])
def test_parse_refusals_are_named(raw, now, why):
  out, got = cb.parse_coverage(raw, now)
  assert out is None and got == why


def test_the_default_switch_is_on_for_the_raven_only(cfgpath):
  assert pv.PnwVehicle(type("CP", (), dict(carFingerprint=TESLA, brand="tesla", openpilotLongitudinalControl=True, dashcamOnly=False))()).vtsc_db_first


# ---------------------------------------------------------------- the table side (curvedb_live)

def _rows_file(tmp_path, anchors, flags):
  d = tmp_path / "t"
  write_db(str(d), anchors, flags=flags)
  return cl.load_rows(str(d))


def test_the_loader_reads_the_flag_only_from_a_declaring_file(tmp_path):
  idx, man = _rows_file(tmp_path, [_flag(_anchor(300.0, 0.004)), _anchor(500.0, 0.002)], flags=True)
  assert idx.has_flags and man["flags"] == "lowerBound"
  assert [b[4] for a in idx.anchors for b in a[3]] == [True, False]
  idx, _ = _rows_file(tmp_path / "x", [_anchor(300.0, 0.004)], flags=False)
  assert not idx.has_flags and idx.anchors[0][3][0][4] is False


@pytest.mark.parametrize("bad", ["undeclared", "value", "no-k"])
def test_a_malformed_flag_is_a_loud_load_failure(tmp_path, bad):
  a = _anchor(300.0, 0.004)
  if bad == "undeclared":
    anchors, flags = [_flag(a)], False
  elif bad == "value":
    anchors, flags = [[*a[:3], [[*a[3][0], 2]]]], True
  else:
    anchors, flags = [[*a[:3], [[*a[3][0][:2], None, 3, 1]]]], True
  with pytest.raises(cl.CurveDbFileError):
    _rows_file(tmp_path, anchors, flags)


def test_match_reliability(tmp_path):
  from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_curvedblive2pnw import _path
  path = _path()
  for flags, anchors, want in ((True, [_anchor(300.0, 0.004)], True), (True, [_flag(_anchor(300.0, 0.004))], False),
                               (False, [_anchor(300.0, 0.004)], False)):
    idx, _ = _rows_file(tmp_path / f"{flags}{want}{len(str(anchors))}", anchors, flags)
    poly = cl.Polyline(path)
    la, lo = _ll(0.0, 300.0)
    m = cl.match_at(idx, poly, poly.project(la, lo)[0], la, lo)
    assert m.why == "ok" and m.reliable is want


def test_the_loaded_event_carries_the_flags_and_the_manifest_count_is_checked(tmp_path, monkeypatch):
  events = []
  monkeypatch.setattr(cl.cloudlog, "event", lambda name, **kw: events.append((name, kw)))
  d = tmp_path / "e"
  write_db(str(d), [_flag(_anchor(300.0, 0.004)), _anchor(500.0, 0.002)], flags=True)
  db = cl.CurveDbLive(True, data_dir=str(d), read_params=lambda: (None, None), start=False)
  db.load()
  ev = [kw for n, kw in events if n == "curvedb_v2_loaded"][0]
  assert db.state == "ok" and ev["flags"] == "lowerBound" and ev["lower_bound_rows"] == 1 and ev["rows"] == 2
  man = json.loads((d / "manifest.json").read_text())
  man["lower_bound_rows"] = 5
  (d / "manifest.json").write_text(json.dumps(man))
  with pytest.raises(cl.CurveDbFileError, match="lowerBound rows"):
    cl.load_rows(str(d))
  d2 = tmp_path / "u"
  write_db(str(d2), [_anchor(300.0, 0.004)])
  events.clear()
  cl.CurveDbLive(True, data_dir=str(d2), read_params=lambda: (None, None), start=False).load()
  assert events[0][1]["flags"] is None and events[0][1]["lower_bound_rows"] == 0          # an unflagged table says so
