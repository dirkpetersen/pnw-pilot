"""dbfirst2pnw: the exporter's lowerBound flags (v2_live_export.py --flags) and the car's loader reading them back."""
import json

import pytest

from openpilot.selfdrive.controls.lib.ces_pnw import curvedb_live as cl
from openpilot.tools.curvedb import roadtable as rt
from openpilot.tools.curvedb import v2_live_export as ex

LAT0, LON0 = 45.0, -122.7
END = (LAT0 + 0.00135, LON0)


def _idx(n_anchors=2):
  idx = rt.AnchorIndex(rt.PROVISIONAL_V2)
  for j in range(n_anchors):
    a = idx.anchors[idx.add(LAT0 + 0.01 * j, LON0, 0.0)]
    for n, date in enumerate(("2026-09-01", "2026-09-02", "2026-09-03")):
      a.obs.append(rt.PassObs(date=date, drive=f"d{n}", car="FORD_F_150_LIGHTNING_MK1", t=float(n), k_ext=0.002, k_loc=0.002, way=0,
                              hwy="motorway", road="I 5|", spl=30.0, mode="op", d_m=1.0, end_lat=END[0] + 0.01 * j, end_lon=END[1]))
  return idx


def _write(tmp_path, flags):
  doc, counts = ex.export_doc(_idx(), None, flags)
  src = tmp_path / "table.json.gz"
  src.write_bytes(b"x")
  ex.write(doc, counts, str(tmp_path), str(src))
  return doc, counts, json.loads((tmp_path / "manifest.json").read_text())


def test_no_flags_declares_nothing_and_adds_nothing(tmp_path):
  doc, counts, man = _write(tmp_path, None)
  assert "flags" not in man and counts["flags"] is False
  assert all(len(b) == 4 for a in doc["anchors"] for b in a[3])
  assert not cl.load_rows(str(tmp_path))[0].has_flags


def test_a_flag_marks_exactly_its_branch_and_the_manifest_declares_it(tmp_path):
  doc, counts, man = _write(tmp_path, [[LAT0, LON0, END[0], END[1]]])
  assert man["flags"] == "lowerBound" and man["lower_bound_rows"] == 1 == counts["lower_bound_rows"] and counts["lower_bound_unmatched"] == 0
  assert [len(b) for a in doc["anchors"] for b in a[3]] == [5, 4]
  idx, _ = cl.load_rows(str(tmp_path))
  assert idx.has_flags and [b[4] for a in idx.anchors for b in a[3]] == [True, False]


def test_an_empty_flag_list_still_declares_flags(tmp_path):
  """A table whose build found no lower-bound row is a flagged table: every row reliable."""
  _, counts, man = _write(tmp_path, [])
  assert man["flags"] == "lowerBound" and counts["lower_bound_rows"] == 0
  assert cl.load_rows(str(tmp_path))[0].has_flags


def test_a_flag_that_matches_no_row_is_counted_not_dropped(tmp_path):
  _, counts = ex.export_doc(_idx(), None, [[LAT0, LON0, END[0], END[1]], [50.0, -100.0, 50.001, -100.0], [LAT0, LON0, END[0] + 0.01, END[1]]])
  assert counts["lower_bound_rows"] == 1 and counts["lower_bound_unmatched"] == 2


def test_the_cli_refuses_a_flag_file_that_does_not_belong_to_the_table(tmp_path, monkeypatch):
  import gzip

  from openpilot.tools.curvedb import v2_replay
  monkeypatch.setattr(v2_replay, "load_table", lambda path: _idx())
  monkeypatch.setattr(ex, "load_table", lambda path: _idx())
  flags = tmp_path / "f.json"
  flags.write_text(json.dumps([[50.0, -100.0, 50.001, -100.0]]))
  with gzip.open(tmp_path / "t.json.gz", "wb") as f:
    f.write(b"{}")
  assert ex.main(["--table", str(tmp_path / "t.json.gz"), "--out", str(tmp_path / "o"), "--flags", str(flags)]) == 2
  assert not (tmp_path / "o").exists()
  flags.write_text(json.dumps([[LAT0, LON0, END[0], END[1]]]))
  assert ex.main(["--table", str(tmp_path / "t.json.gz"), "--out", str(tmp_path / "o"), "--flags", str(flags)]) == 0
  assert json.loads((tmp_path / "o" / "manifest.json").read_text())["flags"] == "lowerBound"


@pytest.mark.parametrize("val", [0, 2, "1"])
def test_the_loader_accepts_only_the_value_1(tmp_path, val):
  from openpilot.selfdrive.controls.lib.ces_pnw.tests.roaddb_fixture import write_db
  write_db(str(tmp_path), [[LAT0, LON0, 0.0, [[END[0], END[1], 0.002, 3, val]]]], flags=True)
  with pytest.raises(cl.CurveDbFileError):
    cl.load_rows(str(tmp_path))


def _two_branch_idx():
  """One anchor, two granted branches whose ends are ~20 m apart (A at END, B 0.00018 deg = 20 m north of it)."""
  idx = rt.AnchorIndex(rt.PROVISIONAL_V2)
  a = idx.anchors[idx.add(LAT0, LON0, 0.0)]
  for tag, dlat in (("a", 0.0), ("b", 0.00018)):
    for n, date in enumerate(("2026-09-01", "2026-09-02", "2026-09-03")):
      a.obs.append(rt.PassObs(date=date, drive=f"{tag}{n}", car="FORD_F_150_LIGHTNING_MK1", t=float(n), k_ext=0.002, k_loc=0.002, way=0,
                              hwy="motorway", road="I 5|", spl=30.0, mode="op", d_m=1.0, end_lat=END[0] + dlat, end_lon=END[1]))
  return idx


def _m(lat):
  return lat * 111320.0


def test_matching_is_one_to_one_and_nearest_not_first_come(tmp_path):
  """f1 is 6 m from B (14 m from A), f2 is 6 m from A. First-match would let f1 mark BOTH branches and leave f2 'unmatched'."""
  doc, counts = ex.export_doc(_two_branch_idx(), None, [[LAT0, LON0, END[0] + 0.00018 - 6 / 111320.0, END[1]], [LAT0, LON0, END[0] + 6 / 111320.0, END[1]]])
  rows = doc["anchors"][0][3]
  assert len(rows) == 2 and all(len(b) == 5 for b in rows)
  assert counts["lower_bound_rows"] == 2 and counts["lower_bound_unmatched"] == 0


def test_two_flags_for_one_branch_mark_it_once_and_the_second_is_unmatched():
  doc, counts = ex.export_doc(_two_branch_idx(), None, [[LAT0, LON0, END[0] + 5 / 111320.0, END[1]], [LAT0, LON0, END[0] + 4 / 111320.0, END[1]]])
  rows = doc["anchors"][0][3]
  assert sorted(len(b) for b in rows) == [4, 5] or sorted(len(b) for b in rows) == [5, 5]     # never a 6-element branch
  assert max(len(b) for b in rows) == 5
  assert counts["lower_bound_rows"] + counts["lower_bound_unmatched"] == 2


# ---------------------------------------------------------------------------------------------
# curvedbraiseonly2pnw: --raise-only
# ---------------------------------------------------------------------------------------------

def _ro_idx():
  """Anchor 0: a granted row. Anchor 1: the dates disagree (a side road pooled in) -> spread-refused. Anchor 2: only one date -> refused for another reason."""
  idx = _idx(2)
  b = idx.anchors[1]
  b.obs.clear()
  for n, (date, k, v) in enumerate((("2026-09-01", 0.0051, 21.0), ("2026-09-02", 0.0021, 25.0), ("2026-09-03", 0.0058, 12.0))):
    b.obs.append(rt.PassObs(date=date, drive=f"s{n}", car="FORD_F_150_LIGHTNING_MK1", t=float(n), k_ext=k, k_loc=k, way=0, hwy="motorway", road="I 5|",
                            spl=30.0, mode="op", d_m=1.0, end_lat=END[0] + 0.01, end_lon=END[1], v_ms=v))
  c = idx.anchors[idx.add(LAT0 + 0.02, LON0, 0.0)]
  c.obs.append(rt.PassObs(date="2026-09-01", drive="o", car="FORD_F_150_LIGHTNING_MK1", t=0.0, k_ext=0.004, k_loc=0.004, way=0, hwy="motorway",
                          road="I 5|", spl=30.0, mode="op", d_m=1.0, end_lat=END[0] + 0.02, end_lon=END[1], v_ms=20.0))
  return idx


def _ro_write(tmp_path, **kw):
  doc, counts = ex.export_doc(_ro_idx(), None, kw.pop("flags", None), **kw)
  src = tmp_path / "table.json.gz"
  src.write_bytes(b"x")
  ex.write(doc, counts, str(tmp_path), str(src))
  return doc, counts, json.loads((tmp_path / "manifest.json").read_text())


def test_without_the_option_the_export_is_byte_identical(tmp_path):
  doc1, c1 = ex.export_doc(_ro_idx())
  doc2, c2 = ex.export_doc(_ro_idx(), None, None, False)
  assert doc1 == doc2 and c1 == c2
  assert all(len(b) == 4 for a in doc1["anchors"] for b in a[3])           # the spread-refused branch stays k=None, no 5th element
  assert [b[2] for a in doc1["anchors"] for b in a[3]] == [0.002, None, None]
  a, b = tmp_path / "a", tmp_path / "b"
  a.mkdir()
  b.mkdir()
  _ro_write(a)
  _ro_write(b, raise_only=False)
  assert (a / cl.ROWS_NAME).read_bytes() == (b / cl.ROWS_NAME).read_bytes()
  m = json.loads((a / "manifest.json").read_text())
  assert "flags" not in m and "raise_only_rows" not in m


def test_with_the_option_the_spread_refused_branch_is_a_raise_only_row(tmp_path):
  doc, counts, man = _ro_write(tmp_path, raise_only=True)
  brs = [b for a in doc["anchors"] for b in a[3]]
  assert brs[0] == [brs[0][0], brs[0][1], 0.002, 3]                               # the granted row is unchanged (4 elements)
  assert brs[1][2] == pytest.approx(0.0058) and brs[1][3] == 3 and brs[1][4] == 2 and brs[1][5] == pytest.approx(25.0)   # k_worst, flag 2, fastest pass
  assert brs[2][2] is None and len(brs[2]) == 4                                  # refused for another reason: never raise-only
  assert counts["raise_only_rows"] == 1 and counts["rows_with_authority"] == 1   # authority count unchanged in meaning
  assert man["flags"] == "raiseOnly" and man["raise_only_rows"] == 1 and man["rows_with_authority"] == 1
  idx, _ = cl.load_rows(str(tmp_path))
  assert idx.n_rows == 1 and idx.n_raise_only == 1 and not idx.has_flags and idx.has_raise_only


def test_both_flags_declare_both(tmp_path):
  doc, counts, man = _ro_write(tmp_path, flags=[[LAT0, LON0, END[0], END[1]]], raise_only=True)
  assert man["flags"] == "lowerBound,raiseOnly" and man["lower_bound_rows"] == 1 and man["raise_only_rows"] == 1
  idx, _ = cl.load_rows(str(tmp_path))
  assert idx.has_flags and idx.has_raise_only and idx.n_lower_bound == 1 and idx.n_raise_only == 1 and idx.n_rows == 1


def test_a_branch_without_a_recorded_speed_is_counted_not_dropped():
  idx = _ro_idx()
  for o in idx.anchors[1].obs:
    object.__setattr__(o, "v_ms", float("nan"))
  _doc, counts = ex.export_doc(idx, None, None, True)
  assert counts["raise_only_rows"] == 0 and counts["raise_only_not_qualified"].get("no pass speed recorded") == 1
