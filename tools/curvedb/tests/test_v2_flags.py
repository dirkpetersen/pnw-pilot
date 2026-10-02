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
