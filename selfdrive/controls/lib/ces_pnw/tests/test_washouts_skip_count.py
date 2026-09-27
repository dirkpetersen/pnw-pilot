"""rule2fixes2pnw -- tools/washouts.py must COUNT the unparseable ces_events lines it skips (Rule 2).

It used to `continue` past a JSONDecodeError with no trace, so a half-corrupt pull produced a smaller registry
that looked complete. The lines are still skipped; now they are counted per file, warned about on stderr and
totalled in the summary line (with the number of files scanned).
"""
import json
import sys

from openpilot.tools import washouts as w

GOOD = [{"t": 100.0 + i, "vEgo": 30.0, "strPrs": True, "strAng": 5.0, "lat": 47.0, "lon": -122.0} for i in range(3)]


def _write(folder, lines):
  folder.mkdir(parents=True, exist_ok=True)
  (folder / "ces_events.jsonl").write_text("".join(line + "\n" for line in lines))


def test_skipped_lines_are_counted_and_warned(tmp_path, capsys):
  d = tmp_path / "2026-09-24" / "lightning-x"
  _write(d, [json.dumps(GOOD[0]), "{truncated", json.dumps(GOOD[1]), "", json.dumps(GOOD[2])])
  skipped = {}
  recs = w._load_folder_records(str(d), skipped)
  assert recs == GOOD                                       # fallback unchanged: good lines kept, bad ones skipped
  assert skipped == {str(d / "ces_events.jsonl"): 2}        # "{truncated" and the blank line
  err = capsys.readouterr().err
  assert "skipped 2 unparseable line(s)" in err and "ces_events.jsonl" in err


def test_clean_file_is_silent_and_counts_zero(tmp_path, capsys):
  d = tmp_path / "2026-09-24" / "lightning-x"
  _write(d, [json.dumps(r) for r in GOOD])
  skipped = {}
  assert w._load_folder_records(str(d), skipped) == GOOD
  assert skipped == {str(d / "ces_events.jsonl"): 0}
  assert capsys.readouterr().err == ""


def test_main_prints_the_total(tmp_path, capsys, monkeypatch):
  d = tmp_path / "2026-09-24" / "lightning-x"
  _write(d, [json.dumps(GOOD[0]), "garbage", json.dumps(GOOD[1])])
  out = tmp_path / "fixture.json"
  monkeypatch.setattr(sys, "argv", ["washouts.py", str(tmp_path), "--out", str(out)])
  w.main()
  cap = capsys.readouterr()
  assert "scanned 1 file(s), skipped 1 unparseable line(s)" in cap.out
  assert "1 unparseable line(s) skipped in total" in cap.err
  assert len(json.loads(out.read_text())["washouts"]) == 1


def test_discrete_record_does_not_displace_the_tick_at_the_same_timestamp(tmp_path):
  # curveshape2pnw: a {"ev":"mapdPath"} (or "steer"/"alert") record lands on the same 0.1 s grid as the tick
  # records. Deduped by `t` alone it replaced the tick at that instant, dropping a steering-override tick.
  d = tmp_path / "2026-09-26" / "lightning-x"
  ticks = [dict(r, t=100.0 + i / 10) for i, r in enumerate(GOOD)]
  lines = [json.dumps(ticks[0]), json.dumps(ticks[1]), json.dumps({"t": ticks[1]["t"], "ev": "mapdPath", "seq": 1}),
           json.dumps(dict(ticks[2], ev="tick"))]
  _write(d, lines)
  recs = w._load_folder_records(str(d))
  assert recs == [ticks[0], ticks[1], dict(ticks[2], ev="tick")]
  assert w._clusters(recs)[0]["n"] == 3
