"""cesarchive2pnw: the archive helpers net_events and the curve corpus use to hand finished generations to
the uploader. Never raise, never let the uploader see a half-written file, and say so on failure."""
import os
import pathlib

import pytest

from openpilot.common import pnw_log_archive as a

GOOD = 1789000000.0      # 2026-09-10T00:26:40Z
STAMP = "20260910T002640Z"


@pytest.fixture
def said(monkeypatch):
  out = {"error": [], "warning": []}
  monkeypatch.setattr(a.cloudlog, "error", lambda m, *x, **k: out["error"].append(m))
  monkeypatch.setattr(a.cloudlog, "warning", lambda m, *x, **k: out["warning"].append(m))
  return out


def _f(p, text="x\n", mtime=GOOD):
  p.write_text(text)
  os.utime(p, (mtime, mtime))
  return p


def test_the_name_is_the_mtime_stamp_and_a_dead_clock_gets_a_random_token(tmp_path):
  assert os.path.basename(a.archive_name(str(tmp_path), "b", GOOD)) == f"b.{STAMP}"
  bad = os.path.basename(a.archive_name(str(tmp_path), "b", 60.0))
  assert bad.startswith("b.19700101T000100Z.b") and len(bad) == len("b.19700101T000100Z.b") + 8


def test_link_is_a_hardlink_not_a_copy(tmp_path):
  src = _f(tmp_path / "log.1")
  dest = a.link_into_archive(str(src), str(tmp_path / "arc"), "log")
  assert os.path.basename(dest) == f"log.{STAMP}"
  assert os.stat(dest).st_ino == os.stat(src).st_ino


def test_a_failing_link_is_loud_and_returns_none(tmp_path, said, monkeypatch):
  monkeypatch.setattr(a.os, "link", lambda s, d: (_ for _ in ()).throw(OSError(18, "EXDEV")))
  assert a.link_into_archive(str(_f(tmp_path / "log.1")), str(tmp_path / "arc"), "log") is None
  assert any("hardlink FAILED" in s for s in said["error"])


def test_snapshot_is_a_copy_and_is_not_repeated_for_unchanged_content(tmp_path):
  src = _f(tmp_path / "obs.jsonl", "one\n")
  arc = tmp_path / "arc"
  d1 = a.snapshot_into_archive(str(src), str(arc), "obs.jsonl")
  assert os.path.basename(d1) == f"obs.jsonl.{STAMP}"
  assert os.stat(d1).st_ino != os.stat(src).st_ino, "a live, appended file must be COPIED"
  assert a.snapshot_into_archive(str(src), str(arc), "obs.jsonl") is None
  _f(src, "one\ntwo\n", mtime=GOOD + 3600)
  d2 = a.snapshot_into_archive(str(src), str(arc), "obs.jsonl")
  assert d2 != d1 and pathlib.Path(d2).read_text() == "one\ntwo\n" and pathlib.Path(d1).read_text() == "one\n"
  assert sorted(os.listdir(arc)) == sorted([os.path.basename(d1), os.path.basename(d2)]), "no temp left"


def test_snapshot_of_a_dead_clock_file_is_skipped_and_said(tmp_path, said):
  src = _f(tmp_path / "obs.jsonl", mtime=60.0)
  assert a.snapshot_into_archive(str(src), str(tmp_path / "arc"), "obs.jsonl") is None
  assert any("pre-2020" in s for s in said["warning"])


def test_a_missing_source_is_a_quiet_none(tmp_path, said):
  assert a.snapshot_into_archive(str(tmp_path / "nope"), str(tmp_path / "arc"), "nope") is None
  assert not said["error"]


def test_a_failing_snapshot_is_loud_and_leaves_no_partial_file(tmp_path, said, monkeypatch):
  def boom(s, d):
    pathlib.Path(d).write_text("partial")
    raise OSError(28, "No space left on device")
  monkeypatch.setattr(a.shutil, "copyfile", boom)
  src = _f(tmp_path / "obs.jsonl")
  arc = tmp_path / "arc"
  assert a.snapshot_into_archive(str(src), str(arc), "obs.jsonl") is None
  assert any("snapshot FAILED" in s for s in said["error"])
  assert os.listdir(arc) == []


def test_prune_deletes_oldest_first_only_within_the_prefix_and_is_loud(tmp_path, said):
  for i, mt in enumerate((GOOD, GOOD + 1, GOOD + 2)):
    _f(tmp_path / f"log.{i}", "x" * 100, mtime=mt)
  _f(tmp_path / "other", "y" * 1000, mtime=GOOD - 100)       # not ours: never counted, never deleted
  assert a.prune_archive(str(tmp_path), "log.", 250) == 1
  assert sorted(os.listdir(tmp_path)) == ["log.1", "log.2", "other"]
  assert any("OVER BUDGET" in s for s in said["error"])


def test_prune_of_a_missing_dir_is_zero(tmp_path, said):
  assert a.prune_archive(str(tmp_path / "nope"), "log.", 1) == 0
  assert not said["error"]
