"""cesarchive2pnw: net_events.jsonl generations reach the upload archive.

Before: the NetLogger rotated only at 10 MB (~25 days at the measured ~0.4 MB/day) into a single .1
that the next rotation overwrote -- nothing was ever archived or uploaded. Now it also rotates at each
UTC day boundary and hardlinks every rotated generation into NET_ARCHIVE_DIR.
"""
import os
import pathlib

import pytest

from openpilot.system.location_services import location_servicesd as ls
from openpilot.common import pnw_log_archive
from openpilot.common.tests.test_clockvalid2pnw import FAKE_PRESYNC, pin_device_floor

DAY1 = 1789000000.0          # 2026-09-10T00:26:40Z
DAY2 = DAY1 + 86400


@pytest.fixture(autouse=True)
def _device_floor(monkeypatch):
  pin_device_floor(monkeypatch)


@pytest.fixture
def said(monkeypatch):
  out = []
  monkeypatch.setattr(ls.cloudlog, "error", lambda m, *x, **k: out.append(m))
  monkeypatch.setattr(pnw_log_archive.cloudlog, "error", lambda m, *x, **k: out.append(m))
  return out


def _live(tmp_path, text="rec\n", mtime=DAY1):
  p = tmp_path / "net_events.jsonl"
  p.write_text(text)
  os.utime(p, (mtime, mtime))
  return p


def test_same_day_small_file_does_not_rotate(tmp_path):
  live = _live(tmp_path)
  assert ls.rotate_net_log(str(live), DAY1 + 60, str(tmp_path / "arc")) is False
  assert live.exists()


def test_a_new_utc_day_rotates_and_archives_by_hardlink(tmp_path):
  live = _live(tmp_path)
  arc = tmp_path / "arc"
  assert ls.rotate_net_log(str(live), DAY2, str(arc)) is True
  gen1 = tmp_path / "net_events.jsonl.1"
  (name,) = os.listdir(arc)
  assert name == "net_events.jsonl.20260910T002640Z"
  assert os.stat(arc / name).st_ino == os.stat(gen1).st_ino
  assert not live.exists()


def test_the_size_cap_still_rotates_within_a_day(tmp_path, monkeypatch):
  monkeypatch.setattr(ls, "NET_EVENT_LOG_MAX_BYTES", 10)
  live = _live(tmp_path, "x" * 11)
  assert ls.rotate_net_log(str(live), DAY1 + 60, str(tmp_path / "arc")) is True


def test_a_dead_clock_never_triggers_the_day_rotation(tmp_path):
  live = _live(tmp_path, mtime=60.0)
  assert ls.rotate_net_log(str(live), DAY2, str(tmp_path / "arc")) is False


def test_a_pre_sync_2026_07_28_mtime_never_triggers_the_day_rotation(tmp_path):
  """clockvalid2pnw: the unsynced 3X reads 2026-07-28, not 1970; against a synced "now" that looked
  like an old day, so the first write after sync rotated -- and named the generation 20260728T150525Z."""
  live = _live(tmp_path, mtime=FAKE_PRESYNC)
  assert ls.rotate_net_log(str(live), DAY2, str(tmp_path / "arc")) is False
  assert ls.rotate_net_log(str(live), FAKE_PRESYNC + 86400, str(tmp_path / "arc")) is False


def test_a_pre_sync_generation_that_does_rotate_is_named_randomly(tmp_path, monkeypatch):
  monkeypatch.setattr(ls, "NET_EVENT_LOG_MAX_BYTES", 10)     # the size cap still applies before sync
  live = _live(tmp_path, "x" * 11, mtime=FAKE_PRESYNC)
  arc = tmp_path / "arc"
  assert ls.rotate_net_log(str(live), FAKE_PRESYNC + 60, str(arc)) is True
  (name,) = os.listdir(arc)
  assert name.startswith("net_events.jsonl.20260728T150525Z.b"), name


def test_a_legacy_unarchived_generation_1_is_rescued_before_it_is_overwritten(tmp_path):
  """The device has a 10.5 MB net_events.jsonl.1 from 2026-09-05 that was never archived."""
  old = tmp_path / "net_events.jsonl.1"
  old.write_text("legacy\n")
  os.utime(old, (DAY1 - 5 * 86400, DAY1 - 5 * 86400))
  live = _live(tmp_path)
  arc = tmp_path / "arc"
  ls.rotate_net_log(str(live), DAY2, str(arc))
  assert sorted(pathlib.Path(arc / n).read_text() for n in os.listdir(arc)) == ["legacy\n", "rec\n"]


def test_an_already_archived_generation_1_is_not_archived_twice(tmp_path):
  arc = tmp_path / "arc"
  live = _live(tmp_path, "d1\n")
  ls.rotate_net_log(str(live), DAY2, str(arc))
  _live(tmp_path, "d2\n", mtime=DAY2)
  ls.rotate_net_log(str(live), DAY2 + 86400, str(arc))
  assert sorted(pathlib.Path(arc / n).read_text() for n in os.listdir(arc)) == ["d1\n", "d2\n"]


def test_a_failing_link_is_loud_and_the_log_still_rotates(tmp_path, said, monkeypatch):
  monkeypatch.setattr(pnw_log_archive.os, "link", lambda s, d: (_ for _ in ()).throw(OSError(18, "EXDEV")))
  live = _live(tmp_path)
  assert ls.rotate_net_log(str(live), DAY2, str(tmp_path / "arc")) is True
  assert (tmp_path / "net_events.jsonl.1").exists()
  assert any("hardlink FAILED" in s for s in said)


def test_a_failing_rotation_is_loud_and_never_raises(tmp_path, said, monkeypatch):
  monkeypatch.setattr(ls.os, "replace", lambda s, d: (_ for _ in ()).throw(PermissionError(13, "EACCES")))
  live = _live(tmp_path)
  assert ls.rotate_net_log(str(live), DAY2, str(tmp_path / "arc")) is False
  assert any("rotation FAILED" in s for s in said)


def test_the_archive_dir_is_outside_location_where_the_waze_key_lives():
  assert ls.NET_ARCHIVE_DIR == "/data/pnw/net_archive"
  assert not ls.NET_ARCHIVE_DIR.startswith("/data/pnw/location")
