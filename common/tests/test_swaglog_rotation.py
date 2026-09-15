"""swaglogrot2pnw: swaglog rotation must delete the OLDEST logs, across restarts.

Before the fix, SwaglogRotatingFileHandler listed the existing files oldest-first, put new files at the front and
deleted from the back, so every restart deleted the newest files of earlier boots first. On the device that left a
frozen block of 08-18..09-04 logs and nothing from 09-05..09-12. Each _boot() below is what logmessaged does at every
start: a fresh handler on the same directory.
"""
import os
import time

import pytest

from openpilot.common import swaglog
from openpilot.common.logging_extra import SwagLogFileFormatter
from openpilot.common.swaglog import SwaglogRotatingFileHandler
from openpilot.system.hardware.hw import Paths


@pytest.fixture
def errors(monkeypatch):
  calls = []
  monkeypatch.setattr(swaglog.cloudlog, "error", lambda msg, *a, **k: calls.append(msg))
  return calls


def _indexes(d):
  return sorted(int(fn.split(".")[-1]) for fn in os.listdir(d))


def _boot(d, backup_count, n_files):
  # interval=0 and max_bytes=0 turn off automatic rollover; __init__ opens the first file
  h = SwaglogRotatingFileHandler(os.path.join(d, "swaglog"), interval=0, max_bytes=0, backup_count=backup_count)
  for _ in range(n_files - 1):
    h.doRollover()
  h.close()


def test_restart_keeps_newest(tmp_path, errors):
  _boot(tmp_path, 10, 10)  # 0..9
  _boot(tmp_path, 10, 3)   # 10..12
  _boot(tmp_path, 10, 2)   # 13..14
  # the buggy order kept [0..6, 10, 13, 14]
  assert _indexes(tmp_path) == list(range(5, 15))


def test_many_short_boots_keep_exactly_the_newest(tmp_path, errors):
  # the device on 2026-09-14 rebooted every ~6 min; each boot writes a few files
  sizes = [30, 1, 7, 2, 5, 3, 1, 6, 4, 2, 7, 1, 1, 5, 3, 2, 6, 4, 1, 7]
  for n in sizes:
    _boot(tmp_path, 25, n)
  total = sum(sizes)
  assert _indexes(tmp_path) == list(range(total - 25, total))


def test_long_boot_after_short_boots(tmp_path, errors):
  _boot(tmp_path, 10, 4)
  _boot(tmp_path, 10, 4)
  _boot(tmp_path, 10, 25)  # 8..32
  assert _indexes(tmp_path) == list(range(23, 33))


def test_index_restarts_below_previous_max(tmp_path, errors):
  # the index is max(existing) + 1, so it drops back if the newest files were removed by something else
  _boot(tmp_path, 10, 10)  # 0..9
  for i in (7, 8, 9):
    os.remove(os.path.join(tmp_path, f"swaglog.{i:010}"))
  _boot(tmp_path, 10, 5)   # 7..11 again
  assert _indexes(tmp_path) == list(range(2, 12))


def _age_all(d, age_s):
  t = time.time() - age_s  # noqa: TID251 -- file mtimes are wall-clock time
  for fn in os.listdir(d):
    os.utime(os.path.join(d, fn), (t, t))


def test_deleting_young_file_warns_once(tmp_path, errors):
  _boot(tmp_path, 5, 5)
  _age_all(tmp_path, 3600)
  _boot(tmp_path, 5, 4)  # deletes four 1-hour-old files
  assert len(errors) == 1
  assert "swaglog.0000000000" in errors[0] and "1.0 h old" in errors[0]
  assert "file cap" in errors[0] and "byte cap" not in errors[0]


def test_deleting_old_file_is_quiet(tmp_path, errors):
  _boot(tmp_path, 5, 5)
  _age_all(tmp_path, 3 * 24 * 3600)
  _boot(tmp_path, 5, 4)
  assert errors == []
  assert _indexes(tmp_path) == list(range(4, 9))


def test_file_newer_than_clock_is_not_called_young(tmp_path, errors):
  # a boot before time sync: the clock is behind the files, so their age is unknown
  _boot(tmp_path, 5, 5)
  _age_all(tmp_path, -2 * 24 * 3600)
  _boot(tmp_path, 5, 2)
  assert errors == []


def test_warning_checks_later_deletions_after_unknown_age(tmp_path, errors):
  _boot(tmp_path, 3, 3)  # 0..2
  _age_all(tmp_path, 60)
  future = time.time() + 86400  # noqa: TID251 -- file mtimes are wall-clock time
  os.utime(os.path.join(tmp_path, "swaglog.0000000000"), (future, future))
  _boot(tmp_path, 3, 2)  # deletes 0 (age unknown, quiet), then 1 (1 min old, warns)
  assert len(errors) == 1 and "swaglog.0000000001" in errors[0]


def test_warn_if_young_survives_a_file_removed_underneath_it(tmp_path):
  """Fable: getmtime can race a concurrent os.remove; the check must return quietly, never raise."""
  from openpilot.common.swaglog import SwaglogRotatingFileHandler
  h = SwaglogRotatingFileHandler.__new__(SwaglogRotatingFileHandler)
  h.warned_young_delete = False
  h._warn_if_young(str(tmp_path / "swaglog.0000000001"), "file cap")   # never existed
  assert h.warned_young_delete is False


# ---- swaglogcap2pnw: 20,000 files and a 200 MiB ceiling on the closed files, whichever is reached first ----

def _make_files(d, sizes, age_s=3 * 24 * 3600):
  """swaglog.<i> for each size. Sparse (truncate), so a large st_size costs no disk. Old by default, so quiet."""
  t = time.time() - age_s  # noqa: TID251 -- file mtimes are wall-clock time
  for i, size in enumerate(sizes):
    fp = os.path.join(d, f"swaglog.{i:010}")
    with open(fp, "w") as f:
      f.truncate(size)
    os.utime(fp, (t, t))


def _handler(d, backup_count, max_total_bytes):
  return SwaglogRotatingFileHandler(os.path.join(d, "swaglog"), interval=0, max_bytes=0, backup_count=backup_count,
                                    max_total_bytes=max_total_bytes)


def _default_handler(d, monkeypatch):
  monkeypatch.setattr(Paths, "swaglog_root", staticmethod(lambda: str(d)))
  return swaglog.get_file_handler()


def _assert_total_consistent(h):
  """The running total matches the closed files on disk, and the handler tracks exactly what is on disk."""
  d = os.path.dirname(h.base_filename)
  closed = h.log_files[1:]
  assert set(h.log_sizes) == set(closed)
  assert h.total_bytes == sum(h.log_sizes.values()) == sum(os.path.getsize(p) for p in closed)
  assert sorted(os.path.join(d, fn) for fn in os.listdir(d)) == sorted(h.log_files)


def test_default_caps_are_20000_files_and_200_mib(tmp_path, monkeypatch, errors):
  h = _default_handler(tmp_path, monkeypatch)
  h.close()
  assert (h.backup_count, h.max_total_bytes) == (20000, 200 * 1024 * 1024)


def test_25k_files_trimmed_to_20000(tmp_path, monkeypatch, errors):
  _make_files(tmp_path, [8500] * 25000)  # 212.5 MB, over the byte cap as well, but the file cap is reached first
  h = _default_handler(tmp_path, monkeypatch)
  h.close()
  assert _indexes(tmp_path) == list(range(5001, 25001))  # 25000 is the file the handler opened
  assert h.total_bytes == 19999 * 8500
  assert errors == []


def test_init_with_large_directory_trims_by_bytes(tmp_path, monkeypatch, errors):
  _make_files(tmp_path, [100_000] * 3000)  # 300 MB of closed files, far under the file cap
  h = _default_handler(tmp_path, monkeypatch)
  keep = (200 * 1024 * 1024) // 100_000  # 2097 closed files fit
  assert _indexes(tmp_path) == list(range(3000 - keep, 3001))
  assert h.total_bytes == keep * 100_000
  _assert_total_consistent(h)
  h.close()
  assert errors == []


def test_byte_cap_deletes_oldest_first_even_when_the_newest_is_big(tmp_path, errors):
  _make_files(tmp_path, [100, 100, 100, 100, 100, 900])  # 1400 bytes, and the newest file is the big one
  h = _handler(tmp_path, 100, 1000)
  h.close()
  assert _indexes(tmp_path) == [4, 5, 6]  # deleting 0..3 leaves exactly 1000, which is not over the cap
  assert h.total_bytes == 1000


def test_byte_cap_stops_as_soon_as_under(tmp_path, errors):
  _make_files(tmp_path, [5000] + [100] * 9)  # the oldest file alone is over the cap
  h = _handler(tmp_path, 100, 1000)
  h.close()
  assert _indexes(tmp_path) == list(range(1, 11))
  assert h.total_bytes == 900


@pytest.mark.parametrize("backup_count,max_total_bytes,kept", [(0, 0, 11), (0, 500, 6), (3, 0, 3)])
def test_zero_turns_a_limit_off(tmp_path, errors, backup_count, max_total_bytes, kept):
  _make_files(tmp_path, [100] * 10)
  h = _handler(tmp_path, backup_count, max_total_bytes)
  h.close()
  assert _indexes(tmp_path) == list(range(11 - kept, 11))


def test_byte_cap_on_rollover(tmp_path, errors):
  h = _handler(tmp_path, 100, 2500)
  for _ in range(6):
    h.stream.write("x" * 1000)
    h.doRollover()
    _assert_total_consistent(h)
  h.close()
  assert _indexes(tmp_path) == [4, 5, 6]  # 6 closed files of 1000 bytes, and two fit under 2500
  assert h.total_bytes == 2000


def test_running_total_across_boots_rollovers_and_deletes(tmp_path, errors):
  sizes = [300, 7000, 50, 2500, 0, 1200, 900, 4096, 10, 3333, 800, 1500]
  n = 0
  for _boot_nr in range(4):
    h = _handler(tmp_path, 6, 9000)
    _assert_total_consistent(h)
    for _ in range(5):
      h.stream.write("y" * sizes[n % len(sizes)])
      n += 1
      h.doRollover()
      _assert_total_consistent(h)
      assert h.total_bytes <= 9000 and len(h.log_files) <= 6
    h.close()
  # both limits did the deleting (the files are young, so each boot warns once, naming its limit), and nothing failed
  assert any("byte cap" in e for e in errors) and any("file cap" in e for e in errors)
  assert not any("could not" in e for e in errors)


def test_file_removed_underneath_before_rotation_deletes_it(tmp_path, errors):
  """The concurrent-delete path (the acb5a21fb7 guard): no raise, the failed delete is logged, the total stays right."""
  h = _handler(tmp_path, 4, 10**6)
  for _ in range(3):
    h.stream.write("z" * 100)
    h.doRollover()
  os.remove(h.log_files[-1])  # swaglog.0000000000, the next one rotation deletes
  h.stream.write("z" * 100)
  h.doRollover()
  assert len(errors) == 1 and "could not delete swaglog.0000000000" in errors[0] and "FileNotFoundError" in errors[0]
  _assert_total_consistent(h)
  assert h.total_bytes == 300
  h.close()


def test_closed_file_size_read_error_is_logged(tmp_path, errors):
  h = _handler(tmp_path, 100, 10**6)
  h.stream.write("a" * 100)
  os.remove(h.log_files[0])  # the open file vanishes before the rollover measures it
  h.doRollover()
  h.close()
  assert len(errors) == 1 and "could not read the size of swaglog.0000000000" in errors[0]
  assert h.total_bytes == 0 and h.log_sizes == {}


def test_init_scan_error_is_logged(tmp_path, monkeypatch, errors):
  _make_files(tmp_path, [100] * 3)
  real_stat = os.stat
  def flaky_stat(p, *a, **k):
    if str(p).endswith("swaglog.0000000001"):
      raise PermissionError(13, "Permission denied")
    return real_stat(p, *a, **k)
  monkeypatch.setattr(swaglog.os, "stat", flaky_stat)
  h = _handler(tmp_path, 100, 10**6)
  monkeypatch.setattr(swaglog.os, "stat", real_stat)
  h.close()
  assert len(errors) == 1 and "could not read 1 log file" in errors[0] and "swaglog.0000000001" in errors[0]
  assert h.total_bytes == 200


def test_stray_directory_is_not_a_log_file(tmp_path, errors):
  _make_files(tmp_path, [100] * 3)
  os.mkdir(os.path.join(tmp_path, "swaglog.0000000099"))
  h = _handler(tmp_path, 2, 10**6)
  h.close()
  assert h.total_bytes == 100 and len(h.log_files) == 2
  assert errors == []


def test_young_file_warning_names_the_byte_cap(tmp_path, errors):
  _make_files(tmp_path, [1000] * 5, age_s=3600)
  h = _handler(tmp_path, 100, 2500)
  h.close()
  assert _indexes(tmp_path) == [3, 4, 5]
  assert len(errors) == 1
  assert "swaglog.0000000000" in errors[0] and "byte cap" in errors[0] and "file cap" not in errors[0]


def test_young_file_warning_names_the_file_cap_when_both_are_over(tmp_path, errors):
  # both limits are exceeded; the file cap is checked first, so it is the one the single warning names
  _make_files(tmp_path, [1000] * 10, age_s=3600)
  h = _handler(tmp_path, 4, 2500)
  h.close()
  assert _indexes(tmp_path) == [8, 9, 10]  # down to 4 files by the file cap, then one more by the byte cap
  assert len(errors) == 1
  assert "file cap" in errors[0] and "byte cap" not in errors[0]


def test_rotation_error_is_written_when_the_handler_is_on_cloudlog(tmp_path):
  """manager.py's crash path attaches the handler to cloudlog itself: an error logged mid-rollover must reach the new file."""
  h = _handler(tmp_path, 100, 10**6)
  h.setFormatter(SwagLogFileFormatter(swaglog.cloudlog))
  swaglog.cloudlog.addHandler(h)
  try:
    os.remove(h.log_files[0])
    h.doRollover()
  finally:
    swaglog.cloudlog.removeHandler(h)
    h.close()
  with open(h.log_files[0]) as f:
    assert "could not read the size of swaglog.0000000000" in f.read()
