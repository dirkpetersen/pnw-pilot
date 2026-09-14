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
from openpilot.common.swaglog import SwaglogRotatingFileHandler


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
  h.backup_count = 2500
  h._warn_if_young(str(tmp_path / "swaglog.0000000001"))   # never existed
  assert h.warned_young_delete is False
