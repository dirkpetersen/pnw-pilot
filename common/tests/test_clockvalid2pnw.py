"""clockvalid2pnw: one definition of "this wall-clock time can be real" -- time_helpers.wall_time_valid.

The 3X's RTC is dead. Before NTP/GPS sync after a boot the clock does NOT read 1970: systemd starts it at its
own build time, which on AGNOS 19.7 is 2026-07-28 ~15:05 UTC (08:05 PT). Every guard in the log/upload code
was "after 2020", so that date passed as real: a net_archive generation was named 20260728T150525Z, and a
metered upload made before sync would have been counted against "2026-07-28" -- a fresh day, one extra 50 MB.

The floor is upstream's own (common/time_helpers.min_date, the one timed publishes as clocks.valid): systemd's
mtime + 1 day. On the device, clocks.valid was False on every one of 160 pre-sync 07-28 messages in 16 boots
(central-oregon-weekend qlogs) and True after sync.
"""
import datetime
import math
import os

import pytest

from openpilot.common import time_helpers as th

UTC = datetime.UTC
# This dev host's /lib/systemd/systemd mtime (Ubuntu's systemd package), 2026-07-28 15:04:45 UTC -- 33 s
# before the device's pre-sync boots stamp initData (15:05:18). The device clock at boot IS systemd's build
# time, so this is the device's floor too, give or take the package build.
SYSTEMD_MTIME = datetime.datetime(2026, 7, 28, 15, 4, 45, tzinfo=UTC).timestamp()
DEVICE_FLOOR = SYSTEMD_MTIME + 86400
FAKE_PRESYNC = datetime.datetime(2026, 7, 28, 15, 5, 25, tzinfo=UTC).timestamp()   # the net_archive name
SYNCED_NOW = datetime.datetime(2026, 9, 23, 22, 20, tzinfo=UTC).timestamp()


def pin_device_floor(monkeypatch, floor=DEVICE_FLOOR):
  """Tests elsewhere use fixed dates; pin the floor so they do not depend on when THIS host's systemd was
  last upgraded (an October upgrade would otherwise make a September fixture read as pre-sync)."""
  monkeypatch.setattr(th, "_wall_time_floor", lambda: floor)


class TestTheFloor:
  """The real _wall_time_floor, reading a stand-in for /lib/systemd/systemd."""

  @pytest.fixture(autouse=True)
  def _fresh_cache(self):
    th._wall_time_floor.cache_clear()
    yield
    th._wall_time_floor.cache_clear()

  def _fake_systemd(self, monkeypatch, tmp_path, mtime):
    f = tmp_path / "systemd"
    f.write_text("")
    os.utime(f, (mtime, mtime))
    monkeypatch.setattr(th, "Path", lambda p: f if p == "/lib/systemd/systemd" else pytest.fail(p))

  def test_it_is_systemds_mtime_plus_one_day(self, monkeypatch, tmp_path):
    self._fake_systemd(monkeypatch, tmp_path, SYSTEMD_MTIME)
    assert th._wall_time_floor() == DEVICE_FLOOR
    assert not th.wall_time_valid(FAKE_PRESYNC), "the pre-sync 2026-07-28 must be rejected"
    assert th.wall_time_valid(SYNCED_NOW)

  def test_it_never_goes_below_upstreams_MIN_DATE(self, monkeypatch, tmp_path):
    self._fake_systemd(monkeypatch, tmp_path, 1.0e9)          # a 2001 systemd
    assert th._wall_time_floor() == th.MIN_DATE.timestamp()

  def test_an_unreadable_systemd_fails_CLOSED_and_says_so(self, monkeypatch):
    """min_date() falls back to MIN_DATE (2025-02-21) here, which would ACCEPT the pre-sync date."""
    import openpilot.common.swaglog as swaglog
    said = []
    monkeypatch.setattr(swaglog.cloudlog, "error", lambda m, *a, **k: said.append(m))
    monkeypatch.setattr(th, "Path", lambda p: _Missing())
    assert th._wall_time_floor() == math.inf
    assert not th.wall_time_valid(SYNCED_NOW), "nothing is trusted -- metered uploads stop"
    assert len(said) == 1 and "UNTRUSTED" in said[0], said
    th.wall_time_valid(SYNCED_NOW)
    assert len(said) == 1, "said ONCE per process, not per call"


class _Missing:
  def stat(self):
    raise FileNotFoundError(2, "No such file or directory")


class TestWallTimeValid:
  @pytest.fixture(autouse=True)
  def _pinned(self, monkeypatch):
    pin_device_floor(monkeypatch)

  @pytest.mark.parametrize("t", [0.0, 60.0, 1577836800.0,                  # 1970 (old dead-RTC), 2020
                                 1764094635.0,                             # 2025-11-25 (the 07-12 gap-analysis stamp)
                                 FAKE_PRESYNC, SYSTEMD_MTIME,
                                 DEVICE_FLOOR])                            # the floor itself is NOT valid (strict)
  def test_pre_sync_times_are_rejected(self, t):
    assert th.wall_time_valid(t) is False

  @pytest.mark.parametrize("t", [DEVICE_FLOOR + 1, SYNCED_NOW])
  def test_synced_times_are_accepted(self, t):
    assert th.wall_time_valid(t) is True

  def test_the_upstream_ceiling_applies(self):
    assert th.wall_time_valid(th.MAX_DATE.timestamp() - 1) is True
    assert th.wall_time_valid(th.MAX_DATE.timestamp()) is False

  @pytest.mark.parametrize("t", [None, "x", float("nan"), float("inf"), [], object()])
  def test_garbage_is_invalid_and_never_raises(self, t):
    assert th.wall_time_valid(t) is False

  def test_a_numeric_string_is_a_number(self):
    assert th.wall_time_valid(str(SYNCED_NOW)) is True
