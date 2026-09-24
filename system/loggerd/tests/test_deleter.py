import errno
import json
import time
import threading
from collections import namedtuple
from pathlib import Path
from collections.abc import Sequence

import pytest
import xattr

import openpilot.selfdrive.car.accdrop_pnw as accdrop_pnw
import openpilot.selfdrive.selfdrived.alertmanager as alertmanager
import openpilot.system.loggerd.deleter as deleter
import openpilot.system.loggerd.xattr_cache as xattr_cache
from openpilot.system.loggerd.uploader import UPLOAD_ATTR_NAME, UPLOAD_ATTR_VALUE
from openpilot.common.timeout import Timeout, TimeoutException
from openpilot.system.loggerd.tests.loggerd_tests_common import UploaderTestCase

Stats = namedtuple("Stats", ['f_bavail', 'f_blocks', 'f_frsize'])


class _Log:
  """cloudlog stand-in: records every message per level."""
  def __init__(self):
    self.calls: dict[str, list[str]] = {}

  def __getattr__(self, level):
    return lambda msg, *a, **k: self.calls.setdefault(level, []).append(str(msg))

  def of(self, level):
    return self.calls.get(level, [])


class TestDeleter(UploaderTestCase):
  def fake_statvfs(self, d):
    return self.fake_stats

  @pytest.fixture(autouse=True)
  def _ces_log(self, tmp_path, monkeypatch):
    # every test (not only the loss ones): a deleted un-uploaded segment appends a deleterLoss record, and the
    # real CES_EVENT_LOG is the live /data/pnw telemetry log. The offroad alert goes through Params, which the
    # root conftest already isolates under OPENPILOT_PREFIX.
    self.ces_log = tmp_path / "ces_events.jsonl"
    monkeypatch.setattr(accdrop_pnw, "CES_EVENT_LOG", str(self.ces_log))

  def setup_method(self):
    self.f_type = "fcamera.hevc"
    super().setup_method()
    self.fake_stats = Stats(f_bavail=0, f_blocks=10, f_frsize=4096)
    deleter.os.statvfs = self.fake_statvfs

  def start_thread(self):
    self.end_event = threading.Event()
    self.del_thread = threading.Thread(target=deleter.deleter_thread, args=[self.end_event])
    self.del_thread.daemon = True
    self.del_thread.start()

  def join_thread(self):
    self.end_event.set()
    self.del_thread.join()

  def test_delete(self):
    f_path = self.make_file_with_data(self.seg_dir, self.f_type, 1)

    self.start_thread()

    try:
      with Timeout(2, "Timeout waiting for file to be deleted"):
        while f_path.exists():
          time.sleep(0.01)
    finally:
      self.join_thread()

  def assertDeleteOrder(self, f_paths: Sequence[Path], timeout: int = 5) -> None:
    deleted_order = []

    self.start_thread()
    try:
      with Timeout(timeout, "Timeout waiting for files to be deleted"):
        while True:
          for f in f_paths:
            if not f.exists() and f not in deleted_order:
              deleted_order.append(f)
          if len(deleted_order) == len(f_paths):
            break
          time.sleep(0.01)
    except TimeoutException:
      print("Not deleted:", [f for f in f_paths if f not in deleted_order])
      raise
    finally:
      self.join_thread()

    assert deleted_order == f_paths, "Files not deleted in expected order"

  def test_delete_order(self):
    self.assertDeleteOrder([
      self.make_file_with_data(self.seg_format.format(0), self.f_type),
      self.make_file_with_data(self.seg_format.format(1), self.f_type),
      self.make_file_with_data(self.seg_format2.format(0), self.f_type),
    ])

  def test_delete_many_preserved(self):
    self.assertDeleteOrder([
      self.make_file_with_data(self.seg_format.format(0), self.f_type),
      self.make_file_with_data(self.seg_format.format(1), self.f_type, preserve_xattr=deleter.PRESERVE_ATTR_VALUE),
      self.make_file_with_data(self.seg_format.format(2), self.f_type),
    ] + [
      self.make_file_with_data(self.seg_format2.format(i), self.f_type, preserve_xattr=deleter.PRESERVE_ATTR_VALUE)
      for i in range(5)
    ])

  def test_delete_last(self):
    self.assertDeleteOrder([
      self.make_file_with_data(self.seg_format.format(1), self.f_type),
      self.make_file_with_data(self.seg_format2.format(0), self.f_type),
      self.make_file_with_data(self.seg_format.format(0), self.f_type, preserve_xattr=deleter.PRESERVE_ATTR_VALUE),
      self.make_file_with_data("boot", self.seg_format[:-4]),
      self.make_file_with_data("crash", self.seg_format2[:-4]),
    ])

  def test_no_delete_when_available_space(self):
    f_path = self.make_file_with_data(self.seg_dir, self.f_type)

    block_size = 4096
    available = (10 * 1024 * 1024 * 1024) / block_size  # 10GB free
    self.fake_stats = Stats(f_bavail=available, f_blocks=10, f_frsize=block_size)

    self.start_thread()
    start_time = time.monotonic()
    while f_path.exists() and time.monotonic() - start_time < 2:
      time.sleep(0.01)
    self.join_thread()

    assert f_path.exists(), "File deleted with available space"

  def test_no_delete_with_lock_file(self):
    f_path = self.make_file_with_data(self.seg_dir, self.f_type, lock=True)

    self.start_thread()
    start_time = time.monotonic()
    while f_path.exists() and time.monotonic() - start_time < 2:
      time.sleep(0.01)
    self.join_thread()

    assert f_path.exists(), "File deleted when locked"

  # deleterloss2pnw: upload state is read UNCACHED, and destroying un-uploaded data is VISIBLE
  # (offroad alert + ces_events record), never only a swaglog line.
  @pytest.fixture
  def loss(self, monkeypatch):
    self.log = _Log()
    monkeypatch.setattr(deleter, "cloudlog", self.log)

  def run_until_deleted(self, paths):
    self.start_thread()
    try:
      with Timeout(5, "Timeout waiting for files to be deleted"):
        while any(p.exists() for p in paths):
          time.sleep(0.01)
    finally:
      self.join_thread()

  def records(self):
    if not self.ces_log.exists():
      return []
    return [json.loads(line) for line in self.ces_log.read_text().splitlines()]

  def alert(self):
    return self.params.get(deleter.LOSS_ALERT)

  def test_uploaded_since_start_is_not_stale(self, loss):
    # The uploader sets user.upload in ANOTHER process: this process's xattr_cache never hears about it.
    f = self.make_file_with_data(self.seg_dir, self.f_type)
    assert deleter.has_unuploaded_firehose(self.seg_dir)
    assert xattr_cache.getxattr(str(f), UPLOAD_ATTR_NAME) is None   # the memo a cached read would keep
    xattr.setxattr(str(f), UPLOAD_ATTR_NAME, UPLOAD_ATTR_VALUE)       # raw: bypasses this process's cache
    assert not deleter.has_unuploaded_firehose(self.seg_dir), "stale cached upload state"

  def test_unuploaded_deletion_raises_alert_and_record(self, loss):
    f = self.make_file_with_data(self.seg_dir, self.f_type)
    size = f.stat().st_size
    self.run_until_deleted([f])
    a = self.alert()
    assert a is not None and "1 segments" in a["extra"]
    recs = self.records()
    assert len(recs) == 1
    r = recs[0]
    assert (r["ev"], r["seg"], r["files"], r["bytes"], r["n_boot"]) == ("deleterLoss", self.seg_dir, [self.f_type], size, 1)
    assert any("UN-UPLOADED" in m for m in self.log.of("error"))

  def test_skip_listed_only_does_not_alert(self, loss):
    self.params.put_bool("SkipWideCameraUpload", True)
    up = self.make_file_with_data(self.seg_dir, "fcamera.hevc", upload_xattr=UPLOAD_ATTR_VALUE)
    wide = self.make_file_with_data(self.seg_dir, "ecamera.hevc")
    self.run_until_deleted([up, wide])
    assert self.alert() is None
    assert self.records() == []
    # the full-set swaglog alarm is unchanged: the wide video still never reached the backend
    assert any("UN-UPLOADED" in m for m in self.log.of("error"))

  def test_fully_uploaded_does_not_alert(self, loss):
    f = self.make_file_with_data(self.seg_dir, self.f_type, upload_xattr=UPLOAD_ATTR_VALUE)
    self.run_until_deleted([f])
    assert self.alert() is None
    assert self.records() == []

  def test_alert_and_record_failures_do_not_stop_deletion(self, monkeypatch, loss):
    def boom(*a, **k):
      raise RuntimeError("boom")
    monkeypatch.setattr(alertmanager, "set_offroad_alert", boom)
    monkeypatch.setattr(accdrop_pnw, "append_to_ces_log", boom)
    fs = [self.make_file_with_data(self.seg_format.format(i), self.f_type) for i in range(2)]
    self.run_until_deleted(fs)
    msgs = self.log.of("exception")
    assert sum("FAILED to raise" in m for m in msgs) == 2
    assert sum("deleterLoss record" in m for m in msgs) == 2

  def test_upload_read_error_is_logged_once_per_sweep(self, monkeypatch, loss):
    def eio(*a, **k):
      raise OSError(errno.EIO, "io error")
    monkeypatch.setattr(deleter, "getxattr_uncached", eio)
    f = self.make_file_with_data(self.seg_dir, self.f_type)
    self.run_until_deleted([f])
    errs = [m for m in self.log.of("error") if "read error" in m]
    assert len(errs) == 1 and "counted as uploaded" in errs[0]
    assert self.records() == []   # a failed read counts as uploaded (never blocks freeing space)
