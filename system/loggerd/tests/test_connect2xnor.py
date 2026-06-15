#!/usr/bin/env python3
# connect2xnor: tests for the two-pass uploader gate and the deleter
# preferential-preservation policy for un-uploaded large (pass-2) files.
import threading
import time
from collections import namedtuple

from cereal import log

import openpilot.system.loggerd.deleter as deleter
import openpilot.system.loggerd.uploader as uploader
from openpilot.common.timeout import Timeout
from openpilot.system.hardware.hw import Paths
from openpilot.system.loggerd.uploader import (
  FIREHOSE_FILES,
  UPLOAD_ATTR_VALUE,
  pass2_allowed,
)
from openpilot.system.loggerd.tests.loggerd_tests_common import UploaderTestCase

NetworkType = log.DeviceState.NetworkType


class TestPass2Gate:
  """"Should pass-2 run now?" -- strictly WiFi-only."""

  def test_wifi_allowed(self):
    assert pass2_allowed(int(NetworkType.wifi)) is True

  def test_cell_blocked(self):
    # LTE / hotspot (never-default -> reports cell) must never trigger pass-2.
    for nt in (NetworkType.cell2G, NetworkType.cell3G, NetworkType.cell4G, NetworkType.cell5G):
      assert pass2_allowed(int(nt)) is False, f"{nt} should be blocked"

  def test_none_blocked(self):
    assert pass2_allowed(int(NetworkType.none)) is False

  def test_ethernet_blocked(self):
    # Only wifi qualifies by design (the gate is "real external WiFi").
    assert pass2_allowed(int(NetworkType.ethernet)) is False


class TestPass2Selection(UploaderTestCase):
  """Pass-1 and pass-2 partition the files: small vs. large (firehose)."""

  def setup_method(self):
    super().setup_method()
    self.up = uploader.Uploader("0000000000000000", Paths.log_root())

  def _gen(self):
    for t in ["qlog", "qcamera.ts", "rlog", "fcamera.hevc", "ecamera.hevc"]:
      self.make_file_with_data(self.seg_dir, t, 1)

  def test_pass1_excludes_firehose(self):
    self._gen()
    names = {name for name, _, _ in self.up.list_upload_files(metered=False, pass2=False)}
    assert names.isdisjoint(FIREHOSE_FILES), f"pass-1 leaked firehose files: {names & FIREHOSE_FILES}"
    assert "qlog" in names and "qcamera.ts" in names

  def test_pass2_only_firehose(self):
    self._gen()
    names = {name for name, _, _ in self.up.list_upload_files(metered=False, pass2=True)}
    assert names <= FIREHOSE_FILES, f"pass-2 leaked non-firehose files: {names - FIREHOSE_FILES}"
    assert {"rlog", "fcamera.hevc", "ecamera.hevc"} <= names

  def test_pass2_skips_already_uploaded(self):
    self.make_file_with_data(self.seg_dir, "rlog", 1, upload_xattr=UPLOAD_ATTR_VALUE)
    self.make_file_with_data(self.seg_dir, "fcamera.hevc", 1)
    names = {name for name, _, _ in self.up.list_upload_files(metered=False, pass2=True)}
    assert "rlog" not in names, "already-uploaded rlog should be skipped"
    assert "fcamera.hevc" in names

  def test_next_pass2_returns_none_when_empty(self):
    # only small files present -> pass-2 has nothing to do
    self.make_file_with_data(self.seg_dir, "qlog", 1)
    assert self.up.next_pass2_file_to_upload(metered=False) is None


class TestDeleterPreserveUnuploaded(UploaderTestCase):
  """Deleter keeps segments with un-uploaded large files until last resort."""

  def fake_statvfs(self, d):
    return self.fake_stats

  def setup_method(self):
    self.f_type = "fcamera.hevc"
    super().setup_method()
    # force "out of space" so the deleter always wants to delete something
    Stats = namedtuple("Stats", ["f_bavail", "f_blocks", "f_frsize"])
    self.fake_stats = Stats(f_bavail=0, f_blocks=10, f_frsize=4096)
    deleter.os.statvfs = self.fake_statvfs

  def _make_seg(self, seg, uploaded: bool):
    xattr = UPLOAD_ATTR_VALUE if uploaded else None
    # each segment gets a large pass-2 file (fcamera.hevc) + a small qlog
    self.make_file_with_data(seg, "fcamera.hevc", 1, upload_xattr=xattr)
    return self.make_file_with_data(seg, "qlog", 1, upload_xattr=UPLOAD_ATTR_VALUE)

  def test_has_unuploaded_firehose(self):
    self._make_seg(self.seg_format.format(0), uploaded=False)
    self._make_seg(self.seg_format.format(1), uploaded=True)
    assert deleter.has_unuploaded_firehose(self.seg_format.format(0)) is True
    assert deleter.has_unuploaded_firehose(self.seg_format.format(1)) is False

  def _run_until_deleted(self, paths, timeout=5):
    deleted_order = []
    end_event = threading.Event()
    th = threading.Thread(target=deleter.deleter_thread, args=[end_event])
    th.daemon = True
    th.start()
    try:
      with Timeout(timeout, "Timeout waiting for deletes"):
        while len(deleted_order) < len(paths):
          for p in paths:
            if not p.exists() and p not in deleted_order:
              deleted_order.append(p)
          time.sleep(0.01)
    finally:
      end_event.set()
      th.join()
    return deleted_order

  def test_uploaded_deleted_before_unuploaded(self):
    # oldest segment is UN-uploaded, newer segment is fully uploaded.
    # Policy: the uploaded one must be deleted first despite being newer.
    seg_old_unuploaded = self.seg_format.format(0)
    seg_new_uploaded = self.seg_format.format(1)
    p_old = self._make_seg(seg_old_unuploaded, uploaded=False)
    p_new = self._make_seg(seg_new_uploaded, uploaded=True)

    order = self._run_until_deleted([p_old, p_new])
    assert order == [p_new, p_old], f"expected uploaded-first delete order, got {order}"

  def test_last_resort_deletes_unuploaded(self):
    # EVERYTHING is un-uploaded -> deleter must still free space (oldest first),
    # never stall logging.
    seg0 = self.seg_format.format(0)
    seg1 = self.seg_format.format(1)
    p0 = self._make_seg(seg0, uploaded=False)
    p1 = self._make_seg(seg1, uploaded=False)

    order = self._run_until_deleted([p0, p1])
    assert order[0] == p0, f"oldest un-uploaded should go first as last resort, got {order}"
