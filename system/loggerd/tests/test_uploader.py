import os
import time
import threading
import logging
import json
from pathlib import Path
from openpilot.system.hardware.hw import Paths

from openpilot.common.swaglog import cloudlog
from openpilot.system.loggerd.uploader import main, pass2_allowed, PASS2_NETWORK_TYPES, UPLOAD_ATTR_NAME, UPLOAD_ATTR_VALUE
from cereal import log

from openpilot.system.loggerd.tests.loggerd_tests_common import UploaderTestCase

WIFI = next(iter(PASS2_NETWORK_TYPES))
CELL = int(log.DeviceState.NetworkType.cell4G)


class TestPass2Gate:
  """firehose2pnw: the pass-2 (rlog/HD) eligibility gate. Base rule is WiFi + not-metered; the onroad
  block is relaxed at a priority (home) network (pure location override) or, on any other WiFi, while
  standstill — so an EV charging at home (ignitionLine on -> onroad) still uploads, but a 75 MB burst
  can never fire mid-maneuver on a random WiFi."""

  def test_offroad_wifi_allows(self):
    assert pass2_allowed(WIFI, metered=False, onroad=False)

  def test_metered_always_blocks(self):
    # metered is an absolute block on every axis — even parked, even at a priority network
    assert not pass2_allowed(WIFI, metered=True, onroad=False)
    assert not pass2_allowed(WIFI, metered=True, onroad=True, at_home=True, standstill=True)

  def test_non_wifi_always_blocks(self):
    assert not pass2_allowed(CELL, metered=False, onroad=False)
    assert not pass2_allowed(CELL, metered=False, onroad=True, at_home=True, standstill=True)

  def test_onroad_plain_blocks(self):
    # driving on a non-priority WiFi, moving -> the original onroad protection still holds
    assert not pass2_allowed(WIFI, metered=False, onroad=True, at_home=False, standstill=False)

  def test_onroad_at_home_overrides(self):
    # EV charging at home keeps ignition on (onroad) but must still upload — location override,
    # no standstill requirement
    assert pass2_allowed(WIFI, metered=False, onroad=True, at_home=True, standstill=False)

  def test_onroad_standstill_other_wifi_allows(self):
    # any other WiFi: onroad pass-2 only while stopped (the standstill guard)
    assert pass2_allowed(WIFI, metered=False, onroad=True, at_home=False, standstill=True)


class FakeLogHandler(logging.Handler):
  def __init__(self):
    logging.Handler.__init__(self)
    self.reset()

  def reset(self):
    self.upload_order = list()
    self.upload_ignored = list()

  def emit(self, record):
    try:
      j = json.loads(record.getMessage())
      if j["event"] == "upload_success":
        self.upload_order.append(j["key"])
      if j["event"] == "upload_ignored":
        self.upload_ignored.append(j["key"])
    except Exception:
      pass

log_handler = FakeLogHandler()
cloudlog.addHandler(log_handler)


class TestUploader(UploaderTestCase):
  def setup_method(self):
    super().setup_method()
    log_handler.reset()

  def start_thread(self):
    self.end_event = threading.Event()
    self.up_thread = threading.Thread(target=main, args=[self.end_event])
    self.up_thread.daemon = True
    self.up_thread.start()

  def join_thread(self):
    self.end_event.set()
    self.up_thread.join()

  def gen_files(self, lock=False, xattr: bytes | None = None, boot=True) -> list[Path]:
    f_paths = []
    for t in ["qlog", "rlog", "dcamera.hevc", "fcamera.hevc"]:
      f_paths.append(self.make_file_with_data(self.seg_dir, t, 1, lock=lock, upload_xattr=xattr))

    if boot:
      f_paths.append(self.make_file_with_data("boot", f"{self.seg_dir}", 1, lock=lock, upload_xattr=xattr))
    return f_paths

  def gen_order(self, seg1: list[int], seg2: list[int], boot=True) -> list[str]:
    keys = []
    if boot:
      keys += [f"boot/{self.seg_format.format(i)}.zst" for i in seg1]
      keys += [f"boot/{self.seg_format2.format(i)}.zst" for i in seg2]
    keys += [f"{self.seg_format.format(i)}/qlog.zst" for i in seg1]
    keys += [f"{self.seg_format2.format(i)}/qlog.zst" for i in seg2]
    return keys

  def test_upload(self):
    self.gen_files(lock=False)

    self.start_thread()
    # allow enough time that files could upload twice if there is a bug in the logic
    time.sleep(1)
    self.join_thread()

    exp_order = self.gen_order([self.seg_num], [])

    assert len(log_handler.upload_ignored) == 0, "Some files were ignored"
    assert not len(log_handler.upload_order) < len(exp_order), "Some files failed to upload"
    assert not len(log_handler.upload_order) > len(exp_order), "Some files were uploaded twice"
    for f_path in exp_order:
      assert os.getxattr((Path(Paths.log_root()) / f_path).with_suffix(""), UPLOAD_ATTR_NAME) == UPLOAD_ATTR_VALUE, "All files not uploaded"

    assert log_handler.upload_order == exp_order, "Files uploaded in wrong order"

  def test_upload_with_wrong_xattr(self):
    self.gen_files(lock=False, xattr=b'0')

    self.start_thread()
    # allow enough time that files could upload twice if there is a bug in the logic
    time.sleep(1)
    self.join_thread()

    exp_order = self.gen_order([self.seg_num], [])

    assert len(log_handler.upload_ignored) == 0, "Some files were ignored"
    assert not len(log_handler.upload_order) < len(exp_order), "Some files failed to upload"
    assert not len(log_handler.upload_order) > len(exp_order), "Some files were uploaded twice"
    for f_path in exp_order:
      assert os.getxattr((Path(Paths.log_root()) / f_path).with_suffix(""), UPLOAD_ATTR_NAME) == UPLOAD_ATTR_VALUE, "All files not uploaded"

    assert log_handler.upload_order == exp_order, "Files uploaded in wrong order"

  def test_upload_412_never_marks_and_retries(self):
    # firehose2pnw (driver req: never lose data): a 412 ("already there"/backend-declined — and, on the
    # broken-API_HOST footgun, a file that did NOT reach OUR S3) must NEVER xattr-mark the file done.
    # It stays un-marked and is retried on every eligible window until it truly uploads (2xx) or the
    # deleter reclaims it while driving. Previously 412 counted as success and marked the file, which
    # was the silent-data-loss bug.
    self.set_ignore()
    self.gen_files(lock=False)

    self.start_thread()
    # allow enough time that a marked-done bug would let the loop "finish" and stop ignoring
    time.sleep(1)
    self.join_thread()

    exp_order = self.gen_order([self.seg_num], [])

    # core contract: nothing succeeded (no 2xx), the file(s) were attempted, and NOTHING got
    # xattr-marked done -> every 412 file remains pending and will retry (after its cooldown).
    assert len(log_handler.upload_order) == 0, "A 412 must not count as an upload success"
    assert len(log_handler.upload_ignored) >= 1, "The 412 file was never attempted"
    for f_path in exp_order:
      p = (Path(Paths.log_root()) / f_path).with_suffix("")
      try:
        marked = os.getxattr(p, UPLOAD_ATTR_NAME) == UPLOAD_ATTR_VALUE
      except OSError:
        marked = False
      assert not marked, "A 412 file must NOT be marked uploaded (it must retry)"

  def test_upload_files_in_create_order(self):
    seg1_nums = [0, 1, 2, 10, 20]
    for i in seg1_nums:
      self.seg_dir = self.seg_format.format(i)
      self.gen_files(boot=False)
    seg2_nums = [5, 50, 51]
    for i in seg2_nums:
      self.seg_dir = self.seg_format2.format(i)
      self.gen_files(boot=False)

    exp_order = self.gen_order(seg1_nums, seg2_nums, boot=False)

    self.start_thread()
    # allow enough time that files could upload twice if there is a bug in the logic
    time.sleep(1)
    self.join_thread()

    assert len(log_handler.upload_ignored) == 0, "Some files were ignored"
    assert not len(log_handler.upload_order) < len(exp_order), "Some files failed to upload"
    assert not len(log_handler.upload_order) > len(exp_order), "Some files were uploaded twice"
    for f_path in exp_order:
      assert os.getxattr((Path(Paths.log_root()) / f_path).with_suffix(""), UPLOAD_ATTR_NAME) == UPLOAD_ATTR_VALUE, "All files not uploaded"

    assert log_handler.upload_order == exp_order, "Files uploaded in wrong order"

  def test_no_upload_with_lock_file(self):
    self.start_thread()

    time.sleep(0.25)
    f_paths = self.gen_files(lock=True, boot=False)

    # allow enough time that files should have been uploaded if they would be uploaded
    time.sleep(1)
    self.join_thread()

    for f_path in f_paths:
      fn = f_path.with_suffix(f_path.suffix.replace(".zst", ""))
      uploaded = UPLOAD_ATTR_NAME in os.listxattr(fn) and os.getxattr(fn, UPLOAD_ATTR_NAME) == UPLOAD_ATTR_VALUE
      assert not uploaded, "File upload when locked"

  def test_no_upload_with_xattr(self):
    self.gen_files(lock=False, xattr=UPLOAD_ATTR_VALUE)

    self.start_thread()
    # allow enough time that files could upload twice if there is a bug in the logic
    time.sleep(1)
    self.join_thread()

    assert len(log_handler.upload_order) == 0, "File uploaded again"

  def test_clear_locks_on_startup(self):
    f_paths = self.gen_files(lock=True, boot=False)
    self.start_thread()
    time.sleep(0.25)
    self.join_thread()

    for f_path in f_paths:
      lock_path = f_path.with_suffix(f_path.suffix + ".lock")
      assert not lock_path.is_file(), "File lock not cleared on startup"
