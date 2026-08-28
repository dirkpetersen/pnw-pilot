import os
import time
import threading
import logging
import json
from pathlib import Path
from openpilot.system.hardware.hw import Paths

from openpilot.common.swaglog import cloudlog
from openpilot.system.loggerd.uploader import (main, pass1_allowed, pass2_allowed, PASS2_NETWORK_TYPES,
                                               UPLOAD_ATTR_NAME, UPLOAD_ATTR_VALUE, Uploader,
                                               uploadable_firehose_files, FIREHOSE_FILES)
from cereal import log

from openpilot.system.loggerd.tests.loggerd_tests_common import UploaderTestCase

WIFI = next(iter(PASS2_NETWORK_TYPES))
CELL = int(log.DeviceState.NetworkType.cell4G)


class TestUploadGate:
  """uploadgate2pnw (driver spec 2026-07-13): pass 2 (rlog/HD) ONLY at a priority/home network —
  never on other WiFi however unmetered (a 75 MB background burst kills an on-the-road hotspot);
  metered blocks ALL file uploads (pass 1 included); onroad needs Park (GearPark param, no msgq)."""

  # -- pass 1: metered blocks everything; unmetered (wifi or LTE) flows --
  def test_pass1_metered_blocks(self):
    assert not pass1_allowed(metered=True)

  def test_pass1_unmetered_allows(self):
    assert pass1_allowed(metered=False)

  # -- pass 2: home-first --
  def test_pass2_home_offroad_allows(self):
    assert pass2_allowed(WIFI, metered=False, at_home=True, onroad=False, parked=False)

  def test_pass2_home_onroad_parked_allows(self):
    # EV charging at home: ignition on -> onroad, gear in Park -> allowed
    assert pass2_allowed(WIFI, metered=False, at_home=True, onroad=True, parked=True)

  def test_pass2_home_onroad_driving_blocks(self):
    # driving away while still in home WiFi range: no burst mid-drive
    assert not pass2_allowed(WIFI, metered=False, at_home=True, onroad=True, parked=False)

  def test_pass2_not_home_never_allows(self):
    # the driver's core rule: unmetered on-the-road hotspot/WiFi is NOT enough — never pass 2 away
    # from a priority network, even offroad, even parked
    assert not pass2_allowed(WIFI, metered=False, at_home=False, onroad=False, parked=False)
    assert not pass2_allowed(WIFI, metered=False, at_home=False, onroad=True, parked=True)

  def test_pass2_metered_blocks_even_at_home(self):
    assert not pass2_allowed(WIFI, metered=True, at_home=True, onroad=False, parked=True)

  def test_pass2_non_wifi_blocks_even_at_home(self):
    assert not pass2_allowed(CELL, metered=False, at_home=True, onroad=False, parked=True)


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

  def test_upload_ignored(self):
    self.set_ignore()
    self.gen_files(lock=False)

    self.start_thread()
    # allow enough time that files could upload twice if there is a bug in the logic
    time.sleep(1)
    self.join_thread()

    exp_order = self.gen_order([self.seg_num], [])

    assert len(log_handler.upload_order) == 0, "Some files were not ignored"
    assert not len(log_handler.upload_ignored) < len(exp_order), "Some files failed to ignore"
    assert not len(log_handler.upload_ignored) > len(exp_order), "Some files were ignored twice"

    # testbaseline2pnw: this assertion is INVERTED from stock openpilot, deliberately. Upstream marks a
    # 412'd file uploaded (user.upload=1) and moves on. This fork does NOT -- uploadretry2pnw made "only
    # a real 2xx marks a file uploaded" after the API_HOST incident, where the host fell back to comma's
    # backend, every proactive upload 412'd, and files were stamped uploaded WITHOUT ever reaching S3:
    # silent data loss, with an idle uploader and a clean-looking device. So the contract here is that a
    # 412'd file keeps NO xattr and stays eligible for retry. The stock assertion (expecting the xattr to
    # be SET) had been failing on this branch ever since, which is exactly backwards -- it would now pass
    # only if the data-loss bug came back.
    for f_path in exp_order:
      fn = (Path(Paths.log_root()) / f_path).with_suffix("")
      assert UPLOAD_ATTR_NAME not in os.listxattr(fn), \
        f"412'd file was marked uploaded without reaching S3 (silent data loss): {fn}"

    assert log_handler.upload_ignored == exp_order, "Files ignored in wrong order"

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


class TestPass2Priority:
  """uploadprio2pnw: rlog is ~10% of the pass-2 bytes but carries the analysis value, and raw
  os.listdir order put it LAST inside every segment. These pin the selection contract."""

  class _FakeUploader:
    """Minimal stand-in exercising the real next_pass2_file_to_upload against a scripted listing."""
    def __init__(self, files):
      self._files = files

    def list_upload_files(self, metered, pass2=False):
      yield from self._files

    next_pass2_file_to_upload = Uploader.next_pass2_file_to_upload

  def test_rlog_wins_over_video_in_the_same_segment(self):
    u = self._FakeUploader([
      ("fcamera.hevc", "seg0/fcamera.hevc", "/d/seg0/fcamera.hevc"),
      ("ecamera.hevc", "seg0/ecamera.hevc", "/d/seg0/ecamera.hevc"),
      ("rlog.zst", "seg0/rlog.zst", "/d/seg0/rlog.zst"),
    ])
    assert u.next_pass2_file_to_upload(metered=False)[0] == "rlog.zst"

  def test_rlog_wins_across_segments_not_just_within_one(self):
    # the whole point: an rlog in a LATER segment still beats video in the oldest one
    u = self._FakeUploader([
      ("fcamera.hevc", "seg0/fcamera.hevc", "/d/seg0/fcamera.hevc"),
      ("ecamera.hevc", "seg0/ecamera.hevc", "/d/seg0/ecamera.hevc"),
      ("fcamera.hevc", "seg1/fcamera.hevc", "/d/seg1/fcamera.hevc"),
      ("rlog.zst", "seg9/rlog.zst", "/d/seg9/rlog.zst"),
    ])
    assert u.next_pass2_file_to_upload(metered=False)[2] == "/d/seg9/rlog.zst"

  def test_oldest_rlog_first_among_rlogs(self):
    u = self._FakeUploader([
      ("rlog.zst", "seg3/rlog.zst", "/d/seg3/rlog.zst"),
      ("rlog.zst", "seg7/rlog.zst", "/d/seg7/rlog.zst"),
    ])
    assert u.next_pass2_file_to_upload(metered=False)[2] == "/d/seg3/rlog.zst"

  def test_falls_back_to_oldest_video_when_no_rlog_left(self):
    # byte-identical to the old behaviour once the rlog backlog is drained
    u = self._FakeUploader([
      ("fcamera.hevc", "seg0/fcamera.hevc", "/d/seg0/fcamera.hevc"),
      ("ecamera.hevc", "seg0/ecamera.hevc", "/d/seg0/ecamera.hevc"),
      ("fcamera.hevc", "seg1/fcamera.hevc", "/d/seg1/fcamera.hevc"),
    ])
    assert u.next_pass2_file_to_upload(metered=False)[2] == "/d/seg0/fcamera.hevc"

  def test_empty_listing_returns_none(self):
    assert self._FakeUploader([]).next_pass2_file_to_upload(metered=False) is None

  def test_uncompressed_rlog_also_prioritised(self):
    u = self._FakeUploader([
      ("fcamera.hevc", "seg0/fcamera.hevc", "/d/seg0/fcamera.hevc"),
      ("rlog", "seg0/rlog", "/d/seg0/rlog"),
    ])
    assert u.next_pass2_file_to_upload(metered=False)[0] == "rlog"


class TestUploadableFirehoseFiles:
  """uploadprio2pnw: the deleter must not count a deliberately-skipped file as 'un-uploaded' --
  otherwise every segment is pinned and the keep-un-uploaded-last ordering flattens to oldest-first."""

  class _P:
    def __init__(self, **vals):
      self._vals = vals

    def get_bool(self, k):
      return self._vals.get(k, False)

  def test_default_is_the_full_set(self):
    assert uploadable_firehose_files(self._P()) == FIREHOSE_FILES

  def test_skip_wide_drops_only_ecamera(self):
    got = uploadable_firehose_files(self._P(SkipWideCameraUpload=True))
    assert "ecamera.hevc" not in got
    assert {"rlog", "rlog.zst", "fcamera.hevc"} <= got

  def test_defer_hd_is_deliberately_IGNORED(self):
    # Defer is a TEMPORARY hold ("skipped-but-KEPT"): its segments must keep sorting last and must
    # keep raising the loud "deleting UN-UPLOADED segment" error. Subtracting it here would turn
    # "hold the video until I'm home" into "silently discard the video under disk pressure".
    assert uploadable_firehose_files(self._P(DeferHDVideoUpload=True)) == FIREHOSE_FILES

  def test_defer_does_not_widen_the_skip_wide_reduction(self):
    got = uploadable_firehose_files(self._P(SkipWideCameraUpload=True, DeferHDVideoUpload=True))
    assert got == FIREHOSE_FILES - {"ecamera.hevc"}
    assert "fcamera.hevc" in got          # defer must NOT strip fcamera from the deleter's view

  def test_param_failure_falls_back_to_full_set(self):
    class Boom:
      def get_bool(self, k):
        raise RuntimeError("params down")
    assert uploadable_firehose_files(Boom()) == FIREHOSE_FILES


class TestSkipWideListing:
  """uploadprio2pnw: the _skip_wide branch inside list_upload_files is the ONE place that changes
  what really gets uploaded, so exercise it directly against a real on-disk segment."""

  def _uploader(self, root, skip_wide):
    u = Uploader.__new__(Uploader)
    u.root = str(root)
    u.params = self._Params(skip_wide)
    u.immediate_priority = {"qlog": 0, "qlog.zst": 0, "qcamera.ts": 1}
    u.immediate_folders = []
    u._retry_after = {}
    u._defer_hd = False
    u._skip_wide = False
    return u

  class _Params:
    def __init__(self, skip_wide):
      self._skip_wide = skip_wide

    def get(self, k):
      return None

    def get_bool(self, k):
      return self._skip_wide if k == "SkipWideCameraUpload" else False

  def _segment(self, tmp_path):
    seg = tmp_path / "00000001--abcdef0123--0"
    seg.mkdir(parents=True)
    for n in ("fcamera.hevc", "ecamera.hevc", "rlog.zst", "qlog.zst", "qcamera.ts"):
      (seg / n).write_bytes(b"x")
    return seg

  def test_ecamera_offered_when_toggle_off(self, tmp_path):
    self._segment(tmp_path)
    u = self._uploader(tmp_path, skip_wide=False)
    names = {n for n, _, _ in u.list_upload_files(metered=False, pass2=True)}
    assert names == {"fcamera.hevc", "ecamera.hevc", "rlog.zst"}

  def test_ecamera_withheld_when_toggle_on(self, tmp_path):
    self._segment(tmp_path)
    u = self._uploader(tmp_path, skip_wide=True)
    names = {n for n, _, _ in u.list_upload_files(metered=False, pass2=True)}
    assert "ecamera.hevc" not in names
    assert names == {"fcamera.hevc", "rlog.zst"}     # nothing else collaterally dropped

  # NOTE: there is deliberately no "skipped file is never xattr-marked" test here. setxattr runs only
  # on a 2xx inside upload(); a test that merely lists and then asserts the xattr is absent can never
  # fail, and a test that cannot fail is worse than no test. The real contract -- a skipped file is
  # never even a CANDIDATE, because the skip is a `continue` before the yield -- is what
  # test_ecamera_withheld_when_toggle_on above actually pins.


class TestSkipWideFullLoop(UploaderTestCase):
  """uploadprio2pnw: end-to-end guard for the skip contract, through the REAL main() loop.

  TestSkipWideListing (above) only proves the generator withholds ecamera. This proves the whole
  loop -- selection, upload, xattr marking -- never stamps a withheld file as uploaded. That is the
  API_HOST-412 family of bug: in that incident files were marked uploaded WITHOUT reaching S3, and
  the device looked clean while data was quietly lost. A future "mark skipped files so they stop
  being re-listed" optimisation would reintroduce exactly that, and this test is what catches it.
  """

  def setup_method(self):
    super().setup_method()
    log_handler.reset()
    # pass-2 gate (uploadgate2pnw): unmetered wifi (force_wifi) + priority network + parked/offroad
    self.params.put_bool("OnPriorityNetwork", True)
    self.params.put_bool("GearPark", True)
    self.params.put_bool("DeferHDVideoUpload", False)

  def teardown_method(self):
    self.params.put_bool("SkipWideCameraUpload", False)

  def _run(self):
    end_event = threading.Event()
    t = threading.Thread(target=main, args=[end_event])
    t.daemon = True
    t.start()
    time.sleep(1.5)         # long enough for several pass-2 picks
    end_event.set()
    t.join()

  def _pass2_files(self):
    seg = Path(Paths.log_root()) / self.seg_dir
    return {n: seg / n for n in ("rlog", "fcamera.hevc", "ecamera.hevc")}

  def _marked(self, p: Path) -> bool:
    return UPLOAD_ATTR_NAME in os.listxattr(p)

  def test_skipped_ecamera_is_never_marked_uploaded(self):
    self.params.put_bool("SkipWideCameraUpload", True)
    for n in ("qlog", "rlog", "fcamera.hevc", "ecamera.hevc"):
      self.make_file_with_data(self.seg_dir, n, 1)

    self._run()
    f = self._pass2_files()

    # the withheld file must be untouched: no xattr, and never even attempted
    assert not self._marked(f["ecamera.hevc"]), \
      "ecamera was marked uploaded while SkipWideCameraUpload was on -- silent data loss"
    assert not any(k.endswith("ecamera.hevc") for k in log_handler.upload_order), \
      "ecamera was uploaded despite the skip toggle"
    # (No "file still exists on disk" assertion: nothing in the uploader path ever deletes a segment
    # -- that is the deleter thread, which does not run here -- so such an assert could never fail.
    # The xattr and upload_order checks above are what actually carry the mutation coverage.)

    # the NON-skipped pass-2 files must still go, or the toggle is dropping too much
    assert self._marked(f["rlog"]), "rlog did not upload with the skip toggle on"
    assert self._marked(f["fcamera.hevc"]), "fcamera did not upload with the skip toggle on"

  def test_ecamera_uploads_normally_when_toggle_is_off(self):
    # the control case: same fixture, toggle off -> ecamera goes. Without this, the test above would
    # also pass if pass 2 were broken entirely.
    self.params.put_bool("SkipWideCameraUpload", False)
    for n in ("qlog", "rlog", "fcamera.hevc", "ecamera.hevc"):
      self.make_file_with_data(self.seg_dir, n, 1)

    self._run()
    f = self._pass2_files()
    assert self._marked(f["ecamera.hevc"]), "ecamera did not upload with the skip toggle OFF"

  def test_rlog_is_picked_before_video(self):
    # uploadprio2pnw ordering, end-to-end: rlog must be the first pass-2 key to succeed even though
    # os.listdir hands back fcamera/ecamera first.
    self.params.put_bool("SkipWideCameraUpload", False)
    for n in ("qlog", "rlog", "fcamera.hevc", "ecamera.hevc"):
      self.make_file_with_data(self.seg_dir, n, 1)

    self._run()
    pass2 = [k for k in log_handler.upload_order
             if k.endswith(("rlog.zst", "rlog", "fcamera.hevc", "ecamera.hevc"))]
    assert pass2, "no pass-2 uploads happened at all"
    assert pass2[0].endswith(("rlog.zst", "rlog")), \
      f"pass 2 did not start with rlog (got {pass2[0]}) -- priority ordering regressed"
