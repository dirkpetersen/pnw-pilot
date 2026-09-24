"""ceslogup2pnw: the device's own pnw logs (the CES corpus) upload to S3 through the drive pipeline.

The one that matters most is TestTheSecretNeverLeaves. /data/pnw holds the Waze proxy API key, so
eligibility is two independent gates -- a named directory AND a required filename prefix -- and both
are tested against the real secret's real filename.

The rest pin the contract: drive data always goes first, the file is uploaded where it lies (no copy,
no hardlink, so the deleter can never reclaim it and the archive's own budget still governs), the key
is unique and asks for on-the-fly compression, and a source that has become UNREADABLE does not read
the same as a source with nothing in it.
"""
import os

import pytest

from openpilot.system.loggerd import uploader
from openpilot.system.loggerd.uploader import (PNW_LOG_PREFIXES, PNW_LOG_SOURCES, PNW_LOG_SUFFIX,
                                               UPLOAD_ATTR_NAME, UPLOAD_ATTR_VALUE, Uploader)

# A real name produced by archive_rotated_generation: basename + the generation's own mtime.
GEN = "ces_events.jsonl.20260916T191100Z"


class _Params:
  def get(self, k):
    return None

  def get_bool(self, k):
    return False


def _uploader(root, archive=None, prefix="pnwlogs", want="ces_events.jsonl."):
  u = Uploader.__new__(Uploader)
  u.root = str(root)
  u.params = _Params()
  u.immediate_priority = {"qlog": 0, "qlog.zst": 0, "qcamera.ts": 1}
  u.immediate_folders = ["crash/", "boot/"]
  u._retry_after = {}
  u._defer_hd = False
  u._skip_wide = False
  u._pnw_log_state = {}
  u._metered_full_day = None
  if archive is not None:
    uploader.PNW_LOG_SOURCES = ((str(archive), prefix, want),)
  return u


@pytest.fixture(autouse=True)
def _restore_sources():
  """The module constant is the real /data path; every test that needs a source swaps in a tmp dir."""
  orig = uploader.PNW_LOG_SOURCES
  yield
  uploader.PNW_LOG_SOURCES = orig


def _archive(tmp_path, *names):
  d = tmp_path / "ces_archive"
  d.mkdir(parents=True, exist_ok=True)
  for n in names:
    (d / n).write_text('{"t": 1, "ev": "curve"}\n')
  return d


def _listed(u, metered=False, pass2=False):
  return list(u.list_upload_files(metered=metered, pass2=pass2))


# ---------------------------------------------------------------------------------------------------
class TestTheSecretNeverLeaves:
  """/data/pnw/location/police_proxy.json holds the Waze API key. A sweep of /data/pnw would have PUT
  it in S3 on the first run. Two gates, tested with the real filename."""

  def test_the_api_key_is_not_eligible_even_sitting_in_the_archive_directory(self, tmp_path):
    d = _archive(tmp_path, GEN)
    (d / "police_proxy.json").write_text('{"key": "SECRET-DO-NOT-UPLOAD"}')
    u = _uploader(tmp_path / "realdata", archive=d)
    names = [n for n, _, _ in _listed(u)]
    assert names == [GEN]
    assert not any("police_proxy" in k or "police_proxy" in f for _, k, f in _listed(u))

  @pytest.mark.parametrize("name", ["police_proxy.json", "angle_tuning.json", "curve.json", "dm.json",
                                    "lanecenter_tuning.json", "notes.txt", "ces_events.jsonl"])
  def test_nothing_but_a_rotated_generation_is_ever_offered(self, tmp_path, name):
    """Note `ces_events.jsonl` itself is in this list: the LIVE log is still being appended to, so it
    is not a finished file and the trailing dot in the required prefix excludes it."""
    d = _archive(tmp_path, name)
    u = _uploader(tmp_path / "realdata", archive=d)
    assert _listed(u) == []

  def test_only_the_named_directory_is_scanned(self, tmp_path):
    """The allowlist is a directory, not a tree walk: a generation one level down is NOT picked up,
    so a future subdirectory of secrets cannot be swept in."""
    d = _archive(tmp_path, GEN)
    sub = d / "nested"
    sub.mkdir()
    (sub / "ces_events.jsonl.20260101T000000Z").write_text("{}\n")
    u = _uploader(tmp_path / "realdata", archive=d)
    assert [n for n, _, _ in _listed(u)] == [GEN]

  def test_a_symlink_wearing_the_right_name_does_NOT_carry_the_key_out(self, tmp_path):
    """Fable 2026-09-16: `startswith` matches a NAME, and a name is not a file. open() follows a
    symlink, so without the islink test this uploads the Waze API key under a pnwlogs/ key."""
    secret = tmp_path / "police_proxy.json"
    secret.write_text('{"key": "SECRET-DO-NOT-UPLOAD"}')
    d = _archive(tmp_path, GEN)
    (d / "ces_events.jsonl.link").symlink_to(secret)
    u = _uploader(tmp_path / "realdata", archive=d)
    listed = _listed(u)
    assert [n for n, _, _ in listed] == [GEN]
    for _, key, fn in listed:
      assert "link" not in key and os.path.realpath(fn) != str(secret)

  def test_a_directory_wearing_the_right_name_is_not_offered(self, tmp_path):
    """Not security -- a permanent failure loop. getsize() returns 4096 and open() raises
    IsADirectoryError, so it retried every 15 minutes forever with no error code and no UP ERR."""
    d = _archive(tmp_path, GEN)
    (d / "ces_events.jsonl.somedir").mkdir()
    u = _uploader(tmp_path / "realdata", archive=d)
    assert [n for n, _, _ in _listed(u)] == [GEN]

  def test_the_rejected_names_are_COUNTED_not_silently_dropped(self, tmp_path, monkeypatch):
    """Rule 2: a name that passed the prefix gate and was then rejected is exactly what someone
    needs to see, and the scan summary is where they would look."""
    d = _archive(tmp_path, GEN)
    (d / "ces_events.jsonl.somedir").mkdir()
    (d / "ces_events.jsonl.link").symlink_to(d / GEN)
    u = _uploader(tmp_path / "realdata", archive=d)
    ev = []
    monkeypatch.setattr(uploader.cloudlog, "event", lambda n, **kw: ev.append((n, kw)))
    _listed(u)
    scanned = [kw for n, kw in ev if n == "pnw_log_upload" and kw.get("state") == "scanned"]
    assert len(scanned) == 1
    assert scanned[0]["notfile"] == 2, scanned[0]
    assert scanned[0]["eligible"] == 1, "the rejects must not be counted as eligible either"

  def test_the_real_shipped_source_list_is_named_archive_directories_only(self):
    # cesarchive2pnw added curvedb_archive and net_archive; each is written by exactly one archiver.
    assert PNW_LOG_SOURCES == (("/data/pnw/ces_archive", "pnwlogs", "ces_events.jsonl."),
                               ("/data/pnw/curvedb_archive", "pnwlogs", "curvedb_obs.jsonl."),
                               ("/data/pnw/net_archive", "pnwlogs", "net_events.jsonl."))
    for src, _, want in PNW_LOG_SOURCES:
      assert want, f"{src} has no filename gate -- it would upload everything in that directory"
      assert src.startswith("/data/pnw/"), src
      assert src != "/data/pnw", "the parent directory holds the API key and must never be a source"


class TestItRidesTheDrivePipeline:
  def test_the_key_is_unique_and_asks_for_compression(self, tmp_path):
    d = _archive(tmp_path, GEN)
    u = _uploader(tmp_path / "realdata", archive=d)
    (name, key, fn), = _listed(u)
    assert name == GEN
    assert key == f"pnwlogs/{GEN}{PNW_LOG_SUFFIX}"
    # do_upload compresses exactly when the KEY ends .zst and the FILE does not. JSONL is ~10x.
    assert key.endswith(".zst") and not fn.endswith(".zst")
    assert fn == str(d / GEN)

  def test_it_is_uploaded_where_it_lies(self, tmp_path):
    """No copy, no hardlink, no staging into the log root: the deleter only walks the log root, so a
    staged copy could be reclaimed and a hardlink would defeat the archive's own 2 GB budget."""
    d = _archive(tmp_path, GEN)
    root = tmp_path / "realdata"
    root.mkdir()
    u = _uploader(root, archive=d)
    (_, _, fn), = _listed(u)
    assert os.path.dirname(fn) == str(d)
    assert os.listdir(root) == [], "a pnw log was staged into the log root, where the deleter can reclaim it"

  def test_an_already_uploaded_generation_is_not_sent_twice(self, tmp_path, monkeypatch):
    d = _archive(tmp_path, GEN)
    u = _uploader(tmp_path / "realdata", archive=d)
    monkeypatch.setattr(uploader, "getxattr",
                        lambda fn, attr: UPLOAD_ATTR_VALUE if attr == UPLOAD_ATTR_NAME else None)
    assert _listed(u) == []

  def test_a_generation_on_retry_cooldown_is_skipped(self, tmp_path, monkeypatch):
    d = _archive(tmp_path, GEN)
    u = _uploader(tmp_path / "realdata", archive=d)
    monkeypatch.setattr(uploader.time, "monotonic", lambda: 100.0)
    u._retry_after = {str(d / GEN): 200.0}
    assert _listed(u) == []
    u._retry_after = {str(d / GEN): 50.0}          # lapsed
    assert [n for n, _, _ in _listed(u)] == [GEN]


class TestWhichPassAndWhichNetwork:
  def test_pass_2_never_takes_one(self, tmp_path):
    """Pass 2 runs only on a priority network. A corpus file routed there would sit for weeks."""
    d = _archive(tmp_path, GEN)
    u = _uploader(tmp_path / "realdata", archive=d)
    assert _listed(u, pass2=True) == []

  def test_a_metered_link_never_takes_one(self, tmp_path):
    d = _archive(tmp_path, GEN)
    u = _uploader(tmp_path / "realdata", archive=d)
    assert _listed(u, metered=True) == []
    assert _listed(u, metered=False) != []


class TestDriveDataStillGoesFirst:
  def _with_segment(self, tmp_path, *seg_files):
    root = tmp_path / "realdata"
    seg = root / "00000001--abcdef0123--0"
    seg.mkdir(parents=True)
    for n in seg_files:
      (seg / n).write_bytes(b"x")
    return root

  def test_the_corpus_now_outranks_a_qlog(self, tmp_path):
    """INVERTED by cesarchive2pnw (owner, 2026-09-23): small device logs are tier 1, qlog tier 2.
    (This used to be test_a_qlog_outranks_the_corpus.) The class name is historical."""
    root = self._with_segment(tmp_path, "qlog.zst")
    d = _archive(tmp_path, GEN)
    u = _uploader(root, archive=d)
    name, _, _ = u.next_file_to_upload(metered=False)
    assert name == GEN

  def test_a_boot_log_outranks_the_corpus(self, tmp_path):
    root = tmp_path / "realdata"
    (root / "boot").mkdir(parents=True)
    (root / "boot" / "abc.bz2").write_bytes(b"x")
    d = _archive(tmp_path, GEN)
    u = _uploader(root, archive=d)
    name, _, _ = u.next_file_to_upload(metered=False)
    assert name == "abc.bz2"

  def test_but_it_IS_chosen_once_the_drive_queue_is_empty(self, tmp_path):
    """Without the third tier in next_file_to_upload the file is listed and never chosen -- a silent
    no-op that would look exactly like a working feature."""
    root = tmp_path / "realdata"
    root.mkdir()
    d = _archive(tmp_path, GEN)
    u = _uploader(root, archive=d)
    got = u.next_file_to_upload(metered=False)
    assert got is not None, "listed but never chosen: the corpus would never reach S3"
    assert got[0] == GEN

  def test_the_DRIVER_CAMERA_is_not_swept_up_by_the_new_tier(self, tmp_path, monkeypatch):
    """Fable 2026-09-16, and the reason the tier tests a KEY PREFIX rather than "whatever is left".
    Pass 1 lists every non-firehose file in a segment, dcamera.hevc included (FIREHOSE_FILES covers
    only rlog/fcamera/ecamera); stock never picks it because only immediate-priority names are
    chosen. This tier is the only thing keeping that true.

    THE REVERSED ORDER IS THE TEST (Fable, round 2). list_upload_files yields the corpus BEFORE the
    drive-file walk, so in natural order even `if True` returns GEN and this passed under the very
    mutation it claims to catch -- the kill belonged entirely to the sibling test below. Feeding the
    tier the reverse order asks the real question: is dcamera.hevc rejected wherever it sits?"""
    root = self._with_segment(tmp_path, "dcamera.hevc")
    d = _archive(tmp_path, GEN)
    u = _uploader(root, archive=d)
    files = _listed(u)
    assert [n for n, _, _ in files] == [GEN, "dcamera.hevc"], "premise: both listed, corpus FIRST"
    monkeypatch.setattr(u, "list_upload_files", lambda metered, pass2=False: iter(reversed(files)))
    name, key, _ = u.next_file_to_upload(metered=False)
    assert name == GEN, f"the driver-facing camera was chosen over the corpus ({name})"
    assert key.startswith("pnwlogs/")

  def test_with_no_corpus_the_leftovers_are_NOT_uploaded_and_the_queue_goes_idle(self, tmp_path):
    """The other half: next_file_to_upload must still return None so the 60 s idle backoff engages.
    A loosened predicate would make it return a leftover forever and the uploader would never rest."""
    root = self._with_segment(tmp_path, "dcamera.hevc")
    u = _uploader(root, archive=tmp_path / "never_created")
    assert "dcamera.hevc" in [n for n, _, _ in _listed(u)]
    assert u.next_file_to_upload(metered=False) is None

  def test_the_oldest_generation_goes_first(self, tmp_path):
    root = tmp_path / "realdata"
    root.mkdir()
    old, new = "ces_events.jsonl.20260101T000000Z", "ces_events.jsonl.20260916T191100Z"
    d = _archive(tmp_path, new, old)
    u = _uploader(root, archive=d)
    # names are mtime stamps, so lexical order IS chronological order
    assert u.next_file_to_upload(metered=False)[0] == old


class TestRule2:
  """A source that cannot be read must not read the same as a source with nothing in it -- the
  '0 of 2,612 files uploaded' class of false conclusion."""

  def test_a_directory_that_does_not_exist_yet_is_not_an_error(self, tmp_path, monkeypatch):
    logged = []
    monkeypatch.setattr(uploader.cloudlog, "error", lambda *a, **k: logged.append(a))
    u = _uploader(tmp_path / "realdata", archive=tmp_path / "never_created")
    assert _listed(u) == []
    assert logged == [], "a device that has not filled 8 generations yet is normal, not broken"

  def test_an_unreadable_directory_is_LOUD(self, tmp_path, monkeypatch):
    d = _archive(tmp_path, GEN)
    u = _uploader(tmp_path / "realdata", archive=d)
    logged = []
    monkeypatch.setattr(uploader.cloudlog, "error", lambda *a, **k: logged.append(a))

    def boom(_):
      raise PermissionError("nope")
    monkeypatch.setattr(uploader.os, "listdir", boom)
    assert _listed(u) == []
    assert len(logged) == 1, "an unreadable archive silently uploaded nothing"
    assert "PermissionError" in str(logged[0])

  def test_the_loud_error_is_change_only(self, tmp_path, monkeypatch):
    u = _uploader(tmp_path / "realdata", archive=_archive(tmp_path, GEN))
    logged = []
    monkeypatch.setattr(uploader.cloudlog, "error", lambda *a, **k: logged.append(a))
    monkeypatch.setattr(uploader.os, "listdir", lambda _: (_ for _ in ()).throw(PermissionError("nope")))
    for _ in range(25):
      _listed(u)
    assert len(logged) == 1, "the listing runs many times a minute; this would flood the log"

  def test_a_generation_whose_upload_MARK_cannot_be_read_is_not_skipped_in_silence(self, tmp_path, monkeypatch):
    """The per-file half of the same rule. An unreadable xattr is an ERROR, not the answer "already
    uploaded" -- and the file is skipped either way, so the log line is the only difference between
    a healthy scan and a corpus that is quietly going nowhere."""
    d = _archive(tmp_path, GEN)
    u = _uploader(tmp_path / "realdata", archive=d)
    ev = []
    monkeypatch.setattr(uploader.cloudlog, "event", lambda n, **kw: ev.append((n, kw)))

    def boom(_fn, _attr):
      raise OSError("nope")
    monkeypatch.setattr(uploader, "getxattr", boom)
    assert _listed(u) == []
    failed = [kw for n, kw in ev if n == "uploader_getxattr_failed"]
    assert len(failed) == 1 and failed[0]["key"] == GEN, ev

  def test_the_summary_says_what_was_scanned(self, tmp_path, monkeypatch):
    d = _archive(tmp_path, GEN, "ces_events.jsonl.20260101T000000Z", "notes.txt")
    u = _uploader(tmp_path / "realdata", archive=d)
    ev = []
    monkeypatch.setattr(uploader.cloudlog, "event", lambda n, **kw: ev.append((n, kw)))
    _listed(u)
    scanned = [kw for n, kw in ev if n == "pnw_log_upload" and kw.get("state") == "scanned"]
    assert len(scanned) == 1
    assert scanned[0]["eligible"] == 2 and scanned[0]["pending"] == 2, scanned[0]


class TestTheTwoTablesCannotDrift:
  def test_the_prefixes_are_derived_from_the_sources(self):
    """next_file_to_upload matches on PNW_LOG_PREFIXES; _list_pnw_log_files yields from
    PNW_LOG_SOURCES. A source added to one and missed in the other lists forever and is never
    chosen."""
    assert PNW_LOG_PREFIXES == tuple(f"{p}/" for _, p, _ in PNW_LOG_SOURCES)

  def test_the_prefix_gate_accepts_what_the_archiver_actually_produces(self, tmp_path):
    """Ties this module to ces_pnw: rename the archive's output and this fails rather than silently
    uploading nothing."""
    from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import archive_rotated_generation
    live = tmp_path / "ces_events.jsonl"
    live.write_text("{}\n")
    (tmp_path / "ces_events.jsonl.8").write_text('{"t": 1}\n')
    dest = archive_rotated_generation(str(live), 8, archive_dir=str(tmp_path / "arch"))
    assert dest is not None, "the archiver produced nothing -- this test proves nothing"
    want = PNW_LOG_SOURCES[0][2]
    assert os.path.basename(dest).startswith(want), (os.path.basename(dest), want)
