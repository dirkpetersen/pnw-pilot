#!/usr/bin/env python3
import os
import shutil
import threading
from openpilot.common.params import Params
from openpilot.system.hardware.hw import Paths
from openpilot.common.swaglog import cloudlog
from openpilot.system.loggerd.config import get_available_bytes, get_available_percent
from openpilot.system.loggerd.uploader import (listdir_by_creation, FIREHOSE_FILES, UPLOAD_ATTR_NAME,
                                                UPLOAD_ATTR_VALUE, uploadable_firehose_files)
from openpilot.system.loggerd.xattr_cache import getxattr

MIN_BYTES = 5 * 1024 * 1024 * 1024
MIN_PERCENT = 10

DELETE_LAST = ['boot', 'crash']

PRESERVE_ATTR_NAME = 'user.preserve'
PRESERVE_ATTR_VALUE = b'1'
PRESERVE_COUNT = 5


def has_preserve_xattr(d: str) -> bool:
  return getxattr(os.path.join(Paths.log_root(), d), PRESERVE_ATTR_NAME) == PRESERVE_ATTR_VALUE


def has_unuploaded_firehose(d: str, firehose_files: set[str] | None = None) -> bool:
  # connect2xnor: True if this segment still has a large pass-2 file
  # (rlog/fcamera/ecamera) that has NOT yet been uploaded (no user.upload=1
  # xattr). Such segments are kept preferentially so proactive WiFi uploads can
  # finish before the data is pruned. A stat error -> treat the file as already
  # handled (don't let a flaky xattr read block freeing space).
  # uploadprio2pnw: `firehose_files` is the set the uploader will EVER send (see
  # uploadable_firehose_files, whose docstring carries the full rationale). A PERMANENTLY skipped file
  # (SkipWideCameraUpload) never gets the xattr, so counting it here would pin EVERY segment as
  # un-uploaded and silently flatten the sort below into plain oldest-first. DeferHDVideoUpload is
  # deliberately NOT in that reduction -- it is a temporary hold whose segments must stay protected.
  # Defaults to the full set, which is also what the caller uses for the un-uploaded ERROR alarm.
  firehose = FIREHOSE_FILES if firehose_files is None else firehose_files
  seg_path = os.path.join(Paths.log_root(), d)
  try:
    names = os.listdir(seg_path)
  except OSError:
    return False
  for name in names:
    if name not in firehose:
      continue
    try:
      if getxattr(os.path.join(seg_path, name), UPLOAD_ATTR_NAME) != UPLOAD_ATTR_VALUE:
        return True
    except OSError:
      continue
  return False


def get_preserved_segments(dirs_by_creation: list[str]) -> set[str]:
  # skip deleting most recent N preserved segments (and their prior segment)
  preserved = set()
  for n, d in enumerate(filter(has_preserve_xattr, reversed(dirs_by_creation))):
    if n == PRESERVE_COUNT:
      break
    date_str, _, seg_str = d.rpartition("--")

    # ignore non-segment directories
    if not date_str:
      continue
    try:
      seg_num = int(seg_str)
    except ValueError:
      continue

    # preserve segment and two prior
    for _seg_num in range(max(0, seg_num - 2), seg_num + 1):
      preserved.add(f"{date_str}--{_seg_num}")

  return preserved


def deleter_thread(exit_event: threading.Event):
  params = Params()          # uploadprio2pnw: one handle, reused for every sweep
  while not exit_event.is_set():
    out_of_bytes = get_available_bytes(default=MIN_BYTES + 1) < MIN_BYTES
    out_of_percent = get_available_percent(default=MIN_PERCENT + 1) < MIN_PERCENT

    if out_of_percent or out_of_bytes:
      dirs = listdir_by_creation(Paths.log_root())
      preserved_dirs = get_preserved_segments(dirs)

      # connect2xnor: precompute which segments still have un-uploaded large
      # pass-2 files so they sort LAST (deleted only as a last resort).
      # uploadprio2pnw: read the skip toggle ONCE per sweep, not per segment (one Params() reused).
      firehose_files = uploadable_firehose_files(params)
      unuploaded_dirs = {d for d in dirs if has_unuploaded_firehose(d, firehose_files)}

      # connect2xnor: sort key tuple, ascending -> first element deleted first.
      #   1. d in DELETE_LAST        (boot/crash kept over normal segments)
      #   2. d in preserved_dirs     (user.preserve segments)
      #   3. d in unuploaded_dirs    (NEW: segments with un-uploaded video/rlog)
      # So a fully-uploaded ordinary segment is always deleted before one whose
      # firehose files haven't left the device yet. If EVERY remaining segment
      # is un-uploaded (truly out of space), the oldest is still deleted so
      # logging never stalls -- and we log that data loss explicitly.
      ordered = sorted(dirs, key=lambda d: (d in DELETE_LAST, d in preserved_dirs, d in unuploaded_dirs))
      for delete_dir in ordered:
        delete_path = os.path.join(Paths.log_root(), delete_dir)

        if any(name.endswith(".lock") for name in os.listdir(delete_path)):
          continue

        try:
          # uploadprio2pnw: the ORDERING above uses the reduced set, but the ALARM uses the full one.
          # A segment can be safe to delete early (its only un-uploaded file is one we deliberately
          # skip) and still be data that never reached the backend — that must never go out as a
          # routine info line. One extra scan, of the single directory we are about to destroy.
          if delete_dir in unuploaded_dirs or has_unuploaded_firehose(delete_dir):
            # last resort: nothing fully-uploaded left to free; we are about to
            # delete data that never made it to the backend.
            cloudlog.error(f"connect2xnor: deleting UN-UPLOADED segment to free space: {delete_path}")
          cloudlog.info(f"deleting {delete_path}")
          shutil.rmtree(delete_path)
          break
        except OSError:
          cloudlog.exception(f"issue deleting {delete_path}")
      exit_event.wait(.1)
    else:
      exit_event.wait(30)


def main():
  deleter_thread(threading.Event())


if __name__ == "__main__":
  main()
