#!/usr/bin/env python3
import os
import shutil
import threading
from openpilot.system.hardware.hw import Paths
from openpilot.common.swaglog import cloudlog
from openpilot.system.loggerd.config import get_available_bytes, get_available_percent
from openpilot.system.loggerd.uploader import listdir_by_creation, FIREHOSE_FILES, UPLOAD_ATTR_NAME, UPLOAD_ATTR_VALUE
from openpilot.system.loggerd.xattr_cache import getxattr

MIN_BYTES = 5 * 1024 * 1024 * 1024
MIN_PERCENT = 10

DELETE_LAST = ['boot', 'crash']

PRESERVE_ATTR_NAME = 'user.preserve'
PRESERVE_ATTR_VALUE = b'1'
PRESERVE_COUNT = 5


def has_preserve_xattr(d: str) -> bool:
  return getxattr(os.path.join(Paths.log_root(), d), PRESERVE_ATTR_NAME) == PRESERVE_ATTR_VALUE


def has_unuploaded_firehose(d: str) -> bool:
  # connect2xnor: True if this segment still has a large pass-2 file
  # (rlog/fcamera/ecamera) that has NOT yet been uploaded (no user.upload=1
  # xattr). Such segments are kept preferentially so proactive WiFi uploads can
  # finish before the data is pruned. A stat error -> treat the file as already
  # handled (don't let a flaky xattr read block freeing space).
  seg_path = os.path.join(Paths.log_root(), d)
  try:
    names = os.listdir(seg_path)
  except OSError:
    return False
  for name in names:
    if name not in FIREHOSE_FILES:
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
  while not exit_event.is_set():
    out_of_bytes = get_available_bytes(default=MIN_BYTES + 1) < MIN_BYTES
    out_of_percent = get_available_percent(default=MIN_PERCENT + 1) < MIN_PERCENT

    if out_of_percent or out_of_bytes:
      dirs = listdir_by_creation(Paths.log_root())
      preserved_dirs = get_preserved_segments(dirs)

      # connect2xnor: precompute which segments still have un-uploaded large
      # pass-2 files so they sort LAST (deleted only as a last resort).
      unuploaded_dirs = {d for d in dirs if has_unuploaded_firehose(d)}

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
          if delete_dir in unuploaded_dirs:
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
