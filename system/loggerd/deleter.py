#!/usr/bin/env python3
import errno
import json
import os
import shutil
import threading
import time

import xattr
from openpilot.common.params import Params
from openpilot.system.hardware.hw import Paths
from openpilot.common.swaglog import cloudlog
from openpilot.system.loggerd.config import get_available_bytes, get_available_percent
from openpilot.system.loggerd.uploader import (listdir_by_creation, FIREHOSE_FILES, UPLOAD_ATTR_NAME,
                                                UPLOAD_ATTR_VALUE, uploadable_firehose_files)

MIN_BYTES = 5 * 1024 * 1024 * 1024
MIN_PERCENT = 10

DELETE_LAST = ['boot', 'crash']

PRESERVE_ATTR_NAME = 'user.preserve'
PRESERVE_ATTR_VALUE = b'1'
PRESERVE_COUNT = 5

# deleterloss2pnw: shown on the home screen when a segment with un-uploaded firehose data is destroyed
LOSS_ALERT = "Offroad_UnuploadedDataDeleted"

# deleterloss2pnw: user.upload read failures seen in the current sweep; logged ONCE per sweep
# (a failed read still counts the file as uploaded -- never block freeing space -- but it is not silent)
_upload_read_errors: list[str] = []
# deleterrain2pnw: the same, for user.preserve reads (logged once per sweep, next to the upload-read line)
_preserve_read_errors: list[str] = []

# deleterrain2pnw: a whole sweep that raises is logged at most once per this many seconds (with a count)
SWEEP_ERR_LOG_S = 60.0
SWEEP_ERR_RETRY_S = 1.0


def has_preserve_xattr(d: str) -> bool:
  # rule2fixes2pnw: UNCACHED, same reason as user.upload below (deleterloss2pnw). loggerd sets user.preserve from
  # ANOTHER process, so xattr_cache's per-process memo kept the first "not set" answer until a reboot and a segment
  # preserved after that first sweep could be deleted as an ordinary one.
  path = os.path.join(Paths.log_root(), d)
  try:
    return getxattr_uncached(path, PRESERVE_ATTR_NAME) == PRESERVE_ATTR_VALUE
  except OSError as e:
    # deleterrain2pnw: a non-ENODATA error (EIO...) used to escape and END the deleter thread, which is not
    # restarted on crash (restart_if_crash=False) -> the disk fills and recording stops. An unreadable flag counts
    # as NOT preserved: the fail-safe direction for disk space is to never block freeing it, the same choice
    # unuploaded_firehose makes for user.upload. Not silent: logged once per sweep, naming the segment.
    _preserve_read_errors.append(f"{path}: {e}")
    return False


def getxattr_uncached(path: str, attr_name: str) -> bytes | None:
  # deleterloss2pnw: the uploader sets user.upload in ITS process, so xattr_cache's per-process memo in
  # THIS process never sees it until a reboot: segments uploaded since boot kept looking un-uploaded
  # (sorted as "keep", and fired false loss alarms). ENODATA/ENOATTR = not set, as in xattr_cache.
  try:
    return xattr.getxattr(path, attr_name)
  except OSError as e:
    if e.errno == errno.ENODATA or (hasattr(errno, 'ENOATTR') and e.errno == errno.ENOATTR):
      return None
    raise


def unuploaded_firehose(d: str, firehose_files: set[str] | None = None) -> list[str]:
  # deleterloss2pnw: the names behind has_unuploaded_firehose (below), for the loss alarm/record.
  firehose = FIREHOSE_FILES if firehose_files is None else firehose_files
  seg_path = os.path.join(Paths.log_root(), d)
  try:
    names = os.listdir(seg_path)
  except OSError:
    return []
  out = []
  for name in sorted(names):
    if name not in firehose:
      continue
    try:
      if getxattr_uncached(os.path.join(seg_path, name), UPLOAD_ATTR_NAME) != UPLOAD_ATTR_VALUE:
        out.append(name)
    except OSError as e:
      _upload_read_errors.append(f"{os.path.join(seg_path, name)}: {e}")
  return out


def has_unuploaded_firehose(d: str, firehose_files: set[str] | None = None) -> bool:
  # connect2xnor: True if this segment still has a large pass-2 file
  # (rlog/fcamera/ecamera) that has NOT yet been uploaded (no user.upload=1
  # xattr). Such segments are kept preferentially so proactive WiFi uploads can
  # finish before the data is pruned. A stat error -> treat the file as already
  # handled (don't let a flaky xattr read block freeing space).
  # deleterloss2pnw: ...but that read error is logged once per sweep (_upload_read_errors).
  # uploadprio2pnw: `firehose_files` is the set the uploader will EVER send (see
  # uploadable_firehose_files, whose docstring carries the full rationale). A PERMANENTLY skipped file
  # (SkipWideCameraUpload) never gets the xattr, so counting it here would pin EVERY segment as
  # un-uploaded and silently flatten the sort below into plain oldest-first. DeferHDVideoUpload is
  # deliberately NOT in that reduction -- it is a temporary hold whose segments must stay protected.
  # Defaults to the full set, which is also what the caller uses for the un-uploaded ERROR alarm.
  return bool(unuploaded_firehose(d, firehose_files))


def report_unuploaded_loss(seg: str, files: list[str], nbytes: int, n_boot: int, bytes_boot: int) -> None:
  """deleterloss2pnw: make a destroyed un-uploaded segment VISIBLE (Rule 2): an offroad alert the
  driver sees once parked, and one ces_events record (first in the metered upload order, rotated into
  the archive at Park). Never raises: recording must never stall on a failed alert or record."""
  try:
    # lazy: alertmanager pulls in selfdrived's event tables; an import failure must not stop the deleter
    from openpilot.selfdrive.selfdrived.alertmanager import set_offroad_alert
    set_offroad_alert(LOSS_ALERT, True, extra_text=f"{n_boot} segments, {bytes_boot / 1e6:.0f} MB since boot")
  except Exception:
    cloudlog.exception(f"deleterloss2pnw: FAILED to raise {LOSS_ALERT} for {seg} -- the driver will not see this loss")
  try:
    from openpilot.selfdrive.car.accdrop_pnw import append_to_ces_log
    t = round(time.time(), 1)  # noqa: TID251 -- wall clock, the ces_events "t" field is a UTC epoch
    rec = {"t": t, "ev": "deleterLoss", "seg": seg, "files": files, "bytes": nbytes, "n_boot": n_boot}
    append_to_ces_log(json.dumps(rec))
  except Exception:
    cloudlog.exception(f"deleterloss2pnw: FAILED to write the ces_events deleterLoss record for {seg}")


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
  loss_count = loss_bytes = 0  # deleterloss2pnw: un-uploaded segments/bytes destroyed since boot
  sweep_err_t: float | None = None
  sweep_err_n = 0
  while not exit_event.is_set():
    # deleterrain2pnw: ANY exception escaping a sweep used to end this thread for good (restart_if_crash=False),
    # after which nothing frees space and loggerd stops. Log it (throttled, with a count), wait, try again.
    try:
      loss_count, loss_bytes = _sweep(exit_event, params, loss_count, loss_bytes)
    except Exception:
      sweep_err_n += 1
      now = time.monotonic()
      if sweep_err_t is None or now - sweep_err_t >= SWEEP_ERR_LOG_S:
        cloudlog.exception(f"deleterrain2pnw: deleter sweep FAILED ({sweep_err_n} failure(s) since the last log) " +
                           f"-- retrying in {SWEEP_ERR_RETRY_S:.0f} s; no space is freed while this persists")
        sweep_err_t = now
        sweep_err_n = 0
      exit_event.wait(SWEEP_ERR_RETRY_S)


def _sweep(exit_event: threading.Event, params: Params, loss_count: int, loss_bytes: int) -> tuple[int, int]:
  # deleterrain2pnw: ONE sweep -- the body of deleter_thread's loop, moved here unchanged (dedented) so the loop can
  # catch anything that escapes it. Returns the updated since-boot loss counters.
  out_of_bytes = get_available_bytes(default=MIN_BYTES + 1) < MIN_BYTES
  out_of_percent = get_available_percent(default=MIN_PERCENT + 1) < MIN_PERCENT

  if out_of_percent or out_of_bytes:
    _upload_read_errors.clear()
    _preserve_read_errors.clear()
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

      try:
        # deleterrain2pnw: moved inside the try -- an OSError here (dir vanished, unreadable) used to escape the
        # sweep; now it is logged below and the NEXT candidate is tried instead of failing the whole sweep.
        if any(name.endswith(".lock") for name in os.listdir(delete_path)):
          continue

        # uploadprio2pnw: the ORDERING above uses the reduced set, but the ALARM uses the full one.
        # A segment can be safe to delete early (its only un-uploaded file is one we deliberately
        # skip) and still be data that never reached the backend — that must never go out as a
        # routine info line. One extra scan, of the single directory we are about to destroy.
        unuploaded = unuploaded_firehose(delete_dir)
        if delete_dir in unuploaded_dirs or unuploaded:
          # last resort: nothing fully-uploaded left to free; we are about to
          # delete data that never made it to the backend.
          cloudlog.error(f"connect2xnor: deleting UN-UPLOADED segment to free space: {delete_path}")
        # deleterloss2pnw: the VISIBLE alarm counts only files the uploader would ever send (the
        # ordering's set) -- a deliberately skipped ecamera is not a loss the driver can act on.
        lost = [n for n in unuploaded if n in firehose_files]
        lost_bytes = 0
        for n in lost:
          try:
            lost_bytes += os.path.getsize(os.path.join(delete_path, n))
          except OSError as e:
            cloudlog.error(f"deleterloss2pnw: cannot size {n} in {delete_path} ({e}) -- not counted in MB lost")
        cloudlog.info(f"deleting {delete_path}")
        shutil.rmtree(delete_path)
        if lost:
          loss_count += 1
          loss_bytes += lost_bytes
          report_unuploaded_loss(delete_dir, lost, lost_bytes, loss_count, loss_bytes)
        break
      except OSError:
        cloudlog.exception(f"issue deleting {delete_path}")
    if _upload_read_errors:
      cloudlog.error(f"deleterloss2pnw: {len(_upload_read_errors)} user.upload read error(s) this sweep, " +
                     f"counted as uploaded; first: {_upload_read_errors[0]}")
    if _preserve_read_errors:
      cloudlog.error(f"deleterrain2pnw: {len(_preserve_read_errors)} user.preserve read error(s) this sweep, " +
                     f"counted as NOT preserved; first: {_preserve_read_errors[0]}")
    exit_event.wait(.1)
  else:
    exit_event.wait(30)
  return loss_count, loss_bytes


def main():
  deleter_thread(threading.Event())


if __name__ == "__main__":
  main()
