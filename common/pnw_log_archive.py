"""cesarchive2pnw: put a FINISHED generation of a small device log into an upload archive directory.

The uploader (system/loggerd/uploader.py, PNW_LOG_SOURCES) only ever uploads files from a named
archive directory under a required filename prefix, never a file that is still being appended to.
These helpers are how net_events (location_servicesd) and the curve-DB observation corpus get their
generations there. ces_events keeps its own, older copy of the same naming rule in ces_pnw.py
(_archive_dest) -- see the note there before changing the shape of a name.

Every function here NEVER raises: they run inside logging threads and selfdrived's startup, where an
exception would take down something that matters more than a log. Every failure is LOGGED (Rule 2).
"""
import os
import shutil
import time

from openpilot.common.swaglog import cloudlog
# clockvalid2pnw: "is this mtime a real time" is time_helpers.wall_time_valid, shared with ces_pnw, the
# uploader and the curve-DB shadow. The 3X's RTC is dead: before NTP/GPS sync the clock reads systemd's build
# date (2026-07-28 on AGNOS 19.7), which a plain "after 2020" check took for real.
from openpilot.common.time_helpers import wall_time_valid


def archive_name(archive_dir: str, base: str, mtime: float) -> str:
  """`<archive_dir>/<base>.<UTC mtime stamp>`, unique in `archive_dir`. Named by content time so the
  uploader's lexical oldest-first walk is chronological. An untrusted (pre-sync) mtime gets a random `.b<hex>`
  token (a reused name is a silent S3 overwrite AND inherits the old file's upload xattr memo in
  xattr_cache); a same-second collision gets `.N`."""
  stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime(mtime))
  if not wall_time_valid(mtime):
    stamp = f"{stamp}.b{os.urandom(4).hex()}"
  dest = os.path.join(archive_dir, f"{base}.{stamp}")
  n = 1
  while os.path.exists(dest):
    dest = os.path.join(archive_dir, f"{base}.{stamp}.{n}")
    n += 1
  return dest


def link_into_archive(src: str, archive_dir: str, base: str) -> str | None:
  """Hardlink a CLOSED generation (nothing appends to `src` any more) into `archive_dir`. Same
  filesystem, so no bytes are copied. Returns the archive path, or None (logged) on failure."""
  dest = None
  try:
    st = os.stat(src)
    os.makedirs(archive_dir, exist_ok=True)
    dest = archive_name(archive_dir, base, st.st_mtime)
    os.link(src, dest)
    return dest
  except OSError as e:
    cloudlog.error(f"pnw_log_archive: hardlink FAILED ({type(e).__name__}: {e}) {src} -> {dest} -- this " +
                   "generation is NOT archived and will not reach S3")
    return None


def snapshot_into_archive(src: str, archive_dir: str, base: str) -> str | None:
  """Copy a file that is STILL APPENDED TO (a hardlink would keep growing with it) into `archive_dir`,
  unless a snapshot of this exact content already exists there.

  "Exact content" = the same mtime stamp: the name is `<base>.<mtime stamp>`, so an unchanged file
  maps to a name that is already present and nothing is copied. An untrusted mtime cannot be deduplicated
  that way (and cannot be named uniquely without a random token), so it is skipped and SAID.

  Written to a dot-prefixed temp name and renamed, so the uploader (which requires `<base>.` as the
  prefix) can never see a half-written copy. Returns the new path, or None (nothing to do / failed)."""
  try:
    st = os.stat(src)
  except FileNotFoundError:
    return None                        # no corpus yet: nothing to snapshot, not an error
  except OSError as e:
    cloudlog.error(f"pnw_log_archive: cannot stat {src} ({type(e).__name__}) -- NOT snapshotted")
    return None
  if not wall_time_valid(st.st_mtime):
    cloudlog.warning(f"pnw_log_archive: {src} has an untrusted (pre-sync) mtime ({st.st_mtime}) -- NOT snapshotted this " +
                     "time; it will be once a write lands with a synced clock")
    return None
  stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime(st.st_mtime))
  dest = os.path.join(archive_dir, f"{base}.{stamp}")
  if os.path.exists(dest):
    return None                        # this content is already archived
  tmp = os.path.join(archive_dir, f".{base}.{stamp}.tmp")
  try:
    os.makedirs(archive_dir, exist_ok=True)
    shutil.copyfile(src, tmp)
    os.utime(tmp, (st.st_atime, st.st_mtime))   # the copy carries the content time, for pruning order
    os.replace(tmp, dest)
    return dest
  except OSError as e:
    cloudlog.error(f"pnw_log_archive: snapshot FAILED ({type(e).__name__}: {e}) {src} -> {dest} -- this " +
                   "content is NOT archived and will not reach S3")
    try:
      os.unlink(tmp)
    except FileNotFoundError:
      pass
    except OSError as e2:
      cloudlog.error(f"pnw_log_archive: could not remove temp file {tmp} ({type(e2).__name__})")
    return None


def prune_archive(archive_dir: str, prefix: str, max_bytes: int) -> int:
  """Keep the `prefix` files in `archive_dir` under `max_bytes`, deleting the oldest (mtime) first.
  Returns how many were deleted. An eviction is data loss if the file had not reached S3 yet, so it is
  logged at error level, as is every failure to enforce the budget."""
  entries = []
  try:
    with os.scandir(archive_dir) as it:
      for e in it:
        if not e.name.startswith(prefix):
          continue
        try:
          if not e.is_file(follow_symlinks=False):
            continue
          st = e.stat(follow_symlinks=False)
        except OSError as ex:
          cloudlog.error(f"pnw_log_archive: stat failed for {e.path} ({type(ex).__name__}) -- not counted " +
                         f"against the {max_bytes} byte budget")
          continue
        entries.append((st.st_mtime, st.st_size, e.path))
  except FileNotFoundError:
    return 0
  except OSError as e:
    cloudlog.error(f"pnw_log_archive: listing {archive_dir} FAILED ({type(e).__name__}) -- the budget is NOT enforced")
    return 0
  total = sum(sz for _, sz, _ in entries)
  if total <= max_bytes:
    return 0
  entries.sort()
  removed = freed = 0
  for _, size, fp in entries:
    if total <= max_bytes:
      break
    try:
      os.remove(fp)
    except OSError as e:
      cloudlog.error(f"pnw_log_archive: could not delete {fp} ({type(e).__name__}) -- still over budget")
      continue
    total -= size
    freed += size
    removed += 1
  if removed:
    cloudlog.error(f"pnw_log_archive: {archive_dir} OVER BUDGET -- deleted the {removed} oldest {prefix}* " +
                   f"file(s) ({freed} bytes); any not yet uploaded are GONE")
  return removed
