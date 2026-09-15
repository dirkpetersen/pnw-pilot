import logging
import os
import stat
import time
import warnings
from pathlib import Path
from logging.handlers import BaseRotatingHandler

import zmq

from openpilot.common.logging_extra import SwagLogger, SwagFormatter, SwagLogFileFormatter
from openpilot.system.hardware.hw import Paths

# pnw (swaglogcap2pnw, owner 2026-09-14): keep up to 20,000 swaglog files (upstream: 2500), bounded by 200 MiB
# of closed files. Whichever limit is reached first deletes the oldest files. At the device's measured 8.5 KB
# per file and ~1 file a minute, the file cap binds: ~170 MB, ~14 days of uptime. The byte cap binds only when
# the average file is over ~10.5 KB, and bounds the worst case (20,000 x 256 KiB = 4.9 GiB) to 200 MiB. /data
# sits at the deleter's free-space floor, so every MB kept here costs about 1 MB of route data.
SWAGLOG_BACKUP_COUNT = 20000
SWAGLOG_MAX_TOTAL_BYTES = 200 * 1024 * 1024

# pnw (swaglogrot2pnw): say so, once per handler, when rotation deletes a log younger than this. Files roll
# over at most once a minute unless one reaches max_bytes, so 20,000 files hold ~14 days and 200 MiB holds
# 24 h unless the log averages over ~145 KB a minute. Deleting a file younger than 24 h means a log flood, or
# newest-first deletion again (the bug that silently erased 2026-09-05..09-12 on the device).
YOUNG_DELETE_WARN_S = 24 * 3600


def get_file_handler():
  Path(Paths.swaglog_root()).mkdir(parents=True, exist_ok=True)
  base_filename = os.path.join(Paths.swaglog_root(), "swaglog")
  handler = SwaglogRotatingFileHandler(base_filename)
  return handler

class SwaglogRotatingFileHandler(BaseRotatingHandler):
  def __init__(self, base_filename, interval=60, max_bytes=1024*256, backup_count=SWAGLOG_BACKUP_COUNT,
               max_total_bytes=SWAGLOG_MAX_TOTAL_BYTES, encoding=None):
    super().__init__(base_filename, mode="a", encoding=encoding, delay=True)
    self.base_filename = base_filename
    self.interval = interval # seconds
    self.max_bytes = max_bytes
    self.backup_count = backup_count
    self.max_total_bytes = max_total_bytes
    # pnw (swaglogcap2pnw): sizes of the closed log files, and their running total. Measured once here, then
    # kept current at each rollover and delete, so a rollover never stats the whole directory.
    self.log_sizes = self.get_existing_logfiles()
    self.total_bytes = sum(self.log_sizes.values())
    # newest first, matching _open()'s insert(0, ...) so doRollover()'s pop() deletes the oldest
    self.log_files = sorted(self.log_sizes, reverse=True)
    log_indexes = [f.split(".")[-1] for f in self.log_files]
    self.last_file_idx = max([int(i) for i in log_indexes if i.isdigit()] or [-1])
    self.last_rollover = None
    self.warned_young_delete = False
    self.doRollover()

  def _open(self):
    self.last_rollover = time.monotonic()
    self.last_file_idx += 1
    next_filename = f"{self.base_filename}.{self.last_file_idx:010}"
    stream = open(next_filename, self.mode, encoding=self.encoding)
    self.log_files.insert(0, next_filename)
    return stream

  def get_existing_logfiles(self):
    """Returns {path: size in bytes} of the existing log files."""
    log_sizes = {}
    failed = []
    base_dir = os.path.dirname(self.base_filename)
    for fn in os.listdir(base_dir):
      fp = os.path.join(base_dir, fn)
      if not fp.startswith(self.base_filename):
        continue
      try:
        st = os.stat(fp)
      except OSError as e:
        failed.append(f"{fn} ({type(e).__name__}: {e})")
        continue
      if stat.S_ISREG(st.st_mode):
        log_sizes[fp] = st.st_size
    if failed:
      cloudlog.error(f"swaglog rotation: could not read {len(failed)} log file(s) in {base_dir}, e.g. {failed[0]}. " +
                     "They are not counted toward either cap and will not be deleted by rotation.")
    return log_sizes

  def shouldRollover(self, record):
    size_exceeded = self.max_bytes > 0 and self.stream.tell() >= self.max_bytes
    time_exceeded = self.interval > 0 and self.last_rollover + self.interval <= time.monotonic()
    return size_exceeded or time_exceeded

  def doRollover(self):
    closed = None
    if self.stream:
      self.stream.close()
      closed = self.log_files[0]
    self.stream = self._open()
    # Only log once the new stream is open: this handler may be attached to cloudlog itself (manager.py's crash path).
    if closed is not None:
      self._count_closed_file(closed)

    while len(self.log_files) > 1:  # never the file just opened
      if self.backup_count > 0 and len(self.log_files) > self.backup_count:
        limit = f"file cap: {len(self.log_files)} files > {self.backup_count}"
      elif self.max_total_bytes > 0 and self.total_bytes > self.max_total_bytes:
        limit = f"byte cap: {self.total_bytes} bytes > {self.max_total_bytes}"
      else:
        break
      to_delete = self.log_files.pop()
      self.total_bytes -= self.log_sizes.pop(to_delete, 0)
      self._warn_if_young(to_delete, limit)
      try:
        os.remove(to_delete)
      except OSError as e:
        cloudlog.error(f"swaglog rotation could not delete {os.path.basename(to_delete)} ({limit}): {type(e).__name__}: {e}")

  def _count_closed_file(self, path):
    try:
      size = os.path.getsize(path)
    except OSError as e:
      cloudlog.error(f"swaglog rotation: could not read the size of {os.path.basename(path)} ({type(e).__name__}: {e}). " +
                     "It is not counted toward the byte cap.")
      return
    self.log_sizes[path] = size
    self.total_bytes += size

  def _warn_if_young(self, path, limit):
    if self.warned_young_delete:
      return
    try:
      age_s = time.time() - os.path.getmtime(path)  # noqa: TID251 -- file mtimes are wall-clock time
    except OSError:
      # Fable: a concurrent handler may remove the file first. logmessaged is restart_if_crash=False, so a raise
      # here would end all logging until the next boot; the age is simply unknown for this file.
      return
    # A negative age means the clock is behind the file's mtime (a boot before time sync), so the age is unknown.
    if 0 <= age_s < YOUNG_DELETE_WARN_S:
      self.warned_young_delete = True
      window_h = YOUNG_DELETE_WARN_S // 3600
      cloudlog.error(f"swaglog rotation deleted {os.path.basename(path)}, only {age_s / 3600:.1f} h old ({limit}): " +
                     f"swaglogs are not covering the last {window_h} h. Warning once per handler.")

class UnixDomainSocketHandler(logging.Handler):
  def __init__(self, formatter):
    logging.Handler.__init__(self)
    self.setFormatter(formatter)
    self.pid = None

    self.zctx = None
    self.sock = None

  def __del__(self):
    self.close()

  def close(self):
    if self.sock is not None:
      self.sock.close()
    if self.zctx is not None:
      self.zctx.term()

  def connect(self):
    self.zctx = zmq.Context()
    self.sock = self.zctx.socket(zmq.PUSH)
    self.sock.setsockopt(zmq.LINGER, 10)
    self.sock.connect(Paths.swaglog_ipc())
    self.pid = os.getpid()

  def emit(self, record):
    if os.getpid() != self.pid:
      # TODO suppresses warning about forking proc with zmq socket, fix root cause
      warnings.filterwarnings("ignore", category=ResourceWarning, message="unclosed.*<zmq.*>")
      self.connect()

    msg = self.format(record).rstrip('\n')
    # print("SEND".format(repr(msg)))
    try:
      s = chr(record.levelno)+msg
      self.sock.send(s.encode('utf8'), zmq.NOBLOCK)
    except zmq.error.Again:
      # drop :/
      pass


class ForwardingHandler(logging.Handler):
  def __init__(self, target_logger):
    super().__init__()
    self.target_logger = target_logger

  def emit(self, record):
    self.target_logger.handle(record)


def add_file_handler(log):
  """
  Function to add the file log handler to swaglog.
  This can be used to store logs when logmessaged is not running.
  """
  handler = get_file_handler()
  handler.setFormatter(SwagLogFileFormatter(log))
  log.addHandler(handler)


cloudlog = log = SwagLogger()
log.setLevel(logging.DEBUG)


outhandler = logging.StreamHandler()

print_level = os.environ.get('LOGPRINT', 'warning')
if print_level == 'debug':
  outhandler.setLevel(logging.DEBUG)
elif print_level == 'info':
  outhandler.setLevel(logging.INFO)
elif print_level == 'warning':
  outhandler.setLevel(logging.WARNING)

ipchandler = UnixDomainSocketHandler(SwagFormatter(log))

log.addHandler(outhandler)
# logs are sent through IPC before writing to disk to prevent disk I/O blocking
log.addHandler(ipchandler)
