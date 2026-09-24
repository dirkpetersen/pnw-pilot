import datetime
import functools
import math
from pathlib import Path

MIN_DATE = datetime.datetime(year=2025, month=2, day=21)
MAX_DATE = datetime.datetime(year=2035, month=1, day=1)

def min_date():
  # on systemd systems, the default time is the systemd build time
  systemd_path = Path("/lib/systemd/systemd")
  if systemd_path.exists():
    d = datetime.datetime.fromtimestamp(systemd_path.stat().st_mtime)
    return max(MIN_DATE, d + datetime.timedelta(days=1))
  return MIN_DATE

def system_time_valid():
  return min_date() < datetime.datetime.now() < MAX_DATE


# clockvalid2pnw: the SAME floor/ceiling system_time_valid() applies to "now", for any wall-clock timestamp
# (an mtime, a record's t). This is pnw's one definition of "this time can be real"; everything that decides
# whether a date is trustworthy (ces_pnw clockBad, archive names, the metered budget's day, net_events day
# rotation, the curve-DB shadow) calls it.
#
# Why this and not "after 2020": the 3X's RTC is dead, and before NTP/GPS sync systemd starts the clock at its
# own build time -- measured 2026-07-28 15:05 UTC on every unsynced boot (the device's clocks.valid, published
# by timed from system_time_valid(), was False on all 160 such messages in 16 boots and True after sync). A
# 2020 floor accepts that date as real. systemd's mtime + 1 day rejects it and moves forward on its own with
# every AGNOS update.
@functools.cache
def _wall_time_floor() -> float:
  """Epoch seconds below which no wall-clock time is trusted. Read once per process: the floor is a file
  on the read-only rootfs and only changes with an AGNOS update, which reboots."""
  try:
    st = Path("/lib/systemd/systemd").stat()
  except OSError as e:
    # min_date() falls back to MIN_DATE here, which ACCEPTS the pre-sync date -- the very bug this exists to
    # stop. On AGNOS the file is PID 1's binary, so this cannot happen on a booted device; if it ever does,
    # fail CLOSED (nothing is trusted: records get marked, metered uploads stop) and say so, once.
    from openpilot.common.swaglog import cloudlog
    cloudlog.error(f"time_helpers: cannot stat /lib/systemd/systemd ({type(e).__name__}) -- the wall clock is " +
                   "treated as UNTRUSTED for this whole process (clockBad records, no metered uploads)")
    return math.inf
  return max(MIN_DATE.timestamp(), st.st_mtime + 86400)


def wall_time_valid(t) -> bool:
  """True when the wall-clock timestamp `t` (epoch seconds) can be real. Garbage input is False; never raises."""
  try:
    t = float(t)
  except (TypeError, ValueError):
    return False
  return _wall_time_floor() < t < MAX_DATE.timestamp()
