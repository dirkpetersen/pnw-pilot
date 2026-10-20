"""curvedbpull2pnw: stdlib library for installing a staged curve-DB table on the device.

Ported from the private ssh gate (`curvedb-swap-gate`): decide, stage, the device loader in a limited child process,
backup + rotate (ONLY `-auto` backups), swap with a re-check right before the rename, verify, recover. The gate and
this library take THE SAME lock file, so they can never swap together. Rule 3: "being driven" is the gear
(GearPark), never IsOnroad. Nothing here reboots anything.

Every function raises Refused (a safety precondition said no) or Failed (a check failed) with the reason.
"""
from __future__ import annotations

import datetime as dt
import fcntl
import hashlib
import json
import os
import re
import resource
import subprocess
import sys
import time
import zoneinfo

ROOT = os.environ.get("CURVEDB_ROOT", "")          # test hook: prefixes every path
LIVE = ROOT + "/data/pnw/curvedb_v2"
NEW = LIVE + ".new"
REPL = LIVE + ".replaced"
LOCK_FILE = ROOT + "/data/pnw/curvedb-swap-gate.log.lock"   # the ssh gate's lock file: do NOT rename
PARAMS = ROOT + "/data/params/d"
SWAGLOG = ROOT + "/data/log"
ROWS, MAN = "curvedb_v2_rows.json.zst", "manifest.json"
MAX_ROWS = 4 * 1024 * 1024         # the real table is ~0.7 MB
MAX_MAN = 64 * 1024
MIN_FREE_MB = 500
MIN_UPTIME_S = 120
LOADER_AS_BYTES = 600 * 1024 * 1024
KEEP_AUTO = 5
LOADER_SCRIPT: str | None = None   # tests: a script printing the loader JSON, run instead of the real loader
LOADER_PY = """import json,sys
from openpilot.selfdrive.controls.lib.ces_pnw.curvedb_live import load_rows
idx, man = load_rows(sys.argv[1])
print(json.dumps({'rows': idx.n_rows, 'manifest_rows': man['rows_with_authority'], 'sha': man['sha256']}))
"""


class Refused(Exception):
  """A safety precondition refused."""


class Failed(Exception):
  """A check failed."""


def sha_of(path: str) -> str:
  h = hashlib.sha256()
  with open(path, "rb") as f:
    for b in iter(lambda: f.read(1 << 20), b""):
      h.update(b)
  return h.hexdigest()


def read_param(name: str) -> str:
  try:
    with open(f"{PARAMS}/{name}") as f:
      return f.read().strip()
  except OSError:
    return "?"


def selfd_uptime() -> str:
  """Seconds selfdrived has been up; '' = not running; '?' = ps failed."""
  me = str(os.getpid())
  for pid in sorted((p for p in os.listdir("/proc") if p.isdigit() and p != me), key=int):
    try:
      with open(f"/proc/{pid}/cmdline", "rb") as f:
        cmd = f.read().replace(b"\0", b" ").decode(errors="replace")
    except OSError:
      continue
    if "selfdrive.selfdrived.selfdrived" in cmd:
      r = subprocess.run(["ps", "-o", "etimes=", "-p", pid], capture_output=True, text=True, timeout=10)
      return r.stdout.strip() or "?"
  return ""


def free_mb() -> int:
  st = os.statvfs(ROOT + "/data")
  return st.f_bavail * st.f_frsize // (1024 * 1024)


def live_sha() -> str:
  p = f"{LIVE}/{ROWS}"
  return sha_of(p) if os.path.exists(p) else ""


def decide(gear: str, onroad: str, et: str) -> tuple[bool, str]:
  """Same rule as the ssh gate's decide() (test_swap.py pins them together). Rule 3: gear, not IsOnroad."""
  if onroad not in ("0", "1"):
    return False, f"device state unreadable (IsOnroad={onroad!r})"
  if onroad == "0":
    return True, "offroad"
  if gear not in ("0", "1"):
    return False, f"device state unreadable (GearPark={gear!r} while IsOnroad=1)"
  if gear == "0":
    return False, "driving (ignition on, not in Park)"
  try:
    up = float(et)
  except ValueError:
    return False, f"in Park but selfdrived uptime unknown ({et!r})"
  if up <= MIN_UPTIME_S:
    return False, f"in Park but selfdrived up only {up:.0f} s (need > {MIN_UPTIME_S})"
  return True, f"in Park, selfdrived up {up:.0f} s"


def now_state() -> tuple[str, str, str]:
  return read_param("GearPark"), read_param("IsOnroad"), selfd_uptime()


def lock(wait_s: float = 60.0) -> object:
  """One swapper at a time, shared with the ssh gate (flock). Keep the returned handle open while working."""
  os.makedirs(os.path.dirname(LOCK_FILE), exist_ok=True)
  fh = open(LOCK_FILE, "w")
  end = time.monotonic() + wait_s
  while True:
    try:
      fcntl.flock(fh, fcntl.LOCK_EX | fcntl.LOCK_NB)
      return fh
    except OSError:
      if time.monotonic() > end:
        fh.close()
        raise Failed("another curve-DB swap holds the lock") from None
      time.sleep(0.2)


def loader(path: str) -> dict:
  """The device's OWN loader on `path` in a child: nice 19, 600 MB address space, 120 s CPU, one BLAS thread, cwd '/'."""
  if LOADER_SCRIPT:
    cmd = ["nice", "-n", "19", sys.executable, LOADER_SCRIPT, path]
    env = dict(os.environ)
  else:
    cmd = ["nice", "-n", "19", "/usr/local/venv/bin/python3", "-c", LOADER_PY, path]
    env = dict(os.environ, PYTHONPATH="/data/openpilot:/data/openpilot/opendbc_repo:/data/pnw/agnos19-compat/site-packages")
  env.update(OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1")   # numpy's thread pool would not fit the address-space cap

  def limits():
    resource.setrlimit(resource.RLIMIT_AS, (LOADER_AS_BYTES, LOADER_AS_BYTES))
    resource.setrlimit(resource.RLIMIT_CPU, (120, 120))

  try:
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=120, env=env, cwd="/", preexec_fn=limits)
  except (OSError, subprocess.TimeoutExpired) as e:
    raise Failed(f"the loader could not run: {e}") from e
  if r.returncode != 0:
    raise Failed(f"the device loader refused the table: {(r.stderr or r.stdout).strip()[-300:]}")
  try:
    return json.loads(r.stdout.strip().splitlines()[-1])
  except (ValueError, IndexError):
    raise Failed(f"the loader printed no JSON: {r.stdout[-200:]!r}") from None


def staged_check(expect_sha: str | None = None) -> dict:
  """Re-verify the staging dir from scratch: both files, rows sha == manifest sha (== expect_sha), loader accepts."""
  for n in (ROWS, MAN):
    if not os.path.exists(f"{NEW}/{n}"):
      raise Failed(f"nothing staged ({n} missing)")
  try:
    with open(f"{NEW}/{MAN}") as f:
      man = json.load(f)
  except ValueError as e:
    raise Failed(f"staged manifest is not JSON: {e}") from e
  if not isinstance(man, dict):
    raise Failed("staged manifest is not a JSON object")
  got = sha_of(f"{NEW}/{ROWS}")
  if man.get("sha256") != got:
    raise Failed(f"staged rows sha {got[:8]} != manifest sha {str(man.get('sha256'))[:8]}")
  if expect_sha and got != expect_sha:
    raise Failed(f"staged sha {got[:8]} != expected {expect_sha[:8]}")
  v = loader(NEW)
  if v.get("sha") != got or v.get("rows") != v.get("manifest_rows") or not v.get("rows"):
    raise Failed(f"loader result inconsistent: {v}")
  return v


def _require_safe_state() -> None:
  ok, why = decide(*now_state())
  if not ok:
    raise Refused(why)


def stage(sha: str, rows: bytes, man: bytes) -> None:
  """Write both files to the staging dir (size caps, sha check, fsync). Caller holds the lock."""
  _require_safe_state()
  if not (0 < len(rows) <= MAX_ROWS and 0 < len(man) <= MAX_MAN):
    raise Failed(f"size out of range (rows {len(rows)} <= {MAX_ROWS}, manifest {len(man)} <= {MAX_MAN})")
  if free_mb() < MIN_FREE_MB:
    raise Refused(f"only {free_mb()} MB free on /data (need {MIN_FREE_MB})")
  if hashlib.sha256(rows).hexdigest() != sha:
    raise Failed(f"sha mismatch: received {hashlib.sha256(rows).hexdigest()[:8]}, announced {sha[:8]}")
  try:
    m = json.loads(man)
  except ValueError as e:
    raise Failed(f"bad manifest: {e}") from e
  if not isinstance(m, dict) or m.get("sha256") != sha:
    raise Failed("bad manifest: not an object with sha256 == the announced sha")
  subprocess.run(["rm", "-rf", NEW, REPL], check=True)   # our own staging dir / leftover of an interrupted swap, nothing else
  os.makedirs(NEW)
  for name, body in ((ROWS, rows), (MAN, man)):
    with open(f"{NEW}/{name}", "wb") as f:
      f.write(body)
      f.flush()
      os.fsync(f.fileno())
  dfd = os.open(NEW, os.O_RDONLY)
  try:
    os.fsync(dfd)
  finally:
    os.close(dfd)


def _today_pt() -> str:
  return dt.datetime.now(dt.UTC).astimezone(zoneinfo.ZoneInfo("America/Los_Angeles")).strftime("%Y%m%d")


def backup_name(today: str) -> str:
  n, bak = 0, f"{LIVE}.bak-{today}-auto"
  while os.path.exists(bak):
    if n >= 26:
      raise Failed(f"more than 27 swaps in one day ({today}): refusing to pick another backup name")
    bak = f"{LIVE}.bak-{today}-auto{'abcdefghijklmnopqrstuvwxyz'[n]}"
    n += 1
  return bak


def rotate() -> list[str]:
  """Keep the KEEP_AUTO newest backups made by us/the gate (name ends -auto[a-z]); nothing else is ever listed or deleted."""
  d, base = os.path.dirname(LIVE), os.path.basename(LIVE)
  pat = re.compile(re.escape(base) + r"\.bak-[0-9]{8}-auto[a-z]?$")
  autos = sorted((n for n in os.listdir(d) if pat.match(n)), reverse=True)
  gone = autos[KEEP_AUTO:]
  for n in gone:
    subprocess.run(["rm", "-rf", f"{d}/{n}"], check=True)
  return gone


def swap(sha: str) -> str:
  """Install the staged table. Re-checks the preconditions itself. Caller holds the lock. Returns the backup name."""
  ok, why = decide(*now_state())
  if not ok:
    raise Refused(why)
  if free_mb() < MIN_FREE_MB:
    raise Refused(f"only {free_mb()} MB free on /data (need {MIN_FREE_MB})")
  if not os.path.exists(f"{LIVE}/{ROWS}"):
    raise Failed("the live table dir is missing; recover first")
  staged_check(sha)
  bak = backup_name(_today_pt())
  subprocess.run(["cp", "-a", LIVE, bak], check=True)
  if sha_of(f"{LIVE}/{ROWS}") != sha_of(f"{bak}/{ROWS}"):
    raise Failed(f"backup {os.path.basename(bak)} does not match the live table")
  rotate()
  subprocess.run(["rm", "-rf", REPL], check=True)
  ok, why = decide(*now_state())      # validation and the backup took time: the car may have started moving
  if not ok:
    raise Refused(f"state changed during the swap preparation: {why}")
  os.rename(LIVE, REPL)
  try:
    os.rename(NEW, LIVE)
  except OSError as e:
    os.rename(REPL, LIVE)
    raise Failed(f"swap failed ({e}); the previous table was restored") from e
  subprocess.run(["rm", "-rf", REPL], check=True)
  os.utime(f"{LIVE}/{ROWS}")   # mtime = install time, so verify() ignores loads of the OLD table that happened during staging
  if live_sha() != sha:
    raise Failed(f"post-check: live sha {live_sha()[:8]} != {sha[:8]}")
  return os.path.basename(bak)


def verify(sha: str) -> tuple[str, str]:
  """Did a selfdrived start since the install log curvedb_v2_loaded? -> ('pending'|'loaded'|'mismatch', last sha8)."""
  p = f"{LIVE}/{ROWS}"
  if not os.path.exists(p):
    raise Failed("no live table")
  since = os.path.getmtime(p)
  last = None
  if os.path.isdir(SWAGLOG):
    pat = re.compile(r'"event\$s":\s*"curvedb_v2_loaded".*?"sha256\$s":\s*"([0-9a-f]{64})"')
    for n in sorted(os.listdir(SWAGLOG)):
      if not n.startswith("swaglog.") or os.path.getmtime(f"{SWAGLOG}/{n}") < since:
        continue
      with open(f"{SWAGLOG}/{n}", errors="replace") as f:
        for ln in f:
          if "curvedb_v2_loaded" in ln:
            m = pat.search(ln)
            if m:
              last = m.group(1)
  if last is None:
    return "pending", ""
  return ("loaded" if last == sha else "mismatch"), last[:8]


def recover() -> str:
  """The live table is missing: put back the interrupted swap's copy, or the newest -auto backup."""
  if os.path.exists(f"{LIVE}/{ROWS}"):
    raise Failed("the live table exists; nothing to recover")
  if os.path.exists(REPL):
    os.rename(REPL, LIVE)
    return os.path.basename(REPL)
  d, base = os.path.dirname(LIVE), os.path.basename(LIVE)
  pat = re.compile(re.escape(base) + r"\.bak-[0-9]{8}-auto[a-z]?$")
  autos = sorted(n for n in os.listdir(d) if pat.match(n))
  if not autos:
    raise Failed("NO AUTO BACKUP to restore from")
  subprocess.run(["rm", "-rf", LIVE], check=True)   # a half-copied/empty live dir without a rows file
  subprocess.run(["cp", "-a", f"{d}/{autos[-1]}", LIVE], check=True)
  return autos[-1]
