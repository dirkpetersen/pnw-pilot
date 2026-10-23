"""curvedbpull2pnw: the device PULLS the approved curve-DB table (design: CURVEDB-AUTOINSTALL.md).

One cycle: toggle -> valid clock -> unmetered network -> GET the approved pointer -> current? -> hand-change hold ->
decide() -> download rows + manifest -> device loader -> swap -> record + report. NEVER reboots, never touches panda.
Subscribes to deviceState ONLY; "being driven" is the GearPark param (Rule 3), never carState (a background process
must not subscribe to a 100 Hz stream). Every step is logged: /data/pnw/curvedb-autoinstall.log, cloudlog.event and the
CurveDbAutoInstallStatus param (shown under the toggle). Presigned URLs are never logged.

`python -m openpilot.system.curvedb.curvedb_updated --adopt` records the live table as ours after a hand change.
"""
from __future__ import annotations

import datetime as dt
import hashlib
import json
import os
import re
import subprocess
import sys
import time
import zoneinfo

import requests

from openpilot.common.swaglog import cloudlog
from openpilot.common.time_helpers import wall_time_valid
from openpilot.system.curvedb import status as st
from openpilot.system.curvedb import swap

LOG_FILE = swap.ROOT + "/data/pnw/curvedb-autoinstall.log"
RECORD_FILE = swap.ROOT + "/data/pnw/curvedb_autoinstall.json"
HOURLY_S = 3600.0
REPORT_EVERY_S = 86400.0
POINTER_MAX = 16 * 1024
HEX64 = re.compile(r"[0-9a-f]{64}")
PT = zoneinfo.ZoneInfo("America/Los_Angeles")
# our internal cycle state -> the gateway's report enum (waiting_clock / waiting_unmetered / disabled are never sent: no report
# goes out until the clock and network checks passed, and never while the toggle is ON)
REPORT_STATE = {"installed": "installed", "current": "current", "none": "none_published", "held": "held_hand_change",
                "waiting": "refused", "retry": "error", "error": "error"}


class Stop(Exception):
  """End this cycle in `state` (disabled/waiting/current/installed/none/held/retry/error). delay None = exponential backoff."""
  def __init__(self, state: str, text: str, reason: str = "", delay: float | None = HOURLY_S, loud: bool = False):
    super().__init__(text)
    self.state, self.text, self.reason, self.delay, self.loud = state, text, reason or text, delay, loud


def clean(s: str, n: int = 200) -> str:
  return re.sub(r"[^\x20-\x7e]", "?", str(s))[:n]


def _md(epoch: float) -> str:
  return dt.datetime.fromtimestamp(epoch, PT).strftime("%m-%d")


def default_token(dongle: str) -> str:
  from openpilot.common.api import Api
  return Api(dongle).get_token()


class Updater:
  def __init__(self, params, net_fn, http=requests, token_fn=default_token, wall=time.time,  # noqa: TID251 -- wall clock by definition (token expiry, dates)
                 mono=time.monotonic):
    self.params, self.net_fn, self.http, self.token_fn, self.wall, self.mono = params, net_fn, http, token_fn, wall, mono
    self.fail_n = 0
    self._state_err: str | None = None
    self._param_errs: dict[str, str] = {}
    self.net_ok = False
    self.ctx: dict = {}
    self.rep_key = None
    self.rep_at = -1e18
    self.git_sha: str | None = None

  # ---- logging / status -------------------------------------------------------------------------------------------
  def log(self, step: str, result: str, reason: str = "", loud: bool = False) -> None:
    line = f"{dt.datetime.now(dt.UTC).strftime('%Y-%m-%dT%H:%M:%SZ')} {step} {result} {clean(reason)}"
    try:
      os.makedirs(os.path.dirname(LOG_FILE), exist_ok=True)
      if os.path.exists(LOG_FILE) and os.path.getsize(LOG_FILE) > 1_000_000:
        with open(LOG_FILE) as f:
          tail = f.readlines()[-500:]
        with open(LOG_FILE, "w") as f:
          f.writelines(tail)
      with open(LOG_FILE, "a") as f:
        f.write(line + "\n")
    except OSError as e:   # a log we cannot write must still be said (Rule 2)
      cloudlog.error(f"curvedb_updated: cannot write {LOG_FILE}: {e}")
    cloudlog.event("curvedb_autoinstall", step=step, result=result, reason=clean(reason), error=loud)

  def set_status(self, text: str) -> None:
    try:
      self.params.put("CurveDbAutoInstallStatus", st.pack(text, self.wall()))
    except Exception as e:   # the row would go stale: say so, keep running
      self._param_problem("CurveDbAutoInstallStatus", e)

  def _param_problem(self, name: str, e: Exception) -> None:
    """Log a Params failure once per distinct message (not per tick)."""
    msg = f"{name}: {type(e).__name__}: {clean(e, 80)}"
    if self._param_errs.get(name) != msg:
      self._param_errs[name] = msg
      self.log("param", "unreadable", msg, loud=True)

  def _bool(self, name: str, on_error: bool) -> bool:
    """A Params bool that cannot be read never crashes the daemon: log once, answer the SAFE value for this cycle."""
    try:
      return self.params.get_bool(name)
    except Exception as e:
      self._param_problem(name, e)
      return on_error

  # ---- the install record ---------------------------------------------------------------------------------------
  def read_record(self) -> dict:
    try:
      with open(RECORD_FILE) as f:
        d = json.load(f)
      if not isinstance(d, dict):
        raise ValueError("not an object")
      return d
    except FileNotFoundError:
      return {}
    except (OSError, ValueError) as e:
      self.log("record", "unreadable", f"{type(e).__name__}: {e} (treated as no install record)", loud=True)
      return {}

  def write_record(self, sha: str, adopted: bool = False) -> None:
    tmp = RECORD_FILE + ".tmp"
    os.makedirs(os.path.dirname(RECORD_FILE), exist_ok=True)
    with open(tmp, "w") as f:
      json.dump({"last_auto_sha": sha, "auto_utc": int(self.wall()), "adopted": adopted}, f)
      f.flush()
      os.fsync(f.fileno())
    os.replace(tmp, RECORD_FILE)

  def mark_loaded(self, sha: str | None) -> None:
    """curvedbloaded2pnw: persist that selfdrived loaded `sha` (None clears it). Swaglogs rotate and later cycles run
    while driving (no scan), so the evidence is kept per sha; write_record() drops it, so a new live sha starts unloaded."""
    rec = self.read_record()
    if sha is None:
      rec.pop("loaded_sha", None)
      rec.pop("loaded_utc", None)
    else:
      rec.update(loaded_sha=sha, loaded_utc=int(self.wall()))
    tmp = RECORD_FILE + ".tmp"
    try:
      with open(tmp, "w") as f:
        json.dump(rec, f)
        f.flush()
        os.fsync(f.fileno())
      os.replace(tmp, RECORD_FILE)
    except OSError as e:   # the evidence is real this cycle; only its persistence failed, so say so loudly
      self.log("record", "loaded_unsaved", f"could not persist loaded_sha: {type(e).__name__}: {e}", loud=True)

  # ---- network ----------------------------------------------------------------------------------------------------
  def _req(self, method: str, url: str, what: str, **kw):
    try:
      return self.http.request(method, url, timeout=(10, 30), **kw)
    except self.http.exceptions.RequestException as e:   # never str(e): it can carry the presigned URL
      raise Stop("retry", f"{what}: {type(e).__name__}", f"{what} failed: {type(e).__name__}", delay=None) from e

  def _backoff(self) -> float:
    self.fail_n += 1
    return min(1800.0, 60.0 * 2 ** (self.fail_n - 1))

  def get_pointer(self, base: str, dongle: str, tok: str) -> dict:
    r = self._req("GET", f"{base}/v1/curvedb/{dongle}/approved", "pointer GET", headers={"Authorization": "JWT " + tok})
    c = r.status_code
    if c == 404:
      raise Stop("none", "none published", "gateway has no approved table (404)")
    if c in (401, 403):
      raise Stop("error", f"error: auth HTTP {c}", f"the gateway refused the device token (HTTP {c})", loud=True)
    if c >= 500:
      raise Stop("retry", f"gateway unavailable (HTTP {c}), retrying", f"pointer GET HTTP {c}", delay=None)
    if c != 200:
      raise Stop("error", f"error: pointer HTTP {c}", f"unexpected pointer status {c}", loud=True)
    if len(r.content) > POINTER_MAX:
      raise Stop("error", "error: pointer too large", f"pointer body {len(r.content)} bytes", loud=True)
    try:
      p = r.json()
      assert isinstance(p, dict)
      ok = (HEX64.fullmatch(str(p["sha256"])) and HEX64.fullmatch(str(p["manifest_sha256"])) and
            type(p["rows_bytes"]) is int and type(p["manifest_bytes"]) is int and
            0 < p["rows_bytes"] <= swap.MAX_ROWS and 0 < p["manifest_bytes"] <= swap.MAX_MAN and
            str(p["rows_url"]).startswith("https://") and str(p["manifest_url"]).startswith("https://"))
    except (ValueError, KeyError, AssertionError, TypeError):
      ok = False
    if not ok:
      raise Stop("error", "error: bad pointer", "pointer JSON failed validation (fields/sizes/sha)", loud=True)
    return p

  def download(self, url: str, size: int, sha: str, what: str) -> bytes:
    r = self._req("GET", url, f"{what} download", stream=True)
    if r.status_code != 200:
      raise Stop("retry", f"{what} download HTTP {r.status_code}", f"{what} download HTTP {r.status_code}", delay=None)
    buf, h, t0 = bytearray(), hashlib.sha256(), self.mono()
    for chunk in r.iter_content(65536):
      buf += chunk
      if len(buf) > size:
        raise Stop("error", f"error: {what} larger than announced", f"{what} exceeds {size} bytes", loud=True)
      if self.mono() - t0 > 120:
        raise Stop("retry", f"{what} download too slow", f"{what} download > 120 s", delay=None)
      h.update(chunk)
    if len(buf) != size or h.hexdigest() != sha:
      raise Stop("error", f"error: {what} sha/size mismatch", f"{what}: got {len(buf)} B sha {h.hexdigest()[:8]}, want {size} B {sha[:8]}", loud=True)
    return bytes(buf)

  # ---- one cycle ------------------------------------------------------------------------------------------------
  def safe_now(self) -> bool:
    """Offroad, or in Park (GearPark param). Cheap; for edge-triggering a cycle only -- decide() is the gate."""
    try:
      return not self.params.get_bool("IsOnroad") or self.params.get_bool("GearPark")
    except Exception as e:   # an unreadable gear/ignition param REFUSES (like the gate's '?'): no edge trigger, said once per change
      msg = f"{type(e).__name__}: {clean(e, 80)}"
      if msg != self._state_err:
        self._state_err = msg
        self.log("state", "unreadable", f"IsOnroad/GearPark unreadable ({msg}); refusing to treat the car as safe", loud=True)
      return False

  def _run(self) -> Stop:
    from openpilot.system.hardware.base import NetworkType
    from openpilot.system.loggerd.uploader import effective_metered
    if self._bool("DisableCurveDbAutoInstall", True):    # unreadable toggle = treat as disabled
      raise Stop("disabled", "disabled", "toggle Disable Curve DB Auto-Install is ON", delay=30.0)
    if not wall_time_valid(self.wall()):
      raise Stop("waiting", "waiting for a valid clock", delay=60.0)
    net = self.net_fn()
    if net is None or net[0] == int(NetworkType.none):
      raise Stop("waiting", "waiting for a network", delay=60.0)
    if effective_metered(net[0], net[1], self._bool("OnPriorityNetwork", False)):
      raise Stop("waiting", "waiting for unmetered network", delay=300.0)
    self.net_ok = True
    from openpilot.common.api import API_HOST
    try:
      dongle = self.params.get("DongleId")
    except Exception as e:
      self._param_problem("DongleId", e)
      dongle = None
    dongle = dongle.decode() if isinstance(dongle, bytes) else (dongle or "")
    if not re.fullmatch(r"[0-9a-f]{16}", dongle):
      raise Stop("error", "error: no dongle id", f"DongleId unusable: {clean(dongle, 40)!r}", loud=True)
    try:
      tok = self.token_fn(dongle)
    except Exception as e:  # signing can fail in several library-specific ways; all are reported
      raise Stop("error", "error: cannot sign device token", f"{type(e).__name__}: {clean(e, 100)}", loud=True) from e
    ptr = self.get_pointer(API_HOST, dongle, tok)
    self.fail_n = 0
    sha = ptr["sha256"]
    self.ctx["pointer_sha"] = sha
    try:
      live = swap.live_sha()
    except OSError as e:
      raise Stop("error", "error: live table unreadable", f"live sha unreadable: {type(e).__name__}: {e}", loud=True) from e
    self.ctx["live_sha"] = live
    rec = self.read_record()
    if live == sha:
      return self._current(live, rec)
    if not live:
      try:
        with swap.lock():
          src = swap.recover()
      except swap.Refused as e:   # driving: leave /data alone; the next offroad/Park transition retries
        raise Stop("waiting", f"waiting: live table missing, {e}", f"live table missing, recover refused: {e}") from e
      except (swap.Failed, OSError, subprocess.CalledProcessError) as e:
        raise Stop("error", "error: live table missing", f"live table missing and recover failed: {clean(e)}", loud=True) from e
      raise Stop("held", "held: live table was missing, restored from backup", f"restored from {src}; run --adopt to resume", loud=True)
    if rec.get("last_auto_sha") != live:
      why = "no install record" if "last_auto_sha" not in rec else f"live {live[:8]} != recorded {str(rec['last_auto_sha'])[:8]}"
      raise Stop("held", "held: table changed by hand", f"{why}; run --adopt to resume", loud=True)
    ok, why = swap.decide(*swap.now_state())
    if not ok:
      raise Stop("waiting", f"waiting: {why}", why, delay=150.0 if "selfdrived up only" in why else HOURLY_S)
    try:
      free = swap.free_mb()
    except OSError as e:
      raise Stop("error", "error: cannot read free space", f"statvfs failed: {type(e).__name__}: {e}", loud=True) from e
    self.ctx["free_mb"] = free
    if free < swap.MIN_FREE_MB:
      raise Stop("waiting", f"waiting: only {free} MB free", f"only {free} MB free on /data (need {swap.MIN_FREE_MB})")
    self.set_status("downloading")
    rows = self.download(ptr["rows_url"], ptr["rows_bytes"], sha, "rows")
    man = self.download(ptr["manifest_url"], ptr["manifest_bytes"], ptr["manifest_sha256"], "manifest")
    try:
      lk = swap.lock()
    except swap.Failed as e:
      raise Stop("waiting", "waiting: another swap in progress", str(e), delay=300.0) from e
    try:
      swap.stage(sha, rows, man)
      bak = swap.swap(sha)
    except swap.Refused as e:
      raise Stop("waiting", f"waiting: {e}", f"swap refused: {e}", delay=300.0) from e
    except swap.Failed as e:
      raise Stop("error", f"error: {clean(e, 100)}", f"install failed: {e}", loud=True) from e
    except (OSError, subprocess.CalledProcessError) as e:   # makedirs / fsync / rm / cp / rename / utime
      raise Stop("error", f"error: install I/O failed ({type(e).__name__})", f"install failed: {type(e).__name__}: {clean(e, 120)}", loud=True) from e
    finally:
      lk.close()
    self.ctx["live_sha"] = sha
    try:
      self.write_record(sha)
    except OSError as e:   # installed, but unrecorded: the next cycle would call it a hand change, so say so loudly
      raise Stop("error", "error: installed but the install record could not be written", f"write_record failed: {type(e).__name__}: {e}", loud=True) from e
    self.log("swap", "ok", f"installed {sha[:8]}, backup {bak}")
    self.ctx["loaded"] = False
    raise Stop("installed", f"installed {sha[:8]} ({_md(self.wall())})", "installed; the loader runs at the next drive start")

  def _current(self, live: str, rec: dict) -> Stop:
    if rec.get("last_auto_sha") != live:
      try:
        self.write_record(live)
      except OSError as e:
        raise Stop("error", "error: could not update the install record", f"write_record failed: {type(e).__name__}: {e}", loud=True) from e
      self.log("record", "adopted", f"live {live[:8]} equals the approved table; install record updated")
      rec = {}   # the rewrite dropped any loaded evidence
    ok, why = swap.decide(*swap.now_state())      # the swaglog scan is disk I/O: never while driving (System Lagging)
    if not ok:
      v, got = "unknown", f"not checked, {why}"
    else:
      try:
        v, got = swap.verify(live)
      except swap.Failed as e:
        raise Stop("error", f"error: {clean(e, 100)}", f"verify failed: {e}", loud=True) from e
    if v == "unknown" and ok:
      self.log("verify", "unknown", got, loud=True)
    if v == "loaded" and rec.get("loaded_sha") != live:
      self.mark_loaded(live)
    elif v == "mismatch" and "loaded_sha" in rec:
      self.mark_loaded(None)
    sticky = v in ("unknown", "pending") and rec.get("loaded_sha") == live
    self.ctx["loaded"] = v == "loaded" or sticky
    if sticky:   # evidence persisted by an earlier cycle; today's check could not see it (driving / rotated swaglogs)
      at = rec.get("loaded_utc")
      v, got = "loaded", f"recorded {_md(at) if isinstance(at, int) else '?'}, now {v}{', ' + got if got else ''}"
    if v == "mismatch":
      raise Stop("error", f"error: selfdrived loaded {got}, not {live[:8]}", f"loaded sha {got} != live {live[:8]}", loud=True)
    try:
      date = _md(os.path.getmtime(f"{swap.LIVE}/{swap.ROWS}"))
    except OSError as e:
      raise Stop("error", "error: live table unreadable", f"getmtime failed: {type(e).__name__}: {e}", loud=True) from e
    return Stop("current", f"installed {live[:8]} ({date})",
                f"live table is the approved table (selfdrived load: {v}{', ' + got if v == 'unknown' or sticky else ''})")

  def cycle(self) -> float:
    self.ctx, self.net_ok = {}, False
    try:
      res = self._run()
    except Stop as s:
      res = s
    if res.delay is None:
      res.delay = self._backoff()
    self.log("cycle", res.state, res.reason, loud=res.loud)
    if res.loud:
      cloudlog.error(f"curvedb_updated: {res.text}: {res.reason}")
    self.set_status(res.text)
    self.maybe_report(res)
    return res.delay

  # ---- report ---------------------------------------------------------------------------------------------------
  def payload(self, res: Stop) -> dict:
    rec = self.read_record()
    if self.git_sha is None:
      try:
        from openpilot.common.git import get_commit
        self.git_sha = re.sub(r"[^A-Za-z0-9:._+-]", "", get_commit() or "")[:64] or "unknown"
      except Exception as e:
        self.log("report", "git_sha_unavailable", f"{type(e).__name__}: {e}")
        self.git_sha = "unknown"
    def sha(x):
      return x if isinstance(x, str) and HEX64.fullmatch(x) else None
    auto = rec.get("auto_utc")
    auto = dt.datetime.fromtimestamp(auto, dt.UTC).strftime("%Y-%m-%dT%H:%M:%SZ") if isinstance(auto, int) and auto > 0 else None
    return {"live_sha": sha(self.ctx.get("live_sha")), "pointer_sha": sha(self.ctx.get("pointer_sha")),
            "last_auto_sha": sha(rec.get("last_auto_sha")), "auto_utc": auto,
            "loaded": bool(self.ctx.get("loaded", False)), "state": REPORT_STATE[res.state], "reason": clean(res.reason),
            "free_mb": max(0, int(self.ctx.get("free_mb", swap.free_mb()))), "git_sha": self.git_sha}

  def maybe_report(self, res: Stop) -> None:
    if not self.net_ok:   # disabled / no clock / no network / metered: nothing is sent, the status already says why
      return
    key = (res.state, res.text, self.ctx.get("live_sha"), self.ctx.get("pointer_sha"), self.ctx.get("loaded"))   # loaded flips -> report now
    if key == self.rep_key and self.mono() - self.rep_at < REPORT_EVERY_S:
      return
    try:
      from openpilot.common.api import API_HOST
      dongle = self.params.get("DongleId")
      dongle = dongle.decode() if isinstance(dongle, bytes) else dongle
      r = self.http.request("POST", f"{API_HOST}/v1/curvedb/{dongle}/report", json=self.payload(res), timeout=(10, 30),
                            headers={"Authorization": "JWT " + self.token_fn(dongle)})
      if r.status_code == 400:   # the gateway's schema check: OUR bug. Say so loudly and do not retry-spin (next try on a state change / daily)
        self.rep_key, self.rep_at = key, self.mono()
        self.log("report", "REJECTED", "gateway answered 400 to the report: schema mismatch, fix the daemon", loud=True)
        return
      if r.status_code // 100 != 2:
        raise RuntimeError(f"HTTP {r.status_code}")
    except Exception as e:  # a report that did not go out is logged and retried next cycle, never fatal
      self.log("report", "failed", f"{type(e).__name__}: {clean(e, 100)} (retry next cycle)", loud=True)
      return
    self.rep_key, self.rep_at = key, self.mono()
    self.log("report", "sent", REPORT_STATE[res.state])


def adopt() -> int:
  sha = swap.live_sha()
  if not sha:
    print("curvedb_updated: no live table to adopt", file=sys.stderr)
    return 1
  u = Updater(None, lambda: None)
  u.write_record(sha, adopted=True)
  u.log("adopt", "ok", f"live {sha[:8]} recorded as the auto-install baseline")
  print(f"adopted {sha[:8]}")
  return 0


def main() -> None:
  if "--adopt" in sys.argv:
    sys.exit(adopt())
  from cereal import messaging
  from openpilot.common.params import Params
  try:
    os.nice(10)   # background work: never compete with the driving processes for CPU
  except OSError as e:
    cloudlog.warning(f"curvedb_updated: cannot renice: {e}")
  params = Params()
  sm = messaging.SubMaster(["deviceState"])

  def net():
    if not sm.seen["deviceState"]:
      return None
    d = sm["deviceState"]
    return d.networkType.raw, d.networkMetered

  up = Updater(params, net)
  cloudlog.warning("curvedb_updated: started")
  next_at, was_safe = 0.0, False
  try:
    while True:
      sm.update(10_000)
      now, safe = time.monotonic(), up.safe_now()
      if now >= next_at or (safe and not was_safe):
        next_at = now + up.cycle()
      was_safe = safe
  except Exception as e:
    up.set_status(f"error: daemon crashed: {type(e).__name__}")   # surface in the UI, then let the manager restart us
    raise


if __name__ == "__main__":
  main()
