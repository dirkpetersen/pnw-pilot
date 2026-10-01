#!/usr/bin/env python3
"""tailscale2pnw: remote SSH over a Tailscale tailnet. Manager process; runs only while TailscaleEnabled is on.

Reaches the device's existing OpenSSH (port 22, the owner's existing keys) from anywhere -- no Tailscale SSH.
Design rules (docs/pnw/TAILSCALE.md):
  * NOTHING FAILS SILENTLY: every subprocess return code is checked; every failure becomes the `TailscaleStatus`
    param ("error <reason>", shown under the toggle) and a cloudlog line. Transitions are logged change-only.
  * tailscaled never touches iptables: kernel mode passes --netfilter-mode=off, userspace mode has no netfilter
    at all (tethering NAT is iptables-legacy rules owned by network_arbiterd).
  * Pinned + sha256-verified binaries in /data/pnw/tailscale (system/tailscale/installer.py).
  * Auth key is read by tailscale itself from a file (`--auth-key=file:...`): never on a command line, never logged.
  * No periodic network use of our own: the 5-30 s poll is a local unix-socket call to tailscaled.
"""
from __future__ import annotations

import json
import os
import re
import signal
import subprocess
import time

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.system.tailscale import installer, status as st

STATE_DIR = installer.INSTALL_DIR
TAILSCALED = os.path.join(STATE_DIR, "tailscaled")
TAILSCALE = os.path.join(STATE_DIR, "tailscale")
STATE_FILE = os.path.join(STATE_DIR, "tailscaled.state")
SOCKET = os.path.join(STATE_DIR, "tailscaled.sock")
DAEMON_LOG = os.path.join(STATE_DIR, "tailscaled.log")  # truncated at each tailscaled start: bounded
AUTHKEY_FILE = "/data/pnw/secrets/tailscale.authkey"
PROC_DIR = "/proc"
TUN_DEVICE = "/dev/net/tun"

INSTALL_BACKOFF_MIN_S = 600.0   # a failed 36 MB download is retried no sooner than 10 min, doubling, capped at 1 h
INSTALL_BACKOFF_MAX_S = 3600.0
CONNECT_TIMEOUT_S = 180.0       # still not connected after this long -> say so (no route to the control plane?)
POLL_STEADY_S = 30.0      # connected: one local status call per 30 s
POLL_TRANSITION_S = 5.0   # connecting / just started
BACKOFF_MAX_S = 300.0
STARTUP_GRACE_S = 20.0    # tailscaled needs a moment before its socket answers
CLI_TIMEOUT_S = 10.0
UP_TIMEOUT_S = 30         # `tailscale up --timeout`; the subprocess timeout is a little longer
SHUTDOWN_DOWN_TIMEOUT_S = 1.5  # manager SIGKILLs us 5 s after SIGINT: the whole shutdown must fit
SHUTDOWN_WAIT_S = 1.0
SHUTDOWN_SETTLE_S = 0.5

_KEY_RE = re.compile(r"tskey-[A-Za-z0-9_-]+")


def redact(text: str) -> str:
  return _KEY_RE.sub("tskey-<redacted>", text)


class TailscaleDaemon:
  def __init__(self, params, run=subprocess.run, popen=subprocess.Popen, clock=time.monotonic,
               exists=os.path.exists, authkey_path: str = AUTHKEY_FILE, sleep=time.sleep,
               proc_dir: str | None = None):
    self._sleep = sleep
    self.proc_dir = proc_dir or PROC_DIR
    self._scan_err = ""
    self._logged: set[str] = set()   # error lines already written during this stop episode (change-only logging)
    self.stuck = False         # a tailscaled we failed to stop is still running while the toggle is OFF
    self.params = params
    self._run_fn = run
    self._popen = popen
    self._clock = clock
    self._exists = exists
    self.authkey_path = authkey_path

    self.proc = None
    self.adopted = False       # a tailscaled we did not start was already answering on the socket
    self.kernel_mode = False
    self.started_at = 0.0
    self.last_status: str | None = None
    self.fail_count = 0
    self.install_blocked = False   # permanent install failure: do not re-download until this process restarts
    self.install_fail_count = 0
    self.connecting_since: float | None = None
    self._logged_key_problem: str | None = None
    self._warned_key_perms = False

  # ---- publishing (change-only) -------------------------------------------------------------------------------
  def publish(self, text: str) -> None:
    if text == self.last_status:
      return
    first = self.last_status is None
    self.last_status = text
    self.params.put("TailscaleStatus", text)
    if first and text == st.OFF:
      return  # steady-state OFF at startup: no log line
    if text.startswith(st.ERROR_PREFIX) or text == st.NEEDS_AUTH_KEY:
      cloudlog.error(f"tailscale: {text}")
    else:
      cloudlog.warning(f"tailscale: {text}")

  def progress_connecting(self) -> None:
    """Publish 'connecting' unless an error is showing: a retry loop must not flap error<->connecting (and spam the log)
    on every attempt; the error stays until the node is actually Running."""
    if not (self.last_status or "").startswith(st.ERROR_PREFIX):
      self.publish(st.CONNECTING)

  def connecting(self, parsed: dict | None = None) -> float:
    """'connecting', but not forever: past CONNECT_TIMEOUT_S it becomes an error naming Tailscale's own health text."""
    now = self._clock()
    if self.connecting_since is None:
      self.connecting_since = now
    waited = now - self.connecting_since
    if waited < CONNECT_TIMEOUT_S:
      self.progress_connecting()
      return POLL_TRANSITION_S
    health = ((parsed or {}).get("Health") or ["no reason reported; is the Tailscale control plane reachable?"])[0]
    self.publish(st.error(redact(f"not connected for over {int(CONNECT_TIMEOUT_S // 60)} min: {health}")))
    return POLL_STEADY_S  # already reported; poll slowly

  def log_error_once(self, msg: str) -> None:
    """Error lines on the stop path repeat identically on every retry; write each distinct one once per episode."""
    if msg not in self._logged:
      self._logged.add(msg)
      cloudlog.error(msg)

  def fail(self, reason: str) -> float:
    """Publish an error, count it, return the backoff to wait before the next attempt."""
    self.fail_count += 1
    self.publish(st.error(redact(reason)))
    return min(BACKOFF_MAX_S, POLL_TRANSITION_S * (2 ** min(self.fail_count, 6)))

  # ---- subprocess helpers: ALWAYS return (rc, out, err), never swallow ---------------------------------------
  def _run(self, args: list[str], timeout: float) -> tuple[int, str, str]:
    try:
      p = self._run_fn(args, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
      return -1, "", f"timed out after {timeout:.0f}s"
    except OSError as e:
      return -1, "", f"cannot run {args[0]}: {e}"
    return p.returncode, p.stdout or "", p.stderr or ""

  def _cli(self, *args: str, timeout: float = CLI_TIMEOUT_S) -> tuple[int, str, str]:
    prefix = ["sudo", "-n"] if self.kernel_mode else []
    return self._run([*prefix, TAILSCALE, f"--socket={SOCKET}", *args], timeout)

  # ---- tailscaled lifecycle -------------------------------------------------------------------------------------
  def _detect_mode(self) -> None:
    """Kernel tun needs root (CAP_NET_ADMIN) via sudo; otherwise userspace networking as the comma user.
    Decided BEFORE anything talks to the socket, so every CLI call (adoption probe, up, down) uses the right user."""
    if not self._exists(TUN_DEVICE):
      cloudlog.warning(f"tailscale: {TUN_DEVICE} absent -> userspace networking")
      self.kernel_mode = False
    else:
      rc, _, err = self._run(["sudo", "-n", "true"], 5)
      if rc == 0:
        self.kernel_mode = True
      else:
        cloudlog.error(f"tailscale: {TUN_DEVICE} present but passwordless sudo failed (rc={rc}: {err.strip()}); "
                       + "falling back to userspace networking")
        self.kernel_mode = False

  def _daemon_cmd(self) -> list[str]:
    args = [TAILSCALED, f"--state={STATE_FILE}", f"--socket={SOCKET}", "--no-logs-no-support"]
    if not self.kernel_mode:
      args.append("--tun=userspace-networking")
    return ["sudo", "-n", *args] if self.kernel_mode else args

  def _daemon_tail(self) -> str:
    try:
      with open(DAEMON_LOG) as f:
        lines = [ln.strip() for ln in f.read().splitlines() if ln.strip()]
    except OSError as e:
      return f"(no daemon log: {e})"
    return lines[-1] if lines else "(daemon log empty)"

  def _start_tailscaled(self) -> float:
    self._detect_mode()
    rc, _, _ = self._cli("status", "--json")
    if rc == 0:
      self.adopted = True  # e.g. a previous supervisor was SIGKILLed; do not start a second daemon
      cloudlog.warning("tailscale: adopting a tailscaled that was already running")
      self.started_at = self._clock()
      return POLL_TRANSITION_S
    cmd = self._daemon_cmd()
    try:
      os.makedirs(STATE_DIR, mode=0o700, exist_ok=True)
      logf = open(DAEMON_LOG, "w")  # the child inherits it; closed right after Popen
    except OSError as e:
      return self.fail(f"cannot open {DAEMON_LOG}: {e}")
    try:
      self.proc = self._popen(cmd, stdout=logf, stderr=subprocess.STDOUT, start_new_session=True,
                              preexec_fn=lambda: os.nice(10))  # background priority; sudo passes it to tailscaled
    except OSError as e:
      return self.fail(f"cannot start tailscaled: {e}")
    finally:
      logf.close()  # the child holds its own descriptor
    self.started_at = self._clock()
    cloudlog.warning(f"tailscale: started tailscaled pid={self.proc.pid} mode={'kernel' if self.kernel_mode else 'userspace'}")
    self.connecting_since = None
    self.progress_connecting()
    return POLL_TRANSITION_S

  def _still_running(self) -> bool:
    """True if any tailscaled is alive: pgrep finds it, or it still answers on the socket. A pgrep that cannot run
    is treated as 'alive' -- an error must never read as 'gone'."""
    rc, _, err = self._run(["pgrep", "-x", "tailscaled"], 1)
    if rc == 0:
      return True
    if rc != 1:
      self.log_error_once(f"tailscale: pgrep failed rc={rc}: {err.strip()}")
      return True
    return self._cli("status", "--json", timeout=1.0)[0] == 0

  def stop_tailscaled(self) -> bool:
    """Stop tailscaled (ours, adopted, or orphaned by a SIGKILLed supervisor) and VERIFY it is gone."""
    if self.proc is not None:
      if self.proc.poll() is None:
        self.proc.terminate()  # signals sudo, which relays to the root tailscaled
        try:
          self.proc.wait(timeout=SHUTDOWN_WAIT_S)
        except subprocess.TimeoutExpired:
          self.log_error_once("tailscale: sudo/tailscaled ignored SIGTERM")
      self.proc = None
    if self._still_running():
      # proc.kill() would only kill the sudo wrapper and orphan the root daemon: signal tailscaled by name instead.
      prefix = ["sudo", "-n"] if self.kernel_mode else []
      rc, out, err = self._run([*prefix, "pkill", "-x", "tailscaled"], 1.5)
      if rc != 0:
        self.log_error_once(f"tailscale: pkill tailscaled failed rc={rc}: {(err.strip() or out.strip())[-100:]}")
      self._sleep(SHUTDOWN_SETTLE_S)
      return not self._still_running()
    return True

  # ---- auth key ---------------------------------------------------------------------------------------------------
  def _authkey_state(self) -> str | None:
    """None when the key file is usable, else the reason it is not."""
    try:
      s = os.stat(self.authkey_path)
      with open(self.authkey_path) as f:
        empty = not f.read().strip()
    except FileNotFoundError:
      return "missing"
    except OSError as e:
      return f"unreadable: {e}"
    if empty:
      return "empty"
    if s.st_mode & 0o077 and not self._warned_key_perms:
      self._warned_key_perms = True
      cloudlog.error(f"tailscale: {self.authkey_path} is accessible by group/others (mode {s.st_mode & 0o777:o}); chmod 600")
    return None

  # ---- one step of the state machine -----------------------------------------------------------------------------
  def tick(self) -> float:
    """Advance once; returns seconds to sleep before the next tick."""
    if not self.params.get_bool("TailscaleEnabled"):
      self._off_tick()
      return POLL_STEADY_S

    # Toggle ON but never enrolled and no usable key: nothing to do yet. Do not download, do not start tailscaled.
    if self.proc is None and not self.adopted and not self._exists(STATE_FILE):
      problem = self._authkey_state()
      if problem is not None:
        return self._needs_key(problem)

    if not installer.is_installed():
      if self.install_blocked:
        return 3600.0
      if self._driving():  # never download/extract on the driving hot path (36 MB + gunzip)
        self.publish(st.INSTALL_DEFERRED)
        return POLL_STEADY_S
      self.publish(st.INSTALLING)
      try:
        installer.ensure_installed()
      except installer.InstallError as e:
        self.install_fail_count += 1
        if e.permanent:
          self.install_blocked = True
        self.publish(st.error(redact(str(e))))
        return min(INSTALL_BACKOFF_MAX_S, INSTALL_BACKOFF_MIN_S * 2 ** (self.install_fail_count - 1))
      self.install_fail_count = 0

    if self.proc is None and not self.adopted:
      return self._start_tailscaled()
    if self.proc is not None and self.proc.poll() is not None:
      rc = self.proc.returncode
      self.proc = None
      return self.fail(f"tailscaled exited rc={rc}: {self._daemon_tail()}")

    rc, out, err = self._cli("status", "--json")
    if rc != 0:
      if self._clock() - self.started_at < STARTUP_GRACE_S:
        return self.connecting()
      return self.fail(f"tailscale status failed rc={rc}: {err.strip() or out.strip()}")
    try:
      parsed = json.loads(out)
      state, ip = st.classify(parsed)
    except (ValueError, AttributeError) as e:
      return self.fail(f"unparseable tailscale status: {e}")

    if state == "running":
      self.fail_count = 0
      self.connecting_since = None
      self.publish(st.connected(ip))
      return POLL_STEADY_S
    if state == "starting":
      return self.connecting(parsed)
    if state == "needs_approval":
      return self.fail("device needs approval in the Tailscale admin console")
    if state == "unknown":
      return self.fail(f"unrecognised tailscale state {parsed.get('BackendState')!r}")
    return self._bring_up(needs_login=(state == "needs_login"))

  def _bring_up(self, needs_login: bool) -> float:
    args = ["up", "--reset", f"--hostname={self._hostname()}", "--ssh=false", "--accept-dns=false",
            "--advertise-tags=tag:comma", f"--timeout={UP_TIMEOUT_S}s"]
    if self.kernel_mode:
      args.append("--netfilter-mode=off")
    if needs_login:  # first enrollment / expired node: needs the key. A merely 'Stopped' node re-ups with its state.
      problem = self._authkey_state()
      if problem is not None:
        return self._needs_key(problem)
      args.append(f"--auth-key=file:{self.authkey_path}")
    self.progress_connecting()
    rc, out, err = self._cli(*args, timeout=UP_TIMEOUT_S + 15)
    if rc != 0:
      return self.fail(f"tailscale up failed rc={rc}: {(err.strip() or out.strip())[-100:]}")
    return POLL_TRANSITION_S

  def _needs_key(self, problem: str) -> float:
    self.fail_count = 0
    self.publish(st.NEEDS_AUTH_KEY)  # change-only: one cloudlog line per state change, not per poll
    if problem != "missing" and problem != self._logged_key_problem:
      self._logged_key_problem = problem
      cloudlog.error(f"tailscale: auth key file {self.authkey_path} is {problem}")
    return POLL_STEADY_S

  def _driving(self) -> bool:
    """Rule 3: IsOnroad follows ignition (a parked, charging Lightning reads 1), so 'driving' = onroad AND not in Park."""
    return self.params.get_bool("IsOnroad") and not self.params.get_bool("GearPark")

  def _tailscaled_pids(self) -> list[int] | None:
    """Scan /proc/*/comm for exactly 'tailscaled' -- no subprocess. None = /proc could not be read (never 'none')."""
    try:
      entries = os.listdir(self.proc_dir)
    except OSError as e:
      msg = f"tailscale: cannot scan {self.proc_dir} for a leftover tailscaled: {e}"
      self._scan_err = msg
      self.log_error_once(msg)
      return None
    self._logged.discard(self._scan_err)  # scan works again: a recurrence must log again
    pids = []
    for name in entries:
      if not name.isdigit():
        continue
      try:
        with open(os.path.join(self.proc_dir, name, "comm"), "rb") as f:  # binary: a non-UTF-8 name must not raise
          comm = f.read().strip()
      except (FileNotFoundError, ProcessLookupError):
        continue  # the process exited while we scanned
      except OSError as e:
        self.log_error_once(f"tailscale: cannot read {self.proc_dir}/{name}/comm: {e}")
        return None
      if comm == b"tailscaled":
        pids.append(int(name))
    return pids

  def _off_tick(self) -> None:
    """Toggle OFF. Steady state costs one /proc scan per poll: no subprocess, no log, no write. A leftover root
    tailscaled (manager killed mid-shutdown, failed stop, orphan) is stopped here, since nothing else will notice it."""
    if self.proc is None and not self.adopted and not self.stuck:
      pids = self._tailscaled_pids()
      if pids is None:
        self.publish(st.error("cannot check for a leftover tailscaled (/proc unreadable)"))
        return
      if not pids:
        self.publish(st.OFF)
        return
      cloudlog.error(f"tailscale: toggle is OFF but tailscaled is running (pids {pids}); stopping it")
      self._detect_mode()
      self.stuck = True
    self.shutdown()

  def _hostname(self) -> str:
    dongle = self.params.get("DongleId")
    if isinstance(dongle, bytes):
      dongle = dongle.decode(errors="replace")
    if not dongle:
      cloudlog.error("tailscale: DongleId not set; using hostname 'comma-unknown'")
      dongle = "unknown"
    return f"comma-{str(dongle).lower()}"

  # ---- shutdown: toggle off, or the manager stopping us ----------------------------------------------------
  def shutdown(self, publish_off: bool = True) -> None:
    """publish_off=False after a crash, so the UI keeps showing the crash instead of a misleading 'off'."""
    if self.proc is None and not self.adopted and not self.stuck:
      if publish_off:
        self.publish(st.OFF)
      return
    rc, out, err = self._cli("down", timeout=SHUTDOWN_DOWN_TIMEOUT_S)
    if rc != 0:
      self.log_error_once(f"tailscale: `tailscale down` failed rc={rc}: {(err.strip() or out.strip())[-100:]}")
    gone = self.stop_tailscaled()
    self.adopted = False
    self.fail_count = 0
    self.connecting_since = None
    if not gone:
      # Never claim 'off' while the node may still be reachable. Retried every tick while the toggle stays off.
      self.stuck = True
      self.publish(st.error("tailscaled still running after toggle off; could not stop it"))
      return
    self.stuck = False
    self._logged.clear()
    if publish_off:
      self.publish(st.OFF)


def main() -> None:
  def _term(*_):
    raise SystemExit(0)  # run the finally block; SIGINT already raises KeyboardInterrupt
  signal.signal(signal.SIGTERM, _term)
  signal.signal(signal.SIGHUP, _term)  # `tmux kill-session` sends SIGHUP: without this the root tailscaled is orphaned

  daemon = TailscaleDaemon(Params())
  crashed = False
  cloudlog.warning("tailscale: tailscale_pnw started")
  try:
    while True:
      try:
        delay = daemon.tick()
      except Exception as e:  # surface in the UI, then re-raise so manager restarts us
        crashed = True
        daemon.publish(st.error(f"daemon crashed: {type(e).__name__}: {e}"))
        raise
      time.sleep(delay)
  finally:
    daemon.shutdown(publish_off=not crashed)


if __name__ == "__main__":
  main()
