"""tailscale2pnw daemon state machine, with subprocess mocked. Failure paths first: every one must end in an
'error ...' / 'unconfigured' / 'disconnected' status, never silence."""
import json
import os
import pathlib
import subprocess
from types import SimpleNamespace

import pytest

from openpilot.system.hardware.base import NetworkType
from openpilot.system.tailscale import installer, tailscale_pnw as tp

# Fake keys built at runtime so no literal in the source matches a secret-scanner pattern.
FAKE_KEY = "tsk" + "ey-auth-" + "notarealkey0"
FAKE_KEY2 = "tsk" + "ey-auth-" + "notarealkey1-notarealkey2"


class FakeParams:
  def __init__(self, enabled=True, dongle="abc123"):
    self.d = {"DisableTailscale": not enabled, "DongleId": dongle}  # enabled = Remote SSH enabled (the default)
    self.puts = []

  def get_bool(self, k):
    return bool(self.d.get(k))

  def get(self, k):
    return self.d.get(k)

  def put(self, k, v):
    self.puts.append((k, v))
    self.d[k] = v

  @property
  def statuses(self):
    return [v for k, v in self.puts if k == "TailscaleStatus"]


class FakeProc:
  def __init__(self, cmd, world=None):
    self.world = world
    self.cmd = cmd
    self.pid = 4242
    self.returncode = None
    self.terminated = False
    self.killed = False

  def poll(self):
    return self.returncode

  def terminate(self):
    self.terminated = True
    self.returncode = 0
    if self.world is not None and not self.world.ignore_term:
      self.world.alive = False

  def wait(self, timeout=None):
    return self.returncode

  def kill(self):
    self.killed = True
    self.returncode = -9


class Harness:
  """Scripted world: `status` is what `tailscale status --json` answers (dict, or an int rc for failure)."""

  def __init__(self, tmp_path, tun=False, sudo_ok=True, enabled=True, enrolled=True):
    self.params = FakeParams(enabled)
    self.tun = tun
    self.procdir = tmp_path.with_name(tmp_path.name + "_proc")   # fake /proc (outside tmp_path: 'OFF writes nothing')
    self.procdir.mkdir()
    self.ignore_term = False     # SIGTERM to the sudo wrapper does not stop the daemon
    self.pkill_works = True
    self.root_socket = False     # socket is root-only: the CLI works only through sudo
    self.enrolled = enrolled     # does the tailscaled state file exist?
    self.calls = []          # every run() arg list
    self.procs = []
    self.status = 1          # rc 1 = tailscaled not answering
    self.up_rc = 0
    self.down_rc = 0
    self.now = 1000.0
    self.sudo_ok = sudo_ok
    self.net = NetworkType.wifi   # what the device's own network state reports
    self.keyfile = tmp_path / "authkey"
    self.d = tp.TailscaleDaemon(self.params, run=self.run, popen=self.popen, clock=lambda: self.now,
                                exists=self._exists, authkey_path=str(self.keyfile),
                                sleep=lambda s: None, proc_dir=str(self.procdir), net_type=lambda: self.net)

  @property
  def alive(self):
    """A tailscaled process exists (root, own session): visible to pgrep, to the socket and to the /proc scan."""
    return (self.procdir / "4242" / "comm").exists()

  @alive.setter
  def alive(self, v):
    d = self.procdir / "4242"
    if v:
      d.mkdir(exist_ok=True)
      (d / "comm").write_text("tailscaled\n")
    elif (d / "comm").exists():
      (d / "comm").unlink()

  def _exists(self, p):
    return self.tun if p == tp.TUN_DEVICE else (self.enrolled if p == tp.STATE_FILE else True)

  def run(self, args, capture_output, text, timeout):
    self.calls.append(list(args))
    a = [x for x in args if x != "sudo" and x != "-n"]
    if a == ["true"]:
      return SimpleNamespace(returncode=0 if self.sudo_ok else 1, stdout="", stderr="" if self.sudo_ok else "sudo: a password is required")
    if a[:1] == ["pgrep"]:
      return SimpleNamespace(returncode=0 if self.alive else 1, stdout="4242\n" if self.alive else "", stderr="")
    if a[:1] == ["pkill"]:
      if self.pkill_works:
        self.alive = False
      return SimpleNamespace(returncode=0 if self.pkill_works else 1, stdout="", stderr="" if self.pkill_works else "not permitted")
    if self.root_socket and args[0] != "sudo" and a[:1] != ["true"]:
      return SimpleNamespace(returncode=1, stdout="", stderr="access denied")
    if "status" in a:
      if not self.alive:                              # nothing answers on the socket
        return SimpleNamespace(returncode=1, stdout="", stderr="cannot connect")
      if isinstance(self.status, int):
        return SimpleNamespace(returncode=self.status, stdout="", stderr="cannot connect")
      return SimpleNamespace(returncode=0, stdout=json.dumps(self.status), stderr="")
    if "up" in a:
      return SimpleNamespace(returncode=self.up_rc, stdout="", stderr="" if self.up_rc == 0 else f"invalid key {FAKE_KEY}")
    if "down" in a:
      return SimpleNamespace(returncode=self.down_rc, stdout="", stderr="" if self.down_rc == 0 else "boom")
    raise AssertionError(f"unexpected command {args}")

  def popen(self, cmd, **kw):
    p = FakeProc(cmd, self)
    self.alive = True
    self.procs.append(p)
    return p

  def cmds(self, word):
    return [c for c in self.calls if word in c]


@pytest.fixture
def h(tmp_path, monkeypatch):
  monkeypatch.setattr(tp.installer, "is_installed", lambda *a, **k: True)
  monkeypatch.setattr(tp, "DAEMON_LOG", str(tmp_path / "tailscaled.log"))
  monkeypatch.setattr(tp, "STATE_DIR", str(tmp_path))
  return Harness(tmp_path)


def run_until_up(h, status, key=True):
  """tick: start tailscaled, then (status answering) tick again."""
  if key:
    h.keyfile.write_text(FAKE_KEY + "\n")
    h.keyfile.chmod(0o600)
  h.d.tick()                 # starts tailscaled
  h.status = status
  return h.d.tick()


# ---- toggle off ----------------------------------------------------------------------------------------------------
def test_off_publishes_off_and_runs_nothing(tmp_path, monkeypatch):
  monkeypatch.setattr(tp.installer, "is_installed", lambda *a, **k: True)
  h = Harness(tmp_path, enabled=False)
  h.d.tick()
  h.d.tick()
  assert h.params.statuses == ["off"] and h.calls == [] and h.procs == []


def test_toggle_off_runs_down_and_stops_tailscaled(h):
  run_until_up(h, {"BackendState": "Running", "TailscaleIPs": ["100.64.0.5"]})
  assert h.params.statuses[-1] == "connected 100.64.0.5"
  h.params.d["DisableTailscale"] = True
  h.d.tick()
  assert h.cmds("down"), "tailscale down was not run"
  assert h.procs[0].terminated, "tailscaled was not stopped"
  assert h.params.statuses[-1] == "off"


def test_toggle_off_down_failure_is_logged_but_daemon_still_stopped(h, monkeypatch):
  errors = []
  monkeypatch.setattr(tp.cloudlog, "error", lambda m, *a, **k: errors.append(m))
  run_until_up(h, {"BackendState": "Running", "TailscaleIPs": ["100.64.0.5"]})
  h.down_rc = 1
  h.params.d["DisableTailscale"] = True
  h.d.tick()
  assert any("tailscale down` failed rc=1" in e for e in errors)
  assert h.procs[0].terminated and h.params.statuses[-1] == "off"


# ---- install ---------------------------------------------------------------------------------------------------------
def test_install_publishes_installing_then_error_on_sha_mismatch(tmp_path, monkeypatch):
  monkeypatch.setattr(tp.installer, "is_installed", lambda *a, **k: False)
  h = Harness(tmp_path)
  seen = []

  def ensure(*a, **k):
    seen.append(h.params.statuses[-1])
    raise installer.InstallError("sha256 mismatch (got aa, pinned bb); refusing to install", permanent=True)
  monkeypatch.setattr(tp.installer, "ensure_installed", ensure)
  h.d.tick()
  assert seen == ["installing"]
  assert h.params.statuses[-1].startswith("error sha256 mismatch")
  assert h.procs == [] and h.calls == []        # never starts the binary it refused
  h.d.tick()
  assert len(seen) == 1, "a permanent failure must not re-download every tick (36 MB each)"


def test_transient_download_failure_is_retried(tmp_path, monkeypatch):
  monkeypatch.setattr(tp.installer, "is_installed", lambda *a, **k: False)
  h = Harness(tmp_path)
  n = []

  def ensure(*a, **k):
    n.append(1)
    raise installer.InstallError("download failed: timed out")
  monkeypatch.setattr(tp.installer, "ensure_installed", ensure)
  d1 = h.d.tick()
  d2 = h.d.tick()
  assert len(n) == 2 and d2 > d1 and h.params.statuses[-1] == "error download failed: timed out"


# ---- starting tailscaled ----------------------------------------------------------------------------------------
def test_userspace_when_no_tun(h):
  h.d.tick()
  cmd = h.procs[0].cmd
  assert "--tun=userspace-networking" in cmd and "sudo" not in cmd
  assert f"--state={tp.STATE_FILE}" in cmd and f"--socket={tp.SOCKET}" in cmd
  assert h.params.statuses[-1] == "connecting"


def test_kernel_mode_uses_sudo_and_netfilter_off(tmp_path, monkeypatch):
  monkeypatch.setattr(tp.installer, "is_installed", lambda *a, **k: True)
  monkeypatch.setattr(tp, "DAEMON_LOG", str(tmp_path / "t.log"))
  monkeypatch.setattr(tp, "STATE_DIR", str(tmp_path))
  h = Harness(tmp_path, tun=True)
  run_until_up(h, {"BackendState": "Stopped"}, key=False)
  assert h.procs[0].cmd[:2] == ["sudo", "-n"] and "--tun=userspace-networking" not in h.procs[0].cmd
  up = h.cmds("up")[0]
  assert "--netfilter-mode=off" in up and up[:2] == ["sudo", "-n"]


def test_tun_but_no_sudo_falls_back_to_userspace_and_says_so(tmp_path, monkeypatch):
  monkeypatch.setattr(tp.installer, "is_installed", lambda *a, **k: True)
  monkeypatch.setattr(tp, "DAEMON_LOG", str(tmp_path / "t.log"))
  monkeypatch.setattr(tp, "STATE_DIR", str(tmp_path))
  errors = []
  monkeypatch.setattr(tp.cloudlog, "error", lambda m, *a, **k: errors.append(m))
  h = Harness(tmp_path, tun=True, sudo_ok=False)
  h.d.tick()
  assert "--tun=userspace-networking" in h.procs[0].cmd and "sudo" not in h.procs[0].cmd
  assert any("falling back to userspace" in e for e in errors)


def test_never_touches_iptables(h):
  run_until_up(h, {"BackendState": "NeedsLogin"})
  h.status = {"BackendState": "Running", "TailscaleIPs": ["100.64.0.5"]}
  h.d.tick()
  h.params.d["DisableTailscale"] = True
  h.d.tick()
  execs = {os.path.basename([a for a in c if a not in ("sudo", "-n")][0]) for c in [*h.calls, *(p.cmd for p in h.procs)]}
  allowed = {"tailscale", "tailscaled", "true", "pgrep", "pkill"}
  assert {"tailscale", "tailscaled"} <= execs <= allowed, execs  # no iptables / nft / sysctl / nmcli / ip anywhere


def test_tailscaled_start_failure_is_an_error(tmp_path, monkeypatch):
  monkeypatch.setattr(tp.installer, "is_installed", lambda *a, **k: True)
  monkeypatch.setattr(tp, "DAEMON_LOG", str(tmp_path / "t.log"))
  monkeypatch.setattr(tp, "STATE_DIR", str(tmp_path))
  h = Harness(tmp_path)

  def nopopen(cmd, **kw):
    raise PermissionError("exec format error")
  h.d._popen = nopopen
  h.d.tick()
  assert h.params.statuses[-1].startswith("error cannot start tailscaled: ")


def test_tailscaled_exit_surfaces_rc_and_log_tail(h):
  h.d.tick()                         # starting truncates the log; tailscaled then writes to it and dies
  pathlib.Path(tp.DAEMON_LOG).write_text("boot\nlisten tcp: address already in use\n")
  h.procs[0].returncode = 2
  h.d.tick()
  assert h.params.statuses[-1] == "error tailscaled exited rc=2: listen tcp: address already in use"
  h.d.tick()                        # next tick restarts it
  assert len(h.procs) == 2


def test_status_call_failing_after_grace_is_an_error_but_not_before(h):
  h.d.tick()
  h.status = 1
  h.now += 5
  h.d.tick()
  assert h.params.statuses[-1] == "connecting"
  h.now += 60
  h.d.tick()
  assert h.params.statuses[-1].startswith("error tailscale status failed rc=1")


# ---- auth ---------------------------------------------------------------------------------------------------------------
def test_logged_out_node_with_missing_key_is_an_error_and_never_calls_up(h):
  run_until_up(h, {"BackendState": "NeedsLogin"}, key=False)     # node state exists (enrolled) = configured
  assert h.params.statuses[-1] == "error node is logged out and the auth key file is missing"
  assert h.cmds("up") == []


def test_logged_out_node_with_empty_key_file_is_an_error(h):
  h.keyfile.write_text("  \n")
  run_until_up(h, {"BackendState": "NeedsLogin"}, key=False)
  assert h.params.statuses[-1] == "error node is logged out and the auth key file is empty" and h.cmds("up") == []


def test_needs_login_with_key_runs_up_with_file_key_and_pinned_flags(h):
  run_until_up(h, {"BackendState": "NeedsLogin"})
  (up,) = h.cmds("up")
  assert f"--auth-key=file:{h.keyfile}" in up
  for flag in ("--ssh=false", "--accept-dns=false", "--advertise-tags=tag:comma", "--hostname=comma-abc123"):
    assert flag in up
  assert "--shields-up" not in up                       # would block the inbound SSH this exists for
  assert not any(FAKE_KEY in " ".join(c) for c in h.calls), "key material must never be on a command line"


def test_up_failure_is_an_error_with_key_redacted(h):
  h.up_rc = 1
  run_until_up(h, {"BackendState": "NeedsLogin"})
  last = h.params.statuses[-1]
  assert last.startswith("error tailscale up failed rc=1") and FAKE_KEY not in last and "<redacted>" in last


def test_stopped_node_reups_without_a_key(h):
  run_until_up(h, {"BackendState": "Stopped"}, key=False)
  (up,) = h.cmds("up")
  assert not any(a.startswith("--auth-key") for a in up)


def test_needs_machine_approval_is_an_error(h):
  run_until_up(h, {"BackendState": "NeedsMachineAuth"})
  assert h.params.statuses[-1].startswith("error device needs approval")


def test_unknown_backend_state_is_an_error_not_connecting(h):
  run_until_up(h, {"BackendState": "Wat"})
  assert h.params.statuses[-1].startswith("error unrecognised tailscale state 'Wat'")


def test_unparseable_status_is_an_error(h):
  h.d.tick()
  h.status = {"BackendState": "Running"}
  h.d._run_fn = lambda args, **kw: SimpleNamespace(returncode=0, stdout="<html>", stderr="")
  h.d.tick()
  assert h.params.statuses[-1].startswith("error unparseable tailscale status")


# ---- connected + change-only ---------------------------------------------------------------------------------------
def test_running_reports_ip_and_publishes_change_only(h):
  run_until_up(h, {"BackendState": "Running", "TailscaleIPs": ["100.64.0.5", "fd7a::5"]})
  for _ in range(5):
    h.d.tick()
  assert h.params.statuses == ["connecting", "connected 100.64.0.5"]


def test_steady_poll_is_30s_and_transition_is_fast(h):
  assert run_until_up(h, {"BackendState": "Running", "TailscaleIPs": ["100.64.0.5"]}) == tp.POLL_STEADY_S
  h.status = {"BackendState": "Starting"}
  assert h.d.tick() == tp.POLL_TRANSITION_S


def test_dongle_missing_still_has_a_hostname_and_logs(h, monkeypatch):
  errors = []
  monkeypatch.setattr(tp.cloudlog, "error", lambda m, *a, **k: errors.append(m))
  h.params.d["DongleId"] = None
  run_until_up(h, {"BackendState": "Stopped"}, key=False)
  assert "--hostname=comma-unknown" in h.cmds("up")[0] and any("DongleId not set" in e for e in errors)


def test_up_timeout_is_an_error(h):
  def run(args, **kw):
    if "status" in args:
      return SimpleNamespace(returncode=0, stdout=json.dumps({"BackendState": "Stopped"}), stderr="")
    raise subprocess.TimeoutExpired(args, kw["timeout"])
  h.d.tick()
  h.d._run_fn = run
  h.d.tick()
  assert h.params.statuses[-1].startswith("error tailscale up failed rc=-1: timed out")


def test_redact():
  assert tp.redact(f"bad {FAKE_KEY2} end") == "bad tskey-<redacted> end"


# =====================================================================================================================
# Owner requirement 2026-09-30: an UNCONFIGURED Tailscale is fine and causes no issues. Cases A-E.
# =====================================================================================================================
def simulate(h, minutes, on_tick=None):
  """Drive the daemon for `minutes` of simulated time, sleeping exactly what tick() asks. Returns tick count."""
  end = h.now + minutes * 60
  ticks = 0
  while h.now < end:
    delay = h.d.tick()
    assert delay >= 1.0, "a delay under 1 s is a busy loop"
    ticks += 1
    h.now += delay
    if on_tick:
      on_tick()
    assert ticks < 5000, "runaway loop"
  return ticks


@pytest.fixture
def world(tmp_path, monkeypatch):
  """installer is observable: `installs` counts ensure_installed calls; `installed` flips is_installed."""
  w = SimpleNamespace(installs=[], installed=False, install_error=None)

  def ensure(*a, **k):
    w.installs.append(1)
    if w.install_error:
      raise w.install_error
    w.installed = True
  monkeypatch.setattr(tp.installer, "is_installed", lambda *a, **k: w.installed)
  monkeypatch.setattr(tp.installer, "ensure_installed", ensure)
  monkeypatch.setattr(tp, "DAEMON_LOG", str(tmp_path / "tailscaled.log"))
  monkeypatch.setattr(tp, "STATE_DIR", str(tmp_path))
  w.logs = []
  for lvl in ("info", "warning", "error"):
    monkeypatch.setattr(tp.cloudlog, lvl, lambda m, *a, _l=lvl, **k: w.logs.append((_l, m)))
  w.tmp = tmp_path
  return w


# ---- A: Remote SSH disabled (Disable toggle ON); unconfigured-but-default-ON -----------------------------------------------------------------------
def test_A_off_is_inert_for_two_hours(world, tmp_path):
  h = Harness(tmp_path, enabled=False, enrolled=False)
  ticks = simulate(h, 120)
  assert world.installs == [] and h.calls == [] and h.procs == [], "OFF must do no download, subprocess or spawn"
  assert h.params.statuses == ["off"]  # one write at startup, then nothing
  assert ticks <= 120 * 60 / tp.POLL_STEADY_S + 1       # param poll only, <= 1 per 30 s
  assert not any(tmp_path.iterdir()), "OFF must not write to disk"
  assert world.logs == [], "OFF in steady state must not log"


def test_A_manager_always_runs_it_but_it_is_inert_while_off():
  from openpilot.system.manager.process_config import always_run, managed_processes
  p = managed_processes["tailscale_pnw"]
  assert p.should_run is always_run and p.restart_if_crash   # inert while OFF: see test_A_off_is_inert_for_two_hours


def test_A_param_registered_default_enabled():
  """No UnknownKeyName / UI crash on a fresh install. toggles2pnw: Remote SSH is ENABLED by default (DisableTailscale
  default 0). The old TailscaleEnabled key is NOT registered any more and nothing in the tree reads it."""
  from openpilot.common.params import Params, UnknownKeyName
  params = Params()
  assert params.get_bool("DisableTailscale") is False
  assert params.get("TailscaleStatus") is None or isinstance(params.get("TailscaleStatus"), str)
  with pytest.raises(UnknownKeyName):
    params.get_bool("TailscaleEnabled")


def test_A_nothing_reads_the_retired_param():
  """No code path may read the old key (a stale reader would raise UnknownKeyName in a manager process / the UI)."""
  root = pathlib.Path(__file__).resolve().parents[3]
  readers = []
  for sub in ("system/tailscale", "selfdrive/ui/layouts", "system/manager"):
    p = root / sub
    for f in ([p] if p.is_file() else p.rglob("*.py")):
      if "tests" not in f.parts and "TailscaleEnabled" in f.read_text():
        readers.append(str(f.relative_to(root)))
  assert readers == [], readers


def test_A_default_on_but_unconfigured_is_harmless(world, tmp_path):
  """The default is now ON. A device with an EMPTY param store (default DisableTailscale=0), no auth key file and no
  enrolled node must: show 'unconfigured', download nothing, spawn nothing, run no subprocess, write nothing."""
  h = Harness(tmp_path, enrolled=False)
  h.params.d.pop("DisableTailscale")          # nothing stored: the registered default (0 = enabled) applies
  assert h.params.get_bool("DisableTailscale") is False
  simulate(h, 120)
  assert h.params.statuses == ["unconfigured"]
  assert world.installs == [] and h.procs == [] and h.calls == []
  assert not any(tmp_path.iterdir()), "unconfigured must not write to disk"
  assert world.logs == [], "unconfigured in steady state must not log"


# ---- B: enabled, nothing configured / key file unusable -------------------------------------------------------------------
def test_B_no_key_file_and_no_state_is_unconfigured_and_nothing_else(world, tmp_path):
  h = Harness(tmp_path, enrolled=False)
  simulate(h, 120)
  assert h.params.statuses == ["unconfigured"]
  assert world.installs == [], "unconfigured -> no 36 MB download"
  assert h.procs == [] and h.calls == [], "unconfigured -> tailscaled is not started, no subprocess at all"
  assert [m for lvl, m in world.logs] == [], "unconfigured: no log lines, not per poll and not at all"


@pytest.mark.parametrize("keyfile", ["empty", "whitespace", "unreadable"])
def test_B_a_key_file_that_exists_but_is_unusable_is_an_error_not_unconfigured(world, tmp_path, keyfile):
  h = Harness(tmp_path, enrolled=False)
  if keyfile == "empty":
    h.keyfile.write_text("")
  elif keyfile == "whitespace":
    h.keyfile.write_text(" \n\t\n")
  elif keyfile == "unreadable":
    h.keyfile.mkdir()        # open() raises IsADirectoryError, an OSError like a permission failure
  simulate(h, 120)
  assert len(h.params.statuses) == 1
  assert h.params.statuses[0].startswith("error auth key file is ")
  assert world.installs == [] and h.procs == [] and h.calls == []
  errs = [m for lvl, m in world.logs if lvl == "error" and "auth key file" in m]
  assert len(errs) == 1, "one error line per distinct problem, not per poll"


def test_B_key_arrives_later_then_it_installs_once_and_connects(world, tmp_path):
  h = Harness(tmp_path, enrolled=False)
  simulate(h, 10)
  assert world.installs == []
  h.keyfile.write_text(FAKE_KEY)
  h.keyfile.chmod(0o600)
  h.status = {"BackendState": "NeedsLogin"}
  simulate(h, 5)
  assert world.installs == [1] and len(h.procs) == 1


def test_B_failed_download_backs_off_at_least_10_min_and_is_bounded(world, tmp_path):
  world.install_error = installer.InstallError("download failed: timed out")
  h = Harness(tmp_path, enrolled=False)
  h.keyfile.write_text(FAKE_KEY)
  stamps = []
  orig = tp.installer.ensure_installed

  def stamp(*a, **k):
    stamps.append(h.now)
    return orig(*a, **k)
  tp.installer.ensure_installed = stamp
  simulate(h, 6 * 60)
  assert 2 <= len(stamps) <= 10
  assert all(b - a >= 600 for a, b in zip(stamps, stamps[1:], strict=False)), stamps
  assert h.params.statuses[-1] == "error download failed: timed out" and h.procs == []


def test_B_sha_mismatch_is_never_retried(world, tmp_path):
  world.install_error = installer.InstallError("sha256 mismatch", permanent=True)
  h = Harness(tmp_path, enrolled=False)
  h.keyfile.write_text(FAKE_KEY)
  simulate(h, 6 * 60)
  assert world.installs == [1] and h.procs == []


# ---- C: ON with a key but the world is broken ------------------------------------------------------------------------------
def test_C_no_internet_reports_a_reason_and_polls_boundedly(world, tmp_path):
  world.installed = True
  h = Harness(tmp_path)
  h.keyfile.write_text(FAKE_KEY)
  h.status = {"BackendState": "Starting", "Health": ["Tailscale can't reach the coordination server"]}
  ticks = simulate(h, 60)
  assert h.params.statuses[0] == "connecting"
  assert h.params.statuses[-1].startswith("error not connected for over 3 min") and "coordination server" in h.params.statuses[-1]
  assert ticks < 200 and len(h.procs) == 1 and world.installs == []     # no reinstall, no restart storm
  assert len([1 for lvl, m in world.logs if "not connected" in m]) == 1  # logged once, not per poll


def test_C_up_failing_backs_off_and_never_reinstalls(world, tmp_path):
  world.installed = True
  h = Harness(tmp_path)
  h.keyfile.write_text(FAKE_KEY)
  h.status = {"BackendState": "NeedsLogin"}
  h.up_rc = 1
  simulate(h, 60)
  ups = h.cmds("up")
  assert 5 <= len(ups) <= 25, len(ups)
  assert world.installs == [] and len(h.procs) == 1
  assert h.params.statuses[-1].startswith("error tailscale up failed")
  assert len([m for lvl, m in world.logs if m.startswith("tailscale: error tailscale up failed")]) == 1


def test_C_key_deleted_from_account_shows_the_reason(world, tmp_path):
  world.installed = True
  h = Harness(tmp_path)
  h.keyfile.write_text(FAKE_KEY)
  h.status = {"BackendState": "NeedsLogin"}
  h.up_rc = 1                      # control plane says: key expired/deleted/revoked
  h.d.tick()
  h.d.tick()
  assert h.params.statuses[-1].startswith("error tailscale up failed")


# ---- D: OFF at any time -------------------------------------------------------------------------------------------------
def test_D_off_while_connecting_stops_cleanly_and_keeps_state(world, tmp_path):
  world.installed = True
  h = Harness(tmp_path)
  h.keyfile.write_text(FAKE_KEY)
  h.status = {"BackendState": "Starting"}
  simulate(h, 1)
  h.params.d["DisableTailscale"] = True
  h.d.tick()
  assert h.cmds("down") and h.procs[0].terminated and not h.procs[0].killed
  assert not h.cmds("logout"), "state must be kept: never `tailscale logout`"
  assert h.params.statuses[-1] == "off"


# ---- E: installer never on the driving hot path -------------------------------------------------------------------------
def test_E_no_install_while_driving_then_installs_when_parked(world, tmp_path):
  h = Harness(tmp_path)
  h.keyfile.write_text(FAKE_KEY)
  h.params.d.update({"IsOnroad": True, "GearPark": False})
  simulate(h, 30)
  assert world.installs == [] and h.params.statuses == ["installing - deferred until parked"]
  h.params.d["GearPark"] = True      # parked + charging: IsOnroad is still 1, Rule 3
  h.status = {"BackendState": "NeedsLogin"}
  simulate(h, 2)
  assert world.installs == [1]


def test_E_offroad_installs(world, tmp_path):
  h = Harness(tmp_path)
  h.keyfile.write_text(FAKE_KEY)
  h.params.d.update({"IsOnroad": False, "GearPark": False})
  simulate(h, 1)
  assert world.installs == [1]


def test_E_import_is_free_of_side_effects():
  """Importing the daemon/installer does no I/O (it must never delay manager start)."""
  import ast
  import inspect
  for mod in (tp, installer):
    for node in ast.parse(inspect.getsource(mod)).body:
      assert not isinstance(node, ast.Expr) or isinstance(node.value, ast.Constant), f"top-level call in {mod.__name__}"


def test_orphan_tailscaled_is_adopted_not_duplicated_and_down_on_off(h):
  h.alive = True
  h.status = {"BackendState": "Running", "TailscaleIPs": ["100.64.0.9"]}
  h.d.tick()
  h.d.tick()
  assert h.procs == [] and h.params.statuses[-1] == "connected 100.64.0.9"
  h.params.d["DisableTailscale"] = True
  h.d.tick()
  assert h.cmds("down") and h.params.statuses[-1] == "off"


# ---- Fable F1/F2: toggle OFF must never read 'off' while a tailscaled can still be reached ---------------------------
@pytest.fixture
def kh(tmp_path, monkeypatch):
  """Kernel-mode world (tun + sudo), as on the device."""
  monkeypatch.setattr(tp.installer, "is_installed", lambda *a, **k: True)
  monkeypatch.setattr(tp, "DAEMON_LOG", str(tmp_path / "t.log"))
  monkeypatch.setattr(tp, "STATE_DIR", str(tmp_path))
  h = Harness(tmp_path, tun=True)
  return h


def test_F1_orphaned_root_tailscaled_is_adopted_with_sudo_and_off_really_stops_it(kh):
  """tmux kill-session left a root tailscaled behind; the next start adopts it through sudo, and OFF stops it."""
  kh.alive, kh.root_socket = True, True
  kh.status = {"BackendState": "Running", "TailscaleIPs": ["100.64.0.9"]}
  kh.d.tick()
  kh.d.tick()
  assert kh.procs == [] and kh.params.statuses[-1] == "connected 100.64.0.9"
  assert all(c[:2] == ["sudo", "-n"] for c in kh.cmds("status")), "adoption probe must use sudo in kernel mode"
  kh.params.d["DisableTailscale"] = True
  kh.d.tick()
  assert kh.alive is False and kh.params.statuses[-1] == "off"
  assert any(c[:2] == ["sudo", "-n"] for c in kh.cmds("down")) and kh.cmds("pkill")


def test_F1_off_with_an_unstoppable_daemon_is_error_never_off(kh):
  """Mutation F1b: publish 'off' without verifying the daemon is gone -> this fails."""
  kh.alive, kh.root_socket, kh.pkill_works = True, True, False
  kh.status = {"BackendState": "Running", "TailscaleIPs": ["100.64.0.9"]}
  kh.d.tick()
  kh.params.d["DisableTailscale"] = True
  kh.d.tick()
  assert kh.params.statuses[-1] == "error tailscaled still running after toggle off; could not stop it"
  assert "off" not in kh.params.statuses
  kh.d.tick()                         # retried while the toggle stays off
  assert len(kh.cmds("pkill")) == 2
  kh.pkill_works = True
  kh.d.tick()
  assert kh.params.statuses[-1] == "off"


def test_F2_sudo_wrapper_ignoring_sigterm_falls_back_to_pkill_by_name_and_verifies(kh):
  kh.status = {"BackendState": "Running", "TailscaleIPs": ["100.64.0.5"]}
  kh.d.tick()
  kh.d.tick()
  kh.ignore_term = True
  kh.params.d["DisableTailscale"] = True
  kh.d.tick()
  assert not kh.procs[0].killed, "proc.kill() would only kill the sudo wrapper"
  pk = kh.cmds("pkill")
  assert pk and pk[0][:2] == ["sudo", "-n"] and pk[0][-2:] == ["-x", "tailscaled"]
  assert kh.alive is False and kh.params.statuses[-1] == "off"


def test_F1_pgrep_failure_is_not_read_as_gone(kh):
  kh.status = {"BackendState": "Running", "TailscaleIPs": ["100.64.0.5"]}
  kh.d.tick()
  kh.d.tick()
  real = kh.run

  def run(args, capture_output, text, timeout):
    if "pgrep" in args:
      return SimpleNamespace(returncode=127, stdout="", stderr="pgrep: not found")
    return real(args, capture_output, text, timeout)
  kh.d._run_fn = run
  kh.params.d["DisableTailscale"] = True
  kh.d.tick()
  assert kh.params.statuses[-1].startswith("error tailscaled still running")


def test_F1_sighup_runs_the_shutdown_path(monkeypatch, tmp_path):
  """tmux kill-session sends SIGHUP: main() must turn it into SystemExit so the finally block stops tailscaled."""
  import signal
  handlers = {}
  (tmp_path / "proc").mkdir()
  monkeypatch.setattr(tp, "PROC_DIR", str(tmp_path / "proc"))  # never scan (or stop!) a real tailscaled of the dev box
  monkeypatch.setattr(tp.signal, "signal", lambda sig, fn: handlers.__setitem__(sig, fn))
  monkeypatch.setattr(tp, "Params", lambda: FakeParams(enabled=False))
  monkeypatch.setattr(tp.time, "sleep", lambda s: (_ for _ in ()).throw(KeyboardInterrupt()))
  with pytest.raises(KeyboardInterrupt):
    tp.main()
  assert signal.SIGHUP in handlers and signal.SIGTERM in handlers
  with pytest.raises(SystemExit):
    handlers[signal.SIGHUP](signal.SIGHUP, None)


def test_F6_up_is_idempotent_and_F7_daemon_is_niced(kh):
  seen = {}
  orig = kh.popen
  kh.popen = lambda cmd, **kw: (seen.update(kw), orig(cmd, **kw))[1]
  kh.d._popen = kh.popen
  kh.keyfile.write_text(FAKE_KEY)
  kh.d.tick()
  kh.status = {"BackendState": "NeedsLogin"}
  kh.d.tick()
  assert "--reset" in kh.cmds("up")[0]
  assert callable(seen["preexec_fn"])


# ---- Fable round 2: the process always runs; OFF notices and stops a leftover root tailscaled ------------------------
def test_off_finds_a_leftover_tailscaled_in_proc_and_stops_it_via_sudo(world, tmp_path):
  h = Harness(tmp_path, tun=True, enabled=False)
  h.alive = True                      # orphan from a killed manager; the daemon has no handle on it
  h.root_socket = True
  h.d.tick()
  assert h.alive is False and h.params.statuses[-1] == "off"
  assert any(c[:2] == ["sudo", "-n"] for c in h.cmds("down")), "mode must be detected before the stop path runs"
  assert h.cmds("pkill")[0][:2] == ["sudo", "-n"]
  assert any("Disable Remote SSH is ON but tailscaled is running" in m for _, m in world.logs)


def test_off_orphan_that_cannot_be_stopped_stays_error_retries_and_logs_once(world, tmp_path):
  """Mutation: disable the OFF-branch /proc probe -> the orphan is never noticed and this fails."""
  h = Harness(tmp_path, tun=True, enabled=False)
  h.alive, h.pkill_works = True, False
  simulate(h, 120)
  assert h.params.statuses == ["error tailscaled still running after toggle off; could not stop it"]
  assert len(h.cmds("pkill")) >= 100                    # retried every 30 s
  repeats = [m for _, m in world.logs if "pkill tailscaled failed" in m or "`tailscale down` failed" in m]
  assert len(repeats) == len(set(repeats)) <= 2, repeats  # each distinct line once, not per attempt
  h.pkill_works = True
  simulate(h, 1)
  assert h.params.statuses[-1] == "off" and h.alive is False


def test_off_steady_state_costs_nothing_over_two_hours(world, tmp_path):
  h = Harness(tmp_path, tun=True, enabled=False)
  (h.procdir / "999").mkdir()                            # a pid that vanished mid-scan (no comm file)
  (h.procdir / "1000").mkdir()
  (h.procdir / "1000" / "comm").write_text("tailscale\n")   # similar name is not tailscaled
  (h.procdir / "1001").mkdir()
  (h.procdir / "1001" / "comm").write_text("tailscaled-x\n")
  (h.procdir / "1002").mkdir()
  (h.procdir / "1002" / "comm").write_bytes(b"x\xff\xfey\n")   # non-UTF-8 name: must not crash the OFF branch
  simulate(h, 120)
  assert h.calls == [] and h.procs == [] and world.logs == [] and world.installs == []


def test_off_unreadable_proc_is_an_error_logged_once_never_off(world, tmp_path):
  h = Harness(tmp_path, enabled=False)
  h.d.proc_dir = str(tmp_path / "no-such-proc")
  simulate(h, 120)
  assert h.params.statuses == ["error cannot check for a leftover tailscaled (/proc unreadable)"]
  assert len([m for _, m in world.logs if "cannot scan" in m]) == 1
  assert h.calls == []


def test_off_scan_error_logs_again_after_it_recovers(world, tmp_path):
  h = Harness(tmp_path, enabled=False)
  good = h.d.proc_dir
  for _ in range(3):
    h.d.proc_dir = str(tmp_path / "gone")
    h.d.tick()
    h.d.tick()
    h.d.proc_dir = good
    h.d.tick()
  assert len([m for _, m in world.logs if "cannot scan" in m]) == 3


def test_tailscale_pnw_is_non_essential_in_selfdrived():
  """A dead remote-SSH daemon must never raise processNotRunning (a driving alert)."""
  import pathlib
  src = (pathlib.Path(tp.__file__).resolve().parents[2] / "selfdrive" / "selfdrived" / "selfdrived.py").read_text()  # source, not import:
  line = next(ln for ln in src.splitlines() if "NON_ESSENTIAL_PROCS = {" in ln)
  assert '"tailscale_pnw"' in line


# ---- F: configured but no internet at all -----------------------------------------------------------------------------
def test_F_offline_is_disconnected_once_with_no_error_log_and_no_backoff(world, tmp_path):
  h = Harness(tmp_path, enrolled=False)
  h.keyfile.write_text(FAKE_KEY)
  h.keyfile.chmod(0o600)
  world.install_error = installer.InstallError("download failed: timed out")   # would back off / spam if attempted
  h.net = NetworkType.none
  ticks = simulate(h, 120)
  assert h.params.statuses == ["disconnected - no internet"]
  assert world.installs == [] and h.procs == [] and h.calls == [], "offline: nothing attempted"
  assert [m for lvl, m in world.logs if lvl == "error"] == [], "offline must not log errors"
  assert len([m for lvl, m in world.logs if "no internet" in m]) == 1, "one line per transition"
  assert ticks <= 120 * 60 / tp.POLL_STEADY_S + 1, "no busy loop"


def test_F_link_returns_and_it_reconnects_by_itself(world, tmp_path):
  h = Harness(tmp_path, enrolled=False)
  h.keyfile.write_text(FAKE_KEY)
  h.keyfile.chmod(0o600)
  h.net = NetworkType.none
  simulate(h, 10)
  assert h.params.statuses == ["disconnected - no internet"]
  h.net = NetworkType.wifi
  h.status = {"BackendState": "NeedsLogin"}
  simulate(h, 5)
  assert world.installs == [1] and len(h.procs) == 1


def test_F_connected_then_link_lost_then_back_publishes_each_transition_once(h):
  run_until_up(h, {"BackendState": "Running", "TailscaleIPs": ["100.64.0.5"]})
  assert h.params.statuses[-1] == "connected 100.64.0.5"
  h.net = NetworkType.none
  h.now += 30
  h.d.tick()
  h.now += 30
  h.d.tick()
  assert h.params.statuses[-1] == "disconnected - no internet"
  assert not h.procs[0].terminated, "losing the link must not stop tailscaled"
  h.net = NetworkType.cell4G
  h.now += 30
  h.d.tick()
  assert h.params.statuses[-1] == "connected 100.64.0.5"
  assert h.params.statuses.count("disconnected - no internet") == 1


def test_F_offline_does_not_hide_a_disabled_toggle(h):
  """Disabled wins: the OFF path runs before any network check (tailscaled must still be stopped offline)."""
  run_until_up(h, {"BackendState": "Running", "TailscaleIPs": ["100.64.0.5"]})
  h.net = NetworkType.none
  h.params.d["DisableTailscale"] = True
  h.d.tick()
  assert h.procs[0].terminated and h.params.statuses[-1] == "off"


def test_F_unconfigured_wins_over_offline(world, tmp_path):
  h = Harness(tmp_path, enrolled=False)
  h.net = NetworkType.none
  simulate(h, 5)
  assert h.params.statuses == ["unconfigured"]
