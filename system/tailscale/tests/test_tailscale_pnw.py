"""tailscale2pnw daemon state machine, with subprocess mocked. Failure paths first: every one must end in an
'error ...' / 'needs auth key' status, never silence."""
import json
import os
import pathlib
import subprocess
from types import SimpleNamespace

import pytest

from openpilot.system.tailscale import installer, tailscale_pnw as tp


class FakeParams:
  def __init__(self, enabled=True, dongle="abc123"):
    self.d = {"TailscaleEnabled": enabled, "DongleId": dongle}
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
  def __init__(self, cmd):
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
    self.preexisting = False     # a tailscaled we did not start already answers on the socket
    self.enrolled = enrolled     # does the tailscaled state file exist?
    self.calls = []          # every run() arg list
    self.procs = []
    self.status = 1          # rc 1 = tailscaled not answering
    self.up_rc = 0
    self.down_rc = 0
    self.now = 1000.0
    self.sudo_ok = sudo_ok
    self.keyfile = tmp_path / "authkey"
    self.d = tp.TailscaleDaemon(self.params, run=self.run, popen=self.popen, clock=lambda: self.now,
                                exists=self._exists, authkey_path=str(self.keyfile))

  def _exists(self, p):
    return self.tun if p == tp.TUN_DEVICE else (self.enrolled if p == tp.STATE_FILE else True)

  def run(self, args, capture_output, text, timeout):
    self.calls.append(list(args))
    a = [x for x in args if x != "sudo" and x != "-n"]
    if a == ["true"]:
      return SimpleNamespace(returncode=0 if self.sudo_ok else 1, stdout="", stderr="" if self.sudo_ok else "sudo: a password is required")
    if "status" in a:
      if not self.procs and not self.preexisting:   # no tailscaled running yet: nothing answers on the socket
        return SimpleNamespace(returncode=1, stdout="", stderr="cannot connect")
      if isinstance(self.status, int):
        return SimpleNamespace(returncode=self.status, stdout="", stderr="cannot connect")
      return SimpleNamespace(returncode=0, stdout=json.dumps(self.status), stderr="")
    if "up" in a:
      return SimpleNamespace(returncode=self.up_rc, stdout="", stderr="" if self.up_rc == 0 else "invalid key tskey-auth-SECRET123")
    if "down" in a:
      return SimpleNamespace(returncode=self.down_rc, stdout="", stderr="" if self.down_rc == 0 else "boom")
    raise AssertionError(f"unexpected command {args}")

  def popen(self, cmd, **kw):
    p = FakeProc(cmd)
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
    h.keyfile.write_text("tskey-auth-SECRET123\n")
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
  h.params.d["TailscaleEnabled"] = False
  h.d.tick()
  assert h.cmds("down"), "tailscale down was not run"
  assert h.procs[0].terminated, "tailscaled was not stopped"
  assert h.params.statuses[-1] == "off"


def test_toggle_off_down_failure_is_logged_but_daemon_still_stopped(h, monkeypatch):
  errors = []
  monkeypatch.setattr(tp.cloudlog, "error", lambda m, *a, **k: errors.append(m))
  run_until_up(h, {"BackendState": "Running", "TailscaleIPs": ["100.64.0.5"]})
  h.down_rc = 1
  h.params.d["TailscaleEnabled"] = False
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
  h.params.d["TailscaleEnabled"] = False
  h.d.tick()
  execs = {os.path.basename([a for a in c if a not in ("sudo", "-n")][0]) for c in [*h.calls, *(p.cmd for p in h.procs)]}
  assert {"tailscale", "tailscaled"} <= execs <= {"tailscale", "tailscaled", "true"}, execs  # no iptables / nft / sysctl / nmcli / ip anywhere


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
def test_missing_key_is_needs_auth_key_and_never_calls_up(h):
  run_until_up(h, {"BackendState": "NeedsLogin"}, key=False)
  assert h.params.statuses[-1] == "needs auth key"
  assert h.cmds("up") == []


def test_empty_key_file_is_needs_auth_key(h):
  h.keyfile.write_text("  \n")
  run_until_up(h, {"BackendState": "NeedsLogin"}, key=False)
  assert h.params.statuses[-1] == "needs auth key" and h.cmds("up") == []


def test_needs_login_with_key_runs_up_with_file_key_and_pinned_flags(h):
  run_until_up(h, {"BackendState": "NeedsLogin"})
  (up,) = h.cmds("up")
  assert f"--auth-key=file:{h.keyfile}" in up
  for flag in ("--ssh=false", "--accept-dns=false", "--advertise-tags=tag:comma", "--hostname=comma-abc123"):
    assert flag in up
  assert "--shields-up" not in up                       # would block the inbound SSH this exists for
  assert not any("SECRET123" in " ".join(c) for c in h.calls), "key material must never be on a command line"


def test_up_failure_is_an_error_with_key_redacted(h):
  h.up_rc = 1
  run_until_up(h, {"BackendState": "NeedsLogin"})
  last = h.params.statuses[-1]
  assert last.startswith("error tailscale up failed rc=1") and "SECRET123" not in last and "<redacted>" in last


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
  assert tp.redact("bad tskey-auth-k123abc-XYZ end") == "bad tskey-<redacted> end"


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


# ---- A: toggle OFF (default / fresh install) -----------------------------------------------------------------------
def test_A_off_is_inert_for_two_hours(world, tmp_path):
  h = Harness(tmp_path, enabled=False, enrolled=False)
  ticks = simulate(h, 120)
  assert world.installs == [] and h.calls == [] and h.procs == [], "OFF must do no download, subprocess or spawn"
  assert h.params.statuses == ["off"]
  assert ticks <= 120 * 60 / tp.POLL_STEADY_S + 1       # param poll only, <= 1 per 30 s
  assert not any(tmp_path.iterdir()), "OFF must not write to disk"
  assert len([m for lvl, m in world.logs]) == 1          # the single 'off' publish


def test_A_manager_only_runs_it_while_the_toggle_is_on():
  from openpilot.system.manager.process_config import managed_processes, tailscale_on
  p = managed_processes["tailscale_pnw"]
  assert p.should_run is tailscale_on and p.restart_if_crash
  assert tailscale_on(False, FakeParams(enabled=False), None) is False
  assert tailscale_on(True, FakeParams(enabled=True), None) is True      # on, driving or parked: it is not tied to IsOnroad


def test_A_param_registered_default_off():
  """No UnknownKeyName / UI crash on a fresh install: both keys exist and the toggle defaults OFF."""
  from openpilot.common.params import Params
  params = Params()
  assert params.get_bool("TailscaleEnabled") is False
  assert params.get("TailscaleStatus") is None or isinstance(params.get("TailscaleStatus"), str)


# ---- B: ON, no usable auth key ----------------------------------------------------------------------------------------
@pytest.mark.parametrize("keyfile", ["missing", "empty", "whitespace", "unreadable"])
def test_B_no_key_means_needs_auth_key_and_nothing_else(world, tmp_path, keyfile):
  h = Harness(tmp_path, enrolled=False)
  if keyfile == "empty":
    h.keyfile.write_text("")
  elif keyfile == "whitespace":
    h.keyfile.write_text(" \n\t\n")
  elif keyfile == "unreadable":
    h.keyfile.mkdir()        # open() raises IsADirectoryError, an OSError like a permission failure
  simulate(h, 120)
  assert h.params.statuses == ["needs auth key"]
  assert world.installs == [], "no key -> no 36 MB download"
  assert h.procs == [] and h.calls == [], "no key -> tailscaled is not started, no subprocess at all"
  warnings = [m for lvl, m in world.logs if lvl in ("warning", "error") and "needs auth key" in m]
  assert len(warnings) == 1, "one log line per state change, not per poll"


def test_B_key_arrives_later_then_it_installs_once_and_connects(world, tmp_path):
  h = Harness(tmp_path, enrolled=False)
  simulate(h, 10)
  assert world.installs == []
  h.keyfile.write_text("tskey-auth-SECRET123")
  h.keyfile.chmod(0o600)
  h.status = {"BackendState": "NeedsLogin"}
  simulate(h, 5)
  assert world.installs == [1] and len(h.procs) == 1


def test_B_failed_download_backs_off_at_least_10_min_and_is_bounded(world, tmp_path):
  world.install_error = installer.InstallError("download failed: timed out")
  h = Harness(tmp_path, enrolled=False)
  h.keyfile.write_text("tskey-auth-SECRET123")
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
  h.keyfile.write_text("tskey-auth-SECRET123")
  simulate(h, 6 * 60)
  assert world.installs == [1] and h.procs == []


# ---- C: ON with a key but the world is broken ------------------------------------------------------------------------------
def test_C_no_internet_reports_a_reason_and_polls_boundedly(world, tmp_path):
  world.installed = True
  h = Harness(tmp_path)
  h.keyfile.write_text("tskey-auth-SECRET123")
  h.status = {"BackendState": "Starting", "Health": ["Tailscale can't reach the coordination server"]}
  ticks = simulate(h, 60)
  assert h.params.statuses[0] == "connecting"
  assert h.params.statuses[-1].startswith("error not connected for over 3 min") and "coordination server" in h.params.statuses[-1]
  assert ticks < 200 and len(h.procs) == 1 and world.installs == []     # no reinstall, no restart storm
  assert len([1 for lvl, m in world.logs if "not connected" in m]) == 1  # logged once, not per poll


def test_C_up_failing_backs_off_and_never_reinstalls(world, tmp_path):
  world.installed = True
  h = Harness(tmp_path)
  h.keyfile.write_text("tskey-auth-SECRET123")
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
  h.keyfile.write_text("tskey-auth-SECRET123")
  h.status = {"BackendState": "NeedsLogin"}
  h.up_rc = 1                      # control plane says: key expired/deleted/revoked
  h.d.tick()
  h.d.tick()
  assert h.params.statuses[-1].startswith("error tailscale up failed")


# ---- D: OFF at any time -------------------------------------------------------------------------------------------------
def test_D_off_while_connecting_stops_cleanly_and_keeps_state(world, tmp_path):
  world.installed = True
  h = Harness(tmp_path)
  h.keyfile.write_text("tskey-auth-SECRET123")
  h.status = {"BackendState": "Starting"}
  simulate(h, 1)
  h.params.d["TailscaleEnabled"] = False
  h.d.tick()
  assert h.cmds("down") and h.procs[0].terminated and not h.procs[0].killed
  assert not h.cmds("logout"), "state must be kept: never `tailscale logout`"
  assert h.params.statuses[-1] == "off"


# ---- E: installer never on the driving hot path -------------------------------------------------------------------------
def test_E_no_install_while_driving_then_installs_when_parked(world, tmp_path):
  h = Harness(tmp_path)
  h.keyfile.write_text("tskey-auth-SECRET123")
  h.params.d.update({"IsOnroad": True, "GearPark": False})
  simulate(h, 30)
  assert world.installs == [] and h.params.statuses == ["install deferred until parked"]
  h.params.d["GearPark"] = True      # parked + charging: IsOnroad is still 1, Rule 3
  h.status = {"BackendState": "NeedsLogin"}
  simulate(h, 2)
  assert world.installs == [1]


def test_E_offroad_installs(world, tmp_path):
  h = Harness(tmp_path)
  h.keyfile.write_text("tskey-auth-SECRET123")
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
  h.preexisting = True
  h.status = {"BackendState": "Running", "TailscaleIPs": ["100.64.0.9"]}
  h.d.tick()
  h.d.tick()
  assert h.procs == [] and h.params.statuses[-1] == "connected 100.64.0.9"
  h.params.d["TailscaleEnabled"] = False
  h.d.tick()
  assert h.cmds("down") and h.params.statuses[-1] == "off"
