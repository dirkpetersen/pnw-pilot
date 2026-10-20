"""Review fixes (Gemini 4 Argon, 2026-10-20): no crash on a vanished file or an unreadable param, no heavy disk I/O while driving,
a bounded swaglog scan that never reports a silent 'not loaded', and a selfdrived uptime probe that cannot throw."""
import os
import pathlib
import subprocess

import pytest

from openpilot.system.curvedb import curvedb_updated as cu
from openpilot.system.curvedb import swap

LOADED = '{"event$s": "curvedb_v2_loaded", "sha256$s": "%s"}\n'


def text(env):
  return env.status().rsplit(" @", 1)[0]


def log(env):
  return pathlib.Path(cu.LOG_FILE).read_text()


def current(env):
  env.route({**env.ptr, "sha256": env.old_sha})
  env.record(env.old_sha)


# ---- (1) _current / install: OSError -> state error, no crash ------------------------------------------------------------
def test_live_table_vanishing_before_the_date_read_is_an_error_not_a_crash(env, monkeypatch):
  current(env)
  real = os.path.getmtime
  monkeypatch.setattr(cu.os.path, "getmtime", lambda p: (_ for _ in ()).throw(FileNotFoundError(p)) if p.endswith(swap.ROWS) else real(p))
  monkeypatch.setattr(swap, "verify", lambda sha: ("pending", ""))
  env.u.cycle()
  assert text(env) == "error: live table unreadable"


def test_record_write_failure_while_current_is_an_error_not_a_crash(env, monkeypatch):
  current(env)
  env.record("z" * 64)
  monkeypatch.setattr(env.u, "write_record", lambda *a, **k: (_ for _ in ()).throw(OSError("read-only file system")))
  env.u.cycle()
  assert text(env) == "error: could not update the install record"


def test_record_write_failure_after_an_install_is_loud(env, monkeypatch):
  env.record(env.old_sha)
  monkeypatch.setattr(env.u, "write_record", lambda *a, **k: (_ for _ in ()).throw(OSError("disk full")))
  env.u.cycle()
  assert env.live_sha() == env.new_sha and text(env) == "error: installed but the install record could not be written"


# ---- (2) verify: never while driving; bounded; unknown is said ----------------------------------------------------------------
def test_verify_never_runs_while_driving(env, monkeypatch):
  current(env)
  env.state = ("0", "1", "900")
  monkeypatch.setattr(swap, "verify", lambda sha: pytest.fail("verify scanned the disk while driving"))
  env.u.cycle()
  assert text(env).startswith("installed ") and "not checked, driving" in log(env)


def test_verify_runs_in_park_and_offroad(env, monkeypatch):
  current(env)
  calls = []
  monkeypatch.setattr(swap, "verify", lambda sha: calls.append(1) or ("pending", ""))
  for st in (("1", "1", "900"), ("0", "0", "")):
    env.state = st
    env.u.cycle()
  assert len(calls) == 2


def _logs(env, n, sha=None, age=0, size=0):
  since = os.path.getmtime(f"{swap.LIVE}/{swap.ROWS}")
  for i in range(n):
    f = pathlib.Path(swap.SWAGLOG, f"swaglog.{i:05d}")
    f.write_text((LOADED % sha if sha else "x\n") + "y" * size)
    os.utime(f, (since + 10 + i - age, since + 10 + i - age))


def test_verify_ignores_logs_older_than_the_install(env):
  _logs(env, 3, sha=env.old_sha, age=1000)
  assert swap.verify(env.old_sha) == ("pending", "")


def test_verify_stops_at_the_file_cap_and_says_unknown(env, monkeypatch):
  _logs(env, 60)                                       # 60 newer logs, none with a load line
  pathlib.Path(swap.SWAGLOG, "swaglog.00000").write_text(LOADED % env.old_sha)   # the oldest has one, past the 50-file cap
  opened = []
  real = open
  monkeypatch.setattr("builtins.open", lambda p, *a, **k: (opened.append(str(p)), real(p, *a, **k))[1])
  v, why = swap.verify(env.old_sha)
  assert v == "unknown" and "50 files" in why
  assert len([p for p in opened if "swaglog." in p]) <= swap.VERIFY_MAX_FILES


def test_verify_newest_first_finds_the_latest_load(env):
  _logs(env, 5)
  since = os.path.getmtime(f"{swap.LIVE}/{swap.ROWS}")
  for name, sha, t in (("swaglog.00004", env.old_sha, 100), ("swaglog.00001", env.new_sha, 50)):   # newest first; the older file differs
    f = pathlib.Path(swap.SWAGLOG, name)
    f.write_text(LOADED % sha)
    os.utime(f, (since + t, since + t))
  assert swap.verify(env.old_sha) == ("loaded", env.old_sha[:8])


def test_verify_byte_cap_says_unknown(env, monkeypatch):
  monkeypatch.setattr(swap, "VERIFY_MAX_BYTES", 1000)
  _logs(env, 3, size=600)
  assert swap.verify(env.old_sha)[0] == "unknown"


def test_verify_missing_or_unreadable_swaglog_is_unknown_not_pending(env):
  os.rmdir(swap.SWAGLOG)
  v, why = swap.verify(env.old_sha)
  assert v == "unknown" and "unreadable" in why


def test_unknown_verify_is_logged_and_the_cycle_continues(env, monkeypatch):
  current(env)
  os.rmdir(swap.SWAGLOG)
  env.u.cycle()
  assert text(env).startswith("installed ") and " verify unknown " in log(env)


# ---- (3) selfdrived uptime probe ---------------------------------------------------------------------------------------------
def test_selfd_uptime_cannot_throw_and_unknown_makes_decide_refuse(monkeypatch):
  monkeypatch.setattr(swap, "_selfd_pid", lambda: "123")

  def run(rc, out, exc=None):
    def f(*a, **k):
      if exc:
        raise exc
      return subprocess.CompletedProcess(a, rc, out, "")
    monkeypatch.setattr(swap.subprocess, "run", f)
    return swap.selfd_uptime()
  assert run(0, "500\n") == "500"
  assert run(0, "", subprocess.TimeoutExpired("ps", 10)) == "?"
  assert run(1, "ps: error\n") == "?"
  assert run(0, "garbage") == "?"
  assert run(0, "", OSError("no ps")) == "?"
  ok, why = swap.decide("1", "1", "?")
  assert not ok and "unknown" in why
  monkeypatch.setattr(swap, "_selfd_pid", lambda: None)
  assert swap.selfd_uptime() == ""


# ---- (4) unreadable params never crash the cycle ------------------------------------------------------------------------------
class Flaky:
  def __init__(self, inner, bad):
    self.inner, self.bad = inner, set(bad)

  def get_bool(self, k):
    if k in self.bad:
      raise RuntimeError(f"UnknownKeyName {k}")
    return self.inner.get_bool(k)

  def get(self, k):
    if k in self.bad:
      raise RuntimeError(f"UnknownKeyName {k}")
    return self.inner.get(k)

  def put(self, k, v):
    if k in self.bad:
      raise RuntimeError(f"UnknownKeyName {k}")
    self.inner.put(k, v)


def test_unreadable_toggle_is_treated_as_disabled(env):
  inner = env.params
  env.u.params = Flaky(inner, ["DisableCurveDbAutoInstall"])
  env.u.cycle()
  env.u.cycle()
  assert env.http.calls == [] and st_text(inner) == "disabled"
  assert log(env).count("DisableCurveDbAutoInstall: RuntimeError") == 1          # once, not per cycle


def st_text(p):
  return p.d["CurveDbAutoInstallStatus"].rsplit(" @", 1)[0]


def test_unreadable_priority_network_means_metered_stays_metered(env):
  env.net = (1, True)                                  # wifi, NM says metered
  env.params.d["OnPriorityNetwork"] = True             # would relax the gate if it could be read
  env.u.params = Flaky(env.params, ["OnPriorityNetwork"])
  env.u.cycle()
  assert env.http.calls == [] and st_text(env.params) == "waiting for unmetered network"


def test_unreadable_dongle_and_status_param_do_not_crash(env):
  env.u.params = Flaky(env.params, ["DongleId", "CurveDbAutoInstallStatus"])
  env.u.cycle()                                        # no dongle -> error status; the failing put is logged, not raised
  assert env.http.calls == [] and "CurveDbAutoInstallStatus: RuntimeError" in log(env)


# ---- re-review: recover is gated; I/O errors in the install path end the cycle, never crash it ---------------------------------
def test_missing_live_table_is_not_recovered_while_driving(env):
  env.record(env.old_sha)
  env.route({**env.ptr, "sha256": "d" * 64})
  swap.stage(env.new_sha, env.new_rows, env.new_man)
  swap.swap(env.new_sha)                              # leaves an -auto backup to restore from
  import shutil
  shutil.rmtree(env.live)
  env.state = ("0", "1", "900")                       # driving
  env.u.cycle()
  assert text(env).startswith("waiting: live table missing, driving") and not os.path.exists(env.live)
  assert [c[2]["json"]["state"] for c in env.http.calls if c[0] == "POST"] == ["refused"]
  env.state = ("1", "1", "900")                       # parked: the next cycle restores it (and holds)
  env.u.cycle()
  assert text(env).startswith("held: live table was missing") and env.live_sha() == env.old_sha


def test_swap_recover_itself_refuses_while_driving(env):
  import shutil
  shutil.rmtree(env.live)
  env.state = ("0", "1", "900")
  with pytest.raises(swap.Refused, match="driving"):
    swap.recover()


@pytest.mark.parametrize("where,exc", [("live_sha", PermissionError("EIO")), ("free_mb", OSError("statvfs")),
                                       ("stage", OSError("disk full")), ("swap", subprocess.CalledProcessError(1, "cp")),
                                       ("swap", PermissionError("rename"))])
def test_io_errors_end_the_cycle_with_an_error_and_a_report(env, monkeypatch, where, exc):
  env.record(env.old_sha)

  def boom(*a, **k):
    raise exc
  monkeypatch.setattr(swap, where, boom)
  assert env.u.cycle() > 0                            # returned, did not raise
  assert text(env).startswith("error: ")
  assert env.live_sha() == env.old_sha if where != "live_sha" else True


def test_report_resent_when_loaded_flips(env):
  env.route({**env.ptr, "sha256": env.old_sha})
  env.record(env.old_sha)
  env.u.cycle()
  n = env.http.n("POST")
  pathlib.Path(swap.SWAGLOG, "swaglog.1").write_text(LOADED % env.old_sha)
  env.u.cycle()
  assert env.http.n("POST") == n + 1 and [c[2]["json"]["loaded"] for c in env.http.calls if c[0] == "POST"][-1] is True
