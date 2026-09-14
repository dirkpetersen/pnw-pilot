"""keepstaged2pnw: a pass whose `git ls-remote` fails (offline, captive portal) keeps the staging overlay and the
finalized update waiting for the next reboot. Stock unlinked OVERLAY_INIT on every failure, so the next pass's
init_overlay() deleted STAGING_ROOT, finalized update included, and the device downloaded all of it again. Any other
failure, and every failure once the fetch has started, still invalidates as stock.

Unlike test_updated_backoff.py, the updater is NOT mocked here. The REAL main() loop, init_overlay(), check_for_update(),
fetch_update(), finalize_update(), set_consistent_flag() and Updater.set_params() run against a tmp staging tree. Only
updated.run() (every git/sudo subprocess) and the overlay mount state are faked, so a wipe is a real delete of the tmp
tree, .overlay_consistent is a real file and UpdateAvailable is the real param.
"""
import os
import shutil
import signal
import subprocess
from types import SimpleNamespace

import pytest

from openpilot.common.params import Params
import openpilot.system.updated.updated as updated

INSTALLED_SHA = "a" * 40
REMOTE_SHA = "b" * 40
BRANCH = "3devpnw"

OK = "ok"
OFFLINE = "offline"        # every `git ls-remote` fails, as with no link or a captive portal
FETCH_FAIL = "fetch_fail"  # the check succeeds, `git fetch` fails

KEPT = "updated: kept staged update after a check failure"
INVALIDATED = "updated: invalidated overlay after update failure"
FETCH_START = "attempting git fetch inside staging overlay"  # fetch_update()'s first log line
FINALIZE_DONE = "done finalizing overlay"                    # logged right after set_consistent_flag(True)
NOOP_CMDS = {("find",), ("sudo", "chmod"), ("git", "config"), ("git", "diff"), ("git", "ls-remote"), ("git", "checkout"),
             ("git", "branch"), ("git", "reset"), ("git", "clean"), ("git", "submodule"), ("git", "gc"), ("git", "lfs")}


class _StopLoop(BaseException):
  """Ends main()'s infinite loop. BaseException, so the loop's own `except Exception` cannot swallow it."""


class _UnexpectedCommand(BaseException):
  """A subprocess the fake does not model. BaseException, so it fails the test instead of looking like a failed pass."""


class _InvariantViolated(BaseException):
  """BaseException for the same reason: an AssertionError raised inside a pass would be caught by main() as a failure."""


@pytest.fixture
def run_updater(mocker, tmp_path):
  saved_handlers = {s: signal.getsignal(s) for s in (signal.SIGHUP, signal.SIGUSR1)}
  n_runs = [0]

  def run(passes: list):
    """One update pass per entry: OK, OFFLINE, FETCH_FAIL, or (k, exc) to raise exc from that pass's k-th subprocess."""
    mocker.stopall()  # a test may run the loop several times; each run patches afresh
    n_runs[0] += 1
    root = tmp_path / f"run{n_runs[0]}"
    basedir, staging = root / "openpilot", root / "safe_staging"
    (basedir / ".git").mkdir(parents=True)
    for name, path in {"BASEDIR": basedir, "STAGING_ROOT": staging, "OVERLAY_UPPER": staging / "upper",
                       "OVERLAY_METADATA": staging / "metadata", "OVERLAY_MERGED": staging / "merged",
                       "FINALIZED": staging / "finalized", "LOCK_FILE": root / "updated.lock"}.items():
      mocker.patch.object(updated, name, str(path))
    mocker.patch.object(updated, "OVERLAY_INIT", basedir / ".overlay_init")
    mocker.patch.object(updated.psutil, "Process")
    mocker.patch.object(updated.os, "sync")  # set_consistent_flag() syncs every filesystem; seconds per call here
    mocker.patch.object(updated, "system_time_valid", return_value=True)
    mocker.patch.object(updated, "get_build_metadata", return_value=mocker.Mock(tested_channel=False))
    mocker.patch.object(updated, "set_offroad_alert")
    state = {"mounted": False, "calls": 0}
    mocker.patch.object(updated.os.path, "ismount", side_effect=lambda p: str(p) == updated.OVERLAY_MERGED and state["mounted"])

    timeline: list[tuple] = []
    cloudlog = mocker.patch.object(updated, "cloudlog")
    cloudlog.info.side_effect = lambda msg, *a, **k: timeline.append(("log", msg))
    cloudlog.event.side_effect = lambda name, *a, **k: timeline.append(("event", name, k))
    params = Params()
    observed: list[dict] = []

    def current_pass() -> int:
      return sum(1 for e in timeline if e[0] == "sleep") - 1  # -1 = startup, before main()'s first_run sleep

    def consistent() -> bool:
      return os.path.isfile(os.path.join(updated.FINALIZED, ".overlay_consistent"))

    def assert_invariant():
      # .overlay_consistent may only exist while no fetch has started since the last COMPLETED finalize. Checked at
      # every subprocess boundary, i.e. wherever a reboot could land while the updater works.
      markers = [e[1] for e in timeline if e[0] == "log" and e[1] in (FETCH_START, FINALIZE_DONE)]
      if consistent() and markers[-1:] != [FINALIZE_DONE]:
        raise _InvariantViolated(timeline)

    def fake_run(cmd, cwd=None):
      assert_invariant()
      spec = passes[current_pass()] if current_pass() >= 0 else OK
      timeline.append(("run", cmd))
      state["calls"] += 1
      if isinstance(spec, tuple) and state["calls"] - 1 == spec[0]:
        raise spec[1]
      if (spec == OFFLINE and cmd[:2] == ["git", "ls-remote"]) or (spec == FETCH_FAIL and cmd[:2] == ["git", "fetch"]):
        raise subprocess.CalledProcessError(128, cmd, output="Could not resolve host: github.com")
      if cmd[:3] == ["sudo", "rm", "-rf"]:
        shutil.rmtree(cmd[3], ignore_errors=True)
      elif cmd[:2] in (["sudo", "mount"], ["sudo", "umount"]):
        state["mounted"] = cmd[1] == "mount"
      elif cmd == ["git", "ls-remote", "--heads"]:
        return f"{REMOTE_SHA}\trefs/heads/{BRANCH}\n"
      elif cmd == ["git", "rev-parse", "--abbrev-ref", "HEAD"]:
        return BRANCH + "\n"
      elif cmd == ["git", "rev-parse", "HEAD"]:
        return (INSTALLED_SHA if cwd == updated.BASEDIR else REMOTE_SHA) + "\n"
      elif cmd[:2] == ["git", "fetch"]:
        pack = os.path.join(updated.OVERLAY_MERGED, "pack")
        if not os.path.exists(pack):
          timeline.append(("download",))
          open(pack, "w").close()
      elif tuple(cmd[:1]) not in NOOP_CMDS and tuple(cmd[:2]) not in NOOP_CMDS:
        raise _UnexpectedCommand(cmd)
      return ""

    def fake_sleep(helper, t):
      assert_invariant()
      p = current_pass()
      timeline.append(("sleep", t))
      state["calls"] = 0
      if p >= 0:
        observed.append({"update_available": params.get_bool("UpdateAvailable"), "consistent": consistent(),
                         "exception": params.get("LastUpdateException"), "overlay_init": updated.OVERLAY_INIT.is_file()})
      if p + 1 >= len(passes):
        raise _StopLoop

    mocker.patch.object(updated, "run", side_effect=fake_run)
    mocker.patch.object(updated.WaitTimeHelper, "sleep", autospec=True, side_effect=fake_sleep)
    with pytest.raises(_StopLoop):
      updated.main()

    assert len(observed) == len(passes), timeline  # the loop really ran every scripted pass
    return SimpleNamespace(timeline=timeline, observed=observed,
                           wipes=sum(1 for e in timeline if e[0] == "run" and e[1][:3] == ["sudo", "rm", "-rf"]),
                           downloads=sum(1 for e in timeline if e[0] == "download"))

  yield run
  for s, h in saved_handlers.items():
    signal.signal(s, h)


def events(r, name):
  return [e[2] for e in r.timeline if e[0] == "event" and e[1] == name]


def test_offline_check_keeps_the_staged_update(run_updater):
  r = run_updater([OK, OFFLINE, OFFLINE])
  # Stock deleted the update finalized in pass 0 at the start of pass 2 (the second wipe).
  assert [o["update_available"] for o in r.observed] == [True, True, True]
  assert [o["consistent"] for o in r.observed] == [True, True, True]
  assert r.observed[1]["exception"] is not None  # the offline pass still counts as failed
  assert r.wipes == 1  # only the startup init_overlay built the staging area
  assert events(r, KEPT) == [{"cmd": ["git", "ls-remote", "--heads"]}] * 2
  assert events(r, INVALIDATED) == []


def test_fetch_failure_still_invalidates(run_updater):
  r = run_updater([OK, FETCH_FAIL, OFFLINE])
  assert [o["update_available"] for o in r.observed] == [True, False, False]
  assert [o["consistent"] for o in r.observed] == [True, False, False]
  assert r.wipes == 2  # the pass after the failed fetch rebuilt the staging area
  assert events(r, INVALIDATED) == [{"cmd": ["git", "fetch", "origin", BRANCH], "fetch_started": True}]


def test_offline_then_online_reuses_the_download(run_updater):
  r = run_updater([OK, OFFLINE, OK])
  assert [o["update_available"] for o in r.observed] == [True, True, True]
  assert (r.wipes, r.downloads) == (1, 1)  # stock: (2, 2), the whole update downloaded twice


def test_only_a_failed_ls_remote_keeps_the_overlay(run_updater):
  # Fail each subprocess of a normal pass in turn, in the pass after an update was finalized. OVERLAY_INIT decides
  # whether the next pass wipes the staging area (the tests above), so it is the outcome. The consistent-flag
  # invariant is asserted at every subprocess boundary of every one of these runs (in the harness).
  probe = run_updater([OK, OK])
  pass1 = probe.timeline[[i for i, e in enumerate(probe.timeline) if e[0] == "sleep"][1] + 1:]
  pass1 = pass1[:pass1.index(("log", "finalize success!"))]  # what follows is outside the pass's try
  fetch_start = pass1.index(("log", FETCH_START))
  runs = [(i > fetch_start, e[1]) for i, e in enumerate(pass1) if e[0] == "run"]

  outcomes = []
  for k, (in_fetch, cmd) in enumerate(runs):
    o = run_updater([OK, (k, subprocess.CalledProcessError(128, cmd, output="boom"))]).observed[1]
    outcome = "not failed" if o["exception"] is None else "kept" if o["overlay_init"] else "invalidated"
    outcomes.append((in_fetch, cmd[:2] if cmd[0] == "git" else cmd[:1], outcome))
    if in_fetch and outcome == "invalidated":
      assert not o["consistent"]

  assert [(f, c) for f, c, out in outcomes if out == "kept"] == [(False, ["git", "ls-remote"])]
  assert [c for f, c, out in outcomes if f and out != "invalidated"] == [["git", "gc"], ["git", "lfs"]]  # cleanup, as stock
  pre_fetch_invalidated = {tuple(c) for f, c, out in outcomes if not f and out == "invalidated"}
  assert pre_fetch_invalidated == {("find",), ("git", "rev-parse"), ("git", "config")}  # a broken overlay: rebuild, as stock
  assert (False, ["git", "ls-remote"], "not failed") in outcomes  # `ls-remote origin HEAD`, caught to set has_internet


def test_non_command_error_in_check_still_invalidates(run_updater):
  # The keep is for a git command that ran and failed, not for any error at that step (git missing, bad output).
  probe = run_updater([OK, OK])
  pass1 = probe.timeline[[i for i, e in enumerate(probe.timeline) if e[0] == "sleep"][1] + 1:]
  k = [e[1] for e in pass1 if e[0] == "run"].index(["git", "ls-remote", "--heads"])
  r = run_updater([OK, (k, OSError("git: not found")), OFFLINE])
  assert [o["update_available"] for o in r.observed] == [True, True, False]
  assert r.wipes == 2
  assert events(r, INVALIDATED)[0] == {"cmd": None, "fetch_started": False}
