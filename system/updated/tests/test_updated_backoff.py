"""updatebackoff2pnw: consecutive failed fetches back off exponentially (5, 10, 20, 40, 80 min, capped at the
normal 1.5 h interval) instead of retrying at a flat 5 min. A failed fetch wipes the staging overlay, so every
retry re-downloads the git pack, the changed LFS objects and any AGNOS image.

Like test_updated_metered.py, these drive the REAL updated.main() loop across many passes -- real Params, the
real WaitTimeHelper and the SIGHUP handler it really installs -- with only git/overlay/filesystem side effects
mocked. The expected waits are literal seconds on purpose, not expressions of the constants under test.
"""
import signal
import subprocess
import time

import pytest

import openpilot.system.updated.updated as updated

BACKOFF_EVENT = "updated: fetch retry backoff"
FETCH_EVENT = "updated: fetching update"

OK = "ok"                  # check and fetch succeed
FETCH_FAIL = "fetch_fail"  # check succeeds, the fetch raises (link died mid-download)
OFFLINE = "offline"        # `git ls-remote --heads` raises, nothing is fetched


class _StopLoop(BaseException):
  """Ends main()'s infinite loop. BaseException, so the loop's own `except Exception` cannot swallow it."""


@pytest.fixture
def run_updater(mocker, tmp_path):
  saved_handlers = {s: signal.getsignal(s) for s in (signal.SIGHUP, signal.SIGUSR1)}

  def run(passes: list[str], sighup_in_sleep: int | None = None, sighup_in_pass: int | None = None):
    """Run one update pass per entry of `passes`. Returns (timeline, waits, cloudlog).
    sighup_in_sleep=i delivers SIGHUP while the loop sleeps after pass i; sighup_in_pass=i delivers it
    while pass i's fetch is running. Either way the real WaitTimeHelper.sleep must then return at once."""
    mocker.patch.object(updated, "LOCK_FILE", str(tmp_path / "updated.lock"))
    mocker.patch.object(updated, "OVERLAY_INIT", tmp_path / ".overlay_init")
    mocker.patch.object(updated.psutil, "Process")
    mocker.patch.object(updated, "init_overlay")
    mocker.patch.object(updated, "set_consistent_flag")
    mocker.patch.object(updated, "system_time_valid", return_value=True)
    # A plain time.sleep could not be woken by SIGHUP; the loop must only ever block in WaitTimeHelper.sleep.
    mocker.patch.object(updated, "time", mocker.Mock(wraps=time, sleep=mocker.Mock(
      side_effect=AssertionError("updated blocked in time.sleep, which SIGHUP cannot wake"))))
    updater = mocker.patch.object(updated, "Updater").return_value
    cloudlog = mocker.patch.object(updated, "cloudlog")

    timeline: list[tuple] = []
    real_sleep = updated.WaitTimeHelper.sleep

    def current_pass() -> int:
      return sum(1 for e in timeline if e[0] == "sleep") - 1  # the first sleep is main()'s first_run wait

    def check_for_update():
      timeline.append(("check", current_pass()))
      if passes[current_pass()] == OFFLINE:
        raise subprocess.CalledProcessError(128, ["git", "ls-remote", "--heads"], output="Could not resolve host")

    def fetch_update():
      i = current_pass()
      timeline.append(("fetch", i))
      if sighup_in_pass == i:
        signal.getsignal(signal.SIGHUP)(signal.SIGHUP, None)
      if passes[i] == FETCH_FAIL:
        raise subprocess.CalledProcessError(128, ["git", "fetch", "origin", "3devpnw"], output="early EOF")

    updater.check_for_update.side_effect = check_for_update
    updater.fetch_update.side_effect = fetch_update

    def fake_sleep(helper, t):
      i = current_pass()
      timeline.append(("sleep", t))
      if sighup_in_sleep == i:
        signal.getsignal(signal.SIGHUP)(signal.SIGHUP, None)
      if i in (sighup_in_sleep, sighup_in_pass):
        # A SIGHUP is pending: the REAL sleep must return immediately whatever the backoff says.
        assert helper.ready_event.is_set(), f"SIGHUP pending but the {t} s sleep would not wake"
        start = time.monotonic()
        real_sleep(helper, t)
        assert time.monotonic() - start < 1.0
      if i + 1 >= len(passes):
        raise _StopLoop

    mocker.patch.object(updated.WaitTimeHelper, "sleep", autospec=True, side_effect=fake_sleep)
    with pytest.raises(_StopLoop):
      updated.main()

    assert timeline[0] == ("sleep", 60)  # first_run
    waits = [e[1] for e in timeline[1:] if e[0] == "sleep"]
    assert len(waits) == len(passes), timeline  # the loop really ran every scripted pass
    return timeline, waits, cloudlog

  yield run
  for s, h in saved_handlers.items():
    signal.signal(s, h)


def backoff_events(cloudlog):
  return [c.kwargs for c in cloudlog.event.call_args_list if c.args and c.args[0] == BACKOFF_EVENT]


def test_normal_path_unchanged(run_updater):
  timeline, waits, cloudlog = run_updater([OK] * 4)
  assert waits == [5400, 5400, 5400, 5400]
  assert [e for e in timeline if e[0] == "fetch"] == [("fetch", 0), ("fetch", 1), ("fetch", 2), ("fetch", 3)]
  assert backoff_events(cloudlog) == []


def test_repeated_fetch_failures_back_off_to_the_cap(run_updater):
  _, waits, _ = run_updater([FETCH_FAIL] * 8)
  # 5, 10, 20, 40, 80 min, then never longer than the normal 1.5 h interval
  assert waits == [300, 600, 1200, 2400, 4800, 5400, 5400, 5400]


def test_success_resets_the_backoff(run_updater):
  _, waits, cloudlog = run_updater([FETCH_FAIL] * 4 + [OK] + [FETCH_FAIL] * 2)
  assert waits == [300, 600, 1200, 2400, 5400, 300, 600]
  assert backoff_events(cloudlog) == [
    {"failed_fetches": 1, "retry_in_s": 300},
    {"failed_fetches": 2, "retry_in_s": 600},
    {"failed_fetches": 3, "retry_in_s": 1200},
    {"failed_fetches": 4, "retry_in_s": 2400},
    {"failed_fetches": 0, "retry_in_s": 5400},  # the reset is logged too
    {"failed_fetches": 1, "retry_in_s": 300},
    {"failed_fetches": 2, "retry_in_s": 600},
  ]


def test_offline_keeps_the_cheap_retry_and_the_backoff_level(run_updater):
  # Offline passes download nothing: they keep the 5 min retry, neither raise nor reset the level, and log
  # no backoff change. The next real fetch failure continues from the level it had reached.
  timeline, waits, cloudlog = run_updater([OFFLINE, OFFLINE, FETCH_FAIL, FETCH_FAIL, FETCH_FAIL, OFFLINE, OFFLINE, FETCH_FAIL])
  assert waits == [300, 300, 300, 600, 1200, 300, 300, 2400]
  assert [e[1] for e in timeline if e[0] == "fetch"] == [2, 3, 4, 7]
  assert [e["failed_fetches"] for e in backoff_events(cloudlog)] == [1, 2, 3, 4]


@pytest.mark.parametrize("where", ["sleep", "pass"])
def test_sighup_fetches_immediately_during_backoff(run_updater, where):
  # Deep in the backoff (the pass-3 failure means a 40 min wait), a SIGHUP -- sent while the loop sleeps, or
  # while the failing fetch is still running -- wakes the real sleep at once, and the very next thing the
  # loop does is check and fetch: no extra sleep, no skip.
  kwargs = {"sighup_in_sleep": 3} if where == "sleep" else {"sighup_in_pass": 3}
  timeline, waits, cloudlog = run_updater([FETCH_FAIL] * 4 + [OK], **kwargs)
  assert waits == [300, 600, 1200, 2400, 5400]
  i = timeline.index(("sleep", 2400))
  assert timeline[i + 1:i + 4] == [("check", 4), ("fetch", 4), ("sleep", 5400)], timeline
  fetches = [c.kwargs for c in cloudlog.event.call_args_list if c.args and c.args[0] == FETCH_EVENT]
  assert fetches[4]["user_requested"] is (where == "sleep")  # a SIGHUP during the attempt is consumed by it


def test_update_failed_alert_still_fires_at_stock_wall_time(mocker):
  # Stock raised "Unable to download updates" after 16 consecutive failures = 75 min at the flat 5 min retry.
  # Under the backoff, 16 failures take ~17.6 h of uptime, so the alert must key on the 5th failure instead,
  # which lands after 300 + 600 + 1200 + 2400 s of backoff (test_repeated_fetch_failures_back_off_to_the_cap)
  # = 75 min, the same wall time as stock. Exercises the REAL Updater.set_params alert logic.
  alert = mocker.patch.object(updated, "set_offroad_alert")
  mocker.patch.object(updated, "get_build_metadata", return_value=mocker.Mock(tested_channel=False))
  mocker.patch.object(updated.Updater, "target_branch", new_callable=mocker.PropertyMock, return_value="3devpnw")
  for prop in ("update_available", "update_ready"):
    mocker.patch.object(updated.Updater, prop, new_callable=mocker.PropertyMock, return_value=False)
  mocker.patch.object(updated, "parse_release_notes", return_value=b"")
  real = updated.Updater()
  real._has_internet = True

  def failed_alert_raised(count: int) -> bool:
    alert.reset_mock()
    real.set_params(False, count, "command failed: git fetch")
    return any(c.args[:2] == ("Offroad_UpdateFailed", True) for c in alert.call_args_list)

  assert not failed_alert_raised(4)
  assert failed_alert_raised(5)
  real._has_internet = False
  assert not failed_alert_raised(5)  # offline is the connectivity alerts' job, unchanged from stock
