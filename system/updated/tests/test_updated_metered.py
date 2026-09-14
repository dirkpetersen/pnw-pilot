"""updatemetered2pnw: code updates fetch over ANY link, metered included (driver directive 2026-07-11,
reaffirmed 2026-09-13). The metered gate belongs to drive-data uploads (loggerd/uploader.py) only.

These drive the REAL updated.main() loop for one update attempt -- real Params (NetworkMetered,
UpdaterLastFetchTime, DisableUpdates), the real WaitTimeHelper and its real SIGHUP/SIGUSR1 handlers --
with only the git/overlay/filesystem side effects mocked out. The decision under test lives inline in
that loop, so this is the narrowest seam that exercises the shipped code rather than a copy of it.
"""
import datetime
import signal

import pytest

from cereal import log
from openpilot.common.params import Params
import openpilot.system.updated.updated as updated

NetworkType = log.DeviceState.NetworkType
FETCH_EVENT = "updated: fetching update"


class _StopLoop(BaseException):
  """Ends main()'s infinite loop. BaseException, so the loop's own `except Exception` cannot swallow it."""


@pytest.fixture
def run_updater(mocker, tmp_path):
  saved_handlers = {s: signal.getsignal(s) for s in (signal.SIGHUP, signal.SIGUSR1)}

  def run(metered: bool, request: signal.Signals | None = None, disable_updates: bool = False,
          time_valid: bool = True, network_type: int = NetworkType.wifi):
    params = Params()
    params.put_bool("NetworkMetered", metered)
    params.put_bool("DisableUpdates", disable_updates)
    # A fetch 1 minute ago: stock's 3-day `timed_out` override is NOT in play, so the metered skip is.
    params.put("UpdaterLastFetchTime", datetime.datetime.now(datetime.UTC).replace(tzinfo=None) - datetime.timedelta(minutes=1))

    mocker.patch.object(updated, "LOCK_FILE", str(tmp_path / "updated.lock"))
    mocker.patch.object(updated.psutil, "Process")
    mocker.patch.object(updated, "init_overlay")
    mocker.patch.object(updated, "set_consistent_flag")
    mocker.patch.object(updated, "system_time_valid", return_value=time_valid)
    mocker.patch.object(updated.HARDWARE, "get_network_type", return_value=network_type)
    updater = mocker.patch.object(updated, "Updater").return_value
    cloudlog = mocker.patch.object(updated, "cloudlog")

    sleeps: list[float] = []

    def fake_sleep(helper, t):
      sleeps.append(t)
      if len(sleeps) == 1 and request is not None:
        # deliver the request through whatever handler WaitTimeHelper really installed for it
        signal.getsignal(request)(request, None)
      if len(sleeps) >= 2:
        raise _StopLoop

    mocker.patch.object(updated.WaitTimeHelper, "sleep", autospec=True, side_effect=fake_sleep)
    with pytest.raises(_StopLoop):
      updated.main()
    assert len(sleeps) == 2, sleeps  # the loop really made it to a second pass
    return updater, cloudlog

  yield run
  for s, h in saved_handlers.items():
    signal.signal(s, h)


def fetch_events(cloudlog):
  return [c for c in cloudlog.event.call_args_list if c.args and c.args[0] == FETCH_EVENT]


def test_metered_automatic_fetches(run_updater):
  # The measured 2026-09-13 defect: the truck on metered Starlink never fetched on its own.
  updater, cloudlog = run_updater(metered=True, network_type=NetworkType.wifi)
  updater.check_for_update.assert_called_once()
  updater.fetch_update.assert_called_once()
  events = fetch_events(cloudlog)
  assert len(events) == 1
  assert events[0].kwargs == {"metered": True, "network_type": "wifi", "user_requested": False}


def test_metered_cellular_automatic_fetches(run_updater):
  updater, cloudlog = run_updater(metered=True, network_type=NetworkType.cell4G)
  updater.fetch_update.assert_called_once()
  assert fetch_events(cloudlog)[0].kwargs == {"metered": True, "network_type": "cell4G", "user_requested": False}


def test_unmetered_automatic_fetches(run_updater):
  updater, cloudlog = run_updater(metered=False)
  updater.fetch_update.assert_called_once()
  assert fetch_events(cloudlog)[0].kwargs == {"metered": False, "network_type": "wifi", "user_requested": False}


@pytest.mark.parametrize("metered", [True, False])
def test_user_requested_fetches(run_updater, metered):
  updater, cloudlog = run_updater(metered=metered, request=signal.SIGHUP)
  updater.fetch_update.assert_called_once()
  assert fetch_events(cloudlog)[0].kwargs["user_requested"] is True


@pytest.mark.parametrize("metered", [True, False])
def test_check_only_request_never_fetches(run_updater, metered):
  # SIGUSR1 ("check for update") still only checks, on any link.
  updater, cloudlog = run_updater(metered=metered, request=signal.SIGUSR1)
  updater.check_for_update.assert_called_once()
  updater.fetch_update.assert_not_called()
  assert fetch_events(cloudlog) == []


@pytest.mark.parametrize("metered", [True, False])
def test_pause_updates_still_blocks(run_updater, metered):
  updater, _ = run_updater(metered=metered, disable_updates=True)
  updater.check_for_update.assert_not_called()
  updater.fetch_update.assert_not_called()


@pytest.mark.parametrize("metered", [True, False])
def test_invalid_system_time_still_blocks(run_updater, metered):
  updater, _ = run_updater(metered=metered, time_valid=False)
  updater.check_for_update.assert_not_called()
  updater.fetch_update.assert_not_called()
