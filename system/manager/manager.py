#!/usr/bin/env python3
import datetime
import os
import signal
import subprocess
import sys
import threading
import time
import traceback

from cereal import log
import cereal.messaging as messaging
import openpilot.system.sentry as sentry
from openpilot.common.utils import atomic_write
from openpilot.common.params import Params, ParamKeyFlag
from openpilot.common.text_window import TextWindow
from openpilot.system.hardware import HARDWARE
from openpilot.system.manager.helpers import unblock_stdout, write_onroad_params, save_bootlog
from openpilot.system.manager.process import ensure_running
from openpilot.system.manager.process_config import managed_processes
from openpilot.system.athena.registration import register, UNREGISTERED_DONGLE_ID
from openpilot.common.swaglog import cloudlog, add_file_handler
from openpilot.system.version import get_build_metadata
from openpilot.system.hardware.hw import Paths


def manager_init() -> None:
  save_bootlog()

  build_metadata = get_build_metadata()

  params = Params()
  params.clear_all(ParamKeyFlag.CLEAR_ON_MANAGER_START)
  params.clear_all(ParamKeyFlag.CLEAR_ON_ONROAD_TRANSITION)
  params.clear_all(ParamKeyFlag.CLEAR_ON_OFFROAD_TRANSITION)
  params.clear_all(ParamKeyFlag.CLEAR_ON_IGNITION_ON)
  if build_metadata.release_channel:
    params.clear_all(ParamKeyFlag.DEVELOPMENT_ONLY)

  if params.get_bool("RecordFrontLock"):
    params.put_bool("RecordFront", True)

  # toggles-invert2pnw: one-time migration restructuring 4 toggles to opt-out semantics, so that
  # "everything OFF" is the good/default behavior on a fresh install (same idiom as the existing
  # DisableLaneCentering toggle) -- NudgelessLaneChange->NudgeForLaneChange,
  # FordAngleLateral->NoFordAngleSteering, LocationServicesEnabled->DisableLocationServices,
  # ShowSpeedLimit->NoSpeedLimitDisplay.
  #
  # MUST run BEFORE the generic default-seeding loop directly below (which treats
  # `params.get(k) is None` as "never persisted, seed the registered default"). params.get() -- not
  # get_bool(), which would coerce a never-set key to some default -- is what lets this code tell a
  # genuinely fresh install (old key file never written by anyone) apart from an existing device that
  # already has a real, driver-relevant value for the old key. That distinction is the whole point:
  # existing devices must keep their exact current effective behavior; only fresh installs should pick
  # up the new opt-out defaults (all four new keys default OFF, registered in params_keys.h).
  if params.get("TogglesInvertedMigrated") is None:
    _invert_map = {
      "NudgelessLaneChange": "NudgeForLaneChange",
      "FordAngleLateral": "NoFordAngleSteering",
      "LocationServicesEnabled": "DisableLocationServices",
      "ShowSpeedLimit": "NoSpeedLimitDisplay",
    }
    for _old_key, _new_key in _invert_map.items():
      _old_val = params.get(_old_key)  # BOOL type -> already a Python bool, or None if never persisted
      if _old_val is None:
        continue  # fresh install: old key was never set -- leave _new_key alone, it falls through
                  # to the generic seeding loop below and gets its OWN registered (new-good) default
      params.put_bool(_new_key, not _old_val)  # opt-out toggle ON == old feature-enabled flag OFF
    # FordAngleLateral's real behavioral gate lives in the opendbc submodule (a separate repo,
    # pnw-opendbc master-pnw, opendbc_repo/opendbc/car/pnw_vehicle.py) which reads the LEGACY
    # FordAngleLateral key directly and cannot be edited from this branch -- see toggles.py's
    # _toggle_callback for the write-through mirror that keeps it in sync going forward. For a
    # genuinely fresh install (no old key -> `continue`d above, so FordAngleLateral is still unset
    # here), also explicitly seed the legacy mirror key to match the new opt-out default (angle
    # steering ON) so the submodule -- unaware of NoFordAngleSteering -- behaves correctly without
    # needing a companion opendbc change first.
    if params.get("FordAngleLateral") is None:
      params.put_bool("FordAngleLateral", True)
    params.put_bool("TogglesInvertedMigrated", True)

  # toggles-invert2pnw / shared-device fix (2026-08-10, Fable finding): unconditional every-boot
  # re-sync of the legacy FordAngleLateral mirror from NoFordAngleSteering, NOT one-shot like the
  # migration above. NoFordAngleSteering is the single source of truth; FordAngleLateral only exists
  # because the opendbc submodule (pnw-opendbc master-pnw, opendbc_repo/opendbc/car/pnw_vehicle.py)
  # still reads the old key directly. This is the ONE physical comma device, swapped between the
  # Tesla and the F-150 Lightning -- toggles.py's _update_toggles grey-out clamp for the Lightning-only
  # NoFordAngleSteering toggle must never PERSIST a value while running on the (non-capable) Tesla, or
  # a Tesla stint would permanently flip the Lightning's default to "angle steering OFF" with no code
  # path to ever flip it back. Re-deriving the mirror here, unconditionally, on every boot self-heals
  # any such stray write (or a manual SSH edit) the moment the device is rebooted, regardless of which
  # car it was last plugged into. Delete this once pnw-opendbc master-pnw reads NoFordAngleSteering
  # directly (see the matching write-through mirror in toggles.py's _toggle_callback).
  params.put_bool("FordAngleLateral", not params.get_bool("NoFordAngleSteering"))

  # set unset params to their default value
  for k in params.all_keys():
    default_value = params.get_default_value(k)
    if default_value is not None and params.get(k) is None:
      params.put(k, default_value)

  # oplongpersist2pnw (supersedes oplongfix2pnw, docs/pnw/op-long-features.md §6): op-long now
  # PERSISTS across a same-car reboot -- this used to force AlphaLongitudinalEnabled=False on every
  # manager startup (every genuine cold boot); that line is REMOVED. The driver's op-long choice
  # survives a same-car reboot now, instead of being a per-session opt-in re-made every drive. Fix 5
  # (Fable INFO, review pass 2): this is ONE global param, not a per-car preference -- it is reset on
  # ANY real car swap (see below), so e.g. Lightning-ON -> a Tesla stint -> back to the Lightning
  # still wipes the ON (the swap back to the Lightning is itself a detected change). Reset happens on
  # exactly one event instead: a real CAR CHANGE (the physical device moved to a different,
  # non-native-op-long car), handled by card.py's _maybe_reset_calibration_on_car_change() (the
  # calswap2pnw hook) -- it clears AlphaLongitudinalEnabled (and requests an OnroadCycleRequested
  # reload so the freshly-swapped car comes up on stock ACC the SAME session, not just next boot) the
  # moment car_swapped_for_oplong() detects the swap (a broader test than the calibration wipe's
  # car_changed_for_recal() -- see card.py's Fix 1 comments). This is unconditional / capability-clean
  # (no car/fingerprint branch here in manager.py): the Tesla is unaffected either way, because its
  # op-long is native (openpilotLongitudinalControl=True regardless of this param; see
  # opendbc_repo/opendbc/car/tesla/interface.py::_get_params_sx, and pnw_vehicle.py's op_long_native
  # capability) -- card.py's car-change hook explicitly skips native cars, and this param never
  # gated the Tesla's op-long in the first place.

  # mapd2pnw: download-at-launch of the (un-vendored) mapd binary. Best-effort and
  # in the background so a slow/absent network never blocks manager startup or
  # driving; idempotent once the pinned binary from mapd_release.json is installed.
  # The mapd process itself is added with the full official integration; when it is,
  # its should_run must gate on the binary existing (os.path.exists) so manager never
  # tries to exec ./mapd before this background download finishes.
  #
  # RETRY-UNTIL-INSTALLED: every OTA update wipes this binary (it isn't in git, so the
  # overlay swap drops it), and right after a reboot/update the network is often not up
  # yet (e.g. a hotspot still re-associating). ensure_mapd()'s few quick retries can then
  # ALL fail, leaving mapd absent — and silently no map data — for the entire session.
  # So keep retrying in the background with backoff until the binary is actually present;
  # the thread exits the moment it's installed. Never blocks startup or driving.
  #
  # NETWORK-AGNOSTIC ON PURPOSE: the binary fetch (installer.ensure_mapd -> plain urlopen
  # of the pinned release URL) is NOT gated on Wi-Fi, priority networks, or metered state —
  # it downloads over whatever connection has internet (LTE, hotspot, any Wi-Fi). Only the
  # ~20 MB map TILE set is Wi-Fi-gated (mapd_configd); the binary must always come back so
  # mapd can run, regardless of which network the device happens to be on.
  def _install_mapd():
    from openpilot.system.mapd.installer import is_installed
    backoff = 10
    while True:
      try:
        # Run the installer in a SEPARATE PROCESS, not this thread. ensure_mapd() holds the
        # download temp file open for writing for the whole fetch, and manager forks its Python
        # children (multiprocessing) CONCURRENTLY with this background install. A forked child
        # inherits that write-fd (fork copies fds; O_CLOEXEC only fires on exec, not fork), and
        # after os.replace(tmp -> mapd) the child keeps the *binary* open for writing -> permanent
        # ETXTBSY -> manager can never exec mapd (exit 126) on a fresh-install boot (the delete-
        # everything-and-reboot recovery). A subprocess keeps that fd out of manager's fd table.
        rc = subprocess.run([sys.executable, "-m", "openpilot.system.mapd.installer"],
                            check=False, timeout=600).returncode
        if rc == 0 or is_installed():
          cloudlog.warning("mapd installer: binary present")
          return
        cloudlog.warning(f"mapd installer: exited {rc}; retrying in {backoff}s")
      except Exception as e:
        cloudlog.warning(f"mapd installer: {e}; retrying in {backoff}s")
      time.sleep(backoff)
      backoff = min(backoff * 2, 300)   # cap at 5 min; network will come up eventually
  threading.Thread(target=_install_mapd, name="mapd_installer", daemon=True).start()

  # Create folders needed for msgq
  try:
    os.mkdir(Paths.shm_path())
  except FileExistsError:
    pass
  except PermissionError:
    print(f"WARNING: failed to make {Paths.shm_path()}")

  # set params
  serial = HARDWARE.get_serial()
  params.put("Version", build_metadata.openpilot.version)
  params.put("GitCommit", build_metadata.openpilot.git_commit)
  params.put("GitCommitDate", build_metadata.openpilot.git_commit_date)
  params.put("GitBranch", build_metadata.channel)
  params.put("GitRemote", build_metadata.openpilot.git_origin)
  params.put_bool("IsTestedBranch", build_metadata.tested_channel)
  params.put_bool("IsReleaseBranch", build_metadata.release_channel)
  params.put("HardwareSerial", serial)

  # set dongle id
  reg_res = register(show_spinner=True)
  if reg_res:
    dongle_id = reg_res
  else:
    raise Exception(f"Registration failed for device {serial}")
  os.environ['DONGLE_ID'] = dongle_id  # Needed for swaglog
  os.environ['GIT_ORIGIN'] = build_metadata.openpilot.git_normalized_origin # Needed for swaglog
  os.environ['GIT_BRANCH'] = build_metadata.channel # Needed for swaglog
  os.environ['GIT_COMMIT'] = build_metadata.openpilot.git_commit # Needed for swaglog

  if not build_metadata.openpilot.is_dirty:
    os.environ['CLEAN'] = '1'

  # init logging
  sentry.init(sentry.SentryProject.SELFDRIVE)
  cloudlog.bind_global(dongle_id=dongle_id,
                       version=build_metadata.openpilot.version,
                       origin=build_metadata.openpilot.git_normalized_origin,
                       branch=build_metadata.channel,
                       commit=build_metadata.openpilot.git_commit,
                       dirty=build_metadata.openpilot.is_dirty,
                       device=HARDWARE.get_device_type())

  # preimport all processes
  for p in managed_processes.values():
    p.prepare()


def manager_cleanup() -> None:
  # send signals to kill all procs
  for p in managed_processes.values():
    p.stop(block=False)

  # ensure all are killed
  for p in managed_processes.values():
    p.stop(block=True)

  cloudlog.info("everything is dead")


def manager_thread() -> None:
  cloudlog.bind(daemon="manager")
  cloudlog.info("manager start")
  cloudlog.info({"environ": os.environ})

  params = Params()

  ignore: list[str] = []
  if params.get("DongleId") in (None, UNREGISTERED_DONGLE_ID):
    ignore += ["manage_athenad", "uploader"]
  if os.getenv("NOBOARD") is not None:
    ignore.append("pandad")
  ignore += [x for x in os.getenv("BLOCK", "").split(",") if len(x) > 0]

  sm = messaging.SubMaster(['deviceState', 'carParams', 'pandaStates'], poll='deviceState')
  pm = messaging.PubMaster(['managerState'])

  write_onroad_params(False, params)
  ensure_running(managed_processes.values(), False, params=params, CP=sm['carParams'], not_run=ignore)

  started_prev = False
  ignition_prev = False

  while True:
    sm.update(1000)

    started = sm['deviceState'].started

    if started and not started_prev:
      params.clear_all(ParamKeyFlag.CLEAR_ON_ONROAD_TRANSITION)
    elif not started and started_prev:
      params.clear_all(ParamKeyFlag.CLEAR_ON_OFFROAD_TRANSITION)

    ignition = any(ps.ignitionLine or ps.ignitionCan for ps in sm['pandaStates'] if ps.pandaType != log.PandaState.PandaType.unknown)
    if ignition and not ignition_prev:
      params.clear_all(ParamKeyFlag.CLEAR_ON_IGNITION_ON)

    # update onroad params, which drives pandad's safety setter thread
    if started != started_prev:
      write_onroad_params(started, params)

    started_prev = started
    ignition_prev = ignition

    ensure_running(managed_processes.values(), started, params=params, CP=sm['carParams'], not_run=ignore)

    running = ' '.join("{}{}\u001b[0m".format("\u001b[32m" if p.proc.is_alive() else "\u001b[31m", p.name)
                       for p in managed_processes.values() if p.proc)
    print(running)
    cloudlog.debug(running)

    # send managerState
    msg = messaging.new_message('managerState', valid=True)
    msg.managerState.processes = [p.get_process_state_msg() for p in managed_processes.values()]
    pm.send('managerState', msg)

    # kick AGNOS power monitoring watchdog
    try:
      if sm.all_checks(['deviceState']):
        with atomic_write("/var/tmp/power_watchdog", "w", overwrite=True) as f:
          f.write(str(time.monotonic()))
    except Exception:
      pass

    # Exit main loop when uninstall/shutdown/reboot is needed
    shutdown = False
    for param in ("DoUninstall", "DoShutdown", "DoReboot"):
      if params.get_bool(param):
        shutdown = True
        params.put("LastManagerExitReason", f"{param} {datetime.datetime.now()}")
        cloudlog.warning(f"Shutting down manager - {param} set")

    if shutdown:
      break


def main() -> None:
  manager_init()
  if os.getenv("PREPAREONLY") is not None:
    return

  # SystemExit on sigterm
  signal.signal(signal.SIGTERM, lambda signum, frame: sys.exit(1))

  try:
    manager_thread()
  except Exception:
    traceback.print_exc()
    sentry.capture_exception()
  finally:
    manager_cleanup()

  params = Params()
  if params.get_bool("DoUninstall"):
    cloudlog.warning("uninstalling")
    HARDWARE.uninstall()
  elif params.get_bool("DoReboot"):
    cloudlog.warning("reboot")
    HARDWARE.reboot()
  elif params.get_bool("DoShutdown"):
    cloudlog.warning("shutdown")
    HARDWARE.shutdown()


if __name__ == "__main__":
  unblock_stdout()

  try:
    main()
  except KeyboardInterrupt:
    print("got CTRL-C, exiting")
  except Exception:
    add_file_handler(cloudlog)
    cloudlog.exception("Manager failed to start")

    try:
      managed_processes['ui'].stop()
    except Exception:
      pass

    # Show last 3 lines of traceback
    error = traceback.format_exc(-3)
    error = "Manager failed to start\n\n" + error
    with TextWindow(error) as t:
      t.wait_for_exit()

    raise

  # manual exit because we are forked
  sys.exit(0)
