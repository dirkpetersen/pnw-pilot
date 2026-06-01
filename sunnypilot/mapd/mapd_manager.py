#!/usr/bin/env python3
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

mapd2xnor: ported from sunnypilot. The pfeiferj mapd binary is bundled (not
downloaded). Default OSM coverage is the Pacific Northwest (WA, OR, ID); change
via the OsmStateName param or by editing PNW_STATES below.

IMPORTANT: state values MUST be 2-letter codes (WA/OR/ID), not full names. The
mapd binary's embedded STATE_BOXES bounding-box table is keyed by 2-letter code;
full names ("Washington") produce "no bounding box data for state code" and never
download. (The binary's custom-bounds path, OSMDownloadBounds, is broken in
v1.12.0 — it nil-panics in DownloadBounds — so the state-code path is the only
working download route.)
"""
import json
import platform
import os
import glob
import shutil
import time
from datetime import datetime

from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.selfdrived.alertmanager import set_offroad_alert
from openpilot.sunnypilot.mapd.live_map_data.osm_map_data import OsmMapData
from openpilot.system.hardware.hw import Paths
from openpilot.sunnypilot.mapd import MAPD_PATH
from openpilot.sunnypilot.mapd.mapd_installer import VERSION, update_installed_version

# mapd2xnor: default Pacific Northwest coverage
PNW_STATES = ["WA", "OR", "ID"]  # mapd2xnor: binary's STATE_BOXES is keyed by 2-letter codes, NOT full names

# mapd2xnor: how often to re-arm the OSM download while the toggle is ON but no map
# data has landed yet (the binary downloads asynchronously; don't re-trigger every loop).
OSM_DOWNLOAD_RETRY_S = 180.0
_last_download_arm = [-1e9]  # monotonic ts of last arm (init far in past -> arm immediately)

params = Params()
mem_params = Params("/dev/shm/params") if platform.system() != "Darwin" else params


def get_files_for_cleanup() -> list[str]:
  paths = [
    f"{Paths.mapd_root()}/db",
    f"{Paths.mapd_root()}/v*"
  ]
  files_to_remove = []
  for path in paths:
    if os.path.exists(path):
      files = glob.glob(path + '/**', recursive=True)
      files_to_remove.extend(files)
  # check for version and mapd files
  if not os.path.isfile(MAPD_PATH):
    files_to_remove.append(MAPD_PATH)
  return files_to_remove


def cleanup_old_osm_data(files_to_remove: list[str]) -> None:
  for file in files_to_remove:
    # Remove trailing slash if path is file
    if file.endswith('/') and os.path.isfile(file[:-1]):
      file = file[:-1]
    # Try to remove as file or symbolic link first
    if os.path.islink(file) or os.path.isfile(file):
      os.remove(file)
    elif os.path.isdir(file):  # If it's a directory
      shutil.rmtree(file, ignore_errors=False)


def request_refresh_osm_location_data(nations: list[str], states: list[str] | None = None) -> None:
  params.put("OsmDownloadedDate", str(datetime.now().timestamp()))
  params.put_bool("OsmDbUpdatesCheck", False)

  osm_download_locations = {
    "nations": nations,
    "states": states or []
  }

  print(f"Downloading maps for {json.dumps(osm_download_locations)}")
  mem_params.put("OSMDownloadLocations", osm_download_locations)


def get_configured_states() -> list[str]:
  """mapd2xnor: read OsmStateName param (comma-separated) or default to PNW."""
  raw = params.get("OsmStateName", return_default=True)
  if not raw:
    return list(PNW_STATES)
  states = [s.strip() for s in str(raw).split(",") if s.strip()]
  return states or list(PNW_STATES)


def osm_data_present() -> bool:
  """True if any OSM .pbf/offline data has already been downloaded."""
  root = Paths.mapd_root()
  if not os.path.isdir(root):
    return False
  for _root, _dirs, files in os.walk(root):
    if any(f.endswith(('.pbf', '.tar.gz')) for f in files):
      return True
  return False


def update_osm_db() -> None:
  # mapd2xnor: the OSM map download is gated on the "Speed limit display/warning (MAPD)"
  # toggle (ShowSpeedLimit). When the toggle is ON and no map data is present yet, KEEP
  # arming the download every loop until data actually lands — the previous one-shot
  # (guarded by OsmAutoRequested) misfired and could never retry. ShowSpeedLimit OFF =
  # never download (the feature is disabled). Re-read live so toggling takes effect.
  if not params.get_bool("ShowSpeedLimit"):
    return

  # retry-until-lands (throttled): while the toggle is ON and no map data is present,
  # re-arm the download every OSM_DOWNLOAD_RETRY_S so a failed/missed first attempt
  # recovers on its own — instead of the old one-shot that got stuck on OsmAutoRequested.
  if not osm_data_present():
    now = time.monotonic()
    if params.get_bool("OsmDbUpdatesCheck"):
      pass  # a download is already armed/in-flight; let it run
    elif now - _last_download_arm[0] > OSM_DOWNLOAD_RETRY_S:
      _last_download_arm[0] = now
      params.put_bool("OsmDbUpdatesCheck", True)

  if params.get_bool("OsmDbUpdatesCheck"):
    cleanup_old_osm_data(get_files_for_cleanup())
    states = get_configured_states()
    # Multiple US states -> drop the country-wide "US" download, keep just the states
    request_refresh_osm_location_data([], states)

  if not mem_params.get("OSMDownloadBounds"):
    mem_params.put("OSMDownloadBounds", "")

  if not mem_params.get("LastGPSPosition"):
    mem_params.put("LastGPSPosition", "{}")


def main_thread():
  update_installed_version(VERSION, params)
  config_realtime_process([0, 1, 2, 3], 5)

  rk = Ratekeeper(1, print_delay_threshold=None)
  live_map_sp = OsmMapData()

  # Create folder needed for OSM
  try:
    os.mkdir(Paths.mapd_root())
  except FileExistsError:
    pass
  except PermissionError:
    cloudlog.exception(f"mapd: failed to make {Paths.mapd_root()}")

  while True:
    show_alert = bool(get_files_for_cleanup()) and params.get_bool("OsmLocal")
    set_offroad_alert("Offroad_OSMUpdateRequired", show_alert, "This alert will be cleared when new maps are downloaded.")

    update_osm_db()
    live_map_sp.tick()
    rk.keep_time()


def main():
  main_thread()


if __name__ == "__main__":
  main()
