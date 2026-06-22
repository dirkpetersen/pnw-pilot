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
from openpilot.sunnypilot.mapd import coverage   # mapd2pnw: GPS -> region coverage for on-demand download

# mapd2pnw: default Pacific Northwest coverage = WA, OR, ID (US states).
#
# IMMATURITY CAVEAT (this feature is copied from sunnypilot/pfeiferj and may need maturing): the
# bundled mapd binary's region index keys US states by 2-letter code (STATE_BOXES). Canada has NO
# province granularity (only a whole-"Canada" nation box, multi-GB) — so British Columbia is NOT in
# the default. Instead, the "Get map for this location" on-demand toggle (see GetMapForLocation /
# coverage.py) lets the driver download whatever region they're currently in — including a Canadian
# nation download if they're in BC — only when they choose to, rather than auto-pulling all of Canada.
PNW_STATES = ["WA", "OR", "ID"]   # US states (binary STATE_BOXES, 2-letter codes) — the shipped default

# mapd2xnor: how often to re-arm the OSM download while the toggle is ON but no map
# data has landed yet (the binary downloads asynchronously; don't re-trigger every loop).
OSM_DOWNLOAD_RETRY_S = 180.0
_last_download_arm = [-1e9]  # monotonic ts of last arm (init far in past -> arm immediately)
_was_in_flight = [False]     # previous-loop download_in_flight() (pass-end edge detection)
_marker_fails = [0]          # consecutive loops the marker looked invalid (hysteresis)
MARKER_FAIL_LOOPS = 30       # require ~30 s of consistent invalidity before re-downloading: a single
                             #   transient read failure (/data/media I/O hiccup) must NOT trigger a
                             #   ~450 MB re-download (suspected cause of the 13:31 rogue pass)

# mapd2xnor: PERSISTENT completion marker. The binary's OSMDownloadProgress and the pending
# OSMDownloadLocations request both live in /dev/shm (tmpfs) and are WIPED ON EVERY REBOOT,
# so after a reboot the manager couldn't tell the download ever finished and re-armed a full
# multi-hundred-MB re-download of data already on disk — on every single reboot. The marker
# lives next to the tiles on /data/media (reboot-safe) and records the tile count + states,
# so "complete" survives reboots and is re-validated against what's actually on disk.
MARKER_FILE = ".osm_download_complete.json"

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
  """mapd2xnor: read OsmStateName param (comma-separated) or default to PNW. mapd2pnw: also folds in
  any region the user has on-demand downloaded via "Get map for this location" (OsmStateName is the
  persistent download set, so on_demand_download() appends to it)."""
  raw = params.get("OsmStateName", return_default=True)
  if not raw:
    return list(PNW_STATES)
  states = [s.strip() for s in str(raw).split(",") if s.strip()]
  return states or list(PNW_STATES)


def _current_gps():
  """(lat, lon) from LastGPSPosition (mapd writes it to the in-memory store), or (None, None)."""
  try:
    raw = mem_params.get("LastGPSPosition")
    if not raw:
      return None, None
    d = json.loads(raw) if isinstance(raw, (str, bytes, bytearray)) else raw
    return float(d["latitude"]), float(d["longitude"])
  except Exception:
    return None, None


def _on_priority_wifi() -> bool:
  """mapd2pnw: True only when connected to a configured Priority Network WiFi (network2xnor). The
  default WA/OR/ID map download is gated on this so large downloads happen only on a trusted home/work
  WiFi, never on cellular/hotspot. Fails CLOSED (no download) if we can't confirm a priority WiFi."""
  try:
    from openpilot.system.networkd import priority_networks as pn
    nets = pn.parse(params.get("TetheringPriorityNetworks"),
                    legacy_ssid=(params.get("TetheringPriorityWifi") or ""),
                    legacy_home_raw=params.get("TetheringHomeLocation"))
    ssids = set(pn.ssids(nets))
    if not ssids:
      return False
    import subprocess
    out = subprocess.run(["nmcli", "-t", "-f", "NAME,TYPE,DEVICE", "con", "show", "--active"],
                         capture_output=True, text=True, timeout=10, check=False).stdout
    for line in out.splitlines():
      parts = line.split(":")
      if len(parts) >= 2 and "wireless" in parts[1]:
        name = parts[0].replace("openpilot connection ", "")
        if name in ssids:
          return True
    return False
  except Exception:
    return False


def update_location_coverage() -> None:
  """mapd2pnw: each loop, work out which region the device is currently in and whether it's already
  covered by a downloaded map, and publish that for the UI (MapForLocationRegion / MapForLocationCovered).
  The UI greys "Get map for this location" when covered (or no fix); enables it (off) when uncovered."""
  lat, lon = _current_gps()
  region = coverage.region_for_gps(lat, lon)
  downloaded = get_configured_states()
  covered = coverage.is_covered(region, downloaded)
  params.put("MapForLocationRegion", region or "")
  params.put_bool("MapForLocationCovered", covered)


def on_demand_download() -> None:
  """mapd2pnw: when the user flips GetMapForLocation ON and the current region isn't already in the
  download set, add it to OsmStateName (persistent) and clear the toggle. The normal update_osm_db()
  arming loop then downloads it. This is how BC (or any out-of-PNW region) gets pulled — only when the
  driver asks, only the region they're actually in (a US state, or a whole nation like Canada for BC)."""
  if not params.get_bool("GetMapForLocation"):
    return
  lat, lon = _current_gps()
  region = coverage.region_for_gps(lat, lon)
  if region is None:
    params.put_bool("GetMapForLocation", False)   # no fix / unknown region -> nothing to add
    return
  current = get_configured_states()
  if region not in current:
    current.append(region)
    params.put("OsmStateName", ",".join(current))
    cloudlog.info(f"mapd2pnw: on-demand add region {region} ({coverage.region_full_name(region)}) " +
                  f"to download set -> {current}")
  params.put_bool("GetMapForLocation", False)   # one-shot: consumed


def osm_data_present() -> bool:
  """True if any OSM .pbf/offline data has already been downloaded."""
  root = Paths.mapd_root()
  if not os.path.isdir(root):
    return False
  for _root, _dirs, files in os.walk(root):
    if any(f.endswith(('.pbf', '.tar.gz')) for f in files):
      return True
  return False


def _count_offline_tiles() -> int:
  """Number of extracted tile files under <mapd_root>/offline (the binary's final store)."""
  root = os.path.join(Paths.mapd_root(), "offline")
  n = 0
  for _r, _d, files in os.walk(root):
    n += len(files)
  return n


def _marker_path() -> str:
  return os.path.join(Paths.mapd_root(), MARKER_FILE)


def _read_marker() -> dict | None:
  try:
    with open(_marker_path()) as f:
      m = json.load(f)
    return m if isinstance(m, dict) else None
  except (FileNotFoundError, ValueError, OSError):
    return None


def _write_marker(states: list[str], tiles: int) -> None:
  try:
    with open(_marker_path(), "w") as f:
      json.dump({"states": sorted(states), "tiles": tiles, "mapd_version": VERSION,
                 "completed_at": datetime.now().isoformat(timespec="seconds")}, f)
    cloudlog.info(f"mapd: wrote completion marker ({tiles} tiles, {states})")
  except OSError:
    cloudlog.exception("mapd: failed to write completion marker")


def _clear_marker() -> None:
  try:
    os.remove(_marker_path())
  except FileNotFoundError:
    pass
  except OSError:
    cloudlog.exception("mapd: failed to clear completion marker")


def _read_mem_param_file(key: str) -> bytes | None:
  """mapd2xnor: read a /dev/shm params value straight off disk.

  The mapd binary writes OSMDownloadProgress / OSMDownloadLocations with its own Go
  param writer, which does NOT register keys. The Python Params wrapper validates keys
  and raises UnknownKeyName for OSMDownloadProgress (it isn't in params_keys.h), so we
  read the file directly to avoid both a registration dependency and that exception.
  """
  try:
    with open(os.path.join("/dev/shm/params/d", key), "rb") as f:
      return f.read()
  except (FileNotFoundError, OSError):
    return None


def osm_download_complete() -> bool:
  """mapd2xnor: True only when the binary's OSMDownloadProgress shows every tile
  fetched (downloaded_files >= total_files, total > 0). The binary writes this
  param as it downloads and clears OSMDownloadLocations when the single pass ends.

  This is the correct 'done' signal — NOT mere file presence. The binary does one
  sequential pass with no resume (v1.12.0); if it's killed partway (e.g. boot/restart
  mid-download), partial .tar.gz tiles remain but the download is incomplete, so we
  must re-arm. Treating 'any file exists' as done froze the download at the partial
  state. A frozen progress where downloaded < total likewise means interrupted -> re-arm.
  """
  raw = _read_mem_param_file("OSMDownloadProgress")
  if not raw:
    return False
  try:
    prog = json.loads(raw)
    total = int(prog.get("total_files", 0))
    done = int(prog.get("downloaded_files", 0))
  except (ValueError, TypeError, AttributeError):
    return False
  return total > 0 and done >= total


def download_in_flight() -> bool:
  """mapd2xnor: a download is actively armed/running if the binary still has a
  pending OSMDownloadLocations request (it clears it when the pass ends)."""
  return bool(_read_mem_param_file("OSMDownloadLocations"))


def update_osm_db() -> None:
  # mapd: (re)arm the OSM download — throttled — until OSMDownloadProgress shows all tiles fetched.
  # The binary does a single non-resuming pass and clears OSMDownloadLocations when done, so an
  # interrupted download (boot/restart mid-pass) must be re-armed to finish. Re-read params live.
  #
  # GATE 1 (purpose): only download if a map is actually wanted — either the speed-limit display is on
  # (ShowSpeedLimit) or the user has on-demand-added a region (so OsmStateName extends past the default).
  # GATE 2 (mapd2pnw, network): only on a configured Priority Network WiFi — never burn cellular/hotspot
  # data on a multi-hundred-MB (or, for a nation, multi-GB) download. Fails CLOSED.
  want_maps = params.get_bool("ShowSpeedLimit") or (get_configured_states() != list(PNW_STATES)) \
    or params.get_bool("GetMapForLocation")
  if not want_maps:
    return
  if not _on_priority_wifi():
    return

  states = get_configured_states()
  in_flight = download_in_flight()

  # --- persistent completion (the reboot re-download fix) -----------------------------
  # OSMDownloadProgress lives in /dev/shm and is wiped every reboot, so it alone cannot
  # prove completion across boots. A marker file next to the tiles records a finished
  # pass; trust it as long as the wanted states haven't changed and the tiles are still
  # on disk (>= 90% of the marker count guards against a wiped/garbage-collected store).
  marker = _read_marker()
  if marker is not None:
    if sorted(states) != marker.get("states"):
      cloudlog.info(f"mapd: states changed {marker.get('states')} -> {sorted(states)}; re-download")
      _clear_marker()
      marker = None
    elif _count_offline_tiles() < int(marker.get("tiles", 0)) * 0.9:
      cloudlog.info("mapd: offline tiles missing vs marker; re-download")
      _clear_marker()
      marker = None
  # hysteresis: a single bad loop (transient /data/media read hiccup) must not trigger a
  # ~450 MB re-download — require sustained invalidity before the arm path may run.
  _marker_fails[0] = 0 if marker is not None else _marker_fails[0] + 1
  if marker is not None:
    # Complete and intact -> NEVER re-arm. Also DISARM any stale download state left by
    # older code or a previous boot: a persistent OsmDbUpdatesCheck=1 or a pending
    # OSMDownloadLocations request would make the binary re-download regardless of what
    # this manager does, so actively clear both (the binary tolerates the param vanishing).
    if params.get_bool("OsmDbUpdatesCheck"):
      params.put_bool("OsmDbUpdatesCheck", False)
    if in_flight:
      cloudlog.info("mapd: marker valid but download request pending — clearing stale request")
      try:
        os.remove("/dev/shm/params/d/OSMDownloadLocations")
      except OSError:
        pass
    _was_in_flight[0] = False
    return  # no more re-downloads on reboot

  # --- pass-end edge: binary just cleared OSMDownloadLocations -------------------------
  # Completion signal: either the in-flight request ended with full progress, or (same
  # boot) the progress param itself shows done. Write the marker so it sticks.
  if osm_download_complete() or (_was_in_flight[0] and not in_flight and osm_data_present()):
    tiles = _count_offline_tiles()
    if tiles > 0:
      _write_marker(states, tiles)
      _was_in_flight[0] = in_flight
      return
  _was_in_flight[0] = in_flight

  # --- retry-until-complete (throttled) ------------------------------------------------
  if _marker_fails[0] < MARKER_FAIL_LOOPS and _marker_fails[0] > 0:
    return  # marker invalid but not yet *consistently* — wait out the hysteresis window

  if not in_flight:
    now = time.monotonic()
    if params.get_bool("OsmDbUpdatesCheck"):
      pass  # already armed for this loop; let request_refresh fire below
    elif now - _last_download_arm[0] > OSM_DOWNLOAD_RETRY_S:
      _last_download_arm[0] = now
      params.put_bool("OsmDbUpdatesCheck", True)

  if params.get_bool("OsmDbUpdatesCheck"):
    cleanup_old_osm_data(get_files_for_cleanup())
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

    # mapd2pnw: publish current-location coverage (for the "Get map for this location" toggle) and
    # consume the toggle if the user flipped it on. Order: compute coverage first, then act on the
    # toggle, then run the (priority-WiFi-gated) download arming.
    try:
      update_location_coverage()
      on_demand_download()
    except Exception:
      cloudlog.exception("mapd2pnw: location-coverage / on-demand update failed")

    update_osm_db()
    live_map_sp.tick()
    rk.keep_time()


def main():
  main_thread()


if __name__ == "__main__":
  main()
