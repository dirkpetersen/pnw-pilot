"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

mapd2xnor: location read adapted to the device GPS stream (gpsLocation on the comma 3X
qcom GPS, or gpsLocationExternal with a ublox) via common.gps.get_gps_location_service.
"""
import json
import os
import platform

from openpilot.common.params import Params
from openpilot.sunnypilot.mapd.live_map_data.base_map_data import BaseMapData
from openpilot.sunnypilot.navd.helpers import Coordinate

_SHM_D = "/dev/shm/params/d"


def _read_shm(key: str) -> str | None:
  """mapd2xnor: read a mapd-binary-written /dev/shm param straight off disk, fresh every call.

  The binary writes MapSpeedLimit/RoadName/... with its own Go param writer. A long-lived
  Python Params("/dev/shm/params") handle constructed before/across a binary (re)start goes
  BLIND — it kept returning nothing while the files plainly held values, so liveMapDataSP
  published speedLimit=0/valid=False forever and the UI showed no limit (observed after boot
  and after any mapd restart). A plain path-based open each tick (1 Hz, tmpfs — free) cannot
  go stale. Same pattern mapd_manager uses for OSMDownloadProgress."""
  try:
    with open(os.path.join(_SHM_D, key)) as f:
      return f.read()
  except OSError:
    return None


class OsmMapData(BaseMapData):
  def __init__(self):
    super().__init__()
    self.mem_params = Params("/dev/shm/params") if platform.system() != "Darwin" else self.params

  def update_location(self) -> None:
    # mapd2xnor: read the runtime-selected GPS stream (gpsLocation on the 3X qcom GPS,
    # or gpsLocationExternal if a ublox is present) — both carry lat/lon/bearingDeg/hasFix,
    # replacing sunnypilot's liveLocationKalman positionGeodetic/calibratedOrientationNED.
    location = self.sm[self.gps_location_service]
    self.localizer_valid = bool(location.hasFix)

    if self.localizer_valid:
      self.last_bearing = float(location.bearingDeg)
      self.last_position = Coordinate(location.latitude, location.longitude)

    if self.last_position is None:
      return

    params = {
      "latitude": self.last_position.latitude,
      "longitude": self.last_position.longitude,
    }

    if self.last_bearing is not None:
      params['bearing'] = self.last_bearing

    self.mem_params.put("LastGPSPosition", json.dumps(params))

  def get_current_speed_limit(self) -> float:
    try:
      return float(_read_shm("MapSpeedLimit") or 0.0)
    except ValueError:
      return 0.0

  def get_current_road_name(self) -> str:
    return str(_read_shm("RoadName") or "")

  def get_next_speed_limit_and_distance(self) -> tuple[float, float]:
    try:
      next_speed_limit_section = json.loads(_read_shm("NextMapSpeedLimit") or "{}")
    except ValueError:
      next_speed_limit_section = {}
    if not isinstance(next_speed_limit_section, dict):
      next_speed_limit_section = {}
    next_speed_limit = next_speed_limit_section.get('speedlimit', 0.0)
    next_speed_limit_latitude = next_speed_limit_section.get('latitude')
    next_speed_limit_longitude = next_speed_limit_section.get('longitude')
    next_speed_limit_distance = 0.0

    if next_speed_limit_latitude and next_speed_limit_longitude:
      next_speed_limit_coordinates = Coordinate(next_speed_limit_latitude, next_speed_limit_longitude)
      next_speed_limit_distance = (self.last_position or Coordinate(0, 0)).distance_to(next_speed_limit_coordinates)

    return next_speed_limit, next_speed_limit_distance
