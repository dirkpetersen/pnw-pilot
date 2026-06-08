"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

mapd2xnor: GPS source adapted from sunnypilot's `liveLocationKalman` (which xnor
does not publish) to the device's actual GPS stream, selected at runtime by
common.gps.get_gps_location_service(): `gpsLocationExternal` when a ublox is
present, else `gpsLocation` (the comma 3X's internal qcom GPS — qcomgpsd). The
mapd binary itself is GPS-source-agnostic — it only reads the `LastGPSPosition`
param this bridge writes — so swapping the location stream here is sufficient;
no C++ locationd port is required.

(Earlier this hard-coded `gpsLocationExternal`, which is NOT published on the
3X — only `gpsLocation` is — so LastGPSPosition stayed empty and no speed limit
ever resolved. Using get_gps_location_service() is exactly how sunnypilot does it.)
"""
from abc import abstractmethod, ABC

import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.constants import CV
from openpilot.common.gps import get_gps_location_service
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot.navd.helpers import coordinate_from_param

MAX_SPEED_LIMIT = V_CRUISE_UNSET * CV.KPH_TO_MS


class BaseMapData(ABC):
  def __init__(self):
    self.params = Params()

    # mapd2xnor: pick the GPS stream the device actually publishes (qcom gpsLocation
    # on the comma 3X; gpsLocationExternal only if a ublox is present).
    self.gps_location_service = get_gps_location_service(self.params)
    self.sm = messaging.SubMaster([self.gps_location_service])
    self.pm = messaging.PubMaster(['liveMapDataSP'])

    self.localizer_valid = False
    self.last_bearing = None
    self.last_position = coordinate_from_param("LastGPSPosition", self.params)

  @abstractmethod
  def update_location(self) -> None:
    pass

  @abstractmethod
  def get_current_speed_limit(self) -> float:
    pass

  @abstractmethod
  def get_next_speed_limit_and_distance(self) -> tuple[float, float]:
    pass

  @abstractmethod
  def get_current_road_name(self) -> str:
    pass

  def publish(self) -> None:
    speed_limit = self.get_current_speed_limit()
    next_speed_limit, next_speed_limit_distance = self.get_next_speed_limit_and_distance()

    mapd_sp_send = messaging.new_message('liveMapDataSP')
    mapd_sp_send.valid = bool(self.sm[self.gps_location_service].hasFix)
    live_map_data = mapd_sp_send.liveMapDataSP

    live_map_data.speedLimitValid = bool(MAX_SPEED_LIMIT > speed_limit > 0)
    live_map_data.speedLimit = speed_limit
    live_map_data.speedLimitAheadValid = bool(MAX_SPEED_LIMIT > next_speed_limit > 0)
    live_map_data.speedLimitAhead = next_speed_limit
    live_map_data.speedLimitAheadDistance = next_speed_limit_distance
    live_map_data.roadName = self.get_current_road_name()

    self.pm.send('liveMapDataSP', mapd_sp_send)

  def tick(self) -> None:
    self.sm.update(0)
    self.update_location()
    self.publish()
