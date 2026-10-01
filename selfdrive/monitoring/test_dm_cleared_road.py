"""mapsl2pnw: mapd_configd clears RoadContext/MapOneWay/MapLanes when mapd dies. The DM helper must read the cleared
values as an UNKNOWN road: hold the last verdict for the 90 s window, then fall to the strict timeouts. (Latched
'freeway' values kept the relaxed Highway timeouts on forever, even on city streets.)"""
import pytest

from openpilot.selfdrive.monitoring.helpers import DriverMonitoring, DRIVER_MONITOR_SETTINGS
from openpilot.system.hardware import HARDWARE


class _Mem:
  def __init__(self, **d):
    self.d = d

  def get(self, k, return_default=False):
    return self.d.get(k)


def _dm(mem):
  dm = DriverMonitoring(settings=DRIVER_MONITOR_SETTINGS(device_type=HARDWARE.get_device_type()))
  dm._dm_tier = None
  dm._dm_mode = 1                       # Highway
  dm.mem_params = mem
  dm.params = type("P", (), {"get": staticmethod(lambda k, return_default=False: "1" if k == "DmMode" else None)})()
  return dm


def _refresh(dm, n):
  for _ in range(n):
    dm._dm_refresh_cnt = dm._DM_REFRESH_FRAMES
    dm._refresh_dm_mode()


def test_cleared_road_values_fall_strict_after_the_hold_while_latched_ones_never_do():
  mem = _Mem(RoadContext="freeway", MapOneWay="1", MapLanes="3")
  dm = _dm(mem)
  _refresh(dm, 3)
  assert dm._road_relaxed, "fixture: a live freeway is relaxed"
  strict = dm.settings._DISTRACTED_TIME
  # latched (what a dead mapd used to leave): relaxed forever
  _refresh(dm, dm._ROAD_HOLD_FRAMES // dm._DM_REFRESH_FRAMES + 5)
  assert dm._road_relaxed and dm._pose_step != pytest.approx(dm.settings._DT_DMON / strict)
  # cleared by mapd_configd: held, then strict
  mem.d.update(RoadContext="", MapOneWay="0", MapLanes="0")
  _refresh(dm, 2)
  assert dm._road_relaxed, "the 90 s hold still applies"
  _refresh(dm, dm._ROAD_HOLD_FRAMES // dm._DM_REFRESH_FRAMES + 2)
  assert not dm._road_relaxed
  assert dm._pose_step == pytest.approx(dm.settings._DT_DMON / strict)
  assert dm._phone_step == pytest.approx(dm.settings._DT_DMON / strict)
