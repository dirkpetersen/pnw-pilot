"""mapsl2pnw: mapd_configd now writes MapSpeedLimit "0.0" when mapd is silent. These tests prove the consumers read that
as UNKNOWN -- not "limit 0" (a clamp to 0), not an error. The writer side is system/mapd/tests/test_speedlimit_clear.py.
"""
import json
import logging
import types

import pytest

from openpilot.common.swaglog import cloudlog
import openpilot.selfdrive.controls.lib.speedadjust_pnw.speedadjust_controller as sa
import openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw as ces
from openpilot.selfdrive.controls.lib.vtsc_pnw.vtsc_controller import VTSCController

MPH = sa.MPH_TO_MS
V75 = 75 * MPH
POLICE = {"state": "alert", "dist_mi": 0.3, "tier": "confirmed", "cap": {"state": "alert", "dist_mi": 0.3, "key": "k"}}


class _Clock:
  t = 1000.0

  @classmethod
  def monotonic(cls):
    return cls.t

  @classmethod
  def time(cls):
    return cls.t


class _Mem:
  def __init__(self, sl):
    self.sl = sl

  def get(self, k, return_default=True):
    if k == "MapSpeedLimit":
      return self.sl
    if k == "LocationServices":
      return json.dumps({"police": POLICE})
    return None

  def put_nonblocking(self, k, v):
    pass


class _P:
  def get(self, k, return_default=True):
    return "2" if k == "AutoSpeedReduce" else None


class _Records(logging.Handler):
  def __init__(self):
    super().__init__(level=logging.DEBUG)
    self.records = []

  def emit(self, record):
    self.records.append(record)


@pytest.fixture
def logs():
  h = _Records()
  cloudlog.addHandler(h)
  yield h
  cloudlog.removeHandler(h)


def _ctrl(monkeypatch, sl):
  _Clock.t = 1000.0
  monkeypatch.setattr(sa, "time", _Clock)
  c = sa.SpeedAdjustController(types.SimpleNamespace(openpilotLongitudinalControl=False), params=_P())
  c.mem_params = _Mem(sl)
  return c


def _read(c):
  c._sl = c._read_speed_limit()                 # what _cap_impl does with the result (the hold keys off self._sl)
  return c._sl


def test_speedadjust_unknown_after_the_hold_and_no_error(monkeypatch, logs):
  c = _ctrl(monkeypatch, str(60 * MPH))
  assert _read(c) == pytest.approx(60 * MPH)
  c.mem_params.sl = "0.0"                       # mapd died, mapd_configd cleared the limit
  _Clock.t += 1.0
  assert _read(c) == pytest.approx(60 * MPH), "a brief dropout is still held (SL_HOLD_S)"
  _Clock.t += sa.SL_HOLD_S + 1.0
  assert _read(c) == 0.0, "after the hold the limit is UNKNOWN, not the last posted one"
  assert c._sl_raw == 0.0
  assert [r for r in logs.records if "unreadable" in str(r.msg)] == [], "'0.0' is a normal value, not a read failure"


def test_speedadjust_rules_stop_on_unknown_and_run_on_a_live_limit(monkeypatch):
  """Rule 1 (limit cap): with the limit live a SpeedAdjustTarget below the 75 mph cruise is published; once the map
  says unknown (after the hold) the cap is released back to the cruise -- no slowdown, not a clamp to 0, not a crash."""
  c = _ctrl(monkeypatch, str(45 * MPH))
  puts = []
  c.mem_params.put_nonblocking = lambda k, v: puts.append((k, v)) if k == "SpeedAdjustTarget" else None
  sm = {"carState": types.SimpleNamespace(gasPressed=False, brakePressed=False,
                                          cruiseState=types.SimpleNamespace(speed=V75, enabled=True))}

  def run(n):
    for _ in range(n):
      _Clock.t += 0.25
      c._sa_pub_t = -1e9
      c.cap(sm, V75, V75, V75, True)
    return next((v for _, v in reversed(puts)), None)

  live = run(40)
  assert live and live.get("target") is not None and live["target"] < V75 - 1.0, \
    f"fixture broken: a live 45 mph limit must publish a target below the 75 mph cruise (got {live})"
  c.mem_params.sl = "0.0"
  puts.clear()
  after = run(80)                                # > SL_HOLD_S of unknown
  assert after in (None, {}) or after.get("target", V75) >= V75 - 0.01, \
    f"unknown limit must release the cap (restore to the 75 mph cruise), got {after}"


def _ces(clock_t=5000.0):
  class _CESP:
    def get(self, k, return_default=False):
      return None

    def get_bool(self, k):
      return False
  cp = types.SimpleNamespace(carFingerprint="TESLA_MODEL_S_HW3", brand="tesla", openpilotLongitudinalControl=True)
  return ces.CESController(cp, params=_CESP())


def test_ces_reads_unknown_as_zero_and_clears_the_conditional_limit():
  c = _ces()

  class Mem:
    d = {"MapSpeedLimit": "26.8", "MapConditionalSpeedLimit": "45 @ (Mo-Fr 07:00-09:00)"}

    def get(self, k, return_default=False):
      return self.d.get(k)
  c.mem_params = Mem()
  c._read_map()
  assert c._speed_limit == pytest.approx(26.8) and c._cond_spd_lim.startswith("45")
  Mem.d = {"MapSpeedLimit": "0.0", "MapConditionalSpeedLimit": ""}
  c._read_map()
  assert c._speed_limit == 0.0 and c._cond_spd_lim == ""


def test_vtsc_freeway_floor_reads_unknown_without_a_failure_log(logs):
  class FP:
    d = {"CESMode": "2"}

    def get(self, k, return_default=False):
      return self.d.get(k)

    def get_bool(self, k):
      return False

    def put_nonblocking(self, k, v):
      pass
  cp = types.SimpleNamespace(carFingerprint="TESLA_MODEL_S_HW3", brand="tesla", openpilotLongitudinalControl=True)
  ctrl = VTSCController(cp, params=FP())
  d = {"MapSpeedLimit": "26.8", "RoadContext": "freeway"}
  ctrl.mem_params = types.SimpleNamespace(get=lambda k, return_default=False: d.get(k))
  ctrl._read_enabled(0.0)
  assert ctrl._speed_limit == pytest.approx(26.8) and ctrl._is_freeway
  d["MapSpeedLimit"] = "0.0"
  ctrl._read_enabled(1.0)
  assert ctrl._speed_limit == 0.0          # no freeway floor at 0 m/s
  assert [r for r in logs.records if "unreadable" in str(r.msg)] == []
