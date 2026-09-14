"""gpssel2pnw: ces_events must say which receiver its lat/lon/bearing came from.

ces_pnw cherry-picks blob keys, so a new LastGPSPosition field reaches no log unless it is read and
emitted here (the VTSCStatus / waysel2pnw lesson). Once the truck fix is selected, `lat`/`lon` equal
`car_gps` and the side-by-side comparison channel silently stops being one; `gpsSrc` is what tells.
"""
import inspect
import json
from collections import deque

from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_ces_record_fields import _record
from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_leadrate2pnw import _FakeLead, _run_steer_log_step


class _Mem:
  def __init__(self, pos):
    self._d = {"LastGPSPosition": pos}

  def get(self, key, return_default=False):
    return self._d.get(key)


def _read_map(pos):
  cls = next(o for o in vars(m).values() if inspect.isclass(o) and hasattr(o, "_read_map"))

  class Stub:
    def __getattr__(self, n):
      return None

  g = Stub()
  g.mem_params = _Mem(pos)
  g._toggles = {"curves": True}
  g._vtsc_tele = {}
  g._bearing_hist = deque(maxlen=8)
  cls._read_map.__get__(g)()
  return g


def test_read_map_takes_src_from_the_blob():
  blob = json.dumps({"latitude": 47.6, "longitude": -122.3, "bearing": 264.9, "speed": 0.0, "src": "car", "ts": 1.0})
  g = _read_map(blob)
  assert g._gps_src == "car" and g._cur_bearing == 264.9
  assert _read_map(json.dumps({"latitude": 47.6, "longitude": -122.3, "bearing": 1.0}))._gps_src is None
  assert _read_map(None)._gps_src is None


def test_gpsSrc_reaches_the_tick_and_steer_records():
  assert _record(_gps_src="car")["gpsSrc"] == "car"
  assert _record(_gps_src="device")["gpsSrc"] == "device"
  sm = {"radarState": type("RS", (), {"leadOne": _FakeLead(False)})()}
  assert _run_steer_log_step(47.6, -122.3, 90.0, sm, _gps_src="car").captured[0]["gpsSrc"] == "car"


def test_end_to_end_from_the_real_bridge_loop(monkeypatch):
  """The REAL mapd_configd loop writes the blob from a CarGps feed; the REAL _read_map reads it."""
  from openpilot.system.mapd.tests import test_gps_source_select as T
  res = T._run(monkeypatch, 5.0, (), [(i + 0.3, 47.6 + i * 1e-4, -122.3, 12.5, 30.0, 0.4, 0.4) for i in range(5)])
  g = _read_map(res.mem.store["LastGPSPosition"])
  assert g._gps_src == "car" and g._cur_bearing == 12.5
