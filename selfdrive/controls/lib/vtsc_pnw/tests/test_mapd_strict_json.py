"""silentexc2pnw -- VTSCStatus `mapD` must be strict JSON: null, not a bare `Infinity`, when the fold finds no map curve.

With VtscMapCurves=1 and no map points the fold stores mapD = inf, and json writes it as `Infinity` -- invalid strict
JSON -- into VTSCStatus and, through ces_pnw's cherry-pick, into every ces_events record (4506 of 8633 records in
drives/2026-09-03/hotspot-drive-tesla/ces_events.jsonl). test_overlay_contract.py only checks a default (all-zero)
payload, so it could not see this. Drives the real cap().
"""
import inspect
import json
import math
import types
from collections import deque

import pytest

from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw
from openpilot.selfdrive.controls.lib.vtsc_pnw import vtsc_controller as vc

LAT0, LON0 = 47.6, -122.3
V_SET = 31.3


def _pt(d_m, v):
  return {"latitude": LAT0, "longitude": LON0 + d_m / (111320.0 * math.cos(math.radians(LAT0))), "velocity": v}


class _CP:
  carFingerprint = "TESLA_MODEL_S_HW3"
  brand = "tesla"
  openpilotLongitudinalControl = True


class _Params:
  def __init__(self, map_curves):
    self.map_curves = map_curves

  def get(self, k, return_default=False):
    return "2" if k == "CESMode" else None

  def get_bool(self, k):
    return self.map_curves if k == "VtscMapCurves" else False


class _Mem:
  def __init__(self, targets):
    self.targets, self.payloads = targets, []

  def get(self, k, return_default=False):
    if k == "MapTargetVelocities":
      return self.targets
    if k == "LastGPSPosition":
      return {"latitude": LAT0, "longitude": LON0, "bearing": 90.0}
    return None

  def put_nonblocking(self, k, v):
    self.payloads.append(json.dumps(v))            # what Params(JSON) stores in /dev/shm


class _NS:
  pass


def _sm(vision_k):
  m = _NS()
  m.orientationRate, m.velocity, m.position, m.action = _NS(), _NS(), _NS(), _NS()
  m.orientationRate.z = [0.0] * 20
  m.orientationRate.z[16] = vision_k * V_SET
  m.orientationRate.t = [i * 0.25 for i in range(20)]
  m.velocity.x = [V_SET] * 20
  m.position.x = [V_SET * i * 0.25 for i in range(20)]
  m.action.shouldStop = False
  cc = _NS()
  cc.orientationNED = [0.0, 0.0, 0.0]
  return {"modelV2": m, "carControl": cc}


@pytest.fixture
def clock(monkeypatch):
  t = [1000.0]
  monkeypatch.setattr(vc, "time", types.SimpleNamespace(monotonic=lambda: t[0]))
  return t


def _drive(clock, targets, map_curves=True, vision_k=0.0, ticks=40):
  c = vc.VTSCController(_CP(), params=_Params(map_curves))
  c.mem_params = _Mem(targets)
  seen = []
  for _ in range(ticks):
    clock[0] += 0.05
    c.cap(_sm(vision_k), V_SET, V_SET)
    seen.append((c._tele_map_d, c.overlay_payload()))
  return seen, c.mem_params.payloads


def _tele_after_ces_read(blob):
  cls = next(o for o in vars(ces_pnw).values() if inspect.isclass(o) and hasattr(o, "_read_map")
             and hasattr(o, "_event_record"))

  class Stub:
    def __getattr__(self, n):
      return None
  g = Stub()
  g.mem_params = types.SimpleNamespace(get=lambda k, return_default=False: blob if k == "VTSCStatus" else None)
  g._toggles, g._vtsc_tele, g._bearing_hist = {"curves": True}, {}, deque(maxlen=8)
  cls._read_map.__get__(g)()
  return g._vtsc_tele


@pytest.mark.parametrize("vision_k", [0.0, 2.5 / 15.0 ** 2], ids=["straight", "vision-curve"])
def test_no_map_points_publishes_null_not_infinity(clock, vision_k):
  seen, blobs = _drive(clock, [], vision_k=vision_k)
  assert all(d == float("inf") for d, _ in seen)      # positive control: the fold ran and found nothing
  assert all(p["mapD"] is None for _, p in seen)
  assert blobs and all("Infinity" not in b and "NaN" not in b for b in blobs)
  for b in blobs:
    json.loads(b, parse_constant=lambda tok: pytest.fail(f"non-strict JSON token {tok} in VTSCStatus"))


def test_a_real_map_distance_is_unchanged(clock):
  seen, _ = _drive(clock, [_pt(200, 13.0)])
  assert all(math.isfinite(d) and d > 100.0 for d, _ in seen)
  assert all(p["mapD"] == round(float(d), 0) and isinstance(p["mapD"], float) for d, p in seen)


def test_no_fold_still_reports_zero(clock):
  seen, _ = _drive(clock, [], map_curves=False)
  assert all(d == 0.0 and p["mapD"] == 0.0 for d, p in seen)


def test_the_ces_events_column_is_null(clock):
  """The last mile: ces_pnw copies the key verbatim, so the record now carries null."""
  _, blobs = _drive(clock, [], ticks=5)
  tele = _tele_after_ces_read(blobs[-1])
  assert "mapD" in tele and tele["mapD"] is None
  assert json.dumps(tele, allow_nan=False)
