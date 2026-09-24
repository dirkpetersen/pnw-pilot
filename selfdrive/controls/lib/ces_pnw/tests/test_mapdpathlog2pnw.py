"""mapdpathlog2pnw -- mapd's path, logged change-only into ces_events (CURVE-MEASURED-SHAPE-DESIGN.md s8 item 0).

Every point here is SYNTHETIC. The real paths are device telemetry and never enter this repo.
"""
import json
import math

import pytest

from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_curvedblive2pnw import (
  LIGHTNING, TESLA, STRAIGHT, _drive,
)

LAT, LON = 10.0, 20.0          # synthetic, nowhere near a road the cars drive


def _line(n, step_m=40.0, lat0=LAT, lon0=LON, v=30.0, heading_deg=0.0):
  pts, la, lo = [], lat0, lon0
  for i in range(n):
    pts.append({"latitude": la, "longitude": lo, "velocity": v + 0.01 * i})
    la += step_m * math.cos(math.radians(heading_deg)) / 111320.0
    lo += step_m * math.sin(math.radians(heading_deg)) / (111320.0 * math.cos(math.radians(la)))
  return pts


class Recorder:
  """The controller surface _mapd_path_step touches."""
  def __init__(self, points=(), lat=LAT, lon=LON):
    self._map_targets = list(points)
    self._cur_lat, self._cur_lon, self._cur_bearing = lat, lon, 0.0
    self._car = "TEST"
    self.recs = []

  def _append_event(self, rec):
    self.recs.append(json.loads(json.dumps(rec)))     # exactly what reaches the file

  def step(self):
    m._mapd_path_log(self)


class TestEncode:
  def test_round_trip_to_a_micro_degree(self):
    pts = _line(40, heading_deg=33.0)
    _, frag = m.mapd_path_encode(pts, LAT, LON)
    back = m.mapd_path_decode(frag)
    assert len(back) == 40 and frag["n"] == 40 and frag["k"] == 40 and frag["i0"] == 0 and frag["bad"] == 0
    for (la, lo, v), p in zip(back, pts, strict=True):
      assert abs(la - p["latitude"]) <= 0.6e-6 and abs(lo - p["longitude"]) <= 0.6e-6
      assert v == pytest.approx(p["velocity"], abs=0.0501)   # 0.1 m/s resolution

  def test_size_bound_on_a_long_path(self):
    """A 1000-point path is logged as at most MAPD_PATH_MAX_PTS points, and the line stays under 8 KB even with
    kilometre-long legs (the widest deltas the encoding can see on a real road)."""
    for step in (40.0, 2000.0):
      rec = Recorder(_line(1000, step_m=step, v=99.9))
      rec.step()
      (r,) = rec.recs
      assert r["k"] == m.MAPD_PATH_MAX_PTS and r["n"] == 1000
      assert len(r["dla"]) == len(r["dlo"]) == m.MAPD_PATH_MAX_PTS - 1 and len(r["v"]) == m.MAPD_PATH_MAX_PTS
      assert len(json.dumps(r)) < 8192, len(json.dumps(r))

  def test_the_window_follows_the_truck_and_only_moves_at_alignment_boundaries(self):
    pts = _line(1000)
    near = 500
    key_a, frag = m.mapd_path_encode(pts, pts[near]["latitude"], pts[near]["longitude"])
    assert frag["i0"] <= near - m.MAPD_PATH_BACK_PTS and near < frag["i0"] + frag["k"]
    assert frag["i0"] % m.MAPD_PATH_ALIGN == 0
    key_b, _ = m.mapd_path_encode(pts, pts[near + 1]["latitude"], pts[near + 1]["longitude"])
    assert key_a == key_b                                     # one node further: no new record
    far = frag["i0"] + m.MAPD_PATH_ALIGN + m.MAPD_PATH_BACK_PTS
    key_c, _ = m.mapd_path_encode(pts, pts[far]["latitude"], pts[far]["longitude"])
    assert key_c != key_a

  def test_bad_points_are_counted_not_logged(self):
    pts = _line(5)
    pts[2] = {"latitude": float("nan"), "longitude": LON, "velocity": 1.0}
    pts.append({"lat": 1.0})
    _, frag = m.mapd_path_encode(pts, LAT, LON)
    assert frag["n"] == 4 and frag["bad"] == 2 and frag["k"] == 4

  def test_a_non_finite_velocity_is_null_not_zero(self):
    pts = _line(3)
    pts[1]["velocity"] = float("inf")
    _, frag = m.mapd_path_encode(pts, LAT, LON)
    assert frag["v"][1] is None and frag["v"][0] is not None


class TestChangeOnly:
  def test_a_static_path_is_one_record(self):
    rec = Recorder(_line(30))
    for _ in range(5):
      rec.step()
    assert len(rec.recs) == 1
    r = rec.recs[0]
    assert r["ev"] == "mapdPath" and r["seq"] == 1 and r["car"] == "TEST" and r["lat"] == LAT

  def test_a_velocity_only_change_is_not_a_new_path(self):
    rec = Recorder(_line(30))
    rec.step()
    rec._map_targets = _line(30, v=12.0)
    rec.step()
    assert len(rec.recs) == 1

  def test_a_geometry_change_and_an_empty_path_are_each_logged_once(self):
    rec = Recorder(_line(30))
    rec.step()
    rec._map_targets = _line(31)
    rec.step()
    rec.step()
    rec._map_targets = []
    rec.step()
    rec.step()
    assert [r["n"] for r in rec.recs] == [30, 31, 0]
    assert [r["seq"] for r in rec.recs] == [1, 2, 3]
    assert rec.recs[-1]["k"] == 0 and rec.recs[-1]["lat0"] is None

  def test_a_failure_is_logged_throttled_and_retried(self, monkeypatch):
    calls = []
    monkeypatch.setattr(m.cloudlog, "exception", lambda msg, *a, **k: calls.append(msg))
    rec = Recorder(_line(30))

    def boom(r):
      raise OSError("disk full")
    rec._append_event = boom
    for _ in range(3):
      rec.step()                                  # never raises
    assert len(calls) == 1 and "mapdpathlog2pnw" in calls[0] and "OSError" in calls[0]
    rec._append_event = lambda r: rec.recs.append(r)
    rec.step()
    assert len(rec.recs) == 1, "a failed write must not mark the path as logged"


class TestOnDisk:
  @pytest.mark.parametrize("fp,brand,op_long", [(LIGHTNING, "ford", False), (TESLA, "tesla", True)])
  def test_the_real_controller_writes_it_on_both_cars(self, monkeypatch, tmp_path, fp, brand, op_long):
    """End to end: the real CESController, the real _append_event, the file on disk. The Tesla too -- its VTSC
    decides on the same path."""
    log = tmp_path / "ces_events.jsonl"
    _drive(monkeypatch, tmp_path / "d", points=STRAIGHT, anchors=None, fp=fp, brand=brand, op_long=op_long,
           ticks=250, log_file=log)
    lines = [json.loads(ln) for ln in log.read_text().splitlines()]
    paths = [r for r in lines if r.get("ev") == "mapdPath"]
    assert len(paths) == 1, [r.get("ev") for r in lines]
    back = m.mapd_path_decode(paths[0])
    assert len(back) == len(STRAIGHT)
    assert abs(back[-1][0] - STRAIGHT[-1]["latitude"]) <= 0.6e-6
    assert any(r.get("ev") == "tick" for r in lines), "the path record must ride alongside the normal ticks"
