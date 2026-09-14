"""truckdecode2pnw (A): the truck's own dead-reckoning flag (CarGps `dr`/`drAge`, 0x463 GPS_Actual_vs_Infer_pos) is
logged next to gpsdr2pnw's inferred DR in mapd_configd_gps_source, and selects NOTHING.

Evidence (drives/2026-09-12/central-oregon-weekend/TRUCK_DECODE.md): 1,345 publishes of every local Lightning rlog through
the real opendbc carstate and the real CarGpsSource -- flag 0 & inferred 0: 1,102; flag 1 & inferred 1: 231; flag 1 &
inferred 0: 12 (10 = the inference's 10 s entry at the Sat 06:24 cold start, 2 = the flag lagging HDOP's recovery at
Sat 14:18:44); flag 0 & inferred 1: none. One DR episode, no tunnel, no observed entry: control stays on the inference.

All scenarios run the REAL mapd_configd.main() loop (configd_replay.py) at 20 Hz.
"""
import pytest

from openpilot.system.mapd import mapd_configd as M
from openpilot.system.mapd.tests import configd_replay as R
from openpilot.system.mapd.tests.test_gps_source_select import LIGHTNING_CP, TESLA_CP, DT, _cargps, _device_track, _grid, \
  _source_events, _straight


def _run(monkeypatch, t_end, device, truck, flags, cp=LIGHTNING_CP):
  """truck rows as test_gps_source_select; flags[i] = extra CarGps keys for publish i (e.g. {"dr": 1, "drAge": 0.3})."""
  steps = {round(k * DT, 3): [("mapdOut", {})] for k in range(int(round(t_end / DT)) + 1)}
  for t, lat, lon, spd, brg, fix in device:
    if 0.0 <= t <= t_end:
      steps[_grid(t)].append(R.gps_msg(lat, lon, has_fix=fix, speed=spd, bearing=brg))
  mem_script = {}
  for i, (t, lat, lon, hdg, spd, hdop, age) in enumerate(truck):
    if 0.0 <= t <= t_end:
      mem_script.setdefault(_grid(t), {})["CarGps"] = {**_cargps(t, lat, lon, hdg, spd, hdop, age), **flags(i)}
  return R.run(monkeypatch, sorted(steps.items()), persistent={"CarParams": cp} if cp is not None else {},
               mem_script=mem_script)


def _with_hdop(truck, hdop_of_i):
  return [r[:5] + (hdop_of_i(i),) + r[6:] for i, r in enumerate(truck)]


def _flag(dr_of_i, age=0.3):
  return lambda i: {"dr": dr_of_i(i), "drAge": age}


class TestLoggedNextToTheInference:
  def test_a_cold_start_logs_both_flags_side_by_side(self, monkeypatch):
    """Sat 06:24-like: HDOP 3.8 and the truck's flag 1 from the first publish. The inference needs 10 s; the flag
    does not -- both are in the same events, so the disagreement window is on record."""
    truck = _with_hdop(_straight(20), lambda i: 3.8)
    res = _run(monkeypatch, 20.0, _device_track(20), truck, _flag(lambda i: 1))
    ev = [(e["car"], e["truck_dr"]) for e in _source_events(res)]
    assert ("ok", 1) in ev, ev               # inferred not-yet-DR while the truck already says inferred
    assert ev[-1] == ("dr", 1), ev

  def test_a_flag_change_alone_is_logged(self, monkeypatch):
    """HDOP stays good (inferred never DR); the truck's flag goes 0 -> 1 -> 0. The source and kind never change, so
    without the flag in the change key these disagreements would be invisible."""
    truck = _straight(30)
    res = _run(monkeypatch, 30.0, _device_track(30), truck, _flag(lambda i: 1 if 10 <= i < 15 else 0))
    ev = [(e["src"], e["car"], e["truck_dr"]) for e in _source_events(res)]
    assert [d for _, _, d in ev[-3:]] == [0, 1, 0], ev
    assert {k for _, k, _ in ev[-3:]} == {"ok"}
    flip = [e for e in _source_events(res) if e["truck_dr"] == 1]
    assert len(flip) == 1 and flip[0]["src"] == "car"

  def test_no_flag_keys_logs_None_and_a_steady_flag_adds_no_lines(self, monkeypatch):
    """An opendbc without the decode publishes no `dr`: truck_dr is None throughout. A flag that never changes
    produces exactly the same (src, car) event sequence -- the change key only adds a line when the flag moves."""
    truck = _with_hdop(_straight(30), lambda i: 3.8 if 5 <= i < 25 else 0.4)
    bare = _source_events(_run(monkeypatch, 30.0, _device_track(30), truck, lambda i: {}))
    steady = _source_events(_run(monkeypatch, 30.0, _device_track(30), truck, _flag(lambda i: 0)))
    assert bare and all(e["truck_dr"] is None for e in bare)
    assert ("device", "dr") in [(e["src"], e["car"]) for e in bare]            # the scenario really yields
    assert [(e["src"], e["car"]) for e in steady] == [(e["src"], e["car"]) for e in bare]


class TestTelemetryOnly:
  @pytest.mark.parametrize("dr_of_i", [lambda i: 1, lambda i: 0, lambda i: i % 2])
  def test_positions_are_identical_whatever_the_flag_says(self, monkeypatch, dr_of_i):
    """The flag CONTRADICTS HDOP here (1 while HDOP is good, 0 while it is degraded long enough to yield): if it
    selected anything, the writes would differ."""
    truck = _with_hdop(_straight(40), lambda i: 3.8 if 5 <= i < 25 else 0.4)
    with_flag = _run(monkeypatch, 40.0, _device_track(40), truck, _flag(dr_of_i))
    without = _run(monkeypatch, 40.0, _device_track(40), truck, lambda i: {})
    assert with_flag.mem.writes == without.mem.writes
    assert {d["src"] for _, d in R.positions(with_flag)} == {"car", "device"}   # the yield really happened

  def test_tesla_never_reads_it(self, monkeypatch):
    truck = _straight(20)
    tesla = _run(monkeypatch, 20.0, _device_track(20), truck, _flag(lambda i: 1), cp=TESLA_CP)
    assert "CarGps" not in tesla.mem.gets
    assert not _source_events(tesla)


class TestUnknown:
  @pytest.mark.parametrize("extra", [
    {"dr": 1, "drAge": M.CAR_GPS_MAX_AGE_S + 0.01},   # stale Nav_2 frame
    {"dr": 1, "drAge": None},
    {"dr": 1, "drAge": float("nan")},
    {"dr": 1, "drAge": -0.5},                          # clocks disagree: not a live reading
    {"dr": None, "drAge": None},                       # never received
    {"dr": 2, "drAge": 0.3},
    {"dr": "1", "drAge": 0.3},
    {"drAge": 0.3},
  ])
  def test_anything_but_a_fresh_0_or_1_is_None(self, monkeypatch, extra):
    res = _run(monkeypatch, 8.0, _device_track(8), _straight(8), lambda i: extra)
    ev = _source_events(res)
    assert ev and all(e["truck_dr"] is None for e in ev), ev
    assert ev[-1]["car"] == "ok", "an odd flag must never make the publish itself unusable"

  def test_edge_age_is_still_live(self, monkeypatch):
    res = _run(monkeypatch, 8.0, _device_track(8), _straight(8), _flag(lambda i: 1, age=M.CAR_GPS_MAX_AGE_S))
    assert _source_events(res)[-1]["truck_dr"] == 1

  def test_the_feed_going_silent_clears_it(self, monkeypatch):
    truck = _straight(10)                              # publishes stop at 9.3
    res = _run(monkeypatch, 20.0, _device_track(20), truck, _flag(lambda i: 1))
    ev = _source_events(res)
    assert any(e["truck_dr"] == 1 for e in ev)
    assert ev[-1]["car"] == "silent" and ev[-1]["truck_dr"] is None, ev

  def test_an_unreadable_publish_clears_it(self, monkeypatch):
    truck = _straight(12)
    res = _run(monkeypatch, 12.0, _device_track(12), truck,
               lambda i: {"dr": 1, "drAge": 0.3, **({"hdop": "x"} if i >= 8 else {})})
    ev = _source_events(res)
    assert any(e["truck_dr"] == 1 for e in ev)
    assert ev[-1]["car"] == "unreadable" and ev[-1]["truck_dr"] is None, ev


def test_the_class_state_directly():
  """The same rule without the loop, including an absent feed."""
  src = M.CarGpsSource()
  base = _cargps(1.0, 47.6, -122.3, 90.0, 30.0)
  src.update({**base, "dr": 1, "drAge": 0.2}, 1.0, None)
  assert src.truck_dr == 1
  src.update({**base, "ts": base["ts"] + 1, "dr": 0, "drAge": 0.2}, 2.0, None)
  assert src.truck_dr == 0
  src.update(None, 3.0, None)
  assert src.truck_dr is None and src.kind == "absent"


def test_the_seam_real_frames_through_the_real_publisher_into_the_real_source():
  """The key names and the age semantics are a contract between two repos. Real 0x462/0x463/0x464 payloads from the
  rlogs go through opendbc's real Ford CarInterface.update, and every CarGps blob it publishes goes straight into
  CarGpsSource: the cold-start frames must read 1 and the normal-driving frames 0."""
  from opendbc.car import gen_empty_fingerprint
  from opendbc.car.car_helpers import interfaces
  from opendbc.car.ford.tests.test_cargps import TestDeadReckoningFlag as F
  from opendbc.car.ford.values import CAR

  def truck_dr_after(frames):
    CarInterface = interfaces[CAR.FORD_F_150_LIGHTNING_MK1]
    fp = gen_empty_fingerprint()
    fp[0][0x5A] = 8
    CI = CarInterface(CarInterface.get_params(CAR.FORD_F_150_LIGHTNING_MK1, fp, [], False, False, False))
    src, seen = M.CarGpsSource(), []

    class Pub:
      def put_nonblocking(self, key, val):
        assert key == "CarGps"
        src.update(dict(val, ts=len(seen)), float(len(seen)), None)
        seen.append(src.truck_dr)
    CI.CS._cargps_params = Pub()
    F._run(CI, 4, frames)
    return seen

  assert truck_dr_after(F.COLD) == [1, 1, 1, 1]
  assert truck_dr_after(F.NORMAL) == [0, 0, 0, 0]
