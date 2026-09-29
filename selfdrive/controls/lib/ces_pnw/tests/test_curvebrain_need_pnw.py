"""curvebrain2b2pnw stage 3 -- the shared curve brain's need layer for the Tesla (ces_pnw/curve_brain.py).

Pinned here, each by a test that fails when the rule is broken (mutation-checked):
  * the row speed is sqrt(A / k) at the TESLA's A, and never assumes more lateral acceleration than the schedule clip
    (target - 0.3) or the steering ceiling allow -- the 09-28 22:35 Terwilliger left curve: 4.0 alone is 70.7 mph, the
    speed that failed; with the limits it is 61.3;
  * the most binding row is chosen through VTSC's own decel envelope;
  * every gate is a `cbWhy`, an exception costs the brain and says so, the payload is always a heartbeat;
  * the CurveBrain param reaches the mem-param from a real CESController on the Tesla and NEVER on the Lightning, the
    kill switch is honoured live, and the cb* fields reach the ces_events line on disk.
"""
from __future__ import annotations

import json
import math

import pytest

from openpilot.selfdrive.controls.lib import drive_helpers as dh
from openpilot.selfdrive.controls.lib import pnw_vehicle as pv
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
from openpilot.selfdrive.controls.lib.ces_pnw import curve_brain as cb
from openpilot.selfdrive.controls.lib.ces_pnw import curvedb_live as cl
from openpilot.selfdrive.controls.lib.ces_pnw.tests.roaddb_fixture import write_db
from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_curvedblive2pnw import (
  LAT0, LON0, STRAIGHT, TESLA, _anchor, _drive, _path)
from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_curvelead2pnw import FakeCP
from openpilot.selfdrive.controls.lib.vtsc_pnw.vtsc_pnw import brake_cap_for_apex

MPH = 0.44704
DEFAULT_MODE = pv.CURVE_BRAIN_DEFAULT      # the Tesla's shipped mode when curve.json says nothing
CEIL = 3.5886 - 0.3          # the Tesla steering ceiling (CarControllerParams MAX_LATERAL_ACCEL - CURVE_STEER_MARGIN)
LIGHTNING = "FORD_F_150_LIGHTNING_MK1"


class Veh:
  """Duck-typed vehicle: only curve_lat_a is read by row_speed / most_binding_row."""
  def __init__(self, f):
    self.f = f

  def curve_lat_a(self, v):
    return self.f(v)


class _Log:
  def __init__(self):
    self.errors, self.exceptions, self.events = [], [], []

  def error(self, msg, *a, **k):
    self.errors.append(msg)

  def exception(self, msg, *a, **k):
    self.exceptions.append(msg)

  def event(self, name, **k):
    self.events.append((name, k))

  def warning(self, *a, **k):
    pass

  def info(self, *a, **k):
    pass

  def debug(self, *a, **k):
    pass


@pytest.fixture
def log(monkeypatch):
  lg = _Log()
  monkeypatch.setattr(cb, "cloudlog", lg)
  monkeypatch.setattr(cl, "cloudlog", lg)
  monkeypatch.setattr(pv, "cloudlog", lg)
  return lg


@pytest.fixture
def schedule(tmp_path, monkeypatch):
  """The shipped lataccel2pnw schedule (6/5/4/3 at 50/60/70/80 mph) in a file of our own."""
  p = tmp_path / "lataccel_limits.json"
  p.write_text(json.dumps({"breakpoints": dh.DEFAULT_LAT_ACCEL_BREAKPOINTS_MPH}))
  monkeypatch.setattr(dh, "LAT_ACCEL_LIMITS_PATH", str(p))
  monkeypatch.setattr(dh, "_lat_accel_schedule", dh._LatAccelSchedule())


@pytest.fixture
def cfgpath(tmp_path, monkeypatch):
  p = tmp_path / "curve.json"
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(p))
  monkeypatch.setattr(pv, "RAIN_CONFIG_PATH", str(tmp_path / "absent-rain.json"))
  return p


def tesla():
  return pv.PnwVehicle(FakeCP(TESLA, "tesla", True))


def lightning():
  return pv.PnwVehicle(FakeCP(LIGHTNING, "ford", False))


# =====================================================================================================
# the row speed: sqrt(A / k) at the Tesla's A, inside every limit
# =====================================================================================================
class TestRowSpeed:
  def test_it_is_sqrt_a_over_k(self):
    v, a = cb.row_speed(Veh(lambda v: 3.0), 0.004, 33.0)
    assert a == 3.0 and v == pytest.approx(math.sqrt(3.0 / 0.004), abs=1e-12)

  def test_it_prices_at_the_lowest_a_over_the_speeds_involved(self):
    """A non-monotonic schedule: 3.0 at speed now (33) and at the first estimate's speed is not the lowest -- the
    speed the first estimate asks for (27.4 m/s) has 2.5. Without the fixed-point loop the answer would be 27.4."""
    f = lambda v: 3.0 if (v > 29.0 or v < 10.0) else 2.5   # noqa: E731
    v, a = cb.row_speed(Veh(f), 0.004, 33.0)
    assert a == 2.5 and v == pytest.approx(25.0, abs=1e-9)

  @pytest.mark.parametrize("bad", [float("nan"), 0.0, -1.0, float("inf") * 0])
  def test_an_unusable_target_is_no_speed_never_a_guess(self, bad):
    assert cb.row_speed(Veh(lambda v: bad), 0.004, 33.0) == (None, None)

  @pytest.mark.parametrize("k", [0.0, -0.001, float("nan")])
  def test_an_unusable_curvature_is_no_speed(self, k):
    assert cb.row_speed(Veh(lambda v: 3.0), k, 33.0) == (None, None)

  def test_a_straight_road_is_capped_to_a_finite_number(self):
    v, _ = cb.row_speed(Veh(lambda v: 3.0), 1e-9, 33.0)
    assert v == cb.V_NEED_MAX

  def test_the_real_tesla_never_assumes_more_than_its_limits_allow(self, cfgpath, schedule):
    """A property over the whole plane: the row speed's lateral acceleration k*v^2 is at most the steering ceiling, at
    most the schedule clip at THAT speed minus 0.3, and at most the configured 4.0 -- at every k and speed now."""
    t = tesla()
    ceiling = pv._tesla_steer_lat_ceiling()
    for k in [0.0004, 0.001, 0.002, 0.004, 0.0057, 0.01, 0.02, 0.06]:
      for v_ego in [8.0, 15.0, 22.0, 27.0, 31.4, 35.0, 40.0, 45.0]:
        v, a = cb.row_speed(t, k, v_ego)
        if v >= cb.V_NEED_MAX:
          continue
        lat = k * v * v
        assert lat <= a + 1e-9 and a <= 4.0 and a <= ceiling + 1e-9, (k, v_ego, a, lat)
        assert a <= dh.lat_accel_target(v) - 0.3 + 1e-9 and a <= dh.lat_accel_target(v_ego) - 0.3 + 1e-9, (k, v_ego)

  def test_terwilliger_left_curve_entry_the_speed_that_failed_is_not_commanded(self, cfgpath, schedule):
    """2026-09-28 22:35, the left curve, DB row k = 0.0040 (R 250 m), the car arriving at 70 mph.
    (a) A = 4.0 alone commands 70.7 mph -- the speed at which the applied angle stalled at the clamp and the driver
        took over. (b) with only the lataccel schedule clip (target - 0.3) it is still 67.8 mph. (c) with the
        steering ceiling it is 64.1 mph."""
    k, v_now = 0.0040, 70.3 * MPH
    va, _ = cb.row_speed(Veh(lambda v: 4.0), k, v_now)
    vb, _ = cb.row_speed(Veh(lambda v: min(4.0, dh.lat_accel_target(v) - 0.3)), k, v_now)
    vc, ac = cb.row_speed(tesla(), k, v_now)
    assert va / MPH == pytest.approx(70.7, abs=0.05)
    assert vb / MPH == pytest.approx(67.8, abs=0.15) and vb < va
    assert vc / MPH == pytest.approx(64.1, abs=0.1) and ac == pytest.approx(CEIL, abs=1e-3)
    assert vc < vb < va


# =====================================================================================================
# the most binding row, through VTSC's own decel envelope
# =====================================================================================================
class _Idx:
  back = 25.0


class _M:
  def __init__(self, k, s_anchor, row="a:0"):
    self.k, self.s_anchor, self.row_id = k, s_anchor, row


class TestMostBinding:
  V = Veh(lambda v: 3.0)

  def test_no_rows_no_need(self):
    assert cb.most_binding_row(_Idx, [], 0.0, self.V, 30.0, 1.2) == (None, 0)

  def test_distance_is_the_anchor_minus_25_and_clamps_at_zero_inside_the_row(self):
    need, n = cb.most_binding_row(_Idx, [_M(0.004, 300.0)], 0.0, self.V, 30.0, 1.2)
    assert n == 1 and need["d"] == 275.0
    need, _ = cb.most_binding_row(_Idx, [_M(0.004, 300.0)], 290.0, self.V, 30.0, 1.2)     # 10 m before the anchor
    assert need["d"] == 0.0
    need, _ = cb.most_binding_row(_Idx, [_M(0.004, 300.0)], 320.0, self.V, 30.0, 1.2)     # 20 m past it
    assert need["d"] == 0.0

  def test_a_sharper_row_far_ahead_can_lose_to_a_milder_one_close_by(self):
    """The choice is by envelope (brake_cap_for_apex), not by lowest v: 500 m of runway makes a slow row far away
    less binding NOW than a somewhat faster row right here."""
    near, far = _M(0.003, 60.0, "near"), _M(0.009, 480.0, "far")
    need, n = cb.most_binding_row(_Idx, [far, near], 0.0, self.V, 33.0, 1.2)
    v_near, v_far = math.sqrt(3.0 / 0.003), math.sqrt(3.0 / 0.009)
    assert v_far < v_near and n == 2
    assert brake_cap_for_apex(v_near, 35.0, 33.0, 1.2) < brake_cap_for_apex(v_far, 455.0, 33.0, 1.2)
    assert need["row"] == "near"

  def test_the_decel_a_row_is_priced_at_is_the_callers(self):
    """A gentler decel needs the row's slowdown earlier -> the same row binds at a LOWER cap."""
    r = [_M(0.004, 200.0)]
    n12, _ = cb.most_binding_row(_Idx, r, 0.0, self.V, 33.0, 1.2)
    n10, _ = cb.most_binding_row(_Idx, r, 0.0, self.V, 33.0, 1.0)
    e12 = brake_cap_for_apex(n12["v"], n12["d"], 33.0, 1.2)
    e10 = brake_cap_for_apex(n10["v"], n10["d"], 33.0, 1.0)
    assert e10 < e12

  def test_a_row_without_a_usable_target_is_skipped_not_guessed(self):
    v = Veh(lambda v: float("nan"))
    assert cb.most_binding_row(_Idx, [_M(0.004, 100.0)], 0.0, v, 30.0, 1.2) == (None, 0)


# =====================================================================================================
# the step: gates, heartbeat, exceptions
# =====================================================================================================
ROW = [_anchor(300.0, 0.004)]                 # a real curve 300 m ahead of the car at y = 0, k = 0.004


def _brain(tmp_path, monkeypatch, anchors=ROW, veh=None):
  d = tmp_path / "db"
  if anchors is not None:
    write_db(str(d), anchors)
  monkeypatch.setattr(cl, "DATA_DIR", str(d))
  return cb.CurveBrain(veh or tesla())


def _step(b, now=100.0, v_ego=33.0, points=None, plat=LAT0, plon=LON0, gps="proj", way="current", hwy="motorway",
          a_decel=1.2):
  return b.step(now, v_ego=v_ego, points=_path() if points is None else points, plat=plat, plon=plon, gps_state=gps,
                way_sel=way, hwy=hwy, a_decel=a_decel)


class TestStep:
  def test_a_row_ahead_is_priced_at_the_teslas_a(self, tmp_path, monkeypatch, cfgpath, schedule, log):
    b = _brain(tmp_path, monkeypatch)
    out = _step(b)
    assert out["v"] == pytest.approx(math.sqrt(CEIL / 0.004), abs=0.01) and out["a"] == pytest.approx(CEIL, abs=0.01)
    assert (out["d"], out["src"], out["ev"], out["row"], out["k"]) == (275.0, "db", "measured", "0:0", 0.004)
    assert out["mode"] == DEFAULT_MODE and out["ts"] == 100.0 and out["seq"] == 1
    t = b.tele(100.0)
    assert t["cbWhy"] == "ok" and t["cbN"] == 1 and t["cbRows"] == 1 and t["cbDb"] == "ok" and t["cbV"] == out["v"]
    assert log.errors == [] and log.exceptions == []

  def test_it_is_a_heartbeat_when_there_is_no_need(self, tmp_path, monkeypatch, cfgpath, schedule, log):
    b = _brain(tmp_path, monkeypatch, anchors=[[0.5, 0.5, 0.0, [[0.5013, 0.5, 0.001, 3]]]])     # a row far away
    o1, o2 = _step(b, 100.0), _step(b, 100.25)
    assert (o1["seq"], o2["seq"]) == (1, 2) and o2["ts"] == 100.25 and o1["v"] is None and o1["mode"] == DEFAULT_MODE
    assert b.tele(100.25)["cbWhy"] == "noRow"

  @pytest.mark.parametrize("kw, why", [
    (dict(gps="stale"), "gpsstale"), (dict(gps="none", plat=None, plon=None), "gpsnone"),
    (dict(way="predicted"), "waySel"), (dict(way=None), "waySel"),
    (dict(hwy="motorwayLink"), "class"), (dict(hwy="unknown"), "class"), (dict(hwy=None), "class"),
    (dict(points=[]), "noPath"), (dict(points=[{"latitude": LAT0, "longitude": LON0}]), "noPath"),
  ])
  def test_every_gate_is_named_and_gives_no_need(self, tmp_path, monkeypatch, cfgpath, schedule, log, kw, why):
    b = _brain(tmp_path, monkeypatch)
    out = _step(b, **kw)
    assert out["v"] is None and out["ts"] == 100.0
    assert b.tele(100.0)["cbWhy"] == why and log.exceptions == []

  def test_a_raw_position_without_a_fix_time_is_accepted_like_the_lightnings(self, tmp_path, monkeypatch, cfgpath, schedule):
    assert _step(_brain(tmp_path, monkeypatch), gps="raw")["v"] is not None

  def test_a_car_off_mapds_path_gets_no_rows(self, tmp_path, monkeypatch, cfgpath, schedule):
    b = _brain(tmp_path, monkeypatch)
    plat, plon = _ll_east(200.0)                       # 200 m east of a due-north path
    assert _step(b, plat=plat, plon=plon)["v"] is None and b.tele(100.0)["cbWhy"] == "offPath"

  def test_a_waysel_flicker_is_ridden_through_for_two_seconds_only(self, tmp_path, monkeypatch, cfgpath, schedule):
    b = _brain(tmp_path, monkeypatch)
    assert _step(b, 100.0)["v"] is not None                       # current at 100.0
    assert _step(b, 101.0, way="predicted")["v"] is not None       # 1 s of flicker: held
    assert _step(b, 102.5, way="predicted")["v"] is None           # sustained: gated
    assert b.tele(102.5)["cbWhy"] == "waySel"

  def test_mode_off_is_no_need_and_says_off(self, tmp_path, monkeypatch, cfgpath, schedule):
    cfgpath.write_text(json.dumps({"tesla": {"curve_brain": "off"}}))
    b = _brain(tmp_path, monkeypatch)
    out = _step(b)
    assert out["mode"] == "off" and out["v"] is None and b.tele(100.0)["cbWhy"] == "off" and b.tele(100.0)["cbOn"] == "off"

  def test_the_db_state_is_a_gate_and_is_loud(self, tmp_path, monkeypatch, cfgpath, schedule, log):
    b = _brain(tmp_path, monkeypatch, anchors=None)               # no file: DB OFF, loudly
    out = _step(b)
    assert out["v"] is None and b.tele(100.0)["cbWhy"] == "dbErr" and b.tele(100.0)["cbErr"]
    assert any("LOAD FAILED" in e for e in log.errors)

  def test_an_exception_costs_the_brain_not_the_caller_and_is_logged(self, tmp_path, monkeypatch, cfgpath, schedule, log):
    b = _brain(tmp_path, monkeypatch)

    def boom(*a, **k):
      raise RuntimeError("on fire")
    monkeypatch.setattr(cb, "scan_ahead", boom)
    out = _step(b)
    assert out["v"] is None and out["seq"] == 1
    t = b.tele(100.0)
    assert t["cbWhy"] == "err" and t["cbErr"] == "RuntimeError" and len(log.exceptions) == 1 and "FAILED" in log.exceptions[0]
    _step(b, 100.25)                                              # throttled: still one line
    assert len(log.exceptions) == 1

  def test_a_stale_tick_reads_idle_never_the_last_need(self, tmp_path, monkeypatch, cfgpath, schedule):
    b = _brain(tmp_path, monkeypatch)
    _step(b, 100.0)
    t = b.tele(100.0 + 5.0)
    assert t["cbWhy"] == "idle" and t["cbV"] is None and t["cbOn"] == DEFAULT_MODE

  def test_the_mode_is_hot_reloaded_into_the_payload(self, tmp_path, monkeypatch, cfgpath, schedule):
    import os
    cfgpath.write_text(json.dumps({"tesla": {"curve_brain": "lower"}}))
    b = _brain(tmp_path, monkeypatch)
    b.veh._tesla_cfg_poll = -1e9                                  # the poll clock is time.monotonic in production
    assert _step(b, 100.0)["mode"] == "lower"
    cfgpath.write_text(json.dumps({"tesla": {"curve_brain": "shadow"}}))
    st = os.stat(cfgpath)
    os.utime(cfgpath, ns=(st.st_atime_ns, st.st_mtime_ns + 5_000_000_000))
    out = _step(b, 102.0)
    assert out["mode"] == "shadow" and out["v"] is not None                  # still computes; only the mode changed

  def test_tele_keys_are_exactly_what_tele_emits(self, tmp_path, monkeypatch, cfgpath, schedule):
    b = _brain(tmp_path, monkeypatch)
    _step(b)
    assert set(b.tele(100.0)) == set(cb.TELE_KEYS) and len(set(cb.TELE_KEYS)) == len(cb.TELE_KEYS)


def _ll_east(dx):
  return LAT0, LON0 + dx / (111320.0 * math.cos(math.radians(LAT0)))


# =====================================================================================================
# the VTSC side of the contract
# =====================================================================================================
class TestParseEntry:
  NOW = 500.0

  def _e(self, **kw):
    e = {"ts": 499.8, "seq": 3, "mode": "lower", "v": 25.0, "d": 200.0, "src": "db", "ev": "measured", "a": 3.0,
         "k": 0.0048, "row": "1:0"}
    e.update(kw)
    return e

  def test_a_fresh_measured_need_parses(self):
    entry, problem, age = cb.parse_entry(self._e(), self.NOW)
    assert problem is None and entry["v"] == 25.0 and entry["d"] == 200.0 and age == pytest.approx(0.2)

  def test_json_text_and_bytes_parse_too(self):
    for raw in (json.dumps(self._e()), json.dumps(self._e()).encode()):
      assert cb.parse_entry(raw, self.NOW)[1] is None

  def test_a_heartbeat_without_a_need_is_the_normal_case_not_a_problem_entry(self):
    entry, problem, _ = cb.parse_entry(self._e(v=None, d=None, ev=None), self.NOW)
    assert problem == "noNeed" and entry["v"] is None and entry["mode"] == "lower"

  @pytest.mark.parametrize("raw, problem", [
    (None, "absent"), ("", "absent"), ({}, "absent"),
    ({"ts": 100.0, "mode": "lower", "v": 25.0, "d": 1.0, "ev": "measured"}, "stale"),               # 400 s old
    ({"ts": 501.0, "mode": "lower", "v": 25.0, "d": 1.0, "ev": "measured"}, "stale"),                # from the future
    ({"ts": 498.9, "mode": "lower", "v": 25.0, "d": 1.0, "ev": "measured"}, "stale"),                # 1.1 s old
    ("not json", "bad"), ([1, 2], "bad"), (42, "bad"),
    ({"ts": "x", "mode": "lower"}, "bad"), ({"ts": True, "mode": "lower"}, "bad"), ({"mode": "lower"}, "bad"),
    ({"ts": 499.9, "mode": "sideways", "v": 25.0, "d": 1.0, "ev": "measured"}, "bad"),
    ({"ts": 499.9, "mode": "lower", "v": float("nan"), "d": 1.0, "ev": "measured"}, "bad"),
    ({"ts": 499.9, "mode": "lower", "v": 0.0, "d": 1.0, "ev": "measured"}, "bad"),
    ({"ts": 499.9, "mode": "lower", "v": -5.0, "d": 1.0, "ev": "measured"}, "bad"),
    ({"ts": 499.9, "mode": "lower", "v": 25.0, "d": -1.0, "ev": "measured"}, "bad"),
    ({"ts": 499.9, "mode": "lower", "v": 25.0, "d": None, "ev": "measured"}, "bad"),
    ({"ts": 499.9, "mode": "lower", "v": True, "d": 1.0, "ev": "measured"}, "bad"),
    # evidence the Tesla may not act on: mapd alone / vision are log-only (design s5.1)
    ({"ts": 499.9, "mode": "lower", "v": 25.0, "d": 1.0, "ev": "rated"}, "bad"),
    ({"ts": 499.9, "mode": "lower", "v": 25.0, "d": 1.0, "ev": "vision"}, "bad"),
    ({"ts": 499.9, "mode": "lower", "v": 25.0, "d": 1.0}, "bad"),
  ])
  def test_anything_else_is_ignored_and_named(self, raw, problem):
    entry, why, _ = cb.parse_entry(raw, self.NOW)
    assert entry is None and why == problem

  def test_it_never_raises(self):
    class Weird:
      def __eq__(self, o):
        raise RuntimeError("eq")
    for raw in (Weird(), {"ts": 10 ** 400, "mode": "lower"}, {"ts": 499.9, "mode": "lower", "v": 10 ** 400, "d": 1,
                                                               "ev": "measured"}):
      try:
        entry, why, _ = cb.parse_entry(raw, self.NOW)
      except RuntimeError:
        pytest.fail("parse_entry raised")
      assert entry is None or why is None


# =====================================================================================================
# through a real CESController
# =====================================================================================================
def _tesla_drive(monkeypatch, tmp_path, anchors, mode=None, points=None, **kw):
  """A Tesla CESController through 300 ticks (v_ego 33 m/s due north, GPS at the origin). `mode` writes curve.json."""
  tmp_path.mkdir(parents=True, exist_ok=True)
  if mode is not None:
    (tmp_path / "curve.json").write_text(json.dumps({"tesla": {"curve_brain": mode}}))
  return _drive(monkeypatch, tmp_path, points=STRAIGHT if points is None else points, anchors=anchors, fp=TESLA,
                brand="tesla", op_long=True, ticks=300, **kw)


def _puts(c):
  return [v for k, v in c.mem_params.puts if k == "CurveBrain"]


class TestController:
  def test_the_tesla_publishes_a_heartbeat_with_the_need_at_about_4_hz(self, monkeypatch, tmp_path, schedule):
    _, recs, c = _tesla_drive(monkeypatch, tmp_path, ROW)
    puts = _puts(c)
    assert 10 <= len(puts) <= 14                                       # 300 ticks at 100 Hz = 3 s -> ~4 Hz
    assert [p["seq"] for p in puts] == list(range(1, len(puts) + 1))
    assert all(p["v"] == pytest.approx(math.sqrt(CEIL / 0.004), abs=0.05) and p["ev"] == "measured" for p in puts)
    assert all(p["mode"] == DEFAULT_MODE for p in puts)
    r = recs[-1]
    assert r["cbOn"] == DEFAULT_MODE and r["cbDb"] == "ok" and r["cbWhy"] == "ok" and r["cbRow"] == "0:0"

  def test_no_row_no_need_but_the_heartbeat_continues(self, monkeypatch, tmp_path, schedule):
    _, recs, c = _tesla_drive(monkeypatch, tmp_path, [[0.5, 0.5, 0.0, [[0.5013, 0.5, 0.001, 3]]]])
    puts = _puts(c)
    assert puts and all(p["v"] is None for p in puts) and recs[-1]["cbWhy"] == "noRow"

  def test_the_lightning_never_publishes_and_reads_off(self, monkeypatch, tmp_path, schedule):
    _, recs, c = _drive(monkeypatch, tmp_path / "l", points=STRAIGHT, anchors=ROW, ticks=300)
    assert c._curve_brain is None and _puts(c) == []
    assert {r["cbOn"] for r in recs} == {"off"} and {r["cbWhy"] for r in recs} == {"noCapability"}

  def test_a_permissive_stub_reads_absent_never_a_missing_column(self):
    class Stub:
      pass
    t = m._curvebrain_tele(Stub())
    assert set(t) == set(cb.TELE_KEYS) and t["cbOn"] == "absent"

  def test_the_kill_switch_off_publishes_no_need(self, monkeypatch, tmp_path, schedule):
    _, recs, c = _tesla_drive(monkeypatch, tmp_path, ROW, mode="off")
    assert _puts(c) and all(p["v"] is None and p["mode"] == "off" for p in _puts(c)) and recs[-1]["cbOn"] == "off"

  def test_the_cb_fields_reach_the_ces_events_line_on_disk(self, monkeypatch, tmp_path, schedule):
    """Not the dict: the JSON line _append_event writes. New fields have evaporated before (cherry-picked keys)."""
    log = tmp_path / "ces_events.jsonl"
    _tesla_drive(monkeypatch, tmp_path / "rec", ROW, log_file=log)
    lines = [json.loads(ln) for ln in log.read_text().splitlines()]
    lines = [r for r in lines if r.get("ev") != "mapdPath"]
    assert lines
    for rec in lines:
      assert set(cb.TELE_KEYS) <= set(rec), set(cb.TELE_KEYS) - set(rec)
    r = [x for x in lines if x["cbWhy"] == "ok"][-1]
    assert r["cbOn"] == DEFAULT_MODE and r["cbA"] == pytest.approx(CEIL, abs=0.01) and r["cbK"] == 0.004 and r["cbSrc"] == "db"
    assert r["cbD"] == pytest.approx(275.0 - 33.0 * 0.3, abs=1.5)         # the 0.3 s old fix, projected to now (keep_s 0)

  def test_a_missing_db_is_a_loud_no_need_and_the_car_still_drives(self, monkeypatch, tmp_path, schedule, log):
    _, recs, c = _tesla_drive(monkeypatch, tmp_path, None)
    assert recs[-1]["cbWhy"] == "dbErr" and recs[-1]["cbErr"]
    assert all(p["v"] is None for p in _puts(c)) and any("LOAD FAILED" in e for e in log.errors)

  def test_a_failing_brain_is_logged_by_the_caller_and_never_raises(self, monkeypatch, tmp_path, schedule, log):
    def boom(self, *a, **k):
      raise RuntimeError("shm gone")
    monkeypatch.setattr(cb.CurveBrain, "step", boom)
    monkeypatch.setattr(m, "cloudlog", log)
    _, recs, c = _tesla_drive(monkeypatch, tmp_path, ROW)
    assert any("publish FAILED" in e and "RuntimeError" in e for e in log.exceptions) and len(log.exceptions) == 1
    assert _puts(c) == [] and recs                                  # no entry (VTSC sees stale), the controller ran

  def test_the_position_is_projected_to_now_not_lagged(self, monkeypatch, tmp_path, schedule):
    """keep_s 0: a 1 s old fix at 33 m/s is projected 33 m ahead, so the curve is 33 m closer than the raw fix says."""
    seen = []
    real = m.icbm_project_position

    def spy(*a, **k):
      seen.append(k.get("keep_s"))
      return real(*a, **k)
    monkeypatch.setattr(m, "icbm_project_position", spy)
    _tesla_drive(monkeypatch, tmp_path, ROW)
    assert seen and set(seen) == {0.0}


# =====================================================================================================
# selfdrived -> mem-param -> plannerd, end to end (the contract as ONE flow, not two halves that agree by construction)
# =====================================================================================================
class TestEndToEnd:
  NEAR = [_anchor(120.0, 0.004)]                # a real curve 120 m ahead: the envelope binds at 33 m/s

  def _vtsc(self, monkeypatch, payload, secs=0.5, v=33.0, v_set=33.5):
    import types

    from openpilot.selfdrive.controls.lib.vtsc_pnw import vtsc_controller as vc
    clock = [payload["ts"] + 0.05]
    monkeypatch.setattr(vc, "time", types.SimpleNamespace(monotonic=lambda: clock[0]))

    class Mem:
      def get(self, k, return_default=False):
        return payload if k == "CurveBrain" else None

      def put_nonblocking(self, k, val):
        pass

    class Prm:
      def get(self, k, return_default=False):
        return "2" if k == "CESMode" else None

      def get_bool(self, k):
        return False

    class NS:
      pass
    mdl = NS()
    mdl.orientationRate, mdl.velocity, mdl.position, mdl.action = NS(), NS(), NS(), NS()
    mdl.orientationRate.z, mdl.orientationRate.t = [0.0] * 20, [i * 0.25 for i in range(20)]
    mdl.velocity.x, mdl.position.x = [v] * 20, [v * i * 0.25 for i in range(20)]
    mdl.action.shouldStop = False
    cc = NS()
    cc.orientationNED = [0.0, 0.0, 0.0]
    c = vc.VTSCController(FakeCP(TESLA, "tesla", True), params=Prm())
    c.mem_params = Mem()
    caps = []
    for i in range(int(secs * 20)):
      clock[0] = payload["ts"] + 0.05 + i / 20.0
      caps.append(c.cap({"modelV2": mdl, "carControl": cc}, v_set, v))
    return caps, c.overlay_payload()

  def test_the_published_need_lowers_the_tesla_vtsc_cap(self, monkeypatch, tmp_path, schedule):
    _, recs, c = _tesla_drive(monkeypatch, tmp_path, self.NEAR, mode="lower")
    payload = _puts(c)[-1]
    assert payload["v"] == pytest.approx(math.sqrt(CEIL / 0.004), abs=0.05) and 40.0 < payload["d"] < 100.0
    caps, p = self._vtsc(monkeypatch, payload)
    want = brake_cap_for_apex(payload["v"], payload["d"], 33.0, 1.2)
    assert p["cbUse"] == "lower" and p["cbWouldV"] < 33.4 and p["cbAge"] == pytest.approx(0.05, abs=0.06 + 0.5)
    assert caps[-1] < 33.5 - 0.5 and caps[-1] >= want - 1e-6 and caps == sorted(caps, reverse=True)

  def test_the_same_flow_in_shadow_changes_nothing(self, monkeypatch, tmp_path, schedule):
    _, recs, c = _tesla_drive(monkeypatch, tmp_path, self.NEAR, mode="shadow")
    payload = _puts(c)[-1]
    assert payload["mode"] == "shadow" and payload["v"] is not None
    caps, p = self._vtsc(monkeypatch, payload)
    assert set(caps) == {33.5} and p["cbUse"] == "shadow"          # acting needs BOTH processes to agree

  def test_a_dead_brain_is_ignored_after_a_second(self, monkeypatch, tmp_path, schedule):
    _, _, c = _tesla_drive(monkeypatch, tmp_path, self.NEAR, mode="lower")
    payload = _puts(c)[-1]
    caps, p = self._vtsc(monkeypatch, payload, secs=2.5)
    assert p["cbUse"] == "stale" and p["cbStaleN"] > 0 and caps[-1] == 33.5    # no heartbeat -> VTSC as before
