"""Behavioural pins for telemetry fields on the REAL ces_events record.

Why this file exists: three separate features shipped telemetry that reached nothing, or that no test
could tell had stopped reaching anything.
  * waysel2pnw (2026-09-03) published 8 fields to the wrong channel -- all read `null` on the car.
  * lcramp2pnw's `spdA` was published to LaneCenterStatus but never cherry-picked into ces_pnw, so
    the authority ramp was invisible in ces_events (Fable A1).
  * curvefloor2pnw claimed a "telemetry dead" mutation was killed; deleting all three emit lines in
    fact survived the entire 375-test suite (Fable F4). Its replacement was a STRUCTURAL grep test,
    honest but weaker: it cannot catch an emit site that survives while carrying a wrong value.

Fable sketched the fix and it turns out to be cheap: a permissive stub whose `__getattr__` returns
None drives the real `_event_record`, so the actual record dict can be asserted on. That closes the
gap the grep test left, and pins `lcSpdA` and `car_gps`, which nothing pinned at all
(mutations M9/M9b/M9c and M10 all survived).

Deliberately NOT a substitute for the isolation tests -- this asserts the LAST mile (does the value
reach the record), not the maths that produced it.
"""
import inspect

import pytest

from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw_constants as C
from openpilot.selfdrive.controls.lib.ces_pnw.park_tick_gate import ParkTickGate


def _record(**over):
  """Drive the real _event_record("tick", ...) with a permissive stub.

  `__getattr__` returning None means any field this record touches that we did not set reads as
  None -- so the test does not have to track every unrelated attribute the record grows over time,
  which is exactly the maintenance trap that made the per-attribute ICBM stubs so brittle.
  """
  cls = next(o for o in vars(m).values() if inspect.isclass(o) and hasattr(o, "_event_record"))

  class Stub:
    def __getattr__(self, n):
      return None

  g = Stub()
  g._vtsc_tele = {}
  g._sa_tele = {}
  g._map_targets = []
  g._speed_limit = 11.2
  g._button = C.BTN_CES
  g._ces2_urg = 0.0
  g._ces2_div = type("D", (), {"count": 0})()
  g._gl = type("G", (), {"state": None, "status": lambda s: None})()
  g._sm = type("S", (), {"state": None, "status": lambda s: None})()
  # the fields under test, overridable per-case
  g._icbm_floor_lim = 11.2
  g._icbm_floor_hit = True
  # icbmcurv2pnw: the record float()s these, so the permissive `__getattr__ -> None` stub cannot
  # supply them. Defaults chosen as a MEASURED gentle bend (not zeros) so a test that forgets to
  # override still distinguishes "reached the record" from "read a default".
  # icbmconsist2pnw: the record float()s the point-matched trio too, same reason as above.
  g._icbm_k_at = 0.0021
  g._icbm_k_at_d = 210.0
  g._icbm_k_at_n = 4
  g._icbm_k_at_gap = 30.0
  g._icbm_k = 0.004
  g._icbm_k_dist = 180.0
  g._icbm_k_v = 25.0
  g._icbm_k_n = 6
  g._icbm_k_ahead = True
  g._lc_spd_a = 0.55
  g._car_gps = {"lat": 47.672952, "lon": -122.365067}
  # parkgate2pnw: the record now reads the gear + the park gate's verdict. A REAL ParkTickGate (not
  # a None-returning stub attribute): `park` must be a genuine bool on every record, so a stub that
  # silently produced None here would hide exactly the field this feature exists to add.
  g._gear_name = "drive"
  g._park_gate = ParkTickGate()
  for k, v in over.items():
    setattr(g, k, v)
  return cls._event_record.__get__(g)("tick", {"vEgo": 11.0})


class TestCurveFloorTelemetry:
  """curvefloor2pnw. Kills the M8d/M8e class: an emit site that survives but carries a constant."""

  def test_floor_values_reach_the_record(self):
    rec = _record()
    assert rec["icbmFlr"] == pytest.approx(11.2, abs=0.05)
    assert rec["icbmFlrHit"] is True

  def test_the_values_are_not_hardcoded(self):
    """A constant would pass the test above; it must track the controller's actual state."""
    rec = _record(_icbm_floor_lim=8.9, _icbm_floor_hit=False)
    assert rec["icbmFlr"] == pytest.approx(8.9, abs=0.05)
    assert rec["icbmFlrHit"] is False

  def test_inactive_floor_reads_zero_not_stale(self):
    rec = _record(_icbm_floor_lim=0.0, _icbm_floor_hit=False)
    assert rec["icbmFlr"] == 0.0 and rec["icbmFlrHit"] is False


class TestLaneCentringRampTelemetry:
  """lcramp2pnw. Fable A1 / mutations M9, M9b, M9c -- ALL of which survived before this."""

  def test_lcSpdA_reaches_the_record(self):
    assert _record()["lcSpdA"] == pytest.approx(0.55)

  def test_lcSpdA_tracks_the_controller(self):
    assert _record(_lc_spd_a=1.0)["lcSpdA"] == pytest.approx(1.0)   # full authority, >= 15 m/s
    assert _record(_lc_spd_a=0.0)["lcSpdA"] == pytest.approx(0.0)   # suppressed, <= 9 m/s

  def test_lcSpdA_is_None_when_lane_centering_is_idle(self):
    """None must survive as None -- coercing it to 0.0 would read as 'ramp fully suppressed',
    which is a different and misleading claim on a drive log."""
    assert _record(_lc_spd_a=None)["lcSpdA"] is None


class TestCarGpsTelemetry:
  """cargps2pnw. Nothing asserted car_gps reached a record; mutation M10 survived."""

  def test_car_gps_reaches_the_record(self):
    rec = _record()
    assert isinstance(rec["car_gps"], dict)
    assert rec["car_gps"]["lat"] == pytest.approx(47.672952)

  def test_car_gps_is_None_on_a_tesla(self):
    """THE requested behaviour: empty on the Tesla. No Ford carstate means nothing publishes
    CarGps, so the field must log as None rather than {} or a zero coordinate."""
    assert _record(_car_gps=None)["car_gps"] is None


class TestIcbmCurvatureTelemetry:
  """icbmcurv2pnw. The map polyline's own geometry, on the ICBM (stock-ACC) path.

  Why this is pinned at the record level and not only in _read_map: the 2026-09-05/06 phantom
  slowdowns were invisible precisely because a measurement that DID exist (VTSC's mapK) never
  reached the ICBM path's records. A measurement that is computed and then dropped on the way to
  ces_events is worth exactly nothing, and that has now happened three times in this file's history.
  """

  def test_curvature_fields_reach_the_record(self):
    rec = _record()
    assert rec["icbmK"] == pytest.approx(0.004)
    assert rec["icbmKD"] == pytest.approx(180.0)
    assert rec["icbmKV"] == pytest.approx(25.0)
    assert rec["icbmKN"] == 6
    assert rec["icbmKAhead"] is True

  def test_the_values_are_not_hardcoded(self):
    """A constant emit would pass the test above; the fields must track the controller's state."""
    rec = _record(_icbm_k=0.0125, _icbm_k_dist=64.0, _icbm_k_v=14.1, _icbm_k_n=2, _icbm_k_ahead=False)
    assert rec["icbmK"] == pytest.approx(0.0125)
    assert rec["icbmKD"] == pytest.approx(64.0)
    assert rec["icbmKV"] == pytest.approx(14.1)
    assert rec["icbmKN"] == 2
    assert rec["icbmKAhead"] is False

  def test_unmeasurable_is_distinguishable_from_straight(self):
    """THE point of the whole change. Both rows carry icbmK == 0.0; only icbmKN separates them, and
    a clamp built on `icbmK ~= 0` without reading icbmKN would fire on every unmeasurable tick --
    i.e. on every Lightning tick today -- and suppress real curves."""
    unmeasurable = _record(_icbm_k=0.0, _icbm_k_v=0.0, _icbm_k_n=0)
    straight = _record(_icbm_k=0.0, _icbm_k_v=0.0, _icbm_k_n=9)
    assert unmeasurable["icbmK"] == straight["icbmK"] == 0.0
    assert unmeasurable["icbmKN"] == 0
    assert straight["icbmKN"] == 9

  def test_icbmK_resolution_survives_rounding(self):
    """A 5 km-radius sweeper is k = 2e-4. Rounding that to the 5 decimals VTSC's mapK uses keeps it,
    but the next decimal down would not -- and the whole point is telling a small real curvature
    apart from a true zero, so the field is emitted at 6."""
    assert _record(_icbm_k=0.000123)["icbmK"] == pytest.approx(0.000123)
