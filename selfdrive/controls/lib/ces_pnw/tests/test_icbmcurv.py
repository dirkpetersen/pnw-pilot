"""icbmcurv2pnw: the map-polyline curvature measurement on the ICBM (stock-ACC) path.

WHY THIS EXISTS. On 2026-09-05 19:48 and 2026-09-06 13:15 ICBM commanded a large unrequested
slowdown on straight I-5 (icbmT 17.8 / 18.2 m/s against a correct 31.3 m/s posted limit, icbmSrc
"map", no curve anywhere in the telemetry). mapd asserted a curve-target VELOCITY where there is no
curve and ICBM faithfully executed it. The signal that would have contradicted it -- curvature
measured from the lat/lon POLYLINE in the same mapd message -- does exist, but
vtsc_controller.py only runs it with CESMode>0 AND op-long AND VtscMapCurves=1, and ICBM exists
precisely because this truck runs STOCK ACC (op-long False). Live proof, from a tick pulled off
the car mid-drive 2026-09-06: mapPts=8 / mapReach=414.0 (mapd delivered a 414 m, 8-point polyline)
with mapK=0.0 / mapKN=0 -- zero of it measured -- on a tick that also carried mapV=49.5 (110 mph).

These tests pin the measurement itself (test_ces_record_fields.py pins the last mile into
ces_events). The load-bearing property throughout is that "no measurement" is DISTINGUISHABLE from
"measured straight": both report icbmK == 0.0 and only icbmKN separates them. A clamp built on
`icbmK ~= 0` alone would fire on every Lightning tick today and suppress every real curve, which is
the single most dangerous thing that could be built on top of this change.

Nothing here is a control path: no test asserts a target, cap or tap, because nothing reads these.
"""
import inspect
import math
from collections import deque

import pytest

from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw_constants as C
from openpilot.selfdrive.controls.lib.ces_pnw import park_tick_gate
from openpilot.selfdrive.controls.lib.vtsc_pnw.vtsc_constants import A_LAT_TARGET, MAP_SOURCE_HORIZON_M

_LAT0, _LON0 = 47.60, -122.30
_M_PER_DEG = 111320.0
_FIX = {"latitude": _LAT0, "longitude": _LON0, "bearing": 0.0}


def _pt(east_m, north_m, v=30.0):
  """A mapd MapTargetVelocities point at a local metric offset from the car."""
  return {"latitude": _LAT0 + north_m / _M_PER_DEG,
          "longitude": _LON0 + east_m / (_M_PER_DEG * math.cos(math.radians(_LAT0))),
          "velocity": v}


def _straight(n=7, step=100.0, start=0.0):
  return [_pt(0.0, start + i * step) for i in range(n)]


def _arc(radius, n=7, step=100.0, start=0.0):
  """`n` points on a circle of `radius` m, `step` m apart along the arc, curving right (east) from
  a northward heading. The Menger curvature of any three of them is exactly 1/radius."""
  dth = step / radius
  return [_pt(radius * (1.0 - math.cos(i * dth)), start + radius * math.sin(i * dth)) for i in range(n)]


class _Mem:
  """Minimal stand-in for the /dev/shm mem-param store. Every key _read_map() reads other than the
  two set here returns None, which its own defensive handlers already treat as 'no data'."""

  def __init__(self, targets, pos):
    self._d = {"MapTargetVelocities": targets, "LastGPSPosition": pos}

  def get(self, key, return_default=False):
    return self._d.get(key)


def _controller(targets, pos=None, curves=True, mem=True):
  cls = next(o for o in vars(m).values() if inspect.isclass(o) and hasattr(o, "_read_map"))

  class Stub:
    def __getattr__(self, n):
      return None

  g = Stub()
  g.mem_params = _Mem(targets, _FIX if pos is None else pos) if mem else None
  g._toggles = {"curves": curves}
  g._vtsc_tele = {}
  g._bearing_hist = deque(maxlen=8)
  cls._read_map.__get__(g)()
  return g, cls


def _refresh(g, cls, targets, pos=None, curves=True, mem=True):
  """A SECOND _read_map() on the same controller -- how the real thing runs, ~1 Hz forever."""
  g.mem_params = _Mem(targets, _FIX if pos is None else pos) if mem else None
  g._toggles = {"curves": curves}
  cls._read_map.__get__(g)()
  return g


class TestMeasurementRunsOnTheIcbmPath:
  """The whole defect was that this measurement was structurally absent here."""

  def test_a_real_curve_is_measured(self):
    g, _ = _controller(_arc(200.0))
    assert g._icbm_k == pytest.approx(1.0 / 200.0, rel=0.02)
    assert g._icbm_k_ahead is True
    # the COUNT, not just "> 0": 7 points -> 5 interior triplets, all with 100 m legs. A count
    # collapsed to a 0/1 flag would still satisfy "> 0" while destroying the N_MIN >= 2 confidence
    # threshold any later consistency check has to lean on.
    assert g._icbm_k_n == 5
    # ...and the DISTANCE, which nothing else asserts is non-zero after a real measurement.
    assert g._icbm_k_dist == pytest.approx(272.0, abs=15.0)

  def test_a_curve_behind_the_car_is_flagged_not_ahead(self):
    """mapd publishes the whole current way, INCLUDING nodes behind the car. A curve just exited
    must not read as one to slow for. Nothing else in this file ever produces ahead=False, so
    without this both `bool(kahead)` and the cur_bearing argument could be hardcoded unnoticed."""
    g, _ = _controller(_arc(200.0, start=-500.0))
    assert g._icbm_k == pytest.approx(1.0 / 200.0, rel=0.02)
    assert g._icbm_k_n > 0
    assert g._icbm_k_ahead is False

  def test_v_safe_uses_the_vtsc_lateral_accel_target(self):
    """Pins WHICH a_lat reached polyline_curvature. A wrong constant still produces a plausible
    number, so only the sqrt(a_lat/k) identity catches it -- and icbmKV is the field a later
    consistency check would compare against mapd's claimed velocity."""
    g, _ = _controller(_arc(200.0))
    assert g._icbm_k_v == pytest.approx(math.sqrt(A_LAT_TARGET / g._icbm_k), rel=0.01)
    assert A_LAT_TARGET == 2.5   # the value VTSC's own mapKV is computed with, so the two compare

  def test_curvature_beyond_the_mapd_horizon_is_not_measured(self):
    """Pins the horizon argument. The straight run alone spans the horizon, so a curve hung off the
    far end must be invisible -- otherwise ICBM telemetry would 'see' geometry mapd's own 500 m path
    cap says is not on the published route."""
    far = _straight(n=7) + _arc(150.0, n=6, start=MAP_SOURCE_HORIZON_M + 20.0)
    g, _ = _controller(far)
    assert g._icbm_k_n > 0        # the straight part WAS measurable
    assert g._icbm_k == 0.0       # ...and the out-of-horizon curve did not leak in
    near = _controller(_straight(n=2) + _arc(150.0, n=6, start=200.0))[0]
    assert near._icbm_k == pytest.approx(1.0 / 150.0, rel=0.05)
    assert 200.0 <= near._icbm_k_dist <= MAP_SOURCE_HORIZON_M


class TestUnmeasurableIsNotStraight:
  """THE distinction the change exists to create. Every case below reports icbmK == 0.0."""

  def test_straight_road_is_measured_as_straight(self):
    g, _ = _controller(_straight())
    assert g._icbm_k == 0.0
    assert g._icbm_k_n > 0, "a straight road must still COUNT as a measurement"
    # polyline_curvature returns v_safe = inf here; it is emitted as 0.0 because a bare `Infinity`
    # is not valid JSON and would lose the whole tick. READ THAT AS "NO FINITE BOUND", NOT "0 m/s":
    # a consumer that compares icbmKV against a speed will find 0.0 fails every `>=` test, which is
    # exactly backwards on the straightest road there is. Curvature space (icbmK) has no such
    # singularity -- see docs/pnw/ICBMCURV2PNW.md section 5.
    assert g._icbm_k_v == 0.0

  def test_nodes_too_close_together_are_unmeasurable(self):
    """OSM node spacing below the jitter floor: a real curve here would read 0.0 as well, so this
    MUST NOT be reported as a measurement."""
    g, _ = _controller(_arc(200.0, n=12, step=10.0))
    assert g._icbm_k == 0.0 and g._icbm_k_n == 0

  def test_no_gps_fix_is_unmeasurable(self):
    g, _ = _controller(_arc(200.0), pos={"nope": 1})
    assert g._icbm_k == 0.0 and g._icbm_k_n == 0

  def test_no_map_points_is_unmeasurable(self):
    g, _ = _controller([])
    assert g._icbm_k == 0.0 and g._icbm_k_n == 0

  def test_curves_toggle_off_is_unmeasurable(self):
    """_read_map() clears _map_targets when the curve condition is off; the measurement must follow
    it rather than keep reporting on a polyline the controller no longer holds."""
    g, _ = _controller(_arc(200.0), curves=False)
    assert g._icbm_k == 0.0 and g._icbm_k_n == 0

  def test_a_coarse_corner_among_dense_nodes_is_a_MEASURED_ZERO(self):
    """The trap one level up, and the reason the proposed step-2 veto needs a coverage guard.

    Mixed node spacing: a dense straight, a real 90-degree corner drawn with two 350 m legs, then a
    dense straight. The straight triplets pass the spacing gate and the corner's do not, so this
    reports icbmKN > 0 (a measurement exists!) with icbmK == 0.0 (the road is straight!) while an
    actual 90-degree corner sits in the horizon. Any rule that reads "measured AND near-zero
    curvature" as "mapd is lying" would veto a genuine slowdown here."""
    pts = _straight(n=4, step=50.0)
    pts.append(_pt(0.0, 150.0 + 350.0))
    pts.append(_pt(350.0, 500.0))
    pts += [_pt(350.0 + 50.0 * i, 500.0) for i in range(1, 4)]
    g, _ = _controller(pts)
    assert g._icbm_k_n >= 2, "the straight legs are measurable, so this is not a 'no data' row"
    assert g._icbm_k == 0.0, "and the corner itself is invisible to the spacing gate"

  def test_the_two_are_only_separable_by_n(self):
    straight, _ = _controller(_straight())
    blind, _ = _controller([])
    assert straight._icbm_k == blind._icbm_k == 0.0
    assert straight._icbm_k_v == blind._icbm_k_v == 0.0
    assert straight._icbm_k_n > 0 and blind._icbm_k_n == 0


class TestNoStaleReadings:
  """A measurement that silently survives its own inputs is worse than none: it reads exactly like
  a live one. Every _read_map() exit path must have cleared it first."""

  def test_a_curve_does_not_survive_into_a_blind_refresh(self):
    g, cls = _controller(_arc(200.0))
    assert g._icbm_k > 0.0 and g._icbm_k_n > 0
    _refresh(g, cls, [])
    assert g._icbm_k == 0.0 and g._icbm_k_n == 0 and g._icbm_k_v == 0.0
    assert g._icbm_k_dist == 0.0 and g._icbm_k_ahead is True

  def test_a_curve_does_not_survive_losing_the_gps_fix(self):
    g, cls = _controller(_arc(200.0))
    _refresh(g, cls, _arc(200.0), pos={"nope": 1})
    assert g._icbm_k == 0.0 and g._icbm_k_n == 0

  def test_a_curve_does_not_survive_the_mem_params_early_return(self):
    """The early return above the GPS block is why the reset sits at the TOP of _read_map()."""
    g, cls = _controller(_arc(200.0))
    _refresh(g, cls, _arc(200.0), mem=False)
    assert g._map_targets == []
    assert g._icbm_k == 0.0 and g._icbm_k_n == 0

  def test_ahead_false_does_not_survive_a_blind_refresh(self):
    """_icbm_k_ahead is the one field whose reset value (True) is also its common measured value, so
    only a FALSE reading followed by a blind refresh can prove the top-of-function reset runs."""
    g, cls = _controller(_arc(200.0, start=-500.0))
    assert g._icbm_k_ahead is False
    _refresh(g, cls, [])
    assert g._icbm_k_ahead is True

  def test_a_tighter_curve_replaces_a_looser_one(self):
    g, cls = _controller(_arc(400.0))
    assert g._icbm_k == pytest.approx(1.0 / 400.0, rel=0.02)
    _refresh(g, cls, _arc(120.0))
    assert g._icbm_k == pytest.approx(1.0 / 120.0, rel=0.02)


class TestJsonSafety:
  """ces_events is one JSON object per line: a NaN or an inf anywhere loses the WHOLE tick, not just
  the field. polyline_curvature returns inf for v_safe on a straight road, so that path is real."""

  @pytest.mark.parametrize("targets", [[], _straight(), _arc(200.0), _arc(200.0, n=12, step=10.0),
                                       [{"latitude": float("nan"), "longitude": 0.0, "velocity": 1.0}],
                                       [{"bogus": 1}, {"latitude": "x", "longitude": None}]])
  def test_every_emitted_field_is_finite(self, targets):
    g, _ = _controller(targets)
    for name in ("_icbm_k", "_icbm_k_dist", "_icbm_k_v"):
      val = getattr(g, name)
      assert isinstance(val, float) and math.isfinite(val), f"{name} = {val!r}"
    assert isinstance(g._icbm_k_n, int)
    assert isinstance(g._icbm_k_ahead, bool)

  def test_malformed_points_do_not_raise_into_the_control_loop(self):
    """_read_map() runs inside selfdrived. mapd's polyline is untrusted input."""
    g, _ = _controller([{"latitude": None}, None, 7, {"latitude": 1e9, "longitude": -1e9}])
    assert g._icbm_k == 0.0 and g._icbm_k_n == 0


class TestOverlayFeed:
  """CESStatus is the 5 Hz live channel: `cat /dev/shm/params/d/CESStatus` on the car answers "is
  the measurement running right now" without waiting for a drive and a log pull. That makes it the
  deploy check for this change, so it gets pinned like any other load-bearing emit -- curvefloor2pnw
  shipped three telemetry emit lines whose deletion survived the entire suite."""

  @staticmethod
  def _status(**over):
    cls = next(o for o in vars(m).values() if inspect.isclass(o) and hasattr(o, "_publish_status"))

    class Stub:
      def __getattr__(self, n):
        return None

    published = {}

    class Mem:
      def get(self, key, return_default=False):
        return None

      def put_nonblocking(self, key, val):
        published[key] = val

    g = Stub()
    g.mem_params = Mem()
    g._tele_last = 0.0
    g._tick_last = 0.0
    g._shadow = True                     # the ICBM path; the icbm* overlay keys live under it
    g._vtsc_tele = {}
    g._sa_tele = {}
    g._map_targets = []
    g._speed_limit = 11.2
    g._button = C.BTN_CES
    g._ces2_urg = 0.0
    g._ces2_div = type("D", (), {"count": 0})()
    g._gl = type("G", (), {"state": None, "status": lambda s: None})()
    g._sm = type("S", (), {"state": None, "status": lambda s: None})()
    g._icbm_floor_lim = 0.0
    g._icbm_floor_hit = False
    g._append_event = lambda rec: None
    # bound explicitly: the permissive `__getattr__ -> None` would otherwise shadow the real method
    g._event_record = cls._event_record.__get__(g)
    g._icbm_k, g._icbm_k_dist, g._icbm_k_v, g._icbm_k_n, g._icbm_k_ahead = 0.004, 180.0, 25.0, 6, True
    # icbmconsist2pnw: the POINT-MATCHED reading rides the same feeds. The permissive
    # `__getattr__ -> None` would otherwise reach float(None) and take the whole publish down.
    g._icbm_k_at, g._icbm_k_at_d, g._icbm_k_at_n, g._icbm_k_at_gap = 0.0021, 210.0, 4, 30.0
    # parkgate2pnw: _publish_status's tick branch now consults the park gate (a REAL one -- the
    # permissive `__getattr__ -> None` would raise inside the publish and hide every overlay key).
    g._park_gate = park_tick_gate.ParkTickGate()
    g._park_gate_on = True
    g._gear = g._gear_name = g._v_ego_raw = None      # no carState here -> fails open, logs as before
    for k, v in over.items():
      setattr(g, k, v)
    cls._publish_status.__get__(g)(None, False)
    assert "CESStatus" in published, "nothing was published to the overlay at all"
    return published["CESStatus"]

  def test_curvature_reaches_the_overlay(self):
    st = self._status()
    assert st["icbmK"] == pytest.approx(0.004)
    assert st["icbmKD"] == pytest.approx(180.0)
    assert st["icbmKV"] == pytest.approx(25.0)
    assert st["icbmKN"] == 6
    assert st["icbmKAhead"] is True

  def test_the_point_matched_reading_reaches_the_overlay(self):
    """icbmconsist2pnw. Its whole purpose is to be READ from drive logs, so a field that silently
    fails to reach the feed is the same as not having built it -- [[vtscstatus-telemetry-not-logged]].
    icbmKAtGap is the load-bearing one: it says whether comparing mapd's claim to the polyline is
    legitimate on that tick at all."""
    st = self._status()
    assert st["icbmKAt"] == pytest.approx(0.0021)
    assert st["icbmKAtD"] == pytest.approx(210.0)
    assert st["icbmKAtN"] == 4
    assert st["icbmKAtGap"] == pytest.approx(30.0)

  def test_the_point_matched_reading_tracks_the_controller(self):
    """A constant or a coarse round on any of them would otherwise sail through."""
    st = self._status(_icbm_k_at=0.000456, _icbm_k_at_d=77.0, _icbm_k_at_n=9, _icbm_k_at_gap=143.0)
    assert st["icbmKAt"] == pytest.approx(0.000456)
    assert st["icbmKAtD"] == pytest.approx(77.0)
    assert st["icbmKAtN"] == 9
    assert st["icbmKAtGap"] == pytest.approx(143.0)

  def test_every_overlay_value_tracks_the_controller(self):
    """All five, at a resolution a coarser rounding would destroy -- a constant or a 3-decimal
    round on ANY of them would otherwise sail through."""
    st = self._status(_icbm_k=0.000123, _icbm_k_dist=64.0, _icbm_k_v=14.1, _icbm_k_n=2,
                      _icbm_k_ahead=False)
    assert st["icbmK"] == pytest.approx(0.000123)
    assert st["icbmKD"] == pytest.approx(64.0)
    assert st["icbmKV"] == pytest.approx(14.1)
    assert st["icbmKN"] == 2
    assert st["icbmKAhead"] is False

  def test_an_unmeasured_tick_reaches_the_overlay_as_such(self):
    st = self._status(_icbm_k=0.0, _icbm_k_dist=0.0, _icbm_k_v=0.0, _icbm_k_n=0, _icbm_k_ahead=True)
    assert st["icbmK"] == 0.0 and st["icbmKN"] == 0
