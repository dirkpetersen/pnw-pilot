"""mapveto2pnw: the map advisory is dropped only when the road GEOMETRY contradicts it.

_map_geometry_contradicts() is a pure predicate over telemetry the controller already computed this
tick, so these drive it directly rather than through cap(). The "artifact" cases below are the real
measurements from the 2026-08-21 I-5 corridor drive (see vtsc_constants.MAPVETO_*), not invented
numbers; the negative cases pin every one of the four AND-conditions plus the fail-safe behaviour.
"""
import math

from openpilot.selfdrive.controls.lib.vtsc_pnw.vtsc_controller import VTSCController
from openpilot.selfdrive.controls.lib.vtsc_pnw import vtsc_constants as C

MPH = 0.44704


class FakeCP:
  carFingerprint = ""
  brand = ""
  openpilotLongitudinalControl = True


class FakeParams:
  def get(self, k, return_default=False):
    return {"CESMode": "2"}.get(k)

  def get_bool(self, k):
    return False

  def put_nonblocking(self, k, v):
    pass


def _ctrl(k, kv, n, ahead=True):
  """A controller with the polyline-curvature telemetry of one tick already filled in."""
  c = VTSCController(FakeCP(), params=FakeParams())
  c.mem_params = None
  c._tele_mapk = k              # measured curvature (1/m)
  c._tele_mapk_v = kv           # geometry-implied safe speed (m/s)
  c._tele_mapk_n = n            # usable triplets; 0 = UNMEASURABLE
  c._tele_mapk_ahead = ahead
  return c


def _from_radius(radius_m, safe_mph):
  return 1.0 / radius_m, safe_mph * MPH


# --- the measured artifacts: mapd advised a big slowdown on a road that is geometrically straight ---

def test_vetoes_the_877m_radius_artifact():
  # measured: mapd advised 61.1 mph where the polyline radius is 877 m (supports 104.9 mph)
  k, kv = _from_radius(877.0, 104.9)
  c = _ctrl(k, kv, n=5)
  assert c._map_geometry_contradicts(61.1 * MPH) is True


def test_vetoes_the_2083m_radius_artifact():
  # measured: 54.8 mph advised at a 2083 m radius (161.5 mph) -- effectively dead straight
  k, kv = _from_radius(2083.0, 161.5)
  c = _ctrl(k, kv, n=3)
  assert c._map_geometry_contradicts(54.8 * MPH) is True


# --- the four AND-conditions, each pinned separately -----------------------------------------------

def test_no_veto_when_geometry_unmeasurable():
  # THE important one: mapKN == 0 happened on 44% of highway samples. Absence of geometry must never
  # be read as "the road is straight" -- that would silently disable map braking half the time.
  k, kv = _from_radius(2083.0, 161.5)
  c = _ctrl(k, kv, n=0)
  assert c._map_geometry_contradicts(54.8 * MPH) is False


def test_no_veto_below_the_confidence_bar():
  k, kv = _from_radius(2083.0, 161.5)
  c = _ctrl(k, kv, n=C.MAPVETO_MIN_N - 1)
  assert c._map_geometry_contradicts(54.8 * MPH) is False


def test_no_veto_on_a_genuine_curve():
  # 400 m radius is a real curve -- geometry AGREES with mapd, so the slowdown must stand even though
  # the speed ratio alone would qualify.
  k, kv = _from_radius(400.0, 120.0)
  c = _ctrl(k, kv, n=6)
  assert (1.0 / k) < C.MAPVETO_MIN_RADIUS
  assert c._map_geometry_contradicts(40.0 * MPH) is False


def test_no_veto_when_geometry_only_mildly_faster():
  # straight enough by radius, but geometry does not disagree hard enough -> keep the slowdown
  k, kv = _from_radius(1500.0, 70.0)
  mv = 60.0 * MPH
  assert (1.0 / k) > C.MAPVETO_MIN_RADIUS            # radius gate WOULD pass
  assert kv / mv < C.MAPVETO_MIN_RATIO               # ...and the ratio gate is what stops it
  assert _ctrl(k, kv, n=5)._map_geometry_contradicts(mv) is False


def test_no_veto_when_curvature_is_behind_us():
  k, kv = _from_radius(2083.0, 161.5)
  c = _ctrl(k, kv, n=5, ahead=False)
  assert c._map_geometry_contradicts(54.8 * MPH) is False


# --- fail-safe: anything unusable must keep the slowdown -------------------------------------------

def test_kill_switch_restores_previous_behaviour():
  k, kv = _from_radius(2083.0, 161.5)
  c = _ctrl(k, kv, n=5)
  assert c._map_geometry_contradicts(54.8 * MPH) is True     # vetoes while enabled
  prev = C.MAPVETO_ENABLED
  try:
    C.MAPVETO_ENABLED = False
    assert c._map_geometry_contradicts(54.8 * MPH) is False  # ...and not while disabled
  finally:
    C.MAPVETO_ENABLED = prev


def test_no_veto_on_degenerate_inputs():
  k, kv = _from_radius(2083.0, 161.5)
  for bad in (0.0, float("nan"), float("inf")):
    assert _ctrl(bad, kv, n=5)._map_geometry_contradicts(54.8 * MPH) is False, bad
    assert _ctrl(k, bad, n=5)._map_geometry_contradicts(54.8 * MPH) is False, bad
  # a zero/absent map advisory is not something to veto either
  assert _ctrl(k, kv, n=5)._map_geometry_contradicts(0.0) is False
  # None telemetry (never solved this tick) must degrade, not raise
  c = _ctrl(k, kv, n=5)
  c._tele_mapk = None
  assert c._map_geometry_contradicts(54.8 * MPH) is False


def test_predicate_never_raises():
  c = _ctrl(0.001, 50.0, n=5)
  for bad in (None, "x", float("nan")):
    try:
      c._map_geometry_contradicts(bad)
    except Exception as e:
      raise AssertionError(f"raised on mv={bad!r}: {e!r}") from e


def test_math_import_is_used_for_finiteness():
  # guards the isfinite path actually being reachable (a NaN kv must not sail through)
  assert math.isfinite(1.0)
  c = _ctrl(0.0005, float("nan"), n=5)
  assert c._map_geometry_contradicts(25.0) is False
