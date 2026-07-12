"""icbm2pnw brain tests — pure icbm_curve_target math (curve cap + ceiling latch + restore).

curveslow-lightning: icbm_curve_target now returns a 3-tuple (target, ceiling, src) and considers a
VISION candidate in addition to the map one; the map-only calls below are byte-equivalent to before.
"""
import math

from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import (icbm_curve_target, icbm_vision_apex,
                                                              ICBM_A_DECEL, ICBM_MARGIN_M,
                                                              ICBM_MIN_DROP_MS, ICBM_VISION_ENTER)
from openpilot.selfdrive.controls.lib.vtsc_pnw.vtsc_constants import A_LAT_TARGET as VTSC_A_LAT

MPH = 0.44704
FLAT = 1.0  # identity scale for readable numbers


def test_no_curve_no_target():
  assert icbm_curve_target(25 * MPH, 60 * MPH, 0.0, float('inf'), None, lambda x: FLAT) == (None, None, None)


def test_curve_engages_inside_decel_envelope():
  v, vset, apex = 60 * MPH, 60 * MPH, 40 * MPH
  brake_dist = (v * v - apex * apex) / (2 * ICBM_A_DECEL) + ICBM_MARGIN_M
  # outside the envelope: not yet
  t, c, s = icbm_curve_target(v, vset, apex, brake_dist + 50, None, lambda x: 1.0)
  assert t is None and c is None and s is None
  # inside: engage, latch the driver's set as ceiling, source = map
  t, c, s = icbm_curve_target(v, vset, apex, brake_dist - 10, None, lambda x: 1.0)
  assert math.isclose(t, apex) and math.isclose(c, vset) and s == "map"


def test_ceiling_latch_survives_lowered_set():
  # after taps walked the stock set down to 42, v_set follows it — ceiling must stay 60
  apex = 40 * MPH
  t, c, _ = icbm_curve_target(38 * MPH, 42 * MPH, apex, 20.0, 60 * MPH, lambda x: 1.0)
  assert math.isclose(t, apex) and math.isclose(c, 60 * MPH)


def test_curve_cleared_goes_silent_and_unlatches_immediately():
  # DEC-ONLY design: no restore path — when the curve clears, silence + unlatch, driver restores
  t, c, s = icbm_curve_target(45 * MPH, 42 * MPH, 0.0, float('inf'), 60 * MPH, lambda x: 1.0)
  assert t is None and c is None and s is None


def test_small_drops_ignored():
  # apex within ICBM_MIN_DROP_MS of set: not worth button taps
  vset = 60 * MPH
  apex = vset - ICBM_MIN_DROP_MS / 2
  t, c, _ = icbm_curve_target(vset, vset, apex, 10.0, None, lambda x: 1.0)
  assert t is None and c is None


def test_reduce_only_scaled_apex_above_set_ignored():
  # scale lifts the apex above the driver's set -> no cap (reduce-only)
  t, c, _ = icbm_curve_target(60 * MPH, 60 * MPH, 55 * MPH, 10.0, None, lambda x: 1.5)
  assert t is None and c is None


def test_invalid_set_speed_hands_off():
  assert icbm_curve_target(60 * MPH, 0.0, 40 * MPH, 10.0, None, lambda x: 1.0) == (None, None, None)
  # even while latched, a dropped set speed (ACC off) goes silent immediately
  assert icbm_curve_target(60 * MPH, 0.0, 0.0, float('inf'), 60 * MPH, lambda x: 1.0) == (None, None, None)


def test_uses_ceiling_as_reference_while_capped():
  # while capped (set already lowered), a still-binding curve keeps the cap even though the
  # lowered v_set is close to the apex (reference must be the LATCHED ceiling, not v_set)
  apex = 40 * MPH
  t, c, _ = icbm_curve_target(41 * MPH, 41 * MPH, apex, 15.0, 60 * MPH, lambda x: 1.0)
  assert t is not None and math.isclose(c, 60 * MPH)


# ---- curveslow-lightning: vision candidate --------------------------------------------------------
def test_vision_apex_straightaway_not_a_candidate():
  # tiny lateral accel (straight road) -> no candidate
  v, d = icbm_vision_apex(30.0, 0.2, 3.0)
  assert v == 0.0 and d == float('inf')


def test_vision_apex_physics():
  # sharp vision curve: apex = v_ego*sqrt(A_LAT/|lat|); check the closed form + distance
  v_ego, lat, ttc = 30.0, 4.0, 2.0
  v, d = icbm_vision_apex(v_ego, lat, ttc)
  assert math.isclose(v, v_ego * math.sqrt(VTSC_A_LAT / lat))
  assert math.isclose(d, ttc * v_ego)
  assert v < v_ego and d > 0.0                     # a real slow-down, ahead of us


def test_vision_curve_binds_when_map_blind():
  # THE 493-curve gap: no map curve, but a sharp vision curve within brake distance now binds
  v_ego, vset = 40.0, 40.0                          # ~90 mph, stock set = ego
  vis_v, vis_dist = icbm_vision_apex(v_ego, 5.0, 1.0)  # sharp curve, ~1 s out
  assert vis_v > 0.0
  t, c, s = icbm_curve_target(v_ego, vset, 0.0, float('inf'), None,
                              lambda x: 1.0, vis_v, vis_dist)
  assert t is not None and t < vset and s == "vis"  # binds BELOW set where before it returned None
  assert math.isclose(c, vset)


def test_vision_dec_only_never_above_ceiling():
  # DEC-only preserved on the vision path: a vision apex ABOVE the ceiling never caps (no speed-up)
  v_ego, vset = 30.0, 30.0
  vis_v, vis_dist = icbm_vision_apex(v_ego, ICBM_VISION_ENTER + 0.01, 0.5)  # barely a curve -> apex ~ v_ego
  t, c, s = icbm_curve_target(v_ego, vset, 0.0, float('inf'), None, lambda x: 1.0, vis_v, vis_dist)
  assert t is None and c is None and s is None      # apex >= ceiling - MIN_DROP -> no cap


def test_lowest_binding_target_wins_map_vs_vision():
  # both candidates bind; the one with the LOWER apex (most slowing) is chosen
  v_ego, vset = 40.0, 40.0
  # map apex ~30, vision apex ~24 -> vision wins
  vis_v, vis_dist = icbm_vision_apex(v_ego, 6.94, 0.5)   # apex ~24 m/s, close
  t, c, s = icbm_curve_target(v_ego, vset, 30.0, 20.0, None, lambda x: 1.0, vis_v, vis_dist)
  assert s == "vis" and t < 30.0


def test_icbm_step_gating_chill_vs_ces():
  """The bridge publishes a curve target only in the CES button state (active=True); forced Chill
  (active=False) publishes {} and clears the ceiling latch. Direct on the manager method, capnp-free."""
  import inspect
  import time as _t
  from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
  from openpilot.selfdrive.controls.lib.pnw_vehicle import PnwVehicle

  cls = next(o for o in vars(m).values() if inspect.isclass(o) and hasattr(o, "_icbm_step"))

  class FakeMem:
    def put_nonblocking(self, k, v): self.last = v

  class Stub: pass
  mgr = Stub(); mgr.mem_params = FakeMem(); mgr._icbm_ceiling = None
  mgr._veh = PnwVehicle(None)             # curveslow-lightning: non-Lightning -> penalty 0.0
  step = cls._icbm_step.__get__(mgr)

  # sharp binding curve, CES active -> real dict target
  sig = {"v_ego": 20 * MPH, "v_set": 45 * MPH, "map_target_v": 18 * MPH, "map_target_dist": 15.0}
  mgr._icbm_last_pub = _t.monotonic() - 1.0
  step(sig, active=True)
  assert isinstance(mgr.mem_params.last, dict) and mgr.mem_params.last.get("target") is not None

  # forced Chill -> {} + ceiling cleared (executor stops within its stale window)
  mgr._icbm_last_pub = _t.monotonic() - 1.0
  step(sig, active=False)
  assert mgr.mem_params.last == {} and mgr._icbm_ceiling is None
