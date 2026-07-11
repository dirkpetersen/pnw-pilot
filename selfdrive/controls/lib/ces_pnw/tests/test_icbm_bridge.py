"""icbm2pnw brain tests — pure icbm_curve_target math (curve cap + ceiling latch + restore)."""
import math

from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import (icbm_curve_target, ICBM_A_DECEL,
                                                              ICBM_MARGIN_M, ICBM_MIN_DROP_MS)

MPH = 0.44704
FLAT = 1.0  # identity scale for readable numbers


def test_no_curve_no_target():
  assert icbm_curve_target(25 * MPH, 60 * MPH, 0.0, float('inf'), None, lambda x: FLAT) == (None, None)


def test_curve_engages_inside_decel_envelope():
  v, vset, apex = 60 * MPH, 60 * MPH, 40 * MPH
  brake_dist = (v * v - apex * apex) / (2 * ICBM_A_DECEL) + ICBM_MARGIN_M
  # outside the envelope: not yet
  t, c = icbm_curve_target(v, vset, apex, brake_dist + 50, None, lambda x: 1.0)
  assert t is None and c is None
  # inside: engage, latch the driver's set as ceiling
  t, c = icbm_curve_target(v, vset, apex, brake_dist - 10, None, lambda x: 1.0)
  assert math.isclose(t, apex) and math.isclose(c, vset)


def test_ceiling_latch_survives_lowered_set():
  # after taps walked the stock set down to 42, v_set follows it — ceiling must stay 60
  apex = 40 * MPH
  t, c = icbm_curve_target(38 * MPH, 42 * MPH, apex, 20.0, 60 * MPH, lambda x: 1.0)
  assert math.isclose(t, apex) and math.isclose(c, 60 * MPH)


def test_curve_cleared_goes_silent_and_unlatches_immediately():
  # DEC-ONLY design: no restore path — when the curve clears, silence + unlatch, driver restores
  t, c = icbm_curve_target(45 * MPH, 42 * MPH, 0.0, float('inf'), 60 * MPH, lambda x: 1.0)
  assert t is None and c is None


def test_small_drops_ignored():
  # apex within ICBM_MIN_DROP_MS of set: not worth button taps
  vset = 60 * MPH
  apex = vset - ICBM_MIN_DROP_MS / 2
  t, c = icbm_curve_target(vset, vset, apex, 10.0, None, lambda x: 1.0)
  assert t is None and c is None


def test_reduce_only_scaled_apex_above_set_ignored():
  # scale lifts the apex above the driver's set -> no cap (reduce-only)
  t, c = icbm_curve_target(60 * MPH, 60 * MPH, 55 * MPH, 10.0, None, lambda x: 1.5)
  assert t is None and c is None


def test_invalid_set_speed_hands_off():
  assert icbm_curve_target(60 * MPH, 0.0, 40 * MPH, 10.0, None, lambda x: 1.0) == (None, None)
  # even while latched, a dropped set speed (ACC off) goes silent immediately
  assert icbm_curve_target(60 * MPH, 0.0, 0.0, float('inf'), 60 * MPH, lambda x: 1.0) == (None, None)


def test_uses_ceiling_as_reference_while_capped():
  # while capped (set already lowered), a still-binding curve keeps the cap even though the
  # lowered v_set is close to the apex (reference must be the LATCHED ceiling, not v_set)
  apex = 40 * MPH
  t, c = icbm_curve_target(41 * MPH, 41 * MPH, apex, 15.0, 60 * MPH, lambda x: 1.0)
  assert t is not None and math.isclose(c, 60 * MPH)


def test_icbm_step_gating_chill_vs_ces():
  """The bridge publishes a curve target only in the CES button state (active=True); forced Chill
  (active=False) publishes {} and clears the ceiling latch. Direct on the manager method, capnp-free."""
  import inspect
  import time as _t
  from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m

  cls = next(o for o in vars(m).values() if inspect.isclass(o) and hasattr(o, "_icbm_step"))

  class FakeMem:
    def put_nonblocking(self, k, v): self.last = v

  class Stub: pass
  mgr = Stub(); mgr.mem_params = FakeMem(); mgr._icbm_ceiling = None
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
