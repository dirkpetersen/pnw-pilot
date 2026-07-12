"""icbm2pnw brain tests — pure icbm_curve_target math (curve cap + ceiling latch + restore).

curveslow-lightning: icbm_curve_target now returns a 3-tuple (target, ceiling, src) and considers a
VISION candidate in addition to the map one; the map-only calls below are byte-equivalent to before.
"""
import math

from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import (icbm_curve_target, icbm_vision_apex,
                                                              icbm_far_map_candidate, icbm_approach_decel,
                                                              upcoming_curve,
                                                              ICBM_A_DECEL, ICBM_MARGIN_M,
                                                              ICBM_MIN_DROP_MS, ICBM_VISION_ENTER,
                                                              ICBM_FIRM_DROP_LO, ICBM_FIRM_DROP_HI)
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
  # descentcurve2pnw: the step now also scans the full map horizon from the cached mapd inputs
  mgr._map_targets = []
  mgr._cur_lat = None
  mgr._cur_lon = None
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


# ---- descentcurve2pnw: full-horizon map candidate + drop-scaled approach decel --------------------
def _pt_north(lat, lon, dist_m, velocity):
  """A mapd path point `dist_m` due north of (lat, lon)."""
  return {"latitude": lat + dist_m / 111320.0, "longitude": lon, "velocity": velocity}


def test_far_map_90_to_65_at_450m_binds_where_before_it_didnt():
  """THE field case (2026-07-11 stock-ACC 90 mph run): a 65 mph map curve 450 m out. The old
  10 s time window (~402 m at 90 mph) never saw it -> ICBM silent. The full-horizon candidate sees
  it, and at the 0.8 comfort envelope (drop 25 mph < the 30 mph firm ramp start) it binds at 450 m."""
  v = vset = 90 * MPH                                   # 40.23 m/s
  lat, lon = 47.0, -122.0
  apex_eff = 65 * MPH                                   # effective (post-scale) target
  points = [_pt_north(lat, lon, 450.0, apex_eff / 0.92)]  # raw so that raw*0.92 = 65 mph

  # BEFORE: the near-window scan (upcoming_curve, 10 s) does not even see the point
  mv, md = upcoming_curve(points, lat, lon, v, 10.0)
  assert mv == 0.0 and md == float('inf')
  t, c, s = icbm_curve_target(v, vset, mv, md, None, lambda x: 1.0)
  assert t is None and s is None

  # AFTER: full-horizon candidate (identity tiered scale, Lightning map_scale 0.92) binds at 450 m
  far_v, far_dist = icbm_far_map_candidate(points, lat, lon, v, vset, lambda x: 1.0,
                                           map_scale=0.92, firm_decel=1.4)
  assert math.isclose(far_v, apex_eff, rel_tol=1e-6) and math.isclose(far_dist, 450.0, rel_tol=0.01)
  t, c, s = icbm_curve_target(v, vset, 0.0, float('inf'), None, lambda x: 1.0,
                              map_scale=0.92, firm_decel=1.4, far_v=far_v, far_dist=far_dist)
  assert t is not None and s == "far" and math.isclose(t, apex_eff, rel_tol=1e-6)
  assert math.isclose(c, vset)                          # ceiling latched at the driver's set


def test_far_map_dec_only_above_ceiling_ignored():
  # a generous OSM sweeper (target above the set even after the 0.92 discount) is never a candidate
  v = vset = 90 * MPH
  lat, lon = 47.0, -122.0
  points = [_pt_north(lat, lon, 300.0, 110 * MPH)]      # mapV ~110 mph (the I-90 sweeper readings)
  far_v, far_dist = icbm_far_map_candidate(points, lat, lon, v, vset, lambda x: 1.0, map_scale=0.92)
  assert far_v == 0.0 and far_dist == float('inf')      # reduce-only: no target, no speed-up path


def test_far_map_most_binding_wins_not_lowest():
  # a NEAR moderate curve inside its envelope must not be shadowed by a FAR sharper curve
  v = vset = 70 * MPH
  lat, lon = 47.0, -122.0
  near = _pt_north(lat, lon, 120.0, 55 * MPH)           # needs action now
  far = _pt_north(lat, lon, 490.0, 40 * MPH)            # sharper but far (envelope not binding yet)
  far_v, far_dist = icbm_far_map_candidate([far, near], lat, lon, v, vset, lambda x: 1.0)
  assert math.isclose(far_dist, 120.0, rel_tol=0.01)    # the near curve is the binding one


def test_far_map_nan_and_bad_points_skipped():
  lat, lon = 47.0, -122.0
  points = [{"latitude": float('nan'), "longitude": lon, "velocity": 20.0},
            {"latitude": lat + 0.001, "longitude": lon, "velocity": float('nan')},
            {"bogus": True}]
  assert icbm_far_map_candidate(points, lat, lon, 30.0, 30.0, lambda x: 1.0) == (0.0, float('inf'))
  assert icbm_far_map_candidate([], lat, lon, 30.0, 30.0, lambda x: 1.0) == (0.0, float('inf'))
  assert icbm_far_map_candidate([_pt_north(lat, lon, 100, 10.0)], None, None, 30.0, 30.0,
                                lambda x: 1.0) == (0.0, float('inf'))


def test_approach_decel_monotonic_and_bounded():
  firm = 1.4
  v = 40.0
  prev = 0.0
  for apex in (38.0, 30.0, 26.0, 20.0, 13.0, 5.0, 0.0):   # growing drop
    a = icbm_approach_decel(v, apex, firm)
    assert ICBM_A_DECEL - 1e-9 <= a <= firm + 1e-9        # always within [base, firm]
    assert a >= prev - 1e-9                               # monotonic non-decreasing in the drop
    prev = a
  # small drops (< 30 mph) keep the full comfort envelope (the field case: a 25 mph drop)
  assert icbm_approach_decel(v, v - ICBM_FIRM_DROP_LO + 0.1, firm) == ICBM_A_DECEL
  # huge drops reach the firm ceiling exactly
  assert icbm_approach_decel(v, v - ICBM_FIRM_DROP_HI - 1.0, firm) == firm


def test_approach_decel_neutral_without_firm():
  # non-Lightning (firm 0.0 / None / <= base) -> base comfort decel, byte-identical envelope
  assert icbm_approach_decel(40.0, 10.0, 0.0) == ICBM_A_DECEL
  assert icbm_approach_decel(40.0, 10.0, None) == ICBM_A_DECEL
  assert icbm_approach_decel(40.0, 10.0, ICBM_A_DECEL) == ICBM_A_DECEL


def test_map_candidate_lightning_scale_lowers_target():
  # the SAME map curve yields a LOWER ICBM target with the Lightning 0.92 discount than without
  v = vset = 70 * MPH
  raw, dist = 60 * MPH, 60.0
  t_plain, _, _ = icbm_curve_target(v, vset, raw, dist, None, lambda x: 1.0)
  t_light, _, _ = icbm_curve_target(v, vset, raw, dist, None, lambda x: 1.0, map_scale=0.92)
  assert t_plain is not None and t_light is not None
  assert math.isclose(t_light, t_plain * 0.92, rel_tol=1e-9)
  assert t_light < t_plain                               # the truck starts slowing to a lower speed


def test_defaults_byte_equivalent_to_pre_descentcurve():
  # explicit: with the neutral knob values the extended signature returns the exact original result
  v, vset, apex = 60 * MPH, 60 * MPH, 40 * MPH
  old = icbm_curve_target(v, vset, apex, 20.0, None, lambda x: 1.0)
  new = icbm_curve_target(v, vset, apex, 20.0, None, lambda x: 1.0,
                          map_scale=1.0, firm_decel=0.0, far_v=0.0, far_dist=float('inf'))
  assert old == new


def test_near_map_candidate_never_delayed_by_firm_decel():
  """Gemini review catch (2026-07-11): a firmer assumed decel SHRINKS the binding envelope (taps
  start later). The near-window map candidate must therefore keep the base comfort envelope —
  identical result with or without the Lightning firm decel, even for a huge drop."""
  v = vset = 90 * MPH
  apex = 15.0                                            # ~34 mph target -> a 56 mph drop
  dist = 700.0                                           # inside the 0.8 envelope, outside the 1.4 one
  base = icbm_curve_target(v, vset, apex, dist, None, lambda x: 1.0)
  firm = icbm_curve_target(v, vset, apex, dist, None, lambda x: 1.0, firm_decel=1.4)
  assert base == firm and base[0] is not None            # binds either way, at the same point
