"""icbmslow2pnw — the ICBM map-rating floor.

Driver, 2026-09-17 14:25 PT: *"button control management also took me down to 38 mph... it's going
too slow."* Measured on that event: mapd rated the curve 43.6 mph and ICBM commanded 38.2.

WHY THE FLOOR EXISTS (the double count). A map candidate reaches the penalty as
`icbm_map_eff_scale(raw) * raw * map_scale`. When the Lightning curve penalty was field-calibrated
(2026-07-11) that composite was a FLAT 1.35 * 0.92 = 1.242x, so the penalty ate part of an
inflation and the July target still landed ABOVE mapd's own rating. icbmcurve2pnw (2026-08-11)
dropped the tight end of the scale to 1.10 for a different and correct reason -- a 50 mph curve
inflated past a 55 mph cruise was being REJECTED as a candidate -- which makes the composite
1.10 * 0.92 = 1.012x, i.e. essentially raw. Nothing re-calibrated the penalty against that, so since
2026-08-11 it has been subtracting a 5 mph inflation margin from a number that no longer carries one.

WHAT IS PINNED HERE
  * the floor is the BINDING candidate's OWN raw rating, for map AND far sources;
  * the descent guard and the left-curve factor still bite BELOW it (they model risk mapd's rating
    does not contain, and flooring the multiplied penalty would make them silently do nothing). The
    left factor is NEUTRAL (1.0) by default since 2026-09-24 -- its "downhill-LEFT washouts" premise
    came from inverted direction labels -- so its test here sets it via config;
  * vision candidates are never floored by THIS floor (curvefix2pnw added their own, at vision's 2.5 m/s^2 speed);
  * the Tesla and every non-Lightning car are byte-identical;
  * `icbm_map_floor_frac = 0.0` reproduces the pre-change target exactly (the documented off switch);
  * the floor is reduce-only: never above the candidate itself, therefore never above the ceiling;
  * `icbmMapFlr` / `icbmMapFlrHit` reach the ces_events record, and clear on an inactive tick.

NOT pinned here, on purpose: `curve_speed_penalty_ms` itself is untouched, so VTSC (the op-long
path) and the 2026-07-11 washout regression registry are unaffected -- see
test_washout_registry.py, which passes unchanged.
"""
import math

import pytest

from openpilot.selfdrive.controls.lib import pnw_vehicle as pv
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
from openpilot.selfdrive.controls.lib.pnw_vehicle import PnwVehicle
from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_icbm_bridge import (
  FakeCPA, LIGHTNING, _icbm_stub, _path, _published_target, _pt_north,
)

MPH = 0.44704
LAT, LON = 47.0, -122.0


@pytest.fixture
def shipped_cfg(tmp_path, monkeypatch):
  """No /data/pnw/curve.json -> the shipped source defaults."""
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(tmp_path / "nope.json"))


def _veh(fp=LIGHTNING, brand="ford", **cfg):
  v = PnwVehicle(FakeCPA(fp, brand))
  if cfg:
    v._curve_cfg = dict(v._curve_cfg, **cfg)
  return v


def _map_sig(v_ego, map_v, dist=205.0, v_set=None, pitch=None):
  """A NEAR-window map candidate: the raw mapd rating is `map_v`, `dist` m ahead."""
  return {"v_ego": v_ego, "v_set": v_set if v_set is not None else v_ego,
          "map_target_v": map_v, "map_target_dist": dist,
          "curve_lat_accel_vision": 0.0, "time_to_curve": 10.0, "pitch": pitch}


def _run(veh, sig, bend=None, points=None):
  """One brain tick -> (published target, the controller stub).

  `_map_targets` defaults to EMPTY on purpose: _path()'s points all carry velocity 20.0, so leaving
  it populated lets the FULL-HORIZON far scanner supply a competing candidate and the test stops
  measuring the near candidate it set up. Pass `bend` only when the test is about turn direction."""
  mgr, step = _icbm_stub(veh)
  mgr._cur_lat, mgr._cur_lon = LAT, LON
  mgr._map_targets = (_path(LAT, LON, bend) if bend is not None else
                      ([] if points is None else points))
  return _published_target(step, mgr, sig), mgr


# --------------------------------------------------------------------------------------------
# the floor itself
# --------------------------------------------------------------------------------------------
def test_floor_is_the_candidates_own_raw_rating(shipped_cfg):
  """A map curve rated 20 m/s: the penalty alone would command ~17.8; the floor gives back 20.0."""
  veh = _veh()
  raw = 20.0
  t, _ = _run(veh, _map_sig(26.0, raw))
  assert t is not None
  assert t == pytest.approx(raw, abs=0.01)
  # and it really was the penalty that was being given back, not a no-op
  off, _ = _run(_veh(icbm_map_floor_frac=0.0), _map_sig(26.0, raw))
  assert off < raw - 1.0 * MPH, f"no penalty to give back at raw={raw} -- the test proves nothing"


def test_frac_zero_is_the_documented_off_switch(shipped_cfg):
  """icbm_map_floor_frac = 0.0 must reproduce the pre-icbmslow2pnw target BIT for bit."""
  for raw in (10.0, 14.0, 18.0, 20.0, 24.0):
    # dist 150 m keeps every raw in the list inside the decel envelope at 30 m/s -- at 260 m the
    # 24 m/s case falls outside BOTH the envelope and the tracking window and binds at all.
    on, _ = _run(_veh(icbm_map_floor_frac=0.0), _map_sig(30.0, raw, dist=150.0))
    eff = m.icbm_map_eff_scale(raw) * raw * 0.92
    expect = max(eff - _veh().curve_speed_penalty_ms(eff), 0.0)
    assert on == pytest.approx(expect, abs=0.01), f"raw={raw}"


def test_partial_fraction_floors_proportionally(shipped_cfg):
  raw = 20.0
  full, _ = _run(_veh(), _map_sig(26.0, raw))
  half, _ = _run(_veh(icbm_map_floor_frac=0.5), _map_sig(26.0, raw))
  off, _ = _run(_veh(icbm_map_floor_frac=0.0), _map_sig(26.0, raw))
  # 0.5 * 20 = 10 is BELOW the penalised target, so a half floor cannot bind -> equals "off"
  assert half == pytest.approx(off, abs=0.01)
  assert half < full - 0.5


def test_floor_is_reduce_only_never_above_the_candidate_or_the_ceiling(shipped_cfg):
  """Sweep: the floored target is never above the unfloored CANDIDATE (pre-penalty), never above
  the driver's set, and never below the unfloored target."""
  veh, off = _veh(), _veh(icbm_map_floor_frac=0.0)
  checked = 0
  for raw in (8.0, 12.0, 16.0, 20.0, 24.0, 28.0, 32.0):
    for v_set in (20.0, 26.0, 32.0, 40.0):
      for dist in (80.0, 205.0, 400.0):
        sig = _map_sig(v_set, raw, dist=dist, v_set=v_set)
        t, _ = _run(veh, sig)
        t0, _ = _run(off, sig)
        if t is None:
          assert t0 is None, "the floor must not change whether a candidate binds at all"
          continue
        eff = m.icbm_map_eff_scale(raw) * raw * 0.92
        assert t <= eff + 1e-9, f"floor raised the target above the candidate ({t} > {eff})"
        assert t <= v_set + 1e-9, "floor raised the target above the driver's set"
        assert t >= t0 - 1e-9, "floor lowered the target"
        checked += 1
  assert checked >= 40, f"only {checked} binding cases -- the sweep proves little"


def test_the_floor_never_changes_candidacy(shipped_cfg):
  """The whole reason this is a floor and not `map_scale = 1.0`: raising the EFFECTIVE speed would
  push candidates past `ref - ICBM_MIN_DROP_MS` and silently delete slowdowns (18 of 88 replayable
  episodes, drives 2026-08-12..09-17). The floor is applied after candidacy, so it cannot."""
  veh, off = _veh(), _veh(icbm_map_floor_frac=0.0)
  near_miss = 0
  for raw in [x * 0.5 for x in range(16, 60)]:
    sig = _map_sig(26.0, raw, dist=205.0)
    t, _ = _run(veh, sig)
    t0, _ = _run(off, sig)
    assert (t is None) == (t0 is None), f"candidacy changed at raw={raw}"
    if t0 is None:
      near_miss += 1
  assert near_miss > 0, "no rejected candidate in the sweep -- the test proves nothing"


# --------------------------------------------------------------------------------------------
# the risks mapd's rating does NOT model still bite
# --------------------------------------------------------------------------------------------
def test_descent_still_lowers_the_target_below_the_floor(shipped_cfg):
  veh = _veh()
  flat, _ = _run(veh, _map_sig(26.0, 20.0, pitch=None))
  down, _ = _run(veh, _map_sig(26.0, 20.0, pitch=-0.05))
  assert down < flat - 0.1, "the descent guard was erased by the floor"
  assert flat == pytest.approx(20.0, abs=0.01)


def test_left_curve_still_lowers_the_target_below_the_floor(shipped_cfg):
  # a curve.json left_factor (neutral by default since 2026-09-24) must not be erased by the floor
  veh = _veh(left_factor=1.15)
  left, _ = _run(veh, _map_sig(26.0, 20.0), bend=+4.0)    # path bends left
  right, _ = _run(veh, _map_sig(26.0, 20.0), bend=-4.0)
  assert left < right - 0.1, "the left-curve factor was erased by the floor"
  assert right == pytest.approx(20.0, abs=0.01)
  # the extra is exactly the multiplied-minus-base penalty, nothing invented
  veh2 = _veh(left_factor=1.15)
  eff = m.icbm_map_eff_scale(20.0) * 20.0 * 0.92
  extra = veh2.curve_speed_penalty_ms(eff, is_left=True) - veh2.curve_speed_penalty_ms(eff)
  assert (right - left) == pytest.approx(extra, abs=0.02)


# --------------------------------------------------------------------------------------------
# sources
# --------------------------------------------------------------------------------------------
def test_far_source_is_floored_at_the_far_points_own_rating(shipped_cfg):
  """The FAR scanner's winning point, not the near window's -- the near window must not leak its
  (higher) rating onto a far candidate. 2026-09-17 14:27:55 PT is exactly that shape: the near
  window read 81.6 mph while the binding far candidate was far slower."""
  veh = _veh()
  far_raw = 18.0
  # 250 m: inside the far scanner's envelope at 26 m/s (300 m is not, and the point would simply
  # not bind). The NEAR candidate is switched off in the sig, so the source can only be "far".
  points = [_pt_north(LAT, LON, 250.0, far_raw)]
  sig = _map_sig(26.0, 0.0, dist=float("inf"))            # no near candidate at all
  t, mgr = _run(veh, sig, points=points)
  assert t is not None and mgr._icbm_src == "far"
  assert t == pytest.approx(far_raw, abs=0.05)
  off, _ = _run(_veh(icbm_map_floor_frac=0.0), sig, points=points)
  assert off < far_raw - 1.0 * MPH, "no penalty to give back on the far path -- proves nothing"


def test_vision_source_is_not_floored(shipped_cfg):
  """icbm_vision_apex is already a physics-derived safe speed; there is no map rating behind it, so
  the Lightning penalty must still apply in full."""
  from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_icbm_bridge import _vis_sig
  veh, off = _veh(), _veh(icbm_map_floor_frac=0.0)
  mgr, step = _icbm_stub(veh)
  t = _published_target(step, mgr, _vis_sig(29.0, -3.474))
  mgr0, step0 = _icbm_stub(off)
  t0 = _published_target(step0, mgr0, _vis_sig(29.0, -3.474))
  assert t is not None and t == pytest.approx(t0, abs=1e-9)


def test_tesla_and_unknown_cars_are_byte_identical(shipped_cfg):
  for cp in (None, FakeCPA("TESLA_MODEL_S_HW3", "tesla", op_long=True), FakeCPA("X", "hyundai")):
    on = PnwVehicle(cp)
    off = PnwVehicle(cp)
    off._curve_cfg = dict(off._curve_cfg, icbm_map_floor_frac=0.0)
    assert on.icbm_map_floor_ms(20.0) == 0.0
    sig = _map_sig(26.0, 20.0)
    a, _ = _run(on, sig)
    b, _ = _run(off, sig)
    assert a == b


# --------------------------------------------------------------------------------------------
# PnwVehicle.icbm_map_floor_ms
# --------------------------------------------------------------------------------------------
def test_floor_accessor_defensive(shipped_cfg):
  v = _veh()
  assert v.icbm_map_floor_ms(20.0) == pytest.approx(20.0)
  for bad in (None, float("nan"), float("inf"), -5.0, 0.0, "x", object()):
    assert v.icbm_map_floor_ms(bad) == 0.0, bad


def test_curve_json_clamps_the_fraction(tmp_path, monkeypatch):
  import json
  p = tmp_path / "curve.json"
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(p))
  for written, expect in ((2.0, 1.0), (-1.0, 0.0), (0.5, 0.5), (float("nan"), 1.0)):
    p.write_text(json.dumps({"lightning": {"icbm_map_floor_frac": written}}))
    v = PnwVehicle(FakeCPA(LIGHTNING, "ford"))
    assert v._curve_cfg["icbm_map_floor_frac"] == pytest.approx(expect), written
  p.write_text("{not json")
  assert PnwVehicle(FakeCPA(LIGHTNING, "ford"))._curve_cfg["icbm_map_floor_frac"] == 1.0


# --------------------------------------------------------------------------------------------
# the field event, and the telemetry that will let the next drive check it
# --------------------------------------------------------------------------------------------
def test_the_2026_09_17_1425_event(shipped_cfg):
  """PT 14:25:15, drives/2026-09-17/curvedb-first-capture: mapd rated the curve 43.6 mph with the
  stock set (ceiling) at 63 mph, and ICBM published 38.2 mph. The forward model reproduces 39.4;
  the floor commands the map's own 43.6."""
  raw = 43.6 * MPH
  veh, off = _veh(), _veh(icbm_map_floor_frac=0.0)
  sig = _map_sig(63.0 * MPH, raw, dist=205.0, v_set=63.0 * MPH)
  before, _ = _run(off, sig)
  after, _ = _run(veh, sig)
  assert before / MPH == pytest.approx(39.4, abs=0.4), f"model drifted: {before / MPH:.1f} mph"
  assert after / MPH == pytest.approx(43.6, abs=0.1)


def test_telemetry_reaches_the_record_and_clears(shipped_cfg):
  from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_ces_record_fields import _record
  rec = _record(_icbm_map_flr=18.75, _icbm_map_flr_hit=True)
  assert rec["icbmMapFlr"] == pytest.approx(18.75) and rec["icbmMapFlrHit"] is True
  rec0 = _record(_icbm_map_flr=0.0, _icbm_map_flr_hit=False)
  assert rec0["icbmMapFlr"] == 0.0 and rec0["icbmMapFlrHit"] is False


def test_telemetry_tracks_the_controller_and_an_inactive_tick_clears_it(shipped_cfg):
  veh = _veh()
  mgr, step = _icbm_stub(veh)
  mgr._cur_lat, mgr._cur_lon = LAT, LON
  mgr._map_targets = []
  t = _published_target(step, mgr, _map_sig(26.0, 20.0))
  assert t is not None
  assert mgr._icbm_map_flr == pytest.approx(20.0, abs=0.01)
  assert mgr._icbm_map_flr_hit is True, "the floor bound but the telemetry says it did not"
  # a tick where ICBM is off must not publish the previous tick's floor
  mgr._icbm_last_pub -= 1.0
  step(_map_sig(26.0, 20.0), active=False)
  assert mgr._icbm_map_flr == 0.0 and mgr._icbm_map_flr_hit is False


def test_flr_hit_is_false_when_the_floor_does_not_bind(shipped_cfg):
  """A sweeper: the 1.35 end of the ICBM scale inflates far more than the penalty removes, so the
  floor sits below the penalised target and must report itself as NOT having bound."""
  veh = _veh()
  mgr, step = _icbm_stub(veh)
  mgr._cur_lat, mgr._cur_lon = LAT, LON
  mgr._map_targets = _path(LAT, LON, 0.0)
  mgr._map_targets = []
  raw = 30.0                                              # 67 mph -> scale 1.35
  t = _published_target(step, mgr, _map_sig(40.0, raw, dist=205.0, v_set=40.0))
  assert t is not None and t > raw, "a sweeper must still be commanded above its raw rating"
  assert mgr._icbm_map_flr == pytest.approx(raw, abs=0.01)
  assert mgr._icbm_map_flr_hit is False


def test_the_far_candidate_reports_the_winning_points_rating(shipped_cfg):
  """icbm_far_map_candidate's third element must belong to the SAME point as its distance."""
  near = _pt_north(LAT, LON, 120.0, 24.0)
  far = _pt_north(LAT, LON, 480.0, 16.0)
  v, d, raw = m.icbm_far_map_candidate([far, near], LAT, LON, 30.0, 32.0, m.icbm_map_eff_scale, 0.92)
  assert math.isclose(d, 120.0, rel_tol=0.02)
  assert raw == pytest.approx(24.0)
  assert v == pytest.approx(m.icbm_map_eff_scale(24.0) * 24.0 * 0.92)


# --------------------------------------------------------------------------------------------
# mutants the first pass of this file did not kill (see _scratch/icbmslow/mutate.py)
# --------------------------------------------------------------------------------------------
def test_far_raw_belongs_to_the_winner_in_BOTH_scan_orders(shipped_cfg):
  """M9: taking the raw from the LAST point scanned instead of the winning one survives a test
  whose winner happens to be last. Both orders are pinned."""
  near = _pt_north(LAT, LON, 120.0, 24.0)               # the binding (most-binding) candidate
  far = _pt_north(LAT, LON, 480.0, 16.0)                # sharper but far -> not selected
  for order in ([far, near], [near, far]):
    v, d, raw = m.icbm_far_map_candidate(order, LAT, LON, 30.0, 32.0, m.icbm_map_eff_scale, 0.92)
    assert math.isclose(d, 120.0, rel_tol=0.02), order
    assert raw == pytest.approx(24.0), f"raw came from the wrong point in order {order}"


def test_floor_is_clamped_to_the_candidate_when_map_scale_deflates_it(tmp_path, monkeypatch):
  """M12: the `min(floor, target)` clamp is unreachable with the SHIPPED scale (composite 1.012x,
  so the candidate always sits above its own raw rating) -- but curve.json may set map_scale as low
  as 0.5, and then the composite is BELOW 1.0 and the raw rating sits ABOVE the candidate. Without
  the clamp the floor would RAISE the target above the curve candidate, i.e. break reduce-only."""
  import json
  p = tmp_path / "curve.json"
  p.write_text(json.dumps({"lightning": {"map_scale": 0.5}}))
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(p))
  veh = PnwVehicle(FakeCPA(LIGHTNING, "ford"))
  raw = 20.0
  eff = m.icbm_map_eff_scale(raw) * raw * 0.5
  assert eff < raw, "the scenario no longer deflates -- the test proves nothing"
  t, _ = _run(veh, _map_sig(26.0, raw))
  assert t is not None
  assert t <= eff + 1e-9, f"the floor raised the target above the candidate ({t} > {eff})"


def test_an_active_tick_with_no_target_clears_the_floor_telemetry(shipped_cfg):
  """M16: the per-tick reset. ICBM stays ACTIVE but the curve clears -- the previous tick's floor
  must not still be published (the inactive-tick reset does not cover this path)."""
  veh = _veh()
  mgr, step = _icbm_stub(veh)
  mgr._cur_lat, mgr._cur_lon = LAT, LON
  mgr._map_targets = []
  assert _published_target(step, mgr, _map_sig(26.0, 20.0)) is not None
  assert mgr._icbm_map_flr > 0.0 and mgr._icbm_map_flr_hit is True
  mgr._icbm_last_pub -= 1.0
  step(_map_sig(26.0, 0.0, dist=float("inf")), active=True)      # straight road, still active
  assert mgr._icbm_last_target is None, "the scenario still has a target -- proves nothing"
  assert mgr._icbm_map_flr == 0.0 and mgr._icbm_map_flr_hit is False


def test_the_target_can_never_go_negative(tmp_path, monkeypatch):
  """M23: the outer zero clamp. A configurable slow-corner penalty of 8 mph on a steep downhill LEFT
  multiplies to the 15 mph cap, so the descent+left EXTRA (7 mph = 3.1 m/s) alone exceeds a very low
  map rating -- and without the clamp the published target goes NEGATIVE, which the executor would
  chase with SET- taps forever. (penalty_min 15 does NOT work: it already sits at penalty_cap_mph,
  so the multipliers have nothing to add and the extra is zero.)

  HONEST LIMIT, recorded rather than hidden: this invariant is enforced TWICE -- here and again by
  the rain2pnw `max(target - rain_penalty_ms(), 0.0)` two statements later -- so deleting either
  clamp alone leaves the published target non-negative and this test still passes. That makes the
  "outer clamp removed" mutant EQUIVALENT (see _scratch/icbmslow/mutate.py M23). The test is kept
  because the INVARIANT is worth pinning: if both clamps ever go, it fires."""
  import json
  p = tmp_path / "curve.json"
  p.write_text(json.dumps({"lightning": {"penalty_min_mph": 8.0, "penalty_max_mph": 8.0}}))
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(p))
  veh = PnwVehicle(FakeCPA(LIGHTNING, "ford"))
  eff = m.icbm_map_eff_scale(2.0) * 2.0 * 0.92
  extra = (veh.curve_speed_penalty_ms(eff, pitch_rad=-0.12, is_left=True)
           - veh.curve_speed_penalty_ms(eff))
  assert extra > eff, f"the extra ({extra:.2f}) does not exceed the floor ({eff:.2f}) -- proves nothing"
  t, _ = _run(veh, _map_sig(26.0, 2.0, dist=80.0, pitch=-0.12), bend=+4.0)
  assert t is not None, "no candidate -- the test proves nothing"
  assert t >= 0.0, f"published a NEGATIVE curve target ({t})"


def test_the_passed_point_gate_returns_the_REPLACEMENT_candidates_raw(shipped_cfg):
  """M22: when the gate drops a passed point and re-decides, the far rating it hands back must
  belong to the NEW candidate. Carrying the old one forward would floor the replacement slowdown at
  a rating from a curve the truck has already driven through."""
  veh = _veh()
  mgr, _step = _icbm_stub(veh)
  mgr._cur_lat, mgr._cur_lon = LAT, LON
  mgr._cur_bearing = 0.0
  behind = _pt_north(LAT, LON, 60.0, 12.0)              # the ORIGINAL winner, about to be "passed"
  ahead = _pt_north(LAT, LON, 240.0, 19.0)              # the replacement, a different rating
  mgr._map_targets = [behind, ahead]
  monkeypatch_mask = [True, False]
  import openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw as mod
  real_passed = mod.icbm_passed_points
  try:
    mod.icbm_passed_points = lambda *a, **k: (list(monkeypatch_mask), "ok")
    sig = {"v_ego": 28.0, "v_set": 32.0, "map_target_v": 0.0, "map_target_dist": float("inf"),
           "curve_lat_accel_vision": 0.0, "time_to_curve": 10.0, "pitch": None}
    old_v, old_d, old_raw = mod.icbm_far_map_candidate(mgr._map_targets, LAT, LON, 28.0, 32.0,
                                                       mod.icbm_map_eff_scale, 0.92)
    assert old_raw == pytest.approx(12.0), "the behind point is not the winner -- proves nothing"
    mgr._icbm_src = "far"
    out = mod._icbm_passed_gate(mgr, 100.0, old_v, sig, LAT, LON, 32.0, old_v, old_d, old_raw,
                                (0.0, float("inf")), ceiling=None, running=False)
  finally:
    mod.icbm_passed_points = real_passed
  assert len(out) == 5
  assert out[4] == pytest.approx(19.0), f"the gate handed back a stale far rating ({out[4]})"


def test_the_effective_floor_config_is_logged_once_at_startup(shipped_cfg, monkeypatch):
  """Rule 2: `_load_curve_config` is silent, so a curve.json setting icbm_map_floor_frac to 0 would
  turn this whole change off with nothing anywhere saying so. The CES controller names the effective
  value once per start -- and only on cars the floor can act on, so the Tesla stays quiet."""
  import inspect
  events = []
  monkeypatch.setattr(m.cloudlog, "event", lambda name, **kw: events.append((name, kw)))
  cls = next(o for o in vars(m).values() if inspect.isclass(o) and hasattr(o, "_icbm_step"))

  class CP:
    carFingerprint = LIGHTNING
    brand = "ford"
    openpilotLongitudinalControl = False
    alphaLongitudinalAvailable = True
  try:
    cls(CP())
  except Exception:
    pass                      # construction may need more of the world; the log fires before that
  hit = [kw for name, kw in events if name == "ces_icbm_map_floor_cfg"]
  assert len(hit) == 1, f"the floor config was not logged exactly once: {events}"
  assert hit[0]["frac"] == pytest.approx(1.0) and hit[0]["map_scale"] == pytest.approx(0.92)

  # ...and the Tesla, which the floor can never act on, must not log it at all
  events.clear()

  class TeslaCP:
    carFingerprint = "TESLA_MODEL_S_HW3"
    brand = "tesla"
    openpilotLongitudinalControl = True
    alphaLongitudinalAvailable = True
  try:
    cls(TeslaCP())
  except Exception:
    pass
  assert not [kw for name, kw in events if name == "ces_icbm_map_floor_cfg"], \
    "the ICBM floor config was logged on a car the floor cannot act on"


def test_the_floor_binding_range_is_bounded_by_the_scale(shipped_cfg):
  """Pins the "only curves rated below ~53 mph" claim the safety argument rests on: above the
  crossover the ICBM scale inflates more than the base hump removes, so the floor cannot bind."""
  veh = _veh()
  binds = []
  for tenth in range(50, 900):
    raw = tenth * 0.1 * MPH
    eff = m.icbm_map_eff_scale(raw) * raw * 0.92
    binds.append((tenth * 0.1, veh.curve_speed_penalty_ms(eff) > eff - raw))
  crossovers = [mph for (mph, b), (_, pb) in zip(binds[1:], binds[:-1], strict=True) if b != pb]
  assert crossovers == [pytest.approx(53.6, abs=0.15)], f"binding range changed: {crossovers}"
  assert all(b for mph, b in binds if mph <= 53.0)
  assert not any(b for mph, b in binds if mph >= 54.0)
