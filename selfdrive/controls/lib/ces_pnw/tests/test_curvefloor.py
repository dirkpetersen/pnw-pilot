"""curvefloor2pnw: posted-limit curve floor + steering-saturation evidence gate.

Field event 2026-08-11: sharp right curve, posted limit 25 mph (spdLim 11.2 m/s), the truck slowed
50->29->~15 mph even though the computed curve target (icbmT) was ~31 mph. Root cause: the ICBM
(stock-ACC) path had NO speed-limit floor at all, and IcbmEpisode._min_target only ever ratchets
DOWN within one capped episode, so a single over-slow tick latches the whole curve there. Driver
principle: never command below the posted limit unless sustained steering-saturation evidence says
the truck genuinely can't hold the curve at the limit speed.

Two layers tested here:
  1. the pure helpers in ces_pnw_constants.py (steering_saturation_tick / evidence_gate_step /
     speed_limit_curve_floor) — signal + hysteresis + floor math in isolation;
  2. the ICBM integration (_icbm_step, via the same bare-stub pattern test_icbm_bridge.py already
     uses) — proves the floor actually reaches the published IcbmTarget and is relaxed by evidence.
"""
import math

from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw_constants as C
from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import icbm_vision_apex
from openpilot.selfdrive.controls.lib.vtsc_pnw.vtsc_constants import A_LAT_TARGET

MPH = 0.44704


# ---- steering_saturation_tick -----------------------------------------------------------------
def test_sat_flag_alone_is_evidence():
  assert C.steering_saturation_tick(True, False, 0.0) is True


def test_curv_lim_without_big_kerr_is_not_evidence():
  assert C.steering_saturation_tick(False, True, 0.0005) is False


def test_curv_lim_with_big_kerr_is_evidence():
  # field-observed range straddles the threshold (0.0016 -> 0.0021); 0.0021 must read as evidence
  assert C.steering_saturation_tick(False, True, 0.0021) is True


def test_kerr_alone_without_curv_lim_is_not_evidence():
  # ISO clip must actually be binding — a big kErr with curvLim False is not (yet) trusted alone
  assert C.steering_saturation_tick(False, False, 0.01) is False


def test_field_event_values_at_the_reported_kerr():
  # the exact field-reported progression: 0.0016 (not yet over threshold) -> 0.0021 (over)
  assert C.steering_saturation_tick(False, True, 0.0016) is False
  assert C.steering_saturation_tick(False, True, 0.0021) is True


def test_bad_input_never_raises_and_reads_as_no_evidence():
  assert C.steering_saturation_tick(None, None, None) is False
  assert C.steering_saturation_tick(False, True, "garbage") is False
  assert C.steering_saturation_tick(False, True, float('nan')) is False  # nan >= thresh is False anyway


# ---- evidence_gate_step (debounce/hysteresis) -------------------------------------------------
def test_evidence_enters_only_after_sustained_ticks():
  count, evid = 0, False
  for _ in range(C.EVIDENCE_ENTER_TICKS - 1):
    count, evid = C.evidence_gate_step(count, evid, True)
    assert evid is False                        # not yet — needs one more tick
  count, evid = C.evidence_gate_step(count, evid, True)
  assert evid is True                            # sustained -> now trusted


def test_single_noisy_tick_does_not_flip_evidence():
  # a same-direction run resets on a reversal (a single opposite tick can't itself trip evidence
  # -- it only ever counts consecutive ticks in one direction) -- evidence stays False throughout
  count, evid = 0, False
  count, evid = C.evidence_gate_step(count, evid, True)
  assert evid is False and count == 1
  count, evid = C.evidence_gate_step(count, evid, False)   # reversal -> restarts the run at -1
  assert evid is False and count == -1
  # and a lone True tick right after does NOT itself trip evidence either -- still needs
  # EVIDENCE_ENTER_TICKS consecutive positives from here
  count, evid = C.evidence_gate_step(count, evid, True)
  assert evid is False and count == 1


def test_evidence_clears_only_after_sustained_absence():
  count, evid = 0, True                          # start already latched (mid-episode)
  for _ in range(C.EVIDENCE_CLEAR_TICKS - 1):
    count, evid = C.evidence_gate_step(count, evid, False)
    assert evid is True                          # still holds — evidence is stickier to clear
  count, evid = C.evidence_gate_step(count, evid, False)
  assert evid is False


def test_evidence_clear_is_stickier_than_enter():
  assert C.EVIDENCE_CLEAR_TICKS >= C.EVIDENCE_ENTER_TICKS


def test_count_never_grows_unbounded():
  count, evid = 0, False
  for _ in range(500):
    count, evid = C.evidence_gate_step(count, evid, True)
  assert abs(count) <= max(C.EVIDENCE_ENTER_TICKS, C.EVIDENCE_CLEAR_TICKS)


# ---- speed_limit_curve_floor -------------------------------------------------------------------
def test_floor_raises_a_too_low_target_to_the_limit():
  assert math.isclose(C.speed_limit_curve_floor(6.7, 11.2, False), 11.2)   # 15 mph target, 25 mph limit


def test_floor_never_lowers_a_target_already_above_limit():
  assert C.speed_limit_curve_floor(20.0, 11.2, False) == 20.0


def test_floor_noop_with_no_limit_data():
  assert C.speed_limit_curve_floor(6.7, 0.0, False) == 6.7


def test_evidence_relaxes_the_floor_entirely():
  assert C.speed_limit_curve_floor(6.7, 11.2, True) == 6.7   # unchanged: evidence permits below-limit


# ---- ICBM integration: the floor actually reaches the published IcbmTarget ---------------------
def _icbm_stub(veh=None, spd_lim=0.0, sl_evidence=False):
  """Bare CESController stand-in exposing exactly what _icbm_step touches (mirrors
  test_icbm_bridge.py's _icbm_stub, plus the curvefloor2pnw evidence attribute)."""
  import inspect
  from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
  from openpilot.selfdrive.controls.lib.pnw_vehicle import PnwVehicle

  cls = next(o for o in vars(m).values() if inspect.isclass(o) and hasattr(o, "_icbm_step"))

  class FakeMem:
    def put_nonblocking(self, k, v): self.last = v

  class Stub:
    pass

  mgr = Stub()
  mgr.mem_params = FakeMem()
  mgr._icbm_ceiling = None
  mgr._veh = veh if veh is not None else PnwVehicle(None)
  mgr._map_targets = []
  mgr._cur_lat = None
  mgr._cur_lon = None
  mgr._icbm_ep = m.IcbmEpisode()
  mgr._icbm_dir = None
  mgr._stock_set = 0.0
  mgr._stock_on = False
  mgr._sl_evidence = sl_evidence
  return mgr, cls._icbm_step.__get__(mgr)


def _published_target(step, mgr, sig):
  import time as _t
  mgr._icbm_last_pub = _t.monotonic() - 1.0
  mgr._icbm_ceiling = None
  step(sig, active=True)
  return mgr.mem_params.last.get("target")


def _field_event_sig(v_ego, v_set, spd_lim, target_frac=0.7):
  """A vision-sourced curve candidate shaped like the field event's END state (the 50->29->~15 mph
  slide bottomed out well BELOW the 25 mph posted limit; the *snapshot* icbmT of ~31 mph was an
  earlier, less-binding tick — this fixture reproduces "the binding tick", the one the floor must
  catch). `curve_lat_accel_vision` is solved backwards from the desired target/limit ratio via the
  exact icbm_vision_apex physics (apex = v_ego*sqrt(A_LAT/lat)), so the fixture is guaranteed
  self-consistent regardless of what v_ego/spd_lim the caller passes. No map data (mapd-blind
  stretch, matching the field event where vision was the active source)."""
  target_v = target_frac * spd_lim
  lat_acc = A_LAT_TARGET * (v_ego / target_v) ** 2
  vis_v, vis_dist = icbm_vision_apex(v_ego, lat_acc, 3.0)
  assert vis_v < spd_lim, "test setup: the raw candidate must itself sit BELOW the posted limit"
  return {"v_ego": v_ego, "v_set": v_set, "map_target_v": 0.0, "map_target_dist": float("inf"),
          "curve_lat_accel_vision": lat_acc, "time_to_curve": 3.0, "pitch": None,
          "spd_lim": spd_lim}


def test_icbm_floor_prevents_the_field_event_over_slow():
  spd_lim = 25 * MPH                                     # posted limit
  mgr, step = _icbm_stub(spd_lim=spd_lim, sl_evidence=False)
  sig = _field_event_sig(v_ego=45 * MPH, v_set=45 * MPH, spd_lim=spd_lim)
  target = _published_target(step, mgr, sig)
  assert target is not None
  assert target >= spd_lim - 1e-6                       # floored AT the posted limit, not below


def test_icbm_floor_relaxed_with_steering_evidence():
  spd_lim = 25 * MPH
  mgr, step = _icbm_stub(spd_lim=spd_lim, sl_evidence=True)   # sustained saturation evidence present
  sig = _field_event_sig(v_ego=45 * MPH, v_set=45 * MPH, spd_lim=spd_lim)
  target = _published_target(step, mgr, sig)
  assert target is not None
  assert target < spd_lim - 0.5                          # evidence permits the physics target below


def test_icbm_floor_never_exceeds_the_ceiling():
  # driver's own set is BELOW the posted limit (e.g. already chose 20 mph on a 25 mph road) -- the
  # floor must never raise the published target above the driver's set/ceiling (DEC-only preserved).
  spd_lim = 25 * MPH
  v_set = 20 * MPH
  mgr, step = _icbm_stub(spd_lim=spd_lim, sl_evidence=False)
  sig = _field_event_sig(v_ego=v_set, v_set=v_set, spd_lim=spd_lim)
  target = _published_target(step, mgr, sig)
  assert target is not None
  assert target <= v_set + 1e-6


def test_icbm_floor_noop_when_target_already_above_limit():
  # a candidate that binds ABOVE the posted limit is untouched by the floor (reduce-only: the floor
  # only ever raises a too-low target, never introduces a new cap above it). v_set is well above the
  # candidate so it actually binds (icbm_curve_target/​_icbm_binding_apex is reduce-only vs the
  # ceiling -- a candidate above v_set would be rejected as a non-candidate long before the floor
  # logic runs, which would test the wrong thing here).
  spd_lim = 25 * MPH
  v_ego = v_set = 70 * MPH   # v_ego above the target too, so the candidate is a genuine slow-DOWN
                            # (falls inside the decel brake envelope, not rejected as "not yet binding")
  mgr, step = _icbm_stub(spd_lim=spd_lim, sl_evidence=False)
  # a candidate whose physics target sits at the midpoint of (spd_lim, v_set) -- above the limit,
  # below v_ego/v_set -- solved backwards the same way _field_event_sig does, so it's guaranteed
  # self-consistent rather than a hand-picked lat_acc that happened to work for one v_ego.
  target_v = (spd_lim + v_set) / 2.0
  lat_acc = A_LAT_TARGET * (v_ego / target_v) ** 2
  vis_v, vis_dist = icbm_vision_apex(v_ego, lat_acc, 3.0)
  assert spd_lim < vis_v < v_set
  sig = {"v_ego": v_ego, "v_set": v_set, "map_target_v": 0.0, "map_target_dist": float("inf"),
        "curve_lat_accel_vision": lat_acc, "time_to_curve": 3.0, "pitch": None, "spd_lim": spd_lim}
  target = _published_target(step, mgr, sig)
  assert target is not None
  assert abs(target - vis_v) < 0.05                      # unchanged by the floor


def test_icbm_floor_noop_with_no_limit_data():
  mgr, step = _icbm_stub(spd_lim=0.0, sl_evidence=False)
  sig = _field_event_sig(v_ego=45 * MPH, v_set=45 * MPH, spd_lim=25 * MPH)
  sig["spd_lim"] = 0.0                                    # no OSM data this tick
  target = _published_target(step, mgr, sig)
  assert target is not None
  assert target < 25 * MPH - 0.5                          # V_MIN-floored physics target, no limit floor


def test_icbm_step_defaults_to_no_evidence_when_attribute_missing():
  """Every pre-curvefloor2pnw stub in test_icbm_bridge.py/test_icbm_track.py/etc. never sets
  `_sl_evidence` — _icbm_step must degrade to the safe default (floor enforced) via getattr, not
  raise AttributeError."""
  import inspect
  from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
  from openpilot.selfdrive.controls.lib.pnw_vehicle import PnwVehicle
  cls = next(o for o in vars(m).values() if inspect.isclass(o) and hasattr(o, "_icbm_step"))

  class FakeMem:
    def put_nonblocking(self, k, v): self.last = v

  class Stub:
    pass
  mgr = Stub()
  mgr.mem_params = FakeMem()
  mgr._icbm_ceiling = None
  mgr._veh = PnwVehicle(None)
  mgr._map_targets = []
  mgr._cur_lat = None
  mgr._cur_lon = None
  mgr._icbm_ep = m.IcbmEpisode()
  mgr._icbm_dir = None
  mgr._stock_set = 0.0
  mgr._stock_on = False
  # deliberately NOT setting mgr._sl_evidence
  step = cls._icbm_step.__get__(mgr)
  sig = _field_event_sig(v_ego=45 * MPH, v_set=45 * MPH, spd_lim=25 * MPH)
  target = _published_target(step, mgr, sig)
  assert target is not None
  assert target >= 25 * MPH - 1e-6                        # defaulted to no-evidence -> floor enforced
