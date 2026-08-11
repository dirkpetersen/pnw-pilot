"""icbmtrack2pnw — continuous curve-profile set-tracking tests + the 19:58:37Z root-cause regression.

Field event (2026-07-12 19:58:31-59Z, I-90): following a lead at ~70 mph with set 90; a rated map
curve ahead (raw mapV 64.9 mph, 118-311 m); driver changed lanes at :35, the lead vanished and the
stock ACC accelerated 72->89 INTO the curve; the (correct) in-curve gate blocked a late vision
start and the driver tapped down manually. Two findings:
  ROOT CAUSE of the missing map bind at :37-39: the tiered sweeper scale (raw >= 29 m/s -> x1.8)
  inflated the curve to an effective 107 mph, so "not binding vs set 90" was computed correctly on
  an absurd target (no bug in the binding condition; fixed by the ICBM-only scale cap
  ICBM_MAP_EFF_SCALE_CAP = MAP_SCALE_MIN, field-calibrated against BOTH 2026-07-12 legs).
  FEATURE: tracking — a binding-RATED map curve within the tap-cadence-sized window starts the set
  walk-down regardless of the v_ego brake envelope (lead-bound protection).
"""
import math

from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw_constants as C
from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import (
  IcbmEpisode, icbm_curve_target, icbm_map_eff_scale, icbm_track_window_m,
  ICBM_TRACK_MAX_M, ICBM_MAP_EFF_SCALE_CAP, ICBM_RESTORE_DELAY_S, ICBM_MIN_DROP_MS,
  ICBM_MAP_SCALE_MIN, ICBM_MAP_SCALE_LO_MPH, ICBM_MAP_SCALE_HI_MPH)
from openpilot.selfdrive.controls.lib import pnw_vehicle as pv
from openpilot.selfdrive.controls.lib.pnw_vehicle import PnwVehicle

MPH = 0.44704
T0 = 100.0
IDENT = lambda x: 1.0  # noqa: E731
LIGHTNING = "FORD_F_150_LIGHTNING_MK1"


# ---- window function ------------------------------------------------------------------------------
def test_track_window_shape():
  # big highway drop -> capped at the max; small drop / low set -> short window (no city tracking)
  assert icbm_track_window_m(31.3, 40.2, 20.0) == ICBM_TRACK_MAX_M      # 45-step walk would need ~890 m
  w_big = icbm_track_window_m(31.3, 40.2, 36.0)                          # the 19:58 geometry: ~312 m
  assert 250.0 < w_big < ICBM_TRACK_MAX_M
  w_city = icbm_track_window_m(8.0, 11.2, 6.7)                           # set 25 mph, apex 15 mph
  assert w_city < 100.0
  # worst-case travel speed is max(ref, v_ego): a lead-bound low v_ego must NOT shrink the window
  assert icbm_track_window_m(20.0, 40.2, 36.0) == icbm_track_window_m(40.2, 40.2, 36.0)
  assert icbm_track_window_m(None, "x", 1.0) == 0.0                      # garbage -> 0 (no tracking)


def test_icbm_scale_cap():
  # sweepers (>= ICBM_MAP_SCALE_HI_MPH raw): UNCHANGED -- still the tight end of the SHARED tiered
  # ramp (byte-identical sweeper/binding behavior to before the icbmcurve2pnw fix below).
  assert icbm_map_eff_scale(29.02) == ICBM_MAP_EFF_SCALE_CAP             # tiered says 1.8 here
  assert ICBM_MAP_EFF_SCALE_CAP < C.tiered_map_scale(29.02)


def test_icbm_scale_is_near_raw_for_tight_moderate_curves():
  """icbmcurve2pnw (ICBM-CURVE-LATE.md Root Cause A): tight/moderate curves (<= ICBM_MAP_SCALE_LO_MPH
  raw, 50 mph) now get the NEAR-RAW ICBM_MAP_SCALE_MIN (1.10), NOT the old flat 1.35 -- the flat cap
  is what inflated a genuine ~50 mph curve above cruise and killed its candidacy (see the field-event
  regression test below). This is strictly LESS inflation than the shared tiered scale everywhere."""
  assert icbm_map_eff_scale(10.0) == ICBM_MAP_SCALE_MIN
  assert icbm_map_eff_scale(10.0) < C.tiered_map_scale(10.0)             # was: equal (1.35 both)
  assert icbm_map_eff_scale(ICBM_MAP_SCALE_LO_MPH) == ICBM_MAP_SCALE_MIN  # boundary: flat MIN at/below
  assert icbm_map_eff_scale(ICBM_MAP_SCALE_HI_MPH) == ICBM_MAP_EFF_SCALE_CAP  # boundary: flat CAP at/above
  # strictly increasing (linear) between the two breakpoints -- no discontinuity
  mid = (ICBM_MAP_SCALE_LO_MPH + ICBM_MAP_SCALE_HI_MPH) / 2.0
  assert ICBM_MAP_SCALE_MIN < icbm_map_eff_scale(mid) < ICBM_MAP_EFF_SCALE_CAP


def test_58mph_event_regression_moderate_curve_now_binds():
  """ICBM-CURVE-LATE.md field event (2026-08-10): raw ~50 mph map curve, 131 m out, at 55 mph
  cruise (Lightning, map_scale=0.92). Under the OLD flat 1.35 cap this was silently discarded --
  eff = 50 * 1.35 * 0.92 = ~62 mph, ABOVE the 55 mph cruise, so the reduce-only candidacy test threw
  it out before distance/window logic ever ran (icbmT/icbmGate stayed None for the whole approach).
  Under the new near-raw ICBM_MAP_SCALE_MIN the same raw target now binds, with lead time to spare."""
  mph = 0.44704
  v_set, dist = 55.0 * mph, 131.0
  raw_50mph = 50.0 * mph
  # OLD behavior (reconstructed inline -- this is exactly what ICBM_MAP_EFF_SCALE_CAP alone computed
  # before this fix): flat 1.35, net eff ABOVE cruise -> silently rejected.
  old_eff = ICBM_MAP_EFF_SCALE_CAP * raw_50mph * 0.92
  assert old_eff > v_set - ICBM_MIN_DROP_MS                              # confirms the pre-fix bug
  # NEW behavior: near-raw floor -> binds, with the map's 131 m of lead time actually used.
  # v_ego=22 m/s (~49 mph, matching the field event's "still accelerating toward 55") -- the pure
  # v_ego decel envelope doesn't need to bind yet; the tracking window (icbmtrack2pnw) is what
  # actually starts the walk-down early here, exactly the "131 m of lead time recovered" fix.
  t, c, s = icbm_curve_target(22.0, v_set, raw_50mph, dist, None, icbm_map_eff_scale,
                              map_scale=0.92, track=True)
  assert s == "map" and t is not None
  assert t < v_set - ICBM_MIN_DROP_MS
  new_eff = icbm_map_eff_scale(raw_50mph) * raw_50mph * 0.92
  assert abs(t - new_eff) < 1e-6
  # the target stays CLOSE to the raw physics rating (no ~20% derate) -- driver direction 2026-08-10
  assert new_eff < raw_50mph * 1.15                                      # < 15% inflation, not ~24%


# ---- tracking start (pure icbm_curve_target) ------------------------------------------------------
def test_lead_bound_tracking_sets_walk_down():
  """Driver intent scenario: lead-bound at 70 mph, set 90, rated curve (eff 36 m/s) at 300 m. The
  v_ego brake envelope (v_ego < apex -> 30 m) never binds -> yesterday: silent. With track=True the
  set walks down NOW; v_ego stays lead-bound below the apex, so nothing decelerates the truck."""
  v_ego, v_set, eff, dist = 31.3, 40.2, 36.0, 300.0
  old = icbm_curve_target(v_ego, v_set, eff, dist, None, IDENT)
  assert old == (None, None, None)                                       # pre-track behavior
  t, c, s = icbm_curve_target(v_ego, v_set, eff, dist, None, IDENT, track=True)
  assert s == "map" and math.isclose(t, eff) and math.isclose(c, v_set)
  # far candidate path tracks identically (far_v is already effective)
  t, c, s = icbm_curve_target(v_ego, v_set, 0.0, float("inf"), None, IDENT,
                              far_v=eff, far_dist=dist, track=True)
  assert s == "far" and math.isclose(t, eff)


def test_vision_never_tracked():
  # a vision candidate outside its brake envelope must NOT start via the tracking window
  v_ego = 40.0
  vis_v, vis_dist = 30.0, 490.0                                          # envelope ~467 m < 490
  t, c, s = icbm_curve_target(v_ego, 40.2, 0.0, float("inf"), None, IDENT,
                              vis_v, vis_dist, track=True)
  assert t is None and s is None


def test_tracking_still_reduce_only():
  # a rated-above-ref curve can never start a walk (reduce-only unchanged by tracking)
  t, c, s = icbm_curve_target(31.3, 40.2, 40.0, 100.0, None, IDENT, track=True)
  assert t is None
  # and within MIN_DROP of ref -> not worth taps, tracked or not
  t, c, s = icbm_curve_target(31.3, 40.2, 40.2 - ICBM_MIN_DROP_MS / 2, 100.0, None, IDENT, track=True)
  assert t is None


# ---- the 19:58 field replays ----------------------------------------------------------------------
def test_1958_37_missing_bind_root_cause_regression():
  """THE 19:58:37Z window with the real numbers (raw mapV 64.9 mph = 29.02 m/s at 181 m, v_ego
  72.7 mph, set 90, Lightning map_scale 0.92). Under the shared tiered scale the effective target
  was 48.1 m/s (107 mph) -> correctly 'not binding' -> ICBM silent with no gate (the log shows
  T=None gate=None while the truck accelerated into the curve). Under the ICBM scale cap the same
  curve is effective 36.0 m/s (80.6 mph — the speed the driver actually chose manually) and BINDS."""
  v_ego, v_set, raw, dist = 32.5, 40.23, 29.02, 181.0
  # old scale: silent even WITH tracking (the root cause was the scale, not the start precondition)
  t, _, s = icbm_curve_target(v_ego, v_set, raw, dist, None, C.tiered_map_scale,
                              map_scale=0.92, track=True)
  assert t is None and s is None
  # capped ICBM scale: binds (tracking window ~312 m >= 181 m; envelope alone would also miss —
  # v_ego 32.5 < eff 36.0 -> brake_dist 30 m)
  t, c, s = icbm_curve_target(v_ego, v_set, raw, dist, None, icbm_map_eff_scale,
                              map_scale=0.92, track=True)
  assert s == "map" and t is not None
  assert abs(t - 29.02 * ICBM_MAP_EFF_SCALE_CAP * 0.92) < 1e-6           # eff 36.0 m/s = 80.6 mph
  assert t < v_set - ICBM_MIN_DROP_MS


def test_1958_morning_leg_still_silent():
  """Anti-regression for the SAME day's morning complaint (vision over-slow on sweepers the map
  correctly rated 'fine'): raw 70.9 mph at set 85 -> ICBM-scaled effective 88 mph -> still silent.
  The scale cap must not resurrect map over-slow on the morning sweepers."""
  t, _, s = icbm_curve_target(33.5, 38.0, 31.7, 200.0, None, icbm_map_eff_scale,
                              map_scale=0.92, track=True)
  assert t is None and s is None


def test_1958_lane_change_protection_scenario(tmp_path, monkeypatch):
  """End-to-end brain replay of the protection the driver designed: lead-bound at 70 with set 90,
  rated curve at 300 m -> the set walks down to ~apex WHILE still behind the lead; when the lead
  vanishes and the ACC accelerates, it can only accelerate TO THE WALKED-DOWN SET (the apex), never
  90 into the curve."""
  import inspect
  import time as _t
  from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(tmp_path / "nope.json"))
  cls = next(o for o in vars(m).values() if inspect.isclass(o) and hasattr(o, "_icbm_step"))

  class FakeMem:
    def put_nonblocking(self, k, v): self.last = v

  class FakeCP:
    carFingerprint = LIGHTNING
    brand = "ford"
    openpilotLongitudinalControl = False

  class Stub:
    pass
  mgr = Stub()
  mgr.mem_params = FakeMem()
  mgr._icbm_ceiling = None
  mgr._veh = PnwVehicle(FakeCP())
  mgr._map_targets = []
  mgr._cur_lat = mgr._cur_lon = None
  mgr._icbm_ep = m.IcbmEpisode()
  mgr._icbm_dir = None
  mgr._icbm_gate = None
  mgr._icbm_map_reach = None
  mgr._stock_set = 0.0
  mgr._stock_on = False
  step = cls._icbm_step.__get__(mgr)

  def run(v_ego, dist, stock_set):
    mgr._stock_set, mgr._stock_on = stock_set, True
    mgr._icbm_last_pub = _t.monotonic() - 1.0
    step({"v_ego": v_ego, "v_set": 40.23, "map_target_v": 29.02, "map_target_dist": dist,
          "curve_lat_accel_vision": 0.0, "time_to_curve": 10.0, "lat_accel_now": 0.0,
          "pitch": None, "gas": False, "brake": False}, active=True)
    return mgr.mem_params.last

  # phase 1: lead-bound at 70 mph, curve 300 m out -> cap starts, ceiling latched at 90
  out = run(31.3, 300.0, 40.23)
  assert out.get("target") is not None and out["target"] < 40.23 - ICBM_MIN_DROP_MS
  assert abs(out["ceiling"] - 40.23) < 0.01 and mgr._icbm_ep.phase == "cap"
  apex_target = out["target"]                       # ~35.4 m/s (79 mph) after the Lightning penalty
  assert 34.0 < apex_target < 36.5
  # phase 2: lane change, lead gone, ACC accelerating (v_ego rising), set already walked to ~apex:
  # the brain keeps commanding the SAME apex target — the ACC may only accelerate to it
  for v_ego, dist, set_now in ((33.0, 250.0, 38.0), (34.5, 200.0, 36.5), (35.5, 150.0, 35.5)):
    out = run(v_ego, dist, set_now)
    assert out.get("target") is not None
    assert abs(out["target"] - apex_target) < 0.7   # target stays the curve apex, never rises


# ---- long episodes / multi-curve / divergence -----------------------------------------------------
def test_cap_phase_has_no_expiry_long_lead_bound_episode():
  """Tracking + traffic means LONG cap phases. The cap phase must have no time bound (only the
  RESTORE phase carries the 45 s window) and the restore must still work after a 120 s cap."""
  ep = IcbmEpisode()
  t = T0
  for i in range(480):                                                   # 120 s at 4 Hz
    out = ep.step(t + i * 0.25, 30.0, 40.2, 40.2 - min(i, 20) * 0.447, True, False,
                  cap_dist=250.0, v_ego=31.0)
    assert out[1] == "dec" and ep.phase == "cap"
  t += 480 * 0.25
  ep.step(t, None, 31.0, 31.0, True, False, cap_dist=20.0, v_ego=31.0)   # curve passed
  tgt, d = ep.step(t + ICBM_RESTORE_DELAY_S + 0.1, None, 31.0, 31.0, True, False)
  assert d == "inc" and math.isclose(tgt, 40.2)                          # ceiling survived 120 s


def test_multi_curve_sequence_merges_into_one_episode():
  """Consecutive rated curves inside the tracking window keep cap_target non-None across the gap ->
  ONE episode, ORIGINAL ceiling — no restore/cap ping-pong between them."""
  ep = IcbmEpisode()
  ep.step(T0, 30.0, 40.2, 40.2, True, False, cap_dist=150.0, v_ego=31.0)   # curve A
  assert math.isclose(ep.ceiling, 40.2)
  # A passed; B (tracked, 300 m) already binds on the very next tick -> same episode
  out = ep.step(T0 + 1, 33.0, 38.0, 33.0, True, False, cap_dist=300.0, v_ego=31.0)
  assert out[1] == "dec" and ep.phase == "cap" and math.isclose(ep.ceiling, 40.2)


def test_route_divergence_self_heals_to_restore():
  """Exit case (Gemini focus c): the set walked down for a tracked curve, then the driver exits and
  mapd re-matches — the candidate vanishes while still FAR (no apex passage) -> the full 3 s
  flicker-proof debounce, then the guarded restore returns the set to the driver's ceiling."""
  ep = IcbmEpisode()
  ep.step(T0, 36.0, 40.2, 40.2, True, False, cap_dist=300.0, v_ego=31.0)  # tracked cap, walked to 36
  ep.step(T0 + 1, None, 36.9, 36.9, True, False, v_ego=31.0)              # diverged: candidate gone
  assert ep._apex_passed is False                                         # far clear -> NOT a passage
  out = ep.step(T0 + 2, None, 36.9, 36.9, True, False)
  assert out == (None, None) and ep.phase == "cap"                        # still the 3 s hold
  tgt, d = ep.step(T0 + 1 + ICBM_RESTORE_DELAY_S + 0.1, None, 36.9, 36.9, True, False)
  assert d == "inc" and math.isclose(tgt, 40.2)                           # self-healed to ceiling


def test_tracked_start_still_gated_in_curve(tmp_path, monkeypatch):
  """Rule 2 survives tracking: a rated tracked curve may NOT start a new episode while the vehicle
  is lateral-loaded (hold the current set)."""
  import inspect
  import time as _t
  from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(tmp_path / "nope.json"))
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
  mgr._cur_lat = mgr._cur_lon = None
  mgr._icbm_ep = m.IcbmEpisode()
  mgr._icbm_dir = None
  mgr._icbm_gate = None
  mgr._icbm_map_reach = None
  mgr._stock_set = 40.2
  mgr._stock_on = True
  step = cls._icbm_step.__get__(mgr)
  mgr._icbm_last_pub = _t.monotonic() - 1.0
  step({"v_ego": 31.3, "v_set": 40.2, "map_target_v": 36.0 / 1.35, "map_target_dist": 300.0,
        "curve_lat_accel_vision": 0.0, "time_to_curve": 10.0, "lat_accel_now": 2.0,
        "pitch": None, "gas": False, "brake": False}, active=True)
  assert mgr.mem_params.last == {} and mgr._icbm_gate == "inCurve"
