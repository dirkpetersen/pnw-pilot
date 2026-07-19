"""icbmmapfirst2pnw — start-policy (MAP-FIRST + in-curve suppression) and early-restore tests.

Driver-directed rework after the 2026-07-12 Snoqualmie->Ellensburg leg (vis=60/map=9/far=3 dec
ticks; vision slowed too much / inside curves; one restore silently killed by a latency tap):
  1. MAP-FIRST: with live map coverage over a stretch, the map verdict (incl. "no slowdown
     needed") is authoritative for STARTING episodes; vision initiates only where the map is
     blind AND with time to act (>= ICBM_VIS_MIN_TTC_S). mapd-dead fallback: reach 0 -> vision
     keeps initiating (map-first must never become vision-never).
  2. Never START a dec episode while already lateral-loaded in a curve (any source).
  3. Early restore: curve provably BEHIND (candidate cleared close by) -> 1 s debounce instead
     of 3 s; restore never BEGINS and PAUSES while lateral-loaded.
  4. Latency-tap fixes: hold-baseline absorption + the widened driver-lower tolerance (see
     test_icbm_restore.py for the 18:08:26Z field replay).
A RUNNING episode is exempt from all start gates ("an episode already running may continue").
"""
import math

from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import (
  IcbmEpisode, icbm_map_reach, icbm_in_curve, icbm_vision_may_start,
  ICBM_VIS_MIN_TTC_S, ICBM_IN_CURVE_LAT, ICBM_RESTORE_DELAY_S, ICBM_RESTORE_DELAY_FAST_S,
  ICBM_HOLD_MAX_S, ICBM_LATE_TAP_GRACE_S, ICBM_MARGIN_M, ICBM_EXEC_STEP_MS, ICBM_VISION_ENTER)
from openpilot.selfdrive.controls.lib import pnw_vehicle as pv
from openpilot.selfdrive.controls.lib.pnw_vehicle import PnwVehicle

MPH = 0.44704
T0 = 100.0
LIGHTNING = "FORD_F_150_LIGHTNING_MK1"


# ---- pure helpers ---------------------------------------------------------------------------------
def _pt_north(lat, lon, dist_m, velocity=30.0):
  return {"latitude": lat + dist_m / 111320.0, "longitude": lon, "velocity": velocity}


def test_map_reach_farthest_valid_point():
  lat, lon = 47.0, -122.0
  pts = [_pt_north(lat, lon, 120.0), _pt_north(lat, lon, 430.0), _pt_north(lat, lon, 900.0)]
  assert abs(icbm_map_reach(pts, lat, lon) - 430.0) < 5.0     # 900 m point is beyond mapd's horizon
  assert icbm_map_reach([], lat, lon) == 0.0                  # mapd dead -> no coverage
  assert icbm_map_reach(pts, None, None) == 0.0               # GPS lost -> no coverage
  bad = [{"latitude": float("nan"), "longitude": lon, "velocity": 1.0}, {"bogus": 1}]
  assert icbm_map_reach(bad, lat, lon) == 0.0                 # NaN/garbage skipped


def test_map_reach_stale_path_decays_out():
  # a dead mapd's last path 600+ m BEHIND us must not count as coverage (liveness fallback)
  lat, lon = 47.0, -122.0
  stale = [_pt_north(lat, lon, -600.0), _pt_north(lat, lon, -800.0)]
  assert icbm_map_reach(stale, lat, lon) == 0.0


def test_map_reach_rejects_curvature_noise_spike():
  """icbmonset: a point mapd itself can't resolve trustworthily (curvature-noise spike, live-
  observed magnitudes below) must not count as 'the map has judged this stretch' -- it was
  previously counted toward reach purely on position, regardless of how implausible its velocity
  was, wrongly blocking vision from filling the gap (icbm_vision_may_start) while the map's own
  read there is garbage. Can only ever SHRINK reach vs before, never grow it."""
  lat, lon = 47.0, -122.0
  noise_only = [_pt_north(lat, lon, 71.0, 77.8), _pt_north(lat, lon, 288.0, 128.6)]
  assert icbm_map_reach(noise_only, lat, lon) == 0.0        # was 288.0 before this fix
  # a sane point still counts normally, noise sitting nearer or farther doesn't change that
  mixed = noise_only + [_pt_north(lat, lon, 199.0, 14.6)]
  assert abs(icbm_map_reach(mixed, lat, lon) - 199.0) < 1.0


def test_in_curve_thresholds():
  assert icbm_in_curve(ICBM_IN_CURVE_LAT + 0.1, 0.0, 10.0)              # loaded now
  assert icbm_in_curve(-ICBM_IN_CURVE_LAT - 0.1, 0.0, 10.0)             # sign-agnostic
  assert not icbm_in_curve(ICBM_IN_CURVE_LAT - 0.2, 0.0, 10.0)          # below exit hysteresis
  # camera's binding curve effectively under us: real curve + under the act window
  assert icbm_in_curve(0.0, ICBM_VISION_ENTER + 0.5, ICBM_VIS_MIN_TTC_S - 0.5)
  assert not icbm_in_curve(0.0, ICBM_VISION_ENTER + 0.5, ICBM_VIS_MIN_TTC_S + 0.5)
  assert not icbm_in_curve(0.0, ICBM_VISION_ENTER - 0.5, 0.5)           # not a real curve
  assert not icbm_in_curve(None, "bogus", None)                         # garbage -> False


def test_vision_may_start_matrix():
  # (a) map covers the stretch -> blocked regardless of time
  assert not icbm_vision_may_start(100.0, 10.0, 430.0)
  # (b) beyond map coverage + time to act -> allowed
  assert icbm_vision_may_start(500.0, 10.0, 430.0)
  # mapd dead (reach 0) -> only the time gate applies
  assert icbm_vision_may_start(100.0, ICBM_VIS_MIN_TTC_S + 0.1, 0.0)
  assert not icbm_vision_may_start(100.0, ICBM_VIS_MIN_TTC_S - 0.1, 0.0)


# ---- _icbm_step start-policy integration (same stub pattern as test_icbm_bridge) ------------------
class FakeCP:
  def __init__(self, fp="", brand="", op_long=False):
    self.carFingerprint = fp
    self.brand = brand
    self.openpilotLongitudinalControl = op_long


def _icbm_stub(veh=None):
  import inspect
  from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
  cls = next(o for o in vars(m).values() if inspect.isclass(o) and hasattr(o, "_icbm_step"))

  class FakeMem:
    def put_nonblocking(self, k, v): self.last = v

  class Stub:
    pass
  mgr = Stub()
  mgr.mem_params = FakeMem()
  mgr._icbm_ceiling = None
  mgr._veh = veh or PnwVehicle(FakeCP(LIGHTNING, "ford"))
  mgr._map_targets = []
  mgr._cur_lat = None
  mgr._cur_lon = None
  mgr._icbm_ep = m.IcbmEpisode()
  mgr._icbm_dir = None
  mgr._icbm_gate = None
  mgr._icbm_map_reach = None
  mgr._stock_set = 0.0
  mgr._stock_on = False
  return mgr, cls._icbm_step.__get__(mgr)


def _run(mgr, step, sig, stock_set=None, stock_on=True):
  import time as _t
  mgr._stock_set = sig["v_set"] if stock_set is None else stock_set
  mgr._stock_on = stock_on
  mgr._icbm_last_pub = _t.monotonic() - 1.0
  step(sig, active=True)
  return mgr.mem_params.last


def _sig(v_ego, v_set, vis_lat=0.0, ttc=10.0, lat_now=0.0, map_v=0.0, map_dist=float("inf")):
  return {"v_ego": v_ego, "v_set": v_set, "map_target_v": map_v, "map_target_dist": map_dist,
          "curve_lat_accel_vision": vis_lat, "time_to_curve": ttc, "lat_accel_now": lat_now,
          "pitch": None, "gas": False, "brake": False}


def test_vision_blocked_when_map_covers_the_stretch(tmp_path, monkeypatch):
  """MAP-FIRST core: live map coverage + no binding map candidate ('no slowdown needed') ->
  a vision candidate INSIDE the coverage may NOT start an episode (the 18:08:22Z field episode:
  raw mapV 70.9 mph -> effective ~117 vs ceiling 85 = map says fine; vision capped anyway)."""
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(tmp_path / "nope.json"))
  lat, lon = 47.0, -122.0
  mgr, step = _icbm_stub()
  mgr._cur_lat, mgr._cur_lon = lat, lon
  # generous sweeper path out to ~430 m: scaled targets sit far ABOVE the set -> no map candidate
  mgr._map_targets = [_pt_north(lat, lon, d, 50.0) for d in (100.0, 250.0, 430.0)]
  # sharp vision candidate at ~116 m (ttc 4 s at 29 m/s) — INSIDE the map's coverage
  out = _run(mgr, step, _sig(29.0, 29.0, vis_lat=3.474, ttc=4.0))
  assert out == {} and mgr._icbm_gate == "visCovered"
  assert mgr._icbm_ep.phase == "idle"                        # no episode started


def test_vision_allowed_beyond_map_coverage(tmp_path, monkeypatch):
  """The same vision candidate BEYOND the map's reach (map blind there) initiates."""
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(tmp_path / "nope.json"))
  lat, lon = 47.0, -122.0
  mgr, step = _icbm_stub()
  mgr._cur_lat, mgr._cur_lon = lat, lon
  mgr._map_targets = [_pt_north(lat, lon, 80.0, 50.0)]       # coverage ends at 80 m
  out = _run(mgr, step, _sig(29.0, 29.0, vis_lat=3.474, ttc=4.0))   # vis at ~116 m > 80 m
  assert out.get("target") is not None and mgr._icbm_src == "vis" and mgr._icbm_gate is None


def test_vision_allowed_when_mapd_dead(tmp_path, monkeypatch):
  """mapd-liveness fallback (2026-07-12 outage): NO map data at all -> vision must still
  initiate (map-first never becomes vision-never), subject only to the time gate."""
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(tmp_path / "nope.json"))
  mgr, step = _icbm_stub()                                   # _map_targets == [] (mapd down)
  out = _run(mgr, step, _sig(29.0, 29.0, vis_lat=3.474, ttc=4.0))
  assert out.get("target") is not None and mgr._icbm_src == "vis"


def test_vision_blocked_when_too_late(tmp_path, monkeypatch):
  """No map data BUT under the act window (ttc < 2.75 s): never begin taps AT the curve."""
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(tmp_path / "nope.json"))
  mgr, step = _icbm_stub()
  out = _run(mgr, step, _sig(29.0, 29.0, vis_lat=3.474, ttc=1.0))
  assert out == {} and mgr._icbm_gate in ("visLate", "inCurve")
  assert mgr._icbm_ep.phase == "idle"


def test_map_beats_vision_when_both_present(tmp_path, monkeypatch):
  """Both a binding MAP candidate and a covered vision candidate: the episode starts from the
  MAP (its 500 m anticipatory verdict), never the vision one."""
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(tmp_path / "nope.json"))
  lat, lon = 47.0, -122.0
  mgr, step = _icbm_stub()
  mgr._cur_lat, mgr._cur_lon = lat, lon
  # path covering ~430 m; near-window map candidate binds (raw 20/1.35 m/s scaled ~ 20*0.92)
  mgr._map_targets = [_pt_north(lat, lon, d, 50.0) for d in (250.0, 430.0)]
  # vision apex (16.8 m/s) LOWER than the map's (18.9) -> yesterday's build would pick vision;
  # map-first suppresses the covered vision start and the MAP candidate starts the episode instead
  sig = _sig(26.0, 26.0, vis_lat=6.0, ttc=4.0, map_v=20.0 / 1.35, map_dist=105.0)
  out = _run(mgr, step, sig)
  assert out.get("target") is not None
  assert mgr._icbm_src in ("map", "far")                     # never "vis" while covered
  assert mgr._icbm_gate == "visCovered"                      # the vision start was suppressed


def test_no_new_episode_starts_in_curve_any_source(tmp_path, monkeypatch):
  """Driver rule 2: lateral-loaded (measured-now |lat| >= exit hysteresis) -> NO new dec episode,
  even for a binding MAP candidate — hold the current set."""
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(tmp_path / "nope.json"))
  mgr, step = _icbm_stub()
  sig = _sig(26.0, 26.0, lat_now=2.0, map_v=15.0 / 1.35, map_dist=60.0)
  out = _run(mgr, step, sig)
  assert out == {} and mgr._icbm_gate == "inCurve" and mgr._icbm_ep.phase == "idle"


def test_running_episode_continues_inside_curve(tmp_path, monkeypatch):
  """'An episode already running may continue': the SAME in-curve signals that block a start do
  NOT interrupt an episode that began before the curve loaded up."""
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(tmp_path / "nope.json"))
  mgr, step = _icbm_stub()
  # start cleanly ahead of the curve (map candidate, not loaded yet)
  out = _run(mgr, step, _sig(26.0, 26.0, map_v=15.0 / 1.35, map_dist=100.0))
  assert out.get("target") is not None and mgr._icbm_ep.phase == "cap"
  # now inside the curve (lat_now high): the running episode keeps steering the set
  out = _run(mgr, step, _sig(24.0, 26.0, lat_now=2.2, map_v=15.0 / 1.35, map_dist=40.0),
             stock_set=20.0)
  assert out.get("target") is not None and mgr._icbm_ep.phase == "cap"


# ---- early restore + in-curve restore deferral (pure IcbmEpisode) ---------------------------------
def test_early_restore_when_apex_passed():
  """Curve provably behind (candidate cleared while close) -> restore begins after the FAST
  debounce (1 s), not the full 3 s."""
  ep = IcbmEpisode()
  v = 30.0
  # candidate 20 m ahead at the last cap tick — within MARGIN + v*1s when it cleared
  ep.step(T0, 40 * MPH, 60 * MPH, 60 * MPH, True, False, cap_dist=20.0, v_ego=v)
  ep.step(T0 + 1, None, 40 * MPH, 40 * MPH, True, False, v_ego=v)          # clear tick
  # at +FAST(+eps): already eligible (would still be silent under the old 3 s hold)
  t_fast = T0 + 1 + ICBM_RESTORE_DELAY_FAST_S + 0.1
  assert t_fast < T0 + 1 + ICBM_RESTORE_DELAY_S                            # sanity: genuinely earlier
  tgt, d = ep.step(t_fast, None, 40 * MPH, 40 * MPH, True, False, v_ego=v)
  assert d == "inc" and math.isclose(tgt, 60 * MPH) and ep.phase == "restore"


def test_full_debounce_when_cleared_far_ahead():
  """A dropout-style clear (candidate still far ahead) keeps the flicker-proof 3 s hold —
  the S-curve-gap protection is unchanged."""
  ep = IcbmEpisode()
  v = 30.0
  far = ICBM_MARGIN_M + v * 1.0 + 150.0                                    # well beyond the pass window
  ep.step(T0, 40 * MPH, 60 * MPH, 60 * MPH, True, False, cap_dist=far, v_ego=v)
  ep.step(T0 + 1, None, 40 * MPH, 40 * MPH, True, False, v_ego=v)
  out = ep.step(T0 + 1 + ICBM_RESTORE_DELAY_FAST_S + 0.1, None, 40 * MPH, 40 * MPH, True, False, v_ego=v)
  assert out == (None, None) and ep.phase == "cap"                         # still holding
  tgt, d = ep.step(T0 + 1 + ICBM_RESTORE_DELAY_S + 0.1, None, 40 * MPH, 40 * MPH, True, False, v_ego=v)
  assert d == "inc" and math.isclose(tgt, 60 * MPH)


def test_no_cap_dist_keeps_old_timing():
  """Callers that never pass cap_dist (legacy path) get the exact pre-mapfirst 3 s behavior."""
  ep = IcbmEpisode()
  ep.step(T0, 40 * MPH, 60 * MPH, 60 * MPH, True, False)
  ep.step(T0 + 1, None, 40 * MPH, 40 * MPH, True, False)
  assert ep.step(T0 + 1 + ICBM_RESTORE_DELAY_FAST_S + 0.1, None, 40 * MPH, 40 * MPH,
                 True, False) == (None, None)
  assert ep.phase == "cap"


def test_restore_entry_deferred_while_lat_loaded():
  """Even after the debounce, a restore may not BEGIN while in_curve — it waits, then enters
  the moment the load clears (bounded by ICBM_HOLD_MAX_S)."""
  ep = IcbmEpisode()
  ep.step(T0, 40 * MPH, 60 * MPH, 60 * MPH, True, False, cap_dist=20.0, v_ego=30.0)
  ep.step(T0 + 1, None, 40 * MPH, 40 * MPH, True, False)
  t = T0 + 1 + ICBM_RESTORE_DELAY_S + 0.1
  out = ep.step(t, None, 40 * MPH, 40 * MPH, True, False, in_curve=True)
  assert out == (None, None) and ep.phase == "cap"                         # deferred, not reset
  tgt, d = ep.step(t + 1.0, None, 40 * MPH, 40 * MPH, True, False, in_curve=False)
  assert d == "inc" and math.isclose(tgt, 60 * MPH)


def test_hold_gives_up_if_loaded_too_long():
  """Loaded past ICBM_HOLD_MAX_S with no re-bind: give up silently — no stale ceiling latch."""
  ep = IcbmEpisode()
  ep.step(T0, 40 * MPH, 60 * MPH, 60 * MPH, True, False)
  ep.step(T0 + 1, None, 40 * MPH, 40 * MPH, True, False)
  out = ep.step(T0 + 1 + ICBM_HOLD_MAX_S + 0.5, None, 40 * MPH, 40 * MPH, True, False, in_curve=True)
  assert out == (None, None) and ep.phase == "idle" and ep.ceiling is None


def test_restore_pauses_while_lat_loaded_and_resumes():
  """Gemini focus (b): a running restore must never press SET+ while lateral-loaded — it goes
  silent (executor stale-stops) WITHOUT resetting, and resumes when the load clears."""
  ep = IcbmEpisode()
  ep.step(T0, 40 * MPH, 60 * MPH, 60 * MPH, True, False, cap_dist=20.0, v_ego=30.0)
  ep.step(T0 + 1, None, 40 * MPH, 40 * MPH, True, False)
  t = T0 + 1 + ICBM_RESTORE_DELAY_S + 0.1
  assert ep.step(t, None, 40 * MPH, 40 * MPH, True, False)[1] == "inc"     # restoring
  out = ep.step(t + 1.0, None, 42 * MPH, 42 * MPH, True, False, in_curve=True)
  assert out == (None, None) and ep.phase == "restore"                     # paused, episode alive
  tgt, d = ep.step(t + 2.0, None, 42 * MPH, 42 * MPH, True, False, in_curve=False)
  assert d == "inc" and math.isclose(tgt, 60 * MPH)                        # resumed
  # aborts stay LIVE through the pause: a set decrease while paused kills the episode
  ep2 = IcbmEpisode()
  ep2.step(T0, 40 * MPH, 60 * MPH, 60 * MPH, True, False)
  ep2.step(T0 + 1, None, 40 * MPH, 40 * MPH, True, False)
  t2 = T0 + 1 + ICBM_RESTORE_DELAY_S + 0.1
  assert ep2.step(t2, None, 40 * MPH, 40 * MPH, True, False)[1] == "inc"
  out = ep2.step(t2 + 1.0, None, 39 * MPH, 39 * MPH, True, False, in_curve=True)
  assert out == (None, None) and ep2.phase == "idle"                       # human SET- during pause


def test_late_tap_absorption_bounds():
  """Hold-baseline absorption: within the grace window a <= 1.6-step DOWNWARD drift is our own
  in-flight tap (absorbed); a bigger drop, or ANY movement after the grace, still reads human."""
  # (a) 1 step down inside the grace -> absorbed -> restore proceeds
  ep = IcbmEpisode()
  ep.step(T0, 40 * MPH, 60 * MPH, 60 * MPH, True, False)
  ep.step(T0 + 1, None, 41 * MPH, 41 * MPH, True, False)                   # snapshot 41
  ep.step(T0 + 1 + ICBM_LATE_TAP_GRACE_S - 0.5, None, 40 * MPH, 40 * MPH, True, False)
  tgt, d = ep.step(T0 + 1 + ICBM_RESTORE_DELAY_S + 0.1, None, 40 * MPH, 40 * MPH, True, False)
  assert d == "inc" and math.isclose(tgt, 60 * MPH)
  # (b) 3 steps down inside the grace (driver burst) -> NOT absorbed -> no restore
  ep = IcbmEpisode()
  ep.step(T0, 40 * MPH, 60 * MPH, 60 * MPH, True, False)
  ep.step(T0 + 1, None, 43 * MPH, 43 * MPH, True, False)                   # snapshot 43
  ep.step(T0 + 1 + 0.5, None, 40 * MPH, 40 * MPH, True, False)
  out = ep.step(T0 + 1 + ICBM_RESTORE_DELAY_S + 0.1, None, 40 * MPH, 40 * MPH, True, False)
  assert out == (None, None) and ep.phase == "idle"
  # (c) 1 step down AFTER the grace -> human -> no restore (the original movement guard)
  ep = IcbmEpisode()
  ep.step(T0, 40 * MPH, 60 * MPH, 60 * MPH, True, False)
  ep.step(T0 + 1, None, 41 * MPH, 41 * MPH, True, False)
  ep.step(T0 + 1 + ICBM_LATE_TAP_GRACE_S + 0.5, None, 40 * MPH, 40 * MPH, True, False)
  out = ep.step(T0 + 1 + ICBM_RESTORE_DELAY_S + 0.1, None, 40 * MPH, 40 * MPH, True, False)
  assert out == (None, None) and ep.phase == "idle"
  # (d) UPWARD movement is never absorbed, even inside the grace
  ep = IcbmEpisode()
  ep.step(T0, 40 * MPH, 60 * MPH, 60 * MPH, True, False)
  ep.step(T0 + 1, None, 41 * MPH, 41 * MPH, True, False)
  ep.step(T0 + 1 + 0.5, None, 43 * MPH, 43 * MPH, True, False)
  out = ep.step(T0 + 1 + ICBM_RESTORE_DELAY_S + 0.1, None, 43 * MPH, 43 * MPH, True, False)
  assert out == (None, None) and ep.phase == "idle"


def test_absorption_cannot_chain_multiple_driver_taps():
  """Gemini adversarial catch (icbmmapfirst2pnw review): the absorbed baseline is ANCHORED to the
  original snapshot and adopted at most ONCE — a driver tapping SET- repeatedly inside the grace
  window (each step small enough to absorb individually) must NOT be walked down tap by tap.
  Scenario where the min_target guard alone would NOT catch it: targets ROSE before the clear
  (min_target 40, last target/floor 44), so two driver taps 44->43->42 stay above
  min_target - tolerance. The movement guard must still block the restore."""
  ep = IcbmEpisode()
  ep.step(T0, 40 * MPH, 60 * MPH, 60 * MPH, True, False)                   # min_target 40
  ep.step(T0 + 0.5, 44 * MPH, 60 * MPH, 45 * MPH, True, False)             # target rose to 44
  ep.step(T0 + 1, None, 44 * MPH, 44 * MPH, True, False)                   # clear: snapshot 44
  ep.step(T0 + 1.3, None, 44 * MPH, 43 * MPH, True, False)                 # driver tap 1 (in grace)
  ep.step(T0 + 1.6, None, 44 * MPH, 42 * MPH, True, False)                 # driver tap 2 (in grace)
  out = ep.step(T0 + 1 + ICBM_RESTORE_DELAY_S + 0.1, None, 44 * MPH, 42 * MPH, True, False)
  assert out == (None, None) and ep.phase == "idle"                        # blocked: human, not us


def test_step_tolerance_sanity():
  """The absorption/lower tolerances stay within one-executor-tap semantics: absorption covers at
  most one full tap + deadband; two full taps beyond can never be self-inflicted."""
  from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import ICBM_LATE_TAP_TOL, ICBM_DRIVER_LOWER_TOL
  assert ICBM_LATE_TAP_TOL < 2.0 * ICBM_EXEC_STEP_MS
  assert ICBM_DRIVER_LOWER_TOL < 2.0 * ICBM_EXEC_STEP_MS
