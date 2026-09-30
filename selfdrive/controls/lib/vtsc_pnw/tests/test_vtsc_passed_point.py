"""vtscpass2pnw (2026-09-29 evening drive, drives/2026-09-29/corvallis-albany-evening/DRIVE_REPORT.md sections 3.3 and 3.5): VTSC's map fold
must never bind on a curve the car has already driven past.

Bug 1: mapd publishes its current way from the way's first node, so the loop ramp just driven stays in MapTargetVelocities, and
`most_binding_map_curve` measured an unsigned haversine distance to it -- a point 165-171 deg BEHIND the car cut the cap 84 -> 42 mph on the
I-5 on-ramp (22:44:49). The fix reuses ces_pnw.icbm_passed_points (the geometry ICBM's behind-gate runs on) as a mask; these tests pin

  * the controller's mask on synthetic roads: ahead binds / behind never binds / beside and inside the 5 m tolerance still bind / a U-turn or
    loop whose later leg lies behind the heading but AHEAD along the path still binds / cannot-tell (slow, no heading, error) = today's
    behaviour, and the change-only log that says so (Rule 2),
  * the two REAL cases replayed through the real VTSCController.cap() (passed_point_frames.py: real geometry, local metres).

Bug 2 (report 3.5): cap() runs every planner cycle whether or not openpilot is engaged, and a `hold` reached while the owner drove a
curve by hand froze the cap at 35 mph and applied it the moment the stalk re-engaged him at 54 mph (54 -> 40 mph). Besides the mask above
(a receding point no longer feeds the latch), every cycle with carControl.enabled False (cruise off; NOT longActive, which a gas
override clears too) drops the state machine's latches and the brain's slew state, so the first engaged cycle starts from idle: no cap,
no freeze. The last tests pin that: reset when cruise is off, NO reset while engaged or during a gas override (closed-loop lift-off), the
release-later freeze is cleared too, an unreadable carControl.enabled is loud and changes nothing, no 20 Hz log, the hold-exit debounce,
"""
import math

import pytest

from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as CES
from openpilot.selfdrive.controls.lib.vtsc_pnw import vtsc_constants as C
from openpilot.selfdrive.controls.lib.vtsc_pnw import vtsc_controller as VC
from openpilot.selfdrive.controls.lib.vtsc_pnw.tests import passed_point_frames as F

MPH = 0.44704
DT = 0.05
LAT0, LON0, M_PER_DEG = 44.5, -123.1, 111320.0
COSLAT = math.cos(math.radians(LAT0))
REAL_MASK = CES.icbm_passed_points


class _CP:
  def __init__(self, fp, brand):
    self.carFingerprint, self.brand, self.openpilotLongitudinalControl = fp, brand, True


class _Params:
  d = {"CESMode": "2", "VtscMapCurves": True}

  def get(self, k, return_default=False):
    return self.d.get(k)

  def get_bool(self, k):
    return bool(self.d.get(k, False))

  def put_nonblocking(self, k, v):
    pass


class _Mem:
  def get(self, k, return_default=False):
    return None

  def put_nonblocking(self, k, v):
    pass


class _NS:
  pass


def make_controller(monkeypatch, fp="TESLA_MODEL_S_HW3", brand="tesla"):
  """a real VTSCController (Standard mode, map curves ON) on a fake clock -> (controller, clock list)"""
  clock = [1000.0]
  monkeypatch.setattr(VC.time, "monotonic", lambda: clock[0])
  monkeypatch.setattr(VC, "apex_turn_direction", lambda model: 0)
  ctrl = VC.VTSCController(_CP(fp, brand), params=_Params())
  ctrl.mem_params = _Mem()
  ctrl._read_enabled(clock[0])
  assert ctrl._enabled and ctrl._map_curves
  return ctrl, clock


def ll(e, n):
  """local east/north metres from the harness origin -> (lat, lon)"""
  return LAT0 + n / M_PER_DEG, LON0 + e / (M_PER_DEG * COSLAT)


def pt(e, n, v):
  la, lo = ll(e, n)
  return {"latitude": la, "longitude": lo, "velocity": v}


class Log:
  """cloudlog stand-in that records every call."""
  def __init__(self):
    self.calls = []

  def __getattr__(self, name):
    return lambda *a, **k: self.calls.append((name, a[0] % a[1:] if len(a) > 1 else (a[0] if a else ""), k))

  def lines(self, level=None):
    return [m for n, m, _ in self.calls if level is None or n == level]


@pytest.fixture
def log(monkeypatch):
  lg = Log()
  monkeypatch.setattr(VC, "cloudlog", lg)
  return lg


def _ctrl(monkeypatch, e=0.0, n=0.0, bearing=0.0, pts=()):
  ctrl, _ = make_controller(monkeypatch)
  ctrl._map_targets = list(pts)
  ctrl._cur_lat, ctrl._cur_lon = ll(e, n)
  ctrl._cur_bearing = bearing
  ctrl._gps_fix_ts = None
  return ctrl


def _fold(ctrl, v_ego=25.0, v_set=38.0):
  """the controller's real fold (mask + most_binding_map_curve) with no camera curve -> (map won?, map distance m, raw m/s)"""
  ctrl._fold_map_curve(0.0, -1.0, float("inf"), v_set, v_ego, 500.0)
  return ctrl._tele_curve_win == "map", ctrl._tele_map_d, ctrl._tele_map_raw


def _road(n0, n1, step=20.0, e=0.0):
  """a straight north-running road from n0 to n1 metres, gentle (raw 60 m/s) nodes, in travel order"""
  out = []
  n = n0
  while n <= n1 + 1e-6:
    out.append((e, n, 60.0))
    n += step
  return out


def _with_curve(road, at_n, raw, e=0.0):
  """replace the node at (east e, north at_n) by a tight one (raw m/s)"""
  return [(x, y, raw if abs(y - at_n) < 1e-6 and abs(x - e) < 1e-6 else v) for x, y, v in road]


# ---------------------------------------------------------------- the controller's mask on synthetic roads

def test_a_curve_ahead_binds(monkeypatch, log):
  road = _with_curve(_road(-200, 400), 240.0, 10.0)
  ctrl = _ctrl(monkeypatch, pts=[pt(*p) for p in road])
  won, d, raw = _fold(ctrl)
  assert won and raw == pytest.approx(10.0) and d == pytest.approx(240.0, abs=1.0)


def test_a_curve_behind_never_binds_and_the_unfixed_fold_would_have(monkeypatch, log):
  road = _with_curve(_road(-300, 400), -140.0, 10.0)             # the tight node is 140 m BEHIND the car
  pts = [pt(*p) for p in road]
  ctrl = _ctrl(monkeypatch, pts=pts)
  won, _, _ = _fold(ctrl)
  assert not won and ctrl._tele_map_raw > 10.0                            # only gentle nodes are left
  monkeypatch.setattr(VC, "icbm_passed_points", lambda *a, **k: (None, "off"))   # today's code: no mask
  ctrl2 = _ctrl(monkeypatch, pts=pts)
  won2, d2, raw2 = _fold(ctrl2)
  assert won2 and raw2 == pytest.approx(10.0) and d2 == pytest.approx(140.0, abs=1.0)   # the bug, reproduced


def test_a_curve_beside_the_car_and_within_the_tolerance_still_binds(monkeypatch, log):
  # 3 m behind along the road (inside ICBM_PASSED_TOL_M = 5): GPS lag / position noise must not hide a curve the car is at
  road = _with_curve(_road(-201, 399, step=3.0), -3.0, 10.0)
  ctrl = _ctrl(monkeypatch, pts=[pt(*p) for p in road])
  won, d, raw = _fold(ctrl, v_ego=20.0)
  assert won and raw == pytest.approx(10.0) and d == pytest.approx(3.0, abs=1.0)
  # exactly abreast (0 m), off the centreline by the lane offset
  road = _with_curve(_road(-201, 399, step=3.0, e=6.0), 0.0, 10.0, e=6.0)
  ctrl = _ctrl(monkeypatch, pts=[pt(*p) for p in road])
  won, _, raw = _fold(ctrl, v_ego=20.0)
  assert won and raw == pytest.approx(10.0)


def test_a_curve_just_past_the_tolerance_is_passed(monkeypatch, log):
  road = _with_curve(_road(-201, 399, step=3.0), -12.0, 10.0)      # 12 m behind: physically behind at 20 m/s + GPS lag
  ctrl = _ctrl(monkeypatch, pts=[pt(*p) for p in road])
  won, _, _ = _fold(ctrl, v_ego=20.0)
  assert not won


def _hairpin():
  """north for 100 m, a U-turn, and back south 30 m to the east: a switchback. The return leg lies BEHIND the heading of a car on the
  outbound leg but AHEAD of it along the path."""
  out = [(0.0, float(n), 60.0) for n in range(-200, 101, 20)]
  out += [(10.0, 110.0, 60.0), (20.0, 100.0, 60.0), (30.0, 90.0, 60.0)]
  out += [(30.0, float(n), 60.0) for n in range(80, -101, -20)]
  return out


def test_a_uturn_return_leg_behind_the_heading_but_ahead_along_the_path_still_binds(monkeypatch, log):
  road = _with_curve(_hairpin(), 20.0, 9.0, e=30.0)                # tight node on the return leg, at north +20
  ctrl = _ctrl(monkeypatch, e=0.0, n=60.0, bearing=0.0, pts=[pt(*p) for p in road])   # car on the outbound leg at north 60
  mask, why = REAL_MASK(ctrl._map_targets, ctrl._cur_lat, ctrl._cur_lon, 0.0, 25.0)
  assert why == "ok"
  # the return-leg node is south of the car (behind the heading) yet not masked: the path still reaches it
  idx = [i for i, p in enumerate(road) if p[2] == 9.0][0]
  assert not mask[idx]
  won, _, raw = _fold(ctrl)
  assert won and raw == pytest.approx(9.0)
  # and the outbound leg the car has driven IS masked
  assert mask[0] and mask[1]


def test_the_car_on_the_return_leg_has_passed_the_outbound_leg(monkeypatch, log):
  road = _with_curve(_hairpin(), 60.0, 9.0, e=0.0)                        # tight node on the OUTBOUND leg at north 60
  ctrl = _ctrl(monkeypatch, e=30.0, n=40.0, bearing=180.0, pts=[pt(*p) for p in road])   # car heading south on the return leg
  won, _, raw = _fold(ctrl)
  assert not (won and raw == pytest.approx(9.0))                            # behind along the path (and 30 m off it): never binds
  mask, why = REAL_MASK(ctrl._map_targets, ctrl._cur_lat, ctrl._cur_lon, 180.0, 25.0)
  assert why == "ok" and mask[[i for i, p in enumerate(road) if p[2] == 9.0][0]]


@pytest.mark.parametrize("why,kw", [("slow", {"v_ego": 3.0}), ("noHeading", {"bearing": None})])
def test_cannot_tell_keeps_todays_behaviour_and_says_so(monkeypatch, log, why, kw):
  road = _with_curve(_road(-300, 400), -140.0, 10.0)
  ctrl = _ctrl(monkeypatch, bearing=kw.get("bearing", 0.0), pts=[pt(*p) for p in road])
  won, _, raw = _fold(ctrl, v_ego=kw.get("v_ego", 25.0))
  assert won and raw == pytest.approx(10.0)                              # no mask -> the point behind still binds, as today
  assert any(f"unknown ({why})" in m for m in log.lines("info"))          # ...and the log says the gate could not tell


def test_a_failing_mask_is_logged_loudly_and_falls_back_to_no_gate(monkeypatch, log):
  def boom(*a, **k):
    raise RuntimeError("boom")
  monkeypatch.setattr(VC, "icbm_passed_points", boom)
  road = _with_curve(_road(-300, 400), -140.0, 10.0)
  ctrl = _ctrl(monkeypatch, pts=[pt(*p) for p in road])
  won, _, raw = _fold(ctrl)
  assert won and raw == pytest.approx(10.0)
  assert any("passed-point mask FAILED (RuntimeError)" in m for m in log.lines("exception"))
  n = len(log.lines("exception"))
  _fold(ctrl)
  assert len(log.lines("exception")) == n                                 # rate-limited, not once per cycle


def test_the_verdict_is_logged_change_only(monkeypatch, log):
  road = _with_curve(_road(-300, 400), -140.0, 10.0)
  ctrl = _ctrl(monkeypatch, pts=[pt(*p) for p in road])
  for _ in range(5):
    _fold(ctrl)
  said = [m for m in log.lines("info") if "passed-point mask" in m]
  assert len(said) == 1 and "passed (ok)" in said[0] and "points already passed" in said[0]
  ctrl._cur_lat, ctrl._cur_lon = ll(0.0, -400.0)                           # ahead of every point: nothing passed any more
  ctrl._map_targets = [pt(*p) for p in _road(-390, 0)]
  _fold(ctrl)
  _fold(ctrl)
  said = [m for m in log.lines("info") if "passed-point mask" in m]
  assert len(said) == 2 and "clear (ok)" in said[1]


def test_ahead_points_drops_exactly_the_passed_ones_and_returns_the_input_when_it_cannot_tell(monkeypatch, log):
  road = _road(-100, 100, step=20.0)
  pts = [pt(*p) for p in road]
  ctrl = _ctrl(monkeypatch, pts=pts)
  ahead = ctrl._ahead_points(25.0)
  assert ahead == [p for p, (_, n, _) in zip(pts, road, strict=True) if n >= -0.0 - 5.0]        # 0 and the tolerance point stay, -20 and back go
  ctrl._cur_bearing = None
  assert ctrl._ahead_points(25.0) is ctrl._map_targets                                          # cannot tell: the very same list, nothing removed


# ---------------------------------------------------------------- the real cases through the real controller

def replay_geo(monkeypatch, ticks, paths, fix=True, dt=DT):
  """Feed the recorded 1 Hz ticks (position, heading, speed, camera curve) and the newest mapdPath record at each moment to the REAL
  VTSCController.cap() at 20 Hz; positions/headings/speeds/camera values are interpolated between ticks, everything downstream (mask,
  fold, state machine, rate limits, floors) is the real code. fix=False disables ONLY the passed-point mask = the code as of 848d115."""
  ctrl, clock = make_controller(monkeypatch)
  monkeypatch.setattr(VC, "icbm_passed_points", REAL_MASK if fix else (lambda *a, **k: (None, "off")))
  vis = {}
  monkeypatch.setattr(VC, "model_curve_state", lambda model, v_cruise, a_lat: vis["s"])
  ns = _NS()
  ns.orientationNED = [0.0, 0.0, 0.0]
  sm = {"modelV2": object(), "carControl": ns}
  out = []
  for i in range(len(ticks) - 1):
    ns.enabled = bool(ticks[i][10])                  # selfdriveState.enabled at that tick (stalk / cruise state)
    a, b = ticks[i], ticks[i + 1]
    n = int(round((b[0] - a[0]) / dt))
    for j in range(n):
      f = j / n
      t = a[0] + j * dt
      e, nn = a[1] + (b[1] - a[1]) * f, a[2] + (b[2] - a[2]) * f
      brg = (a[3] + ((b[3] - a[3] + 180.0) % 360.0 - 180.0) * f) % 360.0      # shortest arc
      v = a[4] + (b[4] - a[4]) * f
      vv, vd = a[6] + (b[6] - a[6]) * f, a[7] + (b[7] - a[7]) * f
      clock[0] += dt
      rec = [p for p in paths if p[0] <= t][-1]
      ctrl._map_targets = [pt(x, y, rv) for x, y, rv in rec[1]]
      ctrl._cur_lat, ctrl._cur_lon, ctrl._cur_bearing = (*ll(e, nn), brg)
      ctrl._last_read = clock[0]
      ctrl._gps_fix_ts = clock[0] - 1.4
      vis["s"] = (C.A_LAT_TARGET / (vv * vv), vd, vv) if (vv > 0.0 and vd >= 0.0) else (0.0, -1.0, float("inf"))
      cap = ctrl.cap(sm, a[5], v)
      out.append(dict(t=t, v=v, set=a[5], cap=cap, state=ctrl._state, win=ctrl._tele_curve_win, raw=ctrl._tele_map_raw,
                      d=ctrl._tele_map_d, e=e, n=nn, brg=brg, path=rec[1], engaged=bool(a[10])))
  return out


def _binding_point_is_ahead(r):
  """independent of the mask: find the path node the fold selected (same raw target, at the reported distance) and check it does not lie BEHIND
  the car (projection on the heading above minus the mask's 5 m tolerance + 1 m). True when the map did not win."""
  if r["win"] != "map":
    return True
  h = (math.sin(math.radians(r["brg"])), math.cos(math.radians(r["brg"])))
  for x, y, rv in r["path"]:
    if abs(rv - r["raw"]) < 1e-6 and abs(math.hypot(x - r["e"], y - r["n"]) - r["d"]) < 1.5:
      return (x - r["e"]) * h[0] + (y - r["n"]) * h[1] > -(CES.ICBM_PASSED_TOL_M + 1.0)   # the mask's 5 m tolerance (+1 m slack) is by design
  raise AssertionError("the selected map point is not in the path record")


def test_replay_reproduces_the_recorded_ramp_slowdown_without_the_mask(monkeypatch):
  """the harness is only worth trusting if today's code reproduces the log: the recorded cap fell from 84 to ~43-44 mph and the binding
  point was behind the car (report 3.3)"""
  rows = replay_geo(monkeypatch, F.RAMP_TICKS, F.RAMP_PATHS, fix=False)
  assert 40.0 <= min(r["cap"] for r in rows) / MPH <= 46.0                                   # recorded: 43 mph at 22:44:54-55
  assert sum(1 for r in rows if not _binding_point_is_ahead(r)) > 200                        # >10 s of cycles bound on a point behind


def test_the_ramp_cap_never_drops_for_a_point_behind_the_car(monkeypatch):
  rows = replay_geo(monkeypatch, F.RAMP_TICKS, F.RAMP_PATHS, fix=True)
  behind = [r for r in rows if not _binding_point_is_ahead(r)]
  assert behind == []
  assert min(r["cap"] for r in rows) / MPH >= 70.0                                           # was 43; what is left is a node genuinely ahead
  late = [r for r in rows if r["t"] >= 6.0]                                                   # 22:44:51 on: the loop is behind, nothing ahead binds
  assert all(r["state"] != "brake" or r["win"] != "map" for r in late)
  assert max(r["cap"] for r in late) / MPH >= 89.0                                            # back at the 90 mph set


def test_the_receding_curve_never_latches_hold_in_the_reengage_window(monkeypatch):
  """report 3.5: the manual curve #16 apex passed at 22:49:47 and the receding point held VTSC in `hold` at 34 mph until 22:49:53"""
  monkeypatch.setattr(VC.VTSCController, "_drop_latched", lambda self: None)      # this test is Bug 1 only: the reset (Bug 2) is off
  old = replay_geo(monkeypatch, F.REENGAGE_TICKS, F.REENGAGE_PATHS, fix=False)
  assert sum(1 for r in old if r["state"] == "hold" and r["t"] >= 9.0) < 25                  # with the hold exit; without it: 93 cycles (4.6 s) latched
  new = replay_geo(monkeypatch, F.REENGAGE_TICKS, F.REENGAGE_PATHS, fix=True)
  assert all(_binding_point_is_ahead(r) for r in new)
  assert all(r["state"] != "hold" for r in new if r["t"] >= 9.0)
  assert all(r["cap"] >= o["cap"] - 1e-9 for r, o in zip(new, old, strict=True) if r["t"] >= 8.0)   # never lower than today after the apex


# ---------------------------------------------------------------- Bug 2: nothing latched while the driver drove applies on re-engage

def _engaged_cycles(rows):
  return [r for r in rows if r["engaged"]]


def test_reengage_window_today_applies_the_manual_curves_cap(monkeypatch):
  """today's code (no mask, no reset) reproduces report 3.5: the engage at 22:49:48.5 lands in `hold` at 34 mph while the car does 54"""
  monkeypatch.setattr(VC.VTSCController, "_drop_latched", lambda self: None)
  rows = replay_geo(monkeypatch, F.REENGAGE_TICKS, F.REENGAGE_PATHS, fix=False)
  eng = _engaged_cycles(rows)
  assert eng and eng[0]["t"] == pytest.approx(9.0, abs=DT)
  assert eng[0]["cap"] / MPH < 36.0 and eng[0]["state"] == "hold"                 # recorded: 35 mph hold applied on the stalk pull
  assert max(r["v"] for r in eng[:20]) / MPH > 53.0                               # ...at 54 mph


def test_the_mask_alone_still_leaves_a_stale_cap_on_engage(monkeypatch):
  """Bug 1's filter removes the receding point, but the cap already applied at the moment of the stalk pull (rising slowly out of `release`)
  is still stale: this is why the reset is needed as well"""
  monkeypatch.setattr(VC.VTSCController, "_drop_latched", lambda self: None)
  rows = replay_geo(monkeypatch, F.REENGAGE_TICKS, F.REENGAGE_PATHS, fix=True)
  first = _engaged_cycles(rows)[0]
  assert first["cap"] / MPH < 45.0 < first["set"] / MPH


def test_after_the_reengage_there_is_no_cap_from_the_manual_curve(monkeypatch):
  rows = replay_geo(monkeypatch, F.REENGAGE_TICKS, F.REENGAGE_PATHS, fix=True)
  eng = _engaged_cycles(rows)
  # the whole engaged part: cap == the 55 mph set. The one exception is the curve BRAIN's own rate-limited term (win == "brain"), which
  # eases up toward a set speed that jumped ~5 mph at the stalk pull (cruise off, the Tesla's set follows the car): <= 1.3 mph for a moment
  assert eng and all(r["cap"] == pytest.approx(r["set"]) or (r["win"] == "brain" and r["cap"] >= r["set"] - 0.6) for r in eng)
  assert all(r["state"] == "idle" and r["win"] != "map" for r in eng)
  assert all(r["state"] != "hold" for r in rows)
  assert all(r["state"] == "idle" for r in rows if not r["engaged"])               # cruise off: nothing is latched from the first cycle on


def test_the_ramp_replay_is_unchanged_by_the_reset(monkeypatch):
  """the ramp is an engaged window throughout: the reset must not fire, so the result is exactly the mask-only result"""
  a = replay_geo(monkeypatch, F.RAMP_TICKS, F.RAMP_PATHS, fix=True)
  monkeypatch.setattr(VC.VTSCController, "_drop_latched", lambda self: None)
  b = replay_geo(monkeypatch, F.RAMP_TICKS, F.RAMP_PATHS, fix=True)
  assert all(x["engaged"] for x in a[20:])
  assert [(x["cap"], x["state"]) for x in a[20:]] == [(x["cap"], x["state"]) for x in b[20:]]


def _scene(monkeypatch, fp="TESLA_MODEL_S_HW3", brand="tesla"):
  """a constant camera curve ahead (15 m/s apex 150 m ahead, car 30 m/s, set 35): braking from the third cycle on"""
  ctrl, clock = make_controller(monkeypatch, fp=fp, brand=brand)
  vis = {"s": (C.A_LAT_TARGET / (15.0 * 15.0), 150.0, 15.0)}
  monkeypatch.setattr(VC, "model_curve_state", lambda model, v_cruise, a_lat: vis["s"])
  ns = _NS()
  ns.orientationNED = [0.0, 0.0, 0.0]
  ns.enabled = True
  sm = {"modelV2": object(), "carControl": ns}

  def step(n=1, cruise_on=True, v=30.0, vset=35.0):
    ns.enabled = cruise_on
    cap = None
    for _ in range(n):
      clock[0] += DT
      cap = ctrl.cap(sm, vset, v)
    return cap
  return ctrl, step, ns, sm


@pytest.mark.parametrize("fp,brand", [("TESLA_MODEL_S_HW3", "tesla"), ("FORD_F_150_LIGHTNING_MK1", "ford")])
def test_the_state_machine_is_reset_on_the_engage_edge_and_the_first_engaged_cycle_has_no_cap(monkeypatch, log, fp, brand):
  ctrl, step, _, _ = _scene(monkeypatch, fp, brand)
  assert step(40) < 35.0 - 0.4                                     # engaged: braking for the curve (cap below the set)
  assert ctrl._state == "brake"
  step(1, cruise_on=False)                                       # the driver takes over (cruise off) ...
  assert ctrl._state == "idle" and ctrl._applied == pytest.approx(35.0)
  assert any("cruise off -- dropped state=brake" in m for m in log.lines("info"))
  assert step(1, cruise_on=True) == pytest.approx(35.0)          # ... and the very first engaged cycle applies nothing


def test_a_hold_reached_while_driving_by_hand_does_not_survive_the_engage(monkeypatch, log):
  ctrl, step, _, _ = _scene(monkeypatch)
  step(40)
  ctrl._state, ctrl._applied = "hold", 15.0                       # the report's latch: hold at 35 mph while the driver drives
  for _ in range(60):                                             # the apex recedes for 3 s with cruise off
    step(1, cruise_on=False, vset=25.0)
  assert ctrl._state == "idle" and ctrl._applied == pytest.approx(25.0)
  assert step(1, cruise_on=True, v=24.0, vset=25.0) == pytest.approx(25.0)


def test_the_release_freeze_state_is_cleared_too(monkeypatch, log):
  ctrl, step, _, _ = _scene(monkeypatch)
  step(5)
  ctrl._rel_latched, ctrl._rel_defer_t0, ctrl._rel_defer_capped = True, 123.0, True
  step(1, cruise_on=False)
  assert not ctrl._rel_latched and ctrl._rel_defer_t0 is None and not ctrl._rel_defer_capped


def test_a_hold_exits_to_brake_only_when_the_apex_clearly_moves_away(monkeypatch, log):
  ctrl, step, _, _ = _scene(monkeypatch)
  step(40)
  ctrl._state, ctrl._applied = "hold", 15.0

  def vis(d):
    monkeypatch.setattr(VC, "model_curve_state", lambda model, v_cruise, a_lat: (C.A_LAT_TARGET / 225.0, d, 15.0))
  vis(30.0)                                                        # apex 30 m ahead at 30 m/s: tta 1 s
  step(1)
  assert ctrl._state == "hold"                                     # close and still too fast: hold as today
  vis(80.0)                                                        # tta 2.7 s: just above HOLD_TTA_S, inside the margin (Fable F4: hovering apex)
  step(30)
  assert ctrl._state == "hold"
  vis(150.0)                                                       # tta 5 s: clearly receding
  step(C.HOLD_EXIT_CYCLES - 1)
  assert ctrl._state == "hold"                                     # debounced
  step(1)
  assert ctrl._state == "brake"


def test_a_hover_around_the_hold_edge_never_exits_the_hold(monkeypatch, log):
  ctrl, step, _, _ = _scene(monkeypatch)
  step(40)
  ctrl._state, ctrl._applied = "hold", 15.0
  for i in range(60):                                              # alternates far / near every 2 cycles: never HOLD_EXIT_CYCLES in a row
    d = 150.0 if (i // 2) % 2 == 0 else 60.0
    monkeypatch.setattr(VC, "model_curve_state", lambda model, v_cruise, a_lat, d=d: (C.A_LAT_TARGET / 225.0, d, 15.0))
    step(1)
    assert ctrl._state == "hold"


def test_no_reset_while_engaged(monkeypatch, log):
  ctrl, step, _, _ = _scene(monkeypatch)
  caps = [step(1) for _ in range(120)]                             # 6 s engaged, the curve stays ahead
  assert ctrl._state == "brake" and caps[-1] < 35.0 - 0.4
  assert caps[-1] <= caps[10]                                      # keeps braking / holding down: never reset back up
  assert not [m for m in log.lines("info") if "not engaged" in m]


def _gas_lift(monkeypatch, t_rel_tta, longactive_is_enabled_and_not_gas=True):
  """Fable F1, closed loop: set 35, car 30, curve-safe 20, apex 400 m ahead; the driver presses the accelerator at t=3 s (holding 31 m/s)
  and lifts off t_rel_tta seconds before the apex. carControl.enabled stays True during the override; carControl.longActive is False
  (controlsd.py:368). Returns (cap at lift-off, car speed at the apex)."""
  ctrl, clock = make_controller(monkeypatch)
  vis = {"s": None}
  monkeypatch.setattr(VC, "model_curve_state", lambda model, v_cruise, a_lat: vis["s"])
  ns = _NS()
  ns.orientationNED = [0.0, 0.0, 0.0]
  sm = {"modelV2": object(), "carControl": ns}
  v, d, t, gas, released, cap_rel = 30.0, 400.0, 0.0, False, False, None
  best = (1e9, v)
  while d > -50.0 and t < 60.0:
    tta = d / max(v, 1.0)
    if not gas and not released and t >= 3.0:
      gas = True
    if gas and tta <= t_rel_tta:
      gas, released = False, True
    ns.enabled = True
    ns.longActive = not gas
    vis["s"] = (C.A_LAT_TARGET / 400.0, d, 20.0) if d >= 0.0 else (0.0, -1.0, float("inf"))
    clock[0] += DT
    ctrl._last_read = clock[0]
    cap = ctrl.cap(sm, 35.0, v)
    if released and cap_rel is None:
      cap_rel = cap
    v += max(min(((31.0 if gas else min(cap, 35.0)) - v) / 1.0, 0.8), -1.5) * DT
    d -= v * DT
    t += DT
    if abs(d) < best[0]:
      best = (abs(d), v)
  return cap_rel, best[1]


@pytest.mark.parametrize("t_rel", [6.0, 3.0, 2.4, 1.5])
def test_a_gas_override_lift_off_near_the_apex_keeps_the_cap_that_was_ready(monkeypatch, log, t_rel):
  """the reset keys on cruise OFF (carControl.enabled), not longActive: a gas-override lift-off must behave exactly as without the reset"""
  cap_new, v_new = _gas_lift(monkeypatch, t_rel)
  monkeypatch.setattr(VC.VTSCController, "_drop_latched", lambda self: None)     # the base behaviour: no reset at all
  cap_old, v_old = _gas_lift(monkeypatch, t_rel)
  assert cap_new == pytest.approx(cap_old) and v_new == pytest.approx(v_old)
  assert cap_new < 35.0 - 1.0                                      # positive control: a cap really was ready at the lift-off


def test_the_accelerator_override_does_not_drop_the_state(monkeypatch, log):
  """carControl.enabled stays True during a gas override (longActive does not): nothing is dropped"""
  ctrl, step, ns, _ = _scene(monkeypatch)
  step(40)
  assert ctrl._state == "brake"
  ns.longActive = False
  step(5)
  assert ctrl._state == "brake"
  assert not [m for m in log.lines("info") if "cruise off" in m]


def test_the_curve_brain_slew_state_is_cleared_with_cruise_off(monkeypatch, log):
  ctrl, step, _, _ = _scene(monkeypatch)
  step(5)
  ctrl._cb_applied = 20.0
  step(1, cruise_on=False)
  assert ctrl._cb_applied == pytest.approx(35.0)                   # restarted from the set speed (the brain runs after the drop), not the stale 20


def test_cruise_off_with_a_twisty_trim_active_does_not_log_every_cycle(monkeypatch, log):
  ctrl, step, _, _ = _scene(monkeypatch)
  step(2, cruise_on=False)
  n0 = len(log.lines("info"))
  for i in range(100):                                             # the working cruise (twisty trim) moves every cycle with cruise off
    step(1, cruise_on=False, vset=35.0 - 0.01 * i)
  assert len(log.lines("info")) == n0
  step(40)
  step(1, cruise_on=False)
  assert len([m for m in log.lines("info") if "cruise off -- dropped" in m]) == 1     # a real drop still logs once


def test_nothing_is_logged_when_there_is_nothing_to_drop(monkeypatch, log):
  ctrl, step, _, _ = _scene(monkeypatch)
  step(2, cruise_on=False)                                       # driving by hand from the start, state idle
  step(2, cruise_on=False)
  assert not [m for m in log.lines("info") if "not engaged" in m]


def test_an_unreadable_cruise_on_changes_nothing_and_is_loud(monkeypatch, log):
  ctrl, step, ns, sm = _scene(monkeypatch)
  step(40)
  del ns.enabled
  for _ in range(5):
    ctrl.cap(sm, 35.0, 30.0)
  assert ctrl._state == "brake"                                    # today's behaviour: treated as engaged, nothing cleared
  errs = [m for m in log.lines("exception") if "enabled unreadable (AttributeError)" in m]
  assert len(errs) == 1 and "NOT cleared" in errs[0]                # once (rate limited), and it says what it costs
