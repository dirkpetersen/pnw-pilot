"""dbfirst2pnw (owner design 2026-10-01): VTSC's map minimum-slowdown notch, DB first.

  * a path point the curve DB COVERS (the curve brain's coverage says 1) is skipped by the OSM fold -- the DB and vision decide;
  * a point it does not cover (0) gets min(its scaled target, UNCOVERED_CURVE_LIMIT_RATIO x the posted limit) instead of the flat notch;
  * posted limit unknown, a point the brain did not classify, no / stale / non-acting brain entry, the switch off, any other car: TODAY.

Pinned through most_binding_map_curve (pure) and the REAL VTSCController.cap() with a fake CurveBrain param. No private data: the DB itself is
exercised in ces_pnw/tests/test_dbfirst_coverage_pnw.py with a synthetic table; here the brain's published coverage is a literal."""
import json
import math
import random

import pytest

from openpilot.selfdrive.controls.lib.vtsc_pnw import vtsc_constants as C
from openpilot.selfdrive.controls.lib.vtsc_pnw import vtsc_controller as VC
from openpilot.selfdrive.controls.lib.vtsc_pnw import vtsc_pnw as VP
from openpilot.selfdrive.controls.lib.vtsc_pnw.tests import release_later_harness as H

MPH = 0.44704
SET = 40.23            # 90 mph
LIMIT = 31.29          # 70 mph, the 2026-10-01 15:24 road
CAP = LIMIT * C.UNCOVERED_CURVE_LIMIT_RATIO
LAT0, LON0, M_PER_DEG = H.LAT0, H.LON0, H.M_PER_DEG
LIGHTNING = dict(fp="FORD_F_150_LIGHTNING_MK1", brand="ford")


def _pts(*dv):
  return [{"latitude": LAT0 + d / M_PER_DEG, "longitude": LON0, "velocity": v} for d, v in dv]


def _key(p):
  return (float(p["latitude"]), float(p["longitude"]))


def _mbmc(pts, v_ego=38.0, **kw):
  return VP.most_binding_map_curve(pts, LAT0, LON0, v_ego, 500.0, C.A_DECEL, C.APEX_FINISH_S, C.SHARP_CURVE_V, C.MAP_SPEED_SCALE, SET,
                                   C.MAP_MIN_SLOWDOWN, 0.0, **kw)


FLAGGED = _pts((273.0, 30.4))            # raw 30.4 m/s: mapd flags it, the MTSC scale erases it -> today the flat notch (set - 4.5)
NOTCH = SET - C.MAP_MIN_SLOWDOWN


def test_the_ratio_is_the_owners_15_percent():
  assert C.UNCOVERED_CURVE_LIMIT_RATIO == 1.15


# ---------------------------------------------------------------- most_binding_map_curve (pure)

def test_covered_flagged_point_is_skipped():
  info = {}
  r = _mbmc(FLAGGED, db_cov={_key(FLAGGED[0]): 1}, uncov_cap=CAP, info=info)
  assert r[0] == 0.0 and r[4] is False                       # nothing from OSM for this curve
  assert (info["src"], info["cov"], info["n_db"], info["capped"]) == ("db", "1", 1, False)


def test_uncovered_flagged_point_gets_the_posted_limit_cap_not_the_notch():
  info = {}
  v, d, sharp, raw, floored = _mbmc(FLAGGED, db_cov={_key(FLAGGED[0]): 0}, uncov_cap=CAP, info=info)
  assert v == pytest.approx(CAP) and floored is True and sharp is False and raw == 30.4 and abs(d - 273.0) < 1.0
  assert (info["src"], info["cov"], info["capped"]) == ("cap15", "0", True)


def test_uncovered_and_limit_unknown_is_todays_notch():
  info = {}
  v = _mbmc(FLAGGED, db_cov={_key(FLAGGED[0]): 0}, uncov_cap=None, info=info)[0]
  assert v == NOTCH and (info["src"], info["cov"], info["capped"]) == ("notch", "0", False)


def test_a_covered_but_unreliable_point_keeps_todays_notch_with_a_reason():
  info = {}
  assert _mbmc(FLAGGED, db_cov={_key(FLAGGED[0]): 2}, uncov_cap=CAP, info=info)[0] == NOTCH
  assert (info["src"], info["cov"], info["capped"]) == ("notch", "u", False)


@pytest.mark.parametrize("state, label", [(3, "r"), (4, "b")])
def test_a_driven_but_unmeasured_point_keeps_todays_notch_not_the_cap(state, label):
  """Owner 2026-10-01: a road DRIVEN where the table refused the row (3) / never recorded this branch (4) is not 'never driven': NOT the 1.15 x limit cap."""
  info = {}
  assert _mbmc(FLAGGED, db_cov={_key(FLAGGED[0]): state}, uncov_cap=CAP, info=info)[0] == NOTCH
  assert (info["src"], info["cov"], info["capped"]) == ("notch", label, False)


def test_an_unclassified_point_is_todays_notch():
  info = {}
  assert _mbmc(FLAGGED, db_cov={}, uncov_cap=CAP, info=info)[0] == NOTCH
  assert (info["src"], info["cov"]) == ("notch", "?")


def test_a_cap_above_the_set_speed_slows_nothing():
  """Set 90 on an 80 mph road: 1.15 x 80 = 92 mph > set -> the curve gets no slowdown (the set speed is already below the cap)."""
  v = _mbmc(FLAGGED, db_cov={_key(FLAGGED[0]): 0}, uncov_cap=1.15 * 35.76)[0]
  assert v >= SET - 0.5 and v <= SET + 1e-9                  # the scaled+clamped target, no floor -> the caller's gate drops it


def test_the_cap_never_exceeds_the_scaled_target():
  rng = random.Random(1001)
  for _ in range(200):
    d, raw = rng.uniform(40.0, 480.0), rng.uniform(8.0, 44.0)
    pts = _pts((d, raw))
    scaled = min(raw * C.tiered_map_scale(raw), SET)
    cap = rng.uniform(10.0, 50.0)
    v = _mbmc(pts, db_cov={_key(pts[0]): 0}, uncov_cap=cap)[0]
    assert v <= scaled + 1e-9


def test_a_real_osm_curve_under_the_notch_is_unchanged_when_uncovered_and_skipped_when_covered():
  pts = _pts((273.0, 14.0))                                   # raw 14 m/s: scaled 19 -> a real slowdown, not a notch
  today = _mbmc(pts)
  info = {}
  assert _mbmc(pts, db_cov={_key(pts[0]): 0}, uncov_cap=CAP, info=info) == today and info["src"] == "osm"
  assert _mbmc(pts, db_cov={_key(pts[0]): 1}, uncov_cap=CAP)[0] == 0.0


def test_no_points_still_fills_info_and_the_controller_survives_an_empty_path(monkeypatch):
  """Found by the corpus replay: the no-data early return left `info` empty and the controller's telemetry line raised OUTSIDE its try."""
  info = {}
  assert _mbmc([], db_cov={}, uncov_cap=CAP, info=info) == (0.0, float("inf"), False, 0.0, False)
  assert info == dict(src="", cov="", capped=False, n_db=0, db_d=float("inf"))
  r = Rig(monkeypatch, pts=[])
  r.pts = []
  for _ in range(5):
    r.mem.entry = {"ts": r.clock[0], "mode": "lower", "v": None, "d": 0.0, "cov": []}
    r.tick()
  assert r.ctrl._tele_map_err == "" and r.ctrl.overlay_payload()["mapSrc"] == ""


def test_db_cov_none_and_all_unknown_are_byte_identical_to_today():
  rng = random.Random(77)
  for _ in range(300):
    pts = _pts(*[(rng.uniform(-30.0, 520.0), rng.choice([0.0, 8.0, 14.0, 18.0, 22.0, 25.0, 28.0, 31.0, 35.0, 45.0, 80.0]))
                 for _ in range(rng.randint(0, 12))])
    v_ego = rng.uniform(5.0, 42.0)
    today = _mbmc(pts, v_ego)
    assert _mbmc(pts, v_ego, db_cov=None, uncov_cap=CAP) == today
    assert _mbmc(pts, v_ego, db_cov={}, uncov_cap=None) == today                      # unknown everywhere + no limit = today
    assert _mbmc(pts, v_ego, db_cov={_key(p): 0 for p in pts}, uncov_cap=None) == today   # uncovered but no limit known = today


def test_covered_points_never_add_slowing():
  rng = random.Random(78)
  for _ in range(300):
    pts = _pts(*[(rng.uniform(5.0, 520.0), rng.choice([8.0, 14.0, 22.0, 28.0, 31.0, 35.0, 45.0])) for _ in range(rng.randint(1, 10))])
    cov = {_key(p): rng.choice([0, 1]) for p in pts}
    v_ego = rng.uniform(15.0, 42.0)
    old = _mbmc(pts, v_ego)
    new = _mbmc(pts, v_ego, db_cov=dict.fromkeys(cov, 1), uncov_cap=CAP)
    env_old = VP.brake_cap_for_apex(old[0], old[1], v_ego) if old[0] > 0 else float("inf")
    env_new = VP.brake_cap_for_apex(new[0], new[1], v_ego) if new[0] > 0 else float("inf")
    assert env_new >= env_old - 1e-9


# ---------------------------------------------------------------- the real controller

class Mem:
  """The /dev/shm params: CurveBrain (the brain's entry, or None) and nothing else."""
  def __init__(self):
    self.entry = None
    self.raw = None

  def get(self, k, return_default=False):
    if k != "CurveBrain":
      return None
    return self.raw if self.raw is not None else (None if self.entry is None else json.dumps(self.entry))

  def put_nonblocking(self, k, v):
    pass


class Rig:
  def __init__(self, monkeypatch, db_first=True, limit=LIMIT, pts=None, **kw):
    self.ctrl, self.clock = H.make_controller(monkeypatch, notch_vego=True, **kw)
    if self.ctrl.veh.curve_brain_vtsc:
      self.ctrl.veh._tesla_curve_cfg["db_first"] = db_first
    self.mem = Mem()
    self.ctrl.mem_params = self.mem
    self.vis = {"s": (0.0, -1.0, float("inf"))}
    monkeypatch.setattr(VC, "model_curve_state", lambda model, v_cruise, a_lat: self.vis["s"])
    ns = H._NS()
    ns.orientationNED = [0.0, 0.0, 0.0]
    ns.enabled = True
    self.sm = {"modelV2": object(), "carControl": ns}
    self.pts = FLAGGED if pts is None else pts
    self.limit = limit

  def entry(self, cov, mode="lower", ts=None, v=None):
    self.mem.entry = {"ts": self.clock[0] if ts is None else ts, "mode": mode, "v": v, "d": 0.0}
    if cov is not None:
      self.mem.entry["cov"] = [[p["latitude"], p["longitude"], c] for p, c in zip(self.pts, cov, strict=True)]

  def tick(self, v_ego=38.0, v_set=SET, y=0.0, dt=0.05):
    c = self.ctrl
    self.clock[0] += dt
    c._map_targets = self.pts
    c._cur_lat, c._cur_lon, c._cur_bearing = LAT0 + y / M_PER_DEG, LON0, 0.0
    c._last_read = self.clock[0]
    c._gps_fix_ts = self.clock[0] - 1.4
    c._speed_limit, c._is_freeway = self.limit, True
    return c.cap(self.sm, v_set, v_ego)


def _run(rig, n=60, cov=None, move=False, v_ego=38.0, **kw):
  """n ticks. move=True: the car drives north at v_ego from y = 0, so the flagged point (273 m) is approached and passed."""
  out = []
  for i in range(n):
    if cov is not False:
      rig.entry(cov)
    out.append(rig.tick(v_ego=v_ego, y=(i * 0.05 * v_ego if move else kw.pop("y", 0.0)), **kw))
  return out


def test_covered_point_no_osm_slowdown_and_telemetry(monkeypatch):
  r = Rig(monkeypatch)
  caps = _run(r, cov=[1])
  assert min(caps) == pytest.approx(SET, abs=1e-6)
  p = r.ctrl.overlay_payload()
  assert (p["mapSrc"], p["mapCov"], p["mapCap15"]) == ("db", "1", round(CAP, 2)) and p["curveWin"] != "map"


def test_uncovered_point_binds_at_the_posted_limit_cap(monkeypatch):
  r = Rig(monkeypatch)
  caps = _run(r, n=125, cov=[0], v_ego=SET, move=True)
  assert min(caps) == pytest.approx(CAP, abs=0.05)                         # 80.5 mph vs today's 80.0: the owner's comparison
  p = r.ctrl.overlay_payload()
  assert (p["mapSrc"], p["mapCov"], p["mapCap15"]) == ("cap15", "0", round(CAP, 2)) and p["curveWin"] == "map"


def test_the_cap_passes_the_meaningful_slowdown_gate_at_a_confidence_cut_not_4_5(monkeypatch):
  """1.15 x 70 mph = 80.5 mph is only 3.9 m/s... no: set - cap = 40.23 - 35.99 = 4.24 m/s < MAP_MIN_SLOWDOWN. Without the gate relaxation the
  cap would be dropped as 'not a meaningful slowdown' and the owner's rule would never bind on a 70 limit at a 90 set."""
  assert SET - CAP < C.MAP_MIN_SLOWDOWN
  r = Rig(monkeypatch)
  assert min(_run(r, n=125, cov=[0], v_ego=SET, move=True)) < SET - 3.0


def test_a_cap_that_does_not_bind_is_not_a_slowdown_the_gate_lets_through(monkeypatch):
  """Raw 23 m/s is flagged (below the notch) and its scaled target is ~37.4 m/s, only ~2.8 m/s under the set. On an 80 mph road the cap
  (92 mph) is above that: no cap bound, so the point is judged at MAP_MIN_SLOWDOWN like any other and drops out -- the ~2.8 m/s trim that the
  CONFIDENCE_CUT gate would have let through must NOT happen (the cap only slows where it binds)."""
  pts = _pts((330.0, 23.0))
  scaled = min(23.0 * C.tiered_map_scale(23.0), SET)
  assert NOTCH < scaled < SET - C.CONFIDENCE_CUT
  r = Rig(monkeypatch, limit=35.76, pts=pts)
  caps = _run(r, n=125, cov=[0], v_ego=SET, move=True)
  assert min(caps) == pytest.approx(SET - C.CONFIDENCE_CUT, abs=1e-6) or min(caps) >= SET - 0.6   # at most the engage cue, no map slowdown
  assert r.ctrl.overlay_payload()["curveWin"] != "map"


def test_covered_unreliable_through_the_controller_keeps_the_notch_and_names_it(monkeypatch):
  r = Rig(monkeypatch)
  caps = _run(r, n=125, cov=[2], v_ego=SET, move=True)
  assert min(caps) == pytest.approx(NOTCH, abs=0.05)
  p = r.ctrl.overlay_payload()
  assert (p["mapSrc"], p["mapCov"]) == ("notch", "u")


def test_limit_unknown_keeps_the_flat_notch(monkeypatch):
  r = Rig(monkeypatch, limit=0.0)
  caps = _run(r, n=125, cov=[0], v_ego=SET, move=True)
  assert min(caps) == pytest.approx(NOTCH, abs=0.05)
  assert r.ctrl.overlay_payload()["mapSrc"] == "notch" and r.ctrl.overlay_payload()["mapCap15"] is None


@pytest.mark.parametrize("entry", ["absent", "stale", "shadow", "bad", "no-cov-key"])
def test_no_usable_coverage_is_todays_notch(monkeypatch, entry):
  r = Rig(monkeypatch)
  caps = []
  for i in range(125):
    if entry == "stale":
      r.entry([0], ts=r.clock[0] - 5.0)
    elif entry == "shadow":
      r.entry([0], mode="shadow")
    elif entry == "bad":
      r.mem.raw = "{not json"
    elif entry == "no-cov-key":
      r.entry(None)
    caps.append(r.tick(v_ego=SET, y=i * 0.05 * SET))
  assert min(caps) == pytest.approx(NOTCH, abs=0.05)
  p = r.ctrl.overlay_payload()
  assert p["mapCov"] == "" and p["mapSrc"] == ""


def test_the_brain_mode_off_in_this_process_is_todays_notch(monkeypatch):
  r = Rig(monkeypatch)
  r.ctrl.veh._tesla_curve_cfg["curve_brain"] = "shadow"
  caps = _run(r, n=125, cov=[1], v_ego=SET, move=True)                          # the entry says covered -- but VTSC is not acting on the brain
  assert min(caps) == pytest.approx(NOTCH, abs=0.05)


def test_switch_off_is_todays_notch_and_no_telemetry(monkeypatch):
  r = Rig(monkeypatch, db_first=False)
  caps = _run(r, n=125, cov=[1], v_ego=SET, move=True)
  assert min(caps) == pytest.approx(NOTCH, abs=0.05)
  p = r.ctrl.overlay_payload()
  assert (p["mapSrc"], p["mapCov"], p["mapCap15"]) == ("", "", None)


def test_switch_off_logs_nothing_about_coverage(monkeypatch):
  lines = []
  monkeypatch.setattr(VC.cloudlog, "info", lambda msg, *a, **k: lines.append(msg))
  monkeypatch.setattr(VC.cloudlog, "warning", lambda msg, *a, **k: lines.append(msg))
  r = Rig(monkeypatch, db_first=False)
  _run(r, n=40, cov=[1])
  assert not [x for x in lines if "db-first" in x]


def test_the_lightning_is_exactly_today(monkeypatch):
  r = Rig(monkeypatch, **LIGHTNING)
  assert r.ctrl.veh.vtsc_db_first is False
  caps = _run(r, n=125, cov=[1], v_ego=SET, move=True)             # a coverage entry, which the Lightning never reads...
  bare = _run(Rig(monkeypatch, **LIGHTNING), n=125, cov=False, v_ego=SET, move=True)
  assert caps == bare and min(caps) < SET - 4.0                      # ...so its caps are bit-for-bit today's (its own curve penalty included)
  assert r.ctrl.overlay_payload()["mapSrc"] == ""


def test_an_unclassified_point_among_classified_ones_is_todays_notch(monkeypatch):
  r = Rig(monkeypatch)
  r.pts = FLAGGED + _pts((90.0, 40.0))
  caps = []
  for i in range(125):
    r.mem.entry = {"ts": r.clock[0], "mode": "lower", "v": None, "d": 0.0,
                   "cov": [[r.pts[1]["latitude"], r.pts[1]["longitude"], 1]]}      # the flagged point is NOT in the brain's list
    caps.append(r.tick(v_ego=SET, y=i * 0.05 * SET))
  assert min(caps) == pytest.approx(NOTCH, abs=0.05)


def test_coverage_state_changes_are_logged_once_each(monkeypatch):
  lines = []
  monkeypatch.setattr(VC.cloudlog, "info", lambda msg, *a, **k: lines.append(msg % a if a else msg))
  monkeypatch.setattr(VC.cloudlog, "warning", lambda msg, *a, **k: lines.append("W " + (msg % a if a else msg)))
  r = Rig(monkeypatch)
  _run(r, n=40, cov=[1])
  _run(r, n=40, cov=[1])
  _run(r, n=40, cov=False)                           # entry goes stale -> one warning
  for _ in range(40):
    r.entry([0], ts=r.clock[0] - 5.0)
    r.tick()
  db = [x for x in lines if "db-first" in x]
  assert sum("coverage of 1 path points" in x for x in db) == 1
  assert sum("NO curve-DB coverage (stale)" in x for x in db) >= 1 and all(x.startswith("W ") for x in db if "(stale)" in x)
  assert len(db) <= 4                                # change-only: never one line per tick


# ---------------------------------------------------------------- 2026-10-01 15:24 PT, closed loop (the numbers of the drive)

def _closed(monkeypatch, cov, db_first, limit=LIMIT):
  """Car at 85 mph, set 90, a flagged map node 330 m ahead on a 70 mph road; the road really needs 1.1 m/s^2 at 90 (k = 1.1 / 40.23^2 =
  0.00068, R ~1470 m: the 15:24 bend). The car follows min(cap, set) with a 1 s lag. Returns (min speed, peak lateral demand)."""
  r = Rig(monkeypatch, db_first=db_first, limit=limit, pts=_pts((330.0, 30.4)))
  k = 1.1 / SET ** 2
  v, y, lo, peak = 38.0, 0.0, 1e9, 0.0
  for _ in range(int(25.0 / 0.05)):
    r.entry(cov)
    cap = r.tick(v_ego=v, y=y)
    v = max(v + max(min((min(cap, SET) - v) / 1.0, 0.8), -2.0) * 0.05, 1.0)
    y += v * 0.05
    if 250.0 < y < 480.0:
      lo, peak = min(lo, v), max(peak, v * v * k)
  return lo, peak


def test_the_1524_bend_today_it_cost_the_car_ten_mph_covered_it_costs_nothing(monkeypatch):
  today, _ = _closed(monkeypatch, [1], db_first=False)
  covered, peak = _closed(monkeypatch, [1], db_first=True)
  assert today < 38.0 - 2.5                                            # the notch dragged an 85 mph car to ~79 mph (the drive: 82)
  assert covered >= 38.0 - 0.8                                         # DB-covered: only the CONFIDENCE_CUT dip, if any
  assert peak < 0.5 * 3.5886 + 0.4                                     # and the bend needed ~1.1 m/s^2 -- a third of the car's 3.59


def test_the_same_bend_uncovered_is_held_to_the_posted_limit_cap_never_below_it(monkeypatch):
  lo, _ = _closed(monkeypatch, [0], db_first=True)
  assert lo >= CAP - 0.6 and lo < 38.0 - 1.5                           # slowed to ~80.5 mph (not the notch's 80.0, not deeper)


def test_a_posted_limit_below_the_cap_everywhere_still_caps_at_1_15_times_it(monkeypatch):
  lim = 24.6                                                            # 55 mph
  lo, _ = _closed(monkeypatch, [0], db_first=True, limit=lim)
  assert lo >= 1.15 * lim - 0.8 and lo < 1.15 * lim + 1.5


# ---------------------------------------------------------------- unknown coverage == switch off, over the recorded frames

def _frames(v_ego, v_set, pts, seconds=9):
  return [(f"10:00:{s:02d}", v_ego, v_set, 0.0, -1.0, None, "idle", [(d - v_ego * s, v) for d, v in pts]) for s in range(seconds + 1)]


_NEW_KEYS = ("mapSrc", "mapCov", "mapCap15")


def _strip(pay):
  return {k: v for k, v in pay.items() if k not in _NEW_KEYS}


def _replay_with_switch(monkeypatch, fr, db_first):
  """H.replay builds its controller internally, so the switch is forced at construction through the capability config default."""
  orig = H.make_controller

  def mk(mp, **kw):
    ctrl, clock = orig(mp, **kw)
    if ctrl.veh.curve_brain_vtsc:
      ctrl.veh._tesla_curve_cfg["db_first"] = db_first
    return ctrl, clock
  monkeypatch.setattr(H, "make_controller", mk)
  try:
    return H.replay(monkeypatch, fr, notch_vego=True)
  finally:
    monkeypatch.setattr(H, "make_controller", orig)


def test_switch_on_without_coverage_is_byte_identical_to_switch_off_over_fuzz(monkeypatch):
  """The switch is set BEFORE the replay runs (Opus review: the first version flipped it after H.replay returned and compared ON with ON)."""
  rng = random.Random(4242)
  for _ in range(20):
    v_set = rng.uniform(20.0, 42.0)
    v_ego = rng.uniform(8.0, v_set + 2.0)
    pts = [(rng.uniform(40.0, 480.0), rng.uniform(10.0, 45.0)) for _ in range(rng.randint(1, 6))]
    fr = _frames(v_ego, v_set, pts, seconds=6)
    off = _replay_with_switch(monkeypatch, fr, False)
    on = _replay_with_switch(monkeypatch, fr, True)             # ON (the default), nothing publishes coverage
    assert off[0]["ctrl"].veh.vtsc_db_first is False and on[0]["ctrl"].veh.vtsc_db_first is True
    assert [o["cap"] for o in on] == [o["cap"] for o in off]
    assert [(o["state"], o["win"], _strip(o["pay"])) for o in on] == [(o["state"], o["win"], _strip(o["pay"])) for o in off]
    assert all(o["pay"]["mapSrc"] == "" for o in on) and not math.isnan(sum(o["cap"] for o in on))
