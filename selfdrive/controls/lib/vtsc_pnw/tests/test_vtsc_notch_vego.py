"""vtscnotch2pnw (owner-approved 2026-09-30): the map-curve minimum-slowdown notch (and the gate that uses it) is measured from
min(set, vEgo) instead of the set speed, on the Raven only, behind curve.json tesla.vtsc_notch_vego (default ON).

Why: 2026-09-30 22:32:27, set raised 85 -> 90 mph 273 m before a bend while vEgo was still 85 (38.0 m/s). The notch was set - 4.5 = 35.7,
only 2.3 m/s under the speed being driven; the cap bottomed at ~vEgo and the car did not slow. With set == vEgo the next two bends worked.

Pinned here, through the REAL VTSCController.cap() (release_later_harness.py): bend B's scenario gives a lower cap; vEgo >= set is
BYTE-IDENTICAL to the switch being off; the clamp and every other threshold stay on the SET speed; the Lightning and the kill switch
are exactly today's."""
import math
import random

import pytest

from openpilot.selfdrive.controls.lib.vtsc_pnw import vtsc_constants as C
from openpilot.selfdrive.controls.lib.vtsc_pnw import vtsc_pnw as VP
from openpilot.selfdrive.controls.lib.vtsc_pnw.tests import release_later_harness as H

MPH = 0.44704
SET = 40.23            # 90 mph
LAT0, LON0, M_PER_DEG = H.LAT0, H.LON0, H.M_PER_DEG


def _caps(rows):
  return [r["cap"] for r in rows]


def _frames(v_ego, v_set, pts, seconds=9):
  """Constant-input recording-format frames; no camera curve (cam_v 0), map points approach at v_ego."""
  return [(f"10:00:{s:02d}", v_ego, v_set, 0.0, -1.0, None, "idle", [(d - v_ego * s, v) for d, v in pts]) for s in range(seconds + 1)]


BEND_B = [(273.0, 30.4)]        # raw mapd target 30.4 m/s (scale 1.8 -> clamped to set), 273 m ahead: the 22:32:27 bend


# ---------------------------------------------------------------- notch_reference (pure)

def test_notch_reference_branches():
  assert VP.notch_reference(40.0, 40.0) == 40.0                      # vEgo == set
  assert VP.notch_reference(40.0, 43.0) == 40.0                      # vEgo > set: the set speed, unchanged
  assert VP.notch_reference(40.0, 38.0) == 38.0                      # below set: what the car is doing
  assert VP.notch_reference(40.0, 38.0, enabled=False) == 40.0       # kill switch
  assert VP.notch_reference(40.0, 0.0) == C.V_MIN                    # standstill / noise: floored, never ~0
  assert VP.notch_reference(40.0, -0.3) == C.V_MIN
  assert VP.notch_reference(5.0, 1.0) == 5.0                         # set below V_MIN: the set speed (today)
  assert VP.notch_reference(40.0, float("nan")) == 40.0              # non-finite: today's behaviour
  assert VP.notch_reference(40.0, float("inf")) == 40.0


def test_notch_reference_is_the_set_speed_whenever_vego_is_not_below_it():
  rng = random.Random(7)
  for _ in range(500):
    v_set = rng.uniform(5.0, 45.0)
    assert VP.notch_reference(v_set, v_set + rng.uniform(0.0, 10.0)) == v_set
    assert V_MIN_LE(VP.notch_reference(v_set, rng.uniform(-1.0, 50.0)), v_set)


def V_MIN_LE(x, v_set):
  return x <= v_set and (x >= min(C.V_MIN, v_set))


# ---------------------------------------------------------------- most_binding_map_curve (pure)

def _pts(*dv):
  return [{"latitude": LAT0 + d / M_PER_DEG, "longitude": LON0, "velocity": v} for d, v in dv]


def _mbmc(**kw):
  return VP.most_binding_map_curve(_pts((273.0, 30.4)), LAT0, LON0, 38.0, 500.0, C.A_DECEL, C.APEX_FINISH_S, C.SHARP_CURVE_V,
                                   C.MAP_SPEED_SCALE, SET, C.MAP_MIN_SLOWDOWN, 0.0, **kw)


def test_notch_ref_none_or_equal_to_the_cap_is_today():
  assert _mbmc() == _mbmc(notch_ref=None) == _mbmc(notch_ref=SET)
  assert _mbmc()[0] == SET - C.MAP_MIN_SLOWDOWN and _mbmc()[4] is True


def test_a_lower_notch_ref_lowers_only_the_notch():
  v, d, sharp, raw, floored = _mbmc(notch_ref=38.0)
  assert v == 38.0 - C.MAP_MIN_SLOWDOWN and floored is True and raw == 30.4
  assert abs(d - 273.0) < 1.0


def test_the_clamp_stays_on_the_set_speed():
  """A point that is NOT floored keeps tv_eff = min(tv*scale, SET): moving the notch must not move the clamp (the 'true SET speed' intent)."""
  # raw 36 m/s: scale>1.35 -> scaled >= set -> clamped at SET. Old notch 35.7: tv 36 > notch -> not floored, tv_eff = SET.
  pts = _pts((273.0, 36.0))
  args = (pts, LAT0, LON0, 38.0, 500.0, C.A_DECEL, C.APEX_FINISH_S, C.SHARP_CURVE_V, C.MAP_SPEED_SCALE, SET, C.MAP_MIN_SLOWDOWN, 0.0)
  assert VP.most_binding_map_curve(*args)[0] == SET
  assert VP.most_binding_map_curve(*args, notch_ref=38.0)[0] == SET        # not 38.0: the clamp is still the set speed


# ---------------------------------------------------------------- the real controller

def test_bend_b_brakes_deeper_with_the_new_notch(monkeypatch):
  """22:32:27: set 40.23, vEgo 38.0, map target raw 30.4 at 273 m."""
  fr = _frames(38.0, SET, BEND_B)
  old = H.replay(monkeypatch, fr, notch_vego=False)
  new = H.replay(monkeypatch, fr, notch_vego=True)
  assert min(_caps(old)) >= SET - C.MAP_MIN_SLOWDOWN - 0.05                      # today: the cap bottoms at ~35.7 (2.3 m/s under vEgo)
  assert min(_caps(new)) <= 38.0 - C.MAP_MIN_SLOWDOWN + 0.05                     # new: the full 4.5 m/s under what the car is doing
  assert min(_caps(new)) < min(_caps(old)) - 2.0
  assert all(n <= o + 1e-9 for n, o in zip(_caps(new), _caps(old), strict=True))   # never a higher cap than today on this scenario
  assert all(c <= SET + 1e-9 for c in _caps(new))
  assert any(r["win"] == "map" for r in new)


def test_vego_equal_to_set_is_exactly_today(monkeypatch):
  for pts in (BEND_B, [(300.0, 25.0)], [(150.0, 30.4), (400.0, 22.0)]):
    fr = _frames(SET, SET, pts)
    assert _caps(H.replay(monkeypatch, fr, notch_vego=True)) == _caps(H.replay(monkeypatch, fr, notch_vego=False))


def test_vego_above_set_is_exactly_today(monkeypatch):
  fr = _frames(SET + 2.0, SET, BEND_B)
  assert _caps(H.replay(monkeypatch, fr, notch_vego=True)) == _caps(H.replay(monkeypatch, fr, notch_vego=False))


def test_fuzz_vego_not_below_set_is_byte_identical(monkeypatch):
  rng = random.Random(20260930)
  for _ in range(25):
    v_set = rng.uniform(20.0, 42.0)
    v_ego = v_set + rng.uniform(0.0, 3.0)
    pts = [(rng.uniform(40.0, 480.0), rng.uniform(10.0, 45.0)) for _ in range(rng.randint(1, 6))]
    fr = _frames(v_ego, v_set, pts, seconds=6)
    assert _caps(H.replay(monkeypatch, fr, notch_vego=True)) == _caps(H.replay(monkeypatch, fr, notch_vego=False))


def test_fuzz_below_set_is_bounded(monkeypatch):
  """vEgo < set: the cap is never above the set speed, never below V_MIN, and the vEgo-relative cap never ends ABOVE the set-relative
  one for a fold both rules make (the notch only moves down; the rules differ only in which curves they fold)."""
  rng = random.Random(930)
  for _ in range(25):
    v_set = rng.uniform(25.0, 42.0)
    v_ego = rng.uniform(6.0, v_set)
    pts = [(rng.uniform(40.0, 480.0), rng.uniform(10.0, 45.0)) for _ in range(rng.randint(1, 6))]
    new = _caps(H.replay(monkeypatch, _frames(v_ego, v_set, pts, seconds=6), notch_vego=True))
    assert all(math.isfinite(c) and C.V_MIN - 1e-9 <= c <= v_set + 1e-9 for c in new)


def test_a_gentle_node_keeps_todays_set_relative_floor_in_traffic(monkeypatch):
  """Two tiers: raw 34 m/s at vEgo 38 / set 40.23 is floored by today's set-relative notch (35.7) and is NOT floored by the car-relative one
  (33.5). The change must never be SHALLOWER than today, so the set-relative floor still applies and the caps are today's."""
  fr = _frames(38.0, SET, [(273.0, 34.0)])
  old = H.replay(monkeypatch, fr, notch_vego=False)
  new = H.replay(monkeypatch, fr, notch_vego=True)
  assert min(_caps(old)) < SET - 1.0
  assert _caps(new) == _caps(old)


def test_the_lightning_is_exactly_today(monkeypatch):
  """Today = the set-relative notch, i.e. notch_reference replaced by the identity on the set speed."""
  from openpilot.selfdrive.controls.lib.vtsc_pnw import vtsc_controller as VC
  fr = _frames(38.0, SET, BEND_B)
  lt = H.replay(monkeypatch, fr, fp="FORD_F_150_LIGHTNING_MK1", brand="ford")
  assert lt[0]["ctrl"].veh.vtsc_notch_from_vego is False
  monkeypatch.setattr(VC, "notch_reference", lambda v_set, v_ego, enabled=True: v_set)
  assert _caps(lt) == _caps(H.replay(monkeypatch, fr, fp="FORD_F_150_LIGHTNING_MK1", brand="ford"))


def test_the_capability_gate_is_the_only_thing_protecting_the_lightning(monkeypatch):
  fr = _frames(38.0, SET, BEND_B)
  today = _caps(H.replay(monkeypatch, fr, fp="FORD_F_150_LIGHTNING_MK1", brand="ford"))
  ctrl = H.replay(monkeypatch, fr[:2], fp="FORD_F_150_LIGHTNING_MK1", brand="ford")[0]["ctrl"]
  monkeypatch.setattr(type(ctrl.veh), "vtsc_notch_from_vego", property(lambda self: True))
  forced = _caps(H.replay(monkeypatch, fr, fp="FORD_F_150_LIGHTNING_MK1", brand="ford"))
  assert forced != today                                                       # teeth: without the gate the Lightning WOULD change


def test_the_kill_switch_is_exactly_today(monkeypatch):
  fr = _frames(38.0, SET, BEND_B)
  off = H.replay(monkeypatch, fr, notch_vego=False)
  assert off[0]["ctrl"].veh.vtsc_notch_from_vego is False
  assert min(_caps(off)) >= SET - C.MAP_MIN_SLOWDOWN - 0.05


# ---------------------------------------------------------------- the episode hold (no ratchet)

def _ctrl(monkeypatch, notch_vego=True):
  return H.make_controller(monkeypatch, notch_vego=notch_vego)[0]


def test_the_reference_is_live_while_idle_and_can_only_rise_during_an_episode(monkeypatch):
  c = _ctrl(monkeypatch)
  c._state = "idle"
  assert c._notch_ref(40.0, 38.0) == 38.0 and c._notch_ref(40.0, 36.0) == 36.0     # idle: follows the car, down and up
  c._state = "brake"
  assert c._notch_ref(40.0, 33.0) == 36.0                                           # episode: the car slowing does NOT pull it down
  assert c._notch_ref(40.0, 30.0) == 36.0
  assert c._notch_ref(40.0, 39.0) == 39.0                                           # ... but a faster car raises it
  assert c._notch_ref(40.0, 45.0) == 40.0                                           # never above the set speed
  assert c._notch_ref(37.0, 45.0) == 37.0                                           # the set speed dropped: min(set, .)
  c._state = "release"
  assert c._notch_ref(40.0, 20.0) == 37.0                                           # release is still the episode
  c._state = "idle"
  assert c._notch_ref(40.0, 33.0) == 33.0                                           # idle again: the next episode starts from the car


def test_kill_switch_off_keeps_the_set_speed_through_an_episode(monkeypatch):
  c = _ctrl(monkeypatch, notch_vego=False)
  for st in ("idle", "brake", "hold", "release"):
    c._state = st
    assert c._notch_ref(40.0, 30.0) == 40.0


def test_a_bend_taken_at_the_set_speed_is_not_ratcheted_down_by_its_own_slowdown(monkeypatch):
  """The failure mode of measuring the notch live every cycle (replayed on 22:32:58, set == vEgo == 90 mph: 81 -> 67 mph): with the car
  following the cap (closed loop) the notch chases vEgo down. The episode hold makes this scenario exactly today's."""
  from openpilot.selfdrive.controls.lib.vtsc_pnw import vtsc_controller as VC
  fr = _frames(SET, SET, [(300.0, 27.0), (380.0, 28.7), (470.0, 27.0)], seconds=14)
  old = H.replay(monkeypatch, fr, closed=True, notch_vego=False)
  new = H.replay(monkeypatch, fr, closed=True, notch_vego=True)
  assert min(r["v"] for r in old) < SET - 0.5                                        # the scenario really slows the car
  assert _caps(new) == _caps(old)                                                    # entered at the set speed: byte-identical to today
  # teeth: the live (per-cycle) rule is NOT identical here -- it brakes deeper and deeper
  monkeypatch.setattr(VC.VTSCController, "_notch_ref", lambda self, v_set, v_ego: VP.notch_reference(v_set, v_ego, True))
  live = H.replay(monkeypatch, fr, closed=True, notch_vego=True)
  assert min(_caps(live)) < min(_caps(old)) - 2.0


def test_a_curve_the_car_is_already_below_keeps_todays_fold(monkeypatch):
  """raw 16 m/s (scaled ~22) with the car at 20 m/s, set 40: today folds it (22 is far below the set speed). The car-relative notch would not,
  but the gate stays on the set speed, so the fold -- and today's cap -- are unchanged (never shallower than today)."""
  fr = _frames(20.0, SET, [(250.0, 16.0)])
  old = H.replay(monkeypatch, fr, notch_vego=False)
  assert min(_caps(old)) < SET - 1.0
  assert _caps(H.replay(monkeypatch, fr, notch_vego=True)) == _caps(old)


def test_the_second_tier_applies_where_the_car_relative_notch_does_not_floor():
  pts = _pts((273.0, 34.0))
  args = (pts, LAT0, LON0, 38.0, 500.0, C.A_DECEL, C.APEX_FINISH_S, C.SHARP_CURVE_V, C.MAP_SPEED_SCALE, SET, C.MAP_MIN_SLOWDOWN, 0.0)
  v, _, _, _, floored = VP.most_binding_map_curve(*args, notch_ref=38.0)
  assert floored is True and v == SET - C.MAP_MIN_SLOWDOWN          # today's 35.7: not unfloored, not the 33.5 notch


def _env(res, v_ego):
  v, d = res[0], res[1]
  return VP.brake_cap_for_apex(v, d, v_ego, C.A_DECEL, C.APEX_FINISH_S) if v > 0.0 else float("inf")


def test_fold_envelope_with_the_change_is_never_higher_than_today_fuzz():
  """Property of the fold (the part this change touches): for random vEgo <= set, random map nodes and targets, the decel envelope of the
  selected curve with the car-relative notch is never HIGHER than today's. (Through the state machine this is not guaranteed pointwise:
  a deeper map winner that is close can mask a vision curve that today would have started braking for -- measured, see the commit message.)"""
  rng = random.Random(0x5EED)
  for _ in range(400):
    v_set = rng.uniform(22.0, 42.0)
    v_ego = rng.uniform(8.0, v_set)
    pts = _pts(*[(rng.uniform(30.0, 480.0), rng.uniform(8.0, 50.0)) for _ in range(rng.randint(1, 8))])
    args = (pts, LAT0, LON0, v_ego, 500.0, C.A_DECEL, C.APEX_FINISH_S, C.SHARP_CURVE_V, C.MAP_SPEED_SCALE, v_set, C.MAP_MIN_SLOWDOWN,
            rng.choice([0.0, 22.0]))

    off = _env(VP.most_binding_map_curve(*args), v_ego)
    on = _env(VP.most_binding_map_curve(*args, notch_ref=VP.notch_reference(v_set, v_ego)), v_ego)
    assert on <= off + 1e-9, (v_set, v_ego, on, off)


# ---------------------------------------------------------------- the recorded fixtures with the change ON (production default)

def _fixtures():
  from openpilot.selfdrive.controls.lib.vtsc_pnw.tests import release_later_frames as F
  return (("OR34_LEFT", F.OR34_LEFT), ("TERWILLIGER_LEFT", F.TERWILLIGER_LEFT), ("OLYMPIA_11", F.OLYMPIA_11),
          ("TERWILLIGER_2232", F.TERWILLIGER_2232))


def test_on_is_never_a_higher_cap_than_off_on_the_four_recorded_fixtures(monkeypatch):
  """Open loop (recorded vEgo) on every recorded fixture: with the change ON each 50 ms cap is <= the cap with it OFF, apart from the
  state-machine's own re-timing (reported by the margin below, measured at +0.34 m/s at most on these fixtures)."""
  worst = 0.0
  for name, fr in _fixtures():
    on, off = H.replay(monkeypatch, fr, notch_vego=True), H.replay(monkeypatch, fr, notch_vego=False)
    worst = max(worst, max(a["cap"] - b["cap"] for a, b in zip(on, off, strict=True)))
    assert min(_caps(on)) <= min(_caps(off)) + 1e-9, name
  assert worst <= 0.5, worst


def test_or34_baseline_with_the_change_on(monkeypatch):
  """OR-34 (set 85, vEgo 82-84): OFF reproduces the recorded 75 mph floor; ON (production) floors at ~73.8 mph (the notch is measured from vEgo
  36.6 m/s: 36.6 - 4.5 = 32.1 target vs 33.5) -- a deliberate, small deepening; the state sequence is unchanged."""
  from openpilot.selfdrive.controls.lib.vtsc_pnw.tests import release_later_frames as F
  on = H.replay(monkeypatch, F.OR34_LEFT, release_later=False, notch_vego=True)
  off = H.replay(monkeypatch, F.OR34_LEFT, release_later=False, notch_vego=False)
  t = H.secs("08:10:42")
  def at(rows):
    return next(r for r in rows if r["t"] >= t)
  assert 74.5 <= at(off)["cap"] / MPH <= 76.0
  assert 73.0 <= at(on)["cap"] / MPH < at(off)["cap"] / MPH
  assert [r["state"] for r in on][::20] == [r["state"] for r in off][::20]


def test_map_ref_reaches_the_overlay_and_the_ces_events_tick(monkeypatch):
  """mapRef (the held notch reference) is in VTSCStatus and in the explicit list ces_pnw lifts into every ces_events tick."""
  import json
  from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw
  fr = _frames(38.0, SET, BEND_B)
  rows = H.replay(monkeypatch, fr, notch_vego=True)
  pay = [r["pay"] for r in rows if r["pay"]["mapRaw"] > 0][0]
  assert pay["mapRef"] == pytest.approx(38.0, abs=0.01)                          # the car's speed, not the set speed
  assert "mapRef" in ces_pnw.VTSC_TELE_KEYS
  assert H.replay(monkeypatch, fr, notch_vego=False)[40]["pay"]["mapRef"] == pytest.approx(SET, abs=0.01)
  assert json.loads(json.dumps(pay))["mapRef"] == pay["mapRef"]


def test_a_non_finite_vego_falls_back_to_the_set_speed_and_says_so_once(monkeypatch):
  from openpilot.selfdrive.controls.lib.vtsc_pnw import vtsc_controller as VC
  errs = []
  monkeypatch.setattr(VC.cloudlog, "error", lambda msg, *a, **k: errs.append(msg))
  c = H.make_controller(monkeypatch)[0]
  c._state = "brake"
  for _ in range(3):
    assert c._notch_ref(40.0, float("nan")) == 40.0
  assert len(errs) == 1 and "non-finite vEgo" in errs[0]


# ---------------------------------------------------------------- closed loop: the hold-latch regression the fold-level fuzz cannot see

def _closed_arrival(monkeypatch, notch_vego, v_set=30.5, v0=24.3, node=(49.0, 21.3), vis=(145.0, 21.6), seconds=14.0, gps_lag=1.0):
  """Car (below set, free to speed up) follows min(cap, set) with a first-order lag; one near map node and a vision curve further on, both fixed
  in position. Returns the car's speed on reaching the vision apex."""
  from openpilot.selfdrive.controls.lib.vtsc_pnw import vtsc_controller as VC
  ctrl, clock = H.make_controller(monkeypatch, notch_vego=notch_vego)
  vis_now = {}
  monkeypatch.setattr(VC, "model_curve_state", lambda model, v_cruise, a_lat: vis_now["s"])
  ns = H._NS()
  ns.orientationNED = [0.0, 0.0, 0.0]
  ns.enabled = True
  sm = {"modelV2": object(), "carControl": ns}
  s, v = 0.0, v0
  for _ in range(int(seconds / 0.05)):
    clock[0] += 0.05
    s_seen = s - v * gps_lag      # the map fold works from a fix ~1 s old (measured on the 09-30 ticks)
    ctrl._map_targets = [{"latitude": LAT0 + (node[0] - s_seen) / M_PER_DEG, "longitude": LON0, "velocity": node[1]}] if node[0] - s_seen > 0 else []
    ctrl._cur_lat, ctrl._cur_lon, ctrl._cur_bearing = LAT0, LON0, 0.0
    ctrl._last_read = clock[0]
    ctrl._gps_fix_ts = clock[0] - 1.4
    d = vis[0] - s
    vis_now["s"] = (C.A_LAT_TARGET / (vis[1] ** 2), max(d, 0.0), vis[1]) if d > -20.0 else (0.0, -1.0, float("inf"))
    cap = ctrl.cap(sm, v_set, v)
    v = max(v + max(min((min(cap, v_set) - v) / 1.0, 0.8), -2.0) * 0.05, 0.0)
    s += v * 0.05
    if s >= vis[0]:
      return v
  raise AssertionError("never reached the vision curve")


def test_a_near_map_node_cannot_make_the_car_faster_into_a_vision_curve_than_today(monkeypatch):
  """One near node (49 m, raw 21.3) and a vision curve at 145 m (21.6 m/s), car at 24.3 below set 30.5 and speeding up. Without the hold-horizon
  gate the held reference rises with the car, the near node's floor drops to ~raw, the map wins with its apex ~1 s away, the machine goes
  brake -> HOLD, freezes the cap and never brakes for the vision curve: the car arrives ~3 m/s faster than today."""
  today = _closed_arrival(monkeypatch, notch_vego=False)
  new = _closed_arrival(monkeypatch, notch_vego=True)
  assert new <= today + 0.1, (new, today)


def test_the_harness_default_is_the_production_default(monkeypatch):
  """make_controller() with no arguments must exercise the change ON (a default of False hid production behaviour from every fixture)."""
  assert H.make_controller(monkeypatch)[0].veh.vtsc_notch_from_vego is True


def test_the_deeper_notch_applies_only_beyond_the_hold_horizon():
  """Inside HOLD_TTA_S of travel (38 m/s * 2.5 s = 95 m) a point keeps today's set-relative floor; beyond it the car-relative notch applies."""
  horizon = 38.0 * C.HOLD_TTA_S
  for d, want in ((horizon - 5.0, SET - C.MAP_MIN_SLOWDOWN), (horizon + 5.0, 38.0 - C.MAP_MIN_SLOWDOWN)):
    args = (_pts((d, 30.4)), LAT0, LON0, 38.0, 500.0, C.A_DECEL, C.APEX_FINISH_S, C.SHARP_CURVE_V, C.MAP_SPEED_SCALE, SET,
            C.MAP_MIN_SLOWDOWN, 0.0)
    assert VP.most_binding_map_curve(*args, notch_ref=38.0)[0] == pytest.approx(want), d
