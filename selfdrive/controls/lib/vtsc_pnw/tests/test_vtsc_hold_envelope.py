"""vtschold2pnw (owner-approved 2026-10-01): VTSC's apex HOLD no longer freezes the cap against a curve that needs a lower one, on the
Raven only, behind curve.json tesla.vtsc_hold_envelope (default ON).

Why: in `hold` the applied cap is frozen (target = applied). A hold entered for a near apex -- or on a harmless map node, with the cap still far
above the car -- left the cap there while a DEEPER curve approached; the hold -> brake exit needs tta > HOLD_TTA_S + margin for 3 cycles, and the
rate-limited brake ramp then starts too late. Closed-loop fuzz of the real controller: today's code arrived up to +4.5 m/s faster than a
latch-free controller (18.06 m/s into a 12.6 m/s vision curve; scenario S1 below), 22 of 1000 scenarios by more than 0.9 m/s.

The fix lowers the held cap, never raises it: while the binding curve is still ahead of the apex window (tta > APEX_TTA_S), above its safe
speed the held cap follows the brake envelope; at safe speed it is lowered only to the car's own speed (so the hold never brakes the car).

Pinned here through the REAL VTSCController.cap(): each branch, the apex window left alone, never above today's cap, the kill switch, the
Lightning untouched, the one-line log, the telemetry key, and a closed-loop regression that fails without the change."""
import json
import random

import pytest

from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw
from openpilot.selfdrive.controls.lib.vtsc_pnw import vtsc_constants as C
from openpilot.selfdrive.controls.lib.vtsc_pnw import vtsc_controller as VC
from openpilot.selfdrive.controls.lib.vtsc_pnw.tests import release_later_frames as F
from openpilot.selfdrive.controls.lib.vtsc_pnw.tests import release_later_harness as H

LAT0, LON0, M_PER_DEG = H.LAT0, H.LON0, H.M_PER_DEG
SET = 40.0


# ---------------------------------------------------------------- one-cycle-at-a-time driver for the branch tests

class Rig:
  """A real controller put straight into `hold` with a chosen applied cap, then run with constant inputs (vision curve of speed `vc` at `d`
  m, car at `v_ego`): the cap converges to the hold's target (rate-limited at 2 m/s^2), so the end value IS the branch's verdict."""

  def __init__(self, monkeypatch, hold_envelope, applied, v_ego, vc, d, v_set=SET, fp="TESLA_MODEL_S_HW3", brand="tesla"):
    self.ctrl, self.clock = H.make_controller(monkeypatch, fp=fp, brand=brand, hold_envelope=hold_envelope)
    self.vis = {"s": (C.A_LAT_TARGET / (vc * vc), d, vc)}
    monkeypatch.setattr(VC, "model_curve_state", lambda model, v_cruise, a_lat: self.vis["s"])
    ns = H._NS()
    ns.orientationNED = [0.0, 0.0, 0.0]
    ns.enabled = True
    self.sm = {"modelV2": object(), "carControl": ns}
    self.v_ego, self.v_set = v_ego, v_set
    self.ctrl._applied = applied
    self.ctrl._state = "hold"
    self.trail = []

  def run(self, cycles):
    for _ in range(cycles):
      self.clock[0] += 0.05
      self.ctrl.cap(self.sm, self.v_set, self.v_ego)
      self.trail.append((self.ctrl._state, self.ctrl._applied, self.ctrl._tele_hold_env))
    return self.ctrl._applied


# apex window: tta = d / v_ego must be in (APEX_TTA_S, HOLD_TTA_S] so the hold persists and the new clause is reachable
V_EGO, VCS, D = 30.0, 20.0, 45.0           # tta 1.5 s; 30 > 1.1 * 20 -> NOT at safe speed
assert C.APEX_TTA_S < D / V_EGO <= C.HOLD_TTA_S


def test_above_the_safe_speed_the_held_cap_follows_the_brake_envelope(monkeypatch):
  r = Rig(monkeypatch, True, applied=34.0, v_ego=V_EGO, vc=VCS, d=D)
  # inside the hold window the envelope is the curve's safe speed itself (the finish distance is 0)
  assert r.run(200) == pytest.approx(VCS, abs=0.01)
  assert {s for s, _, _ in r.trail} == {"hold"}                       # it stayed a hold: no state change, only a lower held cap
  assert r.trail[-1][2] == "env"
  # rate-limited: never faster than the planner's regen ceiling
  caps = [34.0] + [a for _, a, _ in r.trail]
  assert min(b - a for a, b in zip(caps, caps[1:], strict=False)) >= -2.0 * 0.05 - 1e-9


def test_at_safe_speed_the_held_cap_is_lowered_only_to_the_car_s_own_speed(monkeypatch):
  v_ego = 21.5                                                         # 21.5 <= 1.1 * 20: at safe speed
  r = Rig(monkeypatch, True, applied=34.0, v_ego=v_ego, vc=VCS, d=D * v_ego / V_EGO)
  assert r.run(300) == pytest.approx(v_ego, abs=0.01)                  # max(env = 20, v_ego): the hold does not brake the car
  assert r.trail[-1][2] == "speed" and r.trail[-1][0] == "hold"


def test_a_harmless_curve_ahead_still_pulls_the_trailing_cap_down_to_its_envelope(monkeypatch):
  """The car is BELOW the curve's safe speed (18 < 20): the cap that trailed far above both is lowered to the envelope (20), not to the car."""
  v_ego, d = 18.0, 36.0                                                # tta 2.0 s, in the window
  r = Rig(monkeypatch, True, applied=34.0, v_ego=v_ego, vc=VCS, d=d)
  assert r.run(300) == pytest.approx(VCS, abs=0.01)
  assert r.trail[-1][2] == "env"


def test_the_apex_window_is_left_alone(monkeypatch):
  """tta <= APEX_TTA_S: never a fresh brake AT the apex, exactly as before -- the cap stays frozen, with the switch on or off."""
  d = 0.9 * C.APEX_TTA_S * V_EGO * 0.9                                 # tta ~1.1 s, not at safe speed
  for flag in (True, False):
    r = Rig(monkeypatch, flag, applied=34.0, v_ego=V_EGO, vc=VCS, d=d)
    assert r.run(60) == pytest.approx(34.0)
    assert r.trail[-1][0] == "hold" and r.trail[-1][2] == ""


def test_the_held_cap_is_never_raised(monkeypatch):
  """A cap already below what the branch would pick stays where it is (target = min(applied, ...)): the change only ever lowers."""
  r = Rig(monkeypatch, True, applied=18.0, v_ego=V_EGO, vc=VCS, d=D)    # applied 18 < env 20
  assert r.run(60) == pytest.approx(18.0)
  assert r.trail[-1][2] == ""
  r = Rig(monkeypatch, True, applied=21.0, v_ego=21.5, vc=VCS, d=D * 21.5 / V_EGO)   # applied 21 < v_ego 21.5: not lifted to the car's speed
  assert r.run(60) == pytest.approx(21.0)


def test_switch_off_is_the_frozen_hold(monkeypatch):
  for v_ego, d in ((V_EGO, D), (21.5, D * 21.5 / V_EGO)):
    r = Rig(monkeypatch, False, applied=34.0, v_ego=v_ego, vc=VCS, d=d)
    assert r.run(120) == pytest.approx(34.0)
    assert {t for _, _, t in r.trail} == {""}


def test_the_lightning_never_takes_the_branch(monkeypatch):
  r = Rig(monkeypatch, True, applied=34.0, v_ego=V_EGO, vc=VCS, d=D, fp="FORD_F_150_LIGHTNING_MK1", brand="ford")
  assert r.ctrl.veh.vtsc_hold_envelope is False
  assert r.run(120) == pytest.approx(34.0)


def test_a_non_finite_curve_speed_cannot_reach_the_envelope(monkeypatch):
  """No curve speed (a CUE-only bend reports v_curve = inf): no envelope exists, the cap stays frozen (and nothing raises)."""
  r = Rig(monkeypatch, True, applied=34.0, v_ego=V_EGO, vc=VCS, d=D)
  monkeypatch.setattr(VC, "model_curve_state", lambda model, v_cruise, a_lat: (C.CUE_MIN_CURVATURE * 2, D, float("inf")))
  r.run(60)
  assert r.ctrl._applied == pytest.approx(34.0) and r.trail[-1][2] == ""


def test_the_envelope_never_goes_below_vmin_or_above_cruise_minus_the_cut(monkeypatch):
  r = Rig(monkeypatch, True, applied=34.0, v_ego=V_EGO, vc=3.0, d=D)   # a curve target below V_MIN
  assert r.run(300) == pytest.approx(C.V_MIN, abs=0.01)
  r = Rig(monkeypatch, True, applied=39.95, v_ego=V_EGO, vc=39.8, d=D)  # target above cruise - CONFIDENCE_CUT: capped there (the brake state's rule)
  assert r.run(300) == pytest.approx(SET - C.CONFIDENCE_CUT, abs=0.01)


def test_one_log_line_per_hold_episode_and_a_fresh_one_for_the_next(monkeypatch):
  lines = []
  monkeypatch.setattr(VC.cloudlog, "info", lambda msg, *a, **k: lines.append(msg % a if a else msg))
  r = Rig(monkeypatch, True, applied=34.0, v_ego=V_EGO, vc=VCS, d=D)
  r.run(100)
  mine = [ln for ln in lines if ln.startswith("VTSC hold lowers its cap")]
  assert len(mine) == 1 and "(env)" in mine[0] and "34.0" in mine[0]
  # the episode ends (hold -> release: apex reached) and a new one starts: it logs again
  r.ctrl._state = "release"
  r.run(1)
  r.ctrl._state = "hold"
  r.ctrl._applied = 30.0
  r.run(40)
  assert len([ln for ln in lines if ln.startswith("VTSC hold lowers its cap")]) == 2


def test_the_telemetry_key_is_published_and_wired_through_ces_pnw(monkeypatch):
  assert "vtscHoldEnv" in ces_pnw.VTSC_TELE_KEYS
  r = Rig(monkeypatch, True, applied=34.0, v_ego=V_EGO, vc=VCS, d=D)
  r.run(5)
  pay = r.ctrl.overlay_payload()
  assert pay["vtscHoldEnv"] == "env"
  assert json.loads(json.dumps(pay))["vtscHoldEnv"] == "env"
  assert set(ces_pnw.VTSC_TELE_KEYS) <= set(pay)
  r = Rig(monkeypatch, False, applied=34.0, v_ego=V_EGO, vc=VCS, d=D)
  r.run(5)
  assert r.ctrl.overlay_payload()["vtscHoldEnv"] == ""


# ---------------------------------------------------------------- the recorded fixtures: unchanged, switch on or off
# (the passed-point geometry replays in test_vtsc_passed_point.py run with the production default, i.e. the change ON, and keep their expectations)

FIXTURES = {"OR34": F.OR34_LEFT, "TERWILLIGER": F.TERWILLIGER_LEFT, "OLYMPIA": F.OLYMPIA_11, "TERWILLIGER_2232": F.TERWILLIGER_2232}


@pytest.mark.parametrize("name", sorted(FIXTURES))
def test_the_recorded_fixtures_are_unchanged(monkeypatch, name):
  on = [r["cap"] for r in H.replay(monkeypatch, FIXTURES[name], hold_envelope=True)]
  off = [r["cap"] for r in H.replay(monkeypatch, FIXTURES[name], hold_envelope=False)]
  assert on == off, name


# ---------------------------------------------------------------- closed loop: the regression (fails without the change)

def _closed(monkeypatch, hold_envelope, v_set, v0, nodes, vis, seconds=40.0, gps_lag=1.0):
  """The car follows min(cap, set) (first-order lag, accel <= 0.8, decel <= 2.0) toward map nodes and one vision curve fixed in position.
  Returns (speed on reaching the vision apex, lowest cap, every cycle's cap)."""
  ctrl, clock = H.make_controller(monkeypatch, hold_envelope=hold_envelope)
  seen = {}
  monkeypatch.setattr(VC, "model_curve_state", lambda model, v_cruise, a_lat: seen["s"])
  ns = H._NS()
  ns.orientationNED = [0.0, 0.0, 0.0]
  ns.enabled = True
  sm = {"modelV2": object(), "carControl": ns}
  s, v, low, caps = 0.0, v0, 1e9, []
  for _ in range(int(seconds / 0.05)):
    clock[0] += 0.05
    s_seen = s - v * gps_lag
    ctrl._map_targets = [{"latitude": LAT0 + (p - s_seen) / M_PER_DEG, "longitude": LON0, "velocity": tv} for p, tv in nodes if p - s_seen > 0.0]
    ctrl._cur_lat, ctrl._cur_lon, ctrl._cur_bearing = LAT0, LON0, 0.0
    ctrl._last_read = clock[0]
    ctrl._gps_fix_ts = clock[0] - 1.4
    d = vis[0] - s
    seen["s"] = (C.A_LAT_TARGET / (vis[1] ** 2), max(d, 0.0), vis[1]) if d > -5.0 else (0.0, -1.0, float("inf"))
    cap = ctrl.cap(sm, v_set, v)
    low = min(low, cap)
    caps.append(round(cap, 3))
    v = max(v + max(min((min(cap, v_set) - v) / 1.0, 0.8), -2.0) * 0.05, 0.0)
    s += v * 0.05
    if s >= vis[0]:
      return v, low, caps
  raise AssertionError("never reached the vision curve")


# (set, v0, vision (position m, safe speed m/s), map nodes (position m, raw mapd speed m/s)) -- from the closed-loop fuzz of 2026-09-30/10-01.
# today: 18.06 / 15.91 m/s into curves of 12.6 / 10.6 m/s safe speed; latch-free: 13.59 / 11.52.
S1 = (35.1198, 15.2108, (262.4946, 12.6455), [(23.7557, 28.5958), (358.6852, 42.0795), (99.6753, 13.8628)])
S2 = (38.0520, 14.1578, (286.4660, 10.5994), [(59.6678, 16.0178)])


@pytest.mark.parametrize("name,sc,today", [("S1", S1, 18.06), ("S2", S2, 15.91)])
def test_a_latched_hold_no_longer_drives_into_the_deeper_curve(monkeypatch, name, sc, today):
  v_set, v0, vis, nodes = sc
  off = _closed(monkeypatch, False, v_set, v0, nodes, vis)[0]
  on = _closed(monkeypatch, True, v_set, v0, nodes, vis)[0]
  assert off == pytest.approx(today, abs=0.05), (name, off)            # the scenario really does reproduce today's overspeed
  assert on <= off - 3.0, (name, on, off)                              # ... and the change removes most of it
  assert on <= vis[1] + 1.5, (name, on)                                # arrives within ~1.5 m/s of the curve's safe speed


def test_the_switch_off_run_is_bit_for_bit_the_pre_change_controller(monkeypatch):
  """Golden caps recorded from the controller BEFORE this change (origin/3devpnw acc42b5a9f) on S1 and S2, every 20th cycle (1 s), 3 decimals: with
  the switch off the whole closed-loop cap trail must match them exactly."""
  for name, (v_set, v0, vis, nodes) in (("S1", S1), ("S2", S2)):
    assert _closed(monkeypatch, False, v_set, v0, nodes, vis, seconds=20.0)[2][::20] == GOLDEN_OFF[name], name


GOLDEN_OFF = {
    "S1": [35.12, 32.72, 30.72, 28.72, 26.72, 26.32, 27.145, 27.97, 25.97, 23.97, 21.97, 19.97, 17.97, 17.57],
    "S2": [38.052, 35.652, 33.652, 32.952, 33.552, 33.502, 31.502, 29.502, 27.502, 25.502, 23.502, 21.502, 19.502, 17.502, 15.502, 15.402],
}


# ---------------------------------------------------------------- fuzz: the cap is never above today's, the car never faster into a curve

def _fuzz_scenarios(seed, n):
  rng = random.Random(seed)
  for _ in range(n):
    v_set = rng.uniform(22.0, 40.0)
    v0 = rng.uniform(10.0, v_set)
    vis = (rng.uniform(60.0, 350.0), rng.uniform(8.0, v0))
    nodes = [(rng.uniform(3.0, 480.0), rng.uniform(8.0, 45.0)) for _ in range(rng.randint(1, 6))]
    yield v_set, v0, vis, nodes


def test_fuzz_the_car_never_arrives_faster_than_today_by_more_than_the_design_margin(monkeypatch):
  """Closed loop, 40 random scenarios: the fix car's speed on reaching the vision curve is never above today's (trajectories differ, so
  a small tolerance for the lag model) and strictly lower in at least one."""
  worse, better = [], 0
  for v_set, v0, vis, nodes in _fuzz_scenarios(11, 40):
    off = _closed(monkeypatch, False, v_set, v0, nodes, vis)[0]
    on = _closed(monkeypatch, True, v_set, v0, nodes, vis)[0]
    if on > off + 0.3:
      worse.append((v_set, v0, vis, nodes, on, off))
    better += on < off - 0.3
  assert not worse, worse
  assert better >= 1
