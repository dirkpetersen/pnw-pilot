"""vtscfloor2pnw release-later (owner 2026-09-29, option 2): once VTSC's state machine has RELEASED, keep the cap frozen instead of
climbing while the camera's own target is still tightening. The real cases are replayed through the REAL VTSCController.cap()
(release_later_harness.py, recorded inputs in release_later_frames.py), open and closed loop. The state machine is untouched, so the
invariant pinned everywhere is: the cap (and the modelled car) is never higher than today's.
"""
import json
import random

import pytest

from openpilot.selfdrive.controls.lib.vtsc_pnw import vtsc_constants as C
from openpilot.selfdrive.controls.lib.vtsc_pnw import vtsc_controller as VC
from openpilot.selfdrive.controls.lib.vtsc_pnw.tests import release_later_frames as F
from openpilot.selfdrive.controls.lib.vtsc_pnw.tests import release_later_harness as H

MPH = 0.44704
DT = 0.05


def _caps(rows):
  return [r["cap"] for r in rows]


def _peak_decel(rows):
  """Largest cap fall rate (m/s^2) over the run, EXCLUDING the one-off CONFIDENCE_CUT the state machine applies on entering brake
  (an instant 0.5 m/s cut so the driver feels VTSC engage -- existing behaviour, identical with the feature off). Asserts that
  any faster step IS such an entry cut and no larger than it."""
  worst = 0.0
  for a, b in zip(rows, rows[1:], strict=False):
    rate = (a["cap"] - b["cap"]) / DT
    if rate > C.SHARP_A_DECEL_MAX + 1e-6:
      assert b["state"] == "brake" and a["state"] != "brake" and a["cap"] - b["cap"] <= C.CONFIDENCE_CUT + C.SHARP_A_DECEL_MAX * DT + 1e-6
    else:
      worst = max(worst, rate)
  return worst


def _synthetic(v_ego, v_set, cam_v, cam_d, pts, seconds=8):
  """Constant-input frames (the recording format) so a scenario can be stated in one line. cam_v 0 = no camera curve."""
  fr = []
  for s in range(seconds + 1):
    fr.append((f"10:00:{s:02d}", v_ego, v_set, cam_v, cam_d - v_ego * s if cam_d > 0 else cam_d, None, "idle",
               [(d - v_ego * s, v) for d, v in pts]))
  return fr


# ---------------------------------------------------------------- the three real cases

def test_replay_reproduces_the_recorded_vtsc_behaviour(monkeypatch):
  """The harness is only worth trusting if the baseline reproduces the logs: OR-34 capped at the set-10 floor (75 mph), brake -> hold ->
  release at 08:10:44, cap climbing back toward 85 while the camera target falls."""
  rows = H.replay(monkeypatch, F.OR34_LEFT, release_later=False)
  by_s = {}
  for r in rows:
    by_s.setdefault(int(r["t"]), r)
  t = H.secs("08:10:00")
  assert 74.5 <= by_s[int(t + 42)]["cap"] / MPH <= 76.0                  # recorded: 75
  assert by_s[int(t + 43)]["state"] == "hold"                            # recorded: hold at 08:10:43
  assert by_s[int(t + 44)]["state"] == "release"                         # recorded: release at 08:10:44
  assert by_s[int(t + 45)]["cap"] / MPH > 80.0                           # recorded: cap back up to 85 (82 at :45)


def test_or34_release_is_deferred_so_the_cap_holds_at_75(monkeypatch):
  """Release-later (owner 2026-09-29, option 2). Today the machine went hold -> release at 08:10:43.2 (the winner flipped map -> camera,
  apex 56 -> 16 m, tta 1.58 -> 0.45 s) and the cap climbed 75 -> 85 while the camera target fell to 63. Now, IN release, the cap stays
  frozen while the target falls (state machine untouched, so the re-arm to brake still runs). HONEST: the frozen cap is ~75, not the
  camera's 63-66 -- freezing does not reduce."""
  on = H.replay(monkeypatch, F.OR34_LEFT, release_later=True)
  off = H.replay(monkeypatch, F.OR34_LEFT, release_later=False)
  win = [(a, b) for a, b in zip(on, off, strict=True) if H.secs("08:10:43") <= a["t"] <= H.secs("08:10:47")]
  assert max(a["cap"] for a, _ in win) / MPH <= 75.3
  assert max(b["cap"] for _, b in win) / MPH > 84.0                        # today
  assert [a["state"] for a, _ in win] == [b["state"] for _, b in win]      # the state machine itself is today's
  assert any(a["pay"]["vtscRelDefer"] == "falling" for a, _ in win)
  assert all(a["pay"]["vtscRelDefer"] for a, _ in win if a["state"] == "release")   # never blank while held (Rule 2)
  assert all(a["cap"] <= b["cap"] + 1e-9 for a, b in zip(on, off, strict=True))   # never a higher cap than today


def test_terwilliger_release_is_deferred_so_the_cap_holds_at_65(monkeypatch):
  on = H.replay(monkeypatch, F.TERWILLIGER_LEFT, release_later=True)
  off = H.replay(monkeypatch, F.TERWILLIGER_LEFT, release_later=False)
  w = [(a, b) for a, b in zip(on, off, strict=True) if H.secs("22:35:12") <= a["t"] <= H.secs("22:35:14")]
  assert max(a["cap"] for a, _ in w) / MPH <= 65.1
  assert max(b["cap"] for _, b in w) / MPH > 68.0


def _falling_curve(v_ego=25.0, n=20, start=40.0, rate=1.2):
  """4 s far away (brake), then the apex is 5 m ahead and the camera target falls `rate` m/s each second."""
  return [(f"10:00:{s:02d}", v_ego, 38.0, start - rate * s, 200.0 if s < 4 else 5.0, None, "idle", []) for s in range(n)]


def test_release_freeze_is_bounded_and_then_climbs_as_today(monkeypatch):
  """A camera target that keeps falling cannot freeze the cap forever: after REL_DEFER_MAX_S it climbs as today (and says so)."""
  lg = _Log()
  monkeypatch.setattr(VC, "cloudlog", lg)
  rows = H.replay(monkeypatch, _falling_curve(), release_later=True)
  fz = [r for r in rows if r["pay"]["vtscRelDefer"]]
  assert fz and fz[-1]["t"] - fz[0]["t"] == pytest.approx(C.REL_DEFER_MAX_S, abs=0.3)
  after = [r for r in rows if r["t"] > fz[-1]["t"] + 0.5]
  assert after and not any(r["pay"]["vtscRelDefer"] for r in after)         # the freeze is over for good
  assert any("hit its" in m for m in lg.at("warning")) and any("DEFERRED (falling)" in m for m in lg.at("info"))
  off = H.replay(monkeypatch, _falling_curve(), release_later=False)
  assert all(a["cap"] <= b["cap"] + 1e-9 for a, b in zip(rows, off, strict=True))


def test_a_curve_exit_never_freezes_whatever_vego_is(monkeypatch):
  """Independent of the vEgo test: a target that is flat or RISING after release never starts a freeze, even with the car far above it."""
  for v_ego in (30.0, 40.0):
    exit_ = [(f"10:00:{s:02d}", v_ego, 45.0, 34.0 if s < 6 else 34.0 + 3.0 * (s - 6), 200.0 if s < 4 else 5.0, None, "idle", [])
             for s in range(14)]
    rows = H.replay(monkeypatch, exit_, release_later=True)
    assert any(r["state"] == "release" for r in rows)
    assert not any(r["pay"]["vtscRelDefer"] for r in rows)
  tight = [(f"10:00:{s:02d}", 30.0, 38.0, 34.0 if s < 6 else 34.0 - 2.0 * (s - 6), 200.0 if s < 4 else 5.0, None, "idle", [])
           for s in range(14)]
  rows = H.replay(monkeypatch, tight, release_later=True)
  assert any(r["pay"]["vtscRelDefer"] == "falling" for r in rows)            # today's release first, then the freeze once it tightens


def test_a_near_straight_target_never_freezes(monkeypatch):
  """A camera target far above vEgo (>= 1.3x) that wobbles is noise, not a curve."""
  fr = [(f"10:00:{s:02d}", 25.0, 45.0, 60.0 - 1.5 * s, 200.0 if s < 4 else 5.0, None, "idle", []) for s in range(12)]
  rows = H.replay(monkeypatch, fr, release_later=True)
  assert not any(r["pay"]["vtscRelDefer"] for r in rows if r["ctrl"]._tele_vis_v > 1.3 * 25.0)


def _sbend(v=26.0, vis2=21.0, vset=33.0, jump_d=150.0):
  """A first curve whose release freezes, then the camera apex jumps to a SECOND, tighter curve `jump_d` m ahead."""
  fr = [(f"10:00:{s:02d}", v, vset, 25.0, float(d), None, "idle", []) for s, d in enumerate([150, 124, 98, 72, 46, 20, 5])]
  d, s = jump_d, 7
  while d > 0:
    fr.append((f"10:00:{s:02d}", v, vset, vis2, float(d), None, "idle", []))
    d -= v
    s += 1
  fr += [(f"10:00:{s + k:02d}", v, vset, 0.0, -1.0, None, "idle", []) for k in range(4)]
  return fr


def test_the_next_curve_is_still_braked_for_after_a_freeze(monkeypatch):
  """HIGH from review: a frozen cap must never stop the NEXT curve from being braked for. The state machine is today's, so the cap is
  never higher than today's at ANY cycle, on S-bends where the apex jumps while frozen."""
  for kw in (dict(), dict(vis2=18.0), dict(v=30.0, vis2=22.0, jump_d=200.0), dict(vset=38.0, vis2=17.0)):
    fr = _sbend(**kw)
    on, off = H.replay(monkeypatch, fr, release_later=True), H.replay(monkeypatch, fr, release_later=False)
    assert all(a["cap"] <= b["cap"] + 1e-9 for a, b in zip(on, off, strict=True)), kw
    assert min(_caps(on)) <= min(_caps(off)) + 1e-9, kw                      # the second curve's braking is as deep as today's
    assert [a["state"] for a, _ in zip(on, off, strict=True)] == [b["state"] for _, b in zip(on, off, strict=True)]


def test_cap_is_never_higher_than_today_on_the_three_recorded_curves(monkeypatch):
  for fr in (F.OR34_LEFT, F.TERWILLIGER_LEFT, F.OLYMPIA_11):
    on, off = H.replay(monkeypatch, fr, release_later=True), H.replay(monkeypatch, fr, release_later=False)
    assert all(a["cap"] <= b["cap"] + 1e-9 for a, b in zip(on, off, strict=True))


def _bare(monkeypatch, state="release", vis=0.0):
  """A controller whose _release_freeze can be called directly with crafted history."""
  ctrl, clock = H.make_controller(monkeypatch)
  ctrl._state = state
  ctrl._tele_vis_v = vis
  return ctrl, clock


def test_freeze_unit_branches(monkeypatch):
  monkeypatch.setattr(VC, "cloudlog", _Log())

  def step(ctrl, t, vc, vis, v_ego=30.0, has=True):
    ctrl._tele_vis_v = vis
    ctrl._rel_note(t, vc)
    return ctrl._release_freeze(t, has, vc, v_ego)
  ctrl, _ = _bare(monkeypatch)
  assert step(ctrl, 0.0, 32.0, 32.0) == "" and step(ctrl, 0.1, 31.9, 31.9) == ""          # steady: not started
  assert step(ctrl, 0.2, 30.0, 30.0) == "falling"                                          # the camera fell 2 m/s: started
  assert step(ctrl, 0.3, 30.0, 30.0, has=False) == ""                                      # no curve: never
  # a rising camera never starts one, even right after a source switch
  c2, _ = _bare(monkeypatch)
  step(c2, 0.0, 30.0, 28.0)
  c2._rel_win_t = 0.05
  assert step(c2, 0.1, 30.0, 30.0) == ""
  # "fast" (latched only) uses the FRESH camera speed (20), not the merged winner (32): 30 > 1.1 x 20 although 30 < 1.1 x 32
  c3, _ = _bare(monkeypatch)
  step(c3, 0.0, 32.0, 24.0)
  assert step(c3, 0.1, 32.0, 20.0) == "falling"
  last = ""
  for k in range(1, 8):
    last = step(c3, 1.0 + 0.1 * k, 32.0, 20.0)
  assert last == "fast"
  # a source switch alone (camera steady, near vEgo) starts it; a switch with the camera far above vEgo does not
  c5, _ = _bare(monkeypatch)
  step(c5, 0.0, 30.0, 30.0)
  c5._rel_win_t = 0.05
  assert step(c5, 0.1, 30.0, 30.0) == "switch"
  c6, _ = _bare(monkeypatch)
  step(c6, 0.0, 30.0, 34.0)
  c6._rel_win_t = 0.05
  assert step(c6, 0.1, 30.0, 34.0) == ""
  # a near-straight target (>= 1.3 x vEgo) is noise even when it falls fast
  c7, _ = _bare(monkeypatch)
  step(c7, 0.0, 50.0, 50.0)
  assert step(c7, 0.1, 45.0, 45.0) == ""
  c4, _ = _bare(monkeypatch)                                                               # not latched: "fast" alone never starts
  for k in range(8):
    assert step(c4, 0.1 * k, 32.0, 20.0) == ""


def test_bound_resets_per_curve(monkeypatch):
  monkeypatch.setattr(VC, "cloudlog", _Log())
  ctrl, _ = _bare(monkeypatch)
  t, out = 0.0, ""
  for _k in range(int((C.REL_DEFER_MAX_S + 1.0) / 0.05)):
    t += 0.05
    vis = 40.0 - 1.2 * t
    ctrl._tele_vis_v = vis
    ctrl._rel_note(t, vis)
    out = ctrl._release_freeze(t, True, vis, 40.0)
  assert ctrl._rel_defer_capped and out == ""                                # bound hit
  # a new curve: the machine leaves release (re-arm) -> cap() runs the reset block; emulate it and check a fresh bound
  ctrl._rel_defer_t0, ctrl._rel_defer_capped, ctrl._rel_latched = None, False, False
  ctrl._tele_vis_v = 30.0
  ctrl._rel_note(t + 0.1, 30.0)
  ctrl._tele_vis_v = 28.0
  ctrl._rel_note(t + 0.2, 28.0)
  assert ctrl._release_freeze(t + 0.2, True, 28.0, 40.0) == "falling"


def test_the_controller_resets_the_bound_when_braking_is_rearmed(monkeypatch):
  """Through the real cap(): after a freeze hits its bound, a second curve gets a fresh full bound."""
  monkeypatch.setattr(VC, "cloudlog", _Log())
  fr = _falling_curve(n=14)
  # then a second falling curve
  fr += [(f"10:00:{14 + s:02d}", 25.0, 38.0, 40.0 - 1.2 * s, 200.0 if s < 4 else 5.0, None, "idle", []) for s in range(14)]
  rows = H.replay(monkeypatch, fr, release_later=True)
  fz = [r["t"] for r in rows if r["pay"]["vtscRelDefer"]]
  gaps = [b - a for a, b in zip(fz, fz[1:], strict=False) if b - a > 0.5]
  assert gaps and len(fz) > (C.REL_DEFER_MAX_S / DT) * 1.5                   # two separate freezes, each ~ the bound


def test_no_curve_and_lightning_and_kill_switch_are_unchanged(monkeypatch):
  straight = _synthetic(36.0, 38.0, 0.0, -1.0, [])
  on, off = H.replay(monkeypatch, straight, release_later=True), H.replay(monkeypatch, straight, release_later=False)
  assert _caps(on) == _caps(off) and all(r["pay"]["vtscRelDefer"] == "" for r in on)
  lt = H.replay(monkeypatch, F.OR34_LEFT, fp="FORD_F_150_LIGHTNING_MK1", brand="ford")
  assert all(r["pay"]["vtscRelDefer"] == "" for r in lt)
  assert any(r["state"] == "release" for r in lt)                            # today's release, untouched
  ks = H.replay(monkeypatch, F.OR34_LEFT, release_later=False)
  assert all(r["pay"]["vtscRelDefer"] == "" for r in ks)
  monkeypatch.setattr(type(lt[0]["ctrl"].veh), "vtsc_release_later", property(lambda self: True))
  forced = H.replay(monkeypatch, F.OR34_LEFT, fp="FORD_F_150_LIGHTNING_MK1", brand="ford")
  assert any(r["pay"]["vtscRelDefer"] for r in forced)                       # teeth: the gate alone protects the Lightning


# ---------------------------------------------------------------- bounds

def test_fuzz_never_above_set_never_below_vmin_never_faster_than_the_ceiling(monkeypatch):
  rng = random.Random(20260929)
  for _ in range(40):
    v_set = rng.uniform(25.0, 42.0)
    v_ego = rng.uniform(15.0, v_set)
    pts = [(rng.uniform(20.0, 480.0), rng.uniform(8.0, 45.0)) for _ in range(rng.randint(1, 8))]
    cam_v = rng.choice([0.0, rng.uniform(10.0, 60.0)])
    frames = _synthetic(v_ego, v_set, cam_v, rng.uniform(0.0, 300.0), pts, seconds=6)
    rows = H.replay(monkeypatch, frames, release_later=True)
    caps = _caps(rows)
    assert all(C.V_MIN - 1e-9 <= c <= v_set + 1e-9 for c in caps)
    _peak_decel(rows)


# ---------------------------------------------------------------- helpers

class _Log:
  def __init__(self):
    self.lines = []

  def __getattr__(self, level):
    if level in ("debug", "info", "warning", "error", "exception", "critical", "event"):
      return lambda msg, *a, **k: self.lines.append((level, (msg % a) if a else msg))
    raise AttributeError(level)

  def at(self, level):
    return [m for lvl, m in self.lines if lvl == level]


# ---------------------------------------------------------------- closed loop

def test_terwilliger_2232_closed_loop_the_car_is_never_held_above_todays_speed(monkeypatch):
  """Terwilliger 22:32:06-26 replayed CLOSED loop (the car follows the cap): the modelled car and the cap are never above today's --
  the geometry where a hold-based design held the car 4.8 mph above today because hold never brakes again."""
  on = H.replay(monkeypatch, F.TERWILLIGER_2232, release_later=True, closed=True)
  off = H.replay(monkeypatch, F.TERWILLIGER_2232, release_later=False, closed=True)
  assert all(a["cap"] <= b["cap"] + 0.01 * MPH for a, b in zip(on, off, strict=True))
  assert all(a["v"] <= b["v"] + 0.01 * MPH for a, b in zip(on, off, strict=True))       # the modelled car, not just the cap
  assert [r["state"] for r in on] == [r["state"] for r in off]




def _drive(monkeypatch, blink_at=None, seconds=8.0):
  """Drive the real cap() cycle by cycle: 4 s of far curve, then the apex 5 m ahead with a falling camera target, so the machine
  releases and freezes. `blink_at` (s) = ONE cycle with no curve at all."""
  ctrl, clock = H.make_controller(monkeypatch)
  vis = {}
  monkeypatch.setattr(VC, "model_curve_state", lambda model, v_cruise, a: vis["s"])
  ns = H._NS()
  ns.orientationNED = [0.0, 0.0, 0.0]
  sm = {"modelV2": object(), "carControl": ns}
  rows, t = [], 0.0
  while t < seconds:
    clock[0] += DT
    t += DT
    ctrl._last_read = clock[0]
    ctrl._gps_fix_ts = clock[0] - 1.4
    v = 40.0 - 1.2 * t
    vis["s"] = (C.A_LAT_TARGET / (v * v), 200.0 if t < 4.0 else 5.0, v)
    if blink_at is not None and abs(t - blink_at) < DT / 2:
      vis["s"] = (0.0, -1.0, float("inf"))
    ctrl.cap(sm, 38.0, 25.0)
    rows.append((t, ctrl._state, ctrl._rel_defer_t0, ctrl._tele_rel_defer))
  return rows


def test_the_release_budget_survives_a_one_cycle_no_curve_blink(monkeypatch):
  """LOW from review, behavioural: ONE no-curve cycle inside a frozen release must not refill the 5 s budget (`_rel_defer_t0` kept)."""
  monkeypatch.setattr(VC, "cloudlog", _Log())
  base = _drive(monkeypatch)
  frozen = [r for r in base if r[3]]
  assert frozen, "the scenario must freeze"
  t_blink = frozen[len(frozen) // 2][0]
  rows = _drive(monkeypatch, blink_at=t_blink)
  after = [r for r in rows if r[0] > t_blink + DT / 2 and r[1] == "release"]
  assert after and all(r[2] == frozen[0][2] or r[2] is None for r in after)
  starts = {r[2] for r in rows if r[2] is not None and r[0] >= frozen[0][0]}
  assert len(starts) == 1                                                    # one budget for the whole curve, not two


def test_kill_switch_off_and_the_lightning_are_exactly_todays(monkeypatch):
  """Off == today: with the switch off (or on the Lightning) caps and states equal a run with the freeze forced to "" -- open AND closed
  loop, on every recorded curve. Teeth: the same comparison with the switch ON differs on OR-34."""
  frames_all = (F.OR34_LEFT, F.TERWILLIGER_LEFT, F.OLYMPIA_11, F.TERWILLIGER_2232)
  for closed in (False, True):
    for fr in frames_all:
      with monkeypatch.context() as m:
        m.setattr(VC.VTSCController, "_release_freeze", lambda self, *a: "")
        today = H.replay(m, fr, release_later=False, closed=closed)
      off = H.replay(monkeypatch, fr, release_later=False, closed=closed)
      lt = H.replay(monkeypatch, fr, fp="FORD_F_150_LIGHTNING_MK1", brand="ford", closed=closed)
      with monkeypatch.context() as m:
        m.setattr(VC.VTSCController, "_release_freeze", lambda self, *a: "")
        lt_today = H.replay(m, fr, fp="FORD_F_150_LIGHTNING_MK1", brand="ford", closed=closed)
      assert _caps(off) == _caps(today) and [r["state"] for r in off] == [r["state"] for r in today]
      assert _caps(lt) == _caps(lt_today) and [r["state"] for r in lt] == [r["state"] for r in lt_today]
      assert not any(r["pay"]["vtscRelDefer"] for r in off + lt)
  on = H.replay(monkeypatch, F.OR34_LEFT, release_later=True)
  assert _caps(on) != _caps(H.replay(monkeypatch, F.OR34_LEFT, release_later=False))


def test_the_lightning_gate_is_the_only_thing_protecting_it(monkeypatch):
  lt = H.replay(monkeypatch, F.OR34_LEFT, fp="FORD_F_150_LIGHTNING_MK1", brand="ford")
  assert lt[0]["ctrl"].veh.vtsc_release_later is False
  monkeypatch.setattr(type(lt[0]["ctrl"].veh), "vtsc_release_later", property(lambda self: True))
  forced = H.replay(monkeypatch, F.OR34_LEFT, fp="FORD_F_150_LIGHTNING_MK1", brand="ford")
  assert any(r["pay"]["vtscRelDefer"] for r in forced)


def test_vtsc_rel_defer_is_published_and_lifted_into_ces_events(monkeypatch):
  """The publisher -> ces_events contract for vtscRelDefer, in BOTH directions (a field published but not lifted evaporates)."""
  from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import VTSC_TELE_KEYS
  rows = H.replay(monkeypatch, F.OR34_LEFT, release_later=True)
  pay = next(r["pay"] for r in rows if r["pay"]["vtscRelDefer"])
  json.dumps(pay, allow_nan=False)
  assert pay["vtscRelDefer"] == "falling" and "vtscRelDefer" in VTSC_TELE_KEYS
  assert {k for k in pay if k.startswith("vtsc")} <= set(VTSC_TELE_KEYS)
