"""vtscfloor2pnw (owner 2026-09-29, "smarter is mostly better"): VTSC's set-10 mph map floor shrinks when BOTH the map and the camera
ask for much less. The three real cases are replayed through the REAL VTSCController.cap() (agreed_floor_harness.py, recorded inputs
in agreed_floor_frames.py); what they show is pinned below, including the limit the owner needs to know about: on OR-34 and
Terwilliger the camera only agrees AFTER VTSC's state machine has already released, so the shrunk floor is reported but the cap does
not change there (test_or34_...release_latched...). The feature acts where both sources agree EARLY (test_early_agreement_...).
"""
import json
import math
import random

import pytest

from openpilot.selfdrive.controls.lib.vtsc_pnw import vtsc_constants as C
from openpilot.selfdrive.controls.lib.vtsc_pnw import vtsc_controller as VC
from openpilot.selfdrive.controls.lib.vtsc_pnw.vtsc_pnw import brake_cap_for_apex, most_binding_map_curve
from openpilot.selfdrive.controls.lib.vtsc_pnw.tests import agreed_floor_frames as F
from openpilot.selfdrive.controls.lib.vtsc_pnw.tests import agreed_floor_harness as H

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
  rows = H.replay(monkeypatch, F.OR34_LEFT, agreed_floor=False)
  by_s = {}
  for r in rows:
    by_s.setdefault(int(r["t"]), r)
  t = H.secs("08:10:00")
  assert 74.5 <= by_s[int(t + 42)]["cap"] / MPH <= 76.0                  # recorded: 75
  assert by_s[int(t + 43)]["state"] == "hold"                            # recorded: hold at 08:10:43
  assert by_s[int(t + 44)]["state"] == "release"                         # recorded: release at 08:10:44
  assert by_s[int(t + 45)]["cap"] / MPH > 80.0                           # recorded: cap back up to 85 (82 at :45)
  assert all(r["why"] == "set10" for r in rows if r["t"] >= H.secs("08:10:37"))


def test_or34_agreed_floor_reaches_the_agreed_target(monkeypatch):
  """08:10:44-46: map 65 mph, camera 70 -> 66 -> 63. The floor is max(map, camera) + 1 m/s: ~72.5, 68, 66.9 mph (brief: ~66-68)."""
  rows = H.replay(monkeypatch, F.OR34_LEFT, agreed_floor=True)
  agreed = [r for r in rows if r["why"] == "agreed"]
  assert agreed and min(r["t"] for r in agreed) >= H.secs("08:10:43")
  for r in agreed:
    cam = r["vis"]
    assert r["agreed"] >= max(28.9, cam) - 0.06                             # a node at/behind the camera apex, >= the map's deepest 65 mph
    assert r["floor"] == pytest.approx(r["agreed"] + C.AGREED_FLOOR_MARGIN, abs=1e-6)
    assert r["floor"] < 75.0 * MPH                                         # below today's set-10 floor
  last = agreed[-1]
  assert 66.0 <= last["floor"] / MPH <= 68.0
  first = agreed[0]
  assert 70.0 <= first["floor"] / MPH <= 74.9


def test_or34_release_is_deferred_so_the_cap_holds_at_75(monkeypatch):
  """Release-later (owner 2026-09-29, option 2). Today the machine went hold -> release at 08:10:43.2 (the winner flipped map -> camera,
  apex 56 -> 16 m, tta 1.58 -> 0.45 s) and the cap climbed 75 -> 85 while the camera target fell to 63. Now, IN release, the cap stays
  frozen while the target falls (state machine untouched, so the re-arm to brake still runs). HONEST: the frozen cap is ~75, not the
  camera's 63-66 -- freezing does not reduce."""
  on = H.replay(monkeypatch, F.OR34_LEFT, agreed_floor=True)
  off = H.replay(monkeypatch, F.OR34_LEFT, agreed_floor=False)
  win = [(a, b) for a, b in zip(on, off, strict=True) if H.secs("08:10:43") <= a["t"] <= H.secs("08:10:47")]
  assert max(a["cap"] for a, _ in win) / MPH <= 75.3
  assert max(b["cap"] for _, b in win) / MPH > 84.0                        # today
  assert [a["state"] for a, _ in win] == [b["state"] for _, b in win]      # the state machine itself is today's
  assert any(a["pay"]["vtscRelDefer"] == "falling" for a, _ in win)
  assert all(a["pay"]["vtscRelDefer"] for a, _ in win if a["state"] == "release")   # never blank while held (Rule 2)
  assert all(a["cap"] <= b["cap"] + 1e-9 for a, b in zip(on, off, strict=True))   # never a higher cap than today


def test_terwilliger_release_is_deferred_so_the_cap_holds_at_65(monkeypatch):
  on = H.replay(monkeypatch, F.TERWILLIGER_LEFT, agreed_floor=True)
  off = H.replay(monkeypatch, F.TERWILLIGER_LEFT, agreed_floor=False)
  w = [(a, b) for a, b in zip(on, off, strict=True) if H.secs("22:35:12") <= a["t"] <= H.secs("22:35:14")]
  assert max(a["cap"] for a, _ in w) / MPH <= 65.1
  assert max(b["cap"] for _, b in w) / MPH > 68.0
  ag = [r for r in on if r["why"] == "agreed"]
  assert ag and min(r["floor"] for r in ag) / MPH < 64.9                    # the shrunk floor is now reachable (state stays hold)


def _falling_curve(v_ego=25.0, n=20, start=40.0, rate=1.2):
  """4 s far away (brake), then the apex is 5 m ahead and the camera target falls `rate` m/s each second."""
  return [(f"10:00:{s:02d}", v_ego, 38.0, start - rate * s, 200.0 if s < 4 else 5.0, None, "idle", []) for s in range(n)]


def test_release_freeze_is_bounded_and_then_climbs_as_today(monkeypatch):
  """A camera target that keeps falling cannot freeze the cap forever: after REL_DEFER_MAX_S it climbs as today (and says so)."""
  lg = _Log()
  monkeypatch.setattr(VC, "cloudlog", lg)
  rows = H.replay(monkeypatch, _falling_curve(), agreed_floor=True)
  fz = [r for r in rows if r["pay"]["vtscRelDefer"]]
  assert fz and fz[-1]["t"] - fz[0]["t"] == pytest.approx(C.REL_DEFER_MAX_S, abs=0.3)
  after = [r for r in rows if r["t"] > fz[-1]["t"] + 0.5]
  assert after and not any(r["pay"]["vtscRelDefer"] for r in after)         # the freeze is over for good
  assert any("hit its" in m for m in lg.at("warning")) and any("DEFERRED (falling)" in m for m in lg.at("info"))
  off = H.replay(monkeypatch, _falling_curve(), agreed_floor=False)
  assert all(a["cap"] <= b["cap"] + 1e-9 for a, b in zip(rows, off, strict=True))


def test_a_curve_exit_never_freezes_whatever_vego_is(monkeypatch):
  """Independent of the vEgo test: a target that is flat or RISING after release never starts a freeze, even with the car far above it."""
  for v_ego in (30.0, 40.0):
    exit_ = [(f"10:00:{s:02d}", v_ego, 45.0, 34.0 if s < 6 else 34.0 + 3.0 * (s - 6), 200.0 if s < 4 else 5.0, None, "idle", [])
             for s in range(14)]
    rows = H.replay(monkeypatch, exit_, agreed_floor=True)
    assert any(r["state"] == "release" for r in rows)
    assert not any(r["pay"]["vtscRelDefer"] for r in rows)
  tight = [(f"10:00:{s:02d}", 30.0, 38.0, 34.0 if s < 6 else 34.0 - 2.0 * (s - 6), 200.0 if s < 4 else 5.0, None, "idle", [])
           for s in range(14)]
  rows = H.replay(monkeypatch, tight, agreed_floor=True)
  assert any(r["pay"]["vtscRelDefer"] == "falling" for r in rows)            # today's release first, then the freeze once it tightens


def test_a_near_straight_target_never_freezes(monkeypatch):
  """A camera target far above vEgo (>= 1.3x) that wobbles is noise, not a curve."""
  fr = [(f"10:00:{s:02d}", 25.0, 45.0, 60.0 - 1.5 * s, 200.0 if s < 4 else 5.0, None, "idle", []) for s in range(12)]
  rows = H.replay(monkeypatch, fr, agreed_floor=True)
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
    on, off = H.replay(monkeypatch, fr, agreed_floor=True), H.replay(monkeypatch, fr, agreed_floor=False)
    assert all(a["cap"] <= b["cap"] + 1e-9 for a, b in zip(on, off, strict=True)), kw
    assert min(_caps(on)) <= min(_caps(off)) + 1e-9, kw                      # the second curve's braking is as deep as today's
    assert [a["state"] for a, _ in zip(on, off, strict=True)] == [b["state"] for _, b in zip(on, off, strict=True)]


def test_cap_is_never_higher_than_today_on_the_three_recorded_curves(monkeypatch):
  for fr in (F.OR34_LEFT, F.TERWILLIGER_LEFT, F.OLYMPIA_11):
    on, off = H.replay(monkeypatch, fr, agreed_floor=True), H.replay(monkeypatch, fr, agreed_floor=False)
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
  rows = H.replay(monkeypatch, fr, agreed_floor=True)
  fz = [r["t"] for r in rows if r["pay"]["vtscRelDefer"]]
  gaps = [b - a for a, b in zip(fz, fz[1:], strict=False) if b - a > 0.5]
  assert gaps and len(fz) > (C.REL_DEFER_MAX_S / DT) * 1.5                   # two separate freezes, each ~ the bound


def test_no_curve_and_lightning_and_kill_switch_are_unchanged(monkeypatch):
  straight = _synthetic(36.0, 38.0, 0.0, -1.0, [])
  on, off = H.replay(monkeypatch, straight, agreed_floor=True), H.replay(monkeypatch, straight, agreed_floor=False)
  assert _caps(on) == _caps(off) and all(r["pay"]["vtscRelDefer"] == "" for r in on)
  lt = H.replay(monkeypatch, F.OR34_LEFT, fp="FORD_F_150_LIGHTNING_MK1", brand="ford")
  assert all(r["pay"]["vtscRelDefer"] == "" for r in lt)
  assert any(r["state"] == "release" for r in lt)                            # today's release, untouched
  ks = H.replay(monkeypatch, F.OR34_LEFT, agreed_floor=False)
  assert all(r["pay"]["vtscRelDefer"] == "" for r in ks)
  monkeypatch.setattr(type(lt[0]["ctrl"].veh), "vtsc_agreed_floor", property(lambda self: True))
  forced = H.replay(monkeypatch, F.OR34_LEFT, fp="FORD_F_150_LIGHTNING_MK1", brand="ford")
  assert any(r["pay"]["vtscRelDefer"] for r in forced)                       # teeth: the gate alone protects the Lightning


def test_olympia_11_map_alone_is_wrong_and_stays_unchanged(monkeypatch):
  """Map said 50 mph, the camera (and the road) said 80+: the lone map value must not cut deeper. The floor is today's set-10 and the
  cap is never higher than today's; the only difference allowed is release-later holding the cap briefly while the camera really
  tightens (measured: 0.5 mph max, 3 s here)."""
  on = H.replay(monkeypatch, F.OLYMPIA_11, agreed_floor=True)
  off = H.replay(monkeypatch, F.OLYMPIA_11, agreed_floor=False)
  assert all(a <= b + 1e-9 and b - a <= 0.6 * MPH for a, b in zip(_caps(on), _caps(off), strict=True))
  assert [r["state"] for r in on] == [r["state"] for r in off]
  assert not any(r["why"] == "agreed" for r in on)
  assert any(r["why"] == "set10" for r in on)                             # the floor DID apply, at set-10, as today


# ---------------------------------------------------------------- where the feature does act

# both sources agree EARLY: map raw 25 m/s (56 mph) 300 m ahead, camera 26 m/s (58 mph) at 280 m, set 38 m/s (85 mph), car at 36 m/s
EARLY = _synthetic(36.0, 38.0, 26.0, 280.0, [(300.0, 25.0)])


def test_early_agreement_is_reported_and_never_raises_the_cap(monkeypatch):
  """Both sources agree early (map 25 m/s at 300 m, camera 26 m/s at 280 m). The floor is reported as 'agreed' (26 + 1). HONEST LIMIT: with
  the nearer-node guard the map candidate (at/behind the camera apex, target camera+1) can never out-bind the camera's OWN candidate, so
  the cap equals today's -- the floor is telemetry, not control, in every scene we have."""
  on = H.replay(monkeypatch, EARLY, agreed_floor=True)
  off = H.replay(monkeypatch, EARLY, agreed_floor=False)
  assert all(a <= b + 1e-9 for a, b in zip(_caps(on), _caps(off), strict=True))        # never higher than today
  assert all(c <= 38.0 + 1e-9 for c in _caps(on))
  ag = [r for r in on if r["why"] == "agreed"]
  assert ag and all(r["agreed"] == pytest.approx(26.0) and r["floor"] == pytest.approx(27.0) for r in ag)


def test_worst_case_decel_is_the_existing_ceiling(monkeypatch):
  """No new slam. The cap can fall no faster than the rate limiter's ceiling: 2.0 m/s^2 (regen), 2.8 (SHARP_A_DECEL_MAX, the EXISTING
  last-resort friction ceiling a raw target < 30 m/s already unlocks). Worst case, a 26 m/s target 60 m away at 36 m/s."""
  worst = 0.0
  frames_set = (EARLY, _synthetic(36.0, 38.0, 26.0, 60.0, [(60.0, 25.0)]), _synthetic(40.0, 40.0, 20.0, 40.0, [(50.0, 24.0)], 4))
  for frames in frames_set:
    rows = H.replay(monkeypatch, frames, agreed_floor=True)
    assert min(_caps(rows)) >= C.V_MIN
    worst = max(worst, _peak_decel(rows))
  assert worst <= C.SHARP_A_DECEL_MAX + 1e-6
  off_worst = max(_peak_decel(H.replay(monkeypatch, f, agreed_floor=False)) for f in frames_set)
  assert worst <= max(off_worst, C.SHARP_A_DECEL_MAX) + 1e-6              # nothing beyond what today's controller already commands


def test_fuzz_never_above_set_never_below_vmin_never_faster_than_the_ceiling(monkeypatch):
  rng = random.Random(20260929)
  for _ in range(40):
    v_set = rng.uniform(25.0, 42.0)
    v_ego = rng.uniform(15.0, v_set)
    pts = [(rng.uniform(20.0, 480.0), rng.uniform(8.0, 45.0)) for _ in range(rng.randint(1, 8))]
    cam_v = rng.choice([0.0, rng.uniform(10.0, 60.0)])
    frames = _synthetic(v_ego, v_set, cam_v, rng.uniform(0.0, 300.0), pts, seconds=6)
    rows = H.replay(monkeypatch, frames, agreed_floor=True)
    caps = _caps(rows)
    assert all(C.V_MIN - 1e-9 <= c <= v_set + 1e-9 for c in caps)
    _peak_decel(rows)


# ---------------------------------------------------------------- a lone source keeps today's floor

def test_lone_map_low_is_unchanged(monkeypatch):
  for cam_v, cam_d in ((0.0, -1.0), (40.0, 150.0)):                       # no camera curve at all / a camera that says "fine" (89 mph)
    fr = _synthetic(36.0, 38.0, cam_v, cam_d, [(300.0, 23.0)])
    on, off = H.replay(monkeypatch, fr, agreed_floor=True), H.replay(monkeypatch, fr, agreed_floor=False)
    assert _caps(on) == _caps(off)
    assert not any(r["why"] == "agreed" for r in on) and any(r["why"] == "set10" for r in on)


def test_lone_vision_low_is_unchanged(monkeypatch):
  """The camera is low but the map flags nothing (raw target above set-10): no floored point exists, so the floor never applies."""
  fr = _synthetic(36.0, 38.0, 24.0, 150.0, [(300.0, 45.0), (200.0, 60.0)])
  on, off = H.replay(monkeypatch, fr, agreed_floor=True), H.replay(monkeypatch, fr, agreed_floor=False)
  assert _caps(on) == _caps(off)
  assert all(r["why"] == "" for r in on)


def test_unpaired_camera_curve_does_not_count(monkeypatch):
  """A near camera curve (60 m) must not shrink the floor of a far map node (400 m): different curves."""
  fr = _synthetic(36.0, 38.0, 24.0, 60.0, [(400.0, 23.0)], seconds=1)
  rows = H.replay(monkeypatch, fr, agreed_floor=True)
  assert rows[0]["why"] == "set10"
  close = _synthetic(36.0, 38.0, 24.0, 300.0, [(400.0, 23.0)], seconds=1)
  assert H.replay(monkeypatch, close, agreed_floor=True)[0]["why"] == "agreed"      # 100 m apart = paired


# ---------------------------------------------------------------- stale / missing / NaN inputs fall back, and say so

class _Log:
  def __init__(self):
    self.lines = []

  def __getattr__(self, level):
    if level in ("debug", "info", "warning", "error", "exception", "critical", "event"):
      return lambda msg, *a, **k: self.lines.append((level, (msg % a) if a else msg))
    raise AttributeError(level)

  def at(self, level):
    return [m for lvl, m in self.lines if lvl == level]


def test_stale_gps_falls_back_to_the_old_floor_and_logs(monkeypatch):
  lg = _Log()
  monkeypatch.setattr(VC, "cloudlog", lg)
  on = H.replay(monkeypatch, EARLY, agreed_floor=True, gps_age=30.0)
  assert not any(r["why"] == "agreed" for r in on)
  errs = [m for m in lg.at("error") if "agreed-floor input unusable (gpsStale)" in m]
  assert len(errs) == 1                                                     # rate limited: once for the whole 8 s, not 160 times
  fresh = H.replay(monkeypatch, EARLY, agreed_floor=True, gps_age=1.4)     # the normal on-car age
  assert any(r["why"] == "agreed" for r in fresh)


def test_a_position_with_no_fix_time_is_treated_as_stale(monkeypatch):
  lg = _Log()
  monkeypatch.setattr(VC, "cloudlog", lg)
  rows = H.replay(monkeypatch, EARLY, agreed_floor=True, gps_age=False)
  assert not any(r["why"] == "agreed" for r in rows)
  assert {r["pay"]["vtscFloorSkip"] for r in rows if r["why"] == "set10"} == {"gpsNoFix"}
  assert any("(gpsNoFix)" in m for m in lg.at("error"))


def test_gps_age_limit_is_inside_the_pairing_window(monkeypatch):
  assert C.AGREED_FLOOR_GPS_MAX_AGE_S * 36.0 < C.AGREED_FLOOR_PAIR_M          # lag in metres at 80 mph
  monkeypatch.setattr(VC, "cloudlog", _Log())
  assert any(r["why"] == "agreed" for r in H.replay(monkeypatch, EARLY, agreed_floor=True, gps_age=2.9))
  rows = H.replay(monkeypatch, EARLY, agreed_floor=True, gps_age=3.1)
  assert not any(r["why"] == "agreed" for r in rows)


def test_floor_skip_telemetry_names_why_set10_stayed(monkeypatch):
  monkeypatch.setattr(VC, "cloudlog", _Log())
  none = H.replay(monkeypatch, _synthetic(36.0, 38.0, 0.0, -1.0, [(300.0, 23.0)], 1), agreed_floor=True)
  assert none[0]["pay"]["vtscFloorSkip"] == "novision"
  unp = H.replay(monkeypatch, _synthetic(36.0, 38.0, 24.0, 60.0, [(400.0, 23.0)], 1), agreed_floor=True)
  assert unp[0]["pay"]["vtscFloorSkip"] == "unpaired"
  ok = H.replay(monkeypatch, _synthetic(36.0, 38.0, 26.0, 280.0, [(300.0, 25.0)], 1), agreed_floor=True)
  assert ok[0]["pay"]["vtscFloorSkip"] == "" and ok[0]["pay"]["vtscFloorWhy"] == "agreed"


def test_future_dated_gps_fix_is_stale(monkeypatch):
  monkeypatch.setattr(VC, "cloudlog", _Log())
  rows = H.replay(monkeypatch, EARLY, agreed_floor=True, gps_age=-3.0)
  assert not any(r["why"] == "agreed" for r in rows)


def test_nan_camera_input_falls_back_and_logs(monkeypatch):
  lg = _Log()
  monkeypatch.setattr(VC, "cloudlog", lg)
  ctrl, clock = H.make_controller(monkeypatch)
  monkeypatch.setattr(VC, "model_curve_state", lambda model, v_cruise, a: (0.001, 200.0, float("nan")))
  ns = H._NS()
  ns.orientationNED = [0.0, 0.0, 0.0]
  ctrl._map_targets = [{"latitude": H.LAT0 + 300.0 / H.M_PER_DEG, "longitude": H.LON0, "velocity": 23.0}]
  ctrl._cur_lat, ctrl._cur_lon, ctrl._cur_bearing = H.LAT0, H.LON0, 0.0
  for _ in range(5):
    clock[0] += DT
    ctrl._gps_fix_ts = clock[0] - 1.4
    ctrl._last_read = clock[0]
    cap = ctrl.cap({"modelV2": object(), "carControl": ns}, 38.0, 36.0)
    assert math.isfinite(cap)
  assert ctrl._tele_floor_why == "set10"
  assert any("agreed-floor input unusable (nonfinite)" in m for m in lg.at("error"))


def test_pure_fold_bad_inputs_equal_the_unchanged_fold():
  pts = [{"latitude": H.LAT0 + 300.0 / H.M_PER_DEG, "longitude": H.LON0, "velocity": 23.0}]
  args = (pts, H.LAT0, H.LON0, 36.0, 500.0, C.A_DECEL, C.APEX_FINISH_S, C.SHARP_CURVE_V, C.MAP_SPEED_SCALE, 38.0)
  base = most_binding_map_curve(*args)
  for vv, vd in ((float("nan"), 200.0), (26.0, float("nan")), (float("inf"), 200.0), (26.0, -1.0), (0.0, 200.0), (-5.0, 200.0),
                 ("junk", 200.0), (None, 200.0)):
    info = {}
    assert most_binding_map_curve(*args, vision_v=vv, vision_d=vd, agree_margin=1.0, info=info) == base
    assert info["why"] == "set10" and info["agreed"] == 0.0 and info["skip"] in ("nonfinite", "novision")
  info = {}
  assert most_binding_map_curve(*args, vision_v=26.0, vision_d=280.0, agree_margin=1.0, info=info)[0] == pytest.approx(27.0)
  assert info == {"why": "agreed", "floor": pytest.approx(27.0), "agreed": pytest.approx(26.0), "skip": ""}
  # a raw target BELOW ~21.5 m/s is not "erased by the scale", so it is never floored and the agreed logic is never consulted
  low = [{"latitude": H.LAT0 + 300.0 / H.M_PER_DEG, "longitude": H.LON0, "velocity": 18.0}]
  info = {}
  most_binding_map_curve(low, *args[1:], vision_v=26.0, vision_d=280.0, agree_margin=1.0, info=info)
  assert info["why"] == ""


def test_the_agreed_floor_keeps_the_existing_firm_braking_flag_and_the_shallow_one_does_not():
  """A shallow set-10 floor is a synthetic trim and stays regen-only (driver 2026-08-18); an AGREED floor is a real slowdown that keeps
  the existing sharp flag (raw target < 30 m/s -> SHARP_A_DECEL_MAX may unlock), else the car is handed a deep target it cannot reach."""
  pts = [{"latitude": H.LAT0 + 300.0 / H.M_PER_DEG, "longitude": H.LON0, "velocity": 25.0}]
  args = (pts, H.LAT0, H.LON0, 36.0, 500.0, C.A_DECEL, C.APEX_FINISH_S, C.SHARP_CURVE_V, C.MAP_SPEED_SCALE, 38.0)
  assert most_binding_map_curve(*args)[2] is False                                    # today: set-10, regen only
  assert most_binding_map_curve(*args, vision_v=26.0, vision_d=280.0, agree_margin=1.0)[2] is True


def test_pure_fold_is_lower_only_and_bounded():
  """Lower-only where it matters: the binding brake cap (decel envelope over the selected point) never RISES, and an agreed floor is
  never above set-10, never below V_MIN, never below the posted-limit floor. (The selected point's own target can be higher than
  before when a nearer point becomes the binding one -- that is the envelope working, so it is the cap that is compared.)"""
  rng = random.Random(7)
  n_agreed = 0
  for _ in range(600):
    v_cap = rng.uniform(20.0, 42.0)
    v_ego = rng.uniform(15, v_cap)
    pts = [{"latitude": H.LAT0 + rng.uniform(10, 490) / H.M_PER_DEG, "longitude": H.LON0, "velocity": rng.uniform(5.0, 50.0)}
           for _ in range(rng.randint(1, 6))]
    lim = rng.choice([0.0, rng.uniform(15.0, 30.0)])
    args = (pts, H.LAT0, H.LON0, v_ego, 500.0, C.A_DECEL, C.APEX_FINISH_S, C.SHARP_CURVE_V, C.MAP_SPEED_SCALE, v_cap,
            C.MAP_MIN_SLOWDOWN, lim)
    v0, d0, _, _, _ = most_binding_map_curve(*args)
    vv, vd = rng.uniform(5.0, 60.0), rng.uniform(0.0, 500.0)
    info = {}
    v1, d1, _, _, _ = most_binding_map_curve(*args, vision_v=vv, vision_d=vd, agree_margin=C.AGREED_FLOOR_MARGIN,
                                              agree_pair_m=600.0, info=info)
    cap0 = brake_cap_for_apex(v0, d0, v_ego, C.A_DECEL, C.APEX_FINISH_S) if v0 > 0 else float("inf")
    cap1 = brake_cap_for_apex(v1, d1, v_ego, C.A_DECEL, C.APEX_FINISH_S) if v1 > 0 else float("inf")
    assert cap1 <= cap0 + 1e-9
    if info["why"] == "agreed":
      n_agreed += 1
      assert C.V_MIN - 1e-9 <= info["floor"] <= v_cap - C.MAP_MIN_SLOWDOWN + 1e-9
      assert lim <= 0.0 or info["floor"] >= lim - 1e-9
      assert info["floor"] == pytest.approx(max(info["agreed"] + C.AGREED_FLOOR_MARGIN, C.V_MIN, lim), abs=1e-6)
  assert n_agreed > 5                                                        # the property is exercised, not vacuous
  # the default (no vision args) call is the unchanged function
  assert most_binding_map_curve(*args) == most_binding_map_curve(*args, agree_margin=-1.0)


def test_the_posted_limit_floor_still_bounds_the_cap(monkeypatch):
  """freeway + posted limit 27 m/s (60 mph): even with a 26 m/s agreement the final cap never goes below the limit."""
  ctrl_rows = H.replay(monkeypatch, _synthetic(36.0, 38.0, 22.0, 280.0, [(300.0, 23.0)]), agreed_floor=True)
  assert min(_caps(ctrl_rows)) < 27.0                                       # without a limit it goes below 60 mph
  # (the limit path itself is exercised by the existing floor tests; here the fold-level bound is pinned in test_pure_fold_is_lower_only)
  info = {}
  pts = [{"latitude": H.LAT0 + 300.0 / H.M_PER_DEG, "longitude": H.LON0, "velocity": 23.0}]
  v = most_binding_map_curve(pts, H.LAT0, H.LON0, 36.0, 500.0, C.A_DECEL, C.APEX_FINISH_S, C.SHARP_CURVE_V, C.MAP_SPEED_SCALE, 38.0,
                             C.MAP_MIN_SLOWDOWN, 27.0, vision_v=22.0, vision_d=280.0, agree_margin=1.0, info=info)[0]
  assert v == pytest.approx(27.0) and info["why"] == "agreed"               # bounded by floor_limit, not agreed + margin (24)


# ---------------------------------------------------------------- Lightning / kill switch / telemetry

def test_kill_switch_off_is_todays_floor(monkeypatch):
  on = H.replay(monkeypatch, EARLY, agreed_floor=False)
  assert not any(r["why"] == "agreed" for r in on)


def test_the_lightning_never_gets_the_agreed_floor(monkeypatch):
  rows = H.replay(monkeypatch, EARLY, fp="FORD_F_150_LIGHTNING_MK1", brand="ford")
  assert rows[0]["ctrl"].veh.vtsc_agreed_floor is False
  assert not any(r["why"] == "agreed" for r in rows)
  # teeth: the capability gate is the only thing standing between the Lightning and the new floor
  monkeypatch.setattr(type(rows[0]["ctrl"].veh), "vtsc_agreed_floor", property(lambda self: True))
  forced = H.replay(monkeypatch, EARLY, fp="FORD_F_150_LIGHTNING_MK1", brand="ford")
  assert any(r["why"] == "agreed" for r in forced)


def test_telemetry_fields_reach_the_overlay_payload(monkeypatch):
  rows = H.replay(monkeypatch, EARLY, agreed_floor=True)
  ag = next(r for r in rows if r["why"] == "agreed")
  pay = ag["pay"]                                                           # the VTSCStatus dict published on that very cycle
  json.dumps(pay, allow_nan=False)                                          # strict JSON: no NaN / Infinity
  assert pay["vtscFloorWhy"] == "agreed" and pay["vtscAgreed"] == pytest.approx(26.0, abs=0.1)
  assert pay["vtscFloor"] == pytest.approx(27.0, abs=0.1)
  assert ag["floor"] == pytest.approx(pay["vtscFloor"], abs=0.1)
  off = H.replay(monkeypatch, _synthetic(36.0, 38.0, 0.0, -1.0, []), agreed_floor=True)[-1]["pay"]
  assert off["vtscFloor"] is None and off["vtscFloorWhy"] == "" and off["vtscAgreed"] is None


def test_every_vtsc_floor_and_defer_field_the_publisher_emits_is_lifted_into_ces_events(monkeypatch):
  """The contract in BOTH directions for the fields this feature adds (a field published but not lifted evaporates in /dev/shm)."""
  from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import VTSC_TELE_KEYS
  pay = H.replay(monkeypatch, EARLY, agreed_floor=True)[-1]["pay"]
  mine = {"vtscFloor", "vtscFloorWhy", "vtscFloorSkip", "vtscAgreed", "vtscRelDefer"}
  assert mine <= set(pay) and mine <= set(VTSC_TELE_KEYS)
  assert {k for k in pay if k.startswith("vtsc")} <= set(VTSC_TELE_KEYS)


def test_a_paired_map_node_nearer_than_the_camera_apex_keeps_todays_floor():
  """Opus HIGH (closed loop): a node 30 m NEARER than the camera's apex, targeted camera+1, wins the fold with a shorter time-to-apex, the
  machine goes to hold and never brakes again. So a nearer node keeps set-10; one at/behind the apex still agrees."""
  def fold(node_d):
    pts = [{"latitude": H.LAT0 + node_d / H.M_PER_DEG, "longitude": H.LON0, "velocity": 23.0}]
    info = {}
    args = (pts, H.LAT0, H.LON0, 30.0, 500.0, C.A_DECEL, C.APEX_FINISH_S, C.SHARP_CURVE_V, C.MAP_SPEED_SCALE, 33.0)
    most_binding_map_curve(*args, vision_v=25.0, vision_d=100.0, agree_margin=1.0, info=info)
    return info
  assert fold(70.0)["why"] == "set10" and fold(70.0)["skip"] == "nearer"
  assert fold(99.0)["skip"] == "nearer"
  assert fold(101.0)["why"] == "agreed" and fold(160.0)["why"] == "agreed"


def test_terwilliger_2232_closed_loop_the_car_is_never_held_above_todays_speed(monkeypatch):
  """The exact 22:32:15 geometry (map node 58.4 mph at 71 m, camera 56.1 mph at 93 m), replayed CLOSED loop: without the guard the car was
  held at 58.1 through the apex (+4.8 mph over today) because hold never brakes again."""
  on = H.replay(monkeypatch, F.TERWILLIGER_2232, agreed_floor=True, closed=True)
  off = H.replay(monkeypatch, F.TERWILLIGER_2232, agreed_floor=False, closed=True)
  assert all(a["cap"] <= b["cap"] + 0.01 * MPH for a, b in zip(on, off, strict=True))
  assert all(a["v"] <= b["v"] + 0.01 * MPH for a, b in zip(on, off, strict=True))       # the modelled car, not just the cap
  assert [r["state"] for r in on] == [r["state"] for r in off]
  assert any(r["pay"]["vtscFloorSkip"] == "nearer" for r in on)                            # the guard is what acted


def test_the_release_budget_survives_a_one_cycle_curve_flicker(monkeypatch):
  monkeypatch.setattr(VC, "cloudlog", _Log())
  ctrl, _ = _bare(monkeypatch)
  t = 0.0
  for _k in range(int(3.0 / DT)):
    t += DT
    v = 40.0 - 1.2 * t
    ctrl._tele_vis_v = v
    ctrl._rel_note(t, v)
    ctrl._release_freeze(t, True, v, 40.0)
  t0 = ctrl._rel_defer_t0
  assert t0 is not None
  # cap()'s reset block: state stays release, has_curve blinks off for one cycle -> the budget must NOT be refilled
  src = __import__("inspect").getsource(VC.VTSCController.cap)
  assert 'if self._state != "release":\n      self._rel_defer_t0 = None' in src
