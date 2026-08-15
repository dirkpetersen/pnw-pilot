"""cesnochill2pnw — the direction-aware latch that closes the chill-during-a-stop gap.

Field basis: drives/2026-08-15/tesla-redlight-jolt/DRIVE_REPORT.md — Tesla, no lead, stopped 11.1 s
at a red light. CES `reason` sequence: stop -> chill -> standstillHold -> stopHold -> lowSpeed. The
`chill` tick handed longitudinal to the ACC/MPC path for one cycle at v~=0 (Chill does not stop for
lights on the Tesla, CP.vEgoStopping=0.1) — the jolt (aEgo up to 2.5 m/s^2, to 6.8 m/s).

ROOT CAUSE (round 1, reproduced against the PRE-FIX code): both pre-existing standstill demotion
gates — STANDSTILL_LATCH_V's pure latch (only below 0.5 m/s) and A2's STANDSTILL_HOLD_V/
STOP_CLEAR_HOLD_S model-agreement release (0.5-1.5 m/s band) — are conditional. A model_should_stop
dropout that outlasts STOP_CLEAR_HOLD_S (2 s) while creeping in that band falls through to `else:
chill` even though the car has not genuinely moved.

ROUND-1 GAP (Gemini review): the first fix's latch only armed at v_ego < 0.5. A creep-band
chill-drop APPROACHED FROM ABOVE — decelerating from, say, 3 m/s to a steady 0.7-1.4 m/s WITHOUT
ever dipping under 0.5 — was still exposed, because a pure v_ego threshold cannot tell "decelerating/
creeping toward a stop" from "accelerating away on a launch" at the same speed. `test_field_replay_*`
and `test_gemini_creep_from_above_never_dips_below_stop_v` below reproduce and close exactly this.

Driver directive (verbatim): "CES is allowed to go to chill as soon as the car is moving to ensure
smooth acceleration but NEVER before." cesnochill2pnw closes the gap with a DIRECTION-AWARE latch
(v_ego AND a_ego together — see ces_pnw_constants.py's cesnochill2pnw block for the full ARM/STAY/
RELEASE spec), applied as a FINAL override after the whole existing decision core in both
ConditionalExperimentalSwitching and Ces2Core — so it wins over any chill decision from ANY internal
path, not just the two gates above.
"""
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw_constants as C
from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import ConditionalExperimentalSwitching
from openpilot.selfdrive.controls.lib.ces_pnw.ces2_core import Ces2Core

ALL_ON = {"curves": True, "stops": True, "low_speed": True, "lead": True}
ALL_ON2 = {**ALL_ON, "turns": True}
DT = 0.1
# a plausible "genuinely launching" acceleration -- comfortably above NOCHILL_LAUNCH_A(0.1) and
# well under the field's observed jolt peak (1.8-2.5 m/s^2), so it never itself reads as noise
LAUNCH_A = 2.0


def sig(**kw):
  s = {
    "v_ego": 30.0, "has_lead": False, "lead_vlead": 0.0, "lead_drel": 0.0, "blinker": False,
    "map_target_v": 0.0, "map_target_dist": float('inf'),
    "curve_lat_accel_vision": 0.0, "time_to_curve": 10.0,
    "model_should_stop": False, "v_set": 12.5, "spd_lim": 0.0, "toggles": ALL_ON,
  }
  s.update(kw)
  return s


def sig2(**kw):
  """Same primitives, plus the CES2-only keys (Ces2Core.update_decision requires them)."""
  s = sig(**kw)
  s["toggles"] = ALL_ON2
  s.setdefault("mdl_end_x", 0.0)
  s.setdefault("stop_urgency", 0)
  s.setdefault("lead_opening", False)
  s.setdefault("standstill", s["v_ego"] < 0.3)
  s.setdefault("lane_change_intent", False)
  return s


def run(core, s, seconds, dt=DT):
  for _ in range(int(round(seconds / dt))):
    core.update_decision(s, dt)
  return core.mode()


def run_modes(core, s, seconds, dt=DT):
  """Like run(), but returns every per-tick mode() — so a test can assert 'never chill for the
  WHOLE window', not just 'not chill at the end tick' (a single-tick chill is exactly the bug)."""
  modes = []
  for _ in range(int(round(seconds / dt))):
    core.update_decision(s, dt)
    modes.append(core.mode())
  return modes


# ---------------------------------------------------------------------------
# the field replay: decel to a stop, then a stop-trigger flicker during the creep window that
# used to satisfy A2's model-agreement release — the exact mechanism, on both deciders.
# ---------------------------------------------------------------------------

def test_field_replay_no_lead_redlight_stop_never_drops_to_chill_v1():
  """Live v1 decider. Verified (via `git stash`, round 1) to reproduce the field `chill` tick at
  t=1.9s into the flicker on the pre-fix code; post-fix it must never appear, for the whole 11+ s
  stop, and must still release cleanly once the car genuinely launches."""
  sm = ConditionalExperimentalSwitching()
  # decel through the arm threshold to a genuine stop (arms the latch from real v_ego/a_ego, not a
  # teleport straight into the vulnerable band)
  assert run(sm, sig(v_ego=6.0, model_should_stop=True, a_ego=-2.0), 2.0) == "experimental"   # stopIntent
  assert all(m == "experimental" for m in run_modes(sm, sig(v_ego=0.0, model_should_stop=True), 6.0))
  # the field mechanism: a slight creep (0.5-1.5 m/s band, a_ego~=0 -- NOT a launch) + model_should_stop
  # drops for long enough to have cleared A2 (STOP_CLEAR_HOLD_S) on the pre-fix code
  flicker = sig(v_ego=0.7, model_should_stop=False, a_ego=0.0)
  modes = run_modes(sm, flicker, 11.0)
  assert all(m == "experimental" for m in modes), "dropped to chill while stopped — the field bug"
  # the real launch: v_ego genuinely rises AND a_ego confirms it, on open road (spd_lim gates
  # lowSpeed so ONLY the nochill latch — not an unrelated ladder rule — is under test)
  launch = sig(v_ego=25.0, v_set=25.0, spd_lim=30.0, a_ego=LAUNCH_A)
  assert run(sm, launch, 10.0) == "chill"


def test_field_replay_no_lead_redlight_stop_never_drops_to_chill_v2():
  """Same field replay against Ces2Core (parity fix — round 1 verified via `git stash` to
  reproduce the identical bug class at t=2.2s into the flicker on the pre-fix code)."""
  core = Ces2Core()
  assert run(core, sig2(v_ego=6.0, model_should_stop=True, mdl_end_x=5.0, a_ego=-2.0), 2.0) == "experimental"
  stopped = sig2(v_ego=0.0, model_should_stop=True, mdl_end_x=5.0, standstill=True)
  assert all(m == "experimental" for m in run_modes(core, stopped, 6.0))
  flicker = sig2(v_ego=0.7, model_should_stop=False, mdl_end_x=150.0, standstill=False, a_ego=0.0)
  modes = run_modes(core, flicker, 11.0)
  assert all(m == "experimental" for m in modes), "dropped to chill while stopped — the field bug"
  launch = sig2(v_ego=25.0, v_set=25.0, spd_lim=30.0, mdl_end_x=500.0, a_ego=LAUNCH_A)
  assert run(core, launch, 10.0) == "chill"


# ---------------------------------------------------------------------------
# round-2 (Gemini): the creep-band gap approached from ABOVE, never dipping under the old 0.5 m/s
# threshold — decel from highway-adjacent speed straight to a steady low creep.
# ---------------------------------------------------------------------------

def test_gemini_creep_from_above_never_dips_below_stop_v_v1():
  """The exact Gemini-flagged gap: decel from 3 m/s to a STEADY 0.7 m/s creep, never touching the
  old 0.5 m/s threshold, then a >2 s model_should_stop dropout — this used to (round-1 code) still
  chill, because the old latch never armed (v_ego never went below 0.5). The direction-aware arm
  condition (v_ego < NOCHILL_ARM_V(1.5) AND a_ego <= NOCHILL_LAUNCH_A) fires DURING the decel, well
  before the car ever gets close to the old threshold."""
  sm = ConditionalExperimentalSwitching()
  # decelerating through the band: 3.0 -> 0.7, always decelerating (a_ego < 0), never below 0.5
  for v in (3.0, 2.4, 1.8, 1.3, 1.0, 0.8, 0.7):
    run(sm, sig(v_ego=v, model_should_stop=True, a_ego=-1.5), 0.3)
  assert sm.mode() == "experimental"
  # settles to a steady creep (a_ego ~= 0) with model_should_stop dropped for well over 2 s
  modes = run_modes(sm, sig(v_ego=0.7, model_should_stop=False, a_ego=0.0), 11.0)
  assert all(m == "experimental" for m in modes), "dropped to chill on a from-above creep — the Gemini gap"
  assert sm.status() == "stopLatch"
  # a genuine launch still releases it cleanly
  assert run(sm, sig(v_ego=25.0, v_set=25.0, spd_lim=30.0, a_ego=LAUNCH_A), 10.0) == "chill"


def test_gemini_creep_from_above_never_dips_below_stop_v_v2():
  """Same Gemini-gap replay against Ces2Core (parity)."""
  core = Ces2Core()
  for v in (3.0, 2.4, 1.8, 1.3, 1.0, 0.8, 0.7):
    run(core, sig2(v_ego=v, model_should_stop=True, mdl_end_x=5.0, standstill=False, a_ego=-1.5), 0.3)
  assert core.mode() == "experimental"
  modes = run_modes(core, sig2(v_ego=0.7, model_should_stop=False, mdl_end_x=150.0, standstill=False,
                               a_ego=0.0), 11.0)
  assert all(m == "experimental" for m in modes), "dropped to chill on a from-above creep — the Gemini gap"
  assert core.status() == "stopLatch"
  assert run(core, sig2(v_ego=25.0, v_set=25.0, spd_lim=30.0, mdl_end_x=500.0, standstill=False,
                        a_ego=LAUNCH_A), 10.0) == "chill"


def test_decel_to_full_stop_then_launch_releases_only_on_real_launch():
  """The companion case Gemini asked for explicitly: a normal decel all the way to a full stop,
  then a launch — must still release to chill ONLY once the launch is real (v AND a_ego both
  confirm it), never on speed alone and never on accel alone."""
  sm = ConditionalExperimentalSwitching()
  for v in (6.0, 4.0, 2.0, 0.5, 0.0):
    run(sm, sig(v_ego=v, model_should_stop=True, a_ego=-1.5), 0.4)
  assert run(sm, sig(v_ego=0.0, model_should_stop=True), 3.0) == "experimental"
  # early launch: accelerating (a_ego > threshold) but still under the release SPEED -> still held
  modes = run_modes(sm, sig(v_ego=0.4, model_should_stop=False, a_ego=LAUNCH_A), 1.0)
  assert all(m == "experimental" for m in modes)
  # crosses the release speed while still genuinely accelerating -> releases
  assert run(sm, sig(v_ego=6.8, v_set=12.5, spd_lim=30.0, a_ego=LAUNCH_A), 10.0) == "chill"


# ---------------------------------------------------------------------------
# the driver's directive, tested directly
# ---------------------------------------------------------------------------

def test_never_chill_below_stop_v_even_with_nothing_active():
  """Driver directive is unconditional ("Full stop.") — even with NO CES condition active at all
  (no lead, model never asked to stop — e.g. idling with cruise engaged), v_ego at rest and not
  accelerating must still read Experimental, never Chill."""
  sm = ConditionalExperimentalSwitching()
  idle = sig(v_ego=0.0)   # decide_active(idle) == (False, "chill"): nothing the ladder wants
  modes = run_modes(sm, idle, 5.0)
  assert all(m == "experimental" for m in modes)
  assert sm.status() == "stopLatch"   # the latch is the ONLY thing holding it, unconditionally shown


def test_release_requires_both_speed_and_acceleration():
  """Neither condition alone releases the latch: speed past the threshold while still decelerating
  or coasting (a_ego <= the launch floor) must NOT release; nor may a burst of acceleration that
  never actually crosses the release speed. No green-light or lead-departure signal is consulted."""
  sm = ConditionalExperimentalSwitching()
  run(sm, sig(v_ego=0.0, model_should_stop=True), 2.0)
  assert sm.mode() == "experimental"
  # speed alone: past NOCHILL_RELEASE_V but NOT accelerating (e.g. rolling on residual momentum)
  coasting = sig(v_ego=C.NOCHILL_RELEASE_V + 0.5, v_set=12.5, spd_lim=30.0, a_ego=0.0)
  assert run(sm, coasting, 1.0) == "experimental"
  # accel alone: genuinely accelerating but still under the release speed
  early = sig(v_ego=C.NOCHILL_RELEASE_V - 0.2, v_set=12.5, spd_lim=30.0, a_ego=LAUNCH_A)
  assert run(sm, early, 1.0) == "experimental"
  # both together: releases
  real_launch = sig(v_ego=C.NOCHILL_RELEASE_V + 0.5, v_set=12.5, spd_lim=30.0, a_ego=LAUNCH_A)
  assert run(sm, real_launch, 1.0) == "experimental"   # crossed release_v, but core's own dwell
                                                       # machinery (unmodified) still governs from
                                                       # here — see the field-replay test for the
                                                       # full teleport-to-cruise release proof


def test_bad_a_ego_reading_fails_safe_to_holding():
  """A None/NaN/non-numeric a_ego reading must never let the latch release early (the fail-safe
  direction is 'keep holding', matching every other guard in this file) — even with v_ego well
  past the release speed."""
  sm = ConditionalExperimentalSwitching()
  run(sm, sig(v_ego=0.0, model_should_stop=True), 2.0)
  for bad_a in (None, float('nan'), "garbage", object()):
    fast_but_bad = sig(v_ego=25.0, v_set=25.0, spd_lim=30.0, a_ego=bad_a)
    assert run(sm, fast_but_bad, 0.5) == "experimental", f"released on bad a_ego={bad_a!r}"


# ---------------------------------------------------------------------------
# no-wedge proof: the latch cannot stick forever after a normal drive-away
# ---------------------------------------------------------------------------

def test_latch_cannot_wedge_forever_normal_drive_away_then_cruise():
  """Correctness/no-wedge proof: after the latch releases on a genuine launch, ordinary cruising
  resumes with no residual state (mirrors test_standstill.py's
  test_dwell_expiry_works_again_at_speed_after_standstill_episode for the older standstill
  machinery), now exercised through a cesnochill2pnw-latched episode."""
  sm = ConditionalExperimentalSwitching()
  run(sm, sig(v_ego=0.0, model_should_stop=True), 5.0)
  assert sm.mode() == "experimental"
  # drive away normally and keep going — open road, no lead: nothing should re-trip Experimental
  assert run(sm, sig(v_ego=20.0, v_set=25.0, spd_lim=30.0, a_ego=LAUNCH_A), 10.0) == "chill"
  # a fresh, ordinary at-speed cruise afterwards (a_ego settles to ~0 once at target speed): no
  # leftover latch state anywhere close to tripping again
  cruise = sig(v_ego=25.0, v_set=25.0, spd_lim=30.0, a_ego=0.0)
  assert run(sm, cruise, 5.0) == "chill"


def test_latch_re_engages_on_the_next_stop_after_a_release():
  """The latch is a live per-tick predicate, not a one-shot: after releasing on a drive-away it must
  arm again, correctly, on the NEXT stop (a second red light) — no stuck-off state either."""
  sm = ConditionalExperimentalSwitching()
  run(sm, sig(v_ego=0.0, model_should_stop=True), 5.0)
  run(sm, sig(v_ego=20.0, v_set=25.0, spd_lim=30.0, a_ego=LAUNCH_A), 10.0)
  assert sm.mode() == "chill"
  # second light: decel to a genuine stop again (arms the latch from real v_ego/a_ego), then the
  # same creep + stop-trigger flicker as the field replay
  run(sm, sig(v_ego=5.0, model_should_stop=True, a_ego=-1.5), 2.0)
  run(sm, sig(v_ego=0.0, model_should_stop=True), 3.0)
  modes = run_modes(sm, sig(v_ego=0.6, model_should_stop=False, a_ego=0.0), 5.0)
  assert all(m == "experimental" for m in modes)
