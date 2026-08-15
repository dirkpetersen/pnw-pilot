"""cesnochill2pnw — the pure v_ego Schmitt-trigger latch that closes the chill-during-a-stop gap.

Field basis: drives/2026-08-15/tesla-redlight-jolt/DRIVE_REPORT.md — Tesla, no lead, stopped 11.1 s
at a red light. CES `reason` sequence: stop -> chill -> standstillHold -> stopHold -> lowSpeed. The
`chill` tick handed longitudinal to the ACC/MPC path for one cycle at v~=0 (Chill does not stop for
lights on the Tesla, CP.vEgoStopping=0.1) — the jolt (aEgo up to 2.5 m/s^2, to 6.8 m/s).

ROOT CAUSE (round 1, reproduced against the PRE-FIX code): both pre-existing standstill demotion
gates — STANDSTILL_LATCH_V's pure latch (only below 0.5 m/s) and A2's STANDSTILL_HOLD_V/
STOP_CLEAR_HOLD_S model-agreement release (0.5-1.5 m/s band) — are conditional. A model_should_stop
dropout that outlasts STOP_CLEAR_HOLD_S (2 s) while creeping in that band falls through to `else:
chill` even though the car has not genuinely moved.

ROUND-2 DETOUR (a_ego direction gate — REVERTED, Gemini review): gating arm/release on v_ego AND
a_ego together (to distinguish "decelerating through the creep band" from "accelerating through
it") introduced TWO new bugs: a permanent wedge (a gentle/leveling-off launch can sit at v_ego past
the release speed with a_ego <= the launch floor indefinitely — the AND of two independently-timed
live conditions has no guarantee of ever being simultaneously true) and up to 100 Hz status flap
from ordinary a_ego noise dithering across the floor. Both are shape bugs, not tuning issues, so
a_ego was removed ENTIRELY from this latch.

ROUND 3 (current): back to a PURE v_ego Schmitt trigger — proven wedge-free and flap-free (RELEASE
depends on v_ego alone, so it cannot require two conditions to align) — with the ARM threshold
raised to 1.0 m/s so it covers the creep band directly instead of needing to know approach
direction. See ces_pnw_constants.py's cesnochill2pnw block for the full ARM/RELEASE spec and the
wedge-impossibility proof.

Driver directive (verbatim): "CES is allowed to go to chill as soon as the car is moving to ensure
smooth acceleration but NEVER before." cesnochill2pnw closes the gap with this latch, applied as a
FINAL override after the whole existing decision (see ConditionalExperimentalSwitching.
update_decision / Ces2Core.update_decision) — so it wins over any chill decision from ANY internal
path (dwell expiry, A2, filter decay, a stop-trigger flicker), not just the two gates above.
"""
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw_constants as C
from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import ConditionalExperimentalSwitching
from openpilot.selfdrive.controls.lib.ces_pnw.ces2_core import Ces2Core

ALL_ON = {"curves": True, "stops": True, "low_speed": True, "lead": True}
ALL_ON2 = {**ALL_ON, "turns": True}
DT = 0.1


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


def count_flips(modes, start="experimental"):
  """Count mode transitions across a list of per-tick modes (prepending `start` as the state just
  before the first sample) — used to prove no flap around the release band."""
  flips = 0
  last = start
  for m in modes:
    if m != last:
      flips += 1
      last = m
  return flips


# ---------------------------------------------------------------------------
# the field replay: decel to a stop, then a stop-trigger flicker during the creep window that
# used to satisfy A2's model-agreement release — the exact mechanism, on both deciders.
# ---------------------------------------------------------------------------

def test_field_replay_no_lead_redlight_stop_never_drops_to_chill_v1():
  """Live v1 decider. Verified (via `git stash`, round 1) to reproduce the field `chill` tick at
  t=1.9s into the flicker on the pre-fix code; post-fix it must never appear, for the whole 11+ s
  stop, and must still release cleanly once the car genuinely launches."""
  sm = ConditionalExperimentalSwitching()
  # decel through the latch threshold to a genuine stop (arms the Schmitt trigger from real v_ego,
  # not a teleport straight into the vulnerable band)
  assert run(sm, sig(v_ego=6.0, model_should_stop=True), 2.0) == "experimental"   # stopIntent
  assert all(m == "experimental" for m in run_modes(sm, sig(v_ego=0.0, model_should_stop=True), 6.0))
  # the field mechanism: a slight creep (0.5-1.5 m/s band) + model_should_stop drops for long enough
  # to have cleared A2 (STOP_CLEAR_HOLD_S) on the pre-fix code
  flicker = sig(v_ego=0.7, model_should_stop=False)
  modes = run_modes(sm, flicker, 11.0)
  assert all(m == "experimental" for m in modes), "dropped to chill while stopped — the field bug"
  # the real launch: v_ego genuinely rises well past the release threshold, on open road (spd_lim
  # gates lowSpeed so ONLY the nochill latch — not an unrelated ladder rule — is under test)
  launch = sig(v_ego=25.0, v_set=25.0, spd_lim=30.0)
  assert run(sm, launch, 10.0) == "chill"


def test_field_replay_no_lead_redlight_stop_never_drops_to_chill_v2():
  """Same field replay against Ces2Core (parity fix — round 1 verified via `git stash` to
  reproduce the identical bug class at t=2.2s into the flicker on the pre-fix code)."""
  core = Ces2Core()
  assert run(core, sig2(v_ego=6.0, model_should_stop=True, mdl_end_x=5.0), 2.0) == "experimental"
  stopped = sig2(v_ego=0.0, model_should_stop=True, mdl_end_x=5.0, standstill=True)
  assert all(m == "experimental" for m in run_modes(core, stopped, 6.0))
  flicker = sig2(v_ego=0.7, model_should_stop=False, mdl_end_x=150.0, standstill=False)
  modes = run_modes(core, flicker, 11.0)
  assert all(m == "experimental" for m in modes), "dropped to chill while stopped — the field bug"
  launch = sig2(v_ego=25.0, v_set=25.0, spd_lim=30.0, mdl_end_x=500.0)
  assert run(core, launch, 10.0) == "chill"


# ---------------------------------------------------------------------------
# the Gemini-flagged gap: creep-band chill-drop approached FROM ABOVE, never dipping under the old
# (round-1) 0.5 m/s threshold — closed now purely by raising ARM_V to 1.0, no acceleration term.
# ---------------------------------------------------------------------------

def test_gemini_creep_from_above_never_dips_below_arm_v_v1():
  """The exact Gemini-flagged gap: decel from 3 m/s to a STEADY 0.7 m/s creep, never touching the
  OLD (round-1) 0.5 m/s threshold, then a >2 s model_should_stop dropout — this escaped the round-1
  latch entirely (it never armed). Now closed purely by NOCHILL_ARM_V=1.0 sitting ABOVE the field's
  0.7 m/s creep — no direction/acceleration signal involved, just a wider pure-speed threshold."""
  sm = ConditionalExperimentalSwitching()
  for v in (3.0, 2.4, 1.8, 1.3, 1.0, 0.8, 0.7):
    run(sm, sig(v_ego=v, model_should_stop=True), 0.3)
  assert sm.mode() == "experimental"
  # settles to a steady creep (0.7 m/s, inside NOCHILL_ARM_V=1.0) with model_should_stop dropped
  # for well over 2 s
  modes = run_modes(sm, sig(v_ego=0.7, model_should_stop=False), 11.0)
  assert all(m == "experimental" for m in modes), "dropped to chill on a from-above creep — the Gemini gap"
  assert sm.status() == "stopLatch"
  # a genuine launch still releases it cleanly
  assert run(sm, sig(v_ego=25.0, v_set=25.0, spd_lim=30.0), 10.0) == "chill"


def test_gemini_creep_from_above_never_dips_below_arm_v_v2():
  """Same Gemini-gap replay against Ces2Core (parity)."""
  core = Ces2Core()
  for v in (3.0, 2.4, 1.8, 1.3, 1.0, 0.8, 0.7):
    run(core, sig2(v_ego=v, model_should_stop=True, mdl_end_x=5.0, standstill=False), 0.3)
  assert core.mode() == "experimental"
  modes = run_modes(core, sig2(v_ego=0.7, model_should_stop=False, mdl_end_x=150.0, standstill=False),
                    11.0)
  assert all(m == "experimental" for m in modes), "dropped to chill on a from-above creep — the Gemini gap"
  assert core.status() == "stopLatch"
  assert run(core, sig2(v_ego=25.0, v_set=25.0, spd_lim=30.0, mdl_end_x=500.0, standstill=False),
             10.0) == "chill"


# ---------------------------------------------------------------------------
# the driver's directive, tested directly
# ---------------------------------------------------------------------------

def test_never_chill_below_arm_v_even_with_nothing_active():
  """Driver directive is unconditional ("Full stop.") — even with NO CES condition active at all
  (no lead, model never asked to stop — e.g. idling with cruise engaged), v_ego at rest must still
  read Experimental, never Chill."""
  sm = ConditionalExperimentalSwitching()
  idle = sig(v_ego=0.0)   # decide_active(idle) == (False, "chill"): nothing the ladder wants
  modes = run_modes(sm, idle, 5.0)
  assert all(m == "experimental" for m in modes)
  assert sm.status() == "stopLatch"   # the latch is the ONLY thing holding it


def test_release_is_purely_speed_based_with_hysteresis_against_sensor_twitch():
  """A twitch inside the [ARM_V, RELEASE_V) hysteresis gap must not release the latch; only a real
  crossing past NOCHILL_RELEASE_V does — no green-light, lead-departure, or acceleration signal is
  consulted at all (round 2's a_ego term was reverted for exactly this kind of extra-condition
  fragility)."""
  sm = ConditionalExperimentalSwitching()
  run(sm, sig(v_ego=0.0, model_should_stop=True), 2.0)
  assert sm.mode() == "experimental"
  twitch = sig(v_ego=C.NOCHILL_ARM_V + 0.15)   # inside the hysteresis gap (below RELEASE_V)
  assert run(sm, twitch, 1.0) == "experimental"
  real_launch = sig(v_ego=C.NOCHILL_RELEASE_V + 0.05, v_set=12.5, spd_lim=30.0)
  assert run(sm, real_launch, 0.1) == "experimental"   # crossed release_v, but core's own dwell
                                                       # machinery (unmodified) still governs from
                                                       # here — see the field-replay test for the
                                                       # full teleport-to-cruise release proof


def test_no_flap_dithering_near_the_release_band():
  """Gemini's round-2 flap concern, re-tested for the pure-v_ego design: v_ego dithering back and
  forth right around NOCHILL_RELEASE_V (the only edge that can demote) must produce AT MOST ONE
  mode transition — ARM_V < RELEASE_V is the entire anti-flap mechanism, and there is no second
  (e.g. acceleration) signal left to introduce its own independent noise."""
  sm = ConditionalExperimentalSwitching()
  run(sm, sig(v_ego=0.0, model_should_stop=True), 5.0)
  assert sm.mode() == "experimental"
  dither = []
  for i in range(200):
    v = C.NOCHILL_RELEASE_V + (0.02 if i % 2 == 0 else -0.02)   # +/- 0.02 m/s straddling the edge
    dither.append(sm.update_decision(sig(v_ego=v, v_set=12.5, spd_lim=30.0), DT))
  assert count_flips(dither, start="experimental") <= 1


# ---------------------------------------------------------------------------
# no-wedge proof: the latch cannot stick forever after a normal drive-away, including a gentle or
# leveling-off launch (the exact shape that wedged the round-2 a_ego design).
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
  assert run(sm, sig(v_ego=20.0, v_set=25.0, spd_lim=30.0), 10.0) == "chill"
  # a fresh, ordinary at-speed cruise afterwards: no leftover latch state anywhere close to tripping
  cruise = sig(v_ego=25.0, v_set=25.0, spd_lim=30.0)
  assert run(sm, cruise, 5.0) == "chill"


def test_gentle_launch_that_levels_off_at_release_still_releases():
  """The EXACT shape that permanently wedged round-2's a_ego design: a gentle launch whose speed
  rises slowly and then LEVELS OFF (a_ego -> 0) right around the release band — e.g. cruise control
  smoothly catching a low target speed. With a pure v_ego predicate this is a non-issue: the single
  tick where v_ego first exceeds NOCHILL_RELEASE_V releases the latch, full stop, regardless of
  what the acceleration does before or after."""
  sm = ConditionalExperimentalSwitching()
  run(sm, sig(v_ego=0.0, model_should_stop=True), 5.0)
  assert sm.mode() == "experimental"
  # gentle ramp up to just past the release speed, then hold flat there (a real "leveled off" launch)
  for v in (0.3, 0.6, 0.9, 1.1, 1.35):
    run(sm, sig(v_ego=v, v_set=12.5, spd_lim=30.0), 0.5)
  level = sig(v_ego=C.NOCHILL_RELEASE_V + 0.05, v_set=12.5, spd_lim=30.0)   # flat, not accelerating
  assert run(sm, level, 10.0) == "chill", "wedged on a leveled-off launch — the round-2 bug class"


def test_latch_re_engages_on_the_next_stop_after_a_release():
  """The latch is a live per-tick predicate, not a one-shot: after releasing on a drive-away it must
  arm again, correctly, on the NEXT stop (a second red light) — no stuck-off state either."""
  sm = ConditionalExperimentalSwitching()
  run(sm, sig(v_ego=0.0, model_should_stop=True), 5.0)
  run(sm, sig(v_ego=20.0, v_set=25.0, spd_lim=30.0), 10.0)
  assert sm.mode() == "chill"
  # second light: decel to a genuine stop again (arms the latch from real v_ego), then the same
  # creep + stop-trigger flicker as the field replay
  run(sm, sig(v_ego=5.0, model_should_stop=True), 2.0)
  run(sm, sig(v_ego=0.0, model_should_stop=True), 3.0)
  modes = run_modes(sm, sig(v_ego=0.6, model_should_stop=False), 5.0)
  assert all(m == "experimental" for m in modes)
