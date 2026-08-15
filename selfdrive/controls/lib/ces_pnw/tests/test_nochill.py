"""cesnochill2pnw — the hard, purely speed-based latch that closes the chill-during-a-stop gap.

Field basis: drives/2026-08-15/tesla-redlight-jolt/DRIVE_REPORT.md — Tesla, no lead, stopped 11.1 s
at a red light. CES `reason` sequence: stop -> chill -> standstillHold -> stopHold -> lowSpeed. The
`chill` tick handed longitudinal to the ACC/MPC path for one cycle at v~=0 (Chill does not stop for
lights on the Tesla, CP.vEgoStopping=0.1) — the jolt (aEgo up to 2.5 m/s^2, to 6.8 m/s).

ROOT CAUSE (reproduced below against the PRE-FIX code): both pre-existing standstill demotion gates
— STANDSTILL_LATCH_V's pure latch (only below 0.5 m/s) and A2's STANDSTILL_HOLD_V/STOP_CLEAR_HOLD_S
model-agreement release (0.5-1.5 m/s band) — are conditional. A model_should_stop dropout that
outlasts STOP_CLEAR_HOLD_S (2 s) while creeping in that 0.5-1.5 m/s band falls through to `else:
chill` even though the car has not genuinely moved. `test_field_replay_*` below reproduces this
EXACT mechanism (verified to fail pre-fix, `git stash` + rerun) and proves it can no longer happen.

Driver directive (verbatim): "CES is allowed to go to chill as soon as the car is moving to ensure
smooth acceleration but NEVER before." cesnochill2pnw closes the gap with an unconditional, purely
v_ego-based Schmitt-trigger latch applied as a FINAL override after the whole existing decision (see
ConditionalExperimentalSwitching.update_decision / Ces2Core.update_decision) — so it wins over any
chill decision from ANY internal path (dwell expiry, A2, filter decay, a stop-trigger flicker), not
just the two gates above. See ces_pnw_constants.py's cesnochill2pnw block for the full design note
and the correctness argument that this cannot wedge Experimental forever.
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


# ---------------------------------------------------------------------------
# the field replay: decel to a stop, then a stop-trigger flicker during the creep window that
# used to satisfy A2's model-agreement release — the exact mechanism, on both deciders.
# ---------------------------------------------------------------------------

def test_field_replay_no_lead_redlight_stop_never_drops_to_chill_v1():
  """Live v1 decider. Verified (via `git stash`) to reproduce the field `chill` tick at t=1.9s into
  the flicker on the pre-fix code; post-fix it must never appear, for the whole 11+ s stop, and must
  still release cleanly once the car genuinely launches."""
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
  """Same field replay against Ces2Core (parity fix — verified via `git stash` to reproduce the
  identical bug class at t=2.2s into the flicker on the pre-fix code)."""
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
# the driver's directive, tested directly
# ---------------------------------------------------------------------------

def test_never_chill_below_stop_v_even_with_nothing_active():
  """Driver directive is unconditional ("Full stop.") — even with NO CES condition active at all
  (no lead, model never asked to stop — e.g. idling with cruise engaged), v_ego at rest must still
  read Experimental, never Chill."""
  sm = ConditionalExperimentalSwitching()
  idle = sig(v_ego=0.0)   # decide_active(idle) == (False, "chill"): nothing the ladder wants
  modes = run_modes(sm, idle, 5.0)
  assert all(m == "experimental" for m in modes)
  assert sm.status() == "stopLatch"   # the latch is the ONLY thing holding it


def test_release_is_purely_speed_based_with_hysteresis_against_sensor_twitch():
  """A 0.1-0.2 m/s twitch at rest must not release the latch (Schmitt-trigger gap: RELEASE_V >
  STOP_V); only a real crossing past NOCHILL_RELEASE_V does — no green-light or lead-departure
  signal is consulted at all."""
  sm = ConditionalExperimentalSwitching()
  run(sm, sig(v_ego=0.0, model_should_stop=True), 2.0)
  assert sm.mode() == "experimental"
  twitch = sig(v_ego=C.NOCHILL_STOP_V + 0.15)   # inside the hysteresis gap (below RELEASE_V)
  assert run(sm, twitch, 1.0) == "experimental"
  real_launch = sig(v_ego=C.NOCHILL_RELEASE_V + 0.05, v_set=12.5, spd_lim=30.0)
  assert run(sm, real_launch, 0.1) == "experimental"   # crossed release_v, but core's own dwell
                                                       # machinery (unmodified) still governs from
                                                       # here — see the field-replay test for the
                                                       # full teleport-to-cruise release proof


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
  assert run(sm, sig(v_ego=20.0, v_set=25.0, spd_lim=30.0), 10.0) == "chill"
  # a fresh, ordinary at-speed cruise afterwards: no leftover latch state anywhere close to tripping
  cruise = sig(v_ego=25.0, v_set=25.0, spd_lim=30.0)
  assert run(sm, cruise, 5.0) == "chill"


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
