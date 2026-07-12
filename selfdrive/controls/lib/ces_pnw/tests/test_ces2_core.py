"""
ces2core2pnw — unit tests for the CES2 decision core (CES2-STUDY.md adoptions).

Covers, per the build spec:
  - graded stop-urgency (endpoint_urgency + StopEvidence hysteretic decay)
  - the PRECEDENCE PRINCIPLE: stop evidence beats accelerate-preference at 5 / 15 / 50 mph
    (the retired 8 m/s red-light floor's job, without the floor)
  - CEM standstill hold (Experimental held at v=0 while the model still says stopped)
  - CEM turn-signal condition (blinker + no lane-change intent below 55 mph, CESTurns toggle)
  - per-condition debounce filters (a decaying curve charge cannot mask a stop's rise)
  - shadow divergence-edge counting (DivergenceCounter)
"""
import math

from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw_constants as C
from openpilot.selfdrive.controls.lib.ces_pnw import ces2_core as C2
from openpilot.selfdrive.controls.lib.ces_pnw.ces2_core import (
  Ces2Core, DivergenceCounter, StopEvidence, ces2_conditions, decide_active2, endpoint_urgency,
  LEVEL_LOW, LEVEL_MED, LEVEL_HIGH, URGENCY_MED, URGENCY_HIGH,
)

ALL_ON = {"curves": True, "stops": True, "low_speed": True, "lead": True, "turns": True}
MPH = CV.MPH_TO_MS


def base(**kw):
  """Cruising, nothing happening (mirror of test_ces_pnw.base + the CES2-only keys)."""
  s = {
    "v_ego": 30.0, "has_lead": False, "lead_vlead": 0.0, "lead_drel": 0.0, "blinker": False,
    "map_target_v": 0.0, "map_target_dist": float('inf'),
    "curve_lat_accel_vision": 0.0, "time_to_curve": 10.0,
    "model_should_stop": False, "v_set": 0.0, "spd_lim": 0.0, "toggles": ALL_ON,
    "mdl_end_x": 0.0, "stop_urgency": LEVEL_LOW, "lead_opening": False,
    "standstill": False, "lane_change_intent": False,
  }
  s.update(kw)
  return s


# ---------------------------------------------------------------------------
# 1) urgency grading
# ---------------------------------------------------------------------------
def test_urgency_zero_on_long_endpoint():
  # endpoint pinned at the far horizon (on-ramp signature) -> no stop evidence at any speed
  for mph in (5, 15, 50):
    assert endpoint_urgency(mph * MPH, 200.0, False) == 0.0


def test_urgency_neutral_when_endpoint_unknown():
  # mdl_end_x <= 0 = not plumbed / model hiccup -> NEVER invent stopping evidence
  assert endpoint_urgency(20.0, 0.0, False) == 0.0
  assert endpoint_urgency(20.0, None, False) == 0.0
  # ... but binary shouldStop still saturates
  assert endpoint_urgency(20.0, 0.0, True) == 1.0


def test_urgency_grades_with_shortage():
  # 50 kph -> expected 130 m. Mild contraction < deep contraction < critical.
  v = 50 / 3.6
  mild = endpoint_urgency(v, 110.0, False)      # ratio 0.154*2 = 0.31 (then speed factor)
  deep = endpoint_urgency(v, 60.0, False)
  critical = endpoint_urgency(v, 20.0, False)   # < 30% of expected -> doubled -> saturates
  assert 0.0 < mild < deep <= critical == 1.0


def test_urgency_speed_indexed():
  # the same 36 m endpoint is benign at parking speed, alarming at road speed
  slow = endpoint_urgency(5 / 3.6, 36.0, False)     # expected 39m at 5 kph -> tiny shortage
  fast = endpoint_urgency(50 / 3.6, 36.0, False)    # expected 130m -> huge shortage
  assert slow < URGENCY_MED <= fast


def test_stop_evidence_rises_instantly_decays_hysteretically():
  ev = StopEvidence()
  assert ev.update(10.0, 0.0, True, 0.01) == LEVEL_HIGH   # shouldStop -> HIGH in one tick
  assert ev.urgency == 1.0
  # signal drops: decay, not snap — still >= MEDIUM within the ~2 s guard window
  t = 0.0
  while t < 1.5:
    ev.update(10.0, 0.0, False, 0.1)
    t += 0.1
  assert ev.level >= LEVEL_MED            # yellow-light trap: still blocking accel-preference
  while t < 6.0:
    ev.update(10.0, 0.0, False, 0.1)
    t += 0.1
  assert ev.level == LEVEL_LOW            # fully cleared after the guard


# ---------------------------------------------------------------------------
# 2) precedence: stop evidence beats accelerate-preference at ANY speed
# ---------------------------------------------------------------------------
def test_precedence_beats_accel_zone_at_any_speed():
  # red-light approach: no lead, set >> ego (the accel-zone signature) + contracting endpoint.
  # v1 needed the 8 m/s floor to protect the low-speed band; CES2 needs no floor at any speed.
  for mph in (5, 15, 50):
    v = mph * MPH
    # live short endpoint (consistent with MED urgency) so the BLIND fallback is not what blocks
    s = base(v_ego=v, v_set=v + 10.0, stop_urgency=LEVEL_MED, mdl_end_x=25.0)
    conds, gates = ces2_conditions(s)
    assert not gates["wantFaster"], f"accelerate-preference must yield to stop evidence at {mph} mph"
    assert conds["stop"], f"stop condition must hold at {mph} mph"
    active, reason = decide_active2(s)
    assert active and reason == "stop"


def test_no_stop_evidence_keeps_the_merge_chill():
  # on-ramp merge: same open-road + set >> ego signals, but endpoint LIVE at the horizon (urgency
  # LOW) -> WANT-FASTER suppresses lowSpeed AND curve (on-ramp IS a curve), exactly the v1
  # accel-zone behavior — including BELOW the old 8 m/s floor (5 mph), where v1 needed the
  # carve-out. A LONG endpoint is the positive open-road evidence that retires the floor.
  for mph in (5, 15, 38):
    v = mph * MPH
    s = base(v_ego=v, v_set=90 * MPH, stop_urgency=LEVEL_LOW, mdl_end_x=200.0,
             curve_lat_accel_vision=2.5, time_to_curve=2.0)   # the ramp curve the camera sees
    conds, gates = ces2_conditions(s)
    assert gates["wantFaster"]
    assert not conds["lowSpeed"] and not conds["curve"]
    assert not decide_active2(s)[0]


def test_blind_endpoint_reinstates_the_floor():
  """Gemini adversarial catch (Trap 1): with NO endpoint signal (mdl_end_x 0/unknown), the graded
  urgency is blind — the accelerate preference must then behave exactly like v1's 8 m/s floor:
  no-lead below the floor -> NEVER want-faster (presumed red-light approach); above the floor and
  lead-present cases keep v1 semantics."""
  # below the floor, no lead, blind -> suppressed (v1 floor behavior)
  for mph in (5, 15):
    s = base(v_ego=mph * MPH, v_set=90 * MPH, mdl_end_x=0.0)
    conds, gates = ces2_conditions(s)
    assert not gates["wantFaster"], f"blind endpoint must reinstate the floor at {mph} mph"
    assert conds["lowSpeed"]                     # the low-speed Experimental hold stays
  # above the floor, blind -> want-faster allowed (v1 parity: the floor only bit below 8 m/s)
  s = base(v_ego=20 * MPH, v_set=90 * MPH, mdl_end_x=0.0)
  assert ces2_conditions(s)[1]["wantFaster"]
  # below the floor WITH an opening lead, blind -> allowed (v1 pull-away exception parity)
  v = 15 * MPH
  s = base(v_ego=v, v_set=90 * MPH, mdl_end_x=0.0, has_lead=True, lead_drel=20.0,
           lead_vlead=v + 2.0, lead_opening=True)
  assert ces2_conditions(s)[1]["wantFaster"]


def test_pullaway_absorbed_without_speed_band():
  # v1's below-floor pull-away exception, band-free: lead genuinely opening (evidence) at 17 mph
  # -> chill preference; the SAME situation with recent stop evidence -> Experimental holds.
  v = 17 * MPH
  s = base(v_ego=v, v_set=17 * MPH + 8.0, has_lead=True, lead_drel=15.0,
           lead_vlead=v + 2.0, lead_opening=True)
  conds, gates = ces2_conditions(s)
  assert gates["wantFaster"] and gates["leadOpen"] and not conds["lowSpeed"]
  # yellow-light trap: stop evidence still decaying (MEDIUM) -> precedence blocks the adoption
  s2 = dict(s, stop_urgency=LEVEL_MED)
  conds2, gates2 = ces2_conditions(s2)
  assert not gates2["wantFaster"] and not gates2["leadOpen"]   # both preferences yield
  assert conds2["lowSpeed"]              # the low-speed Experimental hold stays


def test_lead_pacing_still_suppresses_curve():
  # 2026-07-08 directive intact: a pacing lead suppresses the curve trip (v1 rule, kept verbatim)
  v = 60 * MPH
  s = base(v_ego=v, has_lead=True, lead_drel=50.0, lead_vlead=v,
           map_target_v=v - 5.0, map_target_dist=v * 5.0)
  conds, gates = ces2_conditions(s)
  assert gates["pacing"] and not conds["curve"]


# ---------------------------------------------------------------------------
# 3) standstill hold (CEM)
# ---------------------------------------------------------------------------
def _run(core, s, seconds, dt=0.1):
  t = 0.0
  while t < seconds:
    core.update_decision(s, dt)
    t += dt
  return core.mode()


def test_standstill_hold_keeps_experimental_at_red_light():
  core = Ces2Core()
  # roll to a stop for a light: shouldStop asserts -> fast path
  s_stop = base(v_ego=5.0, model_should_stop=True)
  core.update_decision(s_stop, 0.01)
  assert core.mode() == "experimental" and core.status() == "stopIntent"
  # now at v=0: binary shouldStop flickers OFF but the endpoint stays short (model still stopped).
  # v1 had only the 8 s dwell here; CES2 holds explicitly for as long as the model says stopped.
  s_stand = base(v_ego=0.0, standstill=True, model_should_stop=False, mdl_end_x=5.0)
  assert _run(core, s_stand, 30.0) == "experimental"
  assert core.status() == "standstill"
  # light turns green: model plans ahead again (endpoint long) -> normal exit path resumes
  s_go = base(v_ego=0.0, standstill=True, model_should_stop=False, mdl_end_x=150.0)
  assert _run(core, s_go, 15.0) == "chill"


def test_standstill_without_stop_evidence_does_not_hold():
  core = Ces2Core()
  core.update_decision(base(v_ego=5.0, model_should_stop=True), 0.01)
  assert core.mode() == "experimental"
  # stopped, but the model does NOT say stopped (e.g. parked with a clear road) -> normal decay
  s = base(v_ego=0.0, standstill=True, mdl_end_x=150.0)
  assert _run(core, s, 15.0) == "chill"


# ---------------------------------------------------------------------------
# 4) turn-signal condition (CEM F2)
# ---------------------------------------------------------------------------
def test_turn_signal_below_55_trips():
  s = base(v_ego=20 * MPH, blinker=True)
  active, reason = decide_active2(s)
  assert active and reason == "turn"


def test_turn_signal_above_55_does_not_trip():
  assert not decide_active2(base(v_ego=60 * MPH, blinker=True))[0]


def test_turn_signal_lane_change_intent_does_not_trip():
  # signaling a LANE CHANGE (model has lane-change intent) is not a turn.
  # 45 mph: above the lowSpeed threshold (40) so only the turn condition is in play.
  s = base(v_ego=45 * MPH, blinker=True, lane_change_intent=True)
  assert not decide_active2(s)[0]


def test_turn_signal_toggle_off_by_default():
  toggles = dict(ALL_ON, turns=False)   # CESTurns defaults OFF (study 5.2: dark first drives)
  s = base(v_ego=45 * MPH, blinker=True, toggles=toggles)
  assert not decide_active2(s)[0]


# ---------------------------------------------------------------------------
# 5) per-condition filters — no cross-condition masking
# ---------------------------------------------------------------------------
def test_per_condition_filters_no_masking():
  """v1's single aggregate filter let one condition's decaying charge mask another's rise. With
  per-condition filters the stop condition (tau=0.5) charges on its own schedule regardless of a
  half-decayed curve charge."""
  core = Ces2Core(exp_min_dwell=0.5, chill_min_dwell=0.0)   # short dwells: filter behavior only
  dt = 0.05
  # charge the curve condition (no stop), then let it decay half-way
  s_curve = base(v_ego=25.0, curve_lat_accel_vision=2.5, time_to_curve=2.0)
  for _ in range(60):
    core.update_decision(s_curve, dt)
  assert core.mode() == "experimental" and core.status() == "curve"
  curve_x = core._f["curve"].f.x
  s_none = base(v_ego=25.0)
  for _ in range(10):
    core.update_decision(s_none, dt)
  assert core._f["curve"].f.x < curve_x          # curve charge decaying...
  assert core._f["stop"].f.x < 0.05              # ...while the stop filter is independently empty
  # now a MEDIUM-urgency stop starts charging: only the stop filter rises
  s_stop = base(v_ego=10.0, stop_urgency=LEVEL_MED, mdl_end_x=20.0)
  x0 = core._f["stop"].f.x
  core.update_decision(s_stop, dt)
  assert core._f["stop"].f.x > x0
  assert core._f["curve"].f.x < curve_x          # untouched by the stop's rise


def test_stop_filter_charges_2x_faster():
  fast = C2.Ces2Condition(C2.STOP_FILTER_TAU)
  slow = C2.Ces2Condition(C.FILTER_TAU)
  t = 0.0
  while t < 0.6:
    fast.update(True, 0.05)
    slow.update(True, 0.05)
    t += 0.05
  assert fast.active and not slow.active


def test_medium_urgency_charges_stop_filter_before_shouldstop():
  """The graded signal's whole point: a contracting endpoint (MEDIUM) charges the stop condition
  BEFORE binary shouldStop asserts — earlier, smoother red-light entries."""
  core = Ces2Core(chill_min_dwell=0.0)
  # v=8 m/s (28.8 kph) -> expected ~84 m; endpoint 63 m -> urgency ~0.53 = genuine MEDIUM
  s = base(v_ego=8.0, mdl_end_x=63.0)  # contracting, no shouldStop
  dt = 0.05
  ticks = 0
  while core.mode() == "chill" and ticks < 100:
    core.update_decision(s, dt)
    ticks += 1
  assert core.mode() == "experimental" and core.status() == "stop"
  assert ticks * dt < 1.5                        # well under the plain tau=1.0 charge time


# ---------------------------------------------------------------------------
# 6) shadow divergence counting
# ---------------------------------------------------------------------------
def test_divergence_counter_counts_edges_not_ticks():
  d = DivergenceCounter()
  for _ in range(10):
    d.update(True, True)
  assert d.count == 0
  for _ in range(25):                            # one sustained disagreement = ONE divergence
    d.update(True, False)
  assert d.count == 1
  d.update(False, False)                          # re-agree
  for _ in range(5):                              # second disagreement = second edge
    d.update(False, True)
  assert d.count == 2


def test_divergence_counter_ignores_none():
  d = DivergenceCounter()
  d.update(True, None)
  d.update(None, False)
  assert d.count == 0
  d.update(True, False)
  assert d.count == 1


# ---------------------------------------------------------------------------
# fast path parity + retired rules truly absent
# ---------------------------------------------------------------------------
def test_fast_path_mirrors_v1_shape():
  # shouldStop + ladder wants Experimental (slowLead behind a stopping lead) -> immediate entry,
  # even though the STOP condition itself is lead-gated (v1 parity: raw_active + shouldStop).
  core = Ces2Core()
  s = base(v_ego=15.0, has_lead=True, lead_drel=20.0, lead_vlead=1.5,
           model_should_stop=True)
  core.update_decision(s, 0.01)
  assert core.mode() == "experimental" and core.status() == "stopIntent"


def test_high_urgency_takes_fast_path_without_shouldstop():
  # DEC's emergency tier: HIGH graded urgency == binary intent for the fast path
  core = Ces2Core()
  s = base(v_ego=12.0, stop_urgency=LEVEL_HIGH, mdl_end_x=8.0)   # deep contraction at ~27 mph
  core.update_decision(s, 0.01)
  assert core.mode() == "experimental" and core.status() == "stopIntent"


def test_retired_constants_not_referenced():
  """The point of CES2: the floor / band / recency-guard rules DO NOT EXIST in the core."""
  import inspect
  src = inspect.getsource(C2)
  # code references (C.<name>) — the docstrings NAME the retired rules on purpose (the record of
  # what absorbed them), so check attribute usage, not prose.
  for retired in ("C.ACCEL_ZONE_MIN_V", "C.PULLAWAY_MIN_V", "C.PULLAWAY_DREL_LO",
                  "C.PULLAWAY_DREL_HI", "C.PULLAWAY_STOP_CLEAR_S",
                  "_lead_pull_away(", "_accelerate_zone("):
    assert retired not in src, f"retired rule {retired} leaked into the CES2 core"


def test_urgency_levels_consistent():
  ev = StopEvidence()
  ev.urgency = URGENCY_MED
  assert ev.level == LEVEL_MED
  ev.urgency = URGENCY_HIGH
  assert ev.level == LEVEL_HIGH
  ev.urgency = math.nextafter(URGENCY_MED, 0.0)
  assert ev.level == LEVEL_LOW
