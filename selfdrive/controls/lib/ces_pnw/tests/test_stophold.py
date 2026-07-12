"""stophold2pnw — A1 (lead-present stop reason at low speed) + A2 (standstill-departure hold).

Field basis: drives/2026-07-12/tesla-redlight/CES_SILENCE_REPORT.md — 21:47:08Z, Tesla, engaged,
stopped behind a stopped lead at a light. The lead crept above STOPPED_LEAD_V, slowLead cleared,
lowSpeed could not hold (its own 1.0 m/s floor), the stop reason was masked by `not has_lead`,
and CES adopted Chill at 0.4 m/s — the MPC launched at up to 1.6 m/s^2 toward vSet 13.4 with
gas=False/strPrs=False (pure machine lurch), re-entering Experimental only 8 s later.

A1: below STOP_HOLD_MAX_V the model's stop intent counts even with a lead present (and thereby
re-arms the stopIntent fast path in this geometry). A2: below STANDSTILL_HOLD_V, Experimental may
not exit to Chill until model_should_stop has been continuously clear for STOP_CLEAR_HOLD_S.
Both are exit/entry guards toward Experimental only — no new acceleration path exists.
"""
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw_constants as C
from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import (ConditionalExperimentalSwitching, CESStub,
                                                              decide_active, decision_telemetry,
                                                              clock_bad, _lead_pull_away, _accelerate_zone)

ALL_ON = {"curves": True, "stops": True, "low_speed": True, "lead": True}
DT = 0.1


def sig(**kw):
  s = {
    "v_ego": 30.0, "has_lead": False, "lead_vlead": 0.0, "lead_drel": 0.0, "blinker": False,
    "map_target_v": 0.0, "map_target_dist": float('inf'),
    "curve_lat_accel_vision": 0.0, "time_to_curve": 10.0,
    "model_should_stop": False, "v_set": 0.0, "spd_lim": 0.0, "toggles": ALL_ON,
  }
  s.update(kw)
  return s


# the 21:47 geometry, parameterized ------------------------------------------------------------
def stopped_at_light(**kw):
  """Stopped behind a STOPPED lead (dRel 5 m) — the pre-lurch hold (slowLead active)."""
  return sig(v_ego=0.0, has_lead=True, lead_vlead=0.0, lead_drel=5.0, v_set=13.4, **kw)


def lead_creeps(**kw):
  """The lead crept to 2.0 m/s (> STOPPED_LEAD_V), ego still at 0.4 m/s — 21:47:08Z exactly."""
  base = dict(v_ego=0.4, has_lead=True, lead_vlead=2.0, lead_drel=7.0, v_set=13.4)
  base.update(kw)
  return sig(**base)


def run(sm, s, seconds):
  for _ in range(int(round(seconds / DT))):
    sm.update_decision(s, DT)
  return sm.mode()


# --- A1: decide_active ------------------------------------------------------------------------

def test_a1_stop_fires_with_lead_below_floor():
  """The exact lurch tick: creeping lead, red still governing -> the stop reason now fires."""
  assert decide_active(lead_creeps(model_should_stop=True)) == (True, "stop")


def test_a1_boundary_at_stop_hold_max_v():
  """Strictly below STOP_HOLD_MAX_V: stop. At/above: the lead mask stands (no stop reason)."""
  below = lead_creeps(model_should_stop=True, v_ego=C.STOP_HOLD_MAX_V - 0.01)
  at = lead_creeps(model_should_stop=True, v_ego=C.STOP_HOLD_MAX_V)
  assert decide_active(below) == (True, "stop")
  active, reason = decide_active(at)
  assert reason != "stop"          # masked again at the boundary...
  assert (active, reason) == (True, "lowSpeed")   # ...but lowSpeed (above ITS 1.0 floor) covers it here


def test_a1_no_lead_path_unchanged():
  """The original no-lead stop reason is untouched (any speed)."""
  assert decide_active(sig(v_ego=10.0, model_should_stop=True)) == (True, "stop")


def test_a1_lead_mask_stands_at_speed():
  """Lead-following WITH stop intent at speed must NOT trip the stop reason (the mask's whole
  point: normal following decel is Chill's job). Highway spd_lim isolates from lowSpeed."""
  s = sig(v_ego=10.0, has_lead=True, lead_vlead=9.0, lead_drel=30.0,
          model_should_stop=True, v_set=13.4, spd_lim=25.0)
  assert decide_active(s) == (False, "chill")


# --- A1 x fast path: the stopIntent re-arm in the lurch geometry -------------------------------

def test_a1_rearms_stopintent_fast_path_from_chill():
  """A CHILL machine (dwell 0 — every normal entry blocked by the cooldown) + creeping lead +
  shouldStop: ONE cycle to Experimental via the fast path. Pre-A1 this geometry had
  raw_active=False (stop masked by the lead), so the fast path was unreachable and Chill launched."""
  sm = ConditionalExperimentalSwitching()
  assert sm.update_decision(lead_creeps(model_should_stop=True), DT) == "experimental"
  assert sm.status() == "stopIntent"


# --- THE 21:47 LURCH REPLAY --------------------------------------------------------------------

def test_lurch_replay_red_light_holds_then_green_releases():
  """Full field sequence: (1) long stop behind a stopped lead (Experimental/slowLead hold);
  (2) lead creeps while shouldStop STILL TRUE (red) -> stays Experimental (A1: reason=stop) —
  pre-fix this is the exact tick CES adopted Chill and lurched; (3) shouldStop clears (green) ->
  A2 holds through the flicker window, then the normal exit path releases to Chill."""
  sm = ConditionalExperimentalSwitching()
  # (1) charge + cooldown + dwell: the pre-lurch hold (no stop intent yet — plain slowLead entry)
  assert run(sm, stopped_at_light(), 15.0) == "experimental"
  # (2) the lurch tick, red still governing: must STAY Experimental for as long as the red lasts
  assert run(sm, lead_creeps(model_should_stop=True), 5.0) == "experimental"
  assert sm.status() == "stop"                      # held by the A1 stop reason, visibly
  # (3) green: A2 requires 2 s of continuous shouldStop-clear before the exit may happen
  assert run(sm, lead_creeps(model_should_stop=False), 1.5) == "experimental"
  assert sm.status() == "stopHold"                  # the hold is the only thing keeping it — tagged
  assert run(sm, lead_creeps(model_should_stop=False), 1.0) == "chill"   # 2.5 s total: released


def test_a2_timer_resets_on_shouldstop_flicker():
  """A 1-cycle shouldStop re-assert during the clear window restarts the 2 s requirement."""
  sm = ConditionalExperimentalSwitching()
  run(sm, stopped_at_light(model_should_stop=True), 10.0)   # enters via fast path, dwell builds
  assert sm.mode() == "experimental"
  run(sm, lead_creeps(model_should_stop=False), 1.5)        # 1.5 s clear (still held)
  sm.update_decision(lead_creeps(model_should_stop=True), DT)   # red flicker: timer resets
  assert run(sm, lead_creeps(model_should_stop=False), 1.5) == "experimental"  # 1.5 s again: held
  assert run(sm, lead_creeps(model_should_stop=False), 1.0) == "chill"         # full 2 s clear: out


def test_a2_no_hold_above_standstill():
  """Same recent-stop history, but ego above STANDSTILL_HOLD_V: A2 does not apply — the exit
  happens on the normal filter decay (well under 2 s). Highway spd_lim isolates from lowSpeed."""
  def green(v):
    return sig(v_ego=v, has_lead=True, lead_vlead=v + 0.5, lead_drel=7.0, v_set=13.4, spd_lim=25.0)
  # below the floor: held at 1.5 s
  sm = ConditionalExperimentalSwitching()
  run(sm, stopped_at_light(model_should_stop=True), 10.0)
  assert run(sm, green(0.4), 1.5) == "experimental"
  # above the floor: released by 1.5 s (only the ~0.5 s filter decay stands between)
  sm2 = ConditionalExperimentalSwitching()
  run(sm2, stopped_at_light(model_should_stop=True), 10.0)
  assert run(sm2, green(2.0), 1.5) == "chill"


def test_a2_respects_stops_toggle():
  """Driver disabled the stops condition: A2 (like the fast path) must not hold."""
  toggles = {**ALL_ON, "stops": False}
  sm = ConditionalExperimentalSwitching()
  run(sm, stopped_at_light(), 15.0)                          # slowLead entry (no stop machinery)
  assert sm.mode() == "experimental"
  sm.update_decision(stopped_at_light(model_should_stop=True, toggles=toggles), DT)  # arm the timer
  assert run(sm, lead_creeps(model_should_stop=False, toggles=toggles), 1.5) == "chill"


# --- precedence: A1/A2 vs the pullaway exception -----------------------------------------------

def test_pullaway_never_fires_while_shouldstop():
  """The lurch geometry inside the pullaway band WITH stop intent: the pull-away exception is
  dead (its own `not model_should_stop` gate), the accel-zone stays closed, and the A1 stop
  reason owns the decision. Explicit precedence check — A1/A2 hold, pullaway waits."""
  s = sig(v_ego=2.5, has_lead=True, lead_vlead=6.0, lead_drel=15.0, v_set=13.4,
          model_should_stop=True, lead_opening=True)
  assert _lead_pull_away(s) is False
  assert _accelerate_zone(s) is False
  assert decide_active(s) == (True, "stop")


def test_pullaway_intact_once_stop_clears():
  """Same geometry with shouldStop clear: the evidence-gated pull-away exception works exactly
  as shipped (Chill adoption tagged pullAway) — A1 changed nothing above its own gate."""
  s = sig(v_ego=2.5, has_lead=True, lead_vlead=6.0, lead_drel=15.0, v_set=13.4,
          model_should_stop=False, lead_opening=True)
  assert _lead_pull_away(s) is True
  assert decide_active(s) == (False, "pullAway")


# --- B: telemetry carries the raw stop intent ---------------------------------------------------

def test_telemetry_stp_mirrors_model_should_stop():
  assert decision_telemetry(lead_creeps(model_should_stop=True))["stp"] is True
  assert decision_telemetry(lead_creeps(model_should_stop=False))["stp"] is False


# --- C: the inert stub -------------------------------------------------------------------------

def test_ces_stub_is_inert():
  stub = CESStub()
  assert stub.experimental_request(None, None) is False
  assert stub.enabled() is False
  assert stub.status() == "chill"


# --- D: clock validity marker ------------------------------------------------------------------

def test_clock_bad():
  assert clock_bad(0.0) is True                    # 1970 (dead RTC, pre-sync)
  assert clock_bad(1764094635.0) is False          # 2025-11-25 — plausible-but-wrong stamps pass;
                                                   #   only OBVIOUS pre-2020 garbage is markable
  assert clock_bad(1577836799.0) is True           # 1 s before the 2020 epoch gate
  assert clock_bad(1783894265.0) is False          # 2026-07-12 (real)
  assert clock_bad(None) is True                   # garbage input counts as bad, never raises
