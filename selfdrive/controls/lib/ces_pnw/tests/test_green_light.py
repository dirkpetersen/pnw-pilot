"""
greenlight2pnw / greenlead2pnw — unit tests for the pure GreenLightDetector (no cereal, no car).

Contract under test (the driver-facing behavior, field split request 2026-07-13):
  - standstill, NO lead -> model releases the path => "green" fires exactly ONCE;
  - a close, STOPPED lead departing from OUR standstill => "lead" fires (NOT "green") — even when
    the radar loses the departing car (a dRel/status dropout can never fake a no-lead green);
  - a lead departing while we already roll => NO alert (telemetry-only "leadMoving");
  - "green" needs the radar lead ABSENT for a sustained >= 1 s (ghost/flicker guard);
  - "lead" dings are rate-limited across stop-and-go cycles (20 s cooldown);
  - a flickering model (short release pulses) can never ding, and never twice per stop;
  - never fires while moving, on gas, or on a short (< 2 s) rolling stop;
  - a fresh stop after driving off re-arms the full cycle.

Run:  pytest selfdrive/controls/lib/ces_pnw/tests/test_green_light.py
"""
from openpilot.selfdrive.controls.lib.ces_pnw.green_light import (
  GreenLightDetector, attentive_now, GL_IDLE, GL_ARMED, GL_FIRED,
  GL_EV_GREEN, GL_EV_LEAD, GL_EV_LEAD_MOVING,
  GL_MIN_STOP_S, GL_RELEASE_S, GL_X_GO_M, GL_NO_LEAD_S, GL_LEAD_FORGET_S, GL_LEAD_COOLDOWN_S,
)

DT = 0.01  # selfdrived cadence (100 Hz)


def step(gl, seconds, v_ego=0.0, gas=False, should_stop=False, end_x=5.0,
         lead=False, drel=0.0, vlead=0.0):
  """Run the detector for `seconds` with constant inputs; return the list of classification
  tokens it emitted (empty list = nothing happened)."""
  evs = []
  for _ in range(int(round(seconds / DT))):
    ev = gl.update(DT, v_ego, gas, should_stop, end_x, lead, drel, vlead)
    if ev is not None:
      evs.append(ev)
  return evs


def settle_at_red(gl, lead=False, drel=0.0, vlead=0.0):
  """Drive up, stop, and hold at a model-held stop long enough to arm."""
  step(gl, 1.0, v_ego=15.0, end_x=100.0)                       # driving
  step(gl, GL_MIN_STOP_S + 0.5, should_stop=True, end_x=4.0,   # held at the light
       lead=lead, drel=drel, vlead=vlead)


# --- the no-lead GREEN ding ---------------------------------------------------------------------

def test_no_lead_green_fires_once():
  gl = GreenLightDetector()
  settle_at_red(gl)
  assert gl.state == GL_ARMED
  # light turns green: model releases (shouldStop drops, endpoint runs long) -> one green ding
  assert step(gl, GL_RELEASE_S + 0.1, should_stop=False, end_x=60.0) == [GL_EV_GREEN]
  assert gl.state == GL_FIRED
  # continued release, still stopped: consumed — never a second ding this stop
  assert step(gl, 5.0, should_stop=False, end_x=60.0) == []


def test_no_stop_context_never_arms():
  # stopped with a clear road the whole time (model never held a stop): no arming, no ding
  gl = GreenLightDetector()
  step(gl, 1.0, v_ego=15.0, end_x=100.0)
  assert step(gl, 10.0, should_stop=False, end_x=60.0) == []
  assert gl.state == GL_IDLE


def test_missing_endpoint_does_not_arm():
  # mdl_end_x == 0.0 is missing data, not a held stop
  gl = GreenLightDetector()
  step(gl, 1.0, v_ego=15.0, end_x=100.0)
  step(gl, GL_MIN_STOP_S + 1.0, should_stop=False, end_x=0.0)
  assert gl.state == GL_IDLE


# --- the LEAD-departure ding (driver rule #2) ---------------------------------------------------

def test_lead_still_stopped_suppresses():
  gl = GreenLightDetector()
  settle_at_red(gl, lead=True, drel=8.0, vlead=0.0)
  # light green (model releases) but the lead is still sitting there: nothing
  assert step(gl, 3.0, should_stop=False, end_x=60.0, lead=True, drel=8.0, vlead=0.0) == []
  assert gl.state == GL_ARMED


def test_lead_departs_at_standstill_fires_lead_not_green():
  gl = GreenLightDetector()
  settle_at_red(gl, lead=True, drel=8.0, vlead=0.0)
  step(gl, 2.0, should_stop=False, end_x=60.0, lead=True, drel=8.0, vlead=0.0)   # still blocked
  # lead pulls away (moving, opening) while WE stay at a standstill: the LEAD ding, never green
  evs = step(gl, GL_RELEASE_S + 0.1, should_stop=False, end_x=60.0,
             lead=True, drel=12.0, vlead=3.0)
  assert evs == [GL_EV_LEAD]
  assert gl.state == GL_FIRED
  # consumed: the departing lead keeps opening, no second ding
  assert step(gl, 5.0, should_stop=False, end_x=60.0, lead=True, drel=30.0, vlead=8.0) == []


def test_lead_lost_by_radar_is_lead_not_green():
  # the departing lead drops off the radar (status flicker / out of range): the sticky
  # had-lead memory classifies it as a departure — a dRel dropout can NEVER fake a no-lead green
  gl = GreenLightDetector()
  settle_at_red(gl, lead=True, drel=8.0, vlead=0.0)
  assert step(gl, GL_RELEASE_S + 0.1, should_stop=False, end_x=60.0, lead=False) == [GL_EV_LEAD]


def test_green_requires_sustained_lead_absence():
  # a (far, never-blocking) lead is on the radar through the arming stop and the release: the
  # green must wait for a full GL_NO_LEAD_S of continuous absence — then still fire (not be lost)
  gl = GreenLightDetector()
  step(gl, 1.0, v_ego=15.0, end_x=100.0)
  step(gl, GL_MIN_STOP_S + 0.5, should_stop=True, end_x=4.0, lead=True, drel=25.0, vlead=0.0)
  assert gl.state == GL_ARMED
  # model releases while the lead is still reported: NO green (driver rule #1: green = no lead)
  assert step(gl, 1.0, should_stop=False, end_x=60.0, lead=True, drel=25.0, vlead=0.0) == []
  # radar report ends: green fires only after the sustained absence window
  evs = step(gl, GL_NO_LEAD_S + 0.1, should_stop=False, end_x=60.0, lead=False)
  assert evs == [GL_EV_GREEN]


def test_brief_blocked_flicker_does_not_become_lead():
  # a single 0.2 s blocked-lead blip (< GL_LEAD_BLOCK_MIN_S) during an otherwise empty stop must
  # not convert the eventual release into a lead-departure ding
  gl = GreenLightDetector()
  step(gl, 1.0, v_ego=15.0, end_x=100.0)
  step(gl, 1.0, should_stop=True, end_x=4.0)
  step(gl, 0.2, should_stop=True, end_x=4.0, lead=True, drel=6.0, vlead=0.0)   # radar blip
  step(gl, GL_MIN_STOP_S, should_stop=True, end_x=4.0)
  assert gl.state == GL_ARMED
  assert step(gl, GL_NO_LEAD_S + GL_RELEASE_S, should_stop=False, end_x=60.0) == [GL_EV_GREEN]


def test_flickering_blocked_lead_departure_is_lead_not_green():
  # Gemini review catch: a stopped lead with FLICKERING radar returns (never 0.5 s continuous,
  # but plenty cumulative) must still be remembered — its departure is the LEAD ding, never a
  # no-lead green, even after the 1 s absence window elapses.
  gl = GreenLightDetector()
  step(gl, 1.0, v_ego=15.0, end_x=100.0)
  for _ in range(6):   # settle behind it: seen 0.3 s / dropped 0.2 s, repeatedly
    step(gl, 0.3, should_stop=True, end_x=4.0, lead=True, drel=8.0, vlead=0.0)
    step(gl, 0.2, should_stop=True, end_x=4.0)
  assert gl.state == GL_ARMED
  # the lead departs for real (radar loses it) and the model releases
  assert step(gl, GL_NO_LEAD_S + 0.5, should_stop=False, end_x=60.0, lead=False) == [GL_EV_LEAD]


def test_far_stopped_lead_blocks_green_until_gone():
  # a stopped car 40 m ahead (next intersection) never blocked us, but it IS a lead — driver
  # rule #1 says green fires only with NO lead. It fires once the radar no longer reports one.
  gl = GreenLightDetector()
  settle_at_red(gl)
  assert step(gl, 2.0, should_stop=False, end_x=60.0, lead=True, drel=40.0, vlead=0.0) == []
  assert gl.state == GL_ARMED                                   # primed, not consumed
  assert step(gl, GL_NO_LEAD_S + 0.1, should_stop=False, end_x=60.0, lead=False) == [GL_EV_GREEN]


def test_stale_departure_becomes_green():
  # car ahead turns right on red and leaves LONG before the light changes: the departure context
  # expires (GL_LEAD_FORGET_S) and the eventual release is a plain green
  gl = GreenLightDetector()
  settle_at_red(gl, lead=True, drel=8.0, vlead=0.0)
  step(gl, 1.0, should_stop=True, end_x=4.0, lead=True, drel=8.0, vlead=0.0)   # settled behind it
  step(gl, GL_LEAD_FORGET_S + 1.0, should_stop=True, end_x=4.0, lead=False)    # it left; still red
  assert step(gl, GL_RELEASE_S + 0.1, should_stop=False, end_x=60.0) == [GL_EV_GREEN]


# --- lead departs while WE are moving: no alert, telemetry only (driver rule #3) ----------------

def test_lead_departs_while_moving_no_alert():
  gl = GreenLightDetector()
  settle_at_red(gl, lead=True, drel=8.0, vlead=0.0)
  # lead starts pulling away, and we creep after it before any release debounce completes
  step(gl, 0.1, should_stop=False, end_x=60.0, lead=True, drel=10.0, vlead=2.0)
  evs = step(gl, 2.0, v_ego=1.5, should_stop=False, end_x=60.0, lead=True, drel=14.0, vlead=3.0)
  # exactly one telemetry classification, never an alert token
  assert evs == [GL_EV_LEAD_MOVING]
  assert gl.state == GL_IDLE
  # and nothing more while we keep rolling
  assert step(gl, 3.0, v_ego=5.0, end_x=100.0, lead=True, drel=20.0, vlead=6.0) == []


# --- stop-and-go anti-spam ----------------------------------------------------------------------

def test_lead_ding_cooldown_in_stop_and_go():
  gl = GreenLightDetector()
  # cycle 1: settle behind a stopped lead, it departs -> one LEAD ding
  settle_at_red(gl, lead=True, drel=8.0, vlead=0.0)
  assert step(gl, GL_RELEASE_S + 0.1, should_stop=False, end_x=60.0,
              lead=True, drel=12.0, vlead=3.0) == [GL_EV_LEAD]
  # cycle 2, seconds later: creep forward, stop behind it again, it departs again -> SUPPRESSED
  step(gl, 3.0, v_ego=2.0, end_x=60.0, lead=True, drel=12.0, vlead=2.0)
  step(gl, GL_MIN_STOP_S + 0.5, should_stop=True, end_x=4.0, lead=True, drel=8.0, vlead=0.0)
  assert step(gl, GL_RELEASE_S + 0.1, should_stop=False, end_x=60.0,
              lead=True, drel=12.0, vlead=3.0) == []
  assert gl.state == GL_FIRED                                   # consumed silently
  # cycle 3, after the cooldown has fully decayed: the ding is back
  step(gl, 3.0, v_ego=2.0, end_x=60.0, lead=True, drel=12.0, vlead=2.0)
  step(gl, GL_LEAD_COOLDOWN_S + 1.0, should_stop=True, end_x=4.0, lead=True, drel=8.0, vlead=0.0)
  assert step(gl, GL_RELEASE_S + 0.1, should_stop=False, end_x=60.0,
              lead=True, drel=12.0, vlead=3.0) == [GL_EV_LEAD]


# --- debounce / flicker -------------------------------------------------------------------------

def test_flicker_never_dings():
  gl = GreenLightDetector()
  settle_at_red(gl)
  # model flickers: release pulses shorter than the debounce, separated by re-stops
  for _ in range(10):
    assert step(gl, GL_RELEASE_S / 2, should_stop=False, end_x=GL_X_GO_M + 20.0) == []
    assert step(gl, 0.1, should_stop=True, end_x=4.0) == []
  assert gl.state == GL_ARMED
  # then a real, sustained release: exactly one green ding
  assert step(gl, GL_RELEASE_S + 0.1, should_stop=False, end_x=60.0) == [GL_EV_GREEN]
  # post-fire flicker: still consumed, no second ding
  assert step(gl, 0.2, should_stop=True, end_x=4.0) == []
  assert step(gl, 2.0, should_stop=False, end_x=60.0) == []


# --- never while already going ------------------------------------------------------------------

def test_moving_never_fires():
  gl = GreenLightDetector()
  # cruising with a wide-open path: never a ding
  assert step(gl, 10.0, v_ego=25.0, should_stop=False, end_x=150.0) == []
  assert gl.state == GL_IDLE


def test_motion_during_release_cancels():
  gl = GreenLightDetector()
  settle_at_red(gl)
  step(gl, GL_RELEASE_S / 2, should_stop=False, end_x=60.0)     # release under way...
  assert step(gl, 1.0, v_ego=2.0, end_x=60.0) == []             # ...but the car is already rolling
  assert gl.state == GL_IDLE


def test_gas_pressed_blocks_and_disarms():
  gl = GreenLightDetector()
  settle_at_red(gl)
  # driver creeps on gas exactly as the light turns green: no ding on top of them
  assert step(gl, 2.0, gas=True, should_stop=False, end_x=60.0) == []
  assert gl.state == GL_IDLE


def test_short_stop_never_fires():
  # a < 2 s rolling stop that releases immediately (stop sign roll-through): no ding
  gl = GreenLightDetector()
  step(gl, 1.0, v_ego=15.0, end_x=100.0)
  assert step(gl, 1.0, should_stop=True, end_x=4.0) == []
  assert step(gl, 0.9, should_stop=False, end_x=60.0) == []
  assert gl.state == GL_IDLE


# --- re-arm cycle -------------------------------------------------------------------------------

def test_rearms_at_next_stop():
  gl = GreenLightDetector()
  settle_at_red(gl)
  assert step(gl, GL_RELEASE_S + 0.1, should_stop=False, end_x=60.0) == [GL_EV_GREEN]
  # drive off, then a fresh red light: the full cycle fires again (greens have no cooldown)
  step(gl, 5.0, v_ego=15.0, end_x=100.0)
  settle_at_red(gl)
  assert step(gl, GL_RELEASE_S + 0.1, should_stop=False, end_x=60.0) == [GL_EV_GREEN]


# --- dmgate2pnw: attention gate for the LEAD-departure ding -------------------------------------
# selfdrived suppresses the leadDeparting alert when attentive_now() is True (an attentive driver
# sees the car ahead leave and goes on their own); the greenLight alert is never gated on this.

def test_attentive_only_when_active_face_and_not_distracted():
  # the one True case: DM active mode + a face this frame + not distracted -> suppress the lead ding
  assert attentive_now(True, True, False) is True


def test_not_attentive_when_distracted():
  # watching away / phone / eyes closed -> DM says distracted -> NOT attentive -> the ding fires
  assert attentive_now(True, True, True) is False


def test_not_attentive_when_no_face():
  # no face this frame -> not confident -> ding fires (safe default)
  assert attentive_now(True, False, False) is False


def test_not_attentive_when_dm_passive():
  # DM not in active mode (face lost / model uncertain) -> not confident -> ding fires
  assert attentive_now(False, True, False) is False


def test_safe_default_on_all_false():
  # a missing/blank driverMonitoringState message reads all-False -> never suppresses (ding fires)
  assert attentive_now(False, False, False) is False
  # even a "not distracted" blank must not read as attentive (no active mode, no face)
  assert attentive_now(False, False, True) is False
