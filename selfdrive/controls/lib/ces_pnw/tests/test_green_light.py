"""
greenlight2pnw — unit tests for the pure GreenLightDetector (no cereal, no car).

Contract under test (the driver-facing behavior):
  - standstill -> model releases the path => fires exactly ONCE;
  - a close, STOPPED lead suppresses the ding; it fires after the lead departs;
  - a flickering model (short release pulses) can never ding, and never twice per stop;
  - never fires while moving, on gas, or on a short (< 2 s) rolling stop;
  - a fresh stop after driving off re-arms the full cycle.

Run:  pytest selfdrive/controls/lib/ces_pnw/tests/test_green_light.py
"""
from openpilot.selfdrive.controls.lib.ces_pnw.green_light import (
  GreenLightDetector, GL_IDLE, GL_ARMED, GL_FIRED,
  GL_MIN_STOP_S, GL_RELEASE_S, GL_X_GO_M,
)

DT = 0.01  # selfdrived cadence (100 Hz)


def step(gl, seconds, v_ego=0.0, gas=False, should_stop=False, end_x=5.0,
         lead=False, drel=0.0, vlead=0.0):
  """Run the detector for `seconds` with constant inputs; return True if it fired at any tick."""
  fired = False
  for _ in range(int(round(seconds / DT))):
    fired |= gl.update(DT, v_ego, gas, should_stop, end_x, lead, drel, vlead)
  return fired


def settle_at_red(gl, lead=False, drel=0.0, vlead=0.0):
  """Drive up, stop, and hold at a model-held stop long enough to arm."""
  step(gl, 1.0, v_ego=15.0, end_x=100.0)                       # driving
  step(gl, GL_MIN_STOP_S + 0.5, should_stop=True, end_x=4.0,   # held at the light
       lead=lead, drel=drel, vlead=vlead)


# --- the basic ding ---------------------------------------------------------------------------

def test_standstill_release_fires_once():
  gl = GreenLightDetector()
  settle_at_red(gl)
  assert gl.state == GL_ARMED
  # light turns green: model releases (shouldStop drops, endpoint runs long) -> one ding
  assert step(gl, GL_RELEASE_S + 0.1, should_stop=False, end_x=60.0) is True
  assert gl.state == GL_FIRED
  # continued release, still stopped: consumed — never a second ding this stop
  assert step(gl, 5.0, should_stop=False, end_x=60.0) is False


def test_no_stop_context_never_arms():
  # stopped with a clear road the whole time (model never held a stop): no arming, no ding
  gl = GreenLightDetector()
  step(gl, 1.0, v_ego=15.0, end_x=100.0)
  assert step(gl, 10.0, should_stop=False, end_x=60.0) is False
  assert gl.state == GL_IDLE


def test_missing_endpoint_does_not_arm():
  # mdl_end_x == 0.0 is missing data, not a held stop
  gl = GreenLightDetector()
  step(gl, 1.0, v_ego=15.0, end_x=100.0)
  step(gl, GL_MIN_STOP_S + 1.0, should_stop=False, end_x=0.0)
  assert gl.state == GL_IDLE


# --- lead suppression -------------------------------------------------------------------------

def test_lead_still_stopped_suppresses():
  gl = GreenLightDetector()
  settle_at_red(gl, lead=True, drel=8.0, vlead=0.0)
  # light green (model releases) but the lead is still sitting there: no ding
  assert step(gl, 3.0, should_stop=False, end_x=60.0, lead=True, drel=8.0, vlead=0.0) is False
  assert gl.state == GL_ARMED


def test_fires_after_lead_departs():
  gl = GreenLightDetector()
  settle_at_red(gl, lead=True, drel=8.0, vlead=0.0)
  step(gl, 2.0, should_stop=False, end_x=60.0, lead=True, drel=8.0, vlead=0.0)   # suppressed
  # lead pulls away (moving, opening): path is free -> ding
  assert step(gl, GL_RELEASE_S + 0.1, should_stop=False, end_x=60.0,
              lead=True, drel=12.0, vlead=3.0) is True


def test_fires_when_lead_lost_by_radar():
  gl = GreenLightDetector()
  settle_at_red(gl, lead=True, drel=8.0, vlead=0.0)
  assert step(gl, GL_RELEASE_S + 0.1, should_stop=False, end_x=60.0, lead=False) is True


def test_far_stopped_lead_does_not_suppress():
  # a stopped car 40 m ahead (next intersection) is not blocking this launch
  gl = GreenLightDetector()
  settle_at_red(gl)
  assert step(gl, GL_RELEASE_S + 0.1, should_stop=False, end_x=60.0,
              lead=True, drel=40.0, vlead=0.0) is True


# --- debounce / flicker -----------------------------------------------------------------------

def test_flicker_never_dings():
  gl = GreenLightDetector()
  settle_at_red(gl)
  # model flickers: release pulses shorter than the debounce, separated by re-stops
  for _ in range(10):
    assert step(gl, GL_RELEASE_S / 2, should_stop=False, end_x=GL_X_GO_M + 20.0) is False
    assert step(gl, 0.1, should_stop=True, end_x=4.0) is False
  assert gl.state == GL_ARMED
  # then a real, sustained release: exactly one ding
  assert step(gl, GL_RELEASE_S + 0.1, should_stop=False, end_x=60.0) is True
  # post-fire flicker: still consumed, no second ding
  assert step(gl, 0.2, should_stop=True, end_x=4.0) is False
  assert step(gl, 2.0, should_stop=False, end_x=60.0) is False


# --- never while already going ----------------------------------------------------------------

def test_moving_never_fires():
  gl = GreenLightDetector()
  # cruising with a wide-open path: never a ding
  assert step(gl, 10.0, v_ego=25.0, should_stop=False, end_x=150.0) is False
  assert gl.state == GL_IDLE


def test_motion_during_release_cancels():
  gl = GreenLightDetector()
  settle_at_red(gl)
  step(gl, GL_RELEASE_S / 2, should_stop=False, end_x=60.0)   # release under way...
  assert step(gl, 1.0, v_ego=2.0, end_x=60.0) is False        # ...but the car is already rolling
  assert gl.state == GL_IDLE


def test_gas_pressed_blocks_and_disarms():
  gl = GreenLightDetector()
  settle_at_red(gl)
  # driver creeps on gas exactly as the light turns green: no ding on top of them
  assert step(gl, 2.0, gas=True, should_stop=False, end_x=60.0) is False
  assert gl.state == GL_IDLE


def test_short_stop_never_fires():
  # a < 2 s rolling stop that releases immediately (stop sign roll-through): no ding
  gl = GreenLightDetector()
  step(gl, 1.0, v_ego=15.0, end_x=100.0)
  assert step(gl, 1.0, should_stop=True, end_x=4.0) is False
  assert step(gl, 0.9, should_stop=False, end_x=60.0) is False
  assert gl.state == GL_IDLE


# --- re-arm cycle ------------------------------------------------------------------------------

def test_rearms_at_next_stop():
  gl = GreenLightDetector()
  settle_at_red(gl)
  assert step(gl, GL_RELEASE_S + 0.1, should_stop=False, end_x=60.0) is True
  # drive off, then a fresh red light: the full cycle fires again
  step(gl, 5.0, v_ego=15.0, end_x=100.0)
  settle_at_red(gl)
  assert step(gl, GL_RELEASE_S + 0.1, should_stop=False, end_x=60.0) is True
