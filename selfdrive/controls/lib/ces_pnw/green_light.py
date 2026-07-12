"""
greenlight2pnw — Green Light Alert detection (PURE, unit-tested; owned by CESController).

Chimes + shows a small green "Light is green" alert when the car is held at a standstill (red
light / stop) and the driving model releases the path ahead — the "driver is goofing around and
missed the green" ding. DISPLAY/SOUND ONLY: this module never touches longitudinal or lateral
control; its single output is a one-cycle "fired" bool that selfdrived turns into a LOW-priority
alert event. Model-based, car-agnostic (no fingerprints, no CAN) — works identically on the Tesla
and the Lightning.

ATTRIBUTION (best-of-both adjudication, studied 2026-07-12 — see CREDITS.md):
  - sunnypilot (Haibin Wen et al., `e2e_alerts_helper.py`, MIT): the core mechanics won.
    We adopt their INACTIVE -> ARMED -> CONSUMED state machine (fires exactly ONCE per stop, only
    a real drive-off re-arms), their trigger signal (model trajectory ENDPOINT beyond ~30 m =
    the e2e model releasing the hold — we already log it as mdl_end_x), their 0.3 s sustained-
    trigger debounce (a flickering model can't ding), and their 2 s "not recently moving" arming
    guard (never fires on a rolling stop). Chosen because it is an explicit, testable state
    machine keyed on exactly the signal our CES telemetry already carries.
  - FrogPilot (FrogAi, `frogpilot_events.py`, MIT): two refinements adopted on top.
    (1) STOP-CONTEXT ARMING — FrogPilot only arms after the model actually detected a stop
    (their `stopped_for_light`); we mirror that by arming only while the model is holding a stop
    (action.shouldStop, or a short trajectory endpoint). sunnypilot arms on ANY eligible
    standstill, which would ding at a clear stop sign the moment 2 s elapse. (2) The LEAD rule —
    both forks SUPPRESS the green-light ding while a lead sits ahead (FrogPilot: tracking_lead;
    sunnypilot: any leadOne). We suppress only while the lead is CLOSE and STOPPED, so when the
    lead departs and the model releases, the ding still comes — the useful "traffic is moving"
    cue both forks provide via a separate lead-departure alert, without adding a second alert.
  - NOT adopted: sunnypilot gates on "openpilot not engaged" (CC.enabled). We follow FrogPilot
    and alert regardless of engagement — with op-long stopped long enough the car waits for a
    resume press, and that is exactly when the driver needs the ding.

Detection contract (all enforced by tests/test_green_light.py):
  - fires only from a true standstill (v_ego <= GL_MOVING_V, stopped >= GL_MIN_STOP_S);
  - arms only with stop context (model holding a stop while stationary);
  - suppressed while a close, stopped lead blocks the path (fires after it departs);
  - release must be sustained GL_RELEASE_S (model flicker can't ding, and never twice per stop);
  - never fires while the driver is already going (gas pressed, or any vehicle motion).
"""

# thresholds (SI). GL_X_GO_M / GL_RELEASE_S / GL_MIN_STOP_S are sunnypilot's field-proven values
# (GREEN_LIGHT_X_THRESHOLD / TRIGGER_TIMER_THRESHOLD / the 2 s recent-moving window).
GL_MOVING_V = 0.1        # m/s; above this the vehicle is moving -> full reset (sunnypilot)
GL_MIN_STOP_S = 2.0      # s at standstill before arming is even considered (sunnypilot)
GL_X_ARM_M = 15.0        # m; trajectory endpoint under this = model holding a stop (arm context)
GL_X_GO_M = 30.0         # m; endpoint beyond this = model released the path (sunnypilot: 30)
GL_RELEASE_S = 0.3       # s the release must be sustained before firing (sunnypilot: 0.3)
GL_LEAD_NEAR_M = 15.0    # m; a lead closer than this ...
GL_LEAD_MOVING_V = 0.5   # m/s; ... and slower than this is a STOPPED lead blocking the path

# detector states (strings so they read directly in the ces_events telemetry)
GL_IDLE = "idle"      # not armed (moving, or stopped without stop context yet)
GL_ARMED = "armed"    # held at a stop by the model; watching for the release
GL_FIRED = "fired"    # alert consumed for this stop; re-arms only after driving off


class GreenLightDetector:
  """Pure per-tick state machine. update() returns True for exactly the one tick the alert
  should fire. No cereal, no params, no side effects — fully host-testable."""

  def __init__(self):
    self.state = GL_IDLE
    self._stopped_s = 0.0
    self._release_s = 0.0

  def reset(self) -> None:
    self.state = GL_IDLE
    self._stopped_s = 0.0
    self._release_s = 0.0

  def update(self, dt: float, v_ego: float, gas_pressed: bool, model_should_stop: bool,
             mdl_end_x: float, has_lead: bool, lead_drel: float, lead_vlead: float) -> bool:
    dt = min(max(float(dt), 0.0), 0.5)          # clamp scheduling hiccups (same as the CES loop)

    # any motion -> full reset (sunnypilot's recent_moving guard: the stop clock starts over,
    # and a consumed FIRED state re-arms only via this path — once per stop by construction)
    if float(v_ego) > GL_MOVING_V:
      self.reset()
      return False
    self._stopped_s += dt

    # driver already going on their own (creep/launch on gas): never ding on top of them, and
    # require a fresh stop context before arming again
    if gas_pressed:
      self.state = GL_IDLE
      self._release_s = 0.0
      return False

    if self._stopped_s < GL_MIN_STOP_S:
      return False                              # not a true, settled standstill yet

    if self.state == GL_IDLE:
      # FrogPilot-style stop-context arming: the model must be HOLDING us here (shouldStop, or a
      # short trajectory endpoint). endpoint 0.0 is treated as missing data, never as context.
      if model_should_stop or 0.0 < float(mdl_end_x) < GL_X_ARM_M:
        self.state = GL_ARMED
        self._release_s = 0.0
      return False

    if self.state == GL_ARMED:
      # the release: model no longer wants to stop AND the trajectory runs past the hold point
      released = (not model_should_stop) and float(mdl_end_x) > GL_X_GO_M
      # the lead rule: a close, stopped lead means the path is NOT actually free — hold the ding
      # until it departs (dRel opening or vLead rising lifts the block; radar losing it does too)
      lead_blocked = bool(has_lead) and 0.0 <= float(lead_drel) < GL_LEAD_NEAR_M \
                     and float(lead_vlead) < GL_LEAD_MOVING_V
      if released and not lead_blocked:
        self._release_s += dt                   # sunnypilot's sustained-trigger debounce
      else:
        self._release_s = 0.0
      if self._release_s >= GL_RELEASE_S:
        self.state = GL_FIRED
        return True

    return False                                # GL_FIRED: consumed until we actually drive off
