"""
greenlight2pnw / greenlead2pnw — Green Light + Lead Departure detection (PURE, unit-tested;
owned by CESController).

Chimes + shows a small alert when the car is held at a standstill and the way ahead opens.
Driver-requested split (field feedback 2026-07-13):
  - GREEN ("green"):   pure traffic-light case — stopped with NO lead car and the driving model
                       releases the path. Ding + "Light is green".
  - LEAD  ("lead"):    we were settled at a standstill behind a close, STOPPED lead and it pulled
                       away (opened up, sped up, or left radar). Ding + "Car ahead is leaving".
  - MOVING ("leadMoving"): the lead departs while we are already rolling — NO alert, telemetry
                       classification only.
DISPLAY/SOUND ONLY: this module never touches longitudinal or lateral control; its single output
is a one-cycle classification string that selfdrived turns into a LOW-priority alert event.
Model-based, car-agnostic (no fingerprints, no CAN) — works identically on the Tesla and the
Lightning.

ATTRIBUTION (best-of-both adjudication, studied 2026-07-12 — see CREDITS.md):
  - sunnypilot (Haibin Wen et al., `e2e_alerts_helper.py`, MIT): the core mechanics won.
    We adopt their INACTIVE -> ARMED -> CONSUMED state machine (fires exactly ONCE per stop, only
    a real drive-off re-arms), their trigger signal (model trajectory ENDPOINT beyond ~30 m =
    the e2e model releasing the hold — we already log it as mdl_end_x), their 0.3 s sustained-
    trigger debounce (a flickering model can't ding), and their 2 s "not recently moving" arming
    guard (never fires on a rolling stop).
  - FrogPilot (FrogAi, `frogpilot_events.py`, MIT): STOP-CONTEXT ARMING (arm only while the model
    is holding a stop) and the split alert pair itself — FrogPilot ships a separate lead-departure
    alert next to the green-light one; as of 2026-07-13 we do too (the earlier port folded lead
    departures into the green ding; the driver asked for the split back).
  - NOT adopted: sunnypilot gates on "openpilot not engaged" (CC.enabled). We follow FrogPilot
    and alert regardless of engagement — with op-long stopped long enough the car waits for a
    resume press, and that is exactly when the driver needs the ding.

Detection contract (all enforced by tests/test_green_light.py):
  - fires only from a true standstill (v_ego <= GL_MOVING_V, stopped >= GL_MIN_STOP_S);
  - arms only with stop context (model holding a stop while stationary);
  - GREEN requires the radar lead ABSENT for a sustained GL_NO_LEAD_S — a momentary dRel/status
    flicker on a departing (or ghost) lead can never turn a lead situation into a green ding;
  - LEAD requires the lead to have genuinely blocked us (close+stopped for GL_LEAD_BLOCK_MIN_S)
    RECENTLY (within GL_LEAD_FORGET_S — a lead that left long before the light change is
    forgotten and the eventual release is a plain green), plus the same sustained model release;
  - LEAD dings are rate-limited (GL_LEAD_COOLDOWN_S across stops) so creeping stop-and-go
    traffic can't chime on every cycle;
  - release must be sustained GL_RELEASE_S (model flicker can't ding, and never twice per stop);
  - never fires while the driver is already going (gas pressed, or any vehicle motion) — a lead
    departing while we roll is classified "leadMoving" for the drive log and nothing else.
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
GL_NO_LEAD_S = 1.0       # s the radar lead must be continuously ABSENT before a release counts
                         # as the no-lead GREEN case (radar flicker guard)
GL_LEAD_BLOCK_MIN_S = 0.5  # s a lead must block before this stop counts as "behind a car"
GL_LEAD_FORGET_S = 10.0  # s after the lead stopped blocking, the departure context expires
                         # (car ahead turned right on red long ago -> the green is still a GREEN)
GL_LEAD_COOLDOWN_S = 20.0  # s minimum spacing between LEAD dings (stop-and-go anti-spam);
                           # intentionally spans stops (NOT cleared by reset())

# detector states (strings so they read directly in the ces_events telemetry, field "glSt")
GL_IDLE = "idle"      # not armed (moving, or stopped without stop context yet)
GL_ARMED = "armed"    # held at a stop by the model; watching for the release
GL_FIRED = "fired"    # alert consumed for this stop; re-arms only after driving off

# classification tokens update() returns (ces_events field "glEv"; None = nothing this tick)
GL_EV_GREEN = "green"            # no-lead path opens -> greenLight alert
GL_EV_LEAD = "lead"              # stopped lead departs from OUR standstill -> leadDeparting alert
GL_EV_LEAD_MOVING = "leadMoving"  # lead departs while we already roll -> telemetry only, NO alert


class GreenLightDetector:
  """Pure per-tick state machine. update() returns a classification token (GL_EV_*) for exactly
  the one tick something happened, else None. No cereal, no params, no side effects — fully
  host-testable."""

  def __init__(self):
    self.state = GL_IDLE
    self._stopped_s = 0.0
    self._release_s = 0.0
    self._lead_absent_s = 0.0     # continuous s with NO radar lead at this standstill
    self._lead_block_s = 0.0      # continuous s the current lead has been blocking (close+stopped)
    self._since_block_s = 0.0     # s since a lead last blocked (departure-context freshness)
    self._had_lead = False        # a lead genuinely blocked us during THIS stop
    self._lead_cooldown_s = 0.0   # LEAD-ding rate limit; deliberately survives reset()

  def reset(self) -> None:
    """Per-stop state only — the LEAD cooldown intentionally survives (anti stop-and-go spam)."""
    self.state = GL_IDLE
    self._stopped_s = 0.0
    self._release_s = 0.0
    self._lead_absent_s = 0.0
    self._lead_block_s = 0.0
    self._since_block_s = 0.0
    self._had_lead = False

  def update(self, dt: float, v_ego: float, gas_pressed: bool, model_should_stop: bool,
             mdl_end_x: float, has_lead: bool, lead_drel: float, lead_vlead: float) -> str | None:
    dt = min(max(float(dt), 0.0), 0.5)          # clamp scheduling hiccups (same as the CES loop)
    self._lead_cooldown_s = max(self._lead_cooldown_s - dt, 0.0)

    lead_present = bool(has_lead)
    # a close, stopped lead = the path is NOT actually free (we're stopped behind a car)
    lead_blocked = lead_present and 0.0 <= float(lead_drel) < GL_LEAD_NEAR_M \
                   and float(lead_vlead) < GL_LEAD_MOVING_V

    # any motion -> full reset (sunnypilot's recent_moving guard: the stop clock starts over,
    # and a consumed FIRED state re-arms only via this path — once per stop by construction).
    # If we were settled behind a stopped lead and are rolling off behind it now, classify the
    # moment for the drive log — the driver is already moving, so NEVER an alert (field rule #3).
    if float(v_ego) > GL_MOVING_V:
      ev = GL_EV_LEAD_MOVING if (self.state == GL_ARMED and self._had_lead and not lead_blocked) else None
      self.reset()
      return ev
    self._stopped_s += dt

    # lead bookkeeping at standstill — state-independent so a lead blocking BEFORE arming counts:
    #  - _lead_absent_s: the sustained-absence window GREEN requires (radar flicker guard);
    #  - _had_lead: sticky once a lead has blocked for GL_LEAD_BLOCK_MIN_S CUMULATIVE within this
    #    stop (Gemini review catch: stationary leads flicker on radar — a momentary dropout must
    #    not reset the evidence, or a real departure could classify as a no-lead green). Expires
    #    GL_LEAD_FORGET_S after the lead last blocked, together with its evidence.
    self._lead_absent_s = (self._lead_absent_s + dt) if not lead_present else 0.0
    if lead_blocked:
      self._lead_block_s += dt
      self._since_block_s = 0.0
      if self._lead_block_s >= GL_LEAD_BLOCK_MIN_S:
        self._had_lead = True
    else:
      self._since_block_s += dt
      if self._since_block_s > GL_LEAD_FORGET_S:
        self._had_lead = False                  # stale departure: the next release is a green
        self._lead_block_s = 0.0                # fresh evidence required for a new lead context

    # driver already going on their own (creep/launch on gas): never ding on top of them, and
    # require a fresh stop context before arming again
    if gas_pressed:
      self.state = GL_IDLE
      self._release_s = 0.0
      return None

    if self._stopped_s < GL_MIN_STOP_S:
      return None                               # not a true, settled standstill yet

    if self.state == GL_IDLE:
      # FrogPilot-style stop-context arming: the model must be HOLDING us here (shouldStop, or a
      # short trajectory endpoint). endpoint 0.0 is treated as missing data, never as context.
      if model_should_stop or 0.0 < float(mdl_end_x) < GL_X_ARM_M:
        self.state = GL_ARMED
        self._release_s = 0.0
      return None

    if self.state == GL_ARMED:
      # the release: model no longer wants to stop AND the trajectory runs past the hold point;
      # a still-blocking lead means the path is NOT actually free — the timer holds at zero
      released = (not model_should_stop) and float(mdl_end_x) > GL_X_GO_M
      if released and not lead_blocked:
        self._release_s += dt                   # sunnypilot's sustained-trigger debounce
      else:
        self._release_s = 0.0
      if self._release_s >= GL_RELEASE_S:
        if self._had_lead:
          # the car we were stopped behind pulled away (opened, sped up, or left radar) and the
          # model followed it out: LEAD departure. Radar losing a departing lead lands HERE (via
          # the sticky _had_lead), never in the green branch — a dRel/status dropout can't turn
          # a departure into a "no lead" green ding.
          self.state = GL_FIRED
          if self._lead_cooldown_s > 0.0:
            return None                         # consumed silently (stop-and-go rate limit)
          self._lead_cooldown_s = GL_LEAD_COOLDOWN_S
          return GL_EV_LEAD
        if self._lead_absent_s >= GL_NO_LEAD_S:
          # pure traffic-light case: nobody ahead for a sustained window and the path opened
          self.state = GL_FIRED
          return GL_EV_GREEN
        # released, but some lead is on the radar (or was < GL_NO_LEAD_S ago) without ever having
        # blocked us — e.g. a stopped car at the NEXT intersection, or a ghost flicker. Neither a
        # pure green (driver rule #1: green only with NO lead) nor a departure. Stay ARMED and
        # primed: the moment the absence window clears (or the lead comes in and later departs),
        # the correct classification fires.
        self._release_s = GL_RELEASE_S
      return None

    return None                                 # GL_FIRED: consumed until we actually drive off
