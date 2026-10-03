"""stopgo2pnw: hand the STOP decision to the lead-aware MPC while a lead is demonstrably pulling away from a standstill.

Why (drives/2026-10-02/tesla-resume-from-stop/DRIVE_REPORT.md): the Raven's CES `stopLatch` pins Experimental until vEgo > 1.3 m/s,
and in Experimental the planner takes `should_stop = e2e OR mpc` and `a = min(e2e, mpc)`. When the lead creeps away (1-1.5 m/s) the
MPC (lead-aware) wants to go for seconds while the e2e model's shouldStop stays 1 (desiredAcceleration ~0.02): the car sits at
`stopping` (-2.0) until the driver presses the gas (4 of 26 engaged stops).

What: a pure per-tick gate. While it says True the planner uses the MPC's should_stop / a_target INSTEAD of the e2e veto (and caps
the launch accel). It is deliberately narrow:
  * arms only when ego is stopped (v < START_V) with a lead that is IN LANE (|yRel|), vision-confirmed (modelProb), radar-backed
    when the car has a radar, OPENING (vLead >= OPEN_V, gap not shrinking) for >= OPEN_S, >= MIN_GAP away, and >= MIN_GAIN further
    than when the window began;
  * sustains only while the lead stays ok and the gap does not shrink, and only until vEgo reaches RELEASE_V (the stopLatch
    release -- after that today's normal following applies);
  * never with the driver braking or forceDecel, never while the e2e model is braking the moving car (accel < -0.3 above 0.3 m/s), at
    most MAX_HANDOFF_S long; any failed condition re-latches to today's behaviour at once;
  * ONE hand-off per stop: once one has ended (for any reason) the next needs a new context -- the lead itself stopping (a real queue),
    the ego above REARM_V, or the driver's gas. A lost lead or a dRel step is NOT one: radar noise on the SAME moving lead does both.
    Otherwise a lead that keeps moving away while the e2e model is
    RIGHT to hold (a red light the lead runs, a pedestrian) would be answered with a lunge every few seconds.
With NO lead (red light, stop sign, crosswalk) the gate is False on every tick: the e2e stop stays authoritative.

Pure logic, no I/O except cloudlog.event on a change.
"""
import math

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw_constants as C

START_V = 0.5            # m/s: ego counts as stopped (arming)
RELEASE_V = C.NOCHILL_RELEASE_V   # m/s: the stopLatch releases here -> the bypass ends (1.3)
MIN_GAP = 8.0            # m: lead at least this far ahead to arm
OPEN_V = 0.5             # m/s: lead speed that counts as pulling away
KEEP_V = 0.3             # m/s: hysteresis -- below this once engaged the lead has stopped/reversed -> re-latch
OPEN_S = 1.0             # s: lead must have been opening this long
MIN_GAIN = 1.0           # m: gap must have grown this much since the window began
SHRINK_TOL = 0.5         # m: gap may dip this far below its running max before it counts as shrinking (radar noise)
KEEP_SHRINK_TOL = 1.0    # m: same, once engaged (the car is rolling up behind the lead)
MAX_YREL = 1.5           # m: lead must be in lane
MIN_MODEL_PROB = 0.5     # vision must confirm the lead (radard's low-speed radar-only override reports modelProb 0)
VISION_GRACE_S = 2.0     # modelProb flickers 0 <-> ~1 on a real, radar-tracked lead at 8-13 m (07:54:26, 09-30 07:50: 1-1.5 s dropouts); a lead vision
                         # confirmed within this long ago and that the radar still tracks counts as confirmed. Longer = vision has really lost it.
MAX_HANDOFF_S = 5.0      # s: one hand-off lasts at most this long (behind a lead crawling <= 1.3 m/s it would otherwise run 5-8 s / 6-10 m)
E2E_BRAKE_V = 0.3        # m/s: the e2e model only says "stop" below this; above it, it slows the car through its requested accel ...
E2E_BRAKE_A = -0.3       # m/s^2: ... so an e2e accel below this while moving means the model is braking for something -> hand-off ends
QUEUE_V = 0.3            # m/s: the lead itself at (near) standstill ...
QUEUE_S = 0.5            # s: ... for this long (continuously) is a real queue stop -> a new context
                         # (with REARM_V and the driver's gas, the ONLY ones: a lost lead / a dRel jump is radar noise)
REARM_V = 3.0            # m/s: the ego got going properly -> a new context
ACCEL_CAP = 0.8          # m/s^2: launch accel ceiling while the bypass is active (the Tesla has no gentle_launch_accel: +inf there)


class StopGoHandoff:
  def __init__(self):
    self.active = False
    self.why = "idle"
    self._t_open = 0.0
    self._d_ref = 0.0
    self._d_max = 0.0
    self._vis_age = math.inf   # s since vision last confirmed the lead; inf = never (in this arming/hand-off run)
    self._dur = 0.0            # s the current hand-off has lasted
    self._used = False         # a hand-off already happened in this stop context: no second until _track says the context changed
    self._stop_s = 0.0

  def _clear(self):
    self._t_open = 0.0
    self._d_ref = 0.0
    self._d_max = 0.0
    self._vis_age = math.inf
    self._dur = 0.0

  def _lead_ok(self, lead, radar_expected: bool, dt: float) -> str | None:
    """None when the lead is usable, else the reason it is not."""
    if lead is None or not bool(getattr(lead, "status", False)):
      return "noLead"
    d, vl, y, mp = (float(getattr(lead, k, float("nan"))) for k in ("dRel", "vLead", "yRel", "modelProb"))
    if not all(math.isfinite(x) for x in (d, vl, y, mp)):
      return "badLead"
    if abs(y) > MAX_YREL:
      return "offLane"
    if mp >= MIN_MODEL_PROB:
      self._vis_age = 0.0
    else:
      self._vis_age += dt
      if self._vis_age > VISION_GRACE_S:
        return "noVision"
    if radar_expected and not bool(getattr(lead, "radar", False)):
      return "noRadar"
    return None

  def update(self, enabled: bool, active: bool, v_ego: float, lead, radar_expected: bool, driver_braking: bool,
             force_decel: bool, dt: float, driver_gas: bool = False, e2e_accel: float = 0.0, experimental: bool = True) -> bool:
    """One planner tick. True = the MPC's stop decision replaces the e2e veto this tick."""
    was = self.active
    why = self._step(enabled, active, v_ego, lead, radar_expected, driver_braking, force_decel, dt, driver_gas, e2e_accel, experimental)
    self.why = why
    if self.active != was:
      d = float(getattr(lead, "dRel", float("nan"))) if lead is not None else float("nan")
      vl = float(getattr(lead, "vLead", float("nan"))) if lead is not None else float("nan")
      cloudlog.event("stop_go_handoff", on=self.active, why=why, v_ego=round(float(v_ego), 2), dRel=round(d, 1), vLead=round(vl, 2),
                     gain=round(self._d_max - self._d_ref, 1), radar=bool(getattr(lead, "radar", False)),
                     modelProb=round(float(getattr(lead, "modelProb", 0.0)), 2) if lead is not None else None,
                     e2e_accel=round(float(e2e_accel), 2), dur=round(self._dur, 1))
    return self.active

  def _end(self):
    """A hand-off ends (for ANY reason): it is USED -- no second one until the stop context changes (see _track)."""
    if self.active:
      self._used = True
    self.active = False
    self._clear()

  def _track(self, v_ego, lead, driver_gas, dt):
    """Context tracking that runs every tick: what makes a NEW hand-off legitimate after one was used. A real queue (the lead itself stops),
    the ego leaving the stop (> REARM_V) or the driver's gas. Anything else -- the e2e braking us back down while
    the lead is still moving away -- is the e2e model being RIGHT, and must not be answered with another lunge."""
    vl = None
    if lead is not None and bool(getattr(lead, "status", False)):
      vl = float(getattr(lead, "vLead", float("nan")))
      if not math.isfinite(vl):
        vl = None
    if vl is None:
      self._stop_s = 0.0      # no usable lead this tick: NOT a new context (radar glitches / dropouts of the SAME moving lead are ordinary)
    elif vl < QUEUE_V:
      self._stop_s += dt
      if self._stop_s >= QUEUE_S:
        self._used = False
    else:
      self._stop_s = 0.0
    if v_ego > REARM_V or driver_gas:
      self._used = False

  def _step(self, enabled, active, v_ego, lead, radar_expected, driver_braking, force_decel, dt, driver_gas, e2e_accel, experimental) -> str:
    if not enabled:
      self._end()
      self._used = False
      return "off"
    if not active or not math.isfinite(v_ego):
      self._end()
      self._used = False      # op-long off / driver took over: a new engagement is a new context
      return "inactive"
    self._track(v_ego, lead, bool(driver_gas), dt)
    if not experimental:
      self._end()
      return "chill"
    if v_ego >= RELEASE_V or driver_braking or force_decel:
      self._end()
      return "inactive"
    if v_ego > E2E_BRAKE_V and float(e2e_accel) < E2E_BRAKE_A:   # the e2e model slows the car through its accel above 0.3 m/s, not through shouldStop
      self._end()
      return "e2eBraking"
    bad = self._lead_ok(lead, radar_expected, dt)
    if bad is not None:
      self._end()
      return bad
    d, vl = float(lead.dRel), float(lead.vLead)
    if self.active:   # sustain: only the lead matters now (v_ego < RELEASE_V is checked above)
      self._dur += dt
      if self._dur >= MAX_HANDOFF_S:
        self._end()
        return "timeCap"
      if vl < KEEP_V:
        self._end()
        return "leadStopped"
      if d < self._d_max - KEEP_SHRINK_TOL:
        self._end()
        return "gapShrinking"
      self._d_max = max(self._d_max, d)
      return "handoff"
    if self._used:
      self._clear()
      return "used"
    if v_ego >= START_V:   # not stopped and not already handed off: cannot arm
      self._clear()
      return "moving"
    if vl >= OPEN_V and (self._t_open == 0.0 or d >= self._d_max - SHRINK_TOL):
      if self._t_open == 0.0:
        self._d_ref = d
        self._d_max = d
      self._t_open += dt
      self._d_max = max(self._d_max, d)
      if self._t_open >= OPEN_S and d >= MIN_GAP and d - self._d_ref >= MIN_GAIN:
        self.active = True
        self._dur = 0.0
        return "handoff"
      return "arming"
    self._clear()
    return "notOpening"


def combine_stop_plan(experimental: bool, handoff: bool, a_e2e: float, stop_e2e: bool, a_mpc: float, stop_mpc: bool):
  """The planner's e2e/MPC combine -> (a_target, should_stop, e2e_binds). handoff False is EXACTLY the stock logic (Experimental: min accel,
  OR of the stops; Chill: MPC only); handoff True (only ever with experimental) uses the MPC's stop and accel, capped at ACCEL_CAP."""
  if not experimental:
    return a_mpc, stop_mpc, False
  if handoff:
    return min(a_mpc, ACCEL_CAP), stop_mpc, False
  return min(a_e2e, a_mpc), bool(stop_e2e or stop_mpc), a_e2e < a_mpc
