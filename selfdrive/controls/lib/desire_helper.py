from cereal import log
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection

LANE_CHANGE_SPEED_MIN = 20 * CV.MPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.

# auto2xnor: nudgeless lane change — hold the blinker this long (no blindspot) to
# auto-start the lane change without a steering-wheel nudge.
AUTO_LANE_CHANGE_DELAY = 1.5  # seconds

# auto2xnor: auto-initiate (overtake) lane change — Tesla only, behind its own toggle.
# Triggered by closing on a confident slower lead on the highway; always LEFT (passing
# side), always BSM-gated. This does NOT see fast traffic approaching from far back in
# the target lane — only the car's rear blind-spot zone — so it is OFF by default and a
# separate opt-in from the blinker-initiated nudgeless feature.
AUTO_INITIATE_MIN_SPEED = 40 * CV.MPH_TO_MS   # only on the highway
AUTO_INITIATE_LEAD_PROB = 0.5                 # model lead confidence required
AUTO_INITIATE_MAX_LEAD_DIST = 90.0            # m — only consider a lead within this range
AUTO_INITIATE_MIN_CLOSING = 2.0               # m/s — must be closing on the lead this fast (vEgo - vLead)
AUTO_INITIATE_MIN_REL_SLOWER = 0.85           # lead must be < 85% of our speed to be "slow"
AUTO_INITIATE_ARM_TIME = 3.0                  # s the slow-lead condition must hold before initiating

DESIRES = {
  LaneChangeDirection.none: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.none,
    LaneChangeState.laneChangeFinishing: log.Desire.none,
  },
  LaneChangeDirection.left: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.laneChangeLeft,
    LaneChangeState.laneChangeFinishing: log.Desire.laneChangeLeft,
  },
  LaneChangeDirection.right: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.laneChangeRight,
    LaneChangeState.laneChangeFinishing: log.Desire.laneChangeRight,
  },
}


class DesireHelper:
  def __init__(self, CP=None):
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.lane_change_ll_prob = 1.0
    self.keep_pulse_timer = 0.0
    self.prev_one_blinker = False
    self.desire = log.Desire.none

    # auto2xnor: nudgeless lane change — Tesla only.
    # The Ford F-150 Lightning is explicitly excluded (driver asked to keep the
    # nudge requirement there); nudgeless is enabled only on Tesla brand cars.
    self.params = Params()
    self.brand = CP.brand if CP is not None else ""
    self.nudgeless_supported = self.brand == "tesla"
    self.nudgeless_lane_change = self.nudgeless_supported and self.params.get_bool("NudgelessLaneChange")
    self.auto_lane_change_timer = 0.0
    self._param_read_counter = 0

    # auto2xnor: auto-initiate (overtake) — Tesla only, separate toggle, default off
    self.auto_initiate_lane_change = self.nudgeless_supported and self.params.get_bool("AutoInitiateLaneChange")
    self.auto_initiate_arm_timer = 0.0     # how long the slow-lead condition has held
    self.auto_initiate_active = False      # latched for this lane-change cycle

  @staticmethod
  def get_lane_change_direction(CS):
    return LaneChangeDirection.left if CS.leftBlinker else LaneChangeDirection.right

  def _update_auto_initiate(self, v_ego, lead_one, blindspot_left, below_lane_change_speed):
    """auto2xnor: arm an automatic LEFT overtake when closing on a confident slow lead.
    Returns True once the condition has held for AUTO_INITIATE_ARM_TIME with the left
    blind spot clear and we're above the highway threshold. Tesla only."""
    if not self.auto_initiate_lane_change:
      self.auto_initiate_arm_timer = 0.0
      return False

    # model lead: prob + absolute speed v[0] + distance x[0]; guard empty lists
    lead_ok = False
    if lead_one is not None and lead_one.prob > AUTO_INITIATE_LEAD_PROB and len(lead_one.x) and len(lead_one.v):
      d_rel = lead_one.x[0]
      v_lead = lead_one.v[0]
      closing = v_ego - v_lead
      lead_ok = (0 < d_rel < AUTO_INITIATE_MAX_LEAD_DIST
                 and closing > AUTO_INITIATE_MIN_CLOSING
                 and v_lead < v_ego * AUTO_INITIATE_MIN_REL_SLOWER)

    # only on the highway, lane-change speed satisfied, and the left blind spot clear
    condition = (lead_ok and v_ego > AUTO_INITIATE_MIN_SPEED
                 and not below_lane_change_speed and not blindspot_left)
    if condition:
      self.auto_initiate_arm_timer += DT_MDL
    else:
      self.auto_initiate_arm_timer = 0.0

    return self.auto_initiate_arm_timer > AUTO_INITIATE_ARM_TIME

  def update(self, carstate, lateral_active, lane_change_prob, lead_one=None):
    v_ego = carstate.vEgo
    one_blinker = carstate.leftBlinker != carstate.rightBlinker
    below_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN

    # auto2xnor: refresh the toggles ~ every 3s so changing them doesn't need a restart
    # (both still gated to Tesla — never effective on the Ford)
    self._param_read_counter += 1
    if self._param_read_counter % 60 == 0:
      self.nudgeless_lane_change = self.nudgeless_supported and self.params.get_bool("NudgelessLaneChange")
      self.auto_initiate_lane_change = self.nudgeless_supported and self.params.get_bool("AutoInitiateLaneChange")

    # auto2xnor: evaluate the auto-overtake arm condition (LEFT only, BSM-gated)
    auto_initiate_armed = self._update_auto_initiate(v_ego, lead_one, carstate.leftBlindspot, below_lane_change_speed)

    if not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.auto_initiate_active = False  # auto2xnor: drop the latch on reset/timeout
    else:
      # LaneChangeState.off
      if self.lane_change_state == LaneChangeState.off and one_blinker and not self.prev_one_blinker and not below_lane_change_speed:
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_ll_prob = 1.0
        self.auto_lane_change_timer = 0.0  # auto2xnor: reset nudgeless timer on entry
        self.auto_initiate_active = False  # auto2xnor: a manual blinker is not an auto-overtake
        # Initialize lane change direction to prevent UI alert flicker
        self.lane_change_direction = self.get_lane_change_direction(carstate)

      # auto2xnor: auto-initiate overtake — no blinker needed. Enter preLaneChange to the
      # LEFT when the slow-lead condition has been armed. Latched so it drives the start.
      elif self.lane_change_state == LaneChangeState.off and auto_initiate_armed and not one_blinker:
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_ll_prob = 1.0
        self.auto_lane_change_timer = 0.0
        self.auto_initiate_active = True
        self.lane_change_direction = LaneChangeDirection.left

      # LaneChangeState.preLaneChange
      elif self.lane_change_state == LaneChangeState.preLaneChange:
        # auto2xnor: an auto-overtake latches the direction LEFT and ignores the blinker;
        # otherwise direction follows the driver's blinker as usual.
        if self.auto_initiate_active:
          self.lane_change_direction = LaneChangeDirection.left
        else:
          self.lane_change_direction = self.get_lane_change_direction(carstate)

        torque_applied = carstate.steeringPressed and \
                         ((carstate.steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.left) or
                          (carstate.steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.right))

        blindspot_detected = ((carstate.leftBlindspot and self.lane_change_direction == LaneChangeDirection.left) or
                              (carstate.rightBlindspot and self.lane_change_direction == LaneChangeDirection.right))

        # auto2xnor: nudgeless — accumulate time while the blinker is held with no
        # blindspot; once past the delay, allow the lane change without a wheel nudge.
        # Reset the timer whenever a blindspot is present so the hold must be clear.
        if self.nudgeless_lane_change and not blindspot_detected:
          self.auto_lane_change_timer += DT_MDL
        else:
          self.auto_lane_change_timer = 0.0
        auto_lane_change = self.nudgeless_lane_change and self.auto_lane_change_timer > AUTO_LANE_CHANGE_DELAY

        # auto2xnor: auto-overtake cancels if the slow-lead/clear condition stops holding
        # (e.g. lead sped up, or a vehicle entered the left blind spot) before it starts.
        if self.auto_initiate_active and not auto_initiate_armed:
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
          self.auto_initiate_active = False
        elif self.auto_initiate_active:
          # latched overtake: start as soon as the left blind spot is clear
          if not blindspot_detected:
            self.lane_change_state = LaneChangeState.laneChangeStarting
        elif not one_blinker or below_lane_change_speed:
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
        elif (torque_applied or auto_lane_change) and not blindspot_detected:
          self.lane_change_state = LaneChangeState.laneChangeStarting

      # LaneChangeState.laneChangeStarting
      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        # fade out over .5s
        self.lane_change_ll_prob = max(self.lane_change_ll_prob - 2 * DT_MDL, 0.0)

        # 98% certainty
        if lane_change_prob < 0.02 and self.lane_change_ll_prob < 0.01:
          self.lane_change_state = LaneChangeState.laneChangeFinishing

      # LaneChangeState.laneChangeFinishing
      elif self.lane_change_state == LaneChangeState.laneChangeFinishing:
        # fade in laneline over 1s
        self.lane_change_ll_prob = min(self.lane_change_ll_prob + DT_MDL, 1.0)

        if self.lane_change_ll_prob > 0.99:
          self.lane_change_direction = LaneChangeDirection.none
          self.auto_initiate_active = False  # auto2xnor: overtake complete — drop the latch
          if one_blinker:
            self.lane_change_state = LaneChangeState.preLaneChange
          else:
            self.lane_change_state = LaneChangeState.off

    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
      self.lane_change_timer = 0.0
    else:
      self.lane_change_timer += DT_MDL

    self.prev_one_blinker = one_blinker

    self.desire = DESIRES[self.lane_change_direction][self.lane_change_state]

    # Send keep pulse once per second during LaneChangeStart.preLaneChange
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.laneChangeStarting):
      self.keep_pulse_timer = 0.0
    elif self.lane_change_state == LaneChangeState.preLaneChange:
      self.keep_pulse_timer += DT_MDL
      if self.keep_pulse_timer > 1.0:
        self.keep_pulse_timer = 0.0
      elif self.desire in (log.Desire.keepLeft, log.Desire.keepRight):
        self.desire = log.Desire.none
