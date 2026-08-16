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
AUTO_LANE_CHANGE_DELAY = 0.75  # seconds (driver feedback: 1.5 felt too slow to commit the change)

# nudgelesshighway2pnw: gate the AUTO (nudgeless) lane change to highways only. Driver report — a
# family member flipped the turn signal to make an ordinary CITY turn and openpilot auto-lane-changed
# into cross-traffic. The manual steering-torque nudge path is UNCHANGED and still works everywhere;
# only the no-touch auto path is restricted here.
HIGHWAY_MIN_SPEED = 45 * CV.MPH_TO_MS  # speed floor: allowed even with no/stale map data (fail-open on speed alone)
# mapd's HighwayClass enum (cereal/custom.capnp), freeway-grade values only. Deliberately excludes
# primary/secondary/tertiary/unclassified/residential/livingStreet — those carry city arterials with
# cross-traffic, which is exactly the road class the driver report happened on.
FREEWAY_CLASSES = frozenset({"motorway", "motorwayLink", "trunk", "trunkLink"})

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

    # auto2xnor: nudgeless lane change — Tesla, plus the Ford F-150 Lightning.
    # Gated to specific platforms, NOT whole brands: every Tesla, and the F-150
    # Lightning specifically (CAR.FORD_F_150_LIGHTNING_MK1). Other Ford models
    # (and all other brands) keep the steering-wheel nudge requirement.
    self.params = Params()
    self.brand = CP.brand if CP is not None else ""
    self.car_fingerprint = CP.carFingerprint if CP is not None else ""
    self.nudgeless_supported = self.brand == "tesla" or self.car_fingerprint == "FORD_F_150_LIGHTNING_MK1"
    # toggles-invert2pnw: NudgeForLaneChange is opt-out (ON == nudge required); nudgeless by default.
    self.nudgeless_lane_change = self.nudgeless_supported and not self.params.get_bool("NudgeForLaneChange")
    self.auto_lane_change_timer = 0.0
    self._param_read_counter = 0

    # nudgelesshighway2pnw: highway/freeway gate for the auto (nudgeless) path — mem-param read of
    # mapd's bridged road class, same guarded-import pattern speedadjust_controller.py uses (a mem
    # Params("/dev/shm/params") read, NOT a msgq/carState subscription).
    self._map_highway_class = ""  # "" == unknown/no data -> fail-safe: only the speed floor can allow auto
    self._map_class_read_counter = 0
    import platform
    try:
      from openpilot.common.params import Params as _P
      self.mem_params = _P("/dev/shm/params") if platform.system() != "Darwin" else self.params
    except Exception:
      self.mem_params = None

  @staticmethod
  def get_lane_change_direction(CS):
    return LaneChangeDirection.left if CS.leftBlinker else LaneChangeDirection.right

  def update(self, carstate, lateral_active, lane_change_prob):
    v_ego = carstate.vEgo
    one_blinker = carstate.leftBlinker != carstate.rightBlinker
    below_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN

    # auto2xnor: refresh the nudgeless toggle ~ every 3s so changing it doesn't need a restart
    # (still gated by nudgeless_supported — Tesla + F-150 Lightning only)
    self._param_read_counter += 1
    if self._param_read_counter % 60 == 0:
      self.nudgeless_lane_change = self.nudgeless_supported and not self.params.get_bool("NudgeForLaneChange")

    # nudgelesshighway2pnw: refresh the map-derived road class ~ every 1s (20 cycles @ 20Hz). Fail-safe
    # to "" (unknown) on any missing/unparseable read — never raise, never let a bad read widen the gate.
    self._map_class_read_counter += 1
    if self._map_class_read_counter % 20 == 0:
      hwy_class = ""
      if self.mem_params is not None:
        try:
          raw = self.mem_params.get("MapHighwayClass", return_default=True)
          raw = raw.decode() if isinstance(raw, bytes) else raw
          hwy_class = raw if raw else ""
        except Exception:
          hwy_class = ""
      self._map_highway_class = hwy_class

    if not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
    else:
      # LaneChangeState.off
      if self.lane_change_state == LaneChangeState.off and one_blinker and not self.prev_one_blinker and not below_lane_change_speed:
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_ll_prob = 1.0
        self.auto_lane_change_timer = 0.0  # auto2xnor: reset nudgeless timer on entry
        # Initialize lane change direction to prevent UI alert flicker
        self.lane_change_direction = self.get_lane_change_direction(carstate)

      # LaneChangeState.preLaneChange
      elif self.lane_change_state == LaneChangeState.preLaneChange:
        # Update lane change direction
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

        # nudgelesshighway2pnw: the auto path only fires on a highway/freeway — a mapd freeway-grade
        # road class, OR (fail-safe when map data is missing/stale) v_ego at/above the speed floor.
        # City streets with no map data and low speed are always blocked, matching the manual-nudge-
        # only behavior a driver would expect making an ordinary city turn.
        on_highway = self._map_highway_class in FREEWAY_CLASSES or v_ego >= HIGHWAY_MIN_SPEED
        auto_lane_change = self.nudgeless_lane_change and self.auto_lane_change_timer > AUTO_LANE_CHANGE_DELAY and on_highway

        if not one_blinker or below_lane_change_speed:
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
