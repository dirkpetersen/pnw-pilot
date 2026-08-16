import time

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
# fail-review fix: hysteresis disarm floor for HIGHWAY_MIN_SPEED — arm the speed-alone path at
# HIGHWAY_MIN_SPEED, only disarm once BELOW this lower floor. Debounces a v_ego reading oscillating
# right at the 45 mph line from repeatedly toggling on_highway (compounds with FIX 2's continuous-hold
# requirement below, but the latch alone also protects the "stayed just above 45" case).
HIGHWAY_DISARM_SPEED = 42 * CV.MPH_TO_MS
# mapd's HighwayClass enum (cereal/custom.capnp), freeway-grade values only. Deliberately excludes
# primary/secondary/tertiary/unclassified/residential/livingStreet — those carry city arterials with
# cross-traffic, which is exactly the road class the driver report happened on.
FREEWAY_CLASSES = frozenset({"motorway", "motorwayLink", "trunk", "trunkLink"})
# fail-review fix: MapHighwayClass is a PERSISTENT mem-param that mapd_configd only overwrites while
# mapdOut is alive -- a dead/stalled mapd otherwise latches whatever freeway class it last saw, which
# would re-enable the exact city-turn auto-lane-change this branch exists to prevent (drive from a
# freeway onto a city street with mapd wedged). Reject the class as unknown once its bridged write-
# timestamp (MapHighwayClassTs, mapd_configd.py) is older than this.
MAP_CLASS_TTL_S = 5.0

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
    self._map_class_ts = -1e9    # fail-review fix: monotonic write-ts of the class above; -1e9 = never seen -> always stale until the first successful read
    self._map_class_read_counter = 0
    self._speed_armed = False    # fail-review fix: hysteresis latch for the HIGHWAY_MIN_SPEED floor
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

    # fail-review fix: hysteresis latch for the HIGHWAY_MIN_SPEED floor — arm at/above HIGHWAY_MIN_SPEED,
    # only disarm once below HIGHWAY_DISARM_SPEED. Runs every tick (not just while a lane change is
    # pending) so it reflects the car's actual recent speed the moment a blinker goes up.
    if v_ego >= HIGHWAY_MIN_SPEED:
      self._speed_armed = True
    elif v_ego < HIGHWAY_DISARM_SPEED:
      self._speed_armed = False

    # auto2xnor: refresh the nudgeless toggle ~ every 3s so changing it doesn't need a restart
    # (still gated by nudgeless_supported — Tesla + F-150 Lightning only)
    self._param_read_counter += 1
    if self._param_read_counter % 60 == 0:
      self.nudgeless_lane_change = self.nudgeless_supported and not self.params.get_bool("NudgeForLaneChange")

    # nudgelesshighway2pnw: refresh the map-derived road class ~ every 0.5s (10 cycles @ 20Hz — FIX 3:
    # tightened from 1s so a freeway->city change can't be missed for longer than the AUTO_LANE_CHANGE_DELAY
    # hold). Fail-safe to "" / very-stale ts on any missing/unparseable read — never raise, never let a
    # bad read widen the gate. Reads BOTH the class and its bridged write-timestamp (mapd_configd.py) —
    # the ts is what mapd_configd stamped, not when WE last read it, so a dead/stalled mapd (which stops
    # advancing the ts) is caught even though our own read keeps happening on schedule.
    self._map_class_read_counter += 1
    if self._map_class_read_counter % 10 == 0:
      hwy_class = ""
      hwy_ts = -1e9
      if self.mem_params is not None:
        try:
          raw = self.mem_params.get("MapHighwayClass", return_default=True)
          raw = raw.decode() if isinstance(raw, bytes) else raw
          hwy_class = raw if raw else ""
        except Exception:
          hwy_class = ""
        try:
          raw_ts = self.mem_params.get("MapHighwayClassTs", return_default=True)
          raw_ts = raw_ts.decode() if isinstance(raw_ts, bytes) else raw_ts
          hwy_ts = float(raw_ts) if raw_ts else -1e9
        except Exception:
          hwy_ts = -1e9
      self._map_highway_class = hwy_class
      self._map_class_ts = hwy_ts

    # fail-review fix: the freeway-class half of the gate only counts if the bridged write-ts is FRESH
    # (mapd_configd is alive and actually publishing) — a dead/stalled mapd can no longer latch a stale
    # freeway reading. Computed every tick (not just while pending) so FIX 2's continuous-hold
    # requirement below sees a stable, up-to-date value.
    # Gemini re-review: bound the age BOTH ways. A ts ahead of "now" (negative age) would otherwise
    # pass `age <= MAP_CLASS_TTL_S` and wrongly trust a stale class. Unreachable via the real
    # /dev/shm tmpfs mem-param path (wiped by the same reboot that resets time.monotonic()), but the
    # Darwin/sim fallback (mem_params = self.params, on-disk) can surface a pre-reboot ts.
    map_class_age = time.monotonic() - self._map_class_ts
    map_class_fresh = 0.0 <= map_class_age <= MAP_CLASS_TTL_S
    on_highway = (map_class_fresh and self._map_highway_class in FREEWAY_CLASSES) or self._speed_armed

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
        # nudgelesshighway2pnw FIX 2: also require on_highway (computed above) for EVERY tick the timer
        # accumulates — not just at the moment it crosses the delay threshold. Without this, the timer
        # could arm fully during a city-street blinker hold (nudgeless_lane_change true, no blindspot,
        # off-highway) and then fire on the very first tick on_highway flips true (a single-tick 45.0 mph
        # speed blip, or a momentary mapd way-match flicker to "motorway"). Gating accumulation itself
        # means a full continuous AUTO_LANE_CHANGE_DELAY of on_highway is required, not just one tick.
        if self.nudgeless_lane_change and not blindspot_detected and on_highway:
          self.auto_lane_change_timer += DT_MDL
        else:
          self.auto_lane_change_timer = 0.0

        # City streets with no fresh map data and low speed are always blocked, matching the manual-
        # nudge-only behavior a driver would expect making an ordinary city turn. `and on_highway` here
        # is redundant with the accumulation gate above (the timer can't exceed the delay without it) but
        # kept as defense-in-depth against a future refactor of the accumulation block.
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
