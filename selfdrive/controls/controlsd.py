#!/usr/bin/env python3
import math
from numbers import Number

from cereal import car, log
import cereal.messaging as messaging
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY, CV
from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, DT_CTRL, Priority, Ratekeeper
from openpilot.common.swaglog import cloudlog

from opendbc.car.car_helpers import interfaces
from opendbc.car.vehicle_model import VehicleModel
from openpilot.selfdrive.controls.lib.drive_helpers import clip_curvature, MAX_LATERAL_ACCEL_NO_ROLL, MIN_SPEED
from openpilot.selfdrive.controls.lib.lane_centering import LaneCenteringController
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.selfdrive.controls.lib.latcontrol_pid import LatControlPID
from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle, STEER_ANGLE_SATURATION_THRESHOLD
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.selfdrive.modeld.modeld import LAT_SMOOTH_SECONDS
from openpilot.selfdrive.locationd.helpers import PoseCalibrator, Pose

State = log.SelfdriveState.OpenpilotState
LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection

ACTUATOR_FIELDS = tuple(car.CarControl.Actuators.schema.fields.keys())


class Controls:
  def __init__(self) -> None:
    self.params = Params()
    cloudlog.info("controlsd is waiting for CarParams")
    self.CP = messaging.log_from_bytes(self.params.get("CarParams", block=True), car.CarParams)
    cloudlog.info("controlsd got CarParams")

    self.CI = interfaces[self.CP.carFingerprint](self.CP)

    self.sm = messaging.SubMaster(['liveDelay', 'liveParameters', 'liveTorqueParameters', 'modelV2', 'selfdriveState',
                                   'liveCalibration', 'livePose', 'longitudinalPlan', 'carState', 'carOutput',
                                   'driverMonitoringState', 'onroadEvents', 'driverAssistance'], poll='selfdriveState')
    self.pm = messaging.PubMaster(['carControl', 'controlsState'])

    self.steer_limited_by_safety = False
    self.curvature = 0.0
    self.desired_curvature = 0.0

    self.pose_calibrator = PoseCalibrator()
    self.calibrated_pose: Pose | None = None

    self.LoC = LongControl(self.CP)
    self.VM = VehicleModel(self.CP)
    self.LaC: LatControl
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      self.LaC = LatControlAngle(self.CP, self.CI, DT_CTRL)
    elif self.CP.lateralTuning.which() == 'pid':
      self.LaC = LatControlPID(self.CP, self.CI, DT_CTRL)
    elif self.CP.lateralTuning.which() == 'torque':
      self.LaC = LatControlTorque(self.CP, self.CI, DT_CTRL)

    # lanecenter2pnw: small bounded curvature trim toward lane-line center. See
    # selfdrive/controls/lib/lane_centering.py for the full design + safety-envelope contract.
    # Enabled BY DEFAULT (see the "DisableLaneCentering" read below in state_control) — the escape
    # hatch is the UI toggle, not this constructor. Tuning is a hot-reloaded JSON file, not Params.
    self.lane_centering = LaneCenteringController()
    # DisableLaneCentering is read at ~1 Hz, not every 100 Hz control tick (see
    # _read_lane_centering_enabled below) — a Params() disk read on every frame is unnecessary
    # here since a toggle flip only needs to take effect within about a second.
    self._lane_centering_frame = 0
    # Fail-safe initial value: stays False (feature inactive) until the first param read below
    # succeeds, even though the feature is enabled-by-default once that read completes. This avoids
    # ever computing a correction before we've actually confirmed the disable-toggle's live state.
    self._lane_centering_enabled = False
    # lanecenter2pnw telemetry: mem-param handle for publishing LaneCenterStatus (see the throttled
    # publish in state_control below), same /dev/shm/params channel VTSCStatus already uses (see
    # vtsc_pnw/vtsc_controller.py). Best-effort: if the mem store can't be opened for any reason, the
    # handle stays None and the publish below is a permanent no-op rather than ever raising here.
    try:
      self._mem_params = Params("/dev/shm/params")
    except Exception:
      self._mem_params = None

  def update(self):
    self.sm.update(15)
    if self.sm.updated["liveCalibration"]:
      self.pose_calibrator.feed_live_calib(self.sm['liveCalibration'])
    if self.sm.updated["livePose"]:
      device_pose = Pose.from_live_pose(self.sm['livePose'])
      self.calibrated_pose = self.pose_calibrator.build_calibrated_pose(device_pose)

  def _read_lane_centering_enabled(self) -> None:
    """
    lanecenter2pnw: refresh the lane-centering master-enable flag at ~1 Hz, not every 100 Hz control
    tick — matches the cadence CES/other pnw features use for their Params() reads (see
    selfdrive/controls/lib/ces_pnw/ces_pnw.py's _read_params for the same `frame % (1/DT_CTRL)`
    pattern). A toggle flip only needs to take effect within about a second; reading Params() at
    100 Hz would be pure overhead.

    The param is `DisableLaneCentering` (NOT `LaneCentering`) because the feature is enabled BY
    DEFAULT — this is the one deliberate exception to this fork's "new toggles default OFF" rule.
    See selfdrive/controls/lib/lane_centering.py and selfdrive/ui/layouts/settings/toggles.py for
    the full justification (short version: the correction is hard-bounded to a tiny curvature
    nudge, confidence-gated, releases smoothly, and can be switched off instantly from this toggle).

    Fail-safe: ANY exception reading the param (missing key, param-store hiccup, etc.) is treated as
    "disabled", not "keep the last-known state" — a param-store problem must never be able to leave
    a lateral-adjacent correction silently stuck on.
    """
    if self._lane_centering_frame % max(1, int(1.0 / DT_CTRL)) == 0:
      try:
        self._lane_centering_enabled = not self.params.get_bool("DisableLaneCentering")
      except Exception:
        self._lane_centering_enabled = False
    self._lane_centering_frame += 1

  def state_control(self):
    CS = self.sm['carState']

    # Update VehicleModel
    lp = self.sm['liveParameters']
    x = max(lp.stiffnessFactor, 0.1)
    sr = max(lp.steerRatio, 0.1)
    self.VM.update_params(x, sr)

    steer_angle_without_offset = math.radians(CS.steeringAngleDeg - lp.angleOffsetDeg)
    self.curvature = -self.VM.calc_curvature(steer_angle_without_offset, CS.vEgo, lp.roll)

    # Update Torque Params
    if self.CP.lateralTuning.which() == 'torque':
      torque_params = self.sm['liveTorqueParameters']
      if self.sm.all_checks(['liveTorqueParameters']) and torque_params.useParams:
        self.LaC.update_live_torque_params(torque_params.latAccelFactorFiltered, torque_params.latAccelOffsetFiltered,
                                           torque_params.frictionCoefficientFiltered)

    long_plan = self.sm['longitudinalPlan']
    model_v2 = self.sm['modelV2']

    CC = car.CarControl.new_message()
    CC.enabled = self.sm['selfdriveState'].enabled

    # Check which actuators can be enabled
    standstill = abs(CS.vEgo) <= max(self.CP.minSteerSpeed, 0.3) or CS.standstill
    CC.latActive = self.sm['selfdriveState'].active and not CS.steerFaultTemporary and not CS.steerFaultPermanent and \
                   (not standstill or self.CP.steerAtStandstill)
    CC.longActive = CC.enabled and not any(e.overrideLongitudinal for e in self.sm['onroadEvents']) and self.CP.openpilotLongitudinalControl

    actuators = CC.actuators
    actuators.longControlState = self.LoC.long_control_state

    # Enable blinkers while lane changing
    if model_v2.meta.laneChangeState != LaneChangeState.off:
      CC.leftBlinker = model_v2.meta.laneChangeDirection == LaneChangeDirection.left
      CC.rightBlinker = model_v2.meta.laneChangeDirection == LaneChangeDirection.right

    if not CC.latActive:
      self.LaC.reset()
      # lanecenter2pnw: zero the carried correction on every lateral disengage so it can't survive
      # into the next engagement and cause a step at re-engage.
      self.lane_centering.reset()
    if not CC.longActive:
      self.LoC.reset()

    # accel PID loop
    pid_accel_limits = self.CI.get_pid_accel_limits(self.CP, CS.vEgo, CS.vCruise * CV.KPH_TO_MS)
    actuators.accel = float(self.LoC.update(CC.longActive, CS, long_plan.aTarget, long_plan.shouldStop, pid_accel_limits))

    # Steering PID loop and lateral MPC
    # Reset desired curvature to current to avoid violating the limits on engage
    new_desired_curvature = model_v2.action.desiredCurvature if CC.latActive else self.curvature

    # lanecenter2pnw: apply the lane-centering trim BEFORE clip_curvature, so the correction still
    # passes through the same ISO lateral jerk/accel limiter as every other curvature source below —
    # this call can never bypass that limiter. When the feature is disabled (DisableLaneCentering=1,
    # or any of the controller's own gates below), update() returns new_desired_curvature unchanged,
    # so this is a byte-for-byte no-op path when the feature is off.
    self._read_lane_centering_enabled()
    lane_centered_curvature = self.lane_centering.update(
      new_desired_curvature,
      model_v2,
      CS.vEgo,
      self._lane_centering_enabled,
      CC.latActive,
      self.sm.all_checks(['modelV2']),
      bool(CS.leftBlinker or CS.rightBlinker),
    )
    # Defense-in-depth: only accept the trimmed curvature if it is finite. A NaN/inf here would flow
    # into self.desired_curvature and then serve as the rate-limit anchor for the NEXT tick, latching
    # a non-finite curvature that deleting the tuning file could not heal. The controller already
    # guarantees finiteness internally; this is the final guard before clip_curvature regardless.
    if math.isfinite(lane_centered_curvature):
      new_desired_curvature = lane_centered_curvature

    # lanecenter2pnw telemetry: publish the controller's status snapshot for the CES event logger
    # (selfdrived/ces_pnw.py reads it back over /dev/shm/params — same cross-process pattern as
    # VTSCStatus). PURE side effect, throttled to ~5 Hz (every 20 of these 100 Hz ticks) since this
    # is diagnostic telemetry, not a control input — it reads self.lane_centering.status, which was
    # already fully computed by the update() call above, and writes nothing back into
    # new_desired_curvature/self.desired_curvature or any other control state. put_nonblocking never
    # blocks the control loop on disk I/O, and the try/except means a param-store hiccup here can
    # never propagate into this 100 Hz loop. Reuses the existing _lane_centering_frame tick counter
    # purely as a modulo gate; it has no other coupling to this publish.
    if self._mem_params is not None and self._lane_centering_frame % 20 == 0:
      try:
        # Pass the dict directly (not a json string): put_nonblocking's JSON-typed-key path already
        # calls json.dumps on a dict (see common/params_pyx.pyx PYTHON_2_CPP), matching exactly how
        # VTSCStatus is published in vtsc_pnw/vtsc_controller.py._publish_overlay.
        self._mem_params.put_nonblocking("LaneCenterStatus", self.lane_centering.status)
      except Exception:
        pass

    self.desired_curvature, curvature_limited = clip_curvature(CS.vEgo, self.desired_curvature, new_desired_curvature, lp.roll)
    lat_delay = self.sm["liveDelay"].lateralDelay + LAT_SMOOTH_SECONDS

    actuators.curvature = self.desired_curvature
    steer, steeringAngleDeg, lac_log = self.LaC.update(CC.latActive, CS, self.VM, lp,
                                                       self.steer_limited_by_safety, self.desired_curvature,
                                                       curvature_limited, lat_delay)
    actuators.torque = float(steer)
    actuators.steeringAngleDeg = float(steeringAngleDeg)
    # Ensure no NaNs/Infs
    for p in ACTUATOR_FIELDS:
      attr = getattr(actuators, p)
      if not isinstance(attr, Number):
        continue

      if not math.isfinite(attr):
        cloudlog.error(f"actuators.{p} not finite {actuators.to_dict()}")
        setattr(actuators, p, 0.0)

    # steerlimit-log2pnw telemetry: PURE OBSERVATION snapshot of this tick's steering-limit signals —
    # never read back into any control value, computed entirely from values already finalized above
    # (curvature_limited, self.desired_curvature, lac_log). Mirrors the LaneCenterStatus publish above
    # byte-for-byte: same /dev/shm/params handle (self._mem_params), same throttle counter
    # (_lane_centering_frame, just a per-tick counter — reused here rather than adding a second one),
    # same try/except isolation so a param-store hiccup can never propagate into this 100 Hz loop. See
    # docs/STEERING-LIMITS.md §4 for the field-by-field design this dict follows.
    if self._mem_params is not None and self._lane_centering_frame % 20 == 0:
      try:
        v_ego_sq = max(CS.vEgo, MIN_SPEED) ** 2
        # Same formula clip_curvature itself uses for its lateral-accel ceiling (drive_helpers.py) —
        # the live ISO cap this tick, which varies with road roll.
        lat_accel_max = MAX_LATERAL_ACCEL_NO_ROLL + lp.roll * ACCELERATION_DUE_TO_GRAVITY
        # Pre-clip demand (new_desired_curvature, not the post-clip self.desired_curvature): using the
        # post-clip value here would make this field redundant with latAccelMax whenever curvLimited is
        # True, since clip_curvature pins the result to the ceiling — the pre-clip value is what shows
        # how much curvature the plan actually wanted before the ISO ceiling reduced it.
        lat_accel_demand = new_desired_curvature * v_ego_sq
        angle_des = getattr(lac_log, "steeringAngleDesiredDeg", None)
        if angle_des is None:
          # Fallback for a steerControlType this fork doesn't currently ship (pid/torque cars' LatControl
          # logs lack steeringAngleDesiredDeg) — same formula latcontrol_angle.py itself uses to compute
          # angle_steers_des (STEERING-LIMITS.md §0/§4). Both current cars (Raven, Lightning) are angle-
          # steered, so this branch is not expected to run in practice.
          angle_des = math.degrees(self.VM.get_steer_from_curvature(-self.desired_curvature, CS.vEgo, lp.roll)) + lp.angleOffsetDeg
        angle_actual = float(CS.steeringAngleDeg)
        angle_err = float(angle_des) - angle_actual
        # fordkappalog2pnw: commanded vs achieved CURVATURE (1/m), the empirical saturation signal --
        # a sustained large kErr while hands-off means the PSCM couldn't deliver the requested path,
        # i.e. the real curvature ceiling of THIS truck (2024+ CAN-FD/Q4, more torque than the
        # 2021-2023 Q3 trucks openpilot's own limits were validated on). Investigated against
        # opendbc_repo/opendbc/dbc/ford_lincoln_base_pt.dbc and opendbc_repo/opendbc/car/ford/{fordcan,
        # carstate}.py before writing this -- see docs/STEERING-LIMITS.md "Ford curvature interface"
        # section for the full writeup. Two findings that matter for what's logged below:
        #   1. There is NO parsed "LatCtlCurv_No_Cmd" and no PSCM curvature-feedback signal on this
        #      DBC. The only "LatCtlCurv_No_Actl" on the bus is a field *inside the LateralMotionControl
        #      /LateralMotionControl2 message openpilot itself transmits* (fordcan.py: `"LatCtlCurv_No_Actl":
        #      curvature` in create_lat_ctl_msg/create_lat_ctl2_msg) -- Ford's "_Actl" naming convention
        #      here means "the ADAS's own commanded value", not a PSCM-measured response. carstate.py
        #      never parses LateralMotionControl(2) as a received message. So "kCmd" below is sourced
        #      from self.desired_curvature (this tick's post-ISO-clip request), matching what
        #      STEERING-LIMITS.md calls the controlsd-level "commanded" value -- pre-clip demand is
        #      already covered by the existing latDem field above.
        #   2. kActl (achieved/measured curvature) is therefore DERIVED, not parsed: CS.yawRate is a
        #      genuine physical sensor value (Yaw_Data_FD1/VehYaw_W_Actl, transmitted by the GWM from
        #      the vehicle's own yaw sensor -- carstate.py:36, opendbc_repo/opendbc/dbc/
        #      ford_lincoln_base_pt.dbc:3169 -- not something openpilot computed), so kActl = yawRate /
        #      vEgo is the standard curvature-from-yaw-rate relation. This is the exact derivation this
        #      fork's own Ford lateral code already uses for "current curvature" (LateralAngleExt.
        #      get_current_curvature(), opendbc_repo/opendbc/car/ford/lateral_angle_pnw.py:318-329:
        #      `-CS.out.yawRate / max(CS.out.vEgoRaw, 0.1)`) -- reused here at the controlsd level
        #      instead of duplicating a second helper, kept car-agnostic (CS.yawRate is a cereal
        #      CarState field, not Ford-only, so this line runs harmlessly for the Raven too).
        # Sign convention: Ford's own wire/DBC convention is positive=LEFT (fordcan.py:60, "c2:
        # curvature of the centerline (positive is left)"), which is the OPPOSITE of openpilot's
        # internal self.desired_curvature convention -- confirmed by carcontroller.py negating it
        # before every CAN write ("-lat.apply_curvature" / "-self.apply_curvature_last") and by
        # latcontrol_angle.py/line 255 above negating it again before converting to a steering angle.
        # kCmd is therefore -self.desired_curvature (flips TO the Ford positive=left convention, not a
        # correction to what's being asked for). CS.yawRate is already positive=left (standard ISO
        # sensor convention -- lateral_angle_pnw.py's own get_current_curvature() negates yawRate
        # *because* it needs openpilot's internal convention; kActl here wants Ford's, so no negation).
        v_ego_kappa = max(CS.vEgo, MIN_SPEED)
        k_cmd = -self.desired_curvature
        k_actl = float(CS.yawRate) / v_ego_kappa
        k_err = k_cmd - k_actl
        # steertele2pnw: two additions for capability-analysis drives, both pure observation.
        #   1. latActive -- CC.latActive is already computed above (line ~144) and fed into
        #      LaC.update() this same tick (line 217); reusing it here (not re-deriving) so this
        #      field can never drift from what actually gated angDes/angAct this tick. Without it,
        #      a capability drive can't tell "openpilot commanded this angle and didn't achieve it"
        #      from "driver was hand-steering" -- when latActive is False, angDes freezes to
        #      CS.steeringAngleDeg (latcontrol_angle.py: angle_steers_des = steeringAngleDeg when
        #      not active), so angDes==angAct and any apparent "capability" in angErr is fake.
        #   2. angSat -- the un-fused angle-only saturation half. "sat" above is lac_log.saturated,
        #      which is LatControlAngle's *time-integrated, hysteresis-smoothed* fusion of
        #      (angle_control_saturated OR curvature_limited) (latcontrol.py:_check_saturation) --
        #      curvLim already isolates the curvature half, but the angle half (angle_control_saturated)
        #      is a local variable inside LatControlAngle.update() that never reaches angle_log, so it
        #      isn't reachable from lac_log here. Recomputed instead using the same threshold constant
        #      latcontrol_angle.py itself compares against (STEER_ANGLE_SATURATION_THRESHOLD, imported
        #      at module level above) applied to angErr (== angle_steers_des - CS.steeringAngleDeg,
        #      the same operands latcontrol_angle.py's non-Tesla branch compares). Caveat: on Tesla
        #      (CP.brand == "tesla"), latcontrol_angle.py's real angle_control_saturated is instead
        #      `steer_limited_by_safety` (already logged above as safeLim) -- this computed angSat is
        #      the Ford/EPS-style angle-saturation proxy, not a byte-for-byte mirror of the Tesla branch.
        angle_control_saturated = abs(angle_err) > STEER_ANGLE_SATURATION_THRESHOLD
        steer_limit_status = {
          "curvLim": bool(curvature_limited),
          "safeLim": bool(self.steer_limited_by_safety),
          "angDes": round(float(angle_des), 3),
          "angAct": round(angle_actual, 3),
          "angErr": round(angle_err, 3),
          "sat": bool(getattr(lac_log, "saturated", False)),
          "angSat": bool(angle_control_saturated),
          "latActive": bool(CC.latActive),
          "latDem": round(float(lat_accel_demand), 4),
          "latMax": round(float(lat_accel_max), 4),
          "curvMax": round(float(lat_accel_max / v_ego_sq), 6),
          "kCmd": round(float(k_cmd), 6),
          "kActl": round(float(k_actl), 6),
          "kErr": round(float(k_err), 6),
        }
        self._mem_params.put_nonblocking("SteerLimitStatus", steer_limit_status)
      except Exception:
        pass

    return CC, lac_log

  def publish(self, CC, lac_log):
    CS = self.sm['carState']

    # Orientation and angle rates can be useful for carcontroller
    # Only calibrated (car) frame is relevant for the carcontroller
    CC.currentCurvature = self.curvature
    if self.calibrated_pose is not None:
      CC.orientationNED = self.calibrated_pose.orientation.xyz.tolist()
      CC.angularVelocity = self.calibrated_pose.angular_velocity.xyz.tolist()

    CC.cruiseControl.override = CC.enabled and not CC.longActive and self.CP.openpilotLongitudinalControl
    CC.cruiseControl.cancel = CS.cruiseState.enabled and (not CC.enabled or not self.CP.pcmCruise)
    CC.cruiseControl.resume = CC.enabled and CS.cruiseState.standstill and not self.sm['longitudinalPlan'].shouldStop

    hudControl = CC.hudControl
    hudControl.setSpeed = float(CS.vCruiseCluster * CV.KPH_TO_MS)
    hudControl.speedVisible = CC.enabled
    hudControl.lanesVisible = CC.enabled
    hudControl.leadVisible = self.sm['longitudinalPlan'].hasLead
    hudControl.leadDistanceBars = self.sm['selfdriveState'].personality.raw + 1
    hudControl.visualAlert = self.sm['selfdriveState'].alertHudVisual

    hudControl.rightLaneVisible = True
    hudControl.leftLaneVisible = True
    if self.sm.valid['driverAssistance']:
      hudControl.leftLaneDepart = self.sm['driverAssistance'].leftLaneDeparture
      hudControl.rightLaneDepart = self.sm['driverAssistance'].rightLaneDeparture

    if self.sm['selfdriveState'].active:
      CO = self.sm['carOutput']
      if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
        self.steer_limited_by_safety = abs(CC.actuators.steeringAngleDeg - CO.actuatorsOutput.steeringAngleDeg) > \
                                              STEER_ANGLE_SATURATION_THRESHOLD
      else:
        self.steer_limited_by_safety = abs(CC.actuators.torque - CO.actuatorsOutput.torque) > 1e-2

    # TODO: both controlsState and carControl valids should be set by
    #       sm.all_checks(), but this creates a circular dependency

    # controlsState
    dat = messaging.new_message('controlsState')
    dat.valid = CS.canValid
    cs = dat.controlsState

    cs.curvature = self.curvature
    cs.longitudinalPlanMonoTime = self.sm.logMonoTime['longitudinalPlan']
    cs.lateralPlanMonoTime = self.sm.logMonoTime['modelV2']
    cs.desiredCurvature = self.desired_curvature
    cs.longControlState = self.LoC.long_control_state
    cs.upAccelCmd = float(self.LoC.pid.p)
    cs.uiAccelCmd = float(self.LoC.pid.i)
    cs.ufAccelCmd = float(self.LoC.pid.f)
    cs.forceDecel = bool((self.sm['driverMonitoringState'].awarenessStatus < 0.) or
                         (self.sm['selfdriveState'].state == State.softDisabling))

    lat_tuning = self.CP.lateralTuning.which()
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      cs.lateralControlState.angleState = lac_log
    elif lat_tuning == 'pid':
      cs.lateralControlState.pidState = lac_log
    elif lat_tuning == 'torque':
      cs.lateralControlState.torqueState = lac_log

    self.pm.send('controlsState', dat)

    # carControl
    cc_send = messaging.new_message('carControl')
    cc_send.valid = CS.canValid
    cc_send.carControl = CC
    self.pm.send('carControl', cc_send)

  def run(self):
    rk = Ratekeeper(100, print_delay_threshold=None)
    while True:
      self.update()
      CC, lac_log = self.state_control()
      self.publish(CC, lac_log)
      rk.monitor_time()


def main():
  config_realtime_process(4, Priority.CTRL_HIGH)
  controls = Controls()
  controls.run()


if __name__ == "__main__":
  main()
