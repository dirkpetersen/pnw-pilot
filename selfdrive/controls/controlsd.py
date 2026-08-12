#!/usr/bin/env python3
import math
import time
from collections import deque
from numbers import Number

from cereal import car, log
import cereal.messaging as messaging
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY, CV
from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, DT_CTRL, Priority, Ratekeeper
from openpilot.common.swaglog import cloudlog

from opendbc.car.car_helpers import interfaces
from opendbc.car.vehicle_model import VehicleModel
from openpilot.selfdrive.controls.lib.drive_helpers import clip_curvature, lat_accel_limit, MIN_SPEED
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

# steerevent2pnw: edge-triggered "flight recorder" tuning (docs/pnw/LANE-DEPARTURE-LOGGING-PROPOSALS.md
# Proposal 1). All PURE OBSERVATION -- these only gate what gets appended to a local ring buffer and
# when the rare edge mem-param publish fires; nothing here is read back into any control value.
FLIGHT_PRE_N = 12          # ring-buffer samples kept at all times (~2.4 s @ the ~5 Hz publish cadence
                            # steer_limit_status already runs at below -- see the append site for why
                            # this doesn't sample any faster than that).
FLIGHT_POST_MAX = 15       # hard cap on post-edge samples appended during one episode (~3 s @ 5 Hz)
FLIGHT_POST_HOLD_S = 0.75  # keep capturing this long after the trigger clears before emitting once
FLIGHT_MAX_EVENT_S = 5.0   # defensive ceiling: force-emit (then wait for a clean clear) if a single
                            # episode somehow runs this long, so an emit can never be starved forever
FLIGHT_ANG_ERR_TRIG_DEG = 8.0   # Proposal 1's own suggested angErr trigger threshold
FLIGHT_LANE_CONF_FLOOR = 0.5    # below this, a lane-offset reading is FLAGGED low-confidence, not trusted
FLIGHT_HALF_VEHICLE_W = 1.0     # m, conservative telemetry-only half-width for the tire-margin estimate
# I1 review fix: a driver steeringPressed within this window is "recent" -- mirrors the
# not-recently-steer-pressed guard selfdrived's own audible steerSaturated alert requires
# (selfdrived.py ~421-429) before the trigger below is allowed to fire/emit.
FLIGHT_OVERRIDE_RECENT_S = 1.0


def _ach_lat_ms2(k_actl, v_ego):
  """steerpower2pnw: PURE OBSERVATION helper -- delivered lateral accel THIS TICK = k_actl * v_ego**2
  (signed, m/s^2), where k_actl is the yaw-rate-derived ACHIEVED curvature (not the commanded/
  kinematic one -- see the kActl derivation comment in state_control() below). This is the truck's
  true hands-off steering capability signal: peak |achLat| while latActive and saturating (under-
  turning), grouped by heading, gives the max lateral accel openpilot can actually deliver at that
  compass direction/speed. Used at both call sites (the 100 Hz peak accumulator and the throttled
  per-sample "achLat" field) so the formula/guards live in exactly one place.
  No I/O, never raises: None or non-finite k_actl/v_ego degrades to None."""
  try:
    if k_actl is None or v_ego is None:
      return None
    ach = float(k_actl) * float(v_ego) ** 2
    return ach if math.isfinite(ach) else None
  except (TypeError, ValueError):
    return None


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

    # steerevent2pnw: edge-triggered flight-recorder state (Proposal 1). Fixed-size ring buffer of
    # cheap already-computed steer/lane samples plus a tiny state machine (idle/armed/cooldown) that
    # detects the saturation/under-turn edge, holds capture open briefly after it clears, and emits
    # ONE SteerEvent mem-param burst per episode -- see the append site in state_control() below for
    # the full design writeup. `deque(maxlen=...)` bounds memory unconditionally; nothing here is
    # read back into control.
    self._flight_ring: deque = deque(maxlen=FLIGHT_PRE_N)
    self._flight_state = "idle"       # "idle" | "armed" | "cooldown"
    self._flight_start_t = 0.0
    self._flight_last_trig_t = 0.0
    self._flight_pre: list = []
    self._flight_post: list = []
    self._flight_peak_ang_err = 0.0
    self._flight_peak_lane_off = None
    self._flight_min_margin = None
    self._flight_min_conf = None
    self._flight_event_id = 0
    # I2 review fix: evId used to restart at 0 on every controlsd start, so a respawned instance's
    # first event (evId=1) collided with (and got dropped by) ces's dedup if a pre-crash instance had
    # already logged evId=1. Salt each emitted id with a per-process stamp derived from this process's
    # own monotonic-clock origin (effectively unique across restarts, since monotonic counts from
    # boot, not per-process) -- evId is emitted as the STRING f"{salt}-{n}", not a bare int.
    self._flight_pid_salt = format(int(time.monotonic() * 1000) & 0xFFFFFF, "x")
    # I1 review fix: recent-driver-override tracker, mirroring selfdrived's own guard on the audible
    # steerSaturated alert -- without it, manual/disengaged curvy driving (latActive False, model
    # still outputs desiredCurvature) and active driver overrides (latActive True, human pushing
    # through -> angErr>8 deg) both spuriously fire this "openpilot couldn't steer" event.
    self._flight_last_steer_pressed_t = -1e9   # monotonic stamp of the last observed CS.steeringPressed
    self._flight_had_override = False          # latched True if steeringPressed anywhere in this episode
    # I4 review fix: 100 Hz accumulator (updated every control tick, OUTSIDE the ~5 Hz sample gate
    # below) so the emitted peakAngErr/satAny reflect the true peak within the ~190 ms gaps between
    # throttled samples, not just the aliased value at the sample instant. Folded into the 5 Hz sample
    # and reset each time one is taken -- see the accumulator + fold-in sites in state_control().
    self._flight_peak_ang_err_acc = 0.0
    self._flight_sat_any_acc = False
    # steerpower2pnw: PURE OBSERVATION -- parallel 100 Hz running-max accumulator + episode peak for
    # achLat (delivered lateral accel, m/s^2), same accumulate/fold/reset/latch pattern as
    # _flight_peak_ang_err_acc/_flight_peak_ang_err directly above. See _ach_lat_ms2() below for the
    # formula and the two call sites (100 Hz accumulator + throttled per-sample field).
    self._flight_peak_achlat_acc = 0.0
    self._flight_peak_achlat = 0.0

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

    # steerevent2pnw I4 review fix: PURE OBSERVATION 100 Hz peak accumulator, deliberately OUTSIDE the
    # ~5 Hz `% 20` gate below (steerlimit-log2pnw / the flight-recorder sample both only run every
    # 20th tick). The flight-recorder's severity fields used to be sampled only at that throttled
    # cadence, aliasing the TRUE peak that can occur in the ~190 ms gaps between samples -- exactly
    # the number the emitted event exists to report, and a saturation blip briefer than one throttle
    # period could be missed by the trigger's own sampling entirely. Reads only values already
    # finalized this tick (lac_log, CS.steeringAngleDeg, curvature_limited) and updates two local
    # running-max/OR-latch scalars -- no dict allocation, no I/O, a handful of float ops/tick. Folded
    # into the throttled sample (and reset) at the gated block below.
    try:
      ang_des_raw = getattr(lac_log, "steeringAngleDesiredDeg", None)
      if ang_des_raw is not None:
        ang_err_raw = abs(float(ang_des_raw) - float(CS.steeringAngleDeg))
        if ang_err_raw > self._flight_peak_ang_err_acc:
          self._flight_peak_ang_err_acc = ang_err_raw
        if ang_err_raw > STEER_ANGLE_SATURATION_THRESHOLD:
          self._flight_sat_any_acc = True
      if getattr(lac_log, "saturated", False) or curvature_limited:
        self._flight_sat_any_acc = True
    except Exception:
      pass

    # steerpower2pnw: PURE OBSERVATION 100 Hz peak |achLat| accumulator, same rationale/placement as
    # the angErr accumulator directly above (a throttled ~5 Hz sample would alias the true peak). k_actl
    # is recomputed here from CS.yawRate/CS.vEgo -- the SAME formula the throttled kActl field below
    # derives (see the "kActl derivation" comment further down) -- duplicated only because those two
    # CarState fields are the only inputs available every tick; the throttled steer_limit_status dict
    # that also carries kActl isn't built until the %20==0 block below. Own try/except: a bad tick here
    # must never affect the angErr accumulator above or anything else in this control loop.
    try:
      k_actl_now = float(CS.yawRate) / max(CS.vEgo, MIN_SPEED)
      ach_now = _ach_lat_ms2(k_actl_now, CS.vEgo)
      if ach_now is not None and abs(ach_now) > self._flight_peak_achlat_acc:
        self._flight_peak_achlat_acc = abs(ach_now)
    except Exception:
      pass

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
        # the live SPEED-SCHEDULED cap this tick (lataccel2pnw's lat_accel_limit(), not the fixed ISO
        # constant), which also varies with road roll. Must track what clip_curvature actually applies
        # or this telemetry silently drifts from the real steer-limit envelope.
        lat_accel_max = lat_accel_limit(CS.vEgo) + lp.roll * ACCELERATION_DUE_TO_GRAVITY
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

        # steerevent2pnw: PURE OBSERVATION flight-recorder burst (LANE-DEPARTURE-LOGGING-PROPOSALS.md
        # Proposal 1, combined with #2's near-field lane offset and #3's rlog pointer). Everything
        # below only READS values already finalized above (steer_limit_status, model_v2, CS,
        # self.curvature, plus the I4 100 Hz accumulator populated just above this gated block) and
        # appends to a bounded deque / rarely publishes a mem-param -- it never writes into
        # new_desired_curvature, self.desired_curvature, actuators, or any other control state. The
        # SAMPLE (ring/pre/post trace entries) is built at THIS already-throttled ~5 Hz cadence (the
        # same `% 20 == 0` gate as the SteerLimitStatus publish just above) because steer_limit_status's
        # own CONTENT only refreshes at this rate -- appending faster would just re-append duplicate
        # trace rows for no benefit. The PEAK fields folded into each sample (angErrPk/satAny) are NOT
        # subject to this aliasing: they come from the true 100 Hz accumulator above, reset each time a
        # sample is taken here (see I4 review fix). Steady-state cost here is one deque append + ~2
        # array reads (near-field lane offset) + a handful of float compares; the only mem-param WRITE
        # is the rare edge emit, a handful of times per drive at most.
        try:
          # --- Proposal 2: cheap near-field lane offset from the ALREADY-DECODED model_v2 (same
          # laneLines[1]/[2] near-field convention lane_centering.py's own gate uses -- no new model
          # decode, ~4 array-index reads + 2 arithmetic ops). Any missing/malformed field degrades to
          # None rather than raising; a low-confidence reading is FLAGGED (laneLowConf below), never
          # silently trusted (the 15:14 lesson: probs collapsed to 0.05-0.3 through the curve apex).
          lane_off = lane_margin = lane_p1 = lane_p2 = None
          try:
            lines = model_v2.laneLines
            probs = model_v2.laneLineProbs
            if len(lines) >= 3 and len(probs) >= 3:
              y1 = float(lines[1].y[0])
              y2 = float(lines[2].y[0])
              p1, p2 = float(probs[1]), float(probs[2])
              if math.isfinite(y1) and math.isfinite(y2) and math.isfinite(p1) and math.isfinite(p2):
                lane_p1, lane_p2 = p1, p2
                lane_off = -(y1 + y2) / 2.0
                lane_margin = (y2 - y1) / 2.0 - FLIGHT_HALF_VEHICLE_W - abs(lane_off)
          except Exception:
            lane_off = lane_margin = lane_p1 = lane_p2 = None
          lane_conf = min(lane_p1, lane_p2) if (lane_p1 is not None and lane_p2 is not None) else None

          now_mono = time.monotonic()

          # I1 review fix: track "was the driver recently overriding steering" from CS.steeringPressed
          # (already sampled every tick by card/carstate -- no new read). Used below to suppress the
          # trigger during/just-after an override, mirroring the not-recently-steer-pressed guard
          # selfdrived's own audible steerSaturated alert requires.
          if CS.steeringPressed:
            self._flight_last_steer_pressed_t = now_mono
          recent_steer_pressed = (now_mono - self._flight_last_steer_pressed_t) < FLIGHT_OVERRIDE_RECENT_S

          # I4 review fix: fold the 100 Hz accumulator (populated above, already includes this tick's
          # own values) into this sample, then reset it for the next ~190 ms window. angErrPk floors
          # at this sample's own angErr so a steerControlType without steeringAngleDesiredDeg (the
          # accumulator no-ops in that case -- see its own comment above) never regresses below the
          # old 5 Hz-only behavior.
          self._flight_peak_ang_err_acc = max(self._flight_peak_ang_err_acc,
                                               abs(steer_limit_status["angErr"] or 0.0))
          self._flight_sat_any_acc = bool(self._flight_sat_any_acc or steer_limit_status["sat"]
                                           or steer_limit_status["angSat"] or steer_limit_status["curvLim"])
          ang_err_pk = round(self._flight_peak_ang_err_acc, 2)
          sat_any = bool(self._flight_sat_any_acc)
          self._flight_peak_ang_err_acc = 0.0
          self._flight_sat_any_acc = False

          # steerpower2pnw: same fold-then-reset as ang_err_pk directly above, for the achLat 100 Hz
          # accumulator populated in the standalone try block near the top of this method.
          ach_lat_pk = round(self._flight_peak_achlat_acc, 3)
          self._flight_peak_achlat_acc = 0.0
          # Per-sample instant achLat (this throttled tick's own kActl/vEgo) -- alongside, not instead
          # of, ach_lat_pk: the PEAK fields (ach_lat_pk / peakAchLat below) capture the true between-
          # sample max, this is just the trace's own point-in-time reading like angDes/angAct/kActl.
          ach_lat_now = _ach_lat_ms2(steer_limit_status["kActl"], CS.vEgo)

          sample = {
            "tm": round(now_mono, 3),   # N2: this is an absolute time.monotonic() stamp, not a delta
            "angDes": steer_limit_status["angDes"], "angAct": steer_limit_status["angAct"],
            "angErr": steer_limit_status["angErr"],
            "angErrPk": ang_err_pk,   # I4: true between-sample peak |angErr|, not just this instant's
            "satAny": sat_any,       # I4: true between-sample OR of sat/angSat/curvLim
            "kCmd": steer_limit_status["kCmd"], "kActl": steer_limit_status["kActl"],
            "kErr": steer_limit_status["kErr"],
            "latDem": steer_limit_status["latDem"], "latMax": steer_limit_status["latMax"],
            "latAct": steer_limit_status["latActive"],
            "sat": steer_limit_status["sat"], "angSat": steer_limit_status["angSat"],
            "curvLim": steer_limit_status["curvLim"],
            "vEgo": round(float(CS.vEgo), 2),
            "laneOff": round(lane_off, 3) if lane_off is not None else None,
            "laneMargin": round(lane_margin, 3) if lane_margin is not None else None,
            "laneP1": round(lane_p1, 3) if lane_p1 is not None else None,
            "laneP2": round(lane_p2, 3) if lane_p2 is not None else None,
            # steerpower2pnw: delivered lateral accel this tick (m/s^2, signed) -- the truck's true
            # hands-off steering-capability signal (see _ach_lat_ms2 docstring). Logging only.
            "achLat": round(ach_lat_now, 3) if ach_lat_now is not None else None,
          }
          # Always-on rolling history -- O(1) append, bounded by maxlen regardless of state.
          self._flight_ring.append(sample)

          # --- Proposal 4's trigger, folded into #1: OR of the already-computed saturation signals,
          # PLUS the exact "undershooting AND turning" triad selfdrived's own steerSaturated alert
          # gates on (selfdrived.py ~423-426) -- recomputed here from controlsd's OWN already-known
          # values (self.curvature, model_v2.action.desiredCurvature; no cross-process read).
          #
          # I1 review fix: the raw OR above used to fire on manual/disengaged curvy driving (latActive
          # False -- the model still outputs desiredCurvature even though lac isn't controlling, so
          # undershoot_turn/angErr are meaningless there) and on active driver overrides (latActive
          # True, human pushing through -> angErr blows past 8 deg even though openpilot isn't failing
          # to steer). This does NOT literally mirror selfdrived's alert gate (that lives in a
          # different process and isn't reachable from here); it approximates the same two guards with
          # values already known in controlsd:
          #   1. latActive gates the undershoot_turn/angErr branches -- they only mean "openpilot
          #      wanted more curvature than it got" when lac is actually active.
          #   2. `not recent_steer_pressed` gates the WHOLE trigger (sat/angSat included) -- none of
          #      these signals should fire an "openpilot couldn't steer" event during/just after a
          #      driver override. Any episode that DID have an override somewhere in its window is
          #      still tagged `driverOverride: true` in the emitted event (see below) so offline
          #      analysis can tell a genuine departure from an override-adjacent one without losing it.
          clipped_v = max(CS.vEgo, 0.3)
          actual_lat_accel = self.curvature * clipped_v ** 2
          desired_lat_accel = model_v2.action.desiredCurvature * clipped_v ** 2
          undershoot_turn = (abs(desired_lat_accel) / (1e-3 + abs(actual_lat_accel)) > 1.2
                              and abs(desired_lat_accel) > 1.0)
          lat_active = bool(steer_limit_status["latActive"])
          trig_departure = lat_active and (abs(steer_limit_status["angErr"]) > FLIGHT_ANG_ERR_TRIG_DEG
                                            or undershoot_turn)
          trig_core = bool(steer_limit_status["sat"] or steer_limit_status["angSat"] or trig_departure)
          trig = bool(trig_core and not recent_steer_pressed)

          # --- Edge-triggered state machine: idle -> armed (rising edge) -> cooldown (after emit,
          # until a clean clear) -> idle. Debounced: a sustained episode (even with brief flicker,
          # since `armed` only exits on a real POST_HOLD_S gap or the MAX_EVENT_S safety cap) emits
          # exactly once; a genuinely separate later episode (idle reached again first) emits again.
          if self._flight_state == "idle":
            if trig:
              self._flight_state = "armed"
              self._flight_start_t = now_mono
              self._flight_last_trig_t = now_mono
              self._flight_pre = list(self._flight_ring)   # snapshot: the pre-edge trace, bounded
              self._flight_post = []
              self._flight_peak_ang_err = ang_err_pk
              self._flight_peak_lane_off = lane_off
              self._flight_min_margin = lane_margin
              self._flight_min_conf = lane_conf
              self._flight_peak_achlat = ach_lat_pk   # steerpower2pnw: same latch-at-onset as peak_ang_err
              # I1: whether an override was already in progress right at the trigger onset (the
              # pre-edge ring can still hold a genuine pre-override departure, so this only tags,
              # never suppresses retroactively).
              self._flight_had_override = bool(CS.steeringPressed) or recent_steer_pressed
          elif self._flight_state == "armed":
            if trig:
              self._flight_last_trig_t = now_mono
            if CS.steeringPressed:
              self._flight_had_override = True
            if len(self._flight_post) < FLIGHT_POST_MAX:
              self._flight_post.append(sample)
            self._flight_peak_ang_err = max(self._flight_peak_ang_err, ang_err_pk)
            self._flight_peak_achlat = max(self._flight_peak_achlat, ach_lat_pk)  # steerpower2pnw
            if lane_off is not None and (self._flight_peak_lane_off is None
                                          or abs(lane_off) > abs(self._flight_peak_lane_off)):
              self._flight_peak_lane_off = lane_off
            if lane_margin is not None:
              self._flight_min_margin = (lane_margin if self._flight_min_margin is None
                                          else min(self._flight_min_margin, lane_margin))
            if lane_conf is not None:
              self._flight_min_conf = (lane_conf if self._flight_min_conf is None
                                        else min(self._flight_min_conf, lane_conf))

            hold_elapsed = now_mono - self._flight_last_trig_t
            total_elapsed = now_mono - self._flight_start_t
            capped = total_elapsed >= FLIGHT_MAX_EVENT_S   # N5: force-emitted by the defensive ceiling
            if (not trig and hold_elapsed >= FLIGHT_POST_HOLD_S) or capped:
              now_wall = time.time()  # noqa: TID251 -- wall clock + rlog pointer, rare edge emit only
              low_conf = self._flight_min_conf is None or self._flight_min_conf < FLIGHT_LANE_CONF_FLOOR
              # I2 review fix: evId used to restart at 0 every controlsd start, so a respawned
              # instance's first event collided with (and was dropped by) ces's dedup if a pre-crash
              # instance had already logged the same small int. Build the candidate id from the
              # per-process salt (see __init__) + the NEXT sequence number -- N1 review fix: the
              # sequence number itself is only committed (self._flight_event_id incremented) after a
              # successful put_nonblocking below, so a failed publish can't burn/skip an id.
              candidate_ev_id = f"{self._flight_pid_salt}-{self._flight_event_id + 1}"
              event = {
                "evId": candidate_ev_id,
                "t": round(now_wall, 1),
                "durationS": round(total_elapsed, 2),
                "capped": bool(capped),   # N5: durationS is a known-truncated lower bound when True
                "driverOverride": bool(self._flight_had_override),   # I1: tag, never silently drop
                "peakAngErr": round(self._flight_peak_ang_err, 2),
                # steerpower2pnw: THE capability number -- the 100 Hz peak |achLat| (m/s^2) over this
                # episode, same accumulate/fold/latch pattern as peakAngErr above. Meaningful for
                # offline direction-of-travel capability analysis when driverOverride is False (see
                # _ach_lat_ms2 docstring / ces_pnw.py's steer/steerEvent records for the heading pairing).
                "peakAchLat": round(self._flight_peak_achlat, 3),
                "peakLaneOff": (round(self._flight_peak_lane_off, 3)
                                if self._flight_peak_lane_off is not None else None),
                "minLaneMargin": (round(self._flight_min_margin, 3)
                                  if self._flight_min_margin is not None else None),
                "minLaneConf": (round(self._flight_min_conf, 3)
                                if self._flight_min_conf is not None else None),
                "laneLowConf": bool(low_conf),   # never silently trust a low-confidence excursion
                # Proposal 3: rlog pointer -- frameId + the modelV2 logMonoTime anchor, the same
                # anchor the 2026-08-11 drive report recovered manually from the `clocks` message.
                # Route/segment id is NOT included: it is owned by loggerd, not exposed to controlsd
                # via any subscribed message or Params key, so resolving it here would mean adding a
                # new filesystem/IPC dependency to a control-adjacent process for a rare-edge nicety
                # -- out of scope for a pure-observation logger. An offline tool correlates wall time
                # `t` (+ frameId/logMonoTime) to the route, exactly as Proposal 3 describes.
                # getattr's default only applies when the attribute is ABSENT -- a present-but-None
                # frameId/logMonoTime (malformed modelV2 / stale SubMaster entry) must still degrade
                # to 0 rather than raising TypeError out of int(None) and silently swallowing this
                # entire rare-edge emit (including the mem-param publish) via the except below.
                "frameId": int(getattr(model_v2, "frameId", None) or 0),
                "modelLogMonoTime": int(self.sm.logMonoTime.get('modelV2') or 0),
                # Bounded trace: FLIGHT_PRE_N + FLIGHT_POST_MAX <= 27 samples, ~19 small fields each
                # -- a few KB at most, nowhere near "dump megabytes".
                "trace": self._flight_pre + self._flight_post,
              }
              try:
                self._mem_params.put_nonblocking("SteerEvent", event)
              except Exception:
                pass   # N1: leave _flight_event_id uncommitted + stay armed -- retried next tick
              else:
                self._flight_event_id += 1
                self._flight_state = "cooldown"
          elif self._flight_state == "cooldown":
            if not trig:
              self._flight_state = "idle"
        except Exception:
          pass
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
