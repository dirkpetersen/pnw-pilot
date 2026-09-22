#!/usr/bin/env python3
import math
import os
import time
import threading

import cereal.messaging as messaging

from cereal import car, log
from msgq.visionipc import VisionIpcClient, VisionStreamType


from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, Priority, Ratekeeper, DT_CTRL
from openpilot.common.swaglog import cloudlog
from openpilot.common.gps import get_gps_location_service

from openpilot.selfdrive.car.car_specific import CarSpecificEvents
from openpilot.selfdrive.locationd.helpers import PoseCalibrator, Pose
from openpilot.selfdrive.selfdrived.events import Events, ET, EVENT_NAME, EventNamePnw  # EVENT_NAME: takecontrol2pnw; EventNamePnw: capnpfork2pnw
from openpilot.selfdrive.selfdrived.helpers import ExcessiveActuationCheck
from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import CESController, CESStub  # ces2xnor / stophold2pnw
from openpilot.selfdrive.controls.lib.ces_pnw.green_light import attentive_now  # dmgate2pnw: attention gate
from openpilot.selfdrive.selfdrived.state import StateMachine
# madsop2pnw: parallel lateral authority
from openpilot.selfdrive.selfdrived.mads_pnw import (MadsPnw, has_blocking_event, off_request_latches,
                                                     MADS_BRAKE_GRACE_FRAMES)
from openpilot.selfdrive.selfdrived.madsquiet_pnw import ChimeDecision, MadsQuiet, apply_chime_decision
from openpilot.selfdrive.controls.lib.madsresume_pnw import MadsResumeBrain, ResumeInputs, speed_unit_name  # madsresume2pnw
from openpilot.selfdrive.controls.lib.pnw_vehicle import PnwVehicle  # madsresume2pnw: capability view
from openpilot.selfdrive.selfdrived.alertmanager import AlertManager, set_offroad_alert

from openpilot.system.version import get_build_metadata
from openpilot.system.hardware import HARDWARE

REPLAY = "REPLAY" in os.environ
SIMULATION = "SIMULATION" in os.environ
TESTING_CLOSET = "TESTING_CLOSET" in os.environ

# takecontrol2pnw: "Take Control" (steerSaturated) alert flight-recorder debounce — an episode stays
# open (no new ces_events record) through any gap in the alert shorter than this; only a continuous
# absence at least this long closes it and re-arms the next rising edge for a new record.
STEER_SATURATED_HOLDOFF_S = 1.0
# onebutton2pnw: how long an ACC ON/OFF press keeps openpilot out after the driver asked for
# everything off. MEASURED 2026-09-07: the truck answers that press by engaging ~0.3 s later in
# about half of the observed cases, so the hold only has to outlast that response and the cancel it
# provokes. Deliberately short -- it is a refusal to engage, and a long one would feel like the
# system had died rather than been switched off.
OFF_REQUEST_HOLD_S = 3.0

# locdebounce2pnw: consecutive livePose frames (20 Hz -> 3 = 150 ms) that must report inputsOK False
# before locationdTemporaryError is raised. Measured 2026-08-20 over 522 segment-minutes: all 8
# occurrences were exactly ONE frame long, and each one painted a red "TAKE CONTROL IMMEDIATELY" +
# softDisabling on an engaged system that recovered on the very next frame. Root cause is upstream
# (locationd.py): the gyro/cameraOdometry yaw-rate cross-check gate is 30 * camodo rotStd[2], which
# floors at ~0.029 rad/s when vision is confident, while this device's residual reaches p99 = 0.031 —
# so a confident camera makes the gate a hair-trigger. Violations are exclusively low-speed (<15 m/s:
# 0.169%, highway: 0.000%). This debounce does NOT relax locationd's math or its detection
# sensitivity; a genuine locationd fault persists for seconds and still alerts within 150 ms. A total
# livePose failure is unaffected — it is caught by the commIssue alive/valid checks above (livePose is
# not in the SubMaster `ignore` list), which do not route through this counter.
LOCATIOND_INVALID_FRAMES = 3

LONGITUDINAL_PERSONALITY_MAP = {v: k for k, v in log.LongitudinalPersonality.schema.enumerants.items()}

ThermalStatus = log.DeviceState.ThermalStatus
State = log.SelfdriveState.OpenpilotState
PandaType = log.PandaState.PandaType
LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection
EventName = log.OnroadEvent.EventName
ButtonType = car.CarState.ButtonEvent.Type
SafetyModel = car.CarParams.SafetyModel

IGNORED_SAFETY_MODES = (SafetyModel.silent, SafetyModel.noOutput)


class SelfdriveD:
  def __init__(self, CP=None):
    self.params = Params()

    # Ensure the current branch is cached, otherwise the first cycle lags
    build_metadata = get_build_metadata()

    if CP is None:
      cloudlog.info("selfdrived is waiting for CarParams")
      self.CP = messaging.log_from_bytes(self.params.get("CarParams", block=True), car.CarParams)
      cloudlog.info("selfdrived got CarParams")
    else:
      self.CP = CP

    self.car_events = CarSpecificEvents(self.CP)

    self.pose_calibrator = PoseCalibrator()
    self.calibrated_pose: Pose | None = None
    self.excessive_actuation_check = ExcessiveActuationCheck()
    self.excessive_actuation = self.params.get("Offroad_ExcessiveActuation") is not None

    # Setup sockets
    self.pm = messaging.PubMaster(['selfdriveState', 'onroadEvents', 'madsState', 'onroadEventsPnw'])  # madsop2pnw, capnpfork2pnw

    self.gps_location_service = get_gps_location_service(self.params)
    self.gps_packets = [self.gps_location_service]
    self.sensor_packets = ["accelerometer", "gyroscope"]
    self.camera_packets = ["roadCameraState", "driverCameraState", "wideRoadCameraState"]

    # TODO: de-couple selfdrived with card/conflate on carState without introducing controls mismatches
    self.car_state_sock = messaging.sub_sock('carState', timeout=20)

    ignore = self.sensor_packets + self.gps_packets + ['alertDebug']
    if SIMULATION:
      ignore += ['driverCameraState', 'managerState']
    if REPLAY:
      # no vipc in replay will make them ignored anyways
      ignore += ['roadCameraState', 'wideRoadCameraState']
    self.sm = messaging.SubMaster(['deviceState', 'pandaStates', 'peripheralState', 'modelV2', 'liveCalibration',
                                   'carOutput', 'driverMonitoringState', 'longitudinalPlan', 'livePose', 'liveDelay',
                                   'managerState', 'liveParameters', 'radarState', 'liveTorqueParameters',
                                   'controlsState', 'carControl', 'driverAssistance', 'alertDebug', 'userBookmark', 'audioFeedback'] + \
                                   self.camera_packets + self.sensor_packets + self.gps_packets,
                                  ignore_alive=ignore, ignore_avg_freq=ignore,
                                  ignore_valid=ignore, frequency=int(1/DT_CTRL))

    # read params
    self.is_metric = self.params.get_bool("IsMetric")
    self.is_ldw_enabled = self.params.get_bool("IsLdwEnabled")
    self.disengage_on_accelerator = self.params.get_bool("DisengageOnAccelerator")

    car_recognized = self.CP.brand != 'mock'

    # cleanup old params — REAL fingerprints only.
    # fpcache2pnw (2026-07-11 dashcam incident): a MOCK session is a flaky / no-car fingerprint on
    # this shared two-car device, not evidence the car lacks these capabilities. MOCK has
    # alphaLongitudinalAvailable=False and openpilotLongitudinalControl=False, so without this guard
    # a single MOCK session silently erased the driver's AlphaLongitudinalEnabled (the op-long vs
    # ICBM A/B switch) and ExperimentalMode preferences.
    if car_recognized:
      if not self.CP.alphaLongitudinalAvailable:
        self.params.remove("AlphaLongitudinalEnabled")
      if not self.CP.openpilotLongitudinalControl:
        self.params.remove("ExperimentalMode")

    self.CS_prev = car.CarState.new_message()
    self.AM = AlertManager()
    self.events = Events()

    # madsop2pnw: the PARALLEL lateral authority. Constructed from CarParams.alternativeExperience
    # -- the exact bitfield card.py handed the panda -- and NOT from params, so openpilot can never
    # hold an opinion the panda does not share. With PandaMadsSafety=0 (the shipping default, and
    # every non-Lightning car) that bitfield is 0, so this is inert and every consumer falls back to
    # selfdriveState. It runs AFTER our own state machine and never removes an event from it.
    self.mads = MadsPnw(self.CP.alternativeExperience)
    # madsquiet2pnw: which engagement chimes to silence while MADS keeps steering.
    self.mads_quiet = MadsQuiet(MADS_BRAKE_GRACE_FRAMES)
    self._chime = ChimeDecision()

    # madsresume2pnw: the bounded auto-resume brain. Pure + inert by construction -- it refuses to
    # do anything unless madsState.available is true, and it can only ARM on the rising edge of
    # lateral_only -- which mads_pnw can only produce when "Disengage on brake" is OFF (one
    # control now governs both halves, onetoggle2pnw). So on the Tesla and on any stock-panda
    # build it is a handful of boolean tests per tick and nothing else. Construction is wrapped for the same reason CESController's is:
    # selfdrived is safety-critical and must never die for a telemetry/comfort feature.
    self.mads_resume = None
    self.mads_resume_mem = None
    self.mads_resume_offered = False        # is an offer currently published on /dev/shm?
    self.mads_resume_pub_t = 0.0
    self.mads_resume_fail = 0               # consecutive _mads_resume_step failures (loud, not silent)
    # onebutton2pnw: monotonic time of the last ACC ON/OFF press made while openpilot was NOT fully
    # engaged (i.e. the driver asking for everything off out of the steering-only state).
    self.off_request_t = 0.0
    try:
      # Fable S2: gate on the SAME capability the executor gates on. `mads.available` alone is not
      # enough -- PnwVehicle.mads_resume additionally requires button_management (stock-ACC buttons
      # AND no op-long). With Alpha Longitudinal enabled the executor is structurally inert, so
      # without this the brain would fire every brake event into a mem-param nobody reads and log a
      # `noCruise` every time: a feature that cannot do its job, failing quietly. Capability view,
      # never a fingerprint test (driver directive).
      veh = PnwVehicle(self.CP)
      if not veh.mads_resume:
        self.mads_resume = None
      else:
        self.mads_resume = MadsResumeBrain()
        self.mads_resume_mem = Params("/dev/shm/params")
    except Exception:
      cloudlog.exception("madsresume2pnw: construction FAILED -> auto-resume permanently inert")
      self.mads_resume = None

    self.initialized = False
    self.enabled = False
    self.active = False
    self.mismatch_counter = 0
    self.cruise_mismatch_counter = 0
    self.lateral_mismatch_counter = 0  # madsheartbeat2pnw
    self.last_steering_pressed_frame = 0
    # takecontrol2pnw: edge-triggered "Take Control" (steerSaturated) alert flight-recorder state —
    # see the steerSaturated block in update_events() and its use at the ces_pnw call site in step().
    # None (not 0 or -1): sm.frame legitimately starts at 0 (and could in principle be some other
    # value pre-sm.update, e.g. -1), so any int sentinel risks a false `sm.frame ==
    # last_steer_saturated_frame` match -> a bogus "Take Control" log record on boot before any real
    # alert ever fired. None can never equal an int, so it's sentinel-proof regardless of frame value.
    self.last_steer_saturated_frame = None  # last frame steerSaturated was added to self.events
    self.steer_saturated_open = False       # is a logging "episode" currently open (already logged)?
    self.steer_saturated_start_frame = 0    # frame the currently-open episode's rising edge fired
    self.distance_traveled = 0
    self.last_functional_fan_frame = 0
    self.events_prev = []
    self.logged_comm_issue = None
    self.locationd_invalid_frames = 0  # locdebounce2pnw: consecutive livePose frames with inputsOK False
    self.not_running_prev = None
    self.experimental_mode = False
    self.manual_experimental_mode = False     # ces2xnor: the ExperimentalMode-param baseline
    # stophold2pnw (C): construction was UNWRAPPED — a CESController.__init__ raise took down
    # selfdrived (safety-critical) entirely. Wrap it: any failure -> loud log + inert stub
    # (experimental_request() always False == stock behavior). The per-cycle call is already
    # wrapped further down; this closes the remaining constructor gap.
    try:
      self.ces_pnw = CESController(self.CP)   # ces2xnor: default OFF
    except Exception:
      cloudlog.exception("ces_pnw: CESController construction FAILED -> CES inert (stock behavior)")
      self.ces_pnw = CESStub()
    self.personality = self.params.get("LongitudinalPersonality", return_default=True)
    self.recalibrating_seen = False
    self.state_machine = StateMachine()
    self.rk = Ratekeeper(100, print_delay_threshold=None)

    # Determine startup event
    self.startup_event = EventName.startup if build_metadata.openpilot.comma_remote and build_metadata.tested_channel else EventName.startupMaster
    if HARDWARE.get_device_type() == 'mici':
      self.startup_event = None
    if not car_recognized:
      self.startup_event = EventName.startupNoCar
    elif car_recognized and self.CP.passive:
      self.startup_event = EventName.startupNoControl
    elif self.CP.secOcRequired and not self.CP.secOcKeyAvailable:
      self.startup_event = EventName.startupNoSecOcKey

    if not car_recognized:
      self.events.add(EventName.carUnrecognized, static=True)
      set_offroad_alert("Offroad_CarUnrecognized", True)
    elif self.CP.passive:
      self.events.add(EventName.dashcamMode, static=True)

  def update_events(self, CS):
    """Compute onroadEvents from carState"""

    self.events.clear()

    if self.sm['controlsState'].lateralControlState.which() == 'debugState':
      self.events.add(EventName.joystickDebug)
      self.startup_event = None

    if self.sm.recv_frame['alertDebug'] > 0:
      self.events.add(EventName.longitudinalManeuver)
      self.startup_event = None

    # Add startup event
    if self.startup_event is not None:
      self.events.add(self.startup_event)
      self.startup_event = None

    # Don't add any more events if not initialized
    if not self.initialized:
      self.events.add(EventName.selfdriveInitializing)
      return

    # Check for user bookmark press (bookmark button or end of LKAS button feedback)
    if self.sm.updated['userBookmark']:
      self.events.add(EventName.userBookmark)

    if self.sm.updated['audioFeedback']:
      self.events.add(EventName.audioFeedback)

    # greenlight2pnw/greenlead2pnw: standstill dings — set by CESController._green_light_step last
    # cycle (always-on, works with CES off too). Display/sound only; no control path.
    # green_light = stopped with NO lead and the path opened; lead_departing = the stopped car we
    # were behind pulled away. Mutually exclusive by construction (one classification per tick);
    # a lead departing while we are already MOVING raises NEITHER (telemetry only, driver rule).
    if self.ces_pnw.green_light:
      self.events.add(EventNamePnw.greenLight)          # ALWAYS — the traffic-light nudge is never
                                                     # gated on attention (driver directive 2026-07-13)
    if self.ces_pnw.lead_departing:
      # dmgate2pnw (driver directive 2026-07-13): the LEAD-departure nudge is suppressed when driver
      # monitoring CONFIDENTLY sees an attentive driver — if you are watching, you see the car ahead
      # leave and pull away on your own, so the ding is just noise. Only ping when the driver is NOT
      # confidently attentive (looking away / on phone / face lost / DM uncertain / message missing).
      # attentive_now() defaults to False on any uncertainty, so the safe direction is to still ding.
      # isActiveMode is gated on the DM model's read quality, NOT vehicle speed, so this is valid at
      # the standstill where lead departures happen. The greenLight nudge above is unaffected.
      try:
        dm = self.sm['driverMonitoringState']
        attentive = attentive_now(dm.isActiveMode, dm.faceDetected, dm.isDistracted)
      except Exception:
        attentive = False
      if not attentive:
        self.events.add(EventNamePnw.leadDeparting)

    # Don't add any more events while in dashcam mode
    if self.CP.passive:
      return

    # onebutton2pnw: the ACC ON/OFF button, pressed while openpilot is NOT fully engaged, means
    # "everything off" -- the driver's rule, and the state they are in after a brake left MADS
    # steering with cruise in Standby. It is latched rather than acted on for one frame because the
    # truck answers that press by ENGAGING roughly half the time (measured; see
    # drives/2026-09-07/lightning-onoff-button/), ~0.3 s later. Without the latch openpilot would
    # engage with it and bring everything straight back -- which is exactly the reported complaint.
    #
    # While the latch stands, the NO_ENTRY below keeps openpilot out, and controlsd's existing
    # `CS.cruiseState.enabled and not CC.enabled` rule then sends cruiseControl.cancel -- so the
    # cancel needs no new code path of its own. Not latched when openpilot IS engaged: from Active
    # the truck's own button reaches Off cleanly and `cruiseState.available` already handles it.
    #
    # REGRESSION FIX 2026-09-07: this condition was `not self.enabled`, which is ALSO true when
    # openpilot is simply OFF. So pressing the button to turn cruise ON latched the block, openpilot
    # refused to engage, and controlsd's cancel rule then cancelled the cruise the driver had just
    # switched on -- "openpilot unavailable / cruise control turned off", every press, cruise
    # unusable. The gate is the STEERING-ONLY state, which is the only state the feature was ever
    # about; `lateral_only` names it exactly. Read from the previous frame (mads.update runs later
    # in this tick), which is correct: the steering-only state persists across frames, and using
    # this frame's value would need an ordering change for no benefit.
    #
    # onoffgas2pnw (OWNER DECISION 2026-09-15, after the drive below): the press is IGNORED while the
    # driver is on the accelerator. Evidence -- drives/2026-09-15/gassetwait-first-drive/: accelerating
    # away from a crossing at 25 mph with `steerOverride` active, the truck reported ONE mainCruise press
    # (Steering_Data_FD1 0x083 `CcButtnOnOffPress`; openpilot sent ZERO 0x083 frames in that window, so it
    # was the wheel, not our own SET spoof). That latched the off-request and MADS dropped lateral on the
    # same frame, which the driver experienced as "at the 3rd or 4th crossing it completely disengages and
    # I don't understand why". Gripping the wheel mid-acceleration is exactly where a thumb finds that
    # button, and a DELIBERATE "turn it all off" is never so urgent that it cannot wait for a lift -- while
    # the one moment the driver has said they want the system to hold on is the acceleration away from a
    # crossing. The rule itself is unchanged everywhere else.
    main_press = any(be.pressed and be.type == ButtonType.mainCruise for be in CS.buttonEvents)
    if off_request_latches(main_press, self.mads.lateral_only, CS.gasPressed):
      self.off_request_t = self.sm.frame * DT_CTRL
    elif main_press and self.mads.lateral_only:
      # Rule 2: a press that is deliberately not acted on must SAY so. Without this a driver who DID mean it
      # sees nothing happen with no way to tell a swallowed press from a missed one.
      cloudlog.warning("onoffgas2pnw: ACC ON/OFF press IGNORED, accelerator is down (v_ego=%.1f m/s) -- lift off and press again", CS.vEgo)
    # An ENGAGE press cancels the off-request outright (Gemini review 2026-09-07, finding B). The
    # driver may press OFF and change their mind a second later; without this the latch would still
    # be standing, openpilot would refuse, and controlsd's cancel rule would kill the engagement
    # they just asked for -- the same shape as the regression this feature already caused once.
    if any(be.pressed and be.type in (ButtonType.accelCruise, ButtonType.decelCruise,
                                      ButtonType.resumeCruise, ButtonType.setCruise)
           for be in CS.buttonEvents):
      self.off_request_t = 0.0
    if self.off_request_t and (self.sm.frame * DT_CTRL - self.off_request_t) <= OFF_REQUEST_HOLD_S:
      self.events.add(EventNamePnw.cruiseOffRequested)

    # Block resume if cruise never previously enabled
    resume_pressed = any(be.type in (ButtonType.accelCruise, ButtonType.resumeCruise) for be in CS.buttonEvents)
    if not self.CP.pcmCruise and CS.vCruise > 250 and resume_pressed:
      self.events.add(EventName.resumeBlocked)

    if not self.CP.notCar:
      self.events.add_from_msg(self.sm['driverMonitoringState'].events)

    # Add car events, ignore if CAN isn't valid
    if CS.canValid:
      car_events = self.car_events.update(CS, self.CS_prev, self.sm['carControl']).to_msg()
      self.events.add_from_msg(car_events)

      if self.CP.notCar:
        # wait for everything to init first
        if self.sm.frame > int(5. / DT_CTRL) and self.initialized:
          # body always wants to enable
          self.events.add(EventName.pcmEnable)

      # Disable on rising edge of accelerator or brake. Also disable on brake when speed > 0
      #
      # nobrakekey2pnw: the auto2pnw `NoDisengageOnBrake` suppression that used to sit here is GONE.
      # It cleared brake_disengage so openpilot would not raise its own disengage -- but nothing
      # stopped the PANDA clearing controls_allowed on the same brake press (generic_rx_checks(),
      # safety.h, outside the whitelist guard). That half-state fed mismatch_counter at 100 Hz and
      # tripped controlsMismatch IMMEDIATE_DISABLE after 200 ticks (2.0 s) -- a worse disengage than
      # the one it suppressed. The UI greyed the toggle, but the param was still read here, so a raw
      # write to /data/params/d/NoDisengageOnBrake armed it for real, on ANY car including the Tesla.
      #
      # Keeping lateral through a brake press is what MADS does properly: the panda carries a
      # SECOND authority (controls_allowed_lateral) that the brake does not clear, so there is no
      # mismatch to detect. See mads_pnw.py and the "Disengage on brake" toggle.
      brake_disengage = (CS.brakePressed and (not self.CS_prev.brakePressed or not CS.standstill)) or \
                        (CS.regenBraking and (not self.CS_prev.regenBraking or not CS.standstill))
      if (CS.gasPressed and not self.CS_prev.gasPressed and self.disengage_on_accelerator) or brake_disengage:
        self.events.add(EventName.pedalPressed)

    # Create events for temperature, disk space, and memory
    if self.sm['deviceState'].thermalStatus >= ThermalStatus.red:
      self.events.add(EventName.overheat)
    if self.sm['deviceState'].freeSpacePercent < 7 and not SIMULATION:
      self.events.add(EventName.outOfSpace)
    if self.sm['deviceState'].memoryUsagePercent > 90 and not SIMULATION:
      self.events.add(EventName.lowMemory)

    # Alert if fan isn't spinning for 5 seconds
    if self.sm['peripheralState'].pandaType != log.PandaState.PandaType.unknown:
      if self.sm['peripheralState'].fanSpeedRpm < 500 and self.sm['deviceState'].fanSpeedPercentDesired > 50:
        # allow enough time for the fan controller in the panda to recover from stalls
        if (self.sm.frame - self.last_functional_fan_frame) * DT_CTRL > 15.0:
          self.events.add(EventName.fanMalfunction)
      else:
        self.last_functional_fan_frame = self.sm.frame

    # Handle calibration status
    cal_status = self.sm['liveCalibration'].calStatus
    if cal_status != log.LiveCalibrationData.Status.calibrated:
      if cal_status == log.LiveCalibrationData.Status.uncalibrated:
        self.events.add(EventName.calibrationIncomplete)
      elif cal_status == log.LiveCalibrationData.Status.recalibrating:
        if not self.recalibrating_seen:
          set_offroad_alert("Offroad_Recalibration", True)
        self.recalibrating_seen = True
        self.events.add(EventName.calibrationRecalibrating)
      else:
        self.events.add(EventName.calibrationInvalid)

    # Lane departure warning
    if self.is_ldw_enabled and self.sm.valid['driverAssistance']:
      if self.sm['driverAssistance'].leftLaneDeparture or self.sm['driverAssistance'].rightLaneDeparture:
        self.events.add(EventName.ldw)

    # ******************************************************************************************
    #  NOTE: To fork maintainers.
    #  Disabling or nerfing safety features will get you and your users banned from our servers.
    #  We recommend that you do not change these numbers from the defaults.
    if self.sm.updated['liveCalibration']:
      self.pose_calibrator.feed_live_calib(self.sm['liveCalibration'])
    if self.sm.updated['livePose']:
      device_pose = Pose.from_live_pose(self.sm['livePose'])
      self.calibrated_pose = self.pose_calibrator.build_calibrated_pose(device_pose)

    if self.calibrated_pose is not None:
      excessive_actuation = self.excessive_actuation_check.update(self.sm, CS, self.calibrated_pose)
      if not self.excessive_actuation and excessive_actuation is not None:
        set_offroad_alert("Offroad_ExcessiveActuation", True, extra_text=str(excessive_actuation))
        self.excessive_actuation = True

    if self.excessive_actuation:
      self.events.add(EventName.excessiveActuation)
    # ******************************************************************************************

    # Handle lane change
    if self.sm['modelV2'].meta.laneChangeState == LaneChangeState.preLaneChange:
      direction = self.sm['modelV2'].meta.laneChangeDirection
      if (CS.leftBlindspot and direction == LaneChangeDirection.left) or \
         (CS.rightBlindspot and direction == LaneChangeDirection.right):
        self.events.add(EventName.laneChangeBlocked)
      else:
        if direction == LaneChangeDirection.left:
          self.events.add(EventName.preLaneChangeLeft)
        else:
          self.events.add(EventName.preLaneChangeRight)
    elif self.sm['modelV2'].meta.laneChangeState in (LaneChangeState.laneChangeStarting,
                                                    LaneChangeState.laneChangeFinishing):
      self.events.add(EventName.laneChange)

    for i, pandaState in enumerate(self.sm['pandaStates']):
      # All pandas must match the list of safetyConfigs, and if outside this list, must be silent or noOutput
      if i < len(self.CP.safetyConfigs):
        safety_mismatch = pandaState.safetyModel != self.CP.safetyConfigs[i].safetyModel or \
                          pandaState.safetyParam != self.CP.safetyConfigs[i].safetyParam or \
                          pandaState.alternativeExperience != self.CP.alternativeExperience
      else:
        safety_mismatch = pandaState.safetyModel not in IGNORED_SAFETY_MODES

      # safety mismatch allows some time for pandad to set the safety mode and publish it back from panda
      if (safety_mismatch and self.sm.frame*DT_CTRL > 10.) or pandaState.safetyRxChecksInvalid or self.mismatch_counter >= 200:
        self.events.add(EventName.controlsMismatch)

      if log.PandaState.FaultType.relayMalfunction in pandaState.faults:
        self.events.add(EventName.relayMalfunction)

    # madsheartbeat2pnw: the LATERAL twin of controlsMismatch above. Raised once the panda has
    # been reporting "lateral not permitted" for 2 s while MADS was still commanding lateral --
    # e.g. the panda's own heartbeat_engaged_mads watchdog revoked the latch, or an rx message
    # went invalid. Without this the failure is SILENT: the panda blocks the tx and the truck
    # simply stops steering with nothing said. Outside the loop because it is one state, not one
    # per panda. Unreachable unless MADS is available AND holding lateral alone (see data_sample).
    if self.lateral_mismatch_counter >= 200:
      self.events.add(EventNamePnw.madsControlsMismatchLateral)

    # Handle HW and system malfunctions
    # Order is very intentional here. Be careful when modifying this.
    # All events here should at least have NO_ENTRY and SOFT_DISABLE.
    num_events = len(self.events)

    # mapd2pnw: mapd / mapd_configd are display + nav helpers (OSM speed-limit / curve hints), NOT
    # safety-critical. Their absence must NEVER block engagement — e.g. the mapd binary can be missing
    # or still downloading on a slow link. Exclude them from the processNotRunning gate so openpilot
    # still drives when mapd is down (map features just go inert). Does not touch panda/control safety.
    NON_ESSENTIAL_PROCS = {"mapd", "mapd_configd", "location_servicesd"}  # location2pnw: display-only, never blocks engagement
    not_running = {p.name for p in self.sm['managerState'].processes
                   if not p.running and p.shouldBeRunning and p.name not in NON_ESSENTIAL_PROCS}
    if self.sm.recv_frame['managerState'] and len(not_running):
      if not_running != self.not_running_prev:
        cloudlog.event("process_not_running", not_running=not_running, error=True)
      self.not_running_prev = not_running
    if self.sm.recv_frame['managerState'] and not_running:
      self.events.add(EventName.processNotRunning)
    else:
      if not SIMULATION and not self.rk.lagging:
        if not self.sm.all_alive(self.camera_packets):
          self.events.add(EventName.cameraMalfunction)
        elif not self.sm.all_freq_ok(self.camera_packets):
          self.events.add(EventName.cameraFrameRate)
    if not REPLAY and self.rk.lagging:
      self.events.add(EventName.selfdrivedLagging)
    if self.CP.openpilotLongitudinalControl:
      if self.sm['radarState'].radarErrors.canError:
        self.events.add(EventName.canError)
      elif self.sm['radarState'].radarErrors.radarUnavailableTemporary:
        self.events.add(EventName.radarTempUnavailable)
      elif any(self.sm['radarState'].radarErrors.to_dict().values()):
        self.events.add(EventName.radarFault)
    if not self.sm.valid['pandaStates']:
      self.events.add(EventName.usbError)
    if CS.canTimeout:
      self.events.add(EventName.canBusMissing)
    elif not CS.canValid:
      self.events.add(EventName.canError)

    # generic catch-all. ideally, a more specific event should be added above instead
    has_disable_events = self.events.contains(ET.NO_ENTRY) and (self.events.contains(ET.SOFT_DISABLE) or self.events.contains(ET.IMMEDIATE_DISABLE))
    no_system_errors = (not has_disable_events) or (len(self.events) == num_events)
    if not self.sm.all_checks() and no_system_errors:
      if not self.sm.all_alive():
        self.events.add(EventName.commIssue)
      elif not self.sm.all_freq_ok():
        self.events.add(EventName.commIssueAvgFreq)
      else:
        self.events.add(EventName.commIssue)

      logs = {
        'invalid': [s for s, valid in self.sm.valid.items() if not valid],
        'not_alive': [s for s, alive in self.sm.alive.items() if not alive],
        'not_freq_ok': [s for s, freq_ok in self.sm.freq_ok.items() if not freq_ok],
      }
      if logs != self.logged_comm_issue:
        cloudlog.event("commIssue", error=True, **logs)
        self.logged_comm_issue = logs
    else:
      self.logged_comm_issue = None

    if not self.CP.notCar:
      if not self.sm['livePose'].posenetOK:
        self.events.add(EventName.posenetInvalid)
      # locdebounce2pnw: count on livePose UPDATES, not selfdrived frames — selfdrived runs at 100 Hz
      # and livePose at 20 Hz, so counting frames here would trip on 30 ms and defeat the debounce.
      if self.sm.updated['livePose']:
        if self.sm['livePose'].inputsOK:
          self.locationd_invalid_frames = 0
        else:
          self.locationd_invalid_frames += 1
      # raised every frame while the condition holds (self.events is rebuilt each frame), not just on
      # the livePose tick that crosses the threshold.
      if self.locationd_invalid_frames >= LOCATIOND_INVALID_FRAMES:
        self.events.add(EventName.locationdTemporaryError)
      if not self.sm['liveParameters'].valid and cal_status == log.LiveCalibrationData.Status.calibrated and not TESTING_CLOSET and (not SIMULATION or REPLAY):
        self.events.add(EventName.paramsdTemporaryError)

    # conservative HW alert. if the data or frequency are off, locationd will throw an error
    if any((self.sm.frame - self.sm.recv_frame[s])*DT_CTRL > 10. for s in self.sensor_packets):
      self.events.add(EventName.sensorDataInvalid)

    if not REPLAY:
      # Check for mismatch between openpilot and car's PCM
      cruise_mismatch = CS.cruiseState.enabled and (not self.enabled or not self.CP.pcmCruise)
      self.cruise_mismatch_counter = self.cruise_mismatch_counter + 1 if cruise_mismatch else 0
      if self.cruise_mismatch_counter > int(6. / DT_CTRL):
        self.events.add(EventName.cruiseMismatch)

    # Send a "steering required alert" if saturation count has reached the limit
    if CS.steeringPressed:
      self.last_steering_pressed_frame = self.sm.frame
    recent_steer_pressed = (self.sm.frame - self.last_steering_pressed_frame)*DT_CTRL < 2.0
    controlstate = self.sm['controlsState']
    lac = getattr(controlstate.lateralControlState, controlstate.lateralControlState.which())
    if lac.active and not recent_steer_pressed and not self.CP.notCar:
      clipped_speed = max(CS.vEgo, 0.3)
      actual_lateral_accel = controlstate.curvature * (clipped_speed**2)
      desired_lateral_accel = self.sm['modelV2'].action.desiredCurvature * (clipped_speed**2)
      undershooting = abs(desired_lateral_accel) / abs(1e-3 + actual_lateral_accel) > 1.2
      turning = abs(desired_lateral_accel) > 1.0
      # TODO: lac.saturated includes speed and other checks, should be pulled out
      if undershooting and turning and lac.saturated:
        self.events.add(EventName.steerSaturated)
        # takecontrol2pnw: PURE OBSERVATION bookkeeping only — records which frame the decision
        # above fired, for the edge-triggered ces_events logger in step() below. Never read back
        # into this decision.
        self.last_steer_saturated_frame = self.sm.frame

    # Check for FCW
    stock_long_is_braking = self.enabled and not self.CP.openpilotLongitudinalControl and CS.aEgo < -1.25
    model_fcw = self.sm['modelV2'].meta.hardBrakePredicted and not CS.brakePressed and not stock_long_is_braking
    planner_fcw = self.sm['longitudinalPlan'].fcw and self.enabled
    if (planner_fcw or model_fcw) and not self.CP.notCar:
      self.events.add(EventName.fcw)

    # GPS checks
    gps_ok = self.sm.recv_frame[self.gps_location_service] > 0 and (self.sm.frame - self.sm.recv_frame[self.gps_location_service]) * DT_CTRL < 2.0
    if not gps_ok and self.sm['livePose'].inputsOK and (self.distance_traveled > 1500):
      self.events.add(EventName.noGps)
    if gps_ok:
      self.distance_traveled = 0
    self.distance_traveled += abs(CS.vEgo) * DT_CTRL

    # TODO: fix simulator
    if not SIMULATION or REPLAY:
      # lebowski2pnw retune (2026-07-09, live drive): upstream pick 205ca5c36e tightened this 20 -> 1
      # for comma-four-class headroom; on the 3X + the deep lebowski model a transient stop-and-go
      # scheduling hiccup can spike past 1% and throw a take-control alert at the driver (observed
      # ~04:4xZ). 5% still catches real sustained lag (steady-state measured 0.00%) without hair-trigger
      # alerts on momentary spikes.
      if self.sm['modelV2'].frameDropPerc > 5:
        self.events.add(EventName.modeldLagging)

    # Decrement personality on distance button press
    if self.CP.openpilotLongitudinalControl:
      if any(not be.pressed and be.type == ButtonType.gapAdjustCruise for be in CS.buttonEvents):
        self.personality = (self.personality - 1) % 3
        self.params.put_nonblocking('LongitudinalPersonality', self.personality)
        self.events.add(EventName.personalityChanged)

  def data_sample(self):
    _car_state = messaging.recv_one(self.car_state_sock)
    CS = _car_state.carState if _car_state else self.CS_prev

    self.sm.update(0)

    if not self.initialized:
      all_valid = CS.canValid and self.sm.all_checks()
      timed_out = self.sm.frame * DT_CTRL > 6.
      if all_valid or timed_out or (SIMULATION and not REPLAY):
        available_streams = VisionIpcClient.available_streams("camerad", block=False)
        if VisionStreamType.VISION_STREAM_ROAD not in available_streams:
          self.sm.ignore_alive.append('roadCameraState')
          self.sm.ignore_valid.append('roadCameraState')
        if VisionStreamType.VISION_STREAM_WIDE_ROAD not in available_streams:
          self.sm.ignore_alive.append('wideRoadCameraState')
          self.sm.ignore_valid.append('wideRoadCameraState')

        if REPLAY and any(ps.controlsAllowed for ps in self.sm['pandaStates']):
          self.state_machine.state = State.enabled

        self.initialized = True
        cloudlog.event(
          "selfdrived.initialized",
          dt=self.sm.frame*DT_CTRL,
          timeout=timed_out,
          canValid=CS.canValid,
          invalid=[s for s, valid in self.sm.valid.items() if not valid],
          not_alive=[s for s, alive in self.sm.alive.items() if not alive],
          not_freq_ok=[s for s, freq_ok in self.sm.freq_ok.items() if not freq_ok],
          error=True,
        )

    # When the panda and selfdrived do not agree on controls_allowed
    # we want to disengage openpilot. However the status from the panda goes through
    # another socket other than the CAN messages and one can arrive earlier than the other.
    # Therefore we allow a mismatch for two samples, then we trigger the disengagement.
    if not self.enabled:
      self.mismatch_counter = 0

    # All pandas not in silent mode must have controlsAllowed when openpilot is enabled
    if self.enabled and any(not ps.controlsAllowed for ps in self.sm['pandaStates']
           if ps.safetyModel not in IGNORED_SAFETY_MODES):
      self.mismatch_counter += 1

    # madsheartbeat2pnw: the same check for the PARALLEL lateral authority. It only applies while
    # MADS holds lateral ALONE -- while openpilot itself is enabled the check above already covers
    # it, because the panda reports controlsAllowedLateral as
    # (controls_allowed || controls_allowed_lateral). The `self.enabled` term is not redundant with
    # `lateral_only`: both are one frame stale here (data_sample runs before mads.update, exactly
    # as it does before the state machine above), and without it a re-engage could carry a
    # saturated counter into an enabled frame and immediate-disable a car that is steering fine.
    #
    # A panda whose health_t LAYOUT is not this build's (healthPacketMismatch) is skipped here, and
    # ONLY here. The Tesla Raven's second (black F4) panda runs the frozen prebuilt DEV-fd39c10f:
    # its health_t is 58 bytes and inserts fan_stall_count at byte 52, where ours (panda/board/
    # health.h @ c5e431e1, 61 bytes) has none. The two layouts are identical through byte 51, so
    # byte 34 controls_allowed_pkt (and safety_mode/param, heartbeat_lost, alt_experience) is
    # trustworthy and the longitudinal check above is deliberately NOT gated. From byte 52 on the
    # parse is misaligned (sbu voltages, sound level) or never written: byte 59
    # controls_allowed_lateral_pkt and byte 60 mads_disengage_reason_pkt read 0 forever. Both
    # Raven pandas carry teslaLegacy, so IGNORED_SAFETY_MODES does not cover it, and without this
    # exclusion the counter climbs every frame from a value nobody measured and fires
    # madsControlsMismatchLateral 2 s into every lateral-only state on that car. "Unknown" is
    # neither True nor False: we do not substitute controlsAllowed for it, and a panda that DOES
    # report the field is still held to it.
    if self.enabled or not (self.mads.available and self.mads.lateral_only):
      self.lateral_mismatch_counter = 0
    elif any(not ps.controlsAllowedLateral for ps in self.sm['pandaStates']
             if ps.safetyModel not in IGNORED_SAFETY_MODES and not ps.healthPacketMismatch):
      self.lateral_mismatch_counter += 1

    return CS

  def update_alerts(self, CS):
    clear_event_types = set()
    if ET.WARNING not in self.state_machine.current_alert_types:
      clear_event_types.add(ET.WARNING)
    if self.enabled:
      clear_event_types.add(ET.NO_ENTRY)

    pers = LONGITUDINAL_PERSONALITY_MAP[self.personality]
    alerts = self.events.create_alerts(self.state_machine.current_alert_types, [self.CP, CS, self.sm, self.is_metric,
                                                                                self.state_machine.soft_disable_timer, pers])
    # madsquiet2pnw: silence engagement chimes only while MADS keeps steering (see madsquiet_pnw.py).
    try:
      alerts = apply_chime_decision(alerts, getattr(self, "_chime", ChimeDecision()))
    except Exception:
      cloudlog.exception("madsquiet: applying the chime decision failed -- stock chimes this frame")
    self.AM.add_many(self.sm.frame, alerts)
    self.AM.process_alerts(self.sm.frame, clear_event_types)

  def publish_selfdriveState(self, CS):
    # selfdriveState
    ss_msg = messaging.new_message('selfdriveState')
    ss_msg.valid = True
    ss = ss_msg.selfdriveState
    ss.enabled = self.enabled
    ss.active = self.active
    ss.state = self.state_machine.state
    ss.engageable = not self.events.contains(ET.NO_ENTRY)
    ss.experimentalMode = self.experimental_mode
    ss.personality = self.personality

    ss.alertText1 = self.AM.current_alert.alert_text_1
    ss.alertText2 = self.AM.current_alert.alert_text_2
    ss.alertSize = self.AM.current_alert.alert_size
    ss.alertStatus = self.AM.current_alert.alert_status
    ss.alertType = self.AM.current_alert.alert_type
    ss.alertSound = self.AM.current_alert.audible_alert
    ss.alertHudVisual = self.AM.current_alert.visual_alert

    # madsop2pnw: publish the lateral authority BEFORE selfdriveState. controlsd polls on
    # selfdriveState, so sending madsState first guarantees the frame's madsState is already queued
    # when controlsd wakes -- the two can never be read a frame apart in the direction that matters.
    mads_msg = messaging.new_message('madsState')
    mads_msg.valid = True
    ms = mads_msg.madsState
    ms.available = self.mads.available
    ms.enabled = self.mads.enabled
    ms.active = self.mads.active
    ms.lateralOnly = self.mads.lateral_only
    ms.disengageOnBrake = self.mads.disengage_on_brake
    self.pm.send('madsState', mads_msg)

    self.pm.send('selfdriveState', ss_msg)

    # onroadEvents - logged every second or on change
    if (self.sm.frame % int(1. / DT_CTRL) == 0) or (self.events.names != self.events_prev):
      # capnpfork2pnw: the fork's own events are not in log.capnp's EventName any more (events.py), so
      # they go out on onroadEventsPnw, in the same frame and under the same condition -- one is never
      # published without the other. FIRST, so a consumer that reacts to onroadEvents changing (card's
      # accdrop logger) already holds this frame's fork events when it does -- the same reason
      # madsState goes out before selfdriveState.
      pe_send = messaging.new_message('onroadEventsPnw')
      pe_send.valid = True
      pe_send.onroadEventsPnw.events = self.events.to_msg_pnw()
      self.pm.send('onroadEventsPnw', pe_send)

      upstream_events = self.events.to_msg()
      ce_send = messaging.new_message('onroadEvents', len(upstream_events))
      ce_send.valid = True
      ce_send.onroadEvents = upstream_events
      self.pm.send('onroadEvents', ce_send)
    self.events_prev = self.events.names.copy()

  def step(self):
    CS = self.data_sample()
    self.update_events(CS)
    if not self.CP.passive and self.initialized:
      self.enabled, self.active = self.state_machine.update(self.events)

    # madsop2pnw: run the parallel lateral authority AFTER our own state machine has already
    # decided self.enabled/self.active. Order matters and is the whole safety argument: openpilot's
    # own disengage has ALREADY happened and is never edited, so mismatch_counter (keyed on
    # self.enabled, reset above) cannot climb because of this, and controlsMismatch cannot fire.
    # MADS only answers the separate question "may openpilot still steer?".
    off_req = bool(self.off_request_t and
                   (self.sm.frame * DT_CTRL - self.off_request_t) <= OFF_REQUEST_HOLD_S)
    self.mads.update(self.enabled, self.active, CS.brakePressed or CS.regenBraking,
                     CS.cruiseState.enabled, self.events, CS.cruiseState.available, off_req)
    # madsquiet2pnw: decide the engagement chimes from THIS frame's engagement and MADS state. It only
    # ever changes a SOUND -- the state machine has already run and is never consulted or edited here.
    # Any failure falls back to the stock chimes: silence is the thing that must never happen by accident.
    try:
      self._chime = self.mads_quiet.step(self.enabled, self.mads.lateral_only, self.mads.available,
                                         self.mads.brake_grace_open)
    except Exception:
      cloudlog.exception("madsquiet: chime decision failed -- stock chimes this frame")
      self._chime = ChimeDecision()
    # madsresume2pnw: decide (never act -- the tap itself is the ford carcontroller's job) whether
    # openpilot may hand back the speed the driver had already set. Runs AFTER mads.update so it
    # sees THIS frame's lateral_only, not the previous one -- the arm edge must not be a frame late.
    self._mads_resume_step(CS)
    if self.mads.active and not self.active:
      # madsop2pnw: openpilot is STEERING while its own state machine sits in `disabled`, whose
      # current_alert_types is [ET.PERMANENT] only -- so update_alerts() below would CLEAR every
      # ET.WARNING alert. That silently swallows "Take Control" (steerSaturated, which IS still
      # raised because lac.active is true), the lane-change prompts (lane changes still execute in
      # this state), belowSteerSpeed and steerTempUnavailableSilent. Re-admit WARNING for exactly
      # the frames MADS is steering alone. The list is rebuilt from scratch at the top of every
      # StateMachine.update(), and is read only by update_alerts(), so appending here cannot leak
      # into engagement. (Fable review 2026-09-05; mirrors sunnypilot's
      # StateMachine.add_current_alert_types(ET.WARNING).)
      self.state_machine.current_alert_types.append(ET.WARNING)
    if self.mads.lateral_only:
      # ET.PERMANENT only -- no disable/no-entry type, so adding it here cannot influence the state
      # machine that already ran, and cannot change ss.engageable. It exists so the car is never
      # steering behind a UI that just says "disengaged".
      self.events.add(EventNamePnw.madsLateralOnly)

    self.update_alerts(CS)

    # ces2xnor: effective experimental = manual ExperimentalMode OR CES's per-cycle decision.
    # SAFETY (Gemini-reviewed): (1) wrap the CES core call in try/except — selfdrived is safety-critical
    # and a raise here would crash it; default to False (chill) on any error. (2) gate the whole result
    # on openpilotLongitudinalControl so it is byte-identical to the stock baseline (and never forces
    # experimental on a stock-ACC car). Default OFF + this gating = behavior-neutral regression baseline.
    try:
      ces_req = self.ces_pnw.experimental_request(CS, self.sm)
    except Exception:
      cloudlog.exception("ces_pnw: experimental_request raised -> chill")
      ces_req = False
    self.experimental_mode = self.CP.openpilotLongitudinalControl and (self.manual_experimental_mode or ces_req)

    self._log_take_control_edge(CS)

    self.publish_selfdriveState(CS)

    self.CS_prev = CS

  def _mads_resume_step(self, CS) -> None:
    """madsresume2pnw: run the auto-resume brain and publish/withdraw its offer.

    This method does ALL the I/O the brain deliberately refuses to do: the param read, the
    radarState read, the /dev/shm publish, and the ces_events append. The brain itself is pure and
    is where every gate lives (selfdrive/controls/lib/madsresume_pnw.py).

    NOTHING FAILS SILENTLY (CLAUDE.md rule 2). Three separate visibility guarantees:
      * a FAILED radarState read is passed to the brain as `has_lead=None`, which the lead gate
        treats as a REFUSAL (`leadUnknown`) -- never as "no lead ahead, go ahead and resume";
      * every arm produces exactly one terminal ces_events record naming the binding gate, so a
        no-resume is always explained rather than merely absent;
      * a repeated exception in THIS method is escalated to cloudlog.error rather than swallowed --
        an auto-resume that has quietly stopped deciding must announce itself.
    """
    if self.mads_resume is None:
      return
    try:
      now = time.monotonic()
      # radarState.leadOne. THREE-STATE on purpose: True/False are real answers, None means the
      # read itself failed (message not alive/valid, or malformed) -- which the brain refuses on.
      has_lead = None
      d_rel = v_lead = None
      try:
        if self.sm.alive['radarState'] and self.sm.valid['radarState']:
          lead = self.sm['radarState'].leadOne
          has_lead = bool(lead.status)
          if has_lead:
            d_rel = float(lead.dRel)
            v_lead = float(lead.vLead)
      except Exception:
        has_lead = None

      inputs = ResumeInputs(
        now=now,
        mads_available=bool(self.mads.available),
        lateral_only=bool(self.mads.lateral_only),
        op_enabled=bool(self.enabled),
        blocked=has_blocking_event(self.events),
        # Fable A1: the same expression publish_selfdriveState uses for ss.engageable. A NO_ENTRY
        # carries no DISABLE type, so `blocked` above does NOT cover it -- see the noEntry gate.
        engageable=not self.events.contains(ET.NO_ENTRY),
        brake_pressed=bool(CS.brakePressed),
        regen_braking=bool(CS.regenBraking),
        gas_pressed=bool(CS.gasPressed),
        cruise_enabled=bool(CS.cruiseState.enabled),
        cruise_available=bool(CS.cruiseState.available),
        set_speed_ms=float(CS.cruiseState.speed),
        # units2pnw: the cluster's unit, for the record (the speed above is already true m/s; getattr: a schema
        # without the field reads unknown)
        set_speed_unit=speed_unit_name(getattr(CS.cruiseState, "speedClusterUnit", None)),
        v_ego=float(CS.vEgo),
        # gassetwait2pnw: sampled every tick, but the brain LATCHES it on the lift-off frame only -- it
        # decides whether that lift-off was "on the power" (0.5 s wait) or "already slowing" (1.0 s).
        a_ego=float(CS.aEgo),
        standstill=bool(CS.standstill),
        driver_cruise_button=any(be.pressed and be.type in (ButtonType.accelCruise, ButtonType.decelCruise,
                                                             ButtonType.resumeCruise, ButtonType.setCruise,
                                                             ButtonType.mainCruise)
                                 for be in CS.buttonEvents),
        has_lead=has_lead, d_rel=d_rel, v_lead=v_lead,
      )
      out = self.mads_resume.update(inputs)

      # --- publish / withdraw the offer -------------------------------------------------------
      # Withdrawal is IMMEDIATE and unthrottled: the executor's freshness bound only limits how
      # long a stale offer can survive, it does not shorten a live one, so the brain going quiet
      # must reach /dev/shm on the very next tick.
      if self.mads_resume_mem is not None:
        if out.offer:
          if now - self.mads_resume_pub_t >= 0.05:      # 20 Hz heartbeat (the executor re-reads
            # this key EVERY frame while it holds a command, and at 4 Hz only while idle, so a
            # withdrawal below lands within ~10 ms -- see carcontroller._resume_button)
            self.mads_resume_pub_t = now
            self.mads_resume_offered = True
            self.mads_resume_mem.put_nonblocking("MadsResumeTarget", {
              # gasset2pnw: "res" taps RESUME (hand back the driver's remembered set speed);
              # "set" taps SET at the speed they just chose with the accelerator.
              "dir": out.mode,
              # MONOTONIC, not wall clock (Gemini review 2026-09-06). CLOCK_MONOTONIC is shared
              # across processes on this host, so the executor can compare against its own
              # time.monotonic(); wall clock could not be trusted for a 0.5 s freshness bound on a
              # device with a dead RTC that takes a large step the first time it syncs. The dec/inc
              # set-speed mem-params keep wall clock -- they are not this code and are not touched.
              "ts": round(now, 3),
              "eid": out.eid,
              "set": round(out.set_ms, 2),
            })
        elif self.mads_resume_offered:
          self.mads_resume_offered = False
          self.mads_resume_mem.put_nonblocking("MadsResumeTarget", {})

      # --- telemetry --------------------------------------------------------------------------
      for rec in out.records:
        if rec.get("phase") == "verify" and rec.get("reason") == "noCruise":
          # Fable D1: we pressed RESUME and stock cruise never came back. Benign in isolation (the
          # press may have been correctly ignored), but it is ALSO the exact signature of the
          # executor being pinned without the matching panda safety gate, where every press is a TX
          # violation the panda drops silently. Either way the feature could not do its job, so it
          # must say so rather than leaving one quiet JSONL line as the only trace.
          # Do NOT name one cause. This warning previously said only "check the panda safety pin",
          # and the executor has since grown gates the brain cannot see (decide_resume's own checks,
          # and the CC.latActive gate on the SET path) -- any of which drops the press silently
          # while the brain has already logged `fire`. Sending a reader to the panda for what was
          # actually an executor refusal is the wild-goose chase this line exists to prevent
          # (Gemini review 2026-09-07 round 3, finding E).
          why = " ".join([
            "(1) the EXECUTOR refused it -- decide_resume gates, or the CC.latActive gate on a SET;",
            "(2) the panda dropped the TX -- check the safety pin;",
            "(3) the PCM ignored a legitimate press.",
            "The executor's state is not in this record, so start there, not at the panda.",
          ])
          cloudlog.warning("madsresume2pnw: %s press sent, stock cruise never re-engaged. %s (record: %s)",
                           rec.get("mode", "?"), why, rec)
        if rec.get("unitAssumed"):
          # units2pnw (Rule 2): carstate could not establish the cluster unit (Cluster_Info1_FD1 never received) and
          # ASSUMED mph, so the verify compared on that assumption. Flagged by the brain once per change to unknown.
          cloudlog.warning("madsresume2pnw: verify compared set speeds with the cluster unit NOT established -- carstate " +
                           "ASSUMED mph; the carstate units2pnw log line has MetricActv_B_Actl (record: %s)", rec)
        if rec.get("waitWhy") == "accelUnknown" and rec.get("phase") in ("fire", "refuse"):
          # gassetwait2pnw (Rule 2): carState.aEgo was not finite on the lift-off frame, so the brain could
          # not tell "on the power" from "already slowing" and fell back to the LONG 1.0 s wait. That is the
          # safe direction, not a silent one -- the driver simply keeps today's behaviour, and this says why.
          cloudlog.error("gassetwait2pnw: aEgo unreadable at lift-off -- kept the 1.0 s wait (record: %s)", rec)
        if rec.get("loud"):
          cloudlog.error("madsresume2pnw: cruise resumed to %.2f m/s, ABOVE the driver's captured set speed %.2f m/s -- investigate (record: %s)",
                         rec.get("gotMs", 0.0), rec.get("wantMs", 0.0), rec)
        try:
          self.ces_pnw.log_mads_resume(rec)
        except Exception:
          cloudlog.exception("madsresume2pnw: ces_events append failed")
      self.mads_resume_fail = 0
    except Exception:
      self.mads_resume_fail += 1
      # Loud on the first failure, then throttled -- this runs at 100 Hz.
      if self.mads_resume_fail == 1 or self.mads_resume_fail % 1000 == 0:
        cloudlog.exception(f"madsresume2pnw: _mads_resume_step FAILED ({self.mads_resume_fail} consecutive) -- auto-resume is NOT deciding")

  def _log_take_control_edge(self, CS) -> None:
    """takecontrol2pnw: PURE OBSERVATION — write the "Take Control" (steerSaturated) alert to
    ces_events.jsonl as a discrete, edge-triggered record, so it doesn't have to be dug out of the
    rlog. Does NOT re-derive or influence the steerSaturated decision (made above, in update_events)
    — it only reads whether that decision fired THIS frame (last_steer_saturated_frame == sm.frame,
    set immediately after self.events.add(EventName.steerSaturated)) and logs it.

    Gentle: per-tick cost is one int compare (this frame vs. last_steer_saturated_frame) + one more
    for the hold-off window — mirrors the existing recent_steer_pressed idiom in update_events(), no
    new allocation, no cereal/JSON/file work. Building the record (co-active event names, GPS/steer
    state via ces_pnw, the JSONL append) only happens on the rare rising or closing edge, via
    ces_pnw.log_take_control_alert() — this method never touches ces_pnw's file writer directly.

    Debounced: once open, an episode stays open (no new record) through any gap in the alert shorter
    than STEER_SATURATED_HOLDOFF_S; only a continuous absence at least that long closes it and
    re-arms the next rising edge for a new record — one record per real episode. durationS on the
    "end" record is measured to the LAST frame the alert actually fired (last_steer_saturated_frame),
    not to the close tick — the close tick is delayed by the hold-off, so measuring to it would
    inflate durationS by up to STEER_SATURATED_HOLDOFF_S (a single-frame alert would read ~1.0 s
    instead of ~0.0 s). Note this also means the "end" record's vEgo/GPS/steer-limit/otherEvents
    fields (built by _emit_take_control_alert from the CURRENT CS, at the close tick) reflect that
    later close-tick snapshot, not the episode's peak/trigger moment — offline analysis should treat
    them as "state ~1 s after the episode", not "state during the episode".

    Exception-isolated: any failure here (missing ces_pnw method on an older stub, bad CS field,
    etc.) is swallowed — this runs inside selfdrived's ~100 Hz control loop and must never raise."""
    try:
      sat_now = self.last_steer_saturated_frame is not None and self.sm.frame == self.last_steer_saturated_frame
      if sat_now:
        if not self.steer_saturated_open:
          # RISING EDGE — the only per-tick branch that does real (rare) work.
          self.steer_saturated_open = True
          self.steer_saturated_start_frame = self.sm.frame
          self._emit_take_control_alert(CS, "start", None)
        return
      if self.steer_saturated_open and self.last_steer_saturated_frame is not None and \
         (self.sm.frame - self.last_steer_saturated_frame) * DT_CTRL >= STEER_SATURATED_HOLDOFF_S:
        # Sustained absence past the hold-off -- close the episode and re-arm.
        self.steer_saturated_open = False
        # Duration to the LAST frame the alert actually fired, not to this (hold-off-delayed) close
        # tick -- see docstring. Clamped to >= 0.0 defensively (frame bookkeeping should guarantee
        # last_steer_saturated_frame >= steer_saturated_start_frame, but never let a clock-ish oddity
        # log a negative duration).
        duration_s = max(0.0, (self.last_steer_saturated_frame - self.steer_saturated_start_frame) * DT_CTRL)
        self._emit_take_control_alert(CS, "end", duration_s)
    except Exception:
      cloudlog.exception("takecontrol2pnw: _log_take_control_edge raised")

  def _emit_take_control_alert(self, CS, phase: str, duration_s) -> None:
    """Builds the (plain-python, no cereal/capnp objects) payload and hands it to ces_pnw to persist.
    Called ONLY from the rare rising/closing edge above. On phase="end" the CS snapshot here is from
    the close tick (~1 s after the episode, see _log_take_control_edge docstring), not the peak."""
    try:
      other_events = [EVENT_NAME.get(e, str(e)) for e in self.events.names if e != EventName.steerSaturated]
    except Exception:
      other_events = []
    try:
      v_ego = float(getattr(CS, 'vEgo', None))
      v_ego = round(v_ego, 2) if math.isfinite(v_ego) else None
    except (TypeError, ValueError):
      v_ego = None
    payload = {"name": "steerSaturated", "phase": phase, "vEgo": v_ego, "otherEvents": other_events}
    if duration_s is not None:
      payload["durationS"] = round(duration_s, 2)
    try:
      self.ces_pnw.log_take_control_alert(payload)
    except Exception:
      cloudlog.exception("ces_pnw: log_take_control_alert raised")

  def params_thread(self, evt):
    while not evt.is_set():
      self.is_metric = self.params.get_bool("IsMetric")
      self.is_ldw_enabled = self.params.get_bool("IsLdwEnabled")
      self.disengage_on_accelerator = self.params.get_bool("DisengageOnAccelerator")
      self.manual_experimental_mode = self.params.get_bool("ExperimentalMode") and self.CP.openpilotLongitudinalControl  # ces2xnor
      self.personality = self.params.get("LongitudinalPersonality", return_default=True)
      time.sleep(0.1)

  def run(self):
    e = threading.Event()
    t = threading.Thread(target=self.params_thread, args=(e, ))
    try:
      t.start()
      while True:
        self.step()
        self.rk.monitor_time()
    finally:
      e.set()
      t.join()


def main():
  config_realtime_process(4, Priority.CTRL_HIGH)
  s = SelfdriveD()
  s.run()

if __name__ == "__main__":
  main()
