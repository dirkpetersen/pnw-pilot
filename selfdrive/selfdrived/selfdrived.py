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
from openpilot.selfdrive.selfdrived.events import Events, ET, EVENT_NAME  # EVENT_NAME: takecontrol2pnw
from openpilot.selfdrive.selfdrived.helpers import ExcessiveActuationCheck
from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import CESController, CESStub  # ces2xnor / stophold2pnw
from openpilot.selfdrive.controls.lib.ces_pnw.green_light import attentive_now  # dmgate2pnw: attention gate
from openpilot.selfdrive.selfdrived.state import StateMachine
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
    self.pm = messaging.PubMaster(['selfdriveState', 'onroadEvents'])

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
    self.no_disengage_on_brake = self.params.get_bool("NoDisengageOnBrake")  # auto2pnw

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

    self.initialized = False
    self.enabled = False
    self.active = False
    self.mismatch_counter = 0
    self.cruise_mismatch_counter = 0
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
      self.events.add(EventName.greenLight)          # ALWAYS — the traffic-light nudge is never
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
        self.events.add(EventName.leadDeparting)

    # Don't add any more events while in dashcam mode
    if self.CP.passive:
      return

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
      # auto2pnw: NoDisengageOnBrake suppresses the brake/regen-braking disengage events so openpilot
      # stays engaged through brake presses. Gas-pedal disengage is unaffected. Panda safety untouched.
      brake_disengage = (CS.brakePressed and (not self.CS_prev.brakePressed or not CS.standstill)) or \
                        (CS.regenBraking and (not self.CS_prev.regenBraking or not CS.standstill))
      if self.no_disengage_on_brake:
        brake_disengage = False
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
      if not self.sm['livePose'].inputsOK:
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

    self.pm.send('selfdriveState', ss_msg)

    # onroadEvents - logged every second or on change
    if (self.sm.frame % int(1. / DT_CTRL) == 0) or (self.events.names != self.events_prev):
      ce_send = messaging.new_message('onroadEvents', len(self.events))
      ce_send.valid = True
      ce_send.onroadEvents = self.events.to_msg()
      self.pm.send('onroadEvents', ce_send)
    self.events_prev = self.events.names.copy()

  def step(self):
    CS = self.data_sample()
    self.update_events(CS)
    if not self.CP.passive and self.initialized:
      self.enabled, self.active = self.state_machine.update(self.events)
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
      self.no_disengage_on_brake = self.params.get_bool("NoDisengageOnBrake")  # auto2pnw
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
