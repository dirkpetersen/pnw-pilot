#!/usr/bin/env python3
import os
import time
import threading

import cereal.messaging as messaging

from cereal import car, log

from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, Priority, Ratekeeper
from openpilot.common.swaglog import cloudlog, ForwardingHandler

from opendbc.car import DT_CTRL, structs
from opendbc.car.can_definitions import CanData, CanRecvCallable, CanSendCallable
from opendbc.car.carlog import carlog
from opendbc.car.fw_versions import ObdCallback
from opendbc.car.car_helpers import get_car, interfaces
from opendbc.car.interfaces import CarInterfaceBase, RadarInterfaceBase
from openpilot.selfdrive.pandad import can_capnp_to_list, can_list_to_can_capnp
from openpilot.selfdrive.car.cruise import VCruiseHelper
from openpilot.selfdrive.controls.lib.pnw_vehicle import PnwVehicle

REPLAY = "REPLAY" in os.environ

EventName = log.OnroadEvent.EventName

# forward
carlog.addHandler(ForwardingHandler(cloudlog))


def obd_callback(params: Params) -> ObdCallback:
  def set_obd_multiplexing(obd_multiplexing: bool):
    if params.get_bool("ObdMultiplexingEnabled") != obd_multiplexing:
      cloudlog.warning(f"Setting OBD multiplexing to {obd_multiplexing}")
      params.remove("ObdMultiplexingChanged")
      params.put_bool("ObdMultiplexingEnabled", obd_multiplexing)
      params.get_bool("ObdMultiplexingChanged", block=True)
      cloudlog.warning("OBD multiplexing set successfully")
  return set_obd_multiplexing


def can_comm_callbacks(logcan: messaging.SubSocket, sendcan: messaging.PubSocket) -> tuple[CanRecvCallable, CanSendCallable]:
  def can_recv(wait_for_one: bool = False) -> list[list[CanData]]:
    """
    wait_for_one: wait the normal logcan socket timeout for a CAN packet, may return empty list if nothing comes

    Returns: CAN packets comprised of CanData objects for easy access
    """
    ret = []
    for can in messaging.drain_sock(logcan, wait_for_one=wait_for_one):
      ret.append([CanData(msg.address, msg.dat, msg.src) for msg in can.can])
    return ret

  def can_send(msgs: list[CanData]) -> None:
    sendcan.send(can_list_to_can_capnp(msgs, msgtype='sendcan'))

  return can_recv, can_send


# calswap2pnw: the 5 learned, CAR-SPECIFIC params the manual "Reset Calibration" button clears
# (device.py). Camera extrinsics + learned steering/torque/vehicle geometry — all wrong after the
# device moves to a different car, which drives "weird" until openpilot slowly re-learns.
_CALIBRATION_PARAMS = ("CalibrationParams", "LiveTorqueParameters", "LiveParameters",
                       "LiveParametersV2", "LiveDelay")


def car_changed_for_recal(stored_car: str, cur_car: str, brand: str, fp_fixed: bool) -> bool:
  """calswap2pnw: True iff we should force a full recalibration — a real, RELIABLE fingerprint that
  differs from the car the current calibration was recorded for. Never on mock / empty / fixed-source
  (the unreliable fleet-VIN fallback — a bad one must not wipe a good calibration, per the poisoned-
  cache incident) / an empty stored tag (first boot or same car). Pure — unit-tested.

  Discriminator = carFingerprint (the car MODEL), chosen deliberately over carVin (Gemini 2026-07-13
  flagged that VIN uniquely IDs the physical car). This device's fleet is exactly one Tesla Raven +
  one Ford Lightning — DIFFERENT models, so the fingerprint distinguishes them perfectly, and VIN is
  rejected precisely because its sporadic empty/flaky reads would spuriously wipe calibration on the
  SAME car every reboot (the exact churn the driver said they do NOT want). Two cars of the SAME
  model are intentionally treated as one: identical mounting geometry means one calibration is
  correct for both, so not distinguishing them is RIGHT, not a gap (driver 2026-07-13)."""
  if brand == "mock" or not cur_car or fp_fixed:
    return False
  return bool(stored_car) and stored_car != cur_car


class Car:
  CI: CarInterfaceBase
  RI: RadarInterfaceBase
  CP: car.CarParams

  def __init__(self, CI=None, RI=None) -> None:
    self.can_sock = messaging.sub_sock('can', timeout=20)
    self.sm = messaging.SubMaster(['pandaStates', 'carControl', 'onroadEvents'])
    self.pm = messaging.PubMaster(['sendcan', 'carState', 'carParams', 'carOutput', 'liveTracks'])

    self.can_rcv_cum_timeout_counter = 0

    self.CC_prev = car.CarControl.new_message()
    self.CS_prev = car.CarState.new_message()
    self.initialized_prev = False

    self.last_actuators_output = structs.CarControl.Actuators()

    self.params = Params()

    self.can_callbacks = can_comm_callbacks(self.can_sock, self.pm.sock['sendcan'])

    is_release = self.params.get_bool("IsReleaseBranch")

    if CI is None:
      # wait for one pandaState and one CAN packet
      print("Waiting for CAN messages...")
      while True:
        can = messaging.recv_one_retry(self.can_sock)
        if len(can.can) > 0:
          break

      alpha_long_allowed = self.params.get_bool("AlphaLongitudinalEnabled")
      num_pandas = len(messaging.recv_one_retry(self.sm.sock['pandaStates']).pandaStates)

      cached_params = None
      cached_params_raw = self.params.get("CarParamsCache")
      if cached_params_raw is not None:
        with car.CarParams.from_bytes(cached_params_raw) as _cached_params:
          cached_params = _cached_params

      self.CI = get_car(*self.can_callbacks, obd_callback(self.params), alpha_long_allowed, is_release, cached_params, num_pandas=num_pandas)
      self.RI = interfaces[self.CI.CP.carFingerprint].RadarInterface(self.CI.CP)
      self.CP = self.CI.CP

      # continue onto next fingerprinting step in pandad
      self.params.put_bool("FirmwareQueryDone", True)
    else:
      self.CI, self.CP = CI, CI.CP
      self.RI = RI

    self.CP.alternativeExperience = 0
    openpilot_enabled_toggle = self.params.get_bool("OpenpilotEnabledToggle")
    controller_available = self.CI.CC is not None and openpilot_enabled_toggle and not self.CP.dashcamOnly
    self.CP.passive = not controller_available or self.CP.dashcamOnly
    if self.CP.passive:
      safety_config = structs.CarParams.SafetyConfig()
      safety_config.safetyModel = structs.CarParams.SafetyModel.noOutput
      self.CP.safetyConfigs = [safety_config]

    if self.CP.secOcRequired:
      # Copy user key if available
      try:
        with open("/cache/params/SecOCKey") as f:
          user_key = f.readline().strip()
          if len(user_key) == 32:
            self.params.put("SecOCKey", user_key)
      except Exception:
        pass

      secoc_key = self.params.get("SecOCKey")
      if secoc_key is not None:
        saved_secoc_key = bytes.fromhex(secoc_key.strip())
        if len(saved_secoc_key) == 16:
          self.CP.secOcKeyAvailable = True
          self.CI.CS.secoc_key = saved_secoc_key
          if controller_available:
            self.CI.CC.secoc_key = saved_secoc_key
        else:
          cloudlog.warning("Saved SecOC key is invalid")

    # calswap2pnw: force a full recalibration when the device is moved to a DIFFERENT car (driver
    # report 2026-07-13: the truck drove weird after a Tesla->Lightning swap until a manual Reset
    # Calibration). MUST run BEFORE the CarParams put below: calibrationd / paramsd / torqued block
    # on CarParams, so clearing the learned params here means they unblock and read the fresh (clean)
    # state with NO restart — race-free. Mirrors the manual Reset button exactly.
    self._maybe_reset_calibration_on_car_change()

    # uploadgate2pnw: GearPark change-only publisher state. Seed the param False at startup so a
    # stale True from a previous session (e.g. hard power cut while parked) can't leave the uploader
    # gate open during a drive that never touches Park.
    self._gear_park_last = False
    self.params.put_bool_nonblocking("GearPark", False)

    # Write CarParams for controls and radard (current session — may be MOCK, which just runs passive)
    cp_bytes = self.CP.to_bytes()
    self.params.put("CarParams", cp_bytes)

    # fingerprint2pnw (re-port of the testing-era fingerprint2xnor fix): a MOCK fingerprint is a
    # flaky / no-car read (e.g. the FW query racing a transitional boot while the car wakes). On this
    # shared Tesla+Lightning device BOTH cars are supported, so MOCK is never a genuine "unsupported
    # car" — and letting it overwrite the persistent/cache CarParams makes the OFFROAD UI show
    # "dashcam" (cached MOCK has dashcamOnly) and strips the FW cache the next boot relies on
    # (happened 2026-07-10 right after a deploy reboot). Never persist MOCK over a previously-good
    # fingerprint; a real fingerprint updates everything normally.
    if self.CP.brand != "mock":
      # Write previous route's CarParams (only when we have a real new fingerprint)
      prev_cp = self.params.get("CarParamsPersistent")
      if prev_cp is not None:
        self.params.put("CarParamsPrevRoute", prev_cp)
      self.params.put_nonblocking("CarParamsPersistent", cp_bytes)
      # fpcache2pnw (2026-07-11 poisoned-cache incident): a fixed-source fingerprint (the fleet-VIN
      # fallback) means the FW set we hold ALREADY failed match_fw_to_car — that's the only way the
      # fallback fires. Caching it poisons the fast-path: a sleepy-bus 10-entry FW set was cached
      # under a Lightning label, every later card restart re-failed the match on it, and the fleet
      # fallback is gated `and not cached` (device-swap guard) -> the whole session fell to
      # MOCK/dashcam even with the truck fully on. Only cache FW/CAN-matched fingerprints;
      # CarParamsPersistent (UI/display) still updates above.
      if self.CP.fingerprintSource != structs.CarParams.FingerprintSource.fixed:
        self.params.put_nonblocking("CarParamsCache", cp_bytes)

    self.v_cruise_helper = VCruiseHelper(self.CP)

    self.is_metric = self.params.get_bool("IsMetric")
    self.experimental_mode = self.params.get_bool("ExperimentalMode")

    # card is driven by can recv, expected at 100Hz
    self.rk = Ratekeeper(100, print_delay_threshold=None)

  def _maybe_reset_calibration_on_car_change(self) -> None:
    """calswap2pnw: if this real fingerprint differs from the car the current calibration belongs to,
    clear the learned car-specific params so openpilot recalibrates cleanly on the new car. Also
    records the current car tag (seeds it on first boot without resetting). Fully defensive — a
    param hiccup here must never break card startup.

    oplongpersist2pnw (req 3b): the SAME car-change event also resets op-long (AlphaLongitudinalEnabled)
    to stock ACC for the newly-swapped car, UNLESS that car's op-long is NATIVE (the Tesla — never
    touched, never cycled; op-long isn't a per-session opt-in there in the first place). This pairs
    with req 3a (system/manager/manager.py no longer force-resets AlphaLongitudinalEnabled on every
    boot): op-long now persists across a SAME-car reboot, so a real car swap is the only remaining
    reset trigger, and it must land here (the one place that already reliably detects a swap) rather
    than at every boot.

    Ordering trap this must solve: self.CP was already fixed at fingerprint time — get_car() in
    __init__ (called BEFORE this method) reads the PERSISTED AlphaLongitudinalEnabled and bakes it
    into self.CP.openpilotLongitudinalControl for this whole process lifetime. So if a Lightning was
    left with op-long ON and the device is then swapped onto it, self.CP.openpilotLongitudinalControl
    is ALREADY True for THIS session's card process — merely writing the param False here would only
    take effect NEXT boot, leaving op-long ON on the freshly-swapped (and freshly-uncalibrated) car
    for the whole current drive. Fix: mirror developer.py::_on_alpha_long_enabled's own mechanism —
    set OnroadCycleRequested=True alongside the param write. hardwared.py
    (system/hardware/hardwared.py) consumes it: OnroadCycleRequested forces not_onroad_cycle=False for
    ONROAD_CYCLE_TIME, which drops should_start and cycles deviceState.started, so manager_thread's
    ensure_running() restarts the ONROAD process set (including this card process) and it re-runs
    get_car() against the now-False param — all within the SAME power-up, no reboot needed. No
    re-trigger loop: CalibrationCar is updated to cur_car below in this same call, so the post-cycle
    re-init of card.py sees stored_car == cur_car and car_changed_for_recal() returns False the second
    time around. Only cycle when op-long was ACTUALLY on for the new car
    (self.CP.openpilotLongitudinalControl) — a swap onto an already-stock-ACC car needs no reload."""
    cur_car = str(getattr(self.CP, 'carFingerprint', '') or '')
    fp_fixed = self.CP.fingerprintSource == structs.CarParams.FingerprintSource.fixed
    try:
      stored = self.params.get("CalibrationCar")
      stored_car = stored.decode() if isinstance(stored, bytes) else (stored or "")
    except Exception:
      stored_car = ""
    if car_changed_for_recal(stored_car, cur_car, str(self.CP.brand or ''), fp_fixed):
      cloudlog.warning(f"card: car changed {stored_car!r} -> {cur_car!r} — forcing recalibration")
      for k in _CALIBRATION_PARAMS:
        try:
          self.params.remove(k)
        except Exception:
          pass
      # oplongpersist2pnw (req 3b): see the docstring above for the full ordering rationale.
      try:
        if not PnwVehicle(self.CP).op_long_native and self.CP.openpilotLongitudinalControl:
          cloudlog.warning("card: car changed with op-long ON — resetting to stock ACC + requesting onroad cycle")
          self.params.put_bool("AlphaLongitudinalEnabled", False)
          self.params.put_bool("OnroadCycleRequested", True)
      except Exception:
        pass
    # tag the car this calibration now belongs to — but only on a real, reliable fingerprint, and
    # only when it changed (avoids churning the param every boot). First boot: stored empty -> no
    # reset above, just record the current car so the NEXT swap is detected.
    if self.CP.brand != "mock" and cur_car and not fp_fixed and stored_car != cur_car:
      try:
        self.params.put_nonblocking("CalibrationCar", cur_car)
      except Exception:
        pass

  def state_update(self) -> tuple[car.CarState, structs.RadarDataT | None]:
    """carState update loop, driven by can"""

    can_strs = messaging.drain_sock_raw(self.can_sock, wait_for_one=True)
    can_list = can_capnp_to_list(can_strs)

    # Update carState from CAN
    CS = self.CI.update(can_list)

    # Update radar tracks from CAN
    RD: structs.RadarDataT | None = self.RI.update(can_list)

    self.sm.update(0)

    can_rcv_valid = len(can_strs) > 0

    # Check for CAN timeout
    if not can_rcv_valid:
      self.can_rcv_cum_timeout_counter += 1

    if can_rcv_valid and REPLAY:
      self.can_log_mono_time = messaging.log_from_bytes(can_strs[0]).logMonoTime

    self.v_cruise_helper.update_v_cruise(CS, self.sm['carControl'].enabled, self.is_metric)
    if self.sm['carControl'].enabled and not self.CC_prev.enabled:
      # Use CarState w/ buttons from the step selfdrived enables on
      self.v_cruise_helper.initialize_v_cruise(self.CS_prev, self.experimental_mode)

    # TODO: mirror the carState.cruiseState struct?
    CS.vCruise = float(self.v_cruise_helper.v_cruise_kph)
    CS.vCruiseCluster = float(self.v_cruise_helper.v_cruise_cluster_kph)

    return CS, RD

  def state_publish(self, CS: car.CarState, RD: structs.RadarDataT | None):
    """carState and carParams publish loop"""

    # carParams - logged every 50 seconds (> 1 per segment)
    if self.sm.frame % int(50. / DT_CTRL) == 0:
      cp_send = messaging.new_message('carParams')
      cp_send.valid = True
      cp_send.carParams = self.CP
      self.pm.send('carParams', cp_send)

    # publish new carOutput
    co_send = messaging.new_message('carOutput')
    co_send.valid = self.sm.all_checks(['carControl'])
    co_send.carOutput.actuatorsOutput = self.last_actuators_output
    self.pm.send('carOutput', co_send)

    # kick off controlsd step while we actuate the latest carControl packet
    cs_send = messaging.new_message('carState')
    cs_send.valid = CS.canValid
    cs_send.carState = CS
    cs_send.carState.canErrorCounter = self.can_rcv_cum_timeout_counter
    cs_send.carState.cumLagMs = -self.rk.remaining * 1000.
    self.pm.send('carState', cs_send)

    # uploadgate2pnw: publish "gear is in Park" as a tiny CHANGE-ONLY param so background processes
    # (uploader) can gate on it WITHOUT subscribing to the 100 Hz carState stream (lesson 2026-07-13:
    # extra msgq readers in the uploader caused a commIssue cascade). ~2-4 writes per drive. Only a
    # VALID read may flip it; an invalid/no-CAN tick keeps the last known value (no flapping).
    # Car-agnostic (gearShifter is a standard CarState field on every brand).
    if CS.canValid:
      parked = CS.gearShifter == car.CarState.GearShifter.park
      if parked != self._gear_park_last:
        self._gear_park_last = parked
        self.params.put_bool_nonblocking("GearPark", parked)

    if RD is not None:
      tracks_msg = messaging.new_message('liveTracks')
      tracks_msg.valid = not any(RD.errors.to_dict().values())
      tracks_msg.liveTracks = RD
      self.pm.send('liveTracks', tracks_msg)

  def controls_update(self, CS: car.CarState, CC: car.CarControl):
    """control update loop, driven by carControl"""

    if not self.initialized_prev:
      # Initialize CarInterface, once controls are ready
      # TODO: this can make us miss at least a few cycles when doing an ECU knockout
      self.CI.init(self.CP, *self.can_callbacks)
      # signal pandad to switch to car safety mode
      self.params.put_bool_nonblocking("ControlsReady", True)

    if self.sm.all_alive(['carControl']):
      # send car controls over can
      now_nanos = self.can_log_mono_time if REPLAY else int(time.monotonic() * 1e9)
      self.last_actuators_output, can_sends = self.CI.apply(CC, now_nanos)
      self.pm.send('sendcan', can_list_to_can_capnp(can_sends, msgtype='sendcan', valid=CS.canValid))

      self.CC_prev = CC

  def step(self):
    CS, RD = self.state_update()

    self.state_publish(CS, RD)

    initialized = (not any(e.name == EventName.selfdriveInitializing for e in self.sm['onroadEvents']) and
                   self.sm.seen['onroadEvents'])
    if not self.CP.passive and initialized:
      self.controls_update(CS, self.sm['carControl'])

    self.initialized_prev = initialized
    self.CS_prev = CS

  def params_thread(self, evt):
    while not evt.is_set():
      self.is_metric = self.params.get_bool("IsMetric")
      self.experimental_mode = self.params.get_bool("ExperimentalMode") and self.CP.openpilotLongitudinalControl
      time.sleep(0.1)

  def card_thread(self):
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
  # pnw: a card crash is otherwise invisible (manager discards child stderr and does not
  # restart crashed processes) — record any fatal exception to swaglog before dying.
  try:
    config_realtime_process(4, Priority.CTRL_HIGH)
    car = Car()
    car.card_thread()
  except Exception:
    cloudlog.exception("card: fatal crash")
    raise


if __name__ == "__main__":
  main()
