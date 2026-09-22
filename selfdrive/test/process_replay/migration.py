from collections import defaultdict
from collections.abc import Callable
from typing import cast
import capnp
import functools
import os
import re
import subprocess
import traceback
import warnings

from cereal import messaging, car, custom, log
from openpilot.common.basedir import BASEDIR
from opendbc.car.fingerprints import MIGRATION
from opendbc.car.toyota.values import EPS_SCALE, ToyotaSafetyFlags
from opendbc.car.ford.values import CAR as FORD, FordFlags, FordSafetyFlags
from opendbc.car.hyundai.values import HyundaiSafetyFlags
from opendbc.car.gm.values import GMSafetyFlags
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.modeld.fill_model_msg import fill_xyz_poly, fill_lane_line_meta
from openpilot.selfdrive.test.process_replay.vision_meta import meta_from_encode_index
from openpilot.selfdrive.controls.lib.longitudinal_planner import get_accel_from_plan, CONTROL_N_T_IDX
from openpilot.system.manager.process_config import managed_processes
from openpilot.tools.lib.logreader import LogIterable

MessageWithIndex = tuple[int, capnp.lib.capnp._DynamicStructReader]
MigrationOps = tuple[list[tuple[int, capnp.lib.capnp._DynamicStructReader]], list[capnp.lib.capnp._DynamicStructReader], list[int]]
MigrationFunc = Callable[[list[MessageWithIndex]], MigrationOps]


# rules for migration functions
# 1. must use the decorator @migration(inputs=[...], product="...") and MigrationFunc signature
# 2. it only gets the messages that are in the inputs list
# 3. product is the message type created by the migration function, and the function will be skipped if product type already exists in lr
# 4. it must return a list of operations to be applied to the logreader (replace, add, delete)
# 5. all migration functions must be independent of each other
def migrate_all(lr: LogIterable, manager_states: bool = False, panda_states: bool = False, camera_states: bool = False):
  migrations = [
    migrate_sensorEvents,
    migrate_carParams,
    migrate_gpsLocation,
    migrate_deviceState,
    migrate_carOutput,
    migrate_controlsState,
    migrate_carState,
    migrate_liveLocationKalman,
    migrate_liveTracks,
    migrate_driverAssistance,
    migrate_drivingModelData,
    migrate_onroadEvents,
    migrate_pnwOnroadEvents,  # capnpfork2pnw: always on -- an unmigrated old log names the wrong events
    migrate_driverMonitoringState,
    migrate_longitudinalPlan,
  ]
  if manager_states:
    migrations.append(migrate_managerState)
  if panda_states:
    migrations.extend([migrate_pandaStates, migrate_peripheralState])
  if camera_states:
    migrations.append(migrate_cameraStates)

  return migrate(lr, migrations)


def migrate(lr: LogIterable, migration_funcs: list[MigrationFunc]):
  lr = list(lr)
  grouped = defaultdict(list)
  for i, msg in enumerate(lr):
    grouped[msg.which()].append(i)

  replace_ops, add_ops, del_ops = [], [], []
  for migration in migration_funcs:
    assert hasattr(migration, "inputs") and hasattr(migration, "product"), "Migration functions must use @migration decorator"
    if migration.product in grouped: # skip if product already exists
      continue

    sorted_indices = sorted(ii for i in cast(list[str], migration.inputs) for ii in grouped.get(i, []))
    msg_gen = [(i, lr[i]) for i in sorted_indices]
    r_ops, a_ops, d_ops = migration(msg_gen)
    replace_ops.extend(r_ops)
    add_ops.extend(a_ops)
    del_ops.extend(d_ops)

  for index, msg in replace_ops:
    lr[index] = msg
  for index in sorted(del_ops, reverse=True):
    del lr[index]
  for msg in add_ops:
    lr.append(msg)
  lr = sorted(lr, key=lambda x: x.logMonoTime)

  return lr


def migration(inputs: list[str], product: str|None=None):
  def decorator(func):
    @functools.wraps(func)
    def wrapper(*args, **kwargs):
      return func(*args, **kwargs)
    wrapper.inputs = inputs
    wrapper.product = product
    return wrapper
  return decorator


@migration(inputs=["longitudinalPlan", "carParams"])
def migrate_longitudinalPlan(msgs):
  ops = []

  needs_migration = all(msg.longitudinalPlan.aTarget == 0.0 for _, msg in msgs if msg.which() == 'longitudinalPlan')
  CP = next((m.carParams for _, m in msgs if m.which() == 'carParams'), None)
  if not needs_migration or CP is None:
    return [], [], []

  for index, msg in msgs:
    if msg.which() != 'longitudinalPlan':
      continue
    new_msg = msg.as_builder()
    a_target, should_stop = get_accel_from_plan(msg.longitudinalPlan.speeds, msg.longitudinalPlan.accels, CONTROL_N_T_IDX)
    new_msg.longitudinalPlan.aTarget, new_msg.longitudinalPlan.shouldStop = float(a_target), bool(should_stop)
    ops.append((index, new_msg.as_reader()))
  return ops, [], []


@migration(inputs=["longitudinalPlan"], product="driverAssistance")
def migrate_driverAssistance(msgs):
  add_ops = []
  for _, msg in msgs:
    new_msg = messaging.new_message('driverAssistance', valid=True, logMonoTime=msg.logMonoTime)
    add_ops.append(new_msg.as_reader())
  return [], add_ops, []


@migration(inputs=["modelV2"], product="drivingModelData")
def migrate_drivingModelData(msgs):
  add_ops = []
  for _, msg in msgs:
    dmd = messaging.new_message('drivingModelData', valid=msg.valid, logMonoTime=msg.logMonoTime)
    for field in ["frameId", "frameIdExtra", "frameDropPerc", "modelExecutionTime", "action"]:
      setattr(dmd.drivingModelData, field, getattr(msg.modelV2, field))
    for meta_field in ["laneChangeState", "laneChangeState"]:
      setattr(dmd.drivingModelData.meta, meta_field, getattr(msg.modelV2.meta, meta_field))
    if len(msg.modelV2.laneLines) and len(msg.modelV2.laneLineProbs):
      fill_lane_line_meta(dmd.drivingModelData.laneLineMeta, msg.modelV2.laneLines, msg.modelV2.laneLineProbs)
    if all(len(a) for a in [msg.modelV2.position.x, msg.modelV2.position.y, msg.modelV2.position.z]):
      fill_xyz_poly(dmd.drivingModelData.path, ModelConstants.POLY_PATH_DEGREE, msg.modelV2.position.x, msg.modelV2.position.y, msg.modelV2.position.z)
    add_ops.append( dmd.as_reader())
  return [], add_ops, []


@migration(inputs=["liveTracksDEPRECATED"], product="liveTracks")
def migrate_liveTracks(msgs):
  ops = []
  for index, msg in msgs:
    new_msg = messaging.new_message('liveTracks')
    new_msg.valid = msg.valid
    new_msg.logMonoTime = msg.logMonoTime

    pts = []
    for track in msg.liveTracksDEPRECATED:
      pt = car.RadarData.RadarPoint()
      pt.trackId = track.trackId

      pt.dRel = track.dRel
      pt.yRel = track.yRel
      pt.vRel = track.vRel
      pt.aRel = track.aRel
      pt.measured = True
      pts.append(pt)

    new_msg.liveTracks.points = pts
    ops.append((index, new_msg.as_reader()))
  return ops, [], []


@migration(inputs=["liveLocationKalmanDEPRECATED"], product="livePose")
def migrate_liveLocationKalman(msgs):
  nans = [float('nan')] * 3
  ops = []
  for index, msg in msgs:
    m = messaging.new_message('livePose')
    m.valid = msg.valid
    m.logMonoTime = msg.logMonoTime
    m.livePose.timestamp = msg.logMonoTime
    for field in ["orientationNED", "velocityDevice", "accelerationDevice", "angularVelocityDevice"]:
      lp_field, llk_field = getattr(m.livePose, field), getattr(msg.liveLocationKalmanDEPRECATED, field)
      lp_field.x, lp_field.y, lp_field.z = llk_field.value or nans
      lp_field.xStd, lp_field.yStd, lp_field.zStd = llk_field.std or nans
      lp_field.valid = llk_field.valid
    for flag in ["inputsOK", "posenetOK", "sensorsOK"]:
      setattr(m.livePose, flag, getattr(msg.liveLocationKalmanDEPRECATED, flag))
    ops.append((index, m.as_reader()))
  return ops, [], []


@migration(inputs=["controlsState"], product="selfdriveState")
def migrate_controlsState(msgs):
  add_ops = []
  for _, msg in msgs:
    m = messaging.new_message('selfdriveState')
    m.valid = msg.valid
    m.logMonoTime = msg.logMonoTime
    ss = m.selfdriveState
    for field in ("enabled", "active", "state", "engageable", "alertText1", "alertText2",
                  "alertStatus", "alertSize", "alertType", "experimentalMode",
                  "personality"):
      setattr(ss, field, getattr(msg.controlsState, field+"DEPRECATED"))
    add_ops.append(m.as_reader())
  return [], add_ops, []


@migration(inputs=["carState", "controlsState"])
def migrate_carState(msgs):
  ops = []
  last_cs = None
  for index, msg in msgs:
    if msg.which() == 'controlsState':
      last_cs = msg
    elif msg.which() == 'carState' and last_cs is not None:
      if last_cs.controlsState.vCruiseDEPRECATED - msg.carState.vCruise > 0.1:
        msg = msg.as_builder()
        msg.carState.vCruise = last_cs.controlsState.vCruiseDEPRECATED
        msg.carState.vCruiseCluster = last_cs.controlsState.vCruiseClusterDEPRECATED
        ops.append((index, msg.as_reader()))
  return ops, [], []


@migration(inputs=["managerState"])
def migrate_managerState(msgs):
  ops = []
  for index, msg in msgs:
    new_msg = msg.as_builder()
    new_msg.managerState.processes = [{'name': name, 'running': True} for name in managed_processes]
    ops.append((index, new_msg.as_reader()))
  return ops, [], []


@migration(inputs=["gpsLocation", "gpsLocationExternal"])
def migrate_gpsLocation(msgs):
  ops = []
  for index, msg in msgs:
    new_msg = msg.as_builder()
    g = getattr(new_msg, new_msg.which())
    # hasFix is a newer field
    if not g.hasFix and g.flags == 1:
      g.hasFix = True
    ops.append((index, new_msg.as_reader()))
  return ops, [], []


@migration(inputs=["deviceState", "initData"])
def migrate_deviceState(msgs):
  init_data = next((m.initData for _, m in msgs if m.which() == 'initData'), None)
  device_state = next((m.deviceState for _, m in msgs if m.which() == 'deviceState'), None)
  if init_data is None or device_state is None:
    return [], [], []

  ops = []
  for i, msg in msgs:
    if msg.which() == 'deviceState':
      n = msg.as_builder()
      n.deviceState.deviceType = init_data.deviceType
      ops.append((i, n.as_reader()))
  return ops, [], []


@migration(inputs=["carControl"], product="carOutput")
def migrate_carOutput(msgs):
  add_ops = []
  for _, msg in msgs:
    co = messaging.new_message('carOutput')
    co.valid = msg.valid
    co.logMonoTime = msg.logMonoTime
    co.carOutput.actuatorsOutput = msg.carControl.actuatorsOutputDEPRECATED
    add_ops.append(co.as_reader())
  return [], add_ops, []


@migration(inputs=["pandaStates", "pandaStateDEPRECATED", "carParams"])
def migrate_pandaStates(msgs):
  # TODO: safety param migration should be handled automatically
  safety_param_migration = {
    "TOYOTA_PRIUS": EPS_SCALE["TOYOTA_PRIUS"] | ToyotaSafetyFlags.STOCK_LONGITUDINAL,
    "TOYOTA_RAV4": EPS_SCALE["TOYOTA_RAV4"] | ToyotaSafetyFlags.ALT_BRAKE,
    # CANFD_LKA_STEERING -> CANFD_LKA_STEER_MSG: our opendbc pin carries the renamed flag and this
    # reference was never updated, so importing this module raised AttributeError. That is process
    # replay, not Hyundai -- every replay_process_with_name consumer in the fork depends on it, so
    # "inert for our cars" was true of car behaviour and false of the test infrastructure.
    "KIA_EV6": HyundaiSafetyFlags.EV_GAS | HyundaiSafetyFlags.CANFD_LKA_STEER_MSG,
    "CHEVROLET_VOLT": GMSafetyFlags.EV,
    "CHEVROLET_BOLT_EUV": GMSafetyFlags.EV | GMSafetyFlags.HW_CAM,
  }
  # TODO: get new Ford route
  safety_param_migration |= dict.fromkeys((set(FORD) - FORD.with_flags(FordFlags.CANFD)), FordSafetyFlags.LONG_CONTROL)

  # Migrate safety param base on carParams
  CP = next((m.carParams for _, m in msgs if m.which() == 'carParams'), None)
  assert CP is not None, "carParams message not found"
  fingerprint = MIGRATION.get(CP.carFingerprint, CP.carFingerprint)
  if fingerprint in safety_param_migration:
    safety_param = safety_param_migration[fingerprint].value
  elif len(CP.safetyConfigs):
    safety_param = CP.safetyConfigs[0].safetyParam
    if CP.safetyConfigs[0].safetyParamDEPRECATED != 0:
      safety_param = CP.safetyConfigs[0].safetyParamDEPRECATED
  else:
    safety_param = CP.safetyParamDEPRECATED

  ops = []
  for index, msg in msgs:
    if msg.which() == 'pandaStateDEPRECATED':
      new_msg = messaging.new_message('pandaStates', 1)
      new_msg.valid = msg.valid
      new_msg.logMonoTime = msg.logMonoTime
      new_msg.pandaStates[0] = msg.pandaStateDEPRECATED
      new_msg.pandaStates[0].safetyParam = safety_param
      ops.append((index, new_msg.as_reader()))
    elif msg.which() == 'pandaStates':
      new_msg = msg.as_builder()
      new_msg.pandaStates[-1].safetyParam = safety_param
      # Clear DISABLE_DISENGAGE_ON_GAS bit to fix controls mismatch
      new_msg.pandaStates[-1].alternativeExperience &= ~1
      ops.append((index, new_msg.as_reader()))
  return ops, [], []


@migration(inputs=["pandaStates", "pandaStateDEPRECATED"], product="peripheralState")
def migrate_peripheralState(msgs):
  add_ops = []

  which = "pandaStates" if any(msg.which() == "pandaStates" for _, msg in msgs) else "pandaStateDEPRECATED"
  for _, msg in msgs:
    if msg.which() != which:
      continue
    new_msg = messaging.new_message("peripheralState")
    new_msg.valid = msg.valid
    new_msg.logMonoTime = msg.logMonoTime
    add_ops.append(new_msg.as_reader())
  return [], add_ops, []


@migration(inputs=["roadEncodeIdx", "wideRoadEncodeIdx", "driverEncodeIdx", "roadCameraState", "wideRoadCameraState", "driverCameraState"])
def migrate_cameraStates(msgs):
  add_ops, del_ops = [], []
  frame_to_encode_id = defaultdict(dict)
  # just for encodeId fallback mechanism
  min_frame_id = defaultdict(lambda: float('inf'))

  for _, msg in msgs:
    if msg.which() not in ["roadEncodeIdx", "wideRoadEncodeIdx", "driverEncodeIdx"]:
      continue

    encode_index = getattr(msg, msg.which())
    meta = meta_from_encode_index(msg.which())

    assert encode_index.segmentId < 1200, f"Encoder index segmentId greater that 1200: {msg.which()} {encode_index.segmentId}"
    frame_to_encode_id[meta.camera_state][encode_index.frameId] = encode_index.segmentId

  for index, msg in msgs:
    if msg.which() not in ["roadCameraState", "wideRoadCameraState", "driverCameraState"]:
      continue

    camera_state = getattr(msg, msg.which())
    min_frame_id[msg.which()] = min(min_frame_id[msg.which()], camera_state.frameId)

    encode_id = frame_to_encode_id[msg.which()].get(camera_state.frameId)
    if encode_id is None:
      print(f"Missing encoded frame for camera feed {msg.which()} with frameId: {camera_state.frameId}")
      if len(frame_to_encode_id[msg.which()]) != 0:
        del_ops.append(index)
        continue

      # fallback mechanism for logs without encodeIdx (e.g. logs from before 2022 with dcamera recording disabled)
      # try to fake encode_id by subtracting lowest frameId
      encode_id = camera_state.frameId - min_frame_id[msg.which()]
      print(f"Faking encodeId to {encode_id} for camera feed {msg.which()} with frameId: {camera_state.frameId}")

    new_msg = messaging.new_message(msg.which())
    new_camera_state = getattr(new_msg, new_msg.which())
    new_camera_state.sensor = camera_state.sensor
    new_camera_state.frameId = encode_id
    new_camera_state.encodeId = encode_id
    # timestampSof was added later so it might be missing on some old segments
    if camera_state.timestampSof == 0 and camera_state.timestampEof > 25000000:
      new_camera_state.timestampSof = camera_state.timestampEof - 18000000
    else:
      new_camera_state.timestampSof = camera_state.timestampSof
    new_camera_state.timestampEof = camera_state.timestampEof
    new_msg.logMonoTime = msg.logMonoTime
    new_msg.valid = msg.valid

    del_ops.append(index)
    add_ops.append(new_msg.as_reader())
  return [], add_ops, del_ops


@migration(inputs=["carParams"])
def migrate_carParams(msgs):
  ops = []
  for index, msg in msgs:
    CP = msg.as_builder()
    CP.carParams.carFingerprint = MIGRATION.get(CP.carParams.carFingerprint, CP.carParams.carFingerprint)
    for car_fw in CP.carParams.carFw:
      car_fw.brand = CP.carParams.brand
    ops.append((index, CP.as_reader()))
  return ops, [], []


@migration(inputs=["sensorEventsDEPRECATED"], product="sensorEvents")
def migrate_sensorEvents(msgs):
  add_ops, del_ops = [], []
  for index, msg in msgs:
    # migrate to split sensor events
    for evt in msg.sensorEventsDEPRECATED:
      # build new message for each sensor type
      sensor_service = ''
      if evt.which() == 'acceleration':
        sensor_service = 'accelerometer'
      elif evt.which() == 'gyro' or evt.which() == 'gyroUncalibrated':
        sensor_service = 'gyroscope'
      elif evt.which() == 'light' or evt.which() == 'proximity':
        sensor_service = 'lightSensor'
      elif evt.which() == 'magnetic' or evt.which() == 'magneticUncalibrated':
        sensor_service = 'magnetometer'
      elif evt.which() == 'temperature':
        sensor_service = 'temperatureSensor'

      m = messaging.new_message(sensor_service)
      m.valid = True
      m.logMonoTime = msg.logMonoTime

      m_dat = getattr(m, sensor_service)
      m_dat.version = evt.version
      m_dat.sensor = evt.sensor
      m_dat.type = evt.type
      m_dat.source = evt.source
      m_dat.timestamp = evt.timestamp
      setattr(m_dat, evt.which(), getattr(evt, evt.which()))

      add_ops.append(m.as_reader())
    del_ops.append(index)
  return [], add_ops, del_ops


@migration(inputs=["onroadEventsDEPRECATED"], product="onroadEvents")
def migrate_onroadEvents(msgs):
  ops = []
  for index, msg in msgs:
    onroadEvents = []
    for event in msg.onroadEventsDEPRECATED:
      try:
        if not str(event.name).endswith('DEPRECATED'):
          # dict converts name enum into string representation
          onroadEvents.append(log.OnroadEvent(**event.to_dict()))
      except RuntimeError:  # Member was null
        traceback.print_exc()

    new_msg = messaging.new_message('onroadEvents', len(msg.onroadEventsDEPRECATED))
    new_msg.valid = msg.valid
    new_msg.logMonoTime = msg.logMonoTime
    new_msg.onroadEvents = onroadEvents
    ops.append((index, new_msg.as_reader()))

  return ops, [], []


@migration(inputs=["driverMonitoringState"])
def migrate_driverMonitoringState(msgs):
  ops = []
  for index, msg in msgs:
    msg = msg.as_builder()
    events = []
    for event in msg.driverMonitoringState.eventsDEPRECATED:
      try:
        if not str(event.name).endswith('DEPRECATED'):
          # dict converts name enum into string representation
          events.append(log.OnroadEvent(**event.to_dict()))
      except RuntimeError:  # Member was null
        traceback.print_exc()

    msg.driverMonitoringState.events = events
    ops.append((index, msg.as_reader()))

  return ops, [], []


# ---------------------------------------------------------------------------------------------------
# capnpfork2pnw: logs recorded BEFORE the fork's own events left log.capnp.
#
# Until capnpfork2pnw the fork put six events at log.capnp OnroadEvent.EventName @99-@104. Upstream
# has since allocated every one of those ordinals (@99 lateralManeuver in 0.11.1, @100-@103
# bigModel*/carNotReady in 0.11.2, @104 userBookmarkNotPaired on master). The events now live in
# custom.capnp (OnroadEventPnw) on their own service, onroadEventsPnw. An old log still carries them
# as raw @99-@104 inside onroadEvents: under this schema reading one raises "Member was null", and on
# a newer upstream base it would silently read as the WRONG event -- @102, the MADS safety event
# madsControlsMismatchLateral, as bigModelFailed. This migration moves them to onroadEventsPnw.
#
# WHO WROTE THE LOG decides it, and it is never guessed from the content: after a rebase an old
# @99 is a perfectly valid upstream enumerant. The writer is identified by initData.gitCommit, and
# the answer is the writer's OWN log.capnp, read from git. Anything that stops that from being
# certain -- no initData, several commits, a dirty tree, a commit git does not have, an ordinal the
# writer's schema does not define -- raises PnwLogSchemaError. docs/pnw/CAPNP-FORK-ORDINALS.md.

# Our upstream base's EventName ends at @98 (stockLkas). Nothing below it was ever the fork's.
PNW_FIRST_FORK_ORDINAL = 99

# The ONE assignment any pnw-pilot build ever used: every fork branch among the 195 refs (2026-09-21)
# carries a prefix of it, and all 115 writer commits in the recorded corpus agree. A writer schema that
# disagrees is a build nobody audited, and raises. Also what the explicit override below asserts.
PNW_FORK_V1_ORDINALS = {
  99: "greenLight",
  100: "leadDeparting",
  101: "madsLateralOnly",
  102: "madsControlsMismatchLateral",
  103: "cruiseOffRequested",
  104: "madsResumeSetTooHigh",
}

# For a log whose writer git cannot resolve (a commit that was never pushed, a foreign clone). A human
# asserting what wrote it -- never a default. "fork-v1": the writer used PNW_FORK_V1_ORDINALS.
# "upstream": its @99+ are not the fork's. Every use is warned about.
PNW_WRITER_SCHEMA_ENV = "PNW_LOG_WRITER_SCHEMA"

_ONROAD_EVENT_FLAGS = ("enable", "noEntry", "warning", "userDisable", "softDisable", "immediateDisable",
                       "preEnable", "permanent", "overrideLateral", "overrideLongitudinal")


class PnwLogSchemaError(Exception):
  """The migration cannot establish which schema wrote this log. Never guessed."""


def _parse_event_names(text: str, where: str) -> dict[int, str]:
  m = re.search(r"struct OnroadEvent @0x[0-9a-f]+ \{.*?enum EventName(?: @0x[0-9a-f]+)? \{(.*?)\}", text, re.S)
  if m is None:
    raise PnwLogSchemaError(f"{where}: no OnroadEvent.EventName enum found")
  entries = re.findall(r"^\s*(\w+)\s*@(\d+)\s*;", m.group(1), re.M)
  names = {int(o): n for n, o in entries}
  if len(names) != len(entries) or sorted(names) != list(range(len(names))):
    raise PnwLogSchemaError(f"{where}: EventName did not parse as contiguous ordinals 0..N ({len(entries)} entries)")
  return names


@functools.cache
def pnw_writer_event_names(git_commit: str) -> dict[int, str]:
  """{ordinal: name} of OnroadEvent.EventName as the build at `git_commit` defined it."""
  errs = []
  for path in ("cereal/log.capnp", "openpilot/cereal/log.capnp"):  # 0.11.1 layout, 0.11.2 layout
    r = subprocess.run(["git", "-C", BASEDIR, "show", f"{git_commit}:{path}"], capture_output=True, text=True)
    if r.returncode == 0:
      return _parse_event_names(r.stdout, f"{git_commit[:12]}:{path}")
    errs.append(r.stderr.strip())
  raise PnwLogSchemaError(f"the log's writer commit {git_commit} cannot be read from the git repo at {BASEDIR} " +
                          f"({'; '.join(errs)}). Fetch it, or set {PNW_WRITER_SCHEMA_ENV} if you KNOW what wrote the log.")


def _pnw_fork_ordinals(msgs, present: set[int]) -> dict[int, str]:
  """raw ordinal -> fork event name, for the contested ordinals `present` in this log.

  Raises PnwLogSchemaError rather than guess."""
  override = os.environ.get(PNW_WRITER_SCHEMA_ENV)
  if override is not None:
    warnings.warn(f"capnpfork2pnw: {PNW_WRITER_SCHEMA_ENV}={override!r} -- the writer schema is ASSERTED, not established",
                  stacklevel=2)
    if override == "fork-v1":
      unknown = present - PNW_FORK_V1_ORDINALS.keys()
      if unknown:
        raise PnwLogSchemaError(f"{PNW_WRITER_SCHEMA_ENV}=fork-v1, but the log has @{sorted(unknown)}, which fork-v1 never defined")
      return {o: PNW_FORK_V1_ORDINALS[o] for o in present}
    if override == "upstream":
      return {}
    raise PnwLogSchemaError(f"{PNW_WRITER_SCHEMA_ENV} must be 'fork-v1' or 'upstream', got {override!r}")

  inits = [m.initData for _, m in msgs if m.which() == "initData"]
  where = f"onroadEvents carry EventName @{sorted(present)}, but"
  if not inits:
    raise PnwLogSchemaError(f"{where} the log has no initData, so its writer is unknown")
  commits = {i.gitCommit for i in inits}
  if len(commits) != 1 or "" in commits:
    raise PnwLogSchemaError(f"{where} its initData names {len(commits)} writer commit(s) {sorted(commits)}; need exactly one")
  commit = commits.pop()
  if any(i.dirty for i in inits):
    raise PnwLogSchemaError(f"{where} its writer {commit[:12]} ran a DIRTY tree, so git cannot say what schema it ran")

  writer = pnw_writer_event_names(commit)
  reader = {v: k for k, v in log.OnroadEvent.EventName.schema.enumerants.items()}
  fork_names = custom.OnroadEventPnw.EventName.schema.enumerants
  fork = {}
  for o in sorted(present):
    name = writer.get(o)
    if name is None:
      raise PnwLogSchemaError(f"{where} writer {commit[:12]}'s own schema has no @{o} -- this log does not match its initData")
    if name in fork_names:
      if PNW_FORK_V1_ORDINALS.get(o) != name:
        raise PnwLogSchemaError(f"writer {commit[:12]} put fork event {name} at @{o}; no audited build did -- audit it first")
      fork[o] = name
    elif reader.get(o) != name:
      raise PnwLogSchemaError(f"{where} writer {commit[:12]} calls @{o} {name!r}, which is neither a fork event nor " +
                              f"what this schema calls @{o} ({reader.get(o)!r})")
  return fork


@migration(inputs=["initData", "onroadEvents"], product="onroadEventsPnw")
def migrate_pnwOnroadEvents(msgs):
  onroad = [(i, m) for i, m in msgs if m.which() == "onroadEvents"]
  present = {e.name.raw for _, m in onroad for e in m.onroadEvents if e.name.raw >= PNW_FIRST_FORK_ORDINAL}
  if not present:
    return [], [], []  # nothing at a contested ordinal: the result is the same whoever wrote the log
  fork = _pnw_fork_ordinals(msgs, present)
  if not fork:
    return [], [], []  # the writer's @99+ are upstream's and this schema agrees on every one

  pnw_ordinal = custom.OnroadEventPnw.EventName.schema.enumerants
  replace_ops, add_ops = [], []
  for index, msg in onroad:
    kept = [e for e in msg.onroadEvents if e.name.raw not in fork]
    moved = [e for e in msg.onroadEvents if e.name.raw in fork]
    if moved:
      new_msg = messaging.new_message('onroadEvents', len(kept), valid=msg.valid, logMonoTime=msg.logMonoTime)
      new_msg.onroadEvents = [e.as_builder() for e in kept]
      replace_ops.append((index, new_msg.as_reader()))
    # One onroadEventsPnw per onroadEvents, as a post-fix selfdrived publishes them.
    pnw_msg = messaging.new_message('onroadEventsPnw', valid=msg.valid, logMonoTime=msg.logMonoTime)
    events = pnw_msg.onroadEventsPnw.init('events', len(moved))
    for j, e in enumerate(moved):
      events[j].name = pnw_ordinal[fork[e.name.raw]]
      for flag in _ONROAD_EVENT_FLAGS:
        setattr(events[j], flag, getattr(e, flag))
    add_ops.append(pnw_msg.as_reader())
  return replace_ops, add_ops, []
