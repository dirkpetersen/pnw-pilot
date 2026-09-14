"""parknorec2pnw: GearPark writer rules, the real per-car gear decode they depend on, and card's call site.

GearPark now stops loggerd (system/manager/park_record_gate.py), so these are the facts that decide
whether a drive gets recorded:
  - the Tesla Raven reports Park through the real opendbc decode, on the chassis bus,
  - a Lightning whose gear message was NEVER received reads `unknown` since pnw-opendbc gearunknown2pnw
    (before it: parser zero-init + TrnRng 0 = "Park"),
  - other brands still read `park` from a bus that delivered nothing (e.g. Hyundai), which is why only valid
    CAN may SET GearPark -- this writer is car-agnostic and ships on the friends channel,
  - a CAN fault spanning Park -> Drive must not leave GearPark True.
"""
import time
from types import SimpleNamespace

import pytest

from cereal import car
from opendbc.can import CANPacker
from opendbc.car import gen_empty_fingerprint
from opendbc.car.can_definitions import CanData
from opendbc.car.car_helpers import interfaces
from opendbc.car.tesla.values import CANBUS as TESLA_CANBUS
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.car.gear_park import GearParkWriter, UNCONFIRMED_LOG_S, parser_valid_now
from openpilot.selfdrive.controls.lib.pnw_vehicle import PnwVehicle

GearShifter = car.CarState.GearShifter
TESLA = "TESLA_MODEL_S_HW3"
LIGHTNING = "FORD_F_150_LIGHTNING_MK1"


# ---------------------------------------------------------------- real decode, real DBCs, real parsers

def _interface(fp, fingerprint_addrs=()):
  CarInterface = interfaces[fp]
  f = gen_empty_fingerprint()
  for addr in fingerprint_addrs:
    f[0][addr] = 8
  return CarInterface(CarInterface.get_params(fp, f, [], False, False, False))


def decode_gear(fp, dbc, msg, values: dict, bus: int | None = None, fingerprint_addrs=(), frames=20):
  """Feed `frames` real CAN frames through the car's own CarState.update; return the last CarState.
  frames=0 = the gear message was never received."""
  CI = _interface(fp, fingerprint_addrs)
  cs = CI.update([(0, [])])
  packer = CANPacker(dbc)
  bus = TESLA_CANBUS.chassis if bus is None else bus   # read AFTER CarState init remaps the Raven's chassis bus
  for i in range(frames):
    vals = dict(values)
    if fp == TESLA:
      vals["DI_torque2Counter"] = i % 16
    addr, dat, b = packer.make_can_msg(msg, bus, vals)
    cs = CI.update([((i + 1) * 10_000_000, [CanData(addr, dat, b)])])
  return cs


def lightning_quiet_can(trnrng, rest_of_pt=True, seconds=2.0, gear=True):
  """The Lightning with its camera bus asleep: PowertrainData_10 (unless gear=False) plus, with rest_of_pt,
  every other alive-checked powertrain message, at 100 Hz, and nothing on the camera bus.
  Returns (last CarState, CarInterface) so a test can hand the real "pt" parser to the writer."""
  CI = _interface(LIGHTNING, (0x5A,))
  cs = CI.update([(0, [])])                                      # registers every message carstate reads
  packer = CANPacker("ford_lincoln_base_pt")
  pt = CI.can_parsers["pt"]
  names = [st.name for st in pt.message_states.values() if not st.ignore_alive and st.name != "PowertrainData_10"]
  for i in range(int(seconds * 100)):
    frames = []
    if gear:
      frames.append(CanData(*packer.make_can_msg("PowertrainData_10", 0, {"TrnRng_D_Rq": trnrng})))
    if rest_of_pt:
      frames += [CanData(*packer.make_can_msg(n, 0, {})) for n in names]
    cs = CI.update([((i + 1) * 10_000_000, frames)])
  return cs, CI


def tesla_gear(di_gear, frames=20):
  return decode_gear(TESLA, "tesla_can", "DI_torque2", {"DI_gear": di_gear}, frames=frames)


def lightning_gear(trnrng, frames=20):
  # 0x5A (Gear_Shift_by_Wire_FD1) in the fingerprint is what makes the interface pick the automatic
  # decode; the truck reports GearPark=1 on the device, so it takes this branch in practice.
  return decode_gear(LIGHTNING, "ford_lincoln_base_pt", "PowertrainData_10", {"TrnRng_D_Rq": trnrng},
                     bus=0, fingerprint_addrs=(0x5A,), frames=frames)


class TestRealGearDecode:
  def test_tesla_raven_reports_park_on_the_chassis_bus(self):
    assert tesla_gear(1).gearShifter == GearShifter.park       # DI_GEAR_P
    assert TESLA_CANBUS.chassis == 1                           # HW3 CarState init remaps the chassis bus
    assert tesla_gear(4).gearShifter == GearShifter.drive      # DI_GEAR_D
    assert tesla_gear(2).gearShifter == GearShifter.reverse    # DI_GEAR_R
    assert tesla_gear(7).gearShifter == GearShifter.unknown    # DI_GEAR_SNA
    assert tesla_gear(0).gearShifter == GearShifter.unknown    # DI_GEAR_INVALID

  def test_tesla_never_received_is_unknown_not_park(self):
    assert tesla_gear(1, frames=0).gearShifter == GearShifter.unknown

  def test_lightning_decode(self):
    assert lightning_gear(0).gearShifter == GearShifter.park
    assert lightning_gear(3).gearShifter == GearShifter.drive
    assert lightning_gear(14).gearShifter == GearShifter.unknown   # Unknown_Position (the DBC start value)

  def test_lightning_never_received_is_unknown(self):
    # The old trap: parser signals start at 0 and TrnRng_D_Rq 0 = "Park", so a dead powertrain bus read Park.
    # pnw-opendbc gearunknown2pnw: unknown until PowertrainData_10 has arrived. Fails if the pin loses it.
    cs = lightning_gear(0, frames=0)
    assert cs.gearShifter == GearShifter.unknown
    assert not cs.canValid

  def test_gear_source_capability_matches_the_decode(self):
    # gearparkcan2pnw: PnwVehicle may only declare gear_unknown_until_seen for a car whose pinned decode reads
    # unknown when the gear message never arrived, and gear_source_bus must name the parser that carries it.
    for fp, fingerprint_addrs, gear_addr, silent in (
        (LIGHTNING, (0x5A,), 0x176, lambda: lightning_gear(0, frames=0)),
        (TESLA, (), None, lambda: tesla_gear(1, frames=0))):
      CI = _interface(fp, fingerprint_addrs)
      CI.update([(0, [])])
      veh = PnwVehicle(CI.CP)
      assert veh.gear_unknown_until_seen and veh.gear_source_bus
      parser = CI.can_parsers[veh.gear_source_bus]
      gear_addr = gear_addr if gear_addr is not None else parser.dbc.name_to_msg["DI_torque2"].address
      assert gear_addr in parser.message_states
      assert silent().gearShifter == GearShifter.unknown

  def test_capability_is_off_for_cars_that_read_park_from_a_silent_bus(self):
    veh = PnwVehicle(_interface("HYUNDAI_SONATA").CP)
    assert not veh.gear_unknown_until_seen and veh.gear_source_bus == ""
    assert not PnwVehicle(None).gear_unknown_until_seen

  def test_other_brands_still_read_park_from_a_silent_bus(self):
    # Why the writer still requires valid CAN to SET: measured 2026-09-14 on the pinned opendbc, 74 platforms
    # (Hyundai/Kia/Genesis, RAM, some Toyota/Subaru) decode `park` from a bus that delivered nothing.
    CI = _interface("HYUNDAI_SONATA")
    cs = CI.update([(0, [])])
    for i in range(300):
      cs = CI.update([((i + 1) * 10_000_000, [])])
    assert cs.gearShifter == GearShifter.park
    assert not cs.canValid


# ---------------------------------------------------------------- writer rules

@pytest.fixture
def events(monkeypatch):
  seen = []
  monkeypatch.setattr(cloudlog, "event", lambda name, *a, **kw: seen.append((name, kw)))
  return seen


def run(writer, gear, can_valid, t0, seconds, dt=1.0):
  """Tick the writer; return the list of values it asked card to write."""
  writes = []
  t = t0
  while t < t0 + seconds:
    w = writer.update(gear, can_valid, t)
    if w is not None:
      writes.append(w)
    t += dt
  return writes


class TestGearParkWriter:
  def test_sets_only_on_valid_park(self, events):
    w = GearParkWriter()
    assert run(w, GearShifter.park, False, 0, 10) == []
    assert run(w, GearShifter.park, True, 10, 1) == [True]

  def test_lightning_dead_bus_never_sets(self, events):
    w = GearParkWriter()
    cs = lightning_gear(0, frames=0)                            # real decode: nothing received -> unknown
    assert run(w, cs.gearShifter, cs.canValid, 0, 3600, dt=0.5) == []
    assert w.value is False
    assert len([e for e in events if e[0] == "gear_park_unconfirmed"]) == 1

  def test_silent_bus_park_of_other_brands_never_sets(self, events):
    w = GearParkWriter()
    assert run(w, GearShifter.park, False, 0, 3600, dt=0.5) == []   # e.g. HYUNDAI_SONATA, see TestRealGearDecode
    assert w.value is False

  def test_lightning_quiet_can_boot_still_cannot_confirm_park(self, events):
    # KNOWN GAP, deliberately still open: a real Park frame with canValid False from the first tick (e.g. camera
    # bus asleep) does not SET, and says so. Closing it needs the per-car rule in the gearunknown2pnw report.
    w = GearParkWriter()
    cs = lightning_gear(0)                                      # only the gear frame on the bus
    assert cs.gearShifter == GearShifter.park and not cs.canValid
    assert run(w, cs.gearShifter, cs.canValid, 0, 3600, dt=0.5) == []
    assert len([e for e in events if e[0] == "gear_park_unconfirmed"]) == 1

  def test_can_fault_spanning_park_to_drive_clears(self, events):
    w = GearParkWriter()
    assert run(w, GearShifter.park, True, 0, 1) == [True]
    assert run(w, GearShifter.park, False, 1, 600) == []           # quiet CAN while charging: stays True
    assert run(w, GearShifter.drive, False, 601, 1) == [False]     # shifted with CAN still invalid

  def test_unknown_on_invalid_can_keeps_park(self, events):
    # Tesla SNA / Ford Unknown_Position while parked with invalid CAN is not evidence of leaving Park.
    w = GearParkWriter()
    run(w, GearShifter.park, True, 0, 1)
    assert run(w, tesla_gear(7).gearShifter, False, 1, 600) == []
    assert w.value is True

  def test_unknown_on_valid_can_clears(self, events):
    w = GearParkWriter()
    run(w, GearShifter.park, True, 0, 1)
    assert run(w, GearShifter.unknown, True, 1, 1) == [False]

  def test_tesla_parked_sequence(self, events):
    w = GearParkWriter()
    writes = []
    writes += run(w, tesla_gear(1, frames=0).gearShifter, False, 0, 3)   # boot: nothing decoded yet
    writes += run(w, tesla_gear(1).gearShifter, True, 3, 60)             # Park, valid CAN
    writes += run(w, tesla_gear(4).gearShifter, True, 63, 60)            # Drive
    writes += run(w, tesla_gear(1).gearShifter, True, 123, 60)           # Park again
    assert writes == [True, False, True]

  def test_unconfirmed_park_is_logged_loudly_once(self, events):
    w = GearParkWriter()
    run(w, GearShifter.park, False, 0, UNCONFIRMED_LOG_S - 1)
    assert [e for e in events if e[0] == "gear_park_unconfirmed"] == []
    run(w, GearShifter.park, False, UNCONFIRMED_LOG_S - 1, 600)
    unconfirmed = [e for e in events if e[0] == "gear_park_unconfirmed"]
    assert len(unconfirmed) == 1 and unconfirmed[0][1]["error"] is True
    run(w, GearShifter.park, True, 700, 1)
    assert [e for e in events if e[0] == "gear_park_unconfirmed_cleared"] != []

  def test_car_that_never_decodes_a_gear_is_logged(self, events):
    w = GearParkWriter()
    assert run(w, GearShifter.unknown, True, 0, 120) == []
    assert len([e for e in events if e[0] == "gear_park_unconfirmed"]) == 1

  def test_driving_is_silent(self, events):
    w = GearParkWriter()
    run(w, GearShifter.drive, True, 0, 3600)
    assert [e for e in events if e[0].startswith("gear_park_unconfirmed")] == []


# ---------------------------------------------------------------- gear source capability (gearparkcan2pnw)

class TestGearSource:
  def test_quiet_camera_bus_park_confirms(self, events):
    # The charging case: powertrain bus alive, camera bus asleep -> canValid False for the whole session.
    cs, CI = lightning_quiet_can(0)
    assert cs.gearShifter == GearShifter.park and not cs.canValid
    assert parser_valid_now(CI.can_parsers["pt"]) and not parser_valid_now(CI.can_parsers["cam"])
    w = GearParkWriter()
    w.attach_gear_source(CI.can_parsers["pt"])
    assert run(w, cs.gearShifter, cs.canValid, 0, 3600, dt=0.5) == [True]
    assert [e for e in events if e[0] == "gear_park_unconfirmed"] == []
    assert [e[1]["gear_source_valid"] for e in events if e[0] == "gear_park"] == [True]

  def test_without_the_capability_the_quiet_camera_bus_still_records(self, events):
    cs, _ = lightning_quiet_can(0)
    w = GearParkWriter()                                          # never attached: today's rule
    assert run(w, cs.gearShifter, cs.canValid, 0, 3600, dt=0.5) == []
    assert len([e for e in events if e[0] == "gear_park_unconfirmed"]) == 1

  def test_park_with_the_rest_of_the_powertrain_bus_missing_does_not_set(self, events):
    cs, CI = lightning_quiet_can(0, rest_of_pt=False)             # only the gear frame: pt parser invalid
    assert cs.gearShifter == GearShifter.park and not parser_valid_now(CI.can_parsers["pt"])
    w = GearParkWriter()
    w.attach_gear_source(CI.can_parsers["pt"])
    assert run(w, cs.gearShifter, cs.canValid, 0, 3600, dt=0.5) == []
    unconfirmed = [e for e in events if e[0] == "gear_park_unconfirmed"]
    assert len(unconfirmed) == 1 and unconfirmed[0][1]["gear_source_valid"] is False

  def test_park_held_on_a_dead_powertrain_bus_does_not_set(self, events):
    # Park arrived while the rest of the bus was missing (so it could not set), then the bus died: the parser
    # keeps decoding that Park for as long as the truck stays asleep. It must never set GearPark.
    cs, CI = lightning_quiet_can(0, rest_of_pt=False)
    w = GearParkWriter()
    w.attach_gear_source(CI.can_parsers["pt"])
    t = cs_t = 2.0
    for i in range(3600 * 10):                                    # an hour of empty batches at 10 Hz
      cs = CI.update([(int((cs_t + i * 0.1) * 1e9), [])])
      assert cs.gearShifter == GearShifter.park                   # the held decode
      assert w.update(cs.gearShifter, cs.canValid, t + i * 0.1) is None
    assert w.value is False

  def test_gear_source_goes_invalid_within_the_bus_timeout_of_the_last_frame(self):
    cs, CI = lightning_quiet_can(0)
    pt = CI.can_parsers["pt"]
    assert parser_valid_now(pt)
    t_last = 2.0
    for dt in (0.05, 0.11, 1.0, 60.0):
      CI.update([(int((t_last + dt) * 1e9), [])])
      assert parser_valid_now(pt) is (dt < 0.1), dt              # 100 Hz messages: 10 missed frames

  def test_gear_source_is_invalid_on_a_silent_bus_before_rates_are_learned(self):
    # 0.5 s of frames: no message rate learned yet, so each message alone counts as alive for 10 s.
    cs, CI = lightning_quiet_can(0, seconds=0.5)
    pt = CI.can_parsers["pt"]
    assert parser_valid_now(pt)
    CI.update([(int(2.0 * 1e9), [])])                             # 1.5 s of silence
    assert all(st.valid(pt._last_update_nanos, False) for st in pt.message_states.values())
    assert not parser_valid_now(pt)

  def test_gear_source_is_invalid_with_a_failing_counter(self):
    from opendbc.can.parser import MAX_BAD_COUNTER
    cs, CI = lightning_quiet_can(0)
    pt = CI.can_parsers["pt"]
    st = next(iter(pt.message_states.values()))
    st.counter_fail = MAX_BAD_COUNTER                               # what CANParser counts for a bad COUNTER signal
    assert not parser_valid_now(pt)

  def test_gear_source_read_does_not_touch_the_parser(self):
    cs, CI = lightning_quiet_can(0, rest_of_pt=False)
    pt = CI.can_parsers["pt"]
    before = (pt.can_invalid_cnt, [st.counter_fail for st in pt.message_states.values()])
    for _ in range(10):
      parser_valid_now(pt)
    assert (pt.can_invalid_cnt, [st.counter_fail for st in pt.message_states.values()]) == before

  def test_drive_on_the_quiet_camera_bus_clears(self, events):
    cs, CI = lightning_quiet_can(0)
    w = GearParkWriter()
    w.attach_gear_source(CI.can_parsers["pt"])
    assert run(w, cs.gearShifter, cs.canValid, 0, 60) == [True]
    cs, _ = lightning_quiet_can(3)
    assert run(w, cs.gearShifter, cs.canValid, 60, 1) == [False]

  def test_tesla_is_identical(self, events):
    # The Tesla declares the capability. The attached writer differs from today's ONLY on a Park read with
    # canValid False while its whole chassis parser is valid. Everywhere else it writes exactly the same:
    # valid-CAN sessions, a never-received gear, SNA, and Park with the rest of the chassis bus missing.
    CI = _interface(TESLA)
    CI.update([(0, [])])
    packer = CANPacker("tesla_can")
    for i in range(200):                                          # DI_torque2 Park only, on the chassis bus
      addr, dat, b = packer.make_can_msg("DI_torque2", TESLA_CANBUS.chassis, {"DI_gear": 1, "DI_torque2Counter": i % 16})
      cs = CI.update([((i + 1) * 10_000_000, [CanData(addr, dat, b)])])
    source = CI.can_parsers[PnwVehicle(CI.CP).gear_source_bus]
    assert cs.gearShifter == GearShifter.park and not cs.canValid and not parser_valid_now(source)
    for sequence in (((1, True), (4, True), (1, True), (7, False), (1, True)), ((1, False),), (("live", False),)):
      plain, attached = GearParkWriter(), GearParkWriter()
      attached.attach_gear_source(source)
      writes_plain, writes_attached, t = [], [], 0.0
      for di_gear, valid in sequence:
        gear = cs.gearShifter if di_gear == "live" else tesla_gear(di_gear, frames=20 if valid else 0).gearShifter
        writes_plain += run(plain, gear, valid, t, 30)
        writes_attached += run(attached, gear, valid, t, 30)
        t += 30
      assert writes_plain == writes_attached

  def test_a_broken_gear_source_falls_back_to_todays_rule_and_says_so(self, events, monkeypatch):
    logged = []
    monkeypatch.setattr(cloudlog, "exception", lambda msg, *a, **kw: logged.append(msg))
    w = GearParkWriter()
    w.attach_gear_source(object())                                # no parser attributes at all
    assert run(w, GearShifter.park, False, 0, 600) == []
    assert len(logged) == 1
    assert run(w, GearShifter.park, True, 600, 1) == [True]


# ---------------------------------------------------------------- card call site

def _wait_param(params, key, expected, timeout=2.0):
  deadline = time.monotonic() + timeout
  while time.monotonic() < deadline:
    if params.get_bool(key) == expected:
      return True
    time.sleep(0.01)
  return params.get_bool(key) == expected


def _make_car(fp=TESLA, brand="tesla", can_parsers=None):
  from opendbc.car import structs
  from openpilot.selfdrive.car.card import Car
  cp = structs.CarParams.new_message()
  cp.brand = brand
  cp.carFingerprint = fp
  CI = SimpleNamespace(CP=cp, CC=None, CS=SimpleNamespace(secoc_key=None), can_parsers=can_parsers or {})
  c = Car(CI=CI, RI=SimpleNamespace())
  c.sm = SimpleNamespace(frame=1, all_checks=lambda _: True)   # skip the 50 s carParams publish
  c.rk = SimpleNamespace(remaining=0.0)
  return c


class TestCardCallSite:
  def test_card_seeds_false_at_startup(self):
    params = Params()
    params.put_bool("GearPark", True)            # stale True from a previous card
    _make_car()
    assert _wait_param(params, "GearPark", False)

  def test_card_attaches_the_capability_gear_source(self):
    parsers = {"pt": object(), "cam": object(), "chassis": object()}
    assert _make_car(LIGHTNING, "ford", parsers)._gear_park._gear_source is parsers["pt"]
    assert _make_car(TESLA, "tesla", parsers)._gear_park._gear_source is parsers["chassis"]
    assert _make_car("HYUNDAI_SONATA", "hyundai", parsers)._gear_park._gear_source is None

  def test_seed_precedes_can_wait_and_fingerprinting(self):
    # A card that hangs in "Waiting for CAN messages" or crash-loops in fingerprinting must already have
    # cleared a stale GearPark, or the recorder stays off for the drive.
    import inspect
    from openpilot.selfdrive.car.card import Car
    src = inspect.getsource(Car.__init__)
    assert 0 <= src.index('put_bool_nonblocking("GearPark", False)') < src.index("Waiting for CAN messages")

  def test_state_publish_writes_gear_park_from_capnp_carstate(self):
    params = Params()
    c = _make_car()
    assert _wait_param(params, "GearPark", False)
    CS = car.CarState.new_message()
    CS.canValid = False                        # Park on invalid CAN (e.g. Lightning dead-bus default)
    CS.gearShifter = GearShifter.park
    for _ in range(5):
      c.state_publish(CS, None)
    assert c._gear_park.value is False
    time.sleep(0.2)
    assert params.get_bool("GearPark") is False
    CS = car.CarState.new_message()
    CS.canValid = True
    CS.gearShifter = GearShifter.park
    c.state_publish(CS, None)
    assert _wait_param(params, "GearPark", True)
    CS = car.CarState.new_message()
    CS.canValid = False                        # CAN still invalid as the shifter leaves Park
    CS.gearShifter = GearShifter.drive
    c.state_publish(CS, None)
    assert _wait_param(params, "GearPark", False)
