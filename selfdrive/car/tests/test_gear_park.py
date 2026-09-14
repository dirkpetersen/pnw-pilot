"""parknorec2pnw: GearPark writer rules, the real per-car gear decode they depend on, and card's call site.

GearPark now stops loggerd (system/manager/park_record_gate.py), so these are the facts that decide
whether a drive gets recorded:
  - the Tesla Raven reports Park through the real opendbc decode, on the chassis bus,
  - a Lightning whose gear message was NEVER received also reads `park` (parser zero-init + TrnRng 0 =
    "Park"), which is why only valid CAN may SET GearPark,
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
from openpilot.selfdrive.car.gear_park import GearParkWriter, UNCONFIRMED_LOG_S

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

  def test_lightning_never_received_reads_park(self):
    # The trap: parser signals start at 0 and TrnRng_D_Rq 0 = "Park". A dead powertrain bus reads Park.
    cs = lightning_gear(3, frames=0)
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

  def test_lightning_dead_bus_default_park_never_sets(self, events):
    w = GearParkWriter()
    gear = lightning_gear(3, frames=0).gearShifter              # real decode: reads park, nothing received
    assert run(w, gear, False, 0, 3600, dt=0.5) == []
    assert w.value is False

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


# ---------------------------------------------------------------- card call site

def _wait_param(params, key, expected, timeout=2.0):
  deadline = time.monotonic() + timeout
  while time.monotonic() < deadline:
    if params.get_bool(key) == expected:
      return True
    time.sleep(0.01)
  return params.get_bool(key) == expected


def _make_car():
  from opendbc.car import structs
  from openpilot.selfdrive.car.card import Car
  cp = structs.CarParams.new_message()
  cp.brand = "tesla"
  cp.carFingerprint = TESLA
  CI = SimpleNamespace(CP=cp, CC=None, CS=SimpleNamespace(secoc_key=None))
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
