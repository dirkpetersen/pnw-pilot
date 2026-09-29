"""teslastalk2pnw: a Raven stalk push while steering-only ends EVERYTHING (the owner's "cruise button = master" rule).

The whole chain, with real pieces: hand-built 0x45 frames -> real DBC/CANParser/Tesla CarInterface.update ->
carState.buttonEvents -> the same mainCruise test selfdrived runs -> off_request_latches -> MadsPnw.update(off_requested).
The stalk codes and their effects are proven from 166 rlogs in drives/2026-09-28/tesla-stalk-decode/DRIVE_REPORT.md.
"""
import ast

import pytest

from cereal import log
from opendbc.car import ButtonType, gen_empty_fingerprint
from opendbc.car.can_definitions import CanData
from opendbc.car.car_helpers import interfaces
from opendbc.car.tesla.values import CANBUS, CAR
from opendbc.safety import ALTERNATIVE_EXPERIENCE

from openpilot.selfdrive.controls.lib.pnw_vehicle import PnwVehicle
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.selfdrive.selfdrived.mads_pnw import MadsPnw, off_request_gas_input, off_request_latches
from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD

EventName = log.OnroadEvent.EventName
MADS_ON = ALTERNATIVE_EXPERIENCE.ENABLE_MADS
DT = 10_000_000


@pytest.fixture
def canbus(monkeypatch):
  for k in ("party", "vehicle", "radar", "autopilot_party", "powertrain", "chassis", "autopilot_powertrain"):
    monkeypatch.setattr(CANBUS, k, getattr(CANBUS, k))


class Raven:
  def __init__(self, platform=CAR.TESLA_MODEL_S_HW3):
    CarInterface = interfaces[platform]
    self.CP = CarInterface.get_params(platform, gen_empty_fingerprint(), [], False, False, False)
    self.CI = CarInterface(self.CP)
    self.bus = self.CI.can_parsers["chassis"].bus
    self.t = 0
    self.CI.update([(self.t, [])])

  def stalk(self, code: int):
    self.t += DT
    return self.CI.update([(self.t, [CanData(0x45, bytes([0x40 | code, 0, 0, 0x30, 0, 0, 0, 0]), self.bus)])])


def main_press(cs) -> bool:
  """The exact expression selfdrived.step uses."""
  return any(be.pressed and be.type == ButtonType.mainCruise for be in cs.buttonEvents)


def lateral_only_mads() -> MadsPnw:
  m = MadsPnw(MADS_ON)
  for _ in range(3):
    m.update(True, True, False, True, Events())
  ev = Events()
  ev.add(EventName.pedalPressed)
  ev.add(EventName.pcmDisable)
  m.update(False, False, True, False, ev)     # a brake press: cruise -> STANDBY, lateral survives
  assert m.lateral_only
  return m


def test_stalk_push_ends_steering_only_including_lateral(canbus):
  car, mads = Raven(), lateral_only_mads()
  car.stalk(0)
  cs = car.stalk(1)                                           # FWD: the push
  assert main_press(cs)
  assert off_request_latches(main_press(cs), mads.lateral_only, off_request_gas_input(False, True))
  mads.update(False, False, False, False, Events(), cruise_available=True, off_requested=True)
  assert (mads.enabled, mads.active, mads.lateral_only) == (False, False, False)


def test_a_push_with_the_foot_on_the_accelerator_still_works_on_the_raven(canbus):
  """The onoffgas2pnw gate is for a thumb; a stalk push must not be swallowed by a resting foot."""
  car, mads = Raven(), lateral_only_mads()
  car.stalk(0)
  cs = car.stalk(1)
  assert off_request_latches(main_press(cs), mads.lateral_only, off_request_gas_input(True, True))


def test_the_lightning_wheel_button_keeps_its_accelerator_gate():
  """Nothing changes on the Lightning: its mainCruise press with the pedal down is still ignored."""
  assert not off_request_latches(True, True, off_request_gas_input(True, False))
  assert off_request_latches(True, True, off_request_gas_input(False, False))


def test_a_stalk_push_while_fully_engaged_is_not_an_off_request(canbus):
  """Engaged behaves exactly as today: the press latches nothing (lateral_only is False), the car's own cancel does the work."""
  car = Raven()
  mads = MadsPnw(MADS_ON)
  for _ in range(3):
    mads.update(True, True, False, True, Events())
  assert not mads.lateral_only
  car.stalk(0)
  assert main_press(car.stalk(1))
  assert not off_request_latches(True, mads.lateral_only, off_request_gas_input(False, True))


def test_the_pull_is_a_resume_press_that_cancels_a_pending_off_request(canbus):
  """selfdrived clears the latch on resumeCruise: push then quickly pull must not leave openpilot locked out for 3 s."""
  car = Raven()
  car.stalk(0)
  cs = car.stalk(2)
  assert [(b.type, b.pressed) for b in cs.buttonEvents] == [(ButtonType.resumeCruise, True)]


def test_selfdrived_clears_the_latch_on_resume_and_uses_the_gas_helper():
  import inspect
  import textwrap
  src = textwrap.dedent(inspect.getsource(SelfdriveD.update_events))
  tree = ast.parse(src)
  calls = [ast.unparse(n) for n in ast.walk(tree) if isinstance(n, ast.Call) and ast.unparse(n.func) == "off_request_latches"]
  assert calls == ["off_request_latches(main_press, self.mads.lateral_only, off_request_gas_input(CS.gasPressed, self.stalk_off_ignores_gas))"]
  assert "ButtonType.resumeCruise" in src and "self.off_request_t = 0.0" in src


def test_capability_only_on_the_raven():
  class CP:
    def __init__(self, fp):
      self.carFingerprint = fp
      self.brand = "tesla" if fp.startswith("TESLA") else "ford"
      self.openpilotLongitudinalControl = False
  assert PnwVehicle(CP("TESLA_MODEL_S_HW3")).stalk_cruise_buttons
  assert PnwVehicle(CP("TESLA_MODEL_S_HW3")).eps_refusal_alert
  for fp in ("FORD_F_150_LIGHTNING_MK1", "TESLA_MODEL_Y", "TESLA_MODEL_S_HW1"):
    assert not PnwVehicle(CP(fp)).stalk_cruise_buttons
    assert not PnwVehicle(CP(fp)).eps_refusal_alert


def test_selfdrived_takes_the_gas_exemption_from_the_capability_not_a_fingerprint():
  import inspect
  import textwrap
  src = textwrap.dedent(inspect.getsource(SelfdriveD.__init__))
  assert "self.stalk_off_ignores_gas = bool(veh.stalk_cruise_buttons)" in src
  assert "TESLA" not in src.split("stalk_off_ignores_gas")[1].split("\n")[0]
