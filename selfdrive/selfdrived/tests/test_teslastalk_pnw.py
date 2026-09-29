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


# --- teslastalk2b F2: a stalk push then a quick brake must not ARM steering-only --------------------------------

def _engaged():
  m = MadsPnw(MADS_ON)
  for _ in range(3):
    m.update(True, True, False, True, Events())
  return m


def _cancel_ev():
  ev = Events()
  ev.add(EventName.pedalPressed)
  ev.add(EventName.pcmDisable)
  return ev


def _push_cancel_brake(mads, brake_after: int, push_lead: int = 0):
  """Frame P: stalk push while engaged. Frame P+push_lead: the car's cancel (op disables). Brake lands `brake_after` frames later."""
  mads.update(True, True, False, True, Events(), stalk_press=True)
  for _ in range(push_lead):
    mads.update(True, True, False, True, Events())
  mads.update(False, False, brake_after == 0, False, _cancel_ev())
  for i in range(1, 46):
    mads.update(False, False, i >= brake_after, False, Events())
  return mads


@pytest.mark.parametrize("brake_after", [0, 5, 30, 44])
def test_push_then_quick_brake_does_not_arm_steering_only(brake_after):
  m = _push_cancel_brake(_engaged(), brake_after)
  assert (m.enabled, m.lateral_only) == (False, False)


def test_control_the_same_cancel_without_the_push_still_arms():
  """The mutation partner: without the stalk push the identical brake sequence arms (today's behaviour)."""
  m = _engaged()
  m.update(True, True, False, True, Events())
  m.update(False, False, True, False, _cancel_ev())
  assert m.lateral_only


def test_a_push_older_than_the_window_does_not_veto():
  from openpilot.selfdrive.selfdrived.mads_pnw import MADS_BRAKE_PANDA_GRACE_FRAMES
  m = _engaged()
  m.update(True, True, False, True, Events(), stalk_press=True)
  for _ in range(MADS_BRAKE_PANDA_GRACE_FRAMES):
    m.update(True, True, False, True, Events())
  m.update(False, False, True, False, _cancel_ev())
  assert m.lateral_only


def test_push_inside_the_window_vetoes_a_later_cancel_too():
  """A push that did not cancel at once, then a brake-induced cancel inside the window: the push said "off"."""
  m = _engaged()
  m.update(True, True, False, True, Events(), stalk_press=True)
  for _ in range(20):
    m.update(True, True, False, True, Events())
  m.update(False, False, True, False, _cancel_ev())
  assert not m.lateral_only


def test_a_car_that_never_passes_stalk_press_is_unchanged():
  """The Lightning path: selfdrived passes stalk_press only when the stalk capability is set (see the source test)."""
  m = _engaged()
  m.update(True, True, False, True, Events(), stalk_press=False)
  m.update(False, False, True, False, _cancel_ev())
  assert m.lateral_only


def test_selfdrived_passes_stalk_press_only_for_the_stalk_capability():
  import inspect
  import textwrap
  step = textwrap.dedent(inspect.getsource(SelfdriveD.step))
  upd = textwrap.dedent(inspect.getsource(SelfdriveD.update_events))
  init = textwrap.dedent(inspect.getsource(SelfdriveD.__init__))
  assert "stalk_press=self.stalk_press_now" in step
  assert "self.stalk_press_now = bool(main_press and self.stalk_off_ignores_gas)" in upd    # Lightning: capability False -> never True
  assert "self.stalk_press_now = False" in init                                              # defined before the first frame


def test_selfdrived_step_has_no_undefined_names():
  """The first version of this wiring named `main_press` (a local of update_events) inside step(): a NameError on the first
  frame that only a source-substring test could miss. Pyflakes-level check on the two functions."""
  import subprocess
  import sys
  r = subprocess.run([sys.executable, "-m", "ruff", "check", "--select", "F821", "--no-cache",
                      inspect_file(SelfdriveD)], capture_output=True, text=True)
  assert r.returncode == 0, r.stdout + r.stderr


def inspect_file(cls) -> str:
  import inspect
  return inspect.getsourcefile(cls)
