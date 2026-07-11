"""Capability-view tests — PnwVehicle maps cars to features (no fingerprint checks in feature code)."""
from openpilot.selfdrive.controls.lib.pnw_vehicle import PnwVehicle


class FakeCP:
  def __init__(self, fp="", brand="", op_long=False):
    self.carFingerprint = fp
    self.brand = brand
    self.openpilotLongitudinalControl = op_long


def test_lightning_stock_acc():
  v = PnwVehicle(FakeCP("FORD_F_150_LIGHTNING_MK1", "ford", op_long=False))
  assert v.stock_acc_buttons and v.ces_shadow and v.ces_capable and v.nudgeless and not v.op_long


def test_lightning_with_alpha_long():
  v = PnwVehicle(FakeCP("FORD_F_150_LIGHTNING_MK1", "ford", op_long=True))
  assert v.op_long and not v.ces_shadow and v.ces_capable  # planner actuates, ICBM inert


def test_tesla_raven():
  v = PnwVehicle(FakeCP("TESLA_MODEL_S_HW3", "tesla", op_long=True))
  assert v.op_long and not v.stock_acc_buttons and not v.ces_shadow and v.ces_capable and v.nudgeless


def test_unknown_car_and_none():
  v = PnwVehicle(FakeCP("SOME_OTHER_CAR", "hyundai", op_long=False))
  assert not (v.ces_shadow or v.ces_capable or v.nudgeless or v.stock_acc_buttons)
  n = PnwVehicle(None)
  assert not (n.op_long or n.ces_shadow or n.ces_capable or n.nudgeless)
