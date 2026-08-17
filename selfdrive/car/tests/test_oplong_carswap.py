# oplongpersist2pnw (req 3b): op-long now persists across a SAME-car reboot (system/manager/manager.py
# no longer force-resets AlphaLongitudinalEnabled every boot — req 3a). The only remaining reset
# trigger is a genuine CAR CHANGE, handled in the SAME card.py hook that already resets calibration
# on a swap (_maybe_reset_calibration_on_car_change / car_changed_for_recal). The Tesla is exempt
# (its op-long is native — pnw_vehicle.PnwVehicle.op_long_native — never a per-session opt-in, so the
# param is never touched and no onroad cycle is ever requested for it).
from types import SimpleNamespace

from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from opendbc.car import structs
from openpilot.selfdrive.car.card import Car

LIGHTNING = "FORD_F_150_LIGHTNING_MK1"
TESLA = "TESLA_MODEL_S_HW3"


def _make_cp(brand, fingerprint, op_long=False, source="fw"):
  cp = structs.CarParams.new_message()
  cp.brand = brand
  cp.carFingerprint = fingerprint
  cp.fingerprintSource = source
  cp.openpilotLongitudinalControl = op_long
  return cp


def _run_card(cp):
  # CI provided -> card skips fingerprinting/CAN wait and runs only the persist/calswap logic under test
  CI = SimpleNamespace(CP=cp, CC=None, CS=SimpleNamespace(secoc_key=None))
  Car(CI=CI, RI=SimpleNamespace())


class TestOpLongResetOnCarChange:
  def test_tesla_to_lightning_swap_with_op_long_on_resets_and_cycles(self):
    # The concrete scenario req 3b targets: a Lightning left with op-long ON, then the device is
    # swapped onto it from the Tesla. self.CP.openpilotLongitudinalControl is ALREADY True for this
    # session (baked in at fingerprint time from the stale persisted param) -- the fix must both
    # write the param False AND request an onroad cycle so the swapped, uncalibrated car actually
    # comes up on stock ACC THIS session, not just next boot.
    with OpenpilotPrefix():
      params = Params()
      params.put("CalibrationCar", TESLA)
      params.put_bool("AlphaLongitudinalEnabled", True)
      _run_card(_make_cp("ford", LIGHTNING, op_long=True))
      assert params.get("AlphaLongitudinalEnabled") is not None
      assert params.get_bool("AlphaLongitudinalEnabled") is False
      assert params.get_bool("OnroadCycleRequested") is True
      # calibration reset still happens (this hook's original job) and the car tag is updated
      assert params.get("CalibrationCar") == LIGHTNING

  def test_lightning_to_tesla_swap_never_touches_param_or_cycles(self):
    # Native op-long car (Tesla): op_long_native short-circuits the reset entirely -- nothing to
    # disable, no cycle. Also proves the gate checks op_long_native, not just openpilotLongitudinalControl
    # (a native Tesla always reports openpilotLongitudinalControl=True).
    with OpenpilotPrefix():
      params = Params()
      params.put("CalibrationCar", LIGHTNING)
      params.put_bool("AlphaLongitudinalEnabled", True)
      _run_card(_make_cp("tesla", TESLA, op_long=True))
      assert params.get_bool("AlphaLongitudinalEnabled") is True  # untouched
      assert params.get_bool("OnroadCycleRequested") is False  # never requested
      assert params.get("CalibrationCar") == TESLA

  def test_swap_to_lightning_already_stock_acc_does_not_needlessly_cycle(self):
    # op-long was already OFF for the new car (openpilotLongitudinalControl=False) -- nothing to
    # disable, so no reload should be requested even though a real car change did occur.
    with OpenpilotPrefix():
      params = Params()
      params.put("CalibrationCar", TESLA)
      params.put_bool("AlphaLongitudinalEnabled", False)
      _run_card(_make_cp("ford", LIGHTNING, op_long=False))
      assert params.get_bool("AlphaLongitudinalEnabled") is False
      assert params.get_bool("OnroadCycleRequested") is False

  def test_same_car_reboot_persists_op_long_untouched(self):
    # req 3a's core promise: a same-car reboot (stored == cur) must not disturb op-long at all --
    # no car-change detected, so this hook's op-long branch never runs.
    with OpenpilotPrefix():
      params = Params()
      params.put("CalibrationCar", LIGHTNING)
      params.put_bool("AlphaLongitudinalEnabled", True)
      _run_card(_make_cp("ford", LIGHTNING, op_long=True))
      assert params.get_bool("AlphaLongitudinalEnabled") is True
      assert params.get_bool("OnroadCycleRequested") is False

  def test_first_boot_no_stored_tag_persists_op_long_untouched(self):
    # First boot / freshly-cleared CalibrationCar -> car_changed_for_recal is False (no stored tag),
    # so this is not treated as a car change and op-long is left exactly as persisted.
    with OpenpilotPrefix():
      params = Params()
      params.put_bool("AlphaLongitudinalEnabled", True)
      _run_card(_make_cp("ford", LIGHTNING, op_long=True))
      assert params.get_bool("AlphaLongitudinalEnabled") is True
      assert params.get_bool("OnroadCycleRequested") is False
      assert params.get("CalibrationCar") == LIGHTNING

  def test_no_cycle_loop_on_reinit_after_swap(self):
    # Simulates the onroad-cycle re-init: after the swap resets CalibrationCar to cur_car, a SECOND
    # card process construction for the SAME (now current) car must see stored == cur and must not
    # re-trigger the op-long reset / another cycle request.
    with OpenpilotPrefix():
      params = Params()
      params.put("CalibrationCar", TESLA)
      params.put_bool("AlphaLongitudinalEnabled", True)
      _run_card(_make_cp("ford", LIGHTNING, op_long=True))
      assert params.get_bool("OnroadCycleRequested") is True
      # simulate hardwared.py consuming the request (system/hardware/hardwared.py does this)
      params.put_bool("OnroadCycleRequested", False)
      # the re-fingerprint after the cycle: same car, and AlphaLongitudinalEnabled is now False so
      # the new session's CP correctly reports op_long=False this time.
      _run_card(_make_cp("ford", LIGHTNING, op_long=False))
      assert params.get_bool("OnroadCycleRequested") is False  # not re-armed
      assert params.get_bool("AlphaLongitudinalEnabled") is False
