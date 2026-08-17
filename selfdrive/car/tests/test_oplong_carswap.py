# oplongpersist2pnw (req 3b): op-long now persists across a SAME-car reboot (system/manager/manager.py
# no longer force-resets AlphaLongitudinalEnabled every boot — req 3a). The only remaining reset
# trigger is a genuine CAR CHANGE, handled in the SAME card.py hook that already resets calibration
# on a swap (_maybe_reset_calibration_on_car_change). The Tesla is exempt (its op-long is native —
# pnw_vehicle.PnwVehicle.op_long_native — never a per-session opt-in, so the param is never touched
# and no onroad cycle is ever requested for it).
#
# Fix 1 (Fable MEDIUM, review pass 2): the op-long reset runs under its OWN, broader swap test
# (car_swapped_for_oplong), decoupled from the calibration wipe's car_changed_for_recal — it does NOT
# exclude a fixed-source (fleet-VIN fallback) fingerprint, because the op-long reset is the FAIL-SAFE
# direction (worst case the driver re-enables it) while the calibration wipe is destructive and must
# stay excluded there (poisoned-cache incident). See TestOpLongResetUnderFixedSourceFingerprint.
import time
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


def _wait_for_calibration_car(params, expected, timeout=1.0):
  """Fix 3 (Fable, test robustness): CalibrationCar is written via params.put_nonblocking (an async
  detached thread) in card.py. A slow flush right after Car.__init__ returns can leave a reader --
  either this test's own assertion, or a SECOND _run_card() simulating the post-cycle re-init -- seeing
  the STALE tag, which for the re-init test would spuriously re-detect a "swap" and could flake the
  very no-loop property under test. Poll briefly instead of asserting/reading immediately."""
  deadline = time.monotonic() + timeout
  while time.monotonic() < deadline:
    if params.get("CalibrationCar") == expected:
      return
    time.sleep(0.01)


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
      _wait_for_calibration_car(params, LIGHTNING)
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
      _wait_for_calibration_car(params, TESLA)
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
      _wait_for_calibration_car(params, LIGHTNING)
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
      # Fix 3: make sure the async CalibrationCar write from the first _run_card has actually landed
      # before the "re-init" second _run_card reads it, or this would flake on exactly the race the
      # real bug would exploit.
      _wait_for_calibration_car(params, LIGHTNING)
      # the re-fingerprint after the cycle: same car, and AlphaLongitudinalEnabled is now False so
      # the new session's CP correctly reports op_long=False this time.
      _run_card(_make_cp("ford", LIGHTNING, op_long=False))
      assert params.get_bool("OnroadCycleRequested") is False  # not re-armed
      assert params.get_bool("AlphaLongitudinalEnabled") is False


class TestOpLongResetUnderFixedSourceFingerprint:
  """Fix 1 (Fable MEDIUM): a sleepy-bus power-up right after a real car swap can resolve only via the
  unreliable fleet-VIN fallback (fingerprintSource=fixed). The calibration wipe correctly stays
  fp_fixed-excluded (must never wipe a good calibration on a flaky read -- poisoned-cache incident),
  but the op-long reset is the FAIL-SAFE direction and must still fire, or the newly-swapped car would
  drive op-long ON on the OLD car's stale calibration -- exactly the failure this feature prevents."""

  def test_fixed_source_swap_with_op_long_on_resets_oplong_but_not_calibration(self):
    with OpenpilotPrefix():
      params = Params()
      params.put("CalibrationCar", TESLA)
      params.put_bool("AlphaLongitudinalEnabled", True)
      params.put("CalibrationParams", b"stale-tesla-calibration")
      _run_card(_make_cp("ford", LIGHTNING, op_long=True, source="fixed"))
      # op-long IS reset (fail-safe direction) even though the fingerprint is unreliable
      assert params.get_bool("AlphaLongitudinalEnabled") is False
      assert params.get_bool("OnroadCycleRequested") is True
      # the destructive calibration wipe stays gated on a RELIABLE fingerprint -- untouched here
      assert params.get("CalibrationParams") == b"stale-tesla-calibration"
      # the CalibrationCar tag likewise does not advance on an unreliable fingerprint
      assert params.get("CalibrationCar") == TESLA

  def test_fixed_source_swap_no_second_cycle_after_reset(self):
    # No re-trigger loop even under fp_fixed: after the cycle re-fingerprints (which may still
    # resolve fixed-source, e.g. the bus is still sleepy), the new session's CP already reports
    # op-long=False (get_car() read the just-written param) -- the op-long gate is then False, so no
    # second cycle fires even though car_swapped_for_oplong() would still evaluate True (the stored
    # tag never advanced under fp_fixed, so it's still a "different car" by that test).
    with OpenpilotPrefix():
      params = Params()
      params.put("CalibrationCar", TESLA)
      params.put_bool("AlphaLongitudinalEnabled", True)
      _run_card(_make_cp("ford", LIGHTNING, op_long=True, source="fixed"))
      assert params.get_bool("OnroadCycleRequested") is True
      params.put_bool("OnroadCycleRequested", False)  # simulate hardwared.py consuming it
      _run_card(_make_cp("ford", LIGHTNING, op_long=False, source="fixed"))
      assert params.get_bool("OnroadCycleRequested") is False  # not re-armed
      assert params.get_bool("AlphaLongitudinalEnabled") is False


class TestOpLongResetFailureRetry:
  """Fix B (Gemini F2, SHOULD-FIX, review pass 3): a failed AlphaLongitudinalEnabled write must not
  permanently skip the reset. card.py's Params (common.params_pyx.Params) is a Cython extension type
  and cannot be monkeypatched at the method level (confirmed: pytest's monkeypatch.setattr(Params,
  'put_bool', ...) raises "cannot set attribute of immutable type"), so these tests instead make
  PnwVehicle(self.CP) raise -- the EXACT SAME try/except block in
  _maybe_reset_calibration_on_car_change wraps both the PnwVehicle construction/capability read AND
  the AlphaLongitudinalEnabled write, so a raise from either exercises the identical
  oplong_reset_ok=False path and the identical CalibrationCar-gating consequence under test."""

  def test_reset_failure_blocks_calibration_car_advance_and_arms_retry(self, monkeypatch):
    with OpenpilotPrefix():
      params = Params()
      params.put("CalibrationCar", TESLA)
      params.put_bool("AlphaLongitudinalEnabled", True)

      def _boom(*_a, **_kw):
        raise RuntimeError("boom")

      monkeypatch.setattr("openpilot.selfdrive.car.card.PnwVehicle", _boom)
      _run_card(_make_cp("ford", LIGHTNING, op_long=True))
      # the reset attempt failed -- AlphaLongitudinalEnabled is UNCHANGED (still True: the write
      # never landed), and CalibrationCar must NOT advance so the swap is retried next boot.
      assert params.get_bool("AlphaLongitudinalEnabled") is True
      assert params.get_bool("OnroadCycleRequested") is False
      assert params.get("CalibrationCar") == TESLA  # stale on purpose -- retry armed

  def test_retry_after_failure_converges_once_the_write_succeeds(self, monkeypatch):
    with OpenpilotPrefix():
      params = Params()
      params.put("CalibrationCar", TESLA)
      params.put_bool("AlphaLongitudinalEnabled", True)

      def _boom(*_a, **_kw):
        raise RuntimeError("boom")

      monkeypatch.setattr("openpilot.selfdrive.car.card.PnwVehicle", _boom)
      _run_card(_make_cp("ford", LIGHTNING, op_long=True))
      assert params.get("CalibrationCar") == TESLA  # still stale after the failed attempt
      monkeypatch.undo()  # restore the real PnwVehicle for the retry pass below

      # NEXT boot: same swap is still detected (CalibrationCar never advanced) and this time the
      # reset attempt succeeds -- converges: param resets, cycle requested, CalibrationCar advances.
      _run_card(_make_cp("ford", LIGHTNING, op_long=True))
      assert params.get_bool("AlphaLongitudinalEnabled") is False
      assert params.get_bool("OnroadCycleRequested") is True
      _wait_for_calibration_car(params, LIGHTNING)
      assert params.get("CalibrationCar") == LIGHTNING

  def test_reset_success_advances_calibration_car(self):
    # Contrast case, explicit per review request: when the reset attempt succeeds (the normal path,
    # already covered implicitly by TestOpLongResetOnCarChange), CalibrationCar DOES advance on the
    # same boot that performed the reset -- no PnwVehicle patch here.
    with OpenpilotPrefix():
      params = Params()
      params.put("CalibrationCar", TESLA)
      params.put_bool("AlphaLongitudinalEnabled", True)
      _run_card(_make_cp("ford", LIGHTNING, op_long=True))
      assert params.get_bool("AlphaLongitudinalEnabled") is False
      _wait_for_calibration_car(params, LIGHTNING)
      assert params.get("CalibrationCar") == LIGHTNING
