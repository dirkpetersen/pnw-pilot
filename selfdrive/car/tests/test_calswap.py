# calswap2pnw: force a full recalibration when the device is moved to a different car (driver report
# 2026-07-13: truck drove weird after a Tesla->Lightning swap until a manual Reset Calibration).
from openpilot.selfdrive.car.card import car_changed_for_recal

LIGHTNING = "FORD_F_150_LIGHTNING_MK1"
TESLA = "TESLA_MODEL_S_HW3"


def test_swap_between_two_real_cars_triggers_reset():
  assert car_changed_for_recal(TESLA, LIGHTNING, "ford", fp_fixed=False)
  assert car_changed_for_recal(LIGHTNING, TESLA, "tesla", fp_fixed=False)


def test_same_car_never_resets():
  assert not car_changed_for_recal(LIGHTNING, LIGHTNING, "ford", fp_fixed=False)
  assert not car_changed_for_recal(TESLA, TESLA, "tesla", fp_fixed=False)


def test_first_boot_no_stored_tag_never_resets():
  # empty stored tag = first boot (or freshly cleared) -> record the car, never wipe a good calibration
  assert not car_changed_for_recal("", LIGHTNING, "ford", fp_fixed=False)


def test_mock_never_resets():
  # a MOCK/dashcam read must never wipe the real car's calibration (slow-fingerprint boots happen)
  assert not car_changed_for_recal(LIGHTNING, "MOCK", "mock", fp_fixed=False)
  assert not car_changed_for_recal(LIGHTNING, "", "mock", fp_fixed=False)


def test_fixed_source_fingerprint_never_resets():
  # the fleet-VIN fallback already failed FW match (poisoned-cache incident) — a bad one must not
  # be trusted to make a persistent reset decision, even if the string differs.
  assert not car_changed_for_recal(TESLA, LIGHTNING, "ford", fp_fixed=True)


def test_empty_current_car_never_resets():
  assert not car_changed_for_recal(TESLA, "", "ford", fp_fixed=False)
