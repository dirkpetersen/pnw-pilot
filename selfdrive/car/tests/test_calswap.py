# calswap2pnw: force a full recalibration when the device is moved to a different car (driver report
# 2026-07-13: truck drove weird after a Tesla->Lightning swap until a manual Reset Calibration).
from openpilot.selfdrive.car.card import car_changed_for_recal, car_swapped_for_oplong

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


# oplongpersist2pnw (req 3b, Fix 1): car_swapped_for_oplong is the BROADER sibling test used to gate
# the op-long reset -- same shape as car_changed_for_recal but with NO fp_fixed parameter at all, so
# it can never exclude on it. The two functions must agree everywhere fp_fixed=False (both gate the
# same underlying "different real car" condition), and car_swapped_for_oplong must additionally fire
# where car_changed_for_recal would have returned False solely because fp_fixed=True.


def test_swap_matches_recal_when_not_fixed():
  assert car_swapped_for_oplong(TESLA, LIGHTNING, "ford")
  assert car_swapped_for_oplong(LIGHTNING, TESLA, "tesla")


def test_swap_same_car_never_triggers():
  assert not car_swapped_for_oplong(LIGHTNING, LIGHTNING, "ford")
  assert not car_swapped_for_oplong(TESLA, TESLA, "tesla")


def test_swap_first_boot_no_stored_tag_never_triggers():
  assert not car_swapped_for_oplong("", LIGHTNING, "ford")


def test_swap_mock_never_triggers():
  assert not car_swapped_for_oplong(LIGHTNING, "MOCK", "mock")
  assert not car_swapped_for_oplong(LIGHTNING, "", "mock")


def test_swap_empty_current_car_never_triggers():
  assert not car_swapped_for_oplong(TESLA, "", "ford")


def test_swap_fires_even_when_fingerprint_is_fixed_source():
  # THE regression this decoupling fixes: car_changed_for_recal would say False here (fp_fixed=True),
  # but car_swapped_for_oplong has no such exclusion -- it must still fire so op-long resets on a
  # swap detected only via the unreliable fleet-VIN fallback.
  assert car_swapped_for_oplong(TESLA, LIGHTNING, "ford")
  assert not car_changed_for_recal(TESLA, LIGHTNING, "ford", fp_fixed=True)  # contrast: calibration stays untouched
