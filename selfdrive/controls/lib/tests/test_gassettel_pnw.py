"""gassettel2pnw: the gas-set record carries the speed the driver actually chose.

The delivered shortfall -- the number the whole gas-set feature is judged by -- is `vLift` minus the set
speed the `verify` record comes back with. Until this commit `vLift` was nowhere in the log, so every
measurement of it had to be reconstructed from qlogs, which is what held the 2026-09-15 analysis to n=5
(drives/2026-09-15/gasset-regen-loss/DRIVE_REPORT.md §0). `vGasMax` -- the peak speed of the accelerator
press -- was already tracked for the creep rule and likewise never emitted.

Pure telemetry: no gate reads either field, and no timing changes.
"""

import math

import pytest

from openpilot.selfdrive.controls.lib import madsresume_pnw as M
from openpilot.selfdrive.controls.lib.tests.test_gassetwait_pnw import ON_THE_POWER_MS2, _lift_off_at
from openpilot.selfdrive.controls.lib.tests.test_madsresume_pnw import (
  STEER_ONLY, Drive, _red_light, mk, normal_brake_and_resume,
)


def _fire(d):
  fire = [r for r in d.records if r["phase"] == "fire"]
  assert fire, d.phases()
  return fire[0]


def test_the_record_carries_the_speed_the_driver_lifted_off_at():
  d, _ = _lift_off_at(ON_THE_POWER_MS2, v=16.0)
  assert _fire(d)["vLift"] == pytest.approx(16.0)


def test_vLift_is_the_LIFT_OFF_speed_not_the_speed_at_the_fire():
  """They differ by exactly what the feature loses to regen, so conflating them would make the shortfall
  read as zero -- the measurement would silently confirm itself."""
  d = Drive()
  _red_light(d, 5.0)
  d.tick(200, gas_pressed=True, v_ego=16.0, a_ego=ON_THE_POWER_MS2, **STEER_ONLY)
  d.tick(1, v_ego=16.0, a_ego=ON_THE_POWER_MS2, **STEER_ONLY)        # lift-off at 16.0
  d.tick(400, v_ego=14.5, **STEER_ONLY)                              # regen has taken 1.5 m/s by the fire
  rec = _fire(d)
  assert rec["vLift"] == pytest.approx(16.0), "vLift must be the lift-off speed"
  assert rec["vEgo"] == pytest.approx(14.5), "precondition: the fire happened at the lower speed"


def test_the_record_carries_the_PEAK_speed_of_the_press_which_can_exceed_the_lift_off_speed():
  """The driver eased off before lifting: peak 18.0, lift-off 16.0. A shortfall measured against the peak
  would look 2 m/s worse than the driver's own intent, so the log has to carry both."""
  d = Drive()
  _red_light(d, 5.0)
  d.tick(100, gas_pressed=True, v_ego=18.0, a_ego=ON_THE_POWER_MS2, **STEER_ONLY)
  d.tick(50, gas_pressed=True, v_ego=16.0, a_ego=ON_THE_POWER_MS2, **STEER_ONLY)
  d.tick(1, v_ego=16.0, a_ego=ON_THE_POWER_MS2, **STEER_ONLY)
  d.tick(400, v_ego=16.0, **STEER_ONLY)
  rec = _fire(d)
  assert rec["vGasMax"] == pytest.approx(18.0), rec
  assert rec["vLift"] == pytest.approx(16.0), rec


def test_an_unreadable_speed_is_logged_as_null_not_as_zero():
  """Rule 2: a real 0 mph and a failed read must not look alike in the log."""
  d = Drive()
  _red_light(d, 5.0)
  d.tick(200, gas_pressed=True, v_ego=16.0, a_ego=ON_THE_POWER_MS2, **STEER_ONLY)
  d.tick(1, v_ego=float("nan"), a_ego=ON_THE_POWER_MS2, **STEER_ONLY)   # lift-off frame, speed unreadable
  d.tick(400, v_ego=16.0, **STEER_ONLY)
  rec = _fire(d)
  assert rec["vLift"] is None, rec


def test_vGasMax_is_null_not_zero_when_no_press_speed_was_ever_recorded():
  """Fable's extra mutant X2: the commit's Rule 2 claim was pinned for vLift but not for vGasMax, so
  `else 0.0` survived. A press whose speed was never recorded and a real standstill must not look alike.

  White-box, like the wait-state tests in test_gassetwait_pnw.py: the state is reachable on the truck only
  when every v_ego during the press was non-finite, which no end-to-end scenario can stage without tripping
  the speed gates first. The record-building rule is what matters, so it is pinned directly."""
  b = M.MadsResumeBrain()
  b._used_gas = True
  b._gas_v_max = 0.0
  rec = b._snap(mk(0.0))
  assert rec["mode"] == "set", rec
  assert rec["vGasMax"] is None, rec

  b._gas_v_max = 12.0
  assert b._snap(mk(0.0))["vGasMax"] == pytest.approx(12.0), "a real peak must still be reported"


def test_an_unreadable_ACCELERATION_does_not_stop_the_speed_being_latched():
  """Fable's extra mutant X3: guarding the vLift latch on a_ego rather than v_ego survived. The two are
  independent readings -- a lift-off whose acceleration could not be read still has a perfectly good speed,
  and that speed is the one number the measurement needs."""
  d = Drive()
  _red_light(d, 5.0)
  d.tick(200, gas_pressed=True, v_ego=16.0, a_ego=float("nan"), **STEER_ONLY)
  d.tick(1, v_ego=16.0, a_ego=float("nan"), **STEER_ONLY)
  d.tick(400, v_ego=16.0, **STEER_ONLY)
  rec = _fire(d)
  assert rec["a0"] is None and rec["waitWhy"] == "accelUnknown", rec
  assert rec["vLift"] == pytest.approx(16.0), rec


@pytest.mark.parametrize("a_ego", [ON_THE_POWER_MS2, float("nan")])
def test_a_RESUME_record_carries_neither_field(a_ego):
  """Same rule as the wait fields: a resume's speed does not come from an accelerator press, so stamping
  it with one would be a false record. An absent field cannot lie."""
  r = normal_brake_and_resume(a_ego=a_ego)
  res = [rec for rec in r.records if rec.get("mode") == "res"]
  assert res, [rec.get("mode") for rec in r.records]
  for rec in res:
    assert "vLift" not in rec and "vGasMax" not in rec, rec


def test_the_fields_do_not_change_any_timing():
  """Pure telemetry. The fire lands exactly where gassetwait2pnw put it."""
  d, lift = _lift_off_at(ON_THE_POWER_MS2)
  assert d.offers, "precondition"
  delay = d.offers[0][0] - lift
  assert M.GAS_SET_RELEASE_MIN_ACCEL_S - 1e-9 <= delay <= M.GAS_SET_RELEASE_MIN_ACCEL_S + 0.02, delay


def test_the_latched_speed_does_not_survive_its_episode():
  d, _ = _lift_off_at(ON_THE_POWER_MS2, v=16.0)
  assert d.b._gas_v_lift == pytest.approx(16.0), "precondition"
  d.b._disarm()
  assert math.isnan(d.b._gas_v_lift)
