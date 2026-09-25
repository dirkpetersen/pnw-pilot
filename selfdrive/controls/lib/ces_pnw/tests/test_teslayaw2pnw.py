"""teslayaw2pnw, the ces_pnw half: kPeak's CAN-achieved curvature comes from carState.yawRate only on a car that has
a yaw sensor (PnwVehicle.yaw_rate_source). Driven through the REAL CESController.experimental_request at 100 Hz, via
test_curvedbtel2pnw's _drive harness, so the wiring is tested and not just a helper."""
import pytest

from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_curvedbtel2pnw import _drive

RAVEN = "TESLA_MODEL_S_HW3"
UNVERIFIED_TESLA = "TESLA_MODEL_S_HW2"   # its carstate never assigns yawRate


def test_the_raven_now_has_a_can_achieved_curvature(monkeypatch, tmp_path):
  """Before teslayaw2pnw kPeak on the Raven could only ever be the commanded half."""
  recs = _drive(monkeypatch, tmp_path, RAVEN, "tesla", True, yaw_of=lambda i: 0.25, cmd_of=lambda i: 0.0)
  assert recs, "no tick/adopt records -- the harness, not the feature, is broken"
  assert recs[-1]["kPeak"] == pytest.approx(0.25 / 25.0, rel=0.02)


def test_a_car_without_a_yaw_sensor_never_feeds_its_default_in(monkeypatch, tmp_path):
  """Whatever sits in yawRate on a car with no sensor is not a measurement. With nothing commanded either, the
  window saw NO curvature at all -> null, not a number made from the placeholder."""
  recs = _drive(monkeypatch, tmp_path, UNVERIFIED_TESLA, "tesla", True, yaw_of=lambda i: 0.25,
                cmd_of=lambda i: 0.0)
  assert recs, "no tick/adopt records -- the harness, not the feature, is broken"
  last = recs[-1]
  assert last["kPeakN"] > 50, "the accumulator must still run -- only the CAN half is withheld"
  assert last["kPeak"] is None


def test_without_a_sensor_the_commanded_half_still_counts(monkeypatch, tmp_path):
  recs = _drive(monkeypatch, tmp_path, UNVERIFIED_TESLA, "tesla", True, yaw_of=lambda i: 0.25,
                cmd_of=lambda i: -0.004)
  assert recs[-1]["kPeak"] == pytest.approx(0.004, rel=0.02)
