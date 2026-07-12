"""Capability-view tests — PnwVehicle maps cars to features (no fingerprint checks in feature code)."""
import json

from openpilot.selfdrive.controls.lib import pnw_vehicle as pv
from openpilot.selfdrive.controls.lib.pnw_vehicle import PnwVehicle

MPH = 0.44704
LIGHTNING = "FORD_F_150_LIGHTNING_MK1"


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


# ---- curveslow-lightning: per-car curve-speed penalty ---------------------------------------------
def test_penalty_tesla_is_zero_at_all_speeds():
  v = PnwVehicle(FakeCP("TESLA_MODEL_S_HW3", "tesla", op_long=True))
  assert not v.lightning_curve_slow
  for mph in (10, 25, 40, 55, 60, 80):
    assert v.curve_speed_penalty_ms(mph * MPH) == 0.0


def test_penalty_other_cars_zero():
  assert PnwVehicle(FakeCP("SOME_OTHER_CAR", "hyundai")).curve_speed_penalty_ms(20 * MPH) == 0.0
  assert PnwVehicle(None).curve_speed_penalty_ms(20 * MPH) == 0.0


def test_penalty_lightning_ramp_endpoints_and_clamps(tmp_path, monkeypatch):
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(tmp_path / "nope.json"))  # force defaults
  v = PnwVehicle(FakeCP(LIGHTNING, "ford", op_long=False))
  assert v.lightning_curve_slow
  # penalty grows WITH target speed: at/above high_v (65 mph) -> ~10 mph; at/below low_v (30) -> ~1.5
  assert abs(v.curve_speed_penalty_ms(70 * MPH) / MPH - 10.0) < 1e-6    # fast sweeper -> full penalty
  assert abs(v.curve_speed_penalty_ms(65 * MPH) / MPH - 10.0) < 1e-6
  assert abs(v.curve_speed_penalty_ms(80 * MPH) / MPH - 10.0) < 1e-6    # above high clamps to max
  assert abs(v.curve_speed_penalty_ms(30 * MPH) / MPH - 1.5) < 1e-6
  assert abs(v.curve_speed_penalty_ms(25 * MPH) / MPH - 1.5) < 1e-6     # slow tight corner -> min
  assert abs(v.curve_speed_penalty_ms(10 * MPH) / MPH - 1.5) < 1e-6     # below low clamps to min
  # midpoint (47.5 mph) -> ~5.75 mph
  assert abs(v.curve_speed_penalty_ms(47.5 * MPH) / MPH - 5.75) < 1e-3


def test_penalty_lightning_monotonic_and_nonnegative(tmp_path, monkeypatch):
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(tmp_path / "nope.json"))
  v = PnwVehicle(FakeCP(LIGHTNING, "ford"))
  prev = -1.0
  for mph in range(15, 80):
    p = v.curve_speed_penalty_ms(mph * MPH)
    assert p >= 0.0                       # never negative -> can never invert to a speed-up
    assert p >= prev - 1e-9               # faster curve -> >= penalty (monotonic non-decreasing in speed)
    prev = p


def test_curve_json_override(tmp_path, monkeypatch):
  cfg = tmp_path / "curve.json"
  cfg.write_text(json.dumps({"lightning": {"penalty_min_mph": 3, "penalty_max_mph": 12,
                                           "low_v_mph": 25, "high_v_mph": 70}}))
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(cfg))
  v = PnwVehicle(FakeCP(LIGHTNING, "ford"))
  assert abs(v.curve_speed_penalty_ms(25 * MPH) / MPH - 3.0) < 1e-6     # at/below low_v -> min
  assert abs(v.curve_speed_penalty_ms(70 * MPH) / MPH - 12.0) < 1e-6    # at/above high_v -> max


def test_curve_json_out_of_bounds_clamped(tmp_path, monkeypatch):
  cfg = tmp_path / "curve.json"
  cfg.write_text(json.dumps({"lightning": {"penalty_min_mph": -5, "penalty_max_mph": 999,
                                           "low_v_mph": 2, "high_v_mph": 500}}))
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(cfg))
  v = PnwVehicle(FakeCP(LIGHTNING, "ford"))
  # penalties clamp to [0,15] -> never a speed-up, never absurd; speeds clamp to [10,80]
  assert v.curve_speed_penalty_ms(80 * MPH) / MPH <= 15.0 + 1e-6
  assert v.curve_speed_penalty_ms(10 * MPH) >= 0.0


def test_curve_json_malformed_falls_back_to_defaults(tmp_path, monkeypatch):
  cfg = tmp_path / "curve.json"
  cfg.write_text("{ this is not json ")
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(cfg))
  v = PnwVehicle(FakeCP(LIGHTNING, "ford"))
  assert abs(v.curve_speed_penalty_ms(65 * MPH) / MPH - 10.0) < 1e-6   # default max (fast)
  assert abs(v.curve_speed_penalty_ms(30 * MPH) / MPH - 1.5) < 1e-6    # default min (slow)


def test_curve_json_nan_ignored(tmp_path, monkeypatch):
  cfg = tmp_path / "curve.json"
  cfg.write_text('{"lightning": {"penalty_max_mph": NaN}}')   # json allows NaN; must be rejected
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(cfg))
  v = PnwVehicle(FakeCP(LIGHTNING, "ford"))
  assert abs(v.curve_speed_penalty_ms(65 * MPH) / MPH - 10.0) < 1e-6   # NaN dropped -> default max
