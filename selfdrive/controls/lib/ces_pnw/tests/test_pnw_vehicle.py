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


def test_penalty_lightning_hump_shape(tmp_path, monkeypatch):
  """Driver-calibrated 2026-07-11 iteration 3: a HUMP — small penalty on slow corners, full cut in
  the mid-speed washout zone (45-62), tapering off for long fast gentle sweepers."""
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(tmp_path / "nope.json"))  # force defaults
  v = PnwVehicle(FakeCP(LIGHTNING, "ford", op_long=False))
  assert v.lightning_curve_slow
  assert abs(v.curve_speed_penalty_ms(20 * MPH) / MPH - 1.0) < 1e-6     # slow corner -> min
  assert abs(v.curve_speed_penalty_ms(30 * MPH) / MPH - 1.0) < 1e-6
  assert abs(v.curve_speed_penalty_ms(45 * MPH) / MPH - 5.0) < 1e-6     # peak plateau start
  assert abs(v.curve_speed_penalty_ms(55 * MPH) / MPH - 5.0) < 1e-6     # the washout zone
  assert abs(v.curve_speed_penalty_ms(62 * MPH) / MPH - 5.0) < 1e-6     # peak plateau end
  assert abs(v.curve_speed_penalty_ms(75 * MPH) / MPH - 1.5) < 1e-6     # fast sweeper -> taper
  assert abs(v.curve_speed_penalty_ms(85 * MPH) / MPH - 1.5) < 1e-6     # beyond taper stays small
  # rises between 30 and 45; falls between 62 and 75
  assert v.curve_speed_penalty_ms(40 * MPH) < v.curve_speed_penalty_ms(45 * MPH)
  assert v.curve_speed_penalty_ms(68 * MPH) < v.curve_speed_penalty_ms(62 * MPH)
  assert v.curve_speed_penalty_ms(68 * MPH) > v.curve_speed_penalty_ms(75 * MPH)


def test_penalty_lightning_nonnegative_everywhere(tmp_path, monkeypatch):
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(tmp_path / "nope.json"))
  v = PnwVehicle(FakeCP(LIGHTNING, "ford"))
  for mph in range(5, 100):
    assert v.curve_speed_penalty_ms(mph * MPH) >= 0.0   # never a speed-up


def test_curve_json_override(tmp_path, monkeypatch):
  cfg = tmp_path / "curve.json"
  cfg.write_text(json.dumps({"lightning": {"penalty_min_mph": 3, "penalty_max_mph": 12,
                                           "penalty_taper_mph": 2,
                                           "low_v_mph": 25, "peak_lo_v_mph": 40,
                                           "peak_hi_v_mph": 60, "taper_v_mph": 72}}))
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(cfg))
  v = PnwVehicle(FakeCP(LIGHTNING, "ford"))
  assert abs(v.curve_speed_penalty_ms(25 * MPH) / MPH - 3.0) < 1e-6     # at/below low_v -> min
  assert abs(v.curve_speed_penalty_ms(50 * MPH) / MPH - 12.0) < 1e-6    # in the peak band -> max
  assert abs(v.curve_speed_penalty_ms(80 * MPH) / MPH - 2.0) < 1e-6     # past taper -> taper value


def test_curve_json_out_of_bounds_clamped(tmp_path, monkeypatch):
  cfg = tmp_path / "curve.json"
  cfg.write_text(json.dumps({"lightning": {"penalty_min_mph": -5, "penalty_max_mph": 999,
                                           "penalty_taper_mph": -9,
                                           "low_v_mph": 2, "taper_v_mph": 500}}))
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
  assert abs(v.curve_speed_penalty_ms(55 * MPH) / MPH - 5.0) < 1e-6    # default peak (washout zone)
  assert abs(v.curve_speed_penalty_ms(30 * MPH) / MPH - 1.0) < 1e-6    # default min (slow)


def test_curve_json_nan_ignored(tmp_path, monkeypatch):
  cfg = tmp_path / "curve.json"
  cfg.write_text('{"lightning": {"penalty_max_mph": NaN}}')   # json allows NaN; must be rejected
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(cfg))
  v = PnwVehicle(FakeCP(LIGHTNING, "ford"))
  assert abs(v.curve_speed_penalty_ms(55 * MPH) / MPH - 5.0) < 1e-6    # NaN dropped -> default peak


# ---- descentcurve2pnw: descent guard / left factor / total cap / new accessors --------------------
def _lightning(tmp_path, monkeypatch, cfg=None):
  p = tmp_path / ("curve.json" if cfg is not None else "nope.json")
  if cfg is not None:
    p.write_text(json.dumps({"lightning": cfg}))
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(p))
  return PnwVehicle(FakeCP(LIGHTNING, "ford"))


def test_descent_monotonic_in_pitch_and_capped(tmp_path, monkeypatch):
  v = _lightning(tmp_path, monkeypatch)
  t = 55 * MPH                                     # peak zone: base 5 mph
  pens = [v.curve_speed_penalty_ms(t, pitch_rad=-p) for p in (0.0, 0.02, 0.05, 0.08, 0.12, 0.2, 0.5)]
  for a, b in zip(pens, pens[1:], strict=False):
    assert b >= a - 1e-9                           # monotonic non-decreasing in |pitch|
  # 5% grade -> +40% at defaults (descent_gain 8.0)
  assert abs(v.curve_speed_penalty_ms(t, pitch_rad=-0.05) - v.curve_speed_penalty_ms(t) * 1.4) < 1e-9
  # capped at descent_pitch_cap (0.12): steeper adds nothing
  assert abs(pens[-1] - pens[-2]) < 1e-9 and abs(pens[-2] - pens[4]) < 1e-9


def test_descent_ignores_uphill_none_and_nan(tmp_path, monkeypatch):
  v = _lightning(tmp_path, monkeypatch)
  t = 55 * MPH
  base = v.curve_speed_penalty_ms(t)
  assert v.curve_speed_penalty_ms(t, pitch_rad=None) == base      # no pitch data -> no-op
  assert v.curve_speed_penalty_ms(t, pitch_rad=0.05) == base      # uphill -> no-op
  assert v.curve_speed_penalty_ms(t, pitch_rad=float('nan')) == base
  assert v.curve_speed_penalty_ms(t, pitch_rad="junk") == base    # never raises


def test_left_factor_only_on_left(tmp_path, monkeypatch):
  v = _lightning(tmp_path, monkeypatch)
  t = 55 * MPH
  assert abs(v.curve_speed_penalty_ms(t, is_left=True) - v.curve_speed_penalty_ms(t) * 1.15) < 1e-9
  assert v.curve_speed_penalty_ms(t, is_left=False) == v.curve_speed_penalty_ms(t)


def test_total_penalty_hard_capped_15mph(tmp_path, monkeypatch):
  # worst legal config + steep descent + left: the TOTAL can never exceed 15 mph
  v = _lightning(tmp_path, monkeypatch, {"penalty_max_mph": 15, "descent_gain": 20,
                                         "descent_pitch_cap": 0.2, "left_factor": 1.5})
  pen = v.curve_speed_penalty_ms(55 * MPH, pitch_rad=-0.3, is_left=True)
  assert pen <= 15.0 * MPH + 1e-9


def test_tesla_zero_with_all_new_args():
  v = PnwVehicle(FakeCP("TESLA_MODEL_S_HW3", "tesla", op_long=True))
  for mph in (30, 55, 75):
    assert v.curve_speed_penalty_ms(mph * MPH, pitch_rad=-0.1, is_left=True) == 0.0
  assert v.overspeed_margin_ms == 0.0
  assert v.icbm_map_scale == 1.0
  assert v.icbm_firm_decel == 0.0
  n = PnwVehicle(None)
  assert n.curve_speed_penalty_ms(20.0, pitch_rad=-0.1, is_left=True) == 0.0
  assert n.overspeed_margin_ms == 0.0 and n.icbm_map_scale == 1.0 and n.icbm_firm_decel == 0.0


def test_new_accessors_lightning_defaults(tmp_path, monkeypatch):
  v = _lightning(tmp_path, monkeypatch)
  assert abs(v.overspeed_margin_ms - 2.0 * MPH) < 1e-9
  assert v.icbm_map_scale == 0.92
  assert v.icbm_firm_decel == 1.4


def test_new_keys_tunable_and_clamped(tmp_path, monkeypatch):
  v = _lightning(tmp_path, monkeypatch, {"descent_gain": 4.0, "left_factor": 1.3,
                                         "overspeed_margin_mph": 3.0, "map_scale": 0.85,
                                         "icbm_firm_decel": 1.2})
  t = 55 * MPH
  assert abs(v.curve_speed_penalty_ms(t, pitch_rad=-0.05) - v.curve_speed_penalty_ms(t) * 1.2) < 1e-9
  assert abs(v.curve_speed_penalty_ms(t, is_left=True) - v.curve_speed_penalty_ms(t) * 1.3) < 1e-9
  assert abs(v.overspeed_margin_ms - 3.0 * MPH) < 1e-9
  assert v.icbm_map_scale == 0.85 and v.icbm_firm_decel == 1.2
  # out-of-bounds values clamp toward NEUTRAL, never invert (a bad config can't cause a speed-up
  # or an inflated map speed or an aggressive decel)
  w = _lightning(tmp_path, monkeypatch, {"descent_gain": -5, "left_factor": 0.2,
                                         "map_scale": 1.8, "icbm_firm_decel": 9.0})
  assert w.curve_speed_penalty_ms(t, pitch_rad=-0.1) == w.curve_speed_penalty_ms(t)  # gain -> 0
  assert w.curve_speed_penalty_ms(t, is_left=True) >= w.curve_speed_penalty_ms(t)    # factor -> 1.0
  assert w.icbm_map_scale <= 1.0
  assert w.icbm_firm_decel <= 1.5


def test_existing_curve_json_keys_still_work_alongside_new(tmp_path, monkeypatch):
  # extend-the-schema requirement: every pre-descentcurve key keeps working in the same file
  v = _lightning(tmp_path, monkeypatch, {"penalty_max_mph": 8, "descent_gain": 10.0})
  t = 55 * MPH
  assert abs(v.curve_speed_penalty_ms(t) / MPH - 8.0) < 1e-6
  assert abs(v.curve_speed_penalty_ms(t, pitch_rad=-0.05) / MPH - 8.0 * 1.5) < 1e-6
