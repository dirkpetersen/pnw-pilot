"""Capability-view tests — PnwVehicle maps cars to features (no fingerprint checks in feature code)."""
import json
import math

from cereal import log
from openpilot.selfdrive.controls.lib import pnw_vehicle as pv
from openpilot.selfdrive.controls.lib.pnw_vehicle import PnwVehicle

MPH = 0.44704
LIGHTNING = "FORD_F_150_LIGHTNING_MK1"


class FakeCP:
  def __init__(self, fp="", brand="", op_long=False, dashcam_only=False):
    self.carFingerprint = fp
    self.brand = brand
    self.openpilotLongitudinalControl = op_long
    self.dashcamOnly = dashcam_only


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


# ---- rain2pnw: wet-weather curve margin (applies to BOTH cars, same reduction) --------------------
def test_rain_default_off_no_reduction():
  for v in (PnwVehicle(FakeCP(LIGHTNING, "ford")),
            PnwVehicle(FakeCP("TESLA_MODEL_S_HW3", "tesla", op_long=True))):
    assert v.rain_penalty_ms() == 0.0                 # tier defaults to None


def test_rain_tiers_both_cars_same_reduction(tmp_path, monkeypatch):
  # no rain.json -> hardcoded 3/5 mph defaults; the SAME reduction on Tesla and Lightning.
  monkeypatch.setattr(pv, "RAIN_CONFIG_PATH", str(tmp_path / "none.json"))
  light = PnwVehicle(FakeCP(LIGHTNING, "ford"))
  tesla = PnwVehicle(FakeCP("TESLA_MODEL_S_HW3", "tesla", op_long=True))
  for v in (light, tesla):
    v.set_rain_tier(1)
    assert abs(v.rain_penalty_ms() / MPH - 3.0) < 1e-6
    v.set_rain_tier(2)
    assert abs(v.rain_penalty_ms() / MPH - 5.0) < 1e-6
    v.set_rain_tier(0)
    assert v.rain_penalty_ms() == 0.0


def test_rain_set_tier_accepts_bytes_str_int_and_junk():
  v = PnwVehicle(FakeCP(LIGHTNING, "ford"))
  v.set_rain_tier(b"2")
  assert v._rain_tier == 2      # Params.get yields bytes
  v.set_rain_tier("1")
  assert v._rain_tier == 1
  v.set_rain_tier(2)
  assert v._rain_tier == 2
  for junk in (None, "x", b"", 3, -1, 99):
    v.set_rain_tier(junk)
    assert v._rain_tier == 0    # anything not 1/2 -> off


def test_rain_config_override_and_clamp(tmp_path, monkeypatch):
  cfg = tmp_path / "rain.json"
  cfg.write_text(json.dumps({"light_mph": 4.5, "heavy_mph": 99.0}))   # heavy clamps to 15
  monkeypatch.setattr(pv, "RAIN_CONFIG_PATH", str(cfg))
  v = PnwVehicle(FakeCP("TESLA_MODEL_S_HW3", "tesla", op_long=True))
  v.set_rain_tier(1)
  assert abs(v.rain_penalty_ms() / MPH - 4.5) < 1e-6
  v.set_rain_tier(2)
  assert abs(v.rain_penalty_ms() / MPH - 15.0) < 1e-6


def test_rain_config_malformed_falls_back(tmp_path, monkeypatch):
  cfg = tmp_path / "rain.json"
  cfg.write_text("{ not json")
  monkeypatch.setattr(pv, "RAIN_CONFIG_PATH", str(cfg))
  v = PnwVehicle(FakeCP(LIGHTNING, "ford"))
  v.set_rain_tier(2)
  assert abs(v.rain_penalty_ms() / MPH - 5.0) < 1e-6   # default heavy


def test_rain_is_separate_from_lightning_curve_penalty(tmp_path, monkeypatch):
  # rain does NOT touch curve_speed_penalty_ms (still Lightning-only + Tesla-zero); it is additive & separate.
  monkeypatch.setattr(pv, "RAIN_CONFIG_PATH", str(tmp_path / "none.json"))
  tesla = PnwVehicle(FakeCP("TESLA_MODEL_S_HW3", "tesla", op_long=True))
  tesla.set_rain_tier(2)
  assert tesla.curve_speed_penalty_ms(55 * MPH) == 0.0     # base curve penalty still zero on Tesla
  assert tesla.rain_penalty_ms() > 0.0                     # rain margin is delivered separately


# ---- standstillsoft2pnw: gentle standstill-launch accel cap ---------------------------------------
def test_gentle_launch_lightning_ramp():
  v = PnwVehicle(FakeCP(LIGHTNING, "ford", op_long=True))
  assert v.gentle_launch
  assert abs(v.gentle_launch_accel(0.0) - 0.6) < 1e-6            # soft floor at a dead stop
  v_lift = 9.0 * MPH
  assert math.isinf(v.gentle_launch_accel(v_lift))              # lifted by launch_v
  assert math.isinf(v.gentle_launch_accel(20.0))               # no cap at speed
  ramp = [v.gentle_launch_accel(x) for x in (0.0, 1.0, 2.0, 3.0)]
  assert all(ramp[i] <= ramp[i + 1] + 1e-9 for i in range(len(ramp) - 1))   # non-decreasing


def test_gentle_launch_tesla_uncapped():
  t = PnwVehicle(FakeCP("TESLA_MODEL_S_HW3", "tesla", op_long=True))
  assert not t.gentle_launch
  assert math.isinf(t.gentle_launch_accel(0.0)) and math.isinf(t.gentle_launch_accel(2.0))


def test_gentle_launch_reduce_only():
  # min(stock_max, cap) must never exceed the stock max -> can only soften, never speed up
  v = PnwVehicle(FakeCP(LIGHTNING, "ford", op_long=True))
  assert all(min(2.0, v.gentle_launch_accel(x * 0.2)) <= 2.0 + 1e-9 for x in range(30))


# ---- fpsidebar2pnw: fingerprint -> display name (offroad "last-known car" sidebar label) ----------
def test_display_name_known_cars():
  # code review 2026-07-18: "F-150 Lightning" overflowed the ~218px sidebar metric box (unclipped
  # text) -> shortened to "Lightning", which fits (see the width-budget comment on _DISPLAY_NAMES).
  assert pv.display_name(FakeCP(LIGHTNING, "ford")) == "Lightning"
  assert pv.display_name(FakeCP("TESLA_MODEL_S_HW3", "tesla")) == "Model S"


def test_display_name_never_shows_mock_or_raw_fingerprint():
  # MOCK must never be shown to the driver, and an unrecognized fingerprint falls back to None
  # (the caller's default text) rather than leaking the raw internal string.
  assert pv.display_name(FakeCP("MOCK", "mock")) is None
  assert pv.display_name(FakeCP("SOME_OTHER_CAR", "hyundai")) is None
  assert pv.display_name(FakeCP("", "")) is None


def test_display_name_dashcam_only_hidden_even_if_fingerprint_known():
  # a recognized fingerprint that's somehow flagged dashcamOnly must still show nothing (defense in
  # depth alongside the card.py MOCK-persist guard).
  assert pv.display_name(FakeCP(LIGHTNING, "ford", dashcam_only=True)) is None


def test_display_name_defensive_on_bad_input():
  # None CP, and any object missing the expected attributes, must never raise -> always None.
  assert pv.display_name(None) is None
  assert pv.display_name(object()) is None

  class Garbage:
    pass
  assert pv.display_name(Garbage()) is None


# --------------------------------------------------------------------------------------------------
# tightfollow2pnw v2 — stability-gated + slewed Aggressive T_FOLLOW (the v1 flat 1.0 was reverted)
# --------------------------------------------------------------------------------------------------
AGGRESSIVE = log.LongitudinalPersonality.aggressive
STANDARD = log.LongitudinalPersonality.standard
BASE = pv._TIGHT_AGGRESSIVE_BASELINE
TIGHT = pv._TIGHT_AGGRESSIVE_T_FOLLOW
DT = pv._TIGHT_DT_DEFAULT


class FakeLead:
  """Mimics a cereal radarState lead reader: only the fields aggressive_t_follow actually reads."""
  def __init__(self, d_rel=50.0, v_lead_k=20.0, a_lead_k=0.0, status=True):
    self.dRel = d_rel
    self.vLeadK = v_lead_k
    self.aLeadK = a_lead_k
    self.status = status
    self.vRel = 99.0        # deliberately garbage: v2 must NEVER read raw vRel (the v1 failure)


def _tight_lightning():
  return PnwVehicle(FakeCP(LIGHTNING, "ford", op_long=True))


def _settle(veh, lead, v_ego=20.0, ticks=400, baseline=BASE):
  """Run the gate for `ticks` planner ticks, returning the final override."""
  out = None
  for _ in range(ticks):
    out = veh.aggressive_t_follow(AGGRESSIVE, lead=lead, v_ego=v_ego, baseline=baseline, dt=DT)
  return out


def test_tight_follow_is_lightning_aggressive_only():
  tesla = PnwVehicle(FakeCP("TESLA_MODEL_S_HW3", "tesla", op_long=True))
  assert not tesla.tight_aggressive_follow
  assert _settle(tesla, FakeLead()) is None                      # Tesla Aggressive unchanged
  assert _tight_lightning().tight_aggressive_follow
  # right car, wrong personality -> untouched upstream path
  veh = _tight_lightning()
  assert veh.aggressive_t_follow(STANDARD, lead=FakeLead(), v_ego=20.0, baseline=1.45, dt=DT) is None


def test_calm_lead_reaches_the_tight_target():
  veh = _tight_lightning()
  assert _settle(veh, FakeLead(v_lead_k=20.0), v_ego=20.0) == TIGHT


def test_no_tightening_before_the_stability_window():
  """The clock must actually gate: one tick in, nothing has moved off baseline yet."""
  veh = _tight_lightning()
  lead = FakeLead(v_lead_k=20.0)
  assert veh.aggressive_t_follow(AGGRESSIVE, lead=lead, v_ego=20.0, baseline=BASE, dt=DT) is None
  # still inside the 5 s window -> still baseline (None)
  for _ in range(int(pv._TIGHT_STABLE_MIN_S / DT) - 2):
    out = veh.aggressive_t_follow(AGGRESSIVE, lead=lead, v_ego=20.0, baseline=BASE, dt=DT)
  assert out is None


def test_output_is_slewed_not_stepped():
  """Crossing the gate must not hand the MPC a discontinuity."""
  veh = _tight_lightning()
  lead = FakeLead(v_lead_k=20.0)
  prev = BASE
  seen_intermediate = False
  for _ in range(400):
    out = veh.aggressive_t_follow(AGGRESSIVE, lead=lead, v_ego=20.0, baseline=BASE, dt=DT)
    cur = BASE if out is None else out
    assert abs(cur - prev) <= pv._TIGHT_SLEW_S_PER_S * DT + 1e-9, "T_FOLLOW stepped"
    if TIGHT < cur < BASE:
      seen_intermediate = True
    prev = cur
  assert seen_intermediate, "never observed a partially-slewed value -- the slew is not engaged"


def test_never_exceeds_the_declared_envelope():
  """Whatever the lead does, the override stays inside [TIGHT, BASE] — never tighter, never looser."""
  veh = _tight_lightning()
  import random
  rng = random.Random(1234)
  for i in range(3000):
    lead = FakeLead(d_rel=rng.uniform(5.0, 90.0), v_lead_k=rng.uniform(0.0, 40.0),
                    a_lead_k=rng.uniform(-3.0, 3.0), status=rng.random() > 0.2)
    out = veh.aggressive_t_follow(AGGRESSIVE, lead=lead, v_ego=20.0, baseline=BASE, dt=DT)
    if out is not None:
      assert TIGHT - 1e-9 <= out <= BASE + 1e-9, f"escaped envelope at {i}: {out}"


def test_excursion_releases_back_toward_baseline():
  veh = _tight_lightning()
  assert _settle(veh, FakeLead(v_lead_k=20.0), v_ego=20.0) == TIGHT
  # lead brakes hard -> filtered aLeadK past the threshold -> must relax back to baseline
  hard = FakeLead(v_lead_k=20.0, a_lead_k=-2.0)
  out = None
  for _ in range(400):
    out = veh.aggressive_t_follow(AGGRESSIVE, lead=hard, v_ego=20.0, baseline=BASE, dt=DT)
  assert out is None, "did not return to baseline after a lead excursion"


def test_lead_swap_restarts_the_clock():
  """A cut-in is a NEW car: credit earned behind the previous lead must not transfer."""
  veh = _tight_lightning()
  assert _settle(veh, FakeLead(d_rel=60.0, v_lead_k=20.0), v_ego=20.0) == TIGHT
  cutin = FakeLead(d_rel=25.0, v_lead_k=20.0)          # 35 m step >> _TIGHT_LEAD_JUMP_M
  out = veh.aggressive_t_follow(AGGRESSIVE, lead=cutin, v_ego=20.0, baseline=BASE, dt=DT)
  assert out is not None and out > TIGHT, "cut-in did not start releasing"
  assert veh._tight_stable_s == 0.0


def test_lost_lead_and_malformed_lead_fail_closed():
  for bad in (None, FakeLead(status=False), FakeLead(v_lead_k=float('nan')),
              FakeLead(a_lead_k=float('inf'))):
    veh = _tight_lightning()
    assert _settle(veh, bad, v_ego=20.0) is None


def test_raw_vrel_is_never_consulted():
  """v1's failure was reading the UNFILTERED vRel. FakeLead.vRel is garbage (99 m/s) on every lead
  in this file; if the gate ever reads it, the calm-lead cases above cannot reach TIGHT."""
  veh = _tight_lightning()
  lead = FakeLead(v_lead_k=20.0)
  assert lead.vRel > pv._TIGHT_MAX_ABS_VREL          # the trap is armed
  assert _settle(veh, lead, v_ego=20.0) == TIGHT     # ...and the gate ignored it


def test_personality_switch_resets_state():
  veh = _tight_lightning()
  assert _settle(veh, FakeLead(v_lead_k=20.0), v_ego=20.0) == TIGHT
  assert veh.aggressive_t_follow(STANDARD, lead=FakeLead(), v_ego=20.0, baseline=1.45, dt=DT) is None
  assert veh._tight_t_follow is None and veh._tight_stable_s == 0.0
  # re-entering Aggressive starts from baseline again, not mid-slew
  out = veh.aggressive_t_follow(AGGRESSIVE, lead=FakeLead(v_lead_k=20.0), v_ego=20.0,
                                baseline=BASE, dt=DT)
  assert out is None


def test_envelope_holds_when_the_baseline_moves_mid_slew():
  """The clamps exist ONLY for a baseline that CHANGES between calls, and nothing else in this file
  varies it -- so without this test the clamps are unfalsifiable (Fable review: removing both clamp
  lines left all other tests passing, which made the commit's mutation claim false).

  Concretely: park `cur` mid-slew at ~1.20 with the 1.25 baseline, then hand in a baseline of 1.16.
  The declared envelope is now [1.15, 1.16], but `cur` is 1.20 -- above it. Unclamped, the override
  returns 1.20 and asks the MPC for a LOOSER gap than the caller's own baseline allows."""
  veh = _tight_lightning()
  lead = FakeLead(v_lead_k=20.0)
  ticks_to_open = int(pv._TIGHT_STABLE_MIN_S / DT)
  for _ in range(ticks_to_open + 20):                 # gate opens, then ~20 ticks of slew
    veh.aggressive_t_follow(AGGRESSIVE, lead=lead, v_ego=20.0, baseline=BASE, dt=DT)
  assert veh._tight_t_follow is not None and veh._tight_t_follow > 1.16, "fixture didn't park mid-slew"

  moved_base = 1.16
  out = veh.aggressive_t_follow(AGGRESSIVE, lead=lead, v_ego=20.0, baseline=moved_base, dt=DT)
  cur = moved_base if out is None else out
  assert TIGHT - 1e-9 <= cur <= moved_base + 1e-9, \
    f"escaped the envelope after a baseline move: {cur} not in [{TIGHT}, {moved_base}]"


def test_envelope_holds_when_the_baseline_drops_below_the_tight_target():
  """Degenerate but reachable if get_T_FOLLOW is ever retuned below 1.15: the envelope inverts
  (lo/hi swap), and the override must still never return something outside it."""
  veh = _tight_lightning()
  lead = FakeLead(v_lead_k=20.0)
  for _ in range(400):
    out = veh.aggressive_t_follow(AGGRESSIVE, lead=lead, v_ego=20.0, baseline=1.05, dt=DT)
    if out is not None:
      assert 1.05 - 1e-9 <= out <= TIGHT + 1e-9, f"escaped inverted envelope: {out}"
