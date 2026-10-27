"""curvehi2pnw (owner 2026-10-27): the Tesla's curve price A (PnwVehicle.curve_lat_a, used by the curve brain) is capped at
curve.json tesla.curve_lat_a_hi (default 3.3) at and above 80 mph. Engaged at 85-90 mph the car delivers only ~2.2 m/s^2 at the
steering-angle clamp, so the 3.5886 price was too fast there. Below 80 mph: unchanged. Every other car: unchanged.

Also pinned: the step at 80 mph cannot make the brain's price flip-flop, because curve_brain.row_speed takes the LOWEST A over the speed now
and the speeds its estimate asks for."""
import json
import math

import pytest

from openpilot.selfdrive.controls.lib import pnw_vehicle as pv
from openpilot.selfdrive.controls.lib.ces_pnw import curve_brain as cb

MPH = 0.44704
CEIL = 3.5886


class CP:
  def __init__(self, fp, brand, op_long):
    self.carFingerprint, self.brand, self.openpilotLongitudinalControl, self.dashcamOnly = fp, brand, op_long, False


class _Log:
  def __init__(self):
    self.lines = []

  def __getattr__(self, lvl):
    return lambda msg, *a, **k: self.lines.append((lvl, msg))


@pytest.fixture
def env(tmp_path, monkeypatch):
  """A private curve.json, and the Tesla's own lataccel entry (3.6 at every speed here: the schedule never binds, the clamp 3.5886 does)."""
  path = tmp_path / "curve.json"
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(path))
  monkeypatch.setattr(pv, "lat_accel_target_source", lambda v, platform=None: (3.6, True))
  log = _Log()
  monkeypatch.setattr(pv, "cloudlog", log)
  return path, log


def tesla():
  return pv.PnwVehicle(CP("TESLA_MODEL_S_HW3", "tesla", True))


def lightning():
  return pv.PnwVehicle(CP("FORD_F_150_LIGHTNING_MK1", "ford", False))


@pytest.mark.parametrize("mph", [30, 60, 70, 79.9])
def test_below_80_mph_the_price_is_unchanged(env, mph):
  assert tesla().curve_lat_a(mph * MPH) == pytest.approx(CEIL, abs=1e-9)


@pytest.mark.parametrize("mph", [80, 85, 90, 120])
def test_at_and_above_80_mph_the_price_is_3_3(env, mph):
  assert pv.TESLA_CURVE_LAT_A_HI_DEFAULT == 3.3 and pv.TESLA_CURVE_LAT_A_HI_FROM == pytest.approx(35.7632)
  assert tesla().curve_lat_a(mph * MPH) == pytest.approx(3.3, abs=1e-9)


def test_the_curve_json_key_sets_it(env):
  path, log = env
  path.write_text(json.dumps({"tesla": {"curve_lat_a_hi": 3.0}}))
  v = tesla()
  assert v.curve_lat_a(85 * MPH) == pytest.approx(3.0) and v.curve_lat_a(70 * MPH) == pytest.approx(CEIL)
  assert v.curve_brain_why == "curve.json"


def test_a_value_above_the_clamp_is_clamped_and_warned(env):
  path, log = env
  path.write_text(json.dumps({"tesla": {"curve_lat_a_hi": 5.0}}))
  assert tesla().curve_lat_a(85 * MPH) == pytest.approx(CEIL)
  assert any(lvl == "warning" and "curve_lat_a_hi=5.0->3.5886" in m for lvl, m in log.lines)


@pytest.mark.parametrize("raw", ["fast", None, True, float("nan")])
def test_a_bad_value_is_the_default_and_an_error(env, raw):
  path, log = env
  path.write_text(json.dumps({"tesla": {"curve_lat_a_hi": raw}}))     # json writes nan as NaN, which json.load reads back
  v = tesla()
  assert v.curve_lat_a(85 * MPH) == pytest.approx(3.3)
  assert any(lvl == "error" and "curve_lat_a_hi" in m for lvl, m in log.lines)
  assert v.curve_brain_why.startswith("INVALID")            # like any bad key: the section is not honored (curve_brain -> shadow)


@pytest.mark.parametrize("mph", [60, 85])
def test_the_lightning_is_unchanged(env, mph):
  path, _ = env
  path.write_text(json.dumps({"tesla": {"curve_lat_a_hi": 2.0}}))
  assert lightning().curve_lat_a(mph * MPH) == pv.CURVE_LAT_A_DEFAULT


def test_the_brain_prices_k_0_0018_at_3_3(env):
  """k 0.0018: at 3.5886 it is sqrt(3.5886/0.0018) = 44.65 m/s (99.9 mph), above 80 mph, so the row is now priced at 3.3: 42.8 m/s (95.8 mph),
  whatever the car's speed now."""
  v = tesla()
  for mph in (60, 72, 85):
    speed, a = cb.row_speed(v, 0.0018, mph * MPH)
    assert a == pytest.approx(3.3) and speed == pytest.approx(math.sqrt(3.3 / 0.0018), abs=1e-6)
  assert math.sqrt(3.3 / 0.0018) / MPH == pytest.approx(95.8, abs=0.1)


@pytest.mark.parametrize("k", [0.0016, 0.0024, 0.0026, 0.0027, 0.0028, 0.0029, 0.004])
@pytest.mark.parametrize("start_mph", [60, 79, 81, 90])
def test_the_step_at_80_mph_does_not_flip_flop(env, k, start_mph):
  """Closed loop of the worst kind: the car goes exactly the speed the row asks for, then the row is priced again from that speed. The
  band k 0.0026-0.0028 prices at 3.5886 above 80 mph but at 3.3 below it; the lowest-A rule keeps 3.3 there from any start, so the price
  settles after at most one step and never alternates."""
  v = tesla()
  speeds = [start_mph * MPH]
  for _ in range(6):
    speeds.append(cb.row_speed(v, k, speeds[-1])[0])
  assert speeds[-1] == speeds[-2] == speeds[-3] == speeds[-4] == speeds[-5]
  if 0.00258 < k < 0.002806:                                  # the band: sqrt(3.3/k) < 80 mph < sqrt(3.5886/k)
    assert speeds[-1] == pytest.approx(math.sqrt(3.3 / k))


def test_a_mid_drive_edit_is_hot_reloaded_and_counts_as_a_change(env):
  path, log = env
  v = tesla()
  assert v.curve_lat_a(85 * MPH) == pytest.approx(3.3)
  path.write_text(json.dumps({"tesla": {"curve_lat_a_hi": 2.8}}))
  assert v.refresh_curve_brain_cfg(now=v._tesla_cfg_poll + 10.0) is True
  assert v.curve_lat_a(85 * MPH) == pytest.approx(2.8)
