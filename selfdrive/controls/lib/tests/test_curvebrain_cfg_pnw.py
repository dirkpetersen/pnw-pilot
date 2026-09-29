"""curvebrain2pnw 1/8 -- the shared curve brain's per-car config (docs/SHARED-CURVE-BRAIN-DESIGN.md s3.4, s5.2).

PnwVehicle.curve_lat_a(v), curve.json's separately-parsed "tesla" section, the curve_brain mode and the curve_brain_cfg
start event. Nothing consumes any of it yet. Pinned here:
  * the loader's bounds / NaN / invalid values, each SAID (Rule 2), a missing file or section silent;
  * the "tesla" section never changes a Lightning value (one device serves both cars) -- the Lightning never opens it;
  * the Tesla's target is capped at openpilot's own lateral clip (the UNSLEWED lataccel2pnw schedule) minus 0.3.
"""
import json
import math

import pytest

from openpilot.selfdrive.controls.lib import drive_helpers as dh
from openpilot.selfdrive.controls.lib import pnw_vehicle as pv

MPH = 0.44704
TESLA = "TESLA_MODEL_S_HW3"
LIGHTNING = "FORD_F_150_LIGHTNING_MK1"


class CP:
  def __init__(self, fp, brand, op_long):
    self.carFingerprint, self.brand, self.openpilotLongitudinalControl, self.dashcamOnly = fp, brand, op_long, False


def tesla():
  return pv.PnwVehicle(CP(TESLA, "tesla", True))


def lightning():
  return pv.PnwVehicle(CP(LIGHTNING, "ford", False))


class _Log:
  def __init__(self):
    self.lines = []

  def __getattr__(self, level):
    if level in ("debug", "info", "warning", "error", "exception", "critical", "event"):
      return lambda msg, *a, **k: self.lines.append((level, msg))
    raise AttributeError(level)

  def at(self, level):
    return [m for lvl, m in self.lines if lvl == level]


@pytest.fixture
def log(monkeypatch):
  lg = _Log()
  monkeypatch.setattr(pv, "cloudlog", lg)
  return lg


@pytest.fixture
def cfg(tmp_path, monkeypatch):
  """Write curve.json (a dict, or raw text) and point the loaders at it; None = no file."""
  p = tmp_path / "curve.json"
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(p))
  monkeypatch.setattr(pv, "RAIN_CONFIG_PATH", str(tmp_path / "absent-rain.json"))

  def write(doc):
    if doc is not None:
      p.write_text(doc if isinstance(doc, str) else json.dumps(doc))
    return str(p)
  return write


@pytest.fixture
def schedule(tmp_path, monkeypatch):
  """A fresh lataccel2pnw schedule reading a file of our own (None = no file: the flat 3.0 fail-safe)."""
  p = tmp_path / "lataccel_limits.json"
  monkeypatch.setattr(dh, "LAT_ACCEL_LIMITS_PATH", str(p))
  monkeypatch.setattr(dh, "_lat_accel_schedule", dh._LatAccelSchedule())

  def write(breakpoints):
    if breakpoints is not None:
      p.write_text(json.dumps({"breakpoints": breakpoints}))
  return write


# ---------------------------------------------------------------------------------------------------------------------
# defaults and the capability
# ---------------------------------------------------------------------------------------------------------------------
def test_defaults_tesla_shadow_4_0_everyone_else_off_2_5(cfg, schedule, log):
  cfg(None)
  schedule(dh.DEFAULT_LAT_ACCEL_BREAKPOINTS_MPH)
  t = tesla()
  assert t.curve_brain_vtsc and t.curve_brain == "shadow" and t.curve_brain_why == "default"
  assert t.curve_lat_a_cfg == 4.0 and t.curve_lat_a(65 * MPH) == 3.0          # the steering ceiling binds the 4.0
  for v in (lightning(), pv.PnwVehicle(None), pv.PnwVehicle(CP("TOYOTA_RAV4", "toyota", True))):
    assert not v.curve_brain_vtsc and v.curve_brain == "off" and v.curve_brain_why == "noCapability"
    assert v.curve_lat_a(20.0) == v.curve_lat_a(40.0) == v.curve_lat_a_cfg == 2.5
  assert log.lines == []                                                      # nothing to say about a default


def test_no_tesla_section_is_silent_and_default(cfg, log):
  cfg({"lightning": {"penalty_max_mph": 6.0}})
  t = tesla()
  assert (t.curve_brain, t.curve_lat_a_cfg, t.curve_brain_why) == ("shadow", 4.0, "default")
  assert log.lines == []


def test_a_valid_section_is_honoured(cfg, log):
  cfg({"tesla": {"curve_lat_a": 3.0, "curve_brain": " LOWER "}})
  t = tesla()
  assert (t.curve_brain, t.curve_lat_a_cfg, t.curve_brain_why) == ("lower", 3.0, "curve.json")
  assert log.lines == []
  cfg({"tesla": {"curve_brain": "raise"}})
  assert tesla().curve_brain == "raise"
  cfg({"tesla": {"curve_brain": 0}})
  assert tesla().curve_brain == "off"


# ---------------------------------------------------------------------------------------------------------------------
# the loader: bounds, NaN, invalid -- each said (Rule 2)
# ---------------------------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("raw, want", [(5.0, 4.5), (9.9, 4.5), (1.0, 2.0), (-3, 2.0), (4.5, 4.5), (4.0, 4.0), (2.0, 2.0)])
def test_curve_lat_a_is_clamped_into_bounds_and_says_so(cfg, log, raw, want):
  path = cfg({"tesla": {"curve_lat_a": raw}})
  t = tesla()
  assert t.curve_lat_a_cfg == want and t.curve_brain_why == "curve.json"
  if raw != want:
    assert len(log.at("warning")) == 1 and path in log.at("warning")[0] and "curve_lat_a" in log.at("warning")[0]
  else:
    assert log.lines == []


@pytest.mark.parametrize("text", [
  '{"tesla": {"curve_lat_a": NaN}}',            # json.load parses the bare token into a float nan
  '{"tesla": {"curve_lat_a": Infinity}}',
  '{"tesla": {"curve_lat_a": "fast"}}',
  '{"tesla": {"curve_lat_a": true}}',
  '{"tesla": {"curve_lat_a": [2.9]}}',
  '{"tesla": {"curve_lat_a": null}}',
  # Fable F1: json.load makes a 309+ digit literal an int that math.isfinite / float() cannot convert (OverflowError).
  # Unguarded, PnwVehicle(Tesla) raised in every process that builds one -- a full-stack crash-loop.
  '{"tesla": {"curve_lat_a": 1' + '0' * 400 + ', "curve_brain": "lower"}}',
])
def test_a_curve_lat_a_that_is_not_a_finite_number_keeps_the_default_and_is_an_error(cfg, log, text):
  path = cfg(text)
  t = tesla()                                    # never raises
  assert t.curve_lat_a_cfg == 4.0 and math.isfinite(t.curve_lat_a(30.0)) and t.curve_brain == "shadow"
  errs = log.at("error")
  assert len(errs) == 1 and path in errs[0]
  if "0" * 400 in text:                          # the parse itself failed: the whole section is its defaults
    assert t.curve_brain_why == "default (curve.json tesla unparsable: OverflowError)" and "tesla" in errs[0]
  else:
    assert t.curve_brain_why.startswith("INVALID") and "curve_lat_a" in errs[0]


@pytest.mark.parametrize("raw", ["bogus", "live", "", 1, True, None, ["lower"]])
def test_a_mode_that_is_not_one_falls_back_to_shadow_and_is_an_error(cfg, log, raw):
  path = cfg({"tesla": {"curve_brain": raw, "curve_lat_a": 2.9}})
  t = tesla()
  assert t.curve_brain == "shadow" and t.curve_lat_a_cfg == 2.9               # a bad key costs only itself
  assert t.curve_brain_why.startswith("INVALID") and "curve_brain" in t.curve_brain_why
  errs = log.at("error")
  assert len(errs) == 1 and path in errs[0] and "curve_brain" in errs[0]


@pytest.mark.parametrize("doc", ['{"tesla": [1, 2]}', '{"tesla": "shadow"}', '{"tesla": 2.8}'])
def test_a_section_that_is_not_an_object_is_an_error(cfg, log, doc):
  path = cfg(doc)
  t = tesla()
  assert (t.curve_brain, t.curve_lat_a_cfg) == ("shadow", 4.0) and t.curve_brain_why.startswith("INVALID")
  assert len(log.at("error")) == 1 and path in log.at("error")[0]


def test_an_unreadable_file_is_an_error_for_the_tesla_section_too(cfg, log):
  path = cfg('{"tesla": {"curve_lat_a": 3.0')                                  # truncated JSON
  t = tesla()
  assert (t.curve_brain, t.curve_lat_a_cfg) == ("shadow", 4.0)
  assert t.curve_brain_why == "default (curve.json unreadable: JSONDecodeError)"
  # one error from each loader, each naming the path: the Lightning section's (it runs on every car) and the Tesla's
  assert len(log.at("error")) == 2 and all(path in e for e in log.at("error"))
  assert any("tesla" in e for e in log.at("error"))


def test_a_directory_in_place_of_the_file_is_an_error(tmp_path, monkeypatch, log):
  d = tmp_path / "curve.json"
  d.mkdir()
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(d))
  t = tesla()
  assert t.curve_brain_why == "default (curve.json unusable)"
  assert any("tesla" in e and str(d) in e for e in log.at("error"))


# ---------------------------------------------------------------------------------------------------------------------
# one device, two cars: the "tesla" section never moves a Lightning value
# ---------------------------------------------------------------------------------------------------------------------
def _lightning_view(v):
  return (dict(v._curve_cfg), v.icbm_map_scale, v.icbm_firm_decel, v.overspeed_margin_ms, v.curvedb_v2_live,
          v.curvedb_v2_lat_a, v.icbm_shape, v.icbm_shape_why, v.icbm_shape_lat_a, v.icbm_shape_lat_a_70,
          v.icbm_lead_lat_accel, v.icbm_restore_hold_lat_accel, v.icbm_restore_hold_poly_lat_accel,
          v.curve_speed_penalty_ms(25.0, -0.05, True), v.icbm_map_floor_ms(30.0), v.icbm_vis_floor_ms(30.0),
          v.gentle_launch_accel(2.0), v.curve_brain, v.curve_brain_why, v.curve_lat_a(30.0))


def _every_lightning_key_changed():
  """A "tesla" section carrying EVERY Lightning key, each at a value different from the default and in bounds."""
  out = {}
  for k, (lo, hi) in pv._CURVE_BOUNDS.items():
    out[k] = hi if pv._CURVE_DEFAULTS[k] != hi else lo
  out["icbm_shape"] = "live"
  return out


@pytest.mark.parametrize("tesla_section", [
  {"curve_lat_a": 3.1, "curve_brain": "raise"},
  _every_lightning_key_changed(),
  "garbage",
  {"curve_lat_a": float("nan"), "curve_brain": "bogus"},
])
def test_the_tesla_section_never_changes_a_lightning_value(cfg, log, tesla_section):
  light = {"penalty_max_mph": 6.0, "map_scale": 0.95, "icbm_shape": "off"}
  cfg({"lightning": light})
  base, loader = _lightning_view(lightning()), pv._load_curve_config()
  cfg(json.dumps({"lightning": light, "tesla": tesla_section}))
  log.lines.clear()
  assert _lightning_view(lightning()) == base
  assert log.lines == []                         # the Lightning never opens the section, so it has nothing to say
  assert pv._load_curve_config() == loader       # and the "lightning" loader never reads it, on ANY car


def test_the_tesla_section_is_never_opened_on_the_lightning(cfg, monkeypatch):
  cfg({"tesla": {"curve_lat_a": 3.0}})
  calls = []
  monkeypatch.setattr(pv, "_load_tesla_curve_config", lambda: calls.append(1) or {})
  lightning()
  pv.PnwVehicle(None)
  assert calls == []


# ---------------------------------------------------------------------------------------------------------------------
# the lateral-clip cap
# ---------------------------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("mph, cfg_a, want", [
  (55.0, 4.0, 3.0),        # schedule 5.5 -> clip 5.2; the STEERING ceiling (3.0) binds the 4.0 target
  (60.0, 4.0, 3.0),        # 5.0 -> 4.7
  (60.0, 4.5, 3.0),        # the upper bound is capped the same way
  (65.0, 4.5, 3.0),        # 4.5 -> 4.2
  (70.0, 4.0, 3.0),        # 4.0 -> 3.7: the schedule clip no longer binds either; steering does
  (70.0, 2.5, 2.5),        # a low config is never RAISED by any clip
  (74.0, 4.0, 3.0),        # 3.6 -> 3.3
  (75.0, 4.0, 3.0),        # 3.5 -> 3.2
  (78.0, 4.5, 2.9),        # 3.2 -> 2.9: now the schedule clip binds
  (80.0, 4.0, 2.7),        # 3.0 (ISO) -> 2.7: EFFECTIVELY 2.7 at >= 80 mph
  (95.0, 4.5, 2.7),        # held flat past the last breakpoint
])
def test_the_target_is_capped_at_the_lateral_clip_minus_0_3_and_the_steering_ceiling(cfg, schedule, mph, cfg_a, want):
  cfg({"tesla": {"curve_lat_a": cfg_a}})
  schedule(dh.DEFAULT_LAT_ACCEL_BREAKPOINTS_MPH)
  assert tesla().curve_lat_a(mph * MPH) == pytest.approx(want, abs=1e-9)


def test_the_schedule_clip_alone_binds_a_high_config_only_above_73_mph(cfg, schedule, monkeypatch):
  """With the steering ceiling out of the picture (a hypothetical 100 m/s^2), the design's min(cfg, target - 0.3) is
  what Task A specified: 4.0 at 60 mph, 3.7 at 70, 2.7 at >= 80. Pins that the schedule term is still in the min."""
  cfg({"tesla": {"curve_lat_a": 4.0}})
  schedule(dh.DEFAULT_LAT_ACCEL_BREAKPOINTS_MPH)
  monkeypatch.setattr(pv, "_steer_lat_ceiling_cache", 100.0)
  t = tesla()
  for mph, want in ((55.0, 4.0), (60.0, 4.0), (65.0, 4.0), (70.0, 3.7), (75.0, 3.2), (80.0, 2.7), (95.0, 2.7)):
    assert t.curve_lat_a(mph * MPH) == pytest.approx(want, abs=1e-9)


def test_the_default_4_0_never_assumes_more_than_the_limits_allow(cfg, schedule):
  """A 4.0 SPEED target is not a steering capability: at every speed the target stays 0.3 under the schedule AND at or
  under the steering ceiling."""
  cfg(None)
  schedule(dh.DEFAULT_LAT_ACCEL_BREAKPOINTS_MPH)
  t = tesla()
  for mph in range(10, 121, 5):
    a = t.curve_lat_a(mph * MPH)
    assert a <= dh.lat_accel_target(mph * MPH) - 0.3 + 1e-9
    assert a <= pv._tesla_steer_lat_ceiling() + 1e-9 and a <= 4.0 + 1e-9


def test_without_a_schedule_the_clip_is_the_flat_iso_3_0(cfg, schedule):
  cfg({"tesla": {"curve_lat_a": 4.5}})
  schedule(None)
  t = tesla()
  for v in (5.0, 25.0, 40.0, float("nan")):
    assert t.curve_lat_a(v) == pytest.approx(2.7)


# ---------------------------------------------------------------------------------------------------------------------
# the steering ceiling: derived from opendbc's own constants, never a copy that can drift
# ---------------------------------------------------------------------------------------------------------------------
def test_the_steering_ceiling_is_the_vehicle_model_clamp_without_its_bank_tolerance():
  from opendbc.car.lateral import ISO_LATERAL_ACCEL, get_max_angle_vm
  from opendbc.car.tesla.values import AVERAGE_ROAD_ROLL, CarControllerParams
  from opendbc.car.vehicle_model import VehicleModel
  from opendbc.car.tesla.interface import CarInterface
  from opendbc.car.tesla.values import ACCELERATION_DUE_TO_GRAVITY
  assert pv._tesla_steer_lat_ceiling() == ISO_LATERAL_ACCEL == pv._STEER_LAT_CEILING_FALLBACK
  lim = CarControllerParams.ANGLE_LIMITS.MAX_LATERAL_ACCEL
  assert lim - ACCELERATION_DUE_TO_GRAVITY * AVERAGE_ROAD_ROLL == pytest.approx(pv._tesla_steer_lat_ceiling())
  # the 09-28 22:35 stall: the applied angle plateaued at 14.42 deg at 70.3 mph == the clamp at that speed
  vm = VehicleModel(CarInterface.get_non_essential_params("TESLA_MODEL_S_HW3"))
  assert get_max_angle_vm(70.3 * MPH, vm, CarControllerParams) == pytest.approx(14.42, abs=0.01)
  # and the clamp is a constant lateral acceleration at every speed: angle -> curvature -> a_lat round-trips to `lim`
  for mph in (40.0, 55.0, 70.3, 80.0):
    v = mph * MPH
    ang = math.radians(get_max_angle_vm(v, vm, CarControllerParams))
    assert vm.calc_curvature(ang, v, 0.0) * v * v == pytest.approx(lim, rel=1e-3)


def test_panda_enforces_the_same_lateral_accel_figure():
  """panda's steer_angle_cmd_checks_vm builds its limit from the same two constants; if either side is edited the
  ceiling above no longer describes the clamp."""
  import pathlib
  import re
  import opendbc
  h = pathlib.Path(opendbc.__file__).parent.joinpath("safety", "lateral.h").read_text()
  assert re.search(r"MAX_LATERAL_ACCEL = ISO_LATERAL_ACCEL \+ \(EARTH_G \* AVERAGE_ROAD_ROLL\)", h)


def test_a_steering_ceiling_that_cannot_be_read_falls_back_loudly(monkeypatch, log):
  monkeypatch.setattr(pv, "_steer_lat_ceiling_cache", None)
  import builtins
  real_import = builtins.__import__

  def broken(name, *a, **k):
    if name == "opendbc.car.lateral":
      raise ImportError("boom")
    return real_import(name, *a, **k)
  monkeypatch.setattr(builtins, "__import__", broken)
  try:
    assert pv._tesla_steer_lat_ceiling() == 3.0
  finally:
    monkeypatch.undo()
  assert any("steering lateral ceiling" in e and "ImportError" in e for e in log.at("error"))


def test_terwilliger_left_curve_entry_a_target_of_4_0_alone_is_the_speed_that_failed(cfg, schedule):
  """The 09-28 22:35 left curve, DB row k = 0.0040 (R 250 m): (a) 4.0 alone = 70.7 mph, the speed at which the applied
  angle stalled; (b) with the lataccel schedule clip (evaluated at that speed) 67.4 mph; (c) with the steering ceiling 61.3 mph."""
  cfg(None)
  schedule(dh.DEFAULT_LAT_ACCEL_BREAKPOINTS_MPH)
  k = 0.0040
  assert math.sqrt(4.0 / k) / MPH == pytest.approx(70.7, abs=0.05)                              # (a)
  a_sched = min(4.0, dh.lat_accel_target(70.7 * MPH) - 0.3)
  assert math.sqrt(a_sched / k) / MPH == pytest.approx(67.4, abs=0.1)                           # (b) at the fail speed
  a_full = tesla().curve_lat_a(70.7 * MPH)
  assert a_full == pytest.approx(3.0)
  assert math.sqrt(a_full / k) / MPH == pytest.approx(61.3, abs=0.1)                            # (c)


# ---------------------------------------------------------------------------------------------------------------------
# the kill switch: hot reload of curve.json's tesla section
# ---------------------------------------------------------------------------------------------------------------------
def _reload(t, cfg_writer, doc, now):
  """Rewrite curve.json (a new mtime is forced) and poll once at `now`."""
  path = cfg_writer(doc)
  import os
  st = os.stat(path)
  os.utime(path, ns=(st.st_atime_ns, st.st_mtime_ns + 5_000_000_000))
  return t.refresh_curve_brain_cfg(now)


def test_a_mode_change_in_curve_json_takes_effect_within_the_poll_interval(cfg, log):
  cfg({"tesla": {"curve_brain": "lower"}})
  t = tesla()
  assert t.curve_brain == "lower"
  assert _reload(t, cfg, {"tesla": {"curve_brain": "shadow"}}, t._tesla_cfg_poll + 1.0) is True
  assert t.curve_brain == "shadow" and t.curve_brain_why == "curve.json"
  assert _reload(t, cfg, {"tesla": {"curve_brain": "off", "curve_lat_a": 3.2}}, t._tesla_cfg_poll + 1.0) is True
  assert (t.curve_brain, t.curve_lat_a_cfg) == ("off", 3.2)
  assert log.at("error") == []


def test_the_poll_is_throttled_and_an_unchanged_file_is_not_reparsed(cfg, monkeypatch):
  cfg({"tesla": {"curve_brain": "lower"}})
  t = tesla()
  calls = []
  real = pv._load_tesla_curve_config
  monkeypatch.setattr(pv, "_load_tesla_curve_config", lambda: calls.append(1) or real())
  t0 = t._tesla_cfg_poll
  assert t.refresh_curve_brain_cfg(t0 + 0.2) is False          # inside the interval: not even a stat
  assert t.refresh_curve_brain_cfg(t0 + 1.5) is False          # a stat, same signature: no re-parse
  assert calls == []


def test_a_typo_mid_drive_keeps_the_last_good_config_and_says_so(cfg, log):
  cfg({"tesla": {"curve_brain": "shadow"}})
  t = tesla()
  assert _reload(t, cfg, {"tesla": {"curve_brain": "shadw"}}, t._tesla_cfg_poll + 1.0) is False
  assert t.curve_brain == "shadow"                                              # NOT the acting default
  assert any("NOT applied" in e for e in log.at("error"))
  log.lines.clear()
  assert _reload(t, cfg, '{"tesla": {"curve_brain": "lower"', t._tesla_cfg_poll + 1.0) is False   # truncated JSON
  assert t.curve_brain == "shadow"
  assert any("NOT applied" in e for e in log.at("error"))
  assert _reload(t, cfg, {"tesla": {"curve_brain": "lower"}}, t._tesla_cfg_poll + 1.0) is True   # a good file applies
  assert t.curve_brain == "lower"


def test_a_deleted_file_reverts_to_the_documented_default_and_says_so(cfg, log):
  path = cfg({"tesla": {"curve_brain": "shadow"}})
  t = tesla()
  import os
  os.unlink(path)
  t.refresh_curve_brain_cfg(t._tesla_cfg_poll + 1.0)
  assert t.curve_brain == pv.CURVE_BRAIN_DEFAULT and t.curve_brain_why == "default"
  assert "curve_brain_cfg_reload" in [m for lvl, m in log.lines if lvl == "event"]


def test_the_reload_is_inert_on_a_car_without_the_capability(cfg):
  cfg({"tesla": {"curve_brain": "lower"}})
  v = lightning()
  assert v.refresh_curve_brain_cfg(1e9) is False and v.curve_brain == "off"


def test_reading_the_clip_never_moves_clip_curvatures_slew(schedule):
  """The cap reads the UNSLEWED target: a curve-brain call must not advance the slewed limit controlsd applies."""
  schedule(dh.DEFAULT_LAT_ACCEL_BREAKPOINTS_MPH)
  s = dh._lat_accel_schedule
  before = (s._eff_cap, s._last_limit_mono)
  for _ in range(50):
    dh.lat_accel_target(40 * MPH)
  assert (s._eff_cap, s._last_limit_mono) == before
  assert dh.lat_accel_target(40 * MPH) == 6.0 and dh.lat_accel_limit(40 * MPH) == 3.0   # the slew starts at 3.0


def test_the_slewed_limit_converges_to_the_target_it_reads(schedule, monkeypatch):
  """target() is limit()'s own first half: after enough time at one speed the slewed cap IS the target."""
  schedule(dh.DEFAULT_LAT_ACCEL_BREAKPOINTS_MPH)
  clock = [1000.0]
  monkeypatch.setattr(dh.time, "monotonic", lambda: clock[0])
  for mph in (40.0, 72.0, 85.0):
    for _ in range(100):
      clock[0] += 0.05
      got = dh.lat_accel_limit(mph * MPH)
    assert got == pytest.approx(dh.lat_accel_target(mph * MPH), abs=1e-12)


# ---------------------------------------------------------------------------------------------------------------------
# the start event
# ---------------------------------------------------------------------------------------------------------------------
def _controller(fp, brand, op_long, monkeypatch):
  from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
  from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_ces_mode_read_failure_logged import _P
  events, errors = [], []
  monkeypatch.setattr(m.cloudlog, "event", lambda name, **kw: events.append((name, kw)))
  monkeypatch.setattr(m.cloudlog, "error", lambda msg, *a, **kw: errors.append(msg))
  m.CESController(CP(fp, brand, op_long), params=_P([0.0]))
  return [kw for name, kw in events if name == "curve_brain_cfg"], [e for e in errors if e.startswith("curve_brain")]


def test_the_tesla_logs_its_curve_brain_config_at_start(cfg, monkeypatch):
  path = cfg({"tesla": {"curve_lat_a": 3.0, "curve_brain": "shadow"}})
  ev, errs = _controller(TESLA, "tesla", True, monkeypatch)
  assert ev == [{"mode": "shadow", "why": "curve.json", "lat_a": 3.0, "clip_margin": 0.3, "path": path}]
  assert errs == []


def test_a_config_it_did_not_honour_is_an_error_at_start(cfg, monkeypatch):
  cfg({"tesla": {"curve_brain": "sideways"}})
  ev, errs = _controller(TESLA, "tesla", True, monkeypatch)
  assert len(ev) == 1 and ev[0]["mode"] == "shadow" and ev[0]["why"].startswith("INVALID")
  assert len(errs) == 1 and "sideways" in errs[0]


def test_the_lightning_logs_no_curve_brain_config(cfg, monkeypatch):
  cfg({"tesla": {"curve_brain": "sideways"}})
  ev, errs = _controller(LIGHTNING, "ford", False, monkeypatch)
  assert ev == [] and errs == []
