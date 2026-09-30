"""latcar2pnw -- the optional per-car entry in lataccel_limits.json ("cars": {platform: {"breakpoints": ...}}).

Pinned: per-car selection (the Tesla's own schedule), the Lightning unchanged (no entry = the shared schedule, bit for bit),
an invalid entry -> the SHARED schedule + a loud error (never a looser value), a missing/invalid file -> flat 3.0 for both
cars, the slew still applies to a per-car target, and the Tesla curve brain's A at 60/70/80/90 mph.
"""
import json

import pytest

from openpilot.selfdrive.controls.lib import drive_helpers as dh
from openpilot.selfdrive.controls.lib import pnw_vehicle as pv

MPH = 0.44704
TESLA = "TESLA_MODEL_S_HW3"
LIGHTNING = "FORD_F_150_LIGHTNING_MK1"
SHARED = [[50, 5.0], [60, 5.0], [70, 4.0], [80, 3.0]]     # the device's live shared schedule
TESLA_BP = [[50, 5.0], [60, 5.0], [70, 4.0], [80, 3.9]]
CEIL = 3.5886


class CP:
  def __init__(self, fp, brand, op_long):
    self.carFingerprint, self.brand, self.openpilotLongitudinalControl, self.dashcamOnly = fp, brand, op_long, False


class _Log:
  def __init__(self):
    self.errors, self.warnings, self.infos = [], [], []

  def error(self, msg, *a, **k):
    self.errors.append(msg)

  def warning(self, msg, *a, **k):
    self.warnings.append(msg)

  def info(self, msg, *a, **k):
    self.infos.append(msg)

  def __getattr__(self, _):
    return lambda *a, **k: None


@pytest.fixture
def log(monkeypatch):
  lg = _Log()
  monkeypatch.setattr(dh, "cloudlog", lg)
  return lg


@pytest.fixture
def sched(tmp_path, monkeypatch):
  """A fresh schedule reading our own file; write(doc) takes the whole JSON document (None = no file)."""
  p = tmp_path / "lataccel_limits.json"
  monkeypatch.setattr(dh, "LAT_ACCEL_LIMITS_PATH", str(p))
  monkeypatch.setattr(dh, "_lat_accel_schedule", dh._LatAccelSchedule())

  def write(doc):
    if doc is not None:
      p.write_text(doc if isinstance(doc, str) else json.dumps(doc))
  return write


def _doc(cars=None, bp=SHARED):
  d = {"breakpoints": bp}
  if cars is not None:
    d["cars"] = cars
  return d


def tesla():
  return pv.PnwVehicle(CP(TESLA, "tesla", True))


def lightning():
  return pv.PnwVehicle(CP(LIGHTNING, "ford", False))


def test_a_car_uses_its_own_entry_and_others_the_shared_schedule(sched, log):
  sched(_doc({TESLA: {"breakpoints": TESLA_BP}}))
  for mph, tes, shared in ((60, 5.0, 5.0), (70, 4.0, 4.0), (75, 3.95, 3.5), (80, 3.9, 3.0), (90, 3.9, 3.0)):
    assert dh.lat_accel_target(mph * MPH, TESLA) == pytest.approx(tes)
    assert dh.lat_accel_target(mph * MPH, LIGHTNING) == pytest.approx(shared)
    assert dh.lat_accel_target(mph * MPH) == pytest.approx(shared)
  assert log.errors == []


def test_the_lightning_is_unchanged_by_the_presence_of_the_tesla_entry(sched, log):
  sched(_doc())
  base = [dh.lat_accel_target(m * MPH, LIGHTNING) for m in range(0, 121, 5)]
  sched(_doc({TESLA: {"breakpoints": TESLA_BP}}))
  dh._lat_accel_schedule._last_check_mono = 0.0
  assert [dh.lat_accel_target(m * MPH, LIGHTNING) for m in range(0, 121, 5)] == base
  assert lightning().curve_override_platform == LIGHTNING and tesla().curve_override_platform == TESLA


@pytest.mark.parametrize("entry", [
  {"breakpoints": [[50, 5.0]]},                       # too short
  {"breakpoints": [[50, 5.0], [50, 4.0]]},            # duplicate speed
  {"breakpoints": [[50, 5.0], [80, 9.0]]},            # out of the [1, 6] clamp
  {"breakpoints": [[50, 5.0], [80, float("nan")]]},   # non-finite
  {"breakpoints": [[50, "5"], [80, 3.0]]},            # wrong type
  {"breakpoints": [[50, True], [80, 3.0]]},           # bool
  {"nope": 1},                                        # no breakpoints
  "3.9",                                              # not an object
  None,
])
def test_an_invalid_entry_falls_back_to_the_shared_schedule_and_is_an_error(sched, log, entry):
  sched(_doc({TESLA: entry}))
  assert dh.lat_accel_target(85 * MPH, TESLA) == pytest.approx(3.0)      # shared, NOT higher
  assert dh.lat_accel_target(65 * MPH, TESLA) == pytest.approx(4.5)
  assert len(log.errors) == 1 and TESLA in log.errors[0] and "shared" in log.errors[0]


def test_a_bad_entry_does_not_invalidate_the_file_or_another_cars_entry(sched, log):
  sched(_doc({TESLA: {"breakpoints": "x"}, LIGHTNING: {"breakpoints": [[50, 4.0], [80, 3.0]]}}))
  assert dh.lat_accel_target(85 * MPH, TESLA) == pytest.approx(3.0)
  assert dh.lat_accel_target(50 * MPH, LIGHTNING) == pytest.approx(4.0)
  assert dh.lat_accel_target(50 * MPH) == pytest.approx(5.0)


def test_cars_that_is_not_an_object_is_ignored_with_an_error(sched, log):
  sched(_doc([1, 2]))
  assert dh.lat_accel_target(85 * MPH, TESLA) == pytest.approx(3.0)
  assert len(log.errors) == 1 and "cars" in log.errors[0]


@pytest.mark.parametrize("state", ["missing", "corrupt", "no_shared"])
def test_a_missing_or_invalid_file_is_flat_3_0_for_both_cars(sched, log, state):
  if state == "corrupt":
    sched('{"cars": {"TESLA_MODEL_S_HW3": {"breakpoints": [[50, 5], [80, 3.9]]}}, "breakpoints": ')
  elif state == "no_shared":
    sched({"cars": {TESLA: {"breakpoints": TESLA_BP}}})   # the per-car entry alone must not activate anything
  # (missing: no file written; the one-time default seed cannot write here because the path's dir is a tmp dir -- it may
  # seed the SHARED default, so pin the seed away)
  for mph in (30, 60, 85):
    if state == "missing":
      dh._lat_accel_schedule._wrote_default = True          # no seeding: the file stays absent
    assert dh.lat_accel_target(mph * MPH, TESLA) == 3.0
    assert dh.lat_accel_target(mph * MPH, LIGHTNING) == 3.0
    assert dh.lat_accel_target(mph * MPH) == 3.0


def test_the_slew_still_applies_to_a_per_car_target(sched, monkeypatch):
  sched(_doc({TESLA: {"breakpoints": TESLA_BP}}))
  clock = [1000.0]
  monkeypatch.setattr(dh.time, "monotonic", lambda: clock[0])
  first = dh.lat_accel_limit(60 * MPH, TESLA)
  assert first == 3.0                                        # starts at the ISO baseline, target is 5.0
  clock[0] += 0.05
  second = dh.lat_accel_limit(60 * MPH, TESLA)
  assert second == pytest.approx(3.0 + dh.LAT_ACCEL_SLEW_RATE * 0.05) and second < 5.0
  for _ in range(100):
    clock[0] += 0.05
    got = dh.lat_accel_limit(60 * MPH, TESLA)
  assert got == pytest.approx(5.0)


def test_clip_curvature_takes_the_platform(sched, monkeypatch):
  sched(_doc({TESLA: {"breakpoints": TESLA_BP}}))
  clock = [1000.0]
  monkeypatch.setattr(dh.time, "monotonic", lambda: clock[0])
  v = 90 * MPH
  for _ in range(200):
    clock[0] += 0.05
    dh.lat_accel_limit(v, TESLA)
  want = 3.85 / v ** 2                                       # 3.85 m/s^2 at 90 mph: above the shared 3.0, under the Tesla's 3.9
  clock[0] += 0.1                                            # one real tick: a dropped platform would slew toward the shared 3.0
  k, limited = dh.clip_curvature(v, want, want, 0.0, TESLA)
  assert k == pytest.approx(want) and limited is False


@pytest.mark.parametrize("mph", [30, 50, 60, 65, 70, 75, 80, 85, 90, 100, 120])
def test_the_tesla_brain_prices_at_the_steering_ceiling_at_every_speed_with_its_entry(sched, mph):
  sched(_doc({TESLA: {"breakpoints": TESLA_BP}}))
  assert tesla().curve_lat_a(mph * MPH) == pytest.approx(CEIL, abs=1e-9)


@pytest.mark.parametrize("mph,want", [(60, CEIL), (70, CEIL), (80, 2.7), (90, 2.7)])
def test_without_the_entry_the_tesla_brain_is_as_before(sched, mph, want):
  sched(_doc())
  assert tesla().curve_lat_a(mph * MPH) == pytest.approx(want, abs=1e-9)


def test_the_tesla_schedule_minus_margin_never_binds_below_the_ceiling():
  """The chosen values themselves: schedule - 0.3 >= the vehicle-model clamp at every breakpoint (linear between)."""
  assert all(a - pv.CURVE_LAT_CLIP_MARGIN >= CEIL for _, a in TESLA_BP)


@pytest.mark.parametrize("key", ["TESLA_MODEL_S_HW3 ", "TESLA_MODEL_S", "tesla_model_s_hw3"])
def test_a_misspelled_platform_key_warns_that_the_car_uses_the_shared_schedule(sched, log, key):
  sched(_doc({key: {"breakpoints": TESLA_BP}}))
  assert dh.lat_accel_target(85 * MPH, TESLA) == pytest.approx(3.0)
  msgs = [m for m in log.warnings if "uses the shared schedule" in m]
  assert len(msgs) == 1 and TESLA in msgs[0] and repr(key) in msgs[0]
  assert any("accepted for" in m and key in m for m in log.warnings)
  assert not [m for m in log.infos if "own schedule" in m]


def test_a_matching_key_says_info_own_schedule_once_per_load(sched, log):
  sched(_doc({TESLA: {"breakpoints": TESLA_BP}}))
  for _ in range(5):
    dh.lat_accel_target(85 * MPH, TESLA)
  assert all(TESLA in m for m in log.infos if "own schedule" in m)
  assert len([m for m in log.infos if "own schedule" in m]) == 1
  assert not [m for m in log.warnings if "uses the shared schedule" in m]


def test_no_cars_section_is_info_shared_not_a_warning(sched, log):
  sched(_doc())
  dh.lat_accel_report_platform(TESLA)
  assert len([m for m in log.infos if "shared schedule" in m]) == 1
  assert not [m for m in log.warnings if "shared schedule" in m]


def test_a_hot_reload_is_reported_again_and_no_platform_is_silent(sched, log):
  import os
  sched(_doc({TESLA: {"breakpoints": TESLA_BP}}))
  dh.lat_accel_report_platform(None)                        # a car without the capability reports nothing
  assert not log.infos
  dh.lat_accel_report_platform(TESLA)
  sched(_doc({"TESLA_MODEL_S": {"breakpoints": TESLA_BP}}))  # the driver mistypes it on the road
  st = os.stat(dh.LAT_ACCEL_LIMITS_PATH)
  os.utime(dh.LAT_ACCEL_LIMITS_PATH, ns=(st.st_atime_ns, st.st_mtime_ns + 5_000_000_000))
  dh._lat_accel_schedule._last_check_mono = 0.0
  dh.lat_accel_target(80 * MPH, TESLA)
  assert len([m for m in log.warnings if "uses the shared schedule" in m]) == 1


def test_the_default_seed_failure_is_logged(tmp_path, monkeypatch, log):
  monkeypatch.setattr(dh, "LAT_ACCEL_LIMITS_PATH", str(tmp_path / "nodir" / "x" / "l.json"))
  monkeypatch.setattr(dh.os, "makedirs", lambda *a, **k: (_ for _ in ()).throw(PermissionError("ro")))
  dh._LatAccelSchedule()._write_default_once()
  assert len([m for m in log.warnings if "could not seed" in m]) == 1
