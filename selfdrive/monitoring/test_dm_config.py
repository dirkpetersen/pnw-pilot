"""dm-variable offline unit tests: JSON tier config resolution + helpers wiring + dm CLI.

Pure-offline: no device, no CAN, no params daemon. The helpers-level tests skip automatically
where the openpilot deps (cereal/numpy) are unavailable.
"""
import json
import os
import subprocess
import sys

import pytest

from openpilot.selfdrive.monitoring import dm_config
from openpilot.selfdrive.monitoring.dm_config import load_dm_tier

DM_CLI = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "tools", "dm")

try:
  from openpilot.common.realtime import DT_DMON
  from openpilot.selfdrive.monitoring.helpers import DriverMonitoring, DRIVER_MONITOR_SETTINGS
  from openpilot.system.hardware import HARDWARE
  HELPERS_AVAILABLE = True
except Exception:
  HELPERS_AVAILABLE = False


class ConfigCase:
  def __init__(self, tmp_path):
    self.path = str(tmp_path / "dm.json")
    self.warnings = []

  def warn(self, msg):
    self.warnings.append(msg)

  def write(self, obj, raw=None):
    with open(self.path, "w", encoding="utf-8") as f:
      f.write(raw if raw is not None else json.dumps(obj))

  def load(self):
    return load_dm_tier(self.path, self.warn)


@pytest.fixture
def cfg(tmp_path):
  return ConfigCase(tmp_path)


class TestLoadDmTier:
  def test_missing_file_is_default(self, cfg):
    assert cfg.load() is None
    assert cfg.warnings == []  # missing file is the normal state, not a warning

  def test_malformed_json_is_default_no_exception(self, cfg):
    cfg.write(None, raw="{not json!!")
    assert cfg.load() is None
    assert cfg.warnings

  def test_top_level_not_object_is_default(self, cfg):
    cfg.write([1, 2, 3])
    assert cfg.load() is None
    assert cfg.warnings

  def test_unreadable_path_is_default(self, cfg):
    # a directory at the config path -> IsADirectoryError on open
    os.mkdir(cfg.path)
    assert cfg.load() is None
    assert cfg.warnings

  def test_fifo_at_config_path_is_default_no_hang(self, cfg):
    # a FIFO would block open() forever — must be rejected on the stat, not opened
    os.mkfifo(cfg.path)
    assert cfg.load() is None
    assert cfg.warnings

  def test_oversized_file_is_default(self, cfg):
    with open(cfg.path, "w", encoding="utf-8") as f:
      f.write("[" + "1," * (dm_config.MAX_CONFIG_BYTES // 2) + "1]")
    assert cfg.load() is None
    assert any("bytes" in w for w in cfg.warnings)

  def test_mode_default_or_absent(self, cfg):
    cfg.write({"mode": "default", "highway": {"pose_s": 45, "phone_s": 90}})
    assert cfg.load() is None
    cfg.write({"highway": {"pose_s": 45, "phone_s": 90}})  # no mode key
    assert cfg.load() is None

  @pytest.mark.parametrize("bad", ["yolo", 1, True, None, ["highway"]])
  def test_invalid_mode_is_default(self, cfg, bad):
    cfg.write({"mode": bad, "highway": {"pose_s": 45, "phone_s": 90}})
    assert cfg.load() is None
    if bad is not None:  # absent-equivalent None warns too (it is present-but-wrong-type)
      assert cfg.warnings

  def test_highway_happy_path(self, cfg):
    cfg.write({"mode": "highway", "highway": {"pose_s": 45, "phone_s": 90}})
    assert cfg.load() == ("highway", 45.0, 90.0)
    assert cfg.warnings == []

  def test_highway_defaults_when_section_missing(self, cfg):
    cfg.write({"mode": "highway"})
    assert cfg.load() == ("highway", 30.0, 60.0)
    assert cfg.warnings == []

  def test_highway_bad_value_falls_back_per_field(self, cfg):
    cfg.write({"mode": "highway", "highway": {"pose_s": "abc", "phone_s": 90}})
    assert cfg.load() == ("highway", 30.0, 90.0)
    assert cfg.warnings

  @pytest.mark.parametrize("bad", [True, None, [30], {"s": 30}, float("nan"), float("inf")])
  def test_highway_bad_values_use_defaults(self, cfg, bad):
    cfg.write({"mode": "highway", "highway": {"pose_s": bad, "phone_s": bad}})
    assert cfg.load() == ("highway", 30.0, 60.0)

  def test_out_of_range_clamped(self, cfg):
    cfg.write({"mode": "highway", "highway": {"pose_s": 5, "phone_s": 99999}})
    assert cfg.load() == ("highway", 10.0, 600.0)
    assert len(cfg.warnings) == 2
    cfg.write({"mode": "highway", "highway": {"pose_s": -50, "phone_s": 0}})
    assert cfg.load() == ("highway", 10.0, 10.0)

  @pytest.mark.parametrize("enabled", [None, False, "true", 1, "yes"])  # only JSON true counts
  def test_relaxed_not_enabled_is_default(self, cfg, enabled):
    section = {"pose_s": 60, "phone_s": 120}
    if enabled is not None:
      section["enabled"] = enabled
    cfg.write({"mode": "relaxed", "relaxed": section})
    assert cfg.load() is None
    assert cfg.warnings

  def test_relaxed_enabled_happy_path(self, cfg):
    cfg.write({"mode": "relaxed", "relaxed": {"enabled": True, "pose_s": 60, "phone_s": 120}})
    assert cfg.load() == ("relaxed", 60.0, 120.0)
    cfg.write({"mode": "relaxed", "relaxed": {"enabled": True}})  # values default
    assert cfg.load() == ("relaxed", 60.0, 120.0)
    cfg.write({"mode": "relaxed", "relaxed": {"enabled": True, "pose_s": 700, "phone_s": 3}})
    assert cfg.load() == ("relaxed", 600.0, 10.0)  # clamped, never beyond the cap

  def test_never_raises_even_with_raising_warn(self, cfg):
    def bad_warn(msg):
      raise RuntimeError("logger blew up")
    cfg.write(None, raw="garbage")
    assert load_dm_tier(cfg.path, bad_warn) is None

  def test_hardcoded_defaults_do_not_depend_on_file(self, tmp_path):
    # the constants live in the Python source; nothing here touches any file
    assert dm_config.TIER_DEFAULTS["highway"] == (30.0, 60.0)
    assert dm_config.TIER_DEFAULTS["relaxed"] == (60.0, 120.0)
    assert load_dm_tier(str(tmp_path / "nope" / "dm.json")) is None


@pytest.mark.skipif(not HELPERS_AVAILABLE, reason="openpilot helpers deps unavailable")
class TestHelpersTierWiring:
  """The _apply_dm_timeouts override: tier None must be byte-identical to today; a tier must set
  exactly the tier's steps/thresholds with capped pre/prompt leads."""

  def setup_method(self):
    self.s = DRIVER_MONITOR_SETTINGS(device_type=HARDWARE.get_device_type())
    self.dm = DriverMonitoring(settings=self.s)

  def test_no_tier_matches_stock_strict(self):
    # fresh Params -> DmMode 0; no /data/pnw/dm.json on the test host -> tier None
    assert self.dm._dm_tier is None
    assert self.dm._pose_step == pytest.approx(DT_DMON / self.s._DISTRACTED_TIME)
    assert self.dm._phone_step == pytest.approx(DT_DMON / self.s._DISTRACTED_TIME)
    assert self.dm._pose_threshold_pre == pytest.approx(self.s._DISTRACTED_PRE_TIME_TILL_TERMINAL / self.s._DISTRACTED_TIME)
    assert self.dm._pose_threshold_prompt == pytest.approx(self.s._DISTRACTED_PROMPT_TIME_TILL_TERMINAL / self.s._DISTRACTED_TIME)

  def test_no_tier_relaxed_dmmode_unchanged(self):
    self.dm._dm_tier = None
    self.dm._dm_mode = 2
    self.dm._apply_dm_timeouts()
    assert self.dm._pose_step == pytest.approx(DT_DMON / self.s._POSE_DISTRACTED_TIME)
    assert self.dm._phone_step == pytest.approx(DT_DMON / self.s._PHONE_DISTRACTED_TIME)

  def test_highway_tier_overrides(self):
    self.dm._dm_tier = ("highway", 30.0, 60.0)
    self.dm._apply_dm_timeouts()
    assert self.dm._pose_step == pytest.approx(DT_DMON / 30.0)
    assert self.dm._phone_step == pytest.approx(DT_DMON / 60.0)
    # leads capped: pose pre min(60, 15)=15 -> 0.5, prompt min(30, 7.5)=7.5 -> 0.25
    assert self.dm._pose_threshold_pre == pytest.approx(0.5)
    assert self.dm._pose_threshold_prompt == pytest.approx(0.25)
    # phone pre min(120, 30)=30 -> 0.5, prompt min(60, 15)=15 -> 0.25
    assert self.dm._phone_threshold_pre == pytest.approx(0.5)
    assert self.dm._phone_threshold_prompt == pytest.approx(0.25)

  def test_relaxed_tier_overrides_and_beats_dmmode(self):
    self.dm._dm_tier = ("relaxed", 60.0, 120.0)
    self.dm._dm_mode = 1  # any DmMode: the JSON tier takes precedence
    self.dm._road_relaxed = True
    self.dm._apply_dm_timeouts()
    assert self.dm._pose_step == pytest.approx(DT_DMON / 60.0)
    assert self.dm._phone_step == pytest.approx(DT_DMON / 120.0)
    # pose pre min(60, 30)=30 -> 0.5, prompt min(30, 15)=15 -> 0.25
    assert self.dm._pose_threshold_pre == pytest.approx(0.5)
    assert self.dm._pose_threshold_prompt == pytest.approx(0.25)


class TestDmCli:
  @pytest.fixture(autouse=True)
  def _cfg(self, cfg):
    self.cfg = cfg

  def run_dm(self, *args, expect_fail=False):
    r = subprocess.run([sys.executable, DM_CLI, "--file", self.cfg.path, *args],
                       capture_output=True, text=True, timeout=30)
    if expect_fail:
      assert r.returncode != 0, r.stdout + r.stderr
    else:
      assert r.returncode == 0, r.stdout + r.stderr
    return r

  def test_round_trip(self):
    self.run_dm("highway", "45", "90")
    self.run_dm("mode", "highway")
    assert load_dm_tier(self.cfg.path) == ("highway", 45.0, 90.0)
    # relaxed values without enable -> still default tier
    self.run_dm("relaxed", "60", "120")
    self.run_dm("mode", "relaxed")
    assert load_dm_tier(self.cfg.path) is None
    self.run_dm("relaxed", "--enable")
    assert load_dm_tier(self.cfg.path) == ("relaxed", 60.0, 120.0)
    self.run_dm("relaxed", "--disable")
    assert load_dm_tier(self.cfg.path) is None
    self.run_dm("mode", "default")
    assert load_dm_tier(self.cfg.path) is None
    # earlier keys preserved
    with open(self.cfg.path, encoding="utf-8") as f:
      data = json.load(f)
    assert data["highway"] == {"pose_s": 45.0, "phone_s": 90.0}

  def test_clamping_and_validation(self):
    self.run_dm("highway", "5", "99999")  # clamps with a warning, still writes
    self.run_dm("mode", "highway")
    assert load_dm_tier(self.cfg.path) == ("highway", 10.0, 600.0)
    self.run_dm("highway", "abc", "60", expect_fail=True)
    self.run_dm("highway", "nan", "60", expect_fail=True)
    self.run_dm("highway", "30", expect_fail=True)      # missing phone_s
    self.run_dm("mode", "turbo", expect_fail=True)
    self.run_dm("highway", "--enable", expect_fail=True)  # enable is relaxed-only
    self.run_dm("bogus", expect_fail=True)

  def test_show_runs_on_missing_and_garbage(self):
    r = self.run_dm("show")
    assert "default" in r.stdout
    self.cfg.write(None, raw="{broken")
    r = self.run_dm("show")
    assert "default" in r.stdout
