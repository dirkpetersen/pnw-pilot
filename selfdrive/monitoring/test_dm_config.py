"""dm-variable offline unit tests: JSON tier config resolution + helpers wiring + dm CLI.

Pure-offline: no device, no CAN, no params daemon. The helpers-level tests skip automatically
where the openpilot deps (cereal/numpy) are unavailable.
"""
import json
import os
import re
import subprocess
import sys

import pytest

from openpilot.selfdrive.monitoring import dm_config
from openpilot.selfdrive.monitoring.dm_config import load_dm_tier, load_dm_timeouts

MONITORING_DIR = os.path.dirname(os.path.abspath(__file__))
DM_CLI = os.path.join(MONITORING_DIR, "..", "..", "tools", "dm")

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

  def load_all(self):
    return load_dm_timeouts(self.path, self.warn)


@pytest.fixture
def cfg(tmp_path):
  return ConfigCase(tmp_path)


class TestSourcePurity:
  """Pin that the removed long-timeout constants never return to the DM source. The controversial
  values live exclusively in the device-local /data/pnw/dm.json — never in this repo."""

  BANNED = re.compile(r"\b(10800|3600|1800|900)(\.\d*)?\b")
  SOURCES = ("helpers.py", "dmonitoringd.py", "dm_config.py")

  @pytest.mark.parametrize("fname", SOURCES)
  def test_no_long_timeout_constants_in_source(self, fname):
    with open(os.path.join(MONITORING_DIR, fname), encoding="utf-8") as f:
      for i, line in enumerate(f, 1):
        m = self.BANNED.search(line)
        assert m is None, f"{fname}:{i}: banned long-timeout constant {m.group(0)!r} in: {line.strip()}"


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
    # a directory at the config path -> rejected by the S_ISREG check
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
    assert cfg.load() == ("highway", dm_config.TIMEOUT_MIN_S, dm_config.TIMEOUT_MAX_S)
    assert len(cfg.warnings) == 2
    cfg.write({"mode": "highway", "highway": {"pose_s": -50, "phone_s": 0}})
    assert cfg.load() == ("highway", dm_config.TIMEOUT_MIN_S, dm_config.TIMEOUT_MIN_S)

  def test_bounds_values(self):
    # the widened external-only ceiling: personal values up to 4 h via JSON, never below 10 s
    assert dm_config.TIMEOUT_MIN_S == 10.0
    assert dm_config.TIMEOUT_MAX_S == 14400.0

  def test_long_personal_values_allowed_via_json_only(self, cfg):
    # values that must never appear in source are configurable through the file
    cfg.write({"mode": "highway", "highway": {"pose_s": 3599, "phone_s": 14400}})
    assert cfg.load() == ("highway", 3599.0, 14400.0)
    cfg.write({"mode": "highway", "highway": {"pose_s": 14401, "phone_s": 1e12}})
    assert cfg.load() == ("highway", 14400.0, 14400.0)  # clamped at the ceiling

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
    cfg.write({"mode": "relaxed", "relaxed": {"enabled": True, "pose_s": 20000, "phone_s": 3}})
    assert cfg.load() == ("relaxed", 14400.0, 10.0)  # clamped, never beyond the ceiling

  def test_never_raises_even_with_raising_warn(self, cfg):
    def bad_warn(msg):
      raise RuntimeError("logger blew up")
    cfg.write(None, raw="garbage")
    assert load_dm_tier(cfg.path, bad_warn) is None

  def test_hardcoded_defaults_do_not_depend_on_file(self, tmp_path):
    # the strict constants live in the Python source; nothing here touches any file
    assert dm_config.TIER_DEFAULTS["highway"] == (30.0, 60.0)
    assert dm_config.TIER_DEFAULTS["relaxed"] == (60.0, 120.0)
    assert load_dm_tier(str(tmp_path / "nope" / "dm.json")) is None


class TestLoadDmTimeouts:
  """The one-read table consumed by helpers: tier + per-regime values for the DmMode selector."""

  def test_no_file_is_all_strict(self, cfg):
    out = cfg.load_all()
    assert out == {"tier": None, "highway": (30.0, 60.0), "relaxed": (60.0, 120.0)}

  def test_malformed_is_all_strict(self, cfg):
    cfg.write(None, raw="!!!")
    assert cfg.load_all() == {"tier": None, "highway": (30.0, 60.0), "relaxed": (60.0, 120.0)}

  def test_highway_values_used_without_mode(self, cfg):
    # DmMode=1 (Settings) consumes highway values even when the JSON mode stays default
    cfg.write({"highway": {"pose_s": 300, "phone_s": 500}})
    out = cfg.load_all()
    assert out["tier"] is None
    assert out["highway"] == (300.0, 500.0)

  def test_relaxed_values_require_enabled_even_for_dmmode(self, cfg):
    # without enabled:true the DmMode=2 regime stays at the strict defaults
    cfg.write({"relaxed": {"pose_s": 5000, "phone_s": 5000}})
    assert cfg.load_all()["relaxed"] == (60.0, 120.0)
    cfg.write({"relaxed": {"enabled": True, "pose_s": 5000, "phone_s": 5000}})
    assert cfg.load_all()["relaxed"] == (5000.0, 5000.0)


@pytest.mark.skipif(not HELPERS_AVAILABLE, reason="openpilot helpers deps unavailable")
class TestHelpersTierWiring:
  """_apply_dm_timeouts: strict stock with nothing configured; DmMode regimes use dm_config values;
  the JSON tier takes precedence, with its highway variant road-gated."""

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

  def test_dmmode_relaxed_uses_strict_defaults_without_json(self):
    self.dm._dm_tier = None
    self.dm._dm_mode = 2
    self.dm._apply_dm_timeouts()
    assert self.dm._pose_step == pytest.approx(DT_DMON / 60.0)
    assert self.dm._phone_step == pytest.approx(DT_DMON / 120.0)

  def test_dmmode_highway_uses_strict_defaults_without_json(self):
    self.dm._dm_tier = None
    self.dm._dm_mode = 1
    self.dm._road_relaxed = True
    self.dm._apply_dm_timeouts()
    assert self.dm._pose_step == pytest.approx(DT_DMON / 30.0)
    assert self.dm._phone_step == pytest.approx(DT_DMON / 60.0)

  def test_dmmode_highway_offroad_is_strict(self):
    self.dm._dm_tier = None
    self.dm._dm_mode = 1
    self.dm._road_relaxed = False
    self.dm._apply_dm_timeouts()
    assert self.dm._pose_step == pytest.approx(DT_DMON / self.s._DISTRACTED_TIME)

  def test_dmmode_regimes_use_json_values(self):
    self.dm._dm_tier = None
    self.dm._dm_highway_t = (300.0, 500.0)
    self.dm._dm_relaxed_t = (4000.0, 2000.0)
    self.dm._dm_mode = 1
    self.dm._road_relaxed = True
    self.dm._apply_dm_timeouts()
    assert self.dm._pose_step == pytest.approx(DT_DMON / 300.0)
    assert self.dm._phone_step == pytest.approx(DT_DMON / 500.0)
    self.dm._dm_mode = 2
    self.dm._apply_dm_timeouts()
    assert self.dm._pose_step == pytest.approx(DT_DMON / 4000.0)
    assert self.dm._phone_step == pytest.approx(DT_DMON / 2000.0)

  def test_json_highway_tier_road_gated(self):
    self.dm._dm_tier = ("highway", 30.0, 60.0)
    self.dm._dm_mode = 0
    # off-freeway -> strict stock, the tier is inert
    self.dm._road_relaxed = False
    self.dm._apply_dm_timeouts()
    assert self.dm._pose_step == pytest.approx(DT_DMON / self.s._DISTRACTED_TIME)
    assert self.dm._phone_step == pytest.approx(DT_DMON / self.s._DISTRACTED_TIME)
    # on-freeway -> tier values
    self.dm._road_relaxed = True
    self.dm._apply_dm_timeouts()
    assert self.dm._pose_step == pytest.approx(DT_DMON / 30.0)
    assert self.dm._phone_step == pytest.approx(DT_DMON / 60.0)
    # leads capped: pose pre min(60, 15)=15 -> 0.5, prompt min(30, 7.5)=7.5 -> 0.25
    assert self.dm._pose_threshold_pre == pytest.approx(0.5)
    assert self.dm._pose_threshold_prompt == pytest.approx(0.25)
    assert self.dm._phone_threshold_pre == pytest.approx(0.5)
    assert self.dm._phone_threshold_prompt == pytest.approx(0.25)

  def test_json_highway_offroad_never_falls_through_to_dmmode(self):
    # JSON highway off-freeway must land on STRICT even if the Settings param says Relaxed
    self.dm._dm_tier = ("highway", 100.0, 200.0)
    self.dm._dm_mode = 2
    self.dm._road_relaxed = False
    self.dm._apply_dm_timeouts()
    assert self.dm._pose_step == pytest.approx(DT_DMON / self.s._DISTRACTED_TIME)

  def test_json_relaxed_tier_everywhere_beats_dmmode(self):
    self.dm._dm_tier = ("relaxed", 60.0, 120.0)
    self.dm._dm_mode = 1
    self.dm._road_relaxed = False  # road must not matter for the relaxed tier
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
    assert load_dm_tier(self.cfg.path) == ("highway", dm_config.TIMEOUT_MIN_S, dm_config.TIMEOUT_MAX_S)
    self.run_dm("highway", "abc", "60", expect_fail=True)
    self.run_dm("highway", "nan", "60", expect_fail=True)
    self.run_dm("highway", "30", expect_fail=True)      # missing phone_s
    self.run_dm("mode", "turbo", expect_fail=True)
    self.run_dm("highway", "--enable", expect_fail=True)  # enable is relaxed-only
    self.run_dm("bogus", expect_fail=True)

  def test_personal_long_values_settable(self):
    self.run_dm("relaxed", "7200", "14400")
    self.run_dm("relaxed", "--enable")
    self.run_dm("mode", "relaxed")
    assert load_dm_tier(self.cfg.path) == ("relaxed", 7200.0, 14400.0)

  def test_show_runs_on_missing_and_garbage(self):
    r = self.run_dm("show")
    assert "default" in r.stdout
    self.cfg.write(None, raw="{broken")
    r = self.run_dm("show")
    assert "default" in r.stdout
