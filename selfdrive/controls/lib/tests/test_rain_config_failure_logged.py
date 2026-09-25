"""deleterrain2pnw -- _load_rain_config must SAY SO when /data/pnw/rain.json is unusable (Rule 2).

Mirror of test_curve_config_failure_logged.py. Every fallback used to be silent: a typo in rain.json put BOTH cars
back on the default 3/5 mph rain margins with no trace. Fallbacks are unchanged. A MISSING file stays silent (the
documented default); an unusable, unreadable or malformed file is a cloudlog.error naming the path and the error;
clamped / NaN values are one warning per load.
"""
import os

import pytest

from openpilot.selfdrive.controls.lib import pnw_vehicle as pv


class _Log:
  def __init__(self):
    self.lines = []

  def _rec(self, level):
    return lambda msg, *a, **k: self.lines.append((level, msg))

  def __getattr__(self, name):
    if name in ("debug", "info", "warning", "error", "exception", "critical"):
      return self._rec(name)
    raise AttributeError(name)

  def at(self, level):
    return [m for lvl, m in self.lines if lvl == level]


@pytest.fixture
def log(monkeypatch):
  lg = _Log()
  monkeypatch.setattr(pv, "cloudlog", lg)
  return lg


def _cfg(monkeypatch, tmp_path, text=None, name="rain.json"):
  p = tmp_path / name
  if text is not None:
    p.write_text(text)
  monkeypatch.setattr(pv, "RAIN_CONFIG_PATH", str(p))
  return str(p)


def test_missing_file_is_silent_and_defaults(monkeypatch, tmp_path, log):
  _cfg(monkeypatch, tmp_path)
  assert pv._load_rain_config() == pv._RAIN_DEFAULTS
  assert log.lines == []


def test_valid_file_in_bounds_is_silent(monkeypatch, tmp_path, log):
  _cfg(monkeypatch, tmp_path, '{"light_mph": 4.0, "heavy_mph": 7.0, "unknown": 1}')
  cfg = pv._load_rain_config()
  assert cfg == {"light_mph": 4.0, "heavy_mph": 7.0}
  assert log.lines == []


@pytest.mark.parametrize("text, err", [
  ('{"light_mph": 4.0', "JSONDecodeError"),       # truncated JSON
  ('{"light_mph": "fast"}', "ValueError"),        # float("fast")
  ('{"light_mph": [1]}', "TypeError"),            # float([1])
])
def test_parse_error_is_logged_with_path_and_error_and_defaults(monkeypatch, tmp_path, log, text, err):
  path = _cfg(monkeypatch, tmp_path, text)
  assert pv._load_rain_config() == pv._RAIN_DEFAULTS                 # fallback unchanged: whole file ignored
  errs = log.at("error")
  assert len(errs) == 1 and path in errs[0] and err in errs[0]


def test_unreadable_file_is_logged(monkeypatch, tmp_path, log):
  if os.geteuid() == 0:
    pytest.skip("root reads a 0o000 file")
  path = _cfg(monkeypatch, tmp_path, '{"light_mph": 4.0}')
  os.chmod(path, 0)
  try:
    assert pv._load_rain_config() == pv._RAIN_DEFAULTS
  finally:
    os.chmod(path, 0o644)
  errs = log.at("error")
  assert len(errs) == 1 and path in errs[0] and "PermissionError" in errs[0]


def test_not_a_regular_file_is_logged(monkeypatch, tmp_path, log):
  (tmp_path / "rain.json").mkdir()
  path = _cfg(monkeypatch, tmp_path)
  assert pv._load_rain_config() == pv._RAIN_DEFAULTS
  errs = log.at("error")
  assert len(errs) == 1 and path in errs[0]


def test_oversize_file_is_logged(monkeypatch, tmp_path, log):
  path = _cfg(monkeypatch, tmp_path, '{}' + " " * (pv._RAIN_CONFIG_MAX_BYTES + 1))
  assert pv._load_rain_config() == pv._RAIN_DEFAULTS
  errs = log.at("error")
  assert len(errs) == 1 and path in errs[0]


@pytest.mark.parametrize("text", ['[1, 2]', '5', '"light"'])
def test_wrong_shape_is_logged(monkeypatch, tmp_path, log, text):
  path = _cfg(monkeypatch, tmp_path, text)
  assert pv._load_rain_config() == pv._RAIN_DEFAULTS
  errs = log.at("error")
  assert len(errs) == 1 and path in errs[0]


def test_clamped_keys_are_named_once_per_load(monkeypatch, tmp_path, log):
  lo, hi = pv._RAIN_BOUNDS["light_mph"]
  _cfg(monkeypatch, tmp_path, f'{{"light_mph": {hi + 100}, "heavy_mph": {lo - 1}}}')
  cfg = pv._load_rain_config()
  assert cfg["light_mph"] == hi and cfg["heavy_mph"] == lo            # clamped exactly as before
  warns = log.at("warning")
  assert len(warns) == 1                                               # ONE line per load
  assert "light_mph" in warns[0] and "heavy_mph" in warns[0]
  assert log.at("error") == []


def test_only_the_clamped_key_is_named(monkeypatch, tmp_path, log):
  _, hi = pv._RAIN_BOUNDS["heavy_mph"]
  _cfg(monkeypatch, tmp_path, f'{{"light_mph": 2.0, "heavy_mph": {hi + 1}}}')
  cfg = pv._load_rain_config()
  assert cfg == {"light_mph": 2.0, "heavy_mph": hi}
  warns = log.at("warning")
  assert len(warns) == 1 and "heavy_mph" in warns[0] and "light_mph" not in warns[0]


def test_nan_is_named_and_keeps_the_default(monkeypatch, tmp_path, log):
  _cfg(monkeypatch, tmp_path, '{"heavy_mph": NaN}')
  cfg = pv._load_rain_config()
  assert cfg["heavy_mph"] == pv._RAIN_DEFAULTS["heavy_mph"]
  warns = log.at("warning")
  assert len(warns) == 1 and "heavy_mph" in warns[0] and "NaN" in warns[0]
