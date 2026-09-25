"""rule2fixes2pnw -- _load_curve_config must SAY SO when /data/pnw/curve.json is unusable (Rule 2).

Every fallback used to be silent: a typo in curve.json put the Lightning back on the default curve ramp with no
trace. Fallbacks are unchanged. A MISSING file stays silent (the documented default); an unusable, unreadable or
malformed file is a cloudlog.error naming the path and the error; clamped / NaN values are one warning per load.
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


def _cfg(monkeypatch, tmp_path, text=None, name="curve.json"):
  p = tmp_path / name
  if text is not None:
    p.write_text(text)
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(p))
  return str(p)


def _is_defaults(cfg):
  """curveshape2pnw: the loader also returns the string knob icbm_shape (+ its why), outside _CURVE_DEFAULTS. A
  fallback is the numeric defaults AND the stage's default mode."""
  numeric = {k: v for k, v in cfg.items() if k not in ("icbm_shape", "icbm_shape_why")}
  return numeric == pv._CURVE_DEFAULTS and cfg["icbm_shape"] == pv.SHAPE_MODE_DEFAULT


def test_missing_file_is_silent_and_defaults(monkeypatch, tmp_path, log):
  _cfg(monkeypatch, tmp_path)
  assert _is_defaults(pv._load_curve_config())
  assert log.lines == []


def test_valid_file_in_bounds_is_silent(monkeypatch, tmp_path, log):
  _cfg(monkeypatch, tmp_path, '{"lightning": {"penalty_max_mph": 6.0}}')
  cfg = pv._load_curve_config()
  assert cfg["penalty_max_mph"] == 6.0
  assert log.lines == []


@pytest.mark.parametrize("text, err", [
  ('{"lightning": {"penalty_max_mph": 6.0', "JSONDecodeError"),       # truncated JSON
  ('{"lightning": {"penalty_max_mph": "fast"}}', "ValueError"),       # float("fast")
  ('{"lightning": {"penalty_max_mph": [1]}}', "TypeError"),           # float([1])
])
def test_parse_error_is_logged_with_path_and_error_and_defaults(monkeypatch, tmp_path, log, text, err):
  path = _cfg(monkeypatch, tmp_path, text)
  assert _is_defaults(pv._load_curve_config())                # fallback unchanged: whole file ignored
  errs = log.at("error")
  assert len(errs) == 1 and path in errs[0] and err in errs[0]


def test_unreadable_file_is_logged(monkeypatch, tmp_path, log):
  if os.geteuid() == 0:
    pytest.skip("root reads a 0o000 file")
  path = _cfg(monkeypatch, tmp_path, '{"lightning": {"penalty_max_mph": 6.0}}')
  os.chmod(path, 0)
  try:
    assert _is_defaults(pv._load_curve_config())
  finally:
    os.chmod(path, 0o644)
  errs = log.at("error")
  assert len(errs) == 1 and path in errs[0] and "PermissionError" in errs[0]


def test_not_a_regular_file_is_logged(monkeypatch, tmp_path, log):
  (tmp_path / "curve.json").mkdir()
  path = _cfg(monkeypatch, tmp_path)
  assert _is_defaults(pv._load_curve_config())
  errs = log.at("error")
  assert len(errs) == 1 and path in errs[0]


def test_oversize_file_is_logged(monkeypatch, tmp_path, log):
  path = _cfg(monkeypatch, tmp_path, '{"lightning": {}}' + " " * (pv._CURVE_CONFIG_MAX_BYTES + 1))
  assert _is_defaults(pv._load_curve_config())
  errs = log.at("error")
  assert len(errs) == 1 and path in errs[0]


@pytest.mark.parametrize("text", ['[1, 2]', '{"lightning": 5}'])
def test_wrong_shape_is_logged(monkeypatch, tmp_path, log, text):
  path = _cfg(monkeypatch, tmp_path, text)
  assert _is_defaults(pv._load_curve_config())
  errs = log.at("error")
  assert len(errs) == 1 and path in errs[0]


def test_clamped_keys_are_named_once_per_load(monkeypatch, tmp_path, log):
  _, hi = pv._CURVE_BOUNDS["penalty_max_mph"]
  lo2, _ = pv._CURVE_BOUNDS["left_factor"]
  _cfg(monkeypatch, tmp_path,
       f'{{"lightning": {{"penalty_max_mph": {hi + 100}, "left_factor": {lo2 - 1}, "low_v_mph": 31.0}}}}')
  cfg = pv._load_curve_config()
  assert cfg["penalty_max_mph"] == hi and cfg["left_factor"] == lo2   # clamped exactly as before
  warns = log.at("warning")
  assert len(warns) == 1                                                  # ONE line per load
  assert "penalty_max_mph" in warns[0] and "left_factor" in warns[0] and "low_v_mph" not in warns[0]
  assert log.at("error") == []


def test_nan_is_named_and_keeps_the_default(monkeypatch, tmp_path, log):
  _cfg(monkeypatch, tmp_path, '{"lightning": {"penalty_max_mph": NaN}}')
  cfg = pv._load_curve_config()
  assert cfg["penalty_max_mph"] == pv._CURVE_DEFAULTS["penalty_max_mph"]
  warns = log.at("warning")
  assert len(warns) == 1 and "penalty_max_mph" in warns[0] and "NaN" in warns[0]


def test_icbm_shape_live_and_a_numeric_knob_are_both_honoured_silently(monkeypatch, tmp_path, log):
  """curveshape2pnw x rule2fixes2pnw: the string switch is parsed outside the numeric float() loop, so a valid
  "live" neither throws the whole file back to defaults nor logs an error, and the numeric knob beside it is kept."""
  _cfg(monkeypatch, tmp_path, '{"lightning": {"icbm_shape": "live", "penalty_max_mph": 6.0}}')
  cfg = pv._load_curve_config()
  assert cfg["icbm_shape"] == "live" and cfg["icbm_shape_why"] == "curve.json"
  assert cfg["penalty_max_mph"] == 6.0
  assert log.lines == []
