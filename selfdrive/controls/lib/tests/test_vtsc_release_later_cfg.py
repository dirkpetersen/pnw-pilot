"""vtscfloor2pnw -- the kill switch and the capability gate for VTSC's release-later.

PnwVehicle.vtsc_release_later: True on the Raven while curve.json's tesla.vtsc_release_later is not switched off (default ON, owner
2026-09-29); False on every other car (the Lightning's VTSC is unchanged); an unreadable/invalid file turns it OFF (today's
behaviour) and says so (Rule 2); a mid-drive typo keeps the last good config like every other tesla key.
"""
import json
import os

import pytest

from openpilot.selfdrive.controls.lib import pnw_vehicle as pv

TESLA = "TESLA_MODEL_S_HW3"
LIGHTNING = "FORD_F_150_LIGHTNING_MK1"


class CP:
  def __init__(self, fp, brand, op_long):
    self.carFingerprint, self.brand, self.openpilotLongitudinalControl, self.dashcamOnly = fp, brand, op_long, False


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


@pytest.fixture(autouse=True)
def _overrides_valid(tmp_path, monkeypatch):
  from openpilot.selfdrive.controls.lib.ces_pnw import curve_brain as cb
  p = tmp_path / "curve_overrides.json"
  p.write_text('{"overrides": []}')
  monkeypatch.setattr(cb, "OVERRIDES_PATH", str(p))


@pytest.fixture
def cfg(tmp_path, monkeypatch):
  p = tmp_path / "curve.json"
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(p))
  monkeypatch.setattr(pv, "RAIN_CONFIG_PATH", str(tmp_path / "absent-rain.json"))

  def write(doc):
    if doc is not None:
      p.write_text(doc if isinstance(doc, str) else json.dumps(doc))
    return str(p)
  return write


def tesla():
  return pv.PnwVehicle(CP(TESLA, "tesla", True))


def test_default_is_on_for_the_raven_and_off_for_everyone_else(cfg):
  cfg(None)
  assert tesla().vtsc_release_later is True
  assert pv.PnwVehicle(CP(LIGHTNING, "ford", False)).vtsc_release_later is False
  assert pv.PnwVehicle(CP(LIGHTNING, "ford", True)).vtsc_release_later is False        # Lightning op-long: still no
  assert pv.PnwVehicle(CP("MOCK", "mock", False)).vtsc_release_later is False


@pytest.mark.parametrize("val,expect", [(True, True), (False, False), (1, True), (0, False)])
def test_the_kill_switch_key(cfg, log, val, expect):
  cfg({"tesla": {"vtsc_release_later": val}})
  t = tesla()
  assert t.vtsc_release_later is expect and t.curve_brain_why == "curve.json"
  assert log.at("error") == []


@pytest.mark.parametrize("val", ["no", 2, None, [False], 0.5])
def test_an_unusable_value_is_off_and_says_so(cfg, log, val):
  cfg({"tesla": {"vtsc_release_later": val}})
  t = tesla()
  assert t.vtsc_release_later is False
  assert any("vtsc_release_later" in e for e in log.at("error"))
  assert t.curve_brain_why.startswith("INVALID")


@pytest.mark.parametrize("doc", ["not json", '{"tesla": ', '{"tesla": [1]}'])
def test_a_corrupt_file_or_section_turns_it_off_and_says_so(cfg, log, doc):
  cfg(doc)
  assert tesla().vtsc_release_later is False
  assert log.at("error")


def test_a_directory_in_place_of_the_file_turns_it_off(cfg, tmp_path, monkeypatch, log):
  d = tmp_path / "dir.json"
  d.mkdir()
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(d))
  assert tesla().vtsc_release_later is False
  assert log.at("error")


def test_the_tesla_section_never_touches_the_lightning(cfg):
  cfg({"tesla": {"vtsc_release_later": True}})
  assert pv.PnwVehicle(CP(LIGHTNING, "ford", True)).vtsc_release_later is False


def _reload(t, cfg_writer, doc, now):
  path = cfg_writer(doc)
  st = os.stat(path)
  os.utime(path, ns=(st.st_atime_ns, st.st_mtime_ns + 5_000_000_000))
  return t.refresh_curve_brain_cfg(now)


def test_hot_reload_flips_it_within_the_poll_interval(cfg, log):
  cfg(None)
  t = tesla()
  assert t.vtsc_release_later is True
  assert _reload(t, cfg, {"tesla": {"vtsc_release_later": False}}, t._tesla_cfg_poll + 1.0) is True
  assert t.vtsc_release_later is False
  ev = [m for lvl, m in log.lines if lvl == "event"]
  assert "curve_brain_cfg_reload" in ev
  assert _reload(t, cfg, {"tesla": {"vtsc_release_later": True}}, t._tesla_cfg_poll + 1.0) is True
  assert t.vtsc_release_later is True


def test_a_typo_mid_drive_keeps_the_last_good_value(cfg, log):
  cfg({"tesla": {"vtsc_release_later": False}})
  t = tesla()
  assert _reload(t, cfg, {"tesla": {"vtsc_release_later": "flase"}}, t._tesla_cfg_poll + 1.0) is False
  assert t.vtsc_release_later is False                                                  # the last good value, not the default (True)
  assert any("NOT applied" in e for e in log.at("error"))


def test_a_rejected_reload_names_the_release_later_in_the_log(cfg, log):
  cfg({"tesla": {"vtsc_release_later": False}})
  t = tesla()
  _reload(t, cfg, {"tesla": {"vtsc_release_later": "flase"}}, t._tesla_cfg_poll + 1.0)
  assert any("release_later=False" in e for e in log.at("error"))
