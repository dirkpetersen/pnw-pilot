import pytest

from openpilot.selfdrive.controls.lib.ces_pnw.tests.roaddb_fixture import isolate


@pytest.fixture(autouse=True)
def _roaddb_isolated(monkeypatch, tmp_path_factory):
  """curvedblive2pnw: every controller built here loads a known, valid curve DB (see roaddb_fixture.py)."""
  return isolate(monkeypatch, tmp_path_factory)


@pytest.fixture(autouse=True)
def _curve_overrides_valid(monkeypatch, tmp_path_factory):
  """ovrcar2pnw: the Lightning's curve DB now reads the per-curve override file too, and a missing file is a loud error (the
  fail-safe). Controllers built here read a VALID empty one, never the device path (same as ces_pnw/tests/conftest.py)."""
  from openpilot.selfdrive.controls.lib.ces_pnw import curve_brain as cb
  p = tmp_path_factory.mktemp("ovr") / "curve_overrides.json"
  p.write_text('{"version": 2, "overrides": []}')
  monkeypatch.setattr(cb, "OVERRIDES_PATH", str(p))
  return p
