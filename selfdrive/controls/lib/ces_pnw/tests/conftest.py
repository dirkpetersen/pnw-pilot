import pytest

from openpilot.selfdrive.controls.lib.ces_pnw.tests.roaddb_fixture import isolate


@pytest.fixture(autouse=True)
def _roaddb_isolated(monkeypatch, tmp_path_factory):
  """curvedblive2pnw: every controller built here loads a known, valid curve DB (see roaddb_fixture.py)."""
  return isolate(monkeypatch, tmp_path_factory)


@pytest.fixture(autouse=True)
def _curve_overrides_valid(monkeypatch, tmp_path_factory):
  """curvebrain2b2pnw A5: every Tesla brain built here reads a VALID, empty per-curve override file (no overrides), never the
  device path -- a missing file is the fail-safe (2.8 everywhere), which the dedicated tests exercise on purpose."""
  from openpilot.selfdrive.controls.lib.ces_pnw import curve_brain as cb
  p = tmp_path_factory.mktemp("ovr") / "curve_overrides.json"
  p.write_text('{"overrides": []}')
  monkeypatch.setattr(cb, "OVERRIDES_PATH", str(p))
  return p
