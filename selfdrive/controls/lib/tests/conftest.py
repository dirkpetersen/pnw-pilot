import pytest

from openpilot.selfdrive.controls.lib.ces_pnw.tests.roaddb_fixture import isolate


@pytest.fixture(autouse=True)
def _roaddb_isolated(monkeypatch, tmp_path_factory):
  """curvedblive2pnw: every controller built here loads a known, valid curve DB (see roaddb_fixture.py)."""
  return isolate(monkeypatch, tmp_path_factory)
