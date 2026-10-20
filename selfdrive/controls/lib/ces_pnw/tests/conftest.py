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


@pytest.fixture(autouse=True)
def _ces_event_writers_stopped(monkeypatch):
  """cesasyncwrite2pnw: a controller's ces_events writer is a daemon thread that outlives its test. Stop and join every one
  BEFORE monkeypatch undoes CES_EVENT_LOG (this fixture requests monkeypatch, so it tears down first) -- otherwise a leftover
  writer could append to the real /data/pnw/ces_events.jsonl on a device (tici)."""
  yield
  from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
  for c in list(m._EV_CONTROLLERS):
    c.close_events(2.0)


@pytest.fixture(autouse=True)
def _device_floor(monkeypatch):
  """clockvalid2pnw: fixed 'good clock' dates in these tests must not depend on when THIS host's systemd was upgraded
  (wall_time_floor = systemd mtime + 1 day, so an October upgrade made every September fixture read as pre-sync)."""
  from openpilot.common.tests.test_clockvalid2pnw import pin_device_floor
  pin_device_floor(monkeypatch)
