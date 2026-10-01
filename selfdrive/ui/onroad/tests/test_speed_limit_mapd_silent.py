"""mapsl2pnw: the on-road limit sign reads mapdOut directly and only updates when a message ARRIVES, so a dead mapd
(restart_if_crash=False) left its last limit on screen until reboot. After MAPD_SILENT_S without a mapdOut the sign is
unknown ("--"); a returning mapd re-publishes through the normal path."""
import pytest

try:
  from openpilot.selfdrive.ui.onroad import speed_limit as SL
  from openpilot.selfdrive.ui.onroad.speed_limit import SpeedLimitRenderer, MAPD_SILENT_S
except Exception as _e:                                    # pragma: no cover - environment guard
  pytest.skip(f"raylib/params UI deps unavailable in this tree: {_e}", allow_module_level=True)


def _r(valid=True, last=100.0):
  r = object.__new__(SpeedLimitRenderer)      # bypass __init__ (fonts / GL)
  r.speed_limit, r.speed_limit_valid = 60.0, valid
  r.speed_limit_ahead, r.speed_limit_ahead_valid = 40.0, valid
  r._mapd_t = last
  return r


def test_a_brief_gap_keeps_the_limit():
  r = _r()
  r._expire_silent_mapd(100.0 + MAPD_SILENT_S - 0.1)
  assert r.speed_limit_valid and r.speed_limit == 60.0


def test_a_silent_mapd_clears_the_sign_once_and_logs_once(monkeypatch):
  r = _r()
  lines = []
  monkeypatch.setattr(SL.cloudlog, "warning", lambda m, *a, **k: lines.append(m))
  for dt in (MAPD_SILENT_S + 0.1, MAPD_SILENT_S + 1.1, MAPD_SILENT_S + 30.0):
    r._expire_silent_mapd(100.0 + dt)
  assert not r.speed_limit_valid and not r.speed_limit_ahead_valid and r.speed_limit == 0.0
  assert len(lines) == 1 and "cleared to unknown" in lines[0]


def test_an_already_unknown_sign_is_left_alone_and_silent(monkeypatch):
  r = _r(valid=False)
  lines = []
  monkeypatch.setattr(SL.cloudlog, "warning", lambda m, *a, **k: lines.append(m))
  r._expire_silent_mapd(1e6)
  assert lines == []
