"""mapsl2pnw: the on-road limit sign reads mapdOut directly and only updates when a message ARRIVES, so a dead mapd
(restart_if_crash=False) left its last limit on screen until reboot. After MAPD_SILENT_S without a mapdOut the sign is
unknown ("--"); a returning mapd re-publishes through the normal path."""
import types

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


class _SM:
  def __init__(self, updated, recv):
    self.updated, self.recv_frame = {"mapdOut": updated}, {"carState": 10, "mapdOut": recv}

  def __getitem__(self, k):
    assert k == "carState"
    return types.SimpleNamespace(vEgoCluster=1.0, vEgo=1.0)


def _tick(monkeypatch, r, updated, recv, started=5):
  monkeypatch.setattr(SL, "ui_state", types.SimpleNamespace(sm=_SM(updated, recv), started_frame=started, is_metric=False))
  r._v_ego_cluster_seen = False
  r.speed = 0.0
  r._update_state()


def test_update_state_expires_a_silent_mapd(monkeypatch):
  """_update_state must actually call _expire_silent_mapd (replacing the call with `pass` fails this)."""
  r = _r(last=0.0)                                       # last mapdOut at monotonic 0 -> long silent
  monkeypatch.setattr(SL.time, "monotonic", lambda: 1000.0)
  monkeypatch.setattr(SL.cloudlog, "warning", lambda *a, **k: None)
  _tick(monkeypatch, r, updated=False, recv=8)           # a mapdOut WAS seen this session (frame 8 >= started 5)
  assert not r.speed_limit_valid and r.speed_limit == 0.0


def test_a_limit_from_before_this_onroad_session_is_forgotten_silently(monkeypatch):
  """First onroad frame, no mapdOut yet this session: no bogus 'silent for <offroad time> s' line, no stale sign."""
  r = _r(last=0.0)
  lines = []
  monkeypatch.setattr(SL.time, "monotonic", lambda: 99999.0)
  monkeypatch.setattr(SL.cloudlog, "warning", lambda m, *a, **k: lines.append(m))
  _tick(monkeypatch, r, updated=False, recv=2)           # frame 2 < started_frame 5: from the previous drive
  assert lines == [] and not r.speed_limit_valid and r.speed_limit == 0.0
