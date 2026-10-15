"""curvedbasync2pnw: the selfdrived loop must never do observation-file I/O.

Same field basis as cesasyncwrite2pnw (drives/2026-10-11/system-lagging): a synchronous getsize + append on a ~90% full /data
stalled the loop. `curvedb_shadow._write` now serialises and hands (shadow, line) to the ces_events writer thread; the writer
does the size check and the append (`write_line`). These tests pin: the loop side never waits, lines land in order in the
RIGHT file, a refused/queued/lost observation is never reported as written, and every loss is counted and loud.
"""
import json
import threading
import time

import pytest

from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
from openpilot.selfdrive.controls.lib.ces_pnw import curvedb_shadow as cs
from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_cesasyncwrite2pnw import LogRec
from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_curvedbshadow2pnw import (LIGHTNING, T0, _drive_past, _obs, _site,
                                                                                    pin_device_floor)


@pytest.fixture(autouse=True)
def _device_floor(monkeypatch):
  pin_device_floor(monkeypatch)


@pytest.fixture
def rig(tmp_path, monkeypatch):
  lg = LogRec()
  monkeypatch.setattr(m, "cloudlog", lg)
  monkeypatch.setattr(cs, "cloudlog", lg)
  monkeypatch.setattr(m, "CES_EVENT_LOG", str(tmp_path / "ces_events.jsonl"))
  monkeypatch.setattr(m, "CES_ARCHIVE_DIR", str(tmp_path / "archive"))
  c = m.CESController.__new__(m.CESController)
  c._event_log_ok = True
  c._append_fail = 0
  obs_path = tmp_path / "obs.jsonl"
  obs_path.write_text("")
  sh = cs.CurveDBShadow(LIGHTNING, obs_path=str(obs_path), config_path=str(tmp_path / "shadow_cfg.json"),
                        load_async=False, sink=c._enqueue_event)
  yield c, sh, obs_path, lg
  c.close_events(2.0)
  if c._ev_thread is not None:
    assert not c._ev_thread.is_alive()


def _obs_lines(path):
  return [json.loads(ln) for ln in path.read_text().splitlines()]


def test_observation_enqueue_is_fast_even_when_the_writer_is_blocked(rig):
  c, sh, obs_path, _ = rig
  release, started = threading.Event(), threading.Event()
  real = sh.write_line

  def blocked(line):
    started.set()
    release.wait(5.0)
    return real(line)
  sh.write_line = blocked
  assert sh._write(_obs(t=T0))
  assert started.wait(2.0)                        # the writer is now stuck inside the append
  durs = []
  for i in range(1, 20):
    t0 = time.monotonic()
    assert sh._write(_obs(t=T0 + i))
    durs.append(time.monotonic() - t0)
  assert max(durs) < 0.005 and sorted(durs)[len(durs) // 2] < 0.001, durs
  release.set()
  assert c.flush_events(5.0)
  assert [r["t"] for r in _obs_lines(obs_path)] == [T0 + i for i in range(20)]


def test_observations_land_in_order_in_the_shadows_own_file_not_ces_events(rig, tmp_path):
  c, sh, obs_path, _ = rig
  for i in range(200):
    c._append_event({"ev": "x", "i": i})          # interleave ces_events lines
    assert sh._write(_obs(t=T0 + i))
  assert c.flush_events(5.0)
  assert [r["t"] for r in _obs_lines(obs_path)] == [T0 + i for i in range(200)]
  assert [json.loads(ln)["i"] for ln in (tmp_path / "ces_events.jsonl").read_text().splitlines()] == list(range(200))
  assert sh.obs_queued == sh.obs_written == 200 and sh.obs_dropped == 0


def test_a_real_passage_reports_queued_then_lands(rig):
  c, sh, obs_path, _ = rig
  tele = _drive_past(sh, _site())
  evs = [t["cdbEv"] for t in tele if t["cdbEv"]]
  assert evs and all("Queued" in e for e in evs) and "up" not in evs, evs      # honest: queued is not written
  assert c.flush_events(5.0)
  assert len(_obs_lines(obs_path)) == sh.obs_written == sh.obs_queued >= 1


def test_queue_full_is_counted_logged_and_visible_in_telemetry(rig, monkeypatch):
  c, sh, obs_path, lg = rig
  monkeypatch.setattr(m, "CES_EVENT_QUEUE_MAX", 3)
  release, started = threading.Event(), threading.Event()
  real = sh.write_line

  def blocked(line):
    started.set()
    release.wait(5.0)
    return real(line)
  sh.write_line = blocked
  assert sh._write(_obs(t=T0))
  assert started.wait(2.0)
  results = [sh._write(_obs(t=T0 + i)) for i in range(1, 9)]       # 3 queue, 5 are refused
  assert results == [True] * 3 + [False] * 5
  assert sh.obs_dropped == 5 and c._ev_drops == 0                  # curvedb losses are NOT ces_events' evDrop
  msgs = [e for e in lg.errors if "DROPPED" in e]
  assert len(msgs) == 1 and str(obs_path) in msgs[0] and "queue FULL" in msgs[0]     # rate-limited, names the file
  assert sh._drop_why() == "queue"
  frag = sh.record(now_wall=T0, lat=None, lon=None, bearing=None, v_ego=25.0, site_pt=None, posted_ms=None,
                   hwy_class=None, icbm_src=None, icbm_target_ms=None, ref_ms=None)
  assert frag["cdbDrop"] == 5
  release.set()
  assert c.flush_events(5.0)
  assert sh.obs_written == 4 and len(_obs_lines(obs_path)) == 4


def test_a_refused_pass_says_dropped_queue_not_queued(rig, monkeypatch):
  c, sh, _, _ = rig
  monkeypatch.setattr(c, "_enqueue_event", lambda item: (item[0].note_queue_drop("test"), False)[1])
  sh._sink = c._enqueue_event
  evs = [t["cdbEv"] for t in _drive_past(sh, _site()) if t["cdbEv"]]
  assert evs == ["upDropped:queue"], evs


def test_dead_writer_drops_are_counted_on_the_shadow(rig, monkeypatch):
  c, sh, _, lg = rig
  monkeypatch.setattr(m, "CES_EVENT_WRITER_MAX_RESTARTS", 0)
  c._ensure_event_writer()
  c._ev_q.put_nowait(m._EV_STOP)
  c._ev_thread.join(2.0)
  assert not c._ev_thread.is_alive()
  assert sh._write(_obs()) is False
  assert sh.obs_dropped == 1 and c._ev_drops == 0
  assert any("DEAD" in e and "curvedb_shadow" in e for e in lg.errors)


def test_close_events_flushes_curvedb_lines_too(rig):
  c, sh, obs_path, _ = rig
  for i in range(50):
    assert sh._write(_obs(t=T0 + i))
  assert c.close_events(5.0)
  assert len(_obs_lines(obs_path)) == 50


def test_cap_freeze_runs_on_the_writer_and_counts_what_it_refuses(rig, monkeypatch):
  c, sh, obs_path, lg = rig
  monkeypatch.setattr(cs, "OBS_MAX_BYTES", 400)
  for i in range(10):
    sh._write(_obs(t=T0 + i))
  assert c.flush_events(5.0)
  assert sh._write_stopped
  n = len(_obs_lines(obs_path))
  assert 0 < n < 10 and sh.obs_written == n
  assert sh.obs_dropped == 10 - n                                 # nothing queued-then-lost goes uncounted
  assert len([e for e in lg.errors if "reached the" in e]) == 1   # the freeze is loud once, not per line
  assert sh._write(_obs()) is False                               # and later passes are refused on the loop side


def test_write_error_on_the_writer_is_logged_with_the_file_and_counted(rig, tmp_path):
  c, sh, obs_path, lg = rig
  sh._obs_path = str(tmp_path / "no_such_dir" / "obs.jsonl")
  assert sh._write(_obs())
  assert c.flush_events(5.0)
  assert sh.obs_written == 0 and sh.obs_dropped == 1 and sh.err == 1
  assert any("no_such_dir" in e for e in lg.exceptions)


def test_the_real_controller_wires_the_shadow_to_its_own_writer():
  from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_ces_mode_read_failure_logged import _P
  from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_curvelead2pnw import FakeCP
  c = m.CESController(FakeCP(LIGHTNING, "ford", False), params=_P(lambda: 0.0))
  try:
    assert c._cdb._sink == c._enqueue_event
  finally:
    c.close_events(2.0)


def test_a_stale_queue_refusal_does_not_mislabel_a_later_cap_refusal(rig, monkeypatch):
  c, sh, _, _ = rig
  monkeypatch.setattr(c, "_enqueue_event", lambda item: (item[0].note_queue_drop("test"), False)[1])
  sh._sink = c._enqueue_event
  assert sh._write(_obs()) is False and sh._drop_why() == "queue"
  sh._write_stopped = True                                         # then the cap freeze hits
  assert sh._write(_obs()) is False
  assert sh._drop_why() == "stopped"


def test_close_events_names_the_file_for_leftover_curvedb_lines(rig):
  c, sh, obs_path, lg = rig
  release, started = threading.Event(), threading.Event()

  def stuck(line):
    started.set()
    release.wait(5.0)
    return True
  sh.write_line = stuck
  assert sh._write(_obs(t=T0))
  assert started.wait(2.0)
  for i in range(1, 4):
    assert sh._write(_obs(t=T0 + i))
  try:
    assert c.close_events(0.05) is False
  finally:
    release.set()
  assert any("LOST at stop" in e and str(obs_path) in e for e in lg.errors)
  stop = [e for e in lg.errors if "ces_events stop:" in e][0]
  assert "3 curvedb observation(s) LOST" in stop and "1 other item(s) LOST" in stop, stop   # 1 = the write in flight
  assert sh.obs_dropped == 3
