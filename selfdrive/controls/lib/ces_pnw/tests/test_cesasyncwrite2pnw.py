"""cesasyncwrite2pnw: the selfdrived loop must never do ces_events file I/O.

Field basis: drives/2026-10-11/system-lagging -- two selfdrivedLagging soft-disables (56 and 64 mph) were each one
~123 ms stall of the selfdrived loop, 96-99% of it the synchronous append in CESController._append_event on a /data
that was ~90% full. The append (size check, rotation, archive hand-off, write) now runs on ONE daemon thread behind a
bounded queue; these tests pin that the loop side never waits, and that every failure of the new machinery is loud.
"""
import json
import threading
import time

import pytest

from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m


class Boom(BaseException):
  """Not an Exception, so it is NOT absorbed per record: it kills the writer thread."""


class LogRec:
  def __init__(self):
    self.errors, self.warnings, self.exceptions = [], [], []

  def error(self, msg, *a, **k):
    self.errors.append(str(msg))

  def warning(self, msg, *a, **k):
    self.warnings.append(str(msg))

  def exception(self, msg, *a, **k):
    self.exceptions.append(str(msg))

  def event(self, *a, **k):
    pass

  def info(self, *a, **k):
    pass


@pytest.fixture
def log(monkeypatch):
  lg = LogRec()
  monkeypatch.setattr(m, "cloudlog", lg)
  return lg


@pytest.fixture
def ctl(tmp_path, monkeypatch, log):
  monkeypatch.setattr(m, "CES_EVENT_LOG", str(tmp_path / "ces_events.jsonl"))
  monkeypatch.setattr(m, "CES_ARCHIVE_DIR", str(tmp_path / "archive"))
  c = m.CESController.__new__(m.CESController)
  c._event_log_ok = True
  c._append_fail = 0
  yield c
  if c._ev_q is not None:                      # the stop path: the writer must exit cleanly
    c._ev_q.put(m._EV_STOP)
    c._ev_thread.join(2.0)


def _lines(tmp_path):
  return [json.loads(ln) for ln in (tmp_path / "ces_events.jsonl").read_text().splitlines()]


def test_enqueue_returns_immediately_even_when_the_writer_is_blocked(ctl, tmp_path):
  def slow_write(line):
    time.sleep(0.2)
  ctl._write_event_line = slow_write
  durs = []
  for i in range(20):
    t0 = time.monotonic()
    ctl._append_event({"ev": "x", "i": i})
    durs.append(time.monotonic() - t0)
  assert max(durs) < 0.03, durs                    # the loop never sees the 200 ms stall (loose max: loaded CI box)
  assert sorted(durs)[len(durs) // 2] < 0.001, durs   # typical enqueue is well under 1 ms


def test_the_loopdiag_timing_sees_the_enqueue_not_the_disk(ctl):
  ctl._write_event_line = lambda line: time.sleep(0.2)
  t0 = time.monotonic()
  ctl._append_event({"ev": "x"})
  ctl._append_event({"ev": "x"})
  assert ctl.event_write_timing(t0)["ces_event_write_max"] < 0.03


def test_records_land_in_order(ctl, tmp_path):
  for i in range(300):
    ctl._append_event({"ev": "x", "i": i})
  assert ctl.flush_events(5.0)
  assert [r["i"] for r in _lines(tmp_path)] == list(range(300))
  assert ctl._append_fail == 0


def test_size_rotation_still_happens_on_the_writer(ctl, tmp_path, monkeypatch):
  monkeypatch.setattr(m, "CES_EVENT_LOG_MAX_BYTES", 300)
  for i in range(40):
    ctl._append_event({"ev": "x", "i": i, "pad": "p" * 40})
  assert ctl.flush_events(5.0)
  assert (tmp_path / "ces_events.jsonl.1").exists()
  seen = []
  for name in sorted(p.name for p in tmp_path.iterdir() if p.name.startswith("ces_events.jsonl")):
    seen += [json.loads(ln)["i"] for ln in (tmp_path / name).read_text().splitlines()]
  assert sorted(seen) == list(range(40))             # nothing lost across generations


def test_the_park_rotation_runs_on_the_writer_thread_not_the_loop(ctl, tmp_path, monkeypatch):
  calls = []
  monkeypatch.setattr(m, "rotate_at_park", lambda path, gens: calls.append((path, threading.current_thread().name)))
  ctl._enqueue_event(m._EV_PARK_ROTATE)
  assert ctl.flush_events(2.0)
  assert calls == [(m.CES_EVENT_LOG, "ces_events_writer")]


def test_queue_full_drops_counts_and_logs_once(ctl, tmp_path, monkeypatch, log):
  monkeypatch.setattr(m, "CES_EVENT_QUEUE_MAX", 3)
  started, release = threading.Event(), threading.Event()
  real = m.CESController._write_event_line

  def blocked(line):
    started.set()
    release.wait(5.0)
    real(ctl, line)
  ctl._write_event_line = blocked
  ctl._append_event({"ev": "x", "i": 0})
  assert started.wait(2.0)                         # the writer holds record 0; the queue (3) is empty
  for i in range(1, 9):
    ctl._append_event({"ev": "x", "i": i})         # 1-3 queue, 4-8 are dropped
  assert ctl._ev_drops == 5
  full = [e for e in log.errors if "queue FULL" in e]
  assert len(full) == 1 and "record(s) dropped in total" in full[0]   # 5 drops, ONE line (rate limited), with the running count
  release.set()
  assert ctl.flush_events(5.0)
  assert [r["i"] for r in _lines(tmp_path)] == [0, 1, 2, 3]


def test_drop_log_is_rate_limited_but_comes_back(ctl, monkeypatch, log):
  monkeypatch.setattr(m, "CES_EVENT_QUEUE_MAX", 1)
  monkeypatch.setattr(m, "CES_EVENT_DROP_LOG_S", 0.05)
  started, release = threading.Event(), threading.Event()
  ctl._write_event_line = lambda line: (started.set(), release.wait(5.0))
  ctl._append_event({"ev": "x"})
  assert started.wait(2.0)
  ctl._append_event({"ev": "x"})
  for _ in range(5):
    ctl._append_event({"ev": "x"})
  assert len([e for e in log.errors if "queue FULL" in e]) == 1
  time.sleep(0.08)
  ctl._append_event({"ev": "x"})
  assert len([e for e in log.errors if "queue FULL" in e]) == 2
  release.set()


def test_a_dead_writer_is_logged_detected_and_restarted(ctl, tmp_path, log):
  state = {"n": 0}
  real = m.CESController._write_event_line

  def die_once(line):
    state["n"] += 1
    if state["n"] == 1:
      raise Boom()
    real(ctl, line)
  ctl._write_event_line = die_once
  ctl._append_event({"ev": "x", "i": 0})
  ctl._ev_thread.join(2.0)
  assert not ctl._ev_thread.is_alive()
  assert any("exiting on Boom" in e for e in log.exceptions)       # the death is logged by the thread itself
  ctl._append_event({"ev": "x", "i": 1})                           # the loop's next call notices and restarts
  assert any("DIED" in e for e in log.errors) and ctl._ev_restarts == 1
  assert ctl.flush_events(2.0)
  assert [r["i"] for r in _lines(tmp_path)] == [1]


def test_restarts_are_bounded_and_the_giving_up_is_loud(ctl, log):
  def always_die(line):
    raise Boom()
  ctl._write_event_line = always_die
  for i in range(m.CES_EVENT_WRITER_MAX_RESTARTS + 3):
    ctl._append_event({"ev": "x", "i": i})
    ctl._ev_thread.join(2.0)
  assert ctl._ev_restarts == m.CES_EVENT_WRITER_MAX_RESTARTS
  assert len([e for e in log.errors if "NOT restarting" in e]) == 1


def test_a_failing_append_keeps_the_writer_alive_and_is_still_counted_loudly(ctl, tmp_path, monkeypatch, log):
  monkeypatch.setattr(m, "CES_EVENT_LOG", str(tmp_path / "no" / "such" / "dir" / "ces_events.jsonl"))
  for i in range(10):
    ctl._append_event({"ev": "x", "i": i})
  assert ctl.flush_events(2.0)
  assert ctl._ev_thread.is_alive() and ctl._append_fail == 10
  assert any("append FAILING (10 consecutive)" in e for e in log.errors)


def test_an_unserialisable_record_is_counted_not_raised(ctl, log):
  ctl._append_event({"ev": "x", "bad": object()})
  assert ctl._append_fail == 1 and ctl._ev_q is None             # nothing queued, nothing raised


def test_flush_drains_a_backlog_for_stop_and_atexit(ctl, tmp_path):
  real = m.CESController._write_event_line
  ctl._write_event_line = lambda line: (time.sleep(0.01), real(ctl, line))
  for i in range(20):
    ctl._append_event({"ev": "x", "i": i})
  m._flush_all_event_writers()                                    # what atexit runs
  assert [r["i"] for r in _lines(tmp_path)] == list(range(20))


def test_a_flush_that_times_out_says_so(ctl, log):
  release = threading.Event()
  ctl._write_event_line = lambda line: release.wait(5.0)
  ctl._append_event({"ev": "x"})
  assert ctl.flush_events(0.05) is False
  assert any("flush gave up" in e for e in log.errors)
  release.set()


def test_a_slow_append_is_warned_on_the_writer_side(ctl, tmp_path, monkeypatch, log):
  monkeypatch.setattr(m, "CES_EVENT_SLOW_WRITE_S", 0.01)
  real = m.os.path.getsize
  monkeypatch.setattr(m.os.path, "getsize", lambda p: (time.sleep(0.03), real(p) if m.os.path.exists(p) else 0)[1])
  ctl._append_event({"ev": "x"})
  assert ctl.flush_events(2.0)
  assert any("ces_events write took" in w for w in log.warnings)


def test_event_log_not_ok_enqueues_nothing(ctl):
  ctl._event_log_ok = False
  ctl._append_event({"ev": "x"})
  assert ctl._ev_q is None
