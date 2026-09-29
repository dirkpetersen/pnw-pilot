"""loopdiag2pnw -- the slow-loop diagnostic: fires on a slow iteration, rate-limits, never raises, sees ces_events writes."""
import inspect
import time

from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw
from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import CESController
from openpilot.selfdrive.selfdrived import selfdrived as sd
from openpilot.selfdrive.selfdrived.loopdiag_pnw import LoopDiag, FAIL_LOG_EVERY


class _Clock:
  def __init__(self):
    self.t = 100.0

  def __call__(self):
    return self.t


def _diag(**kw):
  clk = _Clock()
  lines, excs = [], []
  d = LoopDiag(clock=clk, log=lines.append, log_exc=excs.append, **kw)
  return d, clk, lines, excs


def _iter(d, clk, stage_ms=None, sleep_ms=0.0):
  """One loop iteration: begin, stages, sleep, end."""
  d.begin()
  for name, ms in (stage_ms or {"dataSample": 1.0}).items():
    clk.t += ms / 1e3
    d.mark(name)
  clk.t += sleep_ms / 1e3
  d.end()


def test_normal_loops_log_nothing():
  d, clk, lines, _ = _diag()
  for _ in range(50):
    _iter(d, clk, {"dataSample": 1.0, "ces": 2.0}, sleep_ms=7.0)
  assert lines == [] and d.slow_total == 0


def test_slow_loop_logs_period_and_stage_split():
  d, clk, lines, _ = _diag()
  _iter(d, clk, sleep_ms=9.0)                                  # first iteration only seeds the period
  _iter(d, clk, {"dataSample": 1.0, "ces": 104.0, "publish": 1.0}, sleep_ms=8.0)   # 114 ms
  assert len(lines) == 1
  m = lines[0]
  assert "selfdrived_slow_loop" in m and "period=114.0ms" in m
  assert "ces=104.0ms" in m and "dataSample=1.0ms" in m and "after_step(sleep/sched)=8.0ms" in m


def test_first_iteration_never_logs():
  d, clk, lines, _ = _diag()
  _iter(d, clk, {"dataSample": 500.0})
  assert lines == []


def test_rate_limited_and_suppressed_count_carried():
  d, clk, lines, _ = _diag(min_log_interval_s=5.0)
  _iter(d, clk)
  for _ in range(3):
    _iter(d, clk, {"ces": 80.0})                               # three slow loops inside 5 s
  assert len(lines) == 1 and d.suppressed == 2
  clk.t += 6.0
  _iter(d, clk, {"ces": 80.0})                                 # this one's period is > 6 s: slow, and the window is over
  assert len(lines) == 2 and "suppressed_since_last_line=2" in lines[1] and d.suppressed == 0


def test_a_broken_clock_never_raises_and_is_reported_once():
  d, clk, _, excs = _diag()

  def boom():
    raise RuntimeError("clock down")
  d._clock = boom
  for _ in range(5):
    d.begin()
    d.mark("x")
    d.end()                                                    # must not raise
  assert d.fail == 15 and len(excs) == 1                       # loud on the first failure, not 15 lines


def test_failure_reporting_is_throttled_but_repeats():
  d, clk, _, excs = _diag()
  d._clock = lambda: 1 / 0
  for _ in range(FAIL_LOG_EVERY):
    d.mark("x")
  assert len(excs) == 2                                        # the first, and the FAIL_LOG_EVERY-th


def test_a_failing_logger_does_not_raise_and_is_counted():
  clk = _Clock()
  excs = []

  def bad_log(_):
    raise OSError("logger down")
  d = LoopDiag(clock=clk, log=bad_log, log_exc=excs.append)
  _iter(d, clk)
  _iter(d, clk, {"ces": 100.0})                                # slow -> log raises -> caught
  assert d.fail == 1 and len(excs) == 1


def test_a_failing_extra_does_not_raise():
  d, clk, lines, excs = _diag()
  _iter(d, clk)
  d.begin()
  clk.t += 0.1
  d.mark("ces")
  d.end(lambda since: 1 / 0)
  assert d.fail == 1 and len(excs) == 1


def _ces(tmp_path, monkeypatch):
  ctl = CESController.__new__(CESController)
  ctl._event_log_ok = True
  ctl._append_fail = 0
  monkeypatch.setattr(ces_pnw, "CES_EVENT_LOG", str(tmp_path / "ces_events.jsonl"))
  return ctl


def test_ces_event_write_time_is_recorded_and_windowed(tmp_path, monkeypatch):
  ctl = _ces(tmp_path, monkeypatch)
  t0 = time.monotonic()
  ctl._append_event({"ev": "x"})
  w = ctl.event_write_timing(t0)
  assert w["ces_event_writes"] == 1.0 and w["ces_event_write_total"] > 0.0
  assert (tmp_path / "ces_events.jsonl").read_text().count("\n") == 1     # the write itself is unchanged
  assert ctl.event_write_timing(time.monotonic())["ces_event_writes"] == 0.0   # nothing since: window excludes old writes


def test_ces_event_write_timing_is_bounded(tmp_path, monkeypatch):
  ctl = _ces(tmp_path, monkeypatch)
  for _ in range(100):
    ctl._append_event({"ev": "x"})
  assert len(ctl._event_writes) == 16


def test_a_slow_ces_event_write_is_named_in_the_slow_loop_line(tmp_path, monkeypatch):
  """The 2026-09-28 hypothesis end to end: a synchronous ces_events append (here a slow stat, as a stalled
  filesystem or the rotation would be) inside the loop shows up as ces_event_write_max in the slow-loop line."""
  ctl = _ces(tmp_path, monkeypatch)
  real_getsize = ces_pnw.os.path.getsize

  def slow_getsize(p):
    time.sleep(0.04)
    return real_getsize(p) if ces_pnw.os.path.exists(p) else 0
  lines = []
  d = LoopDiag(threshold_s=0.03, clock=time.monotonic, log=lines.append, log_exc=lines.append)
  d.begin()
  d.mark("dataSample")
  d.end()                                                      # seeds the period
  d.begin()
  monkeypatch.setattr(ces_pnw.os.path, "getsize", slow_getsize)
  ctl._append_event({"ev": "x"})
  monkeypatch.setattr(ces_pnw.os.path, "getsize", real_getsize)
  d.mark("ces")
  d.end(ctl.event_write_timing)
  assert len(lines) == 1, lines
  assert "ces_event_writes=1" in lines[0] and "ces_event_write_max=4" in lines[0] and "ces=4" in lines[0]


def test_selfdrived_is_wired_to_the_diagnostic():
  step = inspect.getsource(sd.SelfdriveD.step)
  for stage in ("dataSample", "events", "mads", "alerts", "ces", "publish"):
    assert f'self.loop_diag.mark("{stage}")' in step
  run = inspect.getsource(sd.SelfdriveD.run)
  assert "self.loop_diag.begin()" in run and "self.loop_diag.end(" in run
  assert "self.rk.monitor_time()" in run
