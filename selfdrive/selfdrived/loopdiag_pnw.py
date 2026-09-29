"""loopdiag2pnw -- always-on selfdrived loop-timing diagnostic (OBSERVATION ONLY).

Why: on 2026-09-28 20:35:23 PT selfdrived paused publishing for ~114 ms at 100 Hz (selfdriveState/madsState gap),
dmonitoringd's 100 ms freshness check failed once, and selfdrived raised "Communication Issue Between Processes".
Nothing in control dropped, but the cause is undetermined (5 alerts with this signature since 09-17). This says where
the time went the next time it happens: one rate-limited cloudlog line when a loop iteration exceeds the threshold,
with the per-stage split and the time spent in synchronous ces_events writes.

It changes NO control behaviour and NOT the 100 ms validity check. Hot-path cost: a few time.monotonic() calls per
iteration. No I/O unless an iteration is slow, and then one cloudlog line, at most one per MIN_LOG_INTERVAL_S.
Nothing escapes: an internal failure is logged (first one at once, then every FAIL_LOG_EVERY-th) and the diagnostic
carries on -- it never fails silently and never raises into the loop (Rule 2).
"""
import time

from openpilot.common.swaglog import cloudlog

SLOW_LOOP_S = 0.050          # a 100 Hz loop iteration longer than this is "slow" (nominal 0.010)
MIN_LOG_INTERVAL_S = 5.0     # at most one slow-loop line per this many seconds; the rest are counted, not logged
FAIL_LOG_EVERY = 1000        # a failing diagnostic logs at once, then every this-many failures


class LoopDiag:
  def __init__(self, threshold_s=SLOW_LOOP_S, min_log_interval_s=MIN_LOG_INTERVAL_S,
               clock=time.monotonic, log=cloudlog.warning, log_exc=cloudlog.exception):
    self.threshold_s = threshold_s
    self.min_log_interval_s = min_log_interval_s
    self._clock = clock
    self._log = log
    self._log_exc = log_exc
    self._stages = {}
    self._last = None          # time of the last mark()/begin()
    self._prev_end = None      # end of the previous iteration (its period is end - prev_end)
    self._step_end = None
    self._next_log_t = -1e18
    self.suppressed = 0        # slow loops seen since the last emitted line
    self.slow_total = 0
    self.fail = 0

  def _failed(self):
    self.fail += 1
    if self.fail == 1 or self.fail % FAIL_LOG_EVERY == 0:
      try:
        self._log_exc(f"loopdiag2pnw: diagnostic FAILED ({self.fail} time(s)) -- slow-loop timing is NOT being recorded; control unaffected")
      except Exception:
        pass   # rule2-ok: the logger itself is down; nothing left to report to, and control must not be affected

  def begin(self):
    """Call at the top of an iteration, before step()."""
    try:
      self._stages.clear()
      self._last = self._clock()
    except Exception:
      self._failed()

  def mark(self, name):
    """Call at the end of a stage inside step(): records the time since the previous mark."""
    try:
      now = self._clock()
      if self._last is not None:
        self._stages[name] = now - self._last
      self._last = now
    except Exception:
      self._failed()

  def end(self, extra=None):
    """Call after the loop's sleep (rk.monitor_time). `extra` is a callable(since) -> dict of {label: seconds} for
    costs measured elsewhere (e.g. ces_events writes) since the previous iteration ended; only called when slow.
    Its "_writes"-style counts are formatted like the rest -- a count is shown as a plain number."""
    try:
      now = self._clock()
      prev_end, self._prev_end = self._prev_end, now
      if prev_end is None:
        return
      period = now - prev_end
      if period <= self.threshold_s:
        return
      self.slow_total += 1
      if now < self._next_log_t:
        self.suppressed += 1
        return
      self._next_log_t = now + self.min_log_interval_s
      # time between the last mark() (end of step()'s final stage) and now is the loop sleep / ratekeeper
      tail = now - self._last if self._last is not None else float("nan")
      st = " ".join(f"{k}={v * 1e3:.1f}ms" for k, v in self._stages.items())
      ex = ""
      if extra is not None:
        ex = " " + " ".join(f"{k}={v * 1e3:.1f}ms" if not k.endswith("_writes") else f"{k}={v:.0f}"
                            for k, v in extra(prev_end).items())
      self._log(f"selfdrived_slow_loop: period={period * 1e3:.1f}ms (limit {self.threshold_s * 1e3:.0f}ms, nominal 10ms) " +
                f"stages[{st}] after_step(sleep/sched)={tail * 1e3:.1f}ms{ex} " +
                f"suppressed_since_last_line={self.suppressed} total_slow={self.slow_total}")
      self.suppressed = 0
    except Exception:
      self._failed()
