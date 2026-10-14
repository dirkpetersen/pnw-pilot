"""portalrecheck2pnw — the captive portal must be re-checked after its session expires.

The bug: after captive_portal.accept() succeeded, `portal_done_for = ssid` and the arbiter never looked at the portal
again while on that SSID. The Visitor hotspot session expires (~5 h), every WiFi request is redirected to the login page,
NM stays "full" because LTE is up, and uploads stalled silently. These drive main() itself (the glue is where it lives).
"""

import threading
import types

import pytest

import openpilot.system.networkd.network_arbiterd as d
from openpilot.system.networkd.tests.test_network_arbiter_sequences import FakeNM, HOME, POLL, run_loop

CID = d.priority_connection_id(HOME)
TICKS_PER_RECHECK = int(d.PORTAL_RECHECK_S / POLL)  # 15


class SyncThread:
  """Runs the target inline, so a scenario is deterministic. (The real-thread case has its own test.)"""

  def __init__(self, target=None, name=None, daemon=None):
    self.target = target

  def start(self):
    self.target()

  def is_alive(self):
    return False


class Portal:
  """Scripts captive_portal.accept (a result per call) and probe_online (a result or Exception per call)."""

  def __init__(self):
    self.accepts = 0
    self.accept_results: list[bool] = []
    self.probe_calls: list[float] = []
    self.probes: list[object] = []
    self.nm: FakeNM | None = None

  def accept(self, handler, already_online=False):
    self.accepts += 1
    return self.accept_results.pop(0) if self.accept_results else True

  def probe(self):
    self.probe_calls.append(self.nm.t)
    r = self.probes.pop(0) if self.probes else True
    if isinstance(r, Exception):
      raise r
    return r


@pytest.fixture
def portal(monkeypatch):
  p = Portal()
  monkeypatch.setattr(d.captive_portal, "accept", p.accept)
  monkeypatch.setattr(d.captive_portal, "probe_online", p.probe)
  orig = d.pn.entry_for_ssid
  monkeypatch.setattr(d.pn, "entry_for_ssid", lambda nets, ssid: dict(e, portal="peak") if (e := orig(nets, ssid)) and e["ssid"] == HOME else e)
  return p


@pytest.fixture
def events(monkeypatch):
  got: list[tuple[str, dict]] = []
  monkeypatch.setattr(d.cloudlog, "event", lambda name, **kw: got.append((name, kw)))
  return got


def go(monkeypatch, portal, ticks, threads=SyncThread):
  nm = FakeNM()
  portal.nm = nm
  nm.active = CID
  nm.ip[CID] = "10.0.0.2"
  monkeypatch.setattr(d, "threading", types.SimpleNamespace(Thread=threads))
  run_loop(monkeypatch, nm, ticks=ticks)
  return nm


def names(events):
  return [n for n, _ in events]


class TestSessionExpiry:
  def test_the_interval_is_five_minutes(self, monkeypatch, portal, events):
    assert d.PORTAL_RECHECK_S == 300.0
    portal.probes = [True]
    go(monkeypatch, portal, TICKS_PER_RECHECK + 4)
    assert 300 <= portal.probe_calls[0] < 300 + 2 * POLL

  def test_no_reprobe_before_300s(self, monkeypatch, portal, events):
    go(monkeypatch, portal, TICKS_PER_RECHECK - 2)
    assert portal.accepts == 1 and portal.probe_calls == []

  def test_a_redirect_at_300s_reruns_accept_and_logs_the_expiry(self, monkeypatch, portal, events):
    portal.probes = [False]
    go(monkeypatch, portal, TICKS_PER_RECHECK + 4)
    assert len(portal.probe_calls) == 1 and portal.probe_calls[0] >= d.PORTAL_RECHECK_S
    assert portal.accepts == 2, "accept() must re-run after the expiry"
    assert names(events).count("network2xnor_portal_expired") == 1
    assert names(events).count("network2xnor_portal_relogin_ok") == 1
    exp = next(kw for n, kw in events if n == "network2xnor_portal_expired")
    assert exp["ssid"] == HOME and exp["since_ok_s"] >= 280

  def test_online_changes_nothing(self, monkeypatch, portal, events):
    portal.probes = [True, True]
    go(monkeypatch, portal, 2 * TICKS_PER_RECHECK + 4)
    assert len(portal.probe_calls) == 2 and portal.accepts == 1
    assert not [n for n in names(events) if "expired" in n or "error" in n]

  def test_a_probe_error_is_logged_not_reset_and_retried(self, monkeypatch, portal, events):
    portal.probes = [TimeoutError("dns"), True]
    go(monkeypatch, portal, 2 * TICKS_PER_RECHECK + 4)
    assert names(events).count("network2xnor_portal_probe_error") == 1
    assert "network2xnor_portal_expired" not in names(events)
    assert portal.accepts == 1, "an error must not be treated as expiry"
    assert len(portal.probe_calls) == 2 and portal.probe_calls[1] - portal.probe_calls[0] >= d.PORTAL_RECHECK_S

  def test_exhausted_tries_stay_loud_and_each_recheck_grants_one_more_login(self, monkeypatch, portal, events):
    # first login fine; then expired, and every re-login fails
    portal.accept_results = [True] + [False] * 50
    portal.probes = [False, False, False]
    go(monkeypatch, portal, 3 * TICKS_PER_RECHECK + 12)
    assert names(events).count("network2xnor_portal_expired") == 1
    still = names(events).count("network2xnor_portal_still_expired")
    assert still >= 2, names(events)
    assert portal.accepts == 1 + d.PORTAL_MAX_TRIES + still, "one extra accept() per still-offline recheck, not a burst"

  def test_a_login_that_never_succeeds_is_still_rechecked(self, monkeypatch, portal, events):
    # accept() False six times (e.g. rejoin with a valid session: no form to submit), then the 300 s probe must still run
    portal.accept_results = [False] * 50
    portal.probes = [True]
    go(monkeypatch, portal, TICKS_PER_RECHECK + 8)
    assert portal.accepts == d.PORTAL_MAX_TRIES
    assert len(portal.probe_calls) >= 1, "silence after the tries ran out is the original bug"
    assert "network2xnor_portal_online_no_login" in names(events)

  def test_leaving_the_ssid_resets_the_timer(self, monkeypatch, portal, events):
    def leave(nm, n):
      if n == 6:
        nm.active = None
      if n == 8:
        nm.active = CID

    nm = FakeNM()
    portal.nm = nm
    nm.active, nm.ip[CID] = CID, "10.0.0.2"
    monkeypatch.setattr(d, "threading", types.SimpleNamespace(Thread=SyncThread))
    run_loop(monkeypatch, nm, ticks=TICKS_PER_RECHECK + 6, hooks=(leave,))
    assert portal.probe_calls == [], "the 300 s clock restarts on re-entry"


class DeferredThread(SyncThread):
  """start() queues the target instead of running it, so a test decides when a worker 'finishes'."""

  queue: list = []

  def start(self):
    DeferredThread.queue.append(self.target)


class TestStaleWorkers:
  def test_a_result_arriving_after_leave_and_rejoin_is_ignored(self, monkeypatch, portal, events):
    DeferredThread.queue = []
    stale: list = []
    at_delivery: list[int] = []

    def hook(nm, n):
      if n == 3:
        nm.active = None
        stale.extend(DeferredThread.queue)
      elif n == 5:
        nm.active = CID
      elif n == 7:
        at_delivery.append(names(events).count("network2xnor_portal_try"))
        for t in stale:
          t()  # the OLD visit's accept() finally returns True

    nm = FakeNM()
    portal.nm = nm
    nm.active, nm.ip[CID] = CID, "10.0.0.2"
    monkeypatch.setattr(d, "threading", types.SimpleNamespace(Thread=DeferredThread))
    run_loop(monkeypatch, nm, ticks=14, hooks=(hook,))
    tries_after = names(events).count("network2xnor_portal_try") - at_delivery[0]
    assert tries_after >= 3, "the stale True was taken as this visit's login, so the loop stopped trying"


class TestProbeOnline:
  @staticmethod
  def _session(monkeypatch, status=None, text="", exc=None):
    import requests

    R = types.SimpleNamespace(status_code=status, text=text)

    class S:
      headers: dict = {}

      def get(self, *a, **k):
        if exc:
          raise exc
        return R

    monkeypatch.setattr(requests, "Session", S)

  @pytest.mark.parametrize("status,text,want", [(204, "", True), (200, "<html>login</html>", False), (302, "", False), (403, "", False), (511, "", False)])
  def test_classification(self, monkeypatch, status, text, want):
    self._session(monkeypatch, status, text)
    assert d.captive_portal.probe_online() is want

  @pytest.mark.parametrize("status,exc", [(500, None), (None, TimeoutError("dns"))])
  def test_an_error_raises_rather_than_answering(self, monkeypatch, status, exc):
    self._session(monkeypatch, status, exc=exc)
    with pytest.raises((RuntimeError, TimeoutError)):
      d.captive_portal.probe_online()


class TestItNeverBlocksTheLoop:
  def test_the_probe_runs_in_a_thread(self, monkeypatch, portal, events):
    release, started = threading.Event(), threading.Event()
    timed_out = []

    def slow():
      started.set()
      if not release.wait(3):  # only the loop's own ticks can release it: blocked inline, it times out
        timed_out.append(True)
      return True

    portal.probe = slow
    monkeypatch.setattr(d.captive_portal, "probe_online", slow)
    ticks_after = []

    def watch(nm, n):
      if started.is_set() and not release.is_set():
        ticks_after.append(n)
        if len(ticks_after) >= 3:
          release.set()

    nm = FakeNM()
    portal.nm = nm
    nm.active, nm.ip[CID] = CID, "10.0.0.2"
    run_loop(monkeypatch, nm, ticks=TICKS_PER_RECHECK + 8, hooks=(watch,))
    release.set()
    assert started.is_set(), "probe never ran"
    assert not timed_out, "the probe blocked the loop: the ticks that release it never ran"
    assert len(ticks_after) >= 3, "the loop kept ticking while the probe was still blocked"
