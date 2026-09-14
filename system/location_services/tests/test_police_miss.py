"""policemiss2pnw -- device-side causes of police reports MISSING from the overlay.

Measured 2026-09-07..13 (drives/2026-09-13/police-miss-week/DRIVE_REPORT.md):
qlogs (carState, gpsLocation, mapdOut, deviceState) against the comma-waze-proxy Lambda REPORT lines,
which record every device poll that reached AWS. Every fixture below is taken from those drives; the
coordinates are real, only the report timestamps are re-anchored to "now" (the tier model ages them).

The governing driver rule (2026-09-08): "if there's a police report too many that's fine, I just don't
want to miss any." Confidence is carried by colour, never by suppression.
"""
import urllib.error

import pytest

from openpilot.system.location_services import location_servicesd as lsd
from openpilot.system.location_services.location_servicesd import (PoliceUpdater, _line_police, _now_epoch,
                                                                   merge_retained_police)


def _real(lat, lon, uuid, age_min, thumbs):
  return {"lat": lat, "lon": lon, "magvar": None, "uuid": uuid, "street": "", "town": "", "thumbs": thumbs,
          "ts": (_now_epoch() - age_min * 60.0) * 1000.0}


# 2026-09-13 16:03:58 PT, Corvallis 3rd Street northbound (mapd RoadContext "freeway"). The forensics line
# that was on screen the moment before the truck dropped below 43 mph at ~16:04:25; the four `kept` reports.
REPORTS_0913_1603 = [
  _real(44.62528, -123.07887, "alert-111807332", 13, 0),    # the displayed one, 11.4 mi, rel 50 deg
  _real(44.48946, -123.06142, "alert-111733490", 40, 21),
  _real(44.49149, -123.06142, "alert-111756682", 34, 3),
  _real(44.48990, -123.06181, "alert-111746780", 36, 4),
]
# qlog gpsLocation over the slow stretch that followed (lat, lon, bearing); carState 43 -> 28 mph.
GPS_0913_1604 = [(44.53477, -123.26693, 4.0), (44.53740, -123.26667, 4.0), (44.54166, -123.26618, 4.0),
                 (44.54574, -123.26581, 4.0), (44.54953, -123.26536, 4.0), (44.55317, -123.26493, 4.0)]


class _FakeParams:
  def __init__(self, disabled=False):
    self.disabled = disabled

  def get_bool(self, key):
    assert key == "DisableLocationServices"
    return self.disabled


class _StopAfter:
  """Stands in for the thread's threading.Event: records every wait and ends run() after `n` of them."""
  def __init__(self, n):
    self.n, self.waits = n, []

  def is_set(self):
    return len(self.waits) >= self.n

  def wait(self, s):
    self.waits.append(s)

  def set(self):
    self.n = 0


def _updater(n_waits=1):
  """A PoliceUpdater without its Thread/Params plumbing, for driving run() deterministically."""
  pu = PoliceUpdater.__new__(PoliceUpdater)
  pu._lock = lsd.threading.Lock()
  pu._alerts, pu._state, pu._err, pu._retain = [], "nodata", "", {}
  pu._speed_ok = False
  pu._poll_state_logged = (None, None)
  pu._device_id = "test"
  pu._params = _FakeParams()
  pu._stop = _StopAfter(n_waits)
  pu._load_cfg = lambda: dict(lsd.DEFAULT_PROXY)
  pu._cur_gps = lambda: GPS_0913_1604[0][:2]
  return pu


def _recede():
  return lsd._PoliceRecede(lsd.POLICE_RECEDE_MI)


class TestHeldWhileNotPolling:
  """Gate-off and failed polls used to BLANK reports the device already had. Measured: 38.1 min on
  freeway-class roads below 43 mph, 21.3 min of it with fetched reports within 15 mi, all blank."""

  def _seed(self, pu):
    live, pu._retain = merge_retained_police({}, REPORTS_0913_1603, _now_epoch())
    return live

  def test_the_real_0913_slowdown_keeps_the_report_on_screen(self):
    pu = _updater()
    self._seed(pu)
    pu._hold(lsd._gate_reason(12.0))                     # 27 mph: the gate disarms
    alerts, state, err = pu.snapshot()
    recede = _recede()
    for lat, lon, brg in GPS_0913_1604:
      out = _line_police(alerts, state, err, lat, lon, brg, [], recede)
      assert out["state"] == "alert", f"a report we already held was blanked at {lat},{lon}"
      assert out["uuid"] == "alert-111807332"
      assert out["tier"] == "unconfirmed" and out["retained"] is True, "held must render amber"
      assert "cap" not in out, "a report we are not re-verifying reached the control channel"

  def test_gate_off_in_run_holds_instead_of_wiping(self):
    pu = _updater(n_waits=1)
    self._seed(pu)
    pu._cur_speed = lambda: 12.0                         # 27 mph, below the arm threshold
    pu.run()
    alerts, state, err = pu.snapshot()
    assert state == "held" and err == lsd._gate_reason(12.0)
    assert {a["uuid"] for a in alerts} == {a["uuid"] for a in REPORTS_0913_1603}
    assert all(a["retained"] for a in alerts)

  def test_failed_poll_in_run_holds_instead_of_blanking(self):
    pu = _updater(n_waits=1)
    self._seed(pu)
    pu._cur_speed = lambda: 26.0                         # 58 mph, armed

    def boom(cfg, lat, lon):
      raise urllib.error.URLError(OSError(101, "Network is unreachable"))
    pu._poll_proxy = boom
    pu.run()
    alerts, state, err = pu.snapshot()
    assert state == "held" and err == "net err"
    assert len(alerts) == len(REPORTS_0913_1603)

  def test_an_empty_hold_is_never_a_false_clear(self):
    out = _line_police([], "held", "speed <45mph", 44.53477, -123.26693, 4.0, [], _recede())
    assert out == {"state": "nodata", "err": "speed <45mph"}

  def test_held_never_produces_a_cap_even_for_a_live_looking_report(self):
    """The explicit guard, independent of the retained flag _hold() sets."""
    al = _real(44.56, -123.265, "fresh", 1, 6)           # ~1.8 mi ahead, 1 min old, 6 thumbs: confirmed
    assert "cap" in _line_police([al], "ok", "", 44.53477, -123.26693, 4.0, [], _recede())
    assert "cap" not in _line_police([al], "held", "net err", 44.53477, -123.26693, 4.0, [], _recede())

  def test_the_hold_still_expires(self):
    """Retention TTL still bounds what a long gate-off can show."""
    pu = _updater()
    _, pu._retain = merge_retained_police({}, REPORTS_0913_1603, _now_epoch() - lsd.POLICE_RETAIN_S - 1)
    pu._hold("speed <45mph")
    assert pu.snapshot()[0] == []


class TestPollStateIsLogged:
  """Rule 2: the gate switched polling off with no log line at all."""

  def test_change_only(self, monkeypatch):
    events = []
    monkeypatch.setattr(lsd.cloudlog, "event", lambda name, **kw: events.append((name, kw)))
    pu = _updater()
    for _ in range(5):
      pu._log_poll_state("gated", "speed <45mph")
    pu._log_poll_state("polling", "")
    pu._log_poll_state("polling", "")
    pu._log_poll_state("gated", "no GPS speed")
    assert [kw["state"] for _, kw in events] == ["gated", "polling", "gated"]
    assert events[-1][1]["reason"] == "no GPS speed" and events[-1][1]["prev_state"] == "polling"

  @pytest.mark.parametrize("speed, state", [(12.0, "gated"), (None, "gated")])
  def test_run_logs_the_gate(self, monkeypatch, speed, state):
    events = []
    monkeypatch.setattr(lsd.cloudlog, "event", lambda name, **kw: events.append(kw))
    pu = _updater(n_waits=3)
    pu._cur_speed = lambda: speed
    pu.run()
    assert len(events) == 1, "gate-off must log once, not once per check"
    assert events[0]["state"] == state and events[0]["reason"] == lsd._gate_reason(speed)


class _Clock:
  """Fake time for run(): wait(s) advances it; run() ends once it passes `end`."""
  def __init__(self, end, start=0.0):
    self.t, self.end, self.waits = start, end, []

  def is_set(self):
    return self.t >= self.end

  def wait(self, s):
    self.waits.append(s)
    self.t += s

  def set(self):
    self.end = self.t


def _clocked(end, speed_at, start=0.0):
  pu = _updater()
  pu._stop = _Clock(end, start)
  pu._cur_speed = lambda: speed_at(pu._stop.t)
  pu.polls = []
  return pu


# 2026-09-13 PT, OR-58 (qlog carState/gpsLocation + swaglog + comma-waze-proxy Lambda log). Last good poll
# 12:07:24 (t=0). The hotspot dropped ~12:08:30; the device logged "net err" at 12:08:24, 12:09:44, 12:11:04
# (-> 120 s) and 12:13:24 (-> 240 s); deviceState shows LTE back at 12:13:34 (t=370); the next poll that
# reached AWS was 12:17:30 (t=606). Each failed attempt took ~20 s (80 s between the 60 s-cadence ones).
LINK_DOWN_0913 = (55.0, 370.0)
SPEED_0913_OR58 = 26.4                                   # 59 mph, armed throughout


class TestLinkFailureDoesNotBackOff:
  def _run(self):
    pu = _clocked(900.0, lambda t: SPEED_0913_OR58)

    def poll(cfg, lat, lon):
      if LINK_DOWN_0913[0] <= pu._stop.t < LINK_DOWN_0913[1]:
        pu._stop.t += 20.0
        raise urllib.error.URLError(OSError(101, "Network is unreachable"))
      pu.polls.append(pu._stop.t)
      return []
    pu._poll_proxy = poll
    pu.run()
    return pu

  def test_the_real_0913_outage_resumes_within_one_cycle_of_the_link(self):
    pu = self._run()
    first_after = min(t for t in pu.polls if t >= LINK_DOWN_0913[1])
    # old rule: 606 s here and 12:17:30 on the truck; one cycle + one in-flight attempt is the bound now
    assert first_after <= LINK_DOWN_0913[1] + lsd.POLICE_POLL_S + 20.0, f"first poll after the link returned: t={first_after}"

  def test_the_old_schedule_is_what_the_truck_did(self):
    """Control: the simulation reproduces the truck's real 12:17:30 resume under the old rule."""
    orig = lsd.next_police_backoff
    try:
      lsd.next_police_backoff = lambda cur, n, denial, link_failure=False: orig(cur, n, denial)
      pu = self._run()
    finally:
      lsd.next_police_backoff = orig
    first_after = min(t for t in pu.polls if t >= LINK_DOWN_0913[1])
    assert abs(first_after - 606.0) <= 30.0

  def test_pure_rule(self):
    b, n = lsd.POLICE_POLL_S, 0
    for _ in range(20):
      b, n = lsd.next_police_backoff(b, n, False, link_failure=True)
      assert (b, n) == (lsd.POLICE_POLL_S, 0), "a link failure escalated the interval"

  def test_link_failure_keeps_an_earned_escalation(self):
    b, n = lsd.POLICE_POLL_S, 0
    for _ in range(5):                                  # upstream failures that reached the proxy
      b, n = lsd.next_police_backoff(b, n, False)
    earned = b
    assert earned > lsd.POLICE_POLL_S
    assert lsd.next_police_backoff(b, n, False, link_failure=True) == (earned, n)

  def test_link_failure_keeps_a_policy_denial_parked(self):
    b, n = lsd.next_police_backoff(lsd.POLICE_POLL_S, 0, True)
    assert lsd.next_police_backoff(b, n, False, link_failure=True)[0] == lsd.POLICE_MAX_BACKOFF_S

  def test_run_classifies_only_link_errors_as_link_failures(self):
    """An upstream error the proxy reported (it reached AWS, and may have billed) must still escalate."""
    pu = _clocked(1000.0, lambda t: SPEED_0913_OR58)

    def poll(cfg, lat, lon):
      pu.polls.append(pu._stop.t)
      raise lsd._ProxyUpstreamErr("upstream TimeoutError")
    pu._poll_proxy = poll
    pu.run()
    gaps = [b - a for a, b in zip(pu.polls, pu.polls[1:], strict=False)]
    assert max(gaps) > lsd.POLICE_POLL_S, "a billable upstream failure no longer backs off"


# 2026-09-13 PT 13:54:31 onward, gpsLocation speed (m/s) every 5 s: an on-ramp. 45 mph (20.1 m/s) is first
# reached at 13:55:31; the first poll that reached AWS was 13:56:36.
SPEED_0913_1354 = [0.6, 6.1, 16.5, 12.1, 11.1, 5.8, 7.9, 10.0, 6.4, 7.3, 13.0, 18.2, 23.2, 27.0, 28.1, 24.9,
                   28.8, 27.0, 26.1, 25.2, 24.9, 24.7, 24.8, 24.1, 22.9, 21.9, 17.1, 14.7, 15.9, 17.7, 16.4]
# 2026-09-12 PT 12:32:34 onward, gpsLocation speed every 5 s: the week's most threshold crossings (17 in
# 3 min, 38-53 mph) -- the case that could turn a faster gate re-check into more than 1 poll/min.
SPEED_0912_1232 = [20.2, 20.8, 19.8, 19.3, 20.0, 20.5, 20.9, 20.2, 20.5, 20.1, 20.4, 20.4, 20.3, 21.1, 20.4,
                   18.8, 20.9, 23.6, 21.8, 20.6, 17.7, 20.3, 17.9, 23.2, 19.2, 20.1, 17.2, 21.8, 20.0, 20.7,
                   18.7, 19.8, 18.9, 21.0, 22.4, 18.5, 21.2]


def _trace(samples, offset=0.0):
  return lambda t: samples[min(int((t + offset) / 5.0), len(samples) - 1)]


class TestGateRecheck:
  def _polls(self, samples, offset, end):
    pu = _clocked(end, _trace(samples, offset))

    def poll(cfg, lat, lon):
      pu.polls.append(pu._stop.t + offset)
      pu._stop.t += 2.4                                  # a proxy cache miss takes ~2.4 s (Lambda log)
      return []
    pu._poll_proxy = poll
    pu.run()
    return pu.polls

  def test_the_real_0913_onramp_polls_within_one_recheck_of_45_mph(self):
    crossing = 12 * 5.0                                   # 13:55:31
    worst = 0.0
    for offset in range(0, 60, 5):                        # every phase of the thread's own schedule
      polls = self._polls(SPEED_0913_1354, float(offset), 140.0 - offset)
      first = min(t for t in polls if t >= crossing)
      worst = max(worst, first - crossing)
    assert worst <= lsd.POLICE_GATE_RECHECK_S, f"first poll up to {worst:.0f} s after reaching 45 mph"

  def test_polls_stay_a_full_interval_apart_on_the_flappiest_real_trace(self):
    """Budget guard: the faster re-check must never produce more than one paid call per POLICE_POLL_S."""
    for offset in range(0, 60, 5):
      polls = self._polls(SPEED_0912_1232, float(offset), 180.0 - offset)
      assert len(polls) >= 2, f"the trace is mostly above 45 mph; it must poll more than once, got {polls}"
      gaps = [b - a for a, b in zip(polls, polls[1:], strict=False)]
      assert all(g >= lsd.POLICE_POLL_S for g in gaps), f"polls {gaps} s apart at phase {offset}"


class TestHeldLineSaysWhyItIsNotRefreshing:
  """policeship2pnw, Fable review of cfa47c85c9 (BLOCK): replaying _hold("daily limit") over the real 09-13
  cache published {state: alert, 11.2 mi, unconfirmed, retained, err: None} -- a 15-min 429 park showed an
  amber far report and nothing else. The reason must ride on the held line, in the payload and on screen."""

  def _held_line(self, reason):
    pu = _updater()
    _, pu._retain = merge_retained_police({}, REPORTS_0913_1603, _now_epoch())
    pu._hold(reason)
    lat, lon, brg = GPS_0913_1604[0]
    return _line_police(*pu.snapshot(), lat, lon, brg, [], _recede())

  @pytest.mark.parametrize("reason", ["daily limit", "net err", "budget exceeded", "speed <45mph"])
  def test_the_held_payload_carries_the_reason(self, reason):
    out = self._held_line(reason)
    assert out["state"] == "alert" and out["retained"] is True
    assert out["err"] == reason

  def test_a_429_park_through_run_reaches_the_line(self):
    pu = _updater(n_waits=1)
    _, pu._retain = merge_retained_police({}, REPORTS_0913_1603, _now_epoch())
    pu._cur_speed = lambda: SPEED_0913_OR58

    def denied(cfg, lat, lon):
      raise urllib.error.HTTPError(cfg["proxy_url"], 429, "Too Many Requests", None, None)
    pu._poll_proxy = denied
    pu.run()
    assert pu._stop.waits == [lsd.POLICE_MAX_BACKOFF_S]
    lat, lon, brg = GPS_0913_1604[0]
    out = _line_police(*pu.snapshot(), lat, lon, brg, [], _recede())
    assert out["state"] == "alert" and out["err"] == "daily limit"

  def test_a_live_poll_line_has_no_err(self):
    lat, lon, brg = GPS_0913_1604[0]
    alerts, _ = merge_retained_police({}, REPORTS_0913_1603, _now_epoch())
    assert "err" not in _line_police(alerts, "ok", "", lat, lon, brg, [], _recede())

  def test_the_overlay_line_shows_the_reason(self):
    import openpilot.selfdrive.ui.onroad.location_services_status as ui
    r = ui.LocationServicesStatusRenderer.__new__(ui.LocationServicesStatusRenderer)
    r._st = {"police": self._held_line("daily limit")}
    txt, color = r._police_line()
    assert txt.startswith("Police") and "daily limit" in txt, txt
    assert color == ui._C.AMBER
    r._st = {"police": {"state": "alert", "dist_mi": 3.0, "tier": "confirmed", "last_seen_min": 0}}
    assert " - " not in r._police_line()[0], "a live line must not grow a reason suffix"


class TestHeldTtlOutlivesNoBackoff:
  """Fable: after a failure the thread sleeps the whole backoff, so _hold()'s expiry ran every 900 s under a
  402/429 park and a held report could outlive POLICE_RETAIN_S by up to 15 min. The 1 Hz line re-checks it."""

  def _park(self, monkeypatch, seen_ago_s, shown_after_s):
    t0 = float(int(_now_epoch()))                      # whole seconds: the TTL boundary test is exact
    pu = _updater()
    _, pu._retain = merge_retained_police({}, REPORTS_0913_1603, t0 - seen_ago_s)
    monkeypatch.setattr(lsd, "_now_epoch", lambda: t0)
    pu._hold("daily limit")                             # the thread holds, then sleeps 900 s
    monkeypatch.setattr(lsd, "_now_epoch", lambda: t0 + shown_after_s)
    lat, lon, brg = GPS_0913_1604[0]
    return _line_police(*pu.snapshot(), lat, lon, brg, [], _recede())

  def test_expired_during_the_park_is_no_longer_shown(self, monkeypatch):
    out = self._park(monkeypatch, seen_ago_s=40 * 60, shown_after_s=10 * 60)   # 50 min since last seen
    assert out == {"state": "nodata", "err": "daily limit"}

  def test_still_inside_the_ttl_is_shown(self, monkeypatch):
    out = self._park(monkeypatch, seen_ago_s=40 * 60, shown_after_s=lsd.POLICE_RETAIN_S - 40 * 60)   # exactly TTL
    assert out["state"] == "alert"
