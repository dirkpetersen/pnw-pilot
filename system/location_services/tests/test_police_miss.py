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
