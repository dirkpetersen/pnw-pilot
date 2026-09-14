"""netrank2pnw (Fable N2) -- _read_gps rejects a LastGPSPosition that does not describe NOW.

mapd_configd is the only writer (in-memory store, ~1 Hz, ts = time.monotonic()), and it stops writing when
GPS dies. A stale position is the wrong input for everything that reads it here, and especially for a pin's
arrival evidence, which reads "more than 500 m from home" as proof the truck left.
"""
import json

import openpilot.system.networkd.network_arbiterd as d


class Store:
  def __init__(self, value):
    self.value = value

  def get(self, key):
    assert key == "LastGPSPosition"
    if isinstance(self.value, Exception):
      raise self.value
    return self.value


def blob(ts=None, lat=47.0, lon=-122.0):
  body = {"latitude": lat, "longitude": lon}
  if ts is not None:
    body["ts"] = ts
  return json.dumps(body)


def _setup(monkeypatch, now=1000.0):
  events = []
  monkeypatch.setattr(d.time, "monotonic", lambda: now)
  monkeypatch.setattr(d.cloudlog, "event", lambda name, **kw: events.append((name, kw)))
  monkeypatch.setattr(d, "_gps_stale_logged", "")
  return events


def test_a_fresh_bridge_reading_is_used(monkeypatch):
  _setup(monkeypatch)
  assert d._read_gps(Store(None), Store(blob(ts=995.0))) == (47.0, -122.0)


def test_a_STALE_reading_is_treated_as_no_gps_and_logged(monkeypatch):
  events = _setup(monkeypatch)
  assert d._read_gps(Store(None), Store(blob(ts=1000.0 - d.GPS_MAX_AGE_S - 1))) is None
  assert events and events[0][0] == "network_arbiterd_gps_unusable" and "stale" in events[0][1]["problem"]


def test_a_ts_from_the_FUTURE_is_a_previous_boot_and_is_rejected(monkeypatch):
  """time.monotonic restarts at boot: a persisted ts can be larger than now."""
  _setup(monkeypatch, now=50.0)
  assert d._read_gps(Store(blob(ts=27256.6)), None) is None


def test_a_reading_WITHOUT_ts_is_not_trusted(monkeypatch):
  """Not from the mapd_configd bridge -- e.g. an old value left in the persistent store."""
  events = _setup(monkeypatch)
  assert d._read_gps(Store(blob()), None) is None
  assert "no ts" in events[0][1]["problem"]


def test_a_stale_persistent_value_does_not_shadow_a_missing_live_one(monkeypatch):
  _setup(monkeypatch)
  assert d._read_gps(Store(blob(ts=10.0)), Store(None)) is None


def test_the_log_is_change_only(monkeypatch):
  events = _setup(monkeypatch)
  stale = Store(blob(ts=1.0))
  for _ in range(5):
    d._read_gps(Store(None), stale)
  assert len(events) == 1


def test_simply_having_no_gps_is_not_logged_as_a_problem(monkeypatch):
  """Absent is the ordinary pre-fix state at boot, not a malfunction worth an event every tick."""
  events = _setup(monkeypatch)
  assert d._read_gps(Store(None), Store(None)) is None
  assert events == []


def test_an_unreadable_store_is_logged_not_raised(monkeypatch):
  events = _setup(monkeypatch)
  assert d._read_gps(Store(ValueError("boom")), Store(None)) is None
  assert "unreadable" in events[0][1]["problem"]
