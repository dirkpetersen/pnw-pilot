"""gpsfix2pnw: the device GPS bridge must not write a position the receiver says is not a fix.

The motivating record: Sat 2026-09-12 06:28:57-06:29:00 PT, NF Road 70 (Klamath Co.), route
0000012f--817cabeb76 segment 4. After a 256 s cold start qcomgpsd published four `hasFix=False`
messages 2.18-2.42 km from the truck, and mapd_configd wrote them into LastGPSPosition, from where
they reached every consumer (drives/2026-09-12/central-oregon-weekend/GPS_TRUCK_VS_COMMA.md s1).

Everything here runs the REAL mapd_configd.main() loop (configd_replay.py) and, where it matters, the
REAL consumers that read the blob afterwards (network_arbiterd._read_gps, location_servicesd._read_mem).
"""
import json
import math

import pytest

from openpilot.system.mapd import mapd_configd
from openpilot.system.mapd.tests import configd_replay as R

# The comma's gpsLocation stream from the qlog, verbatim: (seconds after the first message, lat, lon,
# speed m/s, bearingDeg, hasFix, verticalAccuracy). Times are logMonoTime deltas.
COLD_START = [
  (0.000, 43.09433457961998, -121.9642927672798, 9.916, 338.91, False, 500.0),
  (1.101, 43.092395936369655, -121.96305606193101, 10.965, 339.33, False, 500.0),
  (2.109, 43.09376527977834, -121.96385502022414, 11.853, 337.13, False, 500.0),
  (3.000, 43.09386243944019, -121.96390934848873, 11.608, 337.33, False, 500.0),
  (4.101, 43.074871313665625, -121.95327916641573, 11.086, 334.75, True, 7.833),
  (5.008, 43.075088725785996, -121.95344370758029, 8.780, 333.92, True, 7.832),
  (6.100, 43.075126254349136, -121.95345571548368, 9.103, 333.01, True, 5.247),
  (7.011, 43.07522375061925, -121.95349632703315, 9.367, 335.99, True, 5.547),
  (8.101, 43.07528988247053, -121.95352923370172, 9.547, 336.48, True, 5.547),
  (9.001, 43.07532842943611, -121.95352997035809, 9.898, 339.68, True, 5.548),
  (10.106, 43.07541184686066, -121.95357189335037, 9.893, 339.69, True, 5.548),
  (11.005, 43.075488737232725, -121.95361673903793, 10.081, 339.61, True, 5.252),
  (12.099, 43.07557819283106, -121.95365971815069, 10.555, 339.99, True, 5.253),
  (13.098, 43.07572369311966, -121.95368227403709, 11.723, 346.19, True, 5.254),
  (14.007, 43.07592381914539, -121.95326762554689, 9.668, 346.73, True, 1.727),
]
# The truck's own CAN fix at the same seconds (ces_events car_gps; the reference the report used here,
# where OSM has no mapped road), t relative to the same origin.
TRUCK = [
  (0.892, 43.074432, -121.952465), (1.892, 43.074513, -121.952500), (2.992, 43.074597, -121.952535),
  (3.992, 43.074680, -121.952570), (4.992, 43.074763, -121.952603), (5.992, 43.074848, -121.952638),
  (6.992, 43.074935, -121.952673), (7.992, 43.075020, -121.952707), (8.992, 43.075105, -121.952740),
  (9.992, 43.075188, -121.952772), (10.992, 43.075275, -121.952805), (11.992, 43.075362, -121.952840),
  (12.992, 43.075450, -121.952875), (13.992, 43.075538, -121.952912), (14.992, 43.075625, -121.952947),
]
T0 = 10.0   # the first message arrives 10 s into the replay (the tail of the 256 s silent cold start)


def _hav_m(la1, lo1, la2, lo2):
  p1, p2 = math.radians(la1), math.radians(la2)
  a = math.sin((p2 - p1) / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(math.radians(lo2 - lo1) / 2) ** 2
  return 2 * 6371008.8 * math.asin(math.sqrt(a))


def _truck_at(t):
  return min(TRUCK, key=lambda r: abs(r[0] + T0 - t))


def _steps(fixes, t_end, t0=T0, extra=None):
  """A 20 Hz loop (mapdOut is 20 Hz, so that is how often main() really iterates while mapd runs),
  with each GPS message delivered on the first iteration at or after its publish time."""
  steps = {round(k * 0.05, 3): [("mapdOut", {})] for k in range(int(t_end / 0.05) + 1)}
  for rt, lat, lon, spd, brg, fix, vacc in fixes:
    t = round(math.ceil((rt + t0) / 0.05) * 0.05, 3)
    steps[t] = steps.get(t, []) + [R.gps_msg(lat, lon, has_fix=fix, speed=spd, bearing=brg, vacc=vacc)]
  for t, msgs in (extra or {}).items():
    steps[t] = steps.get(t, []) + msgs
  return sorted(steps.items())


class TestColdStartReplay:
  def test_no_fix_positions_never_reach_the_blob(self, monkeypatch):
    res = R.run(monkeypatch, _steps(COLD_START, T0 + 15.0))
    pos = R.positions(res)
    n_fix = sum(1 for r in COLD_START if r[5])
    assert len(pos) == n_fix, "exactly one write per hasFix message"
    for t, d in pos:
      _, tla, tlo = _truck_at(t)
      off = _hav_m(d["latitude"], d["longitude"], tla, tlo)
      assert off < 150.0, f"wrote a position {off:.0f} m from the truck at t={t}"
    # and the four no-fix coordinates are nowhere in the stream
    bad = {(r[1], r[2]) for r in COLD_START if not r[5]}
    assert not bad & {(d["latitude"], d["longitude"]) for _, d in pos}

  def test_the_real_consumers_never_see_the_bad_positions(self, monkeypatch):
    """What network_arbiterd and location_servicesd read, after EVERY loop iteration."""
    from openpilot.system.networkd import network_arbiterd as na
    from openpilot.system.location_services import location_servicesd as ls
    seen = []

    class Store:
      def __init__(self, d):
        self.d = d

      def get(self, key, return_default=False):
        return self.d.get(key)

    def on_step(t, store):
      monkeypatch.setattr(na.time, "monotonic", lambda: t)
      arb = na._read_gps(Store({}), Store(store))
      lat, lon, _brg, *_ = ls._read_mem(Store(store))
      seen.append((t, arb, (lat, lon) if lat is not None else None))

    R.run(monkeypatch, _steps(COLD_START, T0 + 15.0), on_step=on_step)
    got = [(t, p) for t, a, l in seen for p in (a, l) if p is not None]
    assert got, "the consumers saw nothing at all -- the replay did not run"
    for t, (lat, lon) in got:
      _, tla, tlo = _truck_at(t)
      assert _hav_m(lat, lon, tla, tlo) < 150.0, f"a consumer saw a position far from the truck at t={t}"
    # before the first hasFix message the consumers see NO position, not a wrong one
    first_fix_t = T0 + COLD_START[4][0]
    assert all(a is None and l is None for t, a, l in seen if t < first_fix_t)

  def test_fix_loss_and_recovery_are_logged_change_only(self, monkeypatch):
    res = R.run(monkeypatch, _steps(COLD_START, T0 + 15.0))
    ev = R.fix_events(res)
    assert [e["state"] for e in ev] == ["silent", "nofix", "fix"]
    assert ev[2]["prev"] == "nofix" and ev[2]["nofix_dropped"] == 4
    assert ev[1]["prev"] == "silent"


class TestWriteOnArrivalNotWhileAlive:
  def test_a_fix_is_written_once_with_its_arrival_time(self, monkeypatch):
    fixes = [(0.0, 47.6, -122.3, 25.0, 180.0, True, 5.0)]
    res = R.run(monkeypatch, _steps(fixes, 10.0, t0=1.0))
    pos = R.positions(res)
    assert len(pos) == 1, f"the 20 Hz loop rewrote one fix {len(pos)} times"
    assert pos[0][1]["ts"] == pytest.approx(1.0)

  def test_a_stopped_receiver_stops_the_writes_so_ts_ages(self, monkeypatch):
    """The SR 99 tunnel shape: good fixes, then nothing. Nothing may be written after the last
    message, so the blob's ts ages and network_arbiterd drops it after its 10 s window."""
    from openpilot.system.networkd import network_arbiterd as na
    fixes = [(float(i), 47.6 + i * 1e-4, -122.3, 25.0, 180.0, True, 5.0) for i in range(5)]
    res = R.run(monkeypatch, _steps(fixes, 20.0, t0=1.0))
    pos = R.positions(res)
    assert max(t for t, _ in pos) == pytest.approx(5.0)
    blob = res.mem.store["LastGPSPosition"]
    monkeypatch.setattr(na.time, "monotonic", lambda: 5.0 + na.GPS_MAX_AGE_S + 0.5)
    assert na._read_gps(type("P", (), {"get": lambda s, k: None})(),
                        type("M", (), {"get": lambda s, k: blob})()) is None
    assert [e["state"] for e in R.fix_events(res)] == ["silent", "fix", "silent"]

  def test_a_no_fix_after_a_fix_leaves_the_old_ts_to_expire(self, monkeypatch):
    fixes = [(0.0, 47.6, -122.3, 25.0, 180.0, True, 5.0)] + \
            [(float(i), 47.7, -122.3, 25.0, 180.0, False, 500.0) for i in range(1, 12)]
    res = R.run(monkeypatch, _steps(fixes, 13.0, t0=1.0))
    pos = R.positions(res)
    assert len(pos) == 1 and pos[0][1]["latitude"] == 47.6 and pos[0][1]["ts"] == pytest.approx(1.0)


class TestRegionGateNeedsAFix:
  def test_a_no_fix_position_does_not_pick_a_region_to_download(self, monkeypatch):
    """has_fix used to be `alive` only. A no-fix message placed in Washington must not request WA;
    the first real fix (Oregon) must request OR. tileLoaded False = uncovered; mapdExtendedOut alive."""
    # mapdgrace2pnw: the OR fix must now read unloaded for COVERAGE_GRACE_S before it requests, so the
    # fix keeps arriving and the replay runs past that window.
    ext = {round(k * 1.0, 3): [("mapdExtendedOut", {})] for k in range(20)}
    fixes = [(float(i), 47.6, -122.3, 25.0, 180.0, False, 500.0) for i in range(4)] + \
            [(float(i), 44.0, -121.3, 25.0, 180.0, True, 5.0) for i in range(4, 17)]
    res = R.run(monkeypatch, _steps(fixes, 17.0, t0=1.0, extra=ext))
    keys = [m.mapdIn.str for _, s, m in res.sent if s == "mapdIn"]
    assert keys, "no download request at all -- the uncovered path did not run"
    assert all("OR" in k.upper() or "oregon" in k.lower() for k in keys), keys
    assert res.params.store.get("MapForLocationCovered") is False


def test_blob_shape_is_what_the_readers_parse(monkeypatch):
  res = R.run(monkeypatch, _steps([(0.0, 47.6, -122.3, 25.0, 180.0, True, 5.0)], 2.0, t0=1.0))
  raw = res.mem.store["LastGPSPosition"]
  d = json.loads(raw)
  assert list(d) == ["latitude", "longitude", "bearing", "speed", "src", "ts", "fix_ts"]
  assert d["src"] == "device"
  assert d["fix_ts"] == pytest.approx(d["ts"] - mapd_configd.DEVICE_GPS_FIX_LATENCY_S)   # gpslag2pnw
  assert mapd_configd.DEVICE_GPS_FIX_LATENCY_S == 0.57
  assert mapd_configd.GPS_SILENT_S == 3.0
