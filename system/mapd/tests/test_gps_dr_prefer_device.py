"""gpsdr2pnw: a degraded truck fix (HDOP >= 3.8 for > 10 s) yields to a fresh device fix.

Owner decision 2026-09-13: "Yes, prefer the good comma fix". HDOP is the only quality signal CarGps
carries. On the weekend + I-5 data it read >= 3.8 in three runs: the SR 99 tunnel (device dead -> the
truck must keep the blob), the Sat 06:24 cold start (device dead too) and 6 min parked after the Sat
14:12 boot (device fix fresh -> the device should win).

All scenarios run the REAL mapd_configd.main() loop (configd_replay.py) at 20 Hz.
"""
from openpilot.system.mapd import mapd_configd as M
from openpilot.system.mapd.tests import configd_replay as R
from openpilot.system.mapd.tests.test_gps_source_select import TESLA_CP, _device_track, _grid, _run, _source_events, _straight


def _with_hdop(truck, hdop_of_i):
  return [r[:5] + (hdop_of_i(i),) + r[6:] for i, r in enumerate(truck)]


def _srcs(res, t0, t1):
  return {d["src"] for t, d in R.positions(res) if t0 <= t < t1}


class TestDegradedTruckYields:
  def test_after_10s_degraded_the_fresh_device_fix_is_used_and_the_truck_returns_after_5s_good(self, monkeypatch):
    truck = _with_hdop(_straight(40), lambda i: 3.8 if 5 <= i < 25 else 0.4)
    res = _run(monkeypatch, 40.0, _device_track(40), truck)
    enter = truck[16][0]   # first publish more than 10 s after the first degraded one (5.3 -> 16.3)
    leave = truck[30][0]   # first publish 5 s into the good run (25.3 -> 30.3)
    assert _srcs(res, _grid(truck[3][0]), _grid(enter)) == {"car"}, "switched before 10 s of degraded HDOP"
    assert _srcs(res, _grid(enter), _grid(leave)) == {"device"}
    assert _srcs(res, _grid(leave), 40.0) == {"car"}, "the truck did not come back after 5 s of good HDOP"
    ev = [(e["src"], e["car"]) for e in _source_events(res)]
    assert ("device", "dr") in ev and ev[-1] == ("car", "ok")
    sw = [e for e in _source_events(res) if e["src"] == "device" and e["prev"] == "car"]
    assert len(sw) == 1 and "HDOP 3.8" in sw[0]["car_detail"]

  def test_hdop_flapping_every_publish_never_switches(self, monkeypatch):
    truck = _with_hdop(_straight(40), lambda i: 3.8 if i % 2 else 0.4)
    res = _run(monkeypatch, 40.0, _device_track(40), truck)
    assert _srcs(res, _grid(truck[3][0]), 40.0) == {"car"}
    assert not [e for e in _source_events(res) if e["car"] == "dr"]

  def test_one_good_publish_inside_the_exit_window_does_not_bring_the_truck_back(self, monkeypatch):
    # degraded 3..17 (dr from 14.3); good 18..21; ONE bad at 22; good from 23 -> exit at 28.3, not 23.3
    truck = _with_hdop(_straight(40), lambda i: 3.8 if (3 <= i < 18 or i == 22) else 0.4)
    res = _run(monkeypatch, 40.0, _device_track(40), truck)
    assert _srcs(res, _grid(truck[15][0]), _grid(truck[28][0])) == {"device"}
    assert _srcs(res, _grid(truck[28][0]), 40.0) == {"car"}

  def test_the_device_losing_its_fix_hands_the_blob_back_to_the_degraded_truck(self, monkeypatch):
    truck = _with_hdop(_straight(40), lambda i: 3.8 if i >= 3 else 0.4)
    device = _device_track(20)                                        # device fixes stop at 19.6
    res = _run(monkeypatch, 40.0, device, truck)
    back = 19.6 + M.GPS_SILENT_S
    assert _srcs(res, _grid(truck[15][0]), 19.7) == {"device"}
    assert _srcs(res, back + 1.0, 40.0) == {"car"}, "no position at all once the device went silent"
    ev = _source_events(res)[-1]
    assert (ev["src"], ev["car"], ev["device"]) == ("car", "dr", "silent")

  def test_a_no_fix_device_is_not_a_fresh_device(self, monkeypatch):
    truck = _with_hdop(_straight(30), lambda i: 3.8)
    device = [r[:5] + (False,) for r in _device_track(30)]
    res = _run(monkeypatch, 30.0, device, truck)
    assert _srcs(res, 0.0, 30.0) == {"car"}

  def test_a_nan_hdop_is_not_a_fix(self, monkeypatch):
    truck = _with_hdop(_straight(12), lambda i: float("nan") if i == 6 else 0.4)
    res = _run(monkeypatch, 12.0, _device_track(12), truck)
    assert not [d for t, d in R.positions(res) if d["src"] == "car" and t == _grid(truck[6][0])]
    assert any(e["car"] == "invalid" for e in _source_events(res))

  def test_unknown_hdop_sentinel_counts_as_degraded(self, monkeypatch):
    truck = _with_hdop(_straight(30), lambda i: 6.2)                 # DBC 31 "Invalid" * 0.2
    res = _run(monkeypatch, 30.0, _device_track(30), truck)
    assert _srcs(res, _grid(truck[13][0]), 30.0) == {"device"}

  def test_parked_after_boot_like_sat_1412_keeps_the_truck(self, monkeypatch):
    """gpsdrgate2pnw (Fable): Sat 2026-09-12 14:12:14 PT, parked, truck HDOP a constant 3.8 for 381 s with its
    position exactly right, while the device jittered 3.7 m mean / 8.7 m max and its bearing wandered. The
    device (speed noise up to 4.1 m/s, under DEVICE_GPS_MOVING_MS) must NOT take the blob."""
    truck = [(i + 0.3, 44.0, -121.3, 120.0, 0.0, 3.8, 0.3) for i in range(40)]
    device = [(i + 0.6, 44.00001 + (i % 3) * 3e-5, -121.30001, (0.3, 4.1, 1.2)[i % 3], (i * 53) % 360, True) for i in range(40)]
    res = _run(monkeypatch, 40.0, device, truck)
    assert _srcs(res, _grid(truck[3][0]), 40.0) == {"car"}
    dr = [e for e in _source_events(res) if e["car"] == "dr"]
    assert len(dr) == 1 and dr[0]["src"] == "car" and "never moved" in dr[0]["car_detail"]


class TestDegradedYieldGates:
  """gpsdrgate2pnw: the device takes a degraded truck's blob only while moving, and only once steady."""

  def test_stopping_hands_back_to_the_truck_5_s_after_the_last_moving_publish(self, monkeypatch):
    moving = _with_hdop(_straight(25), lambda i: 3.8)
    last = moving[-1]
    parked = [(last[0] + k, last[1], last[2], last[3], 0.0, 3.8, 0.4) for k in range(1, 16)]
    device = _device_track(25) + [(25 + k + 0.6, 47.61, -122.30002, 0.2, 0.0, True) for k in range(15)]
    res = _run(monkeypatch, 40.0, device, moving + parked)
    assert _srcs(res, _grid(moving[15][0]), _grid(last[0]) + 0.05) == {"device"}
    hand_back = last[0] + M.CAR_GPS_DR_EXIT_S
    assert _srcs(res, _grid(last[0]) + 0.05, hand_back - 0.3) == {"device"}, "handed back before the 5 s hold-off"
    assert _srcs(res, hand_back + 1.0, 40.0) == {"car"}, "the truck did not take back a parked blob"

  def test_crawling_around_3_mph_does_not_swap_receivers_every_publish(self, monkeypatch):
    base = _with_hdop(_straight(40, spd=4.0), lambda i: 3.8)
    crawl = [r[:4] + ((4.0 if i % 2 else 2.0),) + r[5:] for i, r in enumerate(base)]   # moving every other publish
    device = _device_track(40, speed=1.5)
    res = _run(monkeypatch, 40.0, device, crawl)
    assert _srcs(res, _grid(crawl[13][0]), 40.0) == {"device"}
    ev = _source_events(res)
    first_device = next(i for i, e in enumerate(ev) if e["src"] == "device" and e["prev"] == "car")
    assert not [e for e in ev[first_device + 1:] if e["src"] != e["prev"]], ev[first_device:]

  def test_a_device_fix_must_be_steady_for_10_s_before_it_takes_the_blob(self, monkeypatch):
    truck = _with_hdop(_straight(45), lambda i: 3.8)                  # degraded from the start (dr at 11.3)
    device = [r for r in _device_track(45) if r[0] >= 15.0]            # first device fix at 15.6
    device = [r if abs(r[0] - 30.6) > 0.01 else r[:5] + (False,) for r in device]   # one no-fix at 30.6
    res = _run(monkeypatch, 45.0, device, truck)
    assert _srcs(res, 0.0, 25.6) == {"car"}, "a device fix less than 10 s old took the blob"
    assert _srcs(res, 25.7, 30.6) == {"device"}
    assert _srcs(res, 30.7, 41.5) == {"car"}, "one no-fix sample did not restart the steady clock"
    assert _srcs(res, 41.7, 45.0) == {"device"}
    sw = [e for e in _source_events(res) if e["src"] == "device" and e["prev"] == "car"]
    assert sw and all(e["device_fix_s"] >= M.CAR_GPS_DR_ENTER_S for e in sw), sw
    held = [e for e in _source_events(res) if e["src"] == "car" and e["car"] == "dr"]
    assert held and held[0]["device_fix_s"] is not None



def test_ok_detail_names_the_hdop(monkeypatch):
  res = _run(monkeypatch, 8.0, _device_track(8), _straight(8))
  ok = [e for e in _source_events(res) if e["car"] == "ok"]
  assert ok and "HDOP 0.4" in ok[-1]["car_detail"]


def test_tesla_ignores_a_degraded_cargps_feed(monkeypatch):
  truck = _with_hdop(_straight(30), lambda i: 3.8)
  tesla = _run(monkeypatch, 30.0, _device_track(30), truck, cp=TESLA_CP)
  bare = _run(monkeypatch, 30.0, _device_track(30), (), cp=None)
  assert tesla.mem.writes == bare.mem.writes and R.positions(tesla)
  assert "CarGps" not in tesla.mem.gets
