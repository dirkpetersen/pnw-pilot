"""gpssel2pnw: mapd_configd selects the car's CAN GPS for LastGPSPosition only while it is provably live.

Evidence: drives/2026-09-12/central-oregon-weekend/GPS_TRUCK_VS_COMMA.md (the truck fix is 1.6 m vs
3.0 m median from the lane, and covered the two cold starts and the SR 99 tunnel). The failure this
must never reintroduce is the 2026-09-05 wrong-bus freeze: a car_gps that read as live for 15 min
while it sat 25 km behind (drives/2026-09-05/i5-north-everett-burlington/DRIVE_REPORT.md).

Every scenario runs the REAL mapd_configd.main() loop (configd_replay.py) at mapdOut's 20 Hz, with the
CarGps mem-param published the way card publishes it (~1 Hz, {lat, lon, hdg, spd MPH, sats, hdop, ts,
age}), and the real consumers reading the blob where it matters.
"""
import json
import math

import pytest
from cereal import car

from openpilot.selfdrive.controls.lib.pnw_vehicle import PnwVehicle
from openpilot.system.mapd import mapd_configd as M
from openpilot.system.mapd.tests import configd_replay as R
from openpilot.system.mapd.tests.test_gps_fix_gate import COLD_START

DT = 0.05


def _cp(fp, brand):
  cp = car.CarParams.new_message()
  cp.carFingerprint, cp.brand = fp, brand
  return cp.to_bytes()


LIGHTNING_CP = _cp("FORD_F_150_LIGHTNING_MK1", "ford")
TESLA_CP = _cp("TESLA_MODEL_S_HW3", "tesla")

# SR 99 tunnel entry, Tue 2026-09-08, t = s after 19:29:10 PT (ces_events of that drive). The truck's
# CarGps publishes: (t, lat, lon, hdg, spd MPH, hdop, age). hdop goes 0.4 -> 3.8 as it enters.
TUNNEL_TRUCK = [
  (0.81, 47.623168, -122.344137, 195.5, 34.0, 0.4, 0.41), (1.81, 47.623028, -122.344192, 195.4, 37.0, 0.4, 0.42),
  (2.81, 47.62291, -122.344238, 195.2, 38.0, 0.4, 0.44), (3.81, 47.622723, -122.344313, 195.0, 41.0, 0.4, 0.41),
  (4.81, 47.62256, -122.344378, 194.9, 42.0, 0.4, 0.42), (5.81, 47.622427, -122.344432, 194.5, 43.0, 0.4, 0.44),
  (6.81, 47.622223, -122.34451, 192.8, 44.0, 0.4, 0.41), (7.81, 47.622052, -122.34457, 191.6, 44.0, 0.6, 0.42),
  (8.81, 47.621913, -122.344613, 190.8, 44.0, 0.6, 0.44), (9.81, 47.621703, -122.344672, 189.4, 44.0, 0.8, 0.41),
  (10.81, 47.621523, -122.344718, 188.8, 46.0, 1.2, 0.42), (11.81, 47.621372, -122.344752, 188.2, 48.0, 1.2, 0.44),
  (12.81, 47.621132, -122.344808, 187.8, 51.0, 3.8, 0.41), (13.81, 47.620923, -122.344855, 187.2, 52.0, 3.8, 0.42),
  (14.81, 47.620753, -122.344893, 186.7, 53.0, 3.8, 0.44), (15.81, 47.620495, -122.344938, 185.8, 54.0, 3.8, 0.41),
  (16.81, 47.620278, -122.344968, 185.0, 54.0, 3.8, 0.42), (17.81, 47.620103, -122.344997, 184.7, 54.0, 3.8, 0.44),
  (18.81, 47.619842, -122.34503, 183.9, 54.0, 3.8, 0.41), (19.81, 47.619623, -122.345052, 183.0, 54.0, 3.8, 0.42),
  (20.81, 47.619448, -122.345065, 182.5, 54.0, 3.8, 0.44), (21.81, 47.619187, -122.345078, 181.6, 54.0, 3.8, 0.41),
  (22.81, 47.618968, -122.345083, 180.9, 54.0, 3.8, 0.41), (23.81, 47.618792, -122.345082, 180.2, 54.0, 3.8, 0.44),
  (24.81, 47.61853, -122.345075, 179.2, 54.0, 3.8, 0.41), (25.81, 47.61831, -122.345063, 178.5, 54.0, 3.8, 0.42),
  (26.81, 47.618135, -122.34505, 177.7, 54.0, 3.8, 0.44), (27.81, 47.617873, -122.345028, 176.7, 54.0, 3.8, 0.41),
  (28.81, 47.617655, -122.345005, 176.1, 54.0, 3.8, 0.42), (29.81, 47.61748, -122.34498, 175.4, 54.0, 3.8, 0.44),
]
# The device's gpsLocation until it froze at 19:29:19.3: (t, lat, lon, speed m/s, bearing, hasFix).
TUNNEL_DEVICE = [
  (0.2, 47.62357316684328, -122.3440797649004, 15.1, 195.0, True),
  (1.2, 47.62340780954536, -122.34415402829191, 16.1, 195.0, True),
  (2.2, 47.62326204235938, -122.34421442234216, 17.2, 195.0, True),
  (3.2, 47.623116925538895, -122.34425960240752, 18.1, 195.0, True),
  (4.3, 47.62297144700085, -122.34431154472857, 18.6, 195.0, True),
  (5.3, 47.62280525360793, -122.34436205500343, 18.9, 195.0, True),
  (6.3, 47.62266071227674, -122.34440165067146, 19.1, 195.0, True),
  (7.3, 47.62250871269097, -122.34446892836174, 19.2, 195.0, True),
  (8.3, 47.62235773723057, -122.34454493236244, 19.3, 195.0, True),
  (9.3, 47.622206403688864, -122.34460965171661, 19.5, 195.0, True),
]
# Sat 2026-09-12 06:28:40-06:29:10 PT cold start, truck side, t = s after the device's first (no-fix)
# message (as in test_gps_fix_gate.COLD_START). Publish time = ces read time - 0.3 s.
COLD_TRUCK = [
  (-16.5, 43.073058, -121.951795, 344.2, 20.0, 3.8, 0.27), (-15.5, 43.073137, -121.951833, 343.7, 20.0, 1.2, 0.24),
  (-14.5, 43.073215, -121.951872, 342.9, 19.0, 1.4, 0.25), (-13.5, 43.073297, -121.951913, 342.4, 20.0, 1.4, 0.27),
  (-12.4, 43.073377, -121.951955, 341.8, 20.0, 1.2, 0.24), (-11.4, 43.073458, -121.951998, 341.0, 20.0, 1.2, 0.25),
  (-10.4, 43.073542, -121.952042, 340.3, 20.0, 1.2, 0.27), (-9.4, 43.073623, -121.952085, 340.0, 21.0, 1.2, 0.24),
  (-8.4, 43.073705, -121.952128, 339.6, 21.0, 1.2, 0.25), (-7.4, 43.073785, -121.952170, 339.3, 21.0, 1.2, 0.27),
  (-6.4, 43.073865, -121.952210, 339.5, 21.0, 1.2, 0.24), (-5.4, 43.073943, -121.952250, 339.8, 21.0, 1.2, 0.25),
  (-4.4, 43.074023, -121.952285, 340.4, 21.0, 1.2, 0.27), (-3.4, 43.074103, -121.952322, 340.6, 21.0, 1.2, 0.24),
  (-2.4, 43.074185, -121.952357, 340.3, 22.0, 1.2, 0.25), (-1.4, 43.074267, -121.952393, 339.9, 22.0, 1.2, 0.27),
  (-0.4, 43.074348, -121.952428, 340.1, 22.0, 1.2, 0.24), (0.6, 43.074432, -121.952465, 340.2, 22.0, 1.2, 0.25),
  (1.6, 43.074513, -121.952500, 340.3, 22.0, 1.2, 0.27), (2.7, 43.074597, -121.952535, 340.6, 22.0, 1.2, 0.24),
  (3.7, 43.074680, -121.952570, 341.1, 22.0, 1.2, 0.25), (4.7, 43.074763, -121.952603, 341.2, 22.0, 1.2, 0.27),
  (5.7, 43.074848, -121.952638, 341.3, 22.0, 1.2, 0.24), (6.7, 43.074935, -121.952673, 341.3, 23.0, 1.2, 0.25),
  (7.7, 43.075020, -121.952707, 341.5, 22.0, 1.2, 0.27), (8.7, 43.075105, -121.952740, 342.1, 22.0, 1.2, 0.24),
  (9.7, 43.075188, -121.952772, 342.1, 22.0, 1.2, 0.25), (10.7, 43.075275, -121.952805, 342.0, 22.0, 1.2, 0.27),
  (11.7, 43.075362, -121.952840, 341.9, 23.0, 1.2, 0.24), (12.7, 43.075450, -121.952875, 342.0, 23.0, 1.2, 0.25),
]


def _grid(t):
  return round(math.ceil(round(t / DT, 6)) * DT, 3)


def _cargps(t, lat, lon, hdg, spd, hdop=0.4, age=0.4):
  return {"lat": lat, "lon": lon, "hdg": hdg, "spd": spd, "sats": 31, "hdop": hdop, "ts": round(1.789e9 + t, 2), "age": age}


def _run(monkeypatch, t_end, device=(), truck=(), cp=LIGHTNING_CP, t_shift=0.0, on_step=None, params_script=None):
  """device rows: (t, lat, lon, speed, bearing, hasFix); truck rows: (t, lat, lon, hdg, spd, hdop, age).
  t_shift moves both streams later (so a fixture with negative times starts inside the replay)."""
  steps = {round(k * DT, 3): [("mapdOut", {})] for k in range(int(round(t_end / DT)) + 1)}
  for t, lat, lon, spd, brg, fix in device:
    if 0.0 <= t + t_shift <= t_end:
      steps[_grid(t + t_shift)].append(R.gps_msg(lat, lon, has_fix=fix, speed=spd, bearing=brg))
  mem_script = {}
  for t, lat, lon, hdg, spd, hdop, age in truck:
    if 0.0 <= t + t_shift <= t_end:
      mem_script.setdefault(_grid(t + t_shift), {})["CarGps"] = _cargps(t + t_shift, lat, lon, hdg, spd, hdop, age)
  persistent = {"CarParams": cp} if cp is not None else {}
  return R.run(monkeypatch, sorted(steps.items()), persistent=persistent, mem_script=mem_script, on_step=on_step,
               params_script=params_script)


def _source_events(res):
  return [kw for name, kw in res.log.events if name == "mapd_configd_gps_source"]


def _hav_m(la1, lo1, la2, lo2):
  p1, p2 = math.radians(la1), math.radians(la2)
  a = math.sin((p2 - p1) / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(math.radians(lo2 - lo1) / 2) ** 2
  return 2 * 6371008.8 * math.asin(math.sqrt(a))


class _Store:
  def __init__(self, d):
    self.d = d

  def get(self, key, return_default=False):
    return self.d.get(key)


# --- replays from the drives ----------------------------------------------------------------------

class TestTunnelReplay:
  def test_the_truck_keeps_the_position_live_through_the_tunnel(self, monkeypatch):
    from openpilot.system.networkd import network_arbiterd as na
    from openpilot.system.location_services import location_servicesd as ls
    seen = []

    def on_step(t, store):
      monkeypatch.setattr(na.time, "monotonic", lambda: t)
      seen.append((t, na._read_gps(_Store({}), _Store(store)), ls._read_mem(_Store(store))[2]))

    res = _run(monkeypatch, 31.0, TUNNEL_DEVICE, TUNNEL_TRUCK, on_step=on_step)
    pos = R.positions(res)
    car = [(t, d) for t, d in pos if d["src"] == "car"]
    # acquired on the 3rd healthy publish (2.81 s), then exactly one write per publish to the end
    assert car and car[0][0] == pytest.approx(_grid(2.81))
    assert len(car) == len([r for r in TUNNEL_TRUCK if r[0] >= 2.81])
    assert all(d["src"] == "device" for t, d in pos if t < _grid(2.81))
    assert not [t for t, d in pos if d["src"] == "device" and t >= _grid(2.81)], "device written while the car was selected"
    for (_t, d), row in zip(car, [r for r in TUNNEL_TRUCK if r[0] >= 2.81], strict=True):
      assert (d["latitude"], d["longitude"], d["bearing"]) == (row[1], row[2], row[3])
      assert d["speed"] == pytest.approx(row[4] * 0.44704)
    # the consumer never loses the position after acquisition, and heading is the truck's
    after = [(t, p, b) for t, p, b in seen if t >= 3.0]
    assert after and all(p is not None for _, p, _ in after)
    assert after[-1][2] == TUNNEL_TRUCK[-1][3]
    assert [e["src"] for e in _source_events(res)][-1] == "car"
    assert sum(1 for e in _source_events(res) if e["src"] == "car" and e["prev"] != "car") == 1, "the car was lost and re-selected"
    # gpsdr2pnw: HDOP 3.8 from 12.81 s is degraded after 10 s, but the device is dead in the tunnel, so the
    # truck keeps the blob and the degraded state is still logged once
    dr = [e for e in _source_events(res) if e["car"] == "dr"]
    assert len(dr) == 1 and dr[0]["src"] == "car" and dr[0]["device"] == "silent"

  def test_without_the_capability_the_blob_goes_stale_in_the_tunnel_as_before(self, monkeypatch):
    res = _run(monkeypatch, 31.0, TUNNEL_DEVICE, TUNNEL_TRUCK, cp=TESLA_CP)
    pos = R.positions(res)
    assert pos and all(d["src"] == "device" for _, d in pos)
    assert max(t for t, _ in pos) == pytest.approx(_grid(9.3))


class TestColdStartReplay:
  def test_the_truck_covers_the_cold_start_and_no_fix_positions_never_appear(self, monkeypatch):
    device = [(t, la, lo, spd, brg, fix) for t, la, lo, spd, brg, fix, _v in COLD_START]
    res = _run(monkeypatch, 30.0, device, COLD_TRUCK, t_shift=17.0)
    pos = R.positions(res)
    assert pos[0][1]["src"] == "car" and pos[0][0] < 17.0, "no position during the device's silent cold start"
    truck = [(t + 17.0, la, lo) for t, la, lo, *_ in COLD_TRUCK]
    for t, d in pos:
      _, tla, tlo = min(truck, key=lambda r: abs(r[0] - t))
      assert _hav_m(d["latitude"], d["longitude"], tla, tlo) < 40.0, f"t={t} {d}"
    assert all(d["src"] == "car" for _, d in pos)


# --- the freeze classes ---------------------------------------------------------------------------

def _straight(n, t0=0.0, spd=56.0, age=0.4, lat0=47.60):
  """A truck doing `spd` MPH due north, one publish per second."""
  step = spd * 0.44704 / 111320.0
  return [(t0 + i + 0.3, lat0 + i * step, -122.3, 0.0, spd, 0.4, age) for i in range(n)]


def _device_track(n, t0=0.0, speed=25.0, lat0=47.60, fix=True, brg=0.0):
  step = speed / 111320.0
  return [(t0 + i + 0.6, lat0 + i * step, -122.30002, speed, brg, fix) for i in range(n)]


class TestFreezeFallsBackLoudly:
  def test_can_frames_stop_age_climbs(self, monkeypatch):
    """The wrong-bus class, isolated: publishes keep coming (ts advances), position even moves, but
    the CAN frame they carry keeps getting older."""
    truck = _straight(12)
    truck = [r[:6] + (0.4 if i < 6 else 0.4 + (i - 5) * 1.0,) for i, r in enumerate(truck)]   # age 1.4, 2.4, ...
    res = _run(monkeypatch, 13.0, _device_track(12), truck)
    pos = R.positions(res)
    lost_at = _grid(truck[7][0])   # age 2.4 > 2.0
    assert [d["src"] for t, d in pos if t < lost_at][-1] == "car"
    assert all(d["src"] == "device" for t, d in pos if t >= lost_at)
    assert any(d["src"] == "device" for t, d in pos if t >= lost_at), "fell back to nothing"
    ev = _source_events(res)[-1]
    assert (ev["src"], ev["car"]) == ("device", "stale_can")

  def test_wrong_bus_replay_frozen_position_and_climbing_age(self, monkeypatch):
    """What 2026-09-05 actually looked like: lat/lon/hdg held, ts advancing, age climbing."""
    good = _straight(5)
    frozen = [(good[-1][0] + k, good[-1][1], good[-1][2], good[-1][3], good[-1][4], 0.4, 0.4 + k) for k in range(1, 9)]
    res = _run(monkeypatch, 14.0, _device_track(14), good + frozen)
    pos = R.positions(res)
    car_after = [t for t, d in pos if d["src"] == "car" and t > _grid(good[-1][0])]
    assert len(car_after) <= 1, f"a frozen feed was written {len(car_after)} times"
    assert pos[-1][1]["src"] == "device"

  def test_content_frozen_at_speed(self, monkeypatch):
    good = _straight(6)
    last = good[-1]
    frozen = [(last[0] + k,) + last[1:] for k in range(1, 6)]   # identical lat/lon/hdg/spd, fresh age
    res = _run(monkeypatch, 12.0, _device_track(12), good + frozen)
    pos = R.positions(res)
    # one identical publish is normal (decimation); the second identical one (3 in a row) is frozen
    assert [d["src"] for t, d in pos if t == pytest.approx(_grid(frozen[0][0]))] == ["car"]
    assert all(d["src"] == "device" for t, d in pos if t >= _grid(frozen[1][0]))
    assert _source_events(res)[-1]["car"] == "frozen"

  def test_content_frozen_at_zero_mph_while_the_device_moves(self, monkeypatch):
    good = _straight(6)
    last = good[-1]
    frozen = [(last[0] + k, last[1], last[2], last[3], 0.0, 0.4, 0.4) for k in range(1, 6)]
    res = _run(monkeypatch, 12.0, _device_track(12, speed=25.0), good + frozen)
    assert R.positions(res)[-1][1]["src"] == "device"
    assert _source_events(res)[-1]["car"] == "frozen"

  def test_the_publisher_dies(self, monkeypatch):
    truck = _straight(6)
    res = _run(monkeypatch, 14.0, _device_track(14), truck)
    pos = R.positions(res)
    dead_at = truck[-1][0] + M.CAR_GPS_SILENT_S
    assert all(d["src"] == "car" for t, d in pos if _grid(truck[2][0]) <= t <= truck[-1][0] + DT)
    assert all(d["src"] == "device" for t, d in pos if t > dead_at)
    assert any(d["src"] == "device" for t, d in pos if t > dead_at)
    assert _source_events(res)[-1]["car"] == "silent"

  @pytest.mark.parametrize("bad", [dict(lat=166.07), dict(hdg=655.35), dict(spd=255.0), dict(lon=float("nan"))])
  def test_dbc_sentinels_are_not_a_fix(self, monkeypatch, bad):
    truck = _straight(8)
    i = 5
    row = dict(zip(("t", "lat", "lon", "hdg", "spd", "hdop", "age"), truck[i], strict=True)) | bad
    truck[i] = tuple(row.values())
    res = _run(monkeypatch, 9.0, _device_track(9), truck)
    pos = R.positions(res)
    assert not [d for t, d in pos if d["src"] == "car" and t == pytest.approx(_grid(truck[i][0]))]
    assert _source_events(res)[-1]["car"] in ("invalid", "reacquiring")
    assert any(e["car"] == "invalid" for e in _source_events(res))

  def test_reselect_needs_three_healthy_publishes_no_flapping(self, monkeypatch):
    truck = _straight(14)
    truck[6] = truck[6][:6] + (2.5,)    # one stale publish
    res = _run(monkeypatch, 15.0, _device_track(15), truck)
    pos = R.positions(res)
    back = [t for t, d in pos if d["src"] == "car" and t > _grid(truck[6][0])]
    assert back and back[0] == pytest.approx(_grid(truck[9][0])), "re-selected before 3 healthy publishes"
    assert any(d["src"] == "device" for t, d in pos if _grid(truck[6][0]) <= t < back[0])

  def test_parked_truck_does_not_read_as_frozen_and_keeps_its_heading(self, monkeypatch):
    """Parked and charging: identical truck position for a minute, device speed noise up to 4.1 m/s
    and a bearing wandering all over the compass (22 of 46 stops wandered >10 deg this weekend)."""
    from openpilot.system.location_services import location_servicesd as ls
    truck = [(i + 0.3, 47.6, -122.3, 264.9, 0.0, 0.4, 0.3) for i in range(60)]
    noise = [0.3, 4.1, 1.2, 3.9, 0.1, 2.2]
    device = [(i + 0.6, 47.60001, -122.30001, noise[i % 6], (i * 67) % 360, True) for i in range(60)]
    brgs = []
    res = _run(monkeypatch, 61.0, device, truck, on_step=lambda t, s: brgs.append((t, ls._read_mem(_Store(s))[2])))
    pos = R.positions(res)
    assert all(d["src"] == "car" for t, d in pos if t > 3.0)
    assert {b for t, b in brgs if t > 3.0} == {264.9}
    assert not any(e["car"] == "frozen" for e in _source_events(res))


# --- the Tesla: byte-identical --------------------------------------------------------------------

class TestNoCapabilityIsByteIdentical:
  def _scenario(self, monkeypatch, cp, with_cargps):
    device = [(t, la, lo, spd, brg, fix) for t, la, lo, spd, brg, fix, _v in COLD_START] + \
             [(r[0] + 20.0,) + r[1:] for r in TUNNEL_DEVICE]
    truck = COLD_TRUCK + [(r[0] + 20.0,) + r[1:] for r in TUNNEL_TRUCK] if with_cargps else ()
    return _run(monkeypatch, 52.0, device, truck, cp=cp, t_shift=17.0)

  def test_tesla_with_a_stray_cargps_writes_exactly_what_no_cargps_writes(self, monkeypatch):
    tesla = self._scenario(monkeypatch, TESLA_CP, True)
    bare = self._scenario(monkeypatch, None, False)
    assert tesla.mem.writes == bare.mem.writes
    assert R.positions(tesla), "nothing was written -- the comparison proves nothing"
    assert "CarGps" not in tesla.mem.gets, "CarGps was read on a car without the capability"
    assert not _source_events(tesla)

  def test_lightning_without_cargps_writes_the_device_stream_unchanged(self, monkeypatch):
    lightning = self._scenario(monkeypatch, LIGHTNING_CP, False)
    tesla = self._scenario(monkeypatch, TESLA_CP, False)
    assert R.positions(lightning) == R.positions(tesla)


class TestCapability:
  def test_pnw_vehicle(self):
    from cereal import messaging
    assert PnwVehicle(messaging.log_from_bytes(LIGHTNING_CP, car.CarParams)).car_gps is True
    assert PnwVehicle(messaging.log_from_bytes(TESLA_CP, car.CarParams)).car_gps is False
    assert PnwVehicle(None).car_gps is False

  def test_carparams_appearing_mid_run_enables_it_and_is_logged(self, monkeypatch):
    """Offroad -> onroad: CarParams is absent until card fingerprints, then written once."""
    res = _run(monkeypatch, 20.0, _device_track(20), _straight(20), cp=None,
               params_script={6.0: {"CarParams": LIGHTNING_CP}})
    caps = [kw for name, kw in res.log.events if name == "mapd_configd_car_gps_capability"]
    assert caps == [{"capable": True}]
    pos = R.positions(res)
    assert all(d["src"] == "device" for t, d in pos if t < 6.0)
    first_car = min(t for t, d in pos if d["src"] == "car")
    assert 6.0 < first_car <= 6.0 + M.CAR_CAPABILITY_RECHECK_S + 3.0 + DT

  def test_losing_and_regaining_the_capability_starts_the_feed_judgment_over(self, monkeypatch):
    """CarParams is cleared at the onroad transition and rewritten by card. Losing the capability
    must hand the blob back to the device at once, and regaining it must earn the car back with
    fresh healthy publishes, not reuse the count from before."""
    truck = _straight(30)
    res = _run(monkeypatch, 30.0, _device_track(30), truck,
               params_script={10.0: {"CarParams": None}, 20.0: {"CarParams": LIGHTNING_CP}})
    caps = [kw["capable"] for name, kw in res.log.events if name == "mapd_configd_car_gps_capability"]
    assert caps == [True, False, True]
    pos = R.positions(res)
    off_at = min(t for t, d in pos if t > 10.0 and d["src"] == "device")
    assert off_at <= 10.0 + M.CAR_CAPABILITY_RECHECK_S + 1.0
    assert not [t for t, d in pos if off_at <= t < 20.0 and d["src"] == "car"]
    # The recheck at t=20.0 sees CarParams again and starts a FRESH judgment. The publish already in
    # the store (19.3) is its first observation, so it counts once; a dead publisher would still read
    # `silent` 2.5 s later, before a third publish could ever arrive.
    on_at = min(t for t, d in pos if t >= 20.0 and d["src"] == "car")
    after = [r[0] for r in truck if r[0] > 20.0 - 1.0]
    assert on_at == pytest.approx(_grid(after[M.CAR_GPS_REACQUIRE_PUBLISHES - 1])), "re-selected on stale healthy counts"

  def test_unreadable_carparams_is_logged_and_not_capable(self, monkeypatch):
    res = _run(monkeypatch, 6.0, _device_track(6), _straight(6), cp=b"\x00garbage")
    assert all(d["src"] == "device" for _, d in R.positions(res))
    assert any(name == "line" and "CarParams unreadable" in kw["msg"] for name, kw in res.log.events)


def test_car_blob_shape(monkeypatch):
  truck = [(i + 0.3, 47.6 + i * 1e-4, -122.3, 360.0, 45.0, 0.4, 0.4) for i in range(4)]
  res = _run(monkeypatch, 5.0, (), truck)
  t, raw = next((t, v) for t, k, v in res.mem.writes if k == "LastGPSPosition")
  d = json.loads(raw)
  assert list(d) == ["latitude", "longitude", "bearing", "speed", "src", "ts", "fix_ts"]
  assert d["src"] == "car" and d["bearing"] == 0.0 and d["speed"] == pytest.approx(45 * 0.44704)
  assert d["ts"] == pytest.approx(t)
  assert d["fix_ts"] == pytest.approx(t - 0.4)   # gpslag2pnw: the CAN receipt (publish-time age 0.4 s)
