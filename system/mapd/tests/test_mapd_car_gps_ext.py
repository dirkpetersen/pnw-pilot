"""mapdcargps2pnw: the fix mapd_configd SELECTS is RELAYED to MAPD on `gpsLocationExternal`.

Why it matters: mapd's own cereal/gps.go polls gpsLocationExternal FIRST and latches to it permanently
on the first successful read, so a single published message makes this process mapd's only position
source for the life of the process -- and mapd runs offroad too, so that life ends at a device reboot,
not at ignition-off. It also never checks `hasFix`.

So the contract under test is narrow and strict: **the relay publishes exactly what the two
LastGPSPosition write sites just chose, at exactly the moments they chose it, and nothing else.** The
device branch copies its message verbatim; the car branch maps CarGps. Anything that gates the relay
but not the selection would put mapd and the Python consumers back in different coordinate frames,
which is the split this feature exists to close.

Every scenario runs the REAL mapd_configd.main() loop (configd_replay.py) at mapdOut's 20 Hz, with the
CarGps mem-param published the way card publishes it (~1 Hz) and the real selection reading it.
"""
import math

import pytest

import cereal.messaging as messaging

from openpilot.system.mapd import mapd_configd as M
from openpilot.system.mapd.tests import configd_replay as R
from openpilot.system.mapd.tests.test_gps_fix_gate import COLD_START
from openpilot.system.mapd.tests.test_gps_source_select import DT, LIGHTNING_CP, TESLA_CP, TUNNEL_DEVICE, \
  TUNNEL_TRUCK, _cargps, _device_track, _grid, _source_events, _straight

EXT = 'gpsLocationExternal'


def _run(monkeypatch, t_end, device=(), truck=(), use_car_gps=True, cp=LIGHTNING_CP, flags=None,
         device_fields=None, persistent=None, params_script=None):
  """device rows: (t, lat, lon, speed, bearing, hasFix); truck rows: (t, lat, lon, hdg, spd MPH, hdop, age).
  `flags(i)` adds extra CarGps keys to truck publish i; `device_fields(i)` adds extra GpsLocationData
  fields to device message i (how the verbatim-copy test gives every field a distinctive value)."""
  steps = {round(k * DT, 3): [("mapdOut", {})] for k in range(int(round(t_end / DT)) + 1)}
  for i, (t, lat, lon, spd, brg, fix) in enumerate(device):
    if 0.0 <= t <= t_end:
      svc, fields = R.gps_msg(lat, lon, has_fix=fix, speed=spd, bearing=brg)
      steps[_grid(t)].append((svc, {**fields, **(device_fields(i) if device_fields else {})}))
  mem_script = {}
  for i, (t, lat, lon, hdg, spd, hdop, age) in enumerate(truck):
    if 0.0 <= t <= t_end:
      blob = _cargps(t, lat, lon, hdg, spd, hdop, age)
      if flags is not None:
        blob = {**blob, **flags(i)}
      mem_script.setdefault(_grid(t), {})["CarGps"] = blob
  p = {"MapdUseCarGps": use_car_gps}
  if cp is not None:
    p["CarParams"] = cp
  p.update(persistent or {})
  return R.run(monkeypatch, sorted(steps.items()), persistent=p, mem_script=mem_script,
               params_script=params_script)


def _ext(res):
  """Every gpsLocationExternal publish as (t, the message body)."""
  return [(t, m.gpsLocationExternal) for t, s, m in res.sent if s == EXT]


def _car_ext(res):
  return [g for _t, g in _ext(res) if str(g.source) == "car"]


def _starts(res):
  return [kw for name, kw in res.log.events if name == "mapd_cargps_ext_start"]


def _stops(res):
  return [kw for name, kw in res.log.events if name == "mapd_cargps_ext_stop"]


def _lines(res):
  return [kw["msg"] for name, kw in res.log.events if name == "line"]


# --- the relay contract: one publish per selected position, and only then ---------------------------

class TestItRelaysTheSelection:
  def test_every_selected_position_is_relayed_and_nothing_else(self, monkeypatch):
    """The centrepiece. The SR 99 tunnel replay switches source twice (device while the car feed is
    still reacquiring, then car), so this run exercises both branches -- and the publish times must be
    the LastGPSPosition write times exactly: no extra publish, no missing one, no re-publish at the
    loop's 20 Hz. Everything else in this file is a detail of that one invariant."""
    res = _run(monkeypatch, 31.0, TUNNEL_DEVICE, TUNNEL_TRUCK)
    pos = R.positions(res)
    assert len({d["src"] for _t, d in pos}) == 2, "the fixture never switched source; the test is weaker than it reads"
    assert [t for t, _ in _ext(res)] == [t for t, _ in pos]
    for (_t, g), (_pt, d) in zip(_ext(res), pos, strict=True):
      # the relayed position IS the written position, on both branches
      assert (g.latitude, g.longitude) == (d["latitude"], d["longitude"])
      assert g.bearingDeg == pytest.approx(d["bearing"], rel=1e-6)
      assert g.speed == pytest.approx(d["speed"], rel=1e-6)
      assert (str(g.source) == "car") == (d["src"] == "car")

  def test_the_handover_from_car_to_device_never_stalls(self, monkeypatch):
    """v1 published only the car fix, so an ordinary truck-feed outage stalled mapd. Under the relay
    the device fix takes over within its own publish interval and mapd never notices."""
    res = _run(monkeypatch, 20.0, _device_track(20), _straight(6))
    ts = [t for t, _ in _ext(res)]
    assert max(b - a for a, b in zip(ts, ts[1:], strict=False)) < M.EXT_STALL_S
    assert [str(g.source) == "car" for _t, g in _ext(res)][-1] is False, "the device never took over"
    assert len(_starts(res)) == 1 and _stops(res) == []

  def test_a_dead_reckoning_truck_is_relayed_because_the_selection_kept_it(self, monkeypatch):
    """v1 had a `dr` gate of its own (the truck's 0x463 GPS_Actual_vs_Infer_pos flag). It is GONE:
    under the relay an extra gate the Python side does not have re-creates the split-brain this
    feature removes, and the owner's "a dead-reckoned truck fix still beats no position" preference is
    already encoded inside CarGpsSource. The blob keeps the flag as telemetry either way."""
    res = _run(monkeypatch, 20.0, _device_track(20), _straight(20), flags=lambda i: {"dr": 1, "drAge": 0.3})
    assert [d["src"] for _t, d in R.positions(res)][-1] == "car"
    assert _car_ext(res), "the selection kept the truck but the relay dropped it"
    assert _stops(res) == []

  def test_a_degraded_truck_yielding_to_the_device_keeps_the_relay_going(self, monkeypatch):
    """gpsdr2pnw: HDOP >= 3.8 for 10 s while moving, with a device fix steady for 10 s, hands
    LastGPSPosition back to the device. The relay must FOLLOW that switch, not stop at it."""
    truck = [r[:5] + (3.8,) + r[6:] if i >= 10 else r for i, r in enumerate(_straight(40))]
    res = _run(monkeypatch, 40.0, _device_track(40), truck)
    yielded = min(t for t, d in R.positions(res) if t > 20.0 and d["src"] == "device")
    assert [e["src"] for e in _source_events(res)][-1] == "device", "the fixture never yielded"
    after = [(t, g) for t, g in _ext(res) if t >= yielded]
    assert after and not any(str(g.source) == "car" for _t, g in after)
    assert _stops(res) == [] and len(_starts(res)) == 1


# --- the gate: the param and the capability, and nothing else ---------------------------------------

class TestTheGate:
  def test_off_publishes_nothing_and_does_not_even_own_the_queue(self, monkeypatch):
    """The default. The scenario is the one that DOES publish when the param is on, so an empty result
    here is the gate and not an inert fixture. `pubs` matters too: a publisher in main()'s PubMaster
    would take the gpsLocationExternal msgq queue at boot even with the feature off, which would make
    "the default is inert" true of the message count but not of the queue."""
    res = _run(monkeypatch, 31.0, TUNNEL_DEVICE, TUNNEL_TRUCK, use_car_gps=False)
    assert _ext(res) == []
    assert not _starts(res) and not _stops(res)
    assert res.pubs == [["mapdIn"]]
    assert [e["src"] for e in _source_events(res)][-1] == "car", "the feed was not healthy; the gate is unproven"

  def test_on_creates_exactly_one_publisher_when_it_first_relays(self, monkeypatch):
    res = _run(monkeypatch, 31.0, TUNNEL_DEVICE, TUNNEL_TRUCK)
    assert res.pubs == [["mapdIn"], [EXT]], "the relay publisher is lazy and created once"
    assert _ext(res)

  def test_on_and_off_write_identical_mem_params(self, monkeypatch):
    """The feature must be invisible to every EXISTING consumer: it only adds a msgq publish."""
    on = _run(monkeypatch, 31.0, TUNNEL_DEVICE, TUNNEL_TRUCK, use_car_gps=True)
    off = _run(monkeypatch, 31.0, TUNNEL_DEVICE, TUNNEL_TRUCK, use_car_gps=False)
    assert on.mem.writes == off.mem.writes
    assert _ext(on) and not _ext(off), "the comparison proves nothing if neither published"

  def test_turning_it_off_mid_drive_stops_and_says_so(self, monkeypatch):
    res = _run(monkeypatch, 20.0, _device_track(20), _straight(20),
               params_script={10.0: {"MapdUseCarGps": False}})
    assert max(t for t, _ in _ext(res)) < 10.0 + DT
    assert len(_stops(res)) == 1 and _stops(res)[0]["reason"] == "MapdUseCarGps off"
    assert "stalled on its last position" in _stops(res)[0]["note"]

  def test_turning_it_on_mid_drive_starts_relaying(self, monkeypatch):
    res = _run(monkeypatch, 20.0, _device_track(20), _straight(20), use_car_gps=False,
               params_script={10.0: {"MapdUseCarGps": True}})
    assert not [t for t, _ in _ext(res) if t < 10.0]
    assert [t for t, _ in _ext(res) if t >= 10.0]
    assert len(_starts(res)) == 1

  def test_the_tesla_never_relays_and_never_reads_a_thing(self, monkeypatch):
    """Capability, never a fingerprint: without PnwVehicle.car_gps neither CarGps nor the param is even
    read, so the Raven cannot be affected by this feature at all and mapd keeps its own subscription."""
    res = _run(monkeypatch, 31.0, TUNNEL_DEVICE, TUNNEL_TRUCK, cp=TESLA_CP)
    assert _ext(res) == [] and res.pubs == [["mapdIn"]]
    assert "CarGps" not in res.mem.gets
    assert "MapdUseCarGps" not in res.params.gets
    assert not _starts(res) and not _stops(res)
    # ... while the device fix itself was written the whole time, so the run was not simply empty
    assert {d["src"] for _t, d in R.positions(res)} == {"device"}

  def test_no_carparams_at_all_never_relays(self, monkeypatch):
    """Offroad, before card has fingerprinted: absent CarParams is not capable."""
    res = _run(monkeypatch, 10.0, _device_track(10), _straight(10), cp=None)
    assert _ext(res) == [] and R.positions(res), "nothing was selected either; the gate is unproven"

  def test_losing_the_capability_mid_drive_stops_and_names_it(self, monkeypatch):
    """CarParams is cleared at every onroad transition; a device moved to the other car must stop."""
    res = _run(monkeypatch, 20.0, _device_track(20), _straight(20),
               params_script={10.0: {"CarParams": None}})
    assert max(t for t, _ in _ext(res)) <= 10.0 + M.CAR_CAPABILITY_RECHECK_S + 1.0
    assert len(_stops(res)) == 1 and "capability is gone" in _stops(res)[0]["reason"]


# --- the device branch: verbatim, and hasFix-gated ---------------------------------------------------

class TestTheDeviceBranch:
  # every field of GpsLocationData given a distinctive value, so a re-derived copy cannot pass
  RICH = [{"source": "qcomdiag", "horizontalAccuracy": 3.5 + i, "satelliteCount": 9 + i, "altitude": 61.0 + i,
           "flags": 3, "vNED": [1.0 + i, -2.0, 0.5], "verticalAccuracy": 7.25, "bearingAccuracyDeg": 12.5,
           "speedAccuracy": 0.375, "unixTimestampMillis": 1789000000000 + i} for i in range(12)]

  def test_the_device_fix_is_copied_verbatim(self, monkeypatch):
    """Not re-derived. `horizontalAccuracy` alone would change which road mapd matches (it is mapd's
    way-matching tolerance, maps/way.go:361), and a substituted `source` would misreport the receiver
    into the rlog. No truck feed here, so the selection is the device on every message."""
    device = _device_track(12)
    res = _run(monkeypatch, 13.0, device, device_fields=lambda i: self.RICH[i])
    ext = _ext(res)
    assert len(ext) == len(device)
    for (_t, g), (_dt, lat, lon, spd, brg, fix), rich in zip(ext, device, self.RICH, strict=True):
      want = messaging.new_message('gpsLocation')
      for k, v in {**R.gps_msg(lat, lon, has_fix=fix, speed=spd, bearing=brg)[1], **rich}.items():
        setattr(want.gpsLocation, k, v)
      assert g.to_dict() == want.gpsLocation.to_dict()

  def test_a_no_fix_device_tick_relays_nothing(self, monkeypatch):
    """The gate mapd does not have. At the 2026-09-12 06:28:57 PT cold start qcomgpsd published four
    hasFix=False messages 2.18-2.42 km from the truck; relaying a gpsfix2pnw-gated fix keeps them out
    of mapd without forking it (gpsfix2pnw, test_gps_fix_gate.py)."""
    device = [(r[0] + 1.0, r[1], r[2], r[3], r[4], r[5]) for r in COLD_START]
    res = _run(monkeypatch, 17.0, device)
    nofix = {(r[1], r[2]) for r in device if not r[5]}
    assert nofix and len(_ext(res)) == sum(1 for r in device if r[5])
    assert all(g.hasFix for _t, g in _ext(res))
    assert not [1 for _t, g in _ext(res) if (g.latitude, g.longitude) in nofix]

  def test_the_device_builder_directly(self):
    """The copy without the loop, so a field change shows up as a unit failure too."""
    src = messaging.new_message('gpsLocation')
    src.gpsLocation.latitude, src.gpsLocation.longitude = 47.6062, -122.3321
    src.gpsLocation.horizontalAccuracy, src.gpsLocation.satelliteCount = 3.5, 11
    src.gpsLocation.source, src.gpsLocation.hasFix = 'qcomdiag', True
    rd = messaging.log_from_bytes(src.to_bytes()).gpsLocation
    assert M.device_gps_ext_msg(rd).gpsLocationExternal.to_dict() == rd.to_dict()


# --- the car branch: the mapping, including the two fields that must stay 0 --------------------------

class TestTheCarBranch:
  def test_the_tunnel_replay_maps_every_selected_car_fix(self, monkeypatch):
    res = _run(monkeypatch, 31.0, TUNNEL_DEVICE, TUNNEL_TRUCK)
    rows = [r for r in TUNNEL_TRUCK if r[0] >= 2.81]   # the car is selected on its 3rd healthy publish
    ext = [(t, g) for t, g in _ext(res) if str(g.source) == "car"]
    assert len(ext) == len(rows), "not one publish per selected CarGps publish"
    for (_t, g), (_rt, lat, lon, hdg, spd, _hdop, _age) in zip(ext, rows, strict=True):
      assert (g.latitude, g.longitude) == (lat, lon)
      assert g.bearingDeg == pytest.approx(hdg)
      assert g.speed == pytest.approx(spd * M.MPH_TO_MS)
      assert g.hasFix is True

  def test_speed_is_converted_from_MPH_to_m_per_s(self, monkeypatch):
    """CarGps `spd` is MPH per the DBC; GpsLocationData.speed is m/s. This is the single easiest bug to
    ship, and at 56 mph the difference is 56 vs 25 -- a 2.24x error straight into the rlog."""
    res = _run(monkeypatch, 10.0, _device_track(10), _straight(10, spd=56.0))
    speeds = {round(g.speed, 4) for g in _car_ext(res)}
    assert _car_ext(res) and speeds == {round(56.0 * 0.44704, 4)} == {25.0342}
    assert 56.0 not in speeds, "the MPH value was published raw into an m/s field"

  @pytest.mark.parametrize("mph,ms", [(0.0, 0.0), (35.0, 15.6464), (75.0, 33.528)])
  def test_the_conversion_at_several_speeds(self, monkeypatch, mph, ms):
    # the device is stopped too at 0 mph, else a stationary truck next to a moving device reads frozen
    res = _run(monkeypatch, 10.0, _device_track(10, speed=0.0 if mph == 0.0 else 25.0), _straight(10, spd=mph))
    assert _car_ext(res) and all(g.speed == pytest.approx(ms, abs=1e-3) for g in _car_ext(res))

  def test_horizontal_accuracy_is_always_zero_whatever_the_HDOP(self, monkeypatch):
    """THE field that must not be "improved". mapd uses it as its way-matching tolerance --
    `max_dist := max(location.HorizontalAccuracy(), 5) + w.Width()` (pfeiferj mapd maps/way.go:361) --
    so the plausible-looking `hdop * 5` of the first draft would have widened matching by 6-18 m at
    HDOP 1.2-3.6, making adjacent-ramp mis-matching MORE likely. That is the exact phantom-slowdown
    class this work exists to fight. qcomgpsd publishes 0 here as well."""
    res = _run(monkeypatch, 13.0, _device_track(13), _straight(13), flags=lambda i: {"hdop": 0.4 + i * 0.25})
    assert len(_car_ext(res)) >= 8
    assert {g.horizontalAccuracy for g in _car_ext(res)} == {0.0}

  @pytest.mark.parametrize("sats", [31, 30, 29, 12, 0, None, "9", True])
  def test_satellite_count_is_always_zero_because_the_truck_only_sends_sentinels(self, monkeypatch, sats):
    """ford_lincoln_base_pt.dbc VAL_ 1124: GPS_Sat_num_in_view is 5 bits, range [0|29], 31 "Invalid",
    30 "Unknown". Across every ces_events log under drives/ the truck sent 31 on 83,084 publishes, 30
    on 133, and a real count on none -- the "31 satellites" that motivated this feature is a sentinel.
    0 is capnp's "not reported"; anything else would put a fiction in the rlog."""
    res = _run(monkeypatch, 8.0, _device_track(8), _straight(8), flags=lambda i: {"sats": sats})
    assert _car_ext(res) and {g.satelliteCount for g in _car_ext(res)} == {0}

  def test_the_timestamp_is_the_fix_epoch_not_the_publish_time(self, monkeypatch):
    """CarGps `ts` is the publisher's wall clock in SECONDS when it wrote the blob; the CAN frame it
    wrote was already `age` old by then (0.0-1.03 s measured). The cereal field is milliseconds."""
    truck = _straight(8, age=0.7)
    res = _run(monkeypatch, 9.0, _device_track(8), truck)
    published = [g.unixTimestampMillis for g in _car_ext(res)]
    assert published == [int((_cargps(*r)["ts"] - 0.7) * 1000.0) for r in truck[2:]]
    assert published != [int(_cargps(*r)["ts"] * 1000.0) for r in truck[2:]], "the CAN frame age was ignored"

  @pytest.mark.parametrize("ts", [float("nan"), float("inf"), -1.0, 1e18])
  def test_an_absurd_timestamp_cannot_crash_the_daemon(self, monkeypatch, ts):
    """CarGpsSource compares `ts` for change but never range-checks it, and NaN != NaN reads as a new
    publish every loop -- so a bad clock reaches the builder. It must be dropped, not raised on."""
    res = _run(monkeypatch, 8.0, _device_track(8), _straight(8), flags=lambda i: {"ts": ts})
    assert not any("bridge write failed" in m for m in _lines(res)), _lines(res)
    assert all(g.unixTimestampMillis == 0 for g in _car_ext(res))

  def test_fields_this_feed_does_not_carry_stay_at_their_defaults(self, monkeypatch):
    res = _run(monkeypatch, 8.0, _device_track(8), _straight(8))
    for g in _car_ext(res):
      assert (g.flags, g.altitude, list(g.vNED)) == (0, 0.0, [])
      assert (g.verticalAccuracy, g.bearingAccuracyDeg, g.speedAccuracy) == (0.0, 0.0, 0.0)

  def test_the_car_builder_directly(self):
    """The mapping without the loop, so a field change shows up as a unit failure too."""
    cg = _cargps(1.0, 47.6, -122.3, 90.0, 56.0, hdop=3.6, age=0.35)
    fix = {"latitude": cg["lat"], "longitude": cg["lon"], "bearing": cg["hdg"], "speed": cg["spd"] * M.MPH_TO_MS}
    g = M.car_gps_ext_msg(cg, fix).gpsLocationExternal
    assert (g.latitude, g.longitude) == (47.6, -122.3)
    assert g.bearingDeg == pytest.approx(90.0)
    assert g.speed == pytest.approx(56.0 * 0.44704, rel=1e-6)   # Float32, so not bit-exact
    assert g.hasFix and str(g.source) == "car"
    assert (g.horizontalAccuracy, g.satelliteCount) == (0.0, 0)
    assert g.unixTimestampMillis == int((cg["ts"] - 0.35) * 1000.0)


# --- Rule 2: it says what it is doing, once ---------------------------------------------------------

class TestLoggingIsChangeOnly:
  def test_a_long_healthy_run_logs_exactly_one_start_and_no_stop(self, monkeypatch):
    """The loop runs at 20 Hz and relays at ~1 Hz: a per-tick line would be ~600 events, a per-publish
    one ~30. Exactly one is the contract."""
    res = _run(monkeypatch, 30.0, _device_track(30), _straight(30))
    assert len(_ext(res)) == 30
    assert len(_starts(res)) == 1 and _stops(res) == []
    assert _starts(res)[0]["src"] == "device"   # the device carries it until the car feed is trusted

  def test_a_total_outage_logs_one_stall_line_not_one_per_tick(self, monkeypatch):
    """BOTH receivers dead is now the only routine way the relay stops, and it must not be silent: a
    starved mapd keeps publishing mapdOut at 20 Hz with frozen data and tileLoaded=true, so nothing
    downstream can tell. Here both feeds end at ~5.5 s and the replay runs 14 s further."""
    res = _run(monkeypatch, 20.0, _device_track(6), _straight(6))
    last = max(t for t, _ in _ext(res))
    assert len(_stops(res)) == 1
    assert "neither receiver" in _stops(res)[0]["reason"]
    assert "stalled on its last position" in _stops(res)[0]["note"]
    assert last + M.EXT_STALL_S < 20.0, "the replay ended before the stall could be declared"

  def test_the_relay_resumes_with_a_fresh_start_line(self, monkeypatch):
    """Both feeds die at ~5.5 s, the truck comes back at 15.3 s: one stop, two starts, nothing else."""
    res = _run(monkeypatch, 22.0, _device_track(6), _straight(6) + _straight(6, t0=15.0))
    assert (len(_starts(res)), len(_stops(res))) == (2, 1)
    assert [t for t, _ in _ext(res) if t > 15.0]


# --- the failure paths this must never take silently ------------------------------------------------

class TestItNeverFailsSilently:
  def test_a_publish_failure_is_logged_once_and_does_not_break_the_CES_bridge(self, monkeypatch):
    """The realistic cause is MultiplePublishersError: ubloxd nominally owns this service and msgq
    hands the queue to whichever process connected last. A failure must not abort the mapd->CES bridge
    that follows it in the same try, and must not spam at 1 Hz."""
    def boom(cg, fix):
      raise RuntimeError("MultiplePublishersError")
    monkeypatch.setattr(M, "car_gps_ext_msg", boom)
    res = _run(monkeypatch, 20.0, (), _straight(20))
    failures = [m for m in _lines(res) if "gpsLocationExternal publish FAILED" in m]
    assert len(failures) == 1, f"{len(failures)} lines for one failure run"
    assert _ext(res) == [] and not _starts(res)
    # the bridge below it kept running, and so did the rest of the loop
    assert any(k == "MapSpeedLimit" for _t, k, _v in res.mem.writes)
    assert [d["src"] for _t, d in R.positions(res)][-1] == "car"
    assert not any("bridge write failed" in m for m in _lines(res))

  def test_a_ublox_device_refuses_to_relay_rather_than_feed_itself(self, monkeypatch):
    """With UbloxAvailable the DEVICE fix arrives on gpsLocationExternal too, so relaying there would
    make this process read its own publishes back as the device fix and compare the truck against
    itself. Not the case on the 3X (no /dev/ttyHS0), which is exactly why it must fail loudly if it
    ever is. ubloxd/pigeond are not even running here (managerState 0/120 each)."""
    res = _run(monkeypatch, 20.0, (), _straight(20), persistent={"UbloxAvailable": True})
    assert _ext(res) == [] and res.pubs == [["mapdIn"]]
    assert not _starts(res) and not _stops(res)
    assert sum(1 for m in _lines(res) if "relay is DISABLED" in m) == 1
    assert [e["src"] for e in _source_events(res)][-1] == "car", "the feed was not healthy; the refusal is unproven"

  def test_the_daemon_survives_a_garbage_CarGps_blob(self, monkeypatch):
    """mapd_configd must never raise: a crash here now costs mapd its only GPS source for the boot."""
    res = _run(monkeypatch, 12.0, _device_track(12), _straight(12),
               flags=lambda i: {"hdop": float("nan"), "lat": None} if i >= 6 else {})
    assert not any("bridge write failed" in m for m in _lines(res))
    assert math.isfinite(R.positions(res)[-1][1]["latitude"])
    assert _ext(res), "nothing was relayed at all; the survival assertion is vacuous"

  def test_mapd_configd_is_restarted_if_it_crashes(self):
    """The new single point of failure. Once mapd has latched onto gpsLocationExternal it never reads
    gpsLocation again for the life of the process, so a dead mapd_configd starves mapd for the rest of
    the BOOT (mapd runs offroad too) -- while mapdOut keeps publishing frozen data at 20 Hz, so nothing
    downstream notices. Before this feature a crash left mapd on its own subscription, untouched."""
    from openpilot.system.manager.process_config import procs
    proc = next(p for p in procs if p.name == "mapd_configd")
    assert proc.restart_if_crash is True
