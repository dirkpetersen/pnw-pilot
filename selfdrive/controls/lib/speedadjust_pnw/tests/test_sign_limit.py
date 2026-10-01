"""fordtsr2pnw -- the Lightning camera's traffic-sign speed limit weighed against the map in speedadjust.

Owner 2026-09-27: "we still want a Ford toggle for this, enabled by default but can be disabled, and it should always be
enabled if there is no map." The cases below are the measured ones (drives/2026-09-24/ford-tsr-measure/DRIVE_REPORT.md):
the ODOT work-zone 55s, 19:02 (mapd's false 25 while the camera held 45), the SR 99 tunnel (camera stale 50, posted 45),
18:57 (camera 40 7.2 s before mapd, with the look-ahead running), and the 1.6 s misread of 19:07:24.

Everything drives the REAL SpeedAdjustController through cap() on a fake clock, with the map limit and announcement in
the mem store exactly as mapd_configd writes them (next_limit_payload) and the camera in sm['carState'].cruiseState.
"""
import json
import types

import pytest

import openpilot.selfdrive.controls.lib.speedadjust_pnw.speedadjust_controller as sa
import openpilot.selfdrive.controls.lib.speedadjust_pnw.sign_limit as sl_mod
from openpilot.selfdrive.controls.lib.speedadjust_pnw.sign_limit import SignLimitSelector, SIGN_CONFIRM_S, SIGN_RAISE_S
from openpilot.selfdrive.controls.lib.pnw_vehicle import PnwVehicle
from openpilot.system.mapd.mapd_configd import next_limit_payload

MPH = sa.MPH_TO_MS
DT = 0.05                     # plannerd's 20 Hz


class _Clock:
  t = 1000.0

  @classmethod
  def monotonic(cls):
    return cls.t

  @classmethod
  def time(cls):
    return cls.t


SEATTLE = (47.60, -122.33)
VANCOUVER_BC = (49.28, -123.12)
PORTLAND = (45.52, -122.68)


class _Mem:
  def __init__(self):
    self.map_mph, self.nxt, self.mapd_alive, self.police, self.calls = 0.0, None, True, None, []
    self.pos, self.pos_age = SEATTLE, 0.5      # LastGPSPosition; pos None = never written, "bad" = garbage

  def get(self, k, return_default=True):
    if k == "MapSpeedLimit":
      return str(self.map_mph * MPH) if self.map_mph else ""
    if k == "NextMapSpeedLimit":
      n, d = self.nxt if self.nxt else (0.0, 0.0)
      return next_limit_payload(n * MPH, d, _Clock.t - (0.0 if self.mapd_alive else 30.0))
    if k == "LastGPSPosition":
      if self.pos is None:
        return None
      if self.pos == "bad":
        return "{not json"
      return json.dumps({"latitude": self.pos[0], "longitude": self.pos[1], "src": "device", "ts": _Clock.t - self.pos_age})
    if k == "LocationServices":
      return json.dumps({"police": self.police}) if self.police else "{}"
    return None

  def put_nonblocking(self, k, v):
    self.calls.append((k, v))


class _P:
  def __init__(self, mode=2, la_mode=sa.LA_OFF, sign_on=True):
    self.mode, self.la_mode, self.sign_on = mode, la_mode, sign_on

  def get(self, k, return_default=True):
    if k == "DisableFordSignSpeedLimit":
      if isinstance(self.sign_on, Exception):
        raise self.sign_on
      return not self.sign_on          # sign_on = the camera is used = the Disable toggle is OFF
    return {"AutoSpeedReduce": str(self.mode), "LimitAheadMode": self.la_mode}.get(k)


class _SM:
  def __init__(self, set_mph=75.0, with_sign=True):
    cruise = types.SimpleNamespace(speed=set_mph * MPH, enabled=True)
    if with_sign:
      cruise.speedLimitSign, cruise.speedLimitSignStatus = 0.0, "stale"
    self.cs = types.SimpleNamespace(gasPressed=False, brakePressed=False, cruiseState=cruise)

  def cam(self, mph, status="valid"):
    self.cs.cruiseState.speedLimitSign = float(mph) if status == "valid" else 0.0
    self.cs.cruiseState.speedLimitSignStatus = status

  def __getitem__(self, k):
    if k == "carState":
      return self.cs
    raise KeyError(k)


class Rig:
  """One controller, a fake clock and the world it reads. run() steps cap() at 20 Hz."""

  def __init__(self, monkeypatch, sign=True, op_long=True, set_mph=75.0, with_sign=True, **pkw):
    _Clock.t = 1000.0
    monkeypatch.setattr(sa, "time", _Clock)
    self.events, self.errors, self.warnings = [], [], []
    monkeypatch.setattr(sa.cloudlog, "event", lambda name, **kw: self.events.append((name, kw)))
    monkeypatch.setattr(sl_mod.cloudlog, "event", lambda name, **kw: self.events.append((name, kw)))
    for mod in (sa, sl_mod):
      monkeypatch.setattr(mod.cloudlog, "error", lambda msg, *a, **kw: self.errors.append(str(msg)))
      monkeypatch.setattr(mod.cloudlog, "exception", lambda msg, *a, **kw: self.errors.append(str(msg)))
      monkeypatch.setattr(mod.cloudlog, "warning", lambda msg, *a, **kw: self.warnings.append(str(msg)))
    self.p = _P(**pkw)
    self.c = sa.SpeedAdjustController(types.SimpleNamespace(openpilotLongitudinalControl=op_long), params=self.p,
                                      sign_limit=sign)
    self.c.mem_params = self.mem = _Mem()
    self.sm = _SM(set_mph, with_sign)
    self.set = set_mph * MPH
    self.v = set_mph * MPH
    self.trace = []           # (t, working limit mph, selector why, cap out mph)

  def run(self, seconds, map_mph=None, cam=None, cam_status="valid", nxt=None):
    if map_mph is not None:
      self.mem.map_mph = map_mph
    if cam is not None or cam_status != "valid":
      self.sm.cam(cam or 0.0, cam_status)
    self.mem.nxt = nxt
    out = None
    for _ in range(int(round(seconds / DT))):
      _Clock.t += DT
      out = self.c.cap(self.sm, self.set, self.set, self.v, True)
      why = self.c._sign.why if self.c._sign else None
      self.trace.append((_Clock.t - 1000.0, self.c._sl / MPH, why, out / MPH))
    return out

  @property
  def limit(self):
    return round(self.c._sl / MPH, 1)

  def status(self):
    return next(v for k, v in reversed(self.mem.calls) if k == "SpeedAdjustStatus")


# ---------------------------------------------------------------------------------------------------------------------
class TestCapability:
  def test_the_lightning_has_it(self):
    cp = types.SimpleNamespace(carFingerprint="FORD_F_150_LIGHTNING_MK1", brand="ford", openpilotLongitudinalControl=False)
    assert PnwVehicle(cp).camera_speed_limit is True

  @pytest.mark.parametrize("fp, brand", [("TESLA_MODEL_S_HW3", "tesla"), ("FORD_F_150_MK14", "ford"), ("MOCK", "mock")])
  def test_other_cars_do_not(self, fp, brand):
    cp = types.SimpleNamespace(carFingerprint=fp, brand=brand, openpilotLongitudinalControl=False)
    assert PnwVehicle(cp).camera_speed_limit is False

  def test_no_car_yet(self):
    assert PnwVehicle(None).camera_speed_limit is False

  def test_the_planner_passes_the_capability(self):
    """The planner is the one place that constructs speedadjust: it must hand it the capability, not a constant."""
    from pathlib import Path
    src = (Path(sa.__file__).resolve().parents[1] / "longitudinal_planner.py").read_text()
    assert "SpeedAdjustController(CP, sign_limit=self.veh.camera_speed_limit)" in src


class TestMisread:
  @pytest.mark.parametrize("secs", [0.1, 0.5, 1.0, 1.6, 2.0, 2.9])
  @pytest.mark.parametrize("phase", [0.0, 0.3, 0.7])
  def test_a_brief_low_value_never_acts_map_known(self, monkeypatch, secs, phase):
    """19:07:24.6: a 25 for 1.6 s at 52 mph on SR 99 (map and camera 50 around it)."""
    r = Rig(monkeypatch)
    r.run(10.0 + phase, map_mph=50, cam=50)
    r.run(secs, cam=25)
    r.run(10.0, cam=50)
    assert min(lim for _, lim, _, _ in r.trace) == pytest.approx(50, abs=0.1)
    assert min(out for _, _, _, out in r.trace) == pytest.approx(75, abs=0.1), "the cap never moved"

  @pytest.mark.parametrize("secs", [0.5, 1.6, 2.9])
  def test_a_brief_low_value_never_acts_without_a_map(self, monkeypatch, secs):
    r = Rig(monkeypatch)
    r.run(12.0, map_mph=0, cam=50)
    assert r.limit == 50
    r.run(secs, cam=25)
    r.run(10.0, cam=50)
    assert min(lim for t, lim, _, _ in r.trace if t > 12.0) == pytest.approx(50, abs=0.1)


class TestWorkZone:
  def test_camera_55_persisting_overrides_map_70(self, monkeypatch):
    r = Rig(monkeypatch)
    r.run(10.0, map_mph=70, cam=70)
    r.run(SIGN_CONFIRM_S - 0.5, cam=55)
    assert r.limit == 70, "not before the camera value has persisted"
    r.run(4.0)                                        # + the ordinary 2 s drop confirm
    assert r.limit == 55
    assert r.c._sign.why == "cameraOverride" and r.c._sign.src == "camera"
    # rule 1: 75 in a 70 is 7 % over -> the same % over 55 = 58.9 mph (op-long returns the slewed cap)
    r.run(20.0)
    assert r.trace[-1][3] == pytest.approx(55 * 75 / 70, abs=0.1)

  def test_back_to_agree_when_the_zone_ends(self, monkeypatch):
    r = Rig(monkeypatch)
    r.run(5.0, map_mph=65, cam=65)
    r.run(10.0, cam=55)
    assert r.limit == 55
    r.run(10.0, cam=65)
    assert (r.limit, r.c._sign.why) == (65, "agree")


class TestMapGlitch1902:
  def _drive(self, monkeypatch, sign):
    r = Rig(monkeypatch, sign=sign)
    r.run(20.0, map_mph=45, cam=45)
    r.run(5.0, map_mph=25)                            # mapd on the road not taken for 4.95 s
    r.run(20.0, map_mph=50)                           # the SR 99 ramp (camera kept 45)
    return r

  def test_camera_45_wins_over_the_false_25(self, monkeypatch):
    r = self._drive(monkeypatch, sign=True)
    assert min(lim for _, lim, _, _ in r.trace) == pytest.approx(45, abs=0.1)
    assert any(why == "cameraOverride" for t, _, why, _ in r.trace if 20.0 < t < 25.0)

  def test_control_map_only_takes_the_25(self, monkeypatch):
    """The same drive without the capability: the map's 25 is adopted (the 2 s confirm passes at 4.95 s)."""
    r = self._drive(monkeypatch, sign=False)
    assert min(lim for _, lim, _, _ in r.trace) == pytest.approx(25, abs=0.1)


class TestTunnelStaleHigher:
  def test_camera_50_held_against_map_45_never_raises(self, monkeypatch):
    r = Rig(monkeypatch)
    r.run(10.0, map_mph=45, cam=45)
    r.run(143.0, cam=50)                              # 143 s / 3.25 km in the SR 99 tunnel
    assert max(lim for _, lim, _, _ in r.trace) == pytest.approx(45, abs=0.1)
    assert r.c._sign.why == "heldHigher"

  @pytest.mark.parametrize("gap", [3.0, 7.0, 9.0])
  def test_a_short_map_dropout_before_the_tunnel_does_not_let_the_stale_50_in(self, monkeypatch, gap):
    """Found by replaying 2026-09-24: at 19:04:45 PT mapd dropped out for 7 s just before the tunnel portal while the
    camera had shown 50 (vs the map's 40/45) for 65 s. The no-map raise must be timed from when the MAP went unknown,
    not from when the camera first showed 50 -- else the raise passes at once and the hold keeps 50 in the tunnel."""
    r = Rig(monkeypatch)
    r.run(10.0, map_mph=45, cam=45)
    r.run(65.0, cam=50)
    r.mem.mapd_alive = False
    r.run(gap)
    r.mem.mapd_alive = True
    r.run(143.0)
    assert max(lim for _, lim, _, _ in r.trace) == pytest.approx(45, abs=0.1)

  def test_the_no_map_raise_time_must_be_continuous(self, monkeypatch):
    """Two 7 s dropouts with the map back in between are not one 14 s dropout."""
    r = Rig(monkeypatch)
    r.run(10.0, map_mph=45, cam=45)
    r.run(5.0, cam=50)
    for _ in range(2):
      r.mem.mapd_alive = False
      r.run(7.0)
      r.mem.mapd_alive = True
      r.run(5.0)
    assert max(lim for _, lim, _, _ in r.trace) == pytest.approx(45, abs=0.1)

  def test_a_camera_value_raised_during_a_long_gap_is_not_held_against_the_map(self, monkeypatch):
    """With no map for long enough the camera's 50 does become the limit -- but once the map is back at 45 the camera
    may not HOLD a value the map never agreed on."""
    r = Rig(monkeypatch)
    r.run(10.0, map_mph=45, cam=45)
    r.run(5.0, cam=50)
    r.mem.mapd_alive = False
    r.run(SIGN_RAISE_S + 6.0)
    assert (r.limit, r.c._sign.why) == (50, "noMap")
    r.mem.mapd_alive = True
    r.run(6.0)
    assert (r.limit, r.c._sign.why) == (45, "heldHigher")

  def test_on_ramp_early_freeway_limit_is_not_taken(self, monkeypatch):
    """Albany 11:36: the camera showed the freeway 65 while mapd still had the 35 street, 30 s early."""
    r = Rig(monkeypatch, set_mph=40)
    r.run(10.0, map_mph=35, cam=35)
    r.run(30.0, cam=65)
    assert r.limit == 35
    r.run(5.0, map_mph=65)
    assert (r.limit, r.c._sign.why) == (65, "agree")

  def test_without_a_map_a_raise_needs_10_s(self, monkeypatch):
    r = Rig(monkeypatch)
    r.run(10.0, map_mph=0, cam=45)
    assert r.limit == 45
    r.run(SIGN_RAISE_S - 1.0, cam=50)
    assert (r.limit, r.c._sign.why) == (45, "heldHigher")
    r.run(6.0)                                        # + the first read's phase and the ordinary 3 s rise confirm
    assert r.limit == 50


class TestNoMap:
  @pytest.mark.parametrize("toggle", [True, False])
  def test_no_map_limit_camera_40_is_used_whatever_the_toggle(self, monkeypatch, toggle):
    r = Rig(monkeypatch, sign_on=toggle)
    r.run(SIGN_CONFIRM_S + 1.5, map_mph=0, cam=40)
    assert (r.limit, r.c._sign.why, r.c._sign.src) == (40, "noMap", "camera")

  @pytest.mark.parametrize("toggle", [True, False])
  def test_dead_mapd_with_a_latched_limit_counts_as_no_map(self, monkeypatch, toggle):
    """mapd_configd clears MapSpeedLimit only after ~5 s of silence (mapsl2pnw), so inside that window
    it is latched; liveness is NextMapSpeedLimit's timestamp."""
    r = Rig(monkeypatch, sign_on=toggle)
    r.run(10.0, map_mph=60, cam=60)
    r.mem.mapd_alive = False
    r.run(10.0, cam=40)
    assert (r.limit, r.c._sign.why) == (40, "noMap")

  def test_mapd_dying_mid_zone_keeps_the_limit_seamlessly(self, monkeypatch):
    r = Rig(monkeypatch)
    r.run(10.0, map_mph=60, cam=60)
    r.mem.mapd_alive = False
    r.run(1.05)
    assert (r.limit, r.c._sign.why) == (60, "noMap"), "a value the camera has shown for long is taken at once"


class TestLotSigns:
  @pytest.mark.parametrize("lot", [5, 10])
  def test_a_carried_over_lot_sign_never_lowers_the_road_limit(self, monkeypatch, lot):
    """11:35:18 PT on 09-24: the camera still showed the lot's 5 on the Corvallis-Lebanon Hwy (map 40) at 38 mph."""
    r = Rig(monkeypatch)
    r.run(10.0, map_mph=40, cam=40)
    r.run(30.0, cam=lot)
    assert (r.limit, r.c._sign.why, r.c._sign.cam) == (40, "lotSign", lot)

  def test_nor_becomes_the_limit_without_a_map(self, monkeypatch):
    r = Rig(monkeypatch)
    r.run(10.0, map_mph=0, cam=25)
    assert r.limit == 25
    r.run(30.0, cam=5)
    assert (r.limit, r.c._sign.why) == (0, "lotSign"), "no usable limit at all: exactly the map-only answer"

  def test_15_is_a_road_sign(self, monkeypatch):
    r = Rig(monkeypatch)
    r.run(10.0, map_mph=25, cam=25)
    r.run(10.0, cam=15)
    assert (r.limit, r.c._sign.why) == (15, "cameraOverride")


class TestToggleAndStaleness:
  def test_toggle_off_map_known_is_map_only(self, monkeypatch):
    r = Rig(monkeypatch, sign_on=False)
    r.run(10.0, map_mph=70, cam=70)
    r.run(30.0, cam=55)
    assert (r.limit, r.c._sign.why) == (70, "toggleOff")

  @pytest.mark.parametrize("status, why", [("stale", "stale"), ("noLimit", "noSign"), ("unavailable", "unavailable")])
  def test_unusable_camera_is_map_only(self, monkeypatch, status, why):
    r = Rig(monkeypatch)
    r.run(10.0, map_mph=70, cam=55)
    assert r.limit == 55
    r.run(10.0, cam_status=status)
    r.run(5.0)
    assert (r.limit, r.c._sign.why) == (70, why)
    if status == "stale":
      assert any("not usable" in w for w in r.warnings), r.warnings
    if status == "unavailable":
      assert any("reports it unavailable" in e for e in r.errors), r.errors

  def test_camera_unreadable_from_carstate_is_logged_and_map_only(self, monkeypatch):
    r = Rig(monkeypatch, with_sign=False)             # a carState without the fields (an old opendbc pin)
    r.run(5.0, map_mph=70)
    assert (r.limit, r.c._sign.why, r.c._sign.cam_st) == (70, "stale", "unreadable")
    assert any("camera speed limit unreadable" in e for e in r.errors), r.errors

  def test_toggle_unreadable_is_on_and_logged(self, monkeypatch):
    r = Rig(monkeypatch, sign_on=RuntimeError("UnknownKeyName"))
    r.run(10.0, map_mph=70, cam=70)
    r.run(10.0, cam=55)
    assert r.limit == 55
    assert any("DisableFordSignSpeedLimit unreadable" in e for e in r.errors), r.errors

  def test_every_source_change_is_a_cloudlog_event(self, monkeypatch):
    r = Rig(monkeypatch)
    r.run(10.0, map_mph=70, cam=70)
    r.run(10.0, cam=55)
    whys = [kw["why"] for n, kw in r.events if n == "speedadjust_sign_limit"]
    assert whys[:3] == ["agree", "pending", "cameraOverride"], whys


class TestLookAheadAndPolice:
  def test_camera_matching_the_announced_drop_is_taken_at_once(self, monkeypatch):
    """18:57: the look-ahead is running toward mapd's announced 40; the camera shows 40 while mapd still says 60.
    The camera value equals the announcement, so it skips its 3 s persistence, and option 2 skips the 2 s confirm."""
    r = Rig(monkeypatch, la_mode=sa.LA_LIVE, set_mph=75)
    r.v = 74 * MPH
    d = 1000.0                                        # announced 1 km out; the look-ahead starts at ~620 m
    r.run(0.05, map_mph=60, cam=60, nxt=(40, d))
    while d > 300.0:                                  # mapd's distance counts down with the truck's own travel
      d -= r.v * 0.5
      r.run(0.5, nxt=(40, d))
    assert r.c._la is not None and r.c._la["live"] and r.c._la["mat_t"] is None, "the look-ahead must be running"
    assert r.limit == 60
    t0 = r.trace[-1][0]
    r.run(1.05, cam=40, nxt=(40, d - r.v * 1.05))
    assert r.limit == 40, "taken on the first read after the camera changed"
    assert r.c._sign.why == "cameraAhead"
    assert r.c._la is not None and r.c._la["mat_t"] is not None, "the look-ahead sees the drop materialize"
    assert r.trace[-1][0] - t0 <= 1.1

  def test_control_without_announcement_the_camera_waits(self, monkeypatch):
    r = Rig(monkeypatch)
    r.run(5.0, map_mph=60, cam=60)
    r.run(1.05, cam=40)
    assert (r.limit, r.c._sign.why) == (60, "pending")

  def test_police_is_the_working_limit_plus_5(self, monkeypatch):
    """A police alert in a work zone: limit + 5 of the CAMERA's 55, not the map's 70."""
    r = Rig(monkeypatch, mode=1)                     # Police only: no limit-drop trim in the way
    r.mem.police = {"state": "alert", "dist_mi": 0.2, "key": "a"}
    r.run(10.0, map_mph=70, cam=55)
    r.run(20.0)
    assert r.limit == 55
    assert r.trace[-1][3] == pytest.approx(60, abs=0.1)

  def test_police_uses_the_map_when_the_camera_is_held_higher(self, monkeypatch):
    r = Rig(monkeypatch, mode=1)
    r.mem.police = {"state": "alert", "dist_mi": 0.2, "key": "a"}
    r.run(10.0, map_mph=45, cam=45)
    r.run(30.0, cam=50)
    assert r.trace[-1][3] == pytest.approx(50, abs=0.1)


class TestTeslaUnchanged:
  def _scenario(self, r):
    r.mem.police = {"state": "alert", "dist_mi": 0.1, "key": "p"}
    r.run(10.0, map_mph=70, cam=70)
    r.run(20.0, cam=55)
    r.run(10.0, map_mph=45, cam=25)
    r.run(10.0, map_mph=0, cam=40)
    return [(t, lim, out) for t, lim, _, out in r.trace]

  def test_no_capability_ignores_the_camera_entirely(self, monkeypatch):
    with_cam = self._scenario(Rig(monkeypatch, sign=False))
    without = self._scenario(Rig(monkeypatch, sign=False, with_sign=False))
    assert with_cam == without

  def test_its_status_carries_only_new_null_fields(self, monkeypatch):
    r = Rig(monkeypatch, sign=False)
    self._scenario(r)
    st = r.status()
    for k in ("slCam", "slCamSt", "slMap", "slSrc", "slWhy"):
      assert k in st and st[k] is None, (k, st.get(k))
    assert not [n for n, _ in r.events if n == "speedadjust_sign_limit"]


class TestTelemetry:
  def test_values_reach_the_status_and_ces_forwards_every_key(self, monkeypatch):
    from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import SA_TELE_KEYS
    r = Rig(monkeypatch)
    r.run(10.0, map_mph=70, cam=70)
    r.run(10.0, cam=55)
    st = r.status()
    assert (st["slCam"], st["slCamSt"], st["slSrc"], st["slWhy"]) == (55, "valid", "camera", "cameraOverride")
    assert st["slMap"] == pytest.approx(70 * MPH, abs=0.01)
    assert set(st) <= set(SA_TELE_KEYS), set(st) - set(SA_TELE_KEYS)


class TestLookAheadOwnsItsDrop:
  """Fable review 2026-09-27: map 60 announcing 40, look-ahead LIVE, truck at 60 with the set at 66. The map drops to 40
  at the boundary, the camera (agreed on 60) is late. Holding 60 made the look-ahead abort "passed" and restore 66 mph
  inside the 40 zone."""

  def _drive(self, monkeypatch, lag):
    r = Rig(monkeypatch, set_mph=66.0, la_mode=sa.LA_LIVE)
    r.v = 60.0 * MPH
    r.run(5.0, map_mph=60, cam=60)
    d = 600.0
    while d > 0.0:
      r.run(1.0, nxt=(40, d))
      d -= r.v * 1.0
    r.run(lag, map_mph=40, cam=60)
    r.run(15.0, cam=40)
    return r

  @pytest.mark.parametrize("lag", [3.0, 8.0])
  def test_a_late_camera_never_aborts_or_restores_the_look_ahead(self, monkeypatch, lag):
    r = self._drive(monkeypatch, lag)
    aborts = [kw for n, kw in r.events if n == "speedadjust_lookahead" and kw.get("action") == "abort"]
    assert aborts == [], aborts
    assert r.c._la_why == "promote"
    t_drop = max(t for t, lim, _, _ in r.trace if lim > 50)       # the last moment the working limit was 60
    assert max(out for t, _, _, out in r.trace if t > t_drop) <= 44.0 + 0.1, "no restore above the 40 zone's target"
    assert any(why == "lookAhead" for _, _, why, _ in r.trace)

  def test_control_the_hold_is_kept_with_a_shadow_look_ahead(self, monkeypatch):
    """The 19:02 benefit stays wherever the look-ahead does not act: a SHADOW episode toward the same announced 25
    (mapd announced it 675 m ahead) must not release the hold."""
    r = Rig(monkeypatch, set_mph=66.0, la_mode=sa.LA_SHADOW)
    r.v = 47 * MPH
    r.run(10.0, map_mph=45, cam=45)
    d = 300.0
    while d > 0.0:
      r.run(1.0, nxt=(25, d))
      d -= r.v
    assert r.c._la is not None and not r.c._la["live"], "a shadow episode must be running"
    r.run(4.9, map_mph=25, nxt=(25, 1.0))
    assert r.limit == 45 and r.c._sign.why == "cameraOverride"


class TestHoldBound:
  def test_off_ramp_70_held_onto_a_25_ends_within_the_bound(self, monkeypatch):
    """13:24:47 variant (Fable): the camera keeps the freeway's agreed 70 down the off-ramp onto a 25 street."""
    r = Rig(monkeypatch, set_mph=75.0)
    r.run(10.0, map_mph=70, cam=70)
    t0 = r.trace[-1][0]
    r.v = 25.0 * MPH
    r.run(60.0, map_mph=25, cam=70)
    t_25 = min(t for t, lim, _, _ in r.trace if t > t0 and lim < 30)
    # the bound, + the first read's phase (1 s), + the ordinary 2 s drop confirm
    assert t_25 - t0 <= sl_mod.SIGN_HOLD_MAX_S + 1.0 + 2.0 + 0.1, t_25 - t0
    assert t_25 - t0 >= sl_mod.SIGN_HOLD_MAX_S, "the hold itself must still work"
    assert r.limit == 25 and r.c._sign.why == "heldHigher"
    assert any(why == "holdExpired" for _, _, why, _ in r.trace)

  def test_each_hold_gets_its_own_bound(self, monkeypatch):
    """Two separate false drops (the map back in agreement between them) are two holds, not one long one."""
    r = Rig(monkeypatch)
    r.run(10.0, map_mph=45, cam=45)
    r.run(5.0, map_mph=25)
    r.run(10.0, map_mph=45)
    r.run(5.0, map_mph=25)
    assert min(lim for _, lim, _, _ in r.trace) == pytest.approx(45, abs=0.1)

  def test_the_1902_false_25_is_still_held(self, monkeypatch):
    r = Rig(monkeypatch)
    r.run(10.0, map_mph=45, cam=45)
    r.run(5.0, map_mph=25)
    r.run(20.0, map_mph=50)
    assert min(lim for _, lim, _, _ in r.trace) == pytest.approx(45, abs=0.1)


KMH = 1000.0 / 3600.0


class TestRegion:
  def test_canada_map_known_80_to_50_kmh_works_map_only(self, monkeypatch):
    """Fable: in BC a "50" sign read as 50 mph (80 km/h) "agreed" with the map's 80 km/h and then held it through the
    50 km/h zone. Outside a US state the camera must not be used at all."""
    r = Rig(monkeypatch, set_mph=55.0)
    r.mem.pos = VANCOUVER_BC
    r.run(10.0, map_mph=80 * KMH / MPH, cam=50)
    r.run(30.0, map_mph=50 * KMH / MPH, cam=50)
    assert r.c._sl == pytest.approx(50 * KMH, abs=0.05)
    assert (r.c._sign.why, r.c._sign.region) == ("region", "CA")
    assert any("not in a US state" in w for w in r.warnings), r.warnings

  def test_canada_no_map_the_camera_is_not_used(self, monkeypatch):
    r = Rig(monkeypatch, set_mph=55.0)
    r.mem.pos = VANCOUVER_BC
    r.run(15.0, map_mph=0, cam=90)
    assert (r.limit, r.c._sign.why) == (0, "region")

  @pytest.mark.parametrize("pos", [SEATTLE, PORTLAND])
  def test_us_states_are_unchanged(self, monkeypatch, pos):
    r = Rig(monkeypatch)
    r.mem.pos = pos
    r.run(15.0, map_mph=0, cam=40)
    assert (r.limit, r.c._sign.why) == (40, "noMap")

  @pytest.mark.parametrize("pos, age", [(None, 0.5), (SEATTLE, sa.REGION_FIX_MAX_AGE_S + 1.0), ("bad", 0.5)])
  def test_unknown_position_turns_the_camera_off(self, monkeypatch, pos, age):
    r = Rig(monkeypatch)
    r.mem.pos, r.mem.pos_age = pos, age
    r.run(15.0, map_mph=0, cam=40)
    assert (r.limit, r.c._sign.why, r.c._sign.region) == (0, "region", None)
    if pos == "bad":
      assert any("LastGPSPosition unreadable" in e for e in r.errors), r.errors

  CANADA_IN_US_BOXES = {"whitehorse": (60.72, -135.06), "dawson_city": (64.06, -139.43), "inuvik": (68.36, -133.72),
                        "eagle_plains": (66.37, -136.72), "victoria_bc": (48.43, -123.37),
                        "beaver_creek_yt": (62.38, -140.87)}

  @pytest.mark.parametrize("name", sorted(CANADA_IN_US_BOXES))
  def test_the_bbox_table_calls_these_us_states(self, name):
    """The premise: mapd's coverage table alone resolves these Canadian places to a US state (AK / WA)."""
    from openpilot.system.mapd.coverage import region_and_key_for_gps
    assert region_and_key_for_gps(*self.CANADA_IN_US_BOXES[name])[1].startswith("us_state.")

  @pytest.mark.parametrize("map_mph", [0, 50])
  @pytest.mark.parametrize("name", sorted(CANADA_IN_US_BOXES))
  def test_canada_inside_a_us_box_turns_the_camera_off(self, monkeypatch, name, map_mph):
    """Fable re-review: a Whitehorse fix with no map made a "90" sign a 145 km/h working limit."""
    r = Rig(monkeypatch, set_mph=55.0)
    r.mem.pos = self.CANADA_IN_US_BOXES[name]
    r.run(15.0, map_mph=map_mph, cam=90)
    assert (r.limit, r.c._sign.why, r.c._sign.region) == (map_mph, "region", "canada-override")
    assert any("canada-override" in w for w in r.warnings), r.warnings

  # ... Juneau AK (lat 58.3), and Tok AK (63.34, -142.99: just west of the 141st meridian)
  @pytest.mark.parametrize("pos", [SEATTLE, PORTLAND, (58.30, -134.42), (63.34, -142.99)])
  def test_us_places_next_to_the_exclusions_are_unchanged(self, monkeypatch, pos):
    r = Rig(monkeypatch)
    r.mem.pos = pos
    r.run(15.0, map_mph=0, cam=40)
    assert (r.limit, r.c._sign.why) == (40, "noMap")
    assert r.c._sign.region in ("WA", "OR", "AK")

  def test_crossing_the_border_logs_once(self, monkeypatch):
    r = Rig(monkeypatch)
    r.run(10.0, map_mph=60, cam=60)
    r.mem.pos = VANCOUVER_BC
    r.run(20.0, map_mph=60 * 1.609 * KMH / MPH)
    whys = [kw["why"] for n, kw in r.events if n == "speedadjust_sign_limit"]
    assert whys.count("region") == 1, whys


class TestSelectorUnit:
  """Direct rules, no controller. Values in m/s."""

  def test_pending_holds_the_camera_source(self):
    s = SignLimitSelector()
    for i in range(5):
      s.select(float(i), 70 * MPH, True, None, 55, "valid", True, "WA", True)
    assert s.src == "camera"
    out = s.select(5.0, 70 * MPH, True, None, 50, "valid", True, "WA", True)          # the camera moves on inside the zone
    assert (s.src, s.why, round(out / MPH)) == ("hold", "pending", 55)

  def test_a_value_over_the_sane_bound_is_not_a_camera_value(self):
    s = SignLimitSelector()
    out = s.select(0.0, 60 * MPH, True, None, 250, "valid", True, "WA", True)
    assert (round(out / MPH), s.why, s.cam) == (60, "stale", None)


class TestToggleRegistration:
  def test_param_defaults_off_so_the_camera_is_used(self):
    from openpilot.common.params import Params, UnknownKeyName
    assert Params().get("DisableFordSignSpeedLimit", return_default=True) is False
    with pytest.raises(UnknownKeyName):       # the retired positive-sense key is gone and nothing reads it
      Params().get("FordSignSpeedLimit")

  def test_ui_toggle_is_defined_and_capability_gated_display_only(self):
    from pathlib import Path
    src = (Path(sa.__file__).resolve().parents[4] / "selfdrive/ui/layouts/settings/toggles.py").read_text()
    assert src.count('"DisableFordSignSpeedLimit": (\n') == 1 and src.count('"DisableFordSignSpeedLimit": tr_noop(') == 1
    assert '"DisableFordSignSpeedLimit": (lambda v: v.camera_speed_limit' in src      # the CAR_GATED row (test_car_gating.py)
    assert 'put_bool("DisableFordSignSpeedLimit"' not in src, "display-only gate: never rewrite the driver's setting"
    assert "Disable Ford Camera Speed Limit" in src and "BC / km/h countries turn this ON" in src
    assert '"FordSignSpeedLimit"' not in src

  def test_nothing_reads_the_retired_param(self):
    import re
    import subprocess
    root = __import__("pathlib").Path(sa.__file__).resolve().parents[4]
    files = subprocess.run(["git", "ls-files", "*.py", "*.h", "*.cc", "*.pyx", "*.sh"], cwd=root, capture_output=True,
                           text=True, check=True).stdout.split()
    assert len(files) > 500
    bad = [f for f in files if "/tests/" not in f and not f.endswith("params_keys.h")
           and re.search(r"(?<![A-Za-z])FordSignSpeedLimit", (root / f).read_text(errors="replace"))]
    assert bad == [], bad

  def test_controller_reads_the_inverted_param(self):
    from pathlib import Path
    src = Path(sa.__file__).with_name("speedadjust_controller.py").read_text()
    assert 'params.get("DisableFordSignSpeedLimit"' in src and 'params.get("FordSignSpeedLimit"' not in src
