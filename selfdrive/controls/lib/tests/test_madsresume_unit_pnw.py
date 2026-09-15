"""units2pnw: the gas-set verify on a km/h cluster.

Ford CAN FD carstate now converts Veh_V_DsplyCcSet by carState.cruiseState.speedClusterUnit, which follows the cluster's
own Cluster_Info1_FD1.MetricActv_B_Actl (pnw-opendbc units2pnw 1/3 + 2/3). So `set_speed_ms` reaches the brain as TRUE
m/s and the brain converts nothing; `unit` is telemetry, and an unknown unit (carstate ASSUMED mph) is flagged for
selfdrived's warning once per change.

THE REGRESSION: Sun 2026-09-13 21:16:33 PT, Corvallis (drives/2026-09-13/corvallis-resume-55/, corrected by
drives/2026-09-14/units-kmh/). The cluster had been in km/h since 21:08. Standby set 42 km/h; the truck was at 15.36 m/s
(34.4 mph = 55.3 km/h) when our gas-set SET- fired, and the PCM engaged at "55" -- 55 km/h = 15.28 m/s, the tap speed;
it then held 53.7-54.0 km/h. openpilot read 55 as mph: gotMs 24.59, "setHigher", LOUD, and under engagegoal2pnw a cancel.
The replay below feeds that segment's REAL 0x430 frame and real EngBrakeData frames through opendbc's real Ford carstate.

nosetcancel2pnw (owner decision 2026-09-14, "remove it"): that cancel is gone. The verify still reads and logs
setHigher; every record here carries cancel False and overshootAction "none", whatever the unit.
"""
import pathlib

import pytest
from cereal import car, messaging

from openpilot.selfdrive.controls.lib import madsresume_pnw as M
from openpilot.selfdrive.controls.lib.madsresume_pnw import speed_unit_name
from openpilot.selfdrive.controls.lib.tests.test_madsresume_pnw import DT, SET, STEER_ONLY, Drive, _red_light, mk, \
  normal_brake_and_resume

MPH = 0.44704
KPH = 1.0 / 3.6
TAP_V = 15.36     # m/s at 21:16:32.78 PT, the lift-off before our SET- (qlog 0000013f seg 2)


class Truck:
  """opendbc's real Ford CarInterface fed real frames; each read goes through a carState message off the wire, the way
  card publishes it and selfdrived receives it."""

  def __init__(self, cluster_hex):
    from opendbc.car import gen_empty_fingerprint
    from opendbc.car.car_helpers import interfaces
    from opendbc.car.ford.values import CAR
    CarInterface = interfaces[CAR.FORD_F_150_LIGHTNING_MK1]
    fp = gen_empty_fingerprint()
    fp[0][0x5A] = 8
    self.CI = CarInterface(CarInterface.get_params(CAR.FORD_F_150_LIGHTNING_MK1, fp, [], False, False, False))
    self.cluster = bytes.fromhex(cluster_hex)
    self.t = 0

  def state(self, engbrake_hex, ticks=20):
    cs = None
    for _ in range(ticks):
      self.t += 10_000_000
      cs = self.CI.update([(self.t, [(0x430, self.cluster, 0), (0x165, bytes.fromhex(engbrake_hex), 0)])])
    msg = messaging.new_message("carState")
    msg.carState = cs
    return messaging.log_from_bytes(msg.to_bytes()).carState

  @staticmethod
  def inputs(CS):
    """selfdrived's own two expressions for these ResumeInputs fields."""
    return dict(set_speed_ms=float(CS.cruiseState.speed),
                set_speed_unit=speed_unit_name(getattr(CS.cruiseState, "speedClusterUnit", None)),
                cruise_enabled=bool(CS.cruiseState.enabled))


def gas_set_comes_back(stby, engaged, v=TAP_V):
  """The 21:16 gas-set: red light, pull away on the accelerator to v, lift, our SET fires; 0.15 s later (the measured
  PCM response) stock cruise is engaged. `stby` / `engaged` are ResumeInputs overrides for the standby and engaged
  truck. Returns the verify records (asserting on the way that none of them says anything acted)."""
  steer_only = {**STEER_ONLY, **stby, "cruise_enabled": False}
  d = Drive()
  _red_light(d, 5.0)
  d.tick(300, gas_pressed=True, v_ego=v, **steer_only)
  for _ in range(int(5.0 / DT)):
    if d.fired():
      break
    d.tick(1, v_ego=v, **steer_only)
  assert d.fired(), "precondition: the gas-set must fire"
  t_fire, n_rec = d.offers[0][0], len(d.records)
  for _ in range(100):
    now_s = round(d.t - t_fire, 3)
    kw = dict(lateral_only=False, op_enabled=True, v_ego=v, **engaged) if now_s >= 0.15 else dict(steer_only, v_ego=v)
    out = d.b.update(mk(d.t, **kw))
    d.records.extend(out.records)
    d.t += DT
  verify = [r for r in d.records[n_rec:] if r["phase"] == "verify"]
  assert all(r["cancel"] is False and r["overshootAction"] == "none" for r in verify), verify
  return verify


class TestCorvallisReplay:
  def _frames(self):
    from opendbc.car.ford.tests import test_cluster_unit_pnw as T
    return T

  def test_the_2116_sequence_no_longer_reads_setHigher(self):
    """The segment's own metric 0x430 + standby-42 frame, then engaged 55: ok, no cancel, 15.28 m/s, unit kph."""
    T = self._frames()
    truck = Truck(T.CLUSTER_METRIC_2116)
    stby, engaged = Truck.inputs(truck.state(T.EB_KMH_42_STBY)), Truck.inputs(truck.state(T.EB_KMH_55))
    assert (stby["cruise_enabled"], stby["set_speed_ms"], stby["set_speed_unit"]) == (False, pytest.approx(42 * KPH), "kph")
    assert (engaged["cruise_enabled"], engaged["set_speed_ms"]) == (True, pytest.approx(55 * KPH))
    verify = gas_set_comes_back(stby, engaged)
    assert len(verify) == 1, verify
    v = verify[0]
    assert (v["reason"], v["cancel"], v["unit"], v.get("loud")) == ("ok", False, "kph", None), v
    assert v["gotMs"] == pytest.approx(15.28, abs=0.01) and v["wantMs"] == pytest.approx(TAP_V, abs=0.01)
    assert (v["gotDisplayMph"], v["wantDisplayMph"]) == (34.2, 34.4)
    assert "unitAssumed" not in v

  def test_control_the_same_frames_on_an_english_cluster_still_read_setHigher(self):
    """The verify is intact: with the real English 0x430 the same raw 55 is 55 mph -> setHigher, loud, no action."""
    T = self._frames()
    truck = Truck(T.CLUSTER_ENG)
    stby, engaged = Truck.inputs(truck.state(T.EB_KMH_42_STBY)), Truck.inputs(truck.state(T.EB_KMH_55))
    assert engaged["set_speed_ms"] == pytest.approx(55 * MPH) and engaged["set_speed_unit"] == "mph"
    verify = gas_set_comes_back(stby, engaged)
    assert len(verify) == 1
    assert (verify[0]["reason"], verify[0]["loud"], verify[0]["cancel"], verify[0]["unit"]) == ("setHigher", True, False, "mph")
    assert verify[0]["gotMs"] == pytest.approx(24.59, abs=0.01)

  def test_a_real_kmh_overshoot_is_still_logged_loud(self):
    """70 km/h = 19.44 m/s from a 15.36 m/s tap (+9 mph): the true-unit compare must not wave a real overshoot through
    the log (Rule 2) -- it is only no longer acted on."""
    verify = gas_set_comes_back(dict(set_speed_ms=42 * KPH, set_speed_unit="kph"),
                                dict(cruise_enabled=True, set_speed_ms=70 * KPH, set_speed_unit="kph"))
    assert len(verify) == 1 and verify[0]["reason"] == "setHigher" and verify[0]["loud"] is True, verify


class TestTheBrainConvertsNothing:
  def test_kph_is_not_converted_again(self):
    """set_speed_ms is already true m/s: a kph unit must not scale it (24.59 stays 24.59 and reads setHigher)."""
    verify = gas_set_comes_back({}, dict(cruise_enabled=True, set_speed_ms=55 * MPH, set_speed_unit="kph"))
    assert verify[0]["reason"] == "setHigher" and verify[0]["gotMs"] == pytest.approx(24.59, abs=0.01)
    assert verify[0]["unit"] == "kph"

  def test_the_verify_tolerance_is_true_speed_on_kph(self):
    """56 km/h = 15.56 m/s, +0.20 over the tap: ok. 59 km/h = 16.39, +1.03 (> SET_MODE_TOL_MS 1.0): setHigher.
    (A brain that scaled kph again would read both as setLower.)"""
    kph = lambda n: dict(cruise_enabled=True, set_speed_ms=n * KPH, set_speed_unit="kph")  # noqa: E731
    assert gas_set_comes_back({}, kph(56))[0]["reason"] == "ok"
    assert gas_set_comes_back({}, kph(59))[0]["reason"] == "setHigher"

  def test_resume_compares_true_speeds(self):
    d = normal_brake_and_resume(post_ticks=60, set_speed_unit="kph")
    assert d.fired() and d.offers[-1][2] == pytest.approx(SET)
    outs = [d.b.update(mk(d.t + k * DT, lateral_only=False, op_enabled=True, cruise_enabled=True,
                          set_speed_ms=SET + 2.0 * MPH, set_speed_unit="kph")) for k in range(5)]
    verify = [r for o in outs for r in o.records if r["phase"] == "verify"]
    assert len(verify) == 1 and verify[0]["reason"] == "setHigher" and verify[0]["cancel"] is False
    assert verify[0]["gotMs"] - verify[0]["wantMs"] == pytest.approx(2.0 * MPH, abs=0.01)


class TestUnknownUnit:
  def test_unknown_keeps_the_mph_reading_and_is_flagged(self):
    verify = gas_set_comes_back({}, dict(cruise_enabled=True, set_speed_ms=55 * MPH, set_speed_unit="unknown"))
    assert verify[0]["reason"] == "setHigher" and verify[0]["gotMs"] == pytest.approx(24.59, abs=0.01)
    assert verify[0]["unit"] == "unknown" and verify[0]["unitAssumed"] is True

  @pytest.mark.parametrize("garbage", ["KPH", "km/h", "", None, 2])
  def test_anything_but_the_two_names_is_unknown(self, garbage):
    verify = gas_set_comes_back({}, dict(cruise_enabled=True, set_speed_ms=55 * MPH, set_speed_unit=garbage))
    assert verify[0]["reason"] == "setHigher" and verify[0]["unit"] == "unknown"

  def test_flagged_once_per_change_not_per_press(self):
    """Three verifies on one brain: unknown (flag), unknown (no flag), kph, unknown (flag again)."""
    b = M.MadsResumeBrain()
    flags = []
    for unit in ("unknown", "unknown", "kph", "unknown"):
      b._verify_until, b._verify_set, b._verify_mode = 1e9, 15.0, "set"
      out = b.update(mk(0.0, lateral_only=False, op_enabled=True, cruise_enabled=True, set_speed_ms=15.0, v_ego=15.0,
                        set_speed_unit=unit))
      rec = [r for r in out.records if r["phase"] == "verify"]
      assert len(rec) == 1, out.records
      flags.append(rec[0].get("unitAssumed", False))
    assert flags == [True, False, False, True]


class TestSpeedUnitName:
  SpeedUnit = car.CarState.CruiseState.SpeedUnit

  def _reader(self, unit):
    msg = messaging.new_message("carState")
    msg.carState.cruiseState.speedClusterUnit = unit
    return messaging.log_from_bytes(msg.to_bytes()).carState.cruiseState.speedClusterUnit

  def test_real_reader_values(self):
    """What selfdrived actually holds: a reader enum off the wire (str() is the bare name -- compared as an enum)."""
    assert speed_unit_name(self._reader(self.SpeedUnit.mph)) == "mph"
    assert speed_unit_name(self._reader(self.SpeedUnit.kph)) == "kph"
    assert speed_unit_name(self._reader(self.SpeedUnit.unknown)) == "unknown"
    assert speed_unit_name(messaging.new_message("carState").carState.cruiseState.speedClusterUnit) == "unknown"

  def test_builder_values_and_absence(self):
    assert speed_unit_name(self.SpeedUnit.kph) == "kph"
    assert speed_unit_name(None) == "unknown"
    assert speed_unit_name("mph") == "unknown", "a string is not the enum; only the enum means anything"


class TestSelfdrivedWiring:
  """selfdrived cannot be imported on the dev host; pin the lines."""
  SRC = (pathlib.Path(__file__).parents[3] / "selfdrived" / "selfdrived.py").read_text()

  def test_the_brain_is_told_the_speed_and_unit_from_carstate(self):
    call = self.SRC[self.SRC.index("inputs = ResumeInputs("):self.SRC.index("out = self.mads_resume.update(inputs)")]
    assert "set_speed_ms=float(CS.cruiseState.speed)," in call
    assert 'set_speed_unit=speed_unit_name(getattr(CS.cruiseState, "speedClusterUnit", None)),' in call

  def test_an_assumed_unit_is_logged_on_the_brains_flag(self):
    loop = self.SRC[self.SRC.index("for rec in out.records:"):self.SRC.index("self.ces_pnw.log_mads_resume(rec)")]
    assert '\n        if rec.get("unitAssumed"):\n' in loop
    assert "cloudlog.warning(" in loop.split('if rec.get("unitAssumed"):')[1].split('if rec.get("loud")')[0]
