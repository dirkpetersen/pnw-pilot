"""truckdecode2pnw (B): the overshoot-cancel rule compares set speeds in the cluster's real unit.

`set_speed_ms` is the cluster's own number x MPH_TO_MS (Ford CAN FD carstate hardcodes mph). On a km/h cluster that is
1.609x high while a gas-set's want is v_ego, so every gas-set would cancel. carState.cruiseState.speedClusterUnit (pnw-opendbc
truckdecode2pnw) now says which unit the number is in; only "kph" converts, "mph" and "unknown" keep today's comparison
and "unknown" is logged.

Numbers are the Sun 2026-09-13 21:16:33 PT Corvallis case (drives/2026-09-13/corvallis-resume-55/): truck at 15.36 m/s
(34.4 mph, 55.3 km/h) when our SET- fired. On the owner's mph truck the PCM engaged at 55 -> carstate 24.59 m/s -> cancel.
The same raw 55 on a km/h cluster is 55 km/h = 15.28 m/s, i.e. the tap speed: no cancel. km/h was never observed here.
"""
import pathlib

import pytest
from cereal import car, messaging

from openpilot.selfdrive.controls.lib import madsresume_pnw as M
from openpilot.selfdrive.controls.lib.madsresume_pnw import speed_unit_name
from openpilot.selfdrive.controls.lib.tests.test_madsresume_pnw import DT, SET, STEER_ONLY, Drive, _red_light, mk, \
  normal_brake_and_resume

MPH = 0.44704
TAP_V = 15.36


def gas_set_comes_back(raw_set, unit, v=TAP_V):
  """Steering-only after a brake, accelerate to v, lift, our SET fires; 0.15 s later stock cruise engages and carstate
  reports `raw_set` x MPH_TO_MS with speedClusterUnit `unit` on every tick (as it would, from the whole drive)."""
  d = Drive(set_speed_unit=unit)
  _red_light(d, 5.0)
  d.tick(300, gas_pressed=True, v_ego=v, **STEER_ONLY)
  for _ in range(int(5.0 / DT)):
    if d.fired():
      break
    d.tick(1, v_ego=v, **STEER_ONLY)
  assert d.fired(), "precondition: the gas-set must fire"
  t_fire, n_rec, cancels = d.offers[0][0], len(d.records), []
  for _ in range(100):
    now_s = round(d.t - t_fire, 3)
    kw = dict(lateral_only=False, op_enabled=True, cruise_enabled=True, set_speed_ms=raw_set * MPH, v_ego=v) \
      if now_s >= 0.15 else dict(STEER_ONLY, v_ego=v)
    out = d.b.update(mk(d.t, set_speed_unit=unit, **kw))
    if out.cancel:
      cancels.append(now_s)
    d.records.extend(out.records)
    d.t += DT
  return cancels, [r for r in d.records[n_rec:] if r["phase"] == "verify"]


class TestGasSet:
  def test_kph_cluster_engaging_at_the_tap_speed_is_not_cancelled(self):
    cancels, verify = gas_set_comes_back(55.0, "kph")
    assert cancels == [] and len(verify) == 1, (cancels, verify)
    v = verify[0]
    assert (v["reason"], v["cancel"], v["unit"]) == ("ok", False, "kph"), v
    assert v["gotMs"] == pytest.approx(55.0 / 3.6, abs=0.01) and v["wantMs"] == pytest.approx(TAP_V, abs=0.01)
    assert (v["gotDisplayMph"], v["wantDisplayMph"]) == (55.0, 34.4), "the display fields stay the raw numbers"

  def test_the_same_raw_numbers_on_the_mph_truck_still_cancel(self):
    """The 21:16 case must be untouched on the owner's truck."""
    cancels, verify = gas_set_comes_back(55.0, "mph")
    assert cancels == [0.15]
    assert (verify[0]["reason"], verify[0]["cancel"], verify[0]["unit"]) == ("setHigher", True, "mph")
    assert verify[0]["gotMs"] == pytest.approx(24.59, abs=0.01)

  def test_unknown_keeps_the_mph_assumption_and_says_so(self):
    """The safe side: an unestablished unit cancels exactly as today (a real overshoot is never waved through)."""
    cancels, verify = gas_set_comes_back(55.0, "unknown")
    assert cancels == [0.15] and verify[0]["unit"] == "unknown" and verify[0]["gotMs"] == pytest.approx(24.59, abs=0.01)

  @pytest.mark.parametrize("garbage", ["KPH", "km/h", "", None, 2])
  def test_anything_but_the_two_names_is_unknown(self, garbage):
    cancels, verify = gas_set_comes_back(55.0, garbage)
    assert cancels == [0.15] and verify[0]["unit"] == "unknown"

  def test_a_real_kph_overshoot_still_cancels(self):
    """70 km/h from a 55.3 km/h tap is +4.1 m/s (9 mph): the conversion must not wave a real overshoot through."""
    cancels, verify = gas_set_comes_back(70.0, "kph")
    assert cancels == [0.15] and verify[0]["reason"] == "setHigher"
    assert verify[0]["gotMs"] == pytest.approx(70.0 / 3.6, abs=0.01)

  def test_the_kph_threshold_is_3_mph_in_true_speed(self):
    """59 km/h = 16.39 m/s, +1.03 m/s (2.3 mph) over the tap: no cancel. 61 km/h = 16.94 m/s, +1.58 (3.5 mph): cancel.
    Under the mph assumption 59 would read 26.4 m/s and cancel."""
    assert gas_set_comes_back(59.0, "kph")[0] == []
    assert gas_set_comes_back(61.0, "kph")[0] == [0.15]
    assert gas_set_comes_back(59.0, "unknown")[0] == [0.15]


class TestResume:
  def _res_comes_back(self, raw_over, unit):
    d = normal_brake_and_resume(post_ticks=60, set_speed_unit=unit)
    assert d.fired() and d.offers[-1][2] == pytest.approx(SET)
    outs = [d.b.update(mk(d.t + k * DT, lateral_only=False, op_enabled=True, cruise_enabled=True,
                          set_speed_ms=SET + raw_over, set_speed_unit=unit)) for k in range(5)]
    verify = [r for o in outs for r in o.records if r["phase"] == "verify"]
    return [o.cancel for o in outs], verify

  def test_a_resume_converts_BOTH_sides(self):
    """RES want is the captured cruiseState.speed, the same raw unit as got. +1.5 raw on a km/h cluster is +0.93 m/s:
    setHigher, not cancelled. Converting only got would read it as setLower; converting neither would cancel."""
    cancels, verify = self._res_comes_back(1.5, "kph")
    assert cancels == [False] * 5
    v = verify[0]
    assert (v["reason"], v["unit"]) == ("setHigher", "kph"), v
    assert v["gotMs"] - v["wantMs"] == pytest.approx(1.5 * M.KPH_OVER_MPH, abs=0.02)

  def test_a_resume_over_3_mph_true_cancels_on_kph(self):
    cancels, _ = self._res_comes_back(2.5, "kph")                  # 2.5 x 0.621 = 1.55 m/s > 1.34
    assert cancels == [True, False, False, False, False]

  def test_mph_resume_is_unchanged(self):
    cancels, verify = self._res_comes_back(1.5, "mph")
    assert cancels == [True, False, False, False, False] and verify[0]["unit"] == "mph"


def test_the_factor():
  assert M.KPH_OVER_MPH == pytest.approx(0.621371, abs=1e-6)


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

  def test_the_seam_real_frames_through_card_style_publish(self):
    """Real IPMA_Data2 + Cluster_Info1_FD1 payloads through opendbc's real Ford CarInterface.update, the CarState
    assigned into a carState message the way card.state_publish does, read back off the wire, named here."""
    from opendbc.car import gen_empty_fingerprint
    from opendbc.car.car_helpers import interfaces
    from opendbc.car.ford.tests import test_cluster_unit_pnw as T
    from opendbc.car.ford.values import CAR
    names = {}
    for label, ipma, cluster in (("mph", T.IPMA_MPH, T.CLUSTER_ENG), ("kph", T.IPMA_KPH, T.CLUSTER_METRIC),
                                 ("unknown", T.IPMA_NODATA, T.CLUSTER_ENG)):
      CarInterface = interfaces[CAR.FORD_F_150_LIGHTNING_MK1]
      fp = gen_empty_fingerprint()
      fp[0][0x5A] = 8
      CI = CarInterface(CarInterface.get_params(CAR.FORD_F_150_LIGHTNING_MK1, fp, [], False, False, False))
      cs, t = None, 0
      for _ in range(50):
        t += 10_000_000
        cs = CI.update([(t, [(0x3D9, bytes.fromhex(ipma), 2), (0x430, bytes.fromhex(cluster), 0)])])
      msg = messaging.new_message("carState")
      msg.carState = cs
      names[label] = speed_unit_name(getattr(messaging.log_from_bytes(msg.to_bytes()).carState.cruiseState,
                                             "speedClusterUnit", None))
    assert names == {"mph": "mph", "kph": "kph", "unknown": "unknown"}


class TestSelfdrivedWiring:
  """selfdrived cannot be imported on the dev host; pin the lines."""
  SRC = (pathlib.Path(__file__).parents[3] / "selfdrived" / "selfdrived.py").read_text()

  def test_the_brain_is_told_the_unit_from_carstate(self):
    call = self.SRC[self.SRC.index("inputs = ResumeInputs("):self.SRC.index("out = self.mads_resume.update(inputs)")]
    assert 'set_speed_unit=speed_unit_name(getattr(CS.cruiseState, "speedClusterUnit", None)),' in call

  def test_a_converted_or_assumed_unit_is_logged(self):
    loop = self.SRC[self.SRC.index("for rec in out.records:"):self.SRC.index("self.ces_pnw.log_mads_resume(rec)")]
    assert '\n        if rec.get("phase") == "verify" and rec.get("unit") in ("kph", "unknown"):\n' in loop
    assert "cloudlog.warning(" in loop.split('rec.get("unit") in ("kph", "unknown")')[1].split("if rec.get(\"loud\")")[0]
