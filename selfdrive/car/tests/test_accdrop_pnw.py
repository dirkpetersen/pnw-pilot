"""accdroplog2pnw: the stock-ACC dropout logger (selfdrive/car/accdrop_pnw.py).

Runs against the REAL Ford Lightning CAN parsers (so "registered" / "not registered" is what opendbc's carstate
actually registers, and the lazy-registration hazard is tested for real). CAN decode is simulated by writing
values straight into those parsers, exactly where CANParser.update() would put them.
"""
import json
import math
import sys
import time
from types import SimpleNamespace

import pytest

from opendbc.car import structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.ford import fordcan
from opendbc.car.ford.icbm_pnw import IcbmCommand
from opendbc.car.ford.values import CAR
from opendbc.can.packer import CANPacker

from openpilot.selfdrive.car import accdrop_pnw as adp
from openpilot.selfdrive.controls.lib.pnw_vehicle import PnwVehicle

LIGHTNING = CAR.FORD_F_150_LIGHTNING_MK1
MPH = 0.44704

# the capability-listed messages opendbc's Ford carstate does NOT register (pinned opendbc 78477c72)
EXPECTED_NOT_REGISTERED = {
  "pt:BrakeSysFeatures_2", "pt:BrakeSnData_5", "pt:DesiredTorqBrk_2", "pt:VehicleOperatingModes",
  "pt:HEV_Powertrain_Data2", "pt:TorqueDataEngFlags", "pt:Powertrain_Data_4", "pt:SelectDriveModeData",
  "pt:Low_Voltage_Power_Data_FD1",
}


def _registered(parsers):
  return {b: sorted(k for k in dict.keys(p.vl) if isinstance(k, str)) for b, p in parsers.items()}


class FakeSM:
  def __init__(self):
    self.data = {"carControl": structs.CarControl(), "onroadEvents": [], "pandaStates": []}
    self.updated = {"onroadEvents": False, "pandaStates": False}

  def __getitem__(self, k):
    return self.data[k]


class Bench:
  """A Lightning card as the logger sees it: real parsers, a CarState, a SubMaster, the tick clock."""

  def __init__(self, car_controller=None, write_fn=None, route_fn=None):
    CI_cls = interfaces[LIGHTNING]
    CP = CI_cls.get_non_essential_params(LIGHTNING)
    self.CI = CI_cls(CP)
    self.parsers = self.CI.can_parsers
    self.records: list = []
    veh = PnwVehicle(SimpleNamespace(carFingerprint=LIGHTNING, brand="ford", openpilotLongitudinalControl=False))
    # SAME ORDER AS card.py: the logger is built BEFORE the first CarInterface.update(), i.e. before opendbc's
    # carstate has lazily registered anything. A logger that resolved messages at construction recorded no CAN.
    self.log = adp.AccDropLogger(self.parsers, veh.acc_drop_status_msgs, veh.acc_drop_trace, car_controller,
                                 write_fn=write_fn or self.records.append,
                                 route_fn=route_fn or (lambda w: {"route": "00000133--b4cc0b5efd", "seg": 348, "edgeInSegS": 46.9}),
                                 threaded=False)
    self.CI.update([(0, [])])                       # carstate registers what it reads, as on the car's first tick
    self.registered_before = _registered(self.parsers)
    self.t = 1000.0
    self.rx: set = set()
    self.cs = structs.CarState()
    self.cs.canValid = True
    self.cs.gearShifter = structs.CarState.GearShifter.drive
    self.sm = FakeSM()

  def can(self, bus, msg, **sigs):
    """Like CANParser.update() for one frame: set the values, stamp EVERY signal of the message."""
    p = self.parsers[bus]
    for s, v in sigs.items():
      dict.get(p.vl, msg)[s] = float(v)
    self.rx.add((bus, msg))
    self._stamp(bus, msg)

  def _stamp(self, bus, msg):
    ts = self.parsers[bus].ts_nanos[msg]
    for s in ts:
      ts[s] = int(self.t * 1e9)

  def cruise(self, stat):
    """Set the PCM cruise state the way Ford carstate derives it from CcStat_D_Actl."""
    self.can("pt", "EngBrakeData", CcStat_D_Actl=stat, CcMde_D_Actl=1 if stat in (4, 5) else 0)
    self.cs.cruiseState.enabled = stat in (4, 5)
    self.cs.cruiseState.available = stat in (3, 4, 5)
    self.cs.accFaulted = stat in (1, 2)

  def events(self, *names):
    self.sm.data["onroadEvents"] = [SimpleNamespace(name=n) for n in names]
    self.sm.updated["onroadEvents"] = True

  def run(self, seconds, sends=None, every_tick=False):
    for _ in range(round(seconds * 100)):
      self.t += 0.01
      for p in self.parsers.values():
        p._last_update_nanos = int(self.t * 1e9)
      for bus, msg in self.rx:                      # every message seen once keeps arriving, as on the bus
        self._stamp(bus, msg)
      self.log.update(self.cs, self.sm, sends or [], self.t)
      self.sm.updated = {"onroadEvents": False, "pandaStates": False}
      if not every_tick:
        sends = None

  def steady_active(self, mph=57.9):
    self.cs.vEgo = mph * MPH
    self.cruise(5)
    self.can("pt", "DesiredTorqBrk", CcDis_B_Cmd=0, AccBrkDeny_B_Actl=0, TracCtlPtActv_B_Actl=0)
    self.can("cam", "ACCDATA", AccCancl_B_Rq=0, AccDeny_B_Rq=0, AccPrpl_A_Rq=0.3)
    self.can("cam", "ACCDATA_3", Tja_D_Stat=0, AccTrgDist2_D_Dsply=1)
    self.sm.data["carControl"].enabled = True
    self.sm.data["carControl"].latActive = True
    self.events()

  def parsed(self):
    return [json.loads(r) for r in self.records]


def test_capability_view_tesla_inert_lightning_listed():
  tesla = PnwVehicle(SimpleNamespace(carFingerprint="TESLA_MODEL_S_HW3", brand="tesla", openpilotLongitudinalControl=True))
  assert tesla.acc_drop_status_msgs == () and tesla.acc_drop_trace == ()
  assert PnwVehicle(None).acc_drop_status_msgs == ()
  lightning = PnwVehicle(SimpleNamespace(carFingerprint=LIGHTNING, brand="ford", openpilotLongitudinalControl=False))
  assert ("pt", "DesiredTorqBrk") in lightning.acc_drop_status_msgs and lightning.acc_drop_trace


def test_never_registers_a_message_and_names_what_it_cannot_record():
  b = Bench()
  b.steady_active()
  b.run(0.01)
  assert set(b.log.not_registered) == EXPECTED_NOT_REGISTERED, b.log.not_registered
  assert all("needs an opendbc decode" in why for why in b.log.not_registered.values())
  # built before carstate registered anything (card's order), yet every registered message is recorded
  assert len(b.log._status) == 11 and len(b.log.trace_cols) == len(adp.TRACE_COLS) + 3
  b.run(4.0)
  b.cruise(3)
  b.run(1.5)
  assert len(b.records) == 1
  # the canValid hazard: a lazily-registered message is alive-checked and makes the car undriveable
  assert _registered(b.parsers) == b.registered_before


def test_sun_140546_no_input_standby_produces_one_self_explaining_record():
  """Modeled on rlog_extracts/dev2_00000133--b4cc0b5efd--348.txt: Active at 57.9 mph, PCM 5->3 and CcMde 1->0
  in one frame, no brake/button/openpilot frame, pcmDisable, AccTrgDist2 1->2 at +0.05 s."""
  b = Bench()
  b.steady_active()
  b.run(4.0)
  b.cruise(3)
  b.sm.data["carControl"].enabled = False
  b.events("pcmDisable")
  b.run(0.05)
  b.can("cam", "ACCDATA_3", AccTrgDist2_D_Dsply=2)
  b.run(0.90)
  assert b.records == []                               # written POST_S after the edge, not before
  b.run(0.15)
  (rec,) = b.parsed()
  assert rec["ev"] == "accDrop" and rec["edge"] == "active->standby"
  assert rec["route"] == "00000133--b4cc0b5efd" and rec["seg"] == 348
  eng = rec["chg"]["pt:EngBrakeData"]
  assert eng["in"]["CcStat_D_Actl"] == 5 and eng["in"]["CcMde_D_Actl"] == 1
  assert [0.0, {"CcMde_D_Actl": 0, "CcStat_D_Actl": 3}] in eng["rows"]
  assert rec["chg"]["pt:DesiredTorqBrk"]["in"]["CcDis_B_Cmd"] == 0      # the ABS cruise-disable command, now visible
  assert [0.05, {"AccTrgDist2_D_Dsply": 2}] in rec["chg"]["cam:ACCDATA_3"]["rows"]
  assert rec["chg"]["cs"]["in"]["ccEn"] is True and [0.0, {"ccEn": False}] in rec["chg"]["cs"]["rows"]
  assert [0.0, {"opEn": False}] in rec["chg"]["cc"]["rows"]
  assert [0.0, {"names": ["pcmDisable"]}] in rec["chg"]["ev"]["rows"]
  # Rule 2: a registered message that never arrived is null WITH a reason, not zeros
  assert rec["chg"]["cam:IPMA_Data"] == {"in": None, "inWhy": "never received", "rows": []}
  assert rec["rx"]["cam:IPMA_Data"] == {"ageMs": None, "why": "never received"}
  assert rec["rx"]["pt:EngBrakeData"]["ageMs"] <= 10
  assert set(rec["notRegistered"]) == EXPECTED_NOT_REGISTERED
  cols = rec["trace"]["cols"]
  rows = rec["trace"]["rows"]
  assert cols[:2] == ["dt", "vEgo"] and "ACCDATA.AccPrpl_A_Rq" in cols
  assert rows[0][0] <= -2.9 and rows[-1][0] >= 0.9 and len(rows) >= 40
  assert [r for r in rows if r[0] == 0.0], "the edge tick itself is in the trace"
  assert rec["tx083"] == []


def test_every_state_change_emits_exactly_one_record_and_steady_state_none():
  b = Bench()
  b.cruise(0)
  b.run(10.0)
  for stat in (3, 5, 3, 0, 1):                 # off->standby->active->standby->off->fault
    b.cruise(stat)
    b.run(2.0)
  b.run(2.0)
  assert [r["edge"] for r in b.parsed()] == ["off->standby", "standby->active", "active->standby", "standby->off", "off->fault"]
  b.run(30.0)
  assert len(b.records) == 5


def _cancel_frames():
  """What the carcontroller sends every tick while CC.cruiseControl.cancel holds: 0x083 on bus 2 AND bus 0."""
  packer = CANPacker("ford_lincoln_base_pt")
  stock = dict.fromkeys(packer.dbc.name_to_msg["Steering_Data_FD1"].sigs, 0)
  return [fordcan.create_button_msg(packer, 2, stock, cancel=True), fordcan.create_button_msg(packer, 0, stock, cancel=True)]


def test_tx083_covers_the_whole_window_during_a_100hz_cancel():
  b = Bench()
  b.steady_active()
  b.run(4.0, sends=_cancel_frames(), every_tick=True)
  b.cruise(3)
  b.run(1.2, sends=_cancel_frames(), every_tick=True)
  (rec,) = b.parsed()
  tx = rec["tx083"]
  assert tx[0]["t"] <= -2.99 and tx[-1]["t"] >= 0.99, (tx[0]["t"], tx[-1]["t"])
  assert len(tx) >= 2 * 399 and all("CcAslButtnCnclPress" in f["bits"] for f in tx)
  assert "tx083Why" not in rec


def test_tx083_truncation_is_stated_not_silent(monkeypatch):
  monkeypatch.setattr(adp, "TX_MAXLEN", 100)          # a buffer smaller than the window's frames
  b = Bench()
  b.steady_active()
  b.run(4.0, sends=_cancel_frames(), every_tick=True)
  b.cruise(3)
  b.run(1.2, sends=_cancel_frames(), every_tick=True)
  (rec,) = b.parsed()
  assert rec["tx083"][0]["t"] > -3.0 and "truncated" in rec["tx083Why"], rec.get("tx083Why")


def test_no_spurious_record_while_can_is_not_yet_valid():
  """card's first ticks run before CAN decodes: CarState reads "off". Seeding the edge detector from that made
  one off->active record per card start."""
  b = Bench()
  b.cs.canValid = False
  b.cruise(0)
  b.run(1.0)
  b.cs.canValid = True
  b.steady_active()
  b.run(3.0)
  assert b.records == []
  b.cruise(3)                                          # a real edge afterwards is still recorded
  b.run(1.5)
  assert [r["edge"] for r in b.parsed()] == ["active->standby"]


def test_tx083_names_the_bits_and_who_asked():
  class FakeCC:
    _icbm_cmd = IcbmCommand(target_ms=26.8, ceiling_ms=26.8, ts=1.0)
    _sa_cmd = None                              # no _resume_cmd attribute at all -> explicit n/a reason

  b = Bench(car_controller=FakeCC())
  packer = CANPacker("ford_lincoln_base_pt")
  stock = dict.fromkeys(packer.dbc.name_to_msg["Steering_Data_FD1"].sigs, 0)
  frame = fordcan.create_button_msg(packer, 2, stock, set_dec=True)
  b.steady_active()
  b.run(3.0)
  b.run(0.01, sends=[frame])
  b.cruise(3)
  b.run(1.2)
  (rec,) = b.parsed()
  (tx,) = rec["tx083"]
  assert tx["bus"] == 2 and "CcAslButtnSetDecPress" in tx["bits"] and tx["hex"] == bytes(frame[1]).hex()
  assert tx["icbm"]["target_ms"] == 26.8 and tx["sa"] is None
  assert tx["resume"] == "n/a:carcontroller has no _resume_cmd"
  assert tx["ccCancel"] is False and tx["t"] == -0.01


def test_flapping_is_rate_limited_counted_and_warned(mocker):
  warn = mocker.patch.object(adp.cloudlog, "warning")
  b = Bench()
  b.steady_active()
  b.run(1.0)
  for i in range(15):
    b.cruise(3 if i % 2 == 0 else 5)
    b.run(0.1)
  b.run(61.0)
  b.cruise(3 if b.cs.cruiseState.enabled else 5)
  b.run(1.5)
  recs = b.parsed()
  assert len(recs) == adp.MAX_EVENTS_PER_MIN + 1
  assert recs[-1]["suppressed"] == 15 - adp.MAX_EVENTS_PER_MIN
  assert any("suppressing" in str(c) for c in warn.call_args_list)


def test_write_failure_is_loud(mocker):
  exc = mocker.patch.object(adp.cloudlog, "exception")

  def boom(line):
    raise OSError("disk full")

  b = Bench(write_fn=boom)
  b.steady_active()
  b.run(3.5)
  b.cruise(3)
  b.run(1.5)
  assert exc.call_count == 1 and "NOT explained" in exc.call_args[0][0]


def test_nan_becomes_null_never_invalid_json():
  b = Bench()
  b.steady_active()
  b.cs.vEgo = float("nan")
  b.run(3.5)
  b.cruise(3)
  b.run(1.5)
  (line,) = b.records
  assert "NaN" not in line and json.loads(line)["trace"]["rows"][0][1] is None


def test_route_lookup_nulls_carry_a_reason(tmp_path, mocker, monkeypatch):
  wall = time.time  # noqa: TID251 -- segment directories are stamped in wall time, so the edge must be too
  mocker.patch("openpilot.system.hardware.hw.Paths.log_root", return_value=str(tmp_path))
  route = {"v": None}
  monkeypatch.setitem(sys.modules, "openpilot.common.params",
                      SimpleNamespace(Params=lambda: SimpleNamespace(get=lambda k: route["v"] if k == "CurrentRoute" else None)))
  assert adp.current_route_segment(wall()) == {"route": None, "seg": None, "routeWhy": "CurrentRoute param unset (loggerd not recording?)"}
  route["v"] = "00000133--b4cc0b5efd"
  out = adp.current_route_segment(wall())
  assert out["seg"] is None and "no 00000133--b4cc0b5efd--N directory" in out["segWhy"]
  (tmp_path / "00000133--b4cc0b5efd--347").mkdir()
  (tmp_path / "00000133--b4cc0b5efd--348").mkdir()
  out = adp.current_route_segment(wall() + 5)
  assert out["route"] == "00000133--b4cc0b5efd" and out["seg"] == 348 and out["edgeInSegS"] >= 4
  assert out["segsToKeep"] == [348]
  assert out["routeStale"] is False and "routeStaleWhy" not in out
  # parked with loggerd stopped (parknorec2pnw): CurrentRoute still names the old route, no new segment appears
  out = adp.current_route_segment(wall() + 120)
  assert out["route"] == "00000133--b4cc0b5efd" and out["routeStale"] is True
  assert "loggerd was not recording" in out["routeStaleWhy"]
  out = adp.current_route_segment(wall() + adp.SEGMENT_S + 5)   # a segment about to roll is NOT stale
  assert out["routeStale"] is False
  # ...nor one loggerd has not rotated yet because an encoder stalled: its hard fallback is SEGMENT_LENGTH*1.2
  out = adp.current_route_segment(wall() + adp.SEGMENT_S * 1.2)
  assert out["routeStale"] is False, out
  out = adp.current_route_segment(wall() + 76)                  # past STALE_ROUTE_S = 75 s: stale
  assert out["routeStale"] is True, out
  out = adp.current_route_segment(wall() + 1.2)       # Sun 14:05:46: edge 1.2 s into seg 348
  assert out["segsToKeep"] == [347, 348]
  mocker.patch("openpilot.system.hardware.hw.Paths.log_root", side_effect=OSError("gone"))
  assert adp.current_route_segment(wall())["segWhy"] == "lookup failed: OSError: gone"


def test_log_path_is_the_ces_events_log():
  from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import CES_EVENT_LOG
  assert adp.CES_EVENT_LOG == CES_EVENT_LOG


@pytest.mark.parametrize("n", [3000])
def test_update_cost_is_bounded(n):
  b = Bench()
  b.steady_active()
  b.run(1.0)
  cs, sm, log = b.cs, b.sm, b.log
  t = b.t
  start = time.perf_counter()
  for i in range(n):
    log.update(cs, sm, (), t + i * 0.01)
  per_tick = (time.perf_counter() - start) / n
  print(f"accdrop update(): {per_tick * 1e6:.1f} us/tick on this host")
  assert per_tick < 250e-6 and math.isfinite(per_tick)
