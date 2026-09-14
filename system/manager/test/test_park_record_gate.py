"""parknorec2pnw: loop tests for "the device never records while the shifter is in Park".

Every sequence drives the REAL manager path tick by tick: process_config's loggerd should_run (which
calls the gate) through system/manager/process.ensure_running, with real Params, card's real GearPark
writer rules, and the real Tesla / Lightning gear decode. Only three things are faked: the clock, the card
process's liveness, and loggerd's start/stop (counted instead of spawning a binary).
"""
from types import SimpleNamespace

import pytest

from cereal import car
from openpilot.common.params import Params, UnknownKeyName
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.car.gear_park import GearParkWriter
from openpilot.selfdrive.car.tests.test_gear_park import lightning_gear, tesla_gear
import openpilot.system.manager.process_config as pc
from openpilot.system.manager.park_record_gate import PARK_HOLD_S, ParkRecordGate
from openpilot.system.manager.process import ensure_running

GearShifter = car.CarState.GearShifter
TICK = 0.5   # manager loop: sm.update() on deviceState at 2 Hz


class Rig:
  def __init__(self, monkeypatch):
    self.t = 0.0
    self.started = True
    self.card = SimpleNamespace(alive=True)
    self.params = Params()
    self.CP = car.CarParams.new_message()
    self.events = []
    self.loggerd_running = False
    self.starts = 0
    self.stops = 0
    self.first_start_t = []

    monkeypatch.setattr(cloudlog, "event", lambda name, *a, **kw: self.events.append((self.t, name, kw)))
    monkeypatch.setattr(pc.managed_processes["card"], "proc",
                        SimpleNamespace(is_alive=lambda: self.card.alive, exitcode=None, pid=1))
    # a fresh gate on the fake clock, wired to the REAL _card_alive (reads managed_processes["card"])
    monkeypatch.setattr(pc, "PARK_RECORD_GATE", ParkRecordGate(pc._card_alive, clock=lambda: self.t))
    self.loggerd = pc.managed_processes["loggerd"]
    monkeypatch.setattr(self.loggerd, "start", self._start)
    monkeypatch.setattr(self.loggerd, "stop", self._stop)

  def _start(self):
    if not self.loggerd_running:
      self.loggerd_running = True
      self.starts += 1
      self.first_start_t.append(self.t)

  def _stop(self, retry=True, block=True, sig=None):
    if self.loggerd_running:
      self.loggerd_running = False
      self.stops += 1

  def tick(self):
    ensure_running([self.loggerd], self.started, params=self.params, CP=self.CP, not_run=[])
    self.t += TICK

  def hold_for(self, seconds, gear_park=None):
    """Run the manager for `seconds` with GearPark as given (None = leave the param untouched)."""
    if gear_park is not None:
      self.params.put_bool("GearPark", gear_park)
    end = self.t + seconds
    while self.t < end - 1e-9:
      self.tick()

  def recorded_while(self, seconds, gear_park=None):
    """Like hold_for, but returns the fraction of ticks loggerd was running."""
    if gear_park is not None:
      self.params.put_bool("GearPark", gear_park)
    n = running = 0
    end = self.t + seconds
    while self.t < end - 1e-9:
      self.tick()
      n += 1
      running += self.loggerd_running
    return running / n


@pytest.fixture
def rig(monkeypatch):
  return Rig(monkeypatch)


def gate_events(rig, name="park_record_gate"):
  return [e for e in rig.events if e[1] == name]


class TestSequences:
  def test_charging_session_stops_recording_after_the_hold(self, rig):
    rig.hold_for(10, gear_park=False)                     # driving
    assert rig.loggerd_running and rig.starts == 1
    rig.params.put_bool("GearPark", True)                 # shifted into Park
    t_park = rig.t
    rig.hold_for(PARK_HOLD_S - 1)
    assert rig.loggerd_running                            # still inside the hysteresis
    rig.hold_for(2)
    assert not rig.loggerd_running and rig.stops == 1
    assert rig.recorded_while(3600) == 0.0                # an hour charging: nothing recorded
    assert rig.stops == 1 and rig.starts == 1
    holds = [e for e in gate_events(rig) if e[2]["hold"]]
    assert len(holds) == 1 and holds[0][0] - t_park == pytest.approx(PARK_HOLD_S, abs=TICK)

  def test_leaving_park_starts_loggerd_on_the_next_tick(self, rig):
    rig.hold_for(PARK_HOLD_S + 5, gear_park=True)
    assert not rig.loggerd_running
    rig.params.put_bool("GearPark", False)
    t_shift = rig.t
    rig.tick()
    assert rig.loggerd_running and rig.first_start_t[-1] - t_shift <= TICK
    assert [e for e in gate_events(rig) if not e[2]["hold"]][-1][2]["reason"] == "not_in_park"

  def test_parking_lot_shuffle_neither_restarts_nor_splits(self, rig):
    rig.hold_for(5, gear_park=False)
    for park_s, drive_s in [(20, 10), (25, 5), (29, 3), (10, 1), (28, 8)] * 4:
      rig.hold_for(park_s, gear_park=True)
      rig.hold_for(drive_s, gear_park=False)
    assert rig.starts == 1 and rig.stops == 0

  def test_repeated_long_parks_create_one_route_per_park_exit(self, rig):
    rig.hold_for(5, gear_park=False)
    for _ in range(5):
      rig.hold_for(PARK_HOLD_S + 60, gear_park=True)
      rig.hold_for(4, gear_park=False)
    assert rig.stops == 5 and rig.starts == 6              # bounded: never more than one per Park exit

  def test_ignition_on_in_park_records_the_hold_then_stops(self, rig):
    rig.started = False
    rig.card.alive = False                                # card is only_onroad
    rig.hold_for(60, gear_park=True)                      # stale True left from the last drive
    assert rig.starts == 0
    rig.started = True
    rig.tick()                                            # manager starts card in this same tick
    assert rig.loggerd_running
    rig.card.alive = True
    rig.hold_for(PARK_HOLD_S - 1)
    assert rig.loggerd_running
    rig.hold_for(2)
    assert not rig.loggerd_running

  def test_ignition_off_in_park_releases_as_offroad(self, rig):
    rig.hold_for(PARK_HOLD_S + 5, gear_park=True)
    rig.started = False                                   # card is still alive for this tick (shutting down)
    rig.tick()
    rig.card.alive = False
    rig.hold_for(120)
    releases = [e for e in gate_events(rig) if not e[2]["hold"]]
    assert [e[2]["reason"] for e in releases] == ["offroad"]
    assert [e for e in rig.events if e[2].get("error")] == []


class TestFailSafe:
  def test_card_dies_while_holding_records_and_says_so(self, rig):
    rig.hold_for(PARK_HOLD_S + 5, gear_park=True)
    assert not rig.loggerd_running
    rig.card.alive = False                                # card crashed; GearPark is stale
    rig.tick()
    assert rig.loggerd_running
    releases = [e for e in gate_events(rig) if not e[2]["hold"]]
    assert [e[2]["reason"] for e in releases] == ["card_not_running"]
    rig.hold_for(10)
    assert len([e for e in gate_events(rig) if not e[2]["hold"]]) == 1   # once, not per tick

  def test_restarted_card_must_rearm_the_full_hold(self, rig):
    rig.hold_for(PARK_HOLD_S + 5, gear_park=True)
    rig.card.alive = False
    rig.tick()
    rig.card.alive = True                                 # manager restarted card; param still stale True
    rig.hold_for(PARK_HOLD_S - 1)
    assert rig.loggerd_running

  def test_card_crash_loop_never_holds(self, rig):
    rig.params.put_bool("GearPark", True)
    for _ in range(200):
      rig.card.alive = True
      rig.hold_for(20)                                    # dies before a full hold every time
      rig.card.alive = False
      rig.tick()
    assert rig.stops == 0

  def test_car_that_never_writes_gear_park_records_forever(self, rig):
    assert rig.params.get("GearPark") is None             # never written by anyone
    assert rig.recorded_while(3600) == 1.0
    assert rig.starts == 1 and rig.stops == 0

  def test_kill_switch(self, rig):
    rig.hold_for(PARK_HOLD_S + 5, gear_park=True)
    assert not rig.loggerd_running
    rig.params.put_bool("RecordWhileParked", True)
    rig.tick()
    assert rig.loggerd_running
    assert rig.recorded_while(600) == 1.0
    rig.params.put_bool("RecordWhileParked", False)
    rig.hold_for(PARK_HOLD_S + 1)
    assert not rig.loggerd_running

  def test_params_error_records_and_logs(self, rig, monkeypatch):
    rig.hold_for(PARK_HOLD_S + 5, gear_park=True)
    assert not rig.loggerd_running
    logged = []
    monkeypatch.setattr(cloudlog, "exception", lambda msg, *a, **kw: logged.append(msg))
    real = rig.params

    class BrokenParams:
      def get_bool(self, key, block=False):
        if key == "RecordWhileParked":
          raise UnknownKeyName(key)                     # params_pyx.so older than params_keys.h
        return real.get_bool(key)

    rig.params = BrokenParams()
    rig.hold_for(5)
    assert rig.loggerd_running and len(logged) == 1
    releases = [e for e in gate_events(rig) if not e[2]["hold"]]
    assert [e[2]["reason"] for e in releases] == ["params_error"]


class TestEndToEndWithRealGearDecode:
  """card's writer rules -> GearPark param -> gate -> loggerd, fed by the real opendbc decode."""

  def _drive(self, rig, cs, seconds, writer, can_valid=None):
    valid = cs.canValid if can_valid is None else can_valid
    end = rig.t + seconds
    while rig.t < end - 1e-9:
      w = writer.update(cs.gearShifter, valid, rig.t)
      if w is not None:
        rig.params.put_bool("GearPark", w)
      rig.tick()

  def test_tesla_raven_parked(self, rig):
    w = GearParkWriter()
    rig.params.put_bool("GearPark", False)                # card's startup seed
    self._drive(rig, tesla_gear(4), 60, w, can_valid=True)          # Drive
    assert rig.loggerd_running
    self._drive(rig, tesla_gear(1), PARK_HOLD_S + 5, w, can_valid=True)   # Park
    assert not rig.loggerd_running
    self._drive(rig, tesla_gear(1), 1800, w, can_valid=True)
    assert not rig.loggerd_running and rig.stops == 1
    t_shift = rig.t
    self._drive(rig, tesla_gear(4), 5, w, can_valid=True)           # back into Drive
    assert rig.loggerd_running and rig.first_start_t[-1] - t_shift <= TICK

  def test_tesla_sna_while_parked_on_invalid_can_keeps_holding(self, rig):
    w = GearParkWriter()
    self._drive(rig, tesla_gear(1), PARK_HOLD_S + 5, w, can_valid=True)
    assert not rig.loggerd_running
    self._drive(rig, tesla_gear(7), 600, w, can_valid=False)        # DI_GEAR_SNA, CAN invalid
    assert not rig.loggerd_running and rig.stops == 1

  def test_lightning_quiet_can_charging_after_a_valid_park(self, rig):
    w = GearParkWriter()
    self._drive(rig, lightning_gear(0), 2, w, can_valid=True)       # shifted into Park, CAN awake
    self._drive(rig, lightning_gear(0), 3600, w, can_valid=False)   # charging: canValid false for hours
    assert not rig.loggerd_running and rig.stops == 1

  def test_lightning_dead_bus_reads_unknown_and_keeps_recording(self, rig):
    w = GearParkWriter()
    cs = lightning_gear(3, frames=0)                                # nothing received: unknown (gearunknown2pnw)
    assert cs.gearShifter == GearShifter.unknown
    self._drive(rig, cs, 3600, w)                                   # its own canValid: False
    assert rig.loggerd_running and rig.stops == 0

  def test_can_fault_across_park_to_drive_restarts_recording(self, rig):
    w = GearParkWriter()
    self._drive(rig, lightning_gear(0), PARK_HOLD_S + 5, w, can_valid=True)
    assert not rig.loggerd_running
    t_shift = rig.t
    self._drive(rig, lightning_gear(3), 60, w, can_valid=False)     # Drive decoded, CAN still invalid
    assert rig.loggerd_running and rig.first_start_t[-1] - t_shift <= TICK


def test_loggerd_is_the_only_process_gated():
  # camerad / modeld / encoderd / card must keep running in Park so openpilot is ready on the shift.
  assert pc.managed_processes["loggerd"].should_run is pc.logging
  gated = [p.name for p in pc.procs if getattr(p, "should_run", None) is pc.logging]
  assert gated == ["loggerd"]
