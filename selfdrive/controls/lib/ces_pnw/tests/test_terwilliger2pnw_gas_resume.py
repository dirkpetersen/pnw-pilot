"""terwilliger2pnw fix 2: a driver GAS press suspends an ICBM episode instead of ending it; the restore resumes after the lift.

2026-09-24 12:36:13.5 PT, Terwilliger Curves (drives/2026-09-24/terwilliger-too-slow): ICBM had walked the set 60 -> 48
for curve B. The owner pressed the accelerator mid-curve, which hard-aborted the episode, so no restore ever ran: the set
sat at 48 for 44 s on open road until he pressed SET+ 14 times. Owner: "yes restore".

Pinned here, each by a test that fails if the rule is broken:
  * the Terwilliger shape: gas mid-cap, lift, curve clears -> the ordinary restore back to the pre-curve ceiling (60),
    labelled icbmRestoreWhy "afterGas";
  * every restore guard still applies after the lift: the curve-clear debounce, the in-curve deferral, the posted-limit
    zone cap (a limit drop DURING the press included), the curve-ahead hold, never above the ceiling;
  * no restore when the driver took the set over: SET+/RES/SET- during or after the press, brake, ACC off, a set already
    at the ceiling, a press longer than the restore window;
  * one late SET- tap of our own landing after the press is absorbed, a second is the driver;
  * change-only logging, never per tick; the Tesla (gas_resume False) exactly as before; the fields reach the record.
"""
import math

import pytest

from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import (ICBM_EXEC_STEP_MS, ICBM_RATCHET_CONFIRM_S,
                                                              ICBM_RESTORE_DELAY_S, ICBM_RESTORE_WINDOW_S, IcbmEpisode)
from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_ces_record_fields import _record

MPH = 0.44704
T0 = 1000.0
DT = 0.25
CEIL, SET, CAP = 60 * MPH, 48 * MPH, 48.4 * MPH   # Terwilliger curve B: pre-curve set 60, walked to 48, cap 48.4


@pytest.fixture
def events(monkeypatch):
  ev = []
  monkeypatch.setattr(m.cloudlog, "event", lambda name, **kw: ev.append((name, kw)))
  return ev


class Drive:
  """A pure IcbmEpisode driven at the brain's 4 Hz, the Lightning's way (gas_resume=True)."""

  def __init__(self, gas_resume=True, limit=50 * MPH):
    self.ep = IcbmEpisode()
    self.t = T0
    self.gas_resume = gas_resume
    self.limit = limit
    self.out = []

  def tick(self, cap=None, stock=SET, gas=False, brake=False, on=True, n=1, **kw):
    for _ in range(n):
      self.ep.note_limit(self.limit, proportional=False)
      o = self.ep.step(self.t, cap, stock, stock, on, gas or brake, limit_now=self.limit,
                       gas_resume=self.gas_resume, driver_gas=gas and not brake, **kw)
      self.out.append(o)
      self.t += DT
    return self.out[-1]

  def secs(self, s, **kw):
    return self.tick(n=max(1, round(s / DT)), **kw)


def _capped(gas_resume=True, limit=50 * MPH):
  """ceiling 60 latched by a cap; the set walked down to 48 (Terwilliger curve B)."""
  d = Drive(gas_resume, limit)
  d.tick(cap=CAP, stock=CEIL)
  d.secs(ICBM_RATCHET_CONFIRM_S + 1.0, cap=CAP, stock=SET)
  assert d.ep.phase == "cap" and d.ep.ceiling == pytest.approx(CEIL)
  return d


def _restores(d, **kw):
  """clear, sustained: returns every restore publish over the debounce + a few seconds."""
  d.secs(ICBM_RESTORE_DELAY_S + 2.0, **kw)
  return [o for o in d.out if o[1] == "inc"]


# =====================================================================================================
# the Terwilliger shape
# =====================================================================================================
class TestTheTerwilligerShape:
  def test_gas_mid_cap_then_lift_restores_to_the_pre_curve_ceiling(self, events):
    d = _capped()
    assert d.tick(cap=CAP, gas=True) == (CAP, "dec")        # the cap is still FORWARDED, dec-only, as before
    assert d.ep.phase == "gas" and d.ep.ceiling == pytest.approx(CEIL) and d.ep.gas_res == "hold"
    d.secs(5.5, gas=True)                                   # 12:36:13.5 -> 12:36:19, on the power
    d.secs(4.0, cap=CAP)                                    # lifted, still in curve B: the cap binds again
    assert d.ep.phase == "cap" and d.ep.ceiling == pytest.approx(CEIL)
    inc = _restores(d)                                      # curve B behind: the ordinary restore
    assert inc and all(o[0] == pytest.approx(CEIL) for o in inc), inc
    assert d.ep.phase == "restore" and d.ep.restore_why == "afterGas"
    states = [kw["state"] for n, kw in events if n == "ces_icbm_gas_resume"]
    assert states == ["hold", "resumed", "restore"], states

  def test_without_the_capability_the_gas_still_ends_everything(self, events):
    """The Tesla, and any caller that does not pass gas_resume: byte-for-byte the previous behaviour."""
    d = _capped(gas_resume=False)
    assert d.tick(cap=CAP, gas=True) == (CAP, "dec")
    assert d.ep.phase == "idle" and d.ep.ceiling is None
    d.secs(1.0)
    assert not _restores(d)
    assert not [e for e in events if e[0] == "ces_icbm_gas_resume"]

  def test_gas_during_a_running_restore_resumes_it_after_the_lift(self, events):
    d = _capped()
    inc = _restores(d)
    assert inc and d.ep.phase == "restore"
    d.secs(2.0, gas=True)
    assert d.ep.phase == "gas" and d.out[-1] == (None, None)    # nothing is published while on the power
    d.out.clear()
    inc = _restores(d)
    assert inc and all(o[0] == pytest.approx(CEIL) for o in inc)
    assert d.ep.restore_why == "afterGas"

  def test_the_restore_is_the_normal_one_debounced_and_never_above_the_ceiling(self):
    d = _capped()
    d.secs(2.0, gas=True)
    d.out.clear()
    d.secs(ICBM_RESTORE_DELAY_S - 0.5)                      # lifted, clear, but inside the curve-clear debounce
    assert not [o for o in d.out if o[1] == "inc"], "the restore skipped the curve-clear debounce"
    inc = _restores(d)
    assert inc and max(o[0] for o in inc) <= CEIL + 1e-9


# =====================================================================================================
# every restore guard still applies after the lift
# =====================================================================================================
class TestTheRestoreGuardsStillApply:
  def test_no_restore_while_still_loaded_in_the_curve(self):
    d = _capped()
    d.secs(2.0, gas=True)
    d.out.clear()
    d.secs(ICBM_RESTORE_DELAY_S + 3.0, in_curve=True)
    assert not [o for o in d.out if o[1] == "inc"], "raised the set mid-curve"
    assert d.ep.phase == "cap"

  def test_a_limit_drop_during_the_press_caps_the_restore(self):
    """A 50 -> 45 zone entered while on the power: the restore stops at 45 + 5 (icbm_stale_zone_cap), not 60."""
    d = _capped()
    d.secs(1.0, gas=True)
    d.limit = 45 * MPH
    d.secs(2.0, gas=True)
    assert d.ep.zone_cap is not None, "the zone change during the press was not folded in"
    rcap = d.ep.zone_cap
    inc = _restores(d, restore_cap=rcap)
    assert rcap == pytest.approx(50 * MPH) and inc and all(o[0] == pytest.approx(rcap) for o in inc), (rcap, inc)

  def test_the_curve_ahead_hold_holds_it(self):
    d = _capped()
    d.secs(2.0, gas=True)
    d.out.clear()
    inc = _restores(d, hold_ahead=True, v_ego=SET)
    assert d.ep.ahead_cap is not None
    assert all(o[0] <= d.ep.ahead_cap + 1e-9 for o in inc), "the restore ignored the curve ahead"


# =====================================================================================================
# no restore when the driver has taken the set over
# =====================================================================================================
class TestTheDriverOwnsTheSet:
  @pytest.mark.parametrize("moved", [1.0, 5.0, 12.0, -3.0], ids=["SET+", "hold SET+", "SET at speed 60", "SET- x3"])
  def test_a_set_change_during_the_press_declines(self, events, moved):
    d = _capped()
    d.secs(1.0, gas=True)
    d.secs(2.5, gas=True, stock=SET + moved * MPH)
    assert d.ep.phase == "idle" and d.ep.gas_res == "no:driverSet"
    d.out.clear()
    assert not _restores(d, stock=SET + moved * MPH)
    assert [kw["why"] for n, kw in events if n == "ces_icbm_gas_resume" and kw["state"] == "declined"] == ["driverSet"]

  def test_SET_plus_after_the_lift_before_the_restore_declines(self, events):
    """The Terwilliger ending: he pressed SET+ himself. Never restore past him."""
    d = _capped()
    d.secs(2.0, gas=True)
    d.secs(1.0)
    d.tick(stock=SET + 1 * MPH)
    assert d.ep.phase == "idle" and d.ep.gas_res == "no:driverSet"
    d.out.clear()
    assert not _restores(d, stock=SET + 1 * MPH)

  def test_the_brake_ends_everything_as_before(self, events):
    d = _capped()
    d.secs(1.0, gas=True)
    d.tick(gas=True, brake=True)
    assert d.ep.phase == "idle" and d.ep.ceiling is None and d.ep.gas_res == "no:brake"
    d.out.clear()
    assert not _restores(d)

  def test_the_brake_after_the_lift_ends_everything(self):
    d = _capped()
    d.secs(1.0, gas=True)
    d.secs(1.0)
    d.tick(brake=True)
    assert d.ep.phase == "idle" and d.ep.gas_res == "no:brake"

  def test_acc_off_during_the_press_ends_everything(self):
    d = _capped()
    d.secs(1.0, gas=True)
    d.tick(gas=True, on=False)
    assert d.ep.phase == "idle" and d.ep.gas_res == "no:accOff"
    d.out.clear()
    assert not _restores(d)

  def test_a_set_already_at_the_ceiling_is_nothing_to_restore(self, events):
    d = Drive()
    d.tick(cap=CAP, stock=CEIL)
    d.secs(ICBM_RATCHET_CONFIRM_S + 0.5, cap=CAP, stock=CEIL)   # the executor had not tapped yet
    d.tick(cap=CAP, stock=CEIL, gas=True)
    assert d.ep.phase == "idle" and d.ep.gas_res == "no:atCeiling"

  def test_a_press_longer_than_the_restore_window_declines(self):
    d = _capped()
    d.secs(ICBM_RESTORE_WINDOW_S + 1.0, gas=True)
    assert d.ep.phase == "idle" and d.ep.gas_res == "no:gasLong"

  def test_our_own_late_SET_minus_tap_is_absorbed_once(self):
    """The executor stops pressing the instant the pedal goes down, but its last tap is reported ~1 s late."""
    d = _capped()
    d.tick(gas=True)
    d.tick(gas=True, stock=SET - ICBM_EXEC_STEP_MS)          # one tap, inside the grace: ours
    assert d.ep.phase == "gas"
    d.secs(2.0, gas=True, stock=SET - ICBM_EXEC_STEP_MS)
    d.out.clear()
    assert _restores(d, stock=SET - ICBM_EXEC_STEP_MS)
    d2 = _capped()
    d2.tick(gas=True)
    d2.tick(gas=True, stock=SET - ICBM_EXEC_STEP_MS)
    d2.tick(gas=True, stock=SET - 2 * ICBM_EXEC_STEP_MS)     # a second step: the driver
    assert d2.ep.phase == "idle" and d2.ep.gas_res == "no:driverSet"


# =====================================================================================================
# logging and telemetry
# =====================================================================================================
def test_logging_is_change_only(events):
  d = _capped()
  d.secs(20.0, gas=True)                                    # 80 ticks on the power
  d.secs(1.0)
  d.secs(ICBM_RESTORE_DELAY_S + 20.0)
  n = [kw["state"] for name, kw in events if name == "ces_icbm_gas_resume"]
  assert n == ["hold", "resumed", "restore"], n


def test_a_resumed_restore_logs_how_it_ended(events):
  d = _capped()
  d.secs(1.0, gas=True)
  _restores(d)
  d.tick(stock=CEIL)                                        # the executor got there
  assert d.ep.phase == "idle"
  assert [kw["state"] for name, kw in events if name == "ces_icbm_gas_resume"][-1] == "done"


def test_the_fields_reach_the_record():
  ep = IcbmEpisode()
  ep.phase, ep.restore_why, ep.gas_res = "restore", "afterGas", "resumed"
  rec = _record(_icbm_ep=ep)
  assert rec["icbmPhase"] == "restore" and rec["icbmRestoreWhy"] == "afterGas" and rec["icbmGas"] == "resumed"


# =====================================================================================================
# the controller: the capability, not the fingerprint
# =====================================================================================================
@pytest.mark.parametrize("fp,brand,want", [("FORD_F_150_LIGHTNING_MK1", "ford", "gas"), ("TESLA_MODEL_S_HW3", "tesla", "idle")])
def test_the_controller_passes_the_capability(tmp_path, monkeypatch, fp, brand, want):
  import time as _t
  from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_restorehold2pnw import _stub_mgr
  mgr, step = _stub_mgr(tmp_path, monkeypatch, fp=fp, brand=brand)

  def run(sig, stock):
    mgr._stock_set, mgr._stock_on = stock, True
    mgr._icbm_last_pub = _t.monotonic() - 1.0
    step(sig, active=True)
    return mgr.mem_params.last

  cap_sig = {"v_ego": 60 * MPH, "v_set": 60 * MPH, "map_target_v": 30 * MPH, "map_target_dist": 40.0,
             "curve_lat_accel_vision": 0.0, "time_to_curve": 10.0, "gas": False, "brake": False}
  assert run(cap_sig, 60 * MPH).get("target") is not None
  assert mgr._icbm_ep.phase == "cap"
  run({**cap_sig, "v_set": 48 * MPH, "v_ego": 48 * MPH, "gas": True}, 48 * MPH)
  assert mgr._icbm_ep.phase == want
  assert (mgr._icbm_ep.ceiling is not None) == (want == "gas")
  assert math.isfinite(mgr._icbm_ep.ceiling or 0.0)
