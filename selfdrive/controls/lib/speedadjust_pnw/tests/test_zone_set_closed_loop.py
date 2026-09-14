"""sazoneset2pnw — both button brains against the REAL Ford executor, closed loop.

speedadjust (SpeedAdjustTarget) and the ICBM curve episode (IcbmTarget) are ticked at 4 Hz like the car and
merged by the real opendbc arbitrate() / decide_press() / RestoreGuard / PressGovernor at 100 Hz. The truck's
stock set answers each press 0.3 s later by +/-1 mph. Adapted from Fable's review harness (scen_cross), whose
S1..S3 scenarios are the regression targets.

The defect these pin (Fable, measured on sanorestore2pnw): speedadjust held its limit-drop dec for the WHOLE
zone, re-published every 0.25 s, so arbitrate() never let a curve restore run inside the zone -- a curve in a
long trimmed zone left the set at 35 mph on a 60 road until the driver tapped. The driver's model (2026-09-13):
zone entry sets his percentage once and forgets, a curve returns to the speed before the curve, and nothing
restores when the zone ends.
"""
import json
import types

import pytest

import openpilot.selfdrive.controls.lib.speedadjust_pnw.speedadjust_controller as sa
from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import IcbmEpisode
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw_constants as C
from opendbc.car.ford.icbm_pnw import arbitrate, decide_press, RestoreGuard, PressGovernor, IcbmCommand

MPH = sa.MPH_TO_MS


class _Clock:
  t = 1000.0

  @classmethod
  def monotonic(cls):
    return cls.t

  @classmethod
  def time(cls):
    return cls.t


class _Mem:
  def __init__(self):
    self.sl, self.police, self.calls = 0.0, None, []

  def get(self, k, return_default=True):
    if k == "MapSpeedLimit":
      return str(self.sl) if self.sl else ""
    if k == "LocationServices":
      return json.dumps({"police": self.police}) if self.police else "{}"
    return None

  def put_nonblocking(self, k, v):
    self.calls.append((k, v))


class _P:
  def __init__(self, mode):
    self.mode = mode

  def get(self, k, return_default=True):
    return str(self.mode) if k == "AutoSpeedReduce" else None


class _SM:
  def __init__(self):
    self.cs = types.SimpleNamespace(gasPressed=False, brakePressed=False,
                                    cruiseState=types.SimpleNamespace(speed=0.0, enabled=True))

  def __getitem__(self, k):
    if k == "carState":
      return self.cs
    raise KeyError(k)


def _cmd(d):
  if isinstance(d, dict) and all(k in d for k in ("target", "ceiling", "ts")):
    return IcbmCommand(d["target"], d["ceiling"], d["ts"], d.get("dir", "dec"))
  return None


def simulate(monkeypatch, script, T, v0_mph=75, mode=2):
  """script(t) -> (limit_mph, curve_target_mph or None, police or None). Returns (final set mph, trace)."""
  _Clock.t = 1000.0
  monkeypatch.setattr(sa, "time", _Clock)
  c = sa.SpeedAdjustController(types.SimpleNamespace(openpilotLongitudinalControl=False), params=_P(mode))
  c.mem_params = _Mem()
  sm, ep, guard, gov = _SM(), IcbmEpisode(), RestoreGuard(), PressGovernor()
  stock, pending, rcap_state = v0_mph * MPH, [], None
  sa_cmd = icbm_cmd = None
  last_dec_ts, frame, trace = None, 0, []
  while _Clock.t - 1000.0 < T:
    frame += 1
    _Clock.t += 0.01
    now = _Clock.t
    lim, curve, pol = script(now - 1000.0)
    for p in list(pending):
      if now >= p[0]:
        stock += p[1]
        pending.remove(p)
    sm.cs.cruiseState.speed = stock
    if frame % 25 == 0:                                   # both brains at 4 Hz
      c.mem_params.sl, c.mem_params.police = lim * MPH, pol
      c._last_read = -1e9
      c.cap(sm, stock, stock, stock, True)
      rl, rcap_state = C.icbm_restore_limit(lim * MPH, rcap_state, now)
      ep.note_limit(rl if rl > 0 else None, mode >= 2)
      tgt, d = ep.step(now, curve * MPH if curve else None, stock, stock, True, False,
                       restore_cap=ep.zone_cap, limit_now=rl if rl > 0 else None)
      pub = {}
      if tgt is not None:
        pub = {"target": tgt, "ceiling": ep.ceiling if ep.ceiling is not None else stock, "ts": now}
        if d == "inc":
          pub["dir"] = "inc"
      icbm_cmd = _cmd(pub)
      tc = [v for k, v in c.mem_params.calls if k == "SpeedAdjustTarget"]
      sa_cmd = _cmd(tc[-1] if tc else None)
    cmd = arbitrate([icbm_cmd, sa_cmd], now, last_dec_ts)
    if cmd is not None and cmd.dir == "dec":
      last_dec_ts = now
    intent = decide_press(stock, cmd, now, True, False)
    pinc = arbitrate([x for x in (icbm_cmd, sa_cmd) if x is not None and x.dir == "inc"], now)
    intent = guard.filter(intent, stock, now, pinc is not None, pinc.ceiling_ms if pinc else None,
                          cmd is not None and cmd.dir == "dec")
    was = gov._active
    btn = gov.update(frame, intent)
    if btn and was is None:
      pending.append((now + 0.3, (1 if btn == "inc" else -1) * MPH))
    if frame % 100 == 0:
      trace.append((round(now - 1000.0), round(stock / MPH, 1), lim, curve, ep.phase))
  return stock / MPH, trace, c


def _S1(t):
  return (60 if t < 5 or t >= 70 else 45), (35 if 30 <= t < 42 else None), None


def _S1L(t):
  return (60 if t < 5 or t >= 130 else 45), (35 if 30 <= t < 42 else None), None


def _S2(t):
  return (45 if 10 <= t < 45 else 60), (35 if 5 <= t < 25 else None), None


def _S2L(t):
  return (45 if 10 <= t < 120 else 60), (35 if 5 <= t < 25 else None), None


POLICE = {"state": "alert", "dist_mi": 0.3, "tier": "confirmed", "cap": {"state": "alert", "dist_mi": 0.3, "key": "k"}}


def _S3(t):
  return 60, None, (POLICE if 5 <= t < 30 else None)


ZONE = 45 * 75 / 60                                       # 75 on a 60 (+25%) into a 45 zone


class TestTheDriversZoneModel:
  def test_S1L_a_curve_inside_a_LONG_zone_returns_to_the_zone_speed(self, monkeypatch):
    """THE REGRESSION. Before: 35 mph on a 60 road, permanently (curve restore starved by speedadjust)."""
    final, trace, c = simulate(monkeypatch, _S1L, 170)
    assert abs(final - ZONE) <= 1.0, f"final set {final:.1f} mph, want the zone speed ~{ZONE:.1f}\n{trace}"
    assert c._no_restore_why == "zoneSet"

  def test_S1_the_zone_ending_does_not_restore(self, monkeypatch):
    final, trace, _ = simulate(monkeypatch, _S1, 110)
    assert abs(final - ZONE) <= 1.0, f"final {final:.1f}\n{trace}"

  @pytest.mark.parametrize("script,T", [(_S2, 110), (_S2L, 170)])
  def test_S2_a_zone_that_starts_DURING_a_curve_restores_to_the_zone_speed_and_stays(self, monkeypatch, script, T):
    """Curve latched at 75 on a 60; the limit drops mid-curve, so that ceiling is stale: the restore stops at
    the driver's percentage of the new limit and a later limit rise does not resume it. Before: 65 / 50."""
    final, trace, _ = simulate(monkeypatch, script, T)
    assert abs(final - ZONE) <= 1.0, f"final {final:.1f}\n{trace}"

  def test_S3_a_police_slowdown_still_restores(self, monkeypatch):
    final, trace, _ = simulate(monkeypatch, _S3, 110)
    assert abs(final - 75) <= 1.0, f"final {final:.1f}\n{trace}"

  @pytest.mark.parametrize("mode,bound", [(2, 25 * 60 / 45), (1, 25 + 5)])
  def test_the_1546_shape_never_restores_toward_the_old_set(self, monkeypatch, mode, bound):
    """Set 60 on a 45 road, a sharp curve, the limit drops to 25 while the curve is running. The restore is
    capped at the zone speed (mode 2: 60/45 of 25) or limit + 5 (mode 1), and never approaches 60."""
    def script(t):
      return (45 if t < 12 else 25), (15 if 3 <= t < 20 else None), None
    final, trace, _ = simulate(monkeypatch, script, 90, v0_mph=60, mode=mode)
    in_zone_after_curve = [s for (t, s, lim, cv, ph) in trace if t >= 22]
    assert max(in_zone_after_curve) <= bound + 1.0, f"set reached {max(in_zone_after_curve):.1f}, bound {bound:.1f}\n{trace}"
