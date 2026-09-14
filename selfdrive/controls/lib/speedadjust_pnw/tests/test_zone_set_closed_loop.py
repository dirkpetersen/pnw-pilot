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

Wired the way the car is (sazoneset2pnw, Fable review of the first version): speedadjust reads the IcbmTarget
mem-param the executor reads, at its real 1 Hz limit-read cadence, and ICBM sees speedadjust only through its
SpeedAdjustStatus publish, re-read at ~1 Hz and folded in by ces_pnw.icbm_note_speedadjust (the same call
_icbm_step makes). The first harness read the limit every brain tick and never let either brain see the other,
which is how a curve overlapping the zone ENTRY -- 75 mph left in a 45 zone -- got through a green suite.
"""
import json
import types

import pytest

import openpilot.selfdrive.controls.lib.speedadjust_pnw.speedadjust_controller as sa
from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import IcbmEpisode, SA_TELE_KEYS, icbm_note_speedadjust
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
    self.sl, self.police, self.calls, self.icbm = 0.0, None, [], {}

  def get(self, k, return_default=True):
    if k == "MapSpeedLimit":
      return str(self.sl) if self.sl else ""
    if k == "LocationServices":
      return json.dumps({"police": self.police}) if self.police else "{}"
    if k == "IcbmTarget":
      return self.icbm
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


def simulate(monkeypatch, script, T, v0_mph=75, mode=2, tele_phase=0):
  """script(t) -> (limit_mph, curve_target_mph or None, police or None). Returns (final set mph, trace, controller).
  tele_phase (0..99 frames) shifts ICBM's ~1 Hz status read against the brains' ticks."""
  _Clock.t = 1000.0
  monkeypatch.setattr(sa, "time", _Clock)
  c = sa.SpeedAdjustController(types.SimpleNamespace(openpilotLongitudinalControl=False), params=_P(mode))
  c.mem_params = _Mem()
  sm, ep, guard, gov = _SM(), IcbmEpisode(), RestoreGuard(), PressGovernor()
  stock, pending, rcap_state = v0_mph * MPH, [], None
  sa_cmd = icbm_cmd = None
  last_dec_ts, frame, trace, sa_tele = None, 0, [], {}
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
    if frame % 100 == tele_phase:                         # ces_pnw re-reads SpeedAdjustStatus at ~1 Hz
      st = next((v for k, v in reversed(c.mem_params.calls) if k == "SpeedAdjustStatus"), None) or {}
      sa_tele = {"sa" + k[0].upper() + k[1:]: st.get(k) for k in SA_TELE_KEYS}
    if frame % 25 == 0:                                   # both brains at 4 Hz
      c.mem_params.sl, c.mem_params.police = lim * MPH, pol
      c.cap(sm, stock, stock, stock, True)
      c._sa_pub_t = -1e9                                  # it publishes status at 5 Hz from a 20 Hz loop on the car
      rl, rcap_state = C.icbm_restore_limit(lim * MPH, rcap_state, now)
      icbm_note_speedadjust(ep, sa_tele, rl)
      tgt, d = ep.step(now, curve * MPH if curve else None, stock, stock, True, False,
                       restore_cap=ep.zone_cap, limit_now=rl if rl > 0 else None)
      pub = {}
      if tgt is not None:
        pub = {"target": tgt, "ceiling": ep.ceiling if ep.ceiling is not None else stock, "ts": now}
        if d == "inc":
          pub["dir"] = "inc"
      c.mem_params.icbm = pub
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
      trace.append((round(now - 1000.0), round(stock / MPH, 1), lim, curve, ep.phase, ep.zone_why, c._ovr))
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
    in_zone_after_curve = [row[1] for row in trace if row[0] >= 22]
    assert max(in_zone_after_curve) <= bound + 1.0, f"set reached {max(in_zone_after_curve):.1f}, bound {bound:.1f}\n{trace}"


def _overlap(t0, length=12.0, curve=35, drop_t=5.0, lim0=60, lim1=45):
  def script(t):
    return (lim0 if t < drop_t else lim1), (curve if t0 <= t < t0 + length else None), None
  return script


class TestACurveOverlappingTheZoneEntry:
  """Fable review of sazoneset2pnw's first version, MEASURED: the limit drops 60 -> 45 at t=5 s with the set at 75
  and a 35 mph curve starts at t0. Final set in the 45 zone was 56 / 75 / 75 / 75 / 73 / 68 / 64 / 57 for
  t0 = 4 / 5 / 6 / 7 / 9 / 11 / 13 / 16 -- and it stuck. Two mechanisms:
    (i)  ICBM's taps drove the set below the zone target, speedadjust declared the zone reached and forgot it, and
         ICBM restored to its ceiling (the pre-zone set, or one latched mid-slew) -- no stale-limit cap forms,
         because the limit ALREADY read 45 when the curve latched;
    (ii) ICBM's SET- taps landing around the drop confirm read as driver overrides and re-anchored speedadjust's
         ratio to the tapped-down set, so the zone never trimmed at all."""

  @pytest.mark.parametrize("t0", [4.0, 5.0, 6.0, 7.0, 9.0, 11.0, 13.0, 16.0])
  def test_the_measured_family_ends_at_the_zone_speed(self, monkeypatch, t0):
    final, trace, _ = simulate(monkeypatch, _overlap(t0), 120)
    assert abs(final - ZONE) <= 1.0, f"t0={t0}: final {final:.1f} mph in the 45 zone, want ~{ZONE:.1f}\n{trace}"

  @pytest.mark.parametrize("tele_phase", [0, 50])
  def test_a_dense_sweep_of_curve_start_times_and_read_phases(self, monkeypatch, tele_phase):
    bad = []
    for i in range(41):                                    # t0 = 0.0 .. 20.0 s in 0.5 s steps
      t0 = i * 0.5
      final, _, _ = simulate(monkeypatch, _overlap(t0), 120, tele_phase=tele_phase)
      if abs(final - ZONE) > 1.0:
        bad.append((t0, round(final, 1)))
    assert not bad, f"curve start / final set outside {ZONE:.1f} +- 1: {bad}"

  @pytest.mark.parametrize("length,curve", [(3.0, 35), (5.0, 35), (8.0, 50), (12.0, 60)])
  def test_short_curves_and_curve_speeds_near_the_zone_speed(self, monkeypatch, length, curve):
    bad = []
    for i in range(0, 41, 2):
      final, _, _ = simulate(monkeypatch, _overlap(i * 0.5, length=length, curve=curve), 120)
      if abs(final - ZONE) > 1.0:
        bad.append((i * 0.5, round(final, 1)))
    assert not bad, f"{length:.0f} s curve at {curve} mph: {bad}"

  def test_a_small_limit_drop_is_bounded_just_the_same(self, monkeypatch):
    """55 -> 50 at a set of 60: the zone speed is 54.5, a few taps -- so ICBM's taps easily pass it before the drop
    even confirms, and the zone opens and completes between two of ICBM's status reads (the zoneN case)."""
    zone, bad = 50 * 60 / 55, []
    for i in range(41):
      final, _, _ = simulate(monkeypatch, _overlap(i * 0.5, curve=40, lim0=55, lim1=50), 120, v0_mph=60)
      if abs(final - zone) > 1.0:
        bad.append((i * 0.5, round(final, 1)))
    assert not bad, f"want ~{zone:.1f}: {bad}"

  @pytest.mark.parametrize("t0", [4.0, 7.0])
  def test_a_second_drop_during_the_same_curve_repeats_from_the_zone_speed(self, monkeypatch, t0):
    """60 -> 45 -> 35 inside one curve: the second zone is relative to the first zone's speed (35 x 75/60 = 43.75),
    not to the pre-zone ceiling the curve latched. t0=7 latches AFTER the first drop, so the stale-limit cap would
    scale the 75 by 35/45 (58 mph) -- only speedadjust's zone speed bounds it."""
    def script(t):
      return (60 if t < 5 else 45 if t < 12 else 35), (30 if t0 <= t < 20 else None), None
    final, trace, _ = simulate(monkeypatch, script, 90)
    assert abs(final - 35 * 75 / 60) <= 1.0, f"final {final:.1f}\n{trace}"
