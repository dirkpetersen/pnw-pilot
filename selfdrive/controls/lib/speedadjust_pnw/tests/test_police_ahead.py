"""policeahead2pnw -- a latched police cap anticipates a lower limit mapd has announced ahead of the report.

drives/2026-10-01/tesla-police-limit-drop (Tesla, 15:24 PT): a confirmed report latched at 90 mph (set 90, posted 70, cap
75). mapd announced a 60 750 m ahead; the look-ahead went live ~450 m out but its ratio target (77 mph) was ABOVE the
police cap, so the car held 75 to the sign and then ramped 75 -> 65 at CAP_SLEW (1 m/s^2) INSIDE the 60 zone, 4-5 s and
~150 m late. The report was 309 m past the boundary (the log; the owner's impression was ~100 m).

The change: while a LIVE look-ahead runs and the latched report is at/beyond the announced boundary, the police cap is the
announced limit + 5 mph from the moment the look-ahead starts. Owner rules kept: police = exactly (the limit at the
report) + 5; never above the plain cap; never below limit + 5; look-ahead off/shadow = byte-for-byte the old cap.
"""
import json
import types

import pytest

import openpilot.selfdrive.controls.lib.speedadjust_pnw.speedadjust_controller as sa
from openpilot.system.mapd.mapd_configd import next_limit_payload

MPH = sa.MPH_TO_MS
V70, V60, V65 = 70 * MPH, 60 * MPH, 65 * MPH
V75 = 75 * MPH
PM = sa.POLICE_MARGIN


class _CP:
  def __init__(self, op_long=True):
    self.openpilotLongitudinalControl = op_long


class _Params:
  def get(self, *a, **k):
    return None

  def get_bool(self, *a, **k):
    return False


def _pol(dist_m, key="p1"):
  mi = round(dist_m / sa.MILE_M, 1)          # location_servicesd publishes 0.1 mi steps
  return {"state": "alert", "dist_mi": mi, "tier": "confirmed", "cap": {"dist_mi": mi, "key": key, "uuid": key}}


def _ctrl(sl=V70, la=None, odo=0.0, police=None):
  c = sa.SpeedAdjustController(_CP(), params=_Params())
  c._mode = 2
  c._sl = sl
  c._sl_valid_t = 1e12                       # keep the injected limit from ageing out
  c._sl_ref = sl
  c._ratio = 1.0
  c._police = police
  c._la = la
  c._odo = odo
  return c


def _la(n=V60, boundary=500.0, live=True):
  return {"n": n, "b": boundary, "live": live}


# ---- unit: the cap value ----------------------------------------------------------------------------------------------
def test_report_beyond_the_boundary_anticipates_the_announced_limit_plus_5():
  c = _ctrl(la=_la(boundary=500.0), police=_pol(800.0))
  assert c._police_cap(V75, V75) == pytest.approx(V60 + PM)


def test_report_short_of_the_boundary_keeps_the_current_limit_plus_5():
  """The report is IN the 70 zone (300 m, boundary 500 m): its limit is 70, so exactly 75 -- no anticipation."""
  c = _ctrl(la=_la(boundary=500.0), police=_pol(300.0))
  assert c._police_cap(V75, V75) == pytest.approx(V70 + PM)


@pytest.mark.parametrize("la", [None, _la(live=False)], ids=["no-look-ahead", "shadow"])
def test_no_live_look_ahead_is_the_old_cap_exactly(la):
  c = _ctrl(la=la, police=_pol(800.0))
  assert c._police_cap(V75, V75) == V70 + PM


def test_the_anticipated_cap_is_never_above_the_plain_cap():
  """An announced limit that is not lower than the current one (cannot happen via _update_announcement, but the
  arithmetic must hold anyway) never raises the cap."""
  c = _ctrl(sl=V60, la=_la(n=V70, boundary=500.0), police=_pol(800.0))
  assert c._police_cap(V75, V75) == V60 + PM


def test_after_the_boundary_before_the_limit_is_adopted_the_target_stays():
  """Boundary behind us (-20 m), the 1 Hz read has not adopted the 60 yet: still 65, not a jump back up to 75."""
  c = _ctrl(la=_la(boundary=480.0), odo=500.0, police=_pol(400.0))
  assert c._police_cap(V75, V75) == pytest.approx(V60 + PM)


def test_a_boundary_that_aborts_gives_the_cap_back():
  c = _ctrl(la=_la(boundary=500.0), police=_pol(800.0))
  assert c._police_cap(V75, V75) == pytest.approx(V60 + PM)
  c._la = None                               # look-ahead aborted (road not taken / mapd gone)
  assert c._police_cap(V75, V75) == V70 + PM


def test_rounding_flicker_near_the_boundary_does_not_flip_the_target_back():
  """dist_mi is 0.1 mi steps: a report ~30 m past the boundary can read short of it on a later tick. Once judged beyond
  it stays beyond for that report -- a flickering target would surge."""
  la = _la(boundary=500.0)
  c = _ctrl(la=la, police=_pol(530.0))       # 0.3 mi = 483 m < 500?  -> rounds to 0.3: strict check fails
  first = c._police_cap(V75, V75)
  c._police = {"state": "alert", "dist_mi": 0.4, "tier": "confirmed", "cap": {"dist_mi": 0.4, "key": "p1", "uuid": "p1"}}
  assert c._police_cap(V75, V75) == pytest.approx(V60 + PM)      # 0.4 mi = 644 m >= 500: beyond
  c._police = {"state": "alert", "dist_mi": 0.2, "tier": "confirmed", "cap": {"dist_mi": 0.2, "key": "p1", "uuid": "p1"}}
  assert c._police_cap(V75, V75) == pytest.approx(V60 + PM), "held for this report"
  assert first == V70 + PM


def test_another_report_does_not_inherit_the_beyond_verdict():
  la = _la(boundary=500.0)
  c = _ctrl(la=la, police=_pol(800.0, key="A"))
  assert c._police_cap(V75, V75) == pytest.approx(V60 + PM)
  c._police_latched, c._police_latched_key = True, "B"       # control channel moved to report B
  c._police = _pol(200.0, key="B")           # B is in the 70 zone, short of the boundary
  assert c._police_cap(V75, V75) == V70 + PM


# ---- closed loop: the 2026-10-01 15:24 numbers on the REAL controller -------------------------------------------------
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
    self.sl, self.nxt, self.police, self.calls = 0.0, None, None, []

  def get(self, k, return_default=True):
    if k == "MapSpeedLimit":
      return str(self.sl) if self.sl else ""
    if k == "NextMapSpeedLimit":
      n, d = self.nxt if self.nxt else (0.0, 0.0)
      return next_limit_payload(n, d, _Clock.t)
    if k == "LocationServices":
      return json.dumps({"police": self.police}) if self.police else "{}"
    return None

  def put_nonblocking(self, k, v):
    self.calls.append((k, v))


class _P:
  def __init__(self, la_mode):
    self.la_mode = la_mode

  def get(self, k, return_default=True):
    return {"AutoSpeedReduce": "2", "LimitAheadMode": self.la_mode}.get(k)


class _SM:
  cs = types.SimpleNamespace(gasPressed=False, brakePressed=False, cruiseState=types.SimpleNamespace(speed=0.0, enabled=True))

  def __getitem__(self, k):
    return self.cs


def drive(monkeypatch, report_after_boundary, la_mode=sa.LA_LIVE, anticipate=True, tau=1.0, a_dn=2.0, xb=851.0, announce_m=752.0,
          v0_mph=89.7, set_mph=90.0, cur_mph=70, nxt_mph=60, T=60.0, chain=None, op_long=True):
  """The 15:24:30 state: latched, v 89.7 mph, set 90, posted 70, a 60 announced 752 m out, cap 75 -> the boundary at xb m
  (mapd's current limit flips as the truck crosses it, read at 1 Hz). Plant: first-order lag tau toward the cap, decel
  bounded by a_dn (the log: <= 1.9 m/s^2). Report at xb + report_after_boundary. Returns the run record."""
  _Clock.t = 1000.0
  monkeypatch.setattr(sa, "time", _Clock)
  monkeypatch.setattr(sa.cloudlog, "event", lambda *a, **k: None)
  c = sa.SpeedAdjustController(_CP(op_long), params=_P(la_mode))
  c.mem_params = _Mem()
  if not anticipate:                         # the pre-change cap: the new branch reads only self._la, so hide it from there
    orig = c._police_cap

    def _old(v_cruise, v_ego):
      la, c._la = c._la, None
      try:
        return orig(v_cruise, v_ego)
      finally:
        c._la = la
    c._police_cap = _old
  sm = _SM()
  set_ms, v, x = set_mph * MPH, v0_mph * MPH, 0.0
  xr = xb + report_after_boundary
  r = types.SimpleNamespace(v_at_boundary=None, v_at_report=None, rows=[], c=c, tele=[])
  while _Clock.t - 1000.0 < T:
    dt = 0.05
    _Clock.t += dt
    if chain is None:
      cur, nxt, xn = (cur_mph, nxt_mph, xb) if x < xb else (nxt_mph, 0, 0.0)
    else:                                    # chain = (limit mph, boundary x): a SECOND drop after the first
      cur, nxt, xn = ((cur_mph, nxt_mph, xb) if x < xb else (nxt_mph, chain[0], chain[1]) if x < chain[1]
                      else (chain[0], 0, 0.0))
    c.mem_params.sl = cur * MPH
    c.mem_params.nxt = (nxt * MPH, xn - x) if (nxt and xn - x <= announce_m) else None
    c.mem_params.police = _pol(max(xr - x, 0.0)) if x < xr + 150 else None
    out = c.cap(sm, set_ms, set_ms, v, True)
    a = max(-a_dn, min((min(out, set_ms) - v) / tau, 1.0))
    x0 = x
    v = max(0.0, v + a * dt)
    x += v * dt
    if x0 < xb <= x:
      r.v_at_boundary = v / MPH
    if x0 < xr <= x:
      r.v_at_report = v / MPH
    r.rows.append((_Clock.t - 1000.0, x, v / MPH, out / MPH))
    r.tele.append((x, c._pol_ahead_tgt))
  return r


def test_actual_report_309_m_past_the_boundary_is_at_65_by_the_boundary(monkeypatch):
  r = drive(monkeypatch, 309.0)
  assert r.v_at_boundary == pytest.approx(65.0, abs=1.5), r.v_at_boundary
  assert r.v_at_report == pytest.approx(65.0, abs=1.0), r.v_at_report


def test_the_old_behaviour_was_still_75_at_the_boundary(monkeypatch):
  r = drive(monkeypatch, 309.0, anticipate=False)
  assert r.v_at_boundary > 74.0, r.v_at_boundary


@pytest.mark.parametrize("p", [60.0, 100.0, 150.0, 309.0])
def test_a_report_soon_after_the_boundary_is_at_limit_plus_5_not_75(monkeypatch, p):
  new = drive(monkeypatch, p)
  old = drive(monkeypatch, p, anticipate=False)
  assert new.v_at_report <= 65.5, f"report {p} m past the sign: {new.v_at_report:.1f} mph"
  assert new.v_at_report <= old.v_at_report
  assert old.v_at_report > new.v_at_report + 0.5 or p >= 309.0
  # never below the exact target by more than the plant's own undershoot, never a surge: speed never rises
  speeds = [row[2] for row in new.rows if row[1] < 851.0 + p]
  assert all(b <= a + 0.01 for a, b in zip(speeds, speeds[1:], strict=False)), "the car must only slow down on the approach"


def test_report_in_the_70_zone_short_of_the_boundary_stays_at_75(monkeypatch):
  """Report 200 m BEFORE the boundary: its limit is 70, exactly 75 at the report -- the change must not touch it."""
  new = drive(monkeypatch, -200.0)
  old = drive(monkeypatch, -200.0, anticipate=False)
  assert new.v_at_report == pytest.approx(old.v_at_report, abs=0.3)
  assert new.v_at_report == pytest.approx(75.0, abs=1.0)


@pytest.mark.parametrize("tau", [0.5, 1.0, 2.0])
def test_plant_sensitivity_the_report_is_always_at_or_under_65_plus_a_hair(monkeypatch, tau):
  r = drive(monkeypatch, 100.0, tau=tau)
  assert r.v_at_report <= 66.0, f"tau={tau}: {r.v_at_report:.1f}"


def test_look_ahead_in_shadow_is_the_old_behaviour_exactly(monkeypatch):
  """LimitAheadMode shadow (the kill switch: 0/1 = no live look-ahead) must change nothing -- compared sample by sample
  with the anticipation hidden."""
  a = drive(monkeypatch, 100.0, la_mode=sa.LA_SHADOW)
  b = drive(monkeypatch, 100.0, la_mode=sa.LA_SHADOW, anticipate=False)
  assert a.rows == b.rows


def test_a_report_without_an_identity_is_never_judged_beyond_by_default():
  """A control report with no key/uuid latches with key None (and is dropped on the next tick). With no verdict stored
  yet, None == None must not read as 'already judged beyond' -- it is short of the boundary here, so the cap is 75."""
  c = _ctrl(la=_la(boundary=500.0), police={"state": "alert", "dist_mi": 0.1, "tier": "confirmed", "cap": {"dist_mi": 0.1}})
  assert c._police_cap(V75, V75) == V70 + PM
  assert c._police_latched_key is None


def test_the_min_cap_floor_still_applies_to_the_anticipated_target(monkeypatch):
  """An announced 3 mph limit -> anticipated police target 8 mph, but _cap_impl's MIN_CAP (10 mph) floor holds the cap."""
  r = drive(monkeypatch, 309.0, nxt_mph=3, T=45.0)
  assert any(t is not None and t < sa.MIN_CAP for _x, t in r.tele), "the anticipated target (8 mph) was never in effect"
  assert min(row[3] for row in r.rows) >= sa.MIN_CAP / MPH - 1e-6


def test_chained_drop_rejudges_the_report_against_the_new_boundary(monkeypatch):
  """70 -> 60 at 851 m, then 60 -> 50 at 1400 m (the look-ahead
  re-targets ~410 m before it, with the report still ahead). Report A is 300 m into the 60 zone (1151 m): its limit is 60, so
  exactly 65 -- the 50 announced beyond it must NOT reach it. (A stale 'beyond' verdict gave 55: 10 mph under the rule.)"""
  new = drive(monkeypatch, 300.0, chain=(50, 1400.0), T=75.0)
  old = drive(monkeypatch, 300.0, chain=(50, 1400.0), anticipate=False, T=75.0)
  assert new.v_at_report == pytest.approx(65.0, abs=1.5), new.v_at_report
  assert new.v_at_report >= old.v_at_report - 1.5
  assert new.v_at_report > 62.0, f"under the owner's rule limit + 5: {new.v_at_report:.1f}"


def test_a_retarget_clears_the_verdict_and_the_log_latch():
  c = _ctrl(la=_la(boundary=500.0), police=_pol(800.0))
  assert c._police_cap(V75, V75) == pytest.approx(V60 + PM)
  assert c._la["pol_beyond"] and c._la["pol_logged"]
  # the look-ahead re-targets to a 50 whose boundary is BEYOND the report: the report is judged afresh
  c._la.update(n=50 * MPH, b=1500.0)
  c._la.pop("pol_beyond", None)              # what the re-target does
  assert c._police_cap(V75, V75) == V70 + PM


def test_the_verdict_is_keyed_to_the_announced_limit_even_without_the_pop():
  """Belt and braces: a verdict made for the 60 is not valid for a 50 (same report, same episode dict)."""
  c = _ctrl(la=_la(n=V60, boundary=500.0), police=_pol(800.0))
  assert c._police_cap(V75, V75) == pytest.approx(V60 + PM)
  c._la["n"], c._la["b"] = 50 * MPH, 1500.0
  assert c._police_cap(V75, V75) == V70 + PM


def test_boundary_precision_a_report_100_m_past_the_boundary_is_beyond(monkeypatch):
  """Pins the strict comparison: report 100 m past the sign must anticipate. (A mutant requiring >100 m past, or a
  margin of one rounding step, would leave 75 at the report.)"""
  c = _ctrl(la=_la(boundary=500.0), police=_pol(600.0))      # dist_mi rounds 600 m -> 0.4 mi = 644 m > 500
  assert c._police_cap(V75, V75) == pytest.approx(V60 + PM)
  c2 = _ctrl(la=_la(boundary=500.0), police={"state": "alert", "dist_mi": 0.3, "tier": "confirmed",
                                             "cap": {"dist_mi": 0.3, "key": "q", "uuid": "q"}})   # 483 m, short
  assert c2._police_cap(V75, V75) == V70 + PM
  c3 = _ctrl(la=_la(boundary=480.0), police={"state": "alert", "dist_mi": 0.3, "tier": "confirmed",
                                             "cap": {"dist_mi": 0.3, "key": "q", "uuid": "q"}})   # 483 m vs 480: beyond
  assert c3._police_cap(V75, V75) == pytest.approx(V60 + PM)


def test_telemetry_flags_the_anticipated_target_in_the_status_publish(monkeypatch):
  r = drive(monkeypatch, 309.0)
  st = [v for k, v in r.c.mem_params.calls if k == "SpeedAdjustStatus"][-1]
  assert "polAhead" in st and "polTgt" in st
  mid = [t for x, t in r.tele if 450.0 < x < 840.0]
  assert mid and all(t == pytest.approx(V65, abs=1e-3) for t in mid)
  assert all(t is None for x, t in r.tele if x < 50.0), "nothing is flagged before the look-ahead starts"


def test_a_car_without_op_long_gets_announced_limit_plus_5_as_its_speedadjust_target(monkeypatch):
  """Stock-ACC (the Lightning): the same cap rides SpeedAdjustTarget to the button executor."""
  r = drive(monkeypatch, 309.0, op_long=False, v0_mph=89.7)
  tg = [v["target"] for k, v in r.c.mem_params.calls if k == "SpeedAdjustTarget" and v.get("target")]
  assert tg and min(tg) == pytest.approx(V65, abs=0.05), min(tg)
  assert max(tg) <= 90 * MPH + 1e-6
  old = drive(monkeypatch, 309.0, op_long=False, anticipate=False)
  tg_old = [v["target"] for k, v in old.c.mem_params.calls if k == "SpeedAdjustTarget" and v.get("target")]
  assert min(tg_old) == pytest.approx(V65, abs=0.05), "same END state as before (65), just reached earlier"
  first65 = next(i for i, t in enumerate(tg) if t <= V65 + 0.01)
  first65_old = next(i for i, t in enumerate(tg_old) if t <= V65 + 0.01)
  assert first65 < first65_old
