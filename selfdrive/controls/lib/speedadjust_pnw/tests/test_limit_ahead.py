"""limitahead2pnw -- slow down AHEAD of a lower limit mapd announces, closed loop against the REAL Ford executor.

drives/2026-09-24/limit-drop-1857: at 18:57 PT the truck reached a 60 -> 40 boundary at 74 mph (set 75), because
speedadjust only acted on the CURRENT limit, 3 s after the sign; mapd had announced the 40 1,078 m ahead. Five minutes
later (19:02:26) mapd announced a 25 on the road NOT taken and matched onto it for 4.95 s -- the case that makes the
look-ahead restorable.

The harness is test_zone_set_closed_loop's (speedadjust + ICBM episode -> arbitrate/decide_press/RestoreGuard/
PressGovernor, the stock set answering each press 0.3 s later by 1 mph), plus a truck that MOVES: the road is a function
of position, and vEgo follows the stock set down at A_FOLLOW (the measured median, see LA_A_PLAN) and up at A_UP.
"""
import json
import types

import pytest

import openpilot.selfdrive.controls.lib.speedadjust_pnw.speedadjust_controller as sa
from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import IcbmEpisode, SA_TELE_KEYS, icbm_note_speedadjust
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw_constants as C
from openpilot.system.mapd.mapd_configd import next_limit_payload
from opendbc.car.ford.icbm_pnw import arbitrate, decide_press, RestoreGuard, PressGovernor, IcbmCommand

MPH = sa.MPH_TO_MS
A_FOLLOW = 0.56          # m/s^2, the measured median follow rate (LA_A_PLAN's basis)
A_UP = 0.5


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
    self.sl, self.nxt, self.police, self.calls, self.icbm = 0.0, None, None, [], {}
    self.next_ts_age = 0.0

  def get(self, k, return_default=True):
    if k == "MapSpeedLimit":
      return str(self.sl) if self.sl else ""
    if k == "NextMapSpeedLimit":
      n, d = self.nxt if self.nxt else (0.0, 0.0)
      return next_limit_payload(n, d, _Clock.t - self.next_ts_age)   # the bridge's own payload
    if k == "LocationServices":
      return json.dumps({"police": self.police}) if self.police else "{}"
    if k == "IcbmTarget":
      return self.icbm
    return None

  def put_nonblocking(self, k, v):
    self.calls.append((k, v))


class _P:
  def __init__(self, mode, la_mode):
    self.mode, self.la_mode = mode, la_mode

  def get(self, k, return_default=True):
    if k == "AutoSpeedReduce":
      return str(self.mode)
    if k == "LimitAheadMode":
      return self.la_mode
    return None


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


class Run:
  pass


def simulate(monkeypatch, road, T, v0_mph=74.0, set0_mph=75.0, mode=2, la_mode=sa.LA_LIVE, curve=None, police=None,
             driver=None, x_mark=None, op_long=False):
  """road(x, t) -> (current limit mph, announced next limit mph or 0, distance to it m). curve(x, t) -> ICBM curve
  target mph or None. police(x, t) -> police dict or None. driver(t, run) -> +1/-1 for a driver SET+/SET- tap now,
  else None. x_mark: record vEgo when the truck passes this position (the boundary)."""
  _Clock.t = 1000.0
  monkeypatch.setattr(sa, "time", _Clock)
  events = []
  monkeypatch.setattr(sa.cloudlog, "event", lambda name, **kw: events.append((name, kw)))
  c = sa.SpeedAdjustController(types.SimpleNamespace(openpilotLongitudinalControl=op_long), params=_P(mode, la_mode))
  c.mem_params = _Mem()
  sm, ep, guard, gov = _SM(), IcbmEpisode(), RestoreGuard(), PressGovernor()
  stock, v, x, pending, rcap_state = set0_mph * MPH, v0_mph * MPH, 0.0, [], None
  sa_cmd = icbm_cmd = None
  last_dec_ts, frame, sa_tele = None, 0, {}
  r = Run()
  r.trace, r.targets, r.v_at_mark, r.sl_trace, r.outs = [], [], None, [], []
  r.c, r.x = c, 0.0                                       # visible to driver() hooks mid-run
  while _Clock.t - 1000.0 < T:
    frame += 1
    _Clock.t += 0.01
    now, t = _Clock.t, _Clock.t - 1000.0
    for p in list(pending):
      if now >= p[0]:
        stock += p[1]
        pending.remove(p)
    if v > stock:
      v = max(stock, v - A_FOLLOW * 0.01)
    elif v < stock:
      v = min(stock, v + A_UP * 0.01)
    x0, x = x, x + v * 0.01
    r.x = x
    if x_mark is not None and x0 < x_mark <= x:
      r.v_at_mark = v / MPH
    sm.cs.cruiseState.speed = stock
    if driver is not None:
      d = driver(t, r)
      if d:
        pending.append((now + 0.3, d * MPH))
    if frame % 100 == 0:                                  # ces_pnw re-reads SpeedAdjustStatus at ~1 Hz
      st = next((val for k, val in reversed(c.mem_params.calls) if k == "SpeedAdjustStatus"), None) or {}
      sa_tele = {"sa" + k[0].upper() + k[1:]: st.get(k) for k in SA_TELE_KEYS}
    if frame % 25 == 0:                                   # both brains at 4 Hz
      cur, nxt, nd = road(x, t)
      c.mem_params.sl = cur * MPH
      c.mem_params.nxt = (nxt * MPH, nd) if nxt else None
      c.mem_params.police = police(x, t) if police else None
      r.out = c.cap(sm, stock, stock, v, True)
      r.outs.append((x, r.out / MPH))
      c._sa_pub_t = -1e9
      rl, rcap_state = C.icbm_restore_limit(cur * MPH, rcap_state, now)
      icbm_note_speedadjust(ep, sa_tele, rl)
      cv = curve(x, t) if curve else None
      tgt, d = ep.step(now, cv * MPH if cv else None, stock, stock, True, False,
                       restore_cap=ep.zone_cap, limit_now=rl if rl > 0 else None)
      pub = {}
      if tgt is not None:
        pub = {"target": tgt, "ceiling": ep.ceiling if ep.ceiling is not None else stock, "ts": now}
        if d == "inc":
          pub["dir"] = "inc"
      c.mem_params.icbm = pub
      icbm_cmd = _cmd(pub)
      tc = [val for k, val in c.mem_params.calls if k == "SpeedAdjustTarget"]
      sa_cmd = _cmd(tc[-1] if tc else None)
      r.targets.append({k: val for k, val in (tc[-1] if tc else {}).items() if k != "ts"})
      r.sl_trace.append((t, cur, c._sl / MPH))
    cmd = arbitrate([icbm_cmd, sa_cmd], now, last_dec_ts)
    if cmd is not None and cmd.dir == "dec":
      last_dec_ts = now
    intent = decide_press(stock, cmd, now, True, False)
    pinc = arbitrate([q for q in (icbm_cmd, sa_cmd) if q is not None and q.dir == "inc"], now)
    intent = guard.filter(intent, stock, now, pinc is not None, pinc.ceiling_ms if pinc else None,
                          cmd is not None and cmd.dir == "dec")
    was = gov._active
    btn = gov.update(frame, intent)
    if btn and was is None:
      pending.append((now + 0.3, (1 if btn == "inc" else -1) * MPH))
    if frame % 100 == 0:
      r.trace.append((round(t), round(x), round(v / MPH, 1), round(stock / MPH, 1), c._ovr, ep.phase, ep.zone_why))
  r.final, r.x, r.c = stock / MPH, x, c
  r.la = [kw for n, kw in events if n == "speedadjust_lookahead"]
  r.events = events
  return r


def _acts(r, action):
  return [e for e in r.la if e["action"] == action]


# ---- the roads --------------------------------------------------------------------------------------------------------
XB = 1078.0                                               # 18:57: the 40 first announced 1,078 m ahead


def road_1857(x, t, xb=XB):
  """60 -> 40 at xb, announced from x = 0 (1,078 m out). mapd's next after the boundary is 0 here."""
  return (60, 40, xb - x) if x < xb else (40, 0, 0.0)


def road_chain(gap):
  def road(x, t):
    if x < XB:
      return 60, 40, XB - x
    if x < XB + gap:
      return 40, 35, XB + gap - x
    return 35, 0, 0.0
  return road


class _Road1902:
  """19:02:26: current 45, a 25 announced ahead (the road not taken); at the boundary mapd matches onto the 25 for
  `bogus_s` s, then the truck is on the ramp, posted 50, with nothing announced."""
  def __init__(self, xb=700.0, bogus_s=4.95):
    self.xb, self.bogus_s, self.t_cross = xb, bogus_s, None

  def __call__(self, x, t):
    if x < self.xb:
      return 45, 25, self.xb - x
    if self.t_cross is None:
      self.t_cross = t
    return (25, 0, 0.0) if t - self.t_cross < self.bogus_s else (50, 0, 0.0)


# ---- the start distance ----------------------------------------------------------------------------------------------
def test_start_distance_formula_and_constants():
  v, vt = 74 * MPH, 50 * MPH
  d = sa.la_start_distance(v, vt)
  assert d == pytest.approx((v * v - vt * vt) / (2 * sa.LA_A_PLAN) + v * sa.LA_T_LEAD)
  assert 550 < d < 650, f"74 -> 50 mph starts {d:.0f} m out"
  assert sa.la_start_distance(40 * MPH, 50 * MPH) == pytest.approx(40 * MPH * sa.LA_T_LEAD)   # below target: lead only


def test_a_higher_speed_starts_further_out():
  ds = [sa.la_start_distance(v * MPH, 40 * MPH) for v in (45, 55, 65, 75, 85)]
  assert ds == sorted(ds) and len(set(ds)) == len(ds), ds


def test_1857_starts_at_the_computed_distance_with_target_50_and_reaches_it_by_the_boundary(monkeypatch):
  r = simulate(monkeypatch, road_1857, 80, x_mark=XB)
  starts = _acts(r, "start")
  assert len(starts) == 1, r.la
  s = starts[0]
  assert s["target"] == pytest.approx(50 * MPH, abs=0.01), "rule 1: 75 on a 60 -> 25 % over 40 = 50"
  assert s["vPlan"] == pytest.approx(75 * MPH, abs=0.01)     # max(vEgo 74, set 75)
  d_expect = sa.la_start_distance(s["vPlan"], s["target"])
  assert s["d_start"] == pytest.approx(d_expect, abs=1.0)      # the logged v is rounded to 0.01 m/s
  assert d_expect - 12.0 <= s["dist"] <= d_expect, f"started at {s['dist']:.0f} m, d_start {d_expect:.0f} m"
  assert 550 < s["dist"] < 650
  assert r.v_at_mark is not None and r.v_at_mark <= 52.0, f"{r.v_at_mark} mph at the boundary\n{r.trace}"
  assert _acts(r, "promote"), r.la
  assert abs(r.final - 50) <= 1.0 and r.c._no_restore_why == "zoneSet", f"final {r.final}\n{r.trace}"


def test_the_speed_dependence_in_the_loop(monkeypatch):
  """Same 60 -> 40 boundary at set 65 and set 80: the faster truck starts further out, and both arrive at their own
  target (rule 1: 40 x set/60) by the boundary."""
  dist = {}
  for set0 in (65.0, 80.0):
    r = simulate(monkeypatch, road_1857, 90, v0_mph=set0 - 1, set0_mph=set0, x_mark=XB)
    s = _acts(r, "start")[0]
    tgt = 40 * set0 / 60
    assert s["target"] == pytest.approx(tgt * MPH, abs=0.01)
    assert r.v_at_mark <= tgt + 2.0, f"set {set0}: {r.v_at_mark} mph at the boundary, target {tgt:.1f}"
    dist[set0] = s["dist"]
  assert dist[80.0] > dist[65.0] + 150, dist


# ---- the false announcement -------------------------------------------------------------------------------------------
@pytest.mark.parametrize("xb", [700.0, 708.0, 716.0, 724.0])
def test_1902_the_road_not_taken_aborts_and_restores_the_exact_previous_set(monkeypatch, xb):
  """45 road, set 51, a 25 announced; mapd then matches onto the 25 for 4.95 s and the truck is on a 50 ramp.
  Before: a permanent 25-zone set (the 2 s confirm passed it). Now: restorable, restored to exactly 51."""
  rd = _Road1902(xb=xb)
  r = simulate(monkeypatch, rd, 110, v0_mph=47, set0_mph=51)
  assert _acts(r, "start") and _acts(r, "materialized"), r.la
  aborts = _acts(r, "abort")
  assert [a["reason"] for a in aborts] == ["limitRose"], r.la
  assert aborts[0]["restore"] == pytest.approx(51 * MPH, abs=0.01)
  assert not _acts(r, "promote")
  assert not [n for n, _ in r.events if n == "speedadjust_zone_set"], "the bogus 25 became a permanent zone"
  lowest = min(row[3] for row in r.trace)
  assert lowest < 40, f"the look-ahead never slowed (lowest set {lowest})"
  assert r.final == pytest.approx(51.0, abs=0.01), f"restored to {r.final}, want exactly 51\n{r.trace}"


def test_the_promote_hold_is_continuous(monkeypatch):
  """25 for 5 s, the ramp's 50 for ~1 s (too short to abort), 25 for 4 s, then 50 for good: 10 s of 25 in total but
  never 8 s in a row -- still the road not taken, still restored."""
  class Rd(_Road1902):
    def __call__(self, x, t):
      cur, nxt, nd = super().__call__(x, t)
      if self.t_cross is not None:
        dt = t - self.t_cross
        cur = 25 if dt < 5.0 or 6.2 <= dt < 10.2 else 50
      return cur, nxt, nd
  r = simulate(monkeypatch, Rd(bogus_s=0.0), 110, v0_mph=47, set0_mph=51)
  assert not _acts(r, "promote"), r.la
  assert [a["reason"] for a in _acts(r, "abort")] == ["limitRose"], r.la
  assert r.final == pytest.approx(51.0, abs=0.01), f"{r.final}\n{r.trace}"


def test_an_announcement_that_vanishes_before_the_boundary_restores(monkeypatch):
  def road(x, t):
    if x < 400:
      return 45, 25, 700 - x
    return 45, 0, 0.0                                     # mapd changes its mind; the limit never drops
  r = simulate(monkeypatch, road, 90, v0_mph=47, set0_mph=51)
  assert [a["reason"] for a in _acts(r, "abort")] == ["gone"], r.la
  assert r.final == pytest.approx(51.0, abs=0.01), f"{r.final}\n{r.trace}"


def test_an_announcement_that_rises_before_the_boundary_restores(monkeypatch):
  def road(x, t):
    return 45, (25 if x < 400 else 40), 700 - x if x < 700 else 0.0
  r = simulate(monkeypatch, road, 40, v0_mph=47, set0_mph=51)
  assert _acts(r, "abort")[0]["reason"] == "rose", r.la


def test_a_boundary_that_passes_without_the_drop_restores(monkeypatch):
  def road(x, t):
    return 45, 25, max(700 - x, 1.0)                      # mapd keeps announcing, the limit never changes
  r = simulate(monkeypatch, road, 90, v0_mph=47, set0_mph=51)
  assert [a["reason"] for a in _acts(r, "abort")] == ["passed"], r.la
  assert r.final == pytest.approx(51.0, abs=0.01), f"{r.final}\n{r.trace}"


def test_the_restore_never_passes_a_real_zone_that_happened_meanwhile(monkeypatch):
  """An UNANNOUNCED 60 -> 50 drop lands while the truck is slowing for an announced 40 that never comes. The 50 is a
  real zone (75 on a 60 -> 62.5): the false-40 abort must not walk the set back to 75."""
  def road(x, t):
    if x < 700:
      return 60, 40, 1100 - x
    if x < 900:
      return 50, 40, 1100 - x                              # the real 50 lands; the 40 is still announced
    return 50, 0, 0.0                                     # ...and then mapd drops it: the 40 never comes
  r = simulate(monkeypatch, road, 120, x_mark=None)
  assert _acts(r, "start") and _acts(r, "abort"), r.la
  assert _acts(r, "abort")[0]["limitDropJoined"] is True
  assert abs(r.final - 62.5) <= 1.0, f"want the real 50 zone's speed 62.5, got {r.final}\n{r.trace}"


# ---- option 2: the confirm is skipped only for an announced drop ------------------------------------------------------
def _sl_switch_lag(r, new_mph):
  t_raw = next(t for t, cur, _ in r.sl_trace if cur == new_mph)
  t_held = next(t for t, _, held in r.sl_trace if abs(held - new_mph) < 0.01)
  return t_held - t_raw


def test_an_announced_drop_skips_the_confirm(monkeypatch):
  r = simulate(monkeypatch, road_1857, 50)
  assert _sl_switch_lag(r, 40) <= sa.READ_S + 0.26, r.sl_trace
  assert _acts(r, "confirmSkipped"), r.la


def test_an_unannounced_drop_keeps_the_confirm(monkeypatch):
  r = simulate(monkeypatch, lambda x, t: (60, 0, 0.0) if x < XB else (40, 0, 0.0), 50)
  assert _sl_switch_lag(r, 40) >= sa.SL_DROP_CONFIRM_S, r.sl_trace
  assert not r.la


def test_shadow_keeps_the_confirm_and_says_it_would_skip_it(monkeypatch):
  r = simulate(monkeypatch, road_1857, 50, la_mode=sa.LA_SHADOW)
  assert _sl_switch_lag(r, 40) >= sa.SL_DROP_CONFIRM_S, r.sl_trace
  assert _acts(r, "wouldSkipConfirm") and not _acts(r, "confirmSkipped"), r.la


# ---- shadow is inert --------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("road,T,v0,set0", [(road_1857, 80, 74, 75), (_Road1902, 110, 47, 51)])
def test_shadow_changes_nothing_and_logs_what_live_would_do(monkeypatch, road, T, v0, set0):
  off = simulate(monkeypatch, road() if road is _Road1902 else road, T, v0_mph=v0, set0_mph=set0, la_mode=sa.LA_OFF)
  sh = simulate(monkeypatch, road() if road is _Road1902 else road, T, v0_mph=v0, set0_mph=set0, la_mode=sa.LA_SHADOW)
  live = simulate(monkeypatch, road() if road is _Road1902 else road, T, v0_mph=v0, set0_mph=set0, la_mode=sa.LA_LIVE)
  assert sh.trace == off.trace, "shadow moved the truck"
  assert sh.targets == off.targets, "shadow changed a SpeedAdjustTarget publish"
  assert sh.sl_trace == off.sl_trace, "shadow changed the limit speedadjust acts on"
  assert not off.la, "OFF logged look-ahead decisions"
  s_sh, s_live = _acts(sh, "start")[0], _acts(live, "start")[0]
  assert s_sh["live"] is False and s_live["live"] is True
  for k in ("dist", "d_start", "target", "pre", "announced"):
    assert s_sh[k] == pytest.approx(s_live[k], abs=0.01), k


# ---- interactions -----------------------------------------------------------------------------------------------------
def test_police_limit_plus_5_still_wins_as_the_minimum(monkeypatch):
  """A police report comes into range while the look-ahead to 50 runs: on the 60 road the look-ahead's 50 is lower than
  police's 65 and wins; in the 40 zone police's 45 is lower than the zone's 50 and wins -- exactly limit + 5."""
  pol = {"state": "alert", "dist_mi": 0.2, "tier": "confirmed", "cap": {"state": "alert", "dist_mi": 0.2, "key": "p1"}}
  r = simulate(monkeypatch, road_1857, 90, police=lambda x, t: pol if x >= XB - 300 else None)
  at_boundary = [row[3] for row in r.trace if XB - 40 <= row[1] < XB]
  assert at_boundary and max(at_boundary) <= 51, f"police's 65 overrode the look-ahead's lower 50\n{r.trace}"
  assert abs(r.final - 45) <= 1.0, f"police in the 40 zone: exactly 45, got {r.final}\n{r.trace}"


@pytest.mark.parametrize("x0,x1,final_lo", [(XB - 450, XB - 250, 44.0), (150.0, 700.0, 49.0)])
def test_icbm_curve_and_look_ahead_the_lower_wins_and_no_restore_climbs_above_the_target(monkeypatch, x0, x1, final_lo):
  """A 35 mph curve while the look-ahead to 50 runs (x0..x1). The curve's lower target wins; after it, no restore may
  climb above the look-ahead target (our dec is on the bus, so arbitrate() runs no inc), and the 40 zone ends at or
  under 50. A curve that latched its ceiling mid-look-ahead (67 mph) restores at most to ICBM's own proportional cap,
  67 x 40/60 = 44.7 (icbmrestorecap2pnw, unchanged) -- slower than 50, never faster. One that latched before (75) ends
  at the zone speed."""
  def cv(x, t):
    return 35 if x0 <= x < x1 else None
  r = simulate(monkeypatch, road_1857, 120, curve=cv)
  before = [row[3] for row in r.trace if row[1] < XB]
  after = [row[3] for row in r.trace if row[1] >= x1 + 60]
  assert min(before) < 49, f"the curve's lower target did not win\n{r.trace}"
  assert after and max(after) <= 50 + 1.0, f"a restore climbed above the look-ahead target\n{r.trace}"
  assert final_lo <= r.final <= 50 + 1.0, f"final {r.final}\n{r.trace}"


def test_a_driver_set_plus_cancels_the_look_ahead_for_that_announcement(monkeypatch):
  """One SET+ while the look-ahead is walking the set down (opposite to our taps, so unambiguously the driver's)."""
  state = {"pressed": False}

  def driver2(t, run):
    if not state["pressed"] and run.trace and run.trace[-1][3] <= 71 and run.trace[-1][1] < XB - 100:
      state["pressed"] = True
      return +1
    return None

  r = simulate(monkeypatch, road_1857, 90, driver=driver2)
  assert state["pressed"], r.trace
  assert [a["reason"] for a in _acts(r, "abort")] == ["driverOverride"], r.la
  assert len(_acts(r, "start")) == 1, "the look-ahead restarted for the announcement the driver overrode"
  # the drop itself still applies when it arrives, from the set he chose (same % over the new limit)
  assert r.final < 60, f"{r.final}\n{r.trace}"


@pytest.mark.parametrize("gap", [429.0, 200.0])
def test_chained_drops_end_at_the_lowest_limit(monkeypatch, gap):
  """60 -> 40 -> 35 (report: the 35 was announced 429 m past the 40). 429 m: two episodes in a row. 200 m: the 35 enters
  its start distance while the 40 is still being confirmed, and the episode retargets."""
  r = simulate(monkeypatch, road_chain(gap), 140)
  assert abs(r.final - 35 * 75 / 60) <= 1.0, f"final {r.final}, want {35 * 75 / 60:.2f}\n{r.trace}\n{r.la}"
  if gap == 200.0:
    assert _acts(r, "retarget"), r.la


def test_a_one_read_flicker_of_an_announcement_does_not_start(monkeypatch):
  """18:58:25 PT: a 35 flickered for ~1 s. An announcement must be seen LA_STEADY_S before a look-ahead starts."""
  def road(x, t):
    return 60, (40 if 20.0 <= t < 21.2 else 0), 300.0 - (x - 1500.0) if x > 1500.0 else 300.0
  r = simulate(monkeypatch, road, 40)
  assert not _acts(r, "start"), r.la


def test_op_long_gets_the_look_ahead_as_its_cap_and_loses_it_on_a_false_announcement(monkeypatch):
  """The Tesla path: no buttons, cap() RETURNS the look-ahead target to the MPC; when the announcement proves false the cap
  simply goes away (the MPC returns to the driver's set)."""
  r = simulate(monkeypatch, road_1857, 60, op_long=True)
  near = [o for x, o in r.outs if XB - 60 <= x < XB]
  assert near and max(near) <= 50.0 + 0.01, f"cap near the boundary {near}"
  assert r.final == pytest.approx(75.0), "op-long: the set itself is never tapped"
  rd = _Road1902()
  r = simulate(monkeypatch, rd, 60, v0_mph=47, set0_mph=51, op_long=True)
  assert min(o for _, o in r.outs) < 35, "the look-ahead never capped"
  assert [a["reason"] for a in _acts(r, "abort")] == ["limitRose"], r.la
  assert r.outs[-1][1] == pytest.approx(51.0), f"cap not released: {r.outs[-5:]}"


def test_a_limit_that_stays_unknown_after_the_drop_ends_the_look_ahead_without_a_restore(monkeypatch):
  """mapd goes silent right after the 40 materialized: the look-ahead must not hold its cap on nothing forever, and must not
  walk the set back up to 75 either."""
  def road(x, t):
    if x < XB:
      return 60, 40, XB - x
    return (40, 0, 0.0) if x < XB + 60 else (0, 0, 0.0)
  r = simulate(monkeypatch, road, 90)
  assert [a["reason"] for a in _acts(r, "abort")] == ["limitUnknown"], r.la
  assert r.final <= 51.0, f"restored with the limit unknown: {r.final}\n{r.trace}"


# ---- Fable review of 855e913 --------------------------------------------------------------------------------------------
def _dropout_road(t0, length):
  def road(x, t):
    if x < XB:
      return 60, (0 if t0 <= t < t0 + length else 40), XB - x
    return 40, 0, 0.0
  return road


def test_F1_a_short_announcement_dropout_mid_approach_does_not_abort(monkeypatch):
  """Fable: a 2.5 s mapd dropout ~600 m out aborted ("gone"), restored 67 -> 75 and dismissed the announcement for good --
  75 mph at the 40 again. A dropout shorter than SL_HOLD_S is continuity."""
  r = simulate(monkeypatch, _dropout_road(14.0, 2.6), 80, x_mark=XB)
  assert not _acts(r, "abort"), r.la
  assert r.v_at_mark <= 52.0, f"{r.v_at_mark} mph at the boundary\n{r.trace}"
  assert abs(r.final - 50) <= 1.0


def test_F1_a_long_dropout_aborts_but_the_announcement_can_start_again(monkeypatch):
  """A 6 s dropout while the look-ahead runs (it started ~14.5 s in) does abort ("gone", past SL_HOLD_S) -- and when mapd
  announces the 40 again the look-ahead must restart (no dismissal on "gone") and the zone still ends at 50."""
  r = simulate(monkeypatch, _dropout_road(15.0, 6.0), 90, x_mark=XB)
  starts, aborts = _acts(r, "start"), _acts(r, "abort")
  assert [a["reason"] for a in aborts] == ["gone"] and len(starts) == 2, r.la
  assert abs(r.final - 50) <= 1.0, f"{r.final}\n{r.trace}"


def test_F1_a_1hz_announcement_flicker_does_not_churn(monkeypatch):
  def road(x, t):
    if x < XB:
      return 60, (40 if (t % 1.0) < 0.5 else 0), XB - x
    return 40, 0, 0.0
  r = simulate(monkeypatch, road, 80, x_mark=XB)
  assert len(_acts(r, "start")) <= 1 and not _acts(r, "abort"), r.la
  assert abs(r.final - 50) <= 1.0


@pytest.mark.parametrize("lag", [0.0, 1.0, 2.0])
def test_F1_mapd_zeroing_next_at_the_boundary_is_not_gone(monkeypatch, lag):
  """At d = 0 mapd zeroes `next`; the current limit may follow a little later. That is the boundary, not a vanished
  announcement: no abort, no restore, promote."""
  class Rd:
    t_cross = None

    def __call__(self, x, t):
      if x < XB:
        return 60, 40, XB - x
      if self.t_cross is None:
        self.t_cross = t
      return (60, 0, 0.0) if t - self.t_cross < lag else (40, 0, 0.0)
  r = simulate(monkeypatch, Rd(), 90, x_mark=XB)
  assert not _acts(r, "abort") and _acts(r, "promote"), r.la
  assert max(row[3] for row in r.trace if row[1] > XB) <= 50 + 1.0, f"a restore ran at the boundary\n{r.trace}"


def test_F1_at_crawl_speed_passed_still_decides_at_the_boundary(monkeypatch):
  """At ~10 mph the pass margin is its 30 m floor (~6.7 s), longer than the gone grace: mapd zeroing `next` at d = 0 must
  still not read as "gone" while the current limit catches up (5.5 s here)."""
  class Rd:
    t_cross = None

    def __call__(self, x, t):
      if x < 300:
        return 20, 10, 300 - x
      if self.t_cross is None:
        self.t_cross = t
      return (20, 0, 0.0) if t - self.t_cross < 5.5 else (10, 0, 0.0)
  r = simulate(monkeypatch, Rd(), 120, v0_mph=20, set0_mph=20)
  assert _acts(r, "start") and not _acts(r, "abort") and _acts(r, "promote"), r.la


def test_F2_a_false_chained_announcement_restores_to_the_real_zone_speed(monkeypatch):
  """Real 60 -> 40 (rule 1: 50), then a 35 announced 200 m on that never comes: the look-ahead took the set toward 43.75.
  It must come back up to 50 -- the 40 zone's own speed -- not stay at 44, and never above 50."""
  def road(x, t):
    if x < XB:
      return 60, 40, XB - x
    if x < XB + 200:
      return 40, 35, XB + 200 - x
    return 40, 0, 0.0
  r = simulate(monkeypatch, road, 140)
  assert _acts(r, "retarget") and _acts(r, "abort")[0]["limitDropJoined"] is True, r.la
  assert _acts(r, "abort")[0]["restore"] == pytest.approx(50 * MPH, abs=0.01)
  assert abs(r.final - 50) <= 1.0, f"final {r.final}\n{r.trace}"
  assert max(row[3] for row in r.trace if row[1] > XB + 200) <= 50 + 1.0


def test_F5_switching_the_look_ahead_off_puts_an_unconfirmed_drop_back_through_the_confirm(monkeypatch):
  """A 1.5 s bogus 40 at the boundary, taken at once because the look-ahead announced it; the look-ahead is then switched
  off. That value must not become a permanent zone just because the look-ahead stopped owning it."""
  class Rd:
    t_cross = None

    def __call__(self, x, t):
      if x < XB:
        return 60, 40, XB - x
      if self.t_cross is None:
        self.t_cross = t
      return (40, 0, 0.0) if t - self.t_cross < 1.5 else (60, 0, 0.0)
  state = {"off": False}

  def driver(t, run):
    if not state["off"] and run.c._sl == pytest.approx(40 * MPH):
      state["off"] = True
      run.c.params.la_mode = sa.LA_OFF
    return None
  r = simulate(monkeypatch, Rd(), 110, driver=driver)
  assert state["off"] and _acts(r, "confirmSkipped"), r.la
  assert [a["reason"] for a in _acts(r, "abort")] == ["modeOff"], r.la
  assert not [n for n, _ in r.events if n == "speedadjust_zone_set"], "the unconfirmed 40 became a permanent zone"
  assert r.final == pytest.approx(75.0, abs=0.01), f"{r.final}\n{r.trace}"


def test_a_rise_announced_ahead_does_nothing(monkeypatch):
  r = simulate(monkeypatch, lambda x, t: (60, 70, 500 - x) if x < 500 else (70, 0, 0.0), 40)
  assert not r.la and r.final == pytest.approx(75.0)


# ---- inputs and telemetry ---------------------------------------------------------------------------------------------
def test_a_stale_next_limit_is_no_announcement_and_is_logged(monkeypatch):
  errors = []
  monkeypatch.setattr(sa.cloudlog, "error", lambda msg, *a, **k: errors.append(msg))
  _Clock.t = 1000.0
  monkeypatch.setattr(sa, "time", _Clock)
  c = sa.SpeedAdjustController(types.SimpleNamespace(openpilotLongitudinalControl=False), params=_P(2, sa.LA_LIVE))
  c.mem_params = _Mem()
  c.mem_params.nxt, c.mem_params.next_ts_age = (40 * MPH, 300.0), sa.LA_INPUT_STALE_S + 0.5
  assert c._read_next_limit() is None
  assert any("NextMapSpeedLimit unusable" in m and "stale" in m for m in errors), errors
  c.mem_params.next_ts_age = 0.0
  assert c._read_next_limit() == (pytest.approx(40 * MPH, abs=1e-3), 300.0)


def test_an_unreadable_mode_is_off_and_logged(monkeypatch):
  errors = []
  monkeypatch.setattr(sa.cloudlog, "error", lambda msg, *a, **k: errors.append(msg))

  class _Bad:
    def get(self, k, return_default=True):
      raise KeyError(k)
  c = sa.SpeedAdjustController(types.SimpleNamespace(openpilotLongitudinalControl=False), params=_Bad())
  assert c._read_la_mode() == sa.LA_OFF
  assert any("LimitAheadMode unreadable" in m for m in errors), errors
  c.params = _P(2, 7)
  errors.clear()
  c._la_mode_err_t = None
  assert c._read_la_mode() == sa.LA_OFF and any("out of range" in m for m in errors), errors


def test_the_look_ahead_reaches_ces_events(monkeypatch):
  """The VTSCStatus trap: every look-ahead status key must be in the forwarded list, and an episode must show there."""
  r = simulate(monkeypatch, road_1857, 30)
  st = [val for k, val in r.c.mem_params.calls if k == "SpeedAdjustStatus"]
  la_keys = {k for k in st[-1] if k.startswith("la")}
  assert la_keys and la_keys <= set(SA_TELE_KEYS), la_keys - set(SA_TELE_KEYS)
  assert any(s.get("laN") is not None and s.get("laTgt") is not None and s.get("laD") is not None for s in st)
  assert st[-1]["laEvN"] == len(r.la) and st[-1]["laMode"] == sa.LA_LIVE
