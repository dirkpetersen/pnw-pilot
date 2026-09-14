"""gpslag2pnw: ICBM projects its GPS ego position to every ~4 Hz tick, keeping today's start timing.

Owner decision 2026-09-13 ("build it, keep curve timing"):
  * project at every ICBM tick, from the fix's own `fix_ts`, to (now - ICBM_GPS_LAG_KEEP_S): the weekend's
    ICBM starts were decided on a position a median 1.79 s old; the keep (1.59 s) is that minus the truck fix's own
    ~0.20 s receipt lag, so the average start stays put on the truck fix (device-fix starts ~0.2 s earlier by design);
  * a fix older than ICBM_GPS_MAX_AGE_S (5 s) is no GPS for the map-curve lookups, logged change-only;
  * ICBM lookups only: CES's own upcoming_curve and VTSC read the blob exactly as before.

The approach scenarios drive the REAL CESController._icbm_step (a stub `self`, as test_curvelead2pnw does)
at 4 Hz, with ces_pnw's 1 Hz blob read and mapd's 1 Hz path publish emulated around it. The start of an
episode is the first tick that publishes a dec IcbmTarget; its TRUE distance to the curve is what is compared.
"""
import inspect
import json
import math
from collections import deque
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.controls.lib import pnw_vehicle as pv
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_ces_record_fields import _record
from openpilot.selfdrive.controls.lib.pnw_vehicle import PnwVehicle

LAT0, LON0 = 44.0, -121.3
V, VSET, MAPV, CURVE_AT = 25.0, 30.0, 15.0, 700.0


@pytest.fixture(autouse=True)
def _default_curve_cfg(tmp_path, monkeypatch):
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(tmp_path / "absent.json"))
  monkeypatch.setattr(pv, "RAIN_CONFIG_PATH", str(tmp_path / "absent-rain.json"))


def _ll(y, x=0.0):
  return LAT0 + y / 111320.0, LON0 + x / (111320.0 * math.cos(math.radians(LAT0)))


def _y(lat):
  return (lat - LAT0) * 111320.0


def _path(curve_len=400.0, after=0.0):
  """Straight (velocity 0 = no target) to CURVE_AT, `curve_len` m of MAPV curve points, then `after` m straight."""
  pts, y = [], -200.0
  while y < CURVE_AT + curve_len + after:
    la, lo = _ll(y)
    v = MAPV if CURVE_AT <= y < CURVE_AT + curve_len else 0.0
    pts.append({"latitude": la, "longitude": lo, "velocity": v})
    y += 10.0 if CURVE_AT <= y < CURVE_AT + curve_len else 20.0
  return pts


def _approach(monkeypatch, blob, t_end=40.0, vision_at=None, spy=None, read_every=1.0, follow_set=False,
              path=None, y_end=CURVE_AT, clear_at=None, events_out=None, vision_end=float("-inf"), init=None):
  """blob(t_read) -> None or (y_fix, x_fix, fix_ts or None, src): the LastGPSPosition ces_pnw's _read_map
  copies at t_read (1 Hz on the car). Returns (start (t, true distance to the curve, src) or None, published
  targets, logged gps events).

  follow_set: the stock set (and v_set, as on the car) follows the published target by 1 mph per 0.25 s tick,
  the way the executor's taps move it -- without it `_stock_set` never moves and a restore can never become
  eligible, so a test could not see one. path/y_end: a custom mapd path and how far (m) to drive. clear_at:
  from this time mapd reports no curve at all (its velocities read 0). events_out: all logged events."""
  clock = [0.0]
  monkeypatch.setattr(m.time, "monotonic", lambda: clock[0])
  monkeypatch.setattr(m.time, "time", lambda: clock[0])
  events = []
  monkeypatch.setattr(m.cloudlog, "event", lambda name, **kw: events.append((name, kw)))
  if spy is not None:
    real = m.icbm_project_position
    monkeypatch.setattr(m, "icbm_project_position", lambda *a, **k: spy(clock[0], real(*a, **k)))
  cls = next(o for o in vars(m).values() if inspect.isclass(o) and hasattr(o, "_icbm_step"))
  pubs = []
  c = SimpleNamespace(mem_params=SimpleNamespace(put_nonblocking=lambda k, v: pubs.append((clock[0], v)) if k == "IcbmTarget" else None))
  c._veh = PnwVehicle(SimpleNamespace(carFingerprint="FORD_F_150_LIGHTNING_MK1", brand="ford",
                                      openpilotLongitudinalControl=False, dashcamOnly=False))
  c._icbm_ep = m.IcbmEpisode()
  for k, v in dict(_icbm_ceiling=None, _icbm_dir=None, _map_targets=[], _cur_lat=None, _cur_lon=None, _cur_bearing=0.0,
                   _gps_fix_ts=None, _gps_src=None, _icbm_floor_lim=0.0, _icbm_floor_pend=None, _icbm_floor_hit=False,
                   _icbm_k=0.0, _icbm_k_n=0, _icbm_k_ahead=True, _icbm_k_at=0.0, _icbm_k_at_d=0.0, _icbm_k_at_n=0,
                   _icbm_k_at_gap=0.0, _stock_set=VSET, _stock_on=True, _icbm_last_pub=-1e9, **(init or {})).items():
    setattr(c, k, v)
  step = cls._icbm_step.__get__(c)
  pts, start, t = (path if path is not None else _path()), None, 0.0
  while t <= t_end and V * t < y_end:
    clock[0] = t
    if abs(t - round(t)) < 1e-9:                         # mapd path, 1 Hz
      cleared = clear_at is not None and t >= clear_at
      c._map_targets = [dict(p, velocity=0.0) if cleared else p for p in pts if V * t - 5.0 < _y(p["latitude"]) <= V * t + 500.0]
    if abs(t / read_every - round(t / read_every)) < 1e-9:   # ces_pnw blob read
      b = blob(t)
      if b is not None:
        (c._cur_lat, c._cur_lon), c._gps_fix_ts, c._gps_src = _ll(b[0], b[1]), b[2], b[3]
    mtv, mtd = m.upcoming_curve(c._map_targets, c._cur_lat, c._cur_lon, V, m.C.CURVE_MAP_LOOKAHEAD_S)   # the caller
    vis = vision_at is not None and vision_end < CURVE_AT - V * t <= vision_at
    sig = {"v_ego": V, "v_set": c._stock_set if follow_set else VSET, "map_target_v": mtv, "map_target_dist": mtd,
           "curve_lat_accel_vision": 4.0 if vis else 0.0, "time_to_curve": (CURVE_AT - V * t) / V if vis else 10.0,
           "lat_accel_now": 0.0, "has_lead": False, "lead_drel": 0.0, "lead_vlead": 0.0, "gas": False, "brake": False,
           "spd_lim": 0.0, "pitch": None, "vis_k_max": None, "vis_reach": 0.0}
    c._icbm_last_pub = -1e9
    step(sig, active=True)
    tgt = pubs[-1][1].get("target") if pubs else None
    if start is None and tgt is not None:
      start = (t, CURVE_AT - V * t, c._icbm_src)
    if follow_set and tgt is not None:                   # the executor: one tap per tick, dec only down, inc only up
      d = pubs[-1][1].get("dir", "dec")
      if d == "dec" and c._stock_set > tgt + 0.2:
        c._stock_set -= min(c._stock_set - tgt, 0.447)
      elif d == "inc" and c._stock_set < tgt - 0.2:
        c._stock_set += min(tgt - c._stock_set, 0.447)
    t = round(t + 0.25, 6)
  if events_out is not None:
    events_out.extend(events)
  return start, pubs, [kw for name, kw in events if name == "ces_icbm_gps"]


def _fresh(age, src="device"):
  """A healthy receiver: every read sees a fix `age` s old, with its exact fix_ts."""
  return lambda t: (V * (t - age), 0.0, t - age, src)


# --- the pure projection ---------------------------------------------------------------------------------

class TestProjectPosition:
  def test_projects_along_the_bearing_to_now_minus_keep(self):
    la, lo, age, st = m.icbm_project_position(LAT0, LON0, 0.0, 100.0, 20.0, 103.79)
    assert st == "proj" and age == pytest.approx(3.79)
    assert _y(la) == pytest.approx(20.0 * (3.79 - m.ICBM_GPS_LAG_KEEP_S), abs=1e-6) and lo == pytest.approx(LON0)
    la, lo, _, _ = m.icbm_project_position(LAT0, LON0, 90.0, 100.0, 20.0, 103.79)
    east = (lo - LON0) * 111320.0 * math.cos(math.radians(LAT0))
    assert east == pytest.approx(20.0 * (3.79 - m.ICBM_GPS_LAG_KEEP_S), abs=0.01) and la == pytest.approx(LAT0)

  def test_a_fresher_fix_than_the_keep_projects_backwards(self):
    la, _, _, st = m.icbm_project_position(LAT0, LON0, 0.0, 100.0, 20.0, 100.0 + m.ICBM_GPS_LAG_KEEP_S - 1.0)
    assert st == "proj" and _y(la) == pytest.approx(-20.0, abs=1e-6)

  @pytest.mark.parametrize("now,state", [(105.0, "proj"), (105.01, "stale"), (99.9, "stale")])
  def test_age_window(self, now, state):
    la, lo, _, st = m.icbm_project_position(LAT0, LON0, 0.0, 100.0, 20.0, now)
    assert st == state and ((la is None) == (state == "stale"))

  @pytest.mark.parametrize("args,state", [((LAT0, LON0, 0.0, None, 20.0, 1.0), "raw"),
                                          ((LAT0, LON0, None, 100.0, 20.0, 101.0), "raw"),
                                          ((LAT0, LON0, float("nan"), 100.0, 20.0, 101.0), "raw"),
                                          ((None, None, 0.0, 100.0, 20.0, 101.0), "none")])
  def test_fallbacks(self, args, state):
    la, lo, _, st = m.icbm_project_position(*args)
    assert st == state and ((la, lo) == (args[0], args[1]))

  def test_constants_are_the_owners(self):
    assert m.ICBM_GPS_LAG_KEEP_S == 1.59 and m.ICBM_GPS_MAX_AGE_S == 5.0


# --- through _icbm_step ----------------------------------------------------------------------------------

class TestStartTiming:
  def test_the_fix_age_no_longer_moves_the_start(self, monkeypatch):
    starts = {age: _approach(monkeypatch, _fresh(age))[0] for age in (0.6, 1.2, 2.0, 3.0)}
    assert all(s is not None for s in starts.values()), starts
    assert len({s[1] for s in starts.values()}) == 1, f"start distance depends on the fix age: {starts}"

  def test_the_start_is_where_a_position_exactly_the_keep_old_starts_it(self, monkeypatch):
    projected = _approach(monkeypatch, _fresh(0.4))[0]
    # the same code with NO fix_ts (raw, unprojected) and a position exactly ICBM_GPS_LAG_KEEP_S old, re-read
    # every tick: the lag the weekend's starts were decided on, as a constant
    lagged = _approach(monkeypatch, lambda t: (V * (t - m.ICBM_GPS_LAG_KEEP_S), 0.0, None, "device"), read_every=0.25)[0]
    assert projected is not None and lagged is not None
    assert projected[1] == pytest.approx(lagged[1], abs=1e-6), (projected, lagged)
    # and it is not where an up-to-date position would start it (the owner's "keep curve timing")
    now_pos = _approach(monkeypatch, lambda t: (V * t, 0.0, None, "device"), read_every=0.25)[0]
    assert now_pos[1] - projected[1] >= V * m.ICBM_GPS_LAG_KEEP_S - V * 0.25


class TestStaleFix:
  def test_a_fix_older_than_5_s_is_no_gps_for_the_map_lookups_and_it_is_logged_once(self, monkeypatch):
    fresh_start = _approach(monkeypatch, _fresh(0.5))[0]
    assert fresh_start is not None
    # the tunnel: the receiver froze 10 s before the fresh start, then comes back 6 s before the curve
    freeze = fresh_start[0] - 10.0
    back = (CURVE_AT / V) - 6.0

    def tunnel(t):
      if t < freeze or t >= back:
        return (V * (t - 0.5), 0.0, t - 0.5, "device")
      return (V * (freeze - 0.5), 0.0, freeze - 0.5, "device")    # the frozen blob, fix_ts and all
    start, pubs, ev = _approach(monkeypatch, tunnel)
    assert start is not None and start[0] >= back, f"a map/far start ran on a frozen fix: {start}"
    assert [e["state"] for e in ev] == ["proj", "stale", "proj"], ev
    assert all(t < freeze + m.ICBM_GPS_MAX_AGE_S or t >= back or not p for t, p in pubs), "published a map cap while stale"

  def test_vision_still_starts_while_the_fix_is_stale(self, monkeypatch):
    frozen = lambda t: (0.0, 0.0, 0.0, "device")   # noqa: E731  -- a fix from t=0 that never updates
    start, _, ev = _approach(monkeypatch, frozen, vision_at=150.0)
    assert start is not None and start[2] == "vis"
    assert "stale" in [e["state"] for e in ev]


class TestSourceSwitch:
  """Fable, gpssel2pnw note 3: a receiver switch steps the position ~6 m. The projection must not turn it into
  a distance jump that starts or ends an episode."""

  @pytest.mark.parametrize("along", [+6.0, -6.0])
  def test_a_switch_mid_approach_moves_the_start_by_at_most_the_step(self, monkeypatch, along):
    base = _approach(monkeypatch, _fresh(0.6))[0]
    t_sw = base[0] - 2.0
    # device until t_sw; then the truck: its fix is `along` m off the device's and 0.2 s older than its receipt
    # (GPS_TRUCK_VS_COMMA.md s2), and fix_ts is the receipt, as mapd_configd writes it
    blob = lambda t: ((V * (t - 0.6), 3.0, t - 0.6, "device") if t < t_sw          # noqa: E731
                      else (V * (t - 1.0 - 0.2) + along, 0.0, t - 1.0, "car"))
    seen = []
    start, pubs, _ = _approach(monkeypatch, blob, spy=lambda t, r: seen.append((t, r)) or r)
    assert start is not None
    assert abs(start[1] - base[1]) <= abs(along) + V * 0.2 + V * 0.25, (base, start)
    ys = [(t, _y(r[0])) for t, r in seen if r[0] is not None]
    steps = [abs((y1 - y0) - V * (t1 - t0)) for (t0, y0), (t1, y1) in zip(ys, ys[1:], strict=False)]
    assert max(steps) <= abs(along) + V * 0.2 + 0.5, f"the projected position jumped {max(steps):.1f} m"

  def test_a_switch_inside_the_episode_does_not_end_it(self, monkeypatch):
    """Fable, gpslag review (b): the first version could not see a restore -- the stub's stock set never moved,
    so a restore was never eligible. The set now follows the published caps, and the positive control below
    proves this harness DOES see a restore when the curve really clears."""
    base = _approach(monkeypatch, _fresh(0.6))[0]
    t_sw = base[0] + 3.0
    blob = lambda t: ((V * (t - 0.6), 3.0, t - 0.6, "device") if t < t_sw          # noqa: E731
                      else (V * (t - 1.2) - 6.0, 0.0, t - 1.0, "car"))
    start, pubs, _ = _approach(monkeypatch, blob, follow_set=True)
    after = [p for t, p in pubs if t >= start[0]]
    assert after and all(p.get("target") is not None and p.get("dir", "dec") == "dec" for p in after), \
      "the episode ended or turned into a restore across the switch"

  def test_positive_control_the_harness_sees_a_restore_when_the_curve_really_clears(self, monkeypatch):
    base = _approach(monkeypatch, _fresh(0.6))[0]
    _, pubs, _ = _approach(monkeypatch, _fresh(0.6), follow_set=True, clear_at=float(math.ceil(base[0] + 3.0)))
    assert any(p.get("dir") == "inc" for _, p in pubs), "no restore even on a real clear: the harness is blind"


class TestEveryIcbmLookupUsesTheProjectedPosition:
  """One position per tick for ICBM. A lookup left on the 1 Hz held position would disagree with the others by up to
  v*(age - keep) -- the class of bug that no timing test sees when a different candidate happens to start first."""

  def _tick(self, monkeypatch, fix_ts, now=100.0, sig_map=None, held_dist=300.0):
    calls = []

    def spy(name, pos_idx=(1, 2)):
      real = getattr(m, name)

      def f(*a, **k):
        calls.append((name, a[pos_idx[0]], a[pos_idx[1]]))
        return real(*a, **k)
      monkeypatch.setattr(m, name, f)
    for n in ("icbm_far_map_candidate", "icbm_map_reach", "upcoming_curve", "map_turn_direction",
              "polyline_curvature_at", "icbm_path_behind"):
      spy(n)
    seen = []
    real_ct = m.icbm_curve_target
    monkeypatch.setattr(m, "icbm_curve_target", lambda *a, **k: seen.append((a[2], a[3])) or real_ct(*a, **k))
    monkeypatch.setattr(m.time, "monotonic", lambda: now)
    monkeypatch.setattr(m.cloudlog, "event", lambda *a, **k: None)
    cls = next(o for o in vars(m).values() if inspect.isclass(o) and hasattr(o, "_icbm_step"))
    pubs = []
    c = SimpleNamespace(mem_params=SimpleNamespace(put_nonblocking=lambda k, v: pubs.append(v) if k == "IcbmTarget" else None))
    c._veh = PnwVehicle(SimpleNamespace(carFingerprint="FORD_F_150_LIGHTNING_MK1", brand="ford",
                                        openpilotLongitudinalControl=False, dashcamOnly=False))
    c._icbm_ep = m.IcbmEpisode()
    held_y = CURVE_AT - held_dist
    pts = [p for p in _path() if held_y - 5.0 < _y(p["latitude"]) <= held_y + 500.0]
    lat, lon = _ll(held_y)
    for k, v in dict(_icbm_ceiling=None, _icbm_dir=None, _map_targets=pts, _cur_lat=lat, _cur_lon=lon, _cur_bearing=0.0,
                     _gps_fix_ts=fix_ts, _gps_src="car", _icbm_floor_lim=0.0, _icbm_floor_pend=None, _icbm_floor_hit=False,
                     _icbm_k=0.0, _icbm_k_n=0, _icbm_k_ahead=True, _icbm_k_at=0.0, _icbm_k_at_d=0.0, _icbm_k_at_n=0,
                     _icbm_k_at_gap=0.0, _stock_set=VSET, _stock_on=True, _icbm_last_pub=-1e9).items():
      setattr(c, k, v)
    mtv, mtd = sig_map if sig_map is not None else m.upcoming_curve(pts, lat, lon, V, m.C.CURVE_MAP_LOOKAHEAD_S)
    calls.clear()
    sig = {"v_ego": V, "v_set": VSET, "map_target_v": mtv, "map_target_dist": mtd, "curve_lat_accel_vision": 0.0,
           "time_to_curve": 10.0, "lat_accel_now": 0.0, "has_lead": False, "lead_drel": 0.0, "lead_vlead": 0.0,
           "gas": False, "brake": False, "spd_lim": 0.0, "pitch": None, "vis_k_max": None, "vis_reach": 0.0}
    cls._icbm_step.__get__(c)(sig, active=True)
    return c, calls, seen, pubs, (lat, lon)

  # 300 m held -> 250 m projected: inside the 10 s near window, the near "map" candidate starts it;
  # 310 m held -> 260 m: beyond the window, the full-horizon "far" candidate does (each has its own lookups)
  @pytest.mark.parametrize("held_dist,src", [(300.0, "map"), (310.0, "far")])
  def test_all_lookups_in_a_binding_tick_see_one_projected_position(self, monkeypatch, held_dist, src):
    fix_ts = 100.0 - (m.ICBM_GPS_LAG_KEEP_S + 2.0)            # 3.79 s old: 2 s of travel past the keep, +50 m
    c, calls, seen, pubs, held = self._tick(monkeypatch, fix_ts, held_dist=held_dist)
    assert pubs and pubs[-1].get("target") is not None, "the tick did not bind -- the lookups below prove nothing"
    assert c._icbm_src == src, f"expected a {src} start, got {c._icbm_src}"
    names = {n for n, _, _ in calls}
    assert names >= {"icbm_far_map_candidate", "icbm_map_reach", "upcoming_curve", "map_turn_direction",
                     "polyline_curvature_at", "icbm_path_behind"}, names
    exp = m.icbm_project_position(held[0], held[1], 0.0, fix_ts, V, 100.0)[:2]
    assert _y(exp[0]) - _y(held[0]) == pytest.approx(V * 2.0, abs=1e-6)
    wrong = [(n, round(_y(la) - _y(exp[0]), 1)) for n, la, lo in calls if (la, lo) != exp]
    assert not wrong, f"lookups off the projected position (m): {wrong}"
    mtv, mtd = m.upcoming_curve(c._map_targets, exp[0], exp[1], V, m.C.CURVE_MAP_LOOKAHEAD_S)
    assert seen[0] == (mtv, mtd), "icbm_curve_target was not given the near candidate from the projected position"

  def test_a_stale_fix_drops_the_callers_binding_map_candidate(self, monkeypatch):
    # the caller (CES) computed a binding candidate from the frozen held position; ICBM must not use it
    c, calls, seen, pubs, _ = self._tick(monkeypatch, fix_ts=100.0 - 6.0, sig_map=(MAPV, 60.0))
    assert seen and all(s == (0.0, float("inf")) for s in seen), seen
    assert not pubs or pubs[-1].get("target") is None
    assert {n for n, la, lo in calls if la is not None} == set(), "a lookup ran on a stale position"


class TestStaleGpsHoldsTheRunningCap:
  """gpsdrgate2pnw (Fable, gpslag2pnw review (a); probe fable_gpslag_stale_midepisode.py): both receivers gone for
  more than ICBM_GPS_MAX_AGE_S in the MIDDLE of a map/far cap episode. Without a hold the vanished candidates read
  as "curve cleared": the publish emptied 137 m before the curve and a restore could start toward it. The running
  cap is held until the truck has driven past where its binding candidate was, then the normal path resumes."""

  PATH = dict(curve_len=100.0, after=900.0)

  def _frozen_after(self, t_freeze, resume=None):
    def blob(t):
      if t < t_freeze or (resume is not None and t >= resume):
        return (V * (t - 0.5), 0.0, t - 0.5, "car")
      return (V * (t_freeze - 0.5), 0.0, t_freeze - 0.5, "car")
    return blob

  def _run(self, monkeypatch, blob, **kw):
    ev = []
    start, pubs, gps = _approach(monkeypatch, blob, follow_set=True, path=_path(**self.PATH), y_end=CURVE_AT + 700.0,
                                 t_end=80.0, events_out=ev, **kw)
    return start, pubs, [k for n, k in ev if n == "ces_icbm_stale_hold"]

  def test_the_cap_is_held_past_the_curve_then_released_and_only_then_restored(self, monkeypatch):
    base = _approach(monkeypatch, _fresh(0.5), follow_set=True, path=_path(**self.PATH), y_end=CURVE_AT + 700.0)[0]
    freeze = base[0] + 2.0
    start, pubs, hold = self._run(monkeypatch, self._frozen_after(freeze))
    assert start == base
    passed_t = (CURVE_AT + m.ICBM_MARGIN_M) / V                  # the truck is past the curve start + margin
    before = [(t, p) for t, p in pubs if start[0] <= t < passed_t]
    assert before and all(p.get("target") is not None and p.get("dir", "dec") == "dec" for _, p in before), \
      [(t, p) for t, p in before if p.get("target") is None or p.get("dir") == "inc"][:3]
    assert [h["state"] for h in hold] == ["hold", "release"] and hold[1]["why"] == "passed", hold
    stale_t = freeze + m.ICBM_GPS_MAX_AGE_S
    held = {p["target"] for t, p in pubs if stale_t + 0.5 <= t < passed_t}
    assert len(held) == 1, f"the held cap moved: {held}"
    assert any(p.get("dir") == "inc" for t, p in pubs if t >= passed_t), "never restored after the curve was passed"
    # released as PASSED, the episode takes the fast apex-passed restore path (the hold fed it the shrinking distance)
    last_dec = max(t for t, p in pubs if p.get("target") is not None and p.get("dir", "dec") == "dec")
    first_inc = min(t for t, p in pubs if p.get("dir") == "inc")
    assert first_inc - last_dec <= m.ICBM_RESTORE_DELAY_FAST_S + 0.5, (last_dec, first_inc)

  def test_the_hold_is_bounded_in_time(self, monkeypatch):
    monkeypatch.setattr(m, "ICBM_GPS_STALE_HOLD_MAX_S", 2.0)
    base = _approach(monkeypatch, _fresh(0.5), follow_set=True, path=_path(**self.PATH), y_end=CURVE_AT + 700.0)[0]
    freeze = base[0] + 2.0
    _, pubs, hold = self._run(monkeypatch, self._frozen_after(freeze))
    assert [h["state"] for h in hold][:2] == ["hold", "release"] and hold[1]["why"] == "maxTime", hold
    assert 2.0 <= hold[1]["held_s"] <= 2.5
    # Fable B2: the curve is still unlocated at the time bound, so the episode ends WITHOUT a restore
    released_t = freeze + m.ICBM_GPS_MAX_AGE_S + hold[1]["held_s"]
    assert not [(t, p) for t, p in pubs if p.get("dir") == "inc"], "restored toward an unlocated curve after maxTime"
    assert all(p.get("target") is None for t, p in pubs if t >= released_t + 0.5), "kept publishing after maxTime"

  def test_vision_co_binding_does_not_erase_the_map_provenance(self, monkeypatch):
    """Fable B1: far cap at ~306 m, the fix freezes 1 s later, vision joins 200 -> 130 m and drops. A last-binder
    record saw a vision cap and declined the hold: restore published 50 m before the map-rated curve."""
    base = _approach(monkeypatch, _fresh(0.5), follow_set=True, path=_path(**self.PATH), y_end=CURVE_AT + 700.0)[0]
    freeze = base[0] + 1.0
    start, pubs, hold = self._run(monkeypatch, self._frozen_after(freeze), vision_at=200.0, vision_end=130.0)
    assert start == base
    assert [h["state"] for h in hold][:1] == ["hold"], hold
    inc_before = [round(V * t - CURVE_AT, 1) for t, p in pubs if p.get("dir") == "inc" and V * t < CURVE_AT + m.ICBM_MARGIN_M]
    assert not inc_before, f"restore published this many m before the curve start: {inc_before}"

  def test_the_fix_coming_back_ends_the_hold_and_the_map_takes_over(self, monkeypatch):
    base = _approach(monkeypatch, _fresh(0.5), follow_set=True, path=_path(**self.PATH), y_end=CURVE_AT + 700.0)[0]
    freeze = base[0] + 1.0
    resume = freeze + m.ICBM_GPS_MAX_AGE_S + 3.0
    start, pubs, hold = self._run(monkeypatch, self._frozen_after(freeze, resume=resume))
    assert hold and hold[0]["state"] == "hold" and hold[1] == {**hold[1], "state": "release", "why": "gpsBack"}
    assert all(p.get("target") is not None for t, p in pubs if start[0] <= t < CURVE_AT / V), "the cap dropped out"

  def test_a_vision_sourced_episode_is_not_held(self, monkeypatch):
    """GPS going stale does not change what VISION sees, so a vision cap that clears has really cleared."""
    straight = [dict(p, velocity=0.0) for p in _path(**self.PATH)]            # the map rates nothing
    ev = []
    # seeded with a "far" provenance left over from an EARLIER episode: sticky within an episode, never across
    start, pubs, _ = _approach(monkeypatch, self._frozen_after(5.0), follow_set=True, path=straight,
                               y_end=CURVE_AT + 300.0, t_end=80.0, vision_at=150.0, vision_end=40.0, events_out=ev,
                               init={"_icbm_cap_src": "far"})
    assert start is not None and start[2] == "vis"
    hold = [k for n, k in ev if n == "ces_icbm_stale_hold"]
    assert [(h["state"], h["why"]) for h in hold] == [("declined", "notMap")], hold   # logged once, never held

  def test_a_vision_curve_binding_ends_the_hold(self, monkeypatch):
    base = _approach(monkeypatch, _fresh(0.5), follow_set=True, path=_path(**self.PATH), y_end=CURVE_AT + 700.0)[0]
    freeze = base[0] + 1.0
    _, pubs, hold = self._run(monkeypatch, self._frozen_after(freeze), vision_at=60.0)
    assert [h["state"] for h in hold][:2] == ["hold", "release"] and hold[1]["why"] == "capBound", hold


# --- telemetry, and the Tesla ----------------------------------------------------------------------------

def test_icbmGpsAge_reaches_the_record():
  assert _record(_icbm_gps_age=1.25)["icbmGpsAge"] == 1.25
  assert _record(_icbm_gps_age=None)["icbmGpsAge"] is None


class _Mem:
  def __init__(self, pos):
    self.d = {"LastGPSPosition": pos, "MapTargetVelocities": _path()}

  def get(self, key, return_default=False):
    return self.d.get(key)


def test_ces_and_vtsc_read_the_same_position_with_or_without_fix_ts():
  """What the Tesla's CES upcoming_curve and VTSC consume: identical with the new key. Only ICBM (Lightning,
  PnwVehicle.ces_shadow) reads fix_ts."""
  from openpilot.selfdrive.controls.lib.vtsc_pnw.vtsc_controller import VTSCController
  base = {"latitude": 44.1, "longitude": -121.3, "bearing": 12.0, "speed": 20.0, "src": "device", "ts": 5.0}
  states, fix_ts_read = [], []
  for pos in (json.dumps(base), json.dumps({**base, "fix_ts": 4.43})):
    cls = next(o for o in vars(m).values() if inspect.isclass(o) and hasattr(o, "_read_map") and hasattr(o, "_icbm_step"))

    class Stub:
      def __getattr__(self, n):
        return None
    g = Stub()
    g.mem_params, g._toggles, g._vtsc_tele, g._bearing_hist = _Mem(pos), {"curves": True}, {}, deque(maxlen=8)
    cls._read_map.__get__(g)()
    fix_ts_read.append(g._gps_fix_ts)
    vt = object.__new__(VTSCController)
    vt.mem_params = _Mem(pos)
    vt._read_map()
    states.append(((g._cur_lat, g._cur_lon, g._cur_bearing, g._map_targets == _path(), g._gps_src),
                   (vt._cur_lat, vt._cur_lon, vt._cur_bearing, vt._map_targets),
                   m.upcoming_curve(g._map_targets, g._cur_lat, g._cur_lon, 20.0, m.C.CURVE_MAP_LOOKAHEAD_S)))
  assert states[0] == states[1]
  assert fix_ts_read == [None, 4.43], "ces_pnw _read_map did not take fix_ts from the blob"
  assert PnwVehicle(SimpleNamespace(carFingerprint="TESLA_MODEL_S_HW3", brand="tesla", openpilotLongitudinalControl=True,
                                    dashcamOnly=False)).ces_shadow is False
