"""restarea2pnw — the bundled I-5 rest-area data loads whole and is direction-correct.

Found 2026-09-24: the SeaTac northbound rest area (I-5 MP 140, Federal Way; WSDOT "SeaTac - I-5
northbound") was missing (the OSM gatherer skipped relations, and SeaTac is mapped as one), and
Maytown / Scatter Creek / Silver Lake carried dir "" although each serves only one direction
(WSDOT official names), so the southbound-only Maytown showed to northbound traffic.
"""

import inspect
import json
import math
import os

from openpilot.system.location_services import geo
from openpilot.system.location_services import location_servicesd as lsd

I5_FILE = os.path.join(lsd.REST_DIR, "i5_rest_areas.json")


def _i5_items():
  return [r for r in lsd.StaticData().rest if "I 5" in r["refs"]]


def _pick(lat, lon, brg):
  return lsd._line_rest_corridor(_i5_items(), lat, lon, brg, "I 5")


def test_every_i5_entry_loads_and_has_a_direction():
  # The loader skips a malformed entry without a word; count so a bad edit cannot vanish quietly.
  with open(I5_FILE) as f:
    raw = json.load(f)
  items = _i5_items()
  assert len(items) == len(raw)
  # Every I-5 rest area in WA/OR serves one side of the freeway; "" would show it in both directions.
  assert all(r["dir"] in ("N", "S") for r in items), [r["name"] for r in items if r["dir"] not in ("N", "S")]


def test_seatac_shows_northbound():
  # I-5 northbound near Fife/Milton, ~4 mi south of the rest area, heading NNE.
  r = _pick(47.22, -122.36, 30.0)
  assert r is not None
  poi, mi = r
  assert poi["name"] == "SeaTac"
  assert poi["dir"] == "N"
  assert 3.0 < mi < 5.5
  assert lsd.geo.haversine_m(poi["lat"], poi["lon"], 47.271134, -122.314557) < 300.0  # WSDOT coordinates


def test_seatac_hidden_southbound():
  # I-5 southbound just north of it, heading SSW: SeaTac is on the other side; nothing else within 15 mi.
  assert _pick(47.30, -122.29, 210.0) is None


def test_maytown_is_southbound_only():
  # Northbound between Scatter Creek (MP 90) and Maytown (MP 93): Maytown is SB-only, SeaTac is ~29 mi on.
  assert _pick(46.85, -122.98, 20.0) is None
  # Southbound north of Maytown: it shows.
  r = _pick(46.90, -122.955, 200.0)
  assert r is not None and r[0]["name"] == "Maytown"


# ---- restfar2pnw: rest areas reach REST_MAX_AHEAD_MI along a tagged corridor ------------------------------
LAT0, LON0 = 45.0, -122.0
MI_LAT = 1609.344 / (math.pi * 6371000.0 / 180.0)     # degrees of latitude per mile


def _ra(name, mi_north, d="N", refs=("I 5",), lon=LON0):
  return {"name": name, "lat": LAT0 + mi_north * MI_LAT, "lon": lon, "dir": d, "refs": refs, "town": ""}


def _far(items, at_mi, brg=0.0, wayref="I 5"):
  r = lsd._line_rest_corridor(items, LAT0 + at_mi * MI_LAT, LON0, brg, wayref, max_mi=lsd.REST_MAX_AHEAD_MI)
  return None if r is None else (r[0]["name"], r[1])


def test_reach_constant_meets_owner_minimum_and_is_far_only_for_rest():
  assert lsd.REST_MAX_AHEAD_MI == 50.0
  assert lsd.DISPLAY_MAX_MI == 15.0           # police/EV cap untouched
  assert lsd.EV_MAX_DIST_M == 6.0 * geo.M_PER_MILE


def test_next_rest_area_then_the_one_after_as_each_is_passed():
  items = [_ra("a12", 12), _ra("b35", 35), _ra("c48", 48), _ra("d70", 70),
           _ra("behind", -5), _ra("opposite", 20, d="S")]
  assert _far(items, 0)[0] == "a12"
  assert abs(_far(items, 0)[1] - 12.0) < 0.1
  assert _far(items, 13)[0] == "b35"          # a12 passed -> the NEXT one
  assert _far(items, 36)[0] == "c48"
  assert _far(items, 49)[0] == "d70"          # 21 mi away, inside the 50 mi reach


def test_a_49_mi_one_is_shown_and_a_70_mi_one_is_not():
  assert _far([_ra("e49", 49)], 0)[0] == "e49"
  assert _far([_ra("f70", 70)], 0) is None
  assert _far([_ra("g90", 90)], 0) is None


def test_never_behind_and_never_the_opposite_side():
  assert _far([_ra("behind", -30)], 0) is None
  assert _far([_ra("opp", 30, d="S")], 0) is None
  assert _far([_ra("opp", -30, d="N")], 0, brg=180.0) is None      # southbound: a N-side one is the other carriageway


def test_other_corridor_is_never_shown():
  assert _far([_ra("i90", 30, refs=("I 90",))], 0) is None


def test_missing_heading_clamps_to_the_old_15_mi():
  items = [_ra("near", 10), _ra("far", 40)]
  assert lsd._line_rest_corridor(items, LAT0, LON0, None, "I 5", max_mi=lsd.REST_MAX_AHEAD_MI)[0]["name"] == "near"
  assert lsd._line_rest_corridor([_ra("far", 40)], LAT0, LON0, None, "I 5", max_mi=lsd.REST_MAX_AHEAD_MI) is None


def test_missing_wayref_returns_none_so_the_caller_uses_the_15_mi_geometry():
  assert lsd._line_rest_corridor([_ra("a", 10)], LAT0, LON0, 0.0, "", max_mi=lsd.REST_MAX_AHEAD_MI) is None


def test_default_reach_is_unchanged_15_mi():
  assert lsd._line_rest_corridor([_ra("far", 60)], LAT0, LON0, 0.0, "I 5") is None
  assert lsd._line_rest_corridor([_ra("near", 14)], LAT0, LON0, 0.0, "I 5")[0]["name"] == "near"


def test_hold_keeps_a_far_target_and_drops_it_once_passed():
  poi = _ra("far", 60)
  h = lsd._Hold(lsd.POI_HOLD_S)
  assert h.update((poi, 60.0), 0.0, LAT0, LON0)[1] == 60.0
  assert h.update(None, 1.0, LAT0 + 1 * MI_LAT, LON0)[0] is poi            # selection blip: held
  assert h.update(None, 2.0, LAT0 + 59.9 * MI_LAT, LON0)[0] is poi          # about to reach it: still held
  assert h.update(None, 3.0, LAT0 + 61 * MI_LAT, LON0) is None             # passed: dropped
  better = _ra("nearer", 30)
  h.update((poi, 59.0), 4.0, LAT0 + 1 * MI_LAT, LON0)
  assert h.update((better, 29.0), 5.0, LAT0 + 1 * MI_LAT, LON0)[0] is better


def _real_i5():
  return _i5_items()


def test_real_i5_nearest_ahead_each_direction_from_a_midpoint():
  items = _real_i5()
  # midpoint of the I-5 data (mid-WA/OR); every direction's pick must be the nearest by dist among items
  # that serve that direction and lie ahead.
  mid_lat, mid_lon = 46.6, -122.9
  nb = lsd._line_rest_corridor(items, mid_lat, mid_lon, 0.0, "I 5", max_mi=lsd.REST_MAX_AHEAD_MI)
  sb = lsd._line_rest_corridor(items, mid_lat, mid_lon, 180.0, "I 5", max_mi=lsd.REST_MAX_AHEAD_MI)
  assert nb is not None and sb is not None
  assert nb[0]["dir"] == "N" and sb[0]["dir"] == "S"
  assert nb[0]["lat"] > mid_lat and sb[0]["lat"] < mid_lat
  exp_n = min(geo.haversine_m(mid_lat, mid_lon, r["lat"], r["lon"]) for r in items
              if r["dir"] == "N" and r["lat"] > mid_lat and geo.haversine_m(mid_lat, mid_lon, r["lat"], r["lon"]) / geo.M_PER_MILE <= lsd.REST_MAX_AHEAD_MI)
  assert abs(nb[1] - exp_n / geo.M_PER_MILE) < 0.1
  exp_s = min(geo.haversine_m(mid_lat, mid_lon, r["lat"], r["lon"]) for r in items
              if r["dir"] == "S" and r["lat"] < mid_lat and geo.haversine_m(mid_lat, mid_lon, r["lat"], r["lon"]) / geo.M_PER_MILE <= lsd.REST_MAX_AHEAD_MI)
  assert abs(sb[1] - exp_s / geo.M_PER_MILE) < 0.1


# ---- review round 1: select_rest helper, shared WayRef, long-baseline course ---------------------------------
def _at(n_mi, e_mi):
  """lat/lon n_mi north and e_mi east of (LAT0, LON0)."""
  return LAT0 + n_mi * MI_LAT, LON0 + e_mi * MI_LAT / math.cos(math.radians(LAT0))


def _ra_at(name, n_mi, e_mi, d="N", refs=("I 5",)):
  lat, lon = _at(n_mi, e_mi)
  return {"name": name, "lat": lat, "lon": lon, "dir": d, "refs": refs, "town": ""}


def _sel(items, wayref="I 5", brg=0.0, corridor_brg=0.0, path=None, on_freeway=True, at=(0, 0)):
  lat, lon = _at(*at)
  return lsd.select_rest(items, lat, lon, brg, corridor_brg, path or [], wayref, on_freeway, max_mi=lsd.REST_MAX_AHEAD_MI)


def _name(r):
  return None if r[0] is None else r[0][0]["name"]


def test_wayref_parts_split_and_normalise():
  assert lsd._wayref_parts("I 5;US 12") == {"I 5", "US 12"}
  assert lsd._wayref_parts(" i  5 ; us 12 ") == {"I 5", "US 12"}
  assert lsd._wayref_parts("") == set() and lsd._wayref_parts(None) == set()


def test_shared_wayref_matches_either_order_and_single():
  items = [_ra("x", 30)]
  assert _far(items, 0, wayref="I 5;US 12")[0] == "x"
  assert _far(items, 0, wayref="US 12;I 5")[0] == "x"
  assert _far(items, 0, wayref="I 5")[0] == "x"
  assert _far(items, 0, wayref="US 97") is None
  assert _far(items, 0, wayref="US 97;OR 99E") is None


def test_select_rest_reaches_far_and_reports_mode():
  r = _sel([_ra("e49", 49)])
  assert _name(r) == "e49" and r[1] == "far"
  r = _sel([_ra("e10", 10)])
  assert _name(r) == "e10" and r[1] == "corridor"


def test_select_rest_fallback_stays_15_mi_with_the_perp_rule():
  # no WayRef: geometric fallback, cone ahead, 15 mi cap, 1.5 mi perpendicular rule
  r = _sel([_ra_at("near", 10, 0.5)], wayref="")
  assert _name(r) == "near" and r[1] == "15mi-no-wayref"
  assert _name(_sel([_ra_at("far", 30, 0.0)], wayref="")) is None          # 30 mi: NOT the far reach
  assert _name(_sel([_ra_at("wide", 10, 3.0)], wayref="")) is None          # 3 mi off the line: perp rule
  r = _sel([_ra_at("near", 10, 0.5, refs=("I 90",))], wayref="I 5")
  assert _name(r) == "near" and r[1] == "15mi-no-corridor-hit"
  assert _name(_sel([_ra_at("far", 30, 0.0, refs=("I 90",))], wayref="I 5")) is None


def test_select_rest_off_freeway_uses_the_surface_radius():
  r = _sel([_ra_at("a", 2, 0), _ra_at("b", 20, 0)], on_freeway=False)
  assert _name(r) == "a" and r[1] == "surface"
  assert _name(_sel([_ra_at("b", 20, 0)], on_freeway=False)) is None


def test_missing_course_clamps_to_15_in_select_rest():
  assert _name(_sel([_ra_at("m", 40, 0)], corridor_brg=None)) is None
  assert _name(_sel([_ra_at("m", 10, 0)], corridor_brg=None)) == "m"


def test_behind_is_strict_at_100_and_120_degrees():
  for bearing in (100.0, 120.0):
    n, e = 10 * math.cos(math.radians(bearing)), 10 * math.sin(math.radians(bearing))
    assert _name(_sel([_ra_at("b", n, e, d="")])) is None, bearing
  n, e = 10 * math.cos(math.radians(80.0)), 10 * math.sin(math.radians(80.0))
  assert _name(_sel([_ra_at("ok", n, e, d="")])) == "ok"


def test_mode_log_is_change_only(monkeypatch):
  lines = []
  monkeypatch.setattr(lsd.cloudlog, "info", lambda m, *a, **k: lines.append(a))
  log = lsd.RestModeLog()
  for m in ["far", "far", "far", "15mi-no-wayref", "15mi-no-wayref", "far"]:
    log.update(m, "I 5", "track")
  assert [a[1] for a in lines] == ["far", "15mi-no-wayref", "far"]
  assert lines[0][3] == "track"


def test_main_wires_the_helper_with_the_far_reach():
  src = inspect.getsource(lsd.main)
  assert src.count("select_rest(static.rest") == 1
  call = src[src.index("select_rest(static.rest"):]
  call = call[:call.index(")")]
  assert "max_mi=REST_MAX_AHEAD_MI" in call
  assert "heading_track.update(" in src and "rest_log.update(" in src
  assert "_line_rest_corridor(" not in src and "_line_static(static.rest" not in src


# --- HeadingTrack
def _drive(track, pts, brg_inst=None, t0=0.0, dt=10.0):
  out = None
  for i, (lat, lon) in enumerate(pts):
    out = track.update(t0 + i * dt, lat, lon, brg_inst)
  return out


def _line(n0, n1, step=0.05, e=0.0):
  n = n0
  while n <= n1 + 1e-9:
    yield _at(n, e)
    n += step


def test_track_uses_inst_when_short_then_the_long_baseline():
  tr = lsd.HeadingTrack()
  assert tr.update(0.0, *_at(0, 0), 123.0) == 123.0                       # one fix: instantaneous
  assert _drive(tr, list(_line(0, 0.5)), 123.0) == 123.0                  # <1 mi of history: instantaneous
  b = _drive(tr, list(_line(0.5, 8.0)), 123.0, t0=100.0)                  # 8 mi due north: course ~0, not 123
  assert b < 1.0 or b > 359.0
  assert _drive(lsd.HeadingTrack(), [_at(0, 0)], None) is None             # no history, no heading


def test_track_resets_on_a_jump_or_stale_gap():
  tr = lsd.HeadingTrack()
  _drive(tr, list(_line(0, 8.0)), 90.0)
  assert tr.update(500.0, *_at(30, 0), 90.0) == 90.0                       # teleport: history dropped
  tr2 = lsd.HeadingTrack()
  _drive(tr2, list(_line(0, 8.0)), 90.0)
  assert tr2.update(5000.0, *_at(8.0, 0.01), 45.0) == 45.0                 # stale: history dropped


def test_bend_case_real_i5_sideways_heading_still_finds_the_right_side():
  items = _i5_items()
  # northbound on I-5 south of SeaTac (real coordinates); the instantaneous heading is sideways (E-W bend)
  start = (47.10, -122.40)
  end = (47.22, -122.36)
  tr = lsd.HeadingTrack()
  pts = [(start[0] + (end[0] - start[0]) * i / 80, start[1] + (end[1] - start[1]) * i / 80) for i in range(81)]
  course = _drive(tr, pts, 100.0)
  assert course is not None and abs(geo.normalize180(course - 20.0)) < 15.0     # ~NNE, not 100
  lat, lon = end
  bad = lsd._line_rest_corridor(items, lat, lon, 100.0, "I 5", max_mi=lsd.REST_MAX_AHEAD_MI)
  good = lsd._line_rest_corridor(items, lat, lon, course, "I 5", max_mi=lsd.REST_MAX_AHEAD_MI)
  assert good[0]["name"] == "SeaTac" and good[0]["dir"] == "N"
  assert bad is None or bad[0]["name"] != "SeaTac"                              # the sideways heading loses it


def test_uturn_uses_the_instantaneous_heading_on_the_first_reversed_tick(monkeypatch):
  lines = []
  monkeypatch.setattr(lsd.cloudlog, "info", lambda m, *a, **k: lines.append((m, a)))
  tr = lsd.HeadingTrack()
  _drive(tr, list(_line(0, 8.0)), 0.0)                                  # 8 mi north
  assert tr.source == "track"
  b = tr.update(1000.0, *_at(7.95, 0), 180.0)                           # first southbound tick
  assert b == 180.0 and tr.source == "inst" and len(tr.crumbs) == 1
  assert any("reversal" in m or "reversal" in str(a) for m, a in lines)


def test_crumbs_are_bounded():
  tr = lsd.HeadingTrack()
  _drive(tr, list(_line(0, 50.0)), 0.0)
  assert len(tr.crumbs) <= lsd.HeadingTrack.MAX_CRUMBS


def test_crumb_spacing_is_respected():
  tr = lsd.HeadingTrack()
  _drive(tr, list(_line(0, 1.0, step=0.01)), 0.0)                       # 100 fixes in 1 mi
  assert 8 <= len(tr.crumbs) <= 12


def test_jump_and_gap_reset_and_log(monkeypatch):
  lines = []
  monkeypatch.setattr(lsd.cloudlog, "info", lambda m, *a, **k: lines.append(a))
  tr = lsd.HeadingTrack()
  _drive(tr, list(_line(0, 8.0)), 0.0)
  tr.update(500.0, *_at(30, 0), 0.0)
  assert len(tr.crumbs) == 1 and any("jump" in str(a) for a in lines)
  tr.update(5000.0, *_at(30, 0.001), 0.0)
  assert any("gap" in str(a) for a in lines)


def test_fallback_cone_follows_the_instantaneous_heading_not_the_course():
  # no WayRef -> geometric fallback. Course says north, instantaneous heading says east: the fallback must
  # use the instantaneous heading (a rest area 10 mi east shows; one 10 mi north does not).
  east, north = _ra_at("east", 0.0, 10.0), _ra_at("north", 10.0, 0.0)
  r = _sel([east], wayref="", brg=90.0, corridor_brg=0.0)
  assert _name(r) == "east"
  assert _name(_sel([north], wayref="", brg=90.0, corridor_brg=0.0)) is None


def test_most_recent_crumb_five_mi_back_is_used_on_a_dog_leg():
  tr = lsd.HeadingTrack()
  pts = list(_line(0, 6.0)) + [_at(6.0, e) for e in [i * 0.05 for i in range(1, 121)]]   # 6 mi N, then 6 mi E
  c = _drive(tr, pts, 90.0)
  # newest crumb >=5 mi back is on the north leg ending ~6 mi N,  ~1 mi E of the corner: bearing is NE-ish, not
  # the oldest-crumb bearing (start -> now, ~45 deg) -- they differ by well over 10 deg.
  oldest = geo.bearing_deg(*_at(0, 0), *pts[-1])
  assert abs(geo.normalize180(c - oldest)) > 10.0
