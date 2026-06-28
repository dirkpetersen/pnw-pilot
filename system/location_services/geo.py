"""
location2pnw: pure "what's ahead" geometry for pnw_location_services.

PURE — no openpilot/cereal imports, so it's unit-testable standalone (same convention as the
vtsc_pnw / ces_pnw cores). All distances in METERS; bearings in degrees.

The one load-bearing idea (from POLICE_WARNING_DESIGN.md §5-§6): a POI's distance "ahead" is the
ALONG-TRACK distance projected onto our predicted path (mapd `MapTargetVelocities`), NOT the radial
haversine — a point offset to the side of a curving road never reaches range~0, so "behind us" must be
tested by along-track SIGN. The path is SHORT (~1-2 km, split at ramps/bridges), so we degrade
gracefully past its end to a forward-cone + haversine fallback.
"""
import math

_R_EARTH_M = 6371000.0
M_PER_MILE = 1609.344


def haversine_m(lat1, lon1, lat2, lon2):
  p1, p2 = math.radians(lat1), math.radians(lat2)
  dphi = math.radians(lat2 - lat1)
  dlam = math.radians(lon2 - lon1)
  a = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlam / 2) ** 2
  return 2 * _R_EARTH_M * math.asin(math.sqrt(a))


def bearing_deg(lat1, lon1, lat2, lon2):
  p1, p2 = math.radians(lat1), math.radians(lat2)
  dlam = math.radians(lon2 - lon1)
  y = math.sin(dlam) * math.cos(p2)
  x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dlam)
  return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def normalize180(angle):
  return (angle + 180.0) % 360.0 - 180.0


def to_local(lat0, lon0, lat, lon):
  """Equirectangular projection to local east/north METERS about (lat0, lon0). Accurate to well under
  0.1% over the few-km windows we use, and cheap — good enough for segment projection."""
  k = math.cos(math.radians(lat0))
  x = math.radians(lon - lon0) * _R_EARTH_M * k     # east
  y = math.radians(lat - lat0) * _R_EARTH_M         # north
  return x, y


def _project_point_to_polyline(pts_xy, px, py):
  """Project (px,py) onto the polyline `pts_xy` (list of (x,y) meters). Returns
  (along_m, perp_m, past_end): along-track distance to the nearest projection, absolute perpendicular
  offset, and whether the nearest projection fell at/after the LAST vertex (POI is past the path end).
  Empty/one-point path -> (0.0, inf, True)."""
  n = len(pts_xy)
  if n < 2:
    return 0.0, float('inf'), True
  best_perp = float('inf')
  best_along = 0.0
  best_past = True
  cum = 0.0
  for i in range(n - 1):
    ax, ay = pts_xy[i]
    bx, by = pts_xy[i + 1]
    dx, dy = bx - ax, by - ay
    seg_len2 = dx * dx + dy * dy
    if seg_len2 <= 1e-9:
      continue
    # parametric projection t of P onto segment AB, clamped to [0,1]
    t = ((px - ax) * dx + (py - ay) * dy) / seg_len2
    t_clamped = max(0.0, min(1.0, t))
    cx, cy = ax + t_clamped * dx, ay + t_clamped * dy
    perp = math.hypot(px - cx, py - cy)
    if perp < best_perp:
      best_perp = perp
      seg_len = math.sqrt(seg_len2)
      best_along = cum + t_clamped * seg_len
      # "past the end" only if we clamped to the very end of the final segment
      best_past = (i == n - 2) and (t >= 1.0)
    cum += math.sqrt(seg_len2)
  return best_along, best_perp, best_past


def ahead(path, cur_lat, cur_lon, cur_bearing, poi_lat, poi_lon,
          cone_deg=60.0, max_fallback_m=12 * M_PER_MILE):
  """Where is a POI relative to us along our path? Returns a dict
    {along_m, perp_m, source: 'path'|'cone'}  if the POI is AHEAD (and within fallback range), else None.

  `path`: list of dicts with 'latitude'/'longitude' (mapd MapTargetVelocities), our path ahead — may be
  empty. `cur_bearing`: our heading deg (may be None). Behind/off-cone -> None (never a phantom hit).
  """
  # --- path (along-track) branch: trust it while the POI projects onto the polyline interior ---
  if path and len(path) >= 2:
    try:
      lat0, lon0 = float(path[0]['latitude']), float(path[0]['longitude'])
      pts = [to_local(lat0, lon0, float(p['latitude']), float(p['longitude'])) for p in path]
      px, py = to_local(lat0, lon0, poi_lat, poi_lon)
      along, perp, past_end = _project_point_to_polyline(pts, px, py)
      if not past_end and along > 0.0:
        return {"along_m": along, "perp_m": perp, "source": "path"}
      # else fall through to the cone fallback (POI is past the short path's end)
    except (KeyError, TypeError, ValueError):
      pass

  # --- forward-cone + haversine fallback (no path, or POI past the path end) ---
  if cur_bearing is None:
    return None
  d = haversine_m(cur_lat, cur_lon, poi_lat, poi_lon)
  if d > max_fallback_m:
    return None
  rel = normalize180(bearing_deg(cur_lat, cur_lon, poi_lat, poi_lon) - cur_bearing)
  if abs(rel) > cone_deg:        # not ahead (along-track sign, expressed as a heading cone)
    return None
  # perpendicular offset from our heading line = d * sin(rel)
  perp = abs(d * math.sin(math.radians(rel)))
  return {"along_m": d * math.cos(math.radians(rel)), "perp_m": perp, "source": "cone"}


def nearest_ahead(path, cur_lat, cur_lon, cur_bearing, pois, max_perp_m=None,
                  cone_deg=60.0, max_fallback_m=12 * M_PER_MILE):
  """Of `pois` (each must expose .get('lat')/.get('lon')), return (poi, ahead_dict) for the nearest one
  AHEAD whose perpendicular offset is within max_perp_m (None = no perp limit). (None, None) if none."""
  best = None
  best_along = float('inf')
  for poi in pois:
    try:
      a = ahead(path, cur_lat, cur_lon, cur_bearing, float(poi['lat']), float(poi['lon']),
                cone_deg=cone_deg, max_fallback_m=max_fallback_m)
    except (KeyError, TypeError, ValueError):
      continue
    if a is None:
      continue
    if max_perp_m is not None and a["perp_m"] > max_perp_m:
      continue
    if a["along_m"] < best_along:
      best_along = a["along_m"]
      best = (poi, a)
  return best if best is not None else (None, None)
