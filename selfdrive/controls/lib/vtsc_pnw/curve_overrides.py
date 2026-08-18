"""curveoverride2pnw — curated per-location curve speed ceilings (driver proposal 2026-08-18).

WHY THIS EXISTS
PNW-pilot is explicitly a corridor distribution (Seattle <-> Corvallis on I-5, plus I-90). Some
curves have forced a takeover repeatedly, and neither automatic source catches them reliably:
mapd's velocity field cannot separate a real curve from an artifact, and vision's 8 s lookahead is
physically too short to plan a gradual slowdown at 90 mph. Curated knowledge of a handful of known
locations sidesteps both. The same precedent already exists in this repo for rest areas
(system/location_services/data/rest_areas/*.json).

THIS IS A SAFETY NET, NOT A FIX. It does not generalise, it needs maintenance, and it must never
become the reason the underlying curve detection stops being improved. Every entry carries an
`evidence` path to the drive report that justifies it, so a future reader can tell a measured entry
from a guess.

HOW IT PARTICIPATES
resolve() returns a plain (v_target, dist_m) candidate. The caller folds it into the SAME decel
envelope selection as vision and map, so an override inherits every existing guarantee: the gentle
A_DECEL envelope, apex timing, the V_MIN floor, the posted-limit floor, reduce-only, and the state
machine's debounce. It is NOT a separate control path and can only ever LOWER a target.

TUNING ON THE ROAD
Entries ship in data/curve_overrides/*.json. A device-local /data/pnw/curve_overrides.json is merged
on top (same schema, matched by name) so a ceiling can be adjusted between drives without a deploy --
the convention already used by dm.json / lataccel_limits.json / lanecenter_tuning.json. It lives
outside the git tree so an auto-update's `git clean` cannot delete it.
"""
import json
import math
import os

from openpilot.selfdrive.controls.lib.vtsc_pnw import vtsc_constants as C

_DATA_DIR = os.path.join(os.path.dirname(os.path.realpath(__file__)), "data", "curve_overrides")
_LOCAL_PATH = "/data/pnw/curve_overrides.json"
_LOCAL_MAX_B = 256 * 1024
_RELOAD_S = 30.0

# An override is a CEILING on entry speed, so its bounds are deliberately tight: below V_MIN nothing
# downstream would honour it anyway, and above SANE_MAX it is not a slowdown worth expressing.
_V_MIN_MPH = 15.0
_V_MAX_MPH = 85.0
_MATCH_RADIUS_M = 250.0     # how near the stored apex a fix must be for the entry to apply
_LOOKAHEAD_M = 800.0        # start advising this far out; the decel envelope decides when it binds
_BEARING_TOL_DEG = 60.0     # direction gate -- a curve is only a problem in one direction of travel
_MAX_ENTRIES = 512

MPH_TO_MS = 0.44704


def _valid(e):
  """A malformed entry is DROPPED, never clamped into something plausible -- a curated table that
  silently invents a ceiling is worse than one that is missing an entry."""
  try:
    if not isinstance(e, dict):
      return None
    lat, lon = float(e["lat"]), float(e["lon"])
    v_mph = float(e["v_max_mph"])
    if not (math.isfinite(lat) and math.isfinite(lon) and math.isfinite(v_mph)):
      return None
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
      return None
    if not (_V_MIN_MPH <= v_mph <= _V_MAX_MPH):
      return None
    brg = e.get("bearing")
    brg = float(brg) if brg is not None and math.isfinite(float(brg)) else None
    return {"name": str(e.get("name", ""))[:80], "lat": lat, "lon": lon,
            "v": v_mph * MPH_TO_MS, "bearing": brg}
  except (KeyError, TypeError, ValueError):
    return None


def _load_dir():
  out = []
  try:
    for fn in sorted(os.listdir(_DATA_DIR)):
      if not fn.endswith(".json"):
        continue
      try:
        with open(os.path.join(_DATA_DIR, fn)) as f:
          rows = json.load(f)
      except Exception:
        continue
      if isinstance(rows, list):
        out.extend(rows)
  except Exception:
    return []
  return out


def _load_local():
  try:
    st = os.stat(_LOCAL_PATH)
    if not (os.path.isfile(_LOCAL_PATH) and st.st_size <= _LOCAL_MAX_B):
      return []
    with open(_LOCAL_PATH) as f:
      rows = json.load(f)
    return rows if isinstance(rows, list) else []
  except Exception:
    return []


class CurveOverrides:
  def __init__(self):
    self._entries: list = []
    self._last_load = -1e9
    self._loaded_once = False

  def _refresh(self, now: float) -> None:
    if self._loaded_once and now - self._last_load < _RELOAD_S:
      return
    self._last_load = now
    self._loaded_once = True
    merged = {}
    for e in (_load_dir() + _load_local())[:_MAX_ENTRIES]:   # local wins on same name
      v = _valid(e)
      if v is not None:
        merged[v["name"] or f"{v['lat']:.5f},{v['lon']:.5f}"] = v
    self._entries = list(merged.values())

  def resolve(self, now: float, lat, lon, bearing):
    """Most-binding override for the current fix as (v_target_ms, dist_m, name), or None.

    Most-binding = LOWEST ceiling among entries we are approaching, not merely the nearest, so a
    tighter curve just past a gentler one is not masked. Never raises."""
    try:
      self._refresh(now)
      if not self._entries or lat is None or lon is None:
        return None
      best = None
      for e in self._entries:
        d = _haversine_m(lat, lon, e["lat"], e["lon"])
        if d > _LOOKAHEAD_M:
          continue
        # Direction gate. A stored bearing means the entry applies to ONE direction of travel; with
        # no live bearing we keep it (fail toward warning) rather than guessing.
        if e["bearing"] is not None and bearing is not None:
          if abs((float(bearing) - e["bearing"] + 540.0) % 360.0 - 180.0) > _BEARING_TOL_DEG:
            continue
        # Behind us and receding beyond the match radius -> done with this one.
        if bearing is not None and d > _MATCH_RADIUS_M:
          rel = _rel_bearing(lat, lon, e["lat"], e["lon"], float(bearing))
          if rel > 90.0:
            continue
        if best is None or e["v"] < best[0]:
          best = (e["v"], d, e["name"])
      return best
    except Exception:
      return None


def _haversine_m(lat1, lon1, lat2, lon2) -> float:
  r = 6371000.0
  p1, p2 = math.radians(lat1), math.radians(lat2)
  dp, dl = math.radians(lat2 - lat1), math.radians(lon2 - lon1)
  a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
  return 2 * r * math.asin(min(1.0, a ** 0.5))


def _rel_bearing(lat1, lon1, lat2, lon2, ref_deg) -> float:
  dl = math.radians(lon2 - lon1)
  p1, p2 = math.radians(lat1), math.radians(lat2)
  y = math.sin(dl) * math.cos(p2)
  x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
  brg = math.degrees(math.atan2(y, x)) % 360.0
  return abs((brg - ref_deg + 540.0) % 360.0 - 180.0)


assert C.V_MIN <= _V_MIN_MPH * MPH_TO_MS, "override floor must not undercut V_MIN"
