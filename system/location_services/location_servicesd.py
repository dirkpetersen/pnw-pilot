#!/usr/bin/env python3
"""
location2pnw: pnw_location_services — the "HAPPENING AHEAD" daemon (display-only, panda-safe).

Merges three "what's ahead on the highway" sources into one overlay payload:
  • police  — live Waze proxy (NETWORK, isolated thread)
  • rest    — static rest/service areas (local JSON)
  • EV fast — static DC-fast chargers within 1 mile of the highway (local GeoJSON)

HARD RULE (LOCATION_SERVICES_DESIGN.md §2): the network (police) path is isolated in its own thread, so
a hung/403'd/slow Waze poll can NEVER stall or blank the always-on static rest/EV lines. The main loop
does cheap local geometry every tick; the police thread only refreshes a *cache* of raw alerts.

Runs as a NON_ESSENTIAL PythonProcess (always_run) — never on the control/safety path, never blocks
engagement. Reads GPS/path/road from /dev/shm mem params (the mapd_configd bridge); writes a single
`LocationServices` JSON mem param for the lower-left UI overlay. Gated by `LocationServicesEnabled`
(default ON) and, for the lookups, `roadContext == freeway`.

The police proxy key is NOT shipped in-distribution: it is supplied only via the persistent
/data/pnw/location/police_proxy.json override file (survives reboot / git reset). No key -> "—" (no-data);
a failed poll also shows "—", never a false "Clear".
"""
import json
import os
import time
import threading
import urllib.request
import urllib.parse
import urllib.error
from datetime import UTC, datetime

import cereal.messaging as messaging
from cereal import car
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.system.location_services import geo

# POI data is bundled IN the distribution next to this daemon (small enough to vendor). The daemon
# reloads on file-mtime, so editing these on-device still works for quick testing.
_DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
EV_FILE = os.path.join(_DATA, "chargers", "ev_dc_fast.geojson")        # DC-fast: small (~800 KB), bundled
REST_DIR = os.path.join(_DATA, "rest_areas")
# The police proxy key is a runtime secret -> stays in persistent /data, NOT the repo.
PROXY_CFG = "/data/pnw/location/police_proxy.json"

# Slow Level-2 chargers (3 MB) are DELIBERATELY NOT in the deploy branch (they bloated every deploy). The
# file lives alone on the `l2-charger-data` branch and is DOWNLOADED ON DEMAND the first time the user
# enables EvIncludeLevel2 ("Display slow Level 2 chargers"), cached to /data, then read from there. Same
# pattern as mapd's OSM pull. Display-only -> a slow/failed download just means L2 isn't shown yet.
EV_OTHER_CACHE = "/data/pnw/location/ev_other_chargers.geojson"        # downloaded cache (NOT in the repo)
EV_OTHER_URL = "https://raw.githubusercontent.com/dirkpetersen/pnw-pilot/l2-charger-data/ev_other_chargers.geojson"
EV_OTHER_TIMEOUT_S = 120
EV_OTHER_RETRY_S = 300                                                 # min gap between download attempts (no spam on failure)

# Default Waze proxy (RapidAPI). Override-file-only model: NO key ships in the distribution. The key must
# be supplied at runtime via the PROXY_CFG file (/data/pnw/location/police_proxy.json) — which lives on the
# device's persistent /data (survives reboot AND git reset/clean, unlike the in-tree code). With no key,
# police polling stays 'nodata' (no false alerts). This shape only carries the public url/host defaults.
DEFAULT_PROXY = {
  "url": "https://waze-api.p.rapidapi.com/alerts",
  "host": "waze-api.p.rapidapi.com",
  "key": "",                                        # NOT shipped — supplied by the PROXY_CFG override file
}

TICK_HZ = 1.0
EV_MAX_PERP_M = 1.0 * geo.M_PER_MILE      # chargers within 1 mi of the highway (driver rule, reaffirmed
                                          # 2026-06-28: a Renton charger 12 mi off-corridor leaked in at 3 mi).
EV_MAX_DIST_M = 6.0 * geo.M_PER_MILE      # cap EV ahead-range shorter than rest/police (15 mi): beyond the
                                          # short mapd path the "ahead" perp is measured off the extrapolated
                                          # HEADING LINE (not the real curvy road), so far off-corridor POIs
                                          # (Renton) leak through the 1-mi filter. A ~6 mi cap keeps the 1-mi
                                          # corridor meaningful. Trade-off: very-far on-corridor chargers
                                          # preview later (still appear by 6 mi, and on surface within 3 mi).
EV_FAST_DETOUR_MI = 3.0                     # prefer a DC-fast charger over a CLOSER slow L2 unless the fast
                                           # one is MORE than this many mi FURTHER than the slow one (driver
                                           # rule 2026-06-28): only show the slow charger if the fast is >3 mi
                                           # further than it; otherwise the fast (quality) wins.
SURFACE_RANGE_MI = 3.0                      # OFF-freeway: show nearest EV/rest within this straight-line radius (any direction)
POI_HOLD_S = 8.0                            # debounce: keep showing a POI for this long after the ahead-cone
                                            # momentarily drops it (curvy roads swing it in/out) — anti-flicker
POI_RECEDE_MI = 0.3                         # ...but if a held POI gets this much farther than its closest
                                            # approach, we've PASSED it -> drop immediately (distance-trend,
                                            # not bearing, so a sweeping curve doesn't false-drop a still-ahead POI)
# --- Tesla EV alternation + charger drop-when-receding (driver req 2026-07-01) --------------------
EV_ALT_S = 4.0                              # Tesla only: alternate the EV line Supercharger<->other every this many s
EV_RECEDE_MI = 1.0                          # drop a charger once you're >this far PAST your closest approach to it
                                           #   (left it behind) -> show the next-nearest; re-eligible if you re-approach
EV_TRACK_MI = 8.0                          # only recede-track chargers within this straight-line range (small state)
# location2pnw FIX: rest areas ALSO need a perpendicular filter. The design assumed the rest data was
# pre-scoped to the road being driven, but a rest area from another corridor (e.g. an I-5 entry while on
# I-90) projects "ahead" onto the path with a bogus along-track distance. Reject anything far off-road
# (the gatherer scoped rest areas within ~2 km of the mainline, so 1.5 mi comfortably keeps the real ones).
REST_MAX_PERP_M = 1.5 * geo.M_PER_MILE
DISPLAY_MAX_DIST_M = 15.0 * geo.M_PER_MILE   # all three (police/EV/rest) show a POI starting ~15 mi ahead (driver request)
POLICE_POLL_S = 60.0                       # ≤ 1/min (decision §7 / POLICE_WARNING_DESIGN §7)
POLICE_BBOX_DEG = 0.30                     # axis-aligned box (~±20 mi) around current GPS
POLICE_STALE_S = 20 * 60                   # drop crowd reports older than this (fresher-only, driver req 2026-07-01)
POLICE_TIMEOUT_S = 20
POLICE_MAX_BACKOFF_S = 15 * 60


def _now_epoch() -> float:
  """Wall-clock epoch seconds — needed to age crowd reports against Waze's epoch-ms timestamps
  (time.monotonic is banned-for-good-reason for intervals but is NOT a wall clock; datetime is)."""
  return datetime.now(UTC).timestamp()


def _is_supercharger(c):
  """A Tesla Supercharger row (NREL ev_network == 'Tesla' in ev_dc_fast.geojson)."""
  return "tesla" in (c.get("network") or "").lower()


def _read_is_tesla(params):
  """True if the current car is a Tesla (from CarParamsPersistent, same parse as ui_state.py). Any
  failure -> False (fall back to the normal fast/slow EV logic)."""
  try:
    b = params.get("CarParamsPersistent")
    if b:
      return messaging.log_from_bytes(b, car.CarParams).brand == "tesla"
  except Exception:
    pass
  return False


class _RecedeFilter:
  """Drop-when-receding: once you've moved > recede_mi PAST your closest approach to a POI (you've left it
  behind), drop it so the NEXT-nearest shows instead; it's re-eligible if you approach it again. Only tracks
  POIs currently within track_mi so the min-distance table stays tiny. `keep()` returns the input list minus
  the receded POIs. Pure-ish (reads geo only)."""
  def __init__(self, recede_mi, track_mi):
    self.recede_mi = recede_mi
    self.track_mi = track_mi
    self.min_d = {}          # (rlat, rlon) -> closest-approach distance (mi) seen while in range

  def keep(self, items, lat, lon):
    if lat is None or lon is None:
      return items
    out, seen = [], set()
    for it in items:
      try:
        d = geo.haversine_m(lat, lon, float(it["lat"]), float(it["lon"])) / geo.M_PER_MILE
      except (KeyError, TypeError, ValueError):
        out.append(it)                       # can't measure -> never drop
        continue
      if d > self.track_mi:
        out.append(it)                       # too far to track -> pass through (not receding)
        continue
      k = (round(float(it["lat"]), 4), round(float(it["lon"]), 4))
      seen.add(k)
      m = min(self.min_d.get(k, d), d)
      self.min_d[k] = m
      if d <= m + self.recede_mi:            # not yet >recede_mi past closest -> keep; else drop (left behind)
        out.append(it)
    for k in list(self.min_d):               # forget POIs that left tracking range (re-eligible if re-approached)
      if k not in seen:
        del self.min_d[k]
    return out


# ----------------------------- static sources (no network) ------------------------------------------
class StaticData:
  """Loads the EV-DC-fast + rest-area files once, and reloads on a file-mtime change. Pure-local."""
  def __init__(self):
    self.ev: list = []
    self.rest: list = []
    self._ev_sig = None
    self._rest_sig = ()
    self.reload(False)

  def _mtime(self, path):
    try:
      return os.path.getmtime(path)
    except OSError:
      return 0.0

  def _load_ev(self, path, fast, del_on_error=False):
    """Load a charger geojson (FeatureCollection; coordinates=[lon,lat]) -> POI dicts tagged `fast`
    (True=DC-fast, False=slow L1/L2). town comes from the NREL `city` property. del_on_error: for the
    downloaded L2 cache, unlink a corrupt/truncated file so it gets re-downloaded (don't strand it)."""
    out = []
    try:
      with open(path) as f:
        gj = json.load(f)
    except OSError:
      return out
    except ValueError:                                     # corrupt/truncated JSON (e.g. half-written cache)
      if del_on_error:
        try:
          os.remove(path)
        except OSError:
          pass
      return out
    if not isinstance(gj, dict):                            # an array/garbage root would AttributeError below
      return out
    for feat in gj.get("features", []):
      try:
        lon, lat = feat["geometry"]["coordinates"][:2]
        p = feat.get("properties") or {}                   # properties may be null in external GIS data
        out.append({"lat": float(lat), "lon": float(lon), "network": p.get("ev_network") or "",
                    "kw": p.get("ev_max_power_kw"), "town": p.get("city") or "", "fast": fast})
      except (KeyError, TypeError, ValueError, IndexError, AttributeError):
        continue
    return out

  def reload(self, include_l2=False):
    # EV chargers: DC-fast always; slow L1/L2 (ev_other) only when opted in via EvIncludeLevel2. Reload on
    # a file-mtime change OR the include_l2 flag flipping.
    sig = (self._mtime(EV_FILE), self._mtime(EV_OTHER_CACHE) if include_l2 else 0.0, include_l2)
    if sig != self._ev_sig:
      self._ev_sig = sig
      ev = self._load_ev(EV_FILE, fast=True)
      if include_l2:                          # cache may not exist yet (download in flight) -> _load_ev returns []
        ev += self._load_ev(EV_OTHER_CACHE, fast=False, del_on_error=True)
      self.ev = ev
      cloudlog.info("location_services: loaded %d chargers (include_l2=%s)", len(ev), include_l2)
    # rest areas: merge ALL *.json under REST_DIR, each a list of {name, lat, lon, ...}
    try:
      files = sorted(f for f in os.listdir(REST_DIR) if f.endswith(".json"))
    except OSError:
      files = []
    sig = tuple((f, self._mtime(os.path.join(REST_DIR, f))) for f in files)
    if sig != self._rest_sig:
      self._rest_sig = sig
      rest = []
      for f in files:
        try:
          with open(os.path.join(REST_DIR, f)) as fh:
            items = json.load(fh)
          for it in (items if isinstance(items, list) else items.get("rest_areas", [])):
            try:
              rest.append({"lat": float(it["lat"]), "lon": float(it["lon"]),
                           "name": it.get("display") or it.get("name") or "Rest area",
                           "dir": it.get("dir") or "",
                           "town": it.get("town") or it.get("city") or ""})
            except (KeyError, TypeError, ValueError):
              continue
        except (OSError, ValueError):
          continue
      self.rest = rest
      cloudlog.info("location_services: loaded %d rest areas", len(rest))


# ----------------------------- police (network, isolated thread) ------------------------------------
class PoliceUpdater(threading.Thread):
  """Polls the Waze proxy ≤1/min in its OWN thread and caches raw POLICE alerts. Never does geometry
  (the main loop does that against fresh GPS). Defensive: any failure -> state 'nodata' + backoff."""
  def __init__(self):
    super().__init__(daemon=True)
    self._mem = Params("/dev/shm/params")     # mem store: LastGPSPosition lives here
    self._params = Params()                   # persistent store: LocationServicesEnabled lives here (NOT mem!)
    self._lock = threading.Lock()
    self._alerts: list = []         # cached raw POLICE alerts (lat, lon, magvar, ts, uuid, street)
    self._state = "nodata"          # 'ok' (fresh poll, may be empty) | 'nodata' (no config/poll failed)
    self._err = ""                  # short last-error tag for the UI on non-ok (e.g. "quota (429)", "HTTP 403", "no key")
    self._stop = threading.Event()
    self._cfg = None

  def snapshot(self):
    with self._lock:
      return list(self._alerts), self._state, self._err

  def stop(self):
    self._stop.set()

  def _load_cfg(self):
    # Override file (persistent /data) supplies the key; otherwise DEFAULT_PROXY has none -> nodata.
    try:
      with open(PROXY_CFG) as f:
        c = json.load(f)
      if c.get("key") and c.get("url"):
        return c
    except (OSError, ValueError):
      pass
    return dict(DEFAULT_PROXY)

  def _cur_gps(self):
    try:
      pos = self._mem.get("LastGPSPosition", return_default=True)
      if isinstance(pos, (bytes, str)):
        pos = json.loads(pos)
      return float(pos["latitude"]), float(pos["longitude"])
    except (KeyError, TypeError, ValueError):
      return None

  def _poll(self, cfg, lat, lon):
    bl = f"{lat - POLICE_BBOX_DEG},{lon - POLICE_BBOX_DEG}"
    tr = f"{lat + POLICE_BBOX_DEG},{lon + POLICE_BBOX_DEG}"
    q = urllib.parse.urlencode({"bottom-left": bl, "top-right": tr})
    headers = {"x-rapidapi-host": cfg.get("host", ""), "x-rapidapi-key": cfg["key"]}
    req = urllib.request.Request(f"{cfg['url']}?{q}", headers=headers)
    with urllib.request.urlopen(req, timeout=POLICE_TIMEOUT_S) as resp:
      raw = resp.read()
    data = json.loads(raw)                                  # defensive: HTML-error-200 -> ValueError below
    alerts = data if isinstance(data, list) else data.get("alerts", [])
    if not isinstance(alerts, list):
      raise ValueError("unexpected alerts payload")
    out = []
    for a in alerts:
      if not isinstance(a, dict) or a.get("type") != "POLICE":
        continue
      try:
        out.append({"lat": float(a["locationY"]), "lon": float(a["locationX"]),
                    "magvar": a.get("magvar"), "ts": a.get("timestamp"),
                    "uuid": a.get("uuid") or a.get("id"), "street": a.get("street") or "",
                    "town": a.get("city") or ""})
      except (KeyError, TypeError, ValueError):
        continue
    return out

  def run(self):
    backoff = POLICE_POLL_S
    while not self._stop.is_set():
      try:
        cfg = self._cfg or self._load_cfg()
        enabled = False
        try:
          enabled = self._params.get_bool("LocationServicesEnabled")   # PERSISTENT store (was wrongly read
                                                                        # from mem -> always False -> never polled)
        except Exception:
          pass
        nokey = not (cfg and cfg.get("key"))                   # no key (override file absent) -> nodata, don't poll
        if cfg is None or nokey or not enabled:
          with self._lock:
            self._alerts, self._state = [], "nodata"
            self._err = "no key" if (nokey and enabled) else ""   # surface the actionable case; disabled = plain "-"
          self._stop.wait(POLICE_POLL_S)
          continue
        self._cfg = cfg
        gps = self._cur_gps()
        if gps is None:
          self._stop.wait(POLICE_POLL_S)
          continue
        try:
          alerts = self._poll(cfg, gps[0], gps[1])
          with self._lock:
            self._alerts, self._state, self._err = alerts, "ok", ""
          cloudlog.info("location_services: police poll ok (%d alerts)", len(alerts))   # heartbeat for diagnosis
          backoff = POLICE_POLL_S                            # success -> reset backoff
        except (urllib.error.URLError, urllib.error.HTTPError, ValueError, TimeoutError, OSError) as e:
          # Surface the real cause on-screen. HTTPError carries the status code (429 = Waze quota exceeded);
          # HTTPError subclasses URLError so it must be checked first. Non-200 status -> "HTTP <code>".
          if isinstance(e, urllib.error.HTTPError):
            emsg = "quota (429)" if e.code == 429 else f"HTTP {e.code}"
          elif isinstance(e, TimeoutError) or isinstance(getattr(e, "reason", None), TimeoutError):
            emsg = "timeout"                                # urlopen timeouts can arrive wrapped in URLError.reason
          elif isinstance(e, ValueError):
            emsg = "bad resp"
          else:                                              # URLError / OSError -> connectivity
            emsg = "net err"
          cloudlog.warning("location_services: police poll failed (%s: %s); backing off %ds",
                           type(e).__name__, emsg, int(backoff))
          with self._lock:
            self._state, self._err = "nodata", emsg         # NEVER a false 'clear' on failure (decision #4)
          backoff = min(backoff * 2, POLICE_MAX_BACKOFF_S)
      except Exception:
        cloudlog.exception("location_services: police thread loop error (continuing)")  # HARD RULE: never die silently
      self._stop.wait(backoff)


# ----------------------------- helpers --------------------------------------------------------------
def _read_mem(mem):
  """Read GPS (lat/lon/bearing), the path-ahead, road class + enabled flag from the mapd bridge."""
  lat = lon = brg = None
  try:
    pos = mem.get("LastGPSPosition", return_default=True)
    if isinstance(pos, (bytes, str)):
      pos = json.loads(pos)
    lat, lon, brg = float(pos["latitude"]), float(pos["longitude"]), float(pos.get("bearing", 0.0))
  except (KeyError, TypeError, ValueError):
    pass
  try:
    path = mem.get("MapTargetVelocities", return_default=True) or []
  except Exception:
    path = []
  try:
    ctx = mem.get("RoadContext", return_default=True)
    ctx = ctx.decode() if isinstance(ctx, bytes) else (ctx or "")
  except Exception:
    ctx = ""
  return lat, lon, brg, path, ctx


def _police_dir(alert, cur_bearing):
  """§5: direction hint ONLY when Waze gives a real reporter-heading (magvar); else 'none' (silent)."""
  mv = alert.get("magvar")
  if mv is None or cur_bearing is None:
    return "none"
  try:
    d = abs(geo.normalize180(float(mv) - cur_bearing))
  except (TypeError, ValueError):
    return "none"
  if d < 45.0:
    return "same"
  if d > 135.0:
    return "opp"
  return "none"


def _age_min(ts, now):
  """Crowd-report age in minutes from a Waze epoch-ms timestamp; None if no/unparseable timestamp."""
  if not ts:
    return None
  try:
    return max(0, int((now - float(ts) / 1000.0) / 60.0))
  except (TypeError, ValueError):
    return None


_COMPASS8 = ("N", "NE", "E", "SE", "S", "SW", "W", "NW")


def _compass8(bearing):
  """8-point compass label (N/NE/E/SE/S/SW/W/NW) for a 0-360 geographic bearing; "" if None. Absolute
  direction TO a point from here (e.g. the charger is "NW" of me), not relative to heading."""
  if bearing is None:
    return ""
  return _COMPASS8[int(bearing % 360.0 / 45.0 + 0.5) % 8]


def _line_police(alerts, state, err, lat, lon, brg, path):
  if state != "ok":
    return {"state": "nodata", "err": err} if err else {"state": "nodata"}
  now = _now_epoch()
  # Drop STALE reports BEFORE picking the nearest, so a near-but-ancient report can't mask a fresh one
  # further ahead (Gemini bug #1). A report with no timestamp is kept — we can't age it. Also drop
  # OPPOSITE-direction reports ("other side" of the highway) so we don't alert for police on the other
  # carriageway (driver req 2026-07-01); unknown-direction reports (no magvar -> 'none') are KEPT, since
  # we can't tell they're across and dropping them would silently miss most reports (Waze often omits magvar).
  fresh = []
  for al in alerts:
    age = _age_min(al.get("ts"), now)
    if age is not None and age * 60 > POLICE_STALE_S:
      continue                                    # too old
    if _police_dir(al, brg) == "opp":
      continue                                    # other side of the road -> don't alert
    fresh.append(al)
  poi, a = geo.nearest_ahead(path, lat, lon, brg, fresh, max_fallback_m=DISPLAY_MAX_DIST_M)
  if poi is None:
    return {"state": "clear"}                              # fresh poll genuinely returned nothing ahead
  return {"state": "alert", "dist_mi": round(a["along_m"] / geo.M_PER_MILE, 1),
          "dir": _police_dir(poi, brg), "age_min": _age_min(poi.get("ts"), now),
          "uuid": poi.get("uuid"), "town": poi.get("town", "")}


class _Hold:
  """Anti-flicker debounce for one overlay line. On a curvy road the highway 'ahead' cone swings a POI in
  and out each second, so the line blinks. Keep the last good POI for POI_HOLD_S after it drops, re-emitting
  it with a refreshed straight-line distance — BUT only while we're still approaching it. Track the closest
  approach (min_d); once the held POI recedes past it by POI_RECEDE_MI we've PASSED it -> drop immediately
  (so passed chargers don't linger). Distance-trend, not bearing, so a sweeping curve that still has the POI
  ahead doesn't false-drop it."""
  def __init__(self, hold_s):
    self.hold_s = hold_s
    self.poi = None
    self.t = 0.0
    self.min_d = None

  def update(self, found, now, lat, lon):
    # found = (poi, dist_mi) or None. Returns the same shape (possibly the held POI).
    if found is not None:
      self.min_d = found[1] if self.poi is not found[0] else min(self.min_d, found[1])
      self.poi, self.t = found[0], now
      return found
    if self.poi is not None and (now - self.t) < self.hold_s:
      try:
        d = geo.haversine_m(lat, lon, self.poi["lat"], self.poi["lon"]) / geo.M_PER_MILE
      except (KeyError, TypeError, ValueError):
        self.poi = None
        return None
      if self.min_d is not None and d > self.min_d + POI_RECEDE_MI:   # receding past closest approach = passed
        self.poi = None
        return None
      self.min_d = d if self.min_d is None else min(self.min_d, d)
      return (self.poi, round(d, 1))
    self.poi = None
    return None


def _nearest_within(items, lat, lon, max_mi):
  """OFF-freeway proximity search: the nearest POI within max_mi straight-line, in ANY direction.
  On surface streets there is no highway 'path ahead' (you navigate a grid), so use a radius instead of
  along-track distance — you can detour to a charger/rest in range. Returns (poi, dist_mi) or None."""
  best, best_m = None, max_mi * geo.M_PER_MILE
  for it in items:
    try:
      d = geo.haversine_m(lat, lon, it["lat"], it["lon"])
    except (KeyError, TypeError, ValueError):
      continue
    if d <= best_m:
      best, best_m = it, d
  return (best, round(best_m / geo.M_PER_MILE, 1)) if best is not None else None


def _line_static(items, lat, lon, brg, path, max_perp_m=None, max_dist_m=None):
  kw = {"max_perp_m": max_perp_m}
  if max_dist_m is not None:
    kw["max_fallback_m"] = max_dist_m          # how far ahead a POI may be and still show
  poi, a = geo.nearest_ahead(path, lat, lon, brg, items, **kw)
  if poi is None:
    return None
  return poi, round(a["along_m"] / geo.M_PER_MILE, 1)


# ----------------------------- L2 charger download (network, isolated thread) -----------------------
class L2Downloader:
  """Fetches the opt-in slow-Level-2 charger file (3 MB) on demand into a /data cache, in a BACKGROUND
  thread so the one-time 3 MB pull NEVER stalls the always-on main loop (same HARD RULE as the police
  thread). The file is EXCLUDED from the deploy branch; it lives alone on the `l2-charger-data` branch
  and is pulled once via raw.githubusercontent, then cached. Display-only: a failed/slow download just
  means L2 isn't shown yet — it retries on the next tick the cache is still missing."""
  def __init__(self):
    self._thread = None
    self._lock = threading.Lock()
    self._last_attempt = 0.0

  def ensure(self):
    # Non-blocking: if the cache is missing and no download is already in flight, kick one off. Returns
    # immediately every tick (cheap os.path.exists), so the main loop never waits on the network. A failed
    # download exits its thread; EV_OTHER_RETRY_S throttles re-attempts so we don't spam the net/log at 1 Hz.
    if os.path.exists(EV_OTHER_CACHE):
      return
    now = time.monotonic()
    with self._lock:
      if self._thread is not None and self._thread.is_alive():
        return
      if now - self._last_attempt < EV_OTHER_RETRY_S:
        return
      self._last_attempt = now
      self._thread = threading.Thread(target=self._fetch, daemon=True)
      self._thread.start()

  def _fetch(self):
    tmp = EV_OTHER_CACHE + ".tmp"
    try:
      os.makedirs(os.path.dirname(EV_OTHER_CACHE), exist_ok=True)
      cloudlog.info("location_services: downloading L2 charger file %s", EV_OTHER_URL)
      req = urllib.request.Request(EV_OTHER_URL, headers={"User-Agent": "pnw-location/1.0"})
      with urllib.request.urlopen(req, timeout=EV_OTHER_TIMEOUT_S) as resp:
        data = resp.read()
      if len(data) < 1024 or b'"features"' not in data:      # guard against caching an HTML error page
        raise ValueError("L2 download did not look like a charger geojson")
      with open(tmp, "wb") as f:
        f.write(data)
        f.flush()
        os.fsync(f.fileno())                                   # durable before the rename (survive power loss)
      os.replace(tmp, EV_OTHER_CACHE)                          # atomic -> reload's mtime-sig picks it up
      cloudlog.info("location_services: L2 charger file cached (%d bytes)", len(data))
    except (urllib.error.URLError, urllib.error.HTTPError, ValueError, TimeoutError, OSError) as e:
      cloudlog.warning("location_services: L2 download failed (%s)", type(e).__name__)
      try:
        os.remove(tmp)
      except OSError:
        pass


def _pick_ev(items, on_freeway, lat, lon, brg, path):
  """Nearest charger: AHEAD along the mapd path on a freeway, else nearest within the surface radius."""
  if on_freeway:
    return _line_static(items, lat, lon, brg, path, max_perp_m=EV_MAX_PERP_M, max_dist_m=EV_MAX_DIST_M)
  return _nearest_within(items, lat, lon, SURFACE_RANGE_MI)


# ----------------------------- main -----------------------------------------------------------------
def main():
  params = Params()
  mem = Params("/dev/shm/params")
  static = StaticData()
  police = PoliceUpdater()
  police.start()
  l2dl = L2Downloader()
  rest_hold = _Hold(POI_HOLD_S)              # anti-flicker debounce for the rest + EV lines
  ev_hold = _Hold(POI_HOLD_S)
  sc_hold = _Hold(POI_HOLD_S)                # Tesla: anti-flicker for the Supercharger side of the alternation
  other_hold = _Hold(POI_HOLD_S)             # Tesla: anti-flicker for the other-charger side
  ev_recede = _RecedeFilter(EV_RECEDE_MI, EV_TRACK_MI)   # drop chargers left >1 mi behind -> show the next-nearest
  rk = Ratekeeper(TICK_HZ, print_delay_threshold=None)
  last_reload = 0.0
  last_l2 = None
  is_tesla = _read_is_tesla(params)          # Tesla -> alternate Supercharger<->other; refreshed periodically below
  last_car_check = 0.0

  while True:
    enabled = params.get_bool("LocationServicesEnabled")
    if not enabled:
      mem.put_nonblocking("LocationServices", {"enabled": False, "ts": int(_now_epoch())})
      rk.keep_time()
      continue

    now = time.monotonic()
    include_l2 = params.get_bool("EvIncludeLevel2")
    if include_l2:
      l2dl.ensure()                                          # non-blocking: pull the 3 MB L2 file once, in the bg
    if include_l2 != last_l2 or now - last_reload > 30.0:   # reload on the L2 toggle flipping, or every 30s (data mtime)
      static.reload(include_l2)
      last_reload = now
      last_l2 = include_l2
    if now - last_car_check > 30.0:            # the one device moves between cars -> re-check the brand periodically
      is_tesla = _read_is_tesla(params)
      last_car_check = now

    lat, lon, brg, path, ctx = _read_mem(mem)
    out = {"enabled": True, "ts": int(_now_epoch())}

    # Two rules (driver request 2026-06-28):
    #  • FREEWAY (RoadContext=="freeway"): show POIs directly ALONGSIDE the highway, AHEAD, in the agreed
    #    ranges (EV/rest perp-filtered + up to 15 mi ahead along the mapd path).
    #  • SURFACE street (any other road, with a GPS fix): no highway path -> show the nearest EV/rest within
    #    a SURFACE_RANGE_MI (3 mi) straight-line radius, any direction (you can detour).
    # Police stays freeway-only (the "ahead"/banner concept is highway-specific).
    on_freeway = (ctx == "freeway") and lat is not None and lon is not None
    have_gps = lat is not None and lon is not None
    out["freeway"] = on_freeway              # UI header: "HAPPENING AHEAD" (freeway) vs "NEARBY (3 MI)" (surface)
    if not have_gps:
      out["police"] = {"state": "nodata"}
      out["rest"] = {"state": "nodata"}
      out["ev"] = {"state": "nodata"}
      rest_hold.poi = ev_hold.poi = sc_hold.poi = other_hold.poi = None     # no fix -> drop any held POI
    else:
      if on_freeway:
        alerts, pstate, perr = police.snapshot()
        out["police"] = _line_police(alerts, pstate, perr, lat, lon, brg, path)
      else:
        out["police"] = {"state": "nodata"}

      # rest area (car-agnostic)
      if on_freeway:
        r = _line_static(static.rest, lat, lon, brg, path, max_perp_m=REST_MAX_PERP_M, max_dist_m=DISPLAY_MAX_DIST_M)
      else:
        r = _nearest_within(static.rest, lat, lon, SURFACE_RANGE_MI)
      r = rest_hold.update(r, now, lat, lon)   # debounce: anti-flicker on curves + drop-when-passed (distance-trend)

      # EV chargers: first DROP any charger we've left >EV_RECEDE_MI behind (so the next-nearest shows), then select.
      ev_items = ev_recede.keep(static.ev, lat, lon)
      if is_tesla:
        # Tesla: ALTERNATE the EV line between the nearest Tesla SUPERCHARGER and the nearest OTHER charger
        # (driver req 2026-07-01), Supercharger first, toggling every EV_ALT_S. Each side anti-flickered.
        sc = sc_hold.update(_pick_ev([c for c in ev_items if _is_supercharger(c)], on_freeway, lat, lon, brg, path), now, lat, lon)
        oth = other_hold.update(_pick_ev([c for c in ev_items if not _is_supercharger(c)], on_freeway, lat, lon, brg, path), now, lat, lon)
        e = (sc if int(now / EV_ALT_S) % 2 == 0 else oth) or sc or oth   # only one in range -> just show it
      else:
        # non-Tesla (e.g. Lightning): prefer the DC-fast charger; only show a CLOSER slow L2 when the nearest
        # fast one is MORE than EV_FAST_DETOUR_MI further (driver 2026-06-28).
        e_fast = _pick_ev([c for c in ev_items if c.get("fast")], on_freeway, lat, lon, brg, path)
        e_slow = _pick_ev([c for c in ev_items if not c.get("fast")], on_freeway, lat, lon, brg, path)
        if e_fast and e_slow and e_slow[1] < e_fast[1] and (e_fast[1] - e_slow[1]) > EV_FAST_DETOUR_MI:
          e = e_slow
        else:
          e = e_fast or e_slow
        e = ev_hold.update(e, now, lat, lon)

      out["rest"] = ({"state": "ok", "dist_mi": r[1], "name": r[0].get("name"), "dir": r[0].get("dir", ""),
                      "town": r[0].get("town", "")} if r else {"state": "nodata"})
      if e:
        try:                                                # absolute compass direction TO the charger from here
          compass = _compass8(geo.bearing_deg(lat, lon, e[0]["lat"], e[0]["lon"]))
        except (KeyError, TypeError, ValueError):
          compass = ""
        ev = {"state": "ok", "dist_mi": e[1], "network": e[0].get("network") or "",
              "fast": e[0].get("fast", True), "town": e[0].get("town", ""), "compass": compass}
        if e[0].get("kw"):                                  # omit kW for the ~2% lacking it (decision #6)
          ev["kw"] = e[0]["kw"]
        out["ev"] = ev
      else:
        out["ev"] = {"state": "nodata"}

    mem.put_nonblocking("LocationServices", out)
    rk.keep_time()


if __name__ == "__main__":
  main()
