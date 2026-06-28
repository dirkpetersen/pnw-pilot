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

The police API key is NOT hard-coded — it is read from /data/pnw/location/police_proxy.json (key+host+url).
Absent config -> police never polls and its line shows "—" (no-data), never a false "Clear".
"""
import json
import os
import time
import threading
import urllib.request
import urllib.parse
import urllib.error
from datetime import UTC, datetime

from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.system.location_services import geo

DATA_DIR = "/data/pnw/location"
EV_FILE = os.path.join(DATA_DIR, "chargers", "ev_dc_fast.geojson")
REST_DIR = os.path.join(DATA_DIR, "rest_areas")
PROXY_CFG = os.path.join(DATA_DIR, "police_proxy.json")

TICK_HZ = 1.0
EV_MAX_PERP_M = 1.0 * geo.M_PER_MILE      # decision #5: DC-fast within 1 mile perpendicular of the highway
POLICE_POLL_S = 60.0                       # ≤ 1/min (decision §7 / POLICE_WARNING_DESIGN §7)
POLICE_BBOX_DEG = 0.30                     # axis-aligned box (~±20 mi) around current GPS
POLICE_STALE_S = 45 * 60                   # drop crowd reports older than this
POLICE_TIMEOUT_S = 20
POLICE_MAX_BACKOFF_S = 15 * 60


def _now_epoch() -> float:
  """Wall-clock epoch seconds — needed to age crowd reports against Waze's epoch-ms timestamps
  (time.monotonic is banned-for-good-reason for intervals but is NOT a wall clock; datetime is)."""
  return datetime.now(UTC).timestamp()


# ----------------------------- static sources (no network) ------------------------------------------
class StaticData:
  """Loads the EV-DC-fast + rest-area files once, and reloads on a file-mtime change. Pure-local."""
  def __init__(self):
    self.ev: list = []
    self.rest: list = []
    self._ev_mtime = 0.0
    self._rest_sig = ()
    self.reload()

  def _mtime(self, path):
    try:
      return os.path.getmtime(path)
    except OSError:
      return 0.0

  def reload(self):
    # EV DC-fast geojson: FeatureCollection; geometry.coordinates = [lon, lat]
    m = self._mtime(EV_FILE)
    if m != self._ev_mtime:
      self._ev_mtime = m
      ev = []
      try:
        with open(EV_FILE) as f:
          gj = json.load(f)
        for feat in gj.get("features", []):
          try:
            lon, lat = feat["geometry"]["coordinates"][:2]
            p = feat.get("properties", {})
            ev.append({"lat": float(lat), "lon": float(lon),
                       "network": p.get("ev_network") or "",
                       "kw": p.get("ev_max_power_kw")})
          except (KeyError, TypeError, ValueError, IndexError):
            continue
        self.ev = ev
        cloudlog.info("location_services: loaded %d DC-fast chargers", len(ev))
      except (OSError, ValueError):
        self.ev = []
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
                           "name": it.get("name") or "Rest area"})
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
    self._mem = Params("/dev/shm/params")
    self._lock = threading.Lock()
    self._alerts: list = []         # cached raw POLICE alerts (lat, lon, magvar, ts, uuid, street)
    self._state = "nodata"          # 'ok' (fresh poll, may be empty) | 'nodata' (no config/poll failed)
    self._stop = threading.Event()
    self._cfg = None

  def snapshot(self):
    with self._lock:
      return list(self._alerts), self._state

  def stop(self):
    self._stop.set()

  def _load_cfg(self):
    try:
      with open(PROXY_CFG) as f:
        c = json.load(f)
      if c.get("key") and c.get("url"):
        return c
    except (OSError, ValueError):
      pass
    return None

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
                    "uuid": a.get("uuid") or a.get("id"), "street": a.get("street") or ""})
      except (KeyError, TypeError, ValueError):
        continue
    return out

  def run(self):
    backoff = POLICE_POLL_S
    while not self._stop.is_set():
      cfg = self._cfg or self._load_cfg()
      enabled = False
      try:
        enabled = self._mem.get_bool("LocationServicesEnabled")
      except Exception:
        pass
      if cfg is None or not enabled:
        with self._lock:
          self._alerts, self._state = [], "nodata"
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
          self._alerts, self._state = alerts, "ok"
        backoff = POLICE_POLL_S                              # success -> reset backoff
      except (urllib.error.URLError, urllib.error.HTTPError, ValueError, TimeoutError, OSError) as e:
        cloudlog.warning("location_services: police poll failed (%s); backing off %ds", type(e).__name__, int(backoff))
        with self._lock:
          self._state = "nodata"                            # NEVER a false 'clear' on failure (decision #4)
        backoff = min(backoff * 2, POLICE_MAX_BACKOFF_S)
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


def _line_police(alerts, state, lat, lon, brg, path):
  if state != "ok":
    return {"state": "nodata"}
  now = _now_epoch()
  # Drop STALE reports BEFORE picking the nearest, so a near-but-ancient report can't mask a fresh one
  # further ahead (Gemini bug #1). A report with no timestamp is kept — we can't age it.
  fresh = []
  for al in alerts:
    age = _age_min(al.get("ts"), now)
    if age is None or age * 60 <= POLICE_STALE_S:
      fresh.append(al)
  poi, a = geo.nearest_ahead(path, lat, lon, brg, fresh)
  if poi is None:
    return {"state": "clear"}                              # fresh poll genuinely returned nothing ahead
  return {"state": "alert", "dist_mi": round(a["along_m"] / geo.M_PER_MILE, 1),
          "dir": _police_dir(poi, brg), "age_min": _age_min(poi.get("ts"), now), "uuid": poi.get("uuid")}


def _line_static(items, lat, lon, brg, path, max_perp_m=None):
  poi, a = geo.nearest_ahead(path, lat, lon, brg, items, max_perp_m=max_perp_m)
  if poi is None:
    return None
  return poi, round(a["along_m"] / geo.M_PER_MILE, 1)


# ----------------------------- main -----------------------------------------------------------------
def main():
  params = Params()
  mem = Params("/dev/shm/params")
  static = StaticData()
  police = PoliceUpdater()
  police.start()
  rk = Ratekeeper(TICK_HZ, print_delay_threshold=None)
  last_reload = 0.0

  while True:
    enabled = params.get_bool("LocationServicesEnabled")
    if not enabled:
      mem.put_nonblocking("LocationServices", {"enabled": False, "ts": int(_now_epoch())})
      rk.keep_time()
      continue

    now = time.monotonic()
    if now - last_reload > 30.0:        # pick up newly-staged data files without a restart
      static.reload()
      last_reload = now

    lat, lon, brg, path, ctx = _read_mem(mem)
    out = {"enabled": True, "ts": int(_now_epoch())}

    # Freeway-gate the lookups: off-freeway (or no GPS) -> all lines no-data, never guess (§6).
    on_freeway = (ctx == "freeway") and lat is not None and lon is not None
    if not on_freeway:
      out["police"] = {"state": "nodata"}
      out["rest"] = {"state": "nodata"}
      out["ev"] = {"state": "nodata"}
    else:
      alerts, pstate = police.snapshot()
      out["police"] = _line_police(alerts, pstate, lat, lon, brg, path)

      r = _line_static(static.rest, lat, lon, brg, path)
      out["rest"] = {"state": "ok", "dist_mi": r[1], "name": r[0].get("name")} if r else {"state": "nodata"}

      e = _line_static(static.ev, lat, lon, brg, path, max_perp_m=EV_MAX_PERP_M)
      if e:
        ev = {"state": "ok", "dist_mi": e[1], "network": e[0].get("network") or ""}
        if e[0].get("kw"):                                  # omit kW for the ~2% lacking it (decision #6)
          ev["kw"] = e[0]["kw"]
        out["ev"] = ev
      else:
        out["ev"] = {"state": "nodata"}

    mem.put_nonblocking("LocationServices", out)
    rk.keep_time()


if __name__ == "__main__":
  main()
