#!/usr/bin/env python3
"""
mapdstate2pnw: GPS-driven, on-demand, whole-STATE map auto-download for the official pfeiferj mapd
binary, replacing the old fixed WA/OR/ID auto-download.

The mapd binary ships with every speed/curve CONTROL disabled (safe default) and downloads no map
data on its own. This daemon watches the current GPS fix each loop: whenever mapd has no tile loaded
for the car's position ("uncovered"), it asks mapd to download the WHOLE region the car is currently
in — the enclosing US state (never the whole US as a country), or the enclosing nation for a non-US
fix (e.g. a whole-Canada download for British Columbia, since mapd has no province granularity) — via
a `mapdIn` download message. It re-arms automatically every time the car enters a new uncovered
region (a fresh state/nation, or the same one again after "Refresh this location map" deleted its
tiles) and requests on ANY network, metered or not: the owner would rather burn cellular data on an
interstate than be stranded without maps in the back country. It never downloads a tiny per-tile
area (that leaves gaps between drives) and never a whole-country US download (states only, always).

Speed-limit DISPLAY works as soon as the region's maps are present; the user opts into speed/curve
CONTROL later via MapdSettings — this daemon never enables control.
"""
import json
import math
import os
import shutil
import subprocess
import time
import cereal.messaging as messaging
from openpilot.common.gps import get_gps_location_service
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.system.mapd.installer import MAPD_BINARY   # mapdheal2pnw: self-heal relaunch path
from openpilot.system.mapd import coverage                # mapdstate2pnw: GPS -> region + download key

OSM_OFFLINE_DIR = "/data/media/0/osm/offline"   # where mapd stores downloaded region tiles
MAPD_WATCHDOG_LOOPS = 45   # mapdheal2pnw: loops (~s) mapd may be silent before we self-heal relaunch it
# mapdstate2pnw: mapd's own download tile grid (mapd settings/const.go GROUP_AREA_BOX_DEGREES) — tiles
# land on disk as OSM_OFFLINE_DIR/<lat>/<lon>/... where lat/lon are floored to this grid. Mirrored here
# so "Refresh this location map" deletes exactly the directories mapd's own downloader would (re)create.
TILE_GRID_DEGREES = 2
# mapdstate2pnw: while still uncovered in the SAME region, don't resend a download request more often
# than this — only a region CHANGE re-triggers immediately. Bounds the retry-in-case-mapd's-socket-
# wasn't-up-yet resend (the old code's "resend every loop" behavior) to a sane cadence instead of 1 Hz.
REGION_RESEND_INTERVAL_S = 60.0
# mapdgate2pnw: bound that resend. A whole-STATE download is expensive and mapd has NO incremental
# fetch -- every retry refetches the entire region -- so a region that keeps coming back uncovered
# (tiles that never load, a genuine gap in mapd's coverage, a pull that failed) turns the 60 s resend
# above into an unbounded whole-state download loop for as long as the car sits there. That is the
# "maps downloading the whole drive" symptom. Escalate geometrically after each failed attempt and
# cap, so a stuck region degrades to an occasional retry instead of hammering. Deliberately still
# retries forever at the cap: the owner would rather burn cellular than be stranded without maps.
# A region CHANGE, coverage arriving, or "Refresh this location map" all reset the escalation.
REGION_MAX_RESEND_INTERVAL_S = 1800.0   # 30 min


def next_region_interval(attempts: int) -> float:
  """Seconds to wait before re-requesting the SAME still-uncovered region.

  `attempts` = how many requests we have already sent for this region. The first retry keeps the
  plain REGION_RESEND_INTERVAL_S, which covers the common benign case of mapd missing the message
  because its mapdIn socket wasn't up yet; every further failure doubles, capped.
  """
  if attempts <= 1:
    return REGION_RESEND_INTERVAL_S
  return min(REGION_RESEND_INTERVAL_S * (2.0 ** (attempts - 1)), REGION_MAX_RESEND_INTERVAL_S)


def _grid_cells_for_bbox(bbox: list[float]) -> list[tuple[int, int]]:
  """The (lat, lon) tile-grid cell origins (TILE_GRID_DEGREES apart) covering a region's
  [min_lon, min_lat, max_lon, max_lat] bbox — mirrors mapd's settings/download.go adjustedBounds() +
  downloadBounds() loop in Python. Handles an antimeridian-crossing bbox (min_lon > max_lon, e.g.
  Alaska/Russia) by splitting into two longitude ranges."""
  min_lon, min_lat, max_lon, max_lat = bbox
  box = TILE_GRID_DEGREES

  def _adjusted(lo: float, hi: float) -> tuple[int, int]:
    a_lo = int(math.floor(lo / box)) * box
    a_hi = int(math.floor(hi / box)) * box
    if hi > a_hi:
      a_hi += box
    return a_lo, a_hi

  a_min_lat, a_max_lat = _adjusted(min_lat, max_lat)
  if min_lon <= max_lon:
    lon_ranges = [_adjusted(min_lon, max_lon)]
  else:
    lon_ranges = [_adjusted(min_lon, 180.0), _adjusted(-180.0, max_lon)]

  cells = []
  lat = a_min_lat
  while lat < a_max_lat:
    for a_min_lon, a_max_lon in lon_ranges:
      lon = a_min_lon
      while lon < a_max_lon:
        cells.append((lat, lon))
        lon += box
    lat += box
  return cells


def _delete_region_tiles(region: str, is_us_state: bool) -> int:
  """Delete for the "Refresh this location map" toggle: removes the offline tile directories mapd
  downloaded for `region`, so the uncovered-state auto-download logic below re-fetches it fresh next
  loop. Returns the number
  of directories actually removed (0 if the region is unknown or nothing was on disk).

  `is_us_state` MUST come from the same _locate()/region_and_key_for_gps() lookup that produced
  `region` — region_bbox() needs it to pick the right table for the 20 state/nation collision codes
  (e.g. "CA" = California the US state AND Canada the nation). Without it, a Whistler BC ("CA" =
  Canada) refresh would delete California's tiles instead (region_bbox()'s old states-first guess).

  Every delete target is re-derived from mapd_configd-computed integers (never user/param input) via
  _grid_cells_for_bbox, and is still re-checked against OSM_OFFLINE_DIR's realpath before rmtree —
  belt-and-suspenders so this can never remove anything outside the offline-tiles directory."""
  bbox = coverage.region_bbox(region, is_us_state)
  if bbox is None:
    return 0
  base = os.path.realpath(OSM_OFFLINE_DIR)
  removed = 0
  for lat, lon in _grid_cells_for_bbox(bbox):
    target = os.path.realpath(os.path.join(OSM_OFFLINE_DIR, str(lat), str(lon)))
    if target == base or os.path.dirname(target) == target or os.path.commonpath([base, target]) != base:
      continue  # never touch OSM_OFFLINE_DIR itself or anything resolving outside it
    if os.path.isdir(target):
      try:
        shutil.rmtree(target)
        removed += 1
      except OSError:
        cloudlog.exception(f"mapd_configd: RefreshLocationMap failed to remove {target}")
  return removed


def _mapd_process_alive() -> bool:
  """True if a process named 'mapd' exists — so the watchdog never double-launches one manager already
  has. Cheap /proc scan (only called when mapd has been silent a while). Never raises."""
  try:
    for pid in os.listdir("/proc"):
      if not pid.isdigit():
        continue
      try:
        with open(f"/proc/{pid}/comm") as f:
          if f.read().strip() == "mapd":
            return True
      except OSError:
        continue
  except OSError:
    pass
  return False


def _maps_on_disk() -> bool:
  """True if any OSM region tiles are present (i.e. maps have been downloaded at least once). Used to
  report a meaningful 'OK' after a reboot, when downloadProgress reflects no active/this-session pull."""
  try:
    with os.scandir(OSM_OFFLINE_DIR) as it:
      return any(True for _ in it)
  except OSError:
    return False


def main():
  params = Params()
  mem = Params("/dev/shm/params")   # CES + the on-road overlay read the legacy map params from here
  pm = messaging.PubMaster(['mapdIn'])
  gps_service = get_gps_location_service(params)
  sm = messaging.SubMaster(['mapdExtendedOut', 'mapdOut', gps_service])

  last_covered = None
  mapd_down = 0            # consecutive loops mapd (mapdExtendedOut) has been silent — debounces "down"
  mapd_out_down = 0        # nudgelesshighway2pnw: consecutive loops mapdOut has been silent — debounces
                            # the MapHighwayClass self-clear below the same way mapd_down debounces "down"
  last_requested_region = None   # region code mapd_configd last sent a download request for
  last_request_ts = -1e9         # monotonic ts of that request (resend-interval retry, see below)
  region_attempts = 0            # mapdgate2pnw: requests sent for last_requested_region while still
                                 # uncovered; drives next_region_interval's escalating backoff

  while True:
    sm.update(1000)  # paces the loop (blocks up to 1 s); no extra sleep

    # mapd2pnw bridge: the official pfeiferj mapd v2.0.6 publishes everything over CEREAL
    # (mapdOut / mapdExtendedOut), but CES (selfdrive/controls/lib/ces_pnw) and the on-road CES
    # overlay still read the legacy in-memory params the OLD mapd binary used to write directly
    # (MapTargetVelocities / LastGPSPosition / MapSpeedLimit). Translate the cereal output into those
    # mem params so CES's map-curve trigger + the overlay "map" line come alive. Display/decision only;
    # actual map braking is the longitudinal_planner mapdOut.suggestedSpeed cap (separate, gated OFF).
    try:
      if sm.alive[gps_service]:
        g = sm[gps_service]
        mem.put_nonblocking("LastGPSPosition", json.dumps({
          "latitude": float(g.latitude), "longitude": float(g.longitude),
          "bearing": float(getattr(g, "bearingDeg", 0.0)),
          "speed": float(getattr(g, "speed", 0.0)),  # m/s, for the location-services >45mph police gate
          "ts": time.monotonic()}))  # system-wide monotonic clock: lets the police gate reject stale speed
      if sm.alive['mapdOut']:
        mapd_out_down = 0
        mo = sm['mapdOut']
        mem.put_nonblocking("MapSpeedLimit", str(float(mo.speedLimit)))  # m/s; 0 = none
        # location2pnw: bridge the road identity/class so pnw_location_services can name the road and
        # freeway-gate its "happening ahead" lookups. roadContext enum -> 'freeway'|'city'|'unknown'.
        mem.put_nonblocking("RoadName", mo.roadName or "")
        mem.put_nonblocking("WayRef", mo.wayRef or "")
        mem.put_nonblocking("RoadContext", str(mo.roadContext))
        # dmroad2pnw: bridge oneWay + lane count so driver monitoring (DmMode=Highway) can relax on a
        # divided multi-lane carriageway even where mapd doesn't class it 'freeway' (oneWay && lanes>=2).
        mem.put_nonblocking("MapOneWay", "1" if mo.oneWay else "0")
        mem.put_nonblocking("MapLanes", str(int(mo.lanes)))
        # mapd220-2pnw PHASE 1 (plumbing/telemetry only): bridge the three v2.2.0 mapdOut fields as
        # mem-params, mirroring the RoadContext/MapOneWay pattern above.
        # nudgelesshighway2pnw: MapHighwayClass now GATES the nudgeless auto-lane-change (desire_helper.py)
        # in addition to the ces_pnw telemetry read. A dead/stalled mapd must not latch a stale freeway
        # class forever, so bridge a monotonic write-timestamp alongside it (same pattern as the
        # LastGPSPosition "ts" field above) — the reader rejects the class as unknown once this ages past
        # its freshness window, and the else-branch below explicitly self-clears it when mapdOut dies.
        mem.put_nonblocking("MapHighwayClass", str(mo.highwayClass))
        mem.put_nonblocking("MapHighwayClassTs", str(time.monotonic()))
        mem.put_nonblocking("MapWayId", str(int(mo.wayId)))
        mem.put_nonblocking("MapConditionalSpeedLimit", mo.conditionalSpeedLimit or "")
      else:
        # nudgelesshighway2pnw: mapdOut is not alive (mapd crashed / binary wiped / never started, or
        # simply not publishing this field yet on an old mapd version). Self-clear the bridged highway
        # class so desire_helper's freeway gate can't keep using the last value it saw before exiting the
        # freeway onto a city street. Debounced like the MapDownloadStatus "down" state below so one 1 Hz
        # gap doesn't flicker it — sustained silence clears it (and stamps a fresh ts so a reader checking
        # the ts alone still sees "" promptly rather than waiting out the staleness window too).
        mapd_out_down += 1
        if mapd_out_down == 5:
          mem.put_nonblocking("MapHighwayClass", "")
          mem.put_nonblocking("MapHighwayClassTs", str(time.monotonic()))
      if sm.alive['mapdExtendedOut']:
        # mapdExtendedOut.path = List(MapdPathPoint{latitude, longitude, curvature, targetVelocity});
        # CES's upcoming_curve() wants a list of {latitude, longitude, velocity} (m/s). Drop any point
        # with a non-finite (NaN/inf) velocity or position: upstream mapd computes curvature via Heron's
        # formula (math/curvature.go), which returns NaN on near-collinear OSM nodes -> NaN targetVelocity.
        # Downstream VTSC/CES already ignore NaN (all IEEE comparisons are False), but filtering HERE keeps
        # MapTargetVelocities clean so mapPts — and any future consumer that isn't NaN-safe — only ever
        # see real targets. It also makes mapPts a truthful "live forward path present" readiness signal.
        pts = []
        for p in sm['mapdExtendedOut'].path:
          la, lo, tv = float(p.latitude), float(p.longitude), float(p.targetVelocity)
          if math.isfinite(la) and math.isfinite(lo) and math.isfinite(tv):
            pts.append({"latitude": la, "longitude": lo, "velocity": tv})
        mem.put_nonblocking("MapTargetVelocities", pts)
    except Exception:
      cloudlog.exception("mapd_configd: mapd->CES bridge write failed")

    # mapd2pnw/debug: publish a SEPARATE "is the OSM DB downloaded?" status, distinct from the live
    # "is there map data for my current spot" line (mapPts). States: "OK" = mapd ALIVE + tiles on disk +
    # no pull in flight; "downloading"/"incomplete" = a pull is active or stopped short; "down" = tiles
    # exist but mapd itself is NOT publishing (crashed / binary wiped / not started); "none" = nothing
    # downloaded. "OK" now strictly implies mapd is alive, so the overlay can tell a fresh-ignition WARMUP
    # (pts==0 while "OK") apart from a dead mapd (pts==0 while "down") — the former is ORANGE, the latter RED.
    try:
      mdl = "none"
      if sm.alive['mapdExtendedOut']:
        mapd_down = 0
        dp = sm['mapdExtendedOut'].downloadProgress
        if dp.active:
          mdl = f"downloading {dp.downloadedFiles}/{dp.totalFiles}"
        elif dp.totalFiles > 0 and dp.downloadedFiles < dp.totalFiles:
          mdl = f"incomplete {dp.downloadedFiles}/{dp.totalFiles}"
        elif _maps_on_disk():
          mdl = "OK"
      elif _maps_on_disk():
        # tiles present but mapd is silent. Debounce brief 1 Hz gaps (~5 loops) before declaring "down" so
        # a single late mapdExtendedOut doesn't flicker the health dot RED; sustained silence -> "down".
        mapd_down += 1
        mdl = "down" if mapd_down >= 5 else "OK"
      mem.put_nonblocking("MapDownloadStatus", mdl)
    except Exception:
      pass

    # mapdheal2pnw: self-heal watchdog. mapd is a manager NativeProcess (gate = binary exists), so manager
    # SHOULD relaunch it if it dies — but a death-with-no-restart has been seen live (map went "no data"
    # mid-trip; recovered only by a manual `setsid ./mapd`). If mapd stays silent well past manager's
    # restart window (~45 loops @ ~1 Hz) while its binary exists AND no 'mapd' process is running, relaunch
    # it here with the SAME env manager uses (USE_MSGQ_PREFIX=true — else it hits the unprefixed boot-race).
    # The 45-loop gate makes a double-launch effectively impossible: if manager were going to restart it,
    # sm.alive would recover within seconds and reset mapd_down long before 45. Rate-limited to one relaunch
    # per down-episode (reset the counter after firing; it re-trips in ~45 s if still down).
    # TRADEOFF (Gemini): start_new_session detaches the child so it survives the drive, so on car-off it
    # won't catch manager's SIGTERM and orphans until the SOM sleeps — accepted vs "no map data mid-drive",
    # and only in the rare death-with-no-manager-restart case that trips this at all.
    if mapd_down >= MAPD_WATCHDOG_LOOPS and os.path.exists(MAPD_BINARY) and not _mapd_process_alive():
      try:
        subprocess.Popen(["/usr/bin/env", "USE_MSGQ_PREFIX=true", MAPD_BINARY],
                         start_new_session=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        cloudlog.error(f"mapd_configd: mapd silent ~{mapd_down}s with no process -> self-heal relaunch")
      except Exception:
        cloudlog.exception("mapd_configd: self-heal relaunch failed")
      mapd_down = 0

    # mapdstate2pnw: "covered" drives the region-tracking reset for the auto-download logic below;
    # a SEPARATE "map_here" drives the "Refresh this location map" grey-out (param
    # MapForLocationCovered — repurposed: this toggle is an ACTION button that only makes sense when
    # there IS a downloaded map here to refresh). covered = no GPS fix (can't tell where we are, e.g.
    # parked offroad) OR mapd has a map tile loaded for the current position — "no fix" counts as
    # covered here so the tracker stays inert rather than treating GPS-loss as a fresh uncovered spot.
    # map_here is stricter: it's the signal exposed to the UI, and there is nothing to refresh without
    # an actual fix, so (unlike `covered`) a missing fix must grey the button out, not enable it.
    has_fix = sm.alive[gps_service]
    tile_here = sm.alive['mapdOut'] and sm['mapdOut'].tileLoaded
    covered = (not has_fix) or tile_here
    map_here = has_fix and tile_here
    if map_here != last_covered:
      params.put_bool("MapForLocationCovered", map_here)
      last_covered = map_here
    if covered:
      # Left (or never entered) uncovered ground: forget the last-requested region so the NEXT
      # uncovered spot — a genuinely new state/nation, OR this same one again right after a
      # "Refresh this location map" delete — re-triggers a fresh download instead of being treated
      # as a duplicate of a stale request.
      last_requested_region = None
      region_attempts = 0            # mapdgate2pnw: manual refresh clears any backoff

    # mapdstate2pnw: "Refresh this location map" (param RefreshLocationMap) — user-triggered delete
    # of the CURRENT region's tiles, so the uncovered-state auto-download below re-fetches it fresh.
    # One-shot: consumed and cleared every loop it's seen set. No fix / unknown region -> no-op (still
    # clears the flag) rather than guessing at a region to delete.
    #
    # bug2: must resolve the region WITH its is-state/is-nation flag (region_and_key_for_gps(), not
    # the bare region_for_gps()) and pass it through to _delete_region_tiles() -> region_bbox() — for
    # the 20 state/nation collision codes (e.g. "CA" = California AND Canada), the bare code alone
    # can't say which table it came from, and region_bbox()'s old states-first guess deleted
    # California's tiles for a fix in Whistler BC (nation Canada, code "CA").
    #
    # bug3: mapd's tileLoaded is in-memory and only re-reads disk on a 0.25-degree area crossing or a
    # restart (mapd/main.go:95-103), so after deleting the tiles it stays True -> "covered"/"map_here"
    # stay True -> the uncovered-triggers-download block below never fires while parked, defeating the
    # button. Directly (re)send the download request for this region right here instead of waiting on
    # that path, gated on downloadProgress.active so this never stacks a pull on top of one already
    # running (also gates the delete itself -- no point deleting tiles for a region mid-download).
    if params.get_bool("RefreshLocationMap"):
      dl_active = sm.alive['mapdExtendedOut'] and sm['mapdExtendedOut'].downloadProgress.active
      if has_fix and not dl_active:
        g = sm[gps_service]
        region, key = coverage.region_and_key_for_gps(float(g.latitude), float(g.longitude))
        if region is not None and key is not None:
          is_us_state = key.startswith("us_state.")
          n = _delete_region_tiles(region, is_us_state)
          cloudlog.warning(f"mapd_configd: RefreshLocationMap deleted {n} tile dir(s) for {region}")
          msg = messaging.new_message('mapdIn')
          msg.mapdIn.type = 'download'
          msg.mapdIn.str = key
          pm.send('mapdIn', msg)
          last_requested_region = region
          last_request_ts = time.monotonic()
          region_attempts = 1            # mapdgate2pnw: manual refresh restarts the escalation
          cloudlog.warning(f"mapd_configd: RefreshLocationMap re-requested download {key}")
        else:
          cloudlog.warning("mapd_configd: RefreshLocationMap: no region under current GPS fix; no-op")
      elif dl_active:
        cloudlog.warning("mapd_configd: RefreshLocationMap: download already in progress; no-op")
      else:
        cloudlog.warning("mapd_configd: RefreshLocationMap: no GPS fix; no-op")
      params.put_bool("RefreshLocationMap", False)

    # mapdstate2pnw: GPS-driven, on-demand, whole-STATE download. Uncovered = we have a fix but mapd
    # has no tile loaded here. Requests on ANY network (not gated to unmetered Wi-Fi like the old
    # fixed WA/OR/ID download) — the owner is fine burning cellular data for this on an interstate;
    # the alternative is being stranded without maps in the back country. Always the enclosing US
    # STATE, or the enclosing NATION for a non-US fix (never the whole US as a country — see
    # coverage.region_and_key_for_gps / region_for_gps's docstrings for why that's structural, not a
    # runtime check here).
    #
    # Re-arm rule (avoids spamming mapdIn / restarting a state mid-download):
    #   - never while mapdExtendedOut.downloadProgress.active (a pull is already running)
    #   - only send when the region CHANGED since our last request, OR REGION_RESEND_INTERVAL_S has
    #     elapsed (covers a request mapd missed because its mapdIn socket wasn't up yet — the same
    #     reason the old one-shot code resent every loop, just capped instead of literally 1 Hz)
    # GPS jitter across a state line is not expected to thrash this: consumer-GPS noise is tens of
    # meters, negligible against a state bbox, and region_for_gps() picks the tightest enclosing
    # box, so the region code itself doesn't flicker at typical highway speeds. A genuine border
    # crossing (e.g. dipping across I-5's WA/OR line) IS meant to re-trigger — that's the feature.
    uncovered = has_fix and not tile_here
    # mapdgate2pnw: coverage arrived -> the region is fixed, so drop any accumulated backoff. If it
    # goes uncovered again later that is a NEW problem and deserves a prompt retry, not the old cap.
    if tile_here and region_attempts:
      region_attempts = 0
    if uncovered and sm.alive['mapdExtendedOut'] and not sm['mapdExtendedOut'].downloadProgress.active:
      g = sm[gps_service]
      region, key = coverage.region_and_key_for_gps(float(g.latitude), float(g.longitude))
      now = time.monotonic()
      if region is not None and key is not None:
        # mapdgate2pnw: a genuinely NEW region always requests immediately and starts its own count.
        if region != last_requested_region:
          region_attempts = 0
        if region != last_requested_region or now - last_request_ts > next_region_interval(region_attempts):
          msg = messaging.new_message('mapdIn')
          msg.mapdIn.type = 'download'
          msg.mapdIn.str = key
          pm.send('mapdIn', msg)
          last_requested_region = region
          last_request_ts = now
          region_attempts += 1
          nxt = next_region_interval(region_attempts)
          cloudlog.warning(f"mapd_configd: uncovered {region}; req {key} (try {region_attempts}, next {nxt:.0f}s)")


if __name__ == "__main__":
  main()
