#!/usr/bin/env python3
"""
mapd2pnw: one-shot PNW map auto-download for the official pfeiferj mapd binary.

The mapd binary ships with every speed/curve CONTROL disabled (safe default) and downloads no map
data on its own. This tiny daemon asks mapd — once, on the first unmetered Wi-Fi connection — to
download the Pacific Northwest map set (Washington, Oregon, Idaho) via a `mapdIn` download message.

It is guarded by the MapdPnwMapsRequested param so it only fires once, and it keeps re-sending until
mapd reports the download started (the message can be missed if mapd's socket isn't up yet), then
stops. Speed-limit DISPLAY works as soon as the maps are present; the user opts into speed/curve
CONTROL later via MapdSettings — this daemon never enables control.
"""
import json
import math
import os
import subprocess
import time
import cereal.messaging as messaging
from cereal import log
from openpilot.common.gps import get_gps_location_service
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.system.mapd.installer import MAPD_BINARY   # mapdheal2pnw: self-heal relaunch path

# Download-menu paths are period-delimited keys from mapd's download_menu.json. The US states
# table is "us_state" (SINGULAR), e.g. "us_state.WA". Comma-join multiple areas.
PNW_DOWNLOAD = "us_state.WA,us_state.OR,us_state.ID"
NetworkType = log.DeviceState.NetworkType
OSM_OFFLINE_DIR = "/data/media/0/osm/offline"   # where mapd stores downloaded region tiles
MAPD_WATCHDOG_LOOPS = 45   # mapdheal2pnw: loops (~s) mapd may be silent before we self-heal relaunch it


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


def _on_unmetered_wifi(ds) -> bool:
  return ds.networkType == NetworkType.wifi and not ds.networkMetered


def main():
  params = Params()
  mem = Params("/dev/shm/params")   # CES + the on-road overlay read the legacy map params from here
  pm = messaging.PubMaster(['mapdIn'])
  gps_service = get_gps_location_service(params)
  sm = messaging.SubMaster(['deviceState', 'mapdExtendedOut', 'mapdOut', gps_service])

  if params.get_bool("MapdPnwMapsRequested"):
    cloudlog.info("mapd_configd: PNW maps already requested; idle (re-checks the param each loop)")
  last_covered = None
  mapd_down = 0            # consecutive loops mapd (mapdExtendedOut) has been silent — debounces "down"

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
        # mem-params, mirroring the RoadContext/MapOneWay pattern above. No consumer reads these yet —
        # this only makes the data flow + observable (ces_pnw telemetry) so a later phase can gate
        # curve-tiering / freeway-floor / posted-limit logic on them once validated against real drives.
        mem.put_nonblocking("MapHighwayClass", str(mo.highwayClass))
        mem.put_nonblocking("MapWayId", str(int(mo.wayId)))
        mem.put_nonblocking("MapConditionalSpeedLimit", mo.conditionalSpeedLimit or "")
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

    # mapd2pnw: drive the "Get map for this location" toggle grey-out (param MapForLocationCovered).
    # The toggle should be GREYED (covered) unless we KNOW we're somewhere with no downloaded map.
    # covered = no GPS fix (can't tell where we are, e.g. parked offroad) OR mapd has a map tile
    # loaded for the current position. Only a fix in an uncovered area enables the toggle. Replaces
    # the deleted sunnypilot coverage writer; written only on change to avoid churning the param.
    has_fix = sm.alive[gps_service]
    tile_here = sm.alive['mapdOut'] and sm['mapdOut'].tileLoaded
    covered = (not has_fix) or tile_here
    if covered != last_covered:
      params.put_bool("MapForLocationCovered", covered)
      last_covered = covered

    # Re-read the guard each loop (not once at startup) so re-arming the download — resetting
    # MapdPnwMapsRequested to 0 — takes effect without restarting this daemon.
    if params.get_bool("MapdPnwMapsRequested"):
      continue

    if not sm.alive['mapdExtendedOut']:
      continue  # mapd not up yet (binary still downloading at launch, or not started)

    # Check FRESH state at the top of the loop: did our request already start a download? If so, the
    # one-shot is complete. (Checking right after send() would read pre-send state and re-spam.)
    prog = sm['mapdExtendedOut'].downloadProgress
    if prog.active or prog.totalFiles > 0:
      params.put_bool("MapdPnwMapsRequested", True)  # one-shot guard; re-read at the top of the loop
      cloudlog.warning("mapd_configd: PNW download started; one-shot guard set")
      continue

    # Not started yet: on unmetered Wi-Fi, (re)send the download request. Resends each loop until mapd
    # picks it up (a message can be missed before mapd's mapdIn socket is ready), then the check above
    # ends it. mapd_configd is the only mapdIn publisher.
    if sm.alive['deviceState'] and _on_unmetered_wifi(sm['deviceState']):
      msg = messaging.new_message('mapdIn')
      msg.mapdIn.type = 'download'
      msg.mapdIn.str = PNW_DOWNLOAD
      pm.send('mapdIn', msg)
      cloudlog.warning(f"mapd_configd: requested PNW map download: {PNW_DOWNLOAD}")


if __name__ == "__main__":
  main()
