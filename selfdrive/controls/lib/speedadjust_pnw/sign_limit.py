"""fordtsr2pnw -- weigh the car's own traffic-sign speed limit (the Lightning's camera, carState.cruiseState.speedLimitSign)
against the map limit (mapd, MapSpeedLimit) and return the ONE raw limit speedadjust works from.

Owner 2026-09-27: "we still want a Ford toggle for this, enabled by default but can be disabled, and it should always be
enabled if there is no map." Measured (drives/2026-09-24/ford-tsr-measure/DRIVE_REPORT.md, 4.5 h / 416 km):
  * the camera agrees with mapd 94 % of the time both are known, and usually changes ~1.4 s / 34 m BEFORE it;
  * it had the ODOT work-zone 55s mapd did not, and never showed mapd's bogus 25 at 19:02 (it held 45);
  * but it held a STALE 50 for 143 s in the SR 99 tunnel (posted 45), jumps to the freeway limit 23-30 s early on an
    on-ramp (the module is fused with Ford's nav map), and misreads briefly ~1.1 times an hour (a 25 for 1.6 s at 52 mph).
So the camera may LOWER the working limit, or HOLD it against a map drop, once its value has persisted; it may never
RAISE the working limit while the map is known; and with no usable map it is always used, raising only after a long
persistence.

The output is a RAW reading. speedadjust's own machinery still runs on it unchanged: the 2 s drop confirm, the 3 s rise
confirm, the 5 s dropout hold, and the look-ahead's option 2 (a drop the live look-ahead announced skips the confirm).
Police (limit + 5) and the owner's rules 1/1b apply to whatever this returns.

Rules, per read (~1 Hz). "held" = how long the camera has shown its current value (any change or invalid read restarts
it). "prev" = what this returned last time (the working limit it may not raise). "agreed" = the last value on which the
known map and the camera agreed.
  status unavailable (the car does not decode it) ....... map                   why "unavailable" (logged: the capability
                                                                               says this car has it)
  not in a US state, or position unknown ............... map, EVEN WITH NO MAP  why "region" (the number is read as mph;
                                                                               a km/h country's 50 would read 50 mph)
  status stale / noLimit / unreadable .................. map                   why "stale" / "noSign"
  a sign below 15 mph (a parking lot's 5 / 10) ......... map                   why "lotSign"
  map unknown (0, or mapd not publishing) .............. camera, ALWAYS, whatever the toggle says:
      camera <= prev, or nothing before ................ camera once held 3 s    "noMap" (until then prev: "pending")
      camera > prev .................................... camera once it has shown that value for 10 s OF MAP-UNKNOWN
                                                         time (until then prev: "heldHigher")
  toggle OFF ........................................... map                   "toggleOff"
  camera == map (within 1 mph) ......................... map                   "agree"
  camera < map, held >= 3 s ............................ camera                "cameraOverride"  (work zones, Castle Rock)
  camera < map, == the lower limit mapd announces ahead  camera at once         "cameraAhead"     (18:57: 7.2 s early)
  camera > map, camera <= prev, camera == agreed,
    held >= 3 s ........................................ camera (a HOLD)        "cameraOverride"  (19:02: map's false 25)
      ...but never for more than SIGN_HOLD_MAX_S ....... map                   "holdExpired"
      ...and never against the limit a LIVE look-ahead
         announced (the look-ahead owns that drop) ..... map                   "lookAhead"
  camera > map, otherwise .............................. map                   "heldHigher"      (tunnel stale 50, on-ramps)
The two HIGHER-camera guards were found by replaying 2026-09-24 (19:04:45 PT): mapd dropped out for 7 s just before the
SR 99 tunnel while the camera had shown its stale 50 for 65 s. Timed from the camera's first 50, the no-map raise passed
at once, and the hold then kept 50 against the map's 45 for the whole tunnel. So a raise is timed only while the map is
unknown, and the camera can hold only a value the map itself agreed on.
  not yet held long enough ............................. prev if the camera was the source, else map    "pending"
"""
import math

from openpilot.common.swaglog import cloudlog

MPH_TO_MS = 0.44704
SANE_MAX_SL = 90.0 * MPH_TO_MS   # the same garbage bound speedadjust applies to the map limit
SIGN_CONFIRM_S = 3.0             # a camera value must persist this long before it overrides the map (or, with no map,
                                 # replaces the working limit). The one clear misread of 09-24 lasted 1.6 s; all five
                                 # brief values were < 3 s. At the ~1 Hz read that is 4 consecutive reads.
SIGN_RAISE_S = 10.0              # ...and this long before it may RAISE the working limit (map unknown only). Raising is
                                 # the risky direction: the tunnel's stale 50 and the on-ramp jumps are all raises.
SIGN_EQ_TOL = 1.0 * MPH_TO_MS    # "agree" -- the report's definition (|camera - mapd| < 1 mph)
SIGN_HOLD_MAX_S = 8.0            # the longest the camera may HOLD a map-agreed limit against a lower map. 19:02's false 25
                                 # lasted 4.95 s, which the 1 Hz read sees as 4-6 reads; 8 s clears that with margin (the
                                 # same reasoning and value as the look-ahead's LA_PROMOTE_HOLD_S). A real drop the camera
                                 # is merely late for (median +1.8 s, up to 12 s on 09-24; a freeway 70 held down an
                                 # off-ramp onto a 25) must not be held longer (Fable review 2026-09-27).
                                 # The timer restarts if the map flickers back to the agreed value for one read (that ends
                                 # the hold stretch); a flickering map could so extend a hold. Accepted: a map that keeps
                                 # returning to the camera's value is corroborating it.
SIGN_MIN_MPH = 15.0              # a sign below this is a parking-lot / driveway sign, not a road's limit: ignored. Found by
                                 # replaying 2026-09-24: the module KEEPS the last value (23 of 24 routes started with the
                                 # lot's 5), so a 5 or 10 rode out onto the road -- 11:35:18 PT a 5 against the map's 40
                                 # at 38 mph for 11 s, 18:38:58 a 10 against I-5's 60 at 31 mph. 5 and 10 were seen only in
                                 # lots and at business entrances. speedadjust never targets below 10 mph (MIN_CAP) anyway.


# Known holes in mapd's bounding-box region table (system/mapd/coverage.py, which is for map DOWNLOADS and is left as is):
# the Alaska box covers all of Yukon and NWT (Whitehorse, Dawson City, the Dempster, Inuvik all resolve to us_state.AK),
# and the Washington box covers Victoria BC. The bbox result is necessary but not sufficient for "US state"; these two
# boxes are forced to NOT US (camera off), failing safe (Fable re-review 2026-09-27). North of 60.3 N the Alaska/Yukon
# border is the 141st meridian, so the cut is lon > -141 (Fable's -137.5 missed Dawson City, -139.4, and the Top of the
# World Highway). Cost: the camera is also off in the northern panhandle (Skagway, Haines, Yakutat -- lat >= 59) and at
# Neah Bay/Cape Flattery WA; Juneau (58.3) keeps it. NOT covered, still resolved as US by the bbox: Windsor ON (MI),
# Niagara Falls ON (NY), Prince Rupert BC (AK) -- a real country test is follow-up work.
CANADA_NORTH = (59.0, -141.0)                     # lat >= this and lon > this: Yukon / NWT / the Alaska Highway
VANCOUVER_ISLAND = (48.3, 49.0, -124.8, -123.1)   # lat_min, lat_max, lon_min, lon_max


def canada_override(lat: float, lon: float) -> bool:
  """True where mapd's bbox table says "US state" but the point is (or may be) in Canada: the camera must be off."""
  if lat >= CANADA_NORTH[0] and lon > CANADA_NORTH[1]:
    return True
  la0, la1, lo0, lo1 = VANCOUVER_ISLAND
  return la0 <= lat <= la1 and lo0 <= lon <= lo1


class SignLimitSelector:
  def __init__(self):
    self._cam_v = 0.0            # the camera value being timed (m/s), 0 = none
    self._cam_t = None           # when it was first read
    self._out = 0.0              # the last returned limit (m/s)
    self._raise_t = None         # when the current no-map raise became eligible (map unknown, camera above prev)
    self._raise_v = 0.0          # ...and the camera value it is for
    self._agreed = 0.0           # the last value the known map and the camera agreed on (m/s), 0 = none yet
    self._hold_t = None          # when the current hold against a lower map began
    self.region = None           # the region code the last read was judged in (telemetry / log)
    # telemetry, read by SpeedAdjustController._publish_status
    self.src = None              # "map" / "camera" / "hold" (the previous working limit, kept while the camera persists)
    self.why = None
    self.cam = None              # the camera's posted number (as on the sign) while valid, else None
    self.cam_st = None           # the carState status name
    self.map_sl = None           # the map reading (m/s, 0 = unknown)

  def select(self, now: float, map_sl: float, map_fresh: bool, ann_sl, cam_num: float, cam_status: str,
             use_camera: bool, region=None, region_us: bool = False, la_n=None) -> float:
    """map_sl: the sanity-checked MapSpeedLimit (m/s, 0 = none). map_fresh: mapd is publishing (NextMapSpeedLimit is
    fresh) -- mapd_configd clears MapSpeedLimit to 0.0 after ~5 s of silence (mapsl2pnw), so this covers the first seconds
    of a death. ann_sl: the lower limit mapd announces ahead (m/s) or None. cam_num: the number on the sign, read as MPH --
    so the camera is used in US STATES ONLY (region_us; NOT the car's unit flag, which follows the cluster).
    region: the region code for the log (None = position unknown).
    use_camera: the FordSignSpeedLimit toggle. la_n: the limit (m/s) a LIVE look-ahead episode announced, else None."""
    cam = float(cam_num) * MPH_TO_MS if cam_status == "valid" else 0.0
    if not (math.isfinite(cam) and 0.0 < cam <= SANE_MAX_SL):
      cam = 0.0
    self.cam = round(float(cam_num)) if cam > 0.0 else None
    lot = cam > 0.0 and float(cam_num) < SIGN_MIN_MPH
    if lot:
      cam = 0.0
    self.cam_st = cam_status
    self.map_sl = map_sl
    if cam <= 0.0:
      self._cam_v, self._cam_t = 0.0, None
    elif self._cam_t is None or abs(cam - self._cam_v) > SIGN_EQ_TOL:
      self._cam_v, self._cam_t = cam, now
    held = now - self._cam_t if self._cam_t is not None else 0.0
    prev, prev_src = self._out, self.src
    map_known = map_sl > 0.0 and map_fresh

    def pending(why):
      if prev_src in ("camera", "hold") and prev > 0.0:
        return prev, "hold", why
      return map_sl, "map", why

    self.region = region
    holding = False
    if cam_status == "unavailable":
      out, src, why = map_sl, "map", "unavailable"
    elif not region_us:
      out, src, why = map_sl, "map", "region"
    elif cam <= 0.0:
      out, src, why = map_sl, "map", ("lotSign" if lot else "noSign" if cam_status == "noLimit" else "stale")
    elif not map_known:
      if prev > 0.0 and cam > prev + SIGN_EQ_TOL:
        if self._raise_t is None or abs(cam - self._raise_v) > SIGN_EQ_TOL:
          self._raise_t, self._raise_v = now, cam
        wait = "heldHigher"
        held = now - self._raise_t           # a raise is timed only while the map is unknown
        need = SIGN_RAISE_S
      else:
        need, wait = SIGN_CONFIRM_S, "pending"
      if held >= need:
        out, src, why = cam, "camera", "noMap"
      elif prev > 0.0:
        out, src, why = prev, "hold", wait
      else:
        out, src, why = map_sl, "map", wait
    elif not use_camera:
      out, src, why = map_sl, "map", "toggleOff"
    elif abs(cam - map_sl) <= SIGN_EQ_TOL:
      out, src, why = map_sl, "map", "agree"
    elif cam < map_sl:
      if held >= SIGN_CONFIRM_S:
        out, src, why = cam, "camera", "cameraOverride"
      elif ann_sl is not None and abs(cam - ann_sl) <= SIGN_EQ_TOL:
        out, src, why = cam, "camera", "cameraAhead"
      else:
        out, src, why = pending("pending")
    else:                                    # the camera is HIGHER than the map
      if prev <= 0.0 or cam > prev + SIGN_EQ_TOL or abs(cam - self._agreed) > SIGN_EQ_TOL:
        # it would raise the working limit, or hold a value the map never confirmed: never while the map is known
        out, src, why = map_sl, "map", "heldHigher"
      elif la_n is not None and abs(map_sl - la_n) <= SIGN_EQ_TOL:
        # The map just produced the drop a LIVE look-ahead counted down to. The look-ahead owns it (it materializes on
        # this reading, and it is restorable if the drop proves false). Holding here made it abort "passed" and restore
        # the driver's set inside the lower zone (Fable review 2026-09-27).
        out, src, why = map_sl, "map", "lookAhead"
      else:
        holding = True
        if self._hold_t is None:
          self._hold_t = now
        if now - self._hold_t >= SIGN_HOLD_MAX_S:
          out, src, why = map_sl, "map", "holdExpired"
        elif held >= SIGN_CONFIRM_S:
          out, src, why = cam, "camera", "cameraOverride"   # it holds the working limit against a map drop
        else:
          out, src, why = pending("pending")

    if not holding:
      self._hold_t = None                    # a hold is one continuous stretch
    if why != "heldHigher" or map_known:
      self._raise_t = None                   # a no-map raise must be continuous
    if why == "agree":
      self._agreed = map_sl
    if (src, why) != (prev_src, self.why):
      self._log(why, out, cam, map_sl, map_fresh, held)
    self._out, self.src, self.why = out, src, why
    return out

  def _log(self, why, out, cam, map_sl, map_fresh, held) -> None:
    """Rule 2: every change of source or reason is a cloudlog event (camera and map values alongside), and a camera that
    cannot be used says so."""
    kw = dict(why=why, out=round(float(out), 2), cam=self.cam, camSt=self.cam_st, camMs=round(float(cam), 2),
              map=round(float(map_sl), 2), mapFresh=bool(map_fresh), held=round(float(held), 1), region=self.region)
    cloudlog.event("speedadjust_sign_limit", **kw)
    if why == "unavailable":
      cloudlog.error("speedadjust: this car declares a camera speed limit but carState reports it unavailable -- " +
                     "map limit only (is the opendbc pin older than fordtsr2pnw?)")
    elif why == "region":
      cloudlog.warning(f"speedadjust: camera speed limit OFF -- not in a US state (region {self.region}; None = position " +
                       "unknown). The sign number is read as mph; map limit only, even with no map")
    elif why == "stale":
      cloudlog.warning(f"speedadjust: camera speed limit not usable ({self.cam_st}) -- map limit only while this lasts")
