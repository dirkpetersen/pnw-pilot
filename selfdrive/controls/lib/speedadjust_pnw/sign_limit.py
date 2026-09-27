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
SIGN_MIN_MPH = 15.0              # a sign below this is a parking-lot / driveway sign, not a road's limit: ignored. Found by
                                 # replaying 2026-09-24: the module KEEPS the last value (23 of 24 routes started with the
                                 # lot's 5), so a 5 or 10 rode out onto the road -- 11:35:18 PT a 5 against the map's 40
                                 # at 38 mph for 11 s, 18:38:58 a 10 against I-5's 60 at 31 mph. 5 and 10 were seen only in
                                 # lots and at business entrances. speedadjust never targets below 10 mph (MIN_CAP) anyway.


class SignLimitSelector:
  def __init__(self):
    self._cam_v = 0.0            # the camera value being timed (m/s), 0 = none
    self._cam_t = None           # when it was first read
    self._out = 0.0              # the last returned limit (m/s)
    self._raise_t = None         # when the current no-map raise became eligible (map unknown, camera above prev)
    self._raise_v = 0.0          # ...and the camera value it is for
    self._agreed = 0.0           # the last value the known map and the camera agreed on (m/s), 0 = none yet
    # telemetry, read by SpeedAdjustController._publish_status
    self.src = None              # "map" / "camera" / "hold" (the previous working limit, kept while the camera persists)
    self.why = None
    self.cam = None              # the camera's posted number (as on the sign) while valid, else None
    self.cam_st = None           # the carState status name
    self.map_sl = None           # the map reading (m/s, 0 = unknown)

  def select(self, now: float, map_sl: float, map_fresh: bool, ann_sl, cam_num: float, cam_status: str,
             use_camera: bool) -> float:
    """map_sl: the sanity-checked MapSpeedLimit (m/s, 0 = none). map_fresh: mapd is publishing (NextMapSpeedLimit is
    fresh) -- a dead mapd leaves MapSpeedLimit at its last value. ann_sl: the lower limit mapd announces ahead (m/s) or
    None. cam_num: the number on the sign (the unit is the region's: mph on US roads -- NOT the car's unit flag, which
    follows the cluster). use_camera: the FordSignSpeedLimit toggle."""
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

    if cam_status == "unavailable":
      out, src, why = map_sl, "map", "unavailable"
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
      elif held >= SIGN_CONFIRM_S:
        out, src, why = cam, "camera", "cameraOverride"   # it holds the working limit against a map drop
      else:
        out, src, why = pending("pending")

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
              map=round(float(map_sl), 2), mapFresh=bool(map_fresh), held=round(float(held), 1))
    cloudlog.event("speedadjust_sign_limit", **kw)
    if why == "unavailable":
      cloudlog.error("speedadjust: this car declares a camera speed limit but carState reports it unavailable -- " +
                     "map limit only (is the opendbc pin older than fordtsr2pnw?)")
    elif why == "stale":
      cloudlog.warning(f"speedadjust: camera speed limit not usable ({self.cam_st}) -- map limit only while this lasts")
