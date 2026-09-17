"""
CES — Conditional Experimental Switching (xnor)  ⚠️ NOT WIRED / NOT DEPLOYED

Decides per-cycle whether the longitudinal planner should run Chill (ACC/MPC) or Experimental
(blended e2e), keeping the car in Chill for steady cruising and flipping to Experimental only for
curves, stop lights/signs, low-speed/complex (incl. city), and closing on a slow/stopped lead.

Design + decisions: see /home/dp/gh/comma/CES.md. Key properties:
  - Default Chill; ANY condition -> Experimental; return to Chill only when ALL clear + sustained +
    min-dwell (hysteresis on every threshold).
  - Per-condition FirstOrderFilter debounce (THRESHOLD ~ 1 s) — no flapping.
  - Tesla-only, longitudinal-only (Experimental does NOT change steering), default OFF.
  - 3-state top-right button override: CES / forced-Chill / forced-Experimental.

SAFETY: this module is PURE DECISION LOGIC. It does not command the car. It must be wired into the
effective-experimental computation (selfdrived) only after review + on-road verification. It never
touches panda safety. The decision core (`decide_active`) takes primitives and is unit-tested.
"""
import json
import math
import os
import time
from collections import deque

from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_CTRL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw_constants as C
# greenlight2pnw/greenlead2pnw: pure standstill->release detector + release-cause classifier
# (sunnypilot mechanics + FrogPilot arming/lead rules — attribution in green_light.py).
# Display/sound only; never gates control.
from openpilot.selfdrive.controls.lib.ces_pnw.green_light import GreenLightDetector, GL_EV_GREEN, GL_EV_LEAD
# ces2core2pnw: the CES2 decision core (CES2-STUDY.md adoptions) — runs SHADOW every tick, decides
# live only when the Ces2Core param is set (default OFF => v1 path below is byte-identical).
from openpilot.selfdrive.controls.lib.ces_pnw.ces2_core import Ces2Core, DivergenceCounter
from openpilot.selfdrive.controls.lib.pnw_vehicle import PnwVehicle
# curveslow-lightning: ICBM's vision apex uses the SAME lateral-accel target as the VTSC vision path
# (v_safe = v_ego*sqrt(A_LAT/|lat|)) so the two subsystems agree on what a camera-seen curve "means".
# descentcurve2pnw: MAP_SOURCE_HORIZON_M is mapd's hard 500 m path cap — ICBM's full-horizon map scan
# uses the same constant family as VTSC/MTSC so both scan exactly what mapd publishes.
# icbmcurv2pnw: the SAME pure measurement VTSC uses, run on the ICBM path too (see below).
from openpilot.selfdrive.controls.lib.vtsc_pnw.vtsc_pnw import polyline_curvature, polyline_curvature_at
from openpilot.selfdrive.controls.lib.vtsc_pnw.vtsc_constants import (A_LAT_TARGET as VTSC_A_LAT,
                                                                      MAP_SOURCE_HORIZON_M,
                                                                      MAP_SCALE_MIN)

# Persistent, append-only "each adoption" trail. Lives OUTSIDE /data/openpilot so it survives the
# boot overlay-swap AND swaglog rotation (a long drive rotates swaglog and would lose early events).
# One JSON line per CES mode transition, with GPS so we can map where each adoption happened.
# waysel2pnw: the VTSCStatus keys this module lifts off /dev/shm into every ces_events record.
# Module-level so vtsc_pnw's tests can assert the publisher actually emits all of them -- see
# VTSCController.overlay_payload(). Adding a key here without adding it there (or vice versa) silently
# produces a null column that reads as "the feature did not trigger"; that has happened three times.
VTSC_TELE_KEYS = ("mapRaw", "mapEff", "mapD", "mapFlr", "visK", "visD", "visV",
                  "mapK", "mapKD", "mapKV", "mapKN", "mapKAhead",
                  "apexCurvature", "apexDist", "vCurveSafe", "curveWin", "rsnMap", "rsnVis",
                  "timeToApex",
                  "mapErr",   # foldlog2pnw: VTSC's map-curve fold failed this tick ("" = it did not)
                  "gpsAge")   # vtscgpsage2pnw: age (s) of the fix VTSC's map fold used; null = none (NOT icbmGpsAge)

CES_EVENT_LOG = "/data/pnw/ces_events.jsonl"
CES_EVENT_LOG_MAX_BYTES = 20 * 1024 * 1024   # rotate at 20 MB per generation
# cesretain2pnw: keep N rotated generations (.1 .. .N), not one. A single .1 gave a ~1-day window:
# the 2026-08-26 Olympic Peninsula trip (261 mi, two cars) had ALREADY rotated past by the time it
# was analysed — .1 started AFTER the driving ended, and the whole trip had to be reconstructed from
# S3 qlogs instead. Heavy driving writes ~21 MB/day (measured on that trip), so a full week needs
# >=147 MB of ROTATED history: 8 generations x 20 MB = 160 MB clears it, 7 would NOT (140 MB).
# Worst case 180 MB including the live file — 0.2% of the 89 GB /data.
CES_EVENT_LOG_GENERATIONS = 8
# curvedbtel2pnw (docs/CURVEDB2PNW.md section 3.8): WITHOUT THIS DIRECTORY PHASE 1 IS POINTLESS.
# The rotation above retains 8 x 20 MB = 160 MB, i.e. ~17.6 driving hours at the measured 9.1 MB/h.
# The Phase-1 exit criteria (section 3.9 item 3) need 6-8 WEEKS of corridor driving RETAINED, so as
# shipped the log would generate exactly the dataset the go/no-go decision needs and then delete it
# weeks before that decision could be taken. Nothing on 3devpnw archives ces_events off-device.
#
# The fix is the cheapest of the two options section 3.8 offers, implemented as a MOVE rather than a
# copy: the generation the rotation is about to DESTROY (path.N) is os.replace()d into this directory
# instead. Same filesystem (/data), so it is a rename -- O(1), atomic, no extra bytes, and no CPU.
# That matters: rotate_event_log() runs synchronously inside _append_event(), which runs inside
# selfdrived's 100 Hz loop. A shutil.copy of 20 MB (~100-200 ms on this eMMC) or a gzip (seconds)
# would stall the control loop for tens of frames, so neither is acceptable here -- compress on the
# dev host after pulling, not on the car.
CES_ARCHIVE_DIR = "/data/pnw/ces_archive"
# 2 GB. Heavy driving writes ~21 MB/day (measured, 2026-08-26 Olympic Peninsula trip), so this is a
# ~95-day window -- comfortably past the 6-8 weeks section 3.9 asks for.
#
# WHAT IT COSTS, stated plainly (Fable 2026-09-16 -- an earlier version of this comment justified the
# size "against the 8.9 GB free on /data", which MISREAD that number). The free space on /data is not
# headroom: deleterd pins it at max(5 GB, 10% of /data) by evicting the oldest DRIVE SEGMENTS, so it
# reads ~8.9 GB no matter what else is stored. This archive lives outside Paths.log_root(), so the
# deleter cannot reclaim it -- it can only reclaim segments instead. Every byte here therefore
# displaces a drive segment 1:1, and 2 GB is roughly 150 segments (~2.5 h of driving) evicted
# EARLIER than they otherwise would have been. That is the trade, and it is deliberate: a segment is
# already uploaded by the time it is old enough to evict, whereas this corpus had no other copy.
#
# Eviction is LOUD (cloudlog.error, see prune_ces_archive): silently dropping the oldest data
# is the exact failure this constant exists to prevent, so it must never happen unnoticed.
CES_ARCHIVE_MAX_BYTES = 2 * 1024 * 1024 * 1024
# stophold2pnw (D): the comma 3X RTC battery is dead (RTC reads 1970) — every cold boot writes
# event records with a garbage wall clock until NTP/GPS sync (a 2025-11-25-stamped record polluted
# the 2026-07-12 gap analysis). Records written before the clock is plausibly valid are MARKED
# (never dropped — the data is still real, only the timestamp is not).
CLOCK_VALID_EPOCH = 1577836800.0   # 2020-01-01T00:00Z


def clock_bad(t_wall: float) -> bool:
  """True when the wall clock is obviously pre-sync (dead-RTC boot). Pure."""
  try:
    return float(t_wall) < CLOCK_VALID_EPOCH
  except (TypeError, ValueError):
    return True


def prune_ces_archive(archive_dir: str | None = None, max_bytes: int | None = None) -> int:
  """curvedbtel2pnw (section 3.8): keep the archive under `max_bytes` by deleting the OLDEST files
  first. Returns how many were deleted (0 = under budget, nothing to do).

  Rule 2: an eviction is DATA LOSS against the Phase-1 retention window, so it logs at error level
  rather than silently making room. Every failure path logs its own specific errno too -- a
  permission problem or a vanished directory must not read as "the archive is fine, 0 evicted".
  Never raises: this runs on the rotation path inside selfdrived's 100 Hz loop."""
  # Resolved here, not as default arguments: a default is bound at def time, so the module constants
  # could not be overridden (by a test, or by a future param) without editing this signature.
  archive_dir = CES_ARCHIVE_DIR if archive_dir is None else archive_dir
  max_bytes = CES_ARCHIVE_MAX_BYTES if max_bytes is None else max_bytes
  entries = []
  try:
    for name in os.listdir(archive_dir):
      fp = os.path.join(archive_dir, name)
      try:
        if not os.path.isfile(fp):
          continue
        st = os.stat(fp)
      except OSError as e:
        cloudlog.error(f"ces_pnw: ces_archive stat failed for {fp} ({type(e).__name__}) -- it is not counted " +
                       f"against the {max_bytes} byte budget, so the archive may overshoot")
        continue
      entries.append((st.st_mtime, st.st_size, fp))
  except OSError as e:
    cloudlog.error(f"ces_pnw: ces_archive listdir FAILED ({type(e).__name__}) at {archive_dir} -- the budget is " +
                   "NOT being enforced; /data can fill up")
    return 0
  total = sum(sz for _, sz, _ in entries)
  if total <= max_bytes:
    return 0
  entries.sort()                       # oldest mtime first
  removed, freed = 0, 0
  for _, size, fp in entries:
    if total <= max_bytes:
      break
    try:
      os.remove(fp)
    except OSError as e:
      cloudlog.error(f"ces_pnw: ces_archive could not delete {fp} ({type(e).__name__}) -- still over budget")
      continue
    total -= size
    freed += size
    removed += 1
  if removed:
    cloudlog.error(f"ces_pnw: ces_archive OVER BUDGET -- deleted the {removed} oldest generation(s) ({freed} bytes). " +
                   f"The curvedb Phase-1 retention window has been TRUNCATED at the old end; pull {archive_dir} " +
                   "to the dev host before the next eviction.")
  return removed


def archive_rotated_generation(path: str, generations: int, archive_dir: str | None = None,
                               max_bytes: int | None = None):
  """curvedbtel2pnw (section 3.8): rescue the generation rotate_event_log is about to destroy.

  rotate_event_log shifts .1->.2 ... .(N-1)->.N, which OVERWRITES the pre-existing .N. That file is
  the oldest driving history the device holds, and at 8 generations it is only ~17.6 driving hours
  old -- far short of the 6-8 weeks the Phase-1 gate needs. Move it into `archive_dir` first.

  os.replace, not copy: /data/pnw and /data/pnw/ces_archive are the same filesystem, so this is a
  rename -- constant time, atomic, and it does not double the bytes on a /data that is at 90 %.
  A cross-device archive_dir would raise EXDEV, which is logged (loudly) rather than swallowed.

  Returns the destination path, or None when there was nothing to archive / the archive failed.
  Never raises -- the live log must keep rotating even if archiving is broken."""
  archive_dir = CES_ARCHIVE_DIR if archive_dir is None else archive_dir   # see prune_ces_archive
  oldest = f"{path}.{max(int(generations), 1)}"
  try:
    st = os.stat(oldest)
  except FileNotFoundError:
    return None                        # fewer than N rotations so far -- nothing is being destroyed yet
  except OSError as e:
    cloudlog.error(f"ces_pnw: ces_archive could not stat {oldest} ({type(e).__name__}) -- that generation is " +
                   "about to be DESTROYED by the rotation and is NOT archived")
    return None
  try:
    os.makedirs(archive_dir, exist_ok=True)
  except OSError as e:
    cloudlog.error(f"ces_pnw: ces_archive mkdir FAILED ({type(e).__name__}) at {archive_dir} -- ces_events history " +
                   "is being DESTROYED on every rotation; the curvedb Phase-1 corpus is not accumulating")
    return None
  # Name by the file's own mtime (when it last rotated out of live), not by "now": the archive is
  # then sorted by content time, which is what prune_ces_archive's oldest-first eviction needs.
  base = os.path.basename(path)
  stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime(st.st_mtime))
  dest = os.path.join(archive_dir, f"{base}.{stamp}")
  n = 1
  while os.path.exists(dest):          # two rotations inside one second (or a re-run) must not collide
    dest = os.path.join(archive_dir, f"{base}.{stamp}.{n}")
    n += 1
  try:
    os.replace(oldest, dest)
  except OSError as e:
    cloudlog.error(f"ces_pnw: ces_archive move FAILED ({type(e).__name__}) {oldest} -> {dest} -- that generation " +
                   "is about to be DESTROYED by the rotation; the curvedb Phase-1 corpus is NOT accumulating")
    return None
  prune_ces_archive(archive_dir, max_bytes)
  return dest


def rotate_event_log(path: str, generations: int) -> None:
  """cesretain2pnw: shift path.1..path.(N-1) down one and move path -> path.1, keeping `generations`
  rotated files. Oldest (path.N) is dropped. Each step is an atomic os.replace, so a crash mid-rotate
  loses at most one generation and never the live file. Caller has already checked the size.

  curvedbtel2pnw (section 3.8): "dropped" now means "moved to CES_ARCHIVE_DIR" -- see
  archive_rotated_generation. It runs FIRST, before the shift that would overwrite path.N, and it
  never raises, so a broken archive degrades to exactly the pre-curvedbtel2pnw behaviour (the
  generation is lost) with a loud swaglog line instead of silence."""
  archive_rotated_generation(path, generations)
  for i in range(max(int(generations), 1) - 1, 0, -1):
    try:
      os.replace(f"{path}.{i}", f"{path}.{i + 1}")
    except OSError:
      pass          # that generation doesn't exist yet -> nothing to shift
  os.replace(path, f"{path}.1")


# steerpower2pnw: LOGGING ONLY -- measure the truck's true hands-off steering capability by direction.
# achLat = achieved curvature (kActl, yaw-rate-derived, already logged as slKActl/kActl elsewhere) *
# vEgo^2, signed -- the delivered lateral accel this tick. Grouping the steerEvent peakAchLat by the
# compass heading below (offline) yields max(peakAchLat) per direction -> a capability map ->
# slowdown target v=sqrt(cap*R). Both helpers are PURE, no I/O, never raise -- controlsd.py carries an
# independent copy of _ach_lat (as `_ach_lat_ms2`) since the two processes don't share code, only the
# formula (see that function's docstring for why it's duplicated, not imported).
def _ach_lat(k_actl, v_ego):
  """Delivered lateral accel this tick = k_actl * v_ego**2 (signed, m/s^2). None/non-finite k_actl or
  v_ego degrades to None, never raises."""
  try:
    if k_actl is None or v_ego is None:
      return None
    ach = float(k_actl) * float(v_ego) ** 2
    return ach if math.isfinite(ach) else None
  except (TypeError, ValueError):
    return None


# =================================================================================================
# curvedbtel2pnw -- CURVEDB2PNW.md PHASE 1 (sections 3.1-3.7). TELEMETRY ONLY.
#
# NOTHING BELOW IS READ BY ANY CONTROL PATH. Phase 1 exists so that in 6-8 weeks there is data good
# enough to decide whether a learned curve database (Phase 2) is worth building at all, and it pays
# for itself meanwhile on the open ICBM phantom-slowdown items (2026-09-05, 2026-09-08).
#
# The four measured facts that motivate each helper -- re-measured on this workbench's own corpora
# before writing any of it, because Phase 2's whole premise rests on them:
#
#   D1  achieved curvature is DEAD ON THE TESLA. On 2026-09-03 (hotspot-drive-tesla, 7,221 ticks
#       above 5 m/s) `slKActl` is exactly 0.0 on 7,218 and null on 3 -- non-zero ZERO times. The
#       same field on the Lightning (2026-09-08 phantom corpus, 1,136 moving ticks) is non-zero on
#       1,097. Root cause (Fable): controlsd computes kActl = CS.yawRate / vEgo, and
#       opendbc/car/tesla/carstate.py never sets ret.yawRate -- and the Raven party DBC carries no
#       yaw-rate signal at all, so there is nothing to fix in the car interface. A field that reads
#       0.0 curvature is not "no data", it reads as PERFECTLY STRAIGHT ROAD, which is the most
#       dangerous possible default for anything that would later cancel a slowdown. -> _pose_curvature
#       derives it from livePose instead (section 3.1); no opendbc change, no submodule pin bump,
#       which is what keeps Phase 1 behaviour-neutral.
#   D1b a real zero and a dead sensor must never be indistinguishable. -> _zero_is_null (section 3.2),
#       the same discipline gassettel2pnw applied to vLift/vGasMax.
#   D5  the record samples INSTANTANEOUSLY at ~1 Hz while control runs at 100 Hz, so 99 of every 100
#       readings are thrown away. -> CurvePeak (section 3.3). `max`, not mean: Fable's 10 Hz qlog
#       test (75 segments, 1,584 s) found the straight-road per-second max is p99 0.31 / max 0.53
#       m/s^2 with overrides and lane changes excluded, while on curves the within-second max/mean
#       ratio is only 1.06 -- so the peak is not noise-dominated and averaging would erase the curve.
#   D3  the record says where the TRUCK is and how far the candidate is, but never where the
#       candidate IS. -> map_candidate_point (section 3.4). Map/far ticks sit median 49 m, p75 91 m,
#       p90 129 m before the candidate, so clustering truck positions smears one episode across
#       several "sites", mostly covering the straight approach.
# =================================================================================================

# The yaw-rate -> curvature divide's speed floor. DELIBERATELY the same value controlsd uses
# (drive_helpers.MIN_SPEED == 1.0, controlsd.py:446/531) and NOT the 0.1 the design text names:
# kPeak's achieved half has to stay numerically comparable to the slKActl/achLat pair it is checked
# against (section 3.7's cross-consistency invariant), and a 0.1 floor would amplify crawl-speed yaw
# noise by 10x and let every stop-and-go second dominate the per-second max. Duplicated rather than
# imported for the same reason _ach_lat is duplicated in controlsd: keeping selfdrived's import graph
# unchanged. test_curvedbtel2pnw.py pins it equal to drive_helpers.MIN_SPEED so a drift fails loudly.
CURVE_MIN_SPEED = 1.0

# map_candidate_point's match tolerance. mapDist is itself a haversine to a point in the SAME list,
# so a correct match is exact; 30 m is section 3.7's own invariant bound, and anything worse means
# the point list changed underneath us -> log null rather than a coordinate that is quietly wrong.
CURVE_CAND_TOL_M = 30.0

# section 3.5 disqualifier bits. One OR'd field per record, so a whole second can be admitted or
# rejected without re-deriving it from sampled flags that may have been between events at the sample
# instant. Kept as a bitmask internally (one `|=` per tick) and rendered to names only at record time.
DQ_SAT = 1        # steering saturation -- the pass under-reports the road (defect D2)
DQ_DRIVER = 2     # driver steering override
DQ_LANECHG = 4    # lane change in progress
DQ_BLINKER = 8    # turn signal on (signalled turn / exit, not the through road)
_DQ_NAMES = ((DQ_SAT, "sat"), (DQ_DRIVER, "drv"), (DQ_LANECHG, "lc"), (DQ_BLINKER, "blnk"))


def _dq_names(bits: int) -> str:
  """Render a disqualifier bitmask to a stable, comma-joined name list ("" when clean). Pure."""
  try:
    return ",".join(n for b, n in _DQ_NAMES if int(bits) & b)
  except (TypeError, ValueError):
    return ""


def _zero_is_null(x):
  """curvedbtel2pnw section 3.2 (P1-B): an exactly-zero curvature/lateral reading logs as null.

  WHY: see D1 in the block above -- on the Tesla `slKActl` reads 0.0 on 7,218 of 7,221 moving ticks,
  which is a dead signal wearing the costume of a measurement. Downstream, "null" is a question and
  "0.0" is an answer; only one of those is honest here. Section 3.2's companion rule lives offline:
  a pass is admissible only if that sensor produced a non-zero reading somewhere else on the SAME
  drive, which is a count of non-nulls (tools/curvedb_telemetry_check.py reports exactly that).

  Applied to the value AS LOGGED, i.e. after rounding. A reading that rounds to zero at the field's
  own precision (1e-6 1/m, 1e-3 m/s^2) is below any physical meaning on a road, so collapsing it
  into the same "prove the sensor was alive elsewhere" bucket is correct and keeps the rule simple:
  a 0.0 never appears in these fields at all. Pure."""
  if x is None:
    return None
  return None if x == 0.0 else x


def _curvature_from_yaw(yaw_rate, v_ego):
  """Signed path curvature (1/m) from a yaw rate (rad/s) and speed (m/s): k = yaw / max(v, floor).
  None (never 0.0, never an exception) on missing or non-finite input -- see _zero_is_null. Pure."""
  try:
    if yaw_rate is None or v_ego is None:
      return None
    k = float(yaw_rate) / max(float(v_ego), CURVE_MIN_SPEED)
    return k if math.isfinite(k) else None
  except (TypeError, ValueError):
    return None


def _pose_curvature(live_pose, v_ego):
  """curvedbtel2pnw section 3.1 (P1-A): achieved curvature for BOTH cars, from the localizer instead
  of from CAN.

  `livePose.angularVelocityDevice.z` IS the yaw rate -- locationd publishes it
  (locationd.py:221) and openpilot's own tests name it as such
  (locationd/test/test_locationd_scenarios.py: 'yaw_rate': ['angularVelocityDevice', 'z']). It is
  present on every car because it comes from the device's own IMU + vision, which is exactly why it
  fixes D1 without touching a car interface: no opendbc change, no submodule pin bump, no
  both-car regression surface. livePose is ALREADY in selfdrived's SubMaster (selfdrived.py:117), so
  this adds no subscription either.

  KNOWN AND DELIBERATE APPROXIMATION -- device frame, not calibrated frame. paramsd and torqued both
  run Pose.from_live_pose() through the calibrator before using .z as a vehicle yaw rate; this does
  not, because the calibrator is not reachable from here without plumbing selfdrived state into CES.
  The residual is the mount misalignment (a few degrees of yaw/pitch/roll), so the error is a
  ~0.1 % scale term plus cross-coupling from the other two axes. That is immaterial for telemetry,
  and section 3.1 asks for exactly this cross-check to QUANTIFY it: on the Lightning both sources
  are logged side by side (kPose vs slKActl), and tools/curvedb_telemetry_check.py reports their
  ratio. If that ratio turns out not to be ~1.0, the calibrated pose is the fix -- but measure first.

  Gated on `.valid`, the same first test paramsd applies (paramsd.py:73). An invalid pose logs null,
  not 0.0 -- an uninitialised localizer must not read as a straight road. Pure, never raises."""
  try:
    av = live_pose.angularVelocityDevice
    if not av.valid:
      return None
    return _curvature_from_yaw(av.z, v_ego)
  except (AttributeError, KeyError, TypeError, ValueError):
    return None


def map_candidate_point(points, cur_lat, cur_lon, target_dist, tol_m: float = CURVE_CAND_TOL_M):
  """curvedbtel2pnw section 3.4 (P1-D): the (lat, lon) of the mapd path point a candidate DISTANCE
  refers to -- where the curve is, as opposed to where the truck is.

  Matched the same way map_turn_direction already matches (same haversine, nearest |d - target_dist|
  over the same cached point list), so for map/far candidates the match is EXACT: target_dist was
  computed as a haversine to a point in this very list, on this very tick. `tol_m` therefore only
  ever fires when the list changed underneath us, and then the answer is (None, None) -- a null,
  not a coordinate that is silently 300 m wrong.

  target_dist of 0.0 / None / inf means "no candidate this tick" (decision_telemetry renders an
  infinite mapDist as 0.0), and returns (None, None). Pure, never raises."""
  try:
    td = float(target_dist) if target_dist is not None else 0.0
  except (TypeError, ValueError):
    return None, None
  if not points or cur_lat is None or cur_lon is None or td <= 0.0 or not math.isfinite(td):
    return None, None
  best, best_err = None, float('inf')
  for p in points:
    try:
      la, lo = float(p["latitude"]), float(p["longitude"])
    except (KeyError, TypeError, ValueError):
      continue
    if la != la or lo != lo:                          # NaN guard, as every other scanner here does
      continue
    err = abs(_haversine_m(cur_lat, cur_lon, la, lo) - td)
    if err < best_err:
      best, best_err = (la, lo), err
  if best is None or best_err > tol_m:
    return None, None
  return best


class CurvePeak:
  """curvedbtel2pnw section 3.3 (P1-C) + 3.1 + 3.5: the 100 Hz accumulator each ces_events record
  drains. Pure arithmetic on primitives -- no I/O, no clock, no messaging, no exceptions.

  WHY IT LIVES HERE AND NOT IN A DAEMON, which is the single most important structural fact about
  this feature: `slKCmd`/`slKActl` are NOT available at control rate outside controlsd. controlsd
  publishes SteerLimitStatus behind a `% 20 == 0` gate (5 Hz) and ces_pnw reads it in _read_map() at
  ~1 Hz, so a "per-second max" over that feed is a max of at most 5 samples, not 100. And a
  background process polling faster would need carState/controlsState msgq subscriptions -- the
  exact pattern behind the 2026-07-13 uploader commIssue cascade
  ([[feedback-no-carstate-sub-in-background-procs]]). So: the daemon compacts, ces_pnw measures.

  WHY max(commanded, achieved) AND NOT ACHIEVED ALONE (defect D2): achieved curvature is bounded by
  steering authority. 30 % of hands-off curve ticks carry a saturation flag, and on the 2026-09-08
  19:44 PSCM LimitReached event the request went 0.1225 -> 0.162 rad with the wheel flat. Where the
  truck cannot follow, achieved UNDER-reads the road -- precisely on the curves that matter. The
  commanded half is what the planner believed the road was; the achieved half is what happened.

  COST: three abs + three compares per tick. Proven affordable -- controlsd already runs an
  identical 100 Hz peak accumulator (_flight_peak_achlat_acc, controlsd.py:445-449) in a process
  with the same real-time budget.

  WINDOW SEMANTICS: take() drains and restarts, so a record's kPeak covers exactly the ticks since
  the PREVIOUS record -- normally ~100 (the ~1 Hz tick cadence), fewer around an adopt record.
  `n` is reported as kPeakN so "nothing was measured" (the visK failure mode: a field that is logged
  but never computed) is never confusable with "the road was straight"."""
  __slots__ = ("k_peak", "k_pose_peak", "n", "dq_bits")

  def __init__(self):
    self.reset()

  def reset(self) -> None:
    self.k_peak = 0.0
    self.k_pose_peak = 0.0
    self.n = 0
    self.dq_bits = 0

  def step(self, k_actl, k_cmd, k_pose, dq_bits: int = 0) -> None:
    """One control tick. Any of the three curvatures may be None (missing / non-finite / dead
    sensor) and simply does not contribute; `n` still counts the tick, so a window that ran but saw
    nothing is distinguishable from a window that never ran."""
    self.n += 1
    if k_actl is not None:
      a = abs(k_actl)
      if a > self.k_peak:
        self.k_peak = a
    if k_cmd is not None:
      a = abs(k_cmd)
      if a > self.k_peak:
        self.k_peak = a
    if k_pose is not None:
      a = abs(k_pose)
      if a > self.k_pose_peak:
        self.k_pose_peak = a
    if dq_bits:
      self.dq_bits |= int(dq_bits)

  def take(self) -> tuple:
    """Drain this window and start the next: (k_peak, k_pose_peak, n, dq_bits)."""
    out = (self.k_peak, self.k_pose_peak, self.n, self.dq_bits)
    self.reset()
    return out


# Every key _curve_tele() emits, pinned by test_curvedbtel2pnw.py. Same contract as
# CURVELEAD_TELE_KEYS/VTSC_TELE_KEYS: adding a field here without emitting it (or vice versa)
# silently produces a null column that reads as "the feature did not trigger" -- that has now
# happened four times in this file's history (visK, icbmKVis, waysel2pnw's eight, lcSpdA).
CURVE_TELE_KEYS = ("kPeak", "kPeakN", "kPoseP", "kPose", "achLatPose", "dq", "dqWhy", "strTq",
                   "mapLat", "mapLon", "mapCandD")


def _curve_tele(ctl, raw_vego, map_dist) -> dict:
  """curvedbtel2pnw: the telemetry fragment for BOTH ces_events record families (the tick/adopt
  record and the CES-off "steer" breadcrumb), so the two cannot drift -- the one-builder contract
  _curvelead_tele already carries.

  SIDE EFFECT, and it is deliberate: this DRAINS the accumulator (CurvePeak.take), exactly like
  _event_record consuming _gl_ev_pending. It must therefore be called at most once per record, and
  the two callers are mutually exclusive per tick (_steer_log_step returns immediately once
  _enabled is True, and _publish_status builds either an adopt OR a tick record, never both).

  `map_dist` is the distance of the candidate THIS RECORD'S icbmSrc names, and it is emitted beside
  the coordinates as `mapCandD` so a consumer can see which distance was matched rather than having
  to assume it was mapDist. None on the CES-off breadcrumb, which carries no map candidate at all --
  mapLat/mapLon are null there, and the section 3.7 check reports that as an expected-null
  population rather than a defect.

  Fable I1 (2026-09-16): this used to be handed the record's `mapDist`, which is CES's own 10 s
  candidate (`map_target_dist`, horizon ~308 m at 90 mph). ICBM's FAR source reaches
  MAP_SOURCE_HORIZON_M (500 m), so on a far-candidate record mapDist was 0.0/null or named a
  DIFFERENT, nearer curve -- i.e. mapLat/mapLon were wrong or absent on exactly the far-map
  phantoms section 3.4 was added to locate. The caller now passes the latched
  `_icbm_cand_d` (see _icbm_step) whenever ICBM's source is a map point.

  Never raises: a missing/absent accumulator (the permissive test stub, or a controller built before
  this feature) yields an all-null fragment with kPeakN 0, which is the honest reading."""
  peak = getattr(ctl, "_curve_peak", None)
  if peak is not None:
    k_peak, k_pose_peak, n, dq_bits = peak.take()
  else:
    k_peak, k_pose_peak, n, dq_bits = 0.0, 0.0, 0, 0
  pose_k = getattr(ctl, "_pose_k", None)
  map_lat, map_lon = map_candidate_point(getattr(ctl, "_map_targets", None),
                                         getattr(ctl, "_cur_lat", None), getattr(ctl, "_cur_lon", None),
                                         map_dist)
  # Same acceptance map_candidate_point applies (None / non-numeric / <=0 / inf all mean "no
  # candidate this tick"), so mapCandD is null exactly when mapLat/mapLon are null for that reason.
  try:
    cand_d = float(map_dist) if map_dist is not None else 0.0
  except (TypeError, ValueError):
    cand_d = 0.0
  return {
    # section 3.3: the per-second PEAK of max(|achieved|, |commanded|). Null when the window
    # measured nothing at all -- read kPeakN first, never kPeak alone.
    "kPeak": _zero_is_null(round(k_peak, 6)) if n else None,
    "kPeakN": n,
    # section 3.1 at peak resolution: the livePose-derived achieved half on its own. This is the
    # ONLY achieved curvature the Tesla has (D1), and section 3.9's exit criterion 1 ("alive on both
    # cars") cannot be met from a 1 Hz sample of a quantity section 3.3 itself argues must be peaked.
    "kPoseP": _zero_is_null(round(k_pose_peak, 6)) if n else None,
    # section 3.1 instantaneous, sampled exactly the way slKActl/achLat are sampled so the
    # cross-check between the two sources is apples-to-apples. Signed (direction is information).
    "kPose": _zero_is_null(round(pose_k, 6)) if pose_k is not None else None,
    "achLatPose": _zero_is_null(_round_or_none(_ach_lat(pose_k, raw_vego), 3)),
    # section 3.5: one OR over the window. None (not False) when the window never ran -- "no ticks"
    # is not "clean".
    "dq": bool(dq_bits) if n else None,
    "dqWhy": _dq_names(dq_bits) if dq_bits else None,
    # section 3.6: driver steering torque -- SEVERITY, where strPrs is only presence. Ford
    # SteeringColumnTorque is +-8 Nm and Tesla EPAS_torsionBarTorque +-20.5 Nm: same unit, different
    # scale, so it is per-car and must never be thresholded car-agnostically.
    # Known limitation, stated rather than hidden: this is the 1 Hz SAMPLE the design asks for, not
    # a peak, so a short override can be sampled at zero. `dq`/`dqWhy` still flag that second.
    "strTq": _zero_is_null(getattr(ctl, "_str_tq", None)),
    # section 3.4: where the CURVE is, not where the truck is. mapCandD is the distance mapLat/mapLon
    # were resolved AT -- without it, a null pair cannot be told apart from a pair matched against
    # the wrong candidate, and checker I3 has nothing sound to measure the coordinates against.
    "mapLat": map_lat, "mapLon": map_lon,
    "mapCandD": round(cand_d, 0) if cand_d > 0.0 and math.isfinite(cand_d) else None,
  }


_COMPASS_PTS = ("N", "NE", "E", "SE", "S", "SW", "W", "NW")


def _compass(bearing):
  """8-point compass heading from a GPS bearing (deg, 0=N, clockwise). None/non-finite bearing (no
  GPS fix yet) degrades to None, never raises."""
  try:
    if bearing is None:
      return None
    b = float(bearing)
    if not math.isfinite(b):
      return None
    return _COMPASS_PTS[int((b + 22.5) // 45) % 8]
  except (TypeError, ValueError):
    return None


# steerpower2pnw I4 review fix: a GPS fix that has lat/lon but no computed course (standstill, or the
# capnp bearingDeg field simply defaulting to 0.0 before the GPS stack has ever computed one) is
# indistinguishable from a genuine true-north 0.0 reading once it reaches _compass() -- silently
# biasing the "N" bucket with what are actually no-course-yet samples. Gate at the LOGGING site (not
# in _read_map -- other consumers of self._cur_bearing still want its 0.0 default) on the record's own
# `gps` boolean (lat AND lon both present -- the same test every record already uses to populate its
# "gps" field). A frozen/stale bearing while gps_valid is True is accepted (still logged, not nulled)
# -- only a genuinely absent fix degrades to None.
def _heading_if_fixed(bearing, gps_valid: bool):
  """8-pt compass heading, or None if `gps_valid` is False (no current lat/lon fix). Pure."""
  return _compass(bearing) if gps_valid else None


# steerpower2pnw I3 review fix: a steerEvent is emitted 0.75-5 s AFTER the saturation onset (the
# post-hold debounce in controlsd's flight-recorder state machine -- see FLIGHT_POST_HOLD_S /
# FLIGHT_MAX_EVENT_S), but heading was being stamped from self._cur_bearing, the ~1 Hz-refreshed
# value AT EMIT TIME. Through a curve, heading rotates ~15-20 deg/s, so the peak's true direction can
# be 45-90 deg off from the emit-time heading -- mis-filing peakAchLat under the wrong compass
# direction, defeating the by-direction capability map this whole feature exists to build. Fix: keep a
# small bounded ring of (wall_time, bearing, gps_valid) samples -- appended once per _read_map()
# refresh (~1 Hz, see its tail) -- and look up the sample NEAREST the episode's actual onset time
# (controlsd's "onsetT", see the _flight_start_wall comment there) instead of using the live value.
_BEARING_HIST_MAXLEN = 30   # ~30 s of ~1 Hz samples -- comfortably covers the 0.75-5 s emit lag


def _nearest_bearing(hist, t_wall):
  """Return (bearing, gps_valid) from the (wall_time, bearing, gps_valid) tuple in `hist` whose
  wall_time is closest to `t_wall`. (None, False) if `hist` is empty or `t_wall` isn't a real number.
  Pure; never raises."""
  if not hist:
    return None, False
  try:
    t_wall = float(t_wall)
  except (TypeError, ValueError):
    return None, False
  best = min(hist, key=lambda s: abs(s[0] - t_wall))
  return best[1], best[2]


# icbmonset: mapd's curvature calc (Heron's formula on near-collinear OSM nodes, see
# system/mapd/mapd_configd.py) occasionally emits a FINITE but physically-implausible high target
# velocity instead of NaN -- the existing NaN guards below don't catch it. Field-observed live
# 2026-07-18 (Ballard/Shilshole tight city curves, drive_report `drives/2026-07-18/
# lightning-icbm-curve/`): raw reads of 46.5-128.6 m/s (104-288 mph) at curve entry, settling to the
# real target (e.g. 14.6 m/s) 3-4 s later. The ceiling below is set ABOVE the highest RAW target ever
# exercised in this file's own test suite (110 mph / 49.2 m/s, `test_far_map_dec_only_above_ceiling_
# ignored` -- a genuine generous-sweeper reading, not noise) so it can never reclassify a previously
# real value as noise.
#
# SCOPE (Fable review, 2026-07-18): `_map_v_sane` is wired ONLY into `upcoming_curve` and
# `icbm_far_map_candidate` -- the two CANDIDATE scanners, where a rejected point can only ever REMOVE
# a value that would already have failed the reduce-only/binding test on its own (provably a no-op
# for every existing decision downstream: v_ego - tv is always very negative, tv*scale is always far
# above any sharp-curve/binding threshold). It is deliberately NOT wired into `icbm_map_reach`
# (reverted -- see that function's own docstring): pfeiferj's targetVelocity is a `sqrt(2/kappa)`-
# style function of curve RADIUS, so a wide/near-straight node (radius > ~1.7 km) legitimately
# reports tv > this ceiling, and a straight/unmatched node legitimately carries the capnp default
# 0.0 -- both are genuine "no slowdown needed" map verdicts on ordinary straight road, not noise.
# Gating REACH (map "coverage") on tv would have zeroed out coverage on straights and wrongly opened
# the MAP-FIRST gate (icbm_vision_may_start) for vision there -- the exact vision-over-slow class the
# 2026-07-12 driver rule ("vis=60 dec ticks") exists to suppress. `tv` alone can't distinguish "Heron
# glitch at a curve entry" from "legitimate gentle/straight road" -- reach stays position-based only.
#
# KNOWN PARTIAL COVERAGE (LOW, Fable finding 2): the ceiling can't be set below the legitimate 49.2
# m/s (110 mph) sweeper reading above without reclassifying real data as noise, so the observed
# 46.5-58 m/s slice of the garbage band (below this ceiling) still passes `_map_v_sane` and can still
# mask a real curve candidate in `upcoming_curve`/`icbm_far_map_candidate` the same way the >=60 m/s
# spikes used to. This is a PARTIAL fix for the primary late-onset stall, not a complete one -- if the
# stall recurs with an observed garbage read in the 46.5-58 m/s range, that is this known gap, not a
# new regression.
MAP_CURVE_V_SANITY_MAX_MS = 58.0   # m/s (~130 mph)


def _map_v_sane(tv) -> bool:
  """icbmonset: True when a raw mapd curve-target velocity (m/s) is finite, positive, and under the
  implausibility ceiling above. Wired into the two candidate scanners (upcoming_curve,
  icbm_far_map_candidate) so a curvature-noise spike is rejected the same way NaN already is, instead
  of being treated as a legitimate candidate. Deliberately NOT used by icbm_map_reach -- see the
  SCOPE note above. Pure; never raises."""
  try:
    tv = float(tv)
  except (TypeError, ValueError):
    return False
  return tv == tv and 0.0 < tv <= MAP_CURVE_V_SANITY_MAX_MS   # tv==tv rejects NaN


def vision_curve_lat_accel(orientation_rate_z, velocity_x, timebase, v_ego):
  """FrogPilot-style vision curve detector: predicted lateral accel + time-to-curve over the model
  horizon. Returns (predicted_lat_accel m/s^2, time_to_curve s). Pure; lists must be equal length."""
  if not orientation_rate_z or not velocity_x or not timebase:
    return 0.0, 1.0
  n = min(len(orientation_rate_z), len(velocity_x), len(timebase))
  best_acc, best_t, best_abs = 0.0, 1.0, -1.0
  for i in range(n):
    lat = orientation_rate_z[i] * velocity_x[i]   # yaw_rate * speed = lateral accel
    if abs(lat) > best_abs:
      best_abs, best_acc, best_t = abs(lat), lat, timebase[i]
  return best_acc, max(best_t, 1.0)


# icbm2pnw: comfort decel used to decide WHEN a curve starts binding the stock-ACC set speed. The
# stock ACC does the actual braking to the new set point; this only times the hand-off (gentle,
# truck-profile). Reduce-only: targets never exceed the driver's own latched set speed.
ICBM_A_DECEL = 0.8          # m/s^2 comfort approach decel
ICBM_MARGIN_M = 30.0        # start a little early — button steps + ACC response add latency
ICBM_MIN_DROP_MS = 1.0      # ignore caps within ~2 mph of the set speed (not worth taps)
# curveslow-lightning: floor on |predicted lateral accel| before a vision curve is even a candidate
# (div-by-zero guard + straightaway rejector). Reuses the CES vision-enter threshold so a curve the
# camera calls "not a curve" for the CES trip is also not one for ICBM.
ICBM_VISION_ENTER = C.CURVE_LAT_ACCEL_ENTER   # m/s^2 (1.9)
ICBM_VISION_EPS = 0.05                          # m/s^2 floor inside the sqrt (never divide by ~0)
# descentcurve2pnw: at 90 mph the drive's map candidate came from a 10 s time window (~400 m) while
# shedding 25 mph at the 0.8 comfort decel needs ~510 m — the curve became visible already-late.
# The full-horizon scan (ICBM_MAP_HORIZON_M = mapd's 500 m cap) closes that gap. For very LARGE
# drops the assumed approach decel firms from ICBM_A_DECEL toward the Lightning's tunable
# icbm_firm_decel (~1.4; stock ACC does the actual braking — this only shapes the tap-start
# envelope, and the ramp starts at 30 mph of drop so the field case, a 25 mph drop, still uses the
# full comfort envelope and binds at first sight).
ICBM_MAP_HORIZON_M = MAP_SOURCE_HORIZON_M   # m; scan the FULL published map path
ICBM_FIRM_DROP_LO = 13.4    # m/s (~30 mph) required drop where the approach decel starts firming
ICBM_FIRM_DROP_HI = 26.8    # m/s (~60 mph) required drop where it reaches the firm ceiling


ICBM_ERR_LOG_S = 30.0   # rule-2: throttle the _icbm_step failure log (it runs at ~4 Hz)
# silentexc2pnw (Rule 2): the lead-pacing failure log (_curvelead_failed) -- the first failure at once, then at most one
# line per this many seconds, naming the exception type and counting the failures since the previous line (the
# twistyr2pnw / foldlog2pnw pattern). Its own interval: ICBM_ERR_LOG_S above keeps throttling _icbm_step's own log.
CURVELEAD_ERR_LOG_S = 60.0
# gpslag2pnw (owner 2026-09-13: "build it, keep curve timing"). ICBM re-derives its own ego position on
# every ~4 Hz tick from the LastGPSPosition fix and its `fix_ts` (mapd_configd), instead of holding the
# 1 Hz read. The position is projected along the bearing to (now - ICBM_GPS_LAG_KEEP_S), NOT to now:
# every ICBM start rule is a distance threshold, and the weekend's ICBM starts were decided on an ego
# that lagged by a median 1.79 s (p5/95 1.09/2.31, drives/2026-09-12/central-oregon-weekend/gps/
# q1_results.txt). Projecting to now would start every slowdown ~v*1.8 s earlier. The keep is centred on
# the TRUCK fix, the Lightning's primary source (owner 2026-09-13): its fix_ts is the CAN receipt, and the
# truck position is itself ~0.20 s older than that (GPS_TRUCK_VS_COMMA.md s2), so 1.79 - 0.20 = 1.59 s
# reproduces the weekend's average start on the truck fix. Device-fix starts (the fallback) sit ~0.2 s
# earlier by design. Either way the spread goes (fix age, 1 Hz read phase, receiver switches).
ICBM_GPS_LAG_KEEP_S = 1.59
# A fix older than this is not projected: ICBM's map-curve lookups see NO GPS (no map/far candidate,
# map reach 0 so vision may start). 126 s of frozen device GPS in the SR 99 tunnel is the case.
ICBM_GPS_MAX_AGE_S = 5.0
# gpsdrgate2pnw (Fable, gpslag2pnw review): if the fix goes stale while a MAP/FAR cap episode is running, the
# episode would read the vanished candidates as "curve cleared" -> 3 s silent -> RESTORE toward the pre-curve
# set while still approaching the curve the map had rated (Fable's probe: publish emptied 137 m before it).
# Unknown is not clear, and a held lower set is the DEC-only direction, so the running cap is HELD until
# the truck has driven past where its binding candidate was (+ ICBM_MARGIN_M, integrated from v_ego), then
# the normal clear/apex-passed/restore path (with its in-curve pause) takes over. Bounded in time for a
# stopped truck; the value is a judgement (a held low set costs speed, not safety), not a measurement.
ICBM_GPS_STALE_HOLD_MAX_S = 60.0


def icbm_project_position(lat, lon, bearing, fix_ts, v_ego, now,
                          keep_s=ICBM_GPS_LAG_KEEP_S, max_age_s=ICBM_GPS_MAX_AGE_S):
  """gpslag2pnw: ICBM's ego position for THIS tick -> (lat, lon, age, state).

  state: "proj" projected by v_ego*(age - keep_s) along `bearing`; "stale" age outside [0, max_age_s]
  (lat/lon None: no GPS for the map lookups; a negative age is a `fix_ts` from a previous boot);
  "raw" no fix time or no usable bearing/speed (position used unprojected, as before gpslag2pnw);
  "none" no position. Pure; never raises."""
  if lat is None or lon is None:
    return None, None, None, "none"
  try:
    age = float(now) - float(fix_ts)
  except (TypeError, ValueError):
    return lat, lon, None, "raw"
  if not 0.0 <= age <= max_age_s:
    return None, None, age, "stale"
  try:
    d = float(v_ego) * (age - keep_s)
    brg = math.radians(float(bearing))
    la, lo = float(lat), float(lon)
    if not (math.isfinite(d) and math.isfinite(brg)):
      raise ValueError("non-finite projection")
  except (TypeError, ValueError):
    return lat, lon, age, "raw"
  return (la + d * math.cos(brg) / 111320.0,
          lo + d * math.sin(brg) / (111320.0 * max(math.cos(math.radians(la)), 1e-6)), age, "proj")


def icbm_approach_decel(v_ego, apex, firm_decel=0.0, a_base=ICBM_A_DECEL,
                        drop_lo=ICBM_FIRM_DROP_LO, drop_hi=ICBM_FIRM_DROP_HI):
  """descentcurve2pnw: assumed approach decel for the binding envelope — the base comfort decel for
  normal drops, ramping linearly toward `firm_decel` for very large (v_ego - apex) drops.
  firm_decel 0 / None / <= a_base (the non-Lightning default) -> a_base exactly (byte-identical to
  the pre-descentcurve behavior). Monotonic non-decreasing in the drop; always within
  [a_base, firm_decel]. Pure."""
  if not firm_decel or firm_decel <= a_base:
    return a_base
  drop = max(float(v_ego) - float(apex), 0.0)
  if drop <= drop_lo:
    return a_base
  if drop >= drop_hi:
    return firm_decel
  return a_base + (firm_decel - a_base) * (drop - drop_lo) / (drop_hi - drop_lo)


def icbm_vision_apex(v_ego, curve_lat_accel_vision, time_to_curve, a_lat=VTSC_A_LAT):
  """curveslow-lightning: turn the model's predicted lateral accel into a vision curve candidate for
  ICBM, mirroring the VTSC vision path. From lat_accel = v^2 * kappa the safe speed holding a_lat is
  vis_apex = v_ego*sqrt(a_lat/|lat|); distance = time_to_curve*v_ego. Returns (apex_v m/s, dist m), or
  (0.0, inf) when it is NOT a candidate (too straight / no speed). Pure; never raises."""
  try:
    lat = abs(float(curve_lat_accel_vision))
    v = float(v_ego)
    if v <= 0.0 or lat <= ICBM_VISION_ENTER:
      return 0.0, float('inf')
    apex = v * math.sqrt(a_lat / max(lat, ICBM_VISION_EPS))
    dist = max(float(time_to_curve) * v, 0.0)
    return max(apex, 0.0), dist
  except (TypeError, ValueError):
    return 0.0, float('inf')


def _icbm_binding_apex(v_ego, ref, apex, dist, a_decel=ICBM_A_DECEL):
  """One curve candidate -> the apex speed it commands IF it both (a) is reduce-only (apex sits
  ICBM_MIN_DROP_MS below the ceiling `ref`) and (b) has entered the brake envelope at `a_decel`
  (default = the comfort decel; descentcurve2pnw passes the drop-scaled icbm_approach_decel).
  Else None. Identical envelope math to the original map-only path. Pure."""
  if apex is None or dist == float('inf') or apex >= ref - ICBM_MIN_DROP_MS:
    return None
  brake_dist = max(v_ego * v_ego - apex * apex, 0.0) / (2.0 * a_decel) + ICBM_MARGIN_M
  if dist <= brake_dist:
    return max(apex, 0.0)
  return None


def icbm_far_map_candidate(points, cur_lat, cur_lon, v_ego, ref, scale_fn, map_scale=1.0,
                           firm_decel=0.0, horizon_m=ICBM_MAP_HORIZON_M):
  """descentcurve2pnw: FULL-horizon map candidate for ICBM. Scans every mapd path point out to
  `horizon_m` (mapd's 500 m publish cap — vs the old 10 s time window, ~400 m at 90 mph) and
  returns (apex_eff m/s, dist m) of the MOST-BINDING candidate — the one whose decel-limited brake
  cap is lowest right now (same selection idea as VTSC's most_binding_map_curve, so a far sharp
  curve can't shadow a nearer curve that needs action first). Candidates apply the shared tiered
  scale (scale_fn) AND the Lightning map-speed discount `map_scale` (<= 1.0, from PnwVehicle — OSM
  curve speeds are calibrated for stronger-steering cars) BEFORE the reduce-only test, so selection
  and use can't disagree. The per-point envelope uses the same drop-scaled icbm_approach_decel the
  downstream binding test uses. Returns (0.0, inf) if none. The actual DEC-only binding decision
  stays in icbm_curve_target/_icbm_binding_apex. NaN- and curvature-noise-guarded like upcoming_curve
  (icbmonset: `_map_v_sane` rejects implausible finite reads, not just NaN). Pure."""
  if not points or cur_lat is None or cur_lon is None or ref <= 0.0:
    return 0.0, float('inf')
  best_cap = float('inf')
  best_v, best_d = 0.0, float('inf')
  for p in points:
    try:
      d = _haversine_m(cur_lat, cur_lon, p["latitude"], p["longitude"])
      tv = float(p["velocity"])
    except (KeyError, TypeError, ValueError):
      continue
    if not _map_v_sane(tv) or d != d:    # icbmonset: NaN + curvature-noise guard (mapd emits both live)
      continue
    if not (0.0 < d <= horizon_m):
      continue
    eff = scale_fn(tv) * tv * map_scale
    if eff >= ref - ICBM_MIN_DROP_MS:
      continue                                  # reduce-only: not meaningfully below the ceiling
    a = icbm_approach_decel(v_ego, eff, firm_decel)
    cap = math.sqrt(eff * eff + 2.0 * a * max(d - ICBM_MARGIN_M, 0.0))   # decel envelope from here
    if cap < best_cap:
      best_cap, best_v, best_d = cap, eff, d
  return best_v, best_d


def icbm_map_reach(points, cur_lat, cur_lon, horizon_m=ICBM_MAP_HORIZON_M) -> float:
  """icbmmapfirst2pnw: how far ahead the published mapd path COVERS the road (m) — the farthest
  valid path point within mapd's horizon. 0.0 = no usable coverage (mapd down / no data / GPS lost /
  a stale path we have driven > horizon_m away from). Used by the MAP-FIRST start gate: a vision
  candidate INSIDE this reach is on a stretch the map has judged, so the map verdict (including "no
  slowdown needed") wins for STARTING episodes. Distances are recomputed from the CURRENT position
  every call, so a dead mapd's last path decays out of coverage as we drive on (mapd-liveness
  fallback: vision regains the right to initiate). NaN-guarded like the other scanners.

  icbmonset (POSITION-based only, deliberately NOT `_map_v_sane`-gated — see the reversion note at
  MAP_CURVE_V_SANITY_MAX_MS): reach means "mapd published a point here", independent of what its
  velocity says. pfeiferj's targetVelocity is `sqrt(2/kappa)`-derived: a wide/near-straight node
  (radius > ~1.7 km) legitimately reports a HIGH target velocity, and a straight/unmatched node
  legitimately carries the capnp default 0.0 — both are genuine "no slowdown needed here" verdicts,
  not noise, and gating reach on velocity would silently zero out coverage on ordinary straight
  road, wrongly opening icbm_vision_may_start there (the exact vision-over-slow class the
  2026-07-12 driver rule exists to suppress). Pure."""
  if not points or cur_lat is None or cur_lon is None:
    return 0.0
  reach = 0.0
  for p in points:
    try:
      d = _haversine_m(cur_lat, cur_lon, p["latitude"], p["longitude"])
    except (KeyError, TypeError, ValueError):
      continue
    if d != d:                                    # NaN guard
      continue
    if reach < d <= horizon_m:
      reach = d
  return reach


def icbm_in_curve(lat_accel_now, curve_lat_accel_vision, time_to_curve) -> bool:
  """icbmmapfirst2pnw (driver rule 2): True when the vehicle is ALREADY loaded in a curve — either
  the measured-now lateral accel (model yaw_rate*speed at t~0) is above the CES curve-exit
  hysteresis, or the camera's binding curve is effectively under us (a real vision curve with less
  than the act-window left). While True, no NEW dec episode may start (hold the current set), a
  restore may not BEGIN, and a running restore PAUSES. Defensive: bad input -> False (never blocks
  on garbage — the pre-mapfirst behavior). Pure."""
  try:
    if abs(float(lat_accel_now)) >= ICBM_IN_CURVE_LAT:
      return True
    return (abs(float(curve_lat_accel_vision)) > ICBM_VISION_ENTER
            and float(time_to_curve) < ICBM_VIS_MIN_TTC_S)
  except (TypeError, ValueError):
    return False


def icbm_vision_may_start(vis_dist, time_to_curve, map_reach) -> bool:
  """icbmmapfirst2pnw (driver rule 1): vision may INITIATE a new slow-down episode only when
  (a) the map does NOT cover that stretch (candidate beyond the map's coverage reach — includes
  mapd dead/blind, reach 0.0) AND (b) there is still time to act BEFORE the curve. Running
  episodes are not gated by this (callers apply it only when starting). Pure."""
  try:
    return float(vis_dist) > float(map_reach) and float(time_to_curve) >= ICBM_VIS_MIN_TTC_S
  except (TypeError, ValueError):
    return False


def icbm_curve_target(v_ego, v_set, map_v, map_dist, ceiling, scale_fn,
                      vis_v=0.0, vis_dist=float('inf'),
                      map_scale=1.0, firm_decel=0.0,
                      far_v=0.0, far_dist=float('inf'),
                      track=False):
  """Pure ICBM brain step (unit-tested). Returns (target_ms or None, new_ceiling or None, src or None)
  where src is "map" / "vis" / "far". Considers a MAP candidate (pfeiferj target, tiered-scaled like
  VTSC/MTSC), a VISION candidate (from icbm_vision_apex), and descentcurve2pnw's FAR-MAP candidate
  (full-horizon scan, already effective/scaled — from icbm_far_map_candidate) and returns the BINDING
  one with the LOWEST target (most slowing). Same DEC-ONLY, reduce-only, ceiling-latch semantics as
  before — with the descentcurve defaults (map_scale=1.0, firm_decel=0.0, no far candidate) this is
  byte-equivalent to the original.

  descentcurve2pnw knobs (all neutral by default, Lightning-supplied via PnwVehicle):
    map_scale  <= 1.0 discount on the map candidate's suggested speed (OSM speeds too generous for
               the Lightning's weak EPS) — applied BEFORE the reduce-only/binding tests;
    firm_decel assumed approach decel for very LARGE drops (icbm_approach_decel ramp; stock ACC does
               the actual braking — this only shapes the tap-start envelope).

  icbmtrack2pnw: track=True additionally lets MAP/FAR candidates START via the tracking window
  (_icbm_track_apex — set walks down early, e.g. while lead-bound) instead of only the v_ego decel
  envelope. VISION candidates are never tracked. track=False (default) is byte-identical to before.

  ceiling = the driver's own set speed latched when a cap first engages (None when uncapped); while
  capped, v_set follows the button-lowered stock set, so the latched ceiling is the only memory of
  what to restore to. Reduce-only: target is never above the ceiling."""
  if v_set <= 0:
    return None, None, None                # no valid set speed -> hands off
  ref = ceiling if ceiling is not None else v_set
  best_apex, best_src = None, None
  # MAP candidate — same tiered scaling as VTSC/MTSC, plus the Lightning map-speed discount.
  # Deliberately KEEPS the base comfort-decel envelope (NOT the drop-scaled firm decel): a firmer
  # assumed decel SHRINKS the envelope, i.e. starts taps LATER — inside the near window that would
  # be an under-brake regression vs pre-descentcurve behavior (Gemini review catch 2026-07-11).
  # The firm decel applies only to the FAR candidate below, where pre-diff there was NO braking at
  # all, so it can only ever ADD slowing.
  if map_v and map_v > 0 and map_dist != float('inf'):
    eff = scale_fn(map_v) * map_v * map_scale
    a = _icbm_binding_apex(v_ego, ref, eff, map_dist)
    if a is None and track:
      # icbmtrack2pnw: continuous set-tracking — a binding-RATED map curve within the tracking
      # window starts the walk-down even before the v_ego decel envelope binds (lead-bound case).
      a = _icbm_track_apex(v_ego, ref, eff, map_dist)
    if a is not None:
      best_apex, best_src = a, "map"
  # VISION candidate — already a safe speed (icbm_vision_apex). Lowest binding target wins (most slowing).
  if vis_v and vis_v > 0 and vis_dist != float('inf'):
    a = _icbm_binding_apex(v_ego, ref, vis_v, vis_dist)
    if a is not None and (best_apex is None or a < best_apex):
      best_apex, best_src = a, "vis"
  # FAR-MAP candidate — full-horizon scan (already effective: tiered scale + map_scale applied inside
  # icbm_far_map_candidate). Same binding envelope; catches curves beyond the 10 s window at speed.
  if far_v and far_v > 0 and far_dist != float('inf'):
    a = _icbm_binding_apex(v_ego, ref, far_v, far_dist, icbm_approach_decel(v_ego, far_v, firm_decel))
    if a is None and track:
      a = _icbm_track_apex(v_ego, ref, far_v, far_dist)   # icbmtrack2pnw (far_v already effective)
    if a is not None and (best_apex is None or a < best_apex):
      best_apex, best_src = a, "far"
  if best_apex is not None:
    new_ceiling = ceiling if ceiling is not None else v_set
    return best_apex, new_ceiling, best_src
  # curve cleared (or none): go silent and unlatch immediately. Caps remain DEC-ONLY
  # (Gemini-hardened 2026-07-11); icbmrestore2pnw layers the GUARDED restore as a separate
  # episode phase in IcbmEpisode below — this function itself never commands an increase.
  return None, None, None


# ---------------------------------------------------------------------------
# icbmrestore2pnw — guarded restore episode (driver-requested 2026-07-12)
# ---------------------------------------------------------------------------
ICBM_RESTORE_WINDOW_S = 45.0            # restore may run at most this long after the curve clears
ICBM_EXEC_STEP_MS = 1.0 * 0.44704       # one executor tap = 1 mph (mirror of ford icbm_pnw.STEP_MS)
ICBM_RESTORE_DONE_TOL = 0.6 * ICBM_EXEC_STEP_MS   # within this of the ceiling = restored (matches DEADBAND)
ICBM_DRIVER_LOWER_TOL = 1.7 * ICBM_EXEC_STEP_MS   # stock set below our lowest commanded target by more
                                                  # than this = the DRIVER lowered it -> never restore.
                                                  # icbmmapfirst2pnw: WIDENED 0.6 -> 1.7 steps on field
                                                  # forensics (2026-07-12 18:08:26Z, Snoqualmie->Ellensburg):
                                                  # the truck REPORTS the set speed with ~1 s of lag, so
                                                  # while chasing a falling vision target the executor lands
                                                  # one EXTRA tap after the reported set already met the
                                                  # target (min_target 77.3 mph, own floor 76.0 = 1.3 steps
                                                  # below). The old 0.6 tol misread our OWN late tap as a
                                                  # driver SET- and silently killed the restore — the
                                                  # driver's "no re-acceleration on a straight" complaint
                                                  # (stuck at 76 vs ceiling 85 for 33 s, manual gas+SET+).
                                                  # Worst legit self-overshoot = executor DEADBAND (0.6
                                                  # steps) + ONE report-latency tap (1.0) = 1.6 steps; 1.7
                                                  # covers it with margin. Residual (documented, mirrors
                                                  # RestoreGuard's SET+ residual): ONE driver SET- tap
                                                  # landing inside that band is indistinguishable from our
                                                  # own latency tap and now restores — still bounded by the
                                                  # driver's OWN ceiling, and every abort guard (incl. any
                                                  # set decrease DURING the restore) stays live. Two taps
                                                  # (>= 2 steps) still abort.
ICBM_TAP_PERIOD_S = 0.4                 # executor completes at most one tap per this (PRESS+GAP frames)
ICBM_RESTORE_DELAY_S = 3.0              # the curve must stay CLEAR this long before restore begins —
                                        # flicker-proofing: a 1-tick detection dropout must not start
                                        # pressing SET+ and then re-latch a lower ceiling when the
                                        # curve re-binds (S-curve gaps keep the ORIGINAL ceiling)

# --- icbmmapfirst2pnw (driver-directed rework, drive 2026-07-12 Snoqualmie->Ellensburg) -------------
# Field verdict: vision initiated 60/72 dec ticks (map 9, far 3), slowed too much and INSIDE curves,
# and the map's 500 m anticipatory horizon was under-used. New start policy (mirrors the VTSC
# sharpcurve2pnw shape): MAP-FIRST — with live map coverage over a stretch, the map verdict
# (including "no slowdown needed") is authoritative for STARTING slow-down episodes; vision may only
# INITIATE where the map is blind (beyond its coverage reach, or mapd down — the reach is computed
# from the CURRENT gps distance to the published path points, so a stale path left behind decays out
# of coverage and vision automatically takes back over) AND while there is still time to act. No new
# episode may START while the vehicle is already lateral-loaded in a curve — hold the current set. A
# RUNNING episode is untouched by all of this (it may continue steering the set, any source).
ICBM_VIS_MIN_TTC_S = 2.75        # s; vision may only START an episode with >= this much time to the
                                 #    curve (driver spec: 2.5-3.0 s) — never begin taps at/inside it
ICBM_IN_CURVE_LAT = C.CURVE_LAT_ACCEL_EXIT   # m/s^2 (1.3); measured-now |lat accel| above the CES
                                             #    curve-exit hysteresis = still loaded in a curve
ICBM_APEX_PASS_TTA_S = 1.0       # s; the binding candidate cleared within MARGIN + v*this of us =>
                                 #    we PASSED it (apex behind) — not a detection dropout
ICBM_RESTORE_DELAY_FAST_S = 1.0  # s; early-restore debounce when the curve is provably behind us
                                 #    (drive-out promptly instead of the full 3 s silent hold)
ICBM_HOLD_MAX_S = 10.0           # s; still lateral-loaded this long after the cap cleared with no
                                 #    re-bind -> give up the restore silently (no stale ceiling latch)
ICBM_LATE_TAP_GRACE_S = 1.5      # s after going silent in which the executor's final in-flight tap
                                 #    may still land on the reported set (~1 s report lag + margin)
ICBM_LATE_TAP_TOL = 1.6 * ICBM_EXEC_STEP_MS  # one full late tap + the executor deadband

# --- icbmtrack2pnw (driver-approved follow-up; field event 2026-07-12 19:58:31-59Z) -----------------
# Continuous curve-profile SET-TRACKING for MAP candidates. Driver design, verbatim intent: "adjust
# the target even when I'm behind a slow car; if I'm slower anyway it has no impact; if I go faster
# it slows me; and it gives great debugging in traffic." The field event: following a lead at ~70
# with set 90, a rated map curve 300 m ahead; driver changed lanes, the lead vanished and the stock
# ACC accelerated 72->89 INTO the curve — by then the start was correctly in-curve-suppressed and
# the driver tapped down manually. With tracking, the set would already have been walked down to the
# curve apex while still lead-bound, so losing the lead could only accelerate TO THE APEX.
#   - MAP/FAR candidates: a binding-RATED curve (effective apex < ref - MIN_DROP) starts the cap
#     episode when within the TRACKING WINDOW even if the v_ego decel envelope does not bind yet
#     (drop the v_ego brake-distance precondition; the window is sized so the executor can walk the
#     set down at tap cadence before the curve, computed at the WORST-CASE travel speed = ref, i.e.
#     the speed the ACC would reach if the lead vanished — exactly the protection case).
#   - VISION candidates keep ALL existing stricter gates (short horizon; in-curve / too-late / map-
#     first suppression unchanged). Restore machinery and episode/ceiling semantics unchanged —
#     this only changes WHEN a map cap may start, not what it does. Cap phases have NO expiry (only
#     the RESTORE phase carries the 45 s window), so long lead-bound tracking episodes are safe by
#     construction; the executor governor/stale-stop cadence is episode-length-agnostic.
ICBM_TRACK_MARGIN_S = 4.0     # s of slack beyond the pure tap-walk time (publish latency + set-report lag)
ICBM_TRACK_MAX_M = 350.0      # m hard cap on the tracking window — bounds exit/route-divergence
                              #   tracking (mapd re-matches the path after a divergence and the
                              #   candidates are re-derived from CURRENT GPS every tick, so a wrong
                              #   walk-down self-heals: curve clears -> guarded restore to ceiling)
# 19:58:37Z ROOT CAUSE (the "missing bind"): ICBM map candidates used the raw tiered_map_scale, whose
# SWEEPER end (raw >= 29 m/s -> x1.8, calibrated for VTSC/MTSC where binding causes real braking)
# inflated the event's raw 64.9 mph curve to an effective 107 mph — "not binding vs set 90" was
# computed CORRECTLY on an absurd target, so no cap and no gate ever showed. For ICBM (reduce-only
# SET-walking, no braking below the walked set) cap the scale at the tiered ramp's TIGHT end
# (MAP_SCALE_MIN, 1.35): field-calibrated against BOTH 2026-07-12 legs — 19:58 curve raw 64.9 ->
# eff 80.6 mph (driver manually chose 80-82 there) => binds at set 90; morning sweepers raw 70.9 at
# set 85 -> eff 88 => still silent (the morning over-slow complaint stays fixed). VTSC/MTSC/CES
# classification keep the full tiered scale — this cap is ICBM-only.
#
# icbmcurve2pnw (2026-08-11, docs/ICBM-CURVE-LATE.md Root Cause A / "first cut"): the flat 1.35 cap
# above was itself too COARSE — it applied the SAME ~35% inflation to every map curve, tight or
# moderate, not just sweepers. Field replay of a 2026-08-10 drive found a genuine ~50 mph-rated curve
# (131 m out, 55 mph cruise) that the flat cap inflated to an effective ~62 mph (50 * 1.35 * 0.92
# Lightning map_scale discount = 1.242 net) — ABOVE the 55 mph cruise, so the reduce-only candidacy
# test (icbm_curve_target/_icbm_binding_apex: "eff >= ref - ICBM_MIN_DROP_MS -> reject") discarded the
# candidate outright, before distance/window logic ever ran. All 131 m of published map lead time went
# unused — not a late trigger, a non-trigger. Driver direction (2026-08-10 follow-up): the curve
# TARGET should approximate the real physics limit v = sqrt(a_lat_limit / kappa), not a padded number
# — no separate safety-margin term on top. The fix below restores a genuine TWO-POINT ramp (same
# tight->sweeper SHAPE as the shared tiered_map_scale, per ICBM-CURVE-LATE.md Sec 7.A/7.E) instead of
# a flat cap, but stays ICBM-ONLY — it does NOT modify vtsc_pnw.MAP_SCALE_MIN/MAP_SPEED_SCALE/
# tiered_map_scale, so VTSC/MTSC/Tesla are byte-unchanged by this function:
#   - raw <= ICBM_MAP_SCALE_LO_MPH (50 mph): ICBM_MAP_SCALE_MIN (1.10) — NEAR-RAW. A small, documented
#     correction for mapd/GPS curvature-estimate noise (within the doc's Option-1-recommended
#     1.05-1.1x range), not a padding margin. This is what lets a genuine moderate-cut curve (the
#     50 mph / 55 mph field event) qualify as a candidate AND be tracked/targeted close to its real
#     rating instead of being inflated past the driver's cruise speed.
#   - raw >= ICBM_MAP_SCALE_HI_MPH (60 mph): ICBM_MAP_EFF_SCALE_CAP, still == MAP_SCALE_MIN (1.35),
#     UNCHANGED from the flat cap this replaces. The breakpoint is deliberately kept BELOW both
#     2026-07-12 field-calibrated events so their outcomes stay byte-identical: 64.9 mph raw
#     (29.02 m/s) and 70.9 mph raw (31.7 m/s) both sit above 60 mph -> flat 1.35, exactly as before
#     (64.9 -> eff 80.6 mph, binds at set 90; 70.9 -> eff 88 mph, stays silent at set 85).
#   - linear between 50-60 mph raw.
# Net effect vs. the flat cap: tight/moderate curves (<=50 mph raw) now track close to raw (both
# candidacy AND the published target use this SAME eff value — there is no separate candidacy-only
# scale in this first cut, per ICBM-CURVE-LATE.md Sec 7.A Option 1's "smallest diff, easiest to
# review" recommendation); both field-calibrated sweeper/binding events are unchanged; nothing at or
# above 60 mph raw changes at all. CONSERVATIVE FIRST CUT — pending on-road validation against the new
# steerlimit-log2pnw telemetry (docs/STEERING-LIMITS.md); iterate the two MPH breakpoints and
# ICBM_MAP_SCALE_MIN from there, not by guessing further from a desk analysis.
ICBM_MAP_EFF_SCALE_CAP = MAP_SCALE_MIN            # 1.35 — UNCHANGED, still the sweeper-end ceiling
ICBM_MAP_SCALE_MIN = 1.10                         # near-raw floor for tight/moderate curves (<= LO)
ICBM_MAP_SCALE_LO_MPH = 50.0 * CV.MPH_TO_MS       # ~22.35 m/s — ramp start (near-raw at/below this)
ICBM_MAP_SCALE_HI_MPH = 60.0 * CV.MPH_TO_MS       # ~26.82 m/s — ramp end (flat 1.35 at/above this;
                                                   #   below both 64.9/70.9 mph field-calibrated events)


def icbm_map_eff_scale(tv_raw: float) -> float:
  """icbmtrack2pnw + icbmcurve2pnw: the ICBM-only effective scale for a RAW map target speed (m/s).
  NOT the shared vtsc_pnw.tiered_map_scale (VTSC/MTSC/Tesla are untouched by this function) — a
  two-point linear ramp from ICBM_MAP_SCALE_MIN (near-raw, tight/moderate curves) up to
  ICBM_MAP_EFF_SCALE_CAP (1.35, unchanged sweeper-end cap) between ICBM_MAP_SCALE_LO_MPH and
  ICBM_MAP_SCALE_HI_MPH. See the icbmcurve2pnw root-cause note above. Pure."""
  if tv_raw <= ICBM_MAP_SCALE_LO_MPH:
    return ICBM_MAP_SCALE_MIN
  if tv_raw >= ICBM_MAP_SCALE_HI_MPH:
    return ICBM_MAP_EFF_SCALE_CAP
  frac = (tv_raw - ICBM_MAP_SCALE_LO_MPH) / (ICBM_MAP_SCALE_HI_MPH - ICBM_MAP_SCALE_LO_MPH)
  return ICBM_MAP_SCALE_MIN + (ICBM_MAP_EFF_SCALE_CAP - ICBM_MAP_SCALE_MIN) * frac


def icbm_track_window_m(v_ego, ref, apex_eff) -> float:
  """icbmtrack2pnw: how far ahead a binding-RATED map curve may START the set walk-down. Sized from
  what the executor physically needs: one tap per ICBM_TAP_PERIOD_S walks the set (ref - apex_eff)
  down in steps, plus margin — converted to distance at the WORST-CASE travel speed max(ref, v_ego)
  (after a lead vanishes the ACC accelerates toward ref). Capped at ICBM_TRACK_MAX_M. The window
  self-scales: small set-to-apex drops or low set speeds give short windows (no premature city
  tracking); big highway drops use the full cap. Pure; never negative."""
  try:
    steps = max((float(ref) - float(apex_eff)) / ICBM_EXEC_STEP_MS, 0.0)
    t = steps * ICBM_TAP_PERIOD_S + ICBM_TRACK_MARGIN_S
    return min(max(float(ref), float(v_ego), 0.0) * t, ICBM_TRACK_MAX_M)
  except (TypeError, ValueError):
    return 0.0


def _icbm_track_apex(v_ego, ref, eff, dist):
  """icbmtrack2pnw: TRACKING qualification for a MAP candidate that the v_ego decel envelope does
  not (yet) bind: reduce-only rated (eff meaningfully below ref) AND within the tracking window ->
  the apex speed to walk the set toward. Else None. Same reduce-only/apex semantics as
  _icbm_binding_apex — only the start precondition differs. Pure."""
  if eff is None or dist == float('inf') or eff >= ref - ICBM_MIN_DROP_MS:
    return None
  if dist <= icbm_track_window_m(v_ego, ref, eff):
    return max(eff, 0.0)
  return None


# --- curvelead2pnw (driver request #5, drives/2026-09-12/central-oregon-weekend/DRIVE_REPORT.md) -----
# A. LEAD-PACED RELAXATION. "If there's a lead car that takes a curve at a certain speed, why don't you
#    just follow that instead of making up your own mind." Stock ACC already follows the lead; what ICBM
#    must stop doing is tapping the SET below the lead's speed. So ICBM's cap may rise toward the lead's
#    speed -- never above what the truck can take on the TIGHTER of map geometry and vision, never above
#    the driver's set -- and falls back to ICBM's own target the tick the lead is not tracked.
#
#    The safety argument is NOT "the lead is not suicidal". It is: the relaxed set is min(lead speed,
#    truck bound), so if the lead vanishes mid-curve the stock ACC resumes at about the lead's last speed,
#    and that speed was already checked against this truck's bound on measured curvature.
#
#    THE CASE THE GATES EXIST FOR -- Sun 2026-09-13 13:57:00-09 PT, stock ACC on: a lead tracked for 24+ s
#    toward an R~36 m ramp ICBM had correctly rated 18-24 mph; the polyline read that ramp as R 1-10 km
#    (point-match 56-125 m off mapd's point) and the model still predicted a straight road at 165 m; the
#    lead vanished at 13:57:09 as the ramp began. Replay (drives/.../curvelead_replay.py): each of the
#    three gates ICBM_LEAD_MIN_OWN_MS, ICBM_KAT_GAP_MAX_M and ICBM_VIS_TRUST_S refuses it ON ITS OWN;
#    with all three removed, lead pacing raises 13:57:00/01/03 to 39/36/34 mph, which the truck measured at
#    5.7-6.5 m/s^2 on that ramp. With them, 0 of the 14 raised ticks exceed 3.0 m/s^2.
#
# B. MAP-CLAIM SANITY. TELEMETRY ONLY -- see icbm_map_sanity / icbm_path_behind for why neither is wired.
ICBM_LEAD_CONT_S = 3.0      # s of UNINTERRUPTED tracking before a lead may pace ICBM. Weekend corpus:
                            #   57 of 118 lead runs lasted <= 2 s (flicker / brief acquisitions) but they
                            #   hold only 72 of 2,586 lead ticks (2.8%) -- 3 s drops half the runs for <3%
                            #   of real tracking time.
ICBM_LEAD_MAX_GAP_S = 1.0   # s between brain ticks (~0.25 s nominal) beyond which nobody was watching
ICBM_LEAD_JUMP_M = 8.0      # m of dRel change per brain tick that the lead's own relative speed cannot
                            #   explain = a different car (cut-in, or re-acquisition after a loss in a curve).
                            #   PnwVehicle's tight-follow gate calls a raw 8 m dRel step at the planner rate
                            #   a different car; this one is prediction-compensated over the ~0.25 s tick.
ICBM_LEAD_MIN_OWN_MS = 25.0 * CV.MPH_TO_MS   # never relax a curve ICBM itself rates below 25 mph. Tight,
                            #   slow curves are where the lead gets lost (tracked through 1 of 4 close-lead
                            #   tight curves in the weekend corpus) and where a raised set cannot be walked
                            #   back down in time (13:57, above).
ICBM_KAT_GAP_MAX_M = 30.0   # map geometry counts as MEASURED at the candidate only if the nearest
                            #   spacing-gated triplet sits within this of mapd's point (13:57: 56-125 m).
ICBM_VIS_TRUST_S = 8.0      # s of model horizon inside which vision counts as having MEASURED the curve --
                            #   the horizon VTSC already trusts the model path to (vtsc_constants
                            #   LOOKAHEAD_MAX_S), kept as its own constant so a VTSC retune cannot move an
                            #   ICBM safety gate. Model reach alone is not enough: at 13:57:00 it reached
                            #   166 m, covered the 165 m candidate (10 s out), and predicted a straight road.
ICBM_VIS_VX_MIN = 1.0       # m/s floor under the model's planned speed in |yaw rate| / speed
ICBM_LEAD_LOG_S = 5.0       # rule-2: throttle refusal-reason logs; engage/disengage edges always log
ICBM_SANE_DROP_MS = 10.0 * CV.MPH_TO_MS   # B: "a large map-driven drop"
ICBM_SANE_RATIO = 1.5       # B: geometry+vision allow >= 1.5x ICBM's speed = "much gentler"
ICBM_PATH_MAX_PERP_M = 40.0  # B-behind: farther than this from mapd's path -> the path is not our road
ICBM_PATH_AMBIG_M = 10.0     # B-behind: segments within this of the nearest are equally plausible positions
ICBM_PATH_BEHIND_TOL_M = 15.0  # B-behind: a point counts as behind only this far back along the path
ICBM_PATH_MATCH_TOL_M = 0.5    # B-behind: candidate distances are the same haversine on the same points
_ICBM_PATH_MAX_POINTS = 256    # B-behind: bound the scan; the polyline is untrusted input (as in vtsc_pnw)


def _round_or_none(x, nd):
  """Telemetry: round a number, pass None/garbage through as None (never raises)."""
  try:
    return None if x is None else round(float(x), nd)
  except (TypeError, ValueError):
    return None


class IcbmLeadTrack:
  """curvelead2pnw: is ONE lead car being tracked without interruption? Fed once per ICBM brain tick.
  The clock restarts on: no lead; a gap between ticks longer than ICBM_LEAD_MAX_GAP_S (ICBM was silent,
  so nobody was watching); or a dRel jump the lead's own relative speed cannot explain (a different car).
  Pure; never raises."""

  def __init__(self):
    self._t0 = None
    self._prev = None

  def reset(self) -> None:
    self._t0 = None
    self._prev = None

  def update(self, now, has_lead, d_rel, v_lead, v_ego) -> tuple:
    """Returns (seconds tracked continuously, why): why is "ok", or the reason the clock just restarted
    ("noLead" / "gap" / "jump")."""
    try:
      now, d, vl, ve = float(now), float(d_rel), float(v_lead), float(v_ego)
    except (TypeError, ValueError):
      self.reset()
      return 0.0, "noLead"
    if not has_lead or not all(math.isfinite(x) for x in (now, d, vl, ve)) or d <= 0.0:
      self.reset()
      return 0.0, "noLead"
    why = "ok"
    if self._prev is not None:
      t_p, d_p, vl_p, ve_p = self._prev
      dt = now - t_p
      if dt <= 0.0 or dt > ICBM_LEAD_MAX_GAP_S:
        self._t0, why = None, "gap"
      elif abs(d - (d_p + (vl_p - ve_p) * dt)) > ICBM_LEAD_JUMP_M:
        self._t0, why = None, "jump"
    if self._t0 is None:
      self._t0 = now
    self._prev = (now, d, vl, ve)
    return now - self._t0, why


def icbm_vision_curvature(orientation_rate_z, velocity_x, position_x):
  """curvelead2pnw: the TIGHTEST curvature the driving model predicts anywhere on its horizon, and how far
  that horizon reaches. curvature_i = |yaw rate_i| / planned speed_i, the per-point measure
  vtsc_pnw.curvatures_from_model uses -- NOT predicted lateral accel / v_ego^2: the model slows for the
  curve it sees, so dividing by the CURRENT speed under-reads exactly the curves that matter.
  Returns (k 1/m, reach m), or (None, 0.0) when unusable. None means NO relaxation, never "straight".
  Pure; never raises."""
  try:
    n = min(len(orientation_rate_z), len(velocity_x), len(position_x))
    if n < 2:
      return None, 0.0
    k = 0.0
    for i in range(n):
      z, v = float(orientation_rate_z[i]), float(velocity_x[i])
      if not (math.isfinite(z) and math.isfinite(v)):
        return None, 0.0
      k = max(k, abs(z) / max(v, ICBM_VIS_VX_MIN))
    reach = float(position_x[n - 1])
    if not math.isfinite(reach) or reach <= 0.0:
      return None, 0.0
    return k, reach
  except (TypeError, ValueError):
    return None, 0.0


def icbm_lead_pace(own, ref, v_ego, src, cand_dist, lead_v, lead_cont_s, lead_why,
                   k_max, k_max_n, k_at, k_at_n, k_at_gap, vis_k, vis_reach, a_lat, rain_ms=0.0):
  """curvelead2pnw (A): the cap ICBM may raise its own curve target `own` to because a tracked lead is
  taking the curve. Returns (target or None, why, k_tight). target is always > own and <= ref; None keeps
  ICBM's own target. why names the gate that decided ("ok" when relaxed).

    pace = min(lead speed, sqrt(a_lat / k_tight) - rain, ref)
    k_tight = max(map polyline horizon max, point-matched map curvature, vision curvature)

  Every input that is missing, unmeasurable or implausible REFUSES (None) -- the conservative direction
  is ICBM's own target. Gates, in order: a_lat > 0 (capability) / lead tracked >= ICBM_LEAD_CONT_S /
  own >= ICBM_LEAD_MIN_OWN_MS / polyline measurable / map candidates point-matched within
  ICBM_KAT_GAP_MAX_M / vision present and the candidate inside ICBM_VIS_TRUST_S of model horizon / the
  pace actually above own. Pure; never raises."""
  if own is None:
    return None, "noTarget", None
  # No `x or 0.0` defaults anywhere below: a missing curvature is not a straight road and a missing gap is not
  # a perfect match. Each missing input refuses under its own name (the first version read k_max=None as 0).
  try:
    own, ref, v_ego, a_lat = float(own), float(ref), float(v_ego), float(a_lat)
    lead_v, lead_cont_s, cand_dist, rain_ms = float(lead_v), float(lead_cont_s), float(cand_dist), float(rain_ms)
  except (TypeError, ValueError):
    return None, "badInput", None
  if not all(math.isfinite(x) for x in (own, ref, v_ego, a_lat, lead_v, lead_cont_s, cand_dist, rain_ms)):
    return None, "badInput", None
  if not (a_lat > 0.0):
    return None, "off", None
  if lead_cont_s < ICBM_LEAD_CONT_S:
    return None, (lead_why if lead_why in ("noLead", "gap", "jump") else "cont"), None
  if own < ICBM_LEAD_MIN_OWN_MS:
    return None, "tight", None
  try:
    k_map, k_max_n = float(k_max), int(k_max_n)
  except (TypeError, ValueError):
    return None, "noGeom", None
  if k_max_n <= 0 or not math.isfinite(k_map):
    return None, "noGeom", None
  if src in ("map", "far"):
    try:
      k_at, k_at_n, k_at_gap = float(k_at), int(k_at_n), float(k_at_gap)
    except (TypeError, ValueError):
      return None, "gap", None
    if k_at_n <= 0 or not (k_at_gap <= ICBM_KAT_GAP_MAX_M) or not math.isfinite(k_at):
      return None, "gap", None
    k_map = max(k_map, k_at)
  try:
    vis_k, vis_reach = float(vis_k), float(vis_reach)
  except (TypeError, ValueError):
    return None, "noVis", None
  if not (math.isfinite(vis_k) and math.isfinite(vis_reach)):
    return None, "noVis", None
  if cand_dist > min(vis_reach, v_ego * ICBM_VIS_TRUST_S):
    return None, "visReach", None
  k_tight = max(k_map, vis_k)                    # the TIGHTER reading -- never the looser
  v_bound = math.sqrt(a_lat / k_tight) if k_tight > 1e-9 else float("inf")
  pace = min(lead_v, v_bound - rain_ms, ref)
  if not (pace > own + 1e-3):
    return None, "slower", k_tight
  return pace, "ok", k_tight


def icbm_map_sanity(own, ref, v_ego, src, cand_dist, k_at, k_at_n, k_at_gap, vis_k, vis_reach, a_lat):
  """curvelead2pnw (B) -- TELEMETRY ONLY; nothing publishes this. The requested rule: stop mapd's velocity
  alone from driving a LARGE drop when mapd's own point-matched geometry AND vision both say the curve is
  much gentler. Returns (the target it would publish or None, why).

  WHY IT IS NOT WIRED (drives/2026-09-12/central-oregon-weekend/curvelead_replay.py: every map/far ICBM tick
  of the weekend, the raised target judged against the curvature the truck itself measured when it got
  there): the rule raises 86 ticks. On seven the truck measured more than 3.0 m/s^2 at the raised speed on
  real curves -- Sat 15:00:15-16 (29 -> 56/50 mph, 4.1/4.0), 15:01:06 (26 -> 59, 4.4), 15:01:31-32 (39 -> 59,
  3.4/4.5), 15:04:13-14 (37 -> 59, 5.2/5.3) -- plus three at Sat 14:51:44-46 that are a low-speed turn
  artifact of the method. Adding the polyline horizon maximum to the tighter-of still leaves Sat 15:00:15-16,
  where point-matched geometry, horizon max and vision ALL under-read the same curve ~1.6x. And the two
  cases the rule was required never to suppress are not defended by it: 12:43:54 reads gentle on both
  inputs and is spared only because mapd's point sat beyond the vision-trust horizon; 2026-09-08 19:37:47
  predates the point-matched telemetry, so there is nothing to evaluate. (Both carry the behind-the-truck
  signature of icbm_path_behind, i.e. they may not be curves ahead at all -- unproven.)
  Geometry + vision is not a safe veto on this truck. Pure; never raises."""
  if own is None or src not in ("map", "far"):
    return None, None
  try:
    own, ref, v_ego, a_lat, cand_dist = float(own), float(ref), float(v_ego), float(a_lat), float(cand_dist)
  except (TypeError, ValueError):
    return None, "badInput"
  if not all(math.isfinite(x) for x in (own, ref, v_ego, a_lat, cand_dist)):
    return None, "badInput"
  if not (a_lat > 0.0):
    return None, "off"
  if ref - own < ICBM_SANE_DROP_MS:
    return None, "small"
  try:
    k_at, k_at_n, k_at_gap = float(k_at), int(k_at_n), float(k_at_gap)
  except (TypeError, ValueError):
    return None, "gap"
  if k_at_n <= 0 or not (k_at_gap <= ICBM_KAT_GAP_MAX_M) or not math.isfinite(k_at):
    return None, "gap"
  try:
    vis_k, vis_reach = float(vis_k), float(vis_reach)
  except (TypeError, ValueError):
    return None, "noVis"
  if not (math.isfinite(vis_k) and math.isfinite(vis_reach)):
    return None, "noVis"
  if cand_dist > min(vis_reach, v_ego * ICBM_VIS_TRUST_S):
    return None, "visReach"
  k = max(k_at, vis_k)
  v_geo = math.sqrt(a_lat / k) if k > 1e-9 else float("inf")
  if v_geo < ICBM_SANE_RATIO * own:
    return None, "consistent"
  return min(ref, v_geo), "gentler"


def icbm_path_behind(points, cur_lat, cur_lon, cand_dist):
  """curvelead2pnw (B-behind) -- TELEMETRY ONLY. Is the map candidate at `cand_dist` BEHIND the truck along
  mapd's own path? True / False / None (cannot tell).

  WHY THIS EXISTS -- found while replaying B, and it may be the larger defect. mapd publishes its CURRENT
  WAY from the way's first node (pfeiferj/mapd extended_state.go setPath: every node of the current way,
  then the next ways), so points the truck has already passed stay in MapTargetVelocities, and every
  candidate distance here is an unsigned haversine. upcoming_curve and icbm_far_map_candidate can therefore
  keep offering a curve the truck has just driven. The signature is in the log: mapDist GROWING at v_ego
  while mapd's velocity stays put. Weekend corpus, map-sourced ICBM ticks: 428 receding vs 411 approaching;
  with stock ACC on, 34 receding vs 57 -- and three real tap-downs were for a curve already behind the truck (Sat 12:47:22-27 set 40 -> 28, Sun
  12:05:28-31 59 -> 52, Sun 13:55:51-52 64 -> 62). The report's 12:43:54 and 13:18:37 ramp targets, and
  the 2026-09-08 19:37:47 one, carry the same signature: mapDist rose 24 -> 135 m, 5 -> 44 m, 4 -> 22 m.

  Measured ALONG the path (projection onto the polyline, in the path's own order), not by bearing: a
  bearing test calls the far side of a loop ramp or hairpin "behind" when it is ahead. Ambiguity resolves
  toward AHEAD (the smallest plausible along-track position of the truck), and a point must sit
  ICBM_PATH_BEHIND_TOL_M back to count. Not wired: the corpus has no polylines to validate this on, so it
  ships beside mapDist, whose growth rate is the independent check. Pure. Bad points are skipped; anything
  else unexpected raises into _icbm_step's curvelead guard, which logs it and keeps ICBM's own target."""
  if not points or cur_lat is None or cur_lon is None:
    return None
  lat0, lon0, cd = float(cur_lat), float(cur_lon), float(cand_dist)
  if not all(math.isfinite(x) for x in (lat0, lon0, cd)):
    return None
  coslat = math.cos(math.radians(lat0))
  pts = []                                   # (x east m, y north m, haversine m), path order kept
  for p in points[:_ICBM_PATH_MAX_POINTS]:
    try:
      la, lo = float(p["latitude"]), float(p["longitude"])
    except (KeyError, TypeError, ValueError):
      continue
    if not (math.isfinite(la) and math.isfinite(lo)):
      continue
    pts.append(((lo - lon0) * 111320.0 * coslat, (la - lat0) * 111320.0, _haversine_m(lat0, lon0, la, lo)))
  if len(pts) < 2:
    return None
  along = [0.0]
  for i in range(1, len(pts)):
    along.append(along[-1] + math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]))
  proj = []                                  # (perpendicular m, along-track m) of the truck per segment
  for i in range(len(pts) - 1):
    ax, ay = pts[i][0], pts[i][1]
    dx, dy = pts[i + 1][0] - ax, pts[i + 1][1] - ay
    seg2 = dx * dx + dy * dy
    if seg2 <= 1e-9:
      continue
    t = min(max((-ax * dx - ay * dy) / seg2, 0.0), 1.0)
    proj.append((math.hypot(ax + t * dx, ay + t * dy), along[i] + t * math.sqrt(seg2)))
  if not proj:
    return None
  min_perp = min(pp for pp, _ in proj)
  if min_perp > ICBM_PATH_MAX_PERP_M:
    return None                              # the path does not describe the road we are on
  s_truck = min(s for pp, s in proj if pp <= min_perp + ICBM_PATH_AMBIG_M)
  matched = [along[i] for i, q in enumerate(pts) if abs(q[2] - cd) <= ICBM_PATH_MATCH_TOL_M]
  if not matched:
    return None
  return all(s < s_truck - ICBM_PATH_BEHIND_TOL_M for s in matched)


# --- behindgate2pnw ------------------------------------------------------------------------------------------
# ICBM may not START a curve slowdown for a map point the truck has already driven past. mapd publishes its current
# way from the way's first node, so a curve just driven stays in the path, and ICBM's candidate distances are
# unsigned. Weekend + 09-08 I-5 map/far starts located on OSM (the nodes mapd's path is built from): 25 were for a
# point already passed. Sun 12:05:28 (set 60 -> 51 mph) and 13:55:51 are the curve-exit shape: in-curve holds every
# start through the curve, and the moment it releases, the curve just driven starts a new episode.
#
# THE RULE: a point is passed only when two independent readings agree -- it lies more than ICBM_PASSED_TOL_M BEFORE
# the truck ALONG mapd's path (the path is in travel order), AND it lies behind the truck's heading.
# Measured on every moving 1 Hz tick, every path point within 500 m, against when the truck actually passed it
# (drives/2026-09-12/central-oregon-weekend/behindgate, truck fix, 214,877 points still ahead):
#   * heading alone called 3,879 of them passed (2,147 tight-curve points): switchbacks and loop ramps, where the road
#     turns > 90 deg before reaching the point. Distance growing faster than 0.5 v -- the same test at 120 deg: 1,204.
#   * this rule: 0 (device fix: 2, both 17-21 m BESIDE the truck at 8-11 m/s, 1.4-1.6 s before it passed them -- less
#     than ICBM's own ~1.6 s position lag). With 10 % of paths reversed, along-path alone called 840 passed and
#     along + heading 22; the reversed guard in icbm_passed_points makes it 0.
ICBM_PASSED_TOL_M = 5.0        # m along the path. ICBM's position lags the truck by ~1.6-1.8 s (gpslag2pnw keep), so a
                               # point read 5 m back is >= v*1.6 + 5 m (>= 13 m at the 5 m/s floor) physically behind.
                               # The telemetry icbm_path_behind's 15 m missed Sat 12:47:21 (43 -> 28 mph): a way switch
                               # put the curve 7.8 m back, with the truck 10 m off the way's centreline.
ICBM_PASSED_MIN_V = 5.0        # m/s: slower, the GPS heading and that lag margin are not trusted -> cannot tell
ICBM_PASSED_MAX_POINTS = 1024  # the polyline is untrusted input; the weekend's longest path had 643 points


def icbm_passed_points(points, cur_lat, cur_lon, bearing, v_ego):
  """behindgate2pnw: which of mapd's path points has the truck already driven past? -> (mask, why).

  mask is a list of bools aligned with `points` (True = passed), or None when it cannot tell, and why names the reason:
  "ok", "noPath", "slow", "noHeading", "tooMany", "offPath" (nearest segment > ICBM_PATH_MAX_PERP_M away), "reversed"
  (the path runs against the heading where the truck is: mapd's order cannot be trusted here). None never gates.

  The truck's position along the path is its projection onto the nearest segment. Another segment within
  ICBM_PATH_AMBIG_M of that as close is a DIFFERENT part of the path (a loop, an overpass) only when the path between
  the two projections is longer than the straight line through the truck allows; then the smaller along-path position
  wins (toward ahead). icbm_path_behind resolves every such segment toward ahead, which also pulls the truck back to
  the corner it has just passed when it drives beside the centreline -- the Sat 12:47:21 miss. Pure; never raises
  on bad points (skipped, never passed)."""
  if not points or cur_lat is None or cur_lon is None:
    return None, "noPath"
  if len(points) > ICBM_PASSED_MAX_POINTS:
    return None, "tooMany"
  try:
    lat0, lon0, brg, v = float(cur_lat), float(cur_lon), float(bearing), float(v_ego)
  except (TypeError, ValueError):
    return None, "noHeading"
  if not all(math.isfinite(x) for x in (lat0, lon0, brg, v)):
    return None, "noHeading"
  if v < ICBM_PASSED_MIN_V:
    return None, "slow"
  coslat = math.cos(math.radians(lat0))
  valid = []                                 # (index, x east m, y north m), path order kept
  for i, p in enumerate(points):
    try:
      la, lo = float(p["latitude"]), float(p["longitude"])
    except (KeyError, TypeError, ValueError):
      continue
    if math.isfinite(la) and math.isfinite(lo):
      valid.append((i, (lo - lon0) * 111320.0 * coslat, (la - lat0) * 111320.0))
  along, segs = {}, []                       # segs: (perpendicular m, along m of the projection, unit dx, unit dy)
  prev = None
  for i, x, y in valid:
    if prev is None:
      along[i] = 0.0
    else:
      s_prev, px, py = prev
      dx, dy = x - px, y - py
      seg = math.hypot(dx, dy)
      if seg > 1e-6:
        t = min(max((-px * dx - py * dy) / (seg * seg), 0.0), 1.0)
        segs.append((math.hypot(px + t * dx, py + t * dy), s_prev + t * seg, dx / seg, dy / seg))
      along[i] = s_prev + seg
    prev = (along[i], x, y)
  if not segs:
    return None, "noPath"
  p0, s0, ux, uy = min(segs, key=lambda q: q[0])
  if p0 > ICBM_PATH_MAX_PERP_M:
    return None, "offPath"
  hx, hy = math.sin(math.radians(brg)), math.cos(math.radians(brg))
  if ux * hx + uy * hy < 0.0:
    return None, "reversed"
  s_truck = s0
  for pp, ss, _, _ in segs:
    if pp <= p0 + ICBM_PATH_AMBIG_M and abs(ss - s0) > p0 + pp + ICBM_PATH_AMBIG_M:
      s_truck = min(s_truck, ss)
  mask = [False] * len(points)
  for i, x, y in valid:
    mask[i] = along[i] < s_truck - ICBM_PASSED_TOL_M and x * hx + y * hy < 0.0
  return mask, "ok"


def _icbm_passed_log(ctl, state, **kw) -> None:
  """behindgate2pnw (Rule 2): the gate's verdict on a binding map/far candidate, logged when it CHANGES --
  "passed" (it acted), "clear" (the binding point is ahead), "unknown" (it could not tell, with why). A module
  function taking the controller for the same reason as _curvelead_clear. behindrun2pnw: a verdict on a RUNNING
  episode carries phase="run" (and `started` is then the source the episode continues on, None = nothing binds any
  more, so the restore may begin); a START verdict has no phase key, so the two never collapse into one another."""
  key = (state, kw.get("why"), kw.get("started"), kw.get("phase"))
  if getattr(ctl, "_icbm_passed_state", None) != key:
    ctl._icbm_passed_state = key
    cloudlog.event("ces_icbm_passed", state=state, **kw)


def _icbm_passed_gate(ctl, now, target, sig, plat, plon, ref, far_v, far_dist, vis, ceiling=None, running=False):
  """behindgate2pnw: the passed-point gate. Called whenever a map/far candidate would bind. Returns
  (target, sig, far_v, far_dist), updating ctl._icbm_src / _icbm_gate.

  The binding point was passed exactly when that source's candidate CHANGES once the passed points are removed:
  upcoming_curve keeps the first lowest point and icbm_far_map_candidate the lowest cap, so removing other points
  cannot change a pick that was not itself removed. Then the decision is re-taken on the points still ahead (another
  curve may bind instead), with `vis` as the caller's own decision used it. When the binding point is ahead, nothing
  here changes the decision. A failure falls back to the ungated decision and says so (throttled).

  running=False is the START gate (ceiling None -- no episode holds one yet, vision only where it may start).

  behindrun2pnw (owner 2026-09-14, "Behind-curve gate: yes"): running=True is the same gate on a RUNNING cap episode
  -- a passed point may neither lower its target nor keep it bound (which holds the restore back). The caller passes
  what that episode's own decision used: `ceiling` = its latched ceiling (== ref), and vision without the start rule.
  A curve still ahead keeps full authority: the target is re-decided on the points still ahead, exactly as the running
  episode would decide it had the passed points not been there. Labelled "mapPassedRun" / phase="run"."""
  src = ctl._icbm_src
  run = {"phase": "run"} if running else {}
  try:
    points = ctl._map_targets
    mask, why = icbm_passed_points(points, plat, plon, getattr(ctl, "_cur_bearing", None), sig["v_ego"])
    if mask is None:
      _icbm_passed_log(ctl, "unknown", why=why, src=src, **run)
      return target, sig, far_v, far_dist
    ahead = [p for p, gone in zip(points, mask, strict=True) if not gone]
    near = (sig.get("map_target_v", 0.0), sig.get("map_target_dist", float("inf")))
    far = (far_v, far_dist)
    veh = ctl._veh
    if src == "map":
      a_near = upcoming_curve(ahead, plat, plon, sig["v_ego"], C.CURVE_MAP_LOOKAHEAD_S)
      if a_near == near:
        _icbm_passed_log(ctl, "clear", src=src, **run)
        return target, sig, far_v, far_dist
      a_far = icbm_far_map_candidate(ahead, plat, plon, sig["v_ego"], ref, icbm_map_eff_scale,
                                     veh.icbm_map_scale, veh.icbm_firm_decel)
    else:
      a_far = icbm_far_map_candidate(ahead, plat, plon, sig["v_ego"], ref, icbm_map_eff_scale,
                                     veh.icbm_map_scale, veh.icbm_firm_decel)
      if a_far == far:
        _icbm_passed_log(ctl, "clear", src=src, **run)
        return target, sig, far_v, far_dist
      a_near = upcoming_curve(ahead, plat, plon, sig["v_ego"], C.CURVE_MAP_LOOKAHEAD_S)
    new_sig = {**sig, "map_target_v": a_near[0], "map_target_dist": a_near[1]}
    new_target, _, new_src = icbm_curve_target(
      sig["v_ego"], sig["v_set"], a_near[0], a_near[1], ceiling, icbm_map_eff_scale, vis[0], vis[1],
      map_scale=veh.icbm_map_scale, firm_decel=veh.icbm_firm_decel, far_v=a_far[0], far_dist=a_far[1], track=True)
  except Exception:
    try:
      if now - (getattr(ctl, "_icbm_passed_err", None) or -1e9) > ICBM_ERR_LOG_S:
        ctl._icbm_passed_err = now
        cloudlog.exception("behindgate2pnw: passed-point gate FAILED -- " +
                           ("a RUNNING ICBM slowdown continues WITHOUT it (a passed curve can deepen it and hold the restore)"
                            if running else "ICBM starts WITHOUT it (a passed curve can start a slowdown)"))
    except Exception:
      pass                        # logging must not become the thing that raises
    return target, sig, far_v, far_dist
  ctl._icbm_gate = "mapPassedRun" if running else "mapPassed"
  ctl._icbm_src = new_src
  _icbm_passed_log(ctl, "passed", src=src, dist=round(float(near[1] if src == "map" else far[1]), 1),
                   passed=sum(mask), started=new_src, **run)
  return new_target, new_sig, a_far[0], a_far[1]


CURVELEAD_TELE_KEYS = ("icbmOwnT", "icbmLeadT", "icbmLeadWhy", "icbmLeadS", "icbmKVis",
                       "icbmSaneT", "icbmSaneWhy", "icbmBehind")


def _curvelead_tele(ctl) -> dict:
  """curvelead2pnw: the telemetry fragment for BOTH the CESStatus overlay feed and the ces_events record
  (one builder, so the two cannot drift). Every key in CURVELEAD_TELE_KEYS, always present; a missing or
  never-set attribute reads None. Never raises."""
  g = lambda n: getattr(ctl, n, None)   # noqa: E731
  behind = g("_icbm_behind")
  why = g("_icbm_lead_why")
  sane_why = g("_icbm_sane_why")
  return {
    "icbmOwnT": _round_or_none(g("_icbm_own_t"), 2),
    "icbmLeadT": _round_or_none(g("_icbm_lead_t"), 2),
    "icbmLeadWhy": why if isinstance(why, str) else None,
    "icbmLeadS": _round_or_none(g("_icbm_lead_s"), 1),
    "icbmKVis": _round_or_none(g("_icbm_kvis"), 5),
    "icbmSaneT": _round_or_none(g("_icbm_sane_t"), 2),
    "icbmSaneWhy": sane_why if isinstance(sane_why, str) else None,
    "icbmBehind": behind if isinstance(behind, bool) else None,
  }


def _curvelead_clear(ctl) -> None:
  """curvelead2pnw: ICBM inactive (forced Chill / no data) -> no stale telemetry, and the lead clock
  restarts. A module function taking the controller, not a method: the per-attribute controller stubs in
  the ICBM tests bind _icbm_step onto a bare object, and a missing method there would be swallowed by
  _icbm_step's except as a silent ICBM outage."""
  trk = getattr(ctl, "_icbm_lead_trk", None)
  if trk is not None:
    trk.reset()
  ctl._icbm_own_t = ctl._icbm_lead_t = ctl._icbm_lead_why = ctl._icbm_lead_s = None
  ctl._icbm_kvis = ctl._icbm_sane_t = ctl._icbm_sane_why = ctl._icbm_behind = None


def _curvelead_failed(ctl, now, own, e) -> None:
  """curvelead2pnw: lead pacing / sanity telemetry raised. ICBM has already fallen back to its own target;
  mark the record (icbmLeadWhy "error") and log -- throttled, it would otherwise repeat at ~4 Hz.
  silentexc2pnw: the line names the exception type `e` and counts the failures since the previous line; the first
  failure logs at once, then at most one line per CURVELEAD_ERR_LOG_S. getattr defaults, because the ICBM tests bind
  _icbm_step onto bare stubs and this runs on the failure path."""
  try:
    ctl._icbm_own_t, ctl._icbm_lead_t, ctl._icbm_lead_why = own, None, "error"
    ctl._icbm_sane_t = ctl._icbm_sane_why = ctl._icbm_behind = None
    ctl._icbm_lead_log = (False, "error", now)
    ctl._icbm_lead_fail_n = (getattr(ctl, "_icbm_lead_fail_n", 0) or 0) + 1
    last = getattr(ctl, "_icbm_lead_fail_log", None)
    if last is None or now - last >= CURVELEAD_ERR_LOG_S:
      n, ctl._icbm_lead_fail_log, ctl._icbm_lead_fail_n = ctl._icbm_lead_fail_n, now, 0
      cloudlog.exception(f"curvelead2pnw: lead pacing FAILED ({type(e).__name__}) -- ICBM is using its own curve " +
                         f"target (no lead pacing, no B telemetry) ({n} failure(s) since the last log)")
  except Exception:
    pass                        # the fallback itself already happened in the caller; logging must not raise


def _curvelead_note(ctl, now, own, pace, why, lead_s, lead_v, k_tight, vis_k, sane_t, sane_why, behind) -> None:
  """curvelead2pnw: store this tick's lead-pace / sanity telemetry on the controller and LOG the moments
  that matter (rule 2): lead pacing engaging and ending always; a change in WHY it is refused while ICBM has
  a target, throttled to ICBM_LEAD_LOG_S; and the two telemetry-only B verdicts turning on or off. Never
  raises -- logging must not become the thing that breaks ICBM."""
  ctl._icbm_own_t = own
  ctl._icbm_lead_t = pace
  ctl._icbm_lead_why = None if own is None else why
  ctl._icbm_lead_s = lead_s
  ctl._icbm_kvis = vis_k
  ctl._icbm_sane_t = sane_t
  ctl._icbm_sane_why = sane_why
  ctl._icbm_behind = behind
  mph = CV.MS_TO_MPH
  try:
    engaged = pace is not None
    was, last_why, last_t = getattr(ctl, "_icbm_lead_log", None) or (False, None, -1e9)
    if engaged != was:
      if engaged:
        cloudlog.info("curvelead2pnw: lead pacing ENGAGED -- ICBM curve target %.1f -> %.1f mph " +
                      "(lead %.1f mph, tracked %.1f s, tightest curvature %.5f 1/m)",
                      own * mph, pace * mph, float(lead_v) * mph, float(lead_s), float(k_tight))
      else:
        cloudlog.info("curvelead2pnw: lead pacing ENDED (%s) -- ICBM back to its own target %s",
                      why, "none" if own is None else f"{own * mph:.1f} mph")
      ctl._icbm_lead_log = (engaged, why, now)
    elif not engaged and own is not None and why != last_why and now - last_t >= ICBM_LEAD_LOG_S:
      cloudlog.info("curvelead2pnw: lead pacing refused (%s) -- ICBM target %.1f mph stands", why, own * mph)
      ctl._icbm_lead_log = (engaged, why, now)
    b_now = (sane_t is not None, behind is True)
    if b_now != (getattr(ctl, "_icbm_b_log", None) or (False, False)):
      if b_now[0]:
        cloudlog.info("curvelead2pnw: map-claim sanity WOULD raise ICBM %.1f -> %.1f mph -- TELEMETRY ONLY, not applied",
                      own * mph, sane_t * mph)
      if b_now[1]:
        cloudlog.info("curvelead2pnw: ICBM curve target %.1f mph is for a map point BEHIND the truck -- TELEMETRY ONLY, not applied",
                      own * mph)
      ctl._icbm_b_log = b_now
  except Exception:
    # a failed log line must never take ICBM down with it -- but a logger that ALWAYS fails would leave
    # these edges invisible for the whole drive, so say so (throttled, it runs at ~4 Hz).
    try:
      if now - (getattr(ctl, "_icbm_lead_log_err", None) or -1e9) > ICBM_ERR_LOG_S:
        ctl._icbm_lead_log_err = now
        cloudlog.exception("curvelead2pnw: telemetry logging FAILED -- lead-pace / sanity edges are not being logged")
    except Exception:
      pass


# --- icbmratchet2pnw (root-cause fix, field event 2026-08-10: computed ~31 mph target / ~13.86 m/s,
# ~15 mph delivered) --------------------------------------------------------------------------------
# ROOT CAUSE: IcbmEpisode._min_target only ever ratchets DOWN for the life of a cap episode
# (min(_min_target, cap_target) every tick), so a single transient low tick — e.g. a vision-
# curvature reading (orientationRate.z / icbm_vision_apex) distorted for one ~0.25 s brain tick by
# the truck being momentarily off-line/understeering under the ISO 3.0 clip — was treated exactly
# like a genuine, sustained curve reading and floored the whole episode at that outlier value. The
# VERY NEXT tick's recomputed (correct, ~31 mph) target could never undo it: caps are DEC-only by
# design (icbm_curve_target never commands an increase), and the only path back up is the bounded,
# guarded RESTORE — which only fires once the curve clears ENTIRELY, all the way to the driver's
# original pre-curve ceiling, never to an intermediate "that reading was noise, the real target is
# 31" value. So one bad tick permanently pinned the truck at the outlier speed for the rest of the
# curve.
#
# FIX: a tick that wants to drop the ratchet by more than ICBM_RATCHET_OUTLIER_DROP_MS below the
# LAST CONFIRMED target must PERSIST for ICBM_RATCHET_CONFIRM_S before it is adopted (updates
# _min_target) or published. While a drop is pending confirmation, the episode keeps commanding the
# last CONFIRMED target — still braking normally for whatever curve was already recognized, never
# silently going fully hands-off — just not lurching to the unconfirmed outlier. A tick that
# recovers back within the outlier band of the last confirmed target cancels the pending candidate
# outright: proven transient, never adopted, never published, never tapped toward.
#
# Only large SINGLE-tick drops are gated — ordinary continuous refinement (a curve's estimate
# tightening smoothly tick to tick as distance/geometry closes) is always small and passes straight
# through with no delay, so genuine sustained curves brake exactly as before. A drop this large
# across one ~0.25 s tick (icbm_curve_target/_icbm_step run at ~4 Hz, matching IcbmEpisode.step's
# docstring) is not something real road geometry produces — a candidate curve's RATED speed does
# not change tick to tick, only whether/when it starts binding — so this is a deliberately
# conservative "obviously not a real curve reading" gate, not a general smoothing filter that would
# blunt a genuine hard brake. See IcbmEpisode._ratchet_confirm for the mechanism.
ICBM_RATCHET_OUTLIER_DROP_MS = 3.0   # m/s (~6.7 mph). Defined directly in m/s, NOT as a multiple of
                                      # ICBM_EXEC_STEP_MS (that constant is itself already 1 mph in
                                      # m/s — an earlier draft of this fix wrote "3.0 * ICBM_EXEC_STEP_MS"
                                      # intending 3 m/s and got 3 mph instead, a Fable review catch).
                                      # The field glitch was a 16 mph (7.2 m/s) one-tick drop (31->15),
                                      # far above this — comfortably gates it while staying well clear
                                      # of ordinary tick-to-tick source/tiering jitter (a few mph at
                                      # most).
ICBM_RATCHET_CONFIRM_S = 0.6                             # s; ~2-3 ticks at 4 Hz. Worst-case added
                                                          # travel before a genuine large drop is
                                                          # honored: v_ego * this (e.g. ~24 m at
                                                          # 90 mph) — a small fraction of the existing
                                                          # ICBM_MARGIN_M(30 m) start-early buffer and
                                                          # the (typically hundreds-of-metres) comfort-
                                                          # decel envelope; negligible against the many
                                                          # taps (ICBM_TAP_PERIOD_S=0.4 s each) a real
                                                          # multi-mph slowdown needs anyway.


# satele2pnw: every key SpeedAdjustController._publish_status() emits, forwarded into the ces_events tick
# record as "sa<Key>". A module constant so a test can assert publisher keys == forwarded keys against
# the REAL published dict -- a key published to /dev/shm but missing here silently evaporates.
SA_TELE_KEYS = ("mode", "sl", "slRef", "ratio", "cap", "out", "vSet", "vCruise", "lastSet",
                "ovr", "eng", "polLatch", "polSupp", "polKey", "epLim", "noRst", "zoneTgt", "zoneN", "zoneLast",
                "icbmHold", "inst")


def icbm_note_speedadjust(ep, sa_tele, limit_now) -> None:
  """sazoneset2pnw: fold this tick's posted limit and speedadjust's forwarded status (`sa_tele`, the "sa"-prefixed
  SA_TELE_KEYS dict) into the episode's restore bound. The ONE wiring point, shared by _icbm_step and the closed-loop
  tests so they cannot drift apart.
  "Proportional" = the driver has zone speeds on (speedadjust AutoSpeedReduce >= 2). A missing/unreadable mode is not
  evidence it is on: it falls back to limit + 5."""
  sa_tele = sa_tele or {}
  try:
    proportional = sa_tele.get("saMode") is not None and int(sa_tele.get("saMode")) >= 2
  except (TypeError, ValueError):
    proportional = False
  ep.note_limit(limit_now if limit_now is not None and limit_now > 0.0 else None, proportional)
  ep.note_sa_zone(sa_tele.get("saZoneTgt"), sa_tele.get("saZoneN"), sa_tele.get("saZoneLast"), limit_now,
                  sa_tele.get("saInst"))


class IcbmEpisode:
  """Cap -> clear -> RESTORE -> done state machine (pure, unit-tested; owned by CESController).

  DEC remains the rule for CAPS. The restore phase ONLY returns the stock set speed to the driver's
  OWN latched set (the ceiling ICBM itself latched when the first cap of the episode engaged) —
  restore ONLY what ICBM took, never fight the driver:
    - the ceiling is latched exclusively by an ICBM cap engaging (a driver-lowered set never arms it)
    - a restore never publishes a target above the latched ceiling
    - HARD ABORTS (reset -> silent -> executor stale-stops): driver gas/brake; stock ACC off;
      window expiry (ICBM_RESTORE_WINDOW_S); reaching the ceiling; the stock set moving in a way
      our taps can't explain (any decrease, or a rise faster than the executor tap cadence);
      a NEW cap engaging — DEC ALWAYS WINS: the restore episode is cancelled entirely and a fresh
      cap episode latches at the CURRENT set (after a partial restore the new ceiling is the
      partially-restored speed, never the old higher one — conservative on purpose)
    - if the driver lowered the set below anything we commanded during the cap phase, the episode
      ends WITHOUT any restore (their intent, not ours).
  The executor enforces its own independent envelope on top (ceiling clamp, stale heartbeat,
  cruise/override gates, RestoreGuard human-detection latch)."""

  def __init__(self, window_s: float = ICBM_RESTORE_WINDOW_S, clear_delay_s: float = ICBM_RESTORE_DELAY_S):
    self.latch_limit = None             # sazoneset2pnw (also cleared in reset()): limit at ceiling latch
    self.zone_cap = None                # sazoneset2pnw: STICKY stale-ceiling restore cap (m/s)
    self.zone_why = None                # sazoneset2pnw: "prop" / "limit5" / "saZone" / None
    self.zone_n0 = None                 # sazoneset2pnw (also cleared in reset()): speedadjust's zone count at latch
    self._sa_zone_n_idle = None         # ...as last read while idle; deliberately NOT cleared by reset()
    self._sa_restart_logged = False     # zonefollow2pnw (also cleared in reset())
    self._sa_inst0 = None               # zonefollow2pnw (also cleared in reset()): speedadjust's instance at latch
    self._sa_inst_idle = None           # ...as last read while idle; NOT cleared by reset()
    self._window_s = window_s
    self._clear_delay_s = clear_delay_s
    self.phase = "idle"                 # idle | cap | restore
    self.ceiling = None                 # driver's own set (m/s), latched while an episode is active
    # icbmratchet2pnw: THREE distinct trackers, deliberately not conflated (Gemini review catch,
    # round 2 — a single "_min_target that's both the outlier-gating baseline AND whatever we most
    # recently acted on" let one bad engage tick poison the baseline for the rest of the episode):
    #   _min_target      lowest CONFIRMED target — the baseline future ticks are outlier-tested
    #                    against. Only updated when a value is genuinely accepted/confirmed, NEVER
    #                    from the bare, unconfirmed engage action alone.
    #   _committed_target the value currently being published/acted on (may rise; not confirmed-only)
    #                    — used as the "held" fallback during a pending window so a genuinely
    #                    consistent (if not yet confirmed) engage doesn't flicker back to v_set.
    #   _min_published   lowest value EVER published this episode, a plain running min (mirrors the
    #                    ORIGINAL pre-fix _min_target semantics exactly) — feeds the restore-
    #                    eligibility driver-lower-guard below, which cares about anything the
    #                    executor might actually have tapped toward, confirmed or not.
    self._min_target = None
    self._committed_target = None
    self._min_published = None
    # a candidate that would drop the ratchet by more than an outlier-sized step in one tick,
    # awaiting confirmation (see _ratchet_confirm) before it is adopted/published.
    self._pending_low_target = None
    self._pending_low_t0 = None         # monotonic time the pending candidate was first seen
    # icbmratchet2pnw: when the CURRENT ceiling was (re)latched at engage (Gemini review catch) — an
    # engage that goes silent again before this proves itself for ICBM_RATCHET_CONFIRM_S is treated
    # as unconfirmed noise, not a real curve; see step()'s clear-debounce entry.
    self._engage_t0 = None
    self._t0 = None                     # restore start (monotonic)
    self._clear_t0 = None               # first tick the curve was clear while capping (debounce)
    self._hold_set0 = None              # stock set snapshot at hold entry (movement = human)
    self._last_stock = None
    self._last_t = None
    # icbmmapfirst2pnw: where the binding candidate was when it last bound — a candidate that clears
    # while CLOSE to us was PASSED (apex behind -> early restore); one that clears while far ahead
    # was a detection dropout (keep the full flicker-proof debounce).
    self._last_cap_dist = None
    self._last_cap_vego = 0.0
    self._apex_passed = False
    self._late_tap_set = None           # the ONE absorbed late-tap baseline (anchored, never walked)

  def reset(self) -> None:
    self.phase = "idle"
    self.ceiling = None
    self._min_target = None
    self._committed_target = None
    self._min_published = None
    self._pending_low_target = None
    self._pending_low_t0 = None
    self._engage_t0 = None
    self._t0 = None
    self._clear_t0 = None
    self._hold_set0 = None
    self._last_stock = None
    self._last_t = None
    self._last_cap_dist = None
    self._last_cap_vego = 0.0
    self._apex_passed = False
    self._late_tap_set = None
    self._rcap_hold_set = None          # icbmrestorecap2pnw: stock set when the posted-limit hold began
    self.latch_limit = None             # sazoneset2pnw: debounced posted limit when the ceiling latched
    self.zone_cap = None                # sazoneset2pnw: STICKY stale-ceiling restore cap (m/s), only lowered
    self.zone_why = None                # sazoneset2pnw: "prop" / "limit5" / "saZone" / None
    self.zone_n0 = None                 # sazoneset2pnw: speedadjust's zone count when this episode began
    self._sa_restart_logged = False     # zonefollow2pnw: one error per episode
    self._sa_inst0 = None               # zonefollow2pnw: speedadjust's instance when this episode began

  def _ratchet_confirm(self, now: float, cap_target: float, baseline: float) -> tuple:
    """icbmratchet2pnw: the robustness gate on the DOWNWARD ratchet. `baseline` is the reference an
    outlier-sized drop is measured against — self._min_target (the last CONFIRMED target) for an
    already-active cap episode, or the driver's own v_set for the very first tick of a fresh
    episode (there is no confirmed target yet; a Gemini/Fable review catch on the first version of
    this fix found that skipping confirmation entirely at engage let a single bad engage tick latch
    a ceiling/min_target that a subsequent immediate clear-and-reset (see step()'s clear-debounce
    handling of self._engage_t0) could not always undo before it reached the executor). Returns the
    target THIS tick should actually command/ratchet toward:
      - cap_target itself, immediately, when it is not an outlier-sized drop below `baseline` — the
        common case: flat, rising, or an ordinary small/gradual decrease. _min_target is updated
        (ratcheted down, never up; initialized on first use) exactly as before the fix.
      - the last CONFIRMED target (self._min_target, or `baseline` if nothing confirmed yet), while
        an outlier-sized drop is pending confirmation — never act on an unconfirmed low reading.
      - the pending candidate (the lowest value seen while it was pending), once an outlier-sized
        drop has persisted for >= ICBM_RATCHET_CONFIRM_S — a genuinely sustained lower target is
        adopted and _min_target ratchets down to it, same as the pre-fix behavior but proven, not
        assumed.
    A recovery back within the outlier band of `baseline` (checked every tick via the first branch)
    discards any pending candidate outright — a transient never gets a "grace" tap. A reading that
    ITSELF deviates from the currently-pending candidate by more than the outlier band (Gemini
    review catch: an extreme one-tick outlier landing inside an otherwise-legitimate pending window
    must not "hide" there and get adopted via the worst-seen min()) restarts the confirmation window
    at the new value instead of blending into the old one — noisy/unstable readings simply take
    longer to confirm, they never let one wild tick hijack a window opened by a different value.

    Returns (target, accepted): `accepted` is True only when `target` reflects THIS tick's own
    reading (immediate accept, or just-confirmed) -- False while holding at the last confirmed/
    baseline value during a pending window. The caller uses `accepted` to gate anything that should
    only trust an actually-acted-upon reading, e.g. the apex-passage distance snapshot (Fable review
    catch: an unconfirmed outlier's distance must not feed the early-restore decision)."""
    # held (published while pending): the CURRENT commitment, not the confirmed baseline -- a
    # genuinely consistent-but-unconfirmed engage must keep publishing its own value while it
    # confirms, not flicker back to `baseline` (which may be the driver's v_set, i.e. "uncapped").
    held = self._committed_target if self._committed_target is not None else baseline
    if cap_target >= baseline - ICBM_RATCHET_OUTLIER_DROP_MS:
      self._pending_low_target = None
      self._pending_low_t0 = None
      self._min_target = cap_target if self._min_target is None else min(self._min_target, cap_target)
      return cap_target, True
    # outlier-sized drop vs baseline: hold at the last confirmed (or baseline) value until this (or
    # a lower) reading persists ICBM_RATCHET_CONFIRM_S. Track the WORST (lowest) value seen among
    # readings CONSISTENT with each other during the window — conservative on purpose (never less
    # braking than warranted), consistent with the rest of the file's reduce-only bias.
    if (self._pending_low_t0 is None
        or abs(cap_target - self._pending_low_target) > ICBM_RATCHET_OUTLIER_DROP_MS):
      self._pending_low_target = cap_target
      self._pending_low_t0 = now
      return held, False
    self._pending_low_target = min(self._pending_low_target, cap_target)
    if now - self._pending_low_t0 >= ICBM_RATCHET_CONFIRM_S:
      confirmed = self._pending_low_target
      self._min_target = confirmed
      self._pending_low_target = None
      self._pending_low_t0 = None
      return confirmed, True
    return held, False

  def note_limit(self, limit_now, proportional: bool) -> None:
    """sazoneset2pnw: fold this tick's posted limit into the episode's STICKY zone cap. Call once per tick
    BEFORE step(). Only lowers self.zone_cap and never touches self.ceiling: the ceiling is the reference a
    curve's binding is judged against during the cap phase, and lowering it there could make a real curve
    stop binding mid-approach. The cap only ever limits the RESTORE (step's restore_cap)."""
    if self.phase not in ("cap", "restore") or self.ceiling is None:
      return
    cap, why = C.icbm_stale_zone_cap(self.ceiling, self.latch_limit, limit_now, proportional)
    if cap is not None and (self.zone_cap is None or cap < self.zone_cap):
      self.zone_cap, self.zone_why = cap, why

  def note_sa_zone(self, zone_tgt, zone_n, zone_last, limit_now=None, inst=None) -> None:
    """sazoneset2pnw (Fable review, measured): bound the RESTORE by speedadjust's zone speed when a zone set was in
    progress during this episode or BEGAN during it. A curve overlapping a zone entry latches the pre-zone set as
    its ceiling, and the posted limit may already read low at the latch -- so icbm_stale_zone_cap sees nothing
    stale and the restore handed back 75 mph in a 45 zone, permanently. Inputs are speedadjust's forwarded status
    (~1 Hz): zoneTgt (in progress, else None), zoneN (zone episodes opened), zoneLast (the latest one's target).
    zoneN catches a zone that opened and completed between two reads -- which happens when our own taps already
    had the set below the zone speed. Same contract as note_limit: call before step(); only lowers zone_cap;
    never touches the ceiling. An unreadable count at the start only loses the completed-zone case, logged via
    icbmZoneWhy never becoming "saZone"; an in-progress zone still binds."""
    def _pos(x):
      try:
        x = float(x)
      except (TypeError, ValueError):
        return None
      return x if math.isfinite(x) and x > 0.0 else None
    try:
      n = int(zone_n) if zone_n is not None else None
    except (TypeError, ValueError):
      n = None
    if self.phase not in ("cap", "restore") or self.ceiling is None:
      self._sa_zone_n_idle, self._sa_inst_idle = n, inst
      return
    if self.zone_n0 is None:
      self.zone_n0 = self._sa_zone_n_idle if self._sa_zone_n_idle is not None else n
    if self._sa_inst0 is None:
      self._sa_inst0 = self._sa_inst_idle if self._sa_inst_idle is not None else inst
    bound, why = _pos(zone_tgt), "saZone"
    if inst is not None and self._sa_inst0 is not None and inst != self._sa_inst0:
      # zonefollow2pnw (Fable review): speedadjust (plannerd) RESTARTED during this episode -- its zone count and last
      # target are gone, so a zone that ran just before the restart is invisible (measured: 73-75 mph in a 45 zone).
      # Unless this episode already holds a bound (a zone speed seen before the restart stays valid), fall back to the
      # limit + 5 backstop.
      lim = _pos(limit_now) if self.zone_cap is None else None
      if lim is not None and (bound is None or lim + C.ICBM_RESTORE_LIMIT_MARGIN_MS < bound):
        bound, why = lim + C.ICBM_RESTORE_LIMIT_MARGIN_MS, "saRestart"
      if not self._sa_restart_logged:
        self._sa_restart_logged = True
        cloudlog.error("icbm: speedadjust restarted mid-episode; restore capped at "
                       + (f"{bound:.2f} m/s ({why})" if bound is not None else
                          f"the bound it already had ({self.zone_why})" if self.zone_cap is not None else
                          "NOTHING -- posted limit unknown"))
    elif bound is None and n is not None and self.zone_n0 is not None and n != self.zone_n0:
      bound = _pos(zone_last)
    if bound is not None and (self.zone_cap is None or bound < self.zone_cap):
      self.zone_cap, self.zone_why = bound, why

  def _restore_target(self, stock_set, restore_cap):
    """The restore's publish for this tick: (target, "inc"), or (None, None) to HOLD silently.

    icbmrestorecap2pnw (driver report 2026-09-13 15:46 PT): restore used to return the latched
    ceiling unconditionally -- the set from BEFORE the curve -- so a curve that ended as the truck
    entered a 25 mph zone restored the set to 60 there, over 13 s, with the limit reading 25 the whole
    time. The cap bounds what "giving back what the curve took" may mean on the road the truck is now
    on. Holding (rather than ending) at the cap keeps the episode alive so a rising limit can resume
    it; the existing restore window still bounds how long that can last, so after a long low zone the
    set is simply left at the cap for the driver to raise.
    """
    target = self.ceiling
    if restore_cap is not None:
      try:
        cap = float(restore_cap)
      except (TypeError, ValueError):
        cap = None
      if cap is not None and cap == cap and cap > 0.0 and cap < target:
        target = cap
        if stock_set is not None and stock_set >= target - ICBM_RESTORE_DONE_TOL:
          if getattr(self, "_rcap_hold_set", None) is None:
            self._rcap_hold_set = stock_set   # snapshot: from here nothing of OURS moves the set
          return None, None             # AT the posted-limit cap: hold, keep the episode
    self._rcap_hold_set = None          # publishing again -> the snapshot no longer applies
    return target, "inc"

  def step(self, now, cap_target, v_set, stock_set, stock_on, driver_pedal,
           cap_dist=None, v_ego=0.0, in_curve=False, restore_cap=None, limit_now=None):
    """One brain tick (~4 Hz). All inputs SI primitives; cap_target is the (penalty-applied) cap
    from icbm_curve_target or None. Returns (publish_target or None, direction 'dec'/'inc'/None).

    icbmmapfirst2pnw optional inputs (defaults keep the pre-mapfirst behavior identical):
      cap_dist  distance (m) of the binding candidate this tick (None/inf = unknown) — feeds the
                apex-passage detection for the EARLY restore (curve provably behind -> 1 s debounce
                instead of 3 s);
      v_ego     current speed (m/s), for the apex-passage window;
      in_curve  vehicle currently lateral-loaded (icbm_in_curve): a restore may not BEGIN and a
                running restore PAUSES (silent — executor stale-stops — WITHOUT resetting, so it
                resumes when the load clears) while True. Never raise the set mid-curve. All abort
                guards stay live throughout.

    icbmrestorecap2pnw optional input:
      restore_cap  absolute m/s the restore may not raise the set above (posted limit + the driver's
                   margin), or None = no cap. The restore target becomes min(ceiling, restore_cap).
                   At the cap the restore HOLDS silently -- it does not end -- so if the limit rises
                   within the restore window it follows back up toward the ceiling, never above
                   either. The executor already clamps target <= ceiling and presses inc only while
                   the set is below the TARGET, so a target below the ceiling needs no executor change."""
    if v_set is None or v_set <= 0.0:
      self.reset()                      # no valid driver set -> hands off everything
      return None, None
    if not stock_on or driver_pedal:
      # Gemini adversarial catch (ACC off/on survival): ANY pedal press or ACC-off in ANY phase
      # kills the episode entirely — no episode may exist while the driver is braking/gassing or
      # the ACC is disengaged. A cap present right now is still FORWARDED (dec-only; the executor
      # independently gates presses on cruise/pedals), but WITHOUT an episode/latch: when
      # conditions return, the next cap tick starts a FRESH episode latched at the THEN-current
      # set — so a post-brake re-engage at a lower set can never be "restored" to the old ceiling.
      self.reset()
      if cap_target is not None:
        return cap_target, "dec"
      return None, None
    if cap_target is not None:
      cap_target = float(cap_target)
      # DEC ALWAYS WINS. A cap during RESTORE cancels the restore episode entirely and re-latches
      # at the CURRENT set; a cap during CAP just continues the episode (ceiling untouched).
      if self.phase != "cap":
        self.reset()
        self.phase = "cap"
        self.ceiling = float(v_set)
        # sazoneset2pnw: remember which road this ceiling belongs to (None/0 = limit unknown at latch)
        try:
          self.latch_limit = float(limit_now) if limit_now is not None and float(limit_now) > 0.0 else None
        except (TypeError, ValueError):
          self.latch_limit = None
        self._engage_t0 = now             # icbmratchet2pnw: see the clear-debounce handling below
        # icbmratchet2pnw (Gemini review catch, round 2): run the engage tick through the SAME
        # confirmation bookkeeping as any later tick (baseline = v_set, nothing confirmed yet) so a
        # bad first reading can NEVER become the episode's permanent _min_target/baseline just
        # because it happened first — that previously let a LATER, unrelated glitch slip through
        # ungated by comparing against a tainted floor (exploit: glitch engages low, a real curve's
        # correct value arrives on the very next tick with no clear in between — the _engage_t0
        # guard below only fires on a clear, so it alone could not catch this). We still ACT on
        # (publish/tap toward) the engage tick's own reading immediately below — bounded to at most
        # one executor tap's worth of impact at the real ~0.25 s cadence — but _min_target itself is
        # left exactly where this bookkeeping call leaves it (None, unless the engage value is
        # already within the outlier band of v_set, in which case it's legitimately trustworthy
        # immediately, same as any other small/ordinary drop).
        self._ratchet_confirm(now, cap_target, float(v_set))
        publish_target, accepted = cap_target, True
      else:
        # icbmratchet2pnw: an outlier-sized single-tick drop is held pending confirmation instead
        # of ratcheting (and publishing) immediately — see _ratchet_confirm / the constants above.
        # baseline: the last CONFIRMED target, or (nothing confirmed yet — a fresh episode whose
        # very first tick was itself never validated) the driver's current set.
        baseline = self._min_target if self._min_target is not None else float(v_set)
        publish_target, accepted = self._ratchet_confirm(now, cap_target, baseline)
      # icbmratchet2pnw: track what we actually published — as the pending-hold fallback (so a
      # genuinely consistent-but-unconfirmed engage doesn't flicker back to v_set while it confirms)
      # and as the running worst-case-published (for the driver-lower-guard below), independently of
      # the CONFIRMED `_min_target` baseline — see the three fields' docstrings in __init__.
      self._committed_target = publish_target
      self._min_published = (publish_target if self._min_published is None
                              else min(self._min_published, publish_target))
      self._clear_t0 = None             # curve (re)bound: reset the clear debounce
      # icbmmapfirst2pnw: remember where the binding candidate sits — used at clear to tell
      # "passed the curve" (early restore) from "detection dropout" (full debounce).
      # icbmratchet2pnw (Fable review catch): only snapshot from a CONFIRMED/accepted reading — an
      # unconfirmed, still-pending outlier's distance must not feed the apex-passage/early-restore
      # decision (it was never acted on, so the truck's actual position relative to it is moot).
      if accepted:
        try:
          self._last_cap_dist = float(cap_dist) if (cap_dist is not None and cap_dist == cap_dist
                                                    and cap_dist != float('inf')) else None
          self._last_cap_vego = max(float(v_ego), 0.0)
        except (TypeError, ValueError):
          self._last_cap_dist = None
          self._last_cap_vego = 0.0
      return publish_target, "dec"

    if self.phase == "cap":
      # clear DEBOUNCE: hold silent (ceiling retained, executor stale-stops within 2 s) until the
      # curve has stayed clear for clear_delay_s. A detection flicker or an S-curve gap therefore
      # keeps the ORIGINAL ceiling instead of starting a restore and re-latching lower.
      if self._clear_t0 is None:
        # icbmratchet2pnw (Gemini review catch): the engage that latched the CURRENT ceiling/
        # _min_target never had a chance to prove itself for ICBM_RATCHET_CONFIRM_S before the
        # candidate vanished again — an engage-then-immediate-clear is exactly the single-tick-
        # outlier signature this whole fix targets, just at the FIRST tick instead of a later one
        # (where _ratchet_confirm already catches it). Without this, a bad engage tick's ceiling/
        # _min_target could survive the clear and get reused — including by a later, unrelated,
        # genuinely-binding candidate reusing this SAME (unconfirmed, wrong) episode — because caps
        # are DEC-only: once a low value is tapped, only the fully separate, much slower guarded
        # RESTORE can undo it, never an in-episode recompute. Wipe the episode outright so any later
        # rebind starts completely fresh instead of resuming an unproven one.
        if self._engage_t0 is not None and (now - self._engage_t0) < ICBM_RATCHET_CONFIRM_S:
          self.reset()
          return None, None
        self._clear_t0 = now
        self._hold_set0 = stock_set     # snapshot: we go SILENT now, so nothing of ours moves the set
        self._late_tap_set = None       # fresh clear window: any previous absorbed tap is void
        # icbmratchet2pnw (Fable review catch): a candidate awaiting confirmation must NOT survive
        # a gap in cap ticks. _ratchet_confirm measures WALL TIME, not contiguous low readings, so
        # without this an old pending value (e.g. a glitch tick just before a brief detection
        # dropout) could sit stale through the gap and then get instantly "confirmed" — using
        # elapsed time that includes the silent gap — the moment ANY new, unrelated, merely
        # outlier-sized-but-legitimate candidate rebinds, via the worst-seen min(). Void it here so
        # a rebind after any gap always starts its own fresh confirmation window.
        self._pending_low_target = None
        self._pending_low_t0 = None
        # icbmmapfirst2pnw: apex passage — the binding candidate vanished while within the tap
        # margin + ~1 s of travel of us => we drove past it (curve behind), not a dropout.
        self._apex_passed = (self._last_cap_dist is not None
                             and self._last_cap_dist <= ICBM_MARGIN_M + self._last_cap_vego * ICBM_APEX_PASS_TTA_S)
      # icbmmapfirst2pnw late-tap absorption (field false positive, 2026-07-12 18:08:26Z): the truck
      # reports the set with ~1 s lag, so the executor's FINAL in-flight tap can land AFTER we went
      # silent. Within the grace window, ONE small DOWNWARD move (<= one tap + deadband, measured
      # from the ORIGINAL snapshot) is our own tap, not a human — record it as an ALTERNATE
      # baseline. ANCHORED, adopted at most once, never walked (Gemini adversarial catch: walking
      # the baseline would let a driver's repeated SET- taps be absorbed one step at a time).
      # Upward movement is never absorbed; a second downward step lands below BOTH baselines and
      # the movement guard blocks the restore exactly as before.
      if (stock_set is not None and self._hold_set0 is not None
          and self._late_tap_set is None
          and (now - self._clear_t0) <= ICBM_LATE_TAP_GRACE_S
          and 0.0 < self._hold_set0 - stock_set <= ICBM_LATE_TAP_TOL):
        self._late_tap_set = stock_set
      # icbmmapfirst2pnw early restore (driver rule 3): when the curve is provably BEHIND us, begin
      # the restore after a short drive-out instead of the full flicker-proof hold. A dropout-style
      # clear (candidate still far ahead) keeps the original 3 s debounce unchanged.
      delay = min(self._clear_delay_s, ICBM_RESTORE_DELAY_FAST_S) if self._apex_passed else self._clear_delay_s
      if (now - self._clear_t0) < delay:
        return None, None
      if in_curve:
        # still lateral-loaded (e.g. long curve, or the NEXT bend of an S): hold silently — never
        # BEGIN raising the set mid-curve. Bounded: loaded too long with no re-bind -> give up.
        if (now - self._clear_t0) > ICBM_HOLD_MAX_S:
          self.reset()
        return None, None
      # curve cleared (sustained) -> enter RESTORE only when the episode is cleanly ours:
      eligible = (stock_on and not driver_pedal and self.ceiling is not None
                  and stock_set is not None and stock_set > 0.0
                  # the current set is explainable by OUR taps — if the driver went lower than the
                  # lowest target we ever commanded, restoring would fight their intent: don't.
                  # icbmratchet2pnw: uses _min_published (everything actually published/tapped
                  # toward, incl. an unconfirmed-but-acted-on engage tick), NOT the stricter
                  # confirmed-only _min_target — this guard is about the EXECUTOR's real actions,
                  # which _min_target alone would understate.
                  and (self._min_published is None or stock_set >= self._min_published - ICBM_DRIVER_LOWER_TOL)
                  # Gemini adversarial catch (the 3 s blind spot): the brain is SILENT through the
                  # hold, so the executor does nothing — ANY set movement across the hold window is
                  # a HUMAN choosing a speed. Movement (either direction) -> no restore at all.
                  # icbmmapfirst2pnw: the set may alternatively match the ONE absorbed late-tap
                  # baseline (its own executor tap landing after silence) — nothing else.
                  and self._hold_set0 is not None
                  and (abs(stock_set - self._hold_set0) <= ICBM_RESTORE_DONE_TOL
                       or (self._late_tap_set is not None
                           and abs(stock_set - self._late_tap_set) <= ICBM_RESTORE_DONE_TOL))
                  # something to restore (not already at/above the ceiling)
                  and stock_set < self.ceiling - ICBM_RESTORE_DONE_TOL)
      if eligible:
        self.phase = "restore"
        self._t0 = now
        self._last_stock = stock_set
        self._last_t = now
        return self._restore_target(stock_set, restore_cap)
      self.reset()
      return None, None

    if self.phase == "restore":
      # hard aborts / completion — any of these ends the episode entirely (unlatch, go silent)
      if (not stock_on or driver_pedal or stock_set is None or stock_set <= 0.0
          or (now - self._t0) > self._window_s
          or stock_set >= self.ceiling - ICBM_RESTORE_DONE_TOL):
        self.reset()
        return None, None
      # brain-side manual-intervention detection (the executor's RestoreGuard is the fine-grained
      # one; this catches it independently at the 4 Hz brain cadence):
      if self._last_stock is not None:
        dt = max(now - (self._last_t if self._last_t is not None else now), 0.0)
        if stock_set < self._last_stock - ICBM_RESTORE_DONE_TOL:
          self.reset()                  # set went DOWN: only a human does that during restore
          return None, None
        if stock_set > self._last_stock + ICBM_EXEC_STEP_MS * (dt / ICBM_TAP_PERIOD_S + 1.6):
          self.reset()                  # rose faster than our taps can: driver holding SET+
          return None, None
      # icbmrestorecap2pnw (Fable review): while HOLDING at the posted-limit cap the brain publishes
      # nothing, so any rise beyond one in-flight tap is the driver pressing SET+. Without this a
      # single tap every few seconds slipped under the fast-rise detector above and was absorbed as
      # if it were ours -- a residual that existed before, but the hold stretched its window from
      # the ~13 s a restore used to take to the full 45 s restore window. Same tolerance, same
      # reasoning as the cap-phase hold's _hold_set0 snapshot.
      hold0 = getattr(self, "_rcap_hold_set", None)
      if hold0 is not None and stock_set > hold0 + ICBM_LATE_TAP_TOL:
        self.reset()
        return None, None
      self._last_stock = stock_set
      self._last_t = now
      if in_curve:
        # icbmmapfirst2pnw: PAUSE while lateral-loaded (a late-seen next bend) — go silent so the
        # executor stale-stops, but keep the episode so the restore resumes once the load clears.
        # All the aborts above (pedal/ACC/window/decrease/fast-rise) ran this tick and stay live.
        return None, None
      return self._restore_target(stock_set, restore_cap)

    return None, None                   # idle, no cap


def _haversine_m(lat1, lon1, lat2, lon2) -> float:
  """Great-circle distance in metres (pure)."""
  import math
  r = 6371000.0
  p1, p2 = math.radians(lat1), math.radians(lat2)
  dp = math.radians(lat2 - lat1); dl = math.radians(lon2 - lon1)
  a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
  return 2 * r * math.asin(min(1.0, a ** 0.5))


def map_turn_direction(points, cur_lat, cur_lon, target_dist, tol_m: float = 60.0) -> int:
  """icbmalign2pnw: turn DIRECTION of the map path at ~`target_dist` metres ahead: +1 = LEFT,
  -1 = right, 0 = straight/unknown. Finds the path point whose distance from the current position
  is closest to `target_dist` (the ICBM candidate's recorded distance — same haversine metric, so
  the match is exact for map/far candidates), then sums the signed turn (2-D cross product of
  successive segment vectors in a local east/north frame) over a small window around it.
  Sign: with x = east, y = north, cross(v0, v1) > 0 = counterclockwise viewed from above = LEFT —
  the same left-positive convention as vtsc_pnw.apex_turn_direction / openpilot steering.
  Ambiguous / too few points / no match within tol_m -> 0 (neutral: the left factor is simply not
  applied — fail-safe is 'no extra penalty', never a wrong-direction penalty). Pure."""
  if not points or cur_lat is None or cur_lon is None or target_dist == float('inf'):
    return 0
  pts = []
  for p in points:
    try:
      la, lo = float(p["latitude"]), float(p["longitude"])
    except (KeyError, TypeError, ValueError):
      continue
    if la != la or lo != lo:                      # NaN guard
      continue
    pts.append((la, lo))
  if len(pts) < 3:
    return 0
  # nearest path point to the candidate distance
  best_i, best_err = -1, float('inf')
  for i, (la, lo) in enumerate(pts):
    err = abs(_haversine_m(cur_lat, cur_lon, la, lo) - target_dist)
    if err < best_err:
      best_i, best_err = i, err
  if best_err > tol_m:
    return 0
  # signed turn accumulated over up to 2 corners each side of the candidate point (local EN frame;
  # per-corner cross is normalized = sin(turn angle), so GPS point spacing doesn't bias the sum)
  coslat = math.cos(math.radians(cur_lat))
  total = 0.0
  for j in range(max(1, best_i - 2), min(len(pts) - 1, best_i + 3)):
    ax = (pts[j][1] - pts[j - 1][1]) * coslat
    ay = pts[j][0] - pts[j - 1][0]
    bx = (pts[j + 1][1] - pts[j][1]) * coslat
    by = pts[j + 1][0] - pts[j][0]
    na, nb = math.hypot(ax, ay), math.hypot(bx, by)
    if na <= 0.0 or nb <= 0.0:
      continue
    total += (ax * by - ay * bx) / (na * nb)
  if abs(total) < 0.02:                           # ~1 deg total bend: too straight to call
    return 0
  return 1 if total > 0.0 else -1


def upcoming_curve(target_velocities, cur_lat, cur_lon, v_ego, lookahead_s) -> tuple[float, float]:
  """From pfeiferj's MapTargetVelocities (list of {latitude, longitude, velocity}) + current
  position, return (min_target_velocity, distance) of the most-binding upcoming curve within the
  lookahead distance (v_ego * lookahead_s). Returns (0.0, inf) if none / no data. Pure & testable.

  icbmonset: a point failing `_map_v_sane` (NaN, seen live 2026-07-11, OR a physically-implausible
  finite spike, seen live 2026-07-18 — mapd's curvature calc misfiring at curve entry) is skipped
  exactly like the pre-existing NaN case. Shared by CES's own curve trip (decide_active/
  curve_closeness, both CES1 and the ces2_core shadow, all cars) as well as ICBM's near-window
  candidate: provably a no-op for every existing decision there, since a value this high already
  fails every 'is this curve binding' comparison identically to 'no candidate' (v_ego - tv is always
  very negative, tv*scale is always far above any sharp-curve/binding threshold) — only the raw
  telemetry (mapV/curvePct) stops showing the nonsense number."""
  if not target_velocities or cur_lat is None or cur_lon is None:
    return 0.0, float('inf')
  horizon = max(v_ego, 1.0) * lookahead_s
  best_v, best_d = 0.0, float('inf')
  for p in target_velocities:
    try:
      d = _haversine_m(cur_lat, cur_lon, p["latitude"], p["longitude"])
      tv = float(p["velocity"])
    except (KeyError, TypeError, ValueError):
      continue
    if not _map_v_sane(tv) or d != d:   # icbmonset: NaN + curvature-noise guard
      continue
    if 0.0 < d <= horizon:
      # most-binding = lowest target speed ahead within the horizon
      if best_v == 0.0 or tv < best_v:
        best_v, best_d = tv, d
  return best_v, best_d


class Condition:
  """A debounced boolean signal: raw bool -> filtered -> compared to THRESHOLD. The filter is driven
  by the MEASURED loop dt (selfdrived runs at 100 Hz / DT_CTRL, not the model rate) so FILTER_TAU is a
  real time constant — using a fixed DT_MDL here made the debounce run 5x too fast (instant flapping)."""
  def __init__(self):
    self.f = FirstOrderFilter(0.0, C.FILTER_TAU, DT_CTRL)
    self.active = False

  def update(self, raw: bool, dt: float = DT_CTRL) -> bool:
    # tolerance, not equality: the measured dt jitters every cycle, and an exact != recomputed the
    # filter alpha (an exp()) at 100 Hz per condition for no behavioral gain
    if abs(dt - self.f.dt) > 1e-3:
      self.f.dt = dt
      self.f.update_alpha(C.FILTER_TAU)
    self.f.update(1.0 if raw else 0.0)
    self.active = self.f.x >= C.THRESHOLD
    return self.active

  def reset(self):
    self.f.x = 0.0
    self.active = False

  def force(self):
    """stopintent2pnw: charge the debounce to 'fully active' — used ONLY by the stop-intent fast
    path so the EXIT side behaves exactly as after a normal entry (the condition must genuinely
    clear and decay before Chill is considered; the return path keeps its full anti-flap dwell)."""
    self.f.x = 1.0
    self.active = True


def _lead_pull_away(s) -> bool:
  """pullaway2pnw (PURE): evidence-gated pull-away exception BELOW the redlight2pnw floor
  (ACCEL_ZONE_MIN_V). True only when ALL hold — any failure means the floor stands exactly as
  before (incident 2026-07-12 ~14:0x PT: ~17 mph behind a pulling-away lead, CES held Experimental
  and the truck lost the lead; driver rule: "the lead car cannot pull away"):
    (a) a lead is PRESENT with dRel in the sane 5-60 m band,
    (b) the lead is genuinely OPENING: at least PULLAWAY_DV faster than ego AND s["lead_opening"]
        (the PullAwayTracker's 3-spaced-sample monotonic dRel rise, which also enforces the
        model-stop recency guard — the yellow-light trap),
    (c) the model does NOT want to stop this cycle (the exact signal the red-light guard keys on),
    (d) ego is actually moving (>= PULLAWAY_MIN_V, never from standstill) and below the floor
        (above it, nothing changes — byte-identical).
  Callers that do not supply "lead_opening" (older pure tests) get False -> today's behavior."""
  return (bool(s["has_lead"])
          and not s.get("model_should_stop")
          and C.PULLAWAY_MIN_V <= s["v_ego"] < C.ACCEL_ZONE_MIN_V
          and C.PULLAWAY_DREL_LO <= s["lead_drel"] <= C.PULLAWAY_DREL_HI
          and (s["lead_vlead"] - s["v_ego"]) >= C.PULLAWAY_DV
          and bool(s.get("lead_opening", False)))


class PullAwayTracker:
  """pullaway2pnw: the STATEFUL evidence half of the pull-away exception (pure, unit-tested;
  owned by CESController). Produces the s["lead_opening"] bool from per-cycle observations:
    - dRel must RISE monotonically (>= PULLAWAY_OPEN_EPS per step) across PULLAWAY_SAMPLES
      samples spaced >= PULLAWAY_SAMPLE_GAP_S apart (a real, sustained opening — not radar noise);
    - a lead loss, dRel jump (> PULLAWAY_JUMP_M, lead swap / radar reacquire) restarts the
      evidence from scratch;
    - the model must not have wanted to stop within PULLAWAY_STOP_CLEAR_S (recency guard: a
      shouldStop flicker while a lead clears a yellow light must keep blocking after it clears)."""

  def __init__(self):
    self._hist = []            # [(t, dRel)] newest last, at most PULLAWAY_SAMPLES entries
    self._last_stop_t = None   # last time the model wanted to stop

  def update(self, now, has_lead, d_rel, model_should_stop) -> bool:
    if model_should_stop:
      self._last_stop_t = now
    if not has_lead or d_rel is None or d_rel <= 0.0:
      self._hist = []          # no (valid) lead: evidence dies with it
      return False
    if self._hist and abs(float(d_rel) - self._hist[-1][1]) > C.PULLAWAY_JUMP_M:
      self._hist = []          # discontinuity: different lead / radar reacquire
    if not self._hist or (now - self._hist[-1][0]) >= C.PULLAWAY_SAMPLE_GAP_S:
      self._hist.append((now, float(d_rel)))
      self._hist = self._hist[-C.PULLAWAY_SAMPLES:]
    opening = (len(self._hist) == C.PULLAWAY_SAMPLES
               and all(self._hist[i + 1][1] - self._hist[i][1] >= C.PULLAWAY_OPEN_EPS
                       for i in range(C.PULLAWAY_SAMPLES - 1)))
    stop_recent = (self._last_stop_t is not None
                   and (now - self._last_stop_t) < C.PULLAWAY_STOP_CLEAR_S)
    return opening and not stop_recent


def _accelerate_zone_base(s) -> bool:
  """PURE: True when we're slow but should be ACCELERATING into open road, so Experimental's timid
  e2e acceleration would hurt — keep Chill instead. Covers the two cases:
    - highway on-ramp merge (open road ahead, set speed = highway >> ramp speed)
    - stop&go where the lead has pulled away leaving a big gap (catch back up at Chill briskness)
  Requires open road ahead AND a set speed meaningfully above current speed. Only gates `lowSpeed`.

  redlight2pnw (SAFETY, driver report 2026-07-11, confirmed in ces_events — exp->chill at 0-5 mph
  with az=True): approaching a red light with no lead and a high set speed has the SAME signals as an
  on-ramp merge (open road + set >> ego), so this gate suppressed the low-speed Experimental hold at
  the last moment and Chill's MPC accelerated toward the set speed -> lurch through the light. Two
  fail-safe guards (both only ever KEEP Experimental, which DOES stop for lights):
    (a) defer to the model's stop prediction — if it says stop, this is never an accelerate-zone;
    (b) the NO-LEAD branch (pure open road = could be a red light) requires a real merge speed floor;
        near a stop you are never 'merging onto a highway'. The has-lead-far branch (a genuine lead
        pull-away, where a lead is present so it can't be a clear red light) is unchanged."""
  if s.get("model_should_stop"):
    return False
  no_lead = not s["has_lead"]
  if no_lead and s["v_ego"] < C.ACCEL_ZONE_MIN_V:
    return False
  open_ahead = no_lead or (s["lead_drel"] > C.GAP_OPEN_M
                           and s["lead_vlead"] >= s["v_ego"] - C.LEAD_PULLAWAY_MARGIN)
  want_faster = s["v_set"] > 0.0 and (s["v_set"] - s["v_ego"]) > C.ACCEL_ZONE_DV
  return open_ahead and want_faster


def _accelerate_zone(s) -> bool:
  """pullaway2pnw: the accelerate-zone is the UNCHANGED base (redlight2pnw semantics, floor and
  all) OR the evidence-gated lead-pull-away exception below the floor. Above the floor and in
  every no-lead case this is byte-identical to _accelerate_zone_base."""
  return _accelerate_zone_base(s) or _lead_pull_away(s)


def decide_active(s) -> tuple[bool, str]:
  """PURE decision core (no state, no filtering): given a signals dict-like `s`, return
  (any_condition_active, status). Used by both the live controller (post-filter) and the unit tests.

  Expected keys (all SI, primitives):
    v_ego, has_lead, lead_vlead, lead_drel, blinker,
    map_target_v, map_target_dist, curve_lat_accel_vision, time_to_curve,
    model_should_stop,
    toggles: curves/stops/low_speed/lead (bool enables)
  """
  t = s["toggles"]
  v = s["v_ego"]

  # 1) curve — map (primary, ~10 s) OR vision (fallback, ~3.5 s)
  # Freeway gate: on a known-freeway (OSM spd_lim >= CURVE_HWY_GATE) hand MODERATE curves to VTSC+MTSC
  # (bounded, decel-limited) and DON'T trip Experimental e2e curve braking — it stacks below the VTSC floor
  # and over-slows (driver gas-override on the 2026-06-28 Snoqualmie sweepers). spd_lim 0/unknown => not
  # gated (keep tripping, safe default). Only the curve reason is gated; stop/lead/radar are intact.
  # SHARP-curve exception: a genuinely sharp curve (map target < CURVE_SHARP_MAP_V) keeps the trip even on
  # a freeway — that's where steering-limit/EPS risk is (2026-06-28 North Bend descent take-control), so we
  # want maximum braking authority (e2e + VTSC + MTSC), not just the bounded cap.
  freeway_gated = s["spd_lim"] >= C.CURVE_HWY_GATE
  # Compare the SCALED map target (what MTSC actually drives to), not the raw one: mapd's raw safe-speeds
  # run systematically low (~1.5-1.8x), so the raw compare tripped Experimental on I-84 sweepers the driver
  # takes at 79-86 mph (raw 24-30 m/s, 2026-07-06 09:42-09:48 cluster). Genuinely sharp curves (scaled
  # target still < CURVE_SHARP_MAP_V) keep the exception and full braking authority.
  # sharpcurve2pnw iter2: use the TIERED effective scale (shared helper) so this classification and the
  # MTSC fold agree — a flat 1.8 here would inflate a tight curve's target past CURVE_SHARP_MAP_V and
  # drop its sharp flag exactly where full braking authority matters most.
  sharp_curve = 0.0 < s["map_target_v"] * C.tiered_map_scale(s["map_target_v"]) < C.CURVE_SHARP_MAP_V
  # ces2pnw lead-pacing gate (2026-07-08 02:12:57Z, lebowski first drive): a curve-triggered
  # Experimental HELD through an extended 100%-map winding stretch while the (faster) lead pulled away
  # 43->126 m — e2e crawls, and the driver ruled it unacceptable ("the lead car cannot pull away").
  # When a lead within CURVE_LEAD_PACE_DREL is pacing us (not slower than us minus the margin), the
  # curve trip is suppressed ENTIRELY — including sharp curves: the pacing lead is live evidence of a
  # drivable line, and VTSC+MTSC (tiered scale + decel envelope + sharp-curve firmer rate-limit) remain
  # the independent physical cap either way. A lead that brakes/slows flips to the slowLead trigger
  # below; a lead beyond the range (or lost) re-arms the curve trip immediately.
  lead_pacing = (s["has_lead"] and s["lead_drel"] < C.CURVE_LEAD_PACE_DREL
                 and s["lead_vlead"] >= v - C.LEAD_PULLAWAY_MARGIN)
  # ces2pnw accel-zone curve gate (2026-07-09 03:01-03:15Z on-ramp): a highway on-ramp IS a curve, so
  # the curve trip pinned the merge at 38-39 mph (set 90, no lead, aEgo ~0 for 7 s) while the driver
  # rode the gas — _accelerate_zone (already gating lowSpeed for exactly this merge case) now gates the
  # curve trip too. VTSC (vision) + tiered MTSC still cap the curved portion physically; suppressing
  # only the Experimental e2e layer lets Chill's MPC pull to the merge speed the moment VTSC releases.
  # Solo-cruising-at-set into a curve (v_set ~ v_ego, e.g. Terwilliger) keeps the trip: az is False there.
  if t["curves"] and v > C.CRUISING_SPEED and (not freeway_gated or sharp_curve) \
     and not lead_pacing and not _accelerate_zone(s):
    # MAP: pfeiferj MapTargetVelocities gives a safe curve speed ahead. Trip when an upcoming
    # target speed within the lookahead is meaningfully (>MIN_SLOWDOWN) below current speed.
    map_curve = (s["map_target_v"] > 0.0
                 and (v - s["map_target_v"]) > C.CURVE_MAP_MIN_SLOWDOWN
                 and 0.0 < s["map_target_dist"] / max(v, 1.0) < C.CURVE_MAP_LOOKAHEAD_S)
    # VISION fallback: predicted lateral accel over the (short) model horizon.
    vision_curve = (abs(s["curve_lat_accel_vision"]) > C.CURVE_LAT_ACCEL_ENTER
                    and s["time_to_curve"] < C.CURVE_VISION_LOOKAHEAD_S
                    and not s["blinker"])
    if map_curve or vision_curve:
      return True, "curve"

  # 2) stop light / stop sign — model predicts a stop, not currently following a lead.
  # stophold2pnw (A1, red-light lurch 2026-07-12 21:47:08Z): below STOP_HOLD_MAX_V the `not
  # has_lead` mask is LIFTED — stopped/creeping behind a lead at a light, the model's stop intent
  # must count (the original mask exists so lead-following decel AT SPEED doesn't trip
  # Experimental; at a creep the LIGHT governs, not the lead). This also re-arms the stopIntent
  # fast path in exactly the lurch geometry (raw_active becomes True, so a Chill machine re-enters
  # in one cycle when shouldStop asserts). Fail-safe: only ever KEEPS/ENTERS Experimental.
  if t["stops"] and s["model_should_stop"] and (not s["has_lead"] or v < C.STOP_HOLD_MAX_V):
    return True, "stop"

  # 3) low speed (city / complex / construction) — lead-aware threshold. TWO exceptions, both
  #    learned from the drive log (only ever REMOVE Experimental, so safe):
  #    (a) highway gate: skip on a road whose OSM speed limit is high — slow-but-following on a
  #        highway is normal Chill cruising, not a complex zone.
  #    (b) accelerate-zone: skip when we should be accelerating into open road (on-ramp merge /
  #        lead pulled away) — Experimental's timid e2e acceleration is bad there.
  thr = C.CES_SPEED_LEAD if s["has_lead"] else C.CES_SPEED
  on_highway = s["spd_lim"] >= C.LOWSPEED_HWY_GATE
  if t["low_speed"] and 1.0 <= v < thr and not on_highway and not _accelerate_zone(s):
    return True, "lowSpeed"

  # 4) slow / stopped lead — closing on a slower/stopped lead -> let e2e do the smooth decel
  if t["lead"] and s["has_lead"]:
    if (v - s["lead_vlead"]) > C.SLOW_LEAD_DV or s["lead_vlead"] < C.STOPPED_LEAD_V:
      return True, "slowLead"

  # pullaway2pnw telemetry: when the ONLY thing keeping us out of the lowSpeed Experimental hold
  # is the pull-away exception (base accel-zone would NOT fire), name the reason so field
  # validation reads straight off ces_events / the overlay `why` line.
  if (t["low_speed"] and 1.0 <= v < thr and not on_highway
      and not _accelerate_zone_base(s) and _lead_pull_away(s)):
    return False, "pullAway"
  return False, "chill"


def _clamp01(x: float) -> float:
  return 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)


def curve_closeness(s) -> tuple[float, str]:
  """PURE, display-only: 'how close are we to tripping Experimental for a curve', 0.0..1.0, plus
  which half drives it ('map' / 'vision' / ''). 1.0 == at/over the entry threshold (switch imminent).
  Mirrors the curve branch of `decide_active` but as a continuous ratio for the on-screen feedback —
  it does NOT make the decision. 0.80 ~= "very close", 0.99 ~= "about to switch", >=1.0 == tripping."""
  t = s["toggles"]
  v = s["v_ego"]
  if not t["curves"] or v <= C.CRUISING_SPEED:
    return 0.0, ""
  # MAP half: how far the upcoming safe curve speed sits below us vs the slowdown that trips it,
  # but only while that curve is within the lookahead time.
  map_close = 0.0
  mv, md = s["map_target_v"], s["map_target_dist"]
  if mv > 0.0 and 0.0 < md / max(v, 1.0) < C.CURVE_MAP_LOOKAHEAD_S:
    map_close = _clamp01((v - mv) / C.CURVE_MAP_MIN_SLOWDOWN)
  # VISION half: predicted lateral accel vs the entry threshold, within the (short) vision horizon.
  vis_close = 0.0
  if s["time_to_curve"] < C.CURVE_VISION_LOOKAHEAD_S and not s["blinker"]:
    vis_close = _clamp01(abs(s["curve_lat_accel_vision"]) / C.CURVE_LAT_ACCEL_ENTER)
  if map_close >= vis_close:
    return map_close, ("map" if map_close > 0.0 else "")
  return vis_close, "vision"


def lead_metrics(has_lead: bool, d_rel: float, v_lead: float, v_ego: float) -> tuple[float, float]:
  """PURE (vtsctele2pnw): lead-follow metrics for telemetry — (gap seconds, lead speed delta m/s).
  gap = dRel/vEgo rounded to 1 decimal (0.0 when no lead or near-stopped, so a logged 0.0 always
  means 'not following'); delta = vLead - vEgo (positive = lead pulling away, negative = closing).
  Display/logging only — never gates control."""
  if not has_lead:
    return 0.0, 0.0
  gap = round(float(d_rel) / float(v_ego), 1) if float(v_ego) > 0.5 else 0.0
  return gap, round(float(v_lead) - float(v_ego), 1)


def decision_telemetry(s) -> dict:
  """PURE, display-only: a compact snapshot for the on-screen CES overlay. Reports the binding
  reason, the curve 'closeness' as a 0..100 %, and the upcoming map-curve preview (target speed +
  distance). Built from the SAME signals dict `decide_active` consumes, so the overlay can never
  disagree with the live decision."""
  raw_active, reason = decide_active(s)
  cpct, csrc = curve_closeness(s)
  md = s["map_target_dist"]
  gap_s, d_v = lead_metrics(s["has_lead"], s["lead_drel"], s["lead_vlead"], s["v_ego"])
  return {
    "rawActive": bool(raw_active),
    "reason": reason,
    "mdlEndX": round(float(s.get("mdl_end_x", 0.0)), 1),   # ces2-study replay dataset (log-only)
    "curvePct": int(round(cpct * 100)),
    "curveSrc": csrc,
    "mapV": round(float(s["map_target_v"]), 1),
    "mapDist": round(float(md), 0) if md != float('inf') else 0.0,
    "vEgo": round(float(s["v_ego"]), 1),
    # speedlimitdebug2pnw (driver req 2026-07-16): surface the OSM speed limit in the overlay feed
    # too (it was already logged per-event via self._speed_limit, just never in the live CESStatus
    # dict the on-screen box reads) so the overlay can flash it on a real change.
    "spdLim": round(float(s.get("spd_lim", 0.0)), 1),
    # accelerate-zone + the signals that drive it (also logged per event for later tuning)
    "accelZone": _accelerate_zone(s),
    "hwyGate": s.get("spd_lim", 0.0) >= C.LOWSPEED_HWY_GATE,   # lowSpeed suppressed: on a highway
    "vSet": round(float(s["v_set"]), 1),
    "dRel": round(float(s["lead_drel"]), 0),
    "vLead": round(float(s["lead_vlead"]), 1),
    # vtsctele2pnw: explicit lead-present bool (radarState.leadOne.status — a logged dRel of 0.0 is
    # ambiguous: 'no lead' vs 'lead at 0 m'), gap time (s) and lead speed delta (m/s), so
    # LongitudinalExt / curve-entry forensics stop inferring lead state from dRel>0.
    "lead": bool(s["has_lead"]),
    "gapS": gap_s,
    "dV": d_v,
    "aEgo": round(float(s.get("a_ego", 0.0)), 2),
    "gas": bool(s.get("gas", False)),
    # lowspeedcurve2pnw telemetry-first discovery (Hwy 99 2026-07-13 field issue #5: curvePct
    # stuck at 0 all drive): the RAW vision-curve inputs, so the next city drive shows WHICH gate
    # held it — weak predicted lat-accel (visLat vs CURVE_LAT_ACCEL_ENTER), the lookahead window
    # (visTtc vs CURVE_VISION_LOOKAHEAD_S), or blinker suppression (blnk True on signaled turns).
    "visLat": round(float(s.get("curve_lat_accel_vision", 0.0) or 0.0), 2),
    "visTtc": round(float(s.get("time_to_curve", 0.0) or 0.0), 1),
    "blnk": bool(s.get("blinker", False)),
    # stophold2pnw (B): the RAW per-cycle model stop intent (modelV2.action.shouldStop), NOT
    # debounced — this is the exact signal A1/A2/stopIntent/pullaway key on, and its absence from
    # the breadcrumb is why the 21:47:08Z lurch forensics could not tell red from green.
    "stp": bool(s.get("model_should_stop", False)),
  }


class ConditionalExperimentalSwitching:
  """Live controller. Owns the per-condition filters + the mode state machine (min-dwell + sustained
  clear). `mode()` returns 'experimental'/'chill'; `update(sm, toggles)` is called each cycle."""

  def __init__(self, exp_min_dwell: float = C.EXP_MIN_DWELL_S, chill_min_dwell: float = C.CHILL_MIN_DWELL_S):
    # one debounce filter per condition (entry) + one for the all-clear (exit)
    self._cond = Condition()        # "any condition active" (debounced)
    self._is_experimental = False
    self._dwell = 0.0               # s in current mode
    self._status = "chill"
    self._exp_min = exp_min_dwell   # min dwell in Experimental (gentle profile lengthens this)
    self._chill_min = chill_min_dwell  # min dwell in Chill / re-entry cooldown
    # stophold2pnw (A2): seconds model_should_stop has been CONTINUOUSLY clear. Initialized to the
    # hold threshold ("clear long ago") so a fresh machine never spuriously holds.
    self._stop_clear_s = C.STOP_CLEAR_HOLD_S
    # standstill2pnw (see the constants block for the field basis + design):
    self._ss_lead_s = 0.0        # s of continuous lead presence at standstill (promotion debounce)
    self._at_standstill = False  # True while Experimental at v < STANDSTILL_LATCH_V (latch active)
    self._ss_close_lead = False  # a lead was within STANDSTILL_RELEASE_DREL during the standstill
                                 #   (sticky across radar dropouts — the field flip ticks all show
                                 #   lead=False; only a SEEN gap > CLEAR_DREL clears it)
    self._release_hold = False   # armed at the release tick when _ss_close_lead; holds Experimental
                                 #   until v > STANDSTILL_RELEASE_V or the gap opens past CLEAR_DREL
    # cesnochill2pnw: True while the pure-v_ego Schmitt-trigger latch is armed (see the constants
    # block) — starts False (a fresh machine at cruise is not "stopping"; the first genuine
    # decel-to-a-stop arms it on its own from live v_ego).
    self._nochill_armed = False

  def reset(self):
    self._cond.reset()
    self._is_experimental = False
    self._dwell = 0.0
    self._status = "chill"
    self._stop_clear_s = C.STOP_CLEAR_HOLD_S  # stophold2pnw (A2)
    self._ss_lead_s = 0.0                     # standstill2pnw
    self._at_standstill = False
    self._ss_close_lead = False
    self._release_hold = False
    self._nochill_armed = False               # cesnochill2pnw

  def mode(self) -> str:
    return "experimental" if self._is_experimental else "chill"

  def status(self) -> str:
    return self._status

  def update_decision(self, signals: dict, dt: float = DT_CTRL) -> str:
    """Public entry point: run the full decision core, then apply the cesnochill2pnw hard latch as
    a FINAL override so it wins over every internal path (dwell expiry, A2, filter decay, a
    model_should_stop flicker, or any future addition) — closing the ordering gap that let a
    transient `chill` decision through anywhere in the stopping/stopped speed band (see the
    constants block for the field incident, the round-2 a_ego revert, and the wedge-impossibility
    proof). Behavior-neutral once genuinely moving away: the latch is a no-op there and the
    unmodified core governs exactly as before."""
    self._update_decision_core(signals, dt)
    self._apply_nochill_latch(float(signals.get("v_ego", 0.0)))
    return self.mode()

  def _apply_nochill_latch(self, v_now: float) -> None:
    """cesnochill2pnw: PURE v_ego Schmitt-trigger latch — no acceleration term (round 2's a_ego
    direction gate was reverted: Gemini review found it could wedge Experimental permanently on a
    gentle/leveling-off launch, and flapped on a_ego noise near the release band). One bit of
    armed/not memory, re-evaluated every tick from live v_ego alone — see the constants block for
    the full ARM/RELEASE spec and the wedge-impossibility proof (RELEASE depends on v_ego ALONE, so
    any tick above NOCHILL_RELEASE_V clears it unconditionally — a real launch cannot avoid producing
    such a tick). While armed, status is UNCONDITIONALLY "stopLatch" for the whole episode (Gemini
    review, round 1: avoids flapping between a stale core-computed reason and the latch tag every
    time the core's own dwell machinery cycles underneath) — `_dwell` is never touched here."""
    if self._nochill_armed:
      if v_now > C.NOCHILL_RELEASE_V:
        self._nochill_armed = False
    elif v_now < C.NOCHILL_ARM_V:
      self._nochill_armed = True
    if self._nochill_armed:
      self._is_experimental = True
      self._status = "stopLatch"   # cesnochill2pnw telemetry tag: unconditional for the whole hold

  def _update_decision_core(self, signals: dict, dt: float = DT_CTRL) -> str:
    """Advance the state machine one cycle from an extracted `signals` dict (see decide_active).
    `dt` is the MEASURED loop period (selfdrived runs at 100 Hz) so the dwell/debounce are real
    seconds. Separated from `update(sm)` so it is unit-testable without cereal messages."""
    raw_active, status = decide_active(signals)
    v_now = float(signals.get("v_ego", 0.0))
    has_lead = bool(signals.get("has_lead", False))
    d_rel = float(signals.get("lead_drel", 0.0) or 0.0)
    # stophold2pnw (A2): track how long the model's stop intent has been continuously clear —
    # updated FIRST so the timer is correct on every path out of this function (incl. the
    # stopIntent fast-path early return below). Capped at the threshold (no unbounded float).
    if bool(signals.get("model_should_stop")):
      self._stop_clear_s = 0.0
    else:
      self._stop_clear_s = min(self._stop_clear_s + dt, C.STOP_CLEAR_HOLD_S)
    # standstill2pnw: sustained-lead evidence at standstill (the promotion debounce) — updated
    # FIRST like _stop_clear_s so it is correct on every path. A lead DROPOUT resets it (strict
    # continuity: a single-tick radar ghost can never charge it). Capped at the threshold.
    if v_now < C.STANDSTILL_LATCH_V and has_lead:
      self._ss_lead_s = min(self._ss_lead_s + dt, C.STANDSTILL_PROMOTE_LEAD_S)
    else:
      self._ss_lead_s = 0.0
    # stopintent2pnw (driver-approved): ABSOLUTE stop-intent fast path. When the model's stop
    # intent (model_should_stop — the exact signal the red-light guard keys on) asserts AND the
    # decision ladder wants Experimental, entering Experimental bypasses EVERYTHING on the entry
    # side: the CHILL_MIN_DWELL_S re-entry cooldown, the ~1 s condition filter charge, and any
    # accel-zone adoption (the az is already dead while shouldStop by the existing gate). Churn
    # TOWARD stopping is the safe direction and is exempt from anti-flap; the RETURN to Chill
    # keeps the full normal dwell (filter force() + untouched exit path = the asymmetry).
    # This closes the pullaway2pnw occlusion trap (Gemini STOP, 2026-07-12: a departing lead
    # occludes a red light; shouldStop asserts only AFTER a pull-away Chill adoption — the 5 s
    # cooldown then held Chill through the intersection) and the PRE-EXISTING 5 s stop-blind
    # window after ANY accel-zone adoption. Respects the per-condition "stops" toggle; the
    # driver's forced-Chill button still wins upstream (it never reaches this state machine).
    if (not self._is_experimental and raw_active
        and bool(signals.get("model_should_stop"))
        and bool(signals.get("toggles", {}).get("stops", True))):
      self._is_experimental = True
      self._status = "stopIntent"      # telemetry tag: every fast-path preemption is visible
      self._dwell = 0.0
      self._cond.force()               # exit side sees a fully-charged condition (normal semantics)
      self._at_standstill = v_now < C.STANDSTILL_LATCH_V   # standstill2pnw: latch state from entry
      self._release_hold = False
      self._ss_close_lead = False
      return self.mode()
    # standstill2pnw PROMOTE: at standstill in Chill with the ladder wanting Experimental (only
    # slowLead / stop can be raw-active at v~0 — lowSpeed has a 1.0 floor, curve needs
    # CRUISING_SPEED), enter WITHOUT the CHILL_MIN_DWELL_S cooldown or the ~1 s filter charge —
    # at 0 mph those anti-flap timers were FIGHTING the trigger (the 11:34-11:38Z flapping), and
    # Experimental at standstill is strictly safer. Gated on STANDSTILL_PROMOTE_LEAD_S of
    # CONTINUOUS lead presence instead (radar-ghost debounce); the latch below then makes the
    # promoted state absorbing at standstill, so chill<->exp oscillation is structurally impossible
    # there. Exit semantics stay normal (force() = fully-charged condition, full return dwell).
    if (not self._is_experimental and raw_active
        and v_now < C.STANDSTILL_LATCH_V
        and self._ss_lead_s >= C.STANDSTILL_PROMOTE_LEAD_S):
      self._is_experimental = True
      self._status = status            # the real trigger (slowLead/stop) — meaningful in the logs
      self._dwell = 0.0
      self._cond.force()
      self._at_standstill = True
      self._release_hold = False
      self._ss_close_lead = has_lead and d_rel <= C.STANDSTILL_RELEASE_DREL
      return self.mode()
    cond_active = self._cond.update(raw_active, dt)   # debounced (real-time)
    self._dwell += dt

    if not self._is_experimental:
      # enter Experimental once the debounced condition is active AND we've been in Chill at least
      # the re-entry cooldown (de-flap: stops the instant snap-back that caused the stop&go sawtooth)
      if cond_active and self._dwell >= self._chill_min:
        self._is_experimental = True
        self._status = status
        self._dwell = 0.0
        self._at_standstill = v_now < C.STANDSTILL_LATCH_V   # standstill2pnw: latch state from entry
        self._release_hold = False
        self._ss_close_lead = has_lead and d_rel <= C.STANDSTILL_RELEASE_DREL and self._at_standstill
    else:
      # standstill2pnw: maintain the latch / release-hold state EVERY Experimental tick (not only
      # when an exit is eligible — the release tick can land while a condition is still charged).
      if v_now < C.STANDSTILL_LATCH_V:
        self._at_standstill = True
        self._release_hold = False     # moot while stopped — the latch owns standstill
        if has_lead and d_rel <= C.STANDSTILL_RELEASE_DREL:
          self._ss_close_lead = True
        elif has_lead and d_rel > C.STANDSTILL_RELEASE_CLEAR_DREL:
          self._ss_close_lead = False  # a SEEN open gap clears it; lead-absent ticks keep the last
                                       # value (radar dropouts at standstill are the field norm —
                                       # every 11:34-11:38Z flip tick logged lead=False)
      else:
        if self._at_standstill:
          # the RELEASE tick: leaving standstill with a close lead arms the hold — the launch into
          # a short gap stays model-governed instead of a Chill MPC launch into a 9-14 m gap.
          self._release_hold = self._ss_close_lead
          self._at_standstill = False
          self._ss_close_lead = False
        if self._release_hold and (v_now > C.STANDSTILL_RELEASE_V
                                   or (has_lead and d_rel > C.STANDSTILL_RELEASE_CLEAR_DREL)):
          self._release_hold = False   # launch complete / gap opened: hand back to the ladder.
                                       # Lead LOSS deliberately does not disarm (a dropout mid-launch
                                       # must not re-open the jolt); the hold is bounded by v > 5.
      # stay Experimental; return to Chill only when the condition cleared (sustained via filter)
      # AND we've held Experimental at least EXP_MIN_DWELL_S
      if status != "chill":
        self._status = status      # keep showing the active reason
      if not cond_active and self._dwell >= self._exp_min:
        # standstill2pnw LATCH: at standstill (or during the close-lead release hold) an
        # Experimental machine may not demote — zero benefit to Chill at 0 mph, and every demotion
        # there sets up a lurch. A pure demotion GATE: no timer pauses (dwell keeps accumulating,
        # nothing can leak "frozen"), and it releases the instant v_ego rises / the hold disarms —
        # both re-checked from live signals every tick. NOT gated on the stops toggle: this is
        # mode-flap hygiene at 0 mph, not stop machinery. Fail-safe direction only (like A2): it
        # can never enter Experimental, only delay leaving it.
        if v_now < C.STANDSTILL_LATCH_V or self._release_hold:
          self._status = "standstillHold"   # telemetry: the hold is all that keeps Experimental
        # stophold2pnw (A2): standstill-departure hold. Below STANDSTILL_HOLD_V, "the conditions
        # cleared" (typically: the lead crept and slowLead dropped) is NOT sufficient to hand the
        # launch to Chill's MPC — the model must also have agreed GO (shouldStop continuously
        # clear for STOP_CLEAR_HOLD_S). Respects the per-condition "stops" toggle, like the
        # stopIntent fast path. Tagged "stopHold" so field logs show every hold explicitly.
        # Fail-safe direction only: this can never enter Experimental, only delay leaving it.
        elif (v_now < C.STANDSTILL_HOLD_V
              and self._stop_clear_s < C.STOP_CLEAR_HOLD_S
              and bool(signals.get("toggles", {}).get("stops", True))):
          self._status = "stopHold"   # telemetry: the hold is the only thing keeping Experimental
        else:
          self._is_experimental = False
          self._status = "chill"
          self._dwell = 0.0
          self._at_standstill = False   # standstill2pnw: clean slate for the next episode
          self._release_hold = False
          self._ss_close_lead = False
    return self.mode()


# ---------------------------------------------------------------------------
# Phase 2/3 — live wiring. Runs in selfdrived (which publishes the effective
# experimentalMode → both the planner AND the top-right icon follow it).
# Behavior-neutral: experimental_request() returns False whenever CES is
# disabled/non-Tesla, so selfdrived's `manual OR request` == manual == upstream.
# ---------------------------------------------------------------------------

def _toggles_from_params(params) -> dict:
  """Per-condition enables; default ON (the master switch is the real gate)."""
  def gb(k, default=True):
    try:
      return params.get_bool(k)
    except Exception:
      return default
  # ces2core2pnw: "turns" (CESTurns) is the CES2 turn-signal condition — default OFF (study §5.2
  # rule 3: ships dark for the first drives), unlike the four v1 conditions which default ON.
  # decide_active (v1) ignores the key entirely.
  return {"curves": gb("CESCurves"), "stops": gb("CESStops"),
          "low_speed": gb("CESLowSpeed"), "lead": gb("CESLead"),
          "turns": gb("CESTurns", default=False)}


def _signals_from(car_state, lead, model, toggles: dict, map_target_v: float, map_target_dist: float,
                  spd_lim: float = 0.0) -> dict:
  """Build the decision primitives from STOCK messages (carState, radarState.leadOne, modelV2)
  plus the map-curve result (map_target_v/dist) and the OSM speed limit (spd_lim, m/s, for the
  lowSpeed highway gate). Defensive: missing/odd data falls back to 'nothing happening' (stay Chill)."""
  v_ego = float(car_state.vEgo)

  has_lead = bool(getattr(lead, 'status', False))
  lead_vlead = float(getattr(lead, 'vLead', 0.0)) if has_lead else 0.0
  lead_drel = float(getattr(lead, 'dRel', 0.0)) if has_lead else 0.0

  try:
    orz = list(model.orientationRate.z); vx = list(model.velocity.x); tb = list(model.orientationRate.t)
    # ces2-study: the model's trajectory ENDPOINT (position.x[-1], meters ahead) — the signal the
    # graded stop-urgency in the CES2 design keys on (DEC/CEM both use it). Logged from now so the
    # replay acceptance dataset accrues with every drive; NOT used in any decision yet.
    try:
      mdl_end_x = float(model.position.x[-1]) if len(model.position.x) else 0.0
    except Exception:
      mdl_end_x = 0.0
    vis_acc, ttc = vision_curve_lat_accel(orz, vx, tb, v_ego)
    # icbmmapfirst2pnw: lateral accel AT t~0 (yaw_rate*speed at the first model point) — the
    # "currently loaded in a curve" signal for the ICBM start/restore gates (icbm_in_curve).
    lat_now = float(orz[0]) * float(vx[0]) if (orz and vx) else 0.0
  except Exception:
    vis_acc, ttc = 0.0, 10.0
    lat_now = 0.0
  try:
    model_should_stop = bool(model.action.shouldStop)
  except Exception:
    model_should_stop = False
  # ces2core2pnw: lane-change intent (modelV2.meta.laneChangeState != off) — the CES2 TURN
  # condition's "signaling a TURN, not a lane change" test (CEM F2's lane detection, v1 form).
  try:
    lane_change_intent = str(model.meta.laneChangeState) != "off"
  except Exception:
    lane_change_intent = False

  # set speed (openpilot's v_cruise, km/h on carState.vCruise) -> m/s; 255 is the unset sentinel.
  v_set_kph = float(getattr(car_state, 'vCruise', 0.0))
  v_set = v_set_kph * CV.KPH_TO_MS if 0.0 < v_set_kph < C.V_SET_MAX_KPH else 0.0

  return {
    "v_ego": v_ego, "has_lead": has_lead, "lead_vlead": lead_vlead, "lead_drel": lead_drel,
    "blinker": bool(car_state.leftBlinker or car_state.rightBlinker),
    "map_target_v": map_target_v, "map_target_dist": map_target_dist,   # map half (MapTargetVelocities)
    "curve_lat_accel_vision": vis_acc, "time_to_curve": ttc,            # vision fallback
    "mdl_end_x": mdl_end_x,                                             # ces2-study replay dataset
    "lat_accel_now": lat_now,                                           # icbmmapfirst2pnw: in-curve gate
    "model_should_stop": model_should_stop, "toggles": toggles,
    "v_set": v_set,                                                     # accelerate-zone (set-speed gap)
    "spd_lim": float(spd_lim),                                         # OSM speed limit (lowSpeed highway gate)
    "a_ego": float(getattr(car_state, 'aEgo', 0.0)),                   # logged for verification
    "gas": bool(getattr(car_state, 'gasPressed', False)),             # logged for verification
    "brake": bool(getattr(car_state, 'brakePressed', False)),         # icbmrestore2pnw: episode abort
    # ces2core2pnw: CES2-only inputs (v1 decide_active ignores both keys)
    "standstill": bool(getattr(car_state, 'standstill', False)),      # CEM standstill hold
    "lane_change_intent": lane_change_intent,                         # TURN condition lane test
  }


class CESStub:
  """stophold2pnw (C): inert fallback selfdrived installs when CESController CONSTRUCTION raises —
  a CES bug must degrade to stock behavior (never Experimental, never publish), never take
  selfdrived (safety-critical) down. Mirrors the CESController surface selfdrived touches."""

  # greenlead2pnw: selfdrived reads these UNCONDITIONALLY every cycle — without them the stub
  # itself would crash selfdrived with AttributeError (latent since greenlight2pnw shipped).
  green_light = False
  lead_departing = False

  def experimental_request(self, car_state, sm) -> bool:
    return False

  def enabled(self) -> bool:
    return False

  def status(self) -> str:
    return "chill"

  def log_take_control_alert(self, payload) -> None:
    # takecontrol2pnw: no event log writer exists in this fallback (CESController construction
    # already failed) -- consistent with every other telemetry method missing here, this is a no-op.
    pass

  _mads_resume_warned = False

  def log_mads_resume(self, payload) -> None:
    # madsresume2pnw: this method MUST exist here -- selfdrived calls it unconditionally, so without
    # it the stub would crash selfdrived with AttributeError on the first brake. But unlike
    # log_take_control_alert above it does NOT stay a silent no-op (Fable S3): CESController
    # construction having failed means the auto-resume's ONLY forensic channel is gone for the whole
    # drive, and "no madsResume records" is otherwise indistinguishable from "the brain never armed".
    # Warn once per process rather than per record.
    if not CESStub._mads_resume_warned:
      CESStub._mads_resume_warned = True
      try:
        cloudlog.error("madsresume2pnw: CES is a stub (construction failed) -- auto-resume records are NOT being written this drive")
      except Exception:
        pass


class CESController:
  """Live wrapper used by selfdrived. Owns the state machine + ~1 Hz param refresh + the 3-state
  button (CESButtonState: 0=CES, 1=forced Chill, 2=forced Experimental) + the map-curve read.
  Gated on openpilotLongitudinalControl (NOT brand — available on every car, like the stock
  Experimental toggle). experimental_request() returns False when disabled → behavior-neutral."""
  def __init__(self, CP, params=None):
    import platform
    from openpilot.common.params import Params
    self.CP = CP
    self.params = params or Params()
    # pfeiferj mapd writes MapTargetVelocities/LastGPSPosition to the in-memory param store
    try:
      self.mem_params = Params("/dev/shm/params") if platform.system() != "Darwin" else self.params
    except Exception:
      self.mem_params = None
    # light-ces-gentle: the gentle profile is now USER-SELECTED via CESMode (1=Light), NOT gated on
    # carFingerprint. Light = longer dwell + VTSC owns curves (no curve->Experimental) on ANY car;
    # Standard = default tune. _read_params() rebuilds the state machine if the mode changes at runtime.
    self._mode = C.CES_MODE_OFF
    self._gentle = False
    self._sm = ConditionalExperimentalSwitching()
    # ces2core2pnw: the CES2 core runs EVERY tick (pure functions on the same sig dict — cheap).
    # Flag OFF (default): v1 decides, CES2 shadows; its would-be mode/reason/urgency + a cumulative
    # divergence-edge counter go into every ces_events record (ces2Mode/ces2Reason/ces2Urgency/
    # ces2Div). Flag ON (Ces2Core=1): CES2 decides and records mark ces2Live=true.
    self._ces2 = Ces2Core()
    self._ces2_live = False
    self._ces2_mode = None
    self._ces2_reason = None
    self._ces2_urg = 0.0
    self._ces2_div = DivergenceCounter()  # cumulative v1-vs-CES2 divergence EDGES this session
    self._enabled = False
    self._button = C.BTN_CES
    self._toggles = {"curves": True, "stops": True, "low_speed": True, "lead": True}
    self._map_targets = []          # cached MapTargetVelocities (refreshed ~1 Hz)
    # icbmcurv2pnw: measured curvature of the map polyline ahead. TELEMETRY ONLY.
    # icbmKN == 0 means UNMEASURABLE, NOT straight -- k == 0.0 is ambiguous between the two
    # (polyline_curvature docstring). Any gate built on this later MUST read icbmKN first.
    self._icbm_k = 0.0
    self._icbm_k_dist = 0.0
    self._icbm_k_v = 0.0
    self._icbm_k_n = 0
    self._icbm_k_ahead = True
    self._cur_lat = self._cur_lon = self._cur_bearing = None
    self._car_gps = None       # cargps2pnw: last CarGps dict from the ford carstate (None on Tesla)
    self._gps_src = None       # gpssel2pnw: LastGPSPosition "src" -- which receiver lat/lon/bearing came from
    self._gps_fix_ts = None    # gpslag2pnw: LastGPSPosition "fix_ts" (monotonic time the fix was valid)
    self._icbm_gps_age = None  # gpslag2pnw: age (s) of the fix ICBM projected on its last tick; None = no fix / ICBM idle
    # steerpower2pnw I3 review fix: bounded (wall_time, bearing, gps_valid) history, appended once per
    # _read_map() refresh (~1 Hz) -- see _nearest_bearing() above. Lets a steerEvent record look up
    # the bearing at its actual saturation ONSET instead of the live value at emit time.
    self._bearing_hist: deque = deque(maxlen=_BEARING_HIST_MAXLEN)
    self._vtsc_cap = self._vtsc_state = None
    # vtsctele2pnw: VTSC penalty components actually applied (from VTSCStatus) — logging only
    self._vtsc_pen = self._vtsc_pitch = None
    self._vtsc_dir = ""
    self._vtsc_tele: dict = {}   # curve-source forensics from VTSCStatus
    self._sa_tele: dict = {}     # satele2pnw: speedadjust forensics from SpeedAdjustStatus
    # lanecenter2pnw telemetry: lane-centering trim status (from LaneCenterStatus, published by
    # controlsd — see selfdrive/controls/lib/lane_centering.py) — logging only, never gates control
    # here. Defaulted so a missing/never-published param (feature disabled, or before the first
    # controlsd tick lands) reads as a clean "no data" row rather than an AttributeError.
    self._lc_corr = None     # applied correction (1/m)
    self._lc_act = False     # actively nudging this tick
    self._lc_gate = None     # why not acting (or "ok")
    self._lc_err = None      # center error at lookahead (m)
    self._lc_p1 = self._lc_p2 = None    # laneLineProbs[1]/[2]
    self._lc_s1 = self._lc_s2 = None    # laneLineStds[1]/[2] (m)
    self._lc_ystd = None     # E2E path position.yStd at lookahead (m)
    self._lc_w = None        # apparent lane width at lookahead (m)
    # lcroc2pnw: cumulative count of ticks the lane-centering correction_roc growth cap actually
    # clipped. CUMULATIVE, not per-tick — diff two consecutive records to see how often the cap bit
    # in that second. A per-tick boolean would be invisible at this 1 Hz sampling of a 5 Hz publish.
    self._lc_lim_n = None
    self._lc_spd_a = None      # lcramp2pnw: speed-authority multiplier actually applied
    # steerlimit-log2pnw telemetry: steering-limit status (from SteerLimitStatus, published by
    # controlsd — see docs/STEERING-LIMITS.md) — logging only, never gates control here. Defaulted
    # so a missing/never-published param (before the first controlsd tick lands) reads as a clean
    # "no data" row rather than an AttributeError.
    self._sl_curv_lim = False    # curvature_limited: clip_curvature's ISO jerk/accel/max-curv ceiling bound this tick
    self._sl_safe_lim = False    # steer_limited_by_safety: carcontroller/panda had to override the commanded angle
    self._sl_ang_des = None      # commanded steering angle (deg)
    self._sl_ang_act = None      # measured steering angle (deg) -- also already logged as strAng
    self._sl_ang_err = None      # angDes - angActual (deg)
    self._sl_sat = False         # LatControlAngle's own time-integrated saturation flag
    self._sl_lat_dem = None      # pre-clip lateral accel demand (m/s^2)
    self._sl_lat_max = None      # live ISO lateral-accel ceiling this tick (m/s^2), varies with roll
    self._sl_curv_max = None     # live "how tight a curve could we even ask for right now" (1/m)
    # fordkappalog2pnw: commanded vs achieved curvature (1/m), Ford wire convention (positive=left) —
    # see docs/STEERING-LIMITS.md "Ford curvature interface" and the kCmd/kActl/kErr comment block in
    # controlsd.py for the full derivation. Pure observation, same defaulting rationale as the sl*
    # fields above.
    self._sl_k_cmd = None        # commanded curvature this tick (-self.desired_curvature)
    self._sl_k_actl = None       # achieved/measured curvature, derived from CS.yawRate / vEgo
    self._sl_k_err = None        # kCmd - kActl -- sustained large + hands-off = real saturation
    # coopsteer-shadow2pnw: the SHADOW torque-nudge fragment (cp*) riding in the same SteerLimitStatus
    # dict. None == not published / no capability (Ford); on the Raven cpWhy is always a reason string.
    self._cp_off = None          # deg the nudge WOULD have added (never applied this round)
    self._cp_tgt = None          # raw pre-washout target (deg)
    self._cp_cap = None          # magnitude bound in force (deg)
    self._cp_why = None          # reason code (coopsteer_pnw.REASON_*) or "error"
    self._cp_tq = None           # CS.steeringTorque (Nm) -- THE sign-question input
    self._cp_rate = None         # CS.steeringRateDeg -- THE sign-question response
    self._cp_cmd = None          # angle the wire WOULD have carried (cmd + offset)
    # steertele2pnw: capability-analysis additions — see the steer_limit_status comment block in
    # controlsd.py for the full derivation of each. Same defaulting rationale as the sl* fields above.
    self._sl_lat_active = False  # CC.latActive this tick -- False means angDes/angAct froze to manual steering, not an openpilot capability signal
    self._sl_ang_sat = False     # un-fused angle-only saturation half (curvLim already isolates the curvature half of the fused "sat" flag)
    self._speed_limit = 0.0         # OSM speed limit (m/s, 0 = none) from mapd
    # mapd220-2pnw PHASE 1: mapd v2.2.0 mapdOut fields (@24/@26), bridged via mem params
    # (MapHighwayClass/MapConditionalSpeedLimit — see mapd_configd.py). PURE OBSERVATION: logged
    # only, never gates any curve/speed-limit decision here. See docs/MAPD-V220-UPGRADE.md.
    self._hwy_class = None          # HighwayClass enum name, e.g. "motorway" (None = no data yet)
    self._cond_spd_lim = ""         # raw OSM maxspeed:conditional text; "" = none
    # waysel2pnw (PURE OBSERVATION): how confident mapd is about WHICH way we are on, and how far off
    # its centreline. Answers "was the map even looking at our road?" when a curve target is absurd.
    self._way_sel = None            # current / predicted / possible / extended / fail (None = no data)
    self._way_off = None            # metres from the selected way's centreline (None = no data)
    self._frame = 0
    # telemetry / logging (display + diagnostics only — never gates control)
    self._last_mode = "off"         # last logged mode: off / chill / experimental
    self._tele_last = 0.0           # monotonic stamp of last CESStatus publish
    self._tick_last = 0.0           # monotonic stamp of last breadcrumb tick
    self._steer_tick_last = 0.0     # cessteerlog2pnw: monotonic stamp of last CES-off steer breadcrumb
    self._steer_event_seen_id = None  # steerevent2pnw: last SteerEvent.evId already appended (edge dedup;
                                       #   evId is now a per-process-salted STRING, see I2 review fix)
    self._steer_event_frame = 0       # steerevent2pnw B1: call counter -- throttles the mem-param GET
                                       #   itself to ~5 Hz instead of every ~100 Hz call
    self._steer_event_raw_last = None  # steerevent2pnw B1: last raw SteerEvent bytes seen -- skip
                                        #   json.loads entirely when unchanged since the last GET
    self._last_decide_t = None      # monotonic stamp of last state-machine step (for real dt)
    self._bs_l = False              # bsm2pnw: last-seen blind-spot booleans (telemetry only —
    self._bs_r = False              #   proves BSM liveness in ces_events; never gates control here)
    self._event_log_ok = False      # persistent "each adoption" trail (CES_EVENT_LOG)
    self._append_fail = 0           # stophold2pnw (C): consecutive _append_event failures (0 = healthy)
    try:
      os.makedirs(os.path.dirname(CES_EVENT_LOG), exist_ok=True)
      self._event_log_ok = True
    except Exception:
      self._event_log_ok = False
    # stophold2pnw (D): car identity in every record — `shadow` stopped being a car discriminator
    # the day Alpha-Long became an A/B switch on the Lightning (2026-07-12 session misattribution).
    self._car = str(getattr(CP, 'carFingerprint', '') or '') if CP is not None else ''
    # capability view (driver directive 2026-07-11: check CAPABILITIES, never fingerprints here —
    # pnw_vehicle.PnwVehicle is the one place that maps cars to features):
    #   _long_ok -> openpilot owns longitudinal (planner actuation)
    #   _shadow  -> CES runs shadow with ICBM as the actuator (stock-ACC buttons, no op-long)
    veh = PnwVehicle(CP)
    self._veh = veh                        # curveslow-lightning: per-car curve-speed penalty (ICBM apex)
    self._rain_err_t = None                # silentexc3pnw: monotonic time of the last logged RainMode push failure
    self._rain_err_n = 0                   # silentexc3pnw: RainMode push failures since that log line
    self._long_ok = veh.op_long
    self._shadow = veh.ces_shadow
    # icbm2pnw: latched driver set speed while a curve cap is active (see icbm_curve_target), a
    # publish throttle for the IcbmTarget mem-param heartbeat, and the last published target +
    # stock-ACC readings for the ces_events closed-loop trace.
    self._icbm_ceiling = None
    self._icbm_last_pub = 0.0
    self._icbm_last_target = None
    self._icbm_src = None                  # curveslow-lightning: "map"/"vis"/None for the drive log
    # curvedbtel2pnw section 3.4: the DISTANCE of the candidate _icbm_src names, latched beside it so
    # ces_events' mapLat/mapLon resolve that point and not CES's nearer 10 s one. None unless the
    # source is an actual map point ("map"/"far") -- vision/gpsHold/restore have no map coordinate.
    self._icbm_cand_d = None
    # icbmrestore2pnw: the cap->clear->restore episode machine + the current direction for telemetry
    self._icbm_ep = IcbmEpisode()
    self._icbm_dir = None                  # "dec" while capping, "inc" while restoring, None idle
    # curvefloor2pnw: debounced posted limit currently backing the ICBM floor (0.0 = no floor), and
    # whether the floor actually RAISED the target this tick. Both telemetry-visible (icbmFlr/
    # icbmFlrHit) so a drive can show the floor working instead of leaving it to inference --
    # the same trap waysel2pnw fell into when its fields never reached ces_events.
    self._icbm_floor_lim = 0.0
    self._icbm_floor_pend = None   # (candidate_limit, first_seen) while a RISE settles
    self._icbm_rcap_state = None   # icbmrestorecap2pnw: icbm_restore_limit carry-over
    self._icbm_rcap = 0.0          # icbmrestorecap2pnw: the restore cap in force (m/s), 0 = none
    self._icbm_floor_hit = False
    # icbmconsist2pnw: the POINT-MATCHED polyline reading beside mapd's own target -- telemetry only,
    # nothing reads these for control. icbmKAtGap is the load-bearing one: it says how far the nearest
    # measurable triplet fell from mapd's point, i.e. whether comparing them is legitimate at all.
    self._icbm_k_at = 0.0
    self._icbm_k_at_d = 0.0
    self._icbm_k_at_n = 0
    self._icbm_k_at_gap = 0.0
    self._icbm_err_last = -1e9      # rule-2 throttle for the _icbm_step failure log below
    self._stock_set = 0.0
    self._stock_on = False
    # pullaway2pnw: stateful evidence for the below-floor lead-pull-away exception (monotonic
    # dRel rise + model-stop recency). Feeds sig["lead_opening"]; pure logic stays in decide_active.
    self._pullaway_trk = PullAwayTracker()
    # greenlight2pnw/greenlead2pnw: ALWAYS-ON standstill dings (driver decision 2026-07-12: no
    # toggle). Runs every cycle BEFORE the CES-enabled gate so it works with CESMode off too.
    # selfdrived reads .green_light / .lead_departing (each True for exactly one cycle per firing)
    # and raises the matching alert event. _gl_ev_pending is the one-shot "glEv" marker the next
    # ces_events record consumes (records are ~1 Hz, firings are one 100 Hz tick — a latch, not a
    # per-tick flag, or every event would be invisible in the breadcrumb).
    self._gl = GreenLightDetector()
    self._gl_last_t = None
    self.green_light = False
    self.lead_departing = False
    self._gl_ev_pending = None
    # icbmmapfirst2pnw: start-gate telemetry — WHY a would-be new episode was suppressed this tick
    # ("inCurve" / "visCovered" / "visLate" / None) + the map coverage reach (m; 0 = mapd blind/dead,
    # the mapd-liveness evidence for the field logs). Display/log only — never gates control here.
    self._icbm_gate = None
    self._icbm_map_reach = None
    # curvelead2pnw: lead-continuity clock + this tick's lead-pace / map-claim telemetry (_curvelead_note).
    self._icbm_lead_trk = IcbmLeadTrack()
    _curvelead_clear(self)
    # curvedbtel2pnw (CURVEDB2PNW.md sections 3.1-3.6): the 100 Hz curvature/disqualifier accumulator
    # every ces_events record drains, plus the two per-tick samples that go with it. TELEMETRY ONLY --
    # no control path reads any of this. See CurvePeak's docstring for why it cannot live in a daemon.
    self._curve_peak = CurvePeak()
    self._pose_k = None             # section 3.1: livePose-derived achieved curvature, this tick (signed)
    self._str_tq = None             # section 3.6: driver steering torque (per-car scale), this tick
    self._curve_err_t = None        # rule-2 throttle for the _curve_peak_step failure log
    self._curve_err_n = 0           # failures since that log line

  def _set_mode(self, mode: int):
    """Apply a CESMode change: pick the gentle vs default dwell and (re)build the state machine only
    when the gentle flag actually flips, so we don't reset the dwell every ~1 Hz read."""
    gentle = C.ces_is_gentle(mode)
    if mode != self._mode or gentle != self._gentle:
      if gentle != self._gentle:
        if gentle:
          self._sm = ConditionalExperimentalSwitching(C.GENTLE_EXP_MIN_DWELL_S, C.GENTLE_CHILL_MIN_DWELL_S)
          self._ces2 = Ces2Core(C.GENTLE_EXP_MIN_DWELL_S, C.GENTLE_CHILL_MIN_DWELL_S)
        else:
          self._sm = ConditionalExperimentalSwitching()
          self._ces2 = Ces2Core()
      self._mode = mode
      self._gentle = gentle

  def _read_params(self):
    if self._frame % max(1, int(1.0 / DT_CTRL)) == 0:   # ~1 Hz (selfdrived steps at 100 Hz / DT_CTRL)
      # silentexc3pnw: logs its own read failures. cesmodehold2pnw: holds the last good mode through a failed read for
      # C.CES_MODE_HOLD_S. It never raises (every read, the hold and every log line are guarded inside it), so the
      # silent `except Exception: mode = CES_MODE_OFF` that sat here could not run and is gone; selfdrived's own
      # guard around experimental_request() still backstops it.
      self._set_mode(C.read_ces_mode(self.params, who="CES"))
      # rain2pnw: push the live wet-weather tier into the capability view (used by the ICBM curve
      # target below; applies in shadow too). Defensive — never let a param hiccup break _read_params.
      try:
        self._veh.set_rain_tier(self.params.get("RainMode", return_default=True))
      except Exception as e:
        # silentexc3pnw (Rule 2): was `except Exception: pass`. Fallback unchanged: the capability view keeps its last
        # rain tier, so ICBM's curve targets keep the old wet-weather margin. An unset RainMode reads its "0" default
        # and set_rain_tier() maps None / garbage to 0 itself, so neither reaches here; UnknownKeyName on a
        # params_keys.h / params_pyx.so mismatch or a code defect does. Logged once, then at most once per
        # CURVELEAD_ERR_LOG_S (60 s) with a count, own state (this runs ~1 Hz inside selfdrived's guarded call).
        self._rain_err_n += 1
        now = time.monotonic()
        if self._rain_err_t is None or now - self._rain_err_t >= CURVELEAD_ERR_LOG_S:
          cloudlog.exception(f"ces_pnw: RainMode unreadable ({type(e).__name__}) -- the rain tier stays at its last value " +
                             f"while this lasts ({self._rain_err_n} failure(s) since the last log)")
          self._rain_err_t = now
          self._rain_err_n = 0
      # CES is meaningful only when openpilot owns longitudinal (same gate as ExperimentalMode) —
      # except in Lightning shadow mode, where the pipeline runs for telemetry/display only.
      self._enabled = (self._long_ok or self._shadow) and C.ces_enabled(self._mode)
      if self._enabled:
        self._toggles = _toggles_from_params(self.params)
        try:
          self._button = int(self.params.get("CESButtonState", return_default=True) or 0)
        except Exception:
          self._button = C.BTN_CES
        # ces2core2pnw: CES2 live flag (default OFF = shadow-only; any read failure -> OFF)
        try:
          self._ces2_live = bool(self.params.get_bool("Ces2Core"))
        except Exception:
          self._ces2_live = False
        self._read_map()
    self._frame += 1

  def _read_map(self):
    """Refresh map-curve inputs + GPS + OSM speed limit from the pfeiferj mem params (defensive — any
    failure => no map curve, vision fallback still works). GPS + speed limit are read regardless of
    the curves toggle because the event log wants them at all times."""
    # icbmcurv2pnw: clear BEFORE anything can return, so icbmK* always describe the polyline cached
    # on THIS refresh or nothing at all. Measured further down, once _map_targets and the GPS fix are
    # both fresh. Resetting next to the measurement instead would let the mem_params early-return
    # below clear _map_targets while icbmK* kept the PREVIOUS refresh's numbers -- a stale reading
    # that is indistinguishable from a live one, which is the exact failure mode this whole change
    # exists to remove.
    self._icbm_k = 0.0
    self._icbm_k_dist = 0.0
    self._icbm_k_v = 0.0
    self._icbm_k_n = 0
    self._icbm_k_ahead = True
    if self.mem_params is None:
      self._map_targets = []
      return
    # map curve targets — only when the curve condition is enabled
    if self._toggles.get("curves", True):
      try:
        self._map_targets = self.mem_params.get("MapTargetVelocities", return_default=True) or []
      except Exception:
        self._map_targets = []
    else:
      self._map_targets = []
    # GPS (lat/lon/bearing) — always (map-curve distance + logging)
    try:
      pos = self.mem_params.get("LastGPSPosition", return_default=True)
      if isinstance(pos, (bytes, str)):
        pos = json.loads(pos)
      self._cur_lat = float(pos["latitude"]); self._cur_lon = float(pos["longitude"])
      self._cur_bearing = float(pos.get("bearing", 0.0))
      self._gps_src = pos.get("src")   # gpssel2pnw: "car" | "device"; None = written before gpsfix2pnw
      self._gps_fix_ts = float(pos["fix_ts"]) if pos.get("fix_ts") is not None else None   # gpslag2pnw
    except Exception:
      self._cur_lat = self._cur_lon = self._cur_bearing = None
      self._gps_src = self._gps_fix_ts = None
    # icbmcurv2pnw: measure the polyline geometry ICBM is about to act on. TELEMETRY ONLY -- no
    # control path reads these, and polyline_curvature() is pure and documented never to raise.
    # WHY IT LIVES HERE AND NOT IN VTSC: vtsc_controller.py:283 runs the same call, but its own
    # guard comment (:278) says the measurement "exists only with CESMode>0 AND op-long AND
    # VtscMapCurves=1". ICBM is the STOCK-ACC path -- op-long is False by definition on this truck --
    # so that measurement never runs here, and `mapKN` reads 0 on every Lightning tick. That is why
    # a mapd target of 20.3 m/s on a straight 70 mph road (2026-09-06 13:15) had nothing to
    # contradict it. Measuring on THIS path is the prerequisite for any consistency check.
    # DELIBERATELY NOT GATED to the Lightning: no fingerprint branches in feature code (the
    # capability-view rule), and on the Tesla -- where VTSC measures the same polyline with the same
    # A_LAT_TARGET (2.5 in both the DEFAULT and GENTLE profiles) -- icbmK/mapK on the same tick are a
    # free equality check on this wiring, from real drive data, at no risk.
    if self._map_targets and self._cur_lat is not None and self._cur_lon is not None:
      k, kd, kv, kn, kahead = polyline_curvature(self._map_targets, self._cur_lat, self._cur_lon,
                                                 MAP_SOURCE_HORIZON_M, VTSC_A_LAT, self._cur_bearing)
      self._icbm_k = float(k)
      self._icbm_k_dist = float(kd)
      # isfinite, not `== inf`: a NaN would serialise as a bare `NaN` token, which is not valid JSON
      # and would lose the WHOLE tick's snapshot, not just this field (same trap as mapKV).
      # CONSEQUENCE, and it bites the obvious way round: icbmKV == 0.0 encodes "NO FINITE BOUND"
      # (v_safe is inf on a straight road) as well as "unmeasurable". It NEVER means "0 m/s". Compare
      # in curvature space (icbmK) -- a speed comparison against icbmKV fails on the straightest
      # possible road, which is backwards. See docs/pnw/ICBMCURV2PNW.md section 5.
      self._icbm_k_v = float(kv) if math.isfinite(kv) else 0.0
      self._icbm_k_n = int(kn)
      self._icbm_k_ahead = bool(kahead)
    # cargps2pnw: the CAR's own GPS fix (Ford only), published by the ford carstate from the GWM's
    # APIMGPS messages at 1 Hz. Logged ALONGSIDE the device's own fix above, never instead of it --
    # nothing here or downstream consumes it, this is a side-by-side comparison channel so a drive
    # can show whether the truck's roof antenna actually beats the device's windshield view when it
    # matters (cold start, urban canyon). EMPTY ON THE TESLA by construction: no Ford carstate, no
    # publisher, so the key never appears and `car_gps` logs as None.
    try:
      cg = self.mem_params.get("CarGps", return_default=True)
      if isinstance(cg, (bytes, str)):
        cg = json.loads(cg)
      self._car_gps = cg if isinstance(cg, dict) and "lat" in cg else None
    except Exception:
      self._car_gps = None
    # OSM speed limit (m/s; 0 = none) — for the coarse highway guess in the log
    try:
      sl = self.mem_params.get("MapSpeedLimit", return_default=True)
      self._speed_limit = float(sl) if sl not in (None, "", b"") else 0.0
    except Exception:
      self._speed_limit = 0.0
    # mapd220-2pnw PHASE 1: mapd v2.2.0 highwayClass/conditionalSpeedLimit — logging only (see
    # _event_record). Same cross-process mem-param read pattern as MapSpeedLimit just above.
    try:
      hc = self.mem_params.get("MapHighwayClass", return_default=True)
      self._hwy_class = str(hc) if hc not in (None, "", b"") else None
    except Exception:
      self._hwy_class = None
    try:
      csl = self.mem_params.get("MapConditionalSpeedLimit", return_default=True)
      self._cond_spd_lim = str(csl) if csl not in (None, b"") else ""
    except Exception:
      self._cond_spd_lim = ""
    # waysel2pnw: same cross-process mem-param pattern. Both stay None when mapd is DEAD (the bridge
    # self-clears them) -- but note they do NOT protect against an OLD mapd: capnp's default for
    # waySelectionType is ordinal 0, which is `current`, so a binary that never sets the field would
    # log a confident "current" rather than nothing. Only reachable with a pre-v2.0.0 binary behind
    # /data/mapd/.override; the pin is v2.3.1 and mapd has set it unconditionally since v2.0.0.
    try:
      ws = self.mem_params.get("MapWaySel", return_default=True)
      self._way_sel = str(ws) if ws not in (None, "", b"") else None
    except Exception:
      self._way_sel = None
    try:
      wo = self.mem_params.get("MapWayOffset", return_default=True)
      self._way_off = float(wo) if wo not in (None, "", b"") else None
    except Exception:
      self._way_off = None
    # VTSC applied cap + state — logging only (see _event_record)
    try:
      vt = self.mem_params.get("VTSCStatus", return_default=True)
      if isinstance(vt, (bytes, str)):
        vt = json.loads(vt)
      self._vtsc_cap = round(float(vt["cap"]), 1) if vt.get("engaged") else None
      self._vtsc_state = vt.get("state")
      # vtsctele2pnw: penalty components VTSC actually applied this cycle (Lightning hump penalty
      # m/s, road pitch rad it used, apex turn direction "L"/"R"/"") — so over/under-slow curve
      # forensics read the real inputs instead of inferring descent/left multipliers after the fact.
      pen, pitch = vt.get("pen"), vt.get("pitch")
      self._vtsc_pen = round(float(pen), 2) if pen is not None else None
      self._vtsc_pitch = round(float(pitch), 4) if pitch is not None else None
      self._vtsc_dir = str(vt.get("dir") or "")
      # curvefloor2pnw / mapcurv2pnw: WHY each curve source did or didn't bind. Without ingesting
      # these here they never leave /dev/shm -- VTSCStatus is volatile and overwritten at 5 Hz, and
      # this cherry-picking read was the ONLY consumer, so the fields published by 62a51a4772 were
      # being computed and thrown away. That made the drive-replay they exist for impossible.
      # waysel2pnw: apexCurvature/apexDist/vCurveSafe are the FINAL post-fold values that actually set
      # the cap -- they were already being published here and discarded by this very list, so the cap's
      # own curvature had to be back-computed from the cap. curveWin/rsnMap/rsnVis say which source won.
      for k in VTSC_TELE_KEYS:
        self._vtsc_tele[k] = vt.get(k)
    except Exception:
      self._vtsc_cap = self._vtsc_state = None
      self._vtsc_pen = self._vtsc_pitch = None
      self._vtsc_dir = ""
      self._vtsc_tele = {}
    # satele2pnw: speedadjust (police cap + limit-drop trim) internals — logging only. Same volatile
    # /dev/shm channel and same cherry-pick requirement as VTSCStatus above: without this read the
    # fields never leave the mem-param. Prefixed "sa" so they cannot collide with VTSC's keys in the
    # flattened tick record. Driver directive 2026-08-21 (the undiagnosable stuck-at-47-mph episode).
    try:
      st = self.mem_params.get("SpeedAdjustStatus", return_default=True)
      if isinstance(st, (bytes, str)):
        st = json.loads(st)
      # Fable review 2026-08-21: this list MUST cover every key _publish_status() emits, or the
      # missing ones evaporate in /dev/shm -- the exact trap satele2pnw's own commit message named,
      # and it had already caught vCruise + polKey (published, never picked up, while
      # params_keys.h advertised polKey as reaching ces_events).
      self._sa_tele = {"sa" + k[0].upper() + k[1:]: st.get(k) for k in SA_TELE_KEYS}
    except Exception:
      self._sa_tele = {}
    # lanecenter2pnw telemetry: lane-centering trim status — logging only (see _event_record).
    # Same cross-process read as VTSCStatus just above: controlsd (100 Hz) publishes to
    # /dev/shm/params at ~5 Hz, this reads it at ~1 Hz. Fully defensive — any missing key, wrong
    # type, or malformed JSON degrades to the same None/"off" defaults set in __init__ rather than
    # raising; a stale telemetry read can never affect CES's own decisions (this method's output is
    # display/log-only throughout).
    try:
      lc = self.mem_params.get("LaneCenterStatus", return_default=True)
      if isinstance(lc, (bytes, str)):
        lc = json.loads(lc)
      # Before controlsd's first publish (feature/CES just started) the key reads back as None; treat
      # anything that isn't a dict as an empty "no data yet" row so every field cleanly defaults to
      # None below, instead of throwing an AttributeError into the except once per second until live.
      if not isinstance(lc, dict):
        lc = {}
      corr = lc.get("corr")
      self._lc_corr = round(float(corr), 5) if corr is not None else None
      self._lc_act = bool(lc.get("act", False))
      self._lc_gate = str(lc.get("gate")) if lc.get("gate") is not None else None
      err = lc.get("err")
      self._lc_err = round(float(err), 2) if err is not None else None
      p1, p2 = lc.get("p1"), lc.get("p2")
      self._lc_p1 = round(float(p1), 2) if p1 is not None else None
      self._lc_p2 = round(float(p2), 2) if p2 is not None else None
      s1, s2 = lc.get("s1"), lc.get("s2")
      self._lc_s1 = round(float(s1), 2) if s1 is not None else None
      self._lc_s2 = round(float(s2), 2) if s2 is not None else None
      ystd = lc.get("yStd")
      self._lc_ystd = round(float(ystd), 2) if ystd is not None else None
      w = lc.get("w")
      self._lc_w = round(float(w), 2) if w is not None else None
      lim_n = lc.get("limN")
      self._lc_lim_n = int(lim_n) if lim_n is not None else None
      # lcramp2pnw (Fable A1 2026-09-05): spdA was published to LaneCenterStatus but never
      # cherry-picked here, so the ramp was INVISIBLE in ces_events -- the same trap as
      # [[vtscstatus-telemetry-not-logged]]. Without it a drive cannot show whether the ramp
      # engaged at the moment of a complaint, which is the whole reason it was added.
      spd_a = lc.get("spdA")
      self._lc_spd_a = round(float(spd_a), 3) if spd_a is not None else None
    except Exception:
      self._lc_corr = self._lc_err = None
      self._lc_p1 = self._lc_p2 = self._lc_s1 = self._lc_s2 = None
      self._lc_ystd = self._lc_w = None
      self._lc_lim_n = None
      self._lc_spd_a = None
      self._lc_act = False
      self._lc_gate = None
    # steerlimit-log2pnw telemetry: steering-limit status — logging only (see _event_record). Same
    # cross-process read as LaneCenterStatus just above: controlsd (100 Hz) publishes to
    # /dev/shm/params at ~5 Hz, this reads it at ~1 Hz. Fully defensive — any missing key, wrong
    # type, or malformed JSON degrades to the same None/False defaults set in __init__ rather than
    # raising; a stale telemetry read can never affect CES's own decisions (this method's output is
    # display/log-only throughout). See docs/STEERING-LIMITS.md.
    try:
      sl = self.mem_params.get("SteerLimitStatus", return_default=True)
      if isinstance(sl, (bytes, str)):
        sl = json.loads(sl)
      # Before controlsd's first publish the key reads back as None; treat anything that isn't a dict
      # as an empty "no data yet" row so every field cleanly defaults below, same pattern as lc above.
      if not isinstance(sl, dict):
        sl = {}
      self._sl_curv_lim = bool(sl.get("curvLim", False))
      self._sl_safe_lim = bool(sl.get("safeLim", False))
      ang_des = sl.get("angDes")
      self._sl_ang_des = round(float(ang_des), 2) if ang_des is not None else None
      ang_act = sl.get("angAct")
      self._sl_ang_act = round(float(ang_act), 2) if ang_act is not None else None
      ang_err = sl.get("angErr")
      self._sl_ang_err = round(float(ang_err), 2) if ang_err is not None else None
      self._sl_sat = bool(sl.get("sat", False))
      lat_dem = sl.get("latDem")
      self._sl_lat_dem = round(float(lat_dem), 3) if lat_dem is not None else None
      lat_max = sl.get("latMax")
      self._sl_lat_max = round(float(lat_max), 3) if lat_max is not None else None
      curv_max = sl.get("curvMax")
      self._sl_curv_max = round(float(curv_max), 5) if curv_max is not None else None
      # fordkappalog2pnw: commanded vs achieved curvature — same read/default pattern as the sl*
      # fields directly above, same dict, no separate publish/read cycle.
      k_cmd = sl.get("kCmd")
      self._sl_k_cmd = round(float(k_cmd), 6) if k_cmd is not None else None
      k_actl = sl.get("kActl")
      self._sl_k_actl = round(float(k_actl), 6) if k_actl is not None else None
      k_err = sl.get("kErr")
      self._sl_k_err = round(float(k_err), 6) if k_err is not None else None
      # steertele2pnw: same defensive get/default pattern as the sl* fields above, same dict, no
      # separate publish/read cycle.
      self._sl_lat_active = bool(sl.get("latActive", False))
      self._sl_ang_sat = bool(sl.get("angSat", False))
      # coopsteer-shadow2pnw: same dict, same defensive pattern. A missing key reads None -- and on
      # the Raven a None cpWhy therefore means "controlsd never published the fragment", which is
      # exactly the silent-evaporation failure this cherry-pick exists to make visible.
      cp_off = sl.get("cpOff")
      self._cp_off = round(float(cp_off), 3) if cp_off is not None else None
      cp_tgt = sl.get("cpTgt")
      self._cp_tgt = round(float(cp_tgt), 3) if cp_tgt is not None else None
      cp_cap = sl.get("cpCap")
      self._cp_cap = round(float(cp_cap), 2) if cp_cap is not None else None
      cp_why = sl.get("cpWhy")
      self._cp_why = str(cp_why) if cp_why is not None else None
      cp_tq = sl.get("cpTq")
      self._cp_tq = round(float(cp_tq), 3) if cp_tq is not None else None
      cp_rate = sl.get("cpRate")
      self._cp_rate = round(float(cp_rate), 2) if cp_rate is not None else None
      cp_cmd = sl.get("cpCmd")
      self._cp_cmd = round(float(cp_cmd), 3) if cp_cmd is not None else None
    except Exception:
      self._sl_curv_lim = self._sl_safe_lim = self._sl_sat = False
      self._sl_ang_des = self._sl_ang_act = self._sl_ang_err = None
      self._sl_lat_dem = self._sl_lat_max = self._sl_curv_max = None
      self._sl_k_cmd = self._sl_k_actl = self._sl_k_err = None
      self._sl_lat_active = self._sl_ang_sat = False
      self._cp_off = self._cp_tgt = self._cp_cap = self._cp_why = None
      self._cp_tq = self._cp_rate = self._cp_cmd = None
    # steerpower2pnw I3 review fix: append this refresh's (wall_time, bearing, gps_valid) sample to
    # the bounded history — see _nearest_bearing()/_BEARING_HIST_MAXLEN above. gps_valid mirrors the
    # exact "gps" test every record already uses (lat AND lon present); a no-fix sample is still
    # appended (bearing=None, gps_valid=False) so a lookup landing near it correctly resolves to "no
    # heading data at that time" instead of silently falling through to some other sample.
    self._bearing_hist.append((time.time(), self._cur_bearing,   # noqa: TID251 -- wall clock, ~1 Hz
                               self._cur_lat is not None and self._cur_lon is not None))

  def enabled(self) -> bool:
    return self._enabled

  def status(self) -> str:
    return self._sm.status()

  def experimental_request(self, car_state, sm) -> bool:
    """True if CES wants Experimental this cycle. Reads params; advances the state machine.
    Safe to call always — returns False whenever CES is disabled (behavior-neutral)."""
    self._read_params()
    # bsm2pnw: sample the blind-spot booleans every cycle (cheap), so adopt/tick records carry them
    # even while CES is disabled — the point is drive-log evidence that BSM flips with passing cars.
    self._bs_l = bool(getattr(car_state, 'leftBlindspot', False))
    self._bs_r = bool(getattr(car_state, 'rightBlindspot', False))
    # icbm2pnw/lateral telemetry (driver req 2026-07-11 "more good data"): steering angle + driver
    # override per record — quantifies left-pull, curve-tracking failures and override clusters.
    self._str_ang = round(float(getattr(car_state, 'steeringAngleDeg', 0.0)), 1)
    self._str_prs = bool(getattr(car_state, 'steeringPressed', False))
    # icbm2pnw closed-loop trace: the STOCK ACC's reported set speed + engagement — with the
    # published target (icbmT below) this shows every executor tap landing (set stepping down).
    try:
      self._stock_set = round(float(car_state.cruiseState.speed), 2)
      self._stock_on = bool(car_state.cruiseState.enabled)
    except Exception:
      self._stock_set, self._stock_on = 0.0, False
    # curvedbtel2pnw: the 100 Hz curvature/disqualifier accumulator every ces_events record drains.
    # Unconditional (like _steer_event_step below) -- what the road did and whether the driver
    # intervened are steering/localizer facts, not CES decisions. TELEMETRY ONLY; never raises.
    # Placed AFTER the _str_prs sample above (the driver-override bit reads it) and BEFORE the two
    # record writers below, so the record built on this tick includes this tick.
    self._curve_peak_step(car_state, sm)
    # greenlight2pnw: always-on (independent of CESMode/_enabled — display/sound only)
    self._green_light_step(car_state, sm)
    # cessteerlog2pnw: unconditional steer/lane-centering breadcrumb — no-ops once _enabled is True
    # (the normal tick/adopt path below already logs the same fields), so this only ever adds the
    # CES-off records that were previously missing from the log entirely.
    self._steer_log_step(car_state, sm)
    # steerevent2pnw: edge-triggered mirror of controlsd's flight-recorder burst into ces_events —
    # called unconditionally every cycle (NOT gated on C.TICK_S like _steer_log_step above),
    # independent of CESMode/_enabled since the underlying saturation edge is a controlsd/steering
    # fact, not a CES decision. The method throttles its OWN mem-param GET internally to ~5 Hz and
    # dedups on the raw payload before parsing (B1 review fix) — see its docstring.
    self._steer_event_step()
    if not self._enabled:
      if self._last_mode != "off":
        cloudlog.info("CES disabled (master OFF / no openpilot long) -> Chill baseline")
        self._last_mode = "off"
      self._sm.reset()
      self._ces2.reset()               # ces2core2pnw: shadow state resets with the live one
      return False

    # Build the decision signals every cycle while enabled — even in the forced button modes —
    # so the on-screen overlay always reflects what CES sees (curve %, upcoming curve preview).
    sig = None
    try:
      lead = sm['radarState'].leadOne
      model = sm['modelV2']
      v_ego = float(car_state.vEgo)
      mtv, mtd = upcoming_curve(self._map_targets, self._cur_lat, self._cur_lon, v_ego, C.CURVE_MAP_LOOKAHEAD_S)
      # gentle profile: VTSC handles curve speed (smooth, decel-limited), so CES does NOT trip
      # Experimental for curves on the truck — removes the chill<->experimental planner-mode flapping.
      toggles = {**self._toggles, "curves": False} if self._gentle else self._toggles
      sig = _signals_from(car_state, lead, model, toggles, mtv, mtd, self._speed_limit)
      # curvelead2pnw: the model's tightest predicted curvature + horizon reach, for ICBM's lead pacing. Only
      # where ICBM runs (ces_shadow) -- no other car computes or reads it. Own try: a model hiccup must cost
      # lead pacing (None -> refused as "noVis"), not the whole signals dict.
      if self._shadow:
        try:
          sig["vis_k_max"], sig["vis_reach"] = icbm_vision_curvature(
            model.orientationRate.z, model.velocity.x, model.position.x)
        except Exception:
          sig["vis_k_max"], sig["vis_reach"] = None, 0.0
      # icbmalign2pnw: road pitch for the ICBM descent guard — the SAME message/field VTSC reads
      # (carControl.orientationNED[1], rad, < 0 = downhill; selfdrived's SubMaster subscribes
      # carControl). Own inner try: a carControl hiccup must not cost the whole signals dict.
      try:
        ned = sm['carControl'].orientationNED
        sig["pitch"] = float(ned[1]) if len(ned) == 3 else None
      except Exception:
        sig["pitch"] = None
    except Exception:
      sig = None

    # measured loop period — selfdrived steps at ~100 Hz; never assume a fixed DT (was the 5x bug)
    now_t = time.monotonic()
    # pullaway2pnw: advance the pull-away evidence every cycle and inject it into the signals dict
    # BEFORE the decision (decision_telemetry consumes the same dict, so the overlay/why agrees).
    if sig is not None:
      try:
        sig["lead_opening"] = self._pullaway_trk.update(now_t, sig["has_lead"], sig["lead_drel"],
                                                        sig["model_should_stop"])
      except Exception:
        sig["lead_opening"] = False
    dt = (now_t - self._last_decide_t) if self._last_decide_t is not None else DT_CTRL
    self._last_decide_t = now_t
    dt = min(max(dt, 1e-3), 0.5)           # clamp first call / scheduling hiccups

    # ces2core2pnw: advance the CES2 core every tick a sig is available — SHADOW when the flag is
    # OFF (v1 below stays the byte-identical decider), LIVE when Ces2Core=1. Pure functions on the
    # same dict (Ces2Core copies it, never mutates); any CES2 exception degrades to v1.
    ces2_want = None
    if sig is not None and self._button != C.BTN_CHILL:
      try:
        ces2_want = self._ces2.update_decision(sig, dt) == "experimental"
        self._ces2_mode = "experimental" if ces2_want else "chill"
        self._ces2_reason = self._ces2.status()
        self._ces2_urg = self._ces2.urgency
      except Exception:
        ces2_want = None
        self._ces2_mode = self._ces2_reason = None

    if self._button == C.BTN_CHILL:        # forced Chill
      self._sm.reset()
      self._ces2.reset()                   # ces2core2pnw: shadow state resets with the live one
      self._ces2_mode = self._ces2_reason = None
      want = False
    elif self._button == C.BTN_EXP:        # forced full Experimental
      want = True
    elif sig is not None:                  # BTN_CES: condition ladder decides
      # v1 ALWAYS advances (it is the live decider when the flag is OFF, and the reverse-shadow
      # divergence reference when the flag is ON).
      want_v1 = self._sm.update_decision(sig, dt) == "experimental"
      self._ces2_div.update(want_v1, ces2_want)   # divergence EDGES, not per-tick spam
      want = ces2_want if (self._ces2_live and ces2_want is not None) else want_v1
    else:
      want = False

    self._publish_status(sig, want)
    # icbm2pnw: in Lightning shadow mode the CES/planner path never actuates, but the ICBM brain
    # publishes a stock-ACC set-speed target the ford carcontroller executor follows (curve
    # slow-down, dec-only against the driver's own set — see icbm_curve_target). ICBM mirrors the
    # button: ACTIVE only in the CES state, SILENT in forced Chill. This is the driver's kill switch
    # for ICBM, so it must stay reachable in ONE tap from the default boot state (CES) — verified by
    # oplongexp2pnw's select_ces_cycle ordering (CES -> Chill -> Exp-confirm) in exp_button.py.
    # CESButtonState itself still never stores BTN_EXP while op-long is off (exp_button.py forces the
    # landing state back to CES and gates the actual enable behind a separate confirm tap), so forced
    # Exp stays structurally unreachable HERE regardless of the onroad button's own multi-tap UI flow
    # to reach it. Publishing empty in Chill stops the executor.
    if self._shadow:
      self._icbm_step(sig, active=(sig is not None and self._button == C.BTN_CES))
    return want and self._long_ok

  # curvedbtel2pnw: the lateralControlState union members that carry a `saturated` field. The union
  # also has two legacy SCALAR members (desiredLateralJerk, version) that do not, so the membership
  # test -- rather than a try/except around the attribute -- is what keeps this branch free of a
  # per-tick exception that would otherwise flood _curve_peak_step's rule-2 error log.
  _SAT_UNION_MEMBERS = ("angleState", "torqueState", "pidState", "debugState",
                        "lqrStateDEPRECATED", "curvatureStateDEPRECATED")

  def _curve_peak_step(self, car_state, sm) -> None:
    """curvedbtel2pnw sections 3.1 / 3.3 / 3.5: advance the 100 Hz accumulator by one tick.

    TELEMETRY ONLY. Nothing here is read by any control path, and nothing here may raise into
    selfdrived's loop (this process is restart_if_crash=False). Called unconditionally from
    experimental_request BEFORE the `if not self._enabled` gate, because the facts it measures --
    what the road did, what the wheel was asked to do, whether the driver intervened -- are
    steering/localizer facts, not CES decisions, exactly like _steer_event_step's rationale.

    Zero new subscriptions: carState arrives as the argument, and both controlsState and livePose
    are already in selfdrived's SubMaster (selfdrived.py:117/119).

    Rule 2: the failure path is LOGGED, throttled, with a count -- a silently dead accumulator is
    the visK failure (a field logged forever and never computed) that section 3.7 exists to prevent.
    kPeakN drops to 0 in every record while this is broken, so the log and the data agree."""
    try:
      v_ego = getattr(car_state, 'vEgo', None)
      # Achieved, from CAN. DEAD ON THE TESLA by construction (D1) -- tesla/carstate.py never sets
      # ret.yawRate and the Raven party DBC has no yaw signal -- which is why kPoseP exists.
      k_actl = _curvature_from_yaw(getattr(car_state, 'yawRate', None), v_ego)
      # Achieved, from the localizer. Alive on BOTH cars (section 3.1).
      k_pose = _pose_curvature(sm['livePose'], v_ego)
      self._pose_k = k_pose
      # section 3.6: one getattr, per-car scale (Ford +-8 Nm, Tesla +-20.5 Nm).
      str_tq = getattr(car_state, 'steeringTorque', None)
      try:
        self._str_tq = round(float(str_tq), 2) if str_tq is not None and math.isfinite(float(str_tq)) else None
      except (TypeError, ValueError):
        self._str_tq = None

      cs = sm['controlsState']
      k_cmd = float(cs.desiredCurvature)
      if not math.isfinite(k_cmd):
        k_cmd = None

      # section 3.5 disqualifiers, OR'd at control rate wherever the fact is available at control rate.
      dq = 0
      if self._str_prs:                         # already sampled this tick by experimental_request
        dq |= DQ_DRIVER
      if getattr(car_state, 'leftBlinker', False) or getattr(car_state, 'rightBlinker', False):
        dq |= DQ_BLINKER
      # Lane change: modelV2.meta.laneChangeState is the same per-tick source _signals_from already
      # uses. str() is required -- a capnp enum compares False against its own name otherwise
      # ([[capnp-enum-str-trap]]: str(x) is the BARE name, "off", never "LaneChangeState.off").
      try:
        if str(sm['modelV2'].meta.laneChangeState) != "off":
          dq |= DQ_LANECHG
      except (KeyError, AttributeError):
        pass                                    # no model this tick: the lcGate sample below still applies
      if self._lc_gate == "lanechange":         # ~1 Hz sample, OR'd in as a second opinion
        dq |= DQ_LANECHG
      # Saturation, at 100 Hz: lac_log.saturated as controlsd itself computes it. Deliberately NOT
      # taken only from the sl* fields -- those come from SteerLimitStatus, which controlsd publishes
      # at 5 Hz and _read_map() samples at ~1 Hz, and section 3.3's whole argument is that a ~1 Hz
      # sample of a 100 Hz fact aliases. slAngSat in particular is an instantaneous threshold test.
      # The sampled flags are still OR'd in (union of sources, never a substitution): slCurvLim is
      # the curvature half, which lac_log.saturated fuses but SteerLimitStatus isolates.
      lcs = cs.lateralControlState
      w = str(lcs.which())
      if w in self._SAT_UNION_MEMBERS and getattr(lcs, w).saturated:
        dq |= DQ_SAT
      if self._sl_sat or self._sl_ang_sat or self._sl_curv_lim:
        dq |= DQ_SAT

      self._curve_peak.step(k_actl, k_cmd, k_pose, dq)
    except Exception as e:
      # Rule 2: never silent. Throttled to one line per CURVELEAD_ERR_LOG_S with a count, because
      # this runs at 100 Hz and a persistent fault would otherwise flood the log it needs to be seen
      # in. The accumulator simply misses this tick; kPeakN shows how many ticks it did see.
      self._pose_k = None
      self._curve_err_n += 1
      try:
        now = time.monotonic()
        if self._curve_err_t is None or now - self._curve_err_t >= CURVELEAD_ERR_LOG_S:
          n, self._curve_err_t, self._curve_err_n = self._curve_err_n, now, 0
          cloudlog.exception(f"curvedbtel2pnw: _curve_peak_step FAILED ({type(e).__name__}) -- kPeak/kPose/dq are " +
                             f"not accumulating; the curvedb Phase-1 corpus is degraded ({n} failure(s) since the last log)")
      except Exception:
        pass                    # an error handler that can itself raise is worse than none

  def _green_light_step(self, car_state, sm) -> None:
    """greenlight2pnw/greenlead2pnw: advance the pure GreenLightDetector one cycle and latch the
    per-cause alert flags for selfdrived (each True for exactly the firing cycle):
      "green"      -> .green_light    (no lead, path opens: the greenLight alert)
      "lead"       -> .lead_departing (stopped lead pulls away from OUR standstill: leadDeparting)
      "leadMoving" -> telemetry record only, NO alert (driver rule #3: never ding while rolling)
    Best-effort: any message hiccup means 'no ding this cycle', never an exception into
    selfdrived's control loop. Model-based and car-agnostic — reads only vEgo/gasPressed,
    modelV2 (endpoint + shouldStop) and radarState.leadOne; no fingerprints, no CAN, no control
    output."""
    now = time.monotonic()
    dt = (now - self._gl_last_t) if self._gl_last_t is not None else DT_CTRL
    self._gl_last_t = now
    ev = None
    try:
      model = sm['modelV2']
      lead = sm['radarState'].leadOne
      try:
        mdl_end_x = float(model.position.x[-1]) if len(model.position.x) else 0.0
      except Exception:
        mdl_end_x = 0.0
      try:
        should_stop = bool(model.action.shouldStop)
      except Exception:
        should_stop = False
      has_lead = bool(getattr(lead, 'status', False))
      ev = self._gl.update(dt, float(car_state.vEgo), bool(getattr(car_state, 'gasPressed', False)),
                           should_stop, mdl_end_x, has_lead,
                           float(getattr(lead, 'dRel', 0.0)) if has_lead else 0.0,
                           float(getattr(lead, 'vLead', 0.0)) if has_lead else 0.0)
      if ev is not None:
        cloudlog.info("greenlead2pnw: %s mdlEndX=%.1f lead=%s", ev, mdl_end_x, has_lead)
        self._gl_ev_pending = ev   # one-shot marker for the next ces_events record ("glEv")
        # dedicated ces_events record (works even with CES off — log-validation channel)
        self._append_event({
          "t": round(time.time(), 1),  # noqa: TID251 -- wall clock, for route/time correlation
          "ev": "greenLight", "cls": ev, "mdlEndX": round(mdl_end_x, 1), "lead": has_lead,
          "dRel": round(float(getattr(lead, 'dRel', 0.0)), 1) if has_lead else 0.0,
          "vEgo": round(float(car_state.vEgo), 2),
          "lat": self._cur_lat, "lon": self._cur_lon,
        })
    except Exception:
      ev = None
    self.green_light = ev == GL_EV_GREEN
    self.lead_departing = ev == GL_EV_LEAD

  def _steer_log_step(self, car_state, sm) -> None:
    """cessteerlog2pnw: LOGGING ONLY. The sl*/lc*/vtsc* steering/lane-centering diagnostics in the
    normal tick/adopt records (_event_record) are only ever written while CES is enabled
    (CESMode>0) — _read_params() only calls _read_map() (which refreshes those fields from the
    SteerLimitStatus/LaneCenterStatus/VTSCStatus mem-params) inside `if self._enabled:`. On a
    CES-off drive that leaves steering behavior completely invisible in ces_events.jsonl. This
    method closes that gap with its own throttled (~C.TICK_S, same cadence as the normal tick)
    breadcrumb, mirroring _green_light_step's "always-on, log-validation channel" pattern:
      - runs BEFORE the `if not self._enabled: return False` gate in experimental_request, but
        no-ops immediately once _enabled is True, so it NEVER duplicates the existing tick/adopt
        record and never runs while CES is on;
      - when CES is off, calls _read_map() itself (the thing _read_params() skips) — that method
        is a pure, defensive read against self.mem_params (published by controlsd independent of
        CES) with try/except around every field, so it is safe to call regardless of _enabled;
      - does NOT call experimental_request's own decision logic, _publish_status, the ICBM
        executor, CES2, or any mode/actuator path — only reads mem-params and appends one JSONL
        record via the same best-effort _append_event used everywhere else in this file.
    Any exception here is swallowed; a broken breadcrumb must never affect control.

    leadrate2pnw: `sm` is the SAME SubMaster experimental_request() already holds (it's passed
    through from there, this call site adds no new subscription) — used ONLY to read
    sm['radarState'].leadOne, exactly like _green_light_step/experimental_request already do
    elsewhere in this file, so the CES-off "steer" breadcrumb can carry hasLead/dRel/vLead too
    (previously only the enabled tick/adopt path had lead telemetry)."""
    if self._enabled:
      return   # the enabled tick/adopt path already logs sl*/lc*/vtsc* once per ~1 Hz — no duplicate
    now = time.monotonic()
    if now - self._steer_tick_last < C.TICK_S:
      return
    self._steer_tick_last = now
    try:
      self._read_map()   # refresh sl*/lc*/vtsc*/GPS fields that _read_params() skips while CES is off
      # N1 review fix: keep the RAW read separate from the display-friendly v_ego -- a failed/absent
      # read must degrade achLat to None (not the false "driving straight" of k_actl * 0.0). raw_vego
      # is None exactly when the read failed; v_ego (the logged field) still defaults to 0.0 for
      # display continuity, matching controlsd's _ach_lat_ms2 pattern (None in, None out).
      try:
        raw_vego = getattr(car_state, 'vEgo', None)
        v_ego = round(float(raw_vego), 2) if raw_vego is not None else 0.0
      except Exception:
        raw_vego = None
        v_ego = 0.0
      now_wall = time.time()  # noqa: TID251 -- wall clock, for route/time correlation
      gps_valid = self._cur_lat is not None and self._cur_lon is not None
      # steerpower2pnw: pure functions, computed from fields already read just above by _read_map().
      ach_lat = _ach_lat(self._sl_k_actl, raw_vego)
      # leadrate2pnw: lead state, read directly from radarState.leadOne (same message CES already
      # subscribes to and reads elsewhere -- see experimental_request/_green_light_step). Defensive:
      # any failure here degrades to "no lead" telemetry-wise, never raises into this breadcrumb.
      # N-2 (Fable review): hasLead is genuinely THREE-STATE, not a plain bool -- False means "radar
      # read fine, no lead"; None means "the read itself failed / radarState unavailable this tick"
      # (the except below). Offline consumers filtering on the hasLead boolean should treat None as
      # "unknown", not silently coerce it to False -- a read failure is not the same fact as "no lead".
      has_lead = d_rel = v_lead = None
      try:
        lead = sm['radarState'].leadOne
        has_lead = bool(getattr(lead, 'status', False))
        if has_lead:
          d_rel_raw = float(getattr(lead, 'dRel', 0.0))
          v_lead_raw = float(getattr(lead, 'vLead', 0.0))
          # N-1 (Fable review): a NaN dRel/vLead from a corrupted radarState message would otherwise
          # round-trip straight through round()/float() into a bare, invalid-JSON NaN token -- same
          # guard as the existing vEgo NaN guard in the "alert" record above (search "takecontrol2pnw"
          # in this file). Gemini review note: this guards dRel/vLead INDEPENDENTLY of has_lead --
          # a genuinely-present lead (radar's own status bit True) with a corrupted distance reading
          # logs as hasLead=True, dRel=None (not hasLead=None/False) -- the lead itself is real, only
          # the poisoned field nulls. Offline consumers must not assume hasLead=True implies dRel is
          # non-null.
          d_rel = round(d_rel_raw, 1) if math.isfinite(d_rel_raw) else None
          v_lead = round(v_lead_raw, 1) if math.isfinite(v_lead_raw) else None
      except Exception:
        has_lead = d_rel = v_lead = None
      rec = {
        "t": round(now_wall, 1),
        # cesOff: True means self._enabled is False here — this can include CESMode 1/2 (Light/
        # Standard) when the car has neither op-long nor shadow (see _enabled's definition).
        "ev": "steer", "cesOff": True, "cesMode": self._mode, "car": self._car, "vEgo": v_ego,
        "gps": gps_valid,
        "lat": self._cur_lat, "lon": self._cur_lon, "bearing": self._cur_bearing,
        "spdLim": round(self._speed_limit, 1) if self._speed_limit else 0.0,
        # cargps2pnw: the CAR's own GPS, logged ALONGSIDE the device fix (lat/lon/bearing
        # elsewhere in this record), never instead of it. None on the Tesla -- no Ford
        # carstate means nothing publishes CarGps, which is the intended "empty" case.
        "car_gps": self._car_gps,
        "gpsSrc": self._gps_src,   # gpssel2pnw: when "car", lat/lon/bearing ARE the car_gps fix
        # VTSC applied cap + state (from VTSCStatus) — same fields as the enabled-path tick record.
        "vtscCap": self._vtsc_cap, "vtscState": self._vtsc_state, **getattr(self, "_vtsc_tele", {}),
        **getattr(self, "_sa_tele", {}),
        # lanecenter2pnw fields (from LaneCenterStatus) — same subset the enabled-path tick logs.
        "lcCorr": self._lc_corr, "lcAct": self._lc_act, "lcGate": self._lc_gate, "lcErr": self._lc_err,
        "lcLimN": self._lc_lim_n,
        "lcSpdA": self._lc_spd_a,
        # steerlimit-log2pnw / steertele2pnw / fordkappalog2pnw fields (from SteerLimitStatus).
        "slCurvLim": self._sl_curv_lim, "slSafetyLim": self._sl_safe_lim,
        "slAngDes": self._sl_ang_des, "slAngAct": self._sl_ang_act, "slAngErr": self._sl_ang_err,
        "slLatDem": self._sl_lat_dem, "slLatMax": self._sl_lat_max, "slCurvMax": self._sl_curv_max,
        "slSat": self._sl_sat, "slLatAct": self._sl_lat_active, "slAngSat": self._sl_ang_sat,
        # curvedbtel2pnw section 3.2: slKActl exactly 0.0 is a DEAD SENSOR, not a straight road (it
        # is 0.0 on 7,218 of 7,221 moving Tesla ticks) -> null. slKCmd/slKErr are left alone: a
        # commanded exact zero is a genuine command, and slLatAct already says if lateral was active.
        "slKCmd": self._sl_k_cmd, "slKActl": _zero_is_null(self._sl_k_actl), "slKErr": self._sl_k_err,
        # coopsteer-shadow2pnw: SHADOW torque-nudge fields (from SteerLimitStatus). Sign question:
        # sign(cpTq) vs sign(cpRate)/d(slAngAct) at light torque; cpOff is what we WOULD have added.
        "cpOff": self._cp_off, "cpTgt": self._cp_tgt, "cpCap": self._cp_cap, "cpWhy": self._cp_why,
        "cpTq": self._cp_tq, "cpRate": self._cp_rate, "cpCmd": self._cp_cmd,
        # steerpower2pnw: LOGGING ONLY — delivered lateral accel (m/s^2, signed) + 8-pt compass
        # heading, to measure the truck's true hands-off steering capability by direction. I4 review
        # fix: heading nulls (not "N") when there's no current GPS fix, rather than _compass()
        # silently reading a no-fix-defaulted 0.0 bearing as true north.
        # curvedbtel2pnw section 3.2: exact 0.0 -> null here too, same reason as slKActl above
        # (achLat IS slKActl * vEgo^2, so a dead kActl produced a confident 0.00 m/s^2 lateral accel
        # -- that is precisely the reading that voided the design's v1 proof of concept).
        "achLat": _zero_is_null(round(ach_lat, 3) if ach_lat is not None else None),
        "heading": _heading_if_fixed(self._cur_bearing, gps_valid),
        # curvedbtel2pnw sections 3.1/3.3/3.5/3.6: the curvature-peak fragment. mapLat/mapLon are
        # null on this breadcrumb -- the CES-off record carries no map candidate (no mapDist) to
        # match against; the section 3.7 check treats that as an expected-null population.
        **_curve_tele(self, raw_vego, None),
        # leadrate2pnw: LOGGING ONLY — lead-car state alongside this breadcrumb, so offline analysis
        # can separate "driver holding a speed by choice" from "speed forced by a slow lead" even on
        # a CES-off drive (previously only the enabled tick/adopt path carried lead telemetry).
        # dRel/vLead null (not 0.0) when there is no lead -- 0.0 would be indistinguishable from a
        # genuine lead sitting right at the bumper.
        "hasLead": has_lead, "dRel": d_rel, "vLead": v_lead,
      }
      if clock_bad(now_wall):
        rec["clockBad"] = True
      self._append_event(rec)
    except Exception:
      pass

  def _steer_event_step(self) -> None:
    """steerevent2pnw: edge-triggered mirror of controlsd's SteerEvent flight-recorder burst
    (docs/pnw/LANE-DEPARTURE-LOGGING-PROPOSALS.md Proposal 1) into ces_events.jsonl.

    Unlike _steer_log_step (throttled to ~C.TICK_S, CES-off only), this method is CALLED
    unconditionally every cycle from experimental_request() (selfdrived's ~100 Hz loop). It runs
    regardless of CESMode/_enabled: the underlying saturation episode is a controlsd/steering fact,
    independent of what CES itself is deciding.

    B1 review fix -- the mem-param GET itself is throttled internally, it is NOT done every call:
    Params.get() in this fork is a real file read + json.loads on every invocation (not a cached
    lookup, unlike the pattern _read_map's other fields might suggest -- those are also real reads,
    just made at _read_map's own ~1 Hz call cadence, not 100 Hz). Two layers keep this cheap:
      1. self._steer_event_frame counts calls; only every 20th (~5 Hz at this ~100 Hz call site)
         touches the param store at all. A rare event tolerates the <=~200 ms added latency -- the
         record carries its own srcT/t so nothing about the event's own timing is lost.
      2. Even at 5 Hz, the RAW bytes are read directly (mem_params.get_param_path + a plain file
         read) and compared against self._steer_event_raw_last BEFORE any json.loads -- an unchanged
         payload (the overwhelming majority of throttled reads, since a new event is rare) returns
         immediately without ever parsing JSON.
    Cost when idle: one file stat+read every ~200 ms + a bytes comparison. No JSON parsing and no
    _append_event write happens unless the raw payload actually changed.

    I3 review fix -- mem params are NOT cleared between drives (CLEAR_ON_MANAGER_START only fires on
    a manager restart, not on ignition), so a SteerEvent left over from a PREVIOUS drive would
    otherwise be picked up here on the next ignition and appended stamped with the new drive's wall
    time + current GPS -- wrong on both counts. The event's own wall-clock `t` (when controlsd
    actually emitted it) is checked against `now`; anything older than ~45 s is marked seen (so it's
    never reconsidered) but is NOT appended.

    Defensive: a missing mem store, a missing/never-published SteerEvent file, a malformed payload,
    or any field access failure is treated as 'nothing to log' and swallowed — this runs inside
    selfdrived's 100 Hz experimental_request() call and must never raise into it."""
    if self.mem_params is None:
      return
    # B1.1: throttle the GET itself to ~5 Hz -- see docstring.
    self._steer_event_frame += 1
    if self._steer_event_frame % 20 != 0:
      return
    try:
      path = self.mem_params.get_param_path("SteerEvent")
      try:
        with open(path, "rb") as f:
          raw = f.read()
      except FileNotFoundError:
        raw = b""   # never published yet (or a store that predates this key)
      # B1.2: dedup on the RAW bytes before parsing -- an unchanged payload means nothing new to log.
      if raw == self._steer_event_raw_last:
        return
      self._steer_event_raw_last = raw
      if not raw:
        return
      ev = json.loads(raw)
      if not isinstance(ev, dict) or not ev:
        return
      ev_id = ev.get("evId")
      if ev_id is None or ev_id == self._steer_event_seen_id:
        return   # either malformed (no evId) or already appended — edge dedup (evId is a string,
                 # salted per controlsd process -- see I2 review fix -- but plain `==` still works)
      self._steer_event_seen_id = ev_id   # mark seen now: a stale event below must never be
                                           # reconsidered even though it isn't appended
      now_wall = time.time()  # noqa: TID251 -- wall clock, rare edge-triggered append only
      # I3 review fix: skip (but keep marked-seen) a leftover event from a previous drive.
      src_t = ev.get("t")
      try:
        stale = src_t is not None and (now_wall - float(src_t)) > 45.0
      except Exception:
        stale = False
      if stale:
        return
      # steerpower2pnw I3 review fix: anchor the heading lookup on the episode's ONSET time
      # (controlsd's "onsetT", the idle->armed edge -- see the _flight_start_wall comment there), not
      # the emit time `src_t` used for staleness above. Falls back to src_t for an event emitted by a
      # pre-fix controlsd build that doesn't carry onsetT yet -- never worse than the old emit-time
      # behavior, and _nearest_bearing degrades to (None, False) if bearing_anchor_t is also None or
      # the history is empty (e.g. right at boot before _read_map has run once).
      onset_t = ev.get("onsetT")
      bearing_anchor_t = onset_t if onset_t is not None else src_t
      bearing_at_onset, gps_at_onset = _nearest_bearing(self._bearing_hist, bearing_anchor_t)
      rec = {
        "t": round(now_wall, 1),
        "ev": "steerEvent", "evId": ev_id, "car": self._car, "cesMode": self._mode,
        "durationS": ev.get("durationS"), "peakAngErr": ev.get("peakAngErr"),
        # steerpower2pnw: THE capability number — pass through controlsd's 100 Hz peak |achLat|
        # (m/s^2) for this episode as-is (already computed/rounded there — no re-derivation here).
        # Meaningful for offline direction-of-travel capability analysis when driverOverride is False.
        "peakAchLat": ev.get("peakAchLat"),
        # leadrate2pnw: pass through controlsd's 100 Hz peak steering-RATE accumulators as-is (already
        # computed/rounded there, same non-re-derivation contract as peakAchLat above). The S-curve
        # reversal that motivated this showed the binding limit is a SLEW RATE, not lateral accel --
        # peakCmdRate is how fast the plan asked to turn, peakActRate is how fast the wheel actually
        # followed; the gap between them (hands-off, driverOverride False) is the EPS slew ceiling.
        "peakCmdRate": ev.get("peakCmdRate"), "peakActRate": ev.get("peakActRate"),
        "peakLaneOff": ev.get("peakLaneOff"), "minLaneMargin": ev.get("minLaneMargin"),
        "minLaneConf": ev.get("minLaneConf"),
        # Never silently trust a low-confidence excursion: default True (flagged) if the source
        # event is missing the field for any reason, rather than defaulting to "confident".
        "laneLowConf": bool(ev.get("laneLowConf", True)),
        # I1 review fix: whether a driver steering override happened anywhere in the episode's own
        # window -- lets offline analysis separate genuine "openpilot couldn't steer" departures from
        # override-adjacent ones without silently dropping the latter.
        "driverOverride": bool(ev.get("driverOverride", False)),
        # steertrig2pnw: WHICH condition armed this episode ("sat"/"angSat"/"angErr"/"undershoot",
        # comma-joined) plus the two undershoot_turn inputs at the arming edge. Without these the
        # 33-events-in-19-min cluster of 2026-08-21 could not be attributed to any trigger: sat /
        # angSat / undershoot are evaluated at 100 Hz in controlsd while these records are ~1 Hz, so
        # offline reconstruction could only establish a negative (peakAngErr never reached the 8 deg
        # threshold, so it wasn't that branch). Pass-through, no re-derivation here.
        "trigWhy": ev.get("trigWhy"),
        "trigRatio": ev.get("trigRatio"),
        "trigDesLat": ev.get("trigDesLat"),
        # N5 review fix: True only when controlsd's 5 s defensive ceiling force-emitted this event
        # (still armed, no clean clear) -- durationS is then a known-truncated lower bound.
        "capped": bool(ev.get("capped", False)),
        "frameId": ev.get("frameId"), "modelLogMonoTime": ev.get("modelLogMonoTime"),
        "srcT": ev.get("t"),
        "trace": ev.get("trace") or [],
        # lat/lon/bearing stay the CURRENT (emit-time) GPS position -- same "close-tick snapshot"
        # convention as log_take_control_alert's "alert" records (see that docstring's NOTE), fine
        # for PLACING the episode on the map since position barely moves in a few seconds. heading is
        # the one field that needs the ONSET-time value (bearing_at_onset above) -- direction, unlike
        # position, can rotate 45-90 deg in that same window. I4 review fix: null (not "N") when no
        # GPS fix was current at the buffered onset sample (gps_at_onset).
        "gps": self._cur_lat is not None and self._cur_lon is not None,
        "lat": self._cur_lat, "lon": self._cur_lon, "bearing": self._cur_bearing,
        "heading": _heading_if_fixed(bearing_at_onset, gps_at_onset),
      }
      if clock_bad(now_wall):
        rec["clockBad"] = True
      self._append_event(rec)
    except Exception:
      pass

  # takecontrol2pnw: as of this feature, ces_events.jsonl carries THREE overlapping steering-related
  # record families for one physical wheel-saturation episode — each with its own trigger and its own
  # notion of "duration", so don't treat them as contradictory when digging through the log:
  #   "ev":"steer"      — steerlimit-log2pnw's ~1 Hz breadcrumb (see _steer_log_step above), samples
  #                        steer-limit state on a fixed clock regardless of any alert/event.
  #   "ev":"steerEvent" — steerevent2pnw's controlsd-side flight recorder, its OWN edge trigger
  #                        (slAngSat-based) sampled at up to 100 Hz around the peak.
  #   "ev":"alert","name":"steerSaturated" — THIS method: selfdrived's actual "Take Control" / "Turn
  #                        Exceeds Steering Limit" alert (the undershoot+turning+lac.saturated block
  #                        in selfdrived.py's update_events()) — the real driver-facing event, edge-
  #                        triggered + hold-off-debounced (see selfdrived._log_take_control_edge).
  def log_take_control_alert(self, payload: dict) -> None:
    """takecontrol2pnw: append one discrete {"ev":"alert","name":"steerSaturated",...} record to
    ces_events.jsonl. Called ONLY by selfdrived (selfdrive/selfdrived/selfdrived.py
    _log_take_control_edge/_emit_take_control_alert), on the rare rising/closing edge of ITS OWN
    steerSaturated ("Take Control" / "Turn Exceeds Steering Limit") decision — never every tick, and
    unconditionally regardless of CESMode/_enabled (the alert firing is a selfdrived fact, not a CES
    decision).

    `payload` is plain Python (str/float/bool/list/None only — no cereal/capnp objects), already
    built by the caller; this method does NOT decide anything and does NOT re-derive the trigger. It
    only enriches the record with fields ces already has cached from its own ~1 Hz _read_map()
    refresh (kept fresh regardless of CESMode by _steer_log_step/_read_params — steerlimit-log2pnw/
    steertele2pnw) and appends via the existing _append_event writer: GPS (self._cur_lat/_cur_lon/
    _cur_bearing) and steer-limit state (self._sl_ang_des/_sl_ang_act/_sl_ang_err/_sl_lat_dem/
    _sl_lat_max). No fresh mem-param read happens here.

    NOTE on phase="end" records: vEgo/gps/lat/lon/bearing/slAng*/slLat*/otherEvents here are all a
    CLOSE-TICK snapshot (~1 s after the episode, delayed by selfdrived's hold-off) — they are NOT the
    episode's peak/trigger-moment state. durationS (the one field that IS about the episode itself)
    is computed by the caller from the last frame the alert actually fired, not the close tick.

    See also the "three overlapping steering record families" note above CESController (or search
    ces_events.jsonl for '"ev":"steer"' / '"ev":"steerEvent"' / '"ev":"alert"') — this "alert" record
    is one of three different steering-related record types that can appear for one physical
    saturation episode; they have different triggers/durations and are not contradictory.

    Defensive: a malformed/partial payload or any field-access failure degrades to 'nothing logged'
    rather than raising — the caller already wraps this call in try/except as well (belt + braces),
    since this ultimately runs from selfdrived's ~100 Hz control loop."""
    try:
      now_wall = time.time()  # noqa: TID251 -- wall clock, rare edge-triggered append only
      v_ego = payload.get("vEgo")
      if v_ego is not None and not math.isfinite(v_ego):
        v_ego = None  # takecontrol2pnw: NaN/inf would serialize to a bare (invalid-JSON) NaN token
      rec = {
        "t": round(now_wall, 1),
        "ev": "alert", "name": payload.get("name", "steerSaturated"), "phase": payload.get("phase"),
        "car": self._car, "cesMode": self._mode,
        "vEgo": v_ego,
        "gps": self._cur_lat is not None and self._cur_lon is not None,
        "lat": self._cur_lat, "lon": self._cur_lon, "bearing": self._cur_bearing,
        # leadrate2pnw: heading was simply MISSING from this record (not gated too strictly -- there
        # was no "heading" key at all), so it always read back as None even on a valid fix, unlike the
        # "steer"/"tick"/"adopt" records taken the same moment. Same helper/gate those use: null only
        # on a genuine no-fix, not a blanket None.
        "heading": _heading_if_fixed(self._cur_bearing, self._cur_lat is not None and self._cur_lon is not None),
        # steerlimit-log2pnw fields — already-cached, no fresh read here (see docstring above).
        "slAngDes": self._sl_ang_des, "slAngAct": self._sl_ang_act, "slAngErr": self._sl_ang_err,
        "slLatDem": self._sl_lat_dem, "slLatMax": self._sl_lat_max,
        # co-active onroadEvents this frame (e.g. steerTempUnavailable/ldw), for correlation.
        "otherEvents": payload.get("otherEvents") or [],
      }
      if payload.get("durationS") is not None:
        rec["durationS"] = payload["durationS"]
      if clock_bad(now_wall):
        rec["clockBad"] = True
      self._append_event(rec)
    except Exception:
      pass

  def log_mads_resume(self, payload: dict) -> None:
    """madsresume2pnw: append one {"ev":"madsResume", ...} record to ces_events.jsonl.

    Called ONLY by selfdrived (_mads_resume_step), and only on the brain's rare state transitions
    (arm / fire / refuse / offerEnd / verify) -- never every tick. UNCONDITIONAL: it does NOT
    depend on CESMode/_enabled, because the auto-resume decision is a selfdrived/MADS fact, not a
    CES one, and the whole point of these records is that a refusal is visible on a drive with CES
    switched off (the exact gap cessteerlog2pnw exists to close for the steering breadcrumb).

    `payload` is plain Python (str/float/bool/None only), already built by the brain. This method
    decides nothing and re-derives nothing; it only stamps the record with the fields ces already
    has cached (car, CESMode, GPS from its own ~1 Hz refresh) and appends via _append_event.

    READING THE LOG:
      "phase":"arm"       -- a brake press left MADS steering alone; the window is open. `setMs` is
                             the driver's captured set speed (null => gate 6 already failed).
      "phase":"fire"      -- every gate passed; the offer is on the wire. Exactly one per arm.
      "phase":"refuse"    -- the arm ended WITHOUT a resume. `reason` names the binding gate. Exactly
                             one per arm, and mutually exclusive with "fire".
      "phase":"offerEnd"  -- the offer was withdrawn; `reason` is "expired" or the gate that cut it.
      "phase":"verify"    -- what the truck's set speed actually came back at after a press.
                             reason "setHigher" (with "loud":true) is the one that matters: the PCM
                             restored something ABOVE what the driver had set. reason "noCruise"
                             means the press produced no re-engagement at all.
    So: NO records at all for a brake event means the brain never armed (MADS off, toggle off, or
    not this car). A record with "fired":false and a `reason` is an explained no-resume. Those two
    are deliberately impossible to confuse, and neither can be confused with a resume that fired.

    Defensive: any failure degrades to 'nothing logged' rather than raising -- the caller wraps this
    too, since it ultimately runs inside selfdrived's 100 Hz loop."""
    try:
      now_wall = time.time()  # noqa: TID251 -- wall clock, rare edge-triggered append only
      rec = {
        "t": round(now_wall, 1),
        "ev": "madsResume",
        "car": self._car, "cesMode": self._mode,
        "gps": self._cur_lat is not None and self._cur_lon is not None,
        "lat": self._cur_lat, "lon": self._cur_lon, "bearing": self._cur_bearing,
        "heading": _heading_if_fixed(self._cur_bearing,
                                     self._cur_lat is not None and self._cur_lon is not None),
      }
      rec.update(payload)
      if clock_bad(now_wall):
        rec["clockBad"] = True
      self._append_event(rec)
    except Exception:
      # Gemini review 2026-09-06: an `except Exception: pass` here would make the auto-resume's
      # ONLY forensic channel fail silently, which is exactly what CLAUDE.md rule 2 forbids. These
      # records are rare (a handful per brake event), so an unthrottled log cannot flood -- and a
      # broken record builder must announce itself rather than leaving the drive log dark.
      # (Disk-level append failures are ALREADY loud, via _append_event's consecutive-failure
      # escalation; this catch is for the record-building above it.)
      try:
        cloudlog.exception("madsresume2pnw: failed to build/append the madsResume record -- this drive's auto-resume forensics are INCOMPLETE")
      except Exception:
        pass

  def _icbm_step(self, sig, active: bool) -> None:
    """Publish the IcbmTarget mem-param at ~4 Hz (executor treats >2 s silence as stale-stop).
    Best-effort: never raises into the control path. `active` False -> publish empty (ICBM off)."""
    now = time.monotonic()
    if now - self._icbm_last_pub < 0.25 or self.mem_params is None:
      return
    self._icbm_last_pub = now
    try:
      if not active:
        self._icbm_ceiling = None
        self._icbm_last_target = None
        self._icbm_src = None
        self._icbm_cand_d = None        # curvedbtel2pnw: cleared with the source it belongs to
        self._icbm_dir = None
        self._icbm_gate = None          # icbmmapfirst2pnw
        self._icbm_map_reach = None
        self._icbm_gps_age = None         # gpslag2pnw
        self._icbm_stale_hold = None      # gpsdrgate2pnw
        self._icbm_passed_state = None    # behindgate2pnw: the next verdict after a Chill interlude logs again
        # curvefloor2pnw (Fable 2026-09-05, F5): reset the floor state too. Without this a stale
        # icbmFlrHit=True is published alongside icbmT=None, and _icbm_floor_lim survives a Chill
        # interlude -- so the debounce carries a limit from before the gap into the road after it.
        self._icbm_floor_lim = 0.0
        self._icbm_floor_pend = None
        self._icbm_floor_hit = False
        self._icbm_rcap_state = None    # icbmrestorecap2pnw: no stale limit across a Chill interlude
        self._icbm_rcap = 0.0
        # icbmconsist2pnw: never publish a stale point-match alongside icbmT=None
        self._icbm_k_at = 0.0
        self._icbm_k_at_d = 0.0
        self._icbm_k_at_n = 0
        self._icbm_k_at_gap = 0.0
        _curvelead_clear(self)          # curvelead2pnw: no stale lead-pace / sanity telemetry; lead clock restarts
        self._icbm_ep.reset()           # icbmrestore2pnw: forced Chill / no data ends any episode
        self.mem_params.put_nonblocking("IcbmTarget", {})
        return
      # curvelead2pnw: advance the lead-continuity clock on EVERY active tick, not only while a curve binds
      # -- "tracked through the approach" has to be measured before the curve asks for it.
      trk = getattr(self, "_icbm_lead_trk", None)
      if trk is None:
        trk = self._icbm_lead_trk = IcbmLeadTrack()
      lead_s, lead_why = trk.update(now, sig.get("has_lead", False), sig.get("lead_drel", 0.0),
                                    sig.get("lead_vlead", 0.0), sig["v_ego"])
      # gpslag2pnw: ICBM's own ego position for THIS tick (icbm_project_position). Every map lookup below
      # uses plat/plon, and the near map candidate is re-derived from it here, so the CES decision that
      # built `sig` is untouched. A stale fix (> ICBM_GPS_MAX_AGE_S) is NO GPS for these lookups.
      prev_gps_state = getattr(self, "_icbm_gps_state", None)
      plat, plon, gps_age, gps_state = icbm_project_position(self._cur_lat, self._cur_lon, self._cur_bearing,
                                                             getattr(self, "_gps_fix_ts", None), sig["v_ego"], now)
      self._icbm_gps_age = round(gps_age, 2) if gps_age is not None else None
      if gps_state != prev_gps_state:   # Rule 2: change-only, incl. the stale -> no-map-lookups state
        cloudlog.event("ces_icbm_gps", state=gps_state, prev=prev_gps_state, age=self._icbm_gps_age,
                       src=getattr(self, "_gps_src", None))
        self._icbm_gps_state = gps_state
      if gps_state in ("proj", "stale"):
        # "raw"/"none": the caller's candidate was computed from this very position already
        map_tv, map_td = upcoming_curve(self._map_targets, plat, plon, sig["v_ego"], C.CURVE_MAP_LOOKAHEAD_S)
        sig = {**sig, "map_target_v": map_tv, "map_target_dist": map_td}
      # curveslow-lightning: vision curve candidate (the 493-curve gap: ICBM was MAP-ONLY and blind to
      # camera-seen curves). icbm_curve_target picks the more-binding of map / vision / far-map.
      vis_v, vis_dist = icbm_vision_apex(sig["v_ego"], sig.get("curve_lat_accel_vision", 0.0),
                                         sig.get("time_to_curve", float("inf")))
      # descentcurve2pnw: full-horizon map candidate — at highway speed the 10 s window (~400 m at
      # 90 mph) hid curves that need the full 500 m mapd publishes (the 2026-07-11 silent-ICBM run).
      # Capability-supplied knobs: map_scale (<=1 OSM discount) + firm_decel (large-drop envelope);
      # both neutral (1.0 / 0.0) on any non-Lightning.
      # icbmrestore2pnw: the episode machine owns the ceiling latch now. While a CAP episode is
      # active, curve binding is judged against ITS latched ceiling (v_set follows the tapped-down
      # stock set); during restore/idle a fresh cap latches at the current set.
      ep_ceiling = self._icbm_ep.ceiling if self._icbm_ep.phase == "cap" else None
      ref = ep_ceiling if ep_ceiling is not None else sig["v_set"]
      # icbmtrack2pnw: ICBM-only capped scale (19:58:37Z root cause — the tiered sweeper end
      # inflated a raw 64.9 mph curve to an effective 107 mph, so it never bound vs set 90).
      far_v, far_dist = icbm_far_map_candidate(self._map_targets, plat, plon,
                                               sig["v_ego"], ref, icbm_map_eff_scale,
                                               self._veh.icbm_map_scale, self._veh.icbm_firm_decel)
      # icbmmapfirst2pnw start-policy gates (drive 2026-07-12): apply ONLY when a decision would
      # START a new episode — a running cap episode (phase 'cap', incl. its S-gap clear debounce)
      # continues with the full candidate set exactly as before ("an episode may continue").
      ttc = float(sig.get("time_to_curve", float("inf")) or float("inf"))
      in_curve = icbm_in_curve(sig.get("lat_accel_now", 0.0), sig.get("curve_lat_accel_vision", 0.0), ttc)
      starting = self._icbm_ep.phase != "cap"
      self._icbm_gate = None
      self._icbm_map_reach = None
      if starting:
        map_reach = icbm_map_reach(self._map_targets, plat, plon)
        self._icbm_map_reach = round(map_reach, 0)
      target, _, self._icbm_src = icbm_curve_target(
        sig["v_ego"], sig["v_set"], sig.get("map_target_v", 0.0),
        sig.get("map_target_dist", float("inf")), ep_ceiling, icbm_map_eff_scale,
        vis_v, vis_dist,
        map_scale=self._veh.icbm_map_scale, firm_decel=self._veh.icbm_firm_decel,
        far_v=far_v, far_dist=far_dist, track=True)
      if target is not None and starting:
        if in_curve:
          # driver rule 2: NEVER begin a new dec episode while already loaded in the curve — hold
          # the current set (any source: braking mid-curve was the field complaint).
          self._icbm_gate = "inCurve"
          target, self._icbm_src = None, None
        elif self._icbm_src == "vis" and not icbm_vision_may_start(vis_dist, ttc, map_reach):
          # driver rule 1 (MAP-FIRST): live map coverage over this stretch -> the map verdict
          # (incl. "no slowdown needed") is authoritative for anticipatory slowing; and a too-late
          # vision curve must not start taps at the curve. mapd-dead fallback: reach 0.0 -> only
          # the time gate applies, vision keeps initiating where the map is blind.
          self._icbm_gate = "visCovered" if vis_dist <= map_reach else "visLate"
          # a still-binding map/far candidate may start the episode instead of the gated vision one
          target, _, self._icbm_src = icbm_curve_target(
            sig["v_ego"], sig["v_set"], sig.get("map_target_v", 0.0),
            sig.get("map_target_dist", float("inf")), ep_ceiling, icbm_map_eff_scale,
            0.0, float("inf"),
            map_scale=self._veh.icbm_map_scale, firm_decel=self._veh.icbm_firm_decel,
            far_v=far_v, far_dist=far_dist, track=True)
      # behindgate2pnw: a map/far point the truck has already driven past may not START an episode (see
      # icbm_passed_points).
      # behindrun2pnw (owner 2026-09-14, "Behind-curve gate: yes"): nor may it lower a RUNNING cap episode's target or
      # keep it bound after the curve (which holds the restore back). Same test, same cannot-tell -> no gate; the
      # running episode is re-decided on its own inputs (latched ceiling, vision without the start rule).
      if target is not None and self._icbm_src in ("map", "far"):
        # vision exactly as the decision being re-taken used it: the MAP-FIRST start rule applies to a START
        # only; a running episode keeps vision's full authority (icbmmapfirst2pnw). Written as a statement
        # rather than an `or starting` expression so a running tick never NAMES map_reach -- that local only
        # exists on the starting path, and relying on `or` short-circuit to avoid a NameError here would put
        # the whole IcbmTarget publish one edit away from dying in _icbm_step's except.
        vis_gate = (vis_v, vis_dist)
        if starting and not icbm_vision_may_start(vis_dist, ttc, map_reach):
          vis_gate = (0.0, float("inf"))
        target, sig, far_v, far_dist = _icbm_passed_gate(
          self, now, target, sig, plat, plon, ref, far_v, far_dist, vis_gate,
          ceiling=ep_ceiling, running=not starting)
      # gpsdrgate2pnw: whether the running cap episode was ever bound by the MAP (stale hold). Sticky within the
      # episode (Fable B1): on a real approach vision joins the map/far candidate for the same curve, and a
      # last-binder record let a vision tick erase the map provenance -> no hold -> restore 50 m before the curve.
      if target is not None and (self._icbm_src in ("map", "far") or self._icbm_ep.phase != "cap"):
        self._icbm_cap_src = self._icbm_src
      # curveslow-lightning: lower the chosen apex on the Lightning (weaker EPS -> enter curves slower).
      # Penalty is >= 0 (never a speed-up), only lowers -> still reduce-only vs the ceiling; floor 0.
      # icbmalign2pnw: ICBM now applies the SAME descent + left-curve multipliers as VTSC — literally
      # the same shared function (PnwVehicle.curve_speed_penalty_ms, knobs from /data/pnw/curve.json),
      # so stock-ACC and op-long behavior stay aligned. Direction per source:
      #   vis      -> sign of the model's predicted lateral accel (lat = orientationRate.z * v, and
      #               z > 0 = LEFT — the convention verified for apex_turn_direction);
      #   map/far  -> map path geometry at the candidate's distance (map_turn_direction, same
      #               left-positive convention). Unknown direction / no pitch -> neutral (no-op).
      # Multipliers only ever RAISE the penalty (>= 1, clamped), so the target only moves DOWN:
      # DEC-only/ceiling semantics untouched.
      if target is not None:
        is_left = False
        try:
          if self._icbm_src == "vis":
            is_left = float(sig.get("curve_lat_accel_vision", 0.0) or 0.0) > 0.0
          elif self._icbm_src == "far":
            is_left = map_turn_direction(self._map_targets, plat, plon, far_dist) > 0
          elif self._icbm_src == "map":
            is_left = map_turn_direction(self._map_targets, plat, plon,
                                         sig.get("map_target_dist", float("inf"))) > 0
        except Exception:
          is_left = False
        target = max(target - self._veh.curve_speed_penalty_ms(target, pitch_rad=sig.get("pitch"),
                                                               is_left=is_left), 0.0)
        # rain2pnw: driver-selected wet-weather curve margin (same reduction the Tesla/VTSC gets).
        target = max(target - self._veh.rain_penalty_ms(), 0.0)
      # curvefloor2pnw: POSTED-LIMIT FLOOR for the ICBM path, low limits only.
      # 2026-08-11 06:52: spdLim 11.2 (25 mph), map target 7.3 m/s sustained 12 s, stock set tapped
      # 23.7 -> 7.15 (16 mph). icbmratchet2pnw only confirms single-tick OUTLIER drops, so nothing
      # caught it. Placed HERE -- after the penalties, before _icbm_ep.step -- so the floored value
      # is what the ratchet confirms and publishes; a RAISE can never trip the outlier-drop gate.
      # Floored at limit MINUS the same penalties VTSC subtracts (vtsc_controller floor_v), not at
      # the bare limit: flooring at the raw posted value would undo the Lightning curve margin and
      # the rain margin that were just applied, which icbmalign2pnw and rain2pnw exist to preserve.
      # min(..., ref) so the floor can only ever RAISE a too-low target toward the limit, never
      # command anything above the reference the driver/episode already allows.
      self._icbm_floor_lim, self._icbm_floor_pend = C.icbm_floor_limit(
        sig.get("spd_lim", 0.0), self._icbm_floor_lim, now, self._icbm_floor_pend)
      if target is not None and self._icbm_floor_lim > 0.0:
        try:
          floor = self._icbm_floor_lim - self._veh.curve_speed_penalty_ms(self._icbm_floor_lim) \
                                       - self._veh.rain_penalty_ms()
        except Exception:
          floor = 0.0            # any penalty failure -> no floor (pre-curvefloor2pnw behaviour)
        if floor > 0.0:
          floored = min(max(target, floor), ref)
          self._icbm_floor_hit = floored > target + 1e-9
          target = floored
        else:
          self._icbm_floor_hit = False
      else:
        self._icbm_floor_hit = False
      # icbmconsist2pnw: POINT-MATCHED POLYLINE CURVATURE -- TELEMETRY ONLY. NOTHING HERE TOUCHES
      # CONTROL, deliberately.
      #
      # THE GOAL: the 2026-09-08 20:28 phantom (mapd claimed a curve on a straight 60 mph motorway;
      # ICBM dragged the set 70 -> 44 mph) needs mapd's velocity checked against real geometry. The
      # polyline in mapd's own message is that geometry, and icbmcurv2pnw already measures it.
      #
      # WHY THIS IS NOT WIRED, having been built wired and then rejected on the evidence: the
      # existing `icbmK` is the horizon MAXIMUM curvature, not the curvature at MAPD'S point. A
      # review replaying all 4077 ticks of that drive found the two points were 51-328 m apart on
      # every one of the 28 ticks a check would have fired -- never closer than 50 m -- so it was
      # never actually comparing mapd's claim to mapd's curve. On 19:37:47 it contradicted a CORRECT
      # mapd claim (a ramp at R~=38 m, 22 m ahead) using an unrelated gentler curve 172 m further on,
      # and would have suppressed a real slowdown on R 62-172 m geometry. It also fired on 28 of 28
      # eligible ticks with ZERO reading consistent -- a consistency check that never finds
      # consistency is measuring a systematic offset (the pipeline's own scale and penalty factors),
      # not discriminating. And the polyline's under-read tail is p99 3.29x, far beyond any margin
      # that keeps a real curve safe.
      #
      # So: measure first, wire later. `icbmKAt` is the curvature at mapd's OWN target distance and
      # `icbmKAtGap` is how far off the nearest measurable triplet landed -- the number that says
      # whether a comparison is legitimate at all. Same telemetry-first discipline icbmcurv2pnw used
      # before anything read icbmK. Wire a check only once real drives show icbmKAtGap is routinely
      # small; on the one drive we have, it never was.
      self._icbm_k_at = 0.0
      self._icbm_k_at_d = 0.0
      self._icbm_k_at_n = 0
      self._icbm_k_at_gap = 0.0
      if target is not None and self._icbm_src in ("map", "far") and self._map_targets:
        try:
          at_d = {"map": sig.get("map_target_dist", float("inf")), "far": far_dist}.get(self._icbm_src)
          if at_d is not None and math.isfinite(float(at_d)):
            kat, katd, katn, katgap, _ = polyline_curvature_at(
              self._map_targets, plat, plon, MAP_SOURCE_HORIZON_M,
              float(at_d), self._cur_bearing)
            self._icbm_k_at = float(kat)
            self._icbm_k_at_d = float(katd)
            self._icbm_k_at_n = int(katn)
            self._icbm_k_at_gap = float(katgap)
        except (TypeError, ValueError):
          pass                # measurement only; a bad map_target_dist must not disturb control
      # curvelead2pnw (A): LEAD-PACED RELAXATION -- see icbm_lead_pace. After every reduction (penalties,
      # rain) and the posted-limit floor, so it can only RAISE the finished cap, capped at `ref` (the
      # driver's own set / the episode ceiling): ICBM stays DEC-only. When any gate fails -- including the
      # tick the lead stops being tracked -- `target` is simply ICBM's own, unchanged.
      # Known limit, deliberately not "fixed" here: IcbmEpisode's outlier ratchet holds a revert larger
      # than ICBM_RATCHET_OUTLIER_DROP_MS at the last published (relaxed) value for ICBM_RATCHET_CONFIRM_S
      # before publishing it -- the brain reverts the same tick, the PUBLISHED target ~0.6 s later. That
      # value was checked against the truck's bound one tick earlier; the ratchet is out of scope here.
      # Pinned in test_curvelead2pnw.py (closed loop through the Ford executor).
      # A bug anywhere in this block must cost lead pacing, never ICBM's own slowdown: it falls back to `own_t`
      # and says so (_curvelead_failed), instead of escaping into this method's except, where the whole
      # IcbmTarget publish would be lost and the executor would stale-stop.
      own_t = target
      try:
        cand_dist = {"map": sig.get("map_target_dist", float("inf")), "vis": vis_dist,
                     "far": far_dist}.get(self._icbm_src, float("inf"))
        vis_k, vis_reach = sig.get("vis_k_max"), sig.get("vis_reach", 0.0)
        a_lead = self._veh.icbm_lead_lat_accel
        pace, lead_pace_why, k_tight = icbm_lead_pace(
          own_t, ref, sig["v_ego"], self._icbm_src, cand_dist, sig.get("lead_vlead", 0.0), lead_s, lead_why,
          self._icbm_k, self._icbm_k_n, self._icbm_k_at, self._icbm_k_at_n, self._icbm_k_at_gap,
          vis_k, vis_reach, a_lead, self._veh.rain_penalty_ms())
        if pace is not None:
          target = pace
        # curvelead2pnw (B) -- TELEMETRY ONLY. What the geometry+vision sanity rule and the behind-the-truck
        # test WOULD do. Neither value is read by anything below; see icbm_map_sanity / icbm_path_behind.
        sane_t, sane_why = icbm_map_sanity(own_t, ref, sig["v_ego"], self._icbm_src, cand_dist,
                                           self._icbm_k_at, self._icbm_k_at_n, self._icbm_k_at_gap,
                                           vis_k, vis_reach, a_lead)
        behind = (icbm_path_behind(self._map_targets, plat, plon, cand_dist)
                  if own_t is not None and self._icbm_src in ("map", "far") else None)
        _curvelead_note(self, now, own_t, pace, lead_pace_why, lead_s, sig.get("lead_vlead", 0.0), k_tight,
                        vis_k, sane_t, sane_why, behind)
      except Exception as e:
        # silentexc2pnw: caught broadly (a code defect must cost lead pacing, not the IcbmTarget publish). The log
        # now names the exception type and counts failures; _curvelead_failed cannot raise (see there).
        target = own_t
        _curvelead_failed(self, now, own_t, e)
      # icbmrestore2pnw: run the episode machine — it forwards caps unchanged ('dec'), enters the
      # bounded GUARDED restore when the curve clears, and hard-aborts on any driver-intent signal.
      driver_pedal = bool(sig.get("gas")) or bool(sig.get("brake"))
      # gpsdrgate2pnw: STALE-GPS HOLD of a running map/far cap (ICBM_GPS_STALE_HOLD_MAX_S). Vision binding this
      # tick (target set) or the fix coming back ends it at once; so does leaving the cap phase.
      hold = getattr(self, "_icbm_stale_hold", None)
      hold_left = None
      if target is None and gps_state == "stale" and self._icbm_ep.phase == "cap":
        if hold is None:
          declined = ("notMap" if getattr(self, "_icbm_cap_src", None) not in ("map", "far") else
                      "noDistance" if self._icbm_ep._last_cap_dist is None else
                      "noCap" if self._icbm_ep._committed_target is None else None)
          if declined is None:
            hold = self._icbm_stale_hold = {"t0": now, "t": now, "left": self._icbm_ep._last_cap_dist + ICBM_MARGIN_M,
                                            "on": True}
            cloudlog.event("ces_icbm_stale_hold", state="hold", cap=round(self._icbm_ep._committed_target, 2),
                           dist=round(self._icbm_ep._last_cap_dist, 1), age=self._icbm_gps_age)
          else:                     # Rule 2: say why the cap was NOT held, once per episode (hold stays "off")
            self._icbm_stale_hold = {"on": False}
            cloudlog.event("ces_icbm_stale_hold", state="declined", why=declined, src=getattr(self, "_icbm_cap_src", None))
        if hold is not None and hold["on"]:
          hold["left"] -= max(float(sig["v_ego"]), 0.0) * (now - hold["t"])
          hold["t"] = now
          if hold["left"] > 0.0 and now - hold["t0"] <= ICBM_GPS_STALE_HOLD_MAX_S:
            target, self._icbm_src = self._icbm_ep._committed_target, "gpsHold"
            hold_left = max(hold["left"] - ICBM_MARGIN_M, 0.0)
          else:
            hold["on"] = False
            why = "passed" if hold["left"] <= 0.0 else "maxTime"
            cloudlog.event("ces_icbm_stale_hold", state="release", why=why, held_s=round(now - hold["t0"], 1))
            if why == "maxTime":
              # Fable B2: the curve is still unlocated, and "unknown is not clear" holds at the time bound too.
              # End the episode WITHOUT a restore: the set stays where ICBM put it, for the driver to raise.
              self._icbm_ep.reset()
      elif hold is not None:
        if hold["on"]:
          cloudlog.event("ces_icbm_stale_hold", state="release", why="gpsBack" if gps_state != "stale" else
                         ("capBound" if target is not None else "episodeEnded"), held_s=round(now - hold["t0"], 1))
        self._icbm_stale_hold = None
      # icbmmapfirst2pnw: hand the episode the binding candidate's DISTANCE (apex-passage detection
      # for the early restore) and the in-curve flag (restore entry deferral / restore pause).
      src_dist = {"map": sig.get("map_target_dist", float("inf")),
                  "vis": vis_dist, "far": far_dist, "gpsHold": hold_left}.get(self._icbm_src)
      # icbmrestorecap2pnw: bound what a RESTORE may give back by the road the truck is on NOW.
      # Driver report 2026-09-13 15:46 PT: the curve cleared as the truck entered a 25 mph zone and
      # restore tapped the set 27 -> 60 there. Updated every tick (not only during restore) so the
      # debounce history is warm when a restore begins.
      # getattr, not a bare attribute: an AttributeError here would be swallowed by this method's
      # own except and silently skip ICBM for the tick -- same reason _icbm_err_last is read this way.
      rcap_lim, self._icbm_rcap_state = C.icbm_restore_limit(sig.get("spd_lim", 0.0),
                                                             getattr(self, "_icbm_rcap_state", None), now)
      # sazoneset2pnw (driver directive 2026-09-13): the restore goes back to the pre-curve set; it is capped
      # ONLY when the limit dropped while this episode ran (see icbm_stale_zone_cap), and that cap is sticky.
      # "Proportional" = the driver has zone speeds on (speedadjust AutoSpeedReduce >= 2), read from its
      # forwarded status. A missing/unreadable mode is not evidence it is on: it falls back to limit + 5.
      icbm_note_speedadjust(self._icbm_ep, getattr(self, "_sa_tele", None), rcap_lim)
      self._icbm_rcap = float(self._icbm_ep.zone_cap) if self._icbm_ep.zone_cap is not None else 0.0
      pub_target, direction = self._icbm_ep.step(now, target, sig["v_set"],
                                                 self._stock_set, self._stock_on, driver_pedal,
                                                 cap_dist=src_dist, v_ego=sig["v_ego"], in_curve=in_curve,
                                                 restore_cap=self._icbm_rcap if self._icbm_rcap > 0.0 else None,
                                                 limit_now=rcap_lim if rcap_lim > 0.0 else None)
      self._icbm_ceiling = self._icbm_ep.ceiling
      self._icbm_dir = direction
      if direction == "inc":
        self._icbm_src = "restore"      # telemetry: the restore phase is its own source label
      # curvedbtel2pnw section 3.4 (Fable I1) -- TELEMETRY ONLY, nothing below reads it. Latched AFTER
      # the restore relabel so it can never name a distance for a source that is no longer a map
      # point, and taken from `src_dist`, which is the same distance the episode machine was just
      # handed: the record's mapLat/mapLon then locate the curve ICBM is ACTUALLY reacting to,
      # including a far candidate out at MAP_SOURCE_HORIZON_M that CES's own mapDist cannot see.
      self._icbm_cand_d = src_dist if self._icbm_src in ("map", "far") else None
      self._icbm_last_target = round(pub_target, 2) if pub_target is not None else None
      if pub_target is not None:
        # JSON params take a DICT (params_pyx serializes it; a pre-dumped string raises TypeError —
        # Gemini review catch that would have silently killed every publish)
        # ceiling: the episode latch, or — for an episode-less forwarded cap (ACC off / pedal
        # pressed, episode reset) — the driver's current set: the exact pre-restore reduce-only
        # semantics. The executor clamps target <= ceiling either way.
        ceil_pub = self._icbm_ceiling if self._icbm_ceiling is not None else sig["v_set"]
        payload = {"target": round(pub_target, 2), "ceiling": round(ceil_pub, 2), "ts": time.time()}  # noqa: TID251 -- wall clock heartbeat shared with the executor
        if direction == "inc":
          payload["dir"] = "inc"        # explicit marker: executor's inc path is ONLY reachable via this
        self.mem_params.put_nonblocking("IcbmTarget", payload)
      else:
        self.mem_params.put_nonblocking("IcbmTarget", {})
    except Exception:
      # RULE 2. This `except` used to be a bare `pass`, and it bit during this very change: a missing
      # attribute raised here, the publish never happened, and the ONLY symptom was a test failing
      # downstream on an absent target. In production the symptom is worse and quieter -- no
      # IcbmTarget publish means the Ford executor stale-stops after 2 s and ICBM is simply DEAD for
      # the rest of the drive, with nothing anywhere saying so.
      # Still swallowed (this must never raise into the control loop), but no longer silent.
      # Throttled: at ~4 Hz a persistent fault would otherwise flood the log it needs to be seen in.
      # getattr with a default, not self._icbm_err_last: this runs on the failure path, and an
      # AttributeError HERE would escape the except and reach the control loop -- an error handler
      # that can itself raise is worse than none. (Caught by a stub that lacked the field.)
      try:
        now_w = time.monotonic()
        due = now_w - getattr(self, "_icbm_err_last", -1e9) > ICBM_ERR_LOG_S
        if due:
          self._icbm_err_last = now_w
      except Exception:
        due = False
      if due:
        try:
          cloudlog.exception("icbm: _icbm_step FAILED -- no IcbmTarget published; the executor will stale-stop and ICBM is inert until this clears")
        except Exception:
          pass                      # logging must not become the thing that raises

  def _publish_status(self, sig, want: bool) -> None:
    """Log mode transitions and publish a throttled CESStatus snapshot to the in-memory param store
    for the on-screen overlay. Display/diagnostics only — never affects the returned decision."""
    mode = "experimental" if want else "chill"
    tele = decision_telemetry(sig) if sig is not None else {
      "reason": "noData", "curvePct": 0, "curveSrc": "", "mapV": 0.0, "mapDist": 0.0, "vEgo": 0.0,
      "stp": False,   # stophold2pnw (B): keep the key present on the noData path too
    }
    tele["mode"] = mode
    # stopintent2pnw: the adopt record must show WHICH entry path fired — decide_active's reason
    # cannot know the state machine took the fast path, so override from the sm status (it holds
    # "stopIntent" exactly for the cycle the preemption happened).
    # ces2core2pnw: when CES2 is LIVE, the CES2 core's status is the authoritative reason instead.
    if mode == "experimental":
      if self._ces2_live and self._ces2_reason:
        tele["reason"] = self._ces2_reason
      # standstill2pnw: the hold tags too — while a hold is the ONLY thing keeping Experimental,
      # decide_active's reason reads "chill", which made the 11:34 flapping forensics blind to WHY
      # the mode was held. Overlay/log now shows the state machine's authoritative reason.
      # cesnochill2pnw: "stopLatch" added — the hard latch's own override tag, so field telemetry
      # can never again show "reason": "chill" while mode is actually experimental.
      elif self._sm.status() in ("stopIntent", "stopHold", "standstillHold", "stopLatch"):
        tele["reason"] = self._sm.status()
    # ces2core2pnw shadow A/B channel: CES2's would-be decision + graded urgency + the cumulative
    # divergence-edge counter, on EVERY record (the replay/acceptance dataset).
    tele["ces2Mode"] = self._ces2_mode
    tele["ces2Reason"] = self._ces2_reason
    tele["ces2Urgency"] = round(float(self._ces2_urg), 3)
    tele["ces2Div"] = self._ces2_div.count
    tele["ces2Live"] = self._ces2_live
    tele["button"] = int(self._button)
    tele["enabled"] = True
    # mapd diagnostics so the overlay can always show what mapd is up to (curve half is map-driven):
    tele["mapPts"] = len(self._map_targets)                       # MapTargetVelocities points cached
    tele["gps"] = self._cur_lat is not None and self._cur_lon is not None  # LastGPSPosition fix present
    # icbm2pnw overlay feed (driver req 2026-07-11): current button-management target + the truck's
    # reported stock set speed, so the debug box can show "ICBM 24>18" while taps are stepping it down.
    # The shadow flag itself MUST be in the overlay feed too — the renderer keys the grey SHADOW
    # labels and the ICBM line on it (it was only in the ces_events record; the overlay kept showing
    # orange EXPERIMENTAL in shadow — driver caught it twice, 2026-07-11).
    tele["shadow"] = self._shadow
    if self._shadow:
      tele["icbmT"] = self._icbm_last_target
      tele["icbmSrc"] = self._icbm_src           # curveslow-lightning: "map"/"vis"/"restore" source
      tele.update(_curvelead_tele(self))         # curvelead2pnw: lead pacing (A) + sanity telemetry (B)
      # curvefloor2pnw: the posted-limit floor. icbmFlr = the debounced limit backing it (0 = no
      # floor active), icbmFlrHit = the floor actually raised the target on this tick. Without both
      # on the drive log there is no way to tell "the floor never applied" from "the floor applied
      # and was not needed" -- the exact ambiguity that made the 2026-08-11 over-slow hard to close.
      # icbmcurv2pnw: what the polyline ACTUALLY says, beside what mapd claimed. icbmKN==0 means
      # unmeasurable, not straight -- read it before believing icbmK. On the overlay feed as well as
      # the event log because CESStatus is the 5 Hz live channel: `cat /dev/shm/params/d/CESStatus`
      # answers "is the measurement running right now" without waiting for a drive.
      tele["icbmK"] = round(float(self._icbm_k), 6)
      tele["icbmKD"] = round(float(self._icbm_k_dist), 0)
      tele["icbmKV"] = round(float(self._icbm_k_v), 1)
      tele["icbmKN"] = int(self._icbm_k_n)
      tele["icbmKAhead"] = bool(self._icbm_k_ahead)
      # icbmconsist2pnw: the POINT-MATCHED reading beside mapd's own claim. icbmKAtGap says how far
      # the nearest measurable triplet fell from mapd's point -- i.e. whether comparing the two is
      # legitimate on this tick at all. Telemetry only; nothing consumes these for control yet.
      tele["icbmKAt"] = round(float(self._icbm_k_at), 6)
      tele["icbmKAtD"] = round(float(self._icbm_k_at_d), 0)
      tele["icbmKAtN"] = int(self._icbm_k_at_n)
      tele["icbmKAtGap"] = round(float(self._icbm_k_at_gap), 0)
      tele["icbmFlr"] = round(float(self._icbm_floor_lim), 1)
      tele["icbmRCap"] = round(float(getattr(self, "_icbm_rcap", 0.0) or 0.0), 1)   # icbmrestorecap2pnw: 0 = no cap
      # icbmrestorecap2pnw (Fable review): the hold publishes nothing, so without the phase a 45 s
      # hold is indistinguishable from idle in ces_events -- and on-car validation depends on it.
      tele["icbmPhase"] = getattr(getattr(self, "_icbm_ep", None), "phase", None)
      tele["icbmZoneWhy"] = getattr(getattr(self, "_icbm_ep", None), "zone_why", None)      # sazoneset2pnw
      tele["icbmLatchLim"] = getattr(getattr(self, "_icbm_ep", None), "latch_limit", None)  # sazoneset2pnw
      tele["icbmFlrHit"] = bool(self._icbm_floor_hit)
      tele["icbmDir"] = self._icbm_dir           # icbmrestore2pnw: "dec" capping / "inc" restoring
      tele["icbmSet"] = self._stock_set
      tele["icbmOn"] = self._stock_on
      tele["icbmGate"] = self._icbm_gate         # icbmmapfirst2pnw: start suppressed & why (or None)
      tele["mapReach"] = self._icbm_map_reach    # icbmmapfirst2pnw: map coverage m (0/None = blind)

    # (a) transition ("adopt") — one record per chill<->experimental change, cloudlog + event file.
    if mode != self._last_mode:
      cloudlog.info("CES %s->%s button=%d reason=%s curve=%d%%(%s) vEgo=%.1f vSet=%.1f az=%s mapV=%.1f",
                    self._last_mode, mode, self._button, tele.get("reason"),
                    tele.get("curvePct", 0), tele.get("curveSrc", ""), tele.get("vEgo", 0.0),
                    tele.get("vSet", 0.0), tele.get("accelZone"), tele.get("mapV", 0.0))
      rec = self._event_record("adopt", tele)
      rec["from"], rec["to"] = self._last_mode, mode
      self._append_event(rec)
      self._last_mode = mode
    else:
      # (b) heartbeat ("tick") — ~1 Hz breadcrumb so the WHOLE drive's GPS track + state is captured
      # (lets us place every adoption on the route and apply the highway / 300 ft buffer in analysis).
      now2 = time.monotonic()
      if now2 - self._tick_last >= C.TICK_S:
        self._tick_last = now2
        self._append_event(self._event_record("tick", tele))

    # ~5 Hz publish to /dev/shm/params (put a dict -> JSON; nonblocking so the safety loop never waits)
    if self.mem_params is None:
      return
    now = time.monotonic()
    if now - self._tele_last < 0.2:
      return
    self._tele_last = now
    try:
      # stophold2pnw (C): wall-clock heartbeat — the overlay's NO-SIGNAL dead-man compares this
      # against its own clock, so a dead/silent publisher becomes VISIBLE (the driver's "silence
      # must be loud" rule, born of the 2026-07-12 false-silence investigation).
      tele["ts"] = round(time.time(), 2)  # noqa: TID251 -- wall clock heartbeat shared with the overlay
      self.mem_params.put_nonblocking("CESStatus", tele)
    except Exception:
      pass

  def _event_record(self, kind: str, tele: dict) -> dict:
    """Build one rich, flat record for the persistent CES_EVENT_LOG. `kind` is "adopt" (a CES mode
    transition) or "tick" (a ~1 Hz breadcrumb). Includes GPS (lat/lon/bearing), OSM speed limit, a
    coarse highway guess, the accelerate-zone decision + its inputs (vSet/dRel/vLead/aEgo/gas), and
    the curve/map diagnostics — everything needed to verify behavior against the route later."""
    vego = float(tele.get("vEgo") or 0.0)
    hwy = (self._speed_limit >= C.HWY_SPEED_LIMIT) or (vego >= C.HWY_VEGO)  # coarse; authoritative = GPS+OSM+300ft in analysis
    now_wall = time.time()  # noqa: TID251 -- wall clock, for route/time correlation
    # steerpower2pnw: pure functions, computed from fields already read by _read_map() (see __init__).
    # N1 review fix: `vego` above is the "or 0.0"-defaulted value the `hwy` coarse guess intentionally
    # tolerates -- but that same fallback fed straight into achLat made a genuine no-data record
    # (_publish_status's "noData" sentinel tele, or any tele missing "vEgo") indistinguishable from
    # "driving perfectly straight" (k_actl * 0**2 = 0.0). achLat instead uses the RAW reading, None
    # when there isn't one (missing key, or the noData sentinel) -- matching controlsd's
    # _ach_lat_ms2(k_actl, CS.vEgo), which never fakes a 0.0 v_ego in the first place.
    raw_vego = tele.get("vEgo")
    no_data = tele.get("reason") == "noData"
    ach_lat = None if (no_data or raw_vego is None) else _ach_lat(self._sl_k_actl, raw_vego)
    # curvedbtel2pnw section 3.4 (Fable I1): the candidate distance mapLat/mapLon are resolved at.
    # ICBM's own latched one whenever icbmSrc names a map point -- including a FAR candidate out past
    # CES's ~308 m 10 s horizon, where `tele["mapDist"]` reads 0.0 and the coordinates came back null
    # on exactly the far-map phantoms section 3.4 exists to locate. Falls back to mapDist for every
    # other source, exactly as before.
    # getattr, like its _icbm_rcap/_icbm_ep neighbours below: a controller built before this feature
    # (or a permissive test stub) must degrade to the old mapDist behaviour, not raise mid-record.
    cand_d = getattr(self, "_icbm_cand_d", None)
    if cand_d is None:
      cand_d = tele.get("mapDist")
    rec = {
      "t": round(now_wall, 1),
      "ev": kind, "mode": tele.get("mode"), "reason": tele.get("reason"), "button": int(self._button),
      "vEgo": tele.get("vEgo"), "vSet": tele.get("vSet"), "aEgo": tele.get("aEgo"), "gas": tele.get("gas"),
      "accelZone": tele.get("accelZone"),
      # stophold2pnw (B): raw model stop intent — red-vs-green is decidable from the breadcrumb now
      "stp": tele.get("stp"),
      # stophold2pnw (D): car identity (shadow is no longer a car discriminator — alpha-long A/B)
      "car": self._car,
      "curvePct": tele.get("curvePct"), "curveSrc": tele.get("curveSrc"),
      # lowspeedcurve2pnw: raw vision-curve trigger inputs (why did curvePct stay 0? — Hwy 99
      # 2026-07-13). None on the noData path, same as every other tele passthrough.
      "visLat": tele.get("visLat"), "visTtc": tele.get("visTtc"), "blnk": tele.get("blnk"),
      "mapV": tele.get("mapV"), "mapDist": tele.get("mapDist"), "mapPts": tele.get("mapPts"),
      # mapd220-2pnw PHASE 1: mapd v2.2.0 highwayClass/conditionalSpeedLimit telemetry. PURE
      # OBSERVATION — this is the dataset a later phase validates curve-tiering/freeway-floor/
      # posted-limit gating against (docs/MAPD-V220-UPGRADE.md); nothing here changes any target.
      # condSpdLim truncated to bound the JSONL row size (raw OSM text, unbounded upstream).
      "hwyClass": self._hwy_class,
      "condSpdLim": (self._cond_spd_lim[:80] if self._cond_spd_lim else ""),
      "waySel": self._way_sel,          # waysel2pnw
      "wayOff": self._way_off,          # waysel2pnw
      "dRel": tele.get("dRel"), "vLead": tele.get("vLead"),
      # vtsctele2pnw: explicit lead-present bool + gap time (s) + lead speed delta (m/s)
      # leadrate2pnw: "hasLead" is an ALIAS of the same value as "lead" below (both come straight from
      # s["has_lead"] via decision_telemetry's tele dict) -- added under this name so tick/adopt and
      # the CES-off "steer" record (_steer_log_step) share one field name for the same fact.
      "lead": tele.get("lead"), "hasLead": tele.get("lead"), "gapS": tele.get("gapS"), "dV": tele.get("dV"),
      "gps": tele.get("gps"), "lat": self._cur_lat, "lon": self._cur_lon, "bearing": self._cur_bearing,
      "spdLim": round(self._speed_limit, 1), "hwy": bool(hwy),
      "car_gps": self._car_gps,           # cargps2pnw: Ford only; None on the Tesla
      "gpsSrc": self._gps_src,            # gpssel2pnw: when "car", lat/lon/bearing ARE the car_gps fix
      # VTSC applied cap + state (from the VTSCStatus mem param) — without this channel the 2026-07-06
      # I-84 gas-override cluster couldn't be attributed (VTSC/MTSC vs CES) from the log alone.
      "vtscCap": self._vtsc_cap, "vtscState": self._vtsc_state, **getattr(self, "_vtsc_tele", {}),
        **getattr(self, "_sa_tele", {}),
      # vtsctele2pnw: the penalty components VTSC actually applied (Lightning penalty m/s, the road
      # pitch it used, apex turn direction L/R) — 2026-07-12 westbound over-slow forensics needed
      # these and had to infer them.
      "vtscPen": self._vtsc_pen, "vtscPitch": self._vtsc_pitch, "vtscDir": self._vtsc_dir,
      # bsm2pnw: blind-spot booleans (carState.left/rightBlindspot) — liveness evidence for the
      # lane-change BSM gate; expect these to flip as traffic passes on real drives.
      "bsL": self._bs_l, "bsR": self._bs_r,
      # lanecenter2pnw telemetry: lane-centering trim status (from LaneCenterStatus) — display/log
      # only, same as vtscCap/vtscState above. lcGate explains why lcCorr isn't (fully) applied on
      # any given tick ("ok" = acting; see lane_centering.py's _finish_status for the full code list).
      "lcCorr": self._lc_corr, "lcAct": self._lc_act, "lcGate": self._lc_gate, "lcErr": self._lc_err,
      "lcP1": self._lc_p1, "lcP2": self._lc_p2, "lcS1": self._lc_s1, "lcS2": self._lc_s2,
      "lcYStd": self._lc_ystd, "lcW": self._lc_w,
      # lcroc2pnw: cumulative ticks the correction_roc growth cap clipped (diff consecutive records).
      "lcLimN": self._lc_lim_n, "lcSpdA": self._lc_spd_a,
      # steerlimit-log2pnw telemetry: steering-limit status (from SteerLimitStatus) — display/log
      # only, same as lc* above. PURE OBSERVATION: never gates or alters any control value. See
      # docs/STEERING-LIMITS.md for what each field means and how to read them together.
      "slCurvLim": self._sl_curv_lim, "slSafetyLim": self._sl_safe_lim,
      "slAngDes": self._sl_ang_des, "slAngAct": self._sl_ang_act, "slAngErr": self._sl_ang_err,
      "slLatDem": self._sl_lat_dem, "slLatMax": self._sl_lat_max, "slCurvMax": self._sl_curv_max,
      "slSat": self._sl_sat,
      # steertele2pnw: capability-analysis pair — slLatAct disambiguates openpilot-engaged steering
      # from manual maneuvering (angDes/angAct freeze together when False), slAngSat is the un-fused
      # angle-only half of slSat (slCurvLim already isolates the curvature half). See the
      # steer_limit_status comment block in controlsd.py for the exact derivation of each.
      "slLatAct": self._sl_lat_active, "slAngSat": self._sl_ang_sat,
      # fordkappalog2pnw: commanded vs achieved curvature (1/m, Ford wire convention positive=left) —
      # the empirical saturation signal to characterize this truck's real curvature limit. Display/log
      # only, same as sl* above. See docs/STEERING-LIMITS.md "Ford curvature interface" section.
      # curvedbtel2pnw section 3.2: slKActl exactly 0.0 is a DEAD SENSOR, not a straight road (it is
      # 0.0 on 7,218 of 7,221 moving Tesla ticks, and non-zero on 1,097 of 1,136 Lightning ones) ->
      # null. slKCmd/slKErr are deliberately untouched: a commanded exact zero is a genuine command,
      # and slLatAct already distinguishes "lateral was not active".
      "slKCmd": self._sl_k_cmd, "slKActl": _zero_is_null(self._sl_k_actl), "slKErr": self._sl_k_err,
      # coopsteer-shadow2pnw: SHADOW torque-nudge fields (from SteerLimitStatus) -- same fragment the
      # CES-off "steer" breadcrumb carries, so the sign question can be settled on any drive.
      "cpOff": self._cp_off, "cpTgt": self._cp_tgt, "cpCap": self._cp_cap, "cpWhy": self._cp_why,
      "cpTq": self._cp_tq, "cpRate": self._cp_rate, "cpCmd": self._cp_cmd,
      # steerpower2pnw: LOGGING ONLY — delivered lateral accel (m/s^2, signed) + 8-pt compass heading,
      # to measure the truck's true hands-off steering capability by direction (see module docstring
      # near _ach_lat/_compass).
      # I4 review fix: null (not "N") when there's no current GPS fix, rather than _compass() reading
      # a no-fix-defaulted 0.0 bearing as true north.
      # curvedbtel2pnw section 3.2: exact 0.0 -> null, same reason as slKActl above (achLat IS
      # slKActl * vEgo^2, so the dead Tesla sensor produced a confident 0.00 m/s^2 lateral accel --
      # the reading that voided the design's own v1 proof of concept).
      "achLat": _zero_is_null(round(ach_lat, 3) if ach_lat is not None else None),
      "heading": _heading_if_fixed(self._cur_bearing, self._cur_lat is not None and self._cur_lon is not None),
      # curvedbtel2pnw sections 3.1/3.3/3.4/3.5/3.6 -- TELEMETRY ONLY, nothing reads these. The
      # per-second curvature PEAK (kPeak/kPoseP, section 3.3) rather than the 1-in-100 sample, the
      # localizer-derived achieved curvature that gives the Tesla one at all (kPose/achLatPose,
      # section 3.1), the map candidate's OWN position (mapLat/mapLon, section 3.4 -- keyed on where
      # the curve is, not where the truck is), the per-second disqualifier roll-up (dq/dqWhy,
      # section 3.5) and driver steering torque (strTq, section 3.6). Keys pinned by CURVE_TELE_KEYS.
      # cand_d (above) is the candidate distance icbmSrc actually names, so mapLat/mapLon resolve the
      # SAME candidate the record names; it is echoed back out as mapCandD.
      **_curve_tele(self, raw_vego, cand_d),
      # icbm2pnw: steering angle + driver-override flag (lateral quality forensics), and the shadow
      # marker — True on the Lightning where the planner path never actuates (ICBM may).
      "strAng": self._str_ang, "strPrs": self._str_prs, "shadow": self._shadow,
      "mdlEndX": round(float(tele.get("mdlEndX") or 0.0), 1),
      # ces2core2pnw shadow A/B: CES2 would-be mode/reason, graded stop urgency, cumulative
      # divergence edges vs v1, and whether CES2 was LIVE (deciding) for this record.
      "ces2Mode": tele.get("ces2Mode"), "ces2Reason": tele.get("ces2Reason"),
      "ces2Urg": tele.get("ces2Urgency"), "ces2Div": tele.get("ces2Div"),
      "ces2Live": tele.get("ces2Live"),
      # icbm2pnw closed-loop trace: published curve target (m/s, None = ICBM idle), latched driver
      # ceiling, the truck's reported stock set speed + engagement. icbmT stepping the stockSet down
      # in consecutive ticks = executor taps landing.
      "icbmGpsAge": self._icbm_gps_age,   # gpslag2pnw: > ICBM_GPS_MAX_AGE_S = no map lookups that tick
      "icbmT": self._icbm_last_target, "icbmC": self._icbm_ceiling, "icbmSrc": self._icbm_src,
      # curvelead2pnw: icbmOwnT = ICBM's own curve target, icbmLeadT = the lead-paced cap that replaced it
      # (None = not relaxed), icbmLeadWhy = the gate that decided, icbmLeadS = seconds the lead has been
      # tracked, icbmKVis = the model's tightest curvature; icbmSaneT/icbmSaneWhy and icbmBehind are the
      # TELEMETRY-ONLY map-claim verdicts (B). Listed in CURVELEAD_TELE_KEYS -- pinned by a test.
      **_curvelead_tele(self),
      # icbmcurv2pnw: the map polyline's OWN geometry, beside the mapV/icbmT mapd asserted. NOT a
      # verdict: `icbmKN > 0 and icbmK ~= 0` does NOT mean "no curve" -- a real 90-degree corner
      # drawn with two 350 m legs among dense straight nodes reports exactly that, because only the
      # straight triplets clear the spacing gate (reproduced; pinned by test_icbmcurv.py's
      # test_a_coarse_corner_among_dense_nodes_is_a_MEASURED_ZERO). Read these as evidence, and read
      # docs/pnw/ICBMCURV2PNW.md section 5 before building any gate on them.
      "icbmK": round(float(self._icbm_k), 6), "icbmKD": round(float(self._icbm_k_dist), 0),
      "icbmKV": round(float(self._icbm_k_v), 1), "icbmKN": int(self._icbm_k_n),
      "icbmKAhead": bool(self._icbm_k_ahead),
      # icbmconsist2pnw (telemetry only)
      "icbmKAt": round(float(self._icbm_k_at), 6), "icbmKAtD": round(float(self._icbm_k_at_d), 0),
      "icbmKAtN": int(self._icbm_k_at_n), "icbmKAtGap": round(float(self._icbm_k_at_gap), 0),

      "icbmFlr": round(float(self._icbm_floor_lim), 1), "icbmFlrHit": bool(self._icbm_floor_hit),
      "icbmRCap": round(float(getattr(self, "_icbm_rcap", 0.0) or 0.0), 1),   # icbmrestorecap2pnw: restore cap, 0 = none
      "icbmPhase": getattr(getattr(self, "_icbm_ep", None), "phase", None),   # icbmrestorecap2pnw: idle/cap/restore
      "icbmZoneWhy": getattr(getattr(self, "_icbm_ep", None), "zone_why", None),      # sazoneset2pnw: prop/limit5/None
      "icbmLatchLim": getattr(getattr(self, "_icbm_ep", None), "latch_limit", None),  # sazoneset2pnw: limit at latch
      "icbmDir": self._icbm_dir,   # icbmrestore2pnw: "inc" rows in ces_events = restore taps
      "stockSet": self._stock_set, "stockOn": self._stock_on,
      # icbmmapfirst2pnw: start-gate + map coverage forensics (why vision did NOT initiate; whether
      # mapd was alive — mapReach 0/None with mapPts 0 = the mapd-outage signature).
      "icbmGate": self._icbm_gate, "mapReach": self._icbm_map_reach,
      # greenlead2pnw: detector state per record ("idle"/"armed"/"fired" — the arming trail) plus
      # a ONE-SHOT event marker ("green"/"lead"/"leadMoving") on the record that follows an actual
      # firing, else None. Replaces the old "greenLight" field, which logged the (always-truthy)
      # state and read as true in 13,427/13,433 records on the 2026-07-13 drive — the arming trail
      # and the event are now separate, meaningful channels.
      "glSt": self._gl.state,
      "glEv": self._gl_ev_pending,
    }
    self._gl_ev_pending = None   # consumed by exactly one record (adopt or tick, whichever is next)
    # stophold2pnw (D): mark (never drop) records written before the clock is plausibly synced —
    # the dead-RTC boot wrote a 2025-11-25-stamped record that corrupted the 07-12 gap analysis.
    if clock_bad(now_wall):
      rec["clockBad"] = True
    return rec

  def _append_event(self, rec: dict) -> None:
    """Append one JSON line to the persistent CES_EVENT_LOG (append-only, outside the overlay so it
    survives reboot + swaglog rotation). Best-effort; never breaks control — but repeated failure
    is no longer SILENT (stophold2pnw C: the 07-12 investigation burned hours proving a silence
    that wasn't; a genuinely dying writer must announce itself in swaglog)."""
    if not self._event_log_ok:
      return
    try:
      try:
        if os.path.getsize(CES_EVENT_LOG) > CES_EVENT_LOG_MAX_BYTES:
          rotate_event_log(CES_EVENT_LOG, CES_EVENT_LOG_GENERATIONS)
      except OSError:
        pass                                                # no file yet / stat race -> just append
      with open(CES_EVENT_LOG, "a") as f:
        f.write(json.dumps(rec) + "\n")
      self._append_fail = 0
    except Exception:
      # ~1-2 writes/s: warn once at 10 consecutive failures, then re-warn every ~600 (~5-10 min) —
      # loud enough to see, throttled enough to never flood swaglog. The write stays best-effort.
      self._append_fail += 1
      if self._append_fail == 10 or self._append_fail % 600 == 0:
        try:
          cloudlog.error(f"ces_pnw: ces_events append FAILING ({self._append_fail} consecutive) — telemetry trail is dark ({CES_EVENT_LOG})")
        except Exception:
          pass
