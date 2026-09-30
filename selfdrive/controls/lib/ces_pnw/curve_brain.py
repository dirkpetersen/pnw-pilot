"""curvebrain2b2pnw (docs/SHARED-CURVE-BRAIN-DESIGN.md stage 3): the shared curve brain's NEED LAYER, Tesla consumer.

WHAT IT IS
==========
The Tesla's VTSC slows for curves it SEES (vision, 8 s) and for the ones mapd RATES (a number it scales up 1.35-1.8x).
It has never known the curves the owner has actually driven. The curve DB (curvedb_live.py: the measured curvature of
every road position/direction/branch the car has passed, >= 2 dates, one table for both cars) does. This module turns
that table into ONE number for the Tesla, once per ~0.25 s, in selfdrived:

    the speed the most binding DB row ahead needs at THE TESLA's lateral target    v = sqrt(A / k_row)

and publishes it as the `CurveBrain` mem-param (a heartbeat with or without a need). VTSC (plannerd) reads it and, in
"lower" mode, applies it as a LOWER-ONLY cap (vtsc_controller._apply_brain). One shared table; each car its own A (owner
2026-09-28) -- this file is the Tesla's; the Lightning keeps ICBM's own chain, untouched.

A IS A SPEED TARGET, NOT A STEERING CAPABILITY. A = PnwVehicle.curve_lat_a(v) = min(curve.json tesla.curve_lat_a (4.0),
lat_accel_target(v, tesla) - 0.3, the steering ceiling 3.5886). The min() clips a 4.0 target: the lataccel2pnw schedule
(latcar2pnw: the Tesla has its OWN "cars" entry in lataccel_limits.json, [[50,5],[60,5],[70,4],[80,3.9]], so the schedule
never binds and A is 3.5886 at every speed; with no valid per-car entry it falls to the shared 5.0 to 60 mph, 4.0 at 70, 3.0
from 80; flat 3.0 without a valid file, so a flat 2.7), and the vehicle-model steering-ANGLE clamp
(carcontroller / panda: MAX_LATERAL_ACCEL 3.5886 = ISO 3.0 + a 0.6 favourable-camber allowance). Owner 2026-09-29 set the
ceiling to that clamp itself, zero margin: the applied angle stalled at exactly it on 2026-09-28 22:35 (70.3 mph) and the car
delivered only 3.0-3.1 m/s^2 on that adverse camber. That is why KNOWN bad curves carry a per-curve override (Overrides below)
and why a missing override file drops every row to FAILSAFE_A.
The EPS torque abort (2.7-3.8 Nm) is a third limit nothing here can see. So the speed for a row is
priced at the LOWEST A over the speeds involved (the speed now, and the speed the row itself asks for), never above.

WHAT THE TESLA MAY ACT ON: only rows WITH authority (>= 2 dates), i.e. evidence "measured". Not built here, and said so
in the record rather than assumed: the corroborated-shape path (design s3.3 step 3) and the mapd-alone / vision
candidates (VTSC has its own; they are the phantom sources), so a curve the DB has no row for is left to VTSC exactly as
today.

NO EFFECT AT ALL (v is None, `why` names the gate) when: the mode is off; the DB is not loaded; GPS is stale / none;
mapd's way selection is not `current` (2 s hold, as the Lightning's); the road class is unknown or a ramp; mapd's path
is missing or the car is more than OFF_PATH_MAX_M off it; the Tesla lateral target is unusable. Every one is a `cbWhy`.
An exception costs the brain, never the actuator: `cbWhy = "err"`, `cbErr`, a throttled cloudlog.exception.

THIS MODULE DOES NOT IMPORT ces_pnw (no cycle). The caller (CESController) supplies the projected position. The DB is the
SAME file the Lightning reads (/data/pnw/curvedb_v2, curvedb_live.load_rows); nothing of it is copied anywhere else.
"""
from __future__ import annotations

import json
import math
import os
import stat
import threading

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.ces_pnw import curvedb_live as cl
from openpilot.selfdrive.controls.lib.ces_pnw.curvedb_live import (
  CurveDbLive, RAMP_CLASSES, UNKNOWN_CLASSES, scan_ahead)
from openpilot.selfdrive.controls.lib.pnw_vehicle import CURVE_OVERRIDE_PLATFORMS, CURVE_OVERRIDE_V1_PLATFORM
from openpilot.selfdrive.controls.lib.vtsc_pnw import vtsc_constants as VC
from openpilot.selfdrive.controls.lib.vtsc_pnw.vtsc_pnw import brake_cap_for_apex

HORIZON_M = VC.MAP_SOURCE_HORIZON_M     # 500 m: mapd's own path horizon; no path data exists beyond it
PUBLISH_S = 0.25                        # ~4 Hz, the ICBM cadence
ENTRY_MAX_AGE_S = 1.0                   # VTSC ignores an entry older than this (design s5.2)
OFF_PATH_MAX_M = 30.0                   # the car farther than this from mapd's path is not on it: no rows
V_NEED_MAX = 100.0                      # m/s; a row this fast is "no curve" -- keeps the payload finite and readable
ERR_LOG_S = 60.0                        # rule 2: the failure log is throttled (the step runs at ~4 Hz)
EVIDENCE = ("measured", "corroborated")  # the only `ev` values VTSC may act on (design s5.1)
MODES = ("off", "shadow", "lower", "raise")

# Every key tele() emits -- pinned by the tests: a key added here and not emitted (or vice versa) is a silently-null
# ces_events column (that happened four times in ces_pnw.py's history).
TELE_KEYS = ("cbOn", "cbDb", "cbWhy", "cbV", "cbD", "cbSrc", "cbEv", "cbA", "cbK", "cbRow", "cbN", "cbErr", "cbSeq",
             "cbRows", "cbCfg", "cbOvr", "cbOvrN")

# curvebrain2b2pnw A5 (owner 2026-09-29): the general steering ceiling is the vehicle-model clamp itself (3.6), which is acceptable
# only because KNOWN bad curves carry their own lower limit. The DB rows store only k, so the limits are a small separate PRIVATE
# file, shipped like the curve DB (source: ~/gh/comma/workdir/data/curve_overrides.json -> /data/pnw/curve_overrides.json).
#
# ovrcar2pnw SCHEMA v2 (owner 2026-09-29: PER-CAR values, ONLY the Tesla and the Lightning, DIRECTION mandatory):
#   {"version": 2, "overrides": [{"lat", "lon", "radius_m", "heading_deg", "heading_tol_deg",
#                                 "cars": {"<opendbc platform>": {"a_max": <m/s^2>}, ...}, "note"}, ...]}
# * platform keys are pnw_vehicle.CURVE_OVERRIDE_PLATFORMS (exactly the two cars); any other key makes the entry INVALID.
# * an entry applies to a car ONLY if that car's platform is a key of `cars`; each car's Overrides instance filters on its own.
# * heading_deg + heading_tol_deg are REQUIRED and 0 < heading_tol_deg <= HEADING_TOL_MAX (60): no entry can be omnidirectional.
# * A row whose ANCHOR lies within radius_m of (lat, lon) and whose approach heading is within heading_tol_deg of heading_deg is
#   priced at A_row = min(A, a_max): it can only LOWER a speed.
# * SCHEMA v1 COMPATIBILITY: a file with no "version" whose entries carry a top-level `a_max` and no `cars` (what shipped before
#   ovrcar2pnw) is read with every such entry as Tesla-only, with a cloudlog.warning "v1 entry: Tesla only". This is so a deploy
#   that lands the code before the v2 file does not drop the Tesla into fail-safe. Once the v2 file is installed it is never used.
#   A v2 entry (or a file with "version": 2) without `cars` is invalid; an entry with both `a_max` and `cars` is invalid.
# FAIL-SAFE (Rule 2) DIFFERS PER CAR, deliberately: a file that is missing, unreadable, corrupt or holds an invalid entry means the
# safety data cannot be trusted.
#   * TESLA: its general ceiling (3.6) is only safe WITH the overrides, so EVERY row is priced at no more than FAILSAFE_A until a
#     valid file is read.
#   * LIGHTNING: NO overrides are applied (and the error is logged). The Lightning is NOT lowered globally: its curve-DB A
#     (curve.json curvedb_v2_lat_a, default 2.5) already sits below the Tesla's fail-safe 2.8, so the general ceiling never
#     relied on this file; lowering it further would only slow a car whose own default is the conservative one.
# `{"overrides": []}` is valid and means no overrides. Hot-reloaded on the curve.json cadence; a bad edit mid-drive keeps the last
# valid list (error logged) on both cars; a file that DISAPPEARS mid-drive is fail-safe (Tesla) / no overrides (Lightning).
OVERRIDES_PATH = "/data/pnw/curve_overrides.json"
OVERRIDES_MAX_BYTES = 64 * 1024
OVERRIDES_POLL_S = 1.0
OVERRIDES_MAX_ENTRIES = 50   # more is treated as invalid (fail-safe): the file is a short list of known bad curves
NOTE_MAX = 120               # characters of an entry's note kept (it is what cbOvr reports)
HEADING_TOL_MAX = 60.0       # ovrcar2pnw: the widest direction window an entry may have (owner: some curves are one-direction problems)
FAILSAFE_A = 2.8            # == pnw_vehicle.CURVE_STEER_FALLBACK (pinned by a test)


def _dist_m(lat1, lon1, lat2, lon2) -> float:
  return cl.haversine_m(lat1, lon1, lat2, lon2)


class Overrides:
  """The per-curve, per-car lateral-acceleration limits. Never raises. `entries` is None until a valid file has been read (=
  fail-safe). `platform` = the car this instance serves (pnw_vehicle.PnwVehicle.curve_override_platform); `entries` holds ONLY
  that car's limits, each as a flat {lat, lon, radius_m, heading_deg, heading_tol_deg, a_max, note}. The whole file is validated
  whichever car reads it, so a bad entry for the other car is still an error (the file is one safety document)."""

  _GEOM = ("lat", "lon", "radius_m", "heading_deg", "heading_tol_deg")

  def __init__(self, path: str | None = None, platform: str = CURVE_OVERRIDE_V1_PLATFORM):
    if platform not in CURVE_OVERRIDE_PLATFORMS:
      raise ValueError(f"Overrides: platform {platform!r} is not one of {CURVE_OVERRIDE_PLATFORMS}")
    self.platform = platform
    self.path = path if path is not None else OVERRIDES_PATH        # looked up at construction (tests redirect the module path)
    self.entries: list[dict] | None = None
    self.why = "not read yet"
    self._sig = "unset"
    self._poll = -1e9
    self._err_t = None
    self.refresh(0.0)

  @property
  def failsafe(self) -> bool:
    return self.entries is None

  @staticmethod
  def _num(e, n, k):
    x = e.get(k)
    if isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x):
      raise ValueError(f"entry {n}: {k} is not a finite number")
    return float(x)

  def _validate(self, doc) -> list[dict]:
    """The whole document -> this car's entries (flat). Raises ValueError on anything invalid."""
    if not isinstance(doc, dict) or not isinstance(doc.get("overrides"), list):
      raise ValueError('expected {"version": 2, "overrides": [...]}')
    ver = doc.get("version", 1)
    if isinstance(ver, bool) or ver not in (1, 2):
      raise ValueError(f"unknown schema version {ver!r} (expected 2, or 1 = the legacy Tesla-only form)")
    if len(doc["overrides"]) > OVERRIDES_MAX_ENTRIES:
      raise ValueError(f"{len(doc['overrides'])} entries > {OVERRIDES_MAX_ENTRIES}")
    out = []
    for n, e in enumerate(doc["overrides"]):
      if not isinstance(e, dict):
        raise ValueError(f"entry {n} is not an object")
      g = {k: self._num(e, n, k) for k in self._GEOM}     # heading_deg / heading_tol_deg are REQUIRED numbers (direction mandatory)
      if not (-90.0 <= g["lat"] <= 90.0 and -180.0 <= g["lon"] <= 180.0 and 0.0 < g["radius_m"] <= 2000.0
              and 0.0 <= g["heading_deg"] <= 360.0):
        raise ValueError(f"entry {n}: a value is out of range")
      if not 0.0 < g["heading_tol_deg"] <= HEADING_TOL_MAX:
        raise ValueError(f"entry {n}: heading_tol_deg {g['heading_tol_deg']} not in (0, {HEADING_TOL_MAX}] -- every entry is one direction")
      note = str(e.get("note", ""))[:NOTE_MAX]
      if "cars" in e:
        if "a_max" in e:
          raise ValueError(f"entry {n}: has both a top-level a_max and cars")
        if ver != 2:
          raise ValueError(f"entry {n}: `cars` needs \"version\": 2")
        cars = e["cars"]
        if not isinstance(cars, dict) or not cars:
          raise ValueError(f"entry {n}: cars must be a non-empty object")
        for plat, spec in cars.items():
          if plat not in CURVE_OVERRIDE_PLATFORMS:
            raise ValueError(f"entry {n}: platform {plat!r} is not one of {CURVE_OVERRIDE_PLATFORMS}")
          if not isinstance(spec, dict):
            raise ValueError(f"entry {n}: cars.{plat} is not an object")
          a = self._num(spec, n, "a_max")
          if not 1.0 <= a <= 5.0:
            raise ValueError(f"entry {n}: cars.{plat}.a_max {a} out of range [1, 5]")
          if plat == self.platform:
            out.append({**g, "a_max": a, "note": note})
      else:
        if ver != 1:
          raise ValueError(f"entry {n}: a version-2 entry needs `cars`")
        a = self._num(e, n, "a_max")
        if not 1.0 <= a <= 5.0:
          raise ValueError(f"entry {n}: a_max {a} out of range [1, 5]")
        cloudlog.warning(f"curve_brain: {self.path} entry {n}: v1 entry: Tesla only (legacy schema, no `cars`; install the v2 file)")
        if self.platform == CURVE_OVERRIDE_V1_PLATFORM:
          out.append({**g, "a_max": a, "note": note})
    return out

  def _say(self, now, msg, force=False):
    if force or self._err_t is None or now - self._err_t >= ERR_LOG_S:
      cloudlog.error(msg)
      self._err_t = now

  def _failsafe_msg(self):
    if self.platform == CURVE_OVERRIDE_V1_PLATFORM:
      return (f"curve_brain: {self.path} INVALID/MISSING ({self.why}) -- every curve is priced at no more than {FAILSAFE_A} m/s^2 " +
              "until a valid file is read (the general ceiling is only safe with the per-curve overrides)")
    return (f"curve_brain: {self.path} INVALID/MISSING ({self.why}) -- NO per-curve overrides are applied for {self.platform} " +
            "until a valid file is read (its curve-DB A is not raised, and is not lowered globally either)")

  def refresh(self, now: float) -> None:
    """Re-read the file when its (mtime, size) changed, polled at most every OVERRIDES_POLL_S. Never raises."""
    try:
      if now - self._poll < OVERRIDES_POLL_S:
        return
      self._poll = now
      try:
        st = os.stat(self.path)
        sig = (st.st_mtime_ns, st.st_size, st.st_ino, st.st_ctime_ns)   # ctime/inode: an mtime-preserving edit still changes them
      except FileNotFoundError:
        st, sig = None, None
      if sig == self._sig:
        if self.failsafe:                              # keep saying it, once a minute, while it lasts
          self._say(now, self._failsafe_msg())
        return
      self._sig = sig
      try:
        if sig is None:
          raise FileNotFoundError("missing")
        if not stat.S_ISREG(st.st_mode) or st.st_size > OVERRIDES_MAX_BYTES:
          raise ValueError(f"not a regular file, or {st.st_size} B > {OVERRIDES_MAX_BYTES} B")
        with open(self.path) as f:
          new = self._validate(json.load(f))
      except FileNotFoundError:
        self.entries, self.why = None, "missing"          # a safety file that disappears is fail-safe, even mid-drive
        self._say(now, self._failsafe_msg(), force=True)
      except Exception as e:
        why = f"{type(e).__name__}: {e}"
        if self.entries is None:
          self.why = why
          self._say(now, self._failsafe_msg(), force=True)
        else:
          self._say(now, f"curve_brain: {self.path} changed but is INVALID ({why}) -- NOT applied, keeping the last valid " +
                         f"list ({len(self.entries)} entries)", force=True)
      else:
        changed = self.entries is None or new != self.entries
        self.entries, self.why = new, "ok"
        if changed:
          cloudlog.event("curve_brain_overrides", n=len(new), path=self.path)
    except Exception as e:                              # nothing above should raise; if it does, fail SAFE and say so
      self.entries, self.why = None, f"refresh crashed: {type(e).__name__}"
      self._say(now, self._failsafe_msg(), force=True)

  def limit(self, lat: float, lon: float, heading: float) -> tuple[float | None, str | None]:
    """(a_max, note) of the lowest override whose circle and heading window contain this row anchor, else (None, None)."""
    best, note = None, None
    for e in self.entries or ():
      d = _dist_m(lat, lon, e["lat"], e["lon"])
      dh = abs((heading - e["heading_deg"] + 180.0) % 360.0 - 180.0)
      if d <= e["radius_m"] and dh <= e["heading_tol_deg"] and (best is None or e["a_max"] < best):
        best, note = e["a_max"], e["note"] or f"@{e['lat']:.4f},{e['lon']:.4f}"
    return best, note


def row_speed(veh, k: float, v_ego: float, a_cap: float | None = None) -> tuple[float | None, float | None]:
  """(v, A) for one row of curvature k on this car, or (None, None) when the car's target is unusable.

  A = the lowest curve_lat_a over the speeds involved: at the speed now, at the speed the first estimate asks for, and at
  the speed the second asks for. curve_lat_a never rises with speed on the shipped schedules, so the first is normally
  the smallest; taking the minimum makes the result safe for a schedule that is not monotonic. `a_cap` (a per-curve override or
  the fail-safe) only ever LOWERS A. Pure given `veh`."""
  base = _price(veh, k, v_ego, None)
  if a_cap is None or base[0] is None:
    return base
  capped = _price(veh, k, v_ego, float(a_cap))
  # ovrcar2pnw: LOWER-ONLY must hold in SPEED, not just in A. The 2-round fixed point below can stop short of convergence on a
  # non-monotonic schedule, so a cap ABOVE the natural A converges further and prices a row up to ~1.4 mph FASTER than no cap
  # (found on the real table: the Terwilliger k 0.0023 row, cap 2.8 -> 78.0 vs 76.5 mph). An override may never do that.
  return capped if capped[0] is not None and capped[0] <= base[0] else base


def _price(veh, k: float, v_ego: float, a_cap: float | None) -> tuple[float | None, float | None]:
  if not (math.isfinite(k) and k > 0.0 and math.isfinite(v_ego)):
    return None, None
  cap = float("inf") if a_cap is None else float(a_cap)
  a = veh.curve_lat_a(max(v_ego, 0.0))
  if not (math.isfinite(a) and a > 0.0):
    return None, None
  a = min(a, cap)
  for _ in range(2):
    a2 = veh.curve_lat_a(math.sqrt(a / k))
    if not (math.isfinite(a2) and a2 > 0.0):
      return None, None
    a = min(a, a2, cap)
  return min(math.sqrt(a / k), V_NEED_MAX), a


def most_binding_row(idx, rows, s_ego: float, veh, v_ego: float, a_decel: float, finish_s: float = VC.APEX_FINISH_S,
                     overrides: Overrides | None = None):
  """(need dict, n_rows_priced) for the row that needs the LOWEST speed NOW through VTSC's own decel envelope
  (brake_cap_for_apex), or (None, n). d = the distance to 25 m before the anchor (0 once inside the row's extent),
  exactly as the Lightning prices the same rows. `overrides` lowers A per row (or for all rows when it is in fail-safe);
  the returned need carries `ovr` = the override note that priced it ("failsafe" in fail-safe), else None."""
  best, best_env, n = None, float("inf"), 0
  for m in rows:
    a_cap, ovr = None, None
    if overrides is not None:
      if overrides.failsafe:
        a_cap, ovr = FAILSAFE_A, "failsafe"
      else:
        a_cap, ovr = overrides.limit(m.lat, m.lon, idx.anchors[m.anchor][2])
    v, a = row_speed(veh, m.k, v_ego, a_cap)
    if v is None:
      continue
    if ovr is not None and a < a_cap - 1e-9:
      ovr = None                                       # the override did not bind (a lower clip did): do not claim it
    n += 1
    d = max(m.s_anchor - idx.back - s_ego, 0.0)
    env = brake_cap_for_apex(v, d, v_ego, a_decel, finish_s)
    if env < best_env:
      best_env, best = env, {"v": v, "d": d, "a": a, "k": m.k, "row": m.row_id, "ovr": ovr}
  return best, n


class CurveBrain:
  """The Tesla's need layer. One instance per CESController on a car whose VTSC may consume it
  (PnwVehicle.curve_brain_vtsc). Owns its own read-only CurveDbLive index (loaded once in a thread; cdb2On on the
  Lightning-side `_roaddb` is unaffected and stays "off" here)."""

  def __init__(self, veh, db: CurveDbLive | None = None, start: bool = True):
    self.veh = veh
    self.db = db if db is not None else CurveDbLive(True, start=False)   # start=False: no mapd-A poll thread (unused here)
    self._seq = 0
    self._last: dict = {}
    self._last_t = -1e9
    self._err_t = None
    self._err_n = 0
    self.err = None
    if veh.curve_override_platform is None:              # the brain is Tesla-only; a car without the capability must not build one
      raise ValueError("CurveBrain built for a car without curve_override_platform")
    self.overrides = Overrides(platform=veh.curve_override_platform)
    if db is None and start:
      if cl.BACKGROUND[0]:
        threading.Thread(target=self.db.load, name="curvebrain_db", daemon=True).start()
      else:                                                  # test harness only (roaddb_fixture.py): deterministic
        self.db.load()

  # -- the step ------------------------------------------------------------------------------------
  def step(self, now, *, v_ego, points, plat, plon, gps_state, way_sel, hwy, a_decel) -> dict:
    """One ~4 Hz tick -> the CurveBrain payload (always a heartbeat: ts / seq / mode; v is None without a need).
    Never raises."""
    try:
      self.veh.refresh_curve_brain_cfg(now)             # the owner's kill switch: cheap, throttled, never raises
    except Exception as e:                              # rule 2: it is documented never to raise; if it does, say so
      self._note_err(now, e, "curve.json hot-reload")
    self.overrides.refresh(now)                         # the per-curve limits: same 1 s poll as curve.json; never raises
    mode = self.veh.curve_brain
    self._seq += 1
    out = {"ts": round(float(now), 3), "seq": self._seq, "mode": mode, "v": None, "d": None, "src": None, "ev": None,
           "a": None, "k": None, "row": None}
    rec = {"cbWhy": None, "cbN": 0, "cbErr": None, "cbOvr": None}
    try:
      # tracked on EVERY tick before any gate, so the last-"current" time stays true while another gate is closed
      ws = self.db.way_sel_held(way_sel, now)[0] if mode != "off" else way_sel
      why = self._gate(mode, plat, plon, gps_state, ws, hwy, points)
      if why is None:
        poly = self.db.polyline(points)
        s_ego, off = poly.project(plat, plon)
        if off > OFF_PATH_MAX_M:
          why = "offPath"
        else:
          rows = scan_ahead(self.db.index, poly, s_ego, HORIZON_M)
          need, n = most_binding_row(self.db.index, rows, s_ego, self.veh, float(v_ego), float(a_decel),
                                     overrides=self.overrides)
          rec["cbOvr"] = need["ovr"] if need is not None else None
          rec["cbN"] = n
          if need is None:
            why = "noRow" if not rows else "noA"
          else:
            out.update(v=round(need["v"], 2), d=round(need["d"], 0), src="db", ev="measured", a=round(need["a"], 2),
                       k=round(need["k"], 6), row=need["row"])
            why = "ok"
      rec["cbWhy"] = why
    except Exception as e:
      rec["cbWhy"], rec["cbErr"] = "err", type(e).__name__
      out.update(v=None, d=None, src=None, ev=None, a=None, k=None, row=None)
      self._note_err(now, e, "need layer")
    self._last, self._last_t = {**out, **rec}, now
    return out

  def _gate(self, mode, plat, plon, gps_state, ws, hwy, points):
    if mode == "off":
      return "off"
    if self.db.state != "ok":
      return "db" + self.db.state.capitalize()
    if gps_state not in ("proj", "raw") or plat is None or plon is None:
      return "gps" + str(gps_state)
    if ws != "current":                                 # (a 2 s flicker is ridden through, as on the Lightning)
      return "waySel"
    if hwy in UNKNOWN_CLASSES or hwy in RAMP_CLASSES or hwy is None:
      return "class"
    if not self.db.polyline(points).ok:
      return "noPath"
    return None

  def _note_err(self, now, e, what):
    self._err_n += 1
    self.err = f"{type(e).__name__}"
    if self._err_t is None or now - self._err_t >= ERR_LOG_S:
      cloudlog.exception(f"curve_brain: {what} FAILED ({type(e).__name__}) -- the Tesla gets NO curve-DB need while this " +
                         f"lasts, VTSC runs exactly as before ({self._err_n} failure(s) since the last log)")
      self._err_t, self._err_n = now, 0

  # -- telemetry -----------------------------------------------------------------------------------
  def tele(self, now) -> dict:
    """The ces_events fragment. Liveness always; the latest tick's fields only while fresh (a brain that stopped
    stepping reads cbWhy "idle", never a stale need beside a live record)."""
    out = dict.fromkeys(TELE_KEYS)
    out.update(cbOn=self.veh.curve_brain, cbDb=self.db.state, cbErr=self.db.err if self.db.state == "err" else None,
               cbRows=self.db.index.n_rows if self.db.index is not None else 0, cbCfg=self.veh.curve_brain_why,
               cbOvrN=len(self.overrides.entries) if self.overrides.entries is not None else None)   # None = FAIL-SAFE (0 = valid, empty)
    if self._last and now - self._last_t <= 2.0 * PUBLISH_S + 0.5:
      L = self._last
      out.update(cbWhy=L["cbWhy"], cbV=L["v"], cbD=L["d"], cbSrc=L["src"], cbEv=L["ev"], cbA=L["a"], cbK=L["k"],
                 cbRow=L["row"], cbN=L["cbN"], cbSeq=L["seq"], cbOvr=L["cbOvr"])
      if L["cbErr"]:
        out["cbErr"] = L["cbErr"]
    else:
      out["cbWhy"] = "idle"
    return out


def parse_entry(raw, now) -> tuple[dict | None, str | None, float | None]:
  """VTSC's side of the contract: the CurveBrain param -> (entry, problem, age).

  entry: a dict with finite v > 0, finite d >= 0, ev in EVIDENCE, mode in MODES, age in [0, ENTRY_MAX_AGE_S]; else None
  and `problem` names why: "absent" (nothing published yet), "bad" (malformed), "stale" (older than ENTRY_MAX_AGE_S, or a
  timestamp from the future / another boot), "noNeed" (a live heartbeat with no row -- not a problem, the normal case).
  Pure; never raises (any surprise is "bad")."""
  try:
    if raw in (None, b"", ""):
      return None, "absent", None
    if isinstance(raw, (bytes, str)):
      raw = json.loads(raw)
    if not isinstance(raw, dict) or not raw:
      return None, ("absent" if raw == {} else "bad"), None
    ts = raw.get("ts")
    if isinstance(ts, bool) or not isinstance(ts, (int, float)) or not math.isfinite(ts):
      return None, "bad", None
    age = float(now) - float(ts)
    # A publish that landed just AFTER the reader captured `now` (plus ts rounding) has a small negative age: fresh, not
    # stale. Only a timestamp further in the future than one publish period (another boot) is refused.
    if not -PUBLISH_S <= age <= ENTRY_MAX_AGE_S:
      return None, "stale", age
    age = max(age, 0.0)
    mode = raw.get("mode")
    if mode not in MODES:
      return None, "bad", age
    v = raw.get("v")
    if v is None:
      return {"mode": mode, "v": None}, "noNeed", age
    d = raw.get("d")
    if (isinstance(v, bool) or isinstance(d, bool) or not isinstance(v, (int, float)) or not isinstance(d, (int, float))
            or not (math.isfinite(v) and v > 0.0 and math.isfinite(d) and d >= 0.0) or raw.get("ev") not in EVIDENCE):
      return None, "bad", age
    return {"mode": mode, "v": float(v), "d": float(d), "ev": raw["ev"], "src": raw.get("src"), "row": raw.get("row"),
            "a": raw.get("a"), "k": raw.get("k")}, None, age
  except Exception:   # anything else in an untrusted param is "malformed": the caller counts and logs it
    return None, "bad", None
