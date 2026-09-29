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
lat_accel_target(v) - 0.3, the steering ceiling 3.0). The min() is what keeps a 4.0 target from ever becoming a speed the
car cannot steer: the lataccel2pnw schedule (5.0 to 60 mph, 4.0 at 70, 3.0 from 80; flat 3.0 without a valid file), and
the vehicle-model steering-ANGLE clamp (carcontroller / panda: MAX_LATERAL_ACCEL 3.5886 = ISO 3.0 + a 0.6 bank
tolerance; the applied angle stalled at exactly that clamp on 2026-09-28 22:35, at 70.3 mph, and the car delivered
2.9-3.0 m/s^2). The EPS torque abort (2.7-3.8 Nm) is a third limit nothing here can see. So the speed for a row is
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
import threading

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.ces_pnw import curvedb_live as cl
from openpilot.selfdrive.controls.lib.ces_pnw.curvedb_live import (
  CurveDbLive, RAMP_CLASSES, UNKNOWN_CLASSES, scan_ahead)
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
             "cbRows", "cbCfg")


def row_speed(veh, k: float, v_ego: float) -> tuple[float | None, float | None]:
  """(v, A) for one row of curvature k on this car, or (None, None) when the car's target is unusable.

  A = the lowest curve_lat_a over the speeds involved: at the speed now, at the speed the first estimate asks for, and at
  the speed the second asks for. curve_lat_a never rises with speed on the shipped schedules, so the first is normally
  the smallest; taking the minimum makes the result safe for a schedule that is not monotonic. Pure given `veh`."""
  if not (math.isfinite(k) and k > 0.0 and math.isfinite(v_ego)):
    return None, None
  a = veh.curve_lat_a(max(v_ego, 0.0))
  if not (math.isfinite(a) and a > 0.0):
    return None, None
  for _ in range(2):
    a2 = veh.curve_lat_a(math.sqrt(a / k))
    if not (math.isfinite(a2) and a2 > 0.0):
      return None, None
    a = min(a, a2)
  return min(math.sqrt(a / k), V_NEED_MAX), a


def most_binding_row(idx, rows, s_ego: float, veh, v_ego: float, a_decel: float, finish_s: float = VC.APEX_FINISH_S):
  """(need dict, n_rows_priced) for the row that needs the LOWEST speed NOW through VTSC's own decel envelope
  (brake_cap_for_apex), or (None, n). d = the distance to 25 m before the anchor (0 once inside the row's extent),
  exactly as the Lightning prices the same rows. Pure."""
  best, best_env, n = None, float("inf"), 0
  for m in rows:
    v, a = row_speed(veh, m.k, v_ego)
    if v is None:
      continue
    n += 1
    d = max(m.s_anchor - idx.back - s_ego, 0.0)
    env = brake_cap_for_apex(v, d, v_ego, a_decel, finish_s)
    if env < best_env:
      best_env, best = env, {"v": v, "d": d, "a": a, "k": m.k, "row": m.row_id}
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
    mode = self.veh.curve_brain
    self._seq += 1
    out = {"ts": round(float(now), 3), "seq": self._seq, "mode": mode, "v": None, "d": None, "src": None, "ev": None,
           "a": None, "k": None, "row": None}
    rec = {"cbWhy": None, "cbN": 0, "cbErr": None}
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
          need, n = most_binding_row(self.db.index, rows, s_ego, self.veh, float(v_ego), float(a_decel))
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
               cbRows=self.db.index.n_rows if self.db.index is not None else 0, cbCfg=self.veh.curve_brain_why)
    if self._last and now - self._last_t <= 2.0 * PUBLISH_S + 0.5:
      L = self._last
      out.update(cbWhy=L["cbWhy"], cbV=L["v"], cbD=L["d"], cbSrc=L["src"], cbEv=L["ev"], cbA=L["a"], cbK=L["k"],
                 cbRow=L["row"], cbN=L["cbN"], cbSeq=L["seq"])
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
    if not 0.0 <= age <= ENTRY_MAX_AGE_S:
      return None, "stale", age
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
