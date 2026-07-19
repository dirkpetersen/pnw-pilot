#!/usr/bin/env python3
"""
angle_curve_logger.py — curve-triggered event recorder for Alan Polk's angle-steering tuning.

WHY EVENT-TRIGGERED: continuous 20 Hz logging costs ~40 MB/h; /data is 90% full. Straight-line
driving carries no tuning information (Alan Polk: "+/-5 degrees is basically nothing, this is just
road noise"). So: hold everything in a RAM ring buffer, and commit a window to disk only when a real
curve completes. The ring buffer is what makes this work — by the time a curve is detected, its
ENTRY has already happened, and entry tracking is half the measurement.

WHAT IT RECORDS (Alan Polk's green/yellow lateral-debug chart, as data):
  GREEN  cmd_sa = carControl.actuators.steeringAngleDeg  (desired; both cars run steerControlType=angle)
  YELLOW sa     = carState.steeringAngleDeg              (achieved)
  Both in degrees, directly comparable — his rule is "yellow must not exceed green" AT THE PEAK.
The green/yellow pair is CAR-AGNOSTIC and is the core dataset. On top of it, the wire truth is
decoded from sendcan per brand (see MULTI-CAR below) so a shortfall can be attributed to a pipeline
stage, and lane position is kept so a departure is measured, not inferred.

MULTI-CAR: one comma 3X moves between a Ford F-150 Lightning and a Tesla Model S HW3 (Raven).
The car is derived at runtime from the CarParams param (live, falling back to persistent) — never
hardcoded — and re-derived after every data gap / offroad period, so a device swap between cars
does NOT need a logger restart (nothing supervises this process for a month; exiting would strand
it). Every summary and every session marker carries {brand, fp, steer_type}. Wire decode per brand:
  ford  : sendcan bus 0, msg LateralMotionControl2 (982):
          wire_angle = LatCtlPath_An_Actl [rad]   — wire value is the NEGATED internal path_angle
          (carcontroller negates all four LMC signals on the wire; recorded AS-IS, un-negated)
          wire_mode  = LatCtl_D2_Rq (0 = human-turn override / blip commanded nothing)
          wire_c2/c3 = LatCtlCurv_No_Actl / LatCtlCrv_NoRate2_Actl — MUST stay 0 in angle mode
          wire_off   = LatCtlPathOffst_L_Actl [m]
  tesla : sendcan bus 0 (CANBUS.party), msg DAS_steeringControl (1160, TeslaCANRaven):
          wire_angle = DAS_steeringAngleRequest [deg] — teslacan_legacy packs -apply_angle,
          so the wire value is the NEGATED commanded angle (recorded AS-IS, un-negated)
          wire_mode  = DAS_steeringControlType (0 = not enabled)
          wire_c2/c3/wire_off = None (no such signals on this message)
  Wire values are raw signal values in the DBC's units — analysis applies the per-brand negation
  above; do NOT compare wire_angle across brands (rad vs deg, different hardware).
If the car is unknown (MOCK / mid-fingerprint / no CarParams yet), the wire fields are null and the
car-agnostic green/yellow logging continues — never crash, never refuse to log.

VALIDITY IS FLAGGED, NOT FILTERED. ICBM cannot be disabled for everyday driving, and curves taken
while following, braking, or hand-steering are still worth having. Every curve gets flags; analysis
filters later. You can always drop a flagged curve; you can never recover one you didn't write.

INDEXING: rows carry monotonically-increasing ABSOLUTE sample counters (abs_n); the deque evicts
from the front once full, so raw deque indices go stale — all window bookkeeping is done in absolute
counters + monotonic timestamps and converted to ring offsets only at slice time. Durations are
timestamp-based, never index*DT (the loop is decimated to 20 Hz from a 100 Hz carState stream, and
must stay correct even if the achieved rate drifts).

OUTPUT (both under /data/dirk/angle_curves/):
  curves.jsonl              one SUMMARY line per curve + session markers — the aggregatable dataset
  traces/<id>.jsonl         full 20 Hz trace for that curve (pre-roll + curve + post-roll)

Run detached on the device:
  setsid /usr/local/venv/bin/python3 /data/dirk/angle_curve_logger.py >> /data/dirk/angle_curves.log 2>&1 &
"""
import atexit
import importlib
import json
import os
import sys
import time
import traceback
from collections import deque

# Self-contained path setup — a detached setsid/nohup shell does not reliably inherit an exported
# PYTHONPATH, which silently cost us a drive's data on 2026-07-19.
for _p in ("/data/openpilot", "/data/openpilot/opendbc_repo"):
  if _p not in sys.path:
    sys.path.insert(0, _p)

import cereal.messaging as messaging
from cereal import car as car_capnp
from opendbc.can.parser import CANParser

# Alerts openpilot raises when lateral is at/над its limit. These are GROUND TRUTH — the system
# telling the driver it could not hold the path — and cannot be reconstructed from signals after the
# fact. Alan Polk describes seeing exactly this in his own drive video ("a flash saying that we're
# getting off center of our path"). Any curve carrying one of these is a limit event, whatever the
# peak comparison says.
LIMIT_ALERT_TYPES = ("steerSaturated", "steerRequired", "preLaneChange", "ldw",
                     "steerTempUnavailable", "belowSteerSpeed", "steerUnavailable")

try:
  from openpilot.common.params import Params
except ImportError:
  from common.params import Params

OUT_DIR = "/data/dirk/angle_curves"
TRACE_DIR = os.path.join(OUT_DIR, "traces")
SUMMARY = os.path.join(OUT_DIR, "curves.jsonl")
LOCK = os.path.join(OUT_DIR, "logger.lock")
TUNING_JSON = "/data/pnw/angle_tuning.json"   # the config in force — embedded per curve (Ford tuning overlay)

RATE_HZ = 20.0
DT = 1.0 / RATE_HZ

# --- Curve trigger -----------------------------------------------------------------------------
# Alan Polk's own "this is a real curve" threshold: curvature_factor_bp_hi = 0.001 1/m is where his
# gain logic switches fully to the curve gain. Using his constant means we capture exactly the
# regime his tuning method applies to, and it sits far above the road-noise floor.
# Trigger source is actuators.curvature (the planner's wish): when disengaged it is ~0, so
# HAND-DRIVEN curves are NOT captured — deliberate; they have no green line and would burn the
# disk budget on untunable records. Partial disengagement mid-curve IS captured (flagged).
CURVE_ON_KAPPA = 0.001         # 1/m — enter curve state
CURVE_OFF_KAPPA = 0.0005       # 1/m — leave curve state (hysteresis, prevents chatter at the edge)
CURVE_OFF_HOLD_S = 2.0         # must stay below OFF for this long before the curve is "done"
MIN_CURVE_S = 0.8              # ignore blips shorter than this
MIN_TRIGGER_V = 2.0            # m/s — no triggering at parking speeds

PRE_ROLL_S = 8.0               # kept before curve entry
POST_ROLL_S = 8.0              # recorded after curve exit — the EXIT-UNWIND HOLD lives here
MAX_CURVE_S = 30.0             # driver's call: a curve can legitimately run this long
GAP_S = 3.0                    # a hole this long in carState = new session (ignition cycle, car swap)
# Ring must hold pre-roll + the longest curve + off-hold + post-roll, with headroom. A pending
# curve is flushed exactly POST_ROLL_S after its end, so its oldest needed sample is at most
# PRE + MAX + OFF_HOLD + POST = 48 s old < RING_S — no pending window can be evicted.
RING_S = PRE_ROLL_S + MAX_CURVE_S + POST_ROLL_S + 5.0
RING_N = int(RING_S * RATE_HZ)
PRE_N = int(PRE_ROLL_S * RATE_HZ)
POST_N = int(POST_ROLL_S * RATE_HZ)

# --- Disk guard --------------------------------------------------------------------------------
MAX_BYTES = 500 * 1024 * 1024        # 500 MB cap for traces/, oldest-first rotation
SUMMARY_MAX_BYTES = 50 * 1024 * 1024  # curves.jsonl cap; rolled once to .1 (bounded 100 MB total)
MIN_FREE_TRACE = 1024 * 1024 * 1024  # below 1 GB free on /data: stop writing traces
MIN_FREE_ANY = 200 * 1024 * 1024     # below 200 MB free: stop writing anything
CHECK_EVERY_N_CURVES = 5

NOISE_DEG = 5.0                # Alan Polk: +/-5 deg of wheel movement is road noise, not signal
PEAK_DELTA_DEG = 3.0           # verdict deadband on |yellow|-|green| at the peak (stricter than the
                               # raw-trace 5 deg floor, per AUTOTUNER-DESIGN C.2)
WIRE_FRESH_S = 0.5             # wire decode considered live if the msg parsed within this window

# --- per-brand wire decode (small table, keyed off live CarParams.brand — see header) -----------
WIRE_SPECS = {
  "ford":  {"bus": 0, "bus_key": "pt", "msg": "LateralMotionControl2", "rate": 20,
            "angle": "LatCtlPath_An_Actl", "mode": "LatCtl_D2_Rq",
            "c2": "LatCtlCurv_No_Actl", "c3": "LatCtlCrv_NoRate2_Actl", "off": "LatCtlPathOffst_L_Actl"},
  "tesla": {"bus": 0, "bus_key": "party", "msg": "DAS_steeringControl", "rate": 50,
            "angle": "DAS_steeringAngleRequest", "mode": "DAS_steeringControlType",
            "c2": None, "c3": None, "off": None},
}


def free_bytes():
  st = os.statvfs("/data")
  return st.f_bavail * st.f_frsize


def rotate(path, cap):
  """Oldest-first deletion so this can never fill /data (90% full as of 2026-07-19)."""
  try:
    files = sorted((os.path.join(path, f) for f in os.listdir(path)), key=os.path.getmtime)
    total = sum(os.path.getsize(f) for f in files)
    while total > cap and files:
      victim = files.pop(0)
      total -= os.path.getsize(victim)
      os.remove(victim)
  except Exception as e:
    print(f"rotate failed: {e}", flush=True)


def append_summary(obj):
  try:
    if os.path.exists(SUMMARY) and os.path.getsize(SUMMARY) > SUMMARY_MAX_BYTES:
      os.replace(SUMMARY, SUMMARY + ".1")   # keep exactly one rolled generation
    with open(SUMMARY, "a") as sf:
      sf.write(json.dumps(obj) + "\n")
  except Exception as e:
    print(f"summary write failed: {e}", flush=True)


def trace_worth_keeping(summ):
  """Which curves earn a full ~292 kB 20 Hz trace. The 2.7 kB summary is always kept.

  Measured 2026-07-19: 55 curves = 16 MB, and the large majority were parking-lot / intersection
  maneuvers (R < 40 m, disengaged, hand-steered) from which nothing can be learned — pure waste.
  Keep a trace only where someone would actually open it:
    - clean_for_tuning        : the tuning dataset itself
    - limit_alert             : openpilot said "Take Control" — ground truth, always keep
    - angle_saturated >= 5%   : sustained tracking divergence
    - exit_unwind_hold        : the open defect under investigation
    - a real road curve with real amplitude, even if flagged (R > 80 m and green > 10 deg) —
      flagged-but-large curves are where the confounds themselves are diagnosed
  """
  f = summ.get("flags", {})
  k = summ.get("kappa_peak") or 0
  radius = (1.0 / k) if k else 0.0

  # A clean curve is the dataset — always keep, whatever its shape.
  if f.get("clean_for_tuning"):
    return True

  # Everything else must be a ROAD curve to be worth 292 kB. Verified against the 2026-07-19
  # session: without this gate, 7-24 m parking/intersection turns qualified on saturation alone
  # (they saturate trivially — that is the envelope, not information) and the filter saved only 47%.
  if radius <= 40.0:
    return False

  return bool(
    f.get("limit_alert")                              # openpilot said "Take Control" on a real curve
    or f.get("exit_unwind_hold")                      # the open defect
    or (f.get("angle_saturated_frac") or 0) >= 0.05   # sustained tracking divergence
    or abs(summ.get("green_peak_deg") or 0) > 10.0    # real amplitude: confounds get diagnosed here
  )


def read_tuning():
  """Snapshot of the tuning overlay in force (hot-reloads every ~5 s on the car) — per-curve, so
  config boundaries never have to be reconstructed from file mtimes again (2026-07-19 lesson).

  Stores VALUES ONLY -- the reference file's `_doc` help strings are ~2 kB of identical prose per
  curve, which would be tens of MB of duplication over a month and makes curves.jsonl unreadable.
  The docs live in opendbc/car/ford/angle_tuning.reference.json."""
  try:
    st = os.stat(TUNING_JSON)
    with open(TUNING_JSON) as f:
      raw = json.load(f)
    vals = {}
    for k, v in raw.items():
      if k.startswith("_"):
        continue
      vals[k] = v.get("value") if isinstance(v, dict) else v
    return {"mtime": int(st.st_mtime), "values": vals}
  except FileNotFoundError:
    return None
  except Exception as e:
    return {"error": str(e)}


class CarCtx:
  """Everything brand-specific, resolved from the LIVE CarParams. parser=None => wire-null mode."""
  def __init__(self, brand, fp, steer_type):
    self.brand, self.fp, self.steer_type = brand, fp, steer_type
    self.spec = WIRE_SPECS.get(brand)
    self.parser = None
    self.addr = None
    self.last_wire_mono = -1e9
    if self.spec:
      try:
        values_mod = importlib.import_module(f"opendbc.car.{brand}.values")
        dbc_name = values_mod.DBC[fp][self.spec["bus_key"]]
        self.parser = CANParser(dbc_name, [(self.spec["msg"], self.spec["rate"])], self.spec["bus"])
        self.addr = next(a for a in self.parser.addresses)
      except Exception as e:
        print(f"wire decode unavailable for {brand}/{fp}: {e} — logging green/yellow only", flush=True)
        self.parser = None

  def feed(self, sendcan_msgs, mono):
    """Feed drained sendcan readers to the parser. Never raises."""
    if self.parser is None or not sendcan_msgs:
      return
    try:
      entries = [(m.logMonoTime, [(f.address, bytes(f.dat), f.src) for f in m.sendcan]) for m in sendcan_msgs]
      updated = self.parser.update(entries)
      if self.addr in updated:
        self.last_wire_mono = mono
    except Exception:
      pass

  def wire_row(self, mono):
    if self.parser is None:
      return {"wire_angle": None, "wire_mode": None, "wire_c2": None, "wire_c3": None,
              "wire_off": None, "wire_ok": False}
    s = self.spec
    vl = self.parser.vl[s["msg"]]
    return {
      "wire_angle": round(float(vl[s["angle"]]), 5),
      "wire_mode": int(vl[s["mode"]]),
      "wire_c2": round(float(vl[s["c2"]]), 6) if s["c2"] else None,
      "wire_c3": round(float(vl[s["c3"]]), 7) if s["c3"] else None,
      "wire_off": round(float(vl[s["off"]]), 3) if s["off"] else None,
      "wire_ok": (mono - self.last_wire_mono) < WIRE_FRESH_S,
    }

  def ident(self):
    return {"brand": self.brand, "fp": self.fp, "steer_type": self.steer_type}


def load_car(params):
  """Read CarParams (live first, then persistent). Returns CarCtx or None. Never raises."""
  try:
    b = params.get("CarParams") or params.get("CarParamsPersistent")
    if not b:
      return None
    CP = messaging.log_from_bytes(b, car_capnp.CarParams)
    return CarCtx(str(CP.brand), str(CP.carFingerprint), str(CP.steerControlType))
  except Exception as e:
    print(f"load_car failed: {e}", flush=True)
    return None


def summarize(rows, curve_id, i_start, i_end, car_ctx, timed_out, post_truncated):
  """One aggregatable record per curve. This is the dataset; traces are for forensics."""
  curve = rows[i_start:i_end + 1]
  if not curve:
    return None

  # Sign convention (Alan Polk, video 1): right curve trends negative, left positive.
  peak_i = max(range(len(curve)), key=lambda i: abs(curve[i]["cmd_sa"]))
  peak = curve[peak_i]
  direction = "left" if peak["cmd_sa"] > 0 else "right"

  # Peak comparison in the SAME SIGN as the peak — the whole tuning criterion. The verdict deadband
  # is ABSOLUTE degrees on the peak difference (a ratio band collapses to sub-noise at small peaks).
  s = 1.0 if peak["cmd_sa"] > 0 else -1.0
  green_peak = max(r["cmd_sa"] * s for r in curve)
  yellow_peak = max(r["sa"] * s for r in curve)
  delta = yellow_peak - green_peak                       # >0 => yellow exceeded green => OVERSTEER
  measurable = green_peak > NOISE_DEG
  overshoot = (yellow_peak / green_peak) if measurable else None
  # Opposite-sign lobe too, so a merged S-curve keeps both peaks (trigger hysteresis can merge one).
  green_opp = max(r["cmd_sa"] * -s for r in curve)
  yellow_opp = max(r["sa"] * -s for r in curve)

  speeds = [r["v"] for r in curve]
  v_mean = sum(speeds) / len(speeds)
  v_peak = peak["v"]
  kappa_meas_peak = (peak["yaw"] / v_peak) if v_peak > 1.0 else None       # yawRate [rad/s] / v
  a_lat_peak = round(v_peak * abs(peak["yaw"]), 2) if kappa_meas_peak is not None else None  # v^2*|k|

  # --- validity flags (flagged, never filtered) ---
  touched = sum(1 for r in curve if r["sp"])
  # ICBM/longitudinal intervention: speed changing materially through the curve corrupts a
  # steady-state peak comparison. Cannot be turned off in everyday driving, so it is FLAGGED.
  v_swing = max(speeds) - min(speeds)
  # Exit-unwind hold (the 2026-07-19 failure): after the curve, the truck keeps turning IN THE
  # CURVE'S DIRECTION while the command has unwound. Sign-aware (an S-curve's opposite lobe must
  # not read as a hold) and persistence-gated (the real hold lasted >1 s; 0.25 s rejects noise).
  exit_hold = False
  hold_thresh = max(2 * NOISE_DEG, 0.5 * yellow_peak)
  run = 0
  for r in rows[i_end + 1:i_end + 1 + int(3.0 * RATE_HZ)]:
    if abs(r["cmd_sa"]) < NOISE_DEG and r["sa"] * s > hold_thresh:
      run += 1
      if run >= int(0.25 * RATE_HZ):
        exit_hold = True
        break
    else:
      run = 0

  wire_rows = [r for r in curve if r.get("wire_ok")]
  dur = curve[-1]["t"] - curve[0]["t"]
  lds = sorted(r["lat_delay"] for r in curve if r.get("lat_delay") is not None)

  flags = {
    "driver_touched": touched > 0,
    "driver_touched_frac": round(touched / len(curve), 3),
    "speed_unsteady": v_swing > 2.0,           # m/s swing through the curve (ICBM/lead/braking)
    "v_swing_ms": round(v_swing, 2),
    "near_stop": min(speeds) < 3.0,            # Alan Polk: model is "squirrely" at stops — don't tune
    "lane_change": any(r.get("lcs") not in (None, 0) for r in curve),
    "mode_zero_pulse": any(r["wire_mode"] == 0 for r in wire_rows),  # human-turn / stall-blip fired
    "wire_dead_frac": round(1.0 - len(wire_rows) / len(curve), 3),   # 1.0 = wire never decoded
    "not_engaged": any(not r["lat"] for r in curve),
    "exit_unwind_hold": exit_hold,
    # GROUND TRUTH: openpilot itself said it was at/over its lateral limit during this curve.
    # NOT a cleanliness disqualifier — a limit event is the most interesting kind of curve there is
    # (it is what the driver felt), so it stays "clean" and is surfaced separately for analysis.
    "limit_alert": any(r.get("alert_type") for r in curve),
    "limit_alert_types": sorted({r["alert_type"] for r in curve if r.get("alert_type")}) or None,
    "limit_alert_text": next((r["alert1"] for r in curve if r.get("alert1")), None),
    "angle_saturated": any(r.get("sat") for r in curve),
    "angle_saturated_frac": round(sum(1 for r in curve if r.get("sat")) / len(curve), 3),
    "timed_out": timed_out,                    # hit MAX_CURVE_S; peak may be mid-curve, a follow-on
                                               # record covers the continuation
    "post_truncated": post_truncated,          # session ended before full post-roll (exit_hold weak)
    "too_short": dur < MIN_CURVE_S,
  }
  # "Clean" = usable for Alan Polk's peak method. Everything else is still recorded.
  flags["clean_for_tuning"] = not (flags["driver_touched"] or flags["speed_unsteady"]
                                   or flags["near_stop"] or flags["not_engaged"]
                                   or flags["mode_zero_pulse"] or flags["lane_change"]
                                   or flags["timed_out"] or flags["too_short"])

  def wmax(key):
    vals = [abs(r[key]) for r in curve if r.get(key) is not None]
    return round(max(vals), 7) if vals else None

  return {
    "id": curve_id,
    **car_ctx.ident(),
    "wall": round(curve[0]["wall"], 2),
    "dur_s": round(dur, 2),
    "dir": direction,
    "v_mean_ms": round(v_mean, 2),
    "v_mean_mph": round(v_mean * 2.23694, 1),
    "v_min_ms": round(min(speeds), 2),
    "v_max_ms": round(max(speeds), 2),
    "v_peak_ms": round(v_peak, 2),
    "kappa_peak": round(max(abs(r["curv_cmd"]) for r in curve), 5),
    "kappa_meas_peak": round(abs(kappa_meas_peak), 5) if kappa_meas_peak is not None else None,
    "a_lat_peak": a_lat_peak,
    "green_peak_deg": round(green_peak, 2),
    "yellow_peak_deg": round(yellow_peak, 2),
    "delta_deg": round(delta, 2),
    "overshoot_ratio": round(overshoot, 4) if overshoot is not None else None,
    "verdict": (None if not measurable else
                "oversteer" if delta > PEAK_DELTA_DEG else
                "understeer" if delta < -PEAK_DELTA_DEG else "ok"),
    "green_peak_opp_deg": round(green_opp, 2),
    "yellow_peak_opp_deg": round(yellow_opp, 2),
    "wire_angle_peak": wmax("wire_angle"),
    "wire_c2_max": wmax("wire_c2"),      # ford: must stay 0 in angle mode
    "wire_c3_max": wmax("wire_c3"),      # ford: must stay 0
    "lat_delay": lds[len(lds) // 2] if lds else None,   # median over the curve
    "tuning": read_tuning(),
    "flags": flags,
    "trace": f"traces/{curve_id}.jsonl",
  }


def acquire_lock():
  """PID-reuse-proof single-instance lock. A stale file after a hard reboot must never be able to
  silently prevent all future logging — verify the pid is actually *this script* via /proc."""
  if os.path.exists(LOCK):
    try:
      pid = int(open(LOCK).read().strip())
      with open(f"/proc/{pid}/cmdline", "rb") as f:
        cmdline = f.read().decode(errors="replace")
      if "angle_curve_logger" in cmdline:
        print(f"already running as pid {pid}; exiting", flush=True)
        return False
    except (FileNotFoundError, ProcessLookupError, ValueError, PermissionError):
      pass  # stale or unreadable: take over
  with open(LOCK, "w") as f:
    f.write(str(os.getpid()))

  def _cleanup():
    try:
      if int(open(LOCK).read().strip()) == os.getpid():
        os.remove(LOCK)
    except Exception:
      pass
  atexit.register(_cleanup)
  return True


def main():
  os.makedirs(TRACE_DIR, exist_ok=True)
  if not acquire_lock():
    return

  params = Params()
  car_ctx = load_car(params)
  sm = messaging.SubMaster(["carState", "carControl", "modelV2", "liveDelay",
                            "selfdriveState", "controlsState"])
  sendcan = messaging.sub_sock("sendcan", conflate=False, timeout=100)

  def emit_marker(reason):
    ident = car_ctx.ident() if car_ctx else {"brand": None, "fp": None, "steer_type": None}
    m = {"marker": "session", "wall": round(time.time(), 2), "reason": reason, **ident}
    append_summary(m)
    print(f"session marker: {m}", flush=True)

  emit_marker("start")

  ring = deque(maxlen=RING_N)
  abs_n = 0                 # rows ever appended; ring offset = abs_n - len(ring)
  in_curve = False
  start_abs = start_t = None
  below_abs = below_t = None
  pend = []                 # completed curves waiting for post-roll: {start_abs, end_abs, end_t, timed_out}
  n_curves = 0
  last_append_t = -1e9
  last_car_check = time.monotonic()
  carstate_seen = time.monotonic()
  last_err_t = 0.0
  t0 = time.monotonic()

  print(f"curve logger up: car={car_ctx.ident() if car_ctx else None}, trigger |k|>{CURVE_ON_KAPPA}, "
        f"pre {PRE_ROLL_S}s / post {POST_ROLL_S}s, max curve {MAX_CURVE_S}s, ring {RING_S:.0f}s "
        f"({RING_N} rows @ {RATE_HZ:.0f}Hz)", flush=True)

  def flush_pending(rows_now, force=False):
    nonlocal n_curves
    now_t = rows_now[-1]["t"] if rows_now else None
    for p in pend[:]:
      if not force and (now_t is None or now_t - p["end_t"] < POST_ROLL_S):
        continue
      pend.remove(p)
      off = abs_n - len(rows_now)
      i_start, i_end = p["start_abs"] - off, p["end_abs"] - off
      if i_end < 0 or i_start >= len(rows_now):
        continue  # evicted/cleared — cannot happen within RING_S, but never slice garbage
      i_start, i_end = max(0, i_start), min(i_end, len(rows_now) - 1)
      n_curves += 1
      cid = f"{int(rows_now[i_end]['wall'])}_{n_curves:04d}"
      summ = summarize(rows_now, cid, i_start, i_end, car_ctx, p["timed_out"], force)
      if summ is None:
        continue
      try:
        free = free_bytes()
        if free < MIN_FREE_ANY:
          print(f"curve {cid}: DROPPED, only {free // 1048576} MB free on /data", flush=True)
          continue
        if free > MIN_FREE_TRACE and trace_worth_keeping(summ):
          window = rows_now[max(0, i_start - PRE_N):min(len(rows_now), i_end + POST_N + 1)]
          with open(os.path.join(TRACE_DIR, f"{cid}.jsonl"), "w") as tf:
            for r in window:
              tf.write(json.dumps(r) + "\n")
        else:
          # Summary is ALWAYS written (2.7 kB, it is the actual dataset); the ~292 kB trace is
          # forensics and is skipped for curves nothing can be learned from, or under disk pressure.
          summ["trace"] = None
        append_summary(summ)
        print(f"curve {cid}: {summ['dir']} {summ['v_mean_mph']}mph "
              f"green {summ['green_peak_deg']} yellow {summ['yellow_peak_deg']} d {summ['delta_deg']} "
              f"-> {summ['verdict']} clean={summ['flags']['clean_for_tuning']}", flush=True)
        if n_curves % CHECK_EVERY_N_CURVES == 0:
          rotate(TRACE_DIR, MAX_BYTES)
      except Exception as e:
        print(f"curve {cid}: write failed: {e}", flush=True)

  while True:
    try:
      sm.update(100)  # blocks: ~100 Hz onroad, 10 Hz idle when the car is off
      mono = time.monotonic()
      if car_ctx is not None:
        car_ctx.feed(messaging.drain_sock(sendcan), mono)
      else:
        messaging.drain_sock(sendcan)  # keep the queue drained even in wire-null mode

      if not sm.updated["carState"]:
        # Offroad / car off / mid-swap: re-derive the car occasionally so a device moved to the
        # other vehicle (or a late fingerprint) is picked up WITHOUT a restart.
        if mono - carstate_seen > 5.0 and mono - last_car_check > 60.0:
          last_car_check = mono
          new_ctx = load_car(params)
          if (new_ctx.fp if new_ctx else None) != (car_ctx.fp if car_ctx else None):
            rows = list(ring)
            flush_pending(rows, force=True)
            ring.clear()
            in_curve, start_abs, below_abs, below_t = False, None, None, None
            car_ctx = new_ctx
            emit_marker("car_change")
        continue
      carstate_seen = mono

      row_t = mono - t0
      if row_t - last_append_t < DT - 0.005:   # decimate 100 Hz carState to the 20 Hz design rate
        continue
      last_append_t = row_t

      # Session gap (ignition cycle / car swap): flush what we have, then hard-reset — a window
      # stitched across an ignition gap is junk, and the car may have changed.
      if ring and row_t - ring[-1]["t"] > GAP_S:
        rows = list(ring)
        flush_pending(rows, force=True)
        ring.clear()
        in_curve, start_abs, below_abs, below_t = False, None, None, None
        new_ctx = load_car(params)
        last_car_check = mono
        if (new_ctx.fp if new_ctx else None) != (car_ctx.fp if car_ctx else None):
          car_ctx = new_ctx
          emit_marker("car_change")

      cs, cc = sm["carState"], sm["carControl"]
      lane_l = lane_r = lcs = None
      try:
        md = sm["modelV2"]
        ll = md.laneLines
        if len(ll) >= 3:
          lane_l, lane_r = round(float(ll[1].y[0]), 3), round(float(ll[2].y[0]), 3)
        _lcs = md.meta.laneChangeState
        lcs = int(getattr(_lcs, "raw", _lcs))  # pycapnp DynamicEnum-or-int, version-dependent
      except Exception:
        pass

      # Lateral-limit ground truth: what openpilot told the DRIVER. Never inferable afterwards.
      alert_type = alert1 = alert_status = None
      sat = None
      try:
        if sm.alive["selfdriveState"]:
          ss = sm["selfdriveState"]
          at = str(ss.alertType) or None
          # Keep only lateral/limit-relevant alerts; ignore the constant background chatter.
          if at and any(k.lower() in at.lower() for k in LIMIT_ALERT_TYPES):
            alert_type = at
            alert1 = str(ss.alertText1) or None
            _as = ss.alertStatus
            alert_status = int(getattr(_as, "raw", _as))
      except Exception:
        pass
      try:
        if sm.alive["controlsState"]:
          lcst = sm["controlsState"].lateralControlState
          if lcst.which() == "angleState":
            sat = bool(lcst.angleState.saturated)
      except Exception:
        pass

      row = {
        "t": round(row_t, 3),
        "wall": round(time.time(), 2),
        "cmd_sa": round(float(cc.actuators.steeringAngleDeg), 2),   # GREEN
        "sa": round(cs.steeringAngleDeg, 2),                        # YELLOW
        "sr": round(cs.steeringRateDeg, 1),
        "curv_cmd": round(float(cc.actuators.curvature), 6),
        "v": round(cs.vEgo, 2),
        "a": round(cs.aEgo, 2),
        "yaw": round(cs.yawRate, 4),
        "torque": round(cs.steeringTorque, 1),
        "sp": bool(cs.steeringPressed),
        "lat": bool(cc.latActive),
        "en": bool(cc.enabled),
        "lane_l": lane_l,
        "lane_r": lane_r,
        "lcs": lcs,
        # GROUND TRUTH that openpilot itself hit a lateral limit. alert_type is the machine-readable
        # name (e.g. "steerSaturated"); alert1 is what the driver actually saw on screen. sat is the
        # angle controller's own saturation flag. See LIMIT_ALERT_TYPES.
        "alert_type": alert_type,
        "alert1": alert1,
        "alert_status": alert_status,
        "sat": sat,
        "lat_delay": round(float(sm["liveDelay"].lateralDelay), 3) if sm.alive["liveDelay"] else None,
        **(car_ctx.wire_row(mono) if car_ctx else
           {"wire_angle": None, "wire_mode": None, "wire_c2": None, "wire_c3": None,
            "wire_off": None, "wire_ok": False}),
      }
      ring.append(row)
      abs_n += 1

      k = abs(row["curv_cmd"])
      if not in_curve:
        if k > CURVE_ON_KAPPA and row["v"] > MIN_TRIGGER_V:
          in_curve = True
          start_abs, start_t = abs_n - 1, row_t
          below_abs = below_t = None
      else:
        if k < CURVE_OFF_KAPPA:
          if below_t is None:
            below_abs, below_t = abs_n - 1, row_t
        else:
          below_abs = below_t = None

        ended = below_t is not None and row_t - below_t >= CURVE_OFF_HOLD_S
        timed_out = row_t - start_t >= MAX_CURVE_S
        if ended or timed_out:
          end_abs = below_abs if ended else abs_n - 1
          end_t = below_t if ended else row_t
          if end_t - start_t >= MIN_CURVE_S:
            pend.append({"start_abs": start_abs, "end_abs": end_abs, "end_t": end_t,
                         "timed_out": timed_out})
          in_curve = False
          start_abs = below_abs = below_t = None
          # if still in a curve (timeout / S-curve tail), the very next sample can re-trigger

      if pend:
        flush_pending(list(ring))

    except Exception:
      now = time.monotonic()
      if now - last_err_t > 10.0:   # throttled — a persistent fault must not spin the log
        last_err_t = now
        traceback.print_exc()
      time.sleep(0.5)


if __name__ == "__main__":
  main()
