"""
VTSC Phase 2 — live controller for the longitudinal planner.

`VTSCController.cap(sm, v_cruise, v_ego)` returns a possibly-lowered cruise speed (m/s) so the planner
MPC slows for an upcoming curve. Rides the CES master selector (`CESMode`: 0=Off, 1=Light->GENTLE
tune, 2=Standard->DEFAULT tune; default Off) + openpilotLongitudinalControl; returns v_cruise
unchanged when disabled -> behavior-neutral. NEVER raises speed above v_cruise.

Apex state machine (driver feedback, drive #4):
  - the instant a curve is detected -> an immediate >=1 mph cut (CONFIDENCE_CUT) so the driver feels VTSC
    engage right away. ces-i90-2pnw: this now fires for EVERY real bend (apex curvature >= CUE_MIN_CURVATURE,
    ~5 deg / R~2300 m), not only curves that need real slowing -> a guaranteed small cue at the start of
    each curve; gentle bends get just the >=1 mph dip (held through, then released), sharper curves brake more;
  - BRAKE while the apex is clearly ahead (tta > HOLD_TTA_S): slow to reach curve-safe speed BEFORE the
    apex (firmer if needed — pre-apex braking is flexible);
  - HOLD when close/uncertain (APEX_TTA_S < tta <= HOLD_TTA_S): maintain, NEVER reduce further;
  - RELEASE at the apex (tta <= APEX_TTA_S, or the path straightens): accelerate back to cruise.
We never reduce speed at or after the apex.

Logging: publishes the decision as a `vtscState` cereal message every cycle (recorded in qlog/rlog so
drives are analyzable) AND a `VTSCStatus` JSON to /dev/shm/params for the live on-screen overlay.

Runs inside plannerd (20 Hz / DT_MDL). Uses a MEASURED loop dt for the rate-limiter (don't assume a
fixed rate — that was the CES 5x bug). Pure curve/curvature math lives in vtsc_pnw.py.
"""
import json
import time

from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.vtsc_pnw import vtsc_constants as C
from openpilot.selfdrive.controls.lib.vtsc_pnw.vtsc_pnw import model_curve_state, brake_cap_for_apex, apply_limits
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw_constants as CES
from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import upcoming_curve   # ces-i90-2pnw (MTSC)


class VTSCController:
  def __init__(self, CP, params=None):
    import platform
    from openpilot.common.params import Params
    self.CP = CP
    self.params = params or Params()
    try:   # in-memory store for the UI overlay (same channel CES uses)
      self.mem_params = Params("/dev/shm/params") if platform.system() != "Darwin" else self.params
    except Exception:
      self.mem_params = None
    self._long_ok = bool(getattr(CP, 'openpilotLongitudinalControl', False))
    # light-ces-gentle: the tune is now USER-SELECTED via CESMode (1=Light -> GENTLE_PROFILE, soft
    # decel + slow recovery so a series of curves doesn't sawtooth; 2=Standard -> DEFAULT_PROFILE),
    # NOT gated on carFingerprint. _read_enabled() re-selects the tune when the mode changes.
    self._mode = CES.CES_MODE_OFF
    self.tune = dict(C.DEFAULT_PROFILE)
    self._enabled = False
    # ces-i90-2pnw (MTSC): optional pfeiferj map curve fold, gated by VtscMapCurves (default OFF)
    self._map_curves = False
    self._map_targets: list = []
    self._cur_lat = self._cur_lon = None
    self._state = "idle"      # idle | brake | hold | release
    self._applied = None      # current applied cap (m/s); None = none
    self._below = 0           # consecutive cycles a far curve is present (debounce into brake)
    self._clear = 0           # consecutive cycles no curve (debounce release -> idle)
    self._last_t = None       # monotonic stamp of last cap() call (real dt)
    self._last_read = -1e9    # monotonic stamp of last param read
    self._tele_last = 0.0     # monotonic stamp of last overlay publish
    self._engaged = False     # for engage/clear logging
    # last decision, for the logged vtscState message (read by the planner)
    self.msg = dict(enabled=False, active=False, state="idle", vCruise=0.0, vTarget=0.0,
                    vEgo=0.0, apexDist=-1.0, apexCurvature=0.0, vCurveSafe=0.0, timeToApex=-1.0)

  def enabled(self) -> bool:
    return self._enabled

  def _read_enabled(self, now: float) -> None:
    if now - self._last_read >= 1.0:                       # ~1 Hz
      self._last_read = now
      try:
        # VTSC rides the CES master selector (CESMode): non-Off -> VTSC on. The mode also picks the
        # tune: Light -> GENTLE_PROFILE (anti-sawtooth), Standard -> DEFAULT_PROFILE. On ANY car.
        self._mode = CES.read_ces_mode(self.params)
        self._enabled = self._long_ok and CES.ces_enabled(self._mode)
        self.tune = dict(C.GENTLE_PROFILE) if CES.ces_is_gentle(self._mode) else dict(C.DEFAULT_PROFILE)
        # ces-i90-2pnw (MTSC): fold map curves only when VTSC is enabled AND opted-in via the param
        self._map_curves = self._enabled and bool(self.params.get_bool("VtscMapCurves"))
        if self._map_curves:
          self._read_map()
        else:
          self._map_targets = []
      except Exception:
        self._enabled = False
        self._map_curves = False

  def _reset(self):
    self._state = "idle"
    self._applied = None
    self._below = 0
    self._clear = 0

  def _read_map(self):
    """ces-i90-2pnw (MTSC): refresh the pfeiferj map curve inputs (MapTargetVelocities + GPS) from the
    /dev/shm mem params — the SAME source CES reads. Any failure -> no map curve (vision still works)."""
    if self.mem_params is None:
      self._map_targets = []
      return
    try:
      self._map_targets = self.mem_params.get("MapTargetVelocities", return_default=True) or []
    except Exception:
      self._map_targets = []
    try:
      pos = self.mem_params.get("LastGPSPosition", return_default=True)
      if isinstance(pos, (bytes, str)):
        pos = json.loads(pos)
      self._cur_lat = float(pos["latitude"])
      self._cur_lon = float(pos["longitude"])
    except Exception:
      self._cur_lat = self._cur_lon = None

  def _fold_map_curve(self, k_apex, d_apex, v_curve, v_cruise, v_ego):
    """ces-i90-2pnw (MTSC): fold the upcoming MAP curve (pfeiferj MapTargetVelocities) into the curve
    picture, using whichever of vision / map is MORE BINDING (needs the lower speed NOW via the decel
    envelope). Lets VTSC start braking earlier and catch sharp curves the vision under-reads (Snoqualmie
    summit: map ~58 mph vs vision ~82). The chosen curve feeds the SAME decel-limited + floored state
    machine, so even a wrong/low map speed brakes smoothly (never slams) and stays >= V_MIN."""
    try:
      mv, md = upcoming_curve(self._map_targets, self._cur_lat, self._cur_lon, v_ego, C.MAP_LOOKAHEAD_S)
    except Exception:
      return k_apex, d_apex, v_curve
    # soften MTSC (driver: mapd's curve targets ran ~10 mph too slow on I-90): carry more speed through
    # map curves by scaling the target up, capped at cruise. Still feeds the decel-limited + V_MIN-floored
    # state machine below, so it can never slam. Scale applied BEFORE the fold test so trivial curves drop.
    if mv > 0.0:
      mv = min(mv * C.MAP_SPEED_SCALE, v_cruise)
    # only a real map curve meaningfully below cruise counts (ignore GPS noise / trivial targets)
    if not (0.0 < mv < v_cruise - C.MAP_MIN_SLOWDOWN) or md <= 0.0:
      return k_apex, d_apex, v_curve
    rsn_vis = brake_cap_for_apex(v_curve, d_apex, v_ego, self.tune['A_DECEL']) if d_apex >= 0.0 else float('inf')
    rsn_map = brake_cap_for_apex(mv, md, v_ego, self.tune['A_DECEL'])
    if rsn_map < rsn_vis:                       # map curve is the more binding -> use it
      k_map = (self.tune['A_LAT_TARGET'] / (mv * mv)) if mv > 0.0 else 0.0   # equiv curvature, logging only
      return k_map, md, mv
    return k_apex, d_apex, v_curve

  def cap(self, sm, v_cruise: float, v_ego: float) -> float:
    """Return the VTSC-capped cruise speed (m/s). v_cruise when disabled / no curve. Safe: <= v_cruise."""
    now = time.monotonic()
    self._read_enabled(now)
    dt = min(max((now - self._last_t) if self._last_t is not None else DT_MDL, 1e-3), 0.5)
    self._last_t = now

    if not self._enabled:
      self._reset()
      if self._engaged:
        cloudlog.info("VTSC disabled -> no cap")
        self._engaged = False
      return self._finish(v_cruise, v_cruise, v_ego, 0.0, -1.0, float('inf'), now)

    try:
      model = sm['modelV2']
    except Exception:
      return self._finish(v_cruise, v_cruise, v_ego, 0.0, -1.0, float('inf'), now)

    k_apex, d_apex, v_curve = model_curve_state(model, v_cruise, self.tune['A_LAT_TARGET'])
    if self._map_curves:                        # ces-i90-2pnw (MTSC): fold in the upcoming map curve
      k_apex, d_apex, v_curve = self._fold_map_curve(k_apex, d_apex, v_curve, v_cruise, v_ego)
    # ces-i90-2pnw: a curve "counts" if it BINDS (curve-safe speed below cruise -> real braking) OR is a
    # mild bend past CUE_MIN_CURVATURE (~5 deg / R~2300 m). The mild-bend case never needs real slowing, so
    # the state machine below only applies the CONFIDENCE_CUT (>=1 mph) engage dip then releases -> a
    # guaranteed small "I see the curve" cue on EVERY real bend, automatic, no toggle.
    has_curve = d_apex >= 0.0 and (v_curve < v_cruise - 0.1 or k_apex >= C.CUE_MIN_CURVATURE)
    tta = (d_apex / max(v_ego, 1.0)) if has_curve else float('inf')
    # drive #5: have we actually slowed to ~curve-safe speed? gates HOLD/RELEASE below so VTSC keeps
    # braking while still materially too fast, and never accelerates out of a curve before reaching safe.
    at_safe = (not has_curve) or v_curve <= 0.0 or v_ego <= v_curve * (1.0 + C.RELEASE_SPEED_MARGIN)

    if self._applied is None:
      self._applied = v_cruise

    # ---- state machine: brake before apex, hold when unsure, release+accelerate at apex ----
    if self._state == "idle":
      target = v_cruise
      self._below = self._below + 1 if (has_curve and tta > C.HOLD_TTA_S) else 0
      if self._below >= C.CURVE_MIN_POINTS:
        self._state = "brake"
        self._applied = min(self._applied, v_cruise - C.CONFIDENCE_CUT)   # instant >=1mph cut on detect

    if self._state == "brake":
      if not has_curve:
        self._state = "release"
      elif tta <= C.APEX_TTA_S:
        # at the apex: never brake here. accelerate out only if we've slowed enough; else HOLD (no accel).
        self._state = "release" if at_safe else "hold"
      elif tta <= C.HOLD_TTA_S and at_safe:
        self._state = "hold"                       # close AND already at safe speed -> maintain
      else:
        # apex still ahead, OR still too fast inside the hold window -> keep reducing toward curve-safe
        cap = brake_cap_for_apex(v_curve, d_apex, v_ego, self.tune['A_DECEL'])
        # never above cruise-CONFIDENCE_CUT (keep the engage cut), floored at V_MIN
        target = max(min(cap, v_cruise - C.CONFIDENCE_CUT), C.V_MIN)

    if self._state == "hold":
      target = self._applied                       # freeze: never reduce further, never accelerate yet
      if not has_curve or (tta <= C.APEX_TTA_S and at_safe):
        self._state = "release"                    # only accelerate out once we've actually slowed

    if self._state == "release":
      target = v_cruise                            # accelerate back to cruise set speed
      self._clear = self._clear + 1 if not has_curve else 0
      # a genuinely NEW curve far ahead re-arms braking
      self._below = self._below + 1 if (has_curve and tta > C.HOLD_TTA_S) else 0
      if self._below >= C.CURVE_MIN_POINTS:
        self._state = "brake"
        self._clear = 0
        self._applied = min(self._applied, v_cruise - C.CONFIDENCE_CUT)
      elif self._clear >= C.CLEAR_CYCLES:
        self._state = "idle"
        self._below = 0

    # safety rate-limit (bounded decel down to A_DECEL_MAX, ease up at A_RELAX). HOLD target==applied -> no move.
    self._applied = apply_limits(self._applied, target, v_cruise, dt, self.tune['A_DECEL_MAX'], self.tune['A_RELAX'])
    capped = min(v_cruise, self._applied)

    engaged = capped < v_cruise - 0.5
    if engaged != self._engaged:
      cloudlog.info("VTSC %s [%s]: cap=%.1f cruise=%.1f vEgo=%.1f apex=%.0fm tta=%.1fs",
                    "ENGAGE" if engaged else "clear", self._state, capped, v_cruise, v_ego, d_apex, tta)
      self._engaged = engaged
    return self._finish(capped, v_cruise, v_ego, k_apex, d_apex, v_curve, now)

  def _finish(self, capped, v_cruise, v_ego, k_apex, d_apex, v_curve, now):
    active = capped < v_cruise - 0.5
    vcs = 0.0 if v_curve == float('inf') else float(v_curve)
    tta = (d_apex / v_ego) if (d_apex >= 0.0 and v_ego > 0.1) else -1.0
    self.msg = dict(enabled=bool(self._enabled), active=bool(active), state=self._state,
                    vCruise=float(v_cruise), vTarget=float(capped), vEgo=float(v_ego),
                    apexDist=float(d_apex), apexCurvature=float(k_apex), vCurveSafe=vcs,
                    timeToApex=float(tta))
    self._publish_overlay(now)
    return capped

  def _publish_overlay(self, now: float) -> None:
    """Publish a tiny VTSCStatus snapshot to /dev/shm/params (~5 Hz) for the on-screen overlay."""
    if self.mem_params is None or now - self._tele_last < 0.2:
      return
    self._tele_last = now
    try:
      self.mem_params.put_nonblocking("VTSCStatus", {
        "enabled": self.msg["enabled"], "engaged": self.msg["active"], "state": self.msg["state"],
        "cap": round(self.msg["vTarget"], 1), "vCruise": round(self.msg["vCruise"], 1),
      })
    except Exception:
      pass
