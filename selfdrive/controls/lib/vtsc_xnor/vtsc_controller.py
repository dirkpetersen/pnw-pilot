"""
VTSC Phase 2 — live controller for the longitudinal planner.

`VTSCController.cap(sm, v_cruise, v_ego)` returns a possibly-lowered cruise speed (m/s) so the planner
MPC slows for an upcoming curve. Rides the CES master selector (`CESMode`: 0=Off, 1=Light->GENTLE
tune, 2=Standard->DEFAULT tune; default Off) + openpilotLongitudinalControl; returns v_cruise
unchanged when disabled -> behavior-neutral. NEVER raises speed above v_cruise.

Apex state machine (driver feedback, drive #4):
  - the instant a binding curve is detected -> an immediate >=1 mph cut (CONFIDENCE_CUT) so the driver
    feels VTSC engage right away;
  - BRAKE while the apex is clearly ahead (tta > HOLD_TTA_S): slow to reach curve-safe speed BEFORE the
    apex (firmer if needed — pre-apex braking is flexible);
  - HOLD when close/uncertain (APEX_TTA_S < tta <= HOLD_TTA_S): maintain, NEVER reduce further;
  - RELEASE at the apex (tta <= APEX_TTA_S, or the path straightens): accelerate back to cruise.
We never reduce speed at or after the apex.

Logging: publishes the decision as a `vtscState` cereal message every cycle (recorded in qlog/rlog so
drives are analyzable) AND a `VTSCStatus` JSON to /dev/shm/params for the live on-screen overlay.

Runs inside plannerd (20 Hz / DT_MDL). Uses a MEASURED loop dt for the rate-limiter (don't assume a
fixed rate — that was the CES 5x bug). Pure curve/curvature math lives in vtsc_xnor.py.
"""
import time

from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.vtsc_xnor import vtsc_constants as C
from openpilot.selfdrive.controls.lib.vtsc_xnor.vtsc_xnor import model_curve_state, brake_cap_for_apex, apply_limits
from openpilot.selfdrive.controls.lib.ces_xnor import ces_xnor_constants as CES


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
      except Exception:
        self._enabled = False

  def _reset(self):
    self._state = "idle"
    self._applied = None
    self._below = 0
    self._clear = 0

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
    has_curve = d_apex >= 0.0 and v_curve < v_cruise - 0.1
    tta = (d_apex / max(v_ego, 1.0)) if has_curve else float('inf')

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
      if not has_curve or tta <= C.APEX_TTA_S:
        self._state = "release"
      elif tta <= C.HOLD_TTA_S:
        self._state = "hold"
      else:
        cap = brake_cap_for_apex(v_curve, d_apex, v_ego, self.tune['A_DECEL'])
        # never above cruise-CONFIDENCE_CUT (keep the engage cut), floored at V_MIN
        target = max(min(cap, v_cruise - C.CONFIDENCE_CUT), C.V_MIN)

    if self._state == "hold":
      target = self._applied                       # freeze: never reduce further, never accelerate yet
      if tta <= C.APEX_TTA_S or not has_curve:
        self._state = "release"

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
