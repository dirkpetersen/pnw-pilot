"""
VTSC Phase 2 — live controller for the longitudinal planner.

`VTSCController.cap(sm, v_cruise, v_ego)` returns a possibly-lowered cruise speed (m/s) so the planner
MPC decelerates for an upcoming curve. Rides the CES master toggle (`ConditionalExperimentalSwitching`,
default OFF) + openpilotLongitudinalControl; returns v_cruise unchanged when disabled -> behavior-neutral.
Publishes `VTSCStatus` to /dev/shm/params for the lower-right overlay. NEVER raises speed.

Runs inside plannerd (20 Hz / DT_MDL). Uses a MEASURED loop dt for the rate-limiter (don't assume a
fixed rate — that was the CES 5x bug). The pure decel-envelope + curvature math lives in vtsc_xnor.py;
this only adds: the toggle, a debounce against phantom curves, the safety rate-limit, and logging.
"""
import time

from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.vtsc_xnor import vtsc_constants as C
from openpilot.selfdrive.controls.lib.vtsc_xnor.vtsc_xnor import vtsc_from_model, apply_limits


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
    self._enabled = False
    self._applied = None      # current applied cap (m/s); None = none
    self._below = 0           # consecutive cycles raw cap < cruise (debounce)
    self._last_t = None       # monotonic stamp of last cap() call (real dt)
    self._last_read = -1e9    # monotonic stamp of last param read
    self._tele_last = 0.0     # monotonic stamp of last VTSCStatus publish
    self._engaged = False     # for engage/clear logging

  def enabled(self) -> bool:
    return self._enabled

  def _read_enabled(self, now: float) -> None:
    if now - self._last_read >= 1.0:                       # ~1 Hz
      self._last_read = now
      try:
        # VTSC rides the CES master toggle (ConditionalExperimentalSwitching): CES on -> VTSC on.
        self._enabled = self._long_ok and self.params.get_bool("ConditionalExperimentalSwitching")
      except Exception:
        self._enabled = False

  def cap(self, sm, v_cruise: float, v_ego: float) -> float:
    """Return the VTSC-capped cruise speed (m/s). v_cruise when disabled / no curve. Safe: <= v_cruise."""
    now = time.monotonic()
    self._read_enabled(now)
    dt = min(max((now - self._last_t) if self._last_t is not None else DT_MDL, 1e-3), 0.5)
    self._last_t = now

    if not self._enabled:
      self._applied = None
      self._below = 0
      if self._engaged:
        cloudlog.info("VTSC disabled -> no cap")
        self._engaged = False
      self._publish(False, v_cruise, v_cruise, now)
      return v_cruise

    try:
      model = sm['modelV2']
    except Exception:
      return v_cruise

    raw = vtsc_from_model(model, v_cruise)                 # decel-limited cap, <= v_cruise
    # debounce: require the curve sustained CURVE_MIN_POINTS cycles before committing (no phantom braking)
    self._below = self._below + 1 if raw < v_cruise - 0.1 else 0
    target = raw if self._below >= C.CURVE_MIN_POINTS else v_cruise
    # safety rate-limit (bounded decel down, ease back up)
    self._applied = apply_limits(self._applied, target, v_cruise, dt)
    capped = min(v_cruise, self._applied)

    engaged = capped < v_cruise - 0.5
    if engaged != self._engaged:
      cloudlog.info("VTSC %s: cap=%.1f m/s (cruise=%.1f, vEgo=%.1f)",
                    "ENGAGE" if engaged else "clear", capped, v_cruise, v_ego)
      self._engaged = engaged
    self._publish(engaged, capped, v_cruise, now)
    return capped

  def _publish(self, engaged: bool, cap: float, v_cruise: float, now: float) -> None:
    """Publish a tiny VTSCStatus snapshot to /dev/shm/params (~5 Hz) for the on-screen overlay."""
    if self.mem_params is None or now - self._tele_last < 0.2:
      return
    self._tele_last = now
    try:
      self.mem_params.put_nonblocking("VTSCStatus", {
        "enabled": bool(self._enabled), "engaged": bool(engaged),
        "cap": round(float(cap), 1), "vCruise": round(float(v_cruise), 1),
      })
    except Exception:
      pass
