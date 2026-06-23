"""
ces2xnor: on-screen CES feedback overlay — lower-right, one datum per line.

Display-only. Shown only when the CES master toggle is on. Gives at-a-glance feedback on what
Conditional Experimental Switching + mapd are doing, so you can validate it on the road.

Data path: selfdrived's CESController publishes a `CESStatus` snapshot to the in-memory param store
(/dev/shm/params) at ~5 Hz (single source of truth for the live decision + mapd diagnostics). The
OSM speed limit comes from `liveMapDataSP` (already on the UI submaster). This widget never computes
the decision itself.

Lines (lower-right, short, one per line):
  CES AUTO            button mode (AUTO / CHILL* / EXP*  — * = forced)
  > EXPERIMENTAL      effective mode (orange) / > CHILL (grey)
  why lowSpeed        binding reason (only while experimental)
  curve 57% vis       curve closeness % + source (map/vision), color ramps green->orange
  map 24pts gps       mapd liveness: cached MapTargetVelocities points + GPS fix
  next 34 140m        next binding map curve (target speed + distance) or "road clear"
  limit 30            current OSM speed limit
"""
import time
import pyray as rl

from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.ces_xnor.ces_xnor_constants import ces_enabled, read_ces_mode
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

_REFRESH_S = 0.2     # poll the mem param at ~5 Hz (matches the publisher)
_FS = 64             # 2x size (driver feedback: the CES-mode overlay was too small)
_LINE_H = 80         # 2x line height to match
_PAD = 24
_MARGIN = 40         # gap from the screen's right / bottom edges
_REAL_CURVE_MS = 40.0  # a map target speed below this (~90 mph) counts as a real curve to preview


class _C:
  WHITE = rl.Color(255, 255, 255, 235)
  GREY = rl.Color(175, 180, 177, 235)
  ORANGE = rl.Color(255, 149, 0, 240)
  GREEN = rl.Color(90, 205, 115, 240)
  RED = rl.Color(235, 70, 70, 240)
  BG = rl.Color(0, 0, 0, 140)


class CesStatusRenderer(Widget):
  def __init__(self):
    super().__init__()
    try:
      self._mem = Params("/dev/shm/params")
    except Exception:
      self._mem = None
    self._last_poll = 0.0
    self._enabled = False
    self._st: dict = {}
    self._vtsc: dict = {}
    self.font = gui_app.font(FontWeight.MEDIUM)
    self.font_bold = gui_app.font(FontWeight.BOLD)

  @property
  def _conv(self) -> float:
    return CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH

  def _update_state(self):
    now = time.monotonic()
    if now - self._last_poll < _REFRESH_S:
      return
    self._last_poll = now
    # light-ces-gentle: the master is the INT CESMode (0=Off,1=Light,2=Standard); the overlay shows for
    # BOTH Light and Standard (any non-Off). read_ces_mode keeps back-compat with the old bool param.
    self._enabled = ces_enabled(read_ces_mode(ui_state.params))
    if not self._enabled or self._mem is None:
      self._st = {}
      self._vtsc = {}
      return
    try:
      st = self._mem.get("CESStatus", return_default=True)
      self._st = st if isinstance(st, dict) else {}
    except Exception:
      self._st = {}
    try:
      vt = self._mem.get("VTSCStatus", return_default=True)   # vtsc: rides the CES toggle
      self._vtsc = vt if isinstance(vt, dict) else {}
    except Exception:
      self._vtsc = {}

  # ---- build the lines -----------------------------------------------------
  def _lines(self) -> list[tuple]:
    st = self._st
    conv = self._conv
    units = "kph" if ui_state.is_metric else "mph"
    out: list[tuple] = []

    button = int(st.get("button", 0))
    btn = {0: "CES AUTO", 1: "CES CHILL*", 2: "CES EXP*"}.get(button, "CES AUTO")
    out.append((btn, _C.WHITE, self.font_bold))

    is_exp = st.get("mode") == "experimental"
    out.append((">> EXPERIMENTAL" if is_exp else ">> CHILL", _C.ORANGE if is_exp else _C.GREY, self.font_bold))

    # VTSC (curve speed control) — rides the CES toggle; show when slowing for a curve
    vt = self._vtsc
    if vt.get("enabled"):
      if vt.get("engaged"):
        out.append((f"VTSC slowing {round(vt.get('cap', 0.0) * conv)}", _C.ORANGE, self.font_bold))
      else:
        out.append(("VTSC ready", _C.GREY, self.font))

    reason = st.get("reason", "")
    if is_exp and reason and reason not in ("chill", ""):
      out.append((f"why {reason}", _C.WHITE, self.font))

    # accelerate-zone / highway-gate: held in Chill (lowSpeed suppressed) — show why
    if st.get("accelZone"):
      out.append(("accel-zone (open)", _C.GREEN, self.font))
    if st.get("hwyGate"):
      out.append(("hwy-gate (no lowSpd)", _C.GREEN, self.font))

    pct = max(0, min(100, int(st.get("curvePct", 0))))
    src = st.get("curveSrc", "") or "--"
    pct_col = _C.GREEN if pct < 60 else (_C.ORANGE if pct < 100 else _C.RED)
    out.append((f"curve {pct}% {src}", pct_col, self.font))

    # mapd liveness
    pts = int(st.get("mapPts", 0))
    gps = bool(st.get("gps", False))
    if pts == 0:
      out.append(("map no-data", _C.RED, self.font))
    elif not gps:
      out.append((f"map {pts}pts no-gps", _C.ORANGE, self.font))
    else:
      out.append((f"map {pts}pts gps", _C.GREEN, self.font))

    # next binding map curve (only when a real slowdown is ahead)
    mapv = float(st.get("mapV", 0.0))
    mapd = float(st.get("mapDist", 0.0))
    if 0.0 < mapv < _REAL_CURVE_MS and mapd > 0.0:
      out.append((f"next {round(mapv * conv)} {round(mapd)}m", _C.ORANGE, self.font))
    elif pts > 0 and gps:
      out.append(("road clear", _C.GREY, self.font))

    # current OSM speed limit (from liveMapDataSP)
    sl = self._speed_limit_text(conv, units)
    if sl:
      out.append((sl, _C.WHITE, self.font))

    return out

  def _speed_limit_text(self, conv, units):
    try:
      lmd = ui_state.sm["liveMapDataSP"]
      if lmd.speedLimitValid and lmd.speedLimit > 0.3:
        return f"limit {round(lmd.speedLimit * conv)} {units}"
    except Exception:
      pass
    return None

  # ---- render --------------------------------------------------------------
  def _render(self, rect: rl.Rectangle):
    if not self._enabled:
      return
    if not self._st or not self._st.get("enabled"):
      return
    # show the debug overlay ONLY in CES-auto button mode — hide it when the driver forced
    # Chill (1) or Experimental (2) via the top-right button (driver feedback, drive #4)
    if int(self._st.get("button", 0)) != 0:
      return
    lines = self._lines()
    if not lines:
      return

    box_w = max(measure_text_cached(f, t, _FS).x for t, _, f in lines) + _PAD * 2
    box_h = _LINE_H * len(lines) + _PAD * 2
    bx = rect.x + rect.width - box_w - _MARGIN
    by = rect.y + rect.height - box_h - _MARGIN

    rl.draw_rectangle_rounded(rl.Rectangle(bx, by, box_w, box_h), 0.12, 8, _C.BG)
    right = bx + box_w - _PAD
    y = by + _PAD
    for text, color, font in lines:
      w = measure_text_cached(font, text, _FS).x
      rl.draw_text_ex(font, text, rl.Vector2(right - w, y), _FS, 0, color)   # right-aligned
      y += _LINE_H
