"""
ces2xnor: on-screen CES feedback overlay (tiny, bottom of screen).

Display-only. Shows what Conditional Experimental Switching is doing right now so you get visual
feedback while validating it on the road. Rendered ONLY when the CES master toggle is on.

Data path: selfdrived's CESController publishes a `CESStatus` snapshot to the in-memory param store
(/dev/shm/params) at ~5 Hz — the single source of truth for the live decision. This widget polls it
at the same rate and never computes the decision itself, so the overlay can't disagree with control.

It shows:
  - the button mode (AUTO / CHILL / EXP) and the live effective mode (CHILL vs EXPERIMENTAL),
  - the binding reason when Experimental (curve / stop / lowSpeed / slowLead),
  - a curve "closeness" percentage + bar — how close we are to switching for a curve
    (80% = very close, 99% = imminent, 100% = switching), and which half saw it (map / vision),
  - a preview of the next map curve: its safe target speed and distance ahead.
"""
import time
import pyray as rl

from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

_REFRESH_S = 0.2   # poll the mem param at ~5 Hz (matches the publisher)
_FS = 36           # base font size (tiny relative to the 2160-wide screen)


class _Colors:
  WHITE = rl.Color(255, 255, 255, 235)
  GREY = rl.Color(175, 180, 177, 235)
  ORANGE = rl.Color(255, 149, 0, 240)
  GREEN = rl.Color(80, 200, 110, 240)
  BAR_BG = rl.Color(70, 70, 70, 200)
  BG = rl.Color(0, 0, 0, 150)


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
    self.font = gui_app.font(FontWeight.MEDIUM)
    self.font_bold = gui_app.font(FontWeight.BOLD)

  def _update_state(self):
    now = time.monotonic()
    if now - self._last_poll < _REFRESH_S:
      return
    self._last_poll = now
    # master toggle gates everything; cheap enough at 5 Hz
    self._enabled = ui_state.params.get_bool("ConditionalExperimentalSwitching")
    if not self._enabled or self._mem is None:
      self._st = {}
      return
    try:
      st = self._mem.get("CESStatus", return_default=True)
      self._st = st if isinstance(st, dict) else {}
    except Exception:
      self._st = {}

  def _render(self, rect: rl.Rectangle):
    if not self._enabled:
      return
    st = self._st
    if not st or not st.get("enabled"):
      return

    mode = st.get("mode", "chill")
    reason = st.get("reason", "")
    pct = max(0, min(100, int(st.get("curvePct", 0))))
    src = st.get("curveSrc", "")
    button = int(st.get("button", 0))

    conv = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
    units = "km/h" if ui_state.is_metric else "mph"

    btn_txt = {0: "AUTO", 1: "CHILL", 2: "EXP"}.get(button, "AUTO")
    is_exp = mode == "experimental"
    mode_txt = "EXPERIMENTAL" if is_exp else "CHILL"
    mode_color = _Colors.ORANGE if is_exp else _Colors.GREY

    line1 = f"CES {btn_txt} · {mode_txt}"
    if is_exp and reason and reason not in ("chill", ""):
      line1 += f" ({reason})"

    line2 = f"curve {pct}%"
    if src:
      line2 += f" [{src}]"
    mapv = float(st.get("mapV", 0.0))
    mapd = float(st.get("mapDist", 0.0))
    if mapv > 0.0 and mapd > 0.0:
      line2 += f"  ·  next {round(mapv * conv)} {units} in {round(mapd)} m"

    self._draw(rect, line1, line2, mode_color, pct)

  def _draw(self, rect, line1, line2, mode_color, pct):
    pad = 16
    bar_h = 10
    w1 = measure_text_cached(self.font_bold, line1, _FS)
    w2 = measure_text_cached(self.font, line2, _FS)
    box_w = max(w1.x, w2.x) + pad * 2
    box_h = _FS * 2 + bar_h + pad * 2 + 14
    bx = rect.x + (rect.width - box_w) / 2
    by = rect.y + rect.height - box_h - 36   # hug the bottom edge

    rl.draw_rectangle_rounded(rl.Rectangle(bx, by, box_w, box_h), 0.22, 8, _Colors.BG)
    rl.draw_text_ex(self.font_bold, line1, rl.Vector2(bx + pad, by + pad), _FS, 0, mode_color)
    rl.draw_text_ex(self.font, line2, rl.Vector2(bx + pad, by + pad + _FS + 6), _FS, 0, _Colors.WHITE)

    # closeness bar: green -> orange as we approach the switch; full+orange == tripping
    bar_y = by + box_h - pad - bar_h
    bar_w = box_w - pad * 2
    bar_color = _Colors.GREEN if pct < 60 else _Colors.ORANGE
    rl.draw_rectangle_rounded(rl.Rectangle(bx + pad, bar_y, bar_w, bar_h), 1.0, 4, _Colors.BAR_BG)
    if pct > 0:
      rl.draw_rectangle_rounded(rl.Rectangle(bx + pad, bar_y, bar_w * pct / 100.0, bar_h), 1.0, 4, bar_color)
