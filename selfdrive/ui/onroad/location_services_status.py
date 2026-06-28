"""
location2pnw: "HAPPENING AHEAD" on-screen overlay — LOWER-LEFT, display-only.

Mirrors ces_status.py (CES/VTSC overlays render lower-RIGHT; this is lower-LEFT, verified free). Reads
the `LocationServices` JSON the pnw_location_services daemon publishes to /dev/shm/params at ~5 Hz and
renders three advisory lines (police / rest / EV fast). Never computes anything itself; never touches
control/safety. Shown whenever LocationServicesEnabled is on (default ON).

NOTE: plain text labels (not emoji) — the openpilot Inter font has no emoji glyphs, so 👮/🛏/⚡ would
render as tofu. Swap to an icon atlas later if pictograms are wanted.
"""
import time
import pyray as rl

from openpilot.common.params import Params
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

_REFRESH_S = 0.2     # poll the mem param at ~5 Hz (matches the daemon publish cadence)
_FS = 56
_LINE_H = 70
_PAD = 22
_MARGIN = 40
_FT_PER_MILE = 5280.0


class _C:
  WHITE = rl.Color(255, 255, 255, 235)
  GREY = rl.Color(175, 180, 177, 235)
  ORANGE = rl.Color(255, 149, 0, 240)
  GREEN = rl.Color(90, 205, 115, 240)
  DIM = rl.Color(140, 145, 142, 220)
  BG = rl.Color(0, 0, 0, 140)


class LocationServicesStatusRenderer(Widget):
  def __init__(self):
    super().__init__()
    try:
      self._mem = Params("/dev/shm/params")
    except Exception:
      self._mem = None
    self._last_poll = 0.0
    self._st: dict = {}
    self.font = gui_app.font(FontWeight.MEDIUM)
    self.font_bold = gui_app.font(FontWeight.BOLD)

  def _update_state(self):
    now = time.monotonic()
    if now - self._last_poll < _REFRESH_S:
      return
    self._last_poll = now
    if self._mem is None or not ui_state.params.get_bool("LocationServicesEnabled"):
      self._st = {}
      return
    try:
      st = self._mem.get("LocationServices", return_default=True)
      self._st = st if isinstance(st, dict) else {}
    except Exception:
      self._st = {}

  # ---- formatting ----------------------------------------------------------
  def _dist_text(self, dist_mi):
    if dist_mi is None:
      return ""
    if dist_mi < 0.19:                         # under ~1000 ft -> show feet (decision §4)
      return f"{int(round(dist_mi * _FT_PER_MILE / 50.0) * 50)} ft"
    return f"{dist_mi:.1f} mi"

  def _police_line(self):
    p = self._st.get("police", {})
    s = p.get("state")
    if s == "alert":
      txt = f"Police   {self._dist_text(p.get('dist_mi'))}"
      d = p.get("dir")
      if d == "same":
        txt += " - your way"
      elif d == "opp":
        txt += " - other side"
      return txt, _C.ORANGE
    if s == "clear":
      return "Police   Clear", _C.GREEN
    return "Police   -", _C.DIM            # nodata: never conflated with Clear

  def _rest_line(self):
    r = self._st.get("rest", {})
    if r.get("state") == "ok":
      return f"Rest     {self._dist_text(r.get('dist_mi'))}", _C.WHITE
    return "Rest     -", _C.DIM

  def _ev_line(self):
    e = self._st.get("ev", {})
    if e.get("state") == "ok":
      txt = f"EV fast  {self._dist_text(e.get('dist_mi'))}"
      net = e.get("network")
      kw = e.get("kw")
      if net:
        txt += f" - {net}"
      if kw:
        txt += f" {int(kw)} kW"
      return txt, _C.GREEN
    return "EV fast  -", _C.DIM

  def _lines(self):
    return [
      ("HAPPENING AHEAD", _C.WHITE, self.font_bold),
      (*self._police_line(), self.font),
      (*self._rest_line(), self.font),
      (*self._ev_line(), self.font),
    ]

  # ---- render --------------------------------------------------------------
  def _render(self, rect: rl.Rectangle):
    if not self._st or not self._st.get("enabled"):
      return
    lines = self._lines()
    box_w = max(measure_text_cached(f, t, _FS).x for t, _, f in lines) + _PAD * 2
    box_h = _LINE_H * len(lines) + _PAD * 2
    bx = rect.x + _MARGIN                       # LOWER-LEFT
    by = rect.y + rect.height - box_h - _MARGIN

    rl.draw_rectangle_rounded(rl.Rectangle(bx, by, box_w, box_h), 0.12, 8, _C.BG)
    x = bx + _PAD
    y = by + _PAD
    for text, color, font in lines:
      rl.draw_text_ex(font, text, rl.Vector2(x, y), _FS, 0, color)   # left-aligned
      y += _LINE_H
