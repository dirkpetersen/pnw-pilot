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
from openpilot.selfdrive.ui import UI_BORDER_SIZE
from openpilot.selfdrive.ui.onroad.driver_state import BTN_SIZE
from openpilot.selfdrive.ui.onroad.hud_renderer import UI_CONFIG
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
_CONT_INDENT = "   "   # 3-char hanging indent on each advisory line's wrapped continuation line
# Blue flashing "POLICE AHEAD" banner — same box/blink as the red speed-limit warning, but blue, when a
# police report is <= 0.5 mi AHEAD (the police line is already ahead-only, so "behind" never triggers it).
_POLICE_NEAR_MI = 0.5    # banner shows when a police report is this close ahead (restored 0.25->0.5 2026-07-01
                         # so it actually appears; the SIREN stays OFF — audio playback was the CPU/comms-timing
                         # stressor on the near-capacity 3X, the visual banner is far lighter)
_BLINK_PERIOD = 0.7   # s, one on+off cycle (~1.4 Hz), matching the speed-limit warning
_POLICE_BANNER_MAX_S = 15.0   # blink the "POLICE AHEAD" banner for at most this long, then stop (driver req)
# The driver-monitoring icon is a bottom-LEFT circle whose TOP edge is ~(UI_BORDER_SIZE + BTN_SIZE) up
# from the content bottom. Lift the box to sit just ABOVE it (small gap) so they no longer overlap.
_DRIVER_ICON_CLEAR = UI_BORDER_SIZE + BTN_SIZE + 24


class _C:
  WHITE = rl.Color(255, 255, 255, 235)
  GREY = rl.Color(175, 180, 177, 235)
  ORANGE = rl.Color(255, 149, 0, 240)
  GREEN = rl.Color(90, 205, 115, 240)
  RED = rl.Color(235, 70, 60, 240)       # poll error surfaced on the police line (e.g. quota (429), HTTP 403)
  DIM = rl.Color(140, 145, 142, 220)
  BG = rl.Color(0, 0, 0, 140)
  BLUE_BG = rl.Color(20, 90, 220, 235)   # "POLICE AHEAD" flashing banner


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
    self._banner_active = False    # police banner: 15 s blink window per report
    self._banner_uuid = None
    self._banner_start = 0.0

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

  def _town(self, t):
    return f" ({t})" if t else ""    # nearest-town sanity tag at the END of the line, e.g. " (Cle Elum)"

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
      return txt + self._town(p.get("town")), _C.ORANGE
    if s == "clear":
      return "Police   Clear", _C.GREEN
    err = p.get("err")
    if err:                                # surface the real poll error (quota (429), HTTP 403, timeout, no key)
      return f"Police   {err}", _C.RED
    return "Police   -", _C.DIM            # nodata: never conflated with Clear

  def _rest_dist_text(self, d):
    # driver request: coarse far, fine near — whole-mile steps from ~15 mi down to 3 mi, then 0.2-mi
    # steps inside 3 mi. Quantizing here means the line only changes at those steps (no flicker far out).
    if d is None:
      return ""
    if d >= 3.0:
      return f"{round(d):.0f} mi"
    return f"{round(d / 0.2) * 0.2:.1f} mi"

  def _rest_line(self):
    r = self._st.get("rest", {})
    if r.get("state") == "ok":
      txt = f"Rest     {self._rest_dist_text(r.get('dist_mi'))}"
      name = r.get("name") or ""
      d = r.get("dir") or ""
      label = f"{name} ({d})" if (name and d) else name   # display name + direction, e.g. "Gee Creek (N)"
      if label:
        txt += f"  {label}"
      return txt + self._town(r.get("town")), _C.WHITE
    return "Rest     -", _C.DIM

  def _ev_line(self):
    e = self._st.get("ev", {})
    if e.get("state") == "ok":
      label = "EV fast" if e.get("fast", True) else "EV L2"   # DC-fast vs slow Level 2 (opt-in)
      txt = f"{label}  {self._dist_text(e.get('dist_mi'))}"
      c = e.get("compass")
      if c:                                                    # compass direction to the charger, e.g. "0.5 mi (NW)"
        txt += f" ({c})"
      net = e.get("network")
      kw = e.get("kw")
      if net:
        txt += f" - {net}"
      if kw:
        txt += f" {int(kw)} kW"
      return txt + self._town(e.get("town")), _C.GREEN
    return "EV fast  -", _C.DIM

  @staticmethod
  def _split_near(s, p):
    """Split s into (head, tail) near index p, snapped to the NEAREST space so whole words stay intact.
    tail is "" when s already fits (len <= p). p comes from the longest of the 3 advisory lines."""
    if len(s) <= p:
      return s, ""
    left = s.rfind(" ", 0, p + 1)
    right = s.find(" ", p)
    cands = [i for i in (left, right) if i > 0]
    if not cands:
      return s, ""
    i = min(cands, key=lambda j: abs(j - p))
    return s[:i], s[i:].lstrip()

  def _wrap(self, content):
    """Wrap each of the 3 advisory lines (police/rest/ev) onto two lines, breaking near the middle-3 of the
    LONGEST of them (snapped to a space so words aren't cut); the continuation line is hanging-indented 3
    (_CONT_INDENT). All lines stay left-aligned. `content` = [(text,color,font),...]; returns the flat list
    with continuations inserted."""
    if not content:
      return []
    p = max(1, max(len(t) for t, _, _ in content) // 2 - 3)   # "the middle - 3" of the longest line
    out = []
    for t, color, font in content:
      head, tail = self._split_near(t, p)
      out.append((head, color, font))
      if tail:
        out.append((_CONT_INDENT + tail, color, font))
    return out

  def _lines(self):
    # Header doubles as a road-context cue: "HAPPENING AHEAD" on the highway (POIs alongside, ahead) vs
    # "NEARBY (3 MI)" on surface streets (nearest within a 3-mi radius). Police is highway-only, so its
    # line is dropped off-freeway to keep the surface view clean. The 3 advisory lines are wrapped to two
    # lines each (hanging indent); the header is not wrapped.
    freeway = bool(self._st.get("freeway"))
    content = []
    if freeway:
      content.append((*self._police_line(), self.font))
    content.append((*self._rest_line(), self.font))
    content.append((*self._ev_line(), self.font))
    return [("HAPPENING AHEAD" if freeway else "NEARBY (3 MI)", _C.WHITE, self.font_bold)] + self._wrap(content)

  # ---- render --------------------------------------------------------------
  def _render(self, rect: rl.Rectangle):
    if not self._st or not self._st.get("enabled"):
      return
    lines = self._lines()
    box_w = max(measure_text_cached(f, t, _FS).x for t, _, f in lines) + _PAD * 2
    box_h = _LINE_H * len(lines) + _PAD * 2
    bx = rect.x + _MARGIN                       # LOWER-LEFT
    by = rect.y + rect.height - box_h - _DRIVER_ICON_CLEAR   # ABOVE the driver-monitoring icon

    rl.draw_rectangle_rounded(rl.Rectangle(bx, by, box_w, box_h), 0.12, 8, _C.BG)
    x = bx + _PAD
    y = by + _PAD
    for text, color, font in lines:
      rl.draw_text_ex(font, text, rl.Vector2(x, y), _FS, 0, color)   # left-aligned
      y += _LINE_H

    # big blue flashing "POLICE AHEAD" banner when a report is <= 0.5 mi AHEAD (police is ahead-only).
    # NOTE: dist_mi rounds to 0.0 when very close (falsy), so test `is not None`, never `or 99.0`.
    # Blink for at most _POLICE_BANNER_MAX_S (15 s) per report, then stop (driver req); a NEW report
    # (different uuid) or a fresh appearance restarts the window.
    p = self._st.get("police", {})
    pd = p.get("dist_mi")
    if p.get("state") == "alert" and pd is not None and pd <= _POLICE_NEAR_MI:
      uuid = p.get("uuid")
      if not self._banner_active or uuid != self._banner_uuid:
        self._banner_start = time.monotonic()
        self._banner_uuid = uuid
      self._banner_active = True
      if time.monotonic() - self._banner_start < _POLICE_BANNER_MAX_S:
        self._draw_police_banner(rect, p)
    else:
      self._banner_active = False

  @staticmethod
  def _text_centered(font, text, size, cx, cy, color):
    sz = measure_text_cached(font, text, size)
    rl.draw_text_ex(font, text, rl.Vector2(cx - sz.x / 2, cy - sz.y / 2), size, 0, color)

  def _draw_police_banner(self, rect: rl.Rectangle, p: dict):
    # blink ~1.4 Hz like the speed-limit warning — skip the draw on the "off" half-cycle
    if (time.monotonic() % _BLINK_PERIOD) >= _BLINK_PERIOD / 2:
      return
    banner_w, banner_h = 1440, 520
    bx = rect.x + (rect.width - banner_w) / 2
    by = rect.y + UI_CONFIG.header_height + 60
    banner = rl.Rectangle(bx, by, banner_w, banner_h)
    rl.draw_rectangle_rounded(banner, 0.12, 10, _C.BLUE_BG)
    rl.draw_rectangle_rounded_lines_ex(banner, 0.12, 10, 12, _C.WHITE)
    cx = bx + banner_w / 2
    self._text_centered(self.font_bold, "POLICE AHEAD", 120, cx, by + 150, _C.WHITE)
    self._text_centered(self.font_bold, self._dist_text(p.get("dist_mi")), 220, cx, by + 350, _C.WHITE)
