"""
location2pnw: "HAPPENING AHEAD" on-screen overlay — LOWER-LEFT, display-only.

Mirrors ces_status.py (CES/VTSC overlays render lower-RIGHT; this is lower-LEFT, verified free). Reads
the `LocationServices` JSON the pnw_location_services daemon publishes to /dev/shm/params at ~5 Hz and
renders three advisory lines (police / rest / EV fast). Never computes anything itself; never touches
control/safety. Shown unless DisableLocationServices is on (toggles-invert2pnw: opt-out, default OFF = shown).

NOTE: plain text labels (not emoji) — the openpilot Inter font has no emoji glyphs, so 👮/🛏/⚡ would
render as tofu. Swap to an icon atlas later if pictograms are wanted.
"""
import time
import pyray as rl

from cereal import log
from openpilot.common.params import Params
from openpilot.selfdrive.ui.onroad.hud_renderer import UI_CONFIG
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

AlertSize = log.SelfdriveState.AlertSize

_REFRESH_S = 0.2     # poll the mem param at ~5 Hz (matches the daemon publish cadence)
_FS_STEPS = (56, 52, 48, 44)   # base font size, then auto-shrink steps (LAST resort)
_LINE_H_RATIO = 1.25           # line height = font size * this
# Width policy (driver req 2026-07-06 v2): DEFAULT the box to exactly the header ("HAPPENING AHEAD")
# width and grow DOWN in lines; only when the box would run out of VERTICAL room may it widen, in
# steps, up to 1.5x the header. Font shrink only if even the widest cap can't fit an unbreakable word.
_WIDTH_RATIOS = (1.0, 1.25, 1.5)
_MAX_H_FRAC = 0.62             # box may use at most this fraction of the view height (keeps clear of
                               # the top header row and the bottom margin) before widening/shrinking
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
# The DM head moved to the TOP header row (2026-07-06), so this box owns the true lower-left corner.


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
    self._cached_layout = None            # (lines, fs, line_h, box_w, box_h) — rebuilt at poll time, not per frame

  def _update_state(self):
    now = time.monotonic()
    if now - self._last_poll < _REFRESH_S:
      return
    self._last_poll = now
    # toggles-invert2pnw: DisableLocationServices is opt-out (ON == hidden); shown by default.
    if self._mem is None or ui_state.params.get_bool("DisableLocationServices"):
      self._st, self._cached_layout = {}, None
      return
    try:
      st = self._mem.get("LocationServices", return_default=True)
      self._st = st if isinstance(st, dict) else {}
    except Exception:
      self._st = {}
    # Build the wrapped-line layout HERE (5 Hz) instead of in _render (20 Hz): the string assembly +
    # wrapping only depends on the polled state, so doing it per frame was pure waste.
    self._cached_layout = self._build_layout() if self._st.get("enabled") else None

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
      # driver req 2026-07-09: age since the last Waze report/confirmation, so a long-lived icon
      # (we now match the Waze app's display lifetime) can be judged for staleness at a glance.
      a = p.get("age_min")
      if a is not None:
        txt += f" ({int(a)} min)"
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

  def _wrap_px(self, text, font, fs, max_w):
    """Greedy PIXEL-measured wrap to max_w, breaking only at space runs and hanging-indenting
    continuation lines (_CONT_INDENT). Operates on string slices (not split/rejoin) so the
    multi-space tabular alignment inside a line ("Police   2.1 mi") is preserved (Gemini). A
    segment with no breakable space that still overflows is emitted as-is — the font-shrink loop
    in _build_layout handles that case by stepping the size down."""
    lines: list[str] = []
    indent = ""
    rest = text
    while rest:
      if measure_text_cached(font, indent + rest, fs).x <= max_w:
        lines.append(indent + rest)
        break
      cut = -1                               # longest prefix ending at a space that still fits
      i = rest.find(" ")
      while i > 0:
        if measure_text_cached(font, indent + rest[:i], fs).x <= max_w:
          cut = i
        else:
          break
        i = rest.find(" ", i + 1)
      if cut <= 0:                           # nothing breakable fits -> emit (overflow; shrink handles)
        lines.append(indent + rest)
        break
      lines.append(indent + rest[:cut])
      rest = rest[cut:].lstrip(" ")
      indent = _CONT_INDENT
    return lines

  def _build_layout(self):
    """Assemble header + advisory lines, pixel-wrapped. Candidate order implements the width policy:
    at each font size try the NARROW cap first (1.0x header) and only widen (1.25x, 1.5x) when the
    wrapped box would exceed the vertical budget (_MAX_H_FRAC of the view) or an unbreakable word
    overflows; step the font down only when even the widest cap can't fit. First candidate that fits
    BOTH width and height wins; the final (widest, smallest-font) candidate is the fallback.
    Returns (lines, fs, line_h, box_w, box_h)."""
    freeway = bool(self._st.get("freeway"))
    header = "HAPPENING AHEAD" if freeway else "NEARBY (3 MI)"
    content = []
    if freeway:
      content.append((*self._police_line(), self.font))
    content.append((*self._rest_line(), self.font))
    content.append((*self._ev_line(), self.font))

    view_h = self._rect.height if (self._rect is not None and self._rect.height > 200) else 1080.0
    max_h = view_h * _MAX_H_FRAC

    result = None
    for fs in _FS_STEPS:
      for ratio in _WIDTH_RATIOS:
        cap = measure_text_cached(self.font_bold, header, fs).x * ratio
        lines = [(header, _C.WHITE, self.font_bold)]
        fits_w = True
        for t, color, font in content:
          for seg in self._wrap_px(t, font, fs, cap):
            lines.append((seg, color, font))
            if measure_text_cached(font, seg, fs).x > cap:
              fits_w = False                   # unbreakable word overflows this cap
        line_h = int(fs * _LINE_H_RATIO)
        box_w = max(measure_text_cached(f, t, fs).x for t, _, f in lines) + _PAD * 2
        box_h = line_h * len(lines) + _PAD * 2
        result = (lines, fs, line_h, box_w, box_h)
        if fits_w and box_h <= max_h:
          return result
    return result                              # nothing fit the budget -> widest cap at smallest font

  # ---- render --------------------------------------------------------------
  def _render(self, rect: rl.Rectangle):
    if self._cached_layout is None or not self._st.get("enabled"):
      return
    # Yield the (bottom-anchored) space to openpilot alerts — same gate the DM head uses. At the true
    # bottom of the screen the box would otherwise sit under the alert text.
    if ui_state.sm["selfdriveState"].alertSize != AlertSize.none:
      return
    lines, fs, line_h, box_w, box_h = self._cached_layout
    bx = rect.x + _MARGIN                       # true LOWER-LEFT (DM head moved to the top header row)
    by = rect.y + rect.height - box_h - _MARGIN

    rl.draw_rectangle_rounded(rl.Rectangle(bx, by, box_w, box_h), 0.12, 8, _C.BG)
    x = bx + _PAD
    y = by + _PAD
    for text, color, font in lines:
      rl.draw_text_ex(font, text, rl.Vector2(x, y), fs, 0, color)   # left-aligned
      y += line_h

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
