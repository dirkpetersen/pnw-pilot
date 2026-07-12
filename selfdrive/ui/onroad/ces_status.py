"""
ces2xnor: on-screen CES feedback overlay — lower-right, one datum per line.

Display-only. Shown only when the CES master toggle is on. Gives at-a-glance feedback on what
Conditional Experimental Switching + mapd are doing, so you can validate it on the road.

Data path: selfdrived's CESController publishes a `CESStatus` snapshot to the in-memory param store
(/dev/shm/params) at ~5 Hz (single source of truth for the live decision + mapd diagnostics). This
widget never computes the decision itself.

Lines (lower-right, short, one per line):
  CES AUTO            button mode (AUTO / CHILL* / EXP*  — * = forced)
  > EXPERIMENTAL      effective mode (orange) / > CHILL (grey)
  why lowSpeed        binding reason (only while experimental)
  curve 57% vis       curve closeness % + source (map/vision), color ramps green->orange
  map 24pts gps       mapd liveness: cached MapTargetVelocities points + GPS fix
  next 34 140m        next binding map curve (target speed + distance) or "road clear"
  (speed-limit line removed 2026-07-01, driver req — frees bottom space)
"""
import time
import pyray as rl

from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw_constants import ces_enabled, read_ces_mode
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

_REFRESH_S = 0.2     # poll the mem param at ~5 Hz (matches the publisher)
# stophold2pnw (C): "CES: NO SIGNAL" dead-man — the 2026-07-12 false-silence investigation burned
# hours because a quiet CES was indistinguishable from a healthy-but-parked one. The publisher now
# stamps CESStatus with a wall-clock `ts` at ~5 Hz; if the master says CES should be running but
# the newest CESStatus is older than _STALE_S while onroad, the overlay says so LOUDLY (red).
_STALE_S = 5.0       # s: CESStatus ts older than this while onroad => publisher silent/dead
_GRACE_S = 10.0      # s onroad before the dead-man may alarm (selfdrived spawn + first publish;
                     #   also covers a stale /dev/shm leftover from the previous onroad session)
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
    self._ces_enabled = False
    self._onroad_t0 = None       # stophold2pnw (C): monotonic stamp of the current onroad session start
    self._no_signal = False      # stophold2pnw (C): dead-man tripped (CESStatus stale while onroad)
    self._st: dict = {}
    self._vtsc: dict = {}
    self._mapdl: str = ""
    self._cached_layout = None   # (lines, box_w, box_h) — rebuilt at poll time (5 Hz), not per frame (20 Hz)
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
    master_on = ces_enabled(read_ces_mode(ui_state.params))
    self._ces_enabled = master_on
    # ces2pnw (driver req 2026-07-10): "Hide CES debug information" toggle — default OFF (overlay
    # shows). Defensive read: on any params/UI mismatch (unregistered key) fall back to SHOWING —
    # that is the documented default state — and above all never crash the UI (params/UI mismatch
    # is this fork's documented brick scenario). Gemini flagged the fallback direction; show-by-
    # default is the deliberate choice.
    try:
      if ui_state.params.get_bool("HideCESDebug"):
        self._ces_enabled = False
    except Exception:
      pass
    # stophold2pnw (C): onroad-dwell grace timer for the dead-man. The alarm is keyed on the
    # MASTER (CES expected to run), not on the debug-hide cosmetic — an alarm is not debug
    # decoration, so HideCESDebug hides the data lines but never the NO-SIGNAL alarm.
    if not ui_state.started:
      self._onroad_t0 = None
    elif self._onroad_t0 is None:
      self._onroad_t0 = now
    grace_over = self._onroad_t0 is not None and (now - self._onroad_t0) > _GRACE_S
    self._no_signal = False
    if not master_on or self._mem is None:
      self._st = {}
      self._vtsc = {}
      self._cached_layout = None
      if master_on and self._mem is None and grace_over:
        # master says CES should run but the mem-param store is unreachable: that IS a silence
        self._no_signal = True
        self._cached_layout = self._alarm_layout()
      return
    try:
      st = self._mem.get("CESStatus", return_default=True)
      self._st = st if isinstance(st, dict) else {}
    except Exception:
      self._st = {}
    # stophold2pnw (C): staleness — publisher stamps `ts` (wall clock) at ~5 Hz; missing/old means
    # selfdrived is not publishing (dead, disabled-by-bug, or never constructed). Wall-vs-wall
    # compare; a post-NTP clock jump can only make a healthy publisher look stale for one 5 Hz
    # publish cycle (its next stamp uses the same jumped clock).
    try:
      ts = float(self._st.get("ts"))
      stale = (time.time() - ts) > _STALE_S  # noqa: TID251 -- wall-vs-wall compare against the publisher's ts heartbeat
    except (TypeError, ValueError):
      stale = True                     # no ts ever published (or garbage) counts as silence
    self._no_signal = stale and grace_over
    try:
      vt = self._mem.get("VTSCStatus", return_default=True)   # vtsc: rides the CES toggle
      self._vtsc = vt if isinstance(vt, dict) else {}
    except Exception:
      self._vtsc = {}
    try:
      mdl = self._mem.get("MapDownloadStatus", return_default=True)  # OSM DB download state (mapd_configd)
      self._mapdl = mdl.decode() if isinstance(mdl, bytes) else (mdl or "")
    except Exception:
      self._mapdl = ""
    # Build the line list + box size HERE (5 Hz poll) instead of every render frame (20 Hz): the
    # content only changes when the polled state does.
    self._cached_layout = None
    if self._no_signal:
      # stophold2pnw (C): the alarm replaces the data lines entirely — a stale snapshot must not
      # keep painting plausible-looking (dead) numbers next to the alarm.
      self._cached_layout = self._alarm_layout()
    elif self._ces_enabled and self._st.get("enabled") and int(self._st.get("button", 0)) == 0:
      lines = self._lines()
      if lines:
        box_w = max(measure_text_cached(f, t, _FS).x for t, _, f in lines) + _PAD * 2
        box_h = _LINE_H * len(lines) + _PAD * 2
        self._cached_layout = (lines, box_w, box_h)

  def _alarm_layout(self):
    """stophold2pnw (C): the single red NO-SIGNAL line as a cached layout tuple."""
    lines = [("CES: NO SIGNAL", _C.RED, self.font_bold)]
    box_w = measure_text_cached(self.font_bold, lines[0][0], _FS).x + _PAD * 2
    box_h = _LINE_H * len(lines) + _PAD * 2
    return (lines, box_w, box_h)

  # ---- build the lines -----------------------------------------------------
  def _lines(self) -> list[tuple]:
    st = self._st
    conv = self._conv
    out: list[tuple] = []

    # alphalong-mismatch warning (2026-07-12 stale-session incident): the session's longitudinal
    # authority is frozen at card start. If the Settings toggle says Alpha-Long ON but the RUNNING
    # session is on the stock-ACC/shadow path (param flipped after start, or a stale session
    # survived a param restore), the driver had NO way to see it — they drove a whole leg thinking
    # VTSC was active while ICBM stepped the set speed down. Say it loudly. Only the dangerous
    # direction is flagged (toggle ON, session stock) — the reverse is visible as plain no-op-long.
    try:
      if st.get("shadow") and ui_state.params.get_bool("AlphaLongitudinalEnabled"):
        out.append(("LONG MISMATCH - RESTART", _C.RED, self.font_bold))
    except Exception:
      pass

    button = int(st.get("button", 0))
    btn = {0: "CES AUTO", 1: "CES CHILL*", 2: "CES EXP*"}.get(button, "CES AUTO")
    out.append((btn, _C.WHITE, self.font_bold))

    is_exp = st.get("mode") == "experimental"
    # icbm2pnw (driver report 2026-07-11): in SHADOW nothing actuates — orange means "acting", so
    # shadow shows grey SHADOW-prefixed modes; orange EXPERIMENTAL is reserved for real actuation.
    if st.get("shadow"):
      out.append((">> SHADOW EXP" if is_exp else ">> SHADOW CHILL", _C.GREY, self.font_bold))
    else:
      out.append((">> EXPERIMENTAL" if is_exp else ">> CHILL", _C.ORANGE if is_exp else _C.GREY, self.font_bold))

    # VTSC (curve speed control) — rides the CES toggle; show when slowing for a curve
    vt = self._vtsc
    if vt.get("enabled"):
      if vt.get("engaged"):
        out.append((f"VTSC slowing {round(vt.get('cap', 0.0) * conv)}", _C.ORANGE, self.font_bold))
      else:
        out.append(("VTSC ready", _C.GREY, self.font))

    # icbm2pnw (driver req 2026-07-11): stock-ACC button management in ONE line — shows when taps
    # are being issued and how much: "ICBM 24>18" = stepping the stock set speed from 24 toward
    # 18 mph (ORANGE while actively lowering; grey "@18" once the target is reached/held).
    # ALWAYS visible in shadow (driver req: "I couldn't see whether it was engaged"):
    #   ICBM no-ACC  -> armed but the stock cruise is not engaged (engage ACC to give it authority)
    #   ICBM ready   -> armed, ACC engaged, no binding curve right now
    #   ICBM 24>18   -> actively stepping the stock set speed down (orange)
    if st.get("shadow"):
      it = st.get("icbmT")
      if it is not None:
        t_mph = round(float(it) * conv)
        s_mph = round(float(st.get("icbmSet") or 0.0) * conv)
        if st.get("icbmDir") == "inc":
          # icbmrestore2pnw: restoring the driver's own set after a curve — "ICBM 55>75", calm green
          out.append((f"ICBM {s_mph}>{t_mph}", _C.GREEN, self.font))
        elif s_mph > t_mph:
          out.append((f"ICBM {s_mph}>{t_mph}", _C.ORANGE, self.font_bold))
        else:
          out.append((f"ICBM @{t_mph}", _C.GREY, self.font))
      elif st.get("icbmOn"):
        out.append(("ICBM ready", _C.GREEN, self.font))
      else:
        out.append(("ICBM no-ACC", _C.GREY, self.font))

    reason = st.get("reason", "")
    if is_exp and reason and reason not in ("chill", ""):
      out.append((f"why {reason}", _C.WHITE, self.font))

    # accelerate-zone / highway-gate: held in Chill (lowSpeed suppressed) — show why.
    # Width budget (driver req 2026-07-06): no line wider than ">> EXPERIMENTAL" — the old
    # "hwy-gate (no lowSpd)" single line made the whole box ~1.5x wider, so it wraps to two lines.
    if st.get("accelZone"):
      out.append(("accel-zone open", _C.GREEN, self.font))
    if st.get("hwyGate"):
      out.append(("hwy-gate", _C.GREEN, self.font))
      out.append(("(no lowSpd)", _C.GREEN, self.font))

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
      out.append((f"map {pts}p no-gps", _C.ORANGE, self.font))   # "p" not "pts": width budget
    else:
      out.append((f"map {pts}pts gps", _C.GREEN, self.font))

    # SEPARATE from the live map-data line above: is the OSM map DB actually downloaded? (driver asked for
    # this so "map no-data" isn't conflated with "maps not installed".) green=OK, orange=downloading, red=not.
    mdl = self._mapdl
    if mdl:
      col = _C.GREEN if mdl == "OK" else (_C.ORANGE if mdl.startswith("downloading") else _C.RED)
      # keep within the width budget: "downloading 42%" -> "dl 42%"; long/error strings get a
      # visible ellipsis instead of a silent mid-word chop (Gemini: "unreachable" -> "unreacha")
      disp = mdl.replace("downloading", "dl")
      if len(disp) > 8:
        disp = disp[:7] + "…"
      out.append((f"map-DB {disp}", col, self.font))

    # next binding map curve (a real slowdown ahead) -> else the lead gap if one is tracked -> else clear.
    # ces-i90-2pnw: "road clear" used to show even with a car right in front (5/12 drive: "road obviously
    # not clear"). Only call it clear when there's NEITHER an upcoming map curve NOR a tracked lead.
    mapv = float(st.get("mapV", 0.0))
    mapd = float(st.get("mapDist", 0.0))
    drel = float(st.get("dRel", 0.0))
    if 0.0 < mapv < _REAL_CURVE_MS and mapd > 0.0:
      out.append((f"next {round(mapv * conv)} {round(mapd)}m", _C.ORANGE, self.font))
    elif drel > 0.0:
      out.append((f"lead {round(drel)}m", _C.GREY, self.font))
    elif pts > 0 and gps:
      out.append(("road clear", _C.GREEN, self.font))

    # (speed-limit line removed 2026-07-01, driver req — frees space at the bottom of the CES overlay)

    return out

  # ---- render --------------------------------------------------------------
  def _render(self, rect: rl.Rectangle):
    # visibility gates (enabled, CES-auto button mode only, NO-SIGNAL alarm) are applied at poll
    # time in _update_state; _cached_layout is None whenever the overlay should be hidden.
    # stophold2pnw (C): gate on the layout ALONE — the NO-SIGNAL alarm must render even when
    # HideCESDebug has turned the data overlay off (an alarm is not debug decoration).
    if self._cached_layout is None:
      return
    lines, box_w, box_h = self._cached_layout
    bx = rect.x + rect.width - box_w - _MARGIN
    by = rect.y + rect.height - box_h - _MARGIN

    rl.draw_rectangle_rounded(rl.Rectangle(bx, by, box_w, box_h), 0.12, 8, _C.BG)
    right = bx + box_w - _PAD
    y = by + _PAD
    for text, color, font in lines:
      w = measure_text_cached(font, text, _FS).x
      rl.draw_text_ex(font, text, rl.Vector2(right - w, y), _FS, 0, color)   # right-aligned
      y += _LINE_H
