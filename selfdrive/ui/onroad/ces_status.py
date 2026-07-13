"""
ces2xnor: on-screen CES feedback overlay — lower-right, one datum per line.

Display-only. Shown only when the CES master toggle is on. Gives at-a-glance feedback on what
Conditional Experimental Switching + mapd are doing, so you can validate it on the road.

Data path: selfdrived's CESController publishes a `CESStatus` snapshot to the in-memory param store
(/dev/shm/params) at ~5 Hz (single source of truth for the live decision + mapd diagnostics). This
widget never computes the decision itself.

cesui2pnw (driver req 2026-07-12): "dark cockpit" redesign — the box must never grow into the road
path while moving. Three tiers:
  moving, all quiet (2 lines):    CES AUTO ● ● ●        dots = map / gps / long health (green=OK)
  moving, event live (≤5 lines):  + ICBM 75>62 / VTSC slowing / why <reason> / next 34 140m ...
                                  a non-green dot expands into its full text line (map no-data, ...)
  standstill (<~1 mph):           the FULL diagnostic dump at a smaller font — you can read at a
                                  red light; while moving you only glance.
The ">> CHILL / >> EXPERIMENTAL" effective-mode line is GONE from the moving view (driver: the
top-right icon already tracks selfdriveState.experimentalMode live — white=CES-chill, yellow=CES-
exp, orange=forced); it survives only inside the standstill dump (shadow sessions included, where
the icon can't show the shadow decision).
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
_FS = 64             # moving-view font (driver feedback: the original small font was unreadable)
_LINE_H = 80
_FS_SM = 48          # standstill-dump font — 1.5x the original size, readable when stopped
_LINE_H_SM = 62
_PAD = 24
_MARGIN = 40         # gap from the screen's right / bottom edges
_REAL_CURVE_MS = 40.0  # a map target speed below this (~90 mph) counts as a real curve to preview
_MAX_LINES_MOVING = 5  # hard cap while moving — the box must never stack into the road path
_CURVE_SHOW_PCT = 40   # moving view: curve% below this is noise, dot/nothing instead
_NEXT_CURVE_S = 20.0   # moving view: preview the next map curve only within this many seconds
# standstill hysteresis (m/s): crawl speeds must not flicker dump<->tiered every frame
_DUMP_ENTER_V = 0.5
_DUMP_EXIT_V = 1.5


class _C:
  WHITE = rl.Color(255, 255, 255, 235)
  GREY = rl.Color(175, 180, 177, 235)
  ORANGE = rl.Color(255, 149, 0, 240)
  GREEN = rl.Color(90, 205, 115, 240)
  RED = rl.Color(235, 70, 70, 240)
  BG = rl.Color(0, 0, 0, 140)


def _dots_w(fs: float, n: int) -> float:
  """Width the health-dot strip adds after a line's text (lead gap + n dots + gaps between)."""
  if n <= 0:
    return 0.0
  r = fs * 0.19
  return fs * 0.28 + n * 2 * r + (n - 1) * fs * 0.22


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
    self._dump = False           # cesui2pnw: standstill full-dump view active (hysteresis)
    self._st: dict = {}
    self._vtsc: dict = {}
    self._mapdl: str = ""
    self._cached_layout = None   # (entries, box_w, box_h, fs, line_h) — rebuilt at poll time (5 Hz)
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
    # cesui2pnw: standstill detection with hysteresis (enter <0.5 m/s, exit >1.5 m/s) so crawl
    # speeds don't flicker between the dump and the tiered view. Defensive: a dead carState
    # (dashcam session) reads as 0 -> dump, which is the harmless direction when not moving.
    try:
      v_ego = abs(float(ui_state.sm["carState"].vEgo))
    except Exception:
      v_ego = 0.0
    self._dump = (v_ego < _DUMP_EXIT_V) if self._dump else (v_ego < _DUMP_ENTER_V)
    # Build the line list + box size HERE (5 Hz poll) instead of every render frame (20 Hz): the
    # content only changes when the polled state does.
    self._cached_layout = None
    if self._no_signal:
      # stophold2pnw (C): the alarm replaces the data lines entirely — a stale snapshot must not
      # keep painting plausible-looking (dead) numbers next to the alarm.
      self._cached_layout = self._alarm_layout()
    elif self._ces_enabled and self._st.get("enabled") and int(self._st.get("button", 0)) == 0:
      if self._dump:
        entries, fs, line_h = self._lines_full(), _FS_SM, _LINE_H_SM
      else:
        entries, fs, line_h = self._lines_moving(), _FS, _LINE_H
      if entries:
        box_w = max(measure_text_cached(f, t, fs).x + _dots_w(fs, len(d) if d else 0)
                    for t, _, f, d in entries) + _PAD * 2
        box_h = line_h * len(entries) + _PAD * 2
        self._cached_layout = (entries, box_w, box_h, fs, line_h)

  def _alarm_layout(self):
    """stophold2pnw (C): the single red NO-SIGNAL line as a cached layout tuple."""
    entries = [("CES: NO SIGNAL", _C.RED, self.font_bold, None)]
    box_w = measure_text_cached(self.font_bold, entries[0][0], _FS).x + _PAD * 2
    box_h = _LINE_H + _PAD * 2
    return (entries, box_w, box_h, _FS, _LINE_H)

  # ---- shared bits -----------------------------------------------------------
  def _health_dots(self) -> list:
    """cesui2pnw: 3 dots = map-data / gps / long-channel. Green carries the same information the
    old 'map 24pts gps' + 'map-DB OK' + 'ICBM ready' lines did, in a fraction of the space."""
    st = self._st
    pts = int(st.get("mapPts", 0))
    mdl = self._mapdl
    if pts > 0:
      map_dot = _C.GREEN
    elif mdl.startswith("downloading"):
      map_dot = _C.ORANGE
    else:
      map_dot = _C.RED
    gps_dot = _C.GREEN if bool(st.get("gps", False)) else _C.ORANGE
    if st.get("shadow"):
      # grey = idle-by-design (stock cruise not engaged), NOT a fault — grey never expands
      long_dot = _C.GREEN if st.get("icbmOn") else _C.GREY
    else:
      long_dot = _C.GREEN if self._vtsc.get("enabled") else _C.GREY
    return [map_dot, gps_dot, long_dot]

  def _mismatch_line(self):
    # alphalong-mismatch warning (2026-07-12 stale-session incident): the session's longitudinal
    # authority is frozen at card start. If the Settings toggle says Alpha-Long ON but the RUNNING
    # session is on the stock-ACC/shadow path (param flipped after start, or a stale session
    # survived a param restore), the driver had NO way to see it — they drove a whole leg thinking
    # VTSC was active while ICBM stepped the set speed down. Say it loudly. Only the dangerous
    # direction is flagged (toggle ON, session stock) — the reverse is visible as plain no-op-long.
    try:
      if self._st.get("shadow") and ui_state.params.get_bool("AlphaLongitudinalEnabled"):
        return ("LONG MISMATCH - RESTART", _C.RED, self.font_bold, None)
    except Exception:
      pass
    return None

  def _icbm_line(self, active_only: bool):
    """One-line stock-ACC button management state. active_only (moving view) returns None for the
    idle states — the long health dot covers them; the dump shows them spelled out."""
    st = self._st
    conv = self._conv
    it = st.get("icbmT")
    if it is not None:
      t_mph = round(float(it) * conv)
      s_mph = round(float(st.get("icbmSet") or 0.0) * conv)
      if st.get("icbmDir") == "inc":
        # icbmrestore2pnw: restoring the driver's own set after a curve — "ICBM 55>75", calm green
        return (f"ICBM {s_mph}>{t_mph}", _C.GREEN, self.font, None)
      elif s_mph > t_mph:
        return (f"ICBM {s_mph}>{t_mph}", _C.ORANGE, self.font_bold, None)
      else:
        return (f"ICBM @{t_mph}", _C.GREY, self.font, None)
    if active_only:
      return None
    if st.get("icbmOn"):
      return ("ICBM ready", _C.GREEN, self.font, None)
    return ("ICBM no-ACC", _C.GREY, self.font, None)

  # ---- moving view: tiered, capped -------------------------------------------
  def _lines_moving(self) -> list[tuple]:
    """Priority-ordered; hard-capped at _MAX_LINES_MOVING so the box can never grow into the
    road path. Steady-state/liveness lines are dots; only live events earn a text line."""
    st = self._st
    conv = self._conv
    out: list[tuple] = []

    mm = self._mismatch_line()
    if mm:
      out.append(mm)

    button = int(st.get("button", 0))
    btn = {0: "CES AUTO", 1: "CES CHILL*", 2: "CES EXP*"}.get(button, "CES AUTO")
    out.append((btn, _C.WHITE, self.font_bold, self._health_dots()))

    icbm = self._icbm_line(active_only=True)
    if icbm:
      out.append(icbm)

    vt = self._vtsc
    if vt.get("enabled") and vt.get("engaged"):
      out.append((f"VTSC slowing {round(vt.get('cap', 0.0) * conv)}", _C.ORANGE, self.font_bold, None))

    # the ">> EXPERIMENTAL" mode line is gone (top-right icon tracks the live mode) — the why-line
    # alone implies Experimental; grey in shadow where the decision doesn't actuate.
    reason = st.get("reason", "")
    if st.get("mode") == "experimental" and reason and reason not in ("chill", ""):
      out.append((f"why {reason}", _C.GREY if st.get("shadow") else _C.WHITE, self.font, None))

    # next binding map curve — only when it's actually near (within _NEXT_CURVE_S at current speed)
    mapv = float(st.get("mapV", 0.0))
    mapd = float(st.get("mapDist", 0.0))
    if 0.0 < mapv < _REAL_CURVE_MS and mapd > 0.0:
      try:
        v_now = max(abs(float(ui_state.sm["carState"].vEgo)), 1.0)
      except Exception:
        v_now = 1.0
      if mapd / v_now < _NEXT_CURVE_S:
        out.append((f"next {round(mapv * conv)} {round(mapd)}m", _C.ORANGE, self.font, None))

    pct = max(0, min(100, int(st.get("curvePct", 0))))
    if pct >= _CURVE_SHOW_PCT:
      src = st.get("curveSrc", "") or "--"
      pct_col = _C.ORANGE if pct < 100 else _C.RED
      out.append((f"curve {pct}% {src}", pct_col, self.font, None))

    # a non-green health dot expands into its full text line so a problem is spelled out in words
    pts = int(st.get("mapPts", 0))
    if pts == 0:
      mdl = self._mapdl
      if mdl.startswith("downloading"):
        out.append((f"map-DB {mdl.replace('downloading', 'dl')[:8]}", _C.ORANGE, self.font, None))
      else:
        out.append(("map no-data", _C.RED, self.font, None))
    elif not bool(st.get("gps", False)):
      out.append((f"map {pts}p no-gps", _C.ORANGE, self.font, None))

    # accelerate-zone / highway-gate: held in Chill (lowSpeed suppressed). Lowest priority while
    # moving; the dump carries the long form.
    if st.get("accelZone"):
      out.append(("accel-zone open", _C.GREEN, self.font, None))
    if st.get("hwyGate"):
      out.append(("hwy-gate", _C.GREEN, self.font, None))

    return out[:_MAX_LINES_MOVING]

  # ---- standstill view: the full diagnostic dump ------------------------------
  def _lines_full(self) -> list[tuple]:
    """Everything, small font — readable at a red light, which is exactly when you debug."""
    st = self._st
    conv = self._conv
    out: list[tuple] = []

    mm = self._mismatch_line()
    if mm:
      out.append(mm)

    button = int(st.get("button", 0))
    btn = {0: "CES AUTO", 1: "CES CHILL*", 2: "CES EXP*"}.get(button, "CES AUTO")
    out.append((btn, _C.WHITE, self.font_bold, self._health_dots()))

    is_exp = st.get("mode") == "experimental"
    # icbm2pnw (driver report 2026-07-11): in SHADOW nothing actuates — orange means "acting", so
    # shadow shows grey SHADOW-prefixed modes; orange EXPERIMENTAL is reserved for real actuation.
    # (Moving view drops this line — the top-right icon is live — but the dump keeps it: in shadow
    # the icon can't show the shadow decision, and the dump is the full picture by contract.)
    if st.get("shadow"):
      out.append((">> SHADOW EXP" if is_exp else ">> SHADOW CHILL", _C.GREY, self.font_bold, None))
    else:
      out.append((">> EXPERIMENTAL" if is_exp else ">> CHILL", _C.ORANGE if is_exp else _C.GREY, self.font_bold, None))

    vt = self._vtsc
    if vt.get("enabled"):
      if vt.get("engaged"):
        out.append((f"VTSC slowing {round(vt.get('cap', 0.0) * conv)}", _C.ORANGE, self.font_bold, None))
      else:
        out.append(("VTSC ready", _C.GREY, self.font, None))

    if st.get("shadow"):
      icbm = self._icbm_line(active_only=False)
      if icbm:
        out.append(icbm)

    reason = st.get("reason", "")
    if is_exp and reason and reason not in ("chill", ""):
      out.append((f"why {reason}", _C.WHITE, self.font, None))

    if st.get("accelZone"):
      out.append(("accel-zone open", _C.GREEN, self.font, None))
    if st.get("hwyGate"):
      out.append(("hwy-gate", _C.GREEN, self.font, None))
      out.append(("(no lowSpd)", _C.GREEN, self.font, None))

    pct = max(0, min(100, int(st.get("curvePct", 0))))
    src = st.get("curveSrc", "") or "--"
    pct_col = _C.GREEN if pct < 60 else (_C.ORANGE if pct < 100 else _C.RED)
    out.append((f"curve {pct}% {src}", pct_col, self.font, None))

    # mapd liveness
    pts = int(st.get("mapPts", 0))
    gps = bool(st.get("gps", False))
    if pts == 0:
      out.append(("map no-data", _C.RED, self.font, None))
    elif not gps:
      out.append((f"map {pts}p no-gps", _C.ORANGE, self.font, None))   # "p" not "pts": width budget
    else:
      out.append((f"map {pts}pts gps", _C.GREEN, self.font, None))

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
      out.append((f"map-DB {disp}", col, self.font, None))

    # next binding map curve (a real slowdown ahead) -> else the lead gap if one is tracked -> else clear.
    # ces-i90-2pnw: "road clear" used to show even with a car right in front (5/12 drive: "road obviously
    # not clear"). Only call it clear when there's NEITHER an upcoming map curve NOR a tracked lead.
    mapv = float(st.get("mapV", 0.0))
    mapd = float(st.get("mapDist", 0.0))
    drel = float(st.get("dRel", 0.0))
    if 0.0 < mapv < _REAL_CURVE_MS and mapd > 0.0:
      out.append((f"next {round(mapv * conv)} {round(mapd)}m", _C.ORANGE, self.font, None))
    elif drel > 0.0:
      out.append((f"lead {round(drel)}m", _C.GREY, self.font, None))
    elif pts > 0 and gps:
      out.append(("road clear", _C.GREEN, self.font, None))

    return out

  # ---- render --------------------------------------------------------------
  def _render(self, rect: rl.Rectangle):
    # visibility gates (enabled, CES-auto button mode only, NO-SIGNAL alarm) are applied at poll
    # time in _update_state; _cached_layout is None whenever the overlay should be hidden.
    # stophold2pnw (C): gate on the layout ALONE — the NO-SIGNAL alarm must render even when
    # HideCESDebug has turned the data overlay off (an alarm is not debug decoration).
    if self._cached_layout is None:
      return
    entries, box_w, box_h, fs, line_h = self._cached_layout
    bx = rect.x + rect.width - box_w - _MARGIN
    by = rect.y + rect.height - box_h - _MARGIN

    rl.draw_rectangle_rounded(rl.Rectangle(bx, by, box_w, box_h), 0.12, 8, _C.BG)
    right = bx + box_w - _PAD
    y = by + _PAD
    for text, color, font, dots in entries:
      tw = measure_text_cached(font, text, fs).x
      total_w = tw + _dots_w(fs, len(dots) if dots else 0)
      x = right - total_w
      rl.draw_text_ex(font, text, rl.Vector2(x, y), fs, 0, color)   # right-aligned as a block
      if dots:
        r = fs * 0.19
        cx = x + tw + fs * 0.28 + r
        cy = y + fs * 0.55
        for dc in dots:
          rl.draw_circle(int(cx), int(cy), r, dc)
          cx += 2 * r + fs * 0.22
      y += line_h
