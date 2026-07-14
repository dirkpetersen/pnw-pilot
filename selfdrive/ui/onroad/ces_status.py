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
  standstill (<~1 mph):           the full picture as a GROUPED COCKPIT CARD (dumpui2pnw, driver
                                  feedback 2026-07-13: the old one-datum-per-line dump "looks a
                                  little crude"). Structure — alarms (red, full width, on top) /
                                  a HEADLINE (button identity + the 3 health dots) / the effective
                                  mode (shadow-aware) / a REASON line (why <trigger/hold>) / a
                                  fixed-slot 3x2 label:value grid (map,DB,gps | vtsc,long,curve —
                                  every slot always present, "--" when idle, so the eye learns where
                                  each datum lives) / an always-present one-line situation strip
                                  (next-curve | lead | road clear | —) / short active-only tag chips
                                  (rain / accel-zone / hwy-gate+no-lowSpd / STEER 4-SIG). The card
                                  WIDTH for the everything-OK case is fixed (grid + normal lines,
                                  measured once from PER-COLUMN worst-case exemplars, never from live
                                  values) so it never dances left/right as numbers change at a stop.
                                  dumpnarrow2pnw (driver feedback 2026-07-13: "too wide, dead gap
                                  between the columns"): the two grid columns are now sized
                                  INDEPENDENTLY — the left (map/DB/gps: short, "142p"/"dl 00%"/"ok")
                                  is far narrower than the right (vtsc/long/curve: "slow 000"/
                                  "000>000"/"000% map"), so the right column slides left and the dead
                                  gap closes. A rare RED alarm ("LONG MISMATCH - RESTART" / "STEER: NO
                                  4SIG PANDA") may WIDEN the card only when it actually appears (a big,
                                  rare event, measured so it never clips) — that is NOT the per-value
                                  jitter Gemini flagged; the OK card stays put. Height changes only
                                  when an alarm or flag chip appears — itself information. Same data
                                  as the old dump, regrouped.
The ">> CHILL / >> EXPERIMENTAL" effective-mode line is GONE from the moving view (driver: the
top-right icon already tracks selfdriveState.experimentalMode live — white=CES-chill, yellow=CES-
exp, orange=forced); it survives only inside the standstill dump (shadow sessions included, where
the icon can't show the shadow decision).
"""
import time
import pyray as rl

from cereal import log
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw_constants import ces_enabled, read_ces_mode
from openpilot.selfdrive.ui.ui_state import ui_state

# rain2pnw: magnitudes for the "RAIN armed" indicator, from the same source the controllers use
# (defaults 3/5 mph; /data/pnw/rain.json can retune). Never fatal if the import is unavailable.
try:
  from openpilot.selfdrive.controls.lib.pnw_vehicle import _load_rain_config as _rain_cfg_loader
except Exception:
  _rain_cfg_loader = None
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
_FS_SM = 48          # standstill-card font — 1.5x the original size, readable when stopped
_LINE_H_SM = 62
# dumpui2pnw: standstill grouped-card geometry (all derived from the card font _FS_SM)
_SEP_H = 22          # height of a thin group-divider row
_COL_GAP = 30        # horizontal gap between the grid's two columns (dumpnarrow2pnw: 44->30, tighter)
_LBL_GAP = 16        # gap between a grid cell's label and its value
_DOT_R = _FS_SM * 0.19
# STABLE WIDTH: the card's resting (everything-OK) width is measured ONCE from fixed worst-case
# exemplars (with the real font, so it is metric-correct) — NOT from the live values — so it can never
# dance left/right as numbers change at a stop (Gemini review 2026-07-13). The exemplars below cover
# every NORMAL string the card can show; live text is left-aligned inside the fixed box.
# dumpnarrow2pnw: the two grid columns are sized INDEPENDENTLY, each from its own side's worst-case
# values. The left side (map/DB/gps) is short ("142p"/"dl 000%"/"ok"); the right side (vtsc/long/
# curve) is wide ("slow 000"/"000>000"/"000% map"). Sizing BOTH from the right's generous exemplar
# (the old single _GRID_VALUE_EX) left a big dead gap between the columns — the driver's complaint.
_GRID_L_LABELS = ("map", "DB", "gps")                  # left-column labels
_GRID_L_VALUES = ("dl 000%", "0000p")                  # widest left values (DB download / map points)
_GRID_R_LABELS = ("vtsc", "long", "curve")             # right-column labels
_GRID_R_VALUES = ("slow 000", "000>000", "000% map")   # widest right values
# widest NORMAL (non-alarm) full-width lines — effective-mode / why-reason / situation-strip. The rare
# RED alarm lines are deliberately NOT here (dumpnarrow2pnw option b): they are measured per-build in
# _build_card so an APPEARING alarm may widen the card (and never clip) instead of inflating the
# everything-OK width. ">> SHADOW EXPERIMENTAL" (the old 21-char exemplar) is not a string the card
# can emit — the mode row shows at most ">> SHADOW CHILL" / ">> EXPERIMENTAL" (15 chars).
_CARD_LINE_EX = (
  ">> SHADOW CHILL", ">> EXPERIMENTAL",    # widest effective-mode strings (">> SHADOW EXP"/">> CHILL" shorter)
  "why standstillHold",                     # widest why-reason
  "next 000 000m",                          # widest situation strip
)
_PAD = 24
_MARGIN = 40         # gap from the screen's right / bottom edges
_REAL_CURVE_MS = 40.0  # a map target speed below this (~90 mph) counts as a real curve to preview
_MAX_LINES_MOVING = 5  # hard cap while moving — the box must never stack into the road path
_CURVE_SHOW_PCT = 40   # moving view: curve% below this is noise, dot/nothing instead
_NEXT_CURVE_S = 20.0   # moving view: preview the next map curve only within this many seconds
_FORDLAT_STALE_S = 5.0  # fordlatui2pnw: FordLatStatus older than this => not a live Ford session (Tesla)
_STEERFAULT_TRIP = 20   # fordlatui2pnw: leaky-bucket level (~4 s of predominantly-faulting at 5 Hz)
                        #   while sending 4-signal => the panda is rejecting it (stock safety, flash
                        #   reverted). Leaky bucket (+1 on fault, -2 on clean) tolerates CAN/1-frame
                        #   jitter yet still fills on a true continuous reject (Gemini 2026-07-13).
_STEERFAULT_LEAK = 2    # bucket drain per clean poll
_4SIG_CLEAN_LOCK = 50   # ~10 s of clean 4-signal-at-speed => the panda flash is PROVEN good this
                        #   session (panda safety is fixed at boot, can't revert mid-drive), so a
                        #   later transient EPS fault (the known "steering unavailable >20mph" quirk)
                        #   can NEVER false-fire the panda-mismatch alarm afterwards.
_MPH_TO_MS = 0.44704    # rain2pnw: rain.json magnitudes are in mph
_CAL_STATUS = log.LiveCalibrationData.Status
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
  SEP = rl.Color(120, 124, 122, 150)   # dumpui2pnw: dim group-divider line


def _f(v, d: float = 0.0) -> float:
  """Gemini review 2026-07-12: CESStatus is an arbitrary dict off /dev/shm — a key holding an
  explicit None sails through dict.get(k, default) and int(None)/None*conv would crash-loop the
  UI (this fork's documented brick scenario). All numeric reads go through these coercers."""
  try:
    return float(v)
  except (TypeError, ValueError):
    return d


def _i(v, d: int = 0) -> int:
  try:
    return int(v)
  except (TypeError, ValueError):
    return d


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
    self._fordlat: dict = {}     # fordlatui2pnw: {"mode","ts"} published by the Ford carcontroller
    self._steerfault_n = 0       # fordlatui2pnw: leaky-bucket level of steer faults while sending 4-signal
    self._clean_streak = 0       # fordlatui2pnw: consecutive clean 4-signal-at-speed polls
    self._4sig_ok = False        # fordlatui2pnw: panda proven to accept 4-signal this session (locks off the alarm)
    self._cached_layout = None   # (entries, box_w, box_h, fs, line_h) — rebuilt at poll time (5 Hz)
    self._card_layout = None     # dumpui2pnw: standstill grouped-card layout (built at poll time);
                                 #   takes precedence over _cached_layout when non-None
    self._card_metrics = None    # dumpui2pnw: (box_w_inner, label_w, col_w) — fixed, computed once
                                 #   from exemplars so the card width never jitters as values change
    # rain2pnw: read the configured rain magnitudes once (mph); defaults 3/5 if unavailable
    self._rain_mph = {1: 3.0, 2: 5.0}
    if _rain_cfg_loader is not None:
      try:
        cfg = _rain_cfg_loader()
        self._rain_mph = {1: float(cfg["light_mph"]), 2: float(cfg["heavy_mph"])}
      except Exception:
        pass
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
    try:
      fl = self._mem.get("FordLatStatus", return_default=True)  # fordlatui2pnw: which lateral path is live
      self._fordlat = fl if isinstance(fl, dict) else {}
    except Exception:
      self._fordlat = {}
    # fordlatui2pnw (driver req 2026-07-13): the PANDA-mismatch case. When we ARE sending 4-signal
    # (mode=="4sig") but the panda is on STOCK ford safety, it BLOCKS the nonzero curvature_rate and
    # lateral goes DEAD -> a SUSTAINED steer fault at speed. Leaky bucket (fills on fault, drains on
    # clean) tolerates CAN jitter yet fills on a true continuous reject; a clean streak locks the
    # alarm off for the session (panda safety is set at boot, can't revert mid-drive — so if 4-signal
    # drove clean early, a later transient EPS fault is definitely NOT a mismatch). NOTE (limitation,
    # Gemini): relies on carState.steerFault as the reject proxy — an EPS that silently falls back to
    # stock LKAS without a fault flag would go unwarned; catching that would need pandaState txRejects.
    try:
      cs = ui_state.sm["carState"]
      moving = abs(float(cs.vEgo)) > 5.0
      sending_4sig = str(self._fordlat.get("mode") or "") == "4sig"
      faulting = bool(cs.steerFaultTemporary or cs.steerFaultPermanent)
      if sending_4sig and moving:
        if faulting:
          self._steerfault_n = min(self._steerfault_n + 1, _STEERFAULT_TRIP)
          self._clean_streak = 0
        else:
          self._steerfault_n = max(self._steerfault_n - _STEERFAULT_LEAK, 0)
          self._clean_streak += 1
          if self._clean_streak >= _4SIG_CLEAN_LOCK:
            self._4sig_ok = True          # panda proven to accept 4-signal — lock the alarm off
      else:
        self._steerfault_n = 0            # not sending/at speed: no basis to judge (keep the lock)
    except Exception:
      self._steerfault_n = 0
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
    self._card_layout = None      # dumpui2pnw: exactly one of _cached_layout / _card_layout is set
    if self._no_signal:
      # stophold2pnw (C): the alarm replaces the data lines entirely — a stale snapshot must not
      # keep painting plausible-looking (dead) numbers next to the alarm.
      self._cached_layout = self._alarm_layout()
    elif self._ces_enabled and self._st.get("enabled") and _i(self._st.get("button", 0)) == 0:
      if self._dump:
        # dumpui2pnw: standstill grouped cockpit card (own layout + renderer)
        self._card_layout = self._build_card()
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
    pts = _i(st.get("mapPts", 0))
    mdl = self._mapdl
    if pts > 0:
      map_dot = _C.GREEN                               # live forward path -> map curve-braking ARMED
    elif mdl.startswith(("downloading", "incomplete")) or mdl == "OK":
      # ORANGE = "coming, not armed yet": either the OSM DB is still downloading, OR the DB is present
      # ("OK") but mapd has no live matched forward path yet — the ~30-60 s WARMING window after a fresh
      # ignition (GPS lock + way-match pending). Distinct from RED so a "100% downloaded" DB is never
      # misread as "curve-braking is armed" (the Terwilliger fresh-ignition surprise).
      map_dot = _C.ORANGE
    else:
      map_dot = _C.RED                                 # no maps / mapd down -> no map curve capability
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

  def _calib_line(self):
    """suggestion #1: calibration status. openpilot won't engage until calibrated — after the auto
    car-swap reset (calswap2pnw) this is exactly what tells the driver WHY it won't turn on. Shows a
    counting-up % while (re)calibrating; silent once calibrated. Fully defensive."""
    try:
      cal = ui_state.sm["liveCalibration"]
      status = cal.calStatus
      pct = max(0, min(100, int(cal.calPerc)))
    except Exception:
      return None
    if status == _CAL_STATUS.calibrated:
      return None
    if status == _CAL_STATUS.invalid:
      return ("CALIB INVALID", _C.RED, self.font_bold, None)
    return (f"CALIBRATING {pct}%", _C.ORANGE, self.font_bold, None)   # uncalibrated / recalibrating

  def _steer_line(self):
    """suggestion #2 (+ driver req 2026-07-13): the alan-polk 4-signal lateral tell, BOTH failure
    modes. The Ford carcontroller publishes FordLatStatus.mode ('4sig'/'pc'/'stock'). Fresh + not
    '4sig' => the SOFTWARE fell back to stock (LateralCurvExt didn't construct): `STEER: STOCK`.
    Fresh + '4sig' but a SUSTAINED steer fault at speed => the PANDA is rejecting the 4-signal
    (flashed with stock ford safety, so nonzero curvature_rate is blocked and lateral is dead):
    `STEER: NO 4SIG PANDA` — the panda/openpilot mismatch that can't be seen any other way while
    driving. Stale/absent = Tesla/no-Ford -> nothing. Green confirm is dump-only."""
    try:
      ts = float(self._fordlat.get("ts"))
      if (time.time() - ts) > _FORDLAT_STALE_S:  # noqa: TID251 -- wall-vs-wall vs the publisher heartbeat
        return None
      mode = str(self._fordlat.get("mode") or "")
    except (TypeError, ValueError):
      return None
    if not mode:
      return None
    if mode != "4sig":
      return ("STEER: STOCK", _C.RED, self.font_bold, None)      # software fallback
    if self._steerfault_n >= _STEERFAULT_TRIP and not self._4sig_ok:
      return ("STEER: NO 4SIG PANDA", _C.RED, self.font_bold, None)  # panda rejecting 4-signal (flash reverted)
    return None   # healthy 4-signal: dark in the moving view (dump adds a green confirm separately)

  def _rain_line(self):
    """suggestion #3: 'RAIN armed' indicator. Rain slowdown only bites inside curves, so a persistent
    small tag confirms the driver actually left it on. Silent when None. Reduction shown in the
    driver's speed unit."""
    try:
      tier = int(ui_state.params.get("RainMode", return_default=True) or 0)
    except Exception:
      return None
    if tier not in (1, 2):
      return None
    red = round(self._rain_mph.get(tier, 0.0) * _MPH_TO_MS * self._conv)
    return (f"RAIN -{red}", _C.GREY, self.font, None)

  def _upload_err_line(self):
    """uploadretry2pnw: surface the uploader's last HARD upload failure so a stuck upload is visible.
    Silent when clear (the uploader removes LastUploadError on the next success + on startup, and
    writes it change-only). 412 'already there' is never recorded, so this only lights on real errors."""
    try:
      err = ui_state.params.get("LastUploadError")
    except Exception:
      return None
    if not err:
      return None
    if isinstance(err, bytes):
      err = err.decode("utf-8", "replace")
    return (f"UP ERR {err}", _C.RED, self.font, None)

  def _icbm_line(self, active_only: bool):
    """One-line stock-ACC button management state. active_only (moving view) returns None for the
    idle states — the long health dot covers them; the dump shows them spelled out."""
    st = self._st
    conv = self._conv
    it = st.get("icbmT")
    if it is not None:
      t_mph = round(_f(it) * conv)
      s_mph = round(_f(st.get("icbmSet")) * conv)
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

    # highest priority — abnormal states that must never be dropped by the cap (all appended before
    # the events below): long-mismatch (red), 4-signal fell to stock (red), (re)calibrating (orange).
    for line in (self._mismatch_line(), self._steer_line(), self._calib_line(), self._upload_err_line()):
      if line:
        out.append(line)

    button = _i(st.get("button", 0))
    btn = {0: "CES AUTO", 1: "CES CHILL*", 2: "CES EXP*"}.get(button, "CES AUTO")
    out.append((btn, _C.WHITE, self.font_bold, self._health_dots()))

    rain = self._rain_line()   # armed-indicator, right under the identity line
    if rain:
      out.append(rain)

    icbm = self._icbm_line(active_only=True)
    if icbm:
      out.append(icbm)

    vt = self._vtsc
    if vt.get("enabled") and vt.get("engaged"):
      out.append((f"VTSC slowing {round(_f(vt.get('cap')) * conv)}", _C.ORANGE, self.font_bold, None))

    # the ">> EXPERIMENTAL" mode line is gone (top-right icon tracks the live mode) — the why-line
    # alone implies Experimental; grey in shadow where the decision doesn't actuate.
    reason = st.get("reason", "")
    if st.get("mode") == "experimental" and reason and reason not in ("chill", ""):
      out.append((f"why {reason}", _C.GREY if st.get("shadow") else _C.WHITE, self.font, None))

    # next binding map curve — only when it's actually near (within _NEXT_CURVE_S at current speed)
    mapv = _f(st.get("mapV"))
    mapd = _f(st.get("mapDist"))
    if 0.0 < mapv < _REAL_CURVE_MS and mapd > 0.0:
      try:
        v_now = max(abs(float(ui_state.sm["carState"].vEgo)), 1.0)
      except Exception:
        v_now = 1.0
      if mapd / v_now < _NEXT_CURVE_S:
        out.append((f"next {round(mapv * conv)} {round(mapd)}m", _C.ORANGE, self.font, None))

    pct = max(0, min(100, _i(st.get("curvePct", 0))))
    if pct >= _CURVE_SHOW_PCT:
      src = st.get("curveSrc", "") or "--"
      pct_col = _C.ORANGE if pct < 100 else _C.RED
      out.append((f"curve {pct}% {src}", pct_col, self.font, None))

    # a non-green health dot expands into its full text line so a problem is spelled out in words
    pts = _i(st.get("mapPts", 0))
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

  # ---- standstill view: the grouped cockpit card (dumpui2pnw) ------------------
  # The card is a list of tagged ROWS, laid out by _render_card:
  #   ("full", text, color, font, dots|None)   full-width left-aligned line (+ optional right dots)
  #   ("sep",)                                  a thin group divider
  #   ("kv", left_cell, right_cell)             two label:value cells; cell = (label,value,color)|None
  #   ("tags", [(text,color), ...])             a row of short active-only chips
  def _grid_cells(self):
    """The fixed 3x2 grid, ALWAYS six cells (idle -> '--'), so the card holds one stable shape.
    Left column = mapd/OSM group (map / DB / gps); right = control group (vtsc / long / curve)."""
    st = self._st
    conv = self._conv
    # --- left: map points, OSM DB, gps fix ---
    pts = _i(st.get("mapPts", 0))
    mdl = self._mapdl
    # "warm" = DB present/coming but no live forward path yet (fresh-ignition warmup) — distinct from a
    # RED "0" (genuinely no map curve capability), so a downloaded DB isn't misread as curve-braking armed.
    if pts > 0:
      map_cell = ("map", f"{pts}p", _C.GREEN)
    elif mdl.startswith(("downloading", "incomplete")) or mdl == "OK":
      map_cell = ("map", "warm", _C.ORANGE)
    else:
      map_cell = ("map", "0", _C.RED)
    if not mdl:
      db_cell = ("DB", "--", _C.GREY)
    else:
      col = _C.GREEN if mdl == "OK" else (_C.ORANGE if mdl.startswith("downloading") else _C.RED)
      disp = mdl.replace("downloading", "dl")
      if len(disp) > 7:
        disp = disp[:6] + "…"                 # visible ellipsis, never a silent mid-word chop
      db_cell = ("DB", disp, col)
    gps = bool(st.get("gps", False))
    gps_cell = ("gps", "ok" if gps else "no", _C.GREEN if gps else _C.ORANGE)
    # --- right: VTSC, longitudinal authority, curve closeness ---
    vt = self._vtsc
    if vt.get("enabled") and vt.get("engaged"):
      vtsc_cell = ("vtsc", f"slow {round(_f(vt.get('cap')) * conv)}", _C.ORANGE)
    elif vt.get("enabled"):
      vtsc_cell = ("vtsc", "ready", _C.GREY)
    else:
      vtsc_cell = ("vtsc", "--", _C.GREY)
    long_cell = self._long_cell()
    pct = max(0, min(100, _i(st.get("curvePct", 0))))
    src = st.get("curveSrc", "") or "--"
    pct_col = _C.GREEN if pct < 60 else (_C.ORANGE if pct < 100 else _C.RED)
    curve_cell = ("curve", f"{pct}% {src}", pct_col)
    return [(map_cell, vtsc_cell), (db_cell, long_cell), (gps_cell, curve_cell)]

  def _long_cell(self):
    """Grid 'long' cell = longitudinal authority state. Shadow -> the ICBM stock-ACC button manager
    (target vs set); non-shadow -> openpilot longitudinal 'op' when VTSC (the CES-side long) is live."""
    st = self._st
    conv = self._conv
    if st.get("shadow"):
      it = st.get("icbmT")
      if it is not None:
        t_mph = round(_f(it) * conv)
        s_mph = round(_f(st.get("icbmSet")) * conv)
        if st.get("icbmDir") == "inc":
          return ("long", f"{s_mph}>{t_mph}", _C.GREEN)   # icbmrestore2pnw: restoring driver's set
        if s_mph > t_mph:
          return ("long", f"{s_mph}>{t_mph}", _C.ORANGE)  # capping the set down for a curve
        return ("long", f"@{t_mph}", _C.GREY)
      return ("long", "ready" if st.get("icbmOn") else "no-ACC",
              _C.GREEN if st.get("icbmOn") else _C.GREY)
    return ("long", "op" if self._vtsc.get("enabled") else "--",
            _C.GREEN if self._vtsc.get("enabled") else _C.GREY)

  def _build_card(self):
    """Assemble the grouped standstill card + measure its box. Returns
    (rows, box_w, box_h, l_label_w, l_col_w, r_label_w, r_col_w) — per-column label/column widths so
    the render can place the narrow left column and the wide right column independently — or None."""
    st = self._st
    conv = self._conv
    fs = _FS_SM
    rows: list[tuple] = []

    # (1) alarms — red/orange, on top, never grouped away (same three as the moving view). Measured
    # here (dumpnarrow2pnw option b) so an APPEARING alarm may widen the card rather than clip: this
    # is a max over the FIXED alarm strings actually present this poll — never a live grid/situation
    # value — so the everything-OK width never jitters. (The only alarm carrying a live number,
    # "CALIBRATING NN%", is far narrower than the grid so it can't drive the box width anyway.)
    alarm_w = 0.0
    for line in (self._mismatch_line(), self._steer_line(), self._calib_line(), self._upload_err_line()):
      if line:
        rows.append(("full", line[0], line[1], line[2], None))
        alarm_w = max(alarm_w, measure_text_cached(line[2], line[0], fs).x)

    # (2) headline: button identity + the 3 health dots (map / gps / long)
    button = _i(st.get("button", 0))
    btn = {0: "CES AUTO", 1: "CES CHILL*", 2: "CES EXP*"}.get(button, "CES AUTO")
    rows.append(("full", btn, _C.WHITE, self.font_bold, self._health_dots()))

    # (3) effective mode — shadow-aware (orange EXPERIMENTAL is reserved for real actuation; in
    # shadow nothing actuates so it reads grey; the top-right icon can't show the shadow decision,
    # which is exactly why the card keeps this line).
    is_exp = st.get("mode") == "experimental"
    if st.get("shadow"):
      rows.append(("full", ">> SHADOW EXP" if is_exp else ">> SHADOW CHILL", _C.GREY, self.font_bold, None))
    else:
      rows.append(("full", ">> EXPERIMENTAL" if is_exp else ">> CHILL",
                   _C.ORANGE if is_exp else _C.GREY, self.font_bold, None))

    # (4) why <reason> — the binding trigger / hold (stopHold, standstillHold, slowLead, ...). Shown
    # whenever meaningful, in either mode, so the standstill holds are visible right where you debug.
    reason = st.get("reason", "")
    if reason and reason not in ("chill", ""):
      rows.append(("full", f"why {reason}", _C.WHITE, self.font, None))

    # (5) the fixed 3x2 diagnostics grid
    rows.append(("sep",))
    cells = self._grid_cells()
    for left, right in cells:
      rows.append(("kv", left, right))

    # (6) situation strip — next binding map curve -> else the tracked lead gap -> else road clear.
    # ces-i90-2pnw: only "road clear" when there is NEITHER an upcoming curve NOR a tracked lead.
    rows.append(("sep",))
    mapv = _f(st.get("mapV"))
    mapd = _f(st.get("mapDist"))
    drel = _f(st.get("dRel"))
    pts = _i(st.get("mapPts", 0))
    gps = bool(st.get("gps", False))
    # ALWAYS emit exactly one situation line (a "—" fallback), so the common stopped-behind-a-lead
    # case (lead <-> clear, dRel ticking) never adds/removes a row — the card height stays put.
    if 0.0 < mapv < _REAL_CURVE_MS and mapd > 0.0:
      rows.append(("full", f"next {round(mapv * conv)} {round(mapd)}m", _C.ORANGE, self.font, None))
    elif drel > 0.0:
      rows.append(("full", f"lead {round(drel)}m", _C.GREY, self.font, None))
    elif pts > 0 and gps:
      rows.append(("full", "road clear", _C.GREEN, self.font, None))
    else:
      rows.append(("full", "—", _C.GREY, self.font, None))

    if not rows:
      return None

    # --- FIXED metrics (computed once from exemplars — never from live text — so the card width is
    # stable across the 5 Hz rebuilds; live numbers are left-aligned inside the fixed box). Width is
    # driven by the grid + the widest normal full line only; the chip row wraps to fit it (below). The
    # two grid columns are sized INDEPENDENTLY from their own side's exemplars (dumpnarrow2pnw) so the
    # short left column (map/DB/gps) no longer inherits the wide right column's width and leave a gap. ---
    if self._card_metrics is None:
      l_label_w = max(measure_text_cached(self.font, s, fs).x for s in _GRID_L_LABELS)
      l_val_w = max(measure_text_cached(self.font, s, fs).x for s in _GRID_L_VALUES)
      l_col_w = l_label_w + _LBL_GAP + l_val_w
      r_label_w = max(measure_text_cached(self.font, s, fs).x for s in _GRID_R_LABELS)
      r_val_w = max(measure_text_cached(self.font, s, fs).x for s in _GRID_R_VALUES)
      r_col_w = r_label_w + _LBL_GAP + r_val_w
      grid_w = l_col_w + _COL_GAP + r_col_w
      line_w = max(measure_text_cached(self.font_bold, s, fs).x for s in _CARD_LINE_EX)
      line_w = max(line_w, measure_text_cached(self.font_bold, "CES AUTO", fs).x + _dots_w(fs, 3))
      self._card_metrics = (max(grid_w, line_w), l_label_w, l_col_w, r_label_w, r_col_w)
    box_w_inner, l_label_w, l_col_w, r_label_w, r_col_w = self._card_metrics
    # option b: an appearing RED alarm (rare, big event) may widen the resting box so it never clips.
    box_w_inner = max(box_w_inner, alarm_w)

    # (7) active-only tag chips — rain / accel-zone / hwy-gate + no-lowSpd / 4-signal-live. One
    # compact chip row instead of four separate lines (the crudeness the driver flagged); it WRAPS
    # to the fixed card width so a rare all-flags-on stop adds a chip row rather than widening.
    chips: list[tuple] = []
    rain = self._rain_line()
    if rain:
      chips.append((rain[0], rain[1]))                 # "RAIN -3", grey
    if st.get("accelZone"):
      chips.append(("accel-zone", _C.GREEN))
    if st.get("hwyGate"):
      chips.append(("hwy-gate", _C.GREEN))
      chips.append(("no-lowSpd", _C.GREY))             # the old dump's "(no lowSpd)" note, preserved
    try:
      if str(self._fordlat.get("mode") or "") == "4sig" and (time.time() - float(self._fordlat.get("ts"))) <= _FORDLAT_STALE_S:  # noqa: TID251
        chips.append(("STEER 4-SIG", _C.GREEN))        # dump-only green confirm the 4-sig path is live
    except (TypeError, ValueError):
      pass
    line, used = [], 0.0
    for chip in chips:
      w = measure_text_cached(self.font, chip[0], fs).x + _COL_GAP
      if line and used + w > box_w_inner:              # would overflow the fixed width -> wrap
        rows.append(("tags", line))
        line, used = [], 0.0
      line.append(chip)
      used += w
    if line:
      rows.append(("tags", line))

    box_w = box_w_inner + _PAD * 2
    box_h = _PAD * 2
    for tag, *_rest in rows:
      box_h += _SEP_H if tag == "sep" else _LINE_H_SM
    return (rows, box_w, box_h, l_label_w, l_col_w, r_label_w, r_col_w)

  # ---- render --------------------------------------------------------------
  def _render(self, rect: rl.Rectangle):
    # visibility gates (enabled, CES-auto button mode only, NO-SIGNAL alarm) are applied at poll
    # time in _update_state; both layouts are None whenever the overlay should be hidden.
    # dumpui2pnw: the standstill grouped card has its own layout + renderer (takes precedence).
    if self._card_layout is not None:
      self._render_card(rect)
      return
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

  def _render_card(self, rect: rl.Rectangle):
    """dumpui2pnw: render the grouped standstill card. Content is LEFT-aligned (instrument-panel
    feel); the box stays anchored bottom-right. Full rows draw their optional health dots against
    the right edge; kv rows draw two aligned label:value cells; sep rows draw a dim divider."""
    layout = self._card_layout
    if layout is None:
      return
    rows, box_w, box_h, l_label_w, l_col_w, r_label_w, r_col_w = layout
    fs = _FS_SM
    bx = rect.x + rect.width - box_w - _MARGIN
    by = rect.y + rect.height - box_h - _MARGIN
    rl.draw_rectangle_rounded(rl.Rectangle(bx, by, box_w, box_h), 0.10, 8, _C.BG)
    x0 = bx + _PAD
    right = bx + box_w - _PAD
    y = by + _PAD
    for tag, *rest in rows:
      if tag == "sep":
        ly = int(y + _SEP_H * 0.5)
        rl.draw_line_ex(rl.Vector2(x0, ly), rl.Vector2(right, ly), 2.0, _C.SEP)
        y += _SEP_H
        continue
      if tag == "full":
        text, color, font, dots = rest
        rl.draw_text_ex(font, text, rl.Vector2(x0, y), fs, 0, color)
        if dots:
          tw = measure_text_cached(font, text, fs).x
          r = _DOT_R
          # dots hug the right edge so the identity line reads "CES AUTO ........ ● ● ●"
          n = len(dots)
          strip_w = n * 2 * r + (n - 1) * fs * 0.22
          cx = max(x0 + tw + fs * 0.4 + r, right - strip_w + r)
          cy = y + fs * 0.55
          for dc in dots:
            rl.draw_circle(int(cx), int(cy), r, dc)
            cx += 2 * r + fs * 0.22
      elif tag == "kv":
        # dumpnarrow2pnw: left cell at x0 (narrow left label width); right cell placed just past the
        # NARROW left column (x0 + l_col_w + _COL_GAP) with its own label width — closing the old gap.
        left, rightc = rest
        for cell, cx, clab_w in ((left, x0, l_label_w),
                                 (rightc, x0 + l_col_w + _COL_GAP, r_label_w)):
          if not cell:
            continue
          rl.draw_text_ex(self.font, cell[0], rl.Vector2(cx, y), fs, 0, _C.GREY)      # label
          rl.draw_text_ex(self.font, cell[1], rl.Vector2(cx + clab_w + _LBL_GAP, y), fs, 0, cell[2])  # value
      elif tag == "tags":
        chips = rest[0]
        cx = x0
        for text, color in chips:
          rl.draw_text_ex(self.font, text, rl.Vector2(cx, y), fs, 0, color)
          cx += measure_text_cached(self.font, text, fs).x + _COL_GAP
      y += _LINE_H_SM
