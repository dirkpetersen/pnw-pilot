"""
everdrive2pnw: EverDrive auxiliary-charger input power + range — LOWER-RIGHT, directly BELOW the CES box.

Display-only. One line, right-aligned and anchored to the RIGHT EDGE exactly the way ces_status.py
anchors its box, so — like the CES box — it must never grow into the green driving path drawn down
the middle of the screen. That is the driver's hard layout constraint, inherited verbatim from the CES
box docstring; the width is fixed from exemplars (below) precisely so it cannot creep toward centre as
digits change.

Driver-approved formats (2026-09-19):
    charging, moving:    ED: 1.4 kW   112 -> 117 mi
    charging, stopped:   ED: 1.4 kW   112 mi  +2.7 mi/h
    not charging:        ED: --       112 mi

`->` is ASCII on purpose. The device font atlas is built by selfdrive/assets/fonts/process.py from
`chr(32..126)` plus a short EXTRA_CHARS list that does NOT contain U+2192 "→" — a unicode arrow would
render as tofu. ces_status.py makes the same choice (it writes "ICBM 75>62", never "75→62").

Data path: an in-memory param `EverDriveStatus` (/dev/shm/params) published at ~5 Hz by the producer
(the opendbc half, a separate repo). This widget never computes anything about the car and never
touches control or safety — it only formats numbers someone else measured.

INERT WHEN THERE IS NO EVERDRIVE (the Tesla, or a Lightning with the module not fitted): the producer
publishes NOTHING until the EverDrive CAN message has actually been received, so `EverDriveStatus` is
simply ABSENT. That is the NORMAL steady state, not an error — it must never log, never alarm, and
must cost nothing: the poll backs off to _ABSENT_REFRESH_S, _update_state returns before any string
formatting or text measurement, the exemplar measurement is deferred until the box is first actually
shown, and _render is a single None check. The back-off never becomes permanent, because the driver
can plug the charger in mid-drive.
"""
import time
import pyray as rl

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

_REFRESH_S = 0.2          # poll cadence while EverDrive data IS flowing (matches the ~5 Hz publisher)
_ABSENT_REFRESH_S = 2.0   # everdrive2pnw: no EverDriveStatus key at all (Tesla / no module fitted) or
                          #   the driver disabled the box -> re-check this slowly instead of 5x/s.
                          #   NOT a permanent give-up: the charger can be plugged in mid-drive, so the
                          #   slow poll keeps running for the whole session and the box appears within
                          #   ~2 s of the first published payload.
_STALE_S = 5.0            # s: same dead-man value/idiom as ces_status.py — an EverDriveStatus `ts`
                          #   older than this means the producer went silent, so HIDE the box rather
                          #   than keep painting plausible-looking dead numbers. (ces_status pairs
                          #   _STALE_S with a _GRACE_S onroad-dwell timer because its stale path raises
                          #   a RED alarm that must not false-fire during spin-up. This box has no
                          #   alarm — its stale action is "hide", which is also its state before the
                          #   first publish — so a grace timer would change nothing here.)
_FS = 40                  # font size. Chosen as the largest size at which the WIDEST exemplar box
                          #   (text + 2*_PAD) still fits the driver's ~700 px budget: 649.3 px at 40 vs
                          #   709.5 px at 44 (measured against the real Inter-Medium .fnt metrics at the
                          #   device's FONT_SCALE=1.16). Keeping the roomy driver-approved format at a
                          #   slightly smaller size beat falling back to the compact form at a larger one.
_LINE_H = 50              # _FS * 1.25, the same line-height ratio ces_status.py uses
_PAD = 24                 # == ces_status._PAD, so the two stacked boxes have identical inner padding
_MARGIN = 40              # == ces_status._MARGIN: same gap from the screen's right / bottom edges
_STACK_GAP = 12           # vertical gap between this box and the CES box sitting on top of it
_BOX_H = _LINE_H + _PAD * 2   # always exactly one line — the height never varies

# --- maths (see the everdrive2pnw spec; implemented literally) --------------------------------------
# The spec's own conversion literals are used verbatim rather than common.constants.CV (CV.MPH_TO_KPH
# is 1.609344 — a 2.5 ppm difference, invisible at the 3 significant digits this box prints).
_KM_PER_MI = 1.60934
_MS_TO_MPH = 2.23694
_AC_ZERO_KW = 0.05        # at/below this the charger is unplugged: a REAL measured zero -> "not charging"
_PROJ_HEADROOM = 1.05     # the projection is only credible while P_assumed > acKw * this. Below it the
                          #   car is crawling and rangeMi * P/(P-acKw) diverges. The low-speed cutoff is
                          #   DERIVED from this guard, deliberately not a separate magic mph threshold.
_PROJ_MAX_RATIO = 2.0     # hard sanity clamp on the projection, as a multiple of the printed range.
                          #   Protects against (a) a glitched effWhKm/vMs squeezing the denominator
                          #   toward zero just inside the _PROJ_HEADROOM guard, which would print a
                          #   wildly optimistic range, and (b) a 4-digit number overflowing the
                          #   fixed-width box and clipping. An auxiliary charger doubling the truck's
                          #   range is already beyond anything physical here, so hitting this clamp
                          #   means the inputs are wrong — and a projection we had to clamp is not
                          #   worth printing, so we fall back to the stopped form (kW + range + gain
                          #   rate), every term of which is still true. The change of form is the
                          #   visible failure signal (Rule 2), not a silently bent number.

# STABLE WIDTH: measured ONCE from these fixed worst-case strings with the real font — never from live
# values — so the box cannot dance left/right as digits change (the same discipline ces_status.py's
# standstill card uses). Worst case is 2 integer digits of kW, 3 of miles, 2 of mi/h.
_EXEMPLARS = (
  "ED: 00.0 kW   000 -> 000 mi",        # charging + moving  (projection)
  "ED: 00.0 kW   000 mi  +00.0 mi/h",   # charging + stopped (gain rate)  <- widest
  "ED: --       000 mi",                # not charging
)


class _C:
  WHITE = rl.Color(255, 255, 255, 235)
  BG = rl.Color(0, 0, 0, 140)


def _f(v, d: float = 0.0) -> float:
  """EverDriveStatus is an arbitrary dict off /dev/shm — a key holding an explicit None sails straight
  through dict.get(k, default), and float(None) would crash-loop the UI (this fork's documented brick
  scenario, ces_status.py:138). Every numeric read goes through this."""
  try:
    return float(v)
  except (TypeError, ValueError):
    return d


class EverDriveStatusRenderer(Widget):
  def __init__(self):
    super().__init__()
    try:
      self._mem = Params("/dev/shm/params")
    except Exception:
      # Not the no-EverDrive case (that is a missing KEY, handled quietly below) — this is the mem
      # param store itself being unreachable, which is a real fault. Log it once; never raise.
      cloudlog.exception("everdrive2pnw: /dev/shm/params unavailable, EverDrive box disabled")
      self._mem = None
    self._last_poll = 0.0
    self._poll_interval = _REFRESH_S
    self._box_w: float | None = None      # exemplar width, measured LAZILY on first actual display
    self._cached_layout: tuple[str, float, float] | None = None   # (text, box_w, text_w); None == hidden
    # Log each DISTINCT read fault once (not 5x/s), with a flag per fault: sharing one flag would let
    # the second, different failure go completely unlogged.
    self._toggle_err_logged = False
    self._status_err_logged = False
    self.font = gui_app.font(FontWeight.MEDIUM)

  @property
  def stack_height(self) -> float:
    """everdrive2pnw: how far augmented_road_view must lift the CES box so the two stack. Exactly 0.0
    while this box is hidden, so the CES box renders where it does today."""
    return (_BOX_H + _STACK_GAP) if self._cached_layout is not None else 0.0

  # ---- state ---------------------------------------------------------------
  def _update_state(self):
    now = time.monotonic()
    if now - self._last_poll < self._poll_interval:
      return
    self._last_poll = now
    # Hidden until proven otherwise, and slow-polling until a payload actually shows up. Every early
    # return below therefore costs one param read and nothing else — no formatting, no measurement.
    self._cached_layout = None
    self._poll_interval = _ABSENT_REFRESH_S
    if self._mem is None:
      return
    # DisableEverDrive is PERSISTENT, so it MUST be read from the NORMAL params store (ui_state.params),
    # never through the /dev/shm handle above — a PERSISTENT key read from the mem store returns False
    # forever, which would silently nail the toggle to "enabled". Defensive: a params/UI mismatch falls
    # back to SHOWING (the documented default state) and can never crash the UI.
    try:
      if ui_state.params.get_bool("DisableEverDrive"):
        return
    except Exception:
      if not self._toggle_err_logged:
        self._toggle_err_logged = True
        cloudlog.exception("everdrive2pnw: DisableEverDrive unreadable, defaulting to SHOW")
    try:
      st = self._mem.get("EverDriveStatus", return_default=True)
    except Exception:
      # An absent key does NOT come through here (Params returns the default, i.e. None). This is a
      # genuine fault — in practice an UnknownKeyName from a params_keys.h / params_pyx.so mismatch,
      # which would otherwise leave the box permanently and invisibly dead.
      if not self._status_err_logged:
        self._status_err_logged = True
        cloudlog.exception("everdrive2pnw: EverDriveStatus read failed, box hidden")
      return
    if not isinstance(st, dict):
      return          # key absent: NORMAL on the Tesla / a Lightning with no EverDrive. Stay quiet.
    self._poll_interval = _REFRESH_S   # a payload exists -> track the publisher at ~5 Hz from here on
    text = self._build_text(st)
    if text is None:
      return          # producer went silent (stale ts) -> hide; keep polling fast so it can come back
    if self._box_w is None:
      # First time the box is actually shown. Measuring here (not in __init__, not per poll) is what
      # keeps a device with no EverDrive from ever paying for the text measurement at all.
      self._box_w = max(measure_text_cached(self.font, t, _FS).x for t in _EXEMPLARS) + _PAD * 2
    # The live line's own width is measured HERE (5 Hz) and cached, so _render does nothing but draw.
    self._cached_layout = (text, self._box_w, measure_text_cached(self.font, text, _FS).x)

  def _build_text(self, st: dict) -> str | None:
    """The one line, or None to hide. Every guard below fails to a VISIBLY different form — never to a
    plausible-looking number."""
    ts = _f(st.get("ts"), -1.0)
    # TID251 is suppressed below for the same reason ces_status.py suppresses it: this is a wall-vs-wall
    # comparison against the producer's wall-clock `ts` heartbeat, so time.monotonic would be wrong here.
    if ts <= 0.0 or (time.time() - ts) > _STALE_S:  # noqa: TID251
      return None

    ac_kw = _f(st.get("acKw"))
    range_mi = _f(st.get("rangeKm")) / _KM_PER_MI
    eff_wh_km = _f(st.get("effWhKm"))
    eff_wh_mi = eff_wh_km * _KM_PER_MI
    rng = f"{range_mi:.0f}" if range_mi > 0.0 else "--"   # no dash range signal is "--", never "0 mi"

    # acSeen False == the EverDrive CAN message has not been received, so acKw is a pre-filled 0.0 and
    # NOT a measurement. acKw <= _AC_ZERO_KW is the opposite case: a real measured zero (unplugged).
    # Both read "not charging" to the driver, and neither may ever print a manufactured "0.0 kW".
    if not st.get("acSeen") or ac_kw <= _AC_ZERO_KW:
      return f"ED: --       {rng} mi"

    kw = f"ED: {ac_kw:.1f} kW"
    # effOk False == effWhKm is sitting at its -100 encoding floor, i.e. the truck is not telling us its
    # reference efficiency. No projection and no gain rate then, and NO substituted default efficiency:
    # a made-up constant would produce a confident number out of a signal we do not have.
    if not st.get("effOk") or eff_wh_km <= 0.0 or range_mi <= 0.0:
      return f"{kw}   {rng} mi"

    # The projection deliberately uses the truck's OWN efficiency constant rather than a measured
    # consumption, so the projected figure stays internally consistent with the range printed beside it.
    p_assumed = eff_wh_mi * (_f(st.get("vMs")) * _MS_TO_MPH) / 1000.0   # kW the truck's range model assumes
    if p_assumed > ac_kw * _PROJ_HEADROOM:
      proj = range_mi * p_assumed / (p_assumed - ac_kw)
      if proj <= range_mi * _PROJ_MAX_RATIO:
        return f"{kw}   {range_mi:.0f} -> {proj:.0f} mi"
    # Crawling (the projection diverges) or the clamp tripped -> the stopped form. gainMiPerH is
    # EverDrive's CONTRIBUTION to range, not the rate the pack's state of charge rises: measured on this
    # truck the truck's own awake load ate about 1.0 kW of a 1.36 kW input while parked, so SoC climbed
    # far more slowly than this figure. The contribution is still the right number to show a driver,
    # because that awake load would be drawn anyway.
    return f"{kw}   {range_mi:.0f} mi  +{ac_kw / (eff_wh_mi / 1000.0):.1f} mi/h"

  # ---- render --------------------------------------------------------------
  def _render(self, rect: rl.Rectangle):
    if self._cached_layout is None:
      return
    text, box_w, text_w = self._cached_layout
    # Anchored to the RIGHT EDGE and the bottom, exactly as ces_status.py does — this is what keeps the
    # box out of the green driving path down the middle of the screen. The text is right-aligned inside
    # the fixed-width box (also as ces_status.py does), so "mi" stays put as the digits change.
    bx = rect.x + rect.width - box_w - _MARGIN
    by = rect.y + rect.height - _BOX_H - _MARGIN
    rl.draw_rectangle_rounded(rl.Rectangle(bx, by, box_w, _BOX_H), 0.12, 8, _C.BG)
    rl.draw_text_ex(self.font, text, rl.Vector2(bx + box_w - _PAD - text_w, by + _PAD), _FS, 0, _C.WHITE)
