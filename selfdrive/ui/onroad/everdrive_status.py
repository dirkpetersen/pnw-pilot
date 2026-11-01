"""
everdrive2pnw: EverDrive auxiliary-charger input power + range — LOWER-RIGHT, directly BELOW the CES box.

Display-only. One line, right-aligned and anchored to the RIGHT EDGE exactly the way ces_status.py
anchors its box, so — like the CES box — it must never grow into the green driving path drawn down
the middle of the screen. That is the driver's hard layout constraint, inherited verbatim from the CES
box docstring. (This sentence used to claim the width was "fixed from exemplars so it cannot creep
toward centre"; that stopped being true when the box started hugging its text on 2026-09-20. What
actually holds the constraint now is that the box is RIGHT-anchored and every term is digit-bounded
-- see the worst-case table at `_FS`.)

Driver-approved formats. MOVING (producer `moving`, i.e. >= 10 mph; everdriveinstant2pnw, owner
2026-11-01). ALWAYS these shapes, never another while driving; no "@" and no "m/h" anywhere:
    charger off:         132m(55.35kwh),1mi:128m
    charger on:          10.8kw,220m,1mi:135m
    no live 0x441:       10.8kw,?220m,1mi:?218m      ("?" = built from the speed estimate)
    charge >= load:      10.8kw,>999m,1mi:>999m
STOPPED, PARKED or below 10 mph (2026-09-20 revision, unchanged):
    charging, crawling:  1.4kw,100m(55.35kwh)->104m
    charging, stopped:   1.4kw,100m(55.35kwh),+2.7m/h
    not charging:        --,100m(55.35kwh)
    effOk False:         1.4kw,100m(55.35kwh)
    no capKwh/socPct:    the (kwh) parenthetical is OMITTED, never shown as 0.000

THE MOVING TERMS. P is the live measured load pNowKw (k x motor-2 power + accessories, see
everdrive_pnw.py), so the terms show at the first moving publish. With no live 0x441 P is the SPEED
ESTIMATE effWhKm x km/h / 1000 + _P_ACC_KW (the truck's reference efficiency) and every figure built
from it gets a leading "?"; it drops as soon as pNowKw is live again. With no efficiency either, the
first term is the dash range, still "?".
    term 1, charger off  energyKwh / P x mph. A measured P <= 0 (regen, coasting) uses the speed
                         estimate, with "?". Otherwise not usable (no energyKwh, over _RANGE_MAX_MI):
                         the truck's own dash range.
    term 1, charger on   the DASH range scaled by the generator, dash x P / (P - acKw), e.g. dash
                         141 mi, 30 kW, 10.8 kW in -> "220m"; needs no capacity. ">999m" when the
                         measured P <= acKw or over 999; "?>999m" when the ESTIMATE is <= acKw.
                         ">999m" is never printed from a missing reading without "?".
    "1mi:"               the last-mile range with the charge, energyKwh / (mileKwh - mileAcKwh).
                         No "?" only when the whole mile had live pNowKw (producer mileEst False).
                         Before the first full mile it repeats term 1 (with its "?"). Charger on:
                         ">999m" when the mile netted out as charging or regen, or over 999; not
                         computable repeats term 1. Charger off: "1mi:--" when not usable, ">999m"
                         only when the mile netted out as charging after the charger was switched off.
    "--m"                only when the dash range itself is gone and nothing else can stand in.
                         "?" is never put on "--m" or "1mi:--": there is no figure to qualify.
The change of FORM is the signal, the same idiom the box already uses for "--" (no charger). There is
deliberately no legend and no second line.

The kWh is socPct x capKwh, where capKwh is DERIVED by the producer from the truck's own
RngPerChrgAvg x VehElEffAvg -- no capacity is hardcoded anywhere. See everdrive_pnw.py.

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
import math
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
# Font size. MEASURED against the real Inter-Medium.fnt on the device (atlas base 200) at the
# device's FONT_SCALE=1.16 -- raylib scales glyph advances by fontSize/baseSize, so width is linear
# in _FS and these numbers are exact, not estimates.
#
# HISTORY: 48 -> 56 on 2026-09-20 to match the location box, then BACK TO 48 the same day when the
# driver asked for the pack kWh inline. The kWh parenthetical is ~50% more text, and at 56 the box
# crosses screen centre -- into the green driving path this box must never enter. 48 -> 49 on
# 2026-10-31, then 49 -> 52 on 2026-11-01 (owner, everdriveinstant2pnw), measured in the table below.
#
# ⚠️ THE NUMBERS THAT USED TO BE HERE WERE STALE BY 103 px (corrected 2026-09-20, everdrive2pnw).
# They read "widest exemplar 925.4 px text -> 973.4 px box -> +36.6 px clear of centre". 925.4 px is
# `ED:00.0kw,000m(000.000kwh),+00.0m/h` -- i.e. the format BEFORE the "ED:" prefix was dropped and
# BEFORE the kWh term went from 3 decimals to 2, two changes that shipped after the measurement was
# taken and were never folded back in. Nothing was wrong on the car; the comment was simply
# describing a line the box no longer prints, and it understated the clearance by a factor of four.
#
# MEASURED AGAINST THE DEVICE'S OWN Inter-Medium.fnt (atlas base 200) at FONT_SCALE 1.16 -- raylib
# scales glyph advances by fontSize/baseSize, so width is linear in _FS and these are exact. The
# model is cross-checked against the two numbers in EVERDRIVE2PNW.md §0.2, which it reproduces to
# 0.1 px ('ED:00.0kw,000mi,+00.0mi/h' = 633.1 px at 48 and 738.6 px at 56).
#
# DIGITS ARE PROPORTIONAL IN INTER: '4' is the widest at 108 atlas units ('0' is 106, '9' 105), so an
# all-4s line is the true upper bound for any shape. Content right edge is 2130, _MARGIN 40, screen
# centre 1080. Every form the formatter can emit, at its maximum digit count, at _FS 52
# (re-measured 2026-11-01 for the "?" moving forms; "@" is gone from every form):
#
#     44.4kw,444m(444.44kwh),+44.4m/h    899.4 px text -> 947.4 box -> left 1142.6 -> +62.6 clear
#     44.4kw,444m(444.44kwh)->444m       846.3              894.3           1195.7      +115.7
#     ?>999m(444.44kwh),1mi:?>999m       816.7              864.7           1225.3      +145.3
#     ?444m(444.44kwh),1mi:?444m         755.8              803.8           1286.2      +206.2
#     44.4kw,?>999m,1mi:?>999m           698.5              746.5           1343.5      +263.5
#     44.4kw,?444m,1mi:?444m             637.6              685.6           1404.4      +324.4
#     44.4kw,444m(444.44kwh)             648.4              696.4           1393.6      +313.6
#     --,444m(444.44kwh)                 514.2              562.2           1527.8      +447.8
#
# WORST CASE 947.4 px, +62.6 px clear of the driving path, under the 1000 px _MAX_BOX_W tripwire.
# (At 49 the same worst case was 895.5 px. 53 would still fit, 964.7 px / +45.3; the largest _FS
# under the tripwire is 55, 999.3 px; the tripwire binds before screen centre does.)
#
# The ranges stay at THREE digits because the producer bands the truck's own range to 254 mi and
# _RANGE_MAX_MI bands the computed one to 499 mi, which also bounds the projection (<= 2x) to 998.
# _GAIN_MAX_MI_H bounds the rate to 99.9. Adding a fourth digit anywhere costs ~30 px.
_FS = 52
_LINE_H = 65              # _FS * 1.25, the ratio ces_status.py and location_services_status.py both use.
                          #   At 52 the text is 60.3 px high (52 x FONT_SCALE 1.16), over the old 60.
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
_GAIN_MAX_MI_H = 100.0    # the stopped form's mi/h term may use at most TWO integer digits, because
                          #   that is what the fixed-width exemplar reserves. At or above this the term
                          #   is DROPPED rather than printed clipped — see the comment at its use site.
_PROJ_HEADROOM = 1.05     # the projection is only credible while P_assumed > acKw * this. Below it the
                          #   car is crawling and rangeMi * P/(P-acKw) diverges.
                          #   NOTE (corrected 2026-09-19): this is the WEAKER of the two projection
                          #   guards and is not what actually sets the low-speed cutoff — _PROJ_MAX_RATIO
                          #   binds first, since proj <= 2*range requires P_assumed >= 2*acKw, not
                          #   1.05*acKw. Both land on the stopped form, so the driver sees the right
                          #   thing; this constant is the divergence backstop, not the cutoff.
_RANGE_MAX_MI = 499.0     # everdrive2pnw (2026-09-20): plausibility ceiling on the COMPUTED range,
                          #   since 2026-10-31 both moving terms (current load and last mile). Two independent constraints land on the
                          #   same number, which is why it is this one:
                          #     PHYSICS - 499 mi needs ~3.96 mi/kWh on a full 126 kWh pack. The truck's
                          #       own unadjusted full-charge estimate is 246.6 mi, EPA-equivalent is
                          #       2.44 mi/kWh, and the best ever MEASURED here was 2.96 mi/kWh
                          #       (pack-side, the 2026-09-19 city drive). Honest driving cannot reach
                          #       it; a long descent, where the window regenerates most of its energy
                          #       back, can -- and that is precisely a consumption figure that has
                          #       stopped predicting anything.
                          #     WIDTH - the box budgets THREE digits per range term, and the
                          #       projection below is up to _PROJ_MAX_RATIO x this: 2 x 499 = 998.
                          #       At 500 the projection would gain a fourth digit and push the box
                          #       toward the green driving path.
                          #   Over it, fall back to the truck's own range: the fallback form is the
                          #   visible failure signal, the same way the projection's clamp works.
_CHG_RANGE_MAX_MI = 999   # everdriveinstant2pnw (owner 2026-11-01): the moving range terms
                          #   ranges print at most 3 digits; above it ">999m" (see _moving_text)
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

# BOX WIDTH: the background hugs the ACTUAL text. Driver, 2026-09-20 -- the previous fixed-exemplar
# box left 4-5 characters of empty black to the left of the line, because the widest exemplar
# ("00.0kw,000m(000.00kwh),+00.0m/h") is several characters longer than a real line like
# "1.4kw,98m(56.278kwh),+2.7m/h".
#
# This does NOT reintroduce the dancing the exemplars existed to prevent. The box is anchored to the
# RIGHT edge and the text is right-aligned inside it, so the text's right edge sits at a FIXED x and
# the TEXT NEVER MOVES -- only the background's left edge does. Within a form that edge is stable to
# ~2 px anyway (Inter's digits are 106-108 atlas units, near-tabular); it shifts visibly only when the
# FORM changes, which is a real content change the driver should see.
_MAX_BOX_W = 1000.0       # Rule 2 tripwire, NOT a clamp. The driver's hard constraint is that this box
                          #   never enters the green driving path down screen centre: content right
                          #   edge 2130 - _MARGIN 40 - 1000 = 1090, i.e. 10 px clear of centre (1080).
                          #   The producer's bands make it unreachable (kW <= 27.7, range <= 499 mi,
                          #   capacity <= 474.7 kWh, rate <= 99.9), so tripping it means an input is
                          #   out of band -- which must be SAID, not silently drawn over the road.
                          #   Clamping instead would hide exactly the fault worth knowing about.
_CORNER_R = 12.0          # corner radius in PIXELS. raylib's `roundness` is a fraction of the SHORTER
                          #   side, so ces_status's literal 0.12 gives ITS tall box ~10 px but gives
                          #   this short, wide one-line box only ~6.5 px -- which reads as square next
                          #   to the other overlays. Expressed in px and converted at draw time so the
                          #   corners match regardless of how wide the line happens to be.


class _C:
  WHITE = rl.Color(255, 255, 255, 235)
  BG = rl.Color(0, 0, 0, 140)


def _f(v, d: float = 0.0) -> float:
  """EverDriveStatus is an arbitrary dict off /dev/shm — a key holding an explicit None sails straight
  through dict.get(k, default), and float(None) would crash-loop the UI (this fork's documented brick
  scenario, ces_status.py:138). Every numeric read goes through this."""
  try:
    x = float(v)
  except (TypeError, ValueError):
    return d
  return x if math.isfinite(x) else d   # a JSON Infinity / NaN is no reading, like None


def _num(v) -> float | None:
  """A REAL reading or None. Presence checks use this, never `is not None` (review 2026-11-01): a
  NaN / Infinity / non-numeric value is present but is no reading, and _f would turn it into 0.0."""
  x = _f(v, math.nan)
  return None if math.isnan(x) else x


def _miles(r: float) -> str | None:
  """Whole miles as printed, or None. The bound is checked on the ROUNDED value (review 2026-11-01):
  0.5 formats as "0" (round half to even), and a "0m" range is a lie the driver would act on.
  Not finite (a JSON Infinity / NaN) is None too: round() would raise in the render loop."""
  if not math.isfinite(r):
    return None
  m = round(r)
  return str(m) if 1 <= m <= _RANGE_MAX_MI else None


_OVER_RANGE = ">999m"
_P_ACC_KW = 1.2           # MIRRORS everdrive_pnw.P_ACC_KW (opendbc): the accessory load in the "?" speed
                          #   estimate, measured 1.19 kW on 2026-09-19. Change both together.


def _q(text: str, est: bool) -> str:
  """A leading "?" on an estimate. Never on "--m": no figure, nothing to qualify."""
  return f"?{text}" if est and not text.startswith("--") else text


def _range_text(r: float) -> str | None:
  """A computed range WITH the generator as printed: "NNNm" for 1..999 (bound on the ROUNDED value),
  ">999m" above that or if not finite, None if it rounds below 1 (not a range)."""
  if not math.isfinite(r):
    return _OVER_RANGE
  m = round(r)
  if m > _CHG_RANGE_MAX_MI:
    return _OVER_RANGE
  return f"{m}m" if m >= 1 else None


def _gain_text(gain: float) -> str | None:
  """"+N.Nm/h", or None to drop the term. Rounded FIRST, then bounded (Gemini BLOCK 2026-11-01):
  99.95 would print "+100.0" (three integer digits) and -0.0 / 0.04 would print "+-0.0" / "+0.0".
  round(x, 1) passes inf / nan through and both fail the compare, so no finite check is needed."""
  g = round(gain, 1)
  return f"+{g:.1f}m/h" if 0.0 < g < _GAIN_MAX_MI_H else None


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
    self._too_wide_logged = False         # Rule 2 tripwire, fires at most once per session
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
    # Measured HERE (5 Hz) and cached, so _render does nothing but draw. A device with no EverDrive
    # never reaches this line at all, so it never pays for a text measurement.
    text_w = measure_text_cached(self.font, text, _FS).x
    box_w = text_w + _PAD * 2
    # Rule 2: the box must never reach into the green driving path. The producer's bands make this
    # unreachable, so if it ever fires an input is out of band -- say so rather than quietly drawing
    # over the road. Once per session; this is a tripwire, not a clamp (clamping would hide it).
    if box_w > _MAX_BOX_W and not self._too_wide_logged:
      self._too_wide_logged = True
      cloudlog.error("everdrive2pnw: line %.1f px wide (box %.1f) exceeds the %.0f px ceiling -- " +
                     "the box now reaches toward screen centre. An input must be out of band: %r",
                     text_w, box_w, _MAX_BOX_W, text)
    self._cached_layout = (text, box_w, text_w)

  def _build_text(self, st: dict) -> str | None:
    """The one line, or None to hide. Every guard below fails to a VISIBLY different form — never to a
    plausible-looking number."""
    ts = _f(st.get("ts"), -1.0)
    # TID251 is suppressed below for the same reason ces_status.py suppresses it: this is a wall-vs-wall
    # comparison against the producer's wall-clock `ts` heartbeat, so time.monotonic would be wrong here.
    if ts <= 0.0 or (time.time() - ts) > _STALE_S:  # noqa: TID251
      return None

    ac_kw = _f(st.get("acKw"))
    truck_mi = _f(st.get("rangeKm")) / _KM_PER_MI
    eff_wh_km = _f(st.get("effWhKm"))
    eff_wh_mi = eff_wh_km * _KM_PER_MI
    mph = _f(st.get("vMs")) * _MS_TO_MPH

    # Stopped-form first number: the truck's OWN range. (Until 2026-10-31 a measured "@" range from
    # grossKw could land here; the producer only publishes grossKw while moving, and moving now has its
    # own form below, so that path was dead and is removed. grossKw is still published as k's
    # calibration source.)
    truck_txt = _miles(truck_mi)
    range_mi = truck_mi if truck_txt is not None else None
    # today's assumed power: the truck's OWN reference efficiency, so the projection stays
    # internally consistent with the range printed beside it. No substituted default if it is gone.
    p_kw = (eff_wh_mi * mph / 1000.0) if (st.get("effOk") and eff_wh_km > 0.0) else None
    rng = truck_txt if truck_txt is not None else "--"   # never "0 m" from a dead signal

    # everdrive2pnw (driver req 2026-09-20): the pack's energy in kWh, beside the range it buys.
    # capKwh is DERIVED by the producer from the truck's own RngPerChrgAvg x VehElEffAvg, so there is
    # no hardcoded capacity here. Both inputs must be real: if either is missing the parenthetical is
    # OMITTED rather than shown as 0.000 -- Rule 2, the same discipline as every other term.
    #
    # 2 decimals, and that is exactly the resolution the signal carries: SoC is 0.01 %/bit, so one
    # LSB is ~0.013 kWh -- the second decimal is the last digit that means anything. This shipped
    # briefly at 3 decimals (driver's initial format, 2026-09-20) with a note that the third digit
    # was quantisation rather than precision; the driver then asked for 2, which removes the false
    # precision instead of merely documenting it. Do not add digits back.
    cap_kwh = st.get("capKwh")
    soc_pct = st.get("socPct")
    pack = ""
    if _num(cap_kwh) is not None and _num(soc_pct) is not None:
      pack = f"({_f(soc_pct) / 100.0 * _f(cap_kwh):.2f}kwh)"

    # acSeen False == the EverDrive CAN message has not been received, so acKw is a pre-filled 0.0 and
    # NOT a measurement. acKw <= _AC_ZERO_KW is the opposite case: a real measured zero (unplugged).
    # Both read "not charging" to the driver, and neither may ever print a manufactured "0.0 kW".
    charging = bool(st.get("acSeen")) and ac_kw > _AC_ZERO_KW
    if st.get("moving") is True:
      return self._moving_text(st, charging, ac_kw, rng, range_mi, mph, pack,
                               eff_wh_km if st.get("effOk") else 0.0)
    if not charging:
      return f"--,{rng}m{pack}"

    kw = f"{ac_kw:.1f}kw"
    # No range, or no consumption to divide by, means no projection and no gain rate -- and NO
    # substituted default: a made-up constant would produce a confident number out of a signal we do
    # not have. p_kw is None exactly when effOk is False or effWhKm is at its -100 encoding floor.
    if range_mi is None or p_kw is None:
      return f"{kw},{rng}m{pack}"

    if p_kw > ac_kw * _PROJ_HEADROOM:
      proj = range_mi * p_kw / (p_kw - ac_kw)
      if proj <= range_mi * _PROJ_MAX_RATIO:
        return f"{kw},{rng}m{pack}->{proj:.0f}m"
    # Crawling (the projection diverges) or the clamp tripped -> the stopped form. gainMiPerH is
    # EverDrive's CONTRIBUTION to range, not the rate the pack's state of charge rises: measured on this
    # truck the truck's own awake load ate about 1.0 kW of a 1.36 kW input while parked, so SoC climbed
    # far more slowly than this figure. The contribution is still the right number to show a driver,
    # because that awake load would be drawn anyway.
    # The gain rides the truck's reference efficiency, the same basis as the range beside it. p_kw is
    # not None here, so eff_wh_km > 0 and the division is safe.
    gain = ac_kw * 1000.0 / eff_wh_mi
    # everdrive2pnw: the same fixed-width discipline the projection gets from _PROJ_MAX_RATIO. The
    # exemplar budgets TWO integer digits for m/h ("+00.0"); three would overflow the box and CLIP,
    # which is a silent corruption of the whole line, not just of this term. The producer's AC band
    # already bounds this to ~53.8 mi/h, so reaching here means the producer's guard was bypassed or
    # effWhKm is implausibly small -- either way the inputs are wrong. Drop the term rather than print
    # a clipped one: showing LESS is honest, showing a truncated number is not (Rule 2). The change of
    # form is the visible signal, exactly as it is for the projection.
    gain_t = _gain_text(gain)
    if gain_t is None:
      return f"{kw},{rng}m{pack}"
    return f"{kw},{rng}m{pack},{gain_t}"

  @staticmethod
  def _moving_text(st: dict, charging: bool, ac_kw: float, rng: str, range_mi: float | None, mph: float,
                   pack: str, eff_wh_km: float) -> str:
    """everdriveinstant2pnw: the moving form, owner 2026-11-01. See the module docstring. No "@" and
    no "m/h" ever; a leading "?" marks every figure NOT built from the live measured load pNowKw.
    `rng` is the truck's own range ("--" if it is gone); `eff_wh_km` is 0 when effOk is False."""
    e = _f(st.get("energyKwh"))
    ac = ac_kw if charging else 0.0
    p = _num(st.get("pNowKw"))
    # Generator off, measured P <= 0 (regen, coasting): no range at that load, and the bare dash range
    # would look measured, so it takes the speed estimate with "?" like a missing reading.
    est = p is None or (not charging and p <= 0.0)
    if est:
      # The speed estimate: the truck's reference efficiency at this speed + the accessory load.
      p = eff_wh_km / 1000.0 * (mph / _MS_TO_MPH * 3.6) + _P_ACC_KW if eff_wh_km > 0.0 else None
    mile_kwh = _num(st.get("mileKwh"))
    # acSeen gates the AC energy as it gates acKw. NOT `charging`: a charger switched off inside the
    # mile still put real energy in, so with acSeen the whole mile's AC energy is netted.
    mile_ac = _f(st.get("mileAcKwh")) if st.get("acSeen") else 0.0
    # Producer mileEst: any of the mile from the speed estimate. Missing (older producer) is NOT "measured".
    mile_est = st.get("mileEst") is not False
    if charging:
      # Generator ON: ALWAYS "<kw>kw,<R1>,1mi:<R2>", so the form never changes while driving.
      if range_mi is None:
        r1 = "--m"
      elif p is None:
        r1 = f"{rng}m"                       # no load at all: the dash range itself (still "?")
      elif p <= ac:
        r1 = _OVER_RANGE                     # the generator covers the load: no finite range
      else:
        r1 = _range_text(range_mi * p / (p - ac)) or f"{rng}m"
      r1 = _q(r1, est)
      r2 = r1                                # before the first mile, or not computable: R1, with its "?"
      if mile_kwh is not None:
        net_mi = mile_kwh - mile_ac
        if net_mi <= 0.0:
          r2 = _q(_OVER_RANGE, mile_est)     # the mile netted out as charging
        elif e > 0.0:
          t = _range_text(e / net_mi)
          r2 = _q(t, mile_est) if t is not None else r1
      return f"{ac_kw:.1f}kw,{r1},1mi:{r2}"
    # Generator OFF. Regen, coasting or no energy: the truck's own dash range (">999m" is never shown
    # here: there is no charge to cover the load).
    r = e / p * mph if (p is not None and p > 0.0 and e > 0.0) else 0.0
    r_t = _miles(r)
    now_t = _q(f"{r_t}m" if r_t is not None else f"{rng}m", est)
    mile = f",1mi:{now_t}"                   # before the first mile: the instant term, with its "?"
    if mile_kwh is not None:
      net_mi = mile_kwh - mile_ac
      rm_t = _miles(e / net_mi if (net_mi > 0.0 and e > 0.0) else 0.0)
      mile = f",1mi:{_q(f'{rm_t}m', mile_est)}" if rm_t is not None else ",1mi:--"
      # A mile that netted out as CHARGING (its mean AC input at this speed, kWh/mi x mph = kW, above
      # _AC_ZERO_KW, after the charger was switched off) has no finite range. Regen alone stays "1mi:--".
      if net_mi <= 0.0 and mile_ac * mph > _AC_ZERO_KW:
        mile = f",1mi:{_q(_OVER_RANGE, mile_est)}"
    return f"{now_t}{pack}{mile}"

  # ---- render --------------------------------------------------------------
  def _render(self, rect: rl.Rectangle):
    if self._cached_layout is None:
      return
    text, box_w, text_w = self._cached_layout
    # Anchored to the RIGHT EDGE and the bottom, exactly as ces_status.py does — this is what keeps the
    # box out of the green driving path down the middle of the screen. The text is right-aligned, so
    # its right edge sits at a fixed x and the TEXT never moves as digits or form change.
    bx = rect.x + rect.width - box_w - _MARGIN
    by = rect.y + rect.height - _BOX_H - _MARGIN
    # raylib: radius = roundness * min(w, h) / 2. Converting from a PIXEL radius keeps the corners
    # looking like the CES and location boxes regardless of how wide this line happens to be --
    # passing ces_status's literal 0.12 would give this short, wide box only ~6.5 px and read square.
    roundness = min(1.0, 2.0 * _CORNER_R / min(box_w, _BOX_H))
    rl.draw_rectangle_rounded(rl.Rectangle(bx, by, box_w, _BOX_H), roundness, 8, _C.BG)
    rl.draw_text_ex(self.font, text, rl.Vector2(bx + box_w - _PAD - text_w, by + _PAD), _FS, 0, _C.WHITE)
