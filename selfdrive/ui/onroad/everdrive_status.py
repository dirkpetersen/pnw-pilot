"""
everdrive2pnw: EverDrive auxiliary-charger input power + range — LOWER-RIGHT, directly BELOW the CES box.

Display-only. One line, right-aligned and anchored to the RIGHT EDGE exactly the way ces_status.py
anchors its box, so — like the CES box — it must never grow into the green driving path drawn down
the middle of the screen. That is the driver's hard layout constraint, inherited verbatim from the CES
box docstring. (This sentence used to claim the width was "fixed from exemplars so it cannot creep
toward centre"; that stopped being true when the box started hugging its text on 2026-09-20. What
actually holds the constraint now is that the box is RIGHT-anchored and every term is digit-bounded
-- see the worst-case table at `_FS`.)

Driver-approved formats (2026-09-20 revision -- compact shapes, `m` for miles, and the pack's energy
in kWh beside the range it buys):
    charging, moving:    1.4kw,@132m(55.35kwh)->137m
    charging, stopped:   1.4kw,100m(55.35kwh),+2.7m/h
    not charging:        --,100m(55.35kwh)
    effOk False:         1.4kw,100m(55.35kwh)
    no capKwh/socPct:    the (kwh) parenthetical is OMITTED, never shown as 0.000

THE FIRST NUMBER HAS TWO MEANINGS AND THEY ARE NOT THE SAME QUANTITY, so the driver has to be able
to tell them apart at a glance:
    "@132m"  the range at the speed he is doing RIGHT NOW -- remaining pack energy divided by the
             MEASURED rolling consumption the producer published in `grossKw`. "@" reads "at".
    "100m"   the truck's OWN dash range estimate (VehElRnge_L_Dsply), printed exactly as this box
             has shipped since 2026-09-19. This is the FALLBACK, and it is what shows whenever
             `grossKw` is None: stopped, below ~10 mph, window not yet filled, or inputs missing.
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
# crosses screen centre -- into the green driving path this box must never enter.
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
# centre 1080. Every form the formatter can emit, at its maximum digit count:
#
#     44.4kw,@444m(444.44kwh),+44.4m/h   874.5 px text -> 922.5 box -> left 1167.5 -> +87.5 clear
#     44.4kw,444m(444.44kwh),+44.4m/h    830.2              878.2           1211.8      +131.8
#     44.4kw,@444m(444.44kwh)->444m      825.5              873.5           1216.5      +136.5
#     44.4kw,444m(444.44kwh)->444m       781.2              829.2           1260.8      +180.8
#     --,@444m(444.44kwh)                518.9              566.9           1523.1      +443.1
#
# WORST CASE 922.5 px, +87.5 px clear of the driving path, under the 1000 px _MAX_BOX_W tripwire.
# A 145,800-payload sweep across the producer's actual published bands tops out lower still, at
# '27.7kw,@140m(474.63kwh),+60.9m/h' = 897.1 px box, +112.9 px clear.
#
# The ranges stay at THREE digits because the producer bands the truck's own range to 254 mi and
# _RANGE_MAX_MI bands the computed one to 499 mi, which also bounds the projection (<= 2x) to 998.
# _GAIN_MAX_MI_H bounds the rate to 99.9. Adding a fourth digit anywhere costs ~30 px.
_FS = 48
_LINE_H = 60              # _FS * 1.25, the ratio ces_status.py and location_services_status.py both use
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
                          #   `energyKwh / grossKw * mph`. Two independent constraints land on the
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
                          #   The producer's bands make it unreachable (kW <= 27.7, range <= 254 mi,
                          #   capacity <= 470.7 kWh, rate <= 99.9), so tripping it means an input is
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

    # everdrive2pnw (driver req 2026-09-20): the FIRST number is now the range at the speed being
    # driven RIGHT NOW -- remaining pack energy divided by the measured rolling consumption. Both
    # inputs come from the producer, which publishes `grossKw` only while the truck is above ~10 mph
    # AND its window has cleared its resolution floor; see everdrive_pnw._gross_kw. So a None here
    # already means "stopped, or not measured yet", and this widget does not re-derive that.
    #
    # `grossKw` is GROSS of the EverDrive input on purpose. A SoC-derived consumption is already net
    # of whatever the charger is putting in, so projecting `range * P/(P - acKw)` off a NET P would
    # count the charger twice. Using the gross figure keeps the projection meaning what it always did.
    #
    # THE FALLBACK IS THE TRUCK'S OWN RANGE, PRINTED EXACTLY AS IT SHIPS TODAY, and the computed
    # number carries a leading "@" ("at this speed"). The marker is on the COMPUTED value rather than
    # on the fallback for two reasons: the degraded path then stays bit-identical to the shipped box,
    # and the extra character lands in the MOVING form, which is the narrower of the two -- the
    # widest line the box can emit is the stopped form, which always falls back and so is unchanged.
    gross_kw = st.get("grossKw")
    energy_kwh = st.get("energyKwh")
    measured_mi = None
    if gross_kw is not None and energy_kwh is not None:
      g, e = _f(gross_kw), _f(energy_kwh)
      if g > 0.0 and e > 0.0:
        r = e / g * mph
        if 0.0 < r <= _RANGE_MAX_MI:
          measured_mi = r

    if measured_mi is not None:
      # p_kw is what the projection divides by. Measured consumption needs no efficiency constant, so
      # this path stays alive even when effOk is False -- it never depended on that signal.
      range_mi, p_kw = measured_mi, _f(gross_kw)
      rng = f"@{range_mi:.0f}"
    else:
      range_mi = truck_mi if truck_mi > 0.0 else None
      # today's assumed power: the truck's OWN reference efficiency, so the projection stays
      # internally consistent with the range printed beside it. No substituted default if it is gone.
      p_kw = (eff_wh_mi * mph / 1000.0) if (st.get("effOk") and eff_wh_km > 0.0) else None
      rng = f"{range_mi:.0f}" if range_mi is not None else "--"   # never "0 m" from a dead signal

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
    if cap_kwh is not None and soc_pct is not None:
      pack = f"({_f(soc_pct) / 100.0 * _f(cap_kwh):.2f}kwh)"

    # acSeen False == the EverDrive CAN message has not been received, so acKw is a pre-filled 0.0 and
    # NOT a measurement. acKw <= _AC_ZERO_KW is the opposite case: a real measured zero (unplugged).
    # Both read "not charging" to the driver, and neither may ever print a manufactured "0.0 kW".
    if not st.get("acSeen") or ac_kw <= _AC_ZERO_KW:
      return f"--,{rng}m{pack}"

    kw = f"{ac_kw:.1f}kw"
    # No range, or no consumption to divide by, means no projection and no gain rate -- and NO
    # substituted default: a made-up constant would produce a confident number out of a signal we do
    # not have. p_kw is None exactly when effOk is False or effWhKm is at its -100 encoding floor AND
    # there is no measured consumption to use instead.
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
    # everdrive2pnw: the gain rate rides the SAME basis as the range printed beside it -- the measured
    # mi/kWh (mph / grossKw) when there is one, the truck's reference efficiency otherwise. Mixing the
    # two would put a measured range next to a reference-efficiency gain and invite the driver to
    # divide one by the other. It is also what keeps this line safe when effOk is False: on the
    # measured path eff_wh_mi is allowed to be 0, and dividing by it would crash-loop the UI.
    gain = ac_kw * ((mph / p_kw) if measured_mi is not None else (1000.0 / eff_wh_mi))
    # everdrive2pnw: the same fixed-width discipline the projection gets from _PROJ_MAX_RATIO. The
    # exemplar budgets TWO integer digits for m/h ("+00.0"); three would overflow the box and CLIP,
    # which is a silent corruption of the whole line, not just of this term. The producer's AC band
    # already bounds this to ~53.8 mi/h, so reaching here means the producer's guard was bypassed or
    # effWhKm is implausibly small -- either way the inputs are wrong. Drop the term rather than print
    # a clipped one: showing LESS is honest, showing a truncated number is not (Rule 2). The change of
    # form is the visible signal, exactly as it is for the projection.
    if gain >= _GAIN_MAX_MI_H:
      return f"{kw},{rng}m{pack}"
    # `rng`, NOT f"{range_mi:.0f}" -- range_mi is whichever quantity was selected above, and on the
    # measured path the "@" is the ONLY thing telling the driver which one he is reading. Formatting
    # it again here silently dropped the marker on this branch (found reviewing the diff 2026-09-20,
    # after the tests for this branch checked the gain term and not the range term).
    return f"{kw},{rng}m{pack},+{gain:.1f}m/h"

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
