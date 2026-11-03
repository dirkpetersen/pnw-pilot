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
    charger on:          10.8kw,343m,1mi:343m      (2026-11-02: dash 177.6 mi, 22.3 kW load, 10.75 kW in)
    window not filled:   10.8kw,?282m,1mi:?282m      ("?" = the speed estimate, 320 Wh/km at 54 mph)
    charge >= load:      10.8kw,>999m,1mi:>999m      (measured; "?>999m" from the estimate)
    generator idle:      GEN 0kw!,?282m,1mi:?282m    (running, not supplying; no (kwh), for width)
GENERATOR (everdrivegen2pnw, owner 2026-11-02): the kW term is YELLOW while the producer's genState is
"on" (from the first frame with voltage, so it may start at ~0.3 kW and ramp); "GEN 0kw!" is ORANGE-RED
and also replaces the kW term in the stopped form ("GEN 0kw!,112m(63.49kwh)"). A 5 s banner above the
box marks each change: "Generator ON 0.3 kW" (live kW), "Generator OFF", "Generator: NO POWER". No
state, colour or banner without acSeen or from a stale payload. See _GEN_STATES.
TWO LINES WHILE THE GENERATOR CHARGES (everdrivetwoline2pnw, owner 2026-11-02): moving, genState "on" and
acKw above _AC_ZERO_KW. Every other case keeps the one-line forms above and below.
    10.8kw 27kw 59kwh              generator kW (yellow), truck load P ("?" when estimated), pack kWh
    227m 1mi:227m d1m:187m         SoC instant energyKwh / (P - acKw) x mph, SoC mile energyKwh /
                                   mileNetKwh, dash mile (the one-line "1mi:" of the charging form)
Any of the three over 999 or with a net load <= 0 is a plain ">999m", never "?>999m" (owner: "if any of
them reaches >999 just use that number"). Before the first full mile "1mi:" repeats the instant range.
STOPPED, PARKED or below 10 mph (2026-09-20 revision, unchanged):
    charging, crawling:  1.4kw,100m(55.35kwh)->104m
    charging, stopped:   1.4kw,100m(55.35kwh),+2.7m/h
    not charging:        --,100m(55.35kwh)
    effOk False:         1.4kw,100m(55.35kwh)
    no capKwh/socPct:    the (kwh) parenthetical is OMITTED, never shown as 0.000

THE MOVING TERMS (everdrivesoc2pnw, owner 2026-11-02). P is the measured load grossKw: the pack SoC
drop over the producer's 25-180 s window plus the AC input (see everdrive_pnw.py). Until that window
has filled (the first ~25-60 s of a drive on the highway) P is the SPEED ESTIMATE effWhKm x km/h / 1000
+ _P_ACC_KW (the truck's reference efficiency) and every figure built from it gets a leading "?"; it
drops as soon as grossKw is published. With no efficiency either, the first term is the dash range,
still "?". (Until 2026-11-02 P was k x motor-2 power from 0x441. A drive showed that motor 2 does
little at highway cruise, so that figure was not the pack load; it is gone.)
    term 1, charger off  energyKwh / P x mph. Without "?" ONLY from a measured P with a result in
                         1.._RANGE_MAX_MI. Anything else (no reading, regen, coasting, over the
                         ceiling, no energyKwh) is the speed estimate, or failing that the truck's
                         dash range, both with "?".
    term 1, charger on   the DASH range scaled by the generator, dash x P / (P - acKw), e.g. dash
                         141 mi, 30 kW, 10.8 kW in -> "220m"; needs no capacity. ">999m" when the
                         measured P <= acKw or over 999; "?>999m" when the ESTIMATE is <= acKw.
                         ">999m" is never printed from a missing reading without "?".
    "1mi:"               the AVERAGE of term 1 over the last mile (everdrivemile2pnw, owner 2026-11-02):
                         term 1's formula on the producer's distance-weighted mile means of the load
                         (mileLoadKw: grossKw, else the speed estimate) and of acKw (mileAcKw).
                         Charger on: dash x mileLoadKw / (mileLoadKw - mileAcKw), ">999m" when
                         mileLoadKw <= mileAcKw or over 999; not computable repeats term 1.
                         Charger off: energyKwh / mileLoadKw x mph; "1mi:--" when not usable.
                         "?" when any of the mile used the estimate (producer mileEst not False).
                         Before the first full mile it repeats term 1 (with its "?").
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
_BOX_H = _LINE_H + _PAD * 2   # one line; the two-line generator form adds one _LINE_H (178 px)

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


# everdrivegen2pnw (owner 2026-11-02): the producer's `genState` ("off" / "idle" / "on", see
# everdrive_pnw.py GEN_*). "on": the kW term is YELLOW. "idle" (generator running, not supplying): the
# kW term is replaced by an ORANGE-RED _GEN_IDLE_TERM. Every change between the three states shows a
# _BANNER_S banner ABOVE the box, right-anchored like it (display only: no alert, no event, no sound).
# The first state seen in a session is not a change, so it shows no banner.
#
# WIDTHS, same model and margins as the _FS table (Inter-Medium advances at atlas base 200, x _FS 52
# x FONT_SCALE 1.16 / 200; reproduces that table to within 2.4 px, on the wide side):
#     GEN 0kw!,?444m,1mi:?>999m     723.2 px text -> 771.2 box -> left 1318.8 -> +238.8 clear
#     GEN 0kw!,444m(444.44kwh)      703.0              751.0           1339.0      +259.0
#     Generator ON 44.4 kW          544.4              592.4           1497.6      +417.6   (banner)
#     ?>999m 1mi:?>999m d1m:?>999m  828.8              876.8           1213.2      +133.2   (line 2 bound)
#     44.4kw ?444kw 444kwh          596.3              644.3           1445.7      +365.7   (line 1)
# Two-line box: 178 px high (one more _LINE_H); stack_height lifts the CES box by it.
#     Generator: NO POWER           538.4              586.4           1503.6      +423.6   (banner)
# The idle MOVING form drops the (kwh) term: with it, "GEN 0kw!,?444m(444.44kwh),1mi:?444m" is a
# 1053.8 px box, over _MAX_BOX_W and 3.8 px INTO the driving path. Idle never shows a projection or a
# gain rate either: the generator is supplying nothing, so there is nothing to project.
_GEN_STATES = ("off", "idle", "on")
_GEN_IDLE_TERM = "GEN 0kw!"
_BANNER_S = 5.0
_BANNER_TEXT = {"off": "Generator OFF", "idle": "Generator: NO POWER"}   # "on" carries the live kW


class _C:
  WHITE = rl.Color(255, 255, 255, 235)
  BG = rl.Color(0, 0, 0, 140)
  # Owner 2026-11-02: green was hard to read in daylight. Bright yellow for "on"; idle moves from amber
  # (255, 149, 0) to orange-red, because amber sits too close to this yellow to tell apart at a glance.
  YELLOW = rl.Color(255, 220, 0, 255)
  ORANGE_RED = rl.Color(255, 110, 40, 255)


_BANNER_COLOR = {"off": _C.WHITE, "idle": _C.ORANGE_RED, "on": _C.YELLOW}


def _gen_state(st: dict) -> str | None:
  """The producer's genState, or None when it is unknown or the 0x2A7 reading is not a real one
  (acSeen not True): then there is no generator state to show, colour or announce."""
  g = st.get("genState")
  return g if st.get("acSeen") is True and g in _GEN_STATES else None


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


def _finite(v) -> float | None:
  x = _num(v)
  return x if x is not None and math.isfinite(x) else None


def _soc_range(e: float | None, net: float, est: bool, scale: float) -> str:
  """everdrivetwoline2pnw: e / net x scale as printed. Net <= 0 or over 999: a plain ">999m", never with
  "?" (owner 2026-11-02: "if any of them reaches >999 just use that number"). "--m" without energy."""
  if net <= 0.0:
    return _OVER_RANGE
  if e is None or e <= 0.0:
    return "--m"
  t = _range_text(e / net * scale)
  return "--m" if t is None else (t if t == _OVER_RANGE else _q(t, est))


def _two_lines(st: dict, ac_kw: float, p: float | None, est: bool, mph: float, mile_est: bool, d1m: str) -> str:
  """everdrivetwoline2pnw (owner 2026-11-02): moving with the generator "on" AND current flowing (acKw
  above _AC_ZERO_KW: "I only need the two lines if everdrive is actually charging"), two lines.
    line 1: generator kW (yellow), the truck load P (grossKw, else the "?" speed estimate), pack kWh
    line 2: the SoC instant range energyKwh / (P - acKw) x mph, "1mi:" energyKwh / mileNetKwh (the net
            kWh per mile over the last mile; before it, the instant range), "d1m:" the dash-based mile
            figure (the d77ffaf76 "1mi:" term, passed in). Any of the three over 999 is a plain ">999m"."""
  e = _finite(st.get("energyKwh"))
  load = "--kw" if p is None or not math.isfinite(p) else _q(f"{round(p)}kw", est)
  # Owner 2026-11-02: spaces between the terms in the two-line form only.
  line1 = f"{ac_kw:.1f}kw {load}" + (f" {round(e)}kwh" if e is not None else "")
  if p is None or not math.isfinite(p):
    inst = "--m"
  else:
    inst = _soc_range(e, p - ac_kw, est, mph)
  net = _finite(st.get("mileNetKwh"))
  mile = inst if net is None else _soc_range(e, net, mile_est, 1.0)
  d1m = d1m.removeprefix("?") if d1m.endswith(_OVER_RANGE) else d1m
  return f"{line1}\n{inst} 1mi:{mile} d1m:{d1m}"


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
    # everdrivetwoline2pnw: each line with its width, and the box height (one or two lines).
    self._cached_lines: list[tuple[str, float]] = []
    self._cached_box_h = _BOX_H
    # everdrivegen2pnw: the coloured first term (chars, px, colour) and the banner (text, box_w,
    # text_w, colour); None == plain white line / no banner.
    self._cached_head: tuple[int, float, rl.Color] | None = None
    self._cached_banner: tuple[str, float, float, rl.Color] | None = None
    self._gen_prev: str | None = None     # last genState seen; None = none seen since the key appeared
    self._banner_kind: str | None = None
    self._banner_until = 0.0
    # Log each DISTINCT read fault once (not 5x/s), with a flag per fault: sharing one flag would let
    # the second, different failure go completely unlogged.
    self._toggle_err_logged = False
    self._status_err_logged = False
    self.font = gui_app.font(FontWeight.MEDIUM)

  @property
  def stack_height(self) -> float:
    """everdrive2pnw: how far augmented_road_view must lift the CES box so the two stack. Exactly 0.0
    while this box is hidden, so the CES box renders where it does today. While a generator banner
    shows (above this box) the CES box is lifted by one more box height, so the two never overlap."""
    if self._cached_layout is None:
      return 0.0
    return self._cached_box_h + _STACK_GAP + (_BOX_H + _STACK_GAP if self._cached_banner is not None else 0.0)

  # ---- state ---------------------------------------------------------------
  def _update_state(self):
    now = time.monotonic()
    if now - self._last_poll < self._poll_interval:
      return
    self._last_poll = now
    # Hidden until proven otherwise, and slow-polling until a payload actually shows up. Every early
    # return below therefore costs one param read and nothing else — no formatting, no measurement.
    self._cached_layout = None
    self._cached_head = None
    self._cached_banner = None
    self._poll_interval = _ABSENT_REFRESH_S
    if self._mem is None:
      return
    # DisableEverDrive is PERSISTENT, so it MUST be read from the NORMAL params store (ui_state.params),
    # never through the /dev/shm handle above — a PERSISTENT key read from the mem store returns False
    # forever, which would silently nail the toggle to "enabled". Defensive: a params/UI mismatch falls
    # back to SHOWING (the documented default state) and can never crash the UI.
    try:
      if ui_state.params.get_bool("DisableEverDrive"):
        self._gen_prev = self._banner_kind = None   # hidden: the first state after it is not a change
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
      self._gen_prev = self._banner_kind = None   # module gone: its first state back is not a change
      return          # key absent: NORMAL on the Tesla / a Lightning with no EverDrive. Stay quiet.
    self._poll_interval = _REFRESH_S   # a payload exists -> track the publisher at ~5 Hz from here on
    text = self._build_text(st)
    if text is None:
      # Producer went silent (stale ts) -> hide; keep polling fast so it can come back. The state is
      # checked only AFTER this (Gemini 2026-11-02): a stale payload must not feed the banner.
      self._gen_prev = self._banner_kind = None
      return
    # everdrivegen2pnw: a change between two KNOWN states of a fresh payload starts the banner. A missing
    # or unknown genState (older producer), or acSeen not True, changes nothing and starts nothing.
    gen = _gen_state(st)
    if gen is not None:
      if self._gen_prev is not None and gen != self._gen_prev:
        self._banner_kind, self._banner_until = gen, now + _BANNER_S
      self._gen_prev = gen
    # The coloured first term: YELLOW kW while the generator is on, ORANGE-RED "GEN 0kw!" while idle.
    kw_term = f"{_f(st.get('acKw')):.1f}kw"
    if text.startswith(_GEN_IDLE_TERM):
      head = (_GEN_IDLE_TERM, _C.ORANGE_RED)
    elif gen == "on" and text.startswith((kw_term + ",", kw_term + " ")):
      head = (kw_term, _C.YELLOW)
    else:
      head = None
    if head is not None:
      self._cached_head = (len(head[0]), measure_text_cached(self.font, head[0], _FS).x, head[1])
    if now < self._banner_until and self._banner_kind is not None:
      b = _BANNER_TEXT.get(self._banner_kind) or f"Generator ON {_f(st.get('acKw')):.1f} kW"
      b_w = measure_text_cached(self.font, b, _FS).x
      self._cached_banner = (b, b_w + _PAD * 2, b_w, _BANNER_COLOR[self._banner_kind])
    # Measured HERE (5 Hz) and cached, so _render does nothing but draw. A device with no EverDrive
    # never reaches this line at all, so it never pays for a text measurement.
    # everdrivetwoline2pnw: one line, or two while the generator charges a moving truck. Each line is
    # measured on its own and right-aligned; the box hugs the widest.
    self._cached_lines = [(ln, measure_text_cached(self.font, ln, _FS).x) for ln in text.split("\n")]
    self._cached_box_h = _BOX_H + _LINE_H * (len(self._cached_lines) - 1)
    text_w = max(w for _ln, w in self._cached_lines)
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
    # own form below, so that path was dead and is removed. grossKw is the moving load since 2026-11-02.)
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
    # everdrivegen2pnw: genState "on" is charging too, even at a measured ~0 kW: the producer says "on"
    # from the first frame with voltage (owner 2026-11-02), and the kW figure then shows the ramp.
    gen = _gen_state(st)
    charging = bool(st.get("acSeen")) and (ac_kw > _AC_ZERO_KW or gen == "on")
    # everdrivegen2pnw: generator running but not supplying. _GEN_IDLE_TERM replaces the kW term; the
    # rest is the charger-OFF figure (a trickle under the producer's 0.5 kW is not a supply). Moving:
    # the three-term shape WITHOUT the (kwh), for width (see the table at _GEN_IDLE_TERM).
    if gen == "idle":
      if st.get("moving") is True:
        return f"{_GEN_IDLE_TERM}," + self._moving_text(st, False, ac_kw, rng, range_mi, mph, "",
                                                         eff_wh_km if st.get("effOk") else 0.0)
      return f"{_GEN_IDLE_TERM},{rng}m{pack}"
    if st.get("moving") is True:
      return self._moving_text(st, charging, ac_kw, rng, range_mi, mph, pack,
                               eff_wh_km if st.get("effOk") else 0.0,
                               two_line=gen == "on" and ac_kw > _AC_ZERO_KW)
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
                   pack: str, eff_wh_km: float, two_line: bool = False) -> str:
    """everdriveinstant2pnw: the moving form, owner 2026-11-01. See the module docstring. No "@" and
    no "m/h" ever; a leading "?" marks every figure NOT built from the measured load grossKw
    (everdrivesoc2pnw, 2026-11-02; it was pNowKw from motor 2 before).
    `rng` is the truck's own range ("--" if it is gone); `eff_wh_km` is 0 when effOk is False."""
    e = _f(st.get("energyKwh"))
    ac = ac_kw if charging else 0.0
    p_live = _num(st.get("grossKw"))
    # The speed estimate: the truck's reference efficiency at this speed + the accessory load.
    p_est = eff_wh_km / 1000.0 * (mph / _MS_TO_MPH * 3.6) + _P_ACC_KW if eff_wh_km > 0.0 else None
    est = p_live is None
    p = p_est if est else p_live
    # everdrivemile2pnw (owner 2026-11-02): "1mi:" is the AVERAGE of term 1 over the last mile, built
    # as term 1's formula on the producer's mile means of its load and of acKw. acSeen gates the AC
    # mean as it gates acKw.
    mile_p = _num(st.get("mileLoadKw"))
    mile_ac = _f(st.get("mileAcKw")) if st.get("acSeen") else 0.0
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
        r1 = _range_text(range_mi * p / (p - ac))
        # Not reachable today (range_mi >= 0.5 and P / (P - acKw) > 1, so it rounds to >= 1); kept so
        # that a later change can not print the unscaled dash range bare, as if measured.
        if r1 is None:
          r1, est = f"{rng}m", True
      r1 = _q(r1, est)
      r2 = r1                                # before the first mile: R1, with its "?"
      if mile_p is not None and range_mi is not None:
        # Term 1's rule on the mile means: >999m when the mean load is covered by the mean charge.
        t = _OVER_RANGE if mile_p <= mile_ac else _range_text(range_mi * mile_p / (mile_p - mile_ac))
        # None: a full mile that cannot be computed, so R1 stands in, and it is not the mile's own figure.
        r2 = _q(t, mile_est) if t is not None else _q(r1.removeprefix("?"), True)
      if two_line:
        return _two_lines(st, ac_kw, p, est, mph, mile_est, r2)
      return f"{ac_kw:.1f}kw,{r1},1mi:{r2}"
    # Generator OFF. A bare range is ALWAYS a measured one: grossKw and a result in 1.._RANGE_MAX_MI.
    # Anything else (no reading, regen, coasting, over the ceiling, no energy) is the speed estimate,
    # or failing that the dash range, both with "?". ">999m" is never shown: no charge covers the load.
    r_t = _miles(e / p_live * mph) if (p_live is not None and p_live > 0.0 and e > 0.0) else None
    if r_t is not None:
      now_t = f"{r_t}m"
    else:
      r_e = _miles(e / p_est * mph) if (p_est is not None and e > 0.0) else None
      now_t = _q(f"{r_e if r_e is not None else rng}m", True)
    mile = f",1mi:{now_t}"                   # before the first mile: the instant term, with its "?"
    if mile_p is not None:
      # Term 1's formula on the mile mean load, at the current speed. No usable figure: "1mi:--".
      rm_t = _miles(e / mile_p * mph) if (mile_p > 0.0 and e > 0.0) else None
      mile = f",1mi:{_q(f'{rm_t}m', mile_est)}" if rm_t is not None else ",1mi:--"
    return f"{now_t}{pack}{mile}"

  # ---- render --------------------------------------------------------------
  def _render(self, rect: rl.Rectangle):
    if self._cached_layout is None:
      return
    _text, box_w, _text_w = self._cached_layout
    box_h = self._cached_box_h
    # Anchored to the RIGHT EDGE and the bottom, exactly as ces_status.py does — this is what keeps the
    # box out of the green driving path down the middle of the screen. The text is right-aligned, so
    # its right edge sits at a fixed x and the TEXT never moves as digits or form change.
    bx = rect.x + rect.width - box_w - _MARGIN
    by = rect.y + rect.height - box_h - _MARGIN
    # raylib: radius = roundness * min(w, h) / 2. Converting from a PIXEL radius keeps the corners
    # looking like the CES and location boxes regardless of how wide this line happens to be --
    # passing ces_status's literal 0.12 would give this short, wide box only ~6.5 px and read square.
    roundness = min(1.0, 2.0 * _CORNER_R / min(box_w, box_h))
    rl.draw_rectangle_rounded(rl.Rectangle(bx, by, box_w, box_h), roundness, 8, _C.BG)
    for i, (line, line_w) in enumerate(self._cached_lines):
      tx = bx + box_w - _PAD - line_w
      ty = by + _PAD + i * _LINE_H
      if i > 0 or self._cached_head is None:
        rl.draw_text_ex(self.font, line, rl.Vector2(tx, ty), _FS, 0, _C.WHITE)
      else:
        # everdrivegen2pnw: the first term in its colour, the rest white right after it. Spacing is 0 and
        # raylib applies no kerning, so the two halves sit exactly where the one string would.
        n, head_w, color = self._cached_head
        rl.draw_text_ex(self.font, line[:n], rl.Vector2(tx, ty), _FS, 0, color)
        rl.draw_text_ex(self.font, line[n:], rl.Vector2(tx + head_w, ty), _FS, 0, _C.WHITE)
    if self._cached_banner is not None:
      # Directly ABOVE the box, right-anchored the same way (so it can never reach further left than
      # the box can); stack_height lifts the CES box clear of it.
      b, b_box_w, b_w, b_color = self._cached_banner
      b_x = rect.x + rect.width - b_box_w - _MARGIN
      b_y = by - _STACK_GAP - _BOX_H
      b_round = min(1.0, 2.0 * _CORNER_R / min(b_box_w, _BOX_H))
      rl.draw_rectangle_rounded(rl.Rectangle(b_x, b_y, b_box_w, _BOX_H), b_round, 8, _C.BG)
      rl.draw_text_ex(self.font, b, rl.Vector2(b_x + b_box_w - _PAD - b_w, b_y + _PAD), _FS, 0, b_color)
