"""
speedadjust2pnw — automatic cruise-speed reduction for lower speed limits and police-ahead warnings.

`SpeedAdjustController.cap(sm, v_cruise_set, v_cruise, v_ego, v_cruise_initialized)` returns a possibly-
lowered cruise speed (m/s) for the longitudinal planner — **reduce-only, NEVER raises above v_cruise**.
The planner MPC bounds the decel rate, so a cap can never slam; it composes with VTSC/MTSC via `min()`.
(See the `cap()` docstring below for why it takes both a raw `v_cruise_set` and an effective `v_cruise`.)

Selector param `AutoSpeedReduce` (INT, default 0):
  0 = Off
  1 = Police         → ease to (posted speed limit + 5 mph) when a police report is ~30 s ahead
  2 = Police+Limits  → ALSO cap proportionally when the posted limit drops (keeps your over-limit ratio)

Works in BOTH Chill and Experimental (it caps `v_cruise`, which both modes honour). The RETURNED cap
only ever feeds the op-long MPC path, so `cap()`'s return value stays exactly what it always was —
`v_cruise` unchanged whenever openpilot does not own longitudinal (op-long off), so this function is
still behaviour-neutral by default on that axis. Inputs are read from params at ~1 Hz — `MapSpeedLimit`
+ `LocationServices` from the /dev/shm mem store (written by the mapd bridge / location_servicesd) and
`AutoSpeedReduce` persistent — with **NO msgq subscriptions** for those reads (the background-
subscriber cascade lesson); `sm['carState']` (already subscribed by plannerd at its normal control
rate — not a new subscription) is read only as a defense-in-depth abort signal for the restore window
below.

Runs inside plannerd (20 Hz) via the planner's cap chain. Pure-ish: only param reads + monotonic time.

speedadjust-exec2pnw (2026-08): the reduce-only cap math ITSELF now runs identically regardless of
op-long — plannerd runs on every car (not gated on openpilotLongitudinalControl), so the target was
always being computed for op-long cars but silently discarded for stock-ACC ones. On a car with NO
op-long (the Lightning in its normal stock-ACC mode), this module ADDITIONALLY publishes its computed
target as a `SpeedAdjustTarget` mem-param (`CLEAR_ON_MANAGER_START`, JSON) in the SAME
`{target, ceiling, ts, dir?}` shape as icbm2pnw's `IcbmTarget` — so a capability-gated, per-brand
stock-ACC button-tap executor (today: `opendbc/car/ford/icbm_pnw.py`, gated on
`PnwVehicle.button_management`) can steer the truck's own SET-/SET+ buttons toward it, exactly the
way icbm2pnw already does for curve slow-downs. Both mem-params share ONE executor; `arbitrate()`
(icbm_pnw.py) reduces whichever brains are live down to a single most-restrictive target every poll —
see `docs/pnw/SPEEDADJUST-EXECUTOR.md` for the full design. This module stays fully car-agnostic: it never
checks carFingerprint/brand, only `self._long_ok` (openpilotLongitudinalControl) — the actual per-car
capability gating lives in `PnwVehicle`/`icbm_pnw.py`, not here.

The mem-param publish also includes a bounded (`RESTORE_WINDOW_S`, matching icbm2pnw's convention) SET+
restore back to the ceiling latched at cap-engage once the cap clears — deliberately SIMPLER than
icbm2pnw's full episode state machine: it does not track the truck's own reported stock set speed at
all (the shared executor's `decide_press`/`RestoreGuard` already do that closed-loop, human-detection
work directly off the real CAN state); this brain only decides WHEN a restore should be offered
(bounded window, canceled instantly by a new cap or `sm['carState']` driver/ACC intervention) and
WHAT ceiling to offer, never above the driver's own pre-cap set.

speedanchor2pnw (2026-07-18, three review-caught fixes to the anchor/seed math — none change the
reduce-only envelope, all conservative):
  F2  cap() now takes the driver's raw PRE-VTSC set (`v_cruise_set`) separately from the current
      EFFECTIVE ceiling (`v_cruise`, after VTSC/etc. have already reduced it). The limit-drop ratio
      anchor and the cap-slew seed use `v_cruise_set` — anchoring/seeding off a VTSC-curve-reduced
      `v_cruise` instead recorded a too-low ratio (could disable the trim) and seeded the slew from
      curve speed, crawling back up at CAP_SLEW after VTSC released instantly. The emitted output is
      still bounded reduce-only against `v_cruise` (the effective ceiling), unchanged.
  F3  the limit-drop baseline (`_sl_ref`/`_ratio`) is now kept current in mode 1 (police-only) too, via
      `_update_baseline()`, not just inside `_limit_drop_cap()` (mode 2 only) — previously flipping
      AutoSpeedReduce 1→2 mid-drive could resume off a baseline captured possibly hours earlier.
  F_uninit  anchoring/seeding is skipped entirely while `v_cruise_initialized` is False (cruise never
      set — `v_cruise`/`v_cruise_set` are the `V_CRUISE_UNSET` sentinel, ~145 km/h) — anchoring off
      that would inflate `_ratio` and seed `_cap_out` far above the real set.

speedadjustreset2pnw (2026-08-16, driver directive): ANY manual change to the cruise SET speed (up or
down) is treated as an explicit "resume — don't slow me for this" override, resetting BOTH the police
latch/suppression and the limit-drop baseline (see `cap()` + `_is_own_actuation()`). This relies on
`v_cruise_set` being feedback-safe — i.e. that cap()'s own output can never be mistaken for a driver
change on the NEXT tick. CORRECTED (2026-08-16, Fable review pass 2 — the original text below was
factually wrong about the mechanism, though its conclusion held): BOTH fleet cars are
`CP.pcmCruise=True` (neither sets it False) — the non-pcm `VCruiseHelper` branch (`CS.buttonEvents`
button-press tracking) NEVER runs on this fleet. `v_cruise_set` actually comes from the PCM branch of
`selfdrive/car/cruise.py` (`v_cruise_kph = CS.cruiseState.speed`, i.e. `DI_digitalSpeed` on the Tesla) —
but it is STILL feedback-safe on an op-long car. CORRECTED again (Fable review pass 3, F3 — the prior
wording overclaimed "nothing openpilot transmits ever feeds DI_digitalSpeed": openpilot DOES transmit
`DAS_setSpeed` on the Raven, see `teslacan_legacy.py`). The actual, empirically-grounded point is
narrower: the DI does not ECHO `DAS_setSpeed` back into `DI_digitalSpeed` — if it did, `vCruise` would
flap between 0 and the ~145 kph sentinel every time openpilot's own longitudinal loop wrote a set
speed, and that would already be visibly breaking the cruise pipeline (months of sane vSet telemetry
say otherwise). `longitudinal_planner.py:151-153` also only folds `cap()`'s return value LOCALLY into
its own `v_cruise` variable for the MPC target — it is never written back to any CAN signal or param —
so it can never loop back into next tick's `CS.cruiseState.speed` either way.
  * op-long cars: SAFE by construction, per the above — every real `v_cruise_set` change there IS the
    driver (or, on the Tesla specifically, an ENGAGE transition off the `cruiseState.speed` standby
    floor — see FIX A / `_cruise_engaged()` below, which is why the detector additionally requires ACC
    to be engaged on two consecutive ticks before trusting a delta).
  * stock-ACC cars (e.g. the Lightning off Alpha-Long): NOT safe by construction — THIS module's
    speedadjust-exec2pnw SET-/SET+ button-tap executor actually moves the same `CS.cruiseState.speed`
    dial that `v_cruise_set` reads. Without a guard, our own dec taps while capping (or inc taps while
    restoring) would look identical to a driver override and self-cancel the feature after a single
    tap. `_is_own_actuation()` filters this out DIRECTION-ONLY (see FIX C below — a value-tolerance
    check can't distinguish our own tap from a driver's SAME-direction tap, since one tap step is
    smaller than the tolerance needed to absorb our own actuation noise); `SA_ACTUATION_GRACE_S` (FIX
    B below) additionally covers the actuation-latency race at phase boundaries, where `_cap_out`/
    `_restore_ceiling` null instantly but an already-in-flight tap can still land 1-2 ticks later.

speedadjustreset2pnw hardening (2026-08-16, second Fable + Gemini review pass — Fable passed with 2
majors, Gemini blocked on FIX C; all addressed below):
  FIX A  (Fable major — Tesla engage-time false suppression) the `v_cruise_initialized` guard alone
      never protects the Tesla: `cruiseState.speed` is floored at `max(DI_digitalSpeed·conv, 1e-3)`,
      never exactly 0, so the PCM branch's `speed==0 -> V_CRUISE_UNSET` escape never fires and
      `v_cruise_initialized` stays True even in STANDBY — engaging cruise near a latched police report
      makes the set "jump" (e.g. ~0.004 -> 26.8 m/s), reading as a manual override and silently
      defeating the cap at the exact moment it should engage. Fixed: `_cruise_engaged()` gates the
      detector on `sm['carState'].cruiseState.enabled`, and `_last_v_set` is force-reset to None on
      every NOT-engaged tick — mirroring the existing uninit-sentinel treatment — so two consecutive
      ENGAGED ticks with a real delta are required before anything counts as an override.
  FIX B  (Fable major — stock-ACC boundary-race self-cancel) `_is_own_actuation()` reads `_cap_out`/
      `_restore_ceiling` at OBSERVATION time, but our own SET+/SET- taps have 0.3-2 s actuation->CAN->
      carState latency while those fields null INSTANTLY at phase transitions (cap releases, restore
      expires/cancels, a new cap preempts a restore) — an already-in-flight tap can land 1-2 ticks
      AFTER the field went None and get misread as a driver override. Fixed: `SA_ACTUATION_GRACE_S`
      (stock-ACC only — op-long has no physical actuation latency to race) suppresses the detector for
      a short grace window after any such transition, stamped at each of the three transition sites.
  FIX C  (Gemini BLOCKER) on stock-ACC, our own tap and a driver's SAME-direction tap are physically
      indistinguishable by value alone (one 1-mph step is smaller than the tolerance needed to absorb
      our own tap noise) — the old value-tolerance check therefore MASKED a genuine driver SET- during
      an active dec-cap. Fixed: `_is_own_actuation()` on stock-ACC is now DIRECTION-ONLY — any
      SAME-direction move during active actuation (down while dec-capping, up while restoring) is
      treated as ours (not attributable, never resets); only an OPPOSITE-direction move (driver wants
      to go faster than a dec cap, or slower than a restore ceiling) is unambiguous and still resets.
      KNOWN LIMITATION: a single same-direction driver tap during active actuation will not register as
      an override on stock-ACC — an opposite-direction tap (or fully disengaging) still works. Op-long
      keeps full any-direction detection (clean — our own output never moves the dial there).
  FIX D  (Fable minor) the suppression used to fire on ANY set change whenever `state=="alert"`
      existed, even with the report minutes away (`_police_latched` False, no cap active) — a routine
      speed adjustment could silently kill an eventual slowdown the driver never even perceived. Fixed:
      gated on `self._police_latched or self._cap_out is not None` — only dismiss an alert the driver
      could actually perceive acting on.
  FIX E  (Fable minor, telemetry only) the override-release block cleared `_cap_out` but not
      `_engaged`, swallowing the "released" cloudlog and the next engage's log. Fixed: mirrors the
      normal release block's `if self._engaged: ...` bookkeeping.
"""
import json
import math
import time

from openpilot.common.swaglog import cloudlog

MPH_TO_MS = 0.44704
MILE_M = 1609.344

# police
POLICE_MARGIN = 5.0 * MPH_TO_MS          # target = posted limit + 5 mph
POLICE_ENGAGE_S = 30.0                   # LATCH on once the report is <= ~30 s of driving ahead
# NO no-limit fallback (driver directive 2026-07-15, city 15-mph hold): without a posted limit there is
# no basis to pick a target — a guessed trim (old: anchor - 10 mph) capped the car to 15 mph in city
# traffic when the map limit dropped out mid-approach. No limit -> no cap.
# limit-drop
MIN_DROP_FRAC = 0.95                     # ignore < 5% limit dips (GPS/OSM noise) — no cap
SANE_MAX_SL = 90.0 * MPH_TO_MS           # reject implausible posted limits (>90 mph) as garbage
# shared
MIN_CAP = 10.0 * MPH_TO_MS               # never auto-slow below ~10 mph via this feature (garbage reject)
READ_S = 1.0                             # param read cadence
# smoothness (driver directive 2026-07-15, the "wild horse" city ride): the map limit flickered
# valid<->unknown at ~1 Hz, so the cap target STEPPED between two values every second and the truck
# surged/braked chasing it. Three guards make the cap a smooth ceiling instead of a square wave:
SL_HOLD_S = 5.0                          # hold the last valid posted limit through brief map dropouts
SL_DROP_CONFIRM_S = 2.0                  # speedlimitconfirm2pnw: a LOWER posted limit must persist this
                                         # long before it is acted on. mapd re-matches position after a
                                         # restart / GPS re-acquire and can briefly land on a parallel
                                         # surface street; with AutoSpeedReduce=2 that bogus limit feeds
                                         # _limit_drop_cap() directly, which caps at max(sl, sl*ratio) --
                                         # a transient 25 mph read on I-5 would command ~29 mph at 70.
                                         # Increases are still accepted immediately (release direction).
SL_DROP_EPS = 0.1                        # m/s tolerance when matching the pending value across reads
# sazoneset2pnw (Fable review of sanorestore2pnw, measured): the drop confirm above gated only a LOWER
# reading, so a single HIGHER read passed straight through, re-anchored the baseline, and the return to
# the real limit then CONFIRMED as a "drop" 2 s later -- set 66 in a 60, one 65 read, trimmed to 61 with
# the police restore withheld. Under the zone model a confirmed drop is a PERMANENT set change (no
# restore, ever), so a manufactured drop now costs the driver his speed until he taps back up. A higher
# limit must therefore persist too. Same asymmetry and the same 3 s as icbm_restore_limit's rise hold
# (ces_pnw_constants.ICBM_RESTORE_LIMIT_RISE_HOLD_S): flicker and a real 5 mph step are the same size,
# only persistence separates them. Kept LOCAL, like the other mirrors in this file (no cross-feature import).
SL_RISE_CONFIRM_S = 3.0
# Persistence is WALL TIME, and an unknown read does not interrupt it (Fable review, measured: resetting on unknown
# made a rise need 4 consecutive valid reads, and with the ~1 Hz valid<->unknown flicker above the Tesla stayed
# capped at a 35-zone trim on a 60 road indefinitely). The persisted time also counts toward RELEASE_S (see cap()).
# sazoneset2pnw (driver directive 2026-09-13): a limit-drop slowdown is a ONE-SHOT zone set. The episode
# ends -- and this module goes silent -- once the truck's own reported set has reached the zone target.
ZONE_SET_DONE_TOL = 0.6 * 1.0 * MPH_TO_MS      # == the executor's DEADBAND_MS (0.6 of a 1 mph tap)
ZONE_SET_TIMEOUT_S = 60.0                # s of ACC-engaged, pedal-free time the zone set may take before it is
                                         # ABANDONED (logged loudly). A 70 -> 38 mph zone set is ~15 s of slew
                                         # plus taps; 60 s only ever trips on a genuine failure to actuate.
CAP_SLEW = 1.0                           # m/s per s — emitted cap RAMPS toward its target, never steps
RELEASE_S = 2.0                          # cap sources must stay clear this long before the cap releases
# speedadjust-exec2pnw: the stock-ACC button-management publish (mem-param only; never touches the
# op-long return value)
PUB_THROTTLE_S = 0.25                    # publish cadence — matches icbm2pnw's IcbmTarget cadence,
                                          # well inside the executor's STALE_LIMIT_S=2.0
RESTORE_WINDOW_S = 45.0                  # bounded SET+ walk-back window after a cap clears — matches
                                          # icbm2pnw's own restore-episode window convention
# restore2pnw-hardening (2026-08, Gemini + Fable review): the restore (SET+) path must never command
# the truck above the driver's OWN current live stock set. These mirror ces_pnw.IcbmEpisode's own
# guards (see its docstring / ~line 596-693) rather than reinventing them:
SA_STEP_MS = 1.0 * MPH_TO_MS              # one Ford SET tap (mirrors opendbc/car/ford/icbm_pnw.STEP_MS)
SA_DRIVER_LOWER_TOL = 1.7 * SA_STEP_MS    # mirrors ces_pnw.ICBM_DRIVER_LOWER_TOL: a live stock set this
                                           # far below anything WE'VE ever commanded this cap is the
                                           # driver's own SET-, not our own tap lag/latency
SA_IN_CURVE_LAT_ACCEL = 1.3               # m/s^2; mirrors ces_pnw.ces_pnw_constants.CURVE_LAT_ACCEL_EXIT
                                           # (the "curve considered done" hysteresis used by
                                           # ces_pnw.icbm_in_curve's measured-now test) — kept as a LOCAL
                                           # constant rather than an import so this module stays fully
                                           # decoupled from ces_pnw (no cross-feature coupling)

# speedadjustreset2pnw (driver directive 2026-08-16): ANY manual cruise-set change (up OR down) is an
# explicit "resume — stop slowing me for THIS" override. Nudging the physical set dial is a far more
# intuitive override for a novice driver than hunting down the AutoSpeedReduce toggle. 0.15 m/s sits
# below the smallest real driver increment (1 kph = 0.278 m/s, 1 mph = 0.447 m/s) but comfortably above
# float noise, so it can't misfire on a no-op re-read of the same set speed.
SET_CHANGE_EPS = 0.15                     # m/s
# speedadjustreset2pnw hardening FIX B (2026-08, Gemini + Fable review pass 2): matches
# opendbc/car/ford/icbm_pnw.STALE_LIMIT_S (see PUB_THROTTLE_S's comment above — kept as a LOCAL
# constant, no cross-feature import, same rationale as SA_IN_CURVE_LAT_ACCEL). An in-flight SET+/SET-
# tap can land up to this long AFTER _cap_out/_restore_ceiling null at a phase boundary (cap releases,
# restore expires/cancels, a new cap preempts a restore) — stock-ACC set-change detection is
# suppressed for this long after any such transition so a late-landing tap of OUR OWN can't be
# misread as a driver override.
SA_ACTUATION_GRACE_S = 2.0                # s; stock-ACC only
# sazoneset2pnw (Fable review, measured): the CURVE brain (ces_pnw ICBM -> IcbmTarget) taps the same stock set.
# Its SET- taps used to read here as driver overrides -- re-anchoring the zone ratio to the tapped-down set, so
# a curve overlapping a zone entry left the truck at 75 in a 45 zone. We read the IcbmTarget mem-param the Ford
# executor reads; a command older than the executor's STALE_LIMIT_S is not being executed. LOCAL mirror, as above.
SA_ICBM_FRESH_S = 2.0
# policer2pnw (Rule 2): an unreadable police input is logged -- the first failure at once, then at most one line per
# this many seconds, each counting the failed reads since the previous line.
# silentexc2pnw: the AutoSpeedReduce read and the SpeedAdjustTarget publishes log on the same interval, each with its
# own first-failure/count state.
# silentexc3pnw: so does the MapSpeedLimit read in _read_speed_limit.
POLICE_READ_ERR_LOG_S = 60.0

# limitahead2pnw (owner 2026-09-24, "just do 1 and 2 -- the higher the speed the sooner you need to start slowing
# down"). drives/2026-09-24/limit-drop-1857: at a 60 -> 40 boundary the truck was still at 74 mph, because this module
# only acted on the CURRENT limit, 3 s after the sign; mapd had announced the 40 1,078 m ahead.
#   1. LOOK-AHEAD: once the distance to an announced lower limit (mapd nextSpeedLimit, bridged as the
#      NextMapSpeedLimit mem-param) is within la_start_distance(), slow toward the rule-1/1b target for that limit.
#      The slowdown is RESTORABLE until the lower limit is current and has held LA_PROMOTE_HOLD_S; only then does it
#      hand over to the permanent zone set (sazoneset2pnw). If the drop never comes -- the boundary passes, or the
#      announcement vanishes or rises, or the new limit reverts (the 19:02 road not taken) -- the driver's previous set
#      is restored (owner-approved exception to "a limit drop never restores"; a look-ahead is not a limit drop).
#   2. An ANNOUNCED drop skips SL_DROP_CONFIRM_S: the reading that matches the active look-ahead is taken at once.
#      Unannounced drops keep the 2 s confirm.
# LimitAheadMode: 0 off, 1 SHADOW (default: logs every decision, changes nothing), 2 live.
LA_OFF, LA_SHADOW, LA_LIVE = 0, 1, 2
LA_A_PLAN = 0.6       # m/s^2 -- the rate the Lightning's stock ACC actually sheds speed while it is above a tapped-down
                      # set. Measured over all 16 routes of 2026-09-24 (291 qlogs, 7 episodes with ACC on, no pedals,
                      # vEgo >= set + 3 mph for >= 3 s): p25 0.33, median 0.56, p75 0.90 m/s^2 (min aEgo -0.7..-1.8).
                      # The set itself cannot fall faster than CAP_SLEW (1 m/s^2), so 0.6 is the median follow rate,
                      # not a bound. 74 -> 50 mph at 0.6: 496 m of decel (the report's 0.75-1.0 gave 350-450 m).
LA_T_LEAD = 3.0       # s of travel added as margin: up to 1 s for the 1 Hz read, ~0.4 s per tap, and the ~1-1.5 s it
                      # takes the ACC to start following a lowered set (lag-after-last-tap 0.2-1.5 s, same logs).
                      # 74 -> 50 mph: 496 + 99 = 595 m, i.e. 18 s before the boundary at 74 mph.
LA_STEADY_S = 2.0     # an announcement must be seen this long (the same value, >= 2 reads) before a look-ahead starts
LA_PROMOTE_HOLD_S = 8.0  # the lower limit must be CURRENT this long before the look-ahead becomes a permanent zone set.
                         # 19:02:26 PT the same day, mapd matched onto the road not taken (a 25 on a 50 ramp) for
                         # 4.95 s; read at 1 Hz that is 4-6 s. 8 s clears it with margin; the truck is already slowing
                         # meanwhile, so the hold costs no distance -- it only decides whether the slowdown is permanent.
LA_PASS_MARGIN_S = 3.0   # s of travel past the expected boundary before "the drop never came" (read + map-match lag)
LA_PASS_MARGIN_MIN_M = 30.0
LA_GONE_GRACE_S = 2.0    # an announcement that vanished / rose, or a limit that went back up, must stay that way this long
                         # before the look-ahead aborts (one flickering read must not cost a restore and a re-slow)
LA_INPUT_STALE_S = 2.0   # a NextMapSpeedLimit older than this reads as "no announcement" (dead mapd bridge)
LA_SAME_ANN_M = 100.0    # an announced boundary that moves by more than this is a different road, not the same drop


def la_start_distance(v_ego: float, v_tgt: float) -> float:
  """limitahead2pnw: how far ahead of a lower limit the look-ahead starts (m) -- decel distance at LA_A_PLAN plus
  LA_T_LEAD s of travel. Grows with speed (owner: "the higher the speed the sooner"); at or below the target only
  the lead margin remains."""
  v = max(float(v_ego), 0.0)
  vt = max(float(v_tgt), 0.0)
  return max(v * v - vt * vt, 0.0) / (2.0 * LA_A_PLAN) + v * LA_T_LEAD


def _police_key(rep):
  """Stable identity for one police report: its uuid, else the quantized position the daemon falls
  back to. Returns None only for a non-dict/empty report, and a None key never matches another None
  (see _police_cap) so a missing identity can never carry a dismissal onto a different report."""
  if not isinstance(rep, dict):
    return None
  return rep.get("key") or rep.get("uuid") or None


class SpeedAdjustController:
  def __init__(self, CP, params=None):
    import platform
    self.CP = CP
    if params is not None:
      self.params = params
    else:
      from openpilot.common.params import Params
      self.params = Params()
    try:
      from openpilot.common.params import Params as _P
      self.mem_params = _P("/dev/shm/params") if platform.system() != "Darwin" else self.params
    except Exception:
      self.mem_params = None
    self._long_ok = bool(getattr(CP, "openpilotLongitudinalControl", False))
    self._mode = 0
    self._mode_err_t = None      # silentexc2pnw: monotonic time of the last logged AutoSpeedReduce read failure (None = never)
    self._mode_err_n = 0         # silentexc2pnw: failed AutoSpeedReduce reads since that log line
    self._sl = 0.0                # current posted limit (m/s); 0 = unknown
    self._sl_err_t = None         # silentexc3pnw: monotonic time of the last logged MapSpeedLimit read failure (None = never)
    self._sl_err_n = 0            # silentexc3pnw: failed MapSpeedLimit reads since that log line
    self._sl_valid_t = -1e9       # monotonic time of the last VALID limit read (for the dropout hold)
    self._sl_pending = 0.0        # a LOWER limit awaiting SL_DROP_CONFIRM_S confirmation (0 = none)
    self._sl_pending_t = 0.0      # monotonic stamp of when that pending value was first seen
    self._sl_rise_pending = 0.0   # sazoneset2pnw: a HIGHER limit awaiting SL_RISE_CONFIRM_S (0 = none)
    self._sl_rise_pending_t = 0.0
    self._sl_rise_since = None    # first sighting of a rise adopted on THIS read (consumed the same tick)
    self._sl_ref = 0.0            # baseline limit (the limit we were last uncapped at)
    self._ratio = 0.0            # ANCHORED over-limit ratio (v_set/limit) captured at the baseline —
                                 #   NOT live v_cruise, so re-scrolling the set can't double-reduce
    self._police = None          # last LocationServices["police"] dict
    self._police_err_t = None    # policer2pnw: monotonic time of the last logged police-read failure (None = never)
    self._police_err_n = 0       # policer2pnw: failed police reads since that log line
    self._police_latched = False # once within the approach window, hold until the report clears
    # policelatch2pnw: WHICH report the latch belongs to. The latch means "I am approaching THIS
    # report"; it must not survive onto a different one. Without this the cap carried across reports:
    # pass report A, _PoliceRecede drops it, `cap` switches to report B miles ahead, and because
    # _police_latched was still True the POLICE_ENGAGE_S approach gate was skipped and the car stayed
    # capped at limit+5 indefinitely — on a corridor where some confirmed report is almost always in
    # range, the speed never resumed (observed 2026-08-21, cap report 8.3 mi out while capped).
    self._police_latched_key = None
    # speedadjustreset2pnw: True after a detected manual set-change dismisses the CURRENT police
    # alert — suppresses _police_cap() (so it can't simply re-latch next tick) until the report
    # clears (state != "alert"), at which point it clears alongside _police_latched and the feature
    # re-arms for the NEXT alert. See _is_own_actuation()/cap() for why this exists.
    self._police_suppressed = False
    self._police_suppressed_uuid = None   # which report the driver dismissed (see _police_cap)
    self._ovr = "init"                    # satele2pnw: last manual-override verdict (see cap())
    self._sa_pub_t = -1e9                 # satele2pnw: SpeedAdjustStatus publish throttle
    self._sa_pub_warned = False           # satele2pnw: one-shot publish-failure log latch
    # speedadjustreset2pnw: last observed driver v_cruise_set (m/s), used to detect a manual cruise
    # nudge (see cap()). None = not yet tracked — also forced back to None whenever
    # v_cruise_initialized is False OR ACC is not engaged (FIX A) so the uninitialized->initialized
    # transition, the ~145 km/h V_CRUISE_UNSET sentinel, AND an engage transition (Tesla
    # cruiseState.speed's 1e-3 floor) never count as a "change".
    self._last_v_set = None
    # speedadjustreset2pnw hardening FIX B: monotonic time of the last _cap_out/_restore_ceiling
    # None-ness transition (cap released, restore ended/preempted) — see SA_ACTUATION_GRACE_S.
    self._last_actuation_transition_t = -1e9
    self._last_read = -1e9
    self._engaged = False        # for engage/release logging only
    self._cap_out = None         # the SLEWED cap currently emitted (None = not capping)
    self._release_t = None       # when the cap sources first went clear (release debounce)
    self._last_t = None          # last cap() call time (for slew dt)
    # speedadjust-exec2pnw: stock-ACC button-management publish state (SpeedAdjustTarget mem-param).
    # Inert / never touched on any op-long car (self._long_ok True) -- see _publish_target().
    self._pub_ceiling = None     # driver's set latched at cap ENGAGE (icbm2pnw ceiling parity)
    self._restore_ceiling = None # active bounded-restore target (None = no restore in progress)
    # sanorestore2pnw: did a LIMIT-DROP cap take part in the current episode? A released episode that
    # involved one never restores (see the release branch in cap()). Reset at every episode engage.
    self._ep_limit_drop = False
    self._no_restore_why = None  # telemetry: why the last release did NOT open a restore (None = it did / n.a.)
    # sazoneset2pnw: the zone target of the current limit-drop-only episode (m/s, None = none), and how long
    # it has been actuatable without being reached (see ZONE_SET_TIMEOUT_S).
    self._zone_target = None
    self._zone_elapsed = 0.0
    # ...and, for the curve brain, which zone episode this is and the target it had: ICBM bounds a restore by a
    # zone that was in progress or BEGAN during its episode, including one that opened and completed between two
    # of its ~1 Hz status reads (Fable review). Published as zoneN / zoneLast.
    self._zone_n = 0
    self._zone_last = None
    # sazoneset2pnw: the curve brain's live command (stock ACC only; see SA_ICBM_FRESH_S)
    self._icbm_cmd = None
    self._icbm_read_t = -1e9
    self._icbm_read_warned = False
    self._icbm_dec_t = -1e9      # monotonic time a fresh ICBM dec was last seen on the bus
    self._icbm_inc_t = -1e9      # zonefollow2pnw: ...and a fresh ICBM inc (its restore)
    self._inst = round(time.monotonic(), 3)  # zonefollow2pnw: identifies this process start (ICBM detects a restart)
    self._icbm_hold = False      # the set is lowered by ICBM: the limit-drop ratio keeps the pre-curve set
    self._restore_deadline = None  # monotonic deadline for the bounded restore window
    self._min_pub_target = None  # restore-hardening #1: running MIN of _cap_out published this cap
                                  # episode — the "explainability floor" (mirrors ces_pnw's
                                  # IcbmEpisode._min_target): a live stock set below this by more than
                                  # SA_DRIVER_LOWER_TOL is the driver's OWN doing, not ours
    self._restore_last_stock = None  # restore-hardening #1: last observed live stock set DURING an
                                      # active restore (mirrors icbm_pnw.RestoreGuard) — a DECREASE
                                      # relative to this, not merely "below the ceiling", ratchets the
                                      # restore ceiling down
    self._pub_last = -1e9        # publish throttle (monotonic)
    self._pub_active = False     # was the last SpeedAdjustTarget publish non-idle (need one more
                                  # publish to clear it to {} on the transition to idle)
    self._pub_err_t = None       # silentexc2pnw: monotonic time of the last logged SpeedAdjustTarget target-publish failure
    self._pub_err_n = 0          # silentexc2pnw: failed target publishes since that log line
    self._clear_err_t = None     # silentexc2pnw: ...and the same for the {} clear publish
    self._clear_err_n = 0
    # limitahead2pnw (see LA_* above)
    self._la_mode = LA_OFF       # LimitAheadMode, read at READ_S (Off until the first read)
    self._la_mode_err_t = None
    self._la_mode_err_n = 0
    self._next_err_t = None      # NextMapSpeedLimit read failures, logged like the police input
    self._next_err_n = 0
    self._next_raw = None        # last NextMapSpeedLimit reading: (limit m/s, distance m) or None
    self._sl_raw = 0.0           # the RAW MapSpeedLimit reading (no hold, no confirm); 0 = unknown
    self._odo = 0.0              # m travelled (integral of v_ego), the frame announced boundaries are kept in
    self._ann = None             # the lower limit announced ahead: {"n", "b" (boundary odo), "since"} or None
    self._la = None              # the active look-ahead episode (dict, see _la_step) or None
    self._la_hold = False        # live: the limit-drop ratio keeps the pre-look-ahead set (our taps are not the driver's)
    self._la_dismissed = None    # the announced limit (m/s) of an aborted episode: not restarted until that announcement goes
    self._la_why = None          # telemetry: how the last episode ended ("promote" / "abort:<reason>")
    self._la_ev_n = 0            # telemetry: look-ahead decisions logged so far (each one is a cloudlog event)

  # ---- input reads (params only; ~1 Hz) -------------------------------------
  def _read_speed_limit(self) -> float:
    """Current posted limit (m/s), 0 = unknown. A limit that was valid within the last SL_HOLD_S is
    HELD through read dropouts — the map limit flickering valid<->unknown at ~1 Hz turned the cap
    target into a square wave (the "wild horse" ride). Brief unknown = continuity, not a change."""
    sl = 0.0
    if self.mem_params is not None:
      try:
        raw = self.mem_params.get("MapSpeedLimit", return_default=True)
        raw = raw.decode() if isinstance(raw, bytes) else raw
        sl = float(raw) if raw else 0.0
      except Exception as e:
        # silentexc3pnw (Rule 2): was `except Exception: sl = 0.0` with no log. Fallback unchanged: the limit reads as
        # unknown, so the last valid limit is held for SL_HOLD_S and then BOTH slowdowns stop (the limit-drop cap and
        # the police cap each need a posted limit), on both cars. An unset key is not a failure (None -> 0.0 above,
        # no exception). What raises: UnknownKeyName on a params_keys.h / params_pyx.so mismatch, or float() of a
        # non-numeric string. Logged in the policer2pnw style, own state. Caught broadly: plannerd is
        # restart_if_crash=False.
        sl = 0.0
        self._sl_err_n += 1
        now = time.monotonic()
        if self._sl_err_t is None or now - self._sl_err_t >= POLICE_READ_ERR_LOG_S:
          cloudlog.exception(f"speedadjust: MapSpeedLimit unreadable ({type(e).__name__}) -- the limit reads as unknown: " +
                             f"the last one is held up to {SL_HOLD_S:.0f} s, then NO limit or police slowdown while this " +
                             f"lasts ({self._sl_err_n} failed read(s) since the last log)")
          self._sl_err_t = now
          self._sl_err_n = 0
      if not math.isfinite(sl) or sl <= 0.0 or sl > SANE_MAX_SL:   # reject unknown / NaN / garbage-high
        sl = 0.0
    now = time.monotonic()
    self._sl_raw = sl                          # limitahead2pnw: the look-ahead judges the drop on the raw reading
    if sl > 0.0:
      self._sl_valid_t = now
      # speedlimitconfirm2pnw: hold the previous limit until a DECREASE has persisted for
      # SL_DROP_CONFIRM_S. Only the falling direction is gated -- a rising limit releases the cap and
      # is accepted at once. A pending value that stops matching (the reading moved on) is discarded
      # without ever having been acted on, which is exactly the mapd re-match transient we're after.
      # limitdropexact2pnw (Fable F1): after a >SL_HOLD_S dropout self._sl is 0 but _sl_ref survives, so a first
      # reading below the baseline must wait for the same confirm window -- else ONE bogus low read would act
      # at once (and since rule 1b, also on a driver at/under the old limit).
      ref = self._sl if self._sl > 0.0 else self._sl_ref
      if 0.0 < sl < ref:
        self._sl_rise_pending = 0.0            # a lower reading ends any pending rise
        if abs(sl - self._sl_pending) > SL_DROP_EPS:
          self._sl_pending = sl
          self._sl_pending_t = now
        if now - self._sl_pending_t < SL_DROP_CONFIRM_S:
          # limitahead2pnw option 2: a drop the active look-ahead announced is taken at once -- mapd counted down to
          # it, and the look-ahead (not this value) decides whether it becomes permanent (LA_PROMOTE_HOLD_S).
          # Unannounced drops, and every drop in shadow mode, keep the confirm.
          ep = self._la
          if ep is None or abs(sl - ep["n"]) > SL_DROP_EPS:
            return self._sl                    # unconfirmed drop → keep the previous limit
          if not ep["skip_logged"]:
            ep["skip_logged"] = True
            self._la_log(ep, "confirmSkipped" if ep["live"] else "wouldSkipConfirm", limit=round(sl, 2))
          if not ep["live"]:
            return self._sl
      self._sl_pending = 0.0
      # sazoneset2pnw: a HIGHER limit must persist too (see SL_RISE_CONFIRM_S). A first reading after the
      # limit was genuinely unknown (self._sl == 0) is not a "rise" and is taken at once.
      if self._sl > 0.0 and sl > self._sl + SL_DROP_EPS:
        if abs(sl - self._sl_rise_pending) > SL_DROP_EPS:
          self._sl_rise_pending = sl
          self._sl_rise_pending_t = now
        if now - self._sl_rise_pending_t < SL_RISE_CONFIRM_S:
          return self._sl                      # unconfirmed rise → keep the previous limit
        self._sl_rise_since = self._sl_rise_pending_t
      self._sl_rise_pending = 0.0
      return sl
    # Unknown read: break a pending DROP's confirmation chain (a drop that flickers valid/unknown has not
    # "persisted"), then fall through to the existing brief-dropout hold. A pending RISE is deliberately kept:
    # only a valid read at or below the held limit cancels it (see SL_RISE_CONFIRM_S for why).
    self._sl_pending = 0.0
    if now - self._sl_valid_t < SL_HOLD_S and self._sl > 0.0:
      return self._sl                        # brief dropout → hold the last valid limit
    return 0.0

  def _read_police(self):
    if self.mem_params is None:
      return None
    try:
      raw = self.mem_params.get("LocationServices", return_default=True)
      if isinstance(raw, (bytes, str)):
        raw = json.loads(raw)
      if isinstance(raw, dict):
        p = raw.get("police")
        return p if isinstance(p, dict) else None
    except Exception as e:
      # policer2pnw (Rule 2): this was `except Exception: pass`. The fallback is unchanged -- no police report, so no
      # police slowdown -- but it is now logged. The common causes are malformed payloads (ValueError covers a bad JSON
      # string and a bad UTF-8 byte string).
      # Fable: catch broadly. An UnknownKeyName from a params_pyx.so/params_keys.h mismatch is none of those, and
      # plannerd is restart_if_crash=False: letting it escape would disengage both cars with no re-engage. The log
      # names the exception type so a build defect still stands out.
      self._police_err_n += 1
      now = time.monotonic()
      if self._police_err_t is None or now - self._police_err_t >= POLICE_READ_ERR_LOG_S:
        cloudlog.exception(f"speedadjust: LocationServices police input unreadable ({type(e).__name__}) -- NO police " +
                           f"slowdown can engage while this lasts ({self._police_err_n} failed read(s) since the last log)")
        self._police_err_t = now
        self._police_err_n = 0
    return None

  def _read_inputs(self):
    try:
      self._mode = int(self.params.get("AutoSpeedReduce", return_default=True) or 0)
    except Exception as e:
      # silentexc2pnw (Rule 2): this was `except Exception: self._mode = 0`, so an unreadable selector silently
      # switched police and limit slowdowns off on both cars. The fallback is unchanged (Off until a read succeeds,
      # ~1 s later) but it is now logged in the policer2pnw style. A malformed stored value does not reach here
      # (Params returns the default for it, with its own warning); an UnknownKeyName from a params_keys.h /
      # params_pyx.so mismatch does. Caught broadly: plannerd is restart_if_crash=False.
      self._mode = 0
      self._mode_err_n += 1
      now = time.monotonic()
      if self._mode_err_t is None or now - self._mode_err_t >= POLICE_READ_ERR_LOG_S:
        cloudlog.exception(f"speedadjust: AutoSpeedReduce unreadable ({type(e).__name__}) -- mode forced Off, NO police " +
                           f"or limit slowdown while this lasts ({self._mode_err_n} failed read(s) since the last log)")
        self._mode_err_t = now
        self._mode_err_n = 0
    self._sl = self._read_speed_limit()
    self._police = self._read_police()
    self._la_mode = self._read_la_mode()
    self._next_raw = self._read_next_limit()
    self._update_announcement()

  # ---- limitahead2pnw: inputs ------------------------------------------------
  def _read_la_mode(self) -> int:
    """LimitAheadMode (0 off / 1 shadow / 2 live). Unreadable or out of range -> OFF, logged (Rule 2): the look-ahead
    then neither acts nor logs, and the line says why."""
    try:
      m = int(self.params.get("LimitAheadMode", return_default=True) or 0)
      if m in (LA_OFF, LA_SHADOW, LA_LIVE):
        return m
      err = f"out of range ({m})"
    except Exception as e:
      err = type(e).__name__
    self._la_mode_err_n += 1
    now = time.monotonic()
    if self._la_mode_err_t is None or now - self._la_mode_err_t >= POLICE_READ_ERR_LOG_S:
      cloudlog.error(f"speedadjust: LimitAheadMode unreadable ({err}) -- limit look-ahead OFF, no slowdown ahead of " +
                     f"an announced lower limit while this lasts ({self._la_mode_err_n} failed read(s) since the last log)")
      self._la_mode_err_t = now
      self._la_mode_err_n = 0
    return LA_OFF

  def _read_next_limit(self):
    """(limit m/s, distance m) of the limit mapd announces ahead, or None for none / stale / unreadable. A stale or
    unreadable value is logged in the policer2pnw style: it means NO look-ahead, and an active one aborts."""
    if self.mem_params is None:
      return None
    err = None
    try:
      raw = self.mem_params.get("NextMapSpeedLimit", return_default=True)
      if isinstance(raw, (bytes, str)):
        raw = json.loads(raw) if raw else None
      if raw is None:
        return None                            # never written yet (mapd not up): not a failure
      n, d, ts = float(raw["sl"]), float(raw["d"]), float(raw["ts"])
      age = time.monotonic() - ts
      if not (math.isfinite(n) and math.isfinite(d) and math.isfinite(age)):
        err = "non-finite value"
      elif age > LA_INPUT_STALE_S:
        err = f"stale ({age:.1f} s old)"
      elif n <= 0.0 or d <= 0.0 or n > SANE_MAX_SL:
        return None                            # fresh "nothing announced"
      else:
        return n, d
    except Exception as e:
      err = type(e).__name__
    self._next_err_n += 1
    now = time.monotonic()
    if self._next_err_t is None or now - self._next_err_t >= POLICE_READ_ERR_LOG_S:
      cloudlog.error(f"speedadjust: NextMapSpeedLimit unusable ({err}) -- no limit look-ahead while this lasts " +
                     f"({self._next_err_n} bad read(s) since the last log)")
      self._next_err_t = now
      self._next_err_n = 0
    return None

  def _update_announcement(self) -> None:
    """Track the LOWER limit announced ahead (vs the limit held now) and how long the same value has been announced.
    The boundary is kept as an odometer position, so between the 1 Hz reads its distance is dead-reckoned."""
    nxt, now = self._next_raw, time.monotonic()
    ref = self._sl if self._sl > 0.0 else self._sl_ref
    if nxt is None or ref <= 0.0 or nxt[0] / ref > MIN_DROP_FRAC:
      self._ann = None                         # nothing announced, or a rise / < 5 % dip: nothing to do ahead
      return
    n, d = nxt
    if self._ann is None or abs(self._ann["n"] - n) > SL_DROP_EPS or abs(self._ann["b"] - (self._odo + d)) > LA_SAME_ANN_M:
      self._ann = {"n": n, "b": self._odo + d, "since": now}
    else:
      self._ann["b"] = self._odo + d

  # ---- the two reduce-only sources ------------------------------------------
  def _police_cap(self, v_cruise: float, v_ego: float):
    """Target = posted limit + 5 mph. LATCHES on once the report is ~30 s of driving ahead and HOLDS
    until the report clears — so slowing down (which balloons time-to-report) can't drop the cap and
    surge you back up. Returns the target cruise (m/s) or None. NO posted limit → NO cap (driver
    directive 2026-07-15): without a limit there is no basis for a target — the old guessed trim
    capped the car to 15 mph in city traffic. The latch is kept independent of limit availability,
    so if the limit becomes known mid-approach the cap engages then."""
    p = self._police
    if not isinstance(p, dict) or p.get("state") != "alert":
      self._police_latched = False           # cleared/passed report → release
      self._police_latched_key = None
      self._police_suppressed = False        # speedadjustreset2pnw: report gone → re-arm for the next one
      return None
    # policetier2pnw (driver directive 2026-08-18: "slow down only for the ones that show up on Waze").
    # Act on the CONTROL channel, never the displayed report: location_servicesd publishes `cap` = the
    # nearest CONFIRMED report, separately from the line on screen (which is proximity-first and may be
    # an amber, display-only one). Reading the display fields here is what let a nearer stale report
    # suppress a live one, and what would have let the two channels flap each other.
    # A payload with NO `tier` key predates this split: fall back to the whole line, i.e. the previous
    # behaviour, so an older location_servicesd can never silently disable police braking.
    src = p
    if "tier" in p:
      src = p.get("cap")
      if not isinstance(src, dict):
        self._police_latched = False           # nothing confirmed ahead → release, don't hold a cap
        self._police_latched_key = None
        self._police_suppressed = False        # ...and nothing left to be dismissing → re-arm
        self._police_suppressed_uuid = None
        return None
      # A dismissal applies to the report the driver dismissed, not to the police line as a whole.
      # The line now stays "alert" for as long as ANY report (including a display-only amber) is in
      # range, so clearing suppression only on state != "alert" could carry a dismissal of report A
      # across an entire corridor and silently swallow the slowdown for a later, different report C.
      # Key on `key` (uuid, else quantized position -- the same fallback _PoliceRecede._key uses).
      # Keying on uuid alone meant two reports that both lack an alert_id keyed None == None, and the
      # second silently inherited the first's dismissal and lost its slowdown.
      _k, _sk = _police_key(src), self._police_suppressed_uuid
      if self._police_suppressed and (_k is None or _sk is None or _k != _sk):
        self._police_suppressed = False
        self._police_suppressed_uuid = None
        # ...and drop the latch with it: it was latched for the report just dismissed. Carrying it on
        # to a DIFFERENT report skips the POLICE_ENGAGE_S approach gate and caps immediately for a
        # report that may still be miles away.
        self._police_latched = False
    # policelatch2pnw: the latch belongs to ONE report. If the CONTROL report is no longer the one we
    # latched onto — we passed it and `cap` moved to the next confirmed report — drop the latch so the
    # new report has to earn its own cap through the POLICE_ENGAGE_S approach gate. This is the same
    # reasoning the dismissal branch above applies, which previously only ran when the driver had
    # manually dismissed something; without it the cap never released. An unresolvable key (None)
    # deliberately drops the latch too: fail toward resuming the driver's speed, never toward holding
    # a cap we can no longer attribute to a specific report.
    _lk = _police_key(src)
    if self._police_latched and (_lk is None or self._police_latched_key is None
                                 or _lk != self._police_latched_key):
      self._police_latched = False
      self._police_latched_key = None
    if self._police_suppressed:              # speedadjustreset2pnw: driver dismissed THIS alert via a
      return None                            # manual set change — stay off until it clears (see cap())
    try:
      dist_m = float(src.get("dist_mi", 0.0)) * MILE_M
    except (TypeError, ValueError):
      return None
    if not math.isfinite(dist_m) or dist_m <= 0.0:   # NaN/inf/garbage distance → don't act, don't latch
      return None
    ttr = dist_m / max(v_ego, 1.0)           # time-to-report at current speed
    if ttr <= POLICE_ENGAGE_S and not self._police_latched:
      self._police_latched = True            # entered the approach window → latch on
      self._police_latched_key = _lk         # policelatch2pnw: ...for THIS report only
    if not self._police_latched:
      return None                            # not close enough yet — hold current speed
    if self._sl > 0.0:
      return self._sl + POLICE_MARGIN
    return None                              # no posted limit → no basis to pick a target → no cap

  def _update_baseline(self, v_cruise_set: float):
    """speedanchor2pnw (F3): keep the limit-drop baseline (_sl_ref/_ratio) current whenever the feature
    is ACTIVE (mode 1 or 2) — not just inside _limit_drop_cap(), which only ran in mode 2. Previously a
    police-only (mode 1) drive left _sl_ref/_ratio frozen; flipping AutoSpeedReduce 1→2 mid-drive then
    resumed off a baseline captured possibly hours earlier, producing a surprise cap on the switch tick.

    speedanchor2pnw (F2): anchors off `v_cruise_set` — the driver's raw, PRE-VTSC cruise set — not a
    VTSC-curve-reduced `v_cruise`. A curve happening to be active at the moment of re-anchor must not
    record a bogus-low ratio (which can silently disable the trim on the next real limit drop)."""
    sl = self._sl
    if sl > 0.0 and sl >= self._sl_ref:      # known limit, at/above baseline → uncapped; track it up
      if (self._icbm_hold or self._la_hold) and self._sl_ref > 0.0:   # limitahead2pnw: same for our look-ahead taps
        # sazoneset2pnw: a curve has the set tapped down -- that is not the driver's speed. Keep the driver's
        # pre-curve set as the reference (only rescaled to the new baseline limit), so a zone entered during
        # the curve still gets "the same percentage as I was driving before".
        self._ratio *= self._sl_ref / sl
      else:
        self._ratio = v_cruise_set / sl      # anchor the over-limit ratio HERE (v_set/limit)
      self._sl_ref = sl

  def _limit_drop_cap(self):
    """Trim the driver's OVER-limit excess as the posted limit drops: cap = SL × (v_set/SL_ref), using
    the ratio anchored by _update_baseline() (already refreshed this tick, before this is called).
    Persistent while below the baseline; releases once _update_baseline() re-anchors on a rising limit.
    m/s or None.

    Owner rules (2026-09-13 / 2026-09-24): over the old limit -> the SAME PERCENTAGE over the new limit; AT or
    UNDER the old limit -> EXACTLY the new limit. NEVER below the posted limit (max floor) -- that floor is what
    fixed the 2026-07-14 Corvallis bug (30 in a 45 scaled to 16 mph). The 07-14 ratio<1 guard is gone
    (limitdropexact2pnw): it also stopped slowing a driver who was under the old but OVER the new limit."""
    sl = self._sl
    if sl <= 0.0:
      return None                            # unknown limit → no cap, preserve the baseline + ratio
    if sl >= self._sl_ref:
      return None                            # at/above baseline → uncapped (already re-anchored above)
    if sl / self._sl_ref > MIN_DROP_FRAC:    # < 5% drop → noise
      return None
    # limitdropexact2pnw (owner rule 1b, 2026-09-24): a driver AT or UNDER the old limit (ratio <= 1) is slowed to
    # EXACTLY the new limit. The 07-14 guard `if ratio < 1.0: return None` left the truck at 41 mph in a 25 zone
    # (2026-09-24 09:18 PT). The Corvallis bug it fixed (30 in a 45 scaled to ~16 mph) stays fixed by the floor
    # below, and a set already at/below the new limit is untouched because the cap is reduce-only.
    return max(sl, sl * self._ratio)         # over the limit: same % over the new limit; at/under: exactly the limit

  # ---- limitahead2pnw: the look-ahead episode --------------------------------
  def _la_log(self, ep, action: str, **kw) -> None:
    """Rule 2: every look-ahead decision is a cloudlog event (and bumps laEvN, forwarded into ces_events)."""
    self._la_ev_n += 1
    fields = {k: (round(float(v), 2) if isinstance(v, (int, float)) and not isinstance(v, bool) else v) for k, v in kw.items()}
    cloudlog.event("speedadjust_lookahead", action=action, live=bool(ep["live"]), announced=round(float(ep["n"]), 2),
                   target=round(float(ep["tgt"]), 2), **fields)

  def _la_end(self, now: float, why) -> None:
    """End the episode. why None = PROMOTE (the lower limit is current and held: the permanent zone set takes over).
    Otherwise ABORT: the look-ahead cap goes away, so the cap releases and the ordinary bounded restore walks the set
    back to the ceiling latched when this cap engaged -- the driver's set before the look-ahead, never higher -- unless
    a real limit drop joined the episode meanwhile (_ep_limit_drop: then nothing is restored, sanorestore2pnw)."""
    ep, self._la = self._la, None
    if why is None:
      self._la_why = "promote"
      self._la_hold = False                  # the limit is now below the baseline: the ratio is no longer re-anchored
      self._la_log(ep, "promote", limit=self._sl_raw, held_s=now - ep["hold_t"])
      return
    self._la_why = "abort:" + why
    if why not in ("modeOff", "modeChanged", "cruiseUnset"):
      # Not restarted for the same announcement: an aborted look-ahead restarting on the next tick would tap the set
      # down and back up in a loop (a mapd still announcing a limit it has passed, or a frozen distance). Cleared once
      # mapd announces something else or nothing.
      self._la_dismissed = ep["n"]
    if ep["live"] and why == "limitRose" and self._sl_raw > self._sl:
      # We took the lower limit WITHOUT the confirm (option 2). Give it back the same way: kept, the ordinary limit-drop
      # path would turn the bogus value into a permanent zone set the moment the look-ahead stops standing in for it
      # -- exactly the 19:02 road-not-taken case (a 25 on a 50 ramp).
      self._sl = self._sl_raw
      self._sl_rise_pending = 0.0
    # What the ordinary restore will walk back to (live), or would (shadow: the set before the look-ahead -- the shadow
    # controller's own ceiling belongs to whatever the real limit-drop path did, so it is not the look-ahead's answer).
    if ep["live"]:
      restore = None if getattr(self, "_ep_limit_drop", False) else self._pub_ceiling
    else:
      restore = ep["pre"]
    self._la_log(ep, "abort", reason=why, restore=restore, pre=ep["pre"], dist=ep["b"] - self._odo,
                 limit=self._sl_raw, limitDropJoined=bool(ep["live"] and getattr(self, "_ep_limit_drop", False)))

  def _la_step(self, now: float, v_ego: float, v_cruise_set: float):
    """One tick of the look-ahead. Returns the cap to add (m/s) while a LIVE episode is running, else None; a shadow
    episode runs the identical state machine and logs, but never returns a cap."""
    ep, ann = self._la, self._ann
    if self._la_mode == LA_OFF or self._mode < 2:
      if ep is not None:
        self._la_end(now, "modeOff")
      return None
    live = self._la_mode == LA_LIVE
    if self._la_dismissed is not None and (ann is None or abs(ann["n"] - self._la_dismissed) > SL_DROP_EPS):
      self._la_dismissed = None              # the announcement an episode aborted on is gone
    if ep is None:
      if ann is None or now - ann["since"] < LA_STEADY_S or self._la_dismissed is not None:
        return None
      tgt = max(ann["n"], ann["n"] * self._ratio)   # rule 1 (same % over) / 1b (at or under -> exactly the limit)
      # "Nothing to slow for" is judged on the DRIVER's set: while a curve has it tapped down (_icbm_hold) that is the
      # pre-curve set the ratio holds, not the tapped one -- else the curve's restore climbs past the target before the
      # look-ahead starts and the two brains walk the set up and down again (measured in the closed-loop harness).
      v_ref = self._ratio * self._sl_ref if (self._icbm_hold and self._sl_ref > 0.0) else v_cruise_set
      if tgt >= v_ref - SL_DROP_EPS:
        return None                          # already at or under the target: nothing to slow for
      # The start distance is for the speed the truck is heading to: a truck below its set (a lead, a curve) returns
      # to the set -- and a curve's restore walks it there -- so vEgo alone would start the look-ahead far too late.
      v_plan = max(v_ego, v_ref)
      dist, d_start = ann["b"] - self._odo, la_start_distance(v_plan, tgt)
      if dist > d_start:
        return None
      ep = self._la = {"n": ann["n"], "tgt": tgt, "ratio": self._ratio, "pre": v_cruise_set, "b": ann["b"], "b0": ann["b"],
                       "d_start": d_start, "mat_t": None, "hold_t": None, "bad_t": None, "bad_why": None, "live": live,
                       "skip_logged": False}
      if live and not self._long_ok:
        self._la_hold = True                 # our own SET- taps must not re-anchor the driver's ratio
      self._la_log(ep, "start", dist=dist, d_start=d_start, v=v_ego, vPlan=v_plan, pre=v_cruise_set, limit=self._sl_raw)
      return tgt if live else None
    if ep["live"] != live:
      self._la_end(now, "modeChanged")
      return None
    # Chained drops (60 -> 40 -> 35): a still LOWER limit announced within its own start distance takes over the target.
    if ann is not None and ann["n"] < ep["n"] - SL_DROP_EPS and now - ann["since"] >= LA_STEADY_S:
      tgt2 = max(ann["n"], ann["n"] * ep["ratio"])
      d2 = la_start_distance(max(v_ego, ep["tgt"]), tgt2)
      if ann["b"] - self._odo <= d2:
        ep.update(n=ann["n"], tgt=tgt2, b=ann["b"], b0=ann["b"], d_start=d2, mat_t=None, hold_t=None, bad_t=None,
                  skip_logged=False)
        self._la_log(ep, "retarget", dist=ann["b"] - self._odo, d_start=d2, v=v_ego)
    raw = self._sl_raw
    if 0.0 < raw <= ep["n"] + SL_DROP_EPS:
      ep["bad_t"] = None
      if ep["mat_t"] is None:
        ep["mat_t"] = now
        self._la_log(ep, "materialized", limit=raw, past=self._odo - ep["b"])
      if ep["hold_t"] is None:
        ep["hold_t"] = now                   # the hold is CONTINUOUS: any higher reading restarts it
      elif now - ep["hold_t"] >= LA_PROMOTE_HOLD_S:
        self._la_end(now, None)
        return None
      return ep["tgt"] if live else None
    why = None
    if ep["mat_t"] is not None:
      if raw > ep["n"] + SL_DROP_EPS:
        why = "limitRose"                    # the lower limit went away again: the road not taken (19:02)
        ep["hold_t"] = None
      elif raw <= 0.0 and self._sl <= 0.0:
        # The limit has been unknown past the SL_HOLD_S dropout hold (mapd down?). Never hold a cap on nothing: end it,
        # keeping the slowed set -- the drop DID materialize, so no restore toward a limit nobody can read now.
        if ep["live"]:
          self._ep_limit_drop = True
        cloudlog.error("speedadjust: look-ahead ended -- the posted limit went unknown after the announced " +
                       f"{ep['n']:.1f} m/s drop materialized; keeping the slowed set, no restore")
        self._la_end(now, "limitUnknown")
        return None
      # raw unknown within the hold: a dropout is not evidence either way
    else:
      if self._odo - ep["b"] > max(LA_PASS_MARGIN_MIN_M, v_ego * LA_PASS_MARGIN_S):
        self._la_end(now, "passed")          # the boundary is behind us and the limit never changed
        return None
      if ann is None:
        why = "gone"
      elif ann["n"] > ep["n"] + SL_DROP_EPS:
        why = "rose"
      elif abs(ann["b"] - ep["b0"]) > LA_SAME_ANN_M:
        why = "moved"                        # re-routed, or a frozen mapd whose distance no longer counts down
      else:
        ep["b"] = ann["b"]                   # mapd's refined distance
    if why is None:
      ep["bad_t"] = None
    elif ep["bad_t"] is None or ep["bad_why"] != why:
      ep["bad_t"], ep["bad_why"] = now, why
    elif now - ep["bad_t"] >= LA_GONE_GRACE_S:
      self._la_end(now, why)
      return None
    return ep["tgt"] if live else None

  # ---- satele2pnw: diagnostic status publish (mem-param; ces_pnw forwards it into ces_events) -----
  SA_PUB_THROTTLE_S = 0.2                  # 5 Hz — ces_pnw samples at ~1 Hz, this just bounds the cost

  def _publish_status(self, v_cruise_set, v_cruise, out) -> None:
    """Publish the internal state needed to explain any speedadjust behaviour after the fact.

    NOT gated on _long_ok (unlike _publish_target, which is a stock-ACC actuator channel) — this must
    report on every car. Every field here exists because its absence blocked a real diagnosis:
      sl/slRef/ratio  -> reproduce the limit-drop cap arithmetic (cap = sl * ratio, floored at sl)
      ovr             -> WHY a manual set-change did or didn't release the cap (the 2026-08-21 gap)
      lastSet/vSet    -> whether the driver's change was even visible to the controller
      eng             -> the _cruise_engaged() gate that discards the baseline when it flickers
    Writing to /dev/shm alone is NOT enough to be logged — ces_pnw must also cherry-pick these keys
    into its tick record, or they evaporate (lesson: VTSCStatus fields, 2026-08-18)."""
    if self.mem_params is None:
      return
    now = time.monotonic()
    if now - self._sa_pub_t < self.SA_PUB_THROTTLE_S:
      return
    self._sa_pub_t = now
    def _r(v, n=2):
      return round(float(v), n) if isinstance(v, (int, float)) and math.isfinite(float(v)) else None
    self.mem_params.put_nonblocking("SpeedAdjustStatus", {
      "mode": self._mode,
      "sl": _r(self._sl), "slRef": _r(self._sl_ref), "ratio": _r(self._ratio, 3),
      "cap": _r(self._cap_out), "out": _r(out), "vSet": _r(v_cruise_set), "vCruise": _r(v_cruise),
      "lastSet": _r(self._last_v_set),
      "ovr": self._ovr,
      "eng": bool(self._engaged),
      "polLatch": bool(self._police_latched),
      "polSupp": bool(self._police_suppressed),
      "polKey": self._police_latched_key,
      "epLim": bool(getattr(self, "_ep_limit_drop", False)),   # sanorestore2pnw
      "noRst": getattr(self, "_no_restore_why", None),          # sanorestore2pnw (+ zoneSet / zoneAbandoned)
      "zoneTgt": _r(getattr(self, "_zone_target", None)),       # sazoneset2pnw: zone target, None = no zone episode
      "zoneN": self._zone_n,                                    # sazoneset2pnw: zone episodes opened (ICBM restore bound)
      "zoneLast": _r(self._zone_last),                          # ...and the most recent one's target
      "icbmHold": bool(self._icbm_hold),                        # ratio holding the pre-curve set (ICBM has it tapped down)
      "inst": self._inst,                                       # zonefollow2pnw: changes only when plannerd restarts
      # limitahead2pnw: the look-ahead, shadow or live. laNext/laNextD = the lower limit announced ahead (before any
      # episode), laN/laTgt/laPre/laDs/laD = the running episode (announced limit, target, the set before it, the
      # start distance, the distance left to the boundary), laMat = the limit is now current, laWhy = how the last
      # episode ended, laEvN = decisions logged so far (each is a speedadjust_lookahead cloudlog event).
      "laMode": self._la_mode,
      "laNext": _r(self._ann["n"]) if self._ann else None,
      "laNextD": _r(self._ann["b"] - self._odo, 1) if self._ann else None,
      "laLive": bool(self._la["live"]) if self._la else None,
      "laN": _r(self._la["n"]) if self._la else None,
      "laTgt": _r(self._la["tgt"]) if self._la else None,
      "laPre": _r(self._la["pre"]) if self._la else None,
      "laDs": _r(self._la["d_start"], 1) if self._la else None,
      "laD": _r(self._la["b"] - self._odo, 1) if self._la else None,
      "laMat": (self._la["mat_t"] is not None) if self._la else None,
      "laWhy": self._la_why,
      "laEvN": self._la_ev_n,
    })

  # ---- speedadjust-exec2pnw: stock-ACC button-management publish (mem-param side effect only) ----
  def _publish_target(self, target, ceiling=None, direction="dec") -> None:
    """Publish (or clear) the SpeedAdjustTarget mem-param for the shared stock-ACC button executor.
    Car-agnostic: no fingerprint/brand check here — gated ONLY on self._long_ok (this is the ONE
    choke point every call site funnels through, so a future call site can't forget the gate): a
    no-op on any op-long car, since nothing there ever reads this mem-param — the op-long return
    value from cap() is the only thing that ever steers those cars. target=None -> publish {} exactly
    once on the active->idle transition (never spammed every idle tick); otherwise throttled to
    PUB_THROTTLE_S. Best-effort: NEVER raises into the control path."""
    if self._long_ok or self.mem_params is None:
      return
    now = time.monotonic()
    if target is None:
      if self._pub_active:
        try:
          self.mem_params.put_nonblocking("SpeedAdjustTarget", {})
        except Exception as e:
          # silentexc2pnw (Rule 2): was `except Exception: pass`. Fallback unchanged: the clear is not retried, and
          # the last target stays in the mem-param until the executor's staleness check (STALE_LIMIT_S, 2 s on its
          # ts) stands it down. Logged like _read_police; own state, so a failing clear and a failing target
          # publish each get their first line. Caught broadly: plannerd is restart_if_crash=False.
          self._clear_err_n += 1
          if self._clear_err_t is None or now - self._clear_err_t >= POLICE_READ_ERR_LOG_S:
            cloudlog.exception(f"speedadjust: SpeedAdjustTarget clear FAILED ({type(e).__name__}) -- the stock-ACC " +
                               "executor keeps the last target until it goes stale " +
                               f"({self._clear_err_n} failure(s) since the last log)")
            self._clear_err_t = now
            self._clear_err_n = 0
        self._pub_active = False
        self._pub_last = now
      return
    if now - self._pub_last < PUB_THROTTLE_S:
      return
    self._pub_last = now
    self._pub_active = True
    try:
      # restore-hardening: ceiling can legitimately be None here (e.g. a pedal/ACC-off intervention
      # cleared _pub_ceiling mid-cap — see cap()) — fall back to target itself (self-clamping, a no-op
      # for a dec command) rather than let a bare float(None) silently drop this publish and starve
      # the executor of the dec target for the rest of the cap. That would WEAKEN the dec/slow-down
      # side, which must never happen.
      ceiling_val = float(ceiling) if ceiling is not None else float(target)
      payload = {"target": round(float(target), 2), "ceiling": round(ceiling_val, 2),
                 "ts": time.time()}  # noqa: TID251 -- wall clock heartbeat shared with the executor
      if direction == "inc":
        payload["dir"] = "inc"
      self.mem_params.put_nonblocking("SpeedAdjustTarget", payload)
    except Exception as e:
      # silentexc2pnw (Rule 2): was `except Exception: pass`. Fallback unchanged: this publish is dropped, so the
      # stock-ACC executor gets no fresh police/limit target (or restore) and stands down once the last one is
      # STALE_LIMIT_S old -- the truck is NOT slowed. Logged like _read_police. What can raise: the put itself (e.g.
      # UnknownKeyName on a params_keys.h / params_pyx.so mismatch) or float() of a non-numeric target/ceiling.
      self._pub_err_n += 1
      if self._pub_err_t is None or now - self._pub_err_t >= POLICE_READ_ERR_LOG_S:
        cloudlog.exception(f"speedadjust: SpeedAdjustTarget publish FAILED ({type(e).__name__}) -- the stock-ACC " +
                           "executor gets NO police/limit slowdown or restore target while this lasts " +
                           f"({self._pub_err_n} failure(s) since the last log)")
        self._pub_err_t = now
        self._pub_err_n = 0

  @staticmethod
  def _driver_intervening(sm) -> bool:
    """speedadjust-exec2pnw: defense-in-depth abort for the restore window ONLY (mirrors icbm2pnw's
    "driver gas/brake or ACC-off in ANY phase" abort matrix entry). The shared executor's
    decide_press()/RestoreGuard already independently gate EVERY press on the same signals read
    directly off the real stock CAN state at the Ford carcontroller layer — this is a belt-and-
    suspenders check at the brain layer, not the only guard. sm may be None (unit tests / no
    SubMaster) or missing carState -> defensively "not intervening" (never raises)."""
    if sm is None:
      return False
    try:
      cs = sm['carState']
      return bool(cs.gasPressed or cs.brakePressed or not cs.cruiseState.enabled)
    except Exception:
      return False

  @staticmethod
  def _read_stock_set(sm) -> float:
    """restore-hardening #1: the truck's OWN reported stock-ACC set speed (m/s) — the live ground
    truth the restore ceiling must never be commanded above. Returns 0.0 (== "no evidence", NOT "the
    driver commanded 0") whenever the value is absent/unreadable, and also whenever it reports
    0/standby (observed on ACC-off) — callers must treat 0.0 as "unknown", never as a driver-chosen
    floor. sm may be None/missing carState (unit tests, or a background-only sm) -> 0.0, never raises."""
    if sm is None:
      return 0.0
    try:
      v = float(sm['carState'].cruiseState.speed)
      return v if v > 0.0 else 0.0
    except Exception:
      return 0.0

  @staticmethod
  def _in_curve(sm) -> bool:
    """restore-hardening #3: True while the vehicle is laterally loaded in a curve RIGHT NOW — mirrors
    ces_pnw.icbm_in_curve's measured-now test (yaw_rate * v_ego from the model's first point, against
    the CURVE_LAT_ACCEL_EXIT hysteresis) WITHOUT importing ces_pnw (see SA_IN_CURVE_LAT_ACCEL — no
    cross-feature coupling). The restore must not BEGIN and a running restore must PAUSE while this is
    True, exactly like ces_pnw.IcbmEpisode: never raise the stock set speed mid-curve. sm may be
    None/missing modelV2/empty arrays -> False (never blocks on garbage, never raises)."""
    if sm is None:
      return False
    try:
      model = sm['modelV2']
      orz = model.orientationRate.z
      vx = model.velocity.x
      if not orz or not vx:
        return False
      return abs(float(orz[0]) * float(vx[0])) >= SA_IN_CURVE_LAT_ACCEL
    except Exception:
      return False

  def _read_icbm(self, now: float):
    """sazoneset2pnw: the curve brain's live button command, from the same IcbmTarget mem-param the Ford executor
    reads, at the executor's own ~4 Hz. Returns "dec"/"inc" while a command is fresh (SA_ICBM_FRESH_S), else None.
    Stock ACC only -- an op-long car has no executor, and this must not change what cap() returns there. An
    unreadable param is logged once and reads as "no command": the pre-sazoneset behaviour, never a guess."""
    if self._long_ok or self.mem_params is None:
      return None
    if now - self._icbm_read_t >= PUB_THROTTLE_S:
      self._icbm_read_t = now
      try:
        raw = self.mem_params.get("IcbmTarget", return_default=True)
        if isinstance(raw, (bytes, str)):
          raw = json.loads(raw) if raw else None
        self._icbm_cmd = raw if isinstance(raw, dict) else None
      except Exception:
        self._icbm_cmd = None
        if not self._icbm_read_warned:
          self._icbm_read_warned = True
          cloudlog.exception("speedadjust: IcbmTarget unreadable -- curve taps can read as driver set changes")
    c = self._icbm_cmd
    if not c or "target" not in c or "ts" not in c:
      return None
    try:
      age = time.time() - float(c["ts"])  # noqa: TID251 -- the executor's wall-clock heartbeat
    except (TypeError, ValueError):
      return None
    if not math.isfinite(age) or age > SA_ICBM_FRESH_S:
      return None
    d = c.get("dir", "dec")
    return d if d in ("dec", "inc") else None

  def _end_zone(self, now: float, why: str, target: float, stock_now: float, v_cruise: float) -> float:
    """sazoneset2pnw: end a limit-drop-only episode and FORGET it. `why` is "zoneSet" (the truck's set
    reached the zone target) or "zoneAbandoned" (it could not get there within ZONE_SET_TIMEOUT_S of
    actuatable time). Either way the higher baseline is dropped -- the zone limit becomes the reference --
    so the same drop cannot re-trigger, and nothing is ever restored. Stock-ACC only; returns the neutral
    v_cruise, as every stock-ACC path in cap() does."""
    self._sl_ref = self._sl
    self._cap_out = None
    self._release_t = None
    self._pub_ceiling = None
    self._min_pub_target = None
    self._restore_ceiling = None
    self._restore_deadline = None
    self._restore_last_stock = None
    self._zone_target = None
    elapsed, self._zone_elapsed = self._zone_elapsed, 0.0
    # an in-flight SET- tap of our own can still land after this -- give the override detector its grace
    self._last_actuation_transition_t = now
    self._no_restore_why = why
    self._engaged = False
    if why == "zoneSet":
      cloudlog.event("speedadjust_zone_set", limit=round(float(self._sl), 2), target=round(float(target), 2),
                     stock=round(float(stock_now), 2), ratio=round(float(self._ratio), 3))
    else:
      # Rule 2: the zone speed was NOT applied. Loud, because the truck is still above the zone target.
      msg = " ".join([f"speedadjust: zone set ABANDONED after {elapsed:.0f} s actuatable -- set {stock_now:.1f}",
                      f"never reached {target:.1f} (limit {self._sl:.1f}); forgetting the zone, no further taps"])
      cloudlog.error(msg)
      cloudlog.event("speedadjust_zone_set_abandoned", limit=round(float(self._sl), 2),
                     target=round(float(target), 2), stock=round(float(stock_now), 2), elapsed_s=round(elapsed, 1))
    self._publish_target(None)
    return v_cruise

  def _step_restore(self, now: float, sm) -> None:
    """Bookkeeping + publish for the bounded restore window (see module docstring). Called only from
    inside `cap()` while NOT actively capping. Cancels the moment: the window (RESTORE_WINDOW_S)
    expires, the car is op-long (belt-and-suspenders — callers already gate on `not self._long_ok`
    before starting a restore, so this only matters if that ever changes), or `sm['carState']` shows
    driver/ACC intervention."""
    if self._restore_ceiling is None:
      self._restore_last_stock = None
      self._publish_target(None)
      return
    if self._long_ok or now > self._restore_deadline or self._driver_intervening(sm):
      self._restore_ceiling = None
      self._restore_deadline = None
      self._restore_last_stock = None
      # FIX B: an in-flight SET+ tap of our own may still land after this transition -- stamp it so
      # the set-change detector gives it a grace window instead of misreading it as an override.
      self._last_actuation_transition_t = now
      self._publish_target(None)
      return
    # restore-hardening #3: never BEGIN or CONTINUE raising the set while laterally loaded in a curve
    # — mirrors ces_pnw.IcbmEpisode's in-curve pause. PAUSE ONLY: skip the publish this tick (the
    # mem-param heartbeat goes stale -> the executor stale-stops within STALE_LIMIT_S) but keep the
    # episode alive (ceiling/deadline untouched) so the restore resumes the instant the curve clears,
    # still bounded by the exact same window/pedal/ACC guards above.
    if self._in_curve(sm):
      return
    # restore-hardening #1 (BLOCKER): the invariant this enforces — the restore can NEVER command a
    # set speed above the driver's CURRENT live stock set. A restore STARTS below the ceiling by
    # design (that's the gap being walked up), so the ceiling must NOT be floored to the raw current
    # reading every tick (that would collapse the very first tick to a no-op restore) — instead, mirror
    # icbm_pnw.RestoreGuard's own movement detection: ratchet the ceiling down only when the truck's
    # OWN reported set MOVES DOWN relative to its last observed value while we only ever press up —
    # that is a driver SET- (or something we don't understand), never our own tap (which only rises).
    # The first observation in an episode just establishes the baseline (no judgment, same as
    # RestoreGuard's `self._last_set is not None` gate).
    stock_now = self._read_stock_set(sm)
    if stock_now > 0.0:
      if self._restore_last_stock is not None and stock_now < self._restore_last_stock - SA_DRIVER_LOWER_TOL:
        self._restore_ceiling = min(self._restore_ceiling, stock_now)
      self._restore_last_stock = stock_now
    self._publish_target(self._restore_ceiling, self._restore_ceiling, "inc")

  # ---- speedadjustreset2pnw: manual set-change = override detection ---------
  @staticmethod
  def _cruise_engaged(sm) -> bool:
    """speedadjustreset2pnw hardening FIX A: whether ACC is actually engaged right now. Required
    BEFORE trusting any v_cruise_set delta as a manual override — `v_cruise_initialized` alone does
    NOT protect the Tesla: `selfdrive/car/cruise.py`'s PCM branch only maps `cruiseState.speed==0` to
    the V_CRUISE_UNSET sentinel, but the Raven's `cruiseState.speed` is floored at
    `max(DI_digitalSpeed·conv, 1e-3)` — never exactly 0 — so `v_cruise_initialized` stays True straight
    through STANDBY. Without this gate, the ENGAGE transition itself (the set jumping from the ~0.004
    floor to a real value, e.g. 26.8 m/s) reads as a manual override and can silently suppress a
    police cap at the exact moment it should be engaging. sm may be None (unit tests without a
    SubMaster, or missing carState) -> True: on the real car sm is always live, so this is the same
    "assume the permissive default when we truly cannot tell" convention `_driver_intervening()`
    already uses (there, missing sm defaults to "not intervening"; here it defaults to "engaged" —
    both let the rest of this module behave as if the plumbing were present)."""
    if sm is None:
      return True
    try:
      return bool(sm['carState'].cruiseState.enabled)
    except Exception:
      return True

  def _is_own_actuation(self, prev: float, new: float) -> bool:
    """True iff an observed v_cruise_set delta is NOT reliably attributable to a genuine driver
    override — i.e. it could plausibly be THIS module's own commanded actuation. On an op-long car
    this is always False: cap()'s RETURN value never writes back into CS.vCruise (see the module
    docstring's v_cruise_set feedback-safety note), so every real change there IS the driver.

    On a stock-ACC car, `selfdrive/car/cruise.py`'s PCM branch sets `CS.vCruise = CS.cruiseState.speed`
    directly — the truck's OWN live ACC dial — and THIS module's speedadjust-exec2pnw SET-/SET+
    button-tap executor actually moves that dial.

    speedadjustreset2pnw hardening FIX C (Gemini BLOCKER): this is DIRECTION-ONLY, not value-based.
    A value-tolerance check (the original design) cannot distinguish our own tap from a driver's
    SAME-direction tap — one 1-mph step is smaller than the tolerance needed to absorb our own tap's
    lag/latency — so it MASKED a genuine driver SET- during an active dec-cap. Any SAME-direction move
    while we are actively actuating in that direction is therefore treated as "ours" (not attributable
    -> no reset), full stop, regardless of magnitude; only an OPPOSITE-direction move (driver wants to
    go FASTER than an active dec cap, or SLOWER than an active restore ceiling) is unambiguous — we
    NEVER command that direction ourselves in that phase — and still resets. KNOWN LIMITATION: a
    single same-direction driver tap during active actuation will not register as an override on
    stock-ACC; an opposite-direction tap (or fully disengaging) still works."""
    if self._long_ok:
      return False
    if new < prev and self._cap_out is not None:
      return True    # actively dec-capping: WE only ever press SET- (down) -- same direction, not attributable
    if new > prev and self._restore_ceiling is not None:
      return True    # actively restoring: WE only ever press SET+ (up) -- same direction, not attributable
    return False

  # ---- the cap the planner folds (reduce-only) ------------------------------
  def cap(self, sm, v_cruise_set: float, v_cruise: float, v_ego: float, v_cruise_initialized: bool) -> float:
    """satele2pnw: thin wrapper over _cap_impl() that ALWAYS publishes SpeedAdjustStatus.

    Deliberately a wrapper rather than publish-calls inside _cap_impl(): that method has six separate
    return points, and a telemetry channel that misses paths is worse than none — you cannot tell "the
    feature was inactive" from "the logger missed it". The `finally` also captures the state when
    _cap_impl() raises. Driver directive 2026-08-21, after a stuck-at-47-mph episode was undiagnosable
    because this module published nothing: _ratio/_sl_ref/_last_v_set and the engaged gate were
    invisible live AND unrecoverable afterwards.
    """
    out = v_cruise
    try:
      out = self._cap_impl(sm, v_cruise_set, v_cruise, v_ego, v_cruise_initialized)
      return out
    finally:
      try:
        self._publish_status(v_cruise_set, v_cruise, out)
      except Exception:
        # telemetry must never affect the control path -- but it must not fail SILENTLY either.
        # satele2pnw shipped without its params_keys.h entry, so every publish raised
        # UnknownKeyName and this handler swallowed it: the channel was dead for a whole drive and
        # looked exactly like "the feature never ran". Log once so the next such bug is one grep away.
        if not self._sa_pub_warned:
          self._sa_pub_warned = True
          cloudlog.exception("satele2pnw: SpeedAdjustStatus publish failed (telemetry only, control unaffected)")

  def _cap_impl(self, sm, v_cruise_set: float, v_cruise: float, v_ego: float,
                v_cruise_initialized: bool) -> float:
    """
    v_cruise_set: the driver's raw, PRE-VTSC (pre-any-other-cap) cruise set (m/s) — used ONLY to anchor
      the limit-drop ratio/baseline and to seed the cap slew (speedanchor2pnw F2). Using this instead of
      the already-VTSC-reduced `v_cruise` stops a curve in effect at anchor/engage time from poisoning
      the ratio or the slew seed.
    v_cruise: the EFFECTIVE cruise ceiling after upstream reduce-only caps (VTSC etc.) have already been
      applied — the emitted cap remains reduce-only bounded against THIS value, unchanged from before.
    v_cruise_initialized: False before the driver has ever set cruise (v_cruise*/set are the
      V_CRUISE_UNSET sentinel, ~145 km/h) — anchoring/seeding is skipped entirely in that case
      (speedanchor2pnw F_uninit), same treatment as idle.
    """
    now = time.monotonic()
    dt = min(max(now - self._last_t, 0.0), 0.5) if self._last_t is not None else 0.0
    self._last_t = now
    self._odo += max(float(v_ego), 0.0) * dt  # limitahead2pnw: announced boundaries are kept on this odometer
    if now - self._last_read >= READ_S:
      self._last_read = now
      self._read_inputs()
    rise_since, self._sl_rise_since = self._sl_rise_since, None

    # speedanchor2pnw (F_uninit, Fable-caught): an uninitialized cruise is not a real driver set —
    # anchoring/seeding off the ~145 km/h sentinel would inflate _ratio (silently no-ops the next real
    # limit drop) and seed _cap_out far above the eventual real set. No bookkeeping, clean passthrough.
    if not v_cruise_initialized:
      self._engaged = False
      self._police_latched = False
      self._police_suppressed = False        # speedadjustreset2pnw
      self._cap_out = None
      self._release_t = None
      # speedadjust-exec2pnw: an uninitialized cruise can't be a valid restore ceiling either.
      self._pub_ceiling = None
      self._restore_ceiling = None
      self._restore_deadline = None
      self._min_pub_target = None
      self._restore_last_stock = None
      # speedadjustreset2pnw (F_uninit parity): the V_CRUISE_UNSET sentinel is not a real driver set —
      # forget it so the eventual uninitialized->initialized transition can't read as a "change".
      self._last_v_set = None
      self._zone_target = None               # sazoneset2pnw
      self._zone_elapsed = 0.0
      self._icbm_hold = False
      if self._la is not None:               # limitahead2pnw: no set to slow from
        self._la_end(now, "cruiseUnset")
      self._la_hold = False
      self._publish_target(None)
      return v_cruise

    icbm_dir = self._read_icbm(now)          # sazoneset2pnw: None on op-long
    if icbm_dir == "dec":
      self._icbm_dec_t = now
    elif icbm_dir == "inc":
      self._icbm_inc_t = now

    # speedadjustreset2pnw (driver directive 2026-08-16): a manual cruise-set change (either
    # direction) is an explicit "resume — don't slow me for this" override. Must run BEFORE
    # _police_cap()/_update_baseline() below so a detected change takes effect the SAME tick.
    # _is_own_actuation() filters out this module's OWN stock-ACC button taps (see its docstring) so
    # this can never self-cancel the feature it's supposed to be overriding.
    #
    # FIX A: only trust a delta while ACC is engaged on BOTH this tick and the previous one -- see
    # _cruise_engaged()'s docstring for why v_cruise_initialized alone doesn't protect the Tesla
    # (cruiseState.speed's 1e-3 floor keeps it "initialized" straight through standby). Forcing
    # _last_v_set back to None on every not-engaged tick means the FIRST engaged tick after any
    # not-engaged stretch (including a fresh enable) only establishes a new baseline -- it can never
    # itself be read as a change, exactly mirroring the uninit-sentinel treatment above.
    # satele2pnw: record WHY a manual-override was or wasn't honoured this tick. This is the field the
    # 2026-08-21 stuck-at-47-mph episode needed and did not have: "the driver turned the dial and the
    # cap did not release" has five distinct causes and no way to tell them apart after the fact.
    engaged = self._cruise_engaged(sm)
    if not engaged:
      self._last_v_set = None
      self._ovr = "notEngaged"               # gate closed -> baseline discarded, a change here is lost
    elif self._last_v_set is None:
      self._ovr = "baseline"                 # first engaged tick: establishes a baseline, never a change
    elif abs(v_cruise_set - self._last_v_set) <= SET_CHANGE_EPS:
      self._ovr = "noChange"
    if engaged and self._last_v_set is not None and abs(v_cruise_set - self._last_v_set) > SET_CHANGE_EPS:
      # FIX B: a stock-ACC tap of OUR OWN can still be in flight (0.3-2 s actuation->CAN->carState
      # latency) for a short window after _cap_out/_restore_ceiling already nulled at a phase
      # boundary -- suppress detection entirely for that grace window rather than let a late-landing
      # own-tap fall through _is_own_actuation() (which needs the field to still be non-None).
      in_grace = (not self._long_ok
                  and now - self._last_actuation_transition_t < SA_ACTUATION_GRACE_S)
      # satele2pnw: same short-circuit as the original `not in_grace and not _is_own_actuation(...)`
      # (_is_own_actuation is still not called while in_grace), just with the outcome recorded.
      if in_grace:
        self._ovr = "grace"
      elif self._is_own_actuation(self._last_v_set, v_cruise_set):
        self._ovr = "ownTap"
      elif (not self._long_ok and v_cruise_set < self._last_v_set
            and now - self._icbm_dec_t < SA_ACTUATION_GRACE_S):
        # sazoneset2pnw (Fable review, measured): a SET- while the curve brain's dec is on the bus -- or within the
        # in-flight grace after it -- is ICBM's tap. Read as the driver's, it re-anchored the ratio to the tapped-down
        # set and the zone never trimmed. Same known limitation as FIX C: a driver SET- in that window is not an
        # override (an opposite-direction SET+ still is).
        self._ovr = "icbmTap"
      elif (not self._long_ok and v_cruise_set > self._last_v_set and self._cap_out is None
            and now - self._icbm_inc_t < SA_ACTUATION_GRACE_S):
        # zonefollow2pnw (Fable review of sazoneset2pnw, measured): the same for the curve RESTORE's SET+. Read as the
        # driver's, each one re-anchored the ratio to the half-restored set, so a limit drop mid-restore trimmed from
        # it (47/50/54 mph instead of 56). Only while speedadjust is not capping: arbitrate() runs no inc while any
        # dec is on the bus, so a SET+ during our own cap is the driver's (e.g. dismissing a police slowdown).
        self._ovr = "icbmTap"
      else:
        self._ovr = "applied"
      if self._ovr == "applied":
        # FIX D: only dismiss an alert the driver could actually perceive acting on -- a routine set
        # nudge with a report still minutes away (not latched, no active cap) must not silently kill
        # the eventual slowdown.
        # FIX 1 (Fable F2 / Gemini #2, review pass 3): gate on _police_latched ALONE, not
        # "_police_latched or _cap_out is not None" -- a police-sourced _cap_out always implies
        # _police_latched already (the latch precedes any returned police target), so the
        # "or _cap_out is not None" disjunct only ever added LIMIT-DROP (mode 2) caps. That wrongly
        # suppressed a pending, not-yet-latched police alert whenever the driver overrode an active
        # limit trim. The limit-drop cap itself still releases unconditionally below (_sl_ref/_ratio
        # re-anchor + _cap_out=None are OUTSIDE this gate).
        if self._police_latched:
          self._police_suppressed = True
          _capsrc = self._police.get("cap") if isinstance(self._police, dict) else None
          self._police_suppressed_uuid = _police_key(_capsrc or self._police)
        # speed limit: re-anchor the baseline to the new set (speedanchor2pnw's own F2 formula) so
        # the current trim releases; a FURTHER drop below this new baseline will still re-cap.
        self._sl_ref = self._sl
        self._ratio = (v_cruise_set / self._sl) if self._sl > 0.0 else 0.0
        # FIX 3 (Gemini, review pass 3): stamp the actuation-transition grace window BEFORE releasing
        # an active cap here -- an in-flight own SET- tap from the dying cap can otherwise land the
        # NEXT tick (after _cap_out already went None) and phantom-trigger a second override. Mirrors
        # the same edge-guarded pattern as the other three stamp sites (stock-ACC only; op-long has no
        # executor taps in flight).
        if not self._long_ok and self._cap_out is not None:
          self._last_actuation_transition_t = now
        # release any active emitted cap so speed resumes immediately (MPC/executor bounds the accel).
        self._cap_out = None
        self._release_t = None
        self._zone_target = None             # sazoneset2pnw: the driver's own set ends any zone episode
        self._zone_elapsed = 0.0
        # limitahead2pnw: ...and any look-ahead, which is not restarted for the same announcement. The drop itself,
        # when it arrives, is judged against the set he just chose (the re-anchor above), with the ordinary confirm.
        if self._la is not None:
          self._la_end(now, "driverOverride")
        self._la_hold = False
        # FIX E (telemetry only): mirror the normal release block's engaged-bookkeeping so the
        # "released" log actually fires and the next real engage's log isn't swallowed.
        if self._engaged:
          self._engaged = False
          cloudlog.info("speedadjust: released -> cruise (manual override)")
    if engaged:
      self._last_v_set = v_cruise_set

    if self._mode == 0:
      self._engaged = False
      self._police_latched = False
      self._police_suppressed = False        # speedadjustreset2pnw
      self._cap_out = None
      self._release_t = None
      self._pub_ceiling = None
      self._restore_ceiling = None
      self._restore_deadline = None
      self._min_pub_target = None
      self._restore_last_stock = None
      self._sl_ref = self._sl                # keep baseline current while idle (no stale drop on enable)
      # speedanchor2pnw (F2): anchor off the raw set, not a VTSC-curve-reduced v_cruise.
      self._ratio = (v_cruise_set / self._sl) if self._sl > 0.0 else 0.0
      self._zone_target = None               # sazoneset2pnw
      self._zone_elapsed = 0.0
      if self._la is not None:               # limitahead2pnw
        self._la_end(now, "modeOff")
      self._la_hold = False
      self._publish_target(None)
      return v_cruise

    # restore-hardening #1 (BLOCKER): ANY pedal press or ACC-off kills the restore EPISODE IDENTITY in
    # ANY phase — mirrors ces_pnw.IcbmEpisode's reset-in-any-phase (ces_pnw.py ~599-609). The dec/cap
    # math below is completely UNCHANGED/still computed and forwarded (reduce-only slow-down must never
    # be weakened by this — see _publish_target()'s None-ceiling fallback); only the restore-ceiling
    # bookkeeping is cleared, so a ceiling latched before the intervention can never later be walked
    # back up to. If capping continues after the intervention clears, the NEXT fresh cap-engage
    # (_cap_out is None) re-latches a ceiling off the THEN-current set, same as ces_pnw's own "next tick
    # starts a fresh episode" behavior.
    intervening = self._driver_intervening(sm)
    if intervening:
      self._pub_ceiling = None
      # FIX 2 (Fable F1, review pass 3): stamp the transition, edge-guarded on a restore having
      # actually been active, BEFORE nulling it -- on stock-ACC (e.g. a gas press that keeps ACC
      # enabled on the Ford) an in-flight own SET+ tap from the just-cleared restore can otherwise
      # land next tick with no grace window and get misread as a driver override. Same pattern as the
      # existing preempt-site stamp.
      if self._restore_ceiling is not None:
        self._last_actuation_transition_t = now
      self._restore_ceiling = None
      self._restore_deadline = None
      self._min_pub_target = None
      self._restore_last_stock = None

    # speedadjust-exec2pnw: the cap math below now runs for EVERY mode != 0 car, regardless of
    # self._long_ok — plannerd runs on every car, so the target was always being computed for
    # op-long cars and silently discarded otherwise. The RETURN value at the bottom stays gated on
    # self._long_ok exactly as before (unchanged op-long behavior); the NEW mem-param publish is a
    # pure side effect that only ever fires when not self._long_ok (see _publish_target()).
    caps = []
    pc = self._police_cap(v_cruise_set, v_ego)   # modes 1 and 2
    if pc is not None:
      caps.append(pc)
    # speedanchor2pnw (F3): re-anchor the limit-drop baseline whenever the feature is active (mode 1 or
    # 2) — not just when the drop-cap itself is computed (mode 2 only) — so it never goes stale across
    # an AutoSpeedReduce 1→2 switch.
    # sazoneset2pnw: while the curve brain has the set tapped down the ratio holds the pre-curve set. The hold
    # starts on any fresh ICBM command and survives ICBM's silent gaps (clear debounce, in-curve pause); it ends
    # on a pedal or ACC off, or once the set is back at the reference with ICBM quiet -- which includes the driver's
    # own set change: the override above has just re-anchored the reference to the set he chose.
    if self._long_ok or not engaged or intervening:
      self._icbm_hold = False
    elif icbm_dir is not None:
      self._icbm_hold = True
    elif self._icbm_hold and self._read_stock_set(sm) >= self._ratio * self._sl_ref - ZONE_SET_DONE_TOL:
      self._icbm_hold = False
    # limitahead2pnw: the look-ahead runs BEFORE the baseline update, so an episode starting this tick already holds
    # the ratio. The hold outlives an aborted episode until its restore has walked the set back (or ended): those
    # SET+ taps are ours too.
    la_tgt = self._la_step(now, v_ego, v_cruise_set)
    if (self._la_hold and self._la is None and self._cap_out is None
        and (self._restore_ceiling is None or self._read_stock_set(sm) >= self._ratio * self._sl_ref - ZONE_SET_DONE_TOL)):
      self._la_hold = False
    self._update_baseline(v_cruise_set)
    lc = None
    if self._mode >= 2:                       # limit-drop cap itself: mode 2 only
      lc = self._limit_drop_cap()
      # limitahead2pnw: while a LIVE look-ahead owns the drop it announced, it stands in for the limit-drop cap -- the
      # same target, but restorable until LA_PROMOTE_HOLD_S. A different, higher drop (not the announced one) still caps
      # as a real zone and makes the episode non-restorable.
      if lc is not None and self._la is not None and self._la["live"] and self._sl <= self._la["n"] + SL_DROP_EPS:
        lc = None
      if lc is not None:
        caps.append(lc)
    if la_tgt is not None:
      caps.append(la_tgt)

    if not caps:
      # release DEBOUNCE: sources must stay clear for RELEASE_S before the cap lets go — an
      # engage/release oscillation (flapping source) was half of the "wild horse" ride.
      if self._cap_out is None:
        # not currently capping (and the debounce already ran its course, if any) -- offer/continue
        # any pending bounded restore.
        self._step_restore(now, sm)
        return v_cruise
      if self._release_t is None:
        # sazoneset2pnw (Fable review): a limit RISE adopted this tick has already persisted SL_RISE_CONFIRM_S --
        # it is debounced, not a flapping source -- so that time counts toward RELEASE_S. Without the credit the
        # rise hold stacked on the debounce and a Tesla under the ~1 Hz map flicker released 6-8 s after the sign.
        self._release_t = now if rise_since is None else min(now, rise_since)
      if now - self._release_t < RELEASE_S:
        self._publish_target(self._cap_out, self._pub_ceiling, "dec")  # still capping through debounce
        return max(0.0, min(v_cruise, self._cap_out))   # hold the last cap through the debounce window
      # debounce elapsed -> the cap fully releases. speedadjust-exec2pnw: hand off to a bounded SET+
      # restore back to the ceiling this cap latched at engage (stock-ACC only -- _step_restore() is a
      # no-op on any op-long car since _pub_ceiling is never consulted there).
      # restore-hardening #1 (BLOCKER): only OPEN the restore window if the truck's current stock set
      # is explainable by what we ourselves actually commanded this cap (mirrors ces_pnw.IcbmEpisode's
      # `_min_target`/ICBM_DRIVER_LOWER_TOL eligibility check) — if the driver pushed the set below the
      # lowest target we ever asked for, that is their own intent; restoring would fight it, so don't.
      if not self._long_ok and self._pub_ceiling is not None:
        stock_now = self._read_stock_set(sm)
        driver_went_lower = (stock_now > 0.0 and self._min_pub_target is not None
                             and stock_now < self._min_pub_target - SA_DRIVER_LOWER_TOL)
        # sanorestore2pnw (driver directive 2026-09-13): "I want the button control management to
        # never accelerate to the previous speed. I only want it to accelerate to the previous speed if
        # there is a police warning -- that's the only exception."  A slowdown for a LOWER POSTED
        # LIMIT is therefore permanent: when the limit rises again the set stays where it is and the
        # driver raises it himself. Only a police-only episode walks the set back up. An episode that
        # involved BOTH is treated as a limit drop -- restoring it would raise the set past a limit that
        # dropped during it, which is exactly what the directive forbids. Curve slowdowns are a
        # different brain (icbm2pnw) and keep restoring, capped at limit + 5 (icbmrestorecap2pnw),
        # per the same conversation.
        if getattr(self, "_ep_limit_drop", False):
          self._no_restore_why = "limitDrop"
          cloudlog.event("speedadjust_no_restore", reason="limitDrop",
                         ceiling=round(float(self._pub_ceiling), 2), stock=round(float(stock_now), 2))
        elif driver_went_lower:
          self._no_restore_why = "driverLower"
        else:
          self._no_restore_why = None
          self._restore_ceiling = self._pub_ceiling
          self._restore_deadline = now + RESTORE_WINDOW_S
      elif not self._long_ok:
        # Fable review: no ceiling was latched (the driver intervened at engage), so no restore opens --
        # say so. `None` is documented as "a restore opened / not applicable" and must not be reused here.
        self._no_restore_why = "noCeiling"
        # else: no restore this episode — leave _restore_ceiling/_restore_deadline at None (already
        # None unless a prior tick set them, which can't happen on a fresh release).
      self._cap_out = None
      self._release_t = None
      self._pub_ceiling = None
      self._min_pub_target = None
      self._zone_target = None               # sazoneset2pnw
      self._zone_elapsed = 0.0
      # FIX B: an in-flight SET- tap of our own may still land after this transition -- stamp it so
      # the set-change detector gives it a grace window instead of misreading it as an override.
      self._last_actuation_transition_t = now
      if self._engaged:
        self._engaged = False
        cloudlog.info("speedadjust: released -> cruise")
      self._step_restore(now, sm)
      return v_cruise

    self._release_t = None
    # speedadjust-exec2pnw: a NEW cap always preempts any in-progress restore (DEC ALWAYS WINS, same
    # principle icbm2pnw's episode machine uses for its own new-cap-vs-restore conflicts).
    # FIX B: only stamp the transition when a restore was ACTUALLY in progress and is being preempted
    # here -- an in-flight SET+ tap of our own may still land after this. Must check BEFORE nulling
    # _restore_ceiling; stamping unconditionally on every engaged-capping tick would keep the grace
    # window perpetually "fresh" and silently defeat override detection for the entire cap duration.
    if self._restore_ceiling is not None:
      self._last_actuation_transition_t = now
    self._restore_ceiling = None
    self._restore_deadline = None
    self._restore_last_stock = None
    target = max(MIN_CAP, min(caps))          # floor rejects garbage-low targets
    # SLEW: the emitted cap RAMPS toward its target instead of stepping — a step target made the MPC
    # chase a square wave. Seeds at the driver's RAW set on engage (no initial jump — see speedanchor2pnw
    # F2 below) and moves <= CAP_SLEW m/s per s in BOTH directions; the MPC still bounds the actual decel
    # on top of this.
    if self._cap_out is None:
      # speedanchor2pnw (F2): seed from v_cruise_set (the driver's raw, PRE-VTSC set), not v_cruise (the
      # current EFFECTIVE ceiling). Seeding from `v_cruise` meant a VTSC curve cap active exactly at
      # engage time seeded the slew from curve speed; after the curve ended and VTSC released instantly,
      # this cap kept crawling up from curve speed at CAP_SLEW instead of already sitting near the real
      # target ("won't come back up to my set"). `out` below is still bounded by the effective `v_cruise`
      # on every tick, so seeding high here can never cause a jump — the min() catches it immediately.
      self._cap_out = v_cruise_set
      # speedadjust-exec2pnw: latch the ceiling at cap ENGAGE (icbm2pnw ceiling-latch parity) — the
      # value a later bounded restore may walk back up to, never higher. restore-hardening #1: skip
      # the latch if the driver is intervening THIS tick (pedal/ACC-off) — no restore episode should
      # start from a tick where the driver doesn't even have control; _pub_ceiling stays None until a
      # later, clean engage tick.
      if not intervening:
        self._pub_ceiling = v_cruise_set
        self._min_pub_target = self._cap_out
      self._ep_limit_drop = False             # sanorestore2pnw: a fresh episode starts clean
      self._no_restore_why = None             # ...and so does the reason (Fable: it stayed stale through the next cap)
    if lc is not None:
      self._ep_limit_drop = True              # sanorestore2pnw: sticky for the rest of the episode
    # sazoneset2pnw (driver directive 2026-09-13): a LIMIT-DROP slowdown is a one-shot ZONE SET, not a cap
    # held for the length of the zone. Driver, verbatim: "if I go down from 70 to 35 ... we need to set 35
    # plus [the same percentage]. At that point there should be no memory anymore of what could resume,
    # and then I am just driving that speed. Then comes a curve and you reduce for the curve, remember what
    # the speed was before the curve, and go back to [it] after the curve."
    #
    # So once the truck's OWN reported set has reached the zone target the episode ENDS and this module
    # goes silent: the higher baseline is forgotten (the zone limit becomes the new reference, and
    # _update_baseline re-anchors the ratio to the set the truck is now at), no restore is ever offered,
    # and SpeedAdjustTarget is cleared. That silence is also what un-starves a curve restore inside the
    # zone: Fable measured that the continuous dec, re-published every 0.25 s for the whole zone, won every
    # arbitrate() and blocked ICBM's inc -- a curve in a long trimmed zone left the set at 35 on a 60 road.
    #
    # Stock-ACC only: an op-long car (the Tesla) has no set to tap, and cap()'s return value keeps the
    # continuous cap exactly as before. A police cap in the same episode keeps the old hold (mixed rule).
    # "Reached" needs EVIDENCE: an unreadable stock set (0.0) never completes the zone -- it waits.
    zone_only = lc is not None and pc is None and la_tgt is None and not self._long_ok
    if zone_only:
      if self._zone_target is None:
        self._zone_n += 1                    # a zone episode opens (see _zone_n)
      self._zone_target = target
      self._zone_last = target
      stock_now = self._read_stock_set(sm)
      if stock_now > 0.0 and stock_now <= target + ZONE_SET_DONE_TOL:
        return self._end_zone(now, "zoneSet", target, stock_now, v_cruise)
      if stock_now > 0.0 and engaged and not intervening:
        # Bounded, never a silent hold: only time in which our taps COULD land counts (ACC engaged, no
        # pedal, set readable). While ACC is off the episode simply waits -- no press is possible then, and
        # a RES back to the old set inside the zone should still be brought down to the zone speed.
        self._zone_elapsed += dt
        if self._zone_elapsed >= ZONE_SET_TIMEOUT_S:
          return self._end_zone(now, "zoneAbandoned", target, stock_now, v_cruise)
    else:
      self._zone_target = None
      self._zone_elapsed = 0.0
    if target < self._cap_out:
      self._cap_out = max(target, self._cap_out - CAP_SLEW * dt)
    else:
      self._cap_out = min(target, self._cap_out + CAP_SLEW * dt)
    out = max(0.0, min(v_cruise, self._cap_out))   # reduce-only — never raise above the driver's set
    # restore-hardening #1 (BLOCKER): continuously track the explainability floor (the lowest target
    # WE'VE published this cap) and ratchet the latched ceiling DOWN (never up) the instant the truck's
    # OWN reported stock set falls below that floor by more than our own tap latency/lag would explain
    # — that's the driver's own SET-, and the eventual restore must never walk back up above it. This
    # runs every tick a ceiling is latched (mirrors ces_pnw.IcbmEpisode's own explainability check, but
    # applied continuously through the cap rather than only once at the clear transition).
    if self._pub_ceiling is not None:
      self._min_pub_target = self._cap_out if self._min_pub_target is None else min(self._min_pub_target, self._cap_out)
      stock_now = self._read_stock_set(sm)
      if stock_now > 0.0 and stock_now < self._min_pub_target - SA_DRIVER_LOWER_TOL:
        self._pub_ceiling = min(self._pub_ceiling, stock_now)
    # speedadjust-exec2pnw: publish the SAME slewed cap value the op-long path would consume — the
    # stock-ACC executor taps toward the identical target, just via buttons instead of the MPC.
    self._publish_target(self._cap_out, self._pub_ceiling, "dec")
    if not self._engaged:
      self._engaged = True
      cloudlog.info(f"speedadjust: engaged mode={self._mode} cap={out:.1f} (police={pc} sl={self._sl:.1f})")
    # speedadjust-exec2pnw: the RETURN value stays exactly what it always was -- only an op-long car
    # ever gets a non-neutral v_cruise back from this function. Stock-ACC cars are steered solely via
    # the SpeedAdjustTarget mem-param publish above, never through this return path.
    return out if self._long_ok else v_cruise
