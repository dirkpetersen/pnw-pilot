"""
CES (Conditional Experimental Switching) — tunable constants.

ALL values are starting points to be finalized on real drive logs (see CES.md "calibration
anchors": I-5 Terwilliger ~2.0 m/s² @ 50 mph must trip; the R≈550 m curve must be easy @70 / hard
@90). Lateral acceleration is v²·curvature, so curve triggering is speed-adaptive.
"""
import time

from openpilot.common.constants import CV
from openpilot.common.swaglog import cloudlog

# --- speed thresholds (stored in m/s; UI exposes mph) -----------------------
CES_SPEED          = 40 * CV.MPH_TO_MS   # no lead: below this -> allow Experimental (city/complex)
CES_SPEED_RET      = 43 * CV.MPH_TO_MS   # no lead: return-to-Chill (hysteresis gap above enter)
CES_SPEED_LEAD     = 45 * CV.MPH_TO_MS   # with lead: below this -> allow Experimental (was 55 -> caused
                                         #   highway-following Experimental at 50-55 mph; drive log fix)
CES_SPEED_LEAD_RET = 48 * CV.MPH_TO_MS   # with lead: return-to-Chill
# Highway gate: never trip lowSpeed-Experimental on a road whose OSM speed limit is this high — that's
# a highway/expressway, where slow-but-following is normal Chill cruising (drive log: 21 false trips at
# 50-55 mph behind traffic on a 60 mph road). slowLead/curve/stop are NOT gated (still valid on highways).
LOWSPEED_HWY_GATE  = 50 * CV.MPH_TO_MS   # m/s; OSM speed limit (spd_lim) >= this => suppress lowSpeed

# --- curve (lateral accel, m/s^2) -------------------------------------------
CURVE_LAT_ACCEL_ENTER = 1.9   # pinned by the anchor set (>1.8 so "easy@70" curves don't trip; <2.0 so Terwilliger/Marquam do)
CURVE_LAT_ACCEL_EXIT  = 1.3   # hysteresis: curve considered "done" below this
CURVE_MAP_LOOKAHEAD_S    = 10.0  # map primary (smooth early trigger)
CURVE_VISION_LOOKAHEAD_S = 3.5   # vision fallback (capped by model confidence)
CRUISING_SPEED = 5.0          # m/s; below this, curve detection is meaningless
# map half: pfeiferj mapd publishes MapTargetVelocities (per-point curve safe-speeds). Trip the map
# curve when an upcoming target speed within the lookahead is this much BELOW current speed (a real
# curve, not GPS noise). Target-speed based — the binary already did curvature->safe-speed physics.
CURVE_MAP_MIN_SLOWDOWN = 3.0  # m/s
# Freeway gate on the CES *curve* trip (drive 2026-06-28, Snoqualmie): on a freeway, VTSC+MTSC already
# cap curve speed smoothly (decel-limited, bounded floor). CES tripping Experimental for a curve there
# adds redundant e2e braking that STACKS BELOW that floor -> over-slowdown the driver overrides with gas
# (10:05 left curve: braked 85->74 below the 79 mph map floor; 10:03 right: down to 61). Suppress the
# curve trip when the OSM speed limit says we're on a freeway; stop/lead/radar braking stay intact.
# Gated ONLY when spd_lim is KNOWN to be high (0/unknown -> keep tripping, safe default). ~55 mph.
CURVE_HWY_GATE = 55 * CV.MPH_TO_MS  # m/s; spd_lim >= this => hand freeway curves to VTSC+MTSC, no CES e2e curve braking
# SHARP-curve exception to the freeway gate (drive 2026-06-28 take-control on the North Bend descent): if
# the upcoming MAP curve target is below this, the curve is genuinely sharp -> KEEP the CES e2e curve trip
# even on a freeway (maximum braking authority: e2e + VTSC + MTSC), since a sharp curve at freeway speed is
# where steering-limit/EPS saturation risk lives. Only MODERATE freeway curves (map target above this) are
# gated (the over-slowdown fix). 30 m/s ~= 67 mph: the take-control curve's map target was 65 mph.
# I-84 2026-07-06: compared against the SCALED map target (raw * MAP_SPEED_SCALE) in decide_active — mapd
# raw safe-speeds run systematically low, and the raw compare tripped Experimental on sweepers driven at
# 79-86 mph. MAP_SPEED_SCALE is imported from vtsc_constants so MTSC and this exception always agree.
CURVE_SHARP_MAP_V = 30.0  # m/s (vs scaled map target)
from openpilot.selfdrive.controls.lib.vtsc_pnw.vtsc_constants import MAP_SPEED_SCALE, tiered_map_scale  # noqa: E402,F401
# ces2pnw lead-pacing gate (2026-07-08): a lead within this range that is NOT slower than us (see
# LEAD_PULLAWAY_MARGIN) suppresses the curve Experimental trip ENTIRELY — including sharp curves, per
# explicit driver directive ("the lead car cannot pull away"). VTSC (vision) + MTSC (tiered map) with
# the decel envelope + sharp-curve firmer rate-limit remain the independent physical cap either way.
# Range kept tight (Gemini review: a far lead may already be EXITING the curve we are entering) so
# suppression only applies while the lead is genuinely pacing the same road segment.
CURVE_LEAD_PACE_DREL = 100.0  # m

# --- lead -------------------------------------------------------------------
SLOW_LEAD_DV   = 5.0          # m/s: lead this much slower than us -> closing -> Experimental
STOPPED_LEAD_V = 1.0         # m/s: lead below this -> stopped

# --- accelerate-zone --------------------------------------------------------
# Suppress the lowSpeed->Experimental trigger when we're slow but should be ACCELERATING into open
# road -- e2e/Experimental accelerates too timidly there. Two real cases: highway on-ramp merge, and
# stop&go where the lead pulled away leaving a big gap. Only ever REMOVES Experimental (safe: Chill is
# the baseline). Tune on the drive logs (vSet/dRel/vLead are recorded per event).
ACCEL_ZONE_DV        = 6.0   # m/s: set speed at least this far above v_ego => we want to accelerate (~13 mph)
ACCEL_ZONE_MIN_V     = 8.0   # m/s (~18 mph): redlight2pnw — the NO-LEAD accel-zone branch only counts
                             # above this. Below it a no-lead 'open road + high set' is a red-light /
                             # near-stop approach, NOT a merge; suppressing the low-speed Experimental
                             # hold there let Chill accelerate through the light. The on-ramp fix that
                             # created this gate operated at 38-39 mph, far above this floor.
GAP_OPEN_M           = 45.0  # m: a lead farther than this (and not slower) is "not blocking" -> open road
LEAD_PULLAWAY_MARGIN = 1.0   # m/s: lead counts as "not slower than us" if vLead >= vEgo - this
V_SET_MAX_KPH        = 200.0 # kph: above this, treat vCruise as the unset sentinel (255) -> set speed unknown

# --- pullaway2pnw (incident 2026-07-12 ~14:0x PT, city, op-long, Experimental) ------------------
# At ~17 mph behind a lead that pulled away, CES stayed Experimental (e2e slow to accelerate) and
# the truck lost the lead. Root cause: the redlight2pnw floor (ACCEL_ZONE_MIN_V, correct for the
# NO-LEAD red-light approach) also blocked the legitimate lead-pull-away case just below it, and
# the has-lead-far branch needs dRel > GAP_OPEN_M (45 m) — by which point city radar often drops
# the lead entirely (-> no-lead floor again). Driver's standing rule: "the lead car cannot pull
# away, that's unacceptable." The exception below is EVIDENCE-GATED and only ever ADOPTS CHILL
# below the floor when ALL hold: lead PRESENT in a sane band, lead genuinely OPENING (speed delta
# AND monotonic dRel rise over 3 spaced samples), the model does NOT want to stop (and has not
# wanted to stop recently — the yellow-light trap: a lead clearing through while ego must stop),
# and ego is actually moving. Any condition false -> exactly the redlight2pnw behavior.
PULLAWAY_MIN_V        = 2.0   # m/s (~4.5 mph): never fire from (near) standstill — a light turning
                              #   green releases via the existing logic, not this exception
PULLAWAY_DV           = 1.0   # m/s: lead must be at least this much FASTER than ego (opening now)
PULLAWAY_DREL_LO      = 5.0   # m: sane lead band — closer is not "pulled away"
PULLAWAY_DREL_HI      = 60.0  # m: farther city leads are unreliable radar tracks / already gone
PULLAWAY_SAMPLES      = 3     # monotonic dRel-rise evidence: this many spaced samples
PULLAWAY_SAMPLE_GAP_S = 0.25  # s min spacing between samples (evidence spans >= 0.5 s)
PULLAWAY_OPEN_EPS     = 0.3   # m: each sample must exceed the previous by at least this (real rise)
PULLAWAY_JUMP_M       = 10.0  # m: a dRel jump bigger than this between samples = lead swap /
                              #   radar reacquire -> evidence restarts from scratch
PULLAWAY_STOP_CLEAR_S = 2.0   # s: the model must not have wanted to stop for at least this long —
                              #   catches shouldStop flicker/lag while a lead clears a yellow light

# --- stophold2pnw (Tesla red-light lurch 2026-07-12 21:47:08Z; forensics in ---------------------
# drives/2026-07-12/tesla-redlight/CES_SILENCE_REPORT.md). Stopped behind a stopped lead at a red
# light, the ONLY active condition was slowLead; the moment the lead crept above STOPPED_LEAD_V it
# cleared, lowSpeed could not hold (its own 1.0 m/s floor), and the `stop` reason was masked by
# `not has_lead` — so CES adopted Chill at 0.4 m/s and the Chill MPC launched at up to 1.6 m/s^2
# toward the set speed (gas=False, strPrs=False: pure machine lurch). Two guards, both fail-safe
# (they only ever KEEP/ENTER Experimental, which stops for lights; no new acceleration path):
STOP_HOLD_MAX_V   = 3.0   # m/s (~7 mph): below this, model stop intent counts EVEN WITH a lead
                          #   present (A1) — at a creep, the LIGHT governs, not the lead. Above it
                          #   the original `not has_lead` gate stands (lead-following decel at
                          #   speed must not trip Experimental).
STANDSTILL_HOLD_V = 1.5   # m/s: below this, Experimental may not exit to Chill until the model's
                          #   stop intent has been CONTINUOUSLY clear (A2) — "the lead moved" is
                          #   not evidence of a green light; "the model agrees GO for 2 s" is.
STOP_CLEAR_HOLD_S = 2.0   # s of continuous shouldStop-clear required by A2 (mirrors
                          #   PULLAWAY_STOP_CLEAR_S — the same shouldStop flicker/lag envelope).

# --- standstill2pnw (hwy99 stop-and-go 2026-07-13; drives/2026-07-13/lightning-hwy99) ------------
# Field evidence, ces_events 11:34-11:38Z: chill<->experimental flapping every ~10-20 s at vEgo=0.0
# behind a lead at dRel 16-19 m — slowLead fires -> Experimental -> a radar lead DROPOUT at
# standstill decays the condition filter -> dwell expires -> Chill -> lead reacquired -> re-fires.
# Every flip changes the accel profile ("horse bucking"). And 9 of 14 standstill releases that
# drive launched in CHILL with a lead at only 9-14 m (aMax 1.6-2.6 m/s^2 within 3 s — the red-light
# "jolt"): records show stp (model shouldStop) is FALSE at standstill behind a lead, so the
# stophold2pnw A2 machinery (which arms off shouldStop) never engaged, the dwell expired to Chill
# while stationary, and the release was a hard Chill launch into a 9 m gap.
#
# The fundamental fix (driver-approved, car-agnostic — the same bug exists identically on every
# car with CES active, shadow mode included): a STANDSTILL LATCH in the decision core.
#   LATCH   below STANDSTILL_LATCH_V, an Experimental machine may NOT demote to Chill — there is
#           zero benefit to Chill at 0 mph and every demotion there sets up a lurch. Implemented as
#           a DEMOTION GATE (a pure per-tick predicate on v_ego), NOT a timer pause: no timer is
#           ever frozen (nothing to leak), the dwell keeps accumulating, and the latch releases the
#           instant v_ego rises — the no-lead release path is byte-identical to stophold2pnw.
#   HOLD    on release from standstill WITH a close lead (dRel <= STANDSTILL_RELEASE_DREL seen at
#           standstill), Experimental is held until v_ego > STANDSTILL_RELEASE_V OR the gap opens
#           past STANDSTILL_RELEASE_CLEAR_DREL — the launch into a short gap stays model-governed
#           (smooth) instead of a Chill MPC launch (jolt). Generalizes the A2 departure hold (which
#           only arms off model shouldStop) to the stopped-behind-lead case where stp stays False.
#   PROMOTE at standstill in CHILL with the ladder wanting Experimental (only slowLead/stop can be
#           raw-active at 0), entry bypasses the CHILL_MIN_DWELL_S cooldown and the ~1 s filter
#           charge (both were fighting the trigger at 0 mph — the 11:34 flapping), gated instead on
#           STANDSTILL_PROMOTE_LEAD_S of SUSTAINED lead presence (radar-ghost debounce: a 1-tick
#           dRel flicker cannot promote). Being in Experimental at standstill is strictly safer,
#           and the latch makes the promoted state absorbing — no reverse oscillation is possible.
# PRECEDENCE vs pullaway2pnw (documented, tested): at standstill the latch always wins — pullaway
# is dead there anyway by its own PULLAWAY_MIN_V (2.0) floor. During the release hold (a deliberate
# below-floor exception window), the hold wins while the gap is short; the moment the gap opens
# past STANDSTILL_RELEASE_CLEAR_DREL (or v_ego > STANDSTILL_RELEASE_V) the hold disarms and the
# pullaway/accel-zone Chill adoption proceeds exactly as shipped — pullaway matters once moving.
STANDSTILL_LATCH_V            = 0.5   # m/s: below this the car is "at standstill" — demotion gated.
                                      #   Deliberately BELOW creep speed: the 21:47 lurch replay's
                                      #   0.4 m/s creep is standstill; A2 (1.5) covers 0.5..1.5.
STANDSTILL_RELEASE_V          = 5.0   # m/s (~11 mph): the launch is done — hand back to the ladder
STANDSTILL_RELEASE_DREL       = 20.0  # m: lead within this at standstill arms the release hold
                                      #   (field: flapping at 16-19 m, jolt launches into 9-14 m)
STANDSTILL_RELEASE_CLEAR_DREL = 25.0  # m: gap opened past this disarms the hold (hysteresis vs
                                      #   ARM so radar dRel jitter at the edge can't flap the hold)
STANDSTILL_PROMOTE_LEAD_S     = 0.5   # s of continuous lead presence at standstill required by the
                                      #   no-cooldown promotion (single-tick radar ghosts excluded)

# --- cesnochill2pnw (Tesla red-light jolt, drive 2026-08-15: drives/2026-08-15/tesla-redlight-jolt) --
# Field evidence: stopped 11.1 s at a red light with NO lead, `reason` sequence stop -> chill ->
# standstillHold -> stopHold -> lowSpeed. The `chill` tick handed longitudinal to the ACC/MPC path
# (which does not stop for lights) for one cycle at v~=0, and Chill's MPC accelerated toward the
# 12.5 m/s set speed before stopHold/standstillHold caught back up -> the jolt (aEgo 1.8 m/s^2).
# Root cause: BOTH standstill demotion paths above (the STANDSTILL_LATCH_V gate and the A2
# STANDSTILL_HOLD_V/STOP_CLEAR_HOLD_S gate) are conditional — a model_should_stop dropout that
# outlasts STOP_CLEAR_HOLD_S at a creep speed between the two thresholds (or any other tick that
# slips through the ladder/dwell/filter machinery) can still fall through to the `else: chill`
# branch while the car has not genuinely moved.
#
# Driver directive (verbatim): "CES is allowed to go to chill as soon as the car is moving to
# ensure smooth acceleration but NEVER before." Fix: an UNCONDITIONAL latch, applied as a final
# override AFTER the whole existing decision (v1: see ConditionalExperimentalSwitching's
# update_decision wrapper; CES2: Ces2Core.update_decision) so it WINS over any chill decision from
# ANY internal path, timer, or model signal — not just the two gates above.
#
# Gemini review (round 1) catch: a PURE speed threshold cannot tell "decelerating/creeping through
# 0.9 m/s toward a stop" from "accelerating away through 0.9 m/s on a launch" — the original design
# only armed below NOCHILL_STOP_V(0.5), leaving the WHOLE A2 creep band (0.5-1.5 m/s) open to the
# exact field bug if approached from ABOVE without ever dipping under 0.5 (e.g. a steady creep that
# settles at 0.7-1.4 m/s and stays there).
#
# Round 2 tried fixing this with an a_ego (acceleration) direction gate on top of the speed
# threshold. REVERTED (Gemini review, round 2 — CRITICAL, two separate bugs): requiring v_ego >
# RELEASE_V AND a_ego > a floor SIMULTANEOUSLY is not equivalent to "eventually launches" — a_ego
# is not required to STAY positive once the car is moving, so a gentle/gradual launch, or one that
# happens to level off (e.g. cruise catching the target speed) right around the release band, can
# sit at v > RELEASE_V with a_ego <= the floor indefinitely: a genuine, PERMANENT wedge stuck in
# Experimental to highway speed, not a hypothetical corner case. Separately, a_ego dithering across
# the floor in the 0.8-1.5 m/s band (ordinary accelerometer/estimator noise) flapped the release
# condition at up to 100 Hz. Both are correctness bugs in the AND-of-two-live-conditions shape
# itself, not tuning issues, so a_ego is REMOVED ENTIRELY — no acceleration term anywhere below.
#
# Fix (round 3): back to a PURE v_ego Schmitt trigger — the round-1 shape, proven wedge-free and
# flap-free — just with the ARM threshold RAISED to cover the creep band directly, instead of trying
# to distinguish approach direction:
#   ARM     v_ego < NOCHILL_ARM_V — force Experimental, unconditionally.
#   STAY    once armed, stays armed regardless of model_should_stop flicker, dwell timers, or
#           intermediate v_ego wobble — only v_ego rising past RELEASE_V can clear it.
#   RELEASE v_ego > NOCHILL_RELEASE_V — hand back to Chill, unconditionally.
# NOCHILL_ARM_V(1.0) < NOCHILL_RELEASE_V(1.3) is the ONLY thing preventing flap (a twitch at the
# boundary can't oscillate across a two-sided gap) and is now the ENTIRE anti-flap mechanism — no
# other condition gates either transition.
#
# WEDGE-IMPOSSIBILITY PROOF (the property that matters most): RELEASE depends on v_ego ALONE —
# checked every ~10 ms tick against one fixed threshold, no AND term, no debounce, no memory beyond
# the single armed/not bit. So ANY tick where v_ego > NOCHILL_RELEASE_V releases the latch, full
# stop, with no further condition to simultaneously satisfy. A real launch is, by definition, a
# monotonic (over the relevant span) rise in v_ego from ~0 through every intermediate speed
# including RELEASE_V, so it MUST produce at least one such tick — the latch cannot fail to see it,
# and cannot require a second, independently-timed signal to also be true on that same tick. This is
# the identical one-sided-predicate shape as the original, already-proven round-1 design (only the
# two threshold VALUES changed) — no new wedge surface exists, and the a_ego wedge/flap class is
# structurally impossible now that a_ego does not appear in the predicate at all.
#
# Residual (accepted, documented trade-off): a steady creep WITH A STALE model_should_stop strictly
# inside [ARM_V, RELEASE_V] (1.0-1.3 m/s here) approached FROM ABOVE without ever dipping under
# ARM_V is not caught by this latch alone. This is covered instead by the pre-existing A2 stopHold
# gate (STANDSTILL_HOLD_V=1.5, STOP_CLEAR_HOLD_S=2.0 — see stophold2pnw above), which already holds
# Experimental through a genuine BRAKING approach (model_should_stop True, v < 1.5) independent of
# this latch. The residual gap is a real stop's model_should_stop dropping and staying clear >2 s
# while creeping in a <=0.3 m/s-wide band — narrower than the original field incident (which
# reproduced at v~0.7, inside ARM_V=1.0 and thus now fully covered directly) — and is the smallest
# gap the ARM_V<RELEASE_V hysteresis geometry allows.
#
# Correctness argument (no-wedge proof, restated for the final design): Experimental only ever
# HOLDS a stop (never launches on its own), so while latched the car cannot move away except via a
# model-commanded, held launch; ANY v_ego crossing above RELEASE_V is BY CONSTRUCTION part of that
# launch (guaranteed to be observed — see the wedge-impossibility proof above), at which point
# handing back to Chill (smoother acceleration) is exactly what is wanted and the existing
# ladder/dwell machinery resumes unmodified. The latch is a pure per-tick predicate on live v_ego
# plus one bit of Schmitt-trigger memory (armed/not) — no timer, no second live signal in the
# release condition — so it cannot wedge Experimental forever.
#
# Telemetry (Gemini review, round 1 — kept in round 3): while armed, status is UNCONDITIONALLY
# "stopLatch" for the WHOLE armed episode (not just the ticks where the internal core would
# otherwise show chill), and the override never touches `_dwell` — the earlier (round-1) design let
# the core's own dwell-driven status show through in between overrides, which could flap between a
# stale ladder reason and "stopLatch" every ~EXP_MIN_DWELL_S. A consistent tag for the entire hold
# is worth more to field forensics than surfacing the (now largely redundant, since the latch is
# doing the actual holding) internal ladder reason underneath it.
#
# NOT the same knobs as STANDSTILL_LATCH_V/STANDSTILL_RELEASE_V above (that machinery is about
# holding through a close-lead LAUNCH, wide 5.0 m/s release); this latch is about never leaving
# Experimental before a genuine launch at ALL, so its release margin is deliberately small.
NOCHILL_ARM_V     = 1.0   # m/s: below this -> arm, unconditionally (covers the field creep, ~0.7)
NOCHILL_RELEASE_V = 1.3   # m/s: above this -> release, unconditionally (hysteresis vs ARM_V is the
                          #   ENTIRE anti-flap mechanism; no other condition gates either edge)

# --- debounce / dwell (de-flap) ---------------------------------------------
# Drive log showed heavy flapping in stop&go (median 2.3 s between switches, 30 flips/min). Two
# asymmetric dwell gates kill the sawtooth: once in Experimental, hold it EXP_MIN before returning to
# Chill; once in Chill, hold it CHILL_MIN (a re-entry cooldown) before flipping back to Experimental.
FILTER_TAU       = 1.0       # s, FirstOrderFilter time constant per condition
THRESHOLD        = 0.63      # filter level ~= "true for ~1 s"
EXP_MIN_DWELL_S  = 8.0       # s min time in Experimental before it may return to Chill (was MIN_DWELL_S=4)
CHILL_MIN_DWELL_S = 5.0      # s min time in Chill before it may re-enter Experimental (re-entry cooldown)

# --- gentle profile (CESMode==1 "Light", car-agnostic) ----------------------
# On a winding highway the CURVE condition can flip chill<->experimental rapidly, and experimental's
# e2e curve braking can feel jerky ("too aggressive at curve entrance"). The gentle profile (a) hands
# curve speed control ENTIRELY to VTSC — which brakes smoothly and is decel-limited — by NOT tripping
# Experimental for curves (curves suppressed in the CES decision), and (b) lengthens the dwell so the
# remaining triggers (stops / slow leads) can't flip-flop. Experimental is then reserved for where e2e
# genuinely helps: stop lights and closing on a slow lead.
# USER-SELECTED via CESMode==1 (Light) on ANY car — no car/fingerprint gating.
GENTLE_EXP_MIN_DWELL_S   = 12.0  # hold Experimental longer before dropping back to Chill
GENTLE_CHILL_MIN_DWELL_S = 8.0   # longer re-entry cooldown

# --- CESMode (3-way master selector) ----------------------------------------
# light-ces-gentle: the master is now an INT param `CESMode` (0=Off, 1=Light, 2=Standard) instead of
# the old BOOL `ConditionalExperimentalSwitching`. CESMode picks the *profile/aggressiveness*; the
# on-screen 3-state button (CESButtonState) still cycles Chill/CES/Experimental within whatever mode.
#   0 (Off)      -> CES + VTSC disabled entirely (behavior-neutral; == old bool=false)
#   1 (Light)    -> CES + VTSC enabled with the GENTLE profile on ANY car: VTSC GENTLE_PROFILE (soft
#                   decel + slow recovery, anti-sawtooth), CES hands curves entirely to VTSC (curve
#                   condition suppressed so no chill<->experimental flapping), longer gentle dwell.
#   2 (Standard) -> CES + VTSC enabled with the DEFAULT tune on ANY car: VTSC DEFAULT_PROFILE, CES
#                   trips Experimental for curves, normal dwell.
# The gentle profile is a USER choice via CESMode, available on every car (default Off for all).
CES_MODE_OFF      = 0
CES_MODE_LIGHT    = 1   # full gentle behavior on any car
CES_MODE_STANDARD = 2   # today's default tune on any car


def ces_enabled(mode: int) -> bool:
  """True when CES (and VTSC, which rides the same master) should run: any non-Off mode."""
  return int(mode) > CES_MODE_OFF


def ces_is_gentle(mode: int) -> bool:
  """True when the gentle profile applies (Light). Standard / Off -> default tune (irrelevant if Off)."""
  return int(mode) == CES_MODE_LIGHT


# silentexc3pnw (Rule 2): read_ces_mode's two reads used to fall back silently. Each failure is now logged -- the first
# at once, then at most one line per this many seconds, counting the failures since the previous line (the
# twistyr2pnw / silentexc2pnw pattern) -- with its own state per caller (`who`) and per read, so one consumer's failure
# can never hide another's first line.
CES_MODE_READ_ERR_LOG_S = 60.0
_ces_mode_read_err = {}   # (who, param key) -> [monotonic time of the last log line, or None; failures since it]


def _ces_mode_read_failed(who, key, e, consequence) -> None:
  """Log one failed read_ces_mode read (called from inside its except, so cloudlog.exception has the traceback).
  Guarded: read_ces_mode never raised before and the UI overlay calls it with no try of its own, so a failing
  logger must not change its result or make it raise."""
  try:
    st = _ces_mode_read_err.setdefault((who, key), [None, 0])
    st[1] += 1
    now = time.monotonic()
    if st[0] is None or now - st[0] >= CES_MODE_READ_ERR_LOG_S:
      n, st[0], st[1] = st[1], now, 0
      cloudlog.exception(f"read_ces_mode ({who}): {key} unreadable ({type(e).__name__}) -- {consequence} " +
                         f"({n} failure(s) since the last log)")
  except Exception:
    pass                          # the fallback already happened in read_ces_mode; logging must not raise


# cesmodehold2pnw (owner decision 2026-09-14, Fable's silentexc3pnw Q1): a failed read used to switch that caller's CES
# master Off on the very read that failed -- no CES decisions / ICBM curve slowdowns (CES), no VTSC curve slowdowns
# (VTSC), overlay and its NO-SIGNAL dead-man hidden (UI). Each caller now keeps its last GOOD mode for this long after
# its first failed read, then falls back to what that read computed. BOUNDED on purpose: what raises here in practice is
# a params_keys.h / params_pyx.so mismatch, which is NOT transient -- an unbounded hold would drive the old mode forever
# on a device whose params layer is broken, with nothing on screen saying so.
CES_MODE_HOLD_S = 10.0
# who -> [last good mode (None before the first good read), monotonic time of the first failed read of this outage
# (None while reads are fine), fallback in force, that last good mode came from the LEGACY migration]. Module state, so
# per PROCESS and per caller: CES holds in selfdrived, VTSC in plannerd, the overlay in ui, each on its own reads.
_ces_mode_hold_st = {}


def _ces_mode_log(fn, msg) -> None:
  """cesmodehold2pnw: one change-only hold log line. Guarded like _ces_mode_read_failed -- read_ces_mode never raised
  before and the UI overlay calls it with no try of its own, so a failing logger must not make it raise."""
  try:
    fn(msg)
  except Exception:
    pass                          # the logger itself is what failed; there is nowhere left to report it


def _ces_mode_last_was_migrated(who) -> bool:
  """cesmodehold2pnw (Fable): did `who`'s last good mode come from the legacy-bool migration? Only then does a failed
  LEGACY read hide something -- otherwise CESMode itself is readable and is the source of truth."""
  st = _ces_mode_hold_st.get(who)
  return bool(st is not None and len(st) > 3 and st[3])


def _ces_mode_hold(who, mode, ok, migrated=False) -> int:
  """cesmodehold2pnw: what read_ces_mode returns. `mode` is what this read computed, `ok` is False when a read raised.
  A good read stores the mode and ends any hold; a failed read returns the last good mode until CES_MODE_HOLD_S has
  passed, then `mode` (the fallback). With no good read yet -- a failure at startup -- the fallback applies at once.
  Change-only log lines: hold started / hold expired (or nothing to hold) / readable again."""
  try:
    st = _ces_mode_hold_st.setdefault(who, [None, None, False, False])
    if ok:
      if st[1] is not None:
        _ces_mode_log(cloudlog.warning, f"read_ces_mode ({who}): CES master readable again after " +
                      f"{time.monotonic() - st[1]:.1f} s -- mode {mode}")
      st[:] = [mode, None, False, migrated]
      return mode
    now = time.monotonic()
    if st[1] is None:                        # first failed read of this outage
      st[1] = now
      if st[0] is not None:
        _ces_mode_log(cloudlog.warning, f"read_ces_mode ({who}): CES master unreadable -- {who} holds its last good " +
                      f"mode {st[0]} for up to {CES_MODE_HOLD_S:.0f} s")
    if st[0] is not None and now - st[1] < CES_MODE_HOLD_S:
      return st[0]
    if not st[2]:                            # change-only: the hold is over, or there was nothing to hold
      st[2] = True
      why = (f"the {CES_MODE_HOLD_S:.0f} s hold of mode {st[0]} expired" if st[0] is not None else
             "no good read yet (it failed at startup)")
      _ces_mode_log(cloudlog.error, f"read_ces_mode ({who}): CES master unreadable, {why} -- {who} falls back to " +
                    f"mode {mode} until a read succeeds")
    return mode
  except Exception as e:
    # only the hold bookkeeping can raise here; the read already produced `mode`, so that is what the caller gets
    _ces_mode_read_failed(who, "mode hold", e, f"{who} uses this read's own mode {mode}, with no hold")
    return mode


def ces_mode_lost(who) -> bool:
  """cesmodehold2pnw: True while `who`'s CES master is unreadable AND the fallback is in force (the hold expired, or
  there was no good mode to hold) AND that fallback is not what the driver chose -- the last good mode was non-Off, or
  is unknown. False while reads succeed, DURING the hold (the held mode is the driver's, and CES is still running and
  publishing on it), and when the last good mode was Off (the fallback matches it, so nothing is lost). The UI
  overlay's NO-SIGNAL alarm reads this."""
  st = _ces_mode_hold_st.get(who)
  return bool(st is not None and st[2] and (st[0] is None or st[0] != CES_MODE_OFF))


def read_ces_mode(params, who="unnamed") -> int:
  """Read the CESMode INT param (source of truth). Back-compat: if CESMode reads a genuine 0 (including UNSET, which
  reads its "0" default) but the old BOOL `ConditionalExperimentalSwitching` is set, treat that as Standard (2). Used by
  BOTH the CES and VTSC runtime readers so they always agree. Never raises (the UI overlay calls it with no try of its
  own).
  silentexc3pnw: `who` names the caller in the failure log (CES in selfdrived, VTSC in plannerd, the CES overlay in
  the UI). An unset CESMode reads its "0" default and an unset legacy bool reads False: neither raises or logs.
  cesmodehold2pnw: when a read fails, `who` keeps its last good mode for CES_MODE_HOLD_S, then falls back (see
  _ces_mode_hold); after a FAILED CESMode read the legacy bool is not consulted at all (see below)."""
  ok = True
  try:
    mode = int(params.get("CESMode", return_default=True) or 0)
  except Exception as e:
    # silentexc3pnw: what raises here in practice is an UnknownKeyName from a params_keys.h / params_pyx.so mismatch
    # (a malformed stored INT does not: Params returns the default for it and warns itself).
    mode, ok = CES_MODE_OFF, False
    _ces_mode_read_failed(who, "CESMode", e, f"{who} keeps its last good mode for up to {CES_MODE_HOLD_S:.0f} s " +
                          "(none before the first good read), then treats the CES master as Off while this lasts " +
                          "(the legacy ConditionalExperimentalSwitching bool is NOT consulted on a failed read)")
  # cesmodehold2pnw (owner decision 2026-09-14, Fable's silentexc3pnw Q2): the legacy migration applies only when
  # CESMode genuinely READS 0 -- an unset CESMode reads its "0" default, so devices that only ever had the old bool
  # still migrate exactly as before. It must NOT stand in for a FAILED read: the bool cannot tell Light from Standard
  # (settings/toggles.py writes it as `CESMode > 0`), so consulting it after a failure hands a Light driver the
  # STANDARD tune -- a silently different car -- where Off is at least the state the overlay is alarming about.
  migrated = False
  if ok and mode == CES_MODE_OFF:
    try:
      if params.get_bool("ConditionalExperimentalSwitching"):
        mode, migrated = CES_MODE_STANDARD, True
    except Exception as e:
      # silentexc3pnw: was `except Exception: pass`. The back-compat is skipped, so mode stays the Off that CESMode
      # itself reported.
      # Fable (cesmodehold2pnw review): a LEGACY failure arms the hold ONLY when the last good mode came from the
      # migration itself. On a device that does not migrate, CESMode -- the source of truth -- was read fine and says
      # Off, so holding would override a driver who just picked Off for CES_MODE_HOLD_S and would leave
      # ces_mode_lost True (a permanent NO-SIGNAL alarm) for as long as the legacy key stays unreadable, over a
      # perfectly readable master. On a migrating device the effective mode really is lost, so the hold still applies.
      ok = not _ces_mode_last_was_migrated(who)      # ok False == arm the hold
      _ces_mode_read_failed(who, "ConditionalExperimentalSwitching", e,
                            (f"legacy back-compat skipped: {who} keeps its last good (migrated) mode for up to " +
                             f"{CES_MODE_HOLD_S:.0f} s, then treats the CES master as Off while this lasts") if not ok
                            else (f"legacy back-compat skipped: CESMode itself read {mode} (Off) and is used as-is; " +
                                  "the hold is not armed, because the last good mode did not come from the migration"))
  return _ces_mode_hold(who, mode, ok, migrated)


# --- button override states (CESButtonState mem param) ----------------------
BTN_CES  = 0   # CES decides (default)
BTN_CHILL = 1  # forced Chill
BTN_EXP   = 2  # forced full Experimental

# --- event logging (CES_EVENT_LOG: persistent "each adoption" + breadcrumb trail) -------
TICK_S          = 1.0                    # s between heartbeat breadcrumb records (dense for the test drive)
HWY_SPEED_LIMIT = 55 * CV.MPH_TO_MS      # OSM speed limit >= this => coarse "highway" guess
HWY_VEGO        = 55 * CV.MPH_TO_MS       # or sustained speed >= this (authoritative = GPS+OSM+300ft in analysis)

# ---------------------------------------------------------------------------------------------
# curvefloor2pnw (2026-09-05) — a POSTED-LIMIT FLOOR for the Lightning's ICBM (stock-ACC) path.
#
# Evidence (drives/2026-08-11, 06:52:45-57 PT): spdLim 11.2 m/s (25 mph), mapd's target 7.3 m/s
# SUSTAINED for 12 s, icbmSrc 'map', and the stock set speed tapped 23.7 -> 7.15 m/s (16 mph) on a
# 25 mph road. icbmratchet2pnw does NOT catch this: it confirms single-tick OUTLIER drops
# (ICBM_RATCHET_OUTLIER_DROP_MS = 3.0 over CONFIRM_S = 0.6), and this was a sustained map target.
# VTSC has floored its cap at the posted limit since 2026-07-01; the ICBM path never had one.
#
# SCOPE IS DELIBERATELY NARROW — low limits only. Fable review 2026-09-05 rejected the original
# branch's all-roads floor: on a 45 mph surface road with an R=80 m bend, flooring the cap at the
# limit demands 20.1^2/80 = 5.05 m/s^2 lateral — 2x A_LAT_TARGET, above the 3.0 fail-safe clip and
# at/above the Lightning's measured ~4.5 m/s^2 hands-off ceiling. It would understeer into a
# takeover. At <= 13.4 m/s (30 mph) the lateral demand only exceeds 3.0 m/s^2 for R < 60 m, i.e. a
# parking-lot turn, so "the posted limit is physically holdable" is guaranteed rather than assumed.
# That is the whole reason this bound exists — do not raise it without redoing that arithmetic.
ICBM_FLOOR_MAX_LIMIT = 13.5    # m/s; INCLUSIVE of a real 30 mph = 30 * 0.44704 = 13.4112 m/s.
                               # Fable 2026-09-05: the original 13.4 EXCLUDED 30 mph by 1.1 cm/s, so
                               # the constant, the commit message and the test all described a scope
                               # that could never occur. 13.5 makes the stated scope the real one;
                               # the holdability argument is unchanged at that margin.
# The logs show spd_lim flickering (11.2 <-> 8.9 at 06:52:40 and 19:05:16). A 2.3 m/s step passes
# under the ratchet's 3.0 m/s outlier band, so an un-debounced floor would chase it and produce
# SET-button tap flicker. Require a real move before the floor follows.
# TIME debounce, not a value band. Fable 2026-09-05 measured why a value band cannot work here: a
# real 5 mph step is 2.235 m/s, so ANY deadband wide enough to swallow the observed 11.2 <-> 8.9
# flicker also swallows every genuine 20->25 and 25->30 transition -- permanently. On the 2026-08-11
# route that turned the intended 25 mph floor into a 20 mph floor (7 of 22 in-scope rises held
# forever; the event window floored at 8.45 m/s = 19 mph, not the 10.73 = 24 mph the feature was
# written for). Flicker and a real change are the SAME SIZE; only their PERSISTENCE differs.
ICBM_FLOOR_RISE_HOLD_S = 3.0   # a higher limit must persist this long before the floor follows it up


# icbmrestorecap2pnw (driver report 2026-09-13 15:46 PT, live): a curve episode tapped the stock set
# 45 -> 27 mph just as the truck entered a 25 mph zone; the curve cleared and RESTORE tapped the set
# 27 -> 60 over 13 s -- the pre-curve ceiling -- with the posted limit reading 25 the entire time.
# Only a slower lead car kept the truck near 30. Restore "gives back what the curve took", and it
# had no notion that the road it was giving it back on had changed.
#
# The cap is the posted limit PLUS the driver's own habit, measured from his own log rather than
# chosen: 9,063 weekend ticks where HE set the speed (stock ACC on, ICBM silent, limit known) sit at
# the limit to +5 mph (p50 0..5, p75 ~5 across 35/45/55/60/65 zones). Restoring above that would be
# commanding a speed he himself does not choose.
ICBM_RESTORE_LIMIT_MARGIN_MS = 5.0 * 0.44704   # m/s; +5 mph over the posted limit
ICBM_RESTORE_LIMIT_RISE_HOLD_S = 3.0           # a HIGHER limit must persist this long (same reasoning
                                               # as ICBM_FLOOR_RISE_HOLD_S: flicker and a real 5 mph
                                               # step are the same size; only persistence separates them)
ICBM_RESTORE_LIMIT_STALE_S = 30.0              # how long an UNKNOWN reading keeps the last known limit
# gaswin2pnw (owner decision 2026-09-24, Fable terwilliger2pnw finding 4): the longest a driver GAS press may last and
# still restore to the pre-curve ceiling after the lift (terwilliger2pnw). It used to reuse ICBM_RESTORE_WINDOW_S
# (45 s), so a long press on open road declined ("gasLong") and left the set stuck at the curve cap. Longer is safe
# because the ceiling does not go stale during the press: note_limit / note_sa_zone keep the posted-limit and zone cap
# fresh every tick, and after the lift the ordinary restore runs with every restore guard. ICBM_RESTORE_WINDOW_S
# (ces_pnw.py) is unchanged for every other use.
ICBM_GAS_RESUME_MAX_S = 120.0
_ICBM_RCAP_ERR_LOG_S = 30.0                    # throttle for the failure log below (~4 Hz caller)
_icbm_rcap_err_last = -1e9


def icbm_restore_limit(spd_lim: float, state=None, now: float = 0.0):
  """Debounced posted limit that CAPS a restore. Returns (limit_ms, state); limit_ms 0.0 = no cap.

  `state` is (limit, last_seen, pending_limit, pending_since) or None -- carry it across calls.

  ASYMMETRIC, and deliberately the opposite trade-off from a floor, because a cap fails the other way:
    * a LOWER known limit is adopted IMMEDIATELY -- a lower cap only holds the restore sooner;
    * a HIGHER known limit must PERSIST for ICBM_RESTORE_LIMIT_RISE_HOLD_S before the cap rises;
    * an UNKNOWN reading (0, negative, non-finite) does NOT lift the cap. A missing limit is not
      evidence that the limit went up -- mapd drops the limit on rural roads routinely, and letting a
      dropout inside a real 25 zone release the restore toward the pre-curve set is the exact bug
      this exists to prevent. The last known limit is held for ICBM_RESTORE_LIMIT_STALE_S, and only
      after that does it become genuinely unknown (no cap), so a town's 25 cannot follow the truck
      twenty miles down an unmapped highway and cap a restore there.

  Unlike icbm_floor_limit there is NO upper scope bound: a floor can demand an unholdable lateral
  load on a fast road, but a cap can only ever make a restore more conservative, at any speed.

  Pure and TOTAL: nothing raises. Any nonsensical `state` is treated as no history. This matters more
  than it looks: the state is CARRIED across ticks, and _icbm_step swallows exceptions. A state that
  made this raise would therefore raise on every following tick too and silently disable ICBM for the
  rest of the drive -- so a malformed carry-over is discarded, never trusted (Gemini review
  2026-09-13: the first version validated only `lim` and would raise on a bad pending entry).
  """
  try:
    return _icbm_restore_limit(spd_lim, state, now)
  except Exception:
    # Rule 2 (Fable review): falling back to "no cap" is the right BEHAVIOUR, but doing it silently
    # would mean the fix quietly switches itself off on every tick with nothing in the log --
    # precisely the failure CLAUDE.md bans. Throttled, because this runs at ~4 Hz.
    global _icbm_rcap_err_last
    t_log = time.monotonic()
    if t_log - _icbm_rcap_err_last >= _ICBM_RCAP_ERR_LOG_S:
      _icbm_rcap_err_last = t_log
      cloudlog.exception("icbm_restore_limit failed -- restore posted-limit cap DISABLED (no cap) this tick")
    return 0.0, None                               # no cap, no history: the pre-fix behaviour


def _icbm_restore_limit(spd_lim, state, now):
  now = float(now)
  if not (now == now):
    return 0.0, None
  lim, seen, pend, since = 0.0, None, None, None
  if state:
    try:
      lim, seen, pend, since = state
      lim = float(lim)
      seen = None if seen is None else float(seen)
      pend = None if pend is None else float(pend)
      since = None if since is None else float(since)
      if not all(x is None or x == x for x in (lim, seen, pend, since)) or (pend is None) != (since is None):
        raise ValueError("inconsistent state")
    except (TypeError, ValueError):
      lim, seen, pend, since = 0.0, None, None, None   # malformed carry-over -> discard, never trust
  try:
    v = float(spd_lim)
  except (TypeError, ValueError):
    v = 0.0
  known = (v == v) and v > 0.0 and v != float("inf")

  if not known:
    if lim > 0.0 and seen is not None and (now - seen) <= ICBM_RESTORE_LIMIT_STALE_S:
      return lim, (lim, seen, None, None)          # hold the last known limit through a dropout
    return 0.0, None                               # genuinely unknown -> no cap (pre-fix behaviour)
  if lim <= 0.0 or v <= lim + 1e-6:
    return v, (v, now, None, None)                 # first reading, same, or LOWER -> adopt now
  # a rise: keep the lower cap until the higher limit has stood for the full hold
  if pend is None or abs(pend - v) > 1e-6:
    return lim, (lim, now, v, now)                 # new candidate -> start its clock (still SEEN)
  if (now - since) >= ICBM_RESTORE_LIMIT_RISE_HOLD_S:
    return v, (v, now, None, None)                 # it stuck -> adopt
  return lim, (lim, now, pend, since)


# sazoneset2pnw (driver directive 2026-09-13): a curve restore returns to the speed from BEFORE the curve --
# unless the posted limit DROPPED while the curve episode was running, in which case the latched ceiling
# belongs to a road the truck has left (the 15:46 incident: latched at 60 on a 45 road, restored to 60 in a
# 25 zone). Only then is the restore capped, and the cap is the zone speed the driver would have got on
# entering that zone: "the same percentage above the speed limit as I was driving before".
ICBM_LIMIT_DROP_EPS_MS = 1.0     # m/s; a real limit step is 5 mph = 2.235 m/s, float/rounding noise is far below


def icbm_stale_zone_cap(ceiling, latch_limit, limit_now, proportional: bool):
  """(cap_ms, why) for a curve restore whose ceiling may be stale, or (None, None) when it is not.

  NOT stale -- restore all the way to the pre-curve set, even if that is more than 5 mph over the limit --
  when no limit is known now (no evidence of a new zone), or the limit now is the same as or higher than
  the limit at latch.

  STALE when the limit now is lower than at latch, or was unknown at latch and is known now:
    * `proportional` (speedadjust AutoSpeedReduce >= 2, i.e. the driver has asked for zone speeds) and the
      latch-time ratio ceiling/latch_limit is >= 1 -> limit_now x ratio, "prop". The ratio is taken from
      the EPISODE's own latch, not speedadjust's live ratio: Fable measured that ICBM's own SET- taps are
      read by speedadjust as driver overrides and re-anchor its ratio to the tapped-down set mid-curve.
    * otherwise (proportional zone speeds off, latch limit unknown, or the driver was under the limit)
      -> limit_now + ICBM_RESTORE_LIMIT_MARGIN_MS, "limit5" -- the icbmrestorecap2pnw backstop.
  The caller keeps the result STICKY (only ever lowered) for the episode: a limit that later rises again
  must not resume a restore past it -- "no memory", in the driver's words.

  Pure and total: anything non-numeric is treated as unknown."""
  def _num(x):
    try:
      v = float(x)
    except (TypeError, ValueError):
      return None
    return v if (v == v and v != float("inf") and v > 0.0) else None
  ceiling, latch_limit, limit_now = _num(ceiling), _num(latch_limit), _num(limit_now)
  if ceiling is None or limit_now is None:
    return None, None
  if latch_limit is not None and limit_now >= latch_limit - ICBM_LIMIT_DROP_EPS_MS:
    return None, None
  if proportional and latch_limit is not None and ceiling / latch_limit >= 1.0:
    return limit_now * (ceiling / latch_limit), "prop"
  return limit_now + ICBM_RESTORE_LIMIT_MARGIN_MS, "limit5"


def icbm_floor_limit(spd_lim: float, prev: float, now: float = 0.0, pending=None):
  """Debounced posted limit for the ICBM floor. Returns (floor, pending).

  `pending` is (candidate_limit, first_seen_monotonic) or None -- carry it across calls.

  ASYMMETRIC, and that is the whole correctness of this function:
    * a LOWER limit is followed IMMEDIATELY -- a lower floor permits more slowing, never less, so it
      is always the safe direction and must not wait;
    * a HIGHER limit must PERSIST for ICBM_FLOOR_RISE_HOLD_S before the floor follows it up.

  Why persistence rather than magnitude: flicker and a real change are the same size (both are one
  5 mph step = 2.235 m/s), so no value deadband can tell them apart -- the earlier 2.5 m/s band held
  every genuine step-up forever and turned the intended 25 mph floor into a 20 mph one. What actually
  separates them is that flicker does not last.

  Pure and total: any non-finite or nonsensical input yields (0.0, None) -- no floor, which is the
  pre-curvefloor2pnw behaviour and therefore the fail-safe direction.
  """
  try:
    spd_lim = float(spd_lim); prev = float(prev); now = float(now)
  except (TypeError, ValueError):
    return 0.0, None
  if not (spd_lim == spd_lim) or spd_lim <= 0.0 or spd_lim > ICBM_FLOOR_MAX_LIMIT:
    return 0.0, None                            # unknown, or too fast to guarantee holdability
  if prev <= 0.0 or spd_lim <= prev:
    return spd_lim, None                        # first reading, or a DROP -> take it at once
  # A rise: hold the old floor until the new, higher limit has stood for the full window.
  cand, since = pending if pending else (None, None)
  if cand is None or abs(cand - spd_lim) > 1e-6:
    return prev, (spd_lim, now)                 # new candidate -> start its clock
  if (now - since) >= ICBM_FLOOR_RISE_HOLD_S:
    return spd_lim, None                        # it stuck -> adopt it
  return prev, (cand, since)                    # still settling -> keep the lower floor
