"""
CES (Conditional Experimental Switching) — tunable constants.

ALL values are starting points to be finalized on real drive logs (see CES.md "calibration
anchors": I-5 Terwilliger ~2.0 m/s² @ 50 mph must trip; the R≈550 m curve must be easy @70 / hard
@90). Lateral acceleration is v²·curvature, so curve triggering is speed-adaptive.
"""
from openpilot.common.constants import CV

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
# settles at 0.7-1.4 m/s and stays there). Fix: gate on DIRECTION too, via a_ego:
#   ARM     v_ego < NOCHILL_ARM_V (1.5, covers the whole A2 creep band) AND NOT accelerating away
#           (a_ego <= NOCHILL_LAUNCH_A — decelerating or steady both qualify: "stopping or stopped").
#   STAY    once armed, stays armed regardless of model_should_stop flicker, dwell timers, or
#           intermediate v_ego wobble — only the RELEASE condition below can clear it.
#   RELEASE v_ego > NOCHILL_RELEASE_V (0.8, hysteresis vs ARM_V so a twitch at the boundary can't
#           flap it) AND a_ego > NOCHILL_LAUNCH_A (genuinely accelerating, not just coasting past
#           the speed threshold on residual momentum or noise).
# A steady creep at 0.7-1.4 m/s with a_ego~=0 therefore stays latched indefinitely (correct per the
# directive — "stopping or stopped" is exactly the a_ego<=0 regime); a real launch clears it the
# moment BOTH the speed and the acceleration confirm it, typically well before v_ego leaves the old
# STANDSTILL_HOLD_V/STOP_CLEAR_HOLD_S band (field peak aEgo was 1.8-2.5 m/s^2, comfortably above the
# NOCHILL_LAUNCH_A floor). a_ego is read defensively (None/NaN/non-numeric -> 0.0, i.e. "not
# accelerating") — the FAIL-SAFE direction for this predicate is exactly the one that keeps holding,
# matching every other guard in this file (a bad/missing reading must never let the latch release
# early; it may only ever make it hold a little longer than the bare minimum).
#
# Correctness argument (unchanged, no-wedge proof): Experimental only ever HOLDS a stop (never
# launches on its own), so while latched the car cannot move away except via a model-commanded,
# held launch; a genuine (v_ego, a_ego) pair crossing both release thresholds is BY CONSTRUCTION
# that launch, at which point handing back to Chill (smoother acceleration) is exactly what is
# wanted and the existing ladder/dwell machinery resumes unmodified. The latch is a pure per-tick
# predicate on live (v_ego, a_ego) plus one bit of Schmitt-trigger memory (armed/not) — no timer to
# leak/freeze, so it cannot wedge Experimental forever.
#
# Telemetry (Gemini review, round 1): while armed, status is UNCONDITIONALLY "stopLatch" for the
# WHOLE armed episode (not just the ticks where the internal core would otherwise show chill) — the
# earlier design let the core's own dwell-driven status show through in between overrides, which
# could flap between a stale ladder reason and "stopLatch" every ~EXP_MIN_DWELL_S. A consistent tag
# for the entire hold is worth more to field forensics than surfacing the (now largely redundant,
# since the latch is doing the actual holding) internal ladder reason underneath it.
#
# NOT the same knobs as STANDSTILL_LATCH_V/STANDSTILL_RELEASE_V above (that machinery is about
# holding through a close-lead LAUNCH, wide 5.0 m/s release); this latch is about never leaving
# Experimental before a genuine launch at ALL, so its release margin is deliberately small.
NOCHILL_ARM_V     = 1.5   # m/s: below this AND not accelerating -> arm (covers the A2 creep band)
NOCHILL_RELEASE_V = 0.8   # m/s: release requires ALSO being above this (hysteresis vs ARM_V)
NOCHILL_LAUNCH_A  = 0.1   # m/s^2: a_ego threshold separating decel/steady (arm) from a genuine
                          #   accelerating launch (release) — far below the field's 1.8-2.5 m/s^2
                          #   jolt accel, comfortably above sensor/estimator noise at rest.

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


def read_ces_mode(params) -> int:
  """Read the CESMode INT param (source of truth). Back-compat: if CESMode is missing/0 but the old
  BOOL `ConditionalExperimentalSwitching` is set, treat that as Standard (2). Defensive: any failure
  => Off (0). Used by BOTH the CES and VTSC runtime readers so they always agree."""
  try:
    mode = int(params.get("CESMode", return_default=True) or 0)
  except Exception:
    mode = 0
  if mode == CES_MODE_OFF:
    try:
      if params.get_bool("ConditionalExperimentalSwitching"):
        mode = CES_MODE_STANDARD
    except Exception:
      pass
  return mode


# --- button override states (CESButtonState mem param) ----------------------
BTN_CES  = 0   # CES decides (default)
BTN_CHILL = 1  # forced Chill
BTN_EXP   = 2  # forced full Experimental

# --- event logging (CES_EVENT_LOG: persistent "each adoption" + breadcrumb trail) -------
TICK_S          = 1.0                    # s between heartbeat breadcrumb records (dense for the test drive)
HWY_SPEED_LIMIT = 55 * CV.MPH_TO_MS      # OSM speed limit >= this => coarse "highway" guess
HWY_VEGO        = 55 * CV.MPH_TO_MS       # or sustained speed >= this (authoritative = GPS+OSM+300ft in analysis)
