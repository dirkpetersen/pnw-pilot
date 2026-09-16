"""
madsresume2pnw — bounded auto-resume of the driver's OWN cruise set speed after a MADS brake.

WHAT THIS IS
------------
On the F-150 Lightning openpilot does not own longitudinal: speed is the truck's stock ACC.
mads2pnw/madsop2pnw taught the panda and openpilot to keep STEERING alive across a brake press
("Steering only", `madsState.lateralOnly`), but the driver still has to reach for the cruise stalk
to get speed back. This module decides — and it only ever DECIDES; the press itself is executed in
`opendbc/car/ford/icbm_pnw.py` — whether openpilot may tap RESUME once on the driver's behalf.

THIS IS SELF-ENGAGEMENT AND IT IS TREATED AS SUCH
-------------------------------------------------
Lateral authority (held by MADS) is being used to unlock a longitudinal re-engagement openpilot
did not otherwise have. The owner accepted that objection explicitly, on two conditions that are
the axioms of everything below:

  1. Resume ONLY to the speed the driver ALREADY SET. Never higher, never a new speed.
  2. The brake is always under the driver's foot, so they can always take it back.

  AXIOM 1 WAS AMENDED BY THE OWNER on 2026-09-06, and this note exists because the text above is
  otherwise flatly contradicted by the code (Fable review 2026-09-07, E1). They asked for a second
  path -- gasset2pnw -- in which the driver names the speed WITH THE ACCELERATOR and openpilot taps
  SET at whatever speed they reached: *"if I have been braking and I then accelerate with the gas
  and when I stop accelerating can't that be the speed that is then set ... that would be the most
  natural."* That IS "a new speed", so axiom 1 as written no longer holds for SET mode.

  What replaces it, and why the owner's version is the safer one: a SET establishes the speed the
  truck is ALREADY DOING, so it commands no acceleration at all, where a RESUME hands speed back to
  ACC and lets it climb. The acceleration-bounding gates therefore do not apply to SET mode -- but
  the speed floor, engageability, the lead distance floor and TTC all still do (`slowing` did too, until
  the owner removed it on 2026-09-13: "Ignore regen, set"). Axiom 2
  is untouched and still carries the whole envelope.

  Read the two modes as separate features that share a state machine, not as one feature with a
  loophole. `ResumeDecision.mode` is which of them fired.

Condition 1 deserves an honest note, because the mechanism does not let us be more precise than
this: the button we send is Ford's RESUME (`CcAsllButtnResPress` on 0x083). **The PCM chooses the
speed, not openpilot** — Ford ACC resume returns to the PCM's own last set speed. We therefore
cannot *command* a target at all. What we CAN do, and what this module does, is:
  * positively observe the driver's set speed BEFORE the brake and refuse to press at all if we
    never saw one (gate `noSet`);
  * refuse if the truck's currently-reported set speed is ABOVE the one we observed (`setRaised`);
  * refuse to press while stock cruise is ENGAGED, where RES is a SET+ (+1 mph) — enforced again,
    independently, in the executor (`decide_resume`); and
  * VERIFY after the fact: once cruise comes back, compare the truck's set speed against the one
    we captured and emit a LOUD record + `cloudlog.error` if it came back higher (`setHigher`).
That last one is the honest answer to "can the PCM resume to something we didn't expect?" — we
cannot prevent it, so we make it impossible to miss in the log. See docs/pnw/MADSRESUME2PNW.md.

NOTHING FAILS SILENTLY
----------------------
Every arm ends in exactly ONE terminal record ("fire" or "refuse"), and every refusal names the
gate that bound. A missing gate input is a REFUSAL with its own reason (`leadUnknown`, `noSet`) —
never a permissive default. A silent no-resume and a silent wrong-resume are therefore distinct
in ces_events.jsonl: no record at all means this module never even armed.

PURITY
------
No params, no sockets, no clock, no cereal. Everything arrives in `ResumeInputs`; `now` is passed
in (monotonic seconds). That is what makes the whole gate matrix unit-testable, and it is why the
call site (selfdrive/selfdrived/selfdrived.py) does the I/O.
"""

import math
from dataclasses import dataclass, field

# ---------------------------------------------------------------------------------------------
# Thresholds. Every one of these is justified in docs/pnw/MADSRESUME2PNW.md; the short form is in
# the comment beside it. They are deliberately module constants (not JSON-tunable): this is a
# self-engagement envelope, not a ride-feel tune.
# ---------------------------------------------------------------------------------------------

# Gate 3 — the bounded window, measured from the moment the brake is FULLY released.
# MIN: the truck's ACC state (CcStat_D_Actl 4/5 -> 3) and the driver's foot both need to settle;
#      firing on the same tick as the release would also fire on a brake *bounce*.
# MAX: 3 s. At highway speed that is ~90 m — the traffic situation that caused the brake is still
#      the same situation. Past it the driver has demonstrably chosen not to resume, and a resume
#      would be a surprise rather than a completion of what they were doing.
RELEASE_MIN_S = 0.5
RELEASE_MAX_S = 3.0
# engagegoal2pnw, OWNER DECISION 2026-09-13 ("Wait 1.0 s"): the GAS-SET path waits 1.0 s after lift-off
# (RESUME keeps RELEASE_MIN_S). Once regen stopped refusing a gas-set, the SET was offered ~0.6 s after
# lift-off -- ahead of a driver who lifts to slow down and brakes a moment later (weekend, Sat 12:41:50:
# brake at +0.69 s, to a stop). Inside this second a brake re-arms and nothing is set. The executor can
# only DELAY this: it presses only on a published offer (0-250 ms idle poll, carcontroller._resume_button),
# never before one. A lift shorter than this is still the same press (see `_gas_spent`).
GAS_SET_RELEASE_MIN_S = 1.0

# gassetwait2pnw, OWNER REPORT 2026-09-15 08:45 PT: "once I accelerated and lift the foot of the gas one pedal
# braking kicks in and the truck brakes for a second and then the cruise control takes over with the then lower
# speed". The driver's rule, given 2026-09-15 evening: while cruise is on and they have been accelerating on the
# pedal, LIFTING OFF MEANS "lock in this speed", not "slow down". Everywhere else a lift still means they want to
# slow, which is what the 1.0 s above protects.
#
# Both halves survive because the wait now depends on what the driver was doing AT LIFT-OFF:
#   a_ego > GAS_SET_ONPOWER_MS2   -> they were on the power  -> 0.5 s
#   otherwise (incl. unreadable)  -> they were already slowing -> 1.0 s, unchanged
#
# Why 0.5 s is enough, from the one clean current-config trace (drives/2026-09-15/gasset-regen-loss/, 09-14
# 22:01:41.141, route 00000173 seg 3, 10 Hz carState): after lift-off the truck COASTS for the first half second
# -- at +0.50 s it was still at 30.69 mph against a 30.62 mph lift-off, aEgo +0.07. One-pedal regen only bites
# between +0.5 and +1.3 s, and by the time the 1.0 s wait expired the truck was 1.01 mph down, so the PCM latched
# 30 against a 30.62 mph lift-off. Firing at 0.5 s puts the latch back on the speed the driver actually chose.
#
# Why the braking case is NOT reopened: the 1.0 s exists for one logged episode (weekend Sat 12:41:50, brake at
# +0.69 s after lift-off). In that episode the driver had ALREADY been slowing for 5 s before lifting off, so
# a_ego was negative and this branch keeps the full 1.0 s. The short wait is reachable only from a positive
# acceleration, which is the opposite situation.
#
# Why this threshold -- and READ THE METRIC, because the report's number is not the one this code uses.
# The report's `a0` (§2) is the MEAN aEgo over the 0.5 s BEFORE lift-off. On that metric the 5 real firings sit
# in [+0.53, +1.63], none of the 13 lift-offs below -0.2 engaged, and nothing sits between. THIS CODE LATCHES
# aEgo ON THE LIFT-OFF FRAME, which reads LOWER: the Kalman estimate is already decaying as the foot comes off.
# Recomputed on the same 317 segments (Fable review 2026-09-15), the frame metric gives: the 5 firings span
# [+0.12, +1.47], so one of them (09-13 21:16:32, +0.12) takes the LONG wait under +0.5; and two non-firing
# lift-offs sit at +0.49 / +0.51, so the "-0.2 to +0.5 is empty" property does NOT hold here.
# +0.5 is therefore NOT the floor of the firing population on the metric that ships -- it is ABOVE it, i.e.
# conservative in the only direction that matters: a real gas-set can only fall back to today's 1.0 s, never a
# slowing lift-off forward to 0.5 s. All 4 lift-offs followed by a brake inside 1.0 s had frame aEgo <= -0.16.
# n=5 cannot carry a threshold on its own; what carries it is that every error direction is today's behaviour.
# `a0` is in every record, so this can be re-derived on the frame metric once there are more than 5 firings.
GAS_SET_RELEASE_MIN_ACCEL_S = 0.5
GAS_SET_ONPOWER_MS2 = 0.5

# How long the offer stays on the wire once every gate has passed. The executor polls the
# mem-param at 4 Hz and its own freshness limit is 0.5 s, so 1.0 s is comfortably enough for
# exactly one poll+press while keeping the total exposure short. Re-published every tick and
# WITHDRAWN the instant any gate stops holding.
OFFER_S = 1.0

# Gate 6 — the captured set speed.
# The capture is refreshed on EVERY tick stock cruise reports enabled, and is REMEMBERED across the
# cruise-off gap that the brake itself creates. That memory is the whole point: the driver's set
# speed does not stop existing because their foot touched the brake pedal.
#
# It used to expire 0.75 s after the last engaged frame, sized to cover only MADS's brake-grace
# window. That was wrong, and the 2026-09-06 truck drive shows exactly how (drives/2026-09-06/):
# three brake episodes, and the capture was valid for only the FIRST one. Once a resume misses for
# any reason, stock cruise stays off, so nothing refreshes the capture -- and every later brake for
# the rest of the drive refused with `noSet` (observed setAgeS 22.15 s, then -1.0 = never captured).
# The feature could fail exactly once and was then dead until the driver manually re-engaged cruise,
# which is the thing the driver was asking not to have to do.
#
# Why lengthening this does NOT reopen the hazard it was sized against: the specific attack -- driver
# cancels cruise on the stalk, then brakes a beat later, and we resume to the speed they just
# deliberately cancelled -- is blocked ONE LAYER UP and always was. A stalk CANCEL is not in
# mads_pnw.MADS_TOLERATED_EVENTS, so `blocked` is True at the falling edge and MADS refuses to hand
# this module a lateral-only episode at all. The 0.75 s bound was belt-and-braces on top of a gate
# that already holds; removing the braces does not remove the belt.
#
# What remains is a genuine, bounded residual: a set speed captured on a fast road, remembered while
# the driver steers a long way on MADS lateral without cruise, and resumed somewhere it no longer
# suits. Three things bound it -- this 10-minute backstop, clearing the memory the moment the ACC
# master goes off (`cruise_available` False, i.e. the driver switched cruise off entirely), and the
# fact that a resume can never target ABOVE what the driver themselves last set.
SET_MAX_AGE_S = 600.0
# Sanity floor on a CAPTURED set speed -- it rejects a decode fault, it does not impose policy.
#
# This used to be 20 mph, justified as "Ford's ACC will not hold a set speed below 20 mph". That is
# FALSE for this truck, and it was silently refusing the driver's real city speeds. Two independent
# checks, 2026-09-06:
#   * opendbc/car/ford/interface.py:105 sets minEnableSpeed = 20 mph only on the MANUAL-transmission
#     branch. The Lightning is automatic, so that limit never applied to it.
#   * measured on the truck's own log: stock ACC was ENGAGED with set speeds of 15, 16, 17, 18 and
#     19 mph (34 ticks, minimum 6.71 m/s). The car plainly holds a set speed below 20 mph.
# 5.0 m/s (~11 mph) sits below every engaged set speed actually observed, with margin.
SET_MIN_MS = 5.0
# Sanity ceiling — a reading above this is a decode fault, not a driver intent.
SET_MAX_MS = 45.0                    # ~100 mph
# One Ford SET tap is 1 mph; tolerate under half of one so our own comparison can never trip on
# quantisation, while a real driver/PCM change of a full step still trips it.
SET_TOL_MS = 0.4 * 0.44704
# gasset2pnw: a SET establishes a new speed rather than restoring a remembered one. The PCM rounds
# to the nearest mph (0.22 m/s) and the truck keeps moving between our sample and the tap landing,
# so the verify tolerance has to cover both. This is a REPORTING tolerance only -- it decides
# whether the log calls the outcome "ok", and gates nothing.
SET_MODE_TOL_MS = 1.0
# nosetcancel2pnw, OWNER DECISION 2026-09-14 ("remove it"): engagegoal2pnw's overshoot-CANCEL rule (03c9c30ba9,
# SET_HIGH_CANCEL_MS = 3 mph: cancel cruise, steering too, when our own press brought the set back high) is GONE.
# Its only trigger, Corvallis 2026-09-13 21:16:33, was a 55 km/h set read as mph -- the truck set the speed it was
# doing (drives/2026-09-14/units-kmh/). The verify below still LOGS a come-back above what we wanted ("setHigher",
# loud, cloudlog.error in selfdrived); nothing acts on it. The record keeps `cancel` (always False) and says so in
# `overshootAction`.
# engagegoal2pnw (Fable review 2026-09-13, B1): no RES/SET offer while the DRIVER pressed a cruise button in the last
# second -- they are already engaging it themselves. Without this, a driver RES ~0.15 s before our SET- fired was
# invisible to the verify window (it opens at the fire), and the PCM engaging at the driver's memory read as our
# overshoot (then a cancel with steering dropped; since nosetcancel2pnw a false "setHigher" record). Refusing the press
# removes the race at its root (no press of ours, nothing to verify) and keeps our SET- from landing on top of the
# driver's own engagement. 1.0 s covers the measured 0.15-0.26 s PCM response with margin; past it, a button that
# engaged nothing no longer blocks our press.
DRIVER_BTN_HOLDOFF_S = 1.0
# The two wire values `ResumeDecision.mode` may take, and the exact set the executor's parser
# accepts (opendbc icbm_pnw.RESUME_DIR / SET_DIR). Pinned by the wire-contract test: the brain and
# the executor live in different repos with only a JSON mem-param between them, and a drift here
# fails SILENTLY -- the executor parses None and simply never presses, which looks identical to
# "the gates refused".
RESUME_MODES = ("res", "set")

# Gate 5 — lead / TTC. Resuming hands speed back to stock ACC, which will then ACCELERATE toward
# the set speed. So the bar is not "is this safe right now" but "is the road already at least as
# open as the gap ACC itself would hold".
#   HEADWAY 2.0 s  — at or better than stock ACC's own following distance, so re-engaging cannot
#                    ask ACC to close a gap it would not otherwise close.
#   TTC     8.0 s  — at 30 m/s behind a 60 m lead, 8 s TTC means closing at under 7.5 m/s: we are
#                    not meaningfully overtaking. Anything faster-closing is exactly the situation
#                    the driver braked for.
#   DIST   20.0 m  — an absolute floor, because headway alone is far too permissive at low speed
#                    (2 s at 5 m/s is 10 m).
LEAD_MIN_HEADWAY_S = 2.0
LEAD_MIN_TTC_S = 8.0
LEAD_MIN_DIST_M = 20.0

# Speed floor for actually resuming. Kept equal to the capture floor rather than DERIVED from it,
# so that changing one does not silently move the other. Below ~11 mph a brake-and-release is
# creeping in stop-and-go, where handing speed back to ACC is a surprise; `standstill` is gated
# separately. Note this is a POLICY floor -- the lead gate (20 m absolute) is what actually protects
# the low-speed case, and it is unchanged.
V_EGO_MIN_MS = 5.0

# brakeretry2pnw: the driver's OPT-OUT. Two brake presses inside this window mean "no, leave
# longitudinal off" -- auto-resume is then suppressed until the driver themselves brings cruise back
# (a manual RES, or any SET+/SET- adjustment; both engage stock cruise, which is the clear signal).
#
# This exists BECAUSE arming now happens on every brake press. Without it the driver has no way to
# say "stay off": each press would open another resume opportunity, and a driver who genuinely wants
# cruise gone would be arguing with the feature. One deliberate double-tap is a clearer, faster
# statement of intent than any toggle, and it is available in the moment, with the foot already
# there. A single press keeps its plain meaning ("slow down, then carry on"), which is the common
# case, so the opt-out costs the common case nothing.
DOUBLE_BRAKE_S = 1.0
# Pedal BOUNCE filter. Two edges closer together than this are one press that chattered, not two
# presses -- counting them as a double-tap would silently kill the feature on a rough road, and
# counting them as two arms would re-arm on chatter (Gemini review 2026-09-06, finding C).
BRAKE_DEBOUNCE_S = 0.15
# A brake press this soon after WE resumed is the driver REJECTING that resume, not asking for
# another one. Without it, braking to cancel an unwanted resume immediately queues the next one:
# brake, release, surge again -- a fight the driver cannot win using the one reflex they will
# actually reach for (Gemini review 2026-09-06, finding B). It latches the same opt-out as the
# double-tap, so a single firm brake is enough to say "stop".
#
# MEASURED 2026-09-06, and retuned from 5.0 s because of it. On the first drive with this shipped,
# the driver resumed cleanly at 17:55:40 and braked again at 17:55:44 -- 4.6 s later, ordinary city
# traffic, not a rejection -- and that latched the opt-out and left the feature dead for the next
# 30 s ("refuse: suppressed" at 17:56:03). A genuine rejection is a reflex against an acceleration
# the driver can feel: it lands in a second or two, not five. 2.0 s still catches it and stops
# treating normal following traffic as an opt-out.
REJECT_AFTER_FIRE_S = 2.0

# THE CROSS-CONTEXT GUARD (Gemini review 2026-09-06, finding A -- BLOCK).
# Remembering the set speed across the cruise-off gap is what makes this feature work at all, but a
# memory with only a time bound is a loaded gun: set 70 mph on the freeway, exit, drive several
# minutes on MADS lateral through town, brake and release at 15 mph -- and a purely time-bounded
# memory would hand the truck back a 70 mph target on a residential street. The lead gate is no
# protection there, because an empty street has no lead.
#
# Time alone cannot separate that from the case the driver actually wants, so this does not try.
# The discriminator is whether the captured set speed is a speed THIS DRIVE HAS RECENTLY BEEN DOING.
# `_v_max` is an approximate rolling maximum of v_ego over the last V_MAX_WINDOW_S; a resume is
# refused when the captured set speed stands more than V_MAX_MARGIN_MS above it.
#   * brake hard 70 -> 40 for traffic, release:   recent max 70, set 70   -> PASS (the main case)
#   * following a slow lead at 20 with set 31:    recent max ~31          -> PASS
#   * set above what traffic ever allowed:        margin covers it        -> PASS
#   * freeway memory used in a 25 mph town:       recent max ~7, set 31   -> REFUSE
# It also bounds finding D: it caps how much ACCELERATION any resume can command, since the target
# can never stand far above a speed the truck has just been holding.
# gasset2pnw / Fable review 2026-09-07, finding C. `regenBraking` is NEVER SET on Ford
# (0 occurrences in opendbc/car/ford/), so `ResumeInputs.regen_braking` is permanently False on this
# truck and the `regen` telemetry field will read False forever -- do not read it as evidence.
#
# That matters most for gas-set. On a Lightning with 1-Pedal Drive, LIFTING OFF *IS* THE BRAKE, and
# it is invisible to us: a driver lifting off to slow down looks identical to one lifting off to
# cruise. Setting ACC then removes exactly the deceleration they asked for. Speed is the one honest
# witness we have, so refuse a SET while the truck is still slowing meaningfully.
#
# MEASURED on this truck 2026-09-07, route 000000f9 -- 7 gas-release events above 5 m/s, decel over
# the 0.4 s after lift-off:
#     median -0.06    p90 1.33    max 1.33    min -0.67
# The owner's call was right and the theory was wrong: this Lightning COASTS on lift-off, it does
# not hard-regen, so the 1-Pedal hazard this gate was written for is not how the truck behaves. The
# median is essentially zero. Exactly one event showed real deceleration (1.33), and that is the
# one worth refusing.
#
# 1.0 m/s^2 therefore: ~16x the coasting median, and 25% below the only genuine slowing event
# observed. 0.5 would also have caught it, but sits closer to ordinary coasting than the evidence
# justifies. n=7 is a SMALL SAMPLE from one drive -- the `decel`/`decelAgeS` telemetry fields exist
# so this can be re-derived rather than re-argued.
#
# SUPERSEDED BY THE OWNER, 2026-09-13 ("Ignore regen, set"): the 2026-09-11..13 weekend measured
# 1.5-1.9 m/s^2 of regen within ~1 s of every gas lift-off in steering-only, so this gate refused half
# of them, and a SET- to the current speed commands no acceleration. The `slowing` refusal is gone from
# the gas-set path; there is no longer a decel threshold. `decel`/`decelAgeS` stay in every record.
# 0.4 s. NOTE, because the obvious reading is wrong and would invite deleting `decelUnknown` as
# redundant: this being shorter than RELEASE_MIN_S (0.5) does NOT guarantee a fresh window by the
# earliest fire. The estimator resamples on its own cadence, unaligned to the driver, so the first
# window that starts at or after lift-off closes anywhere in [t_release + 0.4, t_release + 0.8).
# What makes the gate safe is `decelUnknown` refusing until a window taken ENTIRELY after lift-off
# exists -- not this constant (Fable review 2026-09-07 round 3, E4).
DECEL_WINDOW_S = 0.4

V_MAX_WINDOW_S = 60.0
V_MAX_MARGIN_MS = 5.0        # ~11 mph of slack for a set speed traffic never let the truck reach
# The ABSOLUTE cap, and the primary bound on uncommanded acceleration (Fable review 2026-09-06, A1).
# `staleContext` alone is time-scoped, so it still permits the freeway-exit-then-yield case: brake
# 70 -> 25 mph down a ramp, release at the yield onto an arterial, and RES targets 70 from 15 mph
# with cross traffic and no lead to gate on. Time cannot separate that from "braked 70 -> 40 for
# traffic"; the SIZE OF THE JUMP can. 15 m/s (~34 mph) clears a hard brake from the set speed and
# refuses a resume that would command more acceleration than any brake-and-continue needs.
RESUME_MAX_DELTA_MS = 15.0

# gasset2pnw (driver request 2026-09-06): "if I have been braking and I then accelerate with the
# gas, when I stop accelerating can't that be the speed that is then set". It is the most natural
# form of the feature -- the driver picks the speed with the pedal they are already using, and the
# truck simply keeps it -- and it is also the SAFEST form, because setting to the current speed
# commands no acceleration whatsoever. Where a RESUME hands speed back to ACC and lets it climb, a
# SET here just holds what the driver has already chosen.
#
# It also removes two whole classes of refusal seen on the 2026-09-06 drive: `gas` (the driver
# using the accelerator used to ABORT the episode -- 17:10:39 and 17:57:11) and `noSet` (no
# remembered set speed -- 17:11:00 and 17:13:42, and unreachable for the rest of a drive once it
# happened). Neither matters here: the accelerator IS the input, and no memory is consulted.
# Total lifetime of one arm. Lateral-only can persist indefinitely (that is the point of MADS);
# this bounds how long a resume can still be pending behind it so a resume can never arrive
# minutes after the brake that armed it.
ARM_MAX_S = 20.0

# How long after the press we keep watching for cruise to come back, to VERIFY what speed it came
# back at. Ford ACC re-engages well inside this.
VERIFY_S = 10.0


@dataclass
class ResumeInputs:
  """One tick of everything the brain is allowed to see. Plain python only."""
  now: float                       # monotonic seconds
  mads_available: bool             # madsState.available -- False => this module is fully inert
  lateral_only: bool               # madsState.lateralOnly
  op_enabled: bool                 # selfdriveState.enabled (openpilot's own engagement)
  blocked: bool                    # any blocking/disabling event this frame
  # selfdriveState.engageable -- would openpilot's OWN state machine accept an engage right now?
  # See gate `noEntry` in _gates(): this is NOT the same question as `blocked`.
  engageable: bool
  brake_pressed: bool
  regen_braking: bool
  gas_pressed: bool
  cruise_enabled: bool             # stock ACC actively engaged
  cruise_available: bool           # stock ACC main on (standby counts)
  set_speed_ms: float              # carState.cruiseState.speed as reported RIGHT NOW
  v_ego: float
  standstill: bool
  # engagegoal2pnw: the DRIVER pressed a cruise button this tick (carState.buttonEvents: RES, SET+, SET-,
  # SET, ON/OFF). These come from the SCCM on bus 0 only; openpilot's own taps never appear there.
  driver_cruise_button: bool
  # radarState.leadOne. `has_lead is None` means the read FAILED -- that is a refusal, not "no
  # lead". d_rel/v_lead are only meaningful when has_lead is True.
  has_lead: bool | None
  d_rel: float | None = None
  v_lead: float | None = None
  # units2pnw: the cluster's set-speed unit, "mph" | "kph" | "unknown" (speed_unit_name() of
  # carState.cruiseState.speedClusterUnit). TELEMETRY ONLY here: set_speed_ms is already true m/s, because Ford CAN FD
  # carstate converts Veh_V_DsplyCcSet by this unit (pnw-opendbc units2pnw). "unknown" means carstate ASSUMED mph.
  set_speed_unit: str = "unknown"
  # gassetwait2pnw: carState.aEgo. Read ONCE, on the tick both pedals come up, to decide whether this
  # lift-off is "on the power" (0.5 s wait) or "already slowing" (1.0 s, today's behaviour). NaN -- the
  # default, and what a caller that does not supply it gets -- takes the LONG wait. See gas_set_wait_s().
  a_ego: float = float("nan")
  # onetoggle2pnw: the separate MadsAutoResume toggle is GONE -- "Disengage on brake" governs both
  # halves of the behaviour. This is not a loosening: the arm gate below requires the rising edge of
  # `lateral_only`, and mads_pnw sets
  #     lateral_only = (not disengage_on_brake) and braking and not blocked
  # so lateral_only can ONLY be true when DisengageOnBrake is OFF. The toggle was therefore already
  # implied by gate 1, and a second control that can never independently be false is a control the
  # driver can be misled by. Pinned by test_resume_impossible_when_disengage_on_brake_is_on.


@dataclass
class ResumeDecision:
  """What the caller should do this tick."""
  offer: bool = False              # publish the resume command
  eid: float = 0.0                 # episode id -- constant for one offer, the executor's one-shot key
  set_ms: float = 0.0              # the target speed (carried for the executor's own check)
  # gasset2pnw: which button this offer is for.
  #   "res" -- tap RESUME, handing back the speed the driver had ALREADY set (accelerates).
  #   "set" -- tap SET at the CURRENT speed, after the driver chose it with the accelerator.
  # They are different actions with different risk: "set" commands no speed change at all, so the
  # gates that exist to bound acceleration do not apply to it.
  mode: str = "res"
  records: list = field(default_factory=list)   # telemetry records to append (usually empty)


def speed_unit_name(unit) -> str:
  """truckdecode2pnw: carState.cruiseState.speedClusterUnit -> "mph" | "kph" | "unknown".

  Compares the capnp ENUM, never its str() (a builder enum stringifies as a bare int, a reader as the bare name: the
  capnp enum str trap). None -- a carState whose schema has no such field -- and a cereal without SpeedUnit are
  "unknown", so a pin mismatch degrades to today's mph assumption instead of raising in selfdrived."""
  from cereal import car
  SpeedUnit = getattr(car.CarState.CruiseState, "SpeedUnit", None)
  if unit is None or SpeedUnit is None:
    return "unknown"
  if unit == SpeedUnit.mph:
    return "mph"
  if unit == SpeedUnit.kph:
    return "kph"
  return "unknown"


def _finite(x) -> bool:
  try:
    return math.isfinite(float(x))
  except (TypeError, ValueError):
    return False


def gas_set_wait_s(a_ego) -> tuple[float, str]:
  """PURE. gassetwait2pnw: how long the GAS-SET path waits after lift-off, and why.

  `a_ego` is the acceleration sampled on the tick BOTH pedals came up -- the driver's last act before
  lifting, not a live value. Returns (seconds, why) where `why` is one of "onPower" / "slowing" /
  "accelUnknown"; `why` is recorded on every madsResume record so the branch taken is never a guess.

  An UNREADABLE acceleration is "slowing" -- the LONG wait. That is the fail-safe direction: it keeps
  today's behaviour and keeps the brake protection the 1.0 s was earned by. A missing input must never
  buy the shorter, more permissive wait (CLAUDE.md rule 2: an error is not a negative result)."""
  if not _finite(a_ego):
    return GAS_SET_RELEASE_MIN_S, "accelUnknown"
  if float(a_ego) > GAS_SET_ONPOWER_MS2:
    return GAS_SET_RELEASE_MIN_ACCEL_S, "onPower"
  return GAS_SET_RELEASE_MIN_S, "slowing"


def lead_gate(has_lead, d_rel, v_lead, v_ego, require_headway: bool = True) -> str | None:
  """PURE. Returns None if the road ahead is open enough to hand speed back, else the name of the
  binding sub-gate. A FAILED radar read (`has_lead is None`) is `leadUnknown` -- a refusal, never
  a permissive default (CLAUDE.md rule 2: an error is not a negative result)."""
  if has_lead is None:
    return "leadUnknown"
  if not has_lead:
    return None
  if not (_finite(d_rel) and _finite(v_lead) and _finite(v_ego)):
    return "leadUnknown"
  d = float(d_rel)
  if d < LEAD_MIN_DIST_M:
    return "leadClose"
  # headway needs a speed to divide by; below the speed floor we have already refused on `slow`,
  # but be defensive rather than divide by ~0.
  v = float(v_ego)
  if v <= 0.1:
    return "leadClose"
  # The HEADWAY check asks "would ACC have to close a gap it would not otherwise close" -- a
  # question only a RESUME raises, because only a resume accelerates toward a remembered speed.
  # A SET at the current speed cannot close anything, so gasset2pnw skips this one sub-gate. The
  # distance floor and TTC below are NOT skipped: they ask "is something about to hit us", which
  # applies no matter what speed is being handed over (Gemini review 2026-09-07, finding C).
  if require_headway and d / v < LEAD_MIN_HEADWAY_S:
    return "leadGap"
  v_close = v - float(v_lead)
  if v_close > 0.0 and (d / v_close) < LEAD_MIN_TTC_S:
    return "leadTtc"
  return None


class MadsResumeBrain:
  """The bounded auto-resume state machine. One instance per drive; `update()` every control tick.

  Lifecycle of ONE brake event:
      (continuously) capture the driver's set speed while stock cruise is enabled
      lateralOnly rising edge, OR any
        later brake press while it holds -> ARM   (record "arm")
      brake+regen both released        -> the release clock starts
      RELEASE_MIN_S..RELEASE_MAX_S     -> if every gate passes: OFFER (record "fire"), latch _done
                                          (a gas-set waits GAS_SET_RELEASE_MIN_S instead of RELEASE_MIN_S)
      offer ends                       -> record "offerEnd"
      window passes without an offer   -> record "refuse" naming the binding gate
      cruise comes back within VERIFY_S-> record "verify" (LOUD if it came back above the capture)
      lateralOnly falls                -> DISARM

  `_done` is the once-per-EPISODE latch: one brake press gets one press attempt, and it is cleared
  by a disarm. Since brakeretry2pnw a disarm no longer requires lateral_only to go False -- the next
  brake press starts a fresh episode. So a failed attempt does not get a retry *within* that press,
  but the driver always gets another attempt simply by braking again, which is the driver's own
  stated rule ("I can always push the brake"). The set-speed capture is REMEMBERED across all of
  this; see SET_MAX_AGE_S for why it must be, and what bounds it."""

  def __init__(self):
    # continuous set-speed capture
    self._set_ms: float | None = None
    self._set_t: float | None = None
    # arm state
    self._armed = False
    self._arm_t = 0.0
    self._armed_set: float | None = None
    self._armed_set_age = 0.0
    self._released_t: float | None = None
    self._used_gas = False          # gasset2pnw: this episode's target comes from the accelerator
    self._done = False              # once-per-event latch
    self._terminal = False          # a terminal record has already been written for this arm
    self._last_block = "init"       # the most recent binding gate, for the terminal record
    # offer state
    self._offer_t: float | None = None
    self._eid = 0.0
    # post-press verification
    self._verify_until: float | None = None
    self._verify_set: float | None = None
    # the mode of the press this verify belongs to, snapshotted at fire. `_fired_mode` is cleared by
    # a gas-reopen, and `_used_gas` keeps changing, so neither can be trusted 10 s later when the
    # verify lands (Fable review 2026-09-07 round 3, C1 -- reproduced: a RESUME press reported
    # "set", and the selfdrived warning then announced the wrong button).
    self._verify_mode: str | None = None
    # engagegoal2pnw: a driver cruise button was seen between our press and its verify (telemetry `driverBtn`: a
    # higher come-back may be the driver's own).
    self._verify_driver_btn = False
    # units2pnw: the set-speed unit on the last verify record (None = no verify yet), so an assumed unit is flagged
    # for selfdrived's warning once per change, not once per press.
    self._verify_unit: str | None = None
    # engagegoal2pnw B1: when the driver last pressed a cruise button (None = not seen).
    self._driver_btn_t: float | None = None
    # Edge detector. THREE-STATE: None = "never observed", which is NOT the same fact as
    # "observed False" (Gemini review 2026-09-06). With a plain False, the first tick after the
    # brain becomes active -- e.g. a selfdrived restart mid-drive
    # on while already steering-only -- reads as a rising edge and ARMS without any brake
    # transition having been observed at all: precisely outside the bounded state. The first
    # observation now only SEEDS the detector; arming needs a genuine False->True after that.
    self._lat_prev = None
    # brakeretry2pnw double-tap opt-out: time of the last brake rising edge, and the latch it sets.
    self._last_brake_t: float | None = None
    self._suppressed = False
    # Pedal-only edge detector (regen deliberately excluded -- see the edge block in update()).
    # THREE-STATE like _lat_prev: None = never observed.
    self._pedal_prev = None
    self._pedal_off_t: float | None = None
    # engagegoal2pnw: accelerator edge detector, THREE-STATE like _pedal_prev (None = never observed).
    self._gas_prev = None
    # engagegoal2pnw: this steering-only state was seen to START with the brake down (an arm that passed
    # the noBrake check). Cleared the moment lateral-only ends. The accelerator may only open an episode
    # of its own inside a state this brain watched begin with a brake -- the one precondition the whole
    # envelope rests on (see the noBrake check in update()).
    self._lat_braked = False
    # engagegoal2pnw, OWNER DECISION 2026-09-13 ("first press only"): after a brake with steering still on
    # there is no time limit, but ONLY THE FIRST accelerator press may set the speed at lift-off. True once
    # that press has been used: its lift-off was judged by the SET gates (fired, or refused by a gate), i.e.
    # both pedals stayed up for GAS_SET_RELEASE_MIN_S. A shorter lift is pedal modulation inside the same press.
    # Cleared ONLY by a new brake arm -- deliberately NOT by an episode ending (armExpired, a refusal), or
    # an overtake minutes later would get a second attempt. Every record carries it (`gasSpent`).
    self._gas_spent = False
    # engagegoal2pnw, OWNER DECISION 2026-09-13 ("Creeping doesn't count"): the highest speed reached during the
    # current accelerator press. A press that never reaches V_EGO_MIN_MS before its lift-off is judged is a creep
    # (stop-and-go) and does not use up the first press. Reset by a brake arm (a press that reached it is spent).
    self._gas_v_max = 0.0
    # gassetwait2pnw: the acceleration latched on the lift-off tick, and the wait it bought. Re-derived on
    # EVERY lift-off (`_released_t` is cleared by any pedal, so the pair can never outlive its release) and
    # reset to the LONG wait whenever nothing is in flight, so a stale short wait is unreachable.
    self._gas_a0 = float("nan")
    self._gas_wait_s = GAS_SET_RELEASE_MIN_S
    self._gas_wait_why = "slowing"
    # when our own resume last fired, for the post-resume rejection check
    self._fired_t: float | None = None
    # cruise_enabled edge detector, for clearing the opt-out. THREE-STATE like _lat_prev.
    self._cc_prev = None
    # approximate rolling max of v_ego, for the cross-context guard
    self._v_max: float | None = None
    self._v_max_t = 0.0
    # deceleration estimate, for the gas-set regen blindness above
    self._v_ref: float | None = None
    self._v_ref_t = 0.0
    self._decel = 0.0                # m/s^2, positive = slowing
    # when the window that produced `_decel` STARTED. The gate requires this to be at or after the
    # moment both pedals came up, so a measurement taken while the driver was still on the power
    # can never be mistaken for a post-lift-off one.
    self._decel_from = 0.0
    # the mode this arm actually FIRED in, sampled at fire. `_used_gas` keeps changing afterwards,
    # so a record that reads it live can report the wrong button for a press already sent.
    self._fired_mode: str | None = None
    # diagnostics: how many times update() raised inside the caller's guard (caller-owned counter
    # lives in selfdrived; this one just proves the brain itself ran).
    self.ticks = 0

  # -- helpers ---------------------------------------------------------------------------------

  def _snap(self, i: ResumeInputs, extra: dict | None = None) -> dict:
    """The common telemetry body. Everything a post-hoc reader needs to re-derive the decision."""
    ttc = None
    hdwy = None
    try:
      if i.has_lead and _finite(i.d_rel) and _finite(i.v_lead) and float(i.v_ego) > 0.1:
        hdwy = round(float(i.d_rel) / float(i.v_ego), 2)
        vc = float(i.v_ego) - float(i.v_lead)
        ttc = round(float(i.d_rel) / vc, 1) if vc > 0.0 else None
    except (TypeError, ValueError, ZeroDivisionError):
      ttc, hdwy = None, None
    mode = self._fired_mode if self._fired_mode is not None else ("set" if self._used_gas else "res")
    rec = {
      "vEgo": round(float(i.v_ego), 2) if _finite(i.v_ego) else None,
      "setMs": round(self._armed_set, 2) if self._armed_set is not None else None,
      "setAgeS": round(self._armed_set_age, 2),
      "stockSet": round(float(i.set_speed_ms), 2) if _finite(i.set_speed_ms) else None,
      "lead": i.has_lead,
      "dRel": round(float(i.d_rel), 1) if (i.has_lead and _finite(i.d_rel)) else None,
      "vLead": round(float(i.v_lead), 1) if (i.has_lead and _finite(i.v_lead)) else None,
      "ttc": ttc, "hdwy": hdwy,
      "relS": round(i.now - self._released_t, 2) if self._released_t is not None else None,
      "armS": round(i.now - self._arm_t, 2) if self._armed else None,
      "brk": bool(i.brake_pressed), "regen": bool(i.regen_braking), "gas": bool(i.gas_pressed),
      "ccOn": bool(i.cruise_enabled), "ccAvail": bool(i.cruise_available),
      "blocked": bool(i.blocked), "latOnly": bool(i.lateral_only),
      "opEn": bool(i.op_enabled), "engbl": bool(i.engageable), "eid": self._eid,
      # without these a staleContext/setFar refusal cannot be re-derived from the record
      "vMax": round(self._v_max, 2) if self._v_max is not None else None,
      "vMaxAgeS": round(i.now - self._v_max_t, 1) if self._v_max is not None else None,
      "supp": bool(self._suppressed),
      "mode": mode,
      "decel": round(self._decel, 2),
      "decelAgeS": round(i.now - self._decel_from, 2) if self._decel_from else None,
      "sinceFireS": round(i.now - self._fired_t, 2) if self._fired_t is not None else None,
      "gasSpent": bool(self._gas_spent),
    }
    # gassetwait2pnw: which wait this lift-off got and what bought it. `a0` is the latched lift-off
    # acceleration (null = it could not be read, which is itself the reason for `waitWhy`). Without these
    # three a short-wait fire and a long-wait fire are indistinguishable in the log.
    #
    # GAS-SET RECORDS ONLY (Fable review 2026-09-15, D2). A resume waits RELEASE_MIN_S and is not governed
    # by any of this, so stamping it with a `waitS` it did not use would be a false record -- and the
    # `accelUnknown` line selfdrived logs off `waitWhy` would then claim "kept the 1.0 s wait" about a
    # resume that waited 0.5 s. An absent field cannot lie; a wrong one can.
    if mode == "set":
      rec.update({
        "a0": round(self._gas_a0, 2) if _finite(self._gas_a0) else None,
        "waitS": round(self._gas_wait_s, 2),
        "waitWhy": self._gas_wait_why,
      })
    if extra:
      rec.update(extra)
    return rec

  def _terminate(self, i: ResumeInputs, out: ResumeDecision, reason: str) -> None:
    """Write the ONE terminal record for this arm, if it hasn't been written yet."""
    if self._terminal:
      return
    self._terminal = True
    out.records.append(self._snap(i, {"phase": "refuse", "reason": reason, "fired": False}))

  def _disarm(self) -> None:
    self._armed = False
    self._used_gas = False
    # gassetwait2pnw: back to the long wait. `_released_t` is cleared below too, so the next lift-off
    # re-latches both before either is read -- this is belt-and-braces against a future path that
    # reads the wait without a release in flight.
    self._gas_a0 = float("nan")
    self._gas_wait_s = GAS_SET_RELEASE_MIN_S
    self._gas_wait_why = "slowing"
    self._fired_mode = None
    self._armed_set = None
    self._armed_set_age = 0.0
    self._released_t = None
    self._done = False
    self._terminal = False
    self._offer_t = None
    self._last_block = "idle"

  # -- the tick --------------------------------------------------------------------------------

  def update(self, i: ResumeInputs) -> ResumeDecision:
    self.ticks += 1
    out = ResumeDecision()

    # --- gate 8: inert unless MADS is actually available (never acts on the Tesla) --------------
    # and gate 0: the driver-facing kill switch, default OFF.
    if not i.mads_available:
      # Hold every latch cleared so enabling mid-drive can never see a stale edge/arm. The edge
      # detector is SEEDED from this tick's observation rather than forced False: forcing False
      # while lateral_only is already True is exactly what would manufacture a rising edge on the
      # tick the driver flips the toggle back on (Gemini review 2026-09-06).
      self._lat_prev = bool(i.lateral_only)
      self._last_brake_t = None
      self._suppressed = False
      self._pedal_prev = bool(i.brake_pressed)
      self._pedal_off_t = None
      self._gas_prev = bool(i.gas_pressed)
      self._lat_braked = False
      self._gas_spent = False
      self._gas_v_max = 0.0
      self._fired_t = None
      self._v_max = None
      self._v_max_t = 0.0
      self._v_ref = None
      self._v_ref_t = 0.0
      self._decel = 0.0
      self._decel_from = 0.0
      self._fired_mode = None
      self._driver_btn_t = None
      self._cc_prev = bool(i.cruise_enabled)
      self._set_ms = None
      self._set_t = None
      self._verify_until = None
      if self._armed:
        self._disarm()
      return out

    # --- continuous capture of the driver's OWN set speed (gate 6's only source) ----------------
    # Refreshed on every tick stock cruise reports engaged. This is the ONLY place _set_ms is
    # written, so it can never pick up a value from a frame where cruise was off.
    # --- rolling max of v_ego, for the cross-context guard ---------------------------------------
    # Approximate by design: hold the max, and let it expire to the current speed once it is older
    # than the window. That is one comparison per tick and needs no buffer. A NON-FINITE v_ego does
    # not update it (and the gate below refuses outright on one), so a bad read can never inflate
    # the ceiling and thereby permit a resume it should have refused.
    if _finite(i.v_ego):
      v = float(i.v_ego)
      if self._v_max is None or v >= self._v_max or (i.now - self._v_max_t) > V_MAX_WINDOW_S:
        self._v_max = v
        self._v_max_t = i.now
      # deceleration over a fixed window. Cheap, and it needs no history buffer.
      if self._v_ref is None:
        self._v_ref, self._v_ref_t = v, i.now
      elif (i.now - self._v_ref_t) >= DECEL_WINDOW_S:
        dt = i.now - self._v_ref_t
        self._decel = (self._v_ref - v) / dt if dt > 0.0 else 0.0
        self._decel_from = self._v_ref_t
        self._v_ref, self._v_ref_t = v, i.now

    if i.driver_cruise_button:
      self._driver_btn_t = i.now
    cc_rising = bool(i.cruise_enabled) and self._cc_prev is False
    self._cc_prev = bool(i.cruise_enabled)
    if cc_rising:
      # The driver has cruise engaged again -- by a manual RES, by a SET+/SET- adjustment, or
      # because our own press landed. Whichever it was, the opt-out is spent.
      #
      # RISING EDGE, not level (Fable review 2026-09-06, P2). On the level, a single frame where
      # cruiseState.enabled still reads True after the driver's rejection brake wipes `_suppressed`
      # immediately -- and the truck resumes again, which is precisely the unwinnable fight the
      # opt-out exists to end. mads_pnw.py:235-237 states the PCM ordering is not guaranteed, so
      # that frame is not hypothetical.
      self._suppressed = False
      self._last_brake_t = None
    if i.cruise_enabled and _finite(i.set_speed_ms):
      s = float(i.set_speed_ms)
      if SET_MIN_MS <= s <= SET_MAX_MS:
        self._set_ms = s
        self._set_t = i.now
    elif not i.cruise_available:
      # The ACC master switch is OFF -- the driver has turned cruise off entirely, not merely had it
      # dropped by the brake. Whatever they had set is no longer "the speed they set"; forget it now
      # rather than letting the 10-minute backstop carry it across an explicit switch-off.
      self._set_ms = None
      self._set_t = None

    # --- post-press verification (runs independently of arm/disarm) ----------------------------
    if self._verify_until is not None:
      self._verify_driver_btn = self._verify_driver_btn or bool(i.driver_cruise_button)
      if i.now > self._verify_until:
        # Cruise never came back inside the window. That is not an error (the press may have been
        # correctly ignored), but it IS the difference between "we pressed and nothing happened"
        # and "we pressed and it worked", so it is logged.
        out.records.append(self._snap(i, {"phase": "verify", "reason": "noCruise", "fired": True,
                                          "mode": self._verify_mode or "res"}))
        self._verify_until = None
      elif i.cruise_enabled and _finite(i.set_speed_ms) and float(i.set_speed_ms) > 0.0:
        # units2pnw: everything here is TRUE m/s. set_speed_ms (and a RESUME's want, captured from it) comes from Ford
        # carstate, which converts Veh_V_DsplyCcSet by the cluster unit; a gas-set's want is v_ego. Before that, a km/h
        # cluster read 1.609x high and every gas-set read setHigher (Corvallis 2026-09-13 21:16: 55 km/h read as 55 mph).
        unit = i.set_speed_unit if i.set_speed_unit in ("mph", "kph") else "unknown"
        got = float(i.set_speed_ms)
        want = self._verify_set if self._verify_set is not None else 0.0
        # Fable B1: the axiom is "the speed the driver ALREADY SET", which is violated by a
        # DIFFERENT speed in either direction -- a come-back well BELOW the capture means the PCM
        # did not restore its remembered set, so something set a new speed. That is not dangerous
        # (it is slower), but reporting it as "ok" would hide the fact that the mechanism did not
        # behave as this whole design assumes, which is exactly the thing the log exists to catch.
        # A SET establishes a NEW speed and the PCM rounds it to the nearest mph, so it cannot be
        # held to the tolerance a RESUME is (which must land exactly on a remembered value).
        tol = SET_MODE_TOL_MS if self._verify_mode == "set" else SET_TOL_MS
        if got > want + tol:
          reason = "setHigher"
        elif got < want - tol:
          reason = "setLower"
        else:
          reason = "ok"
        out.records.append(self._snap(i, {
          "phase": "verify", "reason": reason, "fired": True,
          "gotMs": round(got, 2), "wantMs": round(want, 2),
          # explicit, so it cannot fall back to whatever `_used_gas` happens to be now
          "mode": self._verify_mode or "res",
          # nosetcancel2pnw: `cancel` stays in the record (same keys as before) and is always False -- the overshoot
          # cancel rule was removed 2026-09-14. `overshootAction` says so, so a record is never read as "did not trip".
          "driverBtn": self._verify_driver_btn, "cancel": False, "overshootAction": "none",
          # Fable review (a): the set in mph, for reading the record at a glance. units2pnw: gotMs/wantMs are true m/s,
          # so these are true mph whatever the cluster shows; `unit` says what it showed. On an "unknown" unit carstate
          # assumed mph, and a km/h cluster then reads gotDisplayMph ~1.6x wantDisplayMph on every gas-set.
          "gotDisplayMph": round(got / 0.44704, 1), "wantDisplayMph": round(want / 0.44704, 1),
          "unit": unit,
        }))
        if reason != "ok":
          out.records[-1]["loud"] = True
        if unit == "unknown" and self._verify_unit != "unknown":
          # units2pnw (Rule 2): this verify's reason rests on carstate's mph ASSUMPTION. selfdrived warns on
          # this flag, which is set only when the unit CHANGES to unknown -- not on every press while it stays unknown.
          out.records[-1]["unitAssumed"] = True
        self._verify_unit = unit
        self._verify_until = None

    # --- arm on the lateral-only edge OR on any later brake press (gate 1) ---------------------
    lat = bool(i.lateral_only)
    # `is False`, not `not self._lat_prev`: a None (never-observed) previous state must NOT
    # produce a rising edge -- see the _lat_prev comment in __init__.
    lat_rising = lat and self._lat_prev is False
    self._lat_prev = lat

    # Brake EDGE detection -- for re-arming AND for the opt-out -- reads the PEDAL ONLY, never
    # regen. Regen braking flickers as the driver modulates, and every flicker would be an "edge":
    # that would manufacture both spurious re-arms and spurious double-taps (Gemini review
    # 2026-09-06, finding C). Gating and the arm precondition still use brake-or-regen; this is
    # only about detecting a discrete PRESS.
    pedal = bool(i.brake_pressed)
    pedal_rising = pedal and self._pedal_prev is False
    if pedal_rising and self._pedal_off_t is not None and (i.now - self._pedal_off_t) < BRAKE_DEBOUNCE_S:
      pedal_rising = False                      # chatter within one press, not a second press
    if self._pedal_prev is not False and not pedal:
      self._pedal_off_t = i.now                 # pedal just came up (or first observation, released)
    self._pedal_prev = pedal
    gas_rising = bool(i.gas_pressed) and self._gas_prev is False
    self._gas_prev = bool(i.gas_pressed)
    if not lat:
      self._lat_braked = False

    # brakeretry2pnw: the opt-out. Evaluated on the brake EDGE, before arming, so a second press
    # suppresses rather than re-arms.
    if pedal_rising:
      double = self._last_brake_t is not None and (i.now - self._last_brake_t) <= DOUBLE_BRAKE_S
      post_resume = self._fired_t is not None and (i.now - self._fired_t) <= REJECT_AFTER_FIRE_S
      if double or post_resume:
        self._suppressed = True
        why = "doubleBrake" if double else "postResumeBrake"
        # Rule 2: a feature that quietly stops acting is exactly the thing that must say so.
        out.records.append(self._snap(i, {"phase": "suppress", "reason": why, "fired": False}))
        if self._armed:
          # An episode was open. The driver has just overruled it; end it now rather than letting
          # its window keep running behind the opt-out they just asked for.
          if self._offer_t is not None:
            # Fable review (c): the offer on the wire ends here too; say so, as every other withdrawal does.
            # (A brake during an offer is always within REJECT_AFTER_FIRE_S of the fire, so this is the only
            # disarm path an in-flight offer can reach.)
            out.records.append(self._snap(i, {"phase": "offerEnd", "reason": why, "fired": True}))
          self._terminate(i, out, why)
          self._disarm()
      self._last_brake_t = i.now

    # brakeretry2pnw: EVERY brake press while MADS is holding lateral opens a resume opportunity,
    # not only the first one after cruise dropped. Before this, arming required the RISING EDGE of
    # lateral_only, which can happen only once per cruise-off transition -- so if that single
    # attempt refused for any reason (the 2026-09-06 drive refused on `gas` one second in), there
    # was no second chance until the driver manually re-engaged cruise to create a new edge. The
    # driver's rule is "I can always push the brake", and this is that rule: press, release, resume.
    if pedal_rising and lat and self._suppressed:
      # Rule 2: this is a no-resume class of its own, and the whole 2026-09-06 investigation was
      # about diagnosing a no-resume. Silence here would recreate exactly that problem.
      out.records.append(self._snap(i, {"phase": "refuse", "reason": "suppressed", "fired": False}))

    start = (lat_rising or (lat and pedal_rising)) and not self._suppressed
    # engagegoal2pnw (owner goal 2026-09-13): "If I want to resume longitudinal control and accelerate I
    # can either push the + or I should be able to hit the gas pedal once and then it should ... set the
    # new speed." An episode used to open ONLY on a brake press, and dies ARM_MAX_S after the last pedal
    # activity -- so a red light held on the brake for more than 20 s, or any pedal-free stretch that long,
    # left the accelerator doing nothing for the rest of the steering-only state. The accelerator now opens
    # a SET-mode episode of its own when none is open -- but only for the FIRST press after the brake
    # (`_gas_spent`, owner decision 2026-09-13: no time limit, first press only). It can never offer RESUME
    # (mode is fixed to SET at the arm), so ARM_MAX_S keeps its whole meaning for RESUME: no hand-back of a
    # remembered speed minutes after the brake. Still inside the envelope: steering-only that began with a
    # brake (`_lat_braked`), the double-tap / post-resume opt-out (`_suppressed`), and every SET gate below.
    gas_start = (not start and gas_rising and self._lat_braked         # _lat_braked is cleared whenever not lat
                 and not self._armed and not self._suppressed and not self._gas_spent)
    if not start and gas_rising and lat and not self._armed and not self._suppressed and self._gas_spent:
      # Rule 2: a later press the first-press rule ignores says so, exactly once per press.
      out.records.append(self._snap(i, {"phase": "refuse", "reason": "gasSpent", "gate": None, "fired": False,
                                        "mode": "set"}))
    if not start and gas_rising and lat and not self._armed and self._suppressed and not pedal_rising:
      # Rule 2 (Fable review 2026-09-13, F1): the accelerator is now an engage input, so a press the opt-out
      # refuses must say so exactly as a refused brake press does above -- `gas:true` in the snap tells them apart.
      # Gated on `lat`, NOT `_lat_braked` (Fable re-review): a post-resume rejection brake suppresses on the very
      # frame lateral-only rises, so that stretch never sets `_lat_braked` and the refusal would be silent.
      # `not pedal_rising`: a brake edge on the same tick has already written the refusal above.
      out.records.append(self._snap(i, {"phase": "refuse", "reason": "suppressed", "fired": False, "mode": "set"}))

    if start or gas_start:
      if self._armed:
        # A previous episode is still open. _terminate() is a no-op if its terminal record was
        # already written, so this cannot double-report -- but an arm still inside its window has
        # NO terminal record yet, and dropping it silently would break the one-terminal-record-per
        # -arm contract in the class docstring (Fable review 2026-09-06, P4).
        self._terminate(i, out, "reBrake")
        self._disarm()
      # ARM. Snapshot the captured set speed and its age RIGHT HERE -- nothing after this point may
      # move _armed_set, so the target can never drift after the driver's foot left the brake.
      self._armed = True
      self._arm_t = i.now
      self._done = False
      self._terminal = False
      self._released_t = None
      self._offer_t = None
      self._eid = round(i.now, 3)
      age = (i.now - self._set_t) if self._set_t is not None else float("inf")
      self._armed_set_age = age if math.isfinite(age) else -1.0
      self._armed_set = self._set_ms if (self._set_ms is not None and age <= SET_MAX_AGE_S) else None
      self._last_block = "armed"
      if start:
        self._gas_spent = False        # a new brake press arms the first-press rule again
        self._gas_v_max = 0.0          # ...and forgets the speed of any press before it
      if gas_start:
        self._used_gas = True          # SET mode from the first tick: a one-tick tap must not fall back to RES
        if _finite(i.v_ego):
          self._gas_v_max = max(self._gas_v_max, float(i.v_ego))
        out.records.append(self._snap(i, {"phase": "arm", "reason": "gas", "fired": False}))
        return out
      out.records.append(self._snap(i, {"phase": "arm", "reason": None, "fired": False}))
      if not (i.brake_pressed or i.regen_braking):
        # Fable A2: mads_pnw only ever raises lateral_only on a frame where `braking` is true (both
        # the immediate arm and the brake-grace arm test it), so this is unreachable today. It is
        # here so that a FUTURE mads arming path cannot silently hand this feature an episode that
        # was not brake-induced -- the one precondition the whole envelope rests on.
        self._done = True
        self._terminate(i, out, "noBrake")
        return out
      self._lat_braked = True
      if self._armed_set is None and not i.cruise_available:
        # The ACC master is off: neither button can do anything, so end it now and name the real
        # cause rather than the `noSet` that the master being off just caused.
        self._done = True
        self._terminate(i, out, "accOff")
      return out

    if not self._armed:
      return out

    if not lat:
      # Lateral-only ended -- either openpilot re-engaged (our press worked, or the driver pressed
      # resume themselves) or steering was lost. Either way this arm is over.
      self._terminate(i, out, "latOff")
      self._disarm()
      return out

    # --- gate 7: aborts. Any of these ends the arm outright (no retry until a new brake cycle). --
    # gasset2pnw: the accelerator is no longer an abort -- it is how the driver names the speed.
    # While it is down, this episode's target switches to "whatever speed they end up at", and the
    # release clock is held: it starts when BOTH pedals are up (gate 2 below).
    if i.gas_pressed and self._gas_spent:
      # engagegoal2pnw, first press only: the press that was allowed to name the speed has been used (its
      # lift-off was judged), so this is a LATER press in the same steering-only stretch -- an overtake, or a
      # re-press after a refused lift-off. It must never engage cruise. End the episode with one record:
      # the arm's terminal if it had none yet (`gate` = what refused the first press), else a standalone one.
      if self._offer_t is not None:
        out.records.append(self._snap(i, {"phase": "offerEnd", "reason": "gasSpent", "fired": True}))
        self._offer_t = None
      gate = None if self._terminal else self._last_block
      out.records.append(self._snap(i, {"phase": "refuse", "reason": "gasSpent", "gate": gate, "fired": False,
                                        "mode": "set"}))
      self._disarm()
      return out
    if i.gas_pressed and _finite(i.v_ego):
      self._gas_v_max = max(self._gas_v_max, float(i.v_ego))
    if i.gas_pressed:
      # engagegoal2pnw / D1 (Rule 2): the driver went back on the power while a lift-off window was open
      # and refusing. `_last_block` -- the gate that held it (`decelUnknown`, `slow`, `noEntry`, a lead
      # gate...) -- is about to be overwritten with "gas", and until now that refusal left no record at
      # all (09-13 12:43:28 was visible only through a later record's `decel`). Non-terminal, like
      # `offerEnd`: the arm stays open and still ends in exactly one fire/refuse. Only once the window
      # has actually been judged by a gate (not still "settling"), so pedal modulation cannot flood the log.
      if (self._released_t is not None and not self._done and self._offer_t is None
          and self._last_block != "settling"):
        out.records.append(self._snap(i, {"phase": "lift", "reason": self._last_block, "fired": False,
                                          "liftS": round(i.now - self._released_t, 2)}))
      self._used_gas = True
      self._released_t = None
      self._last_block = "gas"
      # Hold the arm alive while the driver is actually on the power. A freeway on-ramp is easily a
      # 20 s acceleration, and ARM_MAX_S measured from the brake would expire the episode before
      # they ever lifted off -- losing exactly the case this mode exists for. The clock therefore
      # measures from the last moment the driver was doing something, not from the brake; once the
      # foot comes up the ordinary 0.5-3.0 s window applies and settles it within seconds either way.
      self._arm_t = i.now
      if self._done or self._offer_t is not None:
        # This arm has already made its attempt (offer in flight, fired, or window closed) and the
        # driver has now gone back on the power. Close it out and start a FRESH episode.
        #
        # Fable review 2026-09-07, D1/D2/D3 -- three reproduced silent failures, all from re-opening
        # an arm in place instead of restarting it:
        #   D1 the eid was kept, and the executor latches ONE press per eid -- so the SET was
        #      refused while the brain logged `fire`, and selfdrived then warned about the panda
        #      safety pin. A wild-goose chase pointing at the wrong subsystem entirely.
        #   D2 `_terminate` is a no-op once `_terminal` is set, so a later refusal wrote NO record.
        #   D3 if the offer had already expired, `_done` stayed True and `if self._done: return`
        #      killed the gas-set outright -- no press, no record, arm dying 20 s later in silence.
        if self._offer_t is not None:
          out.records.append(self._snap(i, {"phase": "offerEnd", "reason": "gas", "fired": True}))
          self._offer_t = None
        if not self._terminal:
          self._terminate(i, out, "gas")
        self._eid = round(i.now, 3)      # a NEW key, or the executor will refuse the press
        self._done = False
        self._terminal = False
        self._fired_mode = None
        out.records.append(self._snap(i, {"phase": "arm", "reason": "gasReopen", "fired": False}))

    abort = None
    if i.blocked:
      abort = "blocked"
    elif i.op_enabled:
      abort = "opEngaged"
    elif not i.cruise_available:
      abort = "accOff"
    elif i.cruise_enabled:
      # Stock cruise is back without openpilot re-engaging (driver hit resume themselves, or the
      # PCM did). Nothing left to ask for.
      abort = "ccOn"
    elif i.now - self._arm_t > ARM_MAX_S:
      abort = "armExpired"
    if abort is not None:
      if self._offer_t is not None:
        # An offer was on the wire; say so explicitly rather than letting it vanish from the log.
        out.records.append(self._snap(i, {"phase": "offerEnd", "reason": abort, "fired": True}))
      self._terminate(i, out, abort)
      self._disarm()
      return out

    # --- gate 2: the brake must be FULLY released before the clock starts ----------------------
    braking = bool(i.brake_pressed) or bool(i.regen_braking) or bool(i.gas_pressed)
    if braking:
      # The release clock must measure a CONTINUOUS release. A re-press that BRAKE_DEBOUNCE_S
      # swallowed as chatter is still braking, however long it is then held -- without this reset a
      # fire can land 50 ms after the real release, skipping the settle the window exists to
      # enforce. The old `reBrake` abort used to guarantee this for free (Fable review, P1).
      self._released_t = None
      if self._offer_t is not None:
        out.records.append(self._snap(i, {"phase": "offerEnd", "reason": "braking", "fired": True}))
        self._offer_t = None
      self._last_block = "braking"
      return out
    if self._released_t is None:
      self._released_t = i.now
      # gassetwait2pnw: this is the lift-off tick -- the first frame with BOTH pedals up. Latch the
      # driver's acceleration HERE and never re-read it: half a second later regen has already bitten
      # and a live read would call every lift-off "slowing", which is the bug this replaces.
      self._gas_a0 = float(i.a_ego) if _finite(i.a_ego) else float("nan")
      self._gas_wait_s, self._gas_wait_why = gas_set_wait_s(self._gas_a0)
      # gassetwait2pnw: anchor the decel estimator's window to THIS instant, for the gas-set path only.
      #
      # Why this is needed and not tidying: `decelUnknown` refuses until a decel window lies entirely
      # after lift-off, and the estimator resamples on its own free-running DECEL_WINDOW_S cadence,
      # unaligned to the driver -- so the first qualifying window closes anywhere in
      # [lift + 0.4, lift + 0.8). At the 1.0 s wait that always closed first and the gate never bound
      # (see DECEL_WINDOW_S's own note, Fable 2026-09-07 E4). At 0.5 s it becomes THE binding gate on
      # every single gas-set, and the fire would jitter across [0.5, 0.8) depending only on where the
      # estimator's phase happened to sit. Measured before this line: 0.76 s against a 0.5 s wait.
      #
      # Anchoring makes the first post-lift window close at lift + 0.4 s, deterministically, and it is
      # strictly closer to what the gate asks for -- the window is now exactly [lift, lift + 0.4]
      # instead of one that merely happens to start after lift-off. `_decel_from` then equals
      # `_released_t`, which passes the gate's `<` check, and the fire record's `decel` becomes exactly
      # the post-lift regen the decision was made on.
      #
      # RESUME is deliberately NOT anchored: RELEASE_MIN_S has always lived with this jitter, changing
      # it is not what the owner asked for, and a resume commands acceleration where a SET does not.
      if self._used_gas and _finite(i.v_ego):
        self._v_ref, self._v_ref_t = float(i.v_ego), i.now
      self._last_block = "settling"
      return out

    since = i.now - self._released_t
    if self._used_gas and since >= self._gas_wait_s:
      # first press only: this press's lift-off is judged from here on -- unless it was only a creep
      if self._gas_v_max >= V_EGO_MIN_MS:
        self._gas_spent = True

    # --- an offer already in flight: keep it alive only while every gate still holds ------------
    if self._offer_t is not None:
      block = self._gates(i)
      if block is None and (i.now - self._offer_t) <= OFFER_S:
        out.mode = "set" if self._used_gas else "res"
        out.offer = True
        out.eid = self._eid
        # Re-publish the target SAMPLED AT FIRE, never a live value. For a resume the two are the
        # same, but for a gas-set they are not: `_armed_set` may be a stale capture or None, and
        # re-deriving it here would republish the wrong speed for every tick the offer stands (and
        # raise on None). One episode, one target, fixed the moment it fired.
        out.set_ms = float(self._verify_set)
        return out
      # Offer over. Record why, and stop offering. `_done` stays set: no second offer.
      out.records.append(self._snap(i, {
        "phase": "offerEnd", "reason": block or "expired", "fired": True,
      }))
      self._offer_t = None
      return out

    if self._done:
      return out

    # --- gate 3: the bounded window ------------------------------------------------------------
    if since < (self._gas_wait_s if self._used_gas else RELEASE_MIN_S):
      self._last_block = "settling"
      return out
    if since > RELEASE_MAX_S:
      # The window closed without firing. ONE terminal record, naming the gate that was binding.
      self._terminate(i, out, self._last_block if self._last_block not in ("settling", "armed") else "windowExpired")
      self._done = True
      return out

    # --- gates 5 + 6 + speed floor -------------------------------------------------------------
    block = self._gates(i)
    if block is not None:
      self._last_block = block
      return out

    # --- FIRE. Latch first, offer second. -------------------------------------------------------
    self._done = True
    self._offer_t = i.now
    self._fired_t = i.now
    if self._verify_until is not None:
      # A previous press is still being verified and this fire is about to overwrite its window.
      # Emit its outcome first rather than dropping it (Rule 2). Fable round 3 called this "low",
      # but the C2 test proved the record is lost outright, not merely delayed: without this the
      # RESUME press in a resume-then-gas-then-set sequence is never verified at all.
      out.records.append(self._snap(i, {
        "phase": "verify", "reason": "superseded", "fired": True,
        "mode": self._verify_mode or "res",
        "wantMs": round(self._verify_set, 2) if self._verify_set is not None else None,
      }))
    self._verify_until = i.now + VERIFY_S
    self._verify_driver_btn = bool(i.driver_cruise_button)
    # gasset2pnw: the target is the speed the driver just chose with the accelerator, sampled once
    # here and never moved afterwards -- exactly as _armed_set is for a resume.
    target = float(i.v_ego) if self._used_gas else float(self._armed_set)
    self._verify_set = target
    self._fired_mode = "set" if self._used_gas else "res"
    self._verify_mode = self._fired_mode
    self._terminal = True          # "fire" IS this arm's terminal record
    out.records.append(self._snap(i, {"phase": "fire", "reason": None, "fired": True}))
    out.offer = True
    out.eid = self._eid
    out.set_ms = target
    out.mode = "set" if self._used_gas else "res"
    return out

  def _gates(self, i: ResumeInputs) -> str | None:
    """The gates that are re-evaluated every tick of the window AND every tick of the offer.
    Returns None (clear) or the name of the binding gate. Ordered cheapest/most-fundamental first
    so the reported reason is the most informative one."""
    if not _finite(i.v_ego) or float(i.v_ego) < V_EGO_MIN_MS or i.standstill:
      return "slow"
    # Fable A1 (HIGH), and the single most important gate that was MISSING: openpilot's own state
    # machine must be willing to engage. A NO_ENTRY event (resumeBlocked, tooDistracted, outOfSpace,
    # stockLkas, speedTooHigh, selfdriveInitializing...) carries no DISABLE type, so it does not show
    # up in `blocked` and MADS happily keeps holding lateral. But if our RES press engages the stock
    # ACC while a NO_ENTRY stands:
    #     stock cruise engages -> openpilot REFUSES to engage with it (NO_ENTRY)
    #     -> controlsd.py sends cruiseControl.cancel (`CS.cruiseState.enabled and not CC.enabled`)
    #     -> mads_pnw sees `cruise_engage_edge` and REVOKES lateral (mads_pnw.py:238)
    # Net effect: the driver was steering-only, and OUR press blipped cruise on/off and took their
    # STEERING away. Fail-to-stock in direction, but caused by this feature and entirely avoidable.
    if not i.engageable:
      return "noEntry"
    if self._driver_btn_t is not None and i.now - self._driver_btn_t <= DRIVER_BTN_HOLDOFF_S:
      return "driverBtn"                               # B1: the driver is engaging it themselves
    # gasset2pnw: SET at the CURRENT speed commands no speed change at all, so every gate below --
    # `noSet`, `setRaised`, `setFar`, `staleContext` and the lead gate, all of which exist purely to
    # bound how much ACCELERATION a resume may ask stock ACC for -- is inapplicable by construction.
    # Refusing here would deny the driver cruise over a risk this mode cannot create. Note what is
    # still enforced above and below: the speed floor, openpilot's own engageability, and (in the
    # abort chain and again independently in the executor) that stock cruise is not already engaged,
    # since a SET tap while engaged would move the driver's set speed rather than establish it.
    if self._used_gas:
      # engagegoal2pnw, OWNER DECISION 2026-09-13 ("Ignore regen, set"): regen after lifting off the
      # accelerator (measured 1.5-1.9 m/s^2) no longer refuses a gas-set -- the `slowing` refusal is
      # removed from THIS path only (the RES path never had it). A real brake still wins: gate 2 holds
      # the release clock while the pedal is down, a brake edge re-arms (`reBrake`), and the executor
      # refuses any press with a pedal down. A driver who lifts to slow down and brakes a moment later
      # (weekend, Sat 12:41:50: +0.69 s) is covered by the 1.0 s wait, GAS_SET_RELEASE_MIN_S.
      # `decelUnknown` is kept as a fail-closed backstop: the tap needs a decel window taken entirely after
      # lift-off, so the fire record's `decel` is the post-lift regen this decision was made on. Since the
      # 1.0 s wait (GAS_SET_RELEASE_MIN_S) that window always exists at normal cadence (it closes by +0.8 s),
      # so it binds only when speed samples go missing.
      if self._released_t is None or self._decel_from < self._released_t:
        return "decelUnknown"
      # A SET-to-current commands no acceleration, so the gates that bound how much ACC may speed up
      # do not apply. But "commands no acceleration" is NOT "the road ahead is irrelevant": stock
      # ACC takes a moment to react on engagement. So the two sub-gates that ask whether something
      # is about to hit us -- the 20 m floor and the TTC -- still bind. Only the headway requirement
      # is relaxed, and only because it asks a question a SET cannot raise.
      return lead_gate(i.has_lead, i.d_rel, i.v_lead, i.v_ego, require_headway=False)
    if self._armed_set is None:
      return "noSet"                                    # gate 6
    # gate 6, live half: the truck must not be reporting a set speed ABOVE the one we captured.
    # A LOWER reported set is fine -- resume would go there, which is still not above the driver's,
    # and an absent/zero reading is expected (the Lightning may report 0 in ACC standby).
    # A NON-FINITE reading is a REFUSAL, not a pass: gate 6's live half cannot be evaluated at all,
    # and `isfinite(x) and x > thresh` short-circuits to "permit" on a NaN. Same bug, same fix, as
    # decide_resume() in opendbc/car/ford/icbm_pnw.py.
    if not _finite(i.set_speed_ms):
      return "setUnknown"
    if float(i.set_speed_ms) > 0.0 and float(i.set_speed_ms) > self._armed_set + SET_TOL_MS:
      return "setRaised"
    # the cross-context guard -- see V_MAX_WINDOW_S. An absent rolling max is a REFUSAL, not a pass:
    # the gate cannot be evaluated, and this is the gate that bounds uncommanded acceleration.
    if self._armed_set - float(i.v_ego) > RESUME_MAX_DELTA_MS:
      return "setFar"
    if self._v_max is None or self._armed_set > self._v_max + V_MAX_MARGIN_MS:
      return "staleContext"
    return lead_gate(i.has_lead, i.d_rel, i.v_lead, i.v_ego)     # gate 5
