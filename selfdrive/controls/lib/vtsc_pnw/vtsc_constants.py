"""
VTSC (Vision Turn Speed Control) — tunable constants.  See /home/dp/gh/comma/VTSC.md.

All speeds m/s, accels m/s^2, curvature 1/m. Calibrated to the drive-#3 I-5 Terwilliger log
(apex radius ~415 m, the car held 70 mph = 2.6 m/s^2 lateral and did NOT slow -> driver intervened).
Pure literals (no imports) so the core is unit-testable without the openpilot stack.
"""
# --- the two knobs that shape the behavior -----------------------------------
# Tuned for a SMOOTH, SLIGHT adjustment (driver feedback after the first VTSC drive: 57 was too
# aggressive — only a slight trim is wanted). Higher A_LAT_TARGET = less slowdown; lower A_DECEL = gentler.
A_LAT_TARGET = 3.0    # m/s^2 max lateral accel held through a curve. AGGRESSIVENESS knob: lower = slower
                      #   (more margin). RAISED 2.2 -> 3.0 (Snoqualmie I-90 2026-07-01, see
                      #   drives/2026-07-01/hotspot-drive/DRIVE_REPORT.md: VTSC over-capped repeatedly and the
                      #   driver had to override with throttle / take control; calibration point 62 -> wanted
                      #   72 mph == x1.35 on A_LAT (sqrt -> x1.16 on speed). 3.0 ~= 0.31 g lateral; use CES
                      #   Light/Off in poor grip). Prior: RAISED 1.9 -> 2.2 (I-90 2026-06-27, still ~10-15 too low).
                      #   At Terwilliger (R~415): 3.0 -> ~78 mph, 2.2 -> ~67, 1.9 -> ~62, 1.5 -> ~57.
                      #   Gentler curves scale up automatically: v_safe = sqrt(a_lat/kappa), so R~600 m
                      #   -> ~76 mph (no cap at 70) — only curves tighter than ~R550 bind at all.
A_DECEL      = 1.2    # m/s^2 decel the envelope plans for -> how gently speed bleeds off. ~0.12 g, like
                      #   easing off the gas; for the ~8 mph trim it starts braking ~80 m before the apex.

# --- safety bounds -----------------------------------------------------------
# Drive #4 feedback reshaped the goal: the absolute priority is to FINISH slowing BEFORE the apex so we
# can ACCELERATE at the apex. Pre-apex braking may be firmer/earlier if needed (driver doesn't mind), so
# the decel ceiling is raised. The apex behavior is handled by the BRAKE->HOLD->RELEASE state machine.
A_DECEL_MAX     = 2.5   # m/s^2 HARD ceiling on commanded decel (rate-limit) — raised from 1.5 so VTSC CAN
                        #   brake firmly enough (~0.25 g) to reach curve speed BEFORE the apex. Still bounded
                        #   so it can never slam.
A_RELAX         = 1.5   # m/s^2 rate the applied cap eases back UP (apex reached / curve cleared) -> smooth
                        #   acceleration out to cruise, never a jump

# --- apex state machine (drive #4; speed-gated drive #5) ---------------------
# Zones by TIME-TO-APEX (apexDist / vEgo), so they scale with speed — BUT the HOLD/RELEASE
# transitions are now ALSO gated on having actually slowed to ~curve-safe speed (drive #5 Terwilliger
# log: the old time-only logic froze the cap into HOLD ~1.2 s before the apex while still ~20 mph hot,
# then RELEASED/accelerated right at the apex while ~27 mph over safe — driver disengaged):
#   tta >  HOLD_TTA_S, OR still above safe speed (and apex still ahead) -> BRAKE  (reduce toward safe)
#   close to apex AND at safe speed -> HOLD     (maintain, never reduce further)
#   at the apex AND at safe speed (or curve straightens) -> RELEASE  (accelerate back to cruise)
# Never reduce AT/after the apex (tta <= APEX_TTA_S -> HOLD at worst, never a fresh brake); we only
# avoid ACCELERATING out while still materially too fast.
# sharpcurve2pnw: shifted EARLIER so the curve ENTRANCE is the slowest point and we accelerate out
# BEFORE the apex (driver: "the entrance should be the slowest... at the apex you should be able to
# accelerate, sometimes even before the apex"). Reach curve-safe speed ~APEX_FINISH_S before the apex
# (~the entrance), HOLD briefly, then RELEASE ~APEX_TTA_S before the apex. The at-safe gate
# (RELEASE_SPEED_MARGIN) still guards RELEASE, so we only accelerate pre-apex once actually slowed to
# curve-safe speed -> there's lateral margin at the apex.
HOLD_TTA_S      = 2.5   # s; reach this far from the apex (~the entrance) at safe speed, then HOLD (stop reducing)
APEX_TTA_S      = 1.2   # s; begin accelerating out BEFORE the apex (only once at safe speed)
RELEASE_SPEED_MARGIN = 0.10  # release/hold only when vEgo <= vCurveSafe*(1+this); else keep braking/holding
APEX_FINISH_S   = 2.5   # s; reach curve-safe speed this long BEFORE the apex (~the entrance = slowest point)
CONFIDENCE_CUT  = 0.5   # m/s (~1.1 mph) immediate cap cut the instant a binding curve is detected, so the
                        #   driver immediately feels VTSC engage (per drive #4). Then braking continues.
CLEAR_CYCLES    = 5     # cycles with no curve before RELEASE -> IDLE (debounce the exit so we don't re-brake
                        #   on the curve we just left)

V_MIN           = 6.7   # m/s (~15 mph) floor — never command a curve speed below this
MIN_CURVATURE   = 1e-4  # 1/m; at or below this the path is "straight" (ignored)
# ces-i90-2pnw: GUARANTEED >=1 mph cue on EVERY real bend. A gentle curve whose curve-safe speed stays
# ABOVE cruise (the decel envelope never binds) used to get NOTHING; the driver wanted a definite "I see
# the curve" dip at the START of every curve. CUE_MIN_CURVATURE is that trip threshold: ~5 deg of heading
# change over the upcoming ~200 m of road == radius ~2300 m. It sits ABOVE straight-road / lane noise
# (MIN_CURVATURE, R~10 km) and BELOW where VTSC already brakes for real (~R840 m @ 90 mph), so it only
# adds the CONFIDENCE_CUT (>=1 mph) nibble in the "mild curve" band and changes nothing else. The same
# state machine then holds the dip through the bend and eases back out -> nothing stacks (floor is
# v_cruise - CONFIDENCE_CUT, never lower for a cue-only curve). Pure geometry: curvature = 1/radius.
CUE_HEADING_RAD = 5.0 * 3.141592653589793 / 180.0  # 5 degrees, in radians
CUE_OVER_M      = 200.0                             # ...measured over the upcoming ~200 m of path
CUE_MIN_CURVATURE = CUE_HEADING_RAD / CUE_OVER_M    # ~4.36e-4 1/m  (radius ~2292 m)
LOOKAHEAD_MAX_S = 8.0   # s; only trust the model's predicted path out to here
CURVE_MIN_POINTS = 3    # debounce: require the curve sustained over >= this many cycles before braking

# --- map curve speed (MTSC) — added on ces-i90-2pnw from the Snoqualmie Pass drive ----------
# VTSC is otherwise VISION-ONLY (model path curvature, ~5-6 s horizon). On a sharp curve the model sees
# it too LATE to finish braking before the entry (drive feedback "4:56": the decel happened IN the
# curve, should have been before it). pfeiferj mapd publishes MapTargetVelocities (per-point curve
# safe-speeds) with a much longer horizon, so folding the map curve in as an additional brake source
# lets VTSC begin braking earlier AND catch sharp curves the vision under-reads (Snoqualmie summit: map
# target ~58 mph while vision capped at ~82). Default ON via the VtscMapCurves param: the new pfeiferj
# mapd is reliable enough to lean on (map safe-speeds were the old MTSC deferral reason). The map curve
# is fed through the SAME decel-limited + floored (V_MIN) + only-reduce state machine as the vision path,
# so even a wrong map speed brakes SMOOTHLY (never slams) and stays bounded — which is what makes
# defaulting it ON safe. Set VtscMapCurves=0 to fall back to vision-only.
MAP_LOOKAHEAD_S   = 12.0  # s; trust map curve targets within v_ego * this (longer reach than vision's 8 s)
MAP_MIN_SLOWDOWN  = 4.5   # m/s; only fold a map curve whose (scaled) target is this far below cruise (~10 mph)

# --- sharpcurve2pnw: earlier lookahead + regen-coast slowdown for blind curves -
# Root cause of the recurring sharp-curve "TAKE CONTROL" (I-90 descents): pfeiferj mapd publishes a
# FIXED 500 m path horizon (MIN_WAY_DIST in mapd/settings/const.go), but VTSC only scanned
# v_ego*MAP_LOOKAHEAD_S (~370 m @ 70 mph) and CES ~310 m — throwing away ~130-190 m (~6 s) of warning,
# so on the blindest curves the slowdown started too late.
# Driver model (EV / Model S): "on the freeway braking should almost never be required; as soon as you
# decelerate, regen [recoup] makes it much slower." Lifting the accelerator gives ~0.2 g of regen, which
# is plenty to bleed curve speed IF we see the curve early enough. So the fix leans on LOOKAHEAD + REGEN,
# not friction braking. Goal: get off the gas early so the curve ENTRANCE is the slowest point, then
# accelerate out (sometimes before the apex). All changes ride the CES master and only ever REDUCE speed:
#   1) scan the FULL available 500 m and pick the most-binding curve by decel envelope (not nearest);
#   2) cap the normal commanded decel to regen authority (REGEN_A_DECEL) -> coast/regen, no friction brake;
#   3) allow firm friction braking (SHARP_A_DECEL_MAX) ONLY as a last resort when regen alone can't reach
#      curve-safe speed before a blind SHARP curve's entrance (anti-take-control); still bounded — no slam.
MAP_SOURCE_HORIZON_M = 500.0  # m; mapd's hard MIN_WAY_DIST cap — no path data exists beyond this (don't scan past)
MAP_BRAKE_MARGIN_M   = 40.0   # m; cushion added to the computed brake distance when sizing the scan horizon
REGEN_A_DECEL        = 2.0    # m/s^2 (~0.2 g) EV regen authority — NORMAL commanded-decel ceiling (no friction brake)
SHARP_CURVE_V        = 30.0   # m/s (~67 mph); a map curve target below this is "sharp" (in sync with CES CURVE_SHARP_MAP_V)
SHARP_A_DECEL_MAX    = 2.8    # m/s^2 LAST-RESORT friction-brake ceiling, only when regen can't make a sharp curve — bounded

# --- twisty-DESCENT base trim ("auto-lower the set on twisty descents") -------
# Backstop for blind sharp curves on a winding DOWNHILL — where gravity fights regen and packed blind
# curves are the take-control risk. ONLY trims when BOTH (a) several curves are packed into the upcoming
# 500 m AND (b) the road is descending (pitch < TWISTY_DESCENT_PITCH): hold a lower base cruise through
# the section instead of releasing to full set speed between each blind curve. A FLAT twisty section is
# left at full speed (per-curve VTSC handles it) so we don't lose speed where it isn't needed (driver:
# "we do not want to lose any speed overall"). Bounded by TWISTY_MIN_FACTOR; only ever reduces -> safe.
TWISTY_MIN_CURVES    = 3       # >= this many binding curves within the horizon => "twisty section"
TWISTY_SLOWDOWN      = 3.0     # m/s; a map point counts as a curve when its target is this far below cruise
TWISTY_MIN_FACTOR    = 0.82    # never trim the section base below this * v_cruise (~18% floor, descents only)
TWISTY_DESCENT_PITCH = -0.035  # rad (~ -2 deg); road pitch below this is treated as a descent (trim active)
# I-90 22:08-22:10 PT 2026-06-27: mapd's curve targets (60-68 mph at a 90 set) were the BINDING floor,
# ~10 mph too conservative. Carry more speed through map curves by scaling the map target up (then capped
# at cruise). 1.12x ~= +8 mph at 65; still decel-limited + V_MIN-floored downstream, so it can never slam.
MAP_SPEED_SCALE   = 1.5   # >1 = carry more speed through map curves (less conservative MTSC). RAISED
                          #   1.12 -> 1.5 (Snoqualmie I-90 2026-07-01): map safe-speeds came back absurdly low
                          #   (mapV 22-29 mph on curves taken at 60-85), over-capping + forcing throttle
                          #   overrides. Still decel-limited + V_MIN-floored downstream, so it can't slam.

# --- profiles (DEFAULT vs GENTLE) -------------------------------------------
# The above constants are the DEFAULT tune. On a winding highway the default tune can SAWTOOTH: VTSC
# releases all the way to cruise at each apex then re-brakes for the very next curve, so speed bounces
# repeatedly and feels unsettled. The GENTLE profile fixes the sawtooth and softens the ride:
#   - never brake harder than ~0.15 g for a vision curve (A_DECEL_MAX 2.5 -> 1.5)
#   - recover speed SLOWLY after a curve (A_RELAX 1.5 -> 0.6, ~1.3 mph/s) so a follow-on curve in a
#     SERIES catches a still-reduced speed instead of a re-accelerated one -> the car settles at a
#     sustained gentle speed through the winding section rather than bouncing
#   - bleed speed off more gently approaching the curve (A_DECEL 1.2 -> 1.0)
#   - a hair less slowdown so trims feel light (A_LAT_TARGET 1.9 -> 2.0)
# Only ever makes VTSC GENTLER (still decel-limited, still floored, still <= v_cruise) — safe.
DEFAULT_PROFILE = dict(A_LAT_TARGET=A_LAT_TARGET, A_DECEL=A_DECEL, A_DECEL_MAX=A_DECEL_MAX, A_RELAX=A_RELAX)
GENTLE_PROFILE  = dict(A_LAT_TARGET=3.0, A_DECEL=1.0, A_DECEL_MAX=1.5, A_RELAX=0.6)  # A_LAT tracks DEFAULT (2.2->3.0, 2026-07-01)

# light-ces-gentle: which profile is used is USER-SELECTED via CESMode (1=Light->GENTLE,
# 2=Standard->DEFAULT) in vtsc_controller.py, on ANY car — no car/fingerprint gating.
