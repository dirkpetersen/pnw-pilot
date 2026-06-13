"""
VTSC (Vision Turn Speed Control) — tunable constants.  See /home/dp/gh/comma/VTSC.md.

All speeds m/s, accels m/s^2, curvature 1/m. Calibrated to the drive-#3 I-5 Terwilliger log
(apex radius ~415 m, the car held 70 mph = 2.6 m/s^2 lateral and did NOT slow -> driver intervened).
Pure literals (no imports) so the core is unit-testable without the openpilot stack.
"""
# --- the two knobs that shape the behavior -----------------------------------
# Tuned for a SMOOTH, SLIGHT adjustment (driver feedback after the first VTSC drive: 57 was too
# aggressive — only a slight trim is wanted). Higher A_LAT_TARGET = less slowdown; lower A_DECEL = gentler.
A_LAT_TARGET = 1.9    # m/s^2 max lateral accel held through a curve. AGGRESSIVENESS knob: lower = slower
                      #   (more margin). At Terwilliger (R~415): 1.9 -> ~62 mph (an ~8 mph trim from 70,
                      #   lateral 2.6 -> 1.9), 1.5 -> ~57 (too firm per drive #4), 2.2 -> ~67 (barely).
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

# --- apex state machine (drive #4) -------------------------------------------
# Zones by TIME-TO-APEX (apexDist / vEgo), so they scale with speed:
#   tta >  HOLD_TTA_S      -> BRAKE   (apex clearly ahead: reduce, finishing before the apex)
#   APEX_TTA_S < tta <= HOLD_TTA_S -> HOLD  (close/uncertain: maintain, NEVER reduce further)
#   tta <= APEX_TTA_S  (or curve straightens) -> RELEASE (at apex: accelerate back to cruise)
HOLD_TTA_S      = 1.2   # s; stop reducing once within this time of the apex (so braking completes earlier)
APEX_TTA_S      = 0.4   # s; "at the apex" -> release the cap and accelerate out
APEX_FINISH_S   = 1.2   # s; aim to reach curve-safe speed this long BEFORE the apex (firmer if needed)
CONFIDENCE_CUT  = 0.5   # m/s (~1.1 mph) immediate cap cut the instant a binding curve is detected, so the
                        #   driver immediately feels VTSC engage (per drive #4). Then braking continues.
CLEAR_CYCLES    = 5     # cycles with no curve before RELEASE -> IDLE (debounce the exit so we don't re-brake
                        #   on the curve we just left)

V_MIN           = 6.7   # m/s (~15 mph) floor — never command a curve speed below this
MIN_CURVATURE   = 1e-4  # 1/m; at or below this the path is "straight" (ignored)
LOOKAHEAD_MAX_S = 8.0   # s; only trust the model's predicted path out to here
CURVE_MIN_POINTS = 3    # debounce: require the curve sustained over >= this many cycles before braking
