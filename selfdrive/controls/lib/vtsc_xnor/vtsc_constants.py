"""
VTSC (Vision Turn Speed Control) — tunable constants.  See /home/dp/gh/comma/VTSC.md.

All speeds m/s, accels m/s^2, curvature 1/m. Calibrated to the drive-#3 I-5 Terwilliger log
(apex radius ~415 m, the car held 70 mph = 2.6 m/s^2 lateral and did NOT slow -> driver intervened).
Pure literals (no imports) so the core is unit-testable without the openpilot stack.
"""
# --- the two knobs that shape the behavior -----------------------------------
# Tuned for a SMOOTH, SLIGHT adjustment (user feedback: at Terwilliger they only needed a small trim,
# do NOT brake hard). Higher A_LAT_TARGET = less slowdown; lower A_DECEL = gentler.
A_LAT_TARGET = 1.9    # m/s^2 max lateral accel held through a curve. AGGRESSIVENESS knob: lower = slower
                      #   (more margin). At Terwilliger (R~415): 1.9 -> ~62 mph (an ~8 mph trim from 70),
                      #   2.2 -> ~67 (barely), 1.5 -> ~57 (firmer). The user drove it at 70 / 2.6 m/s^2.
A_DECEL      = 1.2    # m/s^2 decel the envelope plans for -> how gently speed bleeds off. ~0.12 g, like
                      #   easing off the gas; starts ~80 m before the apex for the ~8 mph trim above.

# --- safety bounds -----------------------------------------------------------
A_DECEL_MAX     = 1.5   # m/s^2 HARD ceiling on commanded decel (rate-limit) — it can never brake harder
                        #   than ~0.15 g, so a late/sudden curve still won't slam. Smoothness guarantee.
V_MIN           = 6.7   # m/s (~15 mph) floor — never command a curve speed below this
MIN_CURVATURE   = 1e-4  # 1/m; at or below this the path is "straight" (ignored)
LOOKAHEAD_MAX_S = 8.0   # s; only trust the model's predicted path out to here
CURVE_MIN_POINTS = 3    # debounce (Phase-2 wrapper): require the curve sustained over >= this many points
