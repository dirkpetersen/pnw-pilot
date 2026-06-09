"""
VTSC (Vision Turn Speed Control) — tunable constants.  See /home/dp/gh/comma/VTSC.md.

All speeds m/s, accels m/s^2, curvature 1/m. Calibrated to the drive-#3 I-5 Terwilliger log
(apex radius ~415 m, the car held 70 mph = 2.6 m/s^2 lateral and did NOT slow -> driver intervened).
Pure literals (no imports) so the core is unit-testable without the openpilot stack.
"""
# --- the two knobs that shape the behavior -----------------------------------
A_LAT_TARGET = 1.5    # m/s^2 max lateral accel held through a curve. AGGRESSIVENESS knob: lower = slower
                      #   (more margin). At Terwilliger (R~415): 1.5 -> ~57 mph, 1.2 -> ~50 mph, 2.0 -> ~64.
A_DECEL      = 1.5    # m/s^2 decel used to build the speed envelope -> when braking STARTS. Lower = earlier
                      #   + gentler. 1.5 starts ~110 m before the apex (matches the Terwilliger calibration).

# --- safety bounds -----------------------------------------------------------
A_DECEL_MAX     = 3.0   # m/s^2 hard cap on commanded decel (the planner wrapper rate-limits to this)
V_MIN           = 6.7   # m/s (~15 mph) floor — never command a curve speed below this
MIN_CURVATURE   = 1e-4  # 1/m; at or below this the path is "straight" (ignored)
LOOKAHEAD_MAX_S = 8.0   # s; only trust the model's predicted path out to here
CURVE_MIN_POINTS = 3    # debounce (Phase-2 wrapper): require the curve sustained over >= this many points
