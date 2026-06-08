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
CES_SPEED_LEAD     = 55 * CV.MPH_TO_MS   # with lead: below this -> allow Experimental
CES_SPEED_LEAD_RET = 58 * CV.MPH_TO_MS   # with lead: return-to-Chill

# --- curve (lateral accel, m/s^2) -------------------------------------------
CURVE_LAT_ACCEL_ENTER = 1.9   # pinned by the anchor set (>1.8 so "easy@70" curves don't trip; <2.0 so Terwilliger/Marquam do)
CURVE_LAT_ACCEL_EXIT  = 1.3   # hysteresis: curve considered "done" below this
CURVE_MAP_LOOKAHEAD_S    = 10.0  # map primary (smooth early trigger)
CURVE_VISION_LOOKAHEAD_S = 3.5   # vision fallback (capped by model confidence)
CRUISING_SPEED = 5.0          # m/s; below this, curve detection is meaningless

# --- lead -------------------------------------------------------------------
SLOW_LEAD_DV   = 5.0          # m/s: lead this much slower than us -> closing -> Experimental
STOPPED_LEAD_V = 1.0         # m/s: lead below this -> stopped

# --- debounce / dwell -------------------------------------------------------
FILTER_TAU  = 1.0            # s, FirstOrderFilter time constant per condition
THRESHOLD   = 0.63          # filter level ~= "true for ~1 s"
MIN_DWELL_S = 4.0           # s minimum time in a mode before returning to Chill

# --- button override states (CESButtonState mem param) ----------------------
BTN_CES  = 0   # CES decides (default)
BTN_CHILL = 1  # forced Chill
BTN_EXP   = 2  # forced full Experimental
