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

# --- lead -------------------------------------------------------------------
SLOW_LEAD_DV   = 5.0          # m/s: lead this much slower than us -> closing -> Experimental
STOPPED_LEAD_V = 1.0         # m/s: lead below this -> stopped

# --- accelerate-zone --------------------------------------------------------
# Suppress the lowSpeed->Experimental trigger when we're slow but should be ACCELERATING into open
# road -- e2e/Experimental accelerates too timidly there. Two real cases: highway on-ramp merge, and
# stop&go where the lead pulled away leaving a big gap. Only ever REMOVES Experimental (safe: Chill is
# the baseline). Tune on the drive logs (vSet/dRel/vLead are recorded per event).
ACCEL_ZONE_DV        = 6.0   # m/s: set speed at least this far above v_ego => we want to accelerate (~13 mph)
GAP_OPEN_M           = 45.0  # m: a lead farther than this (and not slower) is "not blocking" -> open road
LEAD_PULLAWAY_MARGIN = 1.0   # m/s: lead counts as "not slower than us" if vLead >= vEgo - this
V_SET_MAX_KPH        = 200.0 # kph: above this, treat vCruise as the unset sentinel (255) -> set speed unknown

# --- debounce / dwell (de-flap) ---------------------------------------------
# Drive log showed heavy flapping in stop&go (median 2.3 s between switches, 30 flips/min). Two
# asymmetric dwell gates kill the sawtooth: once in Experimental, hold it EXP_MIN before returning to
# Chill; once in Chill, hold it CHILL_MIN (a re-entry cooldown) before flipping back to Experimental.
FILTER_TAU       = 1.0       # s, FirstOrderFilter time constant per condition
THRESHOLD        = 0.63      # filter level ~= "true for ~1 s"
EXP_MIN_DWELL_S  = 8.0       # s min time in Experimental before it may return to Chill (was MIN_DWELL_S=4)
CHILL_MIN_DWELL_S = 5.0      # s min time in Chill before it may re-enter Experimental (re-entry cooldown)

# --- gentle profile (heavy trucks, e.g. F-150 Lightning) --------------------
# Drive e5f4ecc928: on a winding highway the CURVE condition flipped chill<->experimental ~5x/min, and
# experimental's e2e curve braking felt jerky on the 2948 kg truck ("too aggressive at curve entrance").
# The gentle profile (a) hands curve speed control ENTIRELY to VTSC — which brakes smoothly and is
# decel-limited — by NOT tripping Experimental for curves (curves suppressed in the CES decision), and
# (b) lengthens the dwell so the remaining triggers (stops / slow leads) can't flip-flop. Experimental
# is then reserved for where e2e genuinely helps on a truck: stop lights and closing on a slow lead.
GENTLE_FINGERPRINTS      = ("FORD_F_150_LIGHTNING_MK1",)
GENTLE_EXP_MIN_DWELL_S   = 12.0  # hold Experimental longer before dropping back to Chill
GENTLE_CHILL_MIN_DWELL_S = 8.0   # longer re-entry cooldown

# --- button override states (CESButtonState mem param) ----------------------
BTN_CES  = 0   # CES decides (default)
BTN_CHILL = 1  # forced Chill
BTN_EXP   = 2  # forced full Experimental

# --- event logging (CES_EVENT_LOG: persistent "each adoption" + breadcrumb trail) -------
TICK_S          = 1.0                    # s between heartbeat breadcrumb records (dense for the test drive)
HWY_SPEED_LIMIT = 55 * CV.MPH_TO_MS      # OSM speed limit >= this => coarse "highway" guess
HWY_VEGO        = 55 * CV.MPH_TO_MS       # or sustained speed >= this (authoritative = GPS+OSM+300ft in analysis)
