"""
CES2 — the redesigned CES decision core (ces2core2pnw). Design: CES2-STUDY.md (branch decstudy2pnw).

Three driver-approved adoptions from the sunnypilot-DEC / FrogPilot-CEM study:
  1. GRADED STOP-URGENCY from the model trajectory endpoint (StopEvidence): endpoint shorter than a
     speed-indexed expected distance => stopping evidence, graded LOW/MEDIUM/HIGH — not binary.
  2. PRECEDENCE PRINCIPLE: stop-evidence outranks accelerate-preference at ANY speed. The 8 m/s
     red-light floor (ACCEL_ZONE_MIN_V), the pull-away speed-band carve-out
     (PULLAWAY_MIN_V/DREL_LO/HI band) and the stop-recency guard (PULLAWAY_STOP_CLEAR_S as a
     separate rule) DO NOT EXIST here — the graded signal's own hysteretic ~2 s decay plus the
     "urgency LOW required for WANT-FASTER" precedence absorb all three (study §4/§5.3).
  3. CEM adoptions: standstill hold (keep Experimental at v=0 while the model still says stopped),
     turn-signal condition (blinker + no lane-change intent below 55 mph, CEM fleet default speed),
     and PER-CONDITION debounce filters (stop runs the 2x faster tau=0.5) instead of the single
     aggregate filter — deletes the cross-condition masking artifact.

KEPT from CES v1 (unchanged semantics, same constants module): map-curve machinery incl. the
freeway gate + sharp-curve exception + tiered scale, the OSM lowSpeed highway gate, the
slow/stopped-lead thresholds, the asymmetric 8/5 s mode dwells (12/8 gentle), the stop-intent fast
path (now a special case of the precedence principle: HIGH urgency == fast path), the
PullAwayTracker opening evidence (band-free, as the general "lead genuinely opening" test), and the
per-condition toggles.

Rule count: v1 core ~19 independently-tuned trigger/suppression rules -> CES2 12 (7 ladder rules +
freeway gate + sharp exception + OSM lowSpeed gate + Light-mode handoff + dwell pair).

CALIBRATION CAVEAT (study §4, must-read): the STOP_BP/STOP_DIST expected-distance table below is
ported verbatim from sunnypilot DEC (WMACConstants.SLOW_DOWN_BP/DIST, version 2025-6-30) as the
STARTING ANCHOR ONLY. It is calibrated on *their* models; ours is lebowski. It must be re-anchored
on our own logs via tools/ces2_replay.py + the mdlEndX telemetry field before the graded signal is
trusted at full strength. Until then Ces2Core stays default OFF (shadow-only).

SAFETY: pure decision logic, same wiring surface as CES v1 — decides chill vs Experimental only,
never touches panda safety, gated by the same CESMode master + openpilotLongitudinalControl.
"""
import math

from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw_constants as C

# --- StopEvidence: DEC-anchor expected-distance table (kph -> m) -------------------------------
# sunnypilot DEC WMACConstants.SLOW_DOWN_BP/SLOW_DOWN_DIST, 2025-6-30 — TO BE RE-ANCHORED on
# lebowski rlogs (see module docstring). "At v kph the model's 10 s trajectory endpoint should
# reach at least this far; shorter = the model plans to slow/stop."
STOP_BP_KPH = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 55.0, 60.0]
STOP_DIST_M = [32.0, 46.0, 64.0, 86.0, 108.0, 130.0, 145.0, 165.0]

URGENCY_MED = 0.3            # DEC SLOW_DOWN_PROB — MEDIUM: charges the stop filter normally
URGENCY_HIGH = 0.7           # DEC emergency tier — HIGH: takes the fast path (like shouldStop)
URGENCY_DECAY_TAU_S = 1.7    # hysteretic decay: HIGH(1.0) falls below MEDIUM(0.3) in ~2 s — the
                             # in-signal replacement for the v1 PULLAWAY_STOP_CLEAR_S recency guard
CRITICAL_ENDPOINT_FRAC = 0.3  # endpoint under 30% of expected => imminent stop, urgency doubled
SPEED_FACTOR_MIN_KPH = 25.0   # above this the urgency scales up with speed (DEC speed factor)
SPEED_FACTOR_DIV_KPH = 80.0

LEVEL_LOW, LEVEL_MED, LEVEL_HIGH = 0, 1, 2

# --- TURN condition (CEM F2, fleet default ON @55 mph w/ lane detection) -----------------------
TURN_MAX_V = 55 * CV.MPH_TO_MS   # CEM CESignalSpeed fleet default; v1 of the lane test is the
                                 # model's own lane-change intent (laneChangeState != off)

STOP_FILTER_TAU = 0.5            # CEM stop-light filter: 2x faster charge than the others (1.0)

# Gemini adversarial catch (ces2core2pnw review, Trap 1): when the endpoint signal is BLIND
# (mdl_end_x unknown/<=0 — a model hiccup live, or a pre-mdlEndX log in replay), the graded
# urgency cannot carry the safety burden the retired 8 m/s red-light floor carried — a no-lead
# "open road + high set" below this speed is then presumed a red-light approach exactly as in v1
# (WANT-FASTER stays suppressed). With a LIVE endpoint the graded precedence rules instead.
# Value == v1's ACCEL_ZONE_MIN_V; it applies ONLY in the blind state (belt-and-suspenders per
# study §4 "the floor stays in as a fallback until the replay proof").
ENDPOINT_BLIND_FLOOR_V = 8.0     # m/s (~18 mph)

STANDSTILL_V = 0.3               # m/s fallback when carState.standstill is not in the signals dict

# ladder order == reason priority (first-hit-wins)
REASON_ORDER = ("stop", "turn", "curve", "slowLead", "lowSpeed")


def _interp(x, xp, fp):
  """Tiny pure linear interp (clamped) — keeps this module numpy-free for the replay harness."""
  if x <= xp[0]:
    return fp[0]
  if x >= xp[-1]:
    return fp[-1]
  for i in range(1, len(xp)):
    if x <= xp[i]:
      f = (x - xp[i - 1]) / (xp[i] - xp[i - 1])
      return fp[i - 1] + f * (fp[i] - fp[i - 1])
  return fp[-1]


def endpoint_urgency(v_ego: float, mdl_end_x: float, model_should_stop: bool) -> float:
  """PURE raw stop urgency 0..1 for one tick (DEC's _calculate_slow_down math, unfiltered).
  mdl_end_x <= 0 means 'endpoint unknown / not plumbed' (old logs, model hiccup) -> the endpoint
  half is NEUTRAL (0.0) and only shouldStop contributes — never invent stopping evidence from
  missing data. shouldStop always saturates to 1.0 (binary intent == maximum urgency)."""
  urgency = 0.0
  if mdl_end_x is not None and mdl_end_x > 0.0:
    v_kph = max(float(v_ego), 0.0) * CV.MS_TO_KPH
    expected = _interp(v_kph, STOP_BP_KPH, STOP_DIST_M)
    if mdl_end_x < expected:
      shortage_ratio = (expected - mdl_end_x) / expected
      urgency = min(1.0, shortage_ratio * 2.0)
      if mdl_end_x < expected * CRITICAL_ENDPOINT_FRAC:
        urgency = min(1.0, urgency * 2.0)
      if v_kph > SPEED_FACTOR_MIN_KPH:
        urgency = min(1.0, urgency * (1.0 + (v_kph - SPEED_FACTOR_MIN_KPH) / SPEED_FACTOR_DIV_KPH))
  if model_should_stop:
    urgency = 1.0
  return urgency


class StopEvidence:
  """Graded, hysteretic stop-evidence signal (stateful, unit-tested).
  Rises INSTANTLY to the raw urgency (churn toward stopping is the safe direction — DEC's
  emergency bypass), decays exponentially (tau URGENCY_DECAY_TAU_S) when the raw signal falls, so
  a shouldStop/endpoint flicker while a lead clears a yellow light keeps blocking accelerate-
  preference for ~2 s — the v1 stop-recency guard, absorbed into the signal itself."""

  def __init__(self):
    self.urgency = 0.0

  def reset(self):
    self.urgency = 0.0

  def update(self, v_ego: float, mdl_end_x: float, model_should_stop: bool, dt: float = DT_CTRL) -> int:
    raw = endpoint_urgency(v_ego, mdl_end_x, model_should_stop)
    if raw >= self.urgency:
      self.urgency = raw
    else:
      alpha = 1.0 - math.exp(-max(dt, 1e-4) / URGENCY_DECAY_TAU_S)
      self.urgency += (raw - self.urgency) * alpha
    return self.level

  @property
  def level(self) -> int:
    if self.urgency >= URGENCY_HIGH:
      return LEVEL_HIGH
    if self.urgency >= URGENCY_MED:
      return LEVEL_MED
    return LEVEL_LOW


def ces2_conditions(s) -> tuple[dict, dict]:
  """PURE ladder primitives: per-condition raw booleans + the chill-preference gates.
  `s` is the SAME signals dict CES v1 consumes, plus:
    stop_urgency        int level from StopEvidence (LEVEL_LOW/MED/HIGH)
    lead_opening        bool from PullAwayTracker (band-free opening evidence)
    standstill          bool (carState.standstill)
    lane_change_intent  bool (modelV2.meta.laneChangeState != off)
    toggles["turns"]    bool (CESTurns param, default OFF)
  Returns (conds, gates): conds keyed by REASON_ORDER; gates carries wantFaster/leadOpen/pacing/
  stopEvidence for telemetry + tests."""
  t = s["toggles"]
  v = s["v_ego"]
  has_lead = bool(s["has_lead"])
  urg = int(s.get("stop_urgency", LEVEL_LOW))
  should_stop = bool(s.get("model_should_stop"))
  # graded stop evidence: MEDIUM+ endpoint contraction, or the binary intent itself
  stop_evidence = urg >= LEVEL_MED or should_stop

  # --- rule 2: LEAD-OPEN -> Chill preference (the ONE unified lead rule) ------------------------
  # pacing: a lead within the curve-pace range that is not slower than us (v1's exact rate test) —
  # live evidence of a drivable line; suppresses CURVE exactly as in v1 (2026-07-08 directive).
  pacing = (has_lead and s["lead_drel"] < C.CURVE_LEAD_PACE_DREL
            and s["lead_vlead"] >= v - C.LEAD_PULLAWAY_MARGIN)
  # lead_open: pacing AND the PullAwayTracker's monotonic-rise evidence — the lead is genuinely
  # OPENING, so holding Experimental's timid e2e would let it pull away (driver rule (a)). This is
  # the band-free generalization of v1's below-floor pull-away exception; it gates LOW-SPEED.
  # PRECEDENCE: like every accelerate-preference it yields to stop evidence — the yellow-light
  # trap (lead clears through while ego must stop) is blocked by the urgency's own ~2 s decay,
  # which is where v1's separate PULLAWAY_STOP_CLEAR_S recency guard went.
  lead_open = pacing and bool(s.get("lead_opening", False)) and not stop_evidence

  # --- rule 7: WANT-FASTER -> Chill preference (accel-zone v2, NO speed floor) ------------------
  # DEC D5's precedence, exactly: want-to-accelerate yields to ANY stop evidence — which is what
  # made the 8 m/s floor unnecessary (a red-light approach shows a contracting endpoint at any
  # speed; an on-ramp's endpoint is pinned at the horizon).
  open_ahead = ((not has_lead)
                or (s["lead_drel"] > C.GAP_OPEN_M and s["lead_vlead"] >= v - C.LEAD_PULLAWAY_MARGIN)
                or bool(s.get("lead_opening", False)))
  # blind-endpoint fallback (see ENDPOINT_BLIND_FLOOR_V): no endpoint signal + no lead + slow =>
  # never adopt the accelerate preference — v1's floor semantics, in exactly the blind state.
  endpoint_blind = float(s.get("mdl_end_x") or 0.0) <= 0.0
  want_faster = (not stop_evidence and s["v_set"] > 0.0
                 and (s["v_set"] - v) > C.ACCEL_ZONE_DV and open_ahead
                 and not (endpoint_blind and not has_lead and v < ENDPOINT_BLIND_FLOOR_V))

  # --- rule 1: STOP (graded; HIGH/shouldStop additionally takes the fast path in the SM) --------
  stop = bool(t["stops"]) and stop_evidence and not has_lead

  # --- rule 3: TURN (CEM F2) — blinker + no lane-change intent below 55 mph ---------------------
  turn = (bool(t.get("turns", False)) and bool(s["blinker"]) and v < TURN_MAX_V
          and not bool(s.get("lane_change_intent", False)))

  # --- rule 4: CURVE — v1 machinery verbatim MINUS the lead-pace/accel-zone gates (owned by the
  # ladder now: pacing and want_faster suppress it below). Freeway gate + sharp exception + tiered
  # scale + Light-mode handoff (toggles["curves"] False in gentle) all unchanged.
  curve = False
  if t["curves"] and v > C.CRUISING_SPEED and not pacing and not want_faster:
    freeway_gated = s["spd_lim"] >= C.CURVE_HWY_GATE
    sharp_curve = 0.0 < s["map_target_v"] * C.tiered_map_scale(s["map_target_v"]) < C.CURVE_SHARP_MAP_V
    if not freeway_gated or sharp_curve:
      map_curve = (s["map_target_v"] > 0.0
                   and (v - s["map_target_v"]) > C.CURVE_MAP_MIN_SLOWDOWN
                   and 0.0 < s["map_target_dist"] / max(v, 1.0) < C.CURVE_MAP_LOOKAHEAD_S)
      vision_curve = (abs(s["curve_lat_accel_vision"]) > C.CURVE_LAT_ACCEL_ENTER
                      and s["time_to_curve"] < C.CURVE_VISION_LOOKAHEAD_S
                      and not s["blinker"])
      curve = map_curve or vision_curve

  # --- rule 5: SLOW-LEAD (unchanged; never suppressed by the chill preferences) -----------------
  slow_lead = (bool(t["lead"]) and has_lead
               and ((v - s["lead_vlead"]) > C.SLOW_LEAD_DV or s["lead_vlead"] < C.STOPPED_LEAD_V))

  # --- rule 6: LOW-SPEED — v1 thresholds + OSM highway gate; chill preferences suppress ---------
  thr = C.CES_SPEED_LEAD if has_lead else C.CES_SPEED
  on_highway = s["spd_lim"] >= C.LOWSPEED_HWY_GATE
  low_speed = (bool(t["low_speed"]) and 1.0 <= v < thr and not on_highway
               and not lead_open and not want_faster)

  conds = {"stop": stop, "turn": turn, "curve": curve, "slowLead": slow_lead, "lowSpeed": low_speed}
  gates = {"wantFaster": want_faster, "leadOpen": lead_open, "pacing": pacing,
           "stopEvidence": stop_evidence}
  return conds, gates


def decide_active2(s) -> tuple[bool, str]:
  """PURE first-hit-wins over the CES2 ladder (mirror of v1's decide_active shape, for the replay
  harness + tests). The live per-condition filtering/dwell lives in Ces2Core below."""
  conds, gates = ces2_conditions(s)
  for r in REASON_ORDER:
    if conds[r]:
      return True, r
  if gates["wantFaster"] or gates["leadOpen"]:
    return False, "wantFaster" if gates["wantFaster"] else "leadOpen"
  return False, "chill"


class Ces2Condition:
  """Per-condition debounced boolean (CEM-style): each trigger charges/decays independently, so a
  curve's decaying charge can no longer mask a stop's rise (the v1 aggregate-filter artifact).
  Same FirstOrderFilter semantics as v1's Condition, with a settable tau (stop runs 0.5 s)."""

  def __init__(self, tau: float = C.FILTER_TAU):
    self._tau = tau
    self.f = FirstOrderFilter(0.0, tau, DT_CTRL)
    self.active = False

  def update(self, raw: bool, dt: float = DT_CTRL) -> bool:
    if abs(dt - self.f.dt) > 1e-3:
      self.f.dt = dt
      self.f.update_alpha(self._tau)
    self.f.update(1.0 if raw else 0.0)
    self.active = self.f.x >= C.THRESHOLD
    return self.active

  def reset(self):
    self.f.x = 0.0
    self.active = False

  def force(self):
    self.f.x = 1.0
    self.active = True


class DivergenceCounter:
  """Counts divergence EDGES between the live core and the shadow core (a sustained disagreement
  is ONE divergence, not one per tick). None on either side = no comparison (no count, no edge)."""

  def __init__(self):
    self.count = 0
    self._diverged = False

  def update(self, live, shadow) -> int:
    diverged = live is not None and shadow is not None and live != shadow
    if diverged and not self._diverged:
      self.count += 1
    self._diverged = diverged
    return self.count


class Ces2Core:
  """CES2 state machine: per-condition filters + the v1 asymmetric dwells + the stop fast path +
  the CEM standstill hold. Same interface shape as ConditionalExperimentalSwitching so the
  controller can shadow/swap it. Owns its StopEvidence; NEVER mutates the caller's signals dict."""

  def __init__(self, exp_min_dwell: float = C.EXP_MIN_DWELL_S, chill_min_dwell: float = C.CHILL_MIN_DWELL_S):
    self._ev = StopEvidence()
    self._f = {name: Ces2Condition(STOP_FILTER_TAU if name == "stop" else C.FILTER_TAU)
               for name in REASON_ORDER}
    self._is_experimental = False
    self._dwell = 0.0
    self._status = "chill"
    self._exp_min = exp_min_dwell
    self._chill_min = chill_min_dwell
    self._nochill_stopped = False   # cesnochill2pnw (parity with CES v1 — see ces_pnw.py)

  def reset(self):
    self._ev.reset()
    for f in self._f.values():
      f.reset()
    self._is_experimental = False
    self._dwell = 0.0
    self._status = "chill"
    self._nochill_stopped = False   # cesnochill2pnw

  def mode(self) -> str:
    return "experimental" if self._is_experimental else "chill"

  def status(self) -> str:
    return self._status

  @property
  def urgency(self) -> float:
    return self._ev.urgency

  def update_decision(self, signals: dict, dt: float = DT_CTRL) -> str:
    """Public entry point: runs the CES2 core, then applies the SAME cesnochill2pnw hard latch as
    CES v1 (see ces_pnw.ConditionalExperimentalSwitching.update_decision) so Ces2Core carries the
    identical fix if it ever goes live (Ces2Core param). Behavior-neutral above the release
    threshold."""
    self._update_decision_core(signals, dt)
    v_now = float(signals.get("v_ego", 0.0))
    if self._nochill_stopped:
      if v_now > C.NOCHILL_RELEASE_V:
        self._nochill_stopped = False
    elif v_now < C.NOCHILL_STOP_V:
      self._nochill_stopped = True
    if self._nochill_stopped and not self._is_experimental:
      self._is_experimental = True
      self._status = "stopLatch"   # cesnochill2pnw telemetry tag
      self._dwell = 0.0
    return self.mode()

  def _update_decision_core(self, signals: dict, dt: float = DT_CTRL) -> str:
    v = signals["v_ego"]
    should_stop = bool(signals.get("model_should_stop"))
    lvl = self._ev.update(v, signals.get("mdl_end_x", 0.0), should_stop, dt)
    s2 = dict(signals)                    # never mutate the live core's dict (shared in shadow)
    s2["stop_urgency"] = lvl
    conds, _ = ces2_conditions(s2)

    # STOP FAST PATH — the v1 stop-intent fast path as a special case of the precedence principle,
    # WIDENED by the graded signal: HIGH urgency on the stop condition is treated exactly like the
    # binary shouldStop (DEC's emergency tier). Mirrors v1's trigger shape (shouldStop + ladder
    # wants Experimental via ANY condition — e.g. slowLead behind a stopping lead) so entry
    # latency never regresses vs v1. Entry side bypassed entirely (chill dwell + filter charge);
    # exit side keeps the full dwell (the forced filters must genuinely clear + decay).
    stops_on = bool(signals.get("toggles", {}).get("stops", True))
    if (not self._is_experimental and stops_on
        and ((should_stop and any(conds.values())) or (lvl >= LEVEL_HIGH and conds["stop"]))):
      self._is_experimental = True
      self._status = "stopIntent"
      self._dwell = 0.0
      for name in REASON_ORDER:
        if conds[name]:
          self._f[name].force()
      return self.mode()

    active = [name for name in REASON_ORDER if self._f[name].update(conds[name], dt)]
    self._dwell += dt

    # STANDSTILL HOLD (CEM): at v=0 while the model still says stopped, hold Experimental
    # regardless of filter decay/dwell — the principled anti-lurch anchor at a red light.
    standstill = bool(signals.get("standstill", v < STANDSTILL_V))
    if self._is_experimental and standstill and (lvl >= LEVEL_MED or should_stop):
      self._status = "standstill"
      return self.mode()

    if not self._is_experimental:
      if active and self._dwell >= self._chill_min:
        self._is_experimental = True
        self._status = active[0]          # highest-priority active condition
        self._dwell = 0.0
    else:
      if active:
        self._status = active[0]
      if not active and self._dwell >= self._exp_min:
        self._is_experimental = False
        self._status = "chill"
        self._dwell = 0.0
    return self.mode()
