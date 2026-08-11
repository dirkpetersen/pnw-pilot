"""
VTSC Phase 2 — live controller for the longitudinal planner.

`VTSCController.cap(sm, v_cruise, v_ego)` returns a possibly-lowered cruise speed (m/s) so the planner
MPC slows for an upcoming curve. Rides the CES master selector (`CESMode`: 0=Off, 1=Light->GENTLE
tune, 2=Standard->DEFAULT tune; default Off) + openpilotLongitudinalControl; returns v_cruise
unchanged when disabled -> behavior-neutral. NEVER raises speed above v_cruise.

Apex state machine (driver feedback, drive #4):
  - the instant a curve is detected -> an immediate >=1 mph cut (CONFIDENCE_CUT) so the driver feels VTSC
    engage right away. ces-i90-2pnw: this now fires for EVERY real bend (apex curvature >= CUE_MIN_CURVATURE,
    ~5 deg / R~2300 m), not only curves that need real slowing -> a guaranteed small cue at the start of
    each curve; gentle bends get just the >=1 mph dip (held through, then released), sharper curves brake more;
  - BRAKE while the apex is clearly ahead (tta > HOLD_TTA_S): slow to reach curve-safe speed BEFORE the
    apex (firmer if needed — pre-apex braking is flexible);
  - HOLD when close/uncertain (APEX_TTA_S < tta <= HOLD_TTA_S): maintain, NEVER reduce further;
  - RELEASE at the apex (tta <= APEX_TTA_S, or the path straightens): accelerate back to cruise.
We never reduce speed at or after the apex.

Logging: publishes the decision as a `vtscState` cereal message every cycle (recorded in qlog/rlog so
drives are analyzable) AND a `VTSCStatus` JSON to /dev/shm/params for the live on-screen overlay.

Runs inside plannerd (20 Hz / DT_MDL). Uses a MEASURED loop dt for the rate-limiter (don't assume a
fixed rate — that was the CES 5x bug). Pure curve/curvature math lives in vtsc_pnw.py.
"""
import json
import time

from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.vtsc_pnw import vtsc_constants as C
from openpilot.selfdrive.controls.lib.vtsc_pnw.vtsc_pnw import (
  model_curve_state, brake_cap_for_apex, apply_limits,
  most_binding_map_curve, twisty_section_cap, required_decel,   # sharpcurve2pnw
  apex_turn_direction)                                          # descentcurve2pnw
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw_constants as CES
from openpilot.selfdrive.controls.lib.pnw_vehicle import PnwVehicle   # curveslow-lightning


class VTSCController:
  def __init__(self, CP, params=None):
    import platform
    from openpilot.common.params import Params
    self.CP = CP
    self.params = params or Params()
    try:   # in-memory store for the UI overlay (same channel CES uses)
      self.mem_params = Params("/dev/shm/params") if platform.system() != "Darwin" else self.params
    except Exception:
      self.mem_params = None
    self._long_ok = bool(getattr(CP, 'openpilotLongitudinalControl', False))
    # curveslow-lightning: per-car curve-speed penalty (Lightning enters curves slower; Tesla -> 0.0)
    self.veh = PnwVehicle(CP)
    # light-ces-gentle: the tune is now USER-SELECTED via CESMode (1=Light -> GENTLE_PROFILE, soft
    # decel + slow recovery so a series of curves doesn't sawtooth; 2=Standard -> DEFAULT_PROFILE),
    # NOT gated on carFingerprint. _read_enabled() re-selects the tune when the mode changes.
    self._mode = CES.CES_MODE_OFF
    self.tune = dict(C.DEFAULT_PROFILE)
    self._enabled = False
    # ces-i90-2pnw (MTSC): optional pfeiferj map curve fold, gated by VtscMapCurves (default OFF)
    self._map_curves = False
    self._map_targets: list = []
    self._speed_limit = 0.0    # m/s posted limit (mapd bridge); the VTSC cap is FLOORED here (driver
                               # rule 2026-07-01; curvefloor2pnw 2026-08-11 generalized this off
                               # freeways too — see the floor block in cap() for the full rationale)
    # curvefloor2pnw: debounced steering-saturation evidence gate (ces_pnw_constants.py) — the ONLY
    # thing allowed to relax the speed-limit floor below spdLim. Starts at "no evidence" (floor
    # enforced) until SteerLimitStatus has actually been read at least EVIDENCE_ENTER_TICKS times.
    self._sl_evid_count = 0
    self._sl_evidence = False
    self._cur_lat = self._cur_lon = None
    self._state = "idle"      # idle | brake | hold | release
    self._applied = None      # current applied cap (m/s); None = none
    # sharpcurve2pnw: per-cycle effective decels. Normal commanded decel is capped to EV regen authority
    # (REGEN_A_DECEL ~0.2 g) -> the slowdown is coast/regen, no friction braking. A SHARP curve that regen
    # alone can't make before its entrance raises the rate-limit ceiling to SHARP_A_DECEL_MAX (last resort).
    self._a_decel = C.A_DECEL       # envelope decel (plan shape)
    self._a_decel_max = C.REGEN_A_DECEL  # rate-limit ceiling (regen-only by default)
    self._below = 0           # consecutive cycles a far curve is present (debounce into brake)
    self._clear = 0           # consecutive cycles no curve (debounce release -> idle)
    self._last_t = None       # monotonic stamp of last cap() call (real dt)
    self._last_read = -1e9    # monotonic stamp of last param read
    self._tele_last = 0.0     # monotonic stamp of last overlay publish
    # vtsctele2pnw: penalty components actually applied this cycle, for the VTSCStatus overlay feed
    # (-> ces_events tick). Display/logging only — never read back into control.
    self._tele_pen = 0.0      # Lightning curve penalty applied (m/s; 0.0 = none / non-Lightning)
    self._tele_pitch = None   # road pitch used (rad, carControl.orientationNED[1]; None = no reading)
    self._tele_dir = ""       # apex turn direction: "L" / "R" / "" (no real bend)
    self._engaged = False     # for engage/clear logging
    # last decision, for the logged vtscState message (read by the planner)
    self.msg = dict(enabled=False, active=False, state="idle", vCruise=0.0, vTarget=0.0,
                    vEgo=0.0, apexDist=-1.0, apexCurvature=0.0, vCurveSafe=0.0, timeToApex=-1.0)

  def enabled(self) -> bool:
    return self._enabled

  def _read_enabled(self, now: float) -> None:
    if now - self._last_read >= 1.0:                       # ~1 Hz
      self._last_read = now
      try:
        # VTSC rides the CES master selector (CESMode): non-Off -> VTSC on. The mode also picks the
        # tune: Light -> GENTLE_PROFILE (anti-sawtooth), Standard -> DEFAULT_PROFILE. On ANY car.
        self._mode = CES.read_ces_mode(self.params)
        # rain2pnw: push the live wet-weather tier into the capability view (applies on EVERY car).
        # Isolated try — a RainMode read hiccup must NEVER fall through to the outer except that
        # disables VTSC (curve control is not rain's dependant); worst case rain stays at its last tier.
        try:
          self.veh.set_rain_tier(self.params.get("RainMode", return_default=True))
        except Exception:
          pass
        self._enabled = self._long_ok and CES.ces_enabled(self._mode)
        self.tune = dict(C.GENTLE_PROFILE) if CES.ces_is_gentle(self._mode) else dict(C.DEFAULT_PROFILE)
        # ces-i90-2pnw (MTSC): fold map curves only when VTSC is enabled AND opted-in via the param
        self._map_curves = self._enabled and bool(self.params.get_bool("VtscMapCurves"))
        if self._map_curves:
          self._read_map()
        else:
          self._map_targets = []
        # speed-limit floor input (driver rule 2026-07-01): posted limit from the mapd bridge mem
        # param (same source as the map curves). Applied in cap(); 0 -> no floor (V_MIN still applies).
        # curvefloor2pnw (2026-08-11): the floor used to be gated to RoadContext=='freeway' only —
        # generalized to every road now that the steering-evidence gate below can distinguish a
        # genuinely-too-tight curve from an over-slow artifact without relying on a road-class proxy
        # (the field event that motivated this was a 25 mph SURFACE curve, which the old freeway-only
        # gate would never have floored at all).
        try:
          sl = self.mem_params.get("MapSpeedLimit", return_default=True) if self.mem_params else None
          self._speed_limit = float(sl) if sl is not None else 0.0
        except Exception:
          self._speed_limit = 0.0
        # curvefloor2pnw: steering-saturation evidence (see ces_pnw_constants.py) — same SteerLimitStatus
        # mem-param controlsd already publishes at ~5 Hz for ces_pnw's identical read; same ~1 Hz cadence
        # as the other _read_enabled() reads. Any failure -> raw tick False (safe default, floor stays
        # enforced) rather than touching the debounce state with a poisoned reading.
        try:
          sl_status = self.mem_params.get("SteerLimitStatus", return_default=True) if self.mem_params else None
          if isinstance(sl_status, (bytes, str)):
            sl_status = json.loads(sl_status)
          if not isinstance(sl_status, dict):
            sl_status = {}
          raw = CES.steering_saturation_tick(sl_status.get("sat", False), sl_status.get("curvLim", False),
                                             sl_status.get("kErr"))
        except Exception:
          raw = False
        self._sl_evid_count, self._sl_evidence = CES.evidence_gate_step(
          self._sl_evid_count, self._sl_evidence, raw)
      except Exception:
        self._enabled = False
        self._map_curves = False

  def _reset(self):
    self._state = "idle"
    self._applied = None
    self._below = 0
    self._clear = 0

  def _read_map(self):
    """ces-i90-2pnw (MTSC): refresh the pfeiferj map curve inputs (MapTargetVelocities + GPS) from the
    /dev/shm mem params — the SAME source CES reads. Any failure -> no map curve (vision still works)."""
    if self.mem_params is None:
      self._map_targets = []
      return
    try:
      self._map_targets = self.mem_params.get("MapTargetVelocities", return_default=True) or []
    except Exception:
      self._map_targets = []
    try:
      pos = self.mem_params.get("LastGPSPosition", return_default=True)
      if isinstance(pos, (bytes, str)):
        pos = json.loads(pos)
      self._cur_lat = float(pos["latitude"])
      self._cur_lon = float(pos["longitude"])
    except Exception:
      self._cur_lat = self._cur_lon = None

  def _fold_map_curve(self, k_apex, d_apex, v_curve, v_cruise_set, v_ego, horizon_m):
    """ces-i90-2pnw (MTSC) + sharpcurve2pnw: fold the upcoming MAP curve into the curve picture, using
    whichever of vision / map is MORE BINDING (needs the lower speed NOW via the decel envelope). Now
    scans the FULL available horizon (mapd's ~500 m) and picks the most-binding curve by envelope, not
    nearest — so a blind sharp curve is seen ~6 s earlier than the old v_ego*12 s reach. Returns
    (k, d, v, is_sharp_map): is_sharp_map flags a genuinely sharp MAP curve (for the last-resort firmer
    rate-limit). The chosen curve feeds the SAME decel-limited + V_MIN-floored state machine.

    All thresholds/clamps use the driver's true SET speed (v_cruise_set), NOT a twisty-trimmed working
    cruise — otherwise a lowered base would dismiss a real sharp curve as 'trivial' (it must still brake).
    The MTSC scale (driver: mapd targets ran ~10 mph slow) + clamp are applied INSIDE the selection so the
    chosen curve matches the value used here."""
    try:
      mv, md, sharp = most_binding_map_curve(self._map_targets, self._cur_lat, self._cur_lon, v_ego,
                                             horizon_m, self.tune['A_DECEL'], C.APEX_FINISH_S,
                                             C.SHARP_CURVE_V, C.MAP_SPEED_SCALE, v_cruise_set)
    except Exception:
      return k_apex, d_apex, v_curve, False
    # only a real map curve meaningfully below the SET speed counts (ignore GPS noise / trivial targets)
    if not (0.0 < mv < v_cruise_set - C.MAP_MIN_SLOWDOWN) or md <= 0.0:
      return k_apex, d_apex, v_curve, False
    rsn_vis = brake_cap_for_apex(v_curve, d_apex, v_ego, self.tune['A_DECEL']) if d_apex >= 0.0 else float('inf')
    rsn_map = brake_cap_for_apex(mv, md, v_ego, self.tune['A_DECEL'])
    if rsn_map < rsn_vis:                       # map curve is the more binding -> use it
      k_map = (self.tune['A_LAT_TARGET'] / (mv * mv)) if mv > 0.0 else 0.0   # equiv curvature, logging only
      return k_map, md, mv, sharp
    return k_apex, d_apex, v_curve, False

  def cap(self, sm, v_cruise: float, v_ego: float) -> float:
    """Return the VTSC-capped cruise speed (m/s). v_cruise when disabled / no curve. Safe: <= v_cruise."""
    now = time.monotonic()
    self._read_enabled(now)
    dt = min(max((now - self._last_t) if self._last_t is not None else DT_MDL, 1e-3), 0.5)
    self._last_t = now

    if not self._enabled:
      self._reset()
      if self._engaged:
        cloudlog.info("VTSC disabled -> no cap")
        self._engaged = False
      return self._finish(v_cruise, v_cruise, v_ego, 0.0, -1.0, float('inf'), now)

    try:
      model = sm['modelV2']
    except Exception:
      return self._finish(v_cruise, v_cruise, v_ego, 0.0, -1.0, float('inf'), now)

    # sharpcurve2pnw: keep the driver's SET speed for display; the state machine works against a possibly
    # twisty-trimmed working cruise. Read road pitch (carControl.orientationNED[1], rad; <0 = downhill).
    v_cruise_set = v_cruise
    pitch = None
    try:
      ned = sm['carControl'].orientationNED
      if len(ned) == 3:
        pitch = float(ned[1])
    except Exception:
      pitch = None
    # vtsctele2pnw: reset per-cycle telemetry; pitch is recorded whether or not a curve binds
    self._tele_pen = 0.0
    self._tele_pitch = pitch
    self._tele_dir = ""

    k_apex, d_apex, v_curve = model_curve_state(model, v_cruise, self.tune['A_LAT_TARGET'])
    sharp_map = False
    if self._map_curves:                        # ces-i90-2pnw (MTSC) + sharpcurve2pnw
      horizon_m = C.MAP_SOURCE_HORIZON_M        # scan the FULL ~500 m mapd publishes (envelope gates binding)
      # most-binding upcoming map curve over the FULL horizon (earlier detection of blind sharp curves).
      # Thresholds use the true SET speed, so a real curve isn't dismissed by the twisty trim below.
      k_apex, d_apex, v_curve, sharp_map = self._fold_map_curve(k_apex, d_apex, v_curve, v_cruise_set, v_ego, horizon_m)
      # twisty-section base trim (descent-only): hold a LOWER base cruise through a winding DOWNHILL so we
      # don't re-accelerate to full set between blind curves. Only ever lowers the working cruise.
      try:
        v_cruise = twisty_section_cap(self._map_targets, self._cur_lat, self._cur_lon,
                                      v_cruise, v_ego, horizon_m, pitch)
      except Exception:
        pass

    # curveslow-lightning: per-car curve-speed penalty. v_curve is now the finalized curve-safe speed
    # (vision, or the more-binding map fold). On the Lightning, lower it (weaker EPS -> enter slower);
    # Tesla/other cars -> penalty 0.0 -> byte-unchanged. Only when a real curve BINDS (finite v_curve);
    # the no-curve inf case is left alone. Floored at V_MIN, and it only ever lowers -> stays <= cruise.
    # descentcurve2pnw: the penalty now also sees road pitch (descent guard — downhill scales it up)
    # and the apex turn direction (left-curve factor). Both computed ONLY on the Lightning; the Tesla
    # never runs any of this block (behavior + cycle cost identical).
    # vtsctele2pnw: apex turn direction — telemetry on every car, and the Lightning left-factor
    # input below. Only computed when a curve candidate exists (zero extra work on straights;
    # a pure walk of the model-path points when it runs). Superset of the old Lightning-only gate,
    # so the penalty path sees the identical is_left it always did.
    turn_dir = 0
    if d_apex >= 0.0 or v_curve != float('inf'):
      try:
        turn_dir = apex_turn_direction(model)
      except Exception:
        turn_dir = 0
      self._tele_dir = "L" if turn_dir > 0 else ("R" if turn_dir < 0 else "")
    if v_curve != float('inf') and self.veh.lightning_curve_slow:
      is_left = turn_dir > 0
      penalty = self.veh.curve_speed_penalty_ms(v_curve, pitch_rad=pitch, is_left=is_left)
      self._tele_pen = float(penalty)
      if penalty > 0.0:
        v_curve = max(v_curve - penalty, C.V_MIN)

    # rain2pnw: driver-selected wet-weather curve margin. UNLIKE the Lightning EPS penalty above, rain
    # cuts grip on EVERY car, so this applies to BOTH the Tesla and the Lightning (same reduction —
    # driver directive 2026-07-12). Additive on top of each car's base curve-safe speed; only when a
    # real curve binds (finite v_curve); reduce-only, floored at V_MIN so it can never invert.
    if v_curve != float('inf'):
      rain_pen = self.veh.rain_penalty_ms()
      if rain_pen > 0.0:
        v_curve = max(v_curve - rain_pen, C.V_MIN)

    # sharpcurve2pnw: NORMAL commanded decel = EV regen authority -> coast/regen, no friction braking
    # (driver: "on the freeway braking should almost never be required"). A genuinely SHARP map curve that
    # regen alone can't reach before its ENTRANCE (apex minus the finish lead) raises the ceiling to
    # SHARP_A_DECEL_MAX (last resort). Measured to the entrance, not the apex, since that's where we must
    # already be at curve-safe speed.
    self._a_decel = self.tune['A_DECEL']
    self._a_decel_max = min(self.tune['A_DECEL_MAX'], C.REGEN_A_DECEL)
    d_entrance = max(d_apex - v_ego * C.APEX_FINISH_S, 1.0)
    if sharp_map and d_apex > 0.0 and required_decel(v_ego, v_curve, d_entrance) > C.REGEN_A_DECEL:
      self._a_decel_max = max(self._a_decel_max, C.SHARP_A_DECEL_MAX)
    # descentcurve2pnw (Lightning only): overspeed-into-curve escalation. On a descent gravity eats
    # the regen budget, so the truck can arrive ABOVE the applied cap with the curve entrance closing
    # (the 2026-07-11 18:12 washout: 77 mph vs cap 71, "accelerating downhill into the curve").
    # When BOTH (a) v_ego exceeds the applied cap by the overspeed margin AND (b) the binding curve
    # entrance needs more than regen decel, reuse the EXISTING sharp-curve friction ceiling
    # (SHARP_A_DECEL_MAX — no new decel ceiling). Tesla: whole clause gated off (byte-unchanged).
    if (self.veh.lightning_curve_slow and d_apex > 0.0 and v_curve != float('inf')
        and self._applied is not None
        and v_ego > min(v_cruise_set, self._applied) + self.veh.overspeed_margin_ms
        and required_decel(v_ego, v_curve, d_entrance) > C.REGEN_A_DECEL):
      self._a_decel_max = max(self._a_decel_max, C.SHARP_A_DECEL_MAX)
    # ces-i90-2pnw: a curve "counts" if it BINDS (curve-safe speed below cruise -> real braking) OR is a
    # mild bend past CUE_MIN_CURVATURE (~5 deg / R~2300 m). The mild-bend case never needs real slowing, so
    # the state machine below only applies the CONFIDENCE_CUT (>=1 mph) engage dip then releases -> a
    # guaranteed small "I see the curve" cue on EVERY real bend, automatic, no toggle.
    has_curve = d_apex >= 0.0 and (v_curve < v_cruise - 0.1 or k_apex >= C.CUE_MIN_CURVATURE)
    tta = (d_apex / max(v_ego, 1.0)) if has_curve else float('inf')
    # drive #5: have we actually slowed to ~curve-safe speed? gates HOLD/RELEASE below so VTSC keeps
    # braking while still materially too fast, and never accelerates out of a curve before reaching safe.
    at_safe = (not has_curve) or v_curve <= 0.0 or v_ego <= v_curve * (1.0 + C.RELEASE_SPEED_MARGIN)

    if self._applied is None:
      self._applied = v_cruise

    # ---- state machine: brake before apex, hold when unsure, release+accelerate at apex ----
    if self._state == "idle":
      target = v_cruise
      self._below = self._below + 1 if (has_curve and tta > C.HOLD_TTA_S) else 0
      if self._below >= C.CURVE_MIN_POINTS:
        self._state = "brake"
        self._applied = min(self._applied, v_cruise - C.CONFIDENCE_CUT)   # instant >=1mph cut on detect

    if self._state == "brake":
      if not has_curve:
        self._state = "release"
      elif tta <= C.APEX_TTA_S:
        # at the apex: never brake here. accelerate out only if we've slowed enough; else HOLD (no accel).
        self._state = "release" if at_safe else "hold"
      elif tta <= C.HOLD_TTA_S and at_safe:
        self._state = "hold"                       # close AND already at safe speed -> maintain
      else:
        # apex still ahead, OR still too fast inside the hold window -> keep reducing toward curve-safe
        cap = brake_cap_for_apex(v_curve, d_apex, v_ego, self._a_decel)
        # never above cruise-CONFIDENCE_CUT (keep the engage cut), floored at V_MIN
        target = max(min(cap, v_cruise - C.CONFIDENCE_CUT), C.V_MIN)

    if self._state == "hold":
      target = self._applied                       # freeze: never reduce further, never accelerate yet
      if not has_curve or (tta <= C.APEX_TTA_S and at_safe):
        self._state = "release"                    # only accelerate out once we've actually slowed

    if self._state == "release":
      target = v_cruise                            # accelerate back to cruise set speed
      self._clear = self._clear + 1 if not has_curve else 0
      # a genuinely NEW curve far ahead re-arms braking
      self._below = self._below + 1 if (has_curve and tta > C.HOLD_TTA_S) else 0
      if self._below >= C.CURVE_MIN_POINTS:
        self._state = "brake"
        self._clear = 0
        self._applied = min(self._applied, v_cruise - C.CONFIDENCE_CUT)
      elif self._clear >= C.CLEAR_CYCLES:
        self._state = "idle"
        self._below = 0

    # safety rate-limit (bounded decel down to A_DECEL_MAX, ease up at A_RELAX). HOLD target==applied -> no move.
    self._applied = apply_limits(self._applied, target, v_cruise, dt, self._a_decel_max, self.tune['A_RELAX'])
    capped = min(v_cruise, self._applied)

    # SPEED-LIMIT FLOOR (driver rule 2026-07-01, generalized by curvefloor2pnw 2026-08-11): never trim
    # below the posted limit — only from the set speed DOWN TOWARD the limit. This bounds the downside
    # (worst case = the limit, never a deep over-slow), which is what makes the lower A_LAT_TARGET safe.
    # No limit data -> no floor (V_MIN still applies).
    # curvefloor2pnw: this floor USED TO be gated to RoadContext=='freeway' only — "a curve genuinely
    # too tight for the limit is then the driver's to handle" off-highway. The field event that motivated
    # this branch (2026-08-11: 50->29->~15 mph on a 25 mph SURFACE curve, well below the ~31 mph computed
    # target) is exactly the case that old gate could never catch. Replaced with the more precise
    # `_sl_evidence` gate below: sustained steering-saturation is a direct physical signature of "this
    # curve genuinely can't be held at the limit," so the road-class heuristic is no longer needed to
    # avoid over-constraining genuinely tight backroad curves.
    if self._speed_limit > 0.0:
      # curveslow-lightning floor fix (2026-07-11 evening passes): the floor was erasing the
      # Lightning penalty on posted-limit sweepers — caps pinned at exactly the 65 mph limit
      # (29.1 m/s) through all 13 takeovers while the penalized v_curve sat ~4.5 m/s lower.
      # The Lightning may trim BELOW the posted limit by its own penalty (weaker EPS needs it);
      # the Tesla's penalty is 0.0 so its DRY floor stays exactly at the posted limit (driver rule
      # 2026-07-01 unchanged for the Tesla).
      # rain2pnw (Gemini review 2026-07-12): the rain margin must ALSO lower this floor, or a curve
      # posted at/near the limit snaps right back up to the limit and the driver's wet-weather
      # setting is silently erased on freeways (worst on the Tesla, whose base penalty is 0). rain
      # is car-agnostic, so both cars may trim below the limit by the selected rain margin (only
      # when a curve is actually pulling the cap down — straight-line cruise stays at the limit).
      floor_v = max(self._speed_limit - self.veh.curve_speed_penalty_ms(self._speed_limit)
                    - self.veh.rain_penalty_ms(), C.V_MIN)
      # curvefloor2pnw: sustained steering-saturation evidence (SteerLimitStatus, read in
      # _read_enabled()) is the ONLY thing allowed to relax this floor below the penalty/rain-adjusted
      # value above — see ces_pnw_constants.speed_limit_curve_floor for the exact rule. No evidence ->
      # byte-identical to the pre-curvefloor2pnw floor (now just evaluated on every road, not only
      # freeways).
      if not self._sl_evidence:
        capped = min(v_cruise, max(capped, floor_v))

    engaged = capped < v_cruise_set - 0.5
    if engaged != self._engaged:
      cloudlog.info("VTSC %s [%s]: cap=%.1f cruise=%.1f vEgo=%.1f apex=%.0fm tta=%.1fs",
                    "ENGAGE" if engaged else "clear", self._state, capped, v_cruise_set, v_ego, d_apex, tta)
      self._engaged = engaged
    return self._finish(capped, v_cruise_set, v_ego, k_apex, d_apex, v_curve, now)

  def _finish(self, capped, v_cruise, v_ego, k_apex, d_apex, v_curve, now):
    active = capped < v_cruise - 0.5
    vcs = 0.0 if v_curve == float('inf') else float(v_curve)
    tta = (d_apex / v_ego) if (d_apex >= 0.0 and v_ego > 0.1) else -1.0
    self.msg = dict(enabled=bool(self._enabled), active=bool(active), state=self._state,
                    vCruise=float(v_cruise), vTarget=float(capped), vEgo=float(v_ego),
                    apexDist=float(d_apex), apexCurvature=float(k_apex), vCurveSafe=vcs,
                    timeToApex=float(tta))
    self._publish_overlay(now)
    return capped

  def _publish_overlay(self, now: float) -> None:
    """Publish a tiny VTSCStatus snapshot to /dev/shm/params (~5 Hz) for the on-screen overlay."""
    if self.mem_params is None or now - self._tele_last < 0.2:
      return
    self._tele_last = now
    try:
      self.mem_params.put_nonblocking("VTSCStatus", {
        "enabled": self.msg["enabled"], "engaged": self.msg["active"], "state": self.msg["state"],
        "cap": round(self.msg["vTarget"], 1), "vCruise": round(self.msg["vCruise"], 1),
        # vtsctele2pnw: penalty components actually applied (ces_events picks these up per tick)
        "pen": round(float(self._tele_pen), 2),
        "pitch": round(float(self._tele_pitch), 4) if self._tele_pitch is not None else None,
        "dir": self._tele_dir,
        # curvefloor2pnw: whether the speed-limit floor was RELAXED this tick (steering-saturation
        # evidence) — lets ces_events distinguish "floored at the limit" from "evidence let it go lower."
        "evid": bool(self._sl_evidence),
      })
    except Exception:
      pass
