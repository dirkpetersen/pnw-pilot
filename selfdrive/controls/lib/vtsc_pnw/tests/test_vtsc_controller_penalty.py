"""curveslow-lightning: VTSCController cap() applies the per-car curve-speed penalty.

With a Lightning CP the finalized curve-safe speed (msg['vCurveSafe']) is LOWER than with a Tesla CP
for the identical model curve; the Tesla path is byte-unchanged (penalty provably 0.0). Uses a fake
params + fake modelV2 so no cereal/device stack is needed."""
from openpilot.selfdrive.controls.lib.vtsc_pnw.vtsc_controller import VTSCController

MPH = 0.44704


class FakeCP:
  def __init__(self, fp="", brand="", op_long=True):
    self.carFingerprint = fp
    self.brand = brand
    self.openpilotLongitudinalControl = op_long


class FakeParams:
  """Minimal params: CESMode=Standard (2) so VTSC runs; VtscMapCurves off (vision-only)."""
  def __init__(self, d=None):
    self.d = d or {"CESMode": "2"}
  def get(self, k, return_default=False):
    return self.d.get(k)
  def get_bool(self, k):
    return bool(self.d.get(k, False))
  def put_nonblocking(self, k, v):
    pass


class _NS:
  pass


def _make_model(curvature, vx=27.0, n=20):
  """A modelV2 stub with constant path curvature -> a single binding curve ahead."""
  m = _NS()
  m.orientationRate = _NS()
  m.velocity = _NS()
  m.position = _NS()
  m.action = _NS()
  m.orientationRate.z = [curvature * vx] * n            # yaw rate = kappa * v
  m.orientationRate.t = [i * 0.25 for i in range(n)]    # 0 .. 4.75 s (within LOOKAHEAD_MAX_S)
  m.velocity.x = [vx] * n
  m.position.x = [max(vx * i * 0.25, 0.0) for i in range(n)]
  m.action.shouldStop = False
  return m


def _make_sm(curvature, vx=27.0):
  cc = _NS()
  cc.orientationNED = [0.0, 0.0, 0.0]
  return {"modelV2": _make_model(curvature, vx), "carControl": cc}


def _run_once(cp, v_cruise, v_ego, curvature):
  ctrl = VTSCController(cp, params=FakeParams())
  ctrl.mem_params = None                                # no overlay publish in the test env
  sm = _make_sm(curvature, vx=v_ego)
  cap = ctrl.cap(sm, v_cruise, v_ego)
  return ctrl, cap


def test_lightning_curve_cap_lower_than_tesla():
  # the MID-SPEED washout zone (hump peak, 45-62 mph targets): Tesla curve-safe speed ~24.6 m/s
  # (~55 mph) — inside the peak plateau, so the Lightning gets the full ~5 mph penalty.
  k = 2.5 / (24.6 * 24.6)                               # v_safe ~= 24.6 m/s (~55 mph)
  v_cruise = v_ego = 29.0                               # ~65 mph set

  ctrl_t, _ = _run_once(FakeCP("TESLA_MODEL_S_HW3", "tesla"), v_cruise, v_ego, k)
  ctrl_l, _ = _run_once(FakeCP("FORD_F_150_LIGHTNING_MK1", "ford"), v_cruise, v_ego, k)

  vcs_t = ctrl_t.msg["vCurveSafe"]
  vcs_l = ctrl_l.msg["vCurveSafe"]
  assert ctrl_t.enabled() and ctrl_l.enabled()          # VTSC actually ran on both
  assert 0.0 < vcs_t < v_cruise                          # Tesla curve binds
  # Tesla path unchanged: penalty is provably zero
  assert ctrl_t.veh.curve_speed_penalty_ms(vcs_t) == 0.0
  assert abs(vcs_t - 24.6) < 0.5                          # ~ the pure v_safe, no penalty applied
  # Lightning enters the SAME curve slower — ~the full ~5 mph (2.24 m/s) peak penalty.
  # descentcurve2pnw: the stub model's curve has orientationRate.z > 0 = a LEFT curve, so the
  # left-curve factor (1.15x) now applies on top of the hump; flat pitch -> no descent term.
  assert vcs_l < vcs_t - 0.5
  pen = ctrl_l.veh.curve_speed_penalty_ms(vcs_t, is_left=True)
  assert pen > 2.0 and abs((vcs_t - vcs_l) - pen) < 0.2   # peak penalty * left factor (~2.57 m/s)


def test_fast_gentle_sweeper_tapers():
  """Iteration-3 driver feedback: LONG FAST sweepers must NOT get the full cut — the penalty tapers.
  A gentle curve binding at ~34 m/s (~76 mph) gets only the ~1.5 mph taper penalty."""
  k = 2.5 / (34.0 * 34.0)                               # v_safe ~= 34 m/s (~76 mph)
  ctrl_l, _ = _run_once(FakeCP("FORD_F_150_LIGHTNING_MK1", "ford"), 38.0, 38.0, k)
  pen = ctrl_l.veh.curve_speed_penalty_ms(34.0)
  assert pen < 1.0                                       # ~0.67 m/s (1.5 mph) — carries speed again


def test_no_curve_no_penalty_applied():
  # straight road -> v_curve = inf -> penalty branch skipped (no crash, no cap)
  ctrl_l, cap = _run_once(FakeCP("FORD_F_150_LIGHTNING_MK1", "ford"), 27.0, 27.0, 0.0)
  assert cap == 27.0                                      # no curve -> cruise unchanged


# ---- descentcurve2pnw: descent guard + left factor + overspeed escalation ------------------------
from openpilot.selfdrive.controls.lib.vtsc_pnw import vtsc_constants as C
from openpilot.selfdrive.controls.lib.vtsc_pnw.vtsc_pnw import apex_turn_direction

LIGHTNING = FakeCP("FORD_F_150_LIGHTNING_MK1", "ford")
TESLA = FakeCP("TESLA_MODEL_S_HW3", "tesla")


def _make_model_signed(curvature, vx=27.0, n=20, apex_from=0):
  """Signed model: orientationRate.z keeps the curvature SIGN (>0 = LEFT, openpilot convention).
  apex_from > 0 -> straight until that index (puts the apex at a real distance ahead)."""
  m = _NS()
  m.orientationRate = _NS()
  m.velocity = _NS()
  m.position = _NS()
  m.action = _NS()
  m.orientationRate.z = [0.0 if i < apex_from else curvature * vx for i in range(n)]
  m.orientationRate.t = [i * 0.25 for i in range(n)]
  m.velocity.x = [vx] * n
  m.position.x = [max(vx * i * 0.25, 0.0) for i in range(n)]
  m.action.shouldStop = False
  return m


def _make_sm2(curvature, vx=27.0, pitch=0.0, apex_from=0):
  cc = _NS()
  cc.orientationNED = [0.0, pitch, 0.0]
  return {"modelV2": _make_model_signed(curvature, vx, apex_from=apex_from), "carControl": cc}


def _run2(cp, v_cruise, v_ego, curvature, pitch=0.0, apex_from=0, cycles=1):
  ctrl = VTSCController(cp, params=FakeParams())
  ctrl.mem_params = None
  sm = _make_sm2(curvature, vx=v_ego, pitch=pitch, apex_from=apex_from)
  cap = None
  for _ in range(cycles):
    cap = ctrl.cap(sm, v_cruise, v_ego)
  return ctrl, cap


def test_apex_turn_direction_signs():
  k = 2.5 / (24.0 * 24.0)
  assert apex_turn_direction(_make_model_signed(k)) == 1          # +z = LEFT
  assert apex_turn_direction(_make_model_signed(-k)) == -1        # -z = right
  assert apex_turn_direction(_make_model_signed(0.0)) == 0        # straight
  assert apex_turn_direction(_NS()) == 0                          # bad data -> unknown, never raises


def test_descent_scales_lightning_penalty_up():
  """A downhill (pitch -0.05 rad ~ 5% grade) must cut the Lightning's curve-safe speed FURTHER than
  the same curve on the flat (+40% penalty at defaults). Right curve so only the descent term acts."""
  k = 2.5 / (24.6 * 24.6)                                          # peak-zone curve (~55 mph safe)
  ctrl_flat, _ = _run2(LIGHTNING, 29.0, 29.0, -k, pitch=0.0)
  ctrl_down, _ = _run2(LIGHTNING, 29.0, 29.0, -k, pitch=-0.05)
  vcs_flat, vcs_down = ctrl_flat.msg["vCurveSafe"], ctrl_down.msg["vCurveSafe"]
  assert vcs_down < vcs_flat - 0.5                                 # meaningfully lower on the descent
  # ~1.4x the flat penalty: base ~5 mph -> ~7 mph => delta ~0.89 m/s
  base_pen = ctrl_flat.veh.curve_speed_penalty_ms(24.6)
  down_pen = ctrl_down.veh.curve_speed_penalty_ms(24.6, pitch_rad=-0.05)
  assert abs(down_pen - base_pen * 1.4) < 1e-6


def test_left_curve_penalized_more_than_right():
  k = 2.5 / (24.6 * 24.6)
  ctrl_l, _ = _run2(LIGHTNING, 29.0, 29.0, +k)                     # LEFT (z > 0)
  ctrl_r, _ = _run2(LIGHTNING, 29.0, 29.0, -k)                     # right
  assert ctrl_l.msg["vCurveSafe"] < ctrl_r.msg["vCurveSafe"]       # adverse crown + weak EPS


def test_tesla_byte_unchanged_with_pitch_and_direction():
  """The Tesla's cap is IDENTICAL with/without pitch, left/right — the whole descentcurve block is
  Lightning-gated (hard driver rule)."""
  k = 2.5 / (24.6 * 24.6)
  caps = []
  for kk, pitch in ((+k, 0.0), (+k, -0.08), (-k, -0.08)):
    ctrl, cap = _run2(TESLA, 29.0, 29.0, kk, pitch=pitch)
    caps.append((round(cap, 6), round(ctrl.msg["vCurveSafe"], 6)))
    assert ctrl.veh.curve_speed_penalty_ms(24.6, pitch_rad=pitch, is_left=True) == 0.0
  assert caps[0] == caps[1] == caps[2]


def test_overspeed_escalation_unlocks_sharp_ceiling():
  """Overspeed into a binding curve (descent signature: regen budget eaten by gravity) -> the
  rate-limit ceiling escalates to the EXISTING SHARP_A_DECEL_MAX. Two cycles: _applied exists from
  the first."""
  k = 2.5 / (20.0 * 20.0)                                          # tight curve, v_safe ~20 m/s
  ctrl, _ = _run2(LIGHTNING, 29.0, 31.0, -k, apex_from=10, cycles=2)   # v_ego 2 m/s over the set
  assert ctrl._a_decel_max == C.SHARP_A_DECEL_MAX


def test_no_escalation_when_not_overspeed():
  k = 2.5 / (20.0 * 20.0)
  ctrl, _ = _run2(LIGHTNING, 29.0, 25.0, -k, apex_from=10, cycles=2)   # under the cap
  assert ctrl._a_decel_max == min(ctrl.tune['A_DECEL_MAX'], C.REGEN_A_DECEL)


def test_no_escalation_when_no_binding_curve():
  ctrl, _ = _run2(LIGHTNING, 29.0, 32.0, 0.0, cycles=2)            # overspeed but straight road
  assert ctrl._a_decel_max == min(ctrl.tune['A_DECEL_MAX'], C.REGEN_A_DECEL)


def test_no_escalation_on_tesla_same_scenario():
  k = 2.5 / (20.0 * 20.0)
  ctrl, _ = _run2(TESLA, 29.0, 31.0, -k, apex_from=10, cycles=2)
  assert ctrl._a_decel_max == min(ctrl.tune['A_DECEL_MAX'], C.REGEN_A_DECEL)


# ---- rain2pnw: wet-weather margin reaches the curve pipeline + lowers the freeway floor -----------
class _FakeMem:
  """mem params supplying the freeway floor inputs (MapSpeedLimit + RoadContext)."""
  def __init__(self, limit_ms, ctx):
    self.d = {"MapSpeedLimit": limit_ms, "RoadContext": ctx}
  def get(self, k, return_default=False):
    return self.d.get(k)


def _run_rain(cp, tier, v_cruise, v_ego, curvature):
  ctrl = VTSCController(cp, params=FakeParams({"CESMode": "2", "RainMode": str(tier)}))
  ctrl.mem_params = None
  cap = ctrl.cap(_make_sm(curvature, vx=v_ego), v_cruise, v_ego)
  return ctrl, cap


def test_rain_lowers_curve_safe_speed_both_cars():
  # rain subtracts its margin from the finalized curve-safe speed (vCurveSafe) on BOTH cars.
  k = 2.5 / (24.6 * 24.6)                                 # binds at ~24.6 m/s (~55 mph)
  for fp, brand in (("TESLA_MODEL_S_HW3", "tesla"), ("FORD_F_150_LIGHTNING_MK1", "ford")):
    dry, _ = _run_rain(FakeCP(fp, brand), 0, 29.0, 29.0, k)
    wet, _ = _run_rain(FakeCP(fp, brand), 2, 29.0, 29.0, k)
    drop = dry.msg["vCurveSafe"] - wet.msg["vCurveSafe"]
    assert abs(drop - 5.0 * MPH) < 1e-2                   # exactly the 5 mph heavy margin, every car


def test_rain_lowers_freeway_floor_below_limit():
  # Gemini 2026-07-12: the freeway speed-limit floor must ALSO drop by the rain margin, or a curve at
  # the limit snaps back up and the setting is erased. Pre-engage a low cap (simulating mid-curve
  # braking) so the floor is the binding term, then confirm the floored result honours rain.
  def floored_cap(tier):
    ctrl = VTSCController(FakeCP("TESLA_MODEL_S_HW3", "tesla", op_long=True),
                          params=FakeParams({"CESMode": "2", "RainMode": str(tier)}))
    ctrl.mem_params = _FakeMem(29.06, "freeway")          # 65 mph freeway
    ctrl._state = "hold"
    ctrl._applied = 20.0                                  # already braked well below the floor
    return ctrl.cap(_make_sm(0.0, vx=29.06), 32.0, 29.06)  # straight road -> floor is the only bound
  dry = floored_cap(0)
  wet = floored_cap(2)
  assert abs(dry - 29.06) < 1e-6                          # dry Tesla floor = the posted limit
  assert abs((dry - wet) - 5.0 * MPH) < 1e-6             # heavy rain lowers the floor by 5 mph
  assert wet < 29.06                                     # ... so the car may now trim below the limit
