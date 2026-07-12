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
  # Lightning enters the SAME curve slower — ~the full ~5 mph (2.24 m/s) peak penalty
  assert vcs_l < vcs_t - 0.5
  pen = ctrl_l.veh.curve_speed_penalty_ms(vcs_t)
  assert pen > 2.0 and abs((vcs_t - vcs_l) - pen) < 0.2   # peak penalty at this speed (~2.24 m/s)


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
