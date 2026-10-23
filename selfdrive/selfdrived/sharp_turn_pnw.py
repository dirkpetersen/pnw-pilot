"""handsready2pnw: "Keep hands ready: sharp turn" heads-up while openpilot steers a sharp, low-speed turn with weak lane lines.

WHY. drives/2026-10-21/seattle-left-curve-overshoot: on the F-150 Lightning in angle mode the PSCM unwinds 0.8-1.4 s behind
the command after a sharp low-speed turn. At night, with the model seeing almost no lane lines (laneLineProbs <= 0.07), the
truck went about 1.9 m past the curve exit into the other lane and the driver had to yank it back. We cannot fix the PSCM, so
the driver gets a heads-up in exactly that situation. Display only: no control path, no disengage, no state-machine effect.

FIRES when ALL hold for >= PERSIST_S: steering is active on a car with the capability (the caller passes `armed`), the plan's
largest |curvature| now or within the next PLAN_HORIZON_S is >= K_FIRE, the weaker ego lane line is below the lane-centering
confidence threshold, and V_MIN < vEgo < V_MAX. Not again within REFIRE_S of the last fire.
HOLDS for at least HOLD_S, then until the turn has ended: both the plan and the MEASURED curvature are below K_END. The
measured term keeps it up through the slow PSCM unwind, which is the dangerous part (the plan was straight 1.5 s before the
truck was). Clears at once when steering stops being active (the driver has taken over).

Pure: no params, no sockets, no clock. selfdrived feeds it and owns the event and the log.
"""
import math

from openpilot.selfdrive.controls.lib.lane_centering import DEFAULT_TUNING

K_FIRE = 0.02           # 1/m, about R 50 m
K_END = 0.01            # 1/m, the turn has ended below this
PLAN_HORIZON_S = 2.0    # look this far ahead on the model plan
LANE_PROB_LOW = float(DEFAULT_TUNING["min_lane_prob"])  # the lane-centering "lowconf" probability gate (0.6)
V_MIN = 3.0             # m/s
V_MAX = 15.0            # m/s
PERSIST_S = 0.3
HOLD_S = 2.0
REFIRE_S = 10.0


def plan_curvature(model_v2) -> float:
  """Largest |curvature| the model plans now or within PLAN_HORIZON_S: action.desiredCurvature and yaw rate / speed on
  the plan. Raises on a malformed message (the caller logs it; an error is not "straight road")."""
  k = abs(float(model_v2.action.desiredCurvature))
  rate, vel = model_v2.orientationRate, model_v2.velocity
  for t, wz, vx in zip(rate.t, rate.z, vel.x, strict=False):
    if t > PLAN_HORIZON_S:
      break
    k = max(k, abs(float(wz)) / max(float(vx), 1.0))
  if not math.isfinite(k):
    raise ValueError(f"non-finite plan curvature {k}")
  return k


def ego_lane_prob(model_v2) -> float:
  """The weaker of the two ego lane lines (laneLineProbs[1], [2]). Raises when they are missing or not finite."""
  probs = model_v2.laneLineProbs
  p = min(float(probs[1]), float(probs[2]))
  if not math.isfinite(p):
    raise ValueError(f"non-finite lane line probability {p}")
  return p


class SharpTurnHands:
  def __init__(self):
    self.active = False
    self._cond_t: float | None = None
    self._fire_t = -math.inf

  def update(self, t: float, armed: bool, v_ego: float, k_plan: float, k_meas: float, lane_prob: float) -> bool:
    """One frame. t = monotonic seconds. Returns True only on the frame the alert fires (the caller logs it);
    `self.active` says whether to show it this frame."""
    if not armed:
      self.active, self._cond_t = False, None
      return False
    if self.active:
      if t - self._fire_t >= HOLD_S and max(k_plan, abs(k_meas)) < K_END:
        self.active = False
      return False
    if not (k_plan >= K_FIRE and lane_prob < LANE_PROB_LOW and V_MIN < v_ego < V_MAX):
      self._cond_t = None
      return False
    if self._cond_t is None:
      self._cond_t = t
    if t - self._cond_t >= PERSIST_S and t - self._fire_t >= REFIRE_S:
      self.active, self._fire_t, self._cond_t = True, t, None
      return True
    return False
