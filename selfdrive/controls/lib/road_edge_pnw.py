"""roadedge2pnw: no AUTO (nudgeless) lane change toward a road edge.

Port of sunnypilot's RoadEdgeLaneChangeController (relc.py, sunnypilot 2d859a8ca + a06366ce9; MIT,
rav4kumar / sunnypilot). Thresholds and timers are theirs, unchanged. A side counts as an edge when, for
EDGE_REACTION_TIME, the model sees a confident road edge (1 - roadEdgeStds > EDGE_PROB), no lane line
beyond our own on that side (laneLineProbs[0] / [3] < NEARSIDE_PROB) and the edge is close
(|roadEdges.y[0]| - VEHICLE_EDGE_MARGIN < EDGE_CLEARANCE, metres). It clears EDGE_CLEAR_TIME after the
condition goes away. DesireHelper uses it to hold off the auto lane change only; a driver nudge still
starts the change and the blind-spot block is unchanged.

Differences from sunnypilot: no RoadEdgeLaneChangeEnabled toggle (always on, like the nudgelesshighway
gate it sits next to; the driver can nudge, or set NudgeForLaneChange), and no modelDataV2SP output.
"""
import numpy as np

from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL

NEARSIDE_PROB = 0.2
EDGE_PROB = 0.35
EDGE_REACTION_TIME = 1.0
EDGE_CLEAR_TIME = 0.3
MIN_SPEED = 20 * CV.MPH_TO_MS
VEHICLE_EDGE_MARGIN = 1.08
EDGE_CLEARANCE = 3.7


class RoadEdgeLaneChange:
  """update() once per NEW modelV2 frame (the timers count DT_MDL steps). left/right = edge detected."""

  def __init__(self):
    self.reset()

  def reset(self) -> None:
    self.left = False
    self.right = False
    self._edge_t = [0.0, 0.0]
    self._clear_t = [0.0, 0.0]

  def update(self, model, v_ego: float) -> tuple[bool, bool]:
    if v_ego < MIN_SPEED:
      self.reset()
      return self.left, self.right

    stds, probs, edges = model.roadEdgeStds, model.laneLineProbs, model.roadEdges
    if len(edges) == 2 and len(edges[0].y) > 0 and len(edges[1].y) > 0:
      clearance = (abs(edges[0].y[0]) - VEHICLE_EDGE_MARGIN, abs(edges[1].y[0]) - VEHICLE_EDGE_MARGIN)
    else:
      clearance = (0.0, 0.0)  # sunnypilot: no edge geometry -> treat as close (the prob terms still decide)

    detected = [self.left, self.right]
    for i, lane_prob in ((0, probs[0]), (1, probs[3])):
      edge_prob = np.clip(1.0 - stds[i], 0.0, 1.0)
      if edge_prob > EDGE_PROB and lane_prob < NEARSIDE_PROB and clearance[i] < EDGE_CLEARANCE:
        self._edge_t[i] = min(self._edge_t[i] + DT_MDL, EDGE_REACTION_TIME + EDGE_CLEAR_TIME)
        self._clear_t[i] = 0.0
        if self._edge_t[i] > EDGE_REACTION_TIME:
          detected[i] = True
      else:
        self._clear_t[i] += DT_MDL
        if self._clear_t[i] > EDGE_CLEAR_TIME:
          self._edge_t[i] = 0.0
          detected[i] = False
    self.left, self.right = detected
    return self.left, self.right
