"""wheelweight2pnw: detect an aftermarket steering-wheel weight and tell the driver.

WHY. drives/2026-10-26/tesla-i5-olympia-grand-mound: the owner keeps a weight on one side of the Tesla's wheel for FSD.
It twists the column: the learned angleOffsetDeg (liveParameters) went to 4.1 deg and the straight-road steering torque to
+0.16 Nm (earlier drives: offset 1.3 to 2.9 deg, torque -0.09 to -0.21 Nm). Positive = left.
drives/2026-10-27/tesla-us34-corvallis: with the weight on the RIGHT the torque was -0.19 Nm but the offset stayed at 4.8 to 5.3
deg, so the offset is not (only) the weight and it no longer counts in the decision. `off` is kept for telemetry (wwOff) and
for the curve pricing (ces_pnw._ww_pricing), which follows the offset itself.

DECIDES from the median steering torque over the last WINDOW_S of straight-road samples only: "left" above TQ_LEFT_ON, "right"
below TQ_RIGHT_ON, else "none". To leave left/right, the median must fall back past the threshold by TQ_HYST (no flicker).
A straight-road sample: vEgo > V_MIN, |lateral accel| < LAT_MAX, no driver steering press, engaged or not. With fewer than
MIN_SAMPLES in the window (city driving) the last decision holds. A non-finite carState input makes the state "unknown" (the
caller logs every state change with the reason). An invalid liveParameters only makes `off` None (off_why says why).

Pure: no params, no sockets, no clock. selfdrived feeds it and owns the event and the log.
"""
import math
from collections import deque
from statistics import median

# PROVISIONAL (2026-10-27) until a drive without the weight calibrates them: weight left (10-26) +0.16 Nm, weight right
# (10-27) -0.19 Nm, earlier drives -0.09 to -0.21 Nm (weight probably right). There is no no-weight baseline yet.
TQ_LEFT_ON = 0.05     # Nm, straight-road torque median above this = weight left
TQ_RIGHT_ON = -0.15   # Nm, below this = weight right
TQ_HYST = 0.05        # Nm back past the threshold before the decision clears
WINDOW_S = 60.0       # s of qualifying samples in the torque median
SAMPLE_S = 0.1        # s between decisions and kept samples (10 Hz, so the median sorts at most 600 values)
MIN_SAMPLES = 100     # 10 s of straight road before the torque median counts
V_MIN = 20.0          # m/s
LAT_MAX = 0.1         # m/s^2

UNKNOWN, NONE, LEFT, RIGHT = "unknown", "none", "left", "right"


class WheelWeight:
  def __init__(self):
    self.side = UNKNOWN
    self.off: float | None = None    # angleOffsetDeg of the last frame; None when that frame had no valid offset (the curve pricing reads it)
    self.off_why = "no input yet"    # why `off` is None (meaningful only while it is)
    self.tq: float | None = None     # median straight-road torque, None until MIN_SAMPLES
    self.why = "no input yet"        # why the state is "unknown" (meaningful only while it is)
    self._samples: deque[tuple[float, float]] = deque()
    self._sample_t = -math.inf
    self._missing: bool | None = None   # last frame had a missing input; None before the first frame

  def tele(self) -> dict:
    return {"wwSide": self.side, "wwOff": None if self.off is None else round(self.off, 2),
            "wwTq": None if self.tq is None else round(self.tq, 3)}

  def fail(self, why: str) -> None:
    """The caller could not feed this frame (its input read raised): "unknown" until a good frame, which update() then reports."""
    self.side, self.why, self._missing, self.off, self.off_why = UNKNOWN, why, True, None, why

  def update(self, t: float, lp_valid: bool, off: float, v_ego: float, lat_accel: float, torque: float,
             steering_pressed: bool) -> bool:
    """One frame. t = monotonic seconds. Returns True when the caller must log: the state changed, or an input went
    missing or came back (so a missing input is logged once, also while the state is already "unknown")."""
    prev, prev_missing = self.side, self._missing
    self.off = off if lp_valid and math.isfinite(off) else None   # informational + the curve pricing, not the decision
    if self.off is None:
      self.off_why = f"liveParameters not valid (valid={lp_valid}, angleOffsetDeg={off})"
    if not (math.isfinite(torque) and math.isfinite(lat_accel) and math.isfinite(v_ego)):
      self.why = f"carState not finite (steeringTorque={torque}, latAccel={lat_accel}, vEgo={v_ego})"
    else:
      self._missing = False
      return self._decide(t, v_ego, lat_accel, torque, steering_pressed) or prev_missing is not False
    self._missing, self.side = True, UNKNOWN
    return prev_missing is not True or prev != UNKNOWN

  def _decide(self, t: float, v_ego: float, lat_accel: float, torque: float, steering_pressed: bool) -> bool:
    prev = self.side

    if t - self._sample_t < SAMPLE_S - 0.005:   # half a 100 Hz frame of slack, so float time does not skip to 11 frames
      return False   # decide at 10 Hz: the median sorts up to 600 values
    self._sample_t = t
    if v_ego > V_MIN and abs(lat_accel) < LAT_MAX and not steering_pressed:
      self._samples.append((t, torque))
    while self._samples and t - self._samples[0][0] > WINDOW_S:
      self._samples.popleft()
    if len(self._samples) < MIN_SAMPLES:
      self.tq = None
      if self.side == UNKNOWN:
        self.why = f"{len(self._samples)} of {MIN_SAMPLES} straight-road samples in the last {WINDOW_S:.0f} s"
      return False   # too little straight road: the last decision holds
    self.tq = median(tq for _, tq in self._samples)

    if self.side == LEFT and self.tq > TQ_LEFT_ON - TQ_HYST:
      pass
    elif self.side == RIGHT and self.tq < TQ_RIGHT_ON + TQ_HYST:
      pass
    elif self.tq > TQ_LEFT_ON:
      self.side = LEFT
    elif self.tq < TQ_RIGHT_ON:
      self.side = RIGHT
    else:
      self.side = NONE
    return self.side != prev
