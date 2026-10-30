"""wheelweight2pnw: detect an aftermarket steering-wheel weight and tell the driver.

WHY. drives/2026-10-26/tesla-i5-olympia-grand-mound: the owner keeps a weight on one side of the Tesla's wheel for FSD.
The straight-road steering torque median follows the weight: +0.16 Nm with it on the left (10-26, one drive); on the right
-0.5 Nm on the highway (drives/2026-10-29/tesla-corvallis-seattle, steady every hour) and -0.19 Nm in town (10-27 morning);
-0.10 Nm with no weight (10-27 evening, US-34 Corvallis to Albany). The no-weight baseline is not zero, so the thresholds are asymmetric.
10-29: false "left" flickers (+0.04 to +0.27 Nm) in Seattle stop-and-go traffic at 45 to 72 mph and at 16 mph, because the
60 s median then held few straight-road samples taken at low speed. So a sample now needs 60 mph (V_MIN).
The learned angleOffsetDeg does NOT: it stayed 4.8 to 5.3 deg on 10-27 with the weight on the right, so the offset is not
part of this decision (it only sets off_side, below). Positive = left.

DECIDES "left" when the median steering torque over the last WINDOW_S of straight-road samples > TQ_LEFT_ON; "right" when
< -TQ_RIGHT_ON; else "none". To leave left, the median must fall to TQ_LEFT_ON - TQ_HYST or below; to leave right, it must
rise to -(TQ_RIGHT_ON - TQ_RIGHT_HYST) or above (no flicker). The right clear point (-0.25) is below the no-weight baseline (-0.10),
so "right" clears when the weight comes off.
A straight-road sample: vEgo > V_MIN, |lateral accel| < LAT_MAX, no driver steering press, engaged or not. With fewer than MIN_SAMPLES in the window
(city driving) the last decision holds. A missing or invalid input makes the state "unknown" (the caller logs every state change with the reason).

OFFSET SIDE (off_side, for curve pricing, ww_price_pnw): "left" when angleOffsetDeg > OFF_ON, "right" when < -OFF_ON, cleared below
OFF_ON - OFF_HYST, else "none"; "unknown" while liveParameters is invalid. It is the offset alone, whatever its cause: the clamp takes the
whole offset from the side it points to (drives/2026-10-27/tesla-us34-corvallis: 4.8 to 5.3 deg for 45 min with the weight on the RIGHT).

Pure: no params, no sockets, no clock. selfdrived feeds it and owns the event and the log.
"""
import math
from collections import deque
from statistics import median

OFF_ON = 3.0          # deg, learned angle offset that sets off_side (curve pricing)
OFF_HYST = 1.0        # deg back before off_side clears
# shortcut: the thresholds are from four readings (straight-road median): weight left +0.16 (10-26), weight right -0.5 on the
# highway (10-29) and -0.19 in town (10-27 morning), no weight -0.10 Nm (one 5-minute drive, 10-27 evening). "Right" is set at
# -0.30, between the no-weight and the highway weight-right values, so the town reading -0.19 reads "none". It clears at -0.25,
# below the no-weight baseline -0.10. "Left" has only one measured weight-left drive (+0.16): recalibrate TQ_LEFT_ON and TQ_HYST
# after the next weight-left highway drive.
TQ_LEFT_ON = 0.10     # Nm, straight-road torque median above which the weight is on the left
TQ_RIGHT_ON = 0.30    # Nm, straight-road torque median below minus this is a weight on the right (no weight reads -0.10)
TQ_HYST = 0.05        # Nm back before "left" clears
TQ_RIGHT_HYST = 0.05  # Nm back before "right" clears (clear point -0.25, below the no-weight -0.10)
WINDOW_S = 60.0       # s of qualifying samples in the torque median
SAMPLE_S = 0.1        # s between decisions and kept samples (10 Hz, so the median sorts at most 600 values)
MIN_SAMPLES = 100     # 10 s of straight road before the torque median counts
V_MIN = 26.8          # m/s (60 mph): stop-and-go traffic must not feed the median (10-29 Seattle false "left")
LAT_MAX = 0.1         # m/s^2

UNKNOWN, NONE, LEFT, RIGHT = "unknown", "none", "left", "right"


class WheelWeight:
  def __init__(self):
    self.side = UNKNOWN
    self.off_side = UNKNOWN          # the side the learned angle offset points to (curve pricing), see the module docstring
    self.off: float | None = None    # last valid angleOffsetDeg
    self.tq: float | None = None     # median straight-road torque, None until MIN_SAMPLES
    self.why = "no input yet"        # why the state is "unknown" (meaningful only while it is)
    self._samples: deque[tuple[float, float]] = deque()
    self._sample_t = -math.inf
    self._missing: bool | None = None   # last frame had a missing input; None before the first frame

  def tele(self) -> dict:
    return {"wwSide": self.side, "wwOffSide": self.off_side, "wwOff": None if self.off is None else round(self.off, 2),
            "wwTq": None if self.tq is None else round(self.tq, 3)}

  def fail(self, why: str) -> None:
    """The caller could not feed this frame (its input read raised): "unknown" until a good frame, which update() then reports."""
    self.side, self.off_side, self.why, self._missing = UNKNOWN, UNKNOWN, why, True

  def update(self, t: float, lp_valid: bool, off: float, v_ego: float, lat_accel: float, torque: float,
             steering_pressed: bool) -> bool:
    """One frame. t = monotonic seconds. Returns True when the caller must log: the state changed, or an input went
    missing or came back (so a missing input is logged once, also while the state is already "unknown")."""
    prev, prev_missing, prev_off_side = self.side, self._missing, self.off_side
    lp_ok = lp_valid and math.isfinite(off)
    self.off_side = self._offset_side(off) if lp_ok else UNKNOWN
    if not lp_ok:
      self.why = f"liveParameters not valid (valid={lp_valid}, angleOffsetDeg={off})"
    elif not (math.isfinite(torque) and math.isfinite(lat_accel) and math.isfinite(v_ego)):
      self.why = f"carState not finite (steeringTorque={torque}, latAccel={lat_accel}, vEgo={v_ego})"
    else:
      self._missing = False
      return self._decide(t, off, v_ego, lat_accel, torque, steering_pressed) or prev_missing is not False or self.off_side != prev_off_side
    self._missing, self.side = True, UNKNOWN
    return prev_missing is not True or prev != UNKNOWN or self.off_side != prev_off_side

  def _offset_side(self, off: float) -> str:
    if off > OFF_ON or (self.off_side == LEFT and off > OFF_ON - OFF_HYST):
      return LEFT
    if off < -OFF_ON or (self.off_side == RIGHT and off < -(OFF_ON - OFF_HYST)):
      return RIGHT
    return NONE

  def _decide(self, t: float, off: float, v_ego: float, lat_accel: float, torque: float, steering_pressed: bool) -> bool:
    prev = self.side
    self.off = off

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
    elif self.side == RIGHT and self.tq < -(TQ_RIGHT_ON - TQ_RIGHT_HYST):
      pass
    elif self.tq > TQ_LEFT_ON:
      self.side = LEFT
    elif self.tq < -TQ_RIGHT_ON:
      self.side = RIGHT
    else:
      self.side = NONE
    return self.side != prev
