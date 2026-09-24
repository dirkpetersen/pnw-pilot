"""curvefix2pnw: two REAL modelV2 frames with the car's own left/right witnesses at the same instant.

Source: F-150 Lightning, OR-34 eastbound 2026-09-24, route 000001fd--84bd0d9914 segments 7-8 (rlogs in
drives/2026-09-24/corvallis-albany/raw/). Extracted frame by frame, rounded to 5 significant places; nothing here
is synthesised. The sign of modelV2.orientationRate.z is the thing under test, so it must come from the road, not
from a convention someone believed: the previous test asserted "+z = LEFT" and was wrong
(drives/2026-09-24/vision-left-flag-check.md, confirmed by the claim-verifier).

Each frame carries three independent witnesses of the turn direction, all from other sensors:
  str_ang_deg   carState.steeringAngleDeg   (+ = left, openpilot/opendbc convention)
  yaw_rate      carState.yawRate            (+ = left, Ford Yaw_Data_FD1)
  bearing_deg   GPS bearing ~3 s before and after (compass: FALLING = turning left)
"""

# 11:18:29.147 PT -- the left-hander at ~44.5616, -123.1418 (measured k 0.00261, taken at 77 mph / 2.92 m/s^2)
LEFT_HANDER = {
  "pt": "2026-09-24 11:18:29.147",
  "v_ego": 34.387, "str_ang_deg": 17.4, "yaw_rate": 0.0816, "bearing_deg": (126.3, 118.3),
  "z": [-0.08247, -0.08306, -0.08349, -0.08427, -0.08624, -0.08872, -0.08986, -0.08975, -0.08867, -0.08619,
        -0.08386, -0.08104, -0.07666, -0.07291, -0.06856, -0.06512, -0.06149, -0.05823, -0.05348, -0.04796,
        -0.04022, -0.03199, -0.02388, -0.01669, -0.01155, -0.00762, -0.00556, -0.00253, -0.00118, 0.00039,
        0.00153, 0.00253, 0.00318],
  "vx": [34.595, 34.608, 34.595, 34.559, 34.554, 34.567, 34.534, 34.519, 34.544, 34.524, 34.564, 34.5, 34.564,
         34.508, 34.466, 34.453, 34.5, 34.519, 34.551, 34.531, 34.605, 34.622, 34.683, 34.707, 34.722, 34.717,
         34.767, 34.77, 34.808, 34.8, 34.723, 34.652, 34.649],
}

# 11:19:13.247 PT -- the right-hander at ~44.5606, -123.1217 (measured k 0.00256; the OR-34 over-slow curve)
RIGHT_HANDER = {
  "pt": "2026-09-24 11:19:13.247",
  "v_ego": 32.248, "str_ang_deg": -16.7, "yaw_rate": -0.0772, "bearing_deg": (98.6, 105.9),
  "z": [0.07908, 0.07947, 0.07968, 0.07938, 0.0799, 0.08059, 0.0809, 0.08162, 0.08218, 0.08171, 0.08187, 0.08214,
        0.08139, 0.0806, 0.07924, 0.07812, 0.07684, 0.07588, 0.07432, 0.07292, 0.07087, 0.0688, 0.06564, 0.06123,
        0.05577, 0.04862, 0.04095, 0.03416, 0.02727, 0.02183, 0.01732, 0.01368, 0.01059],
  "vx": [32.97, 32.995, 32.976, 32.984, 32.957, 32.956, 32.941, 32.923, 32.919, 32.905, 32.884, 32.847, 32.868,
         32.829, 32.811, 32.798, 32.804, 32.787, 32.792, 32.763, 32.776, 32.772, 32.817, 32.846, 32.868, 32.869,
         32.868, 32.844, 32.849, 32.819, 32.749, 32.741, 32.693],
}

# modelV2.orientationRate.t -- identical in both frames (the model's fixed time grid)
T = [0.0, 0.0098, 0.0391, 0.0879, 0.1562, 0.2441, 0.3516, 0.4785, 0.625, 0.791, 0.9766, 1.1816, 1.4062, 1.6504,
     1.9141, 2.1973, 2.5, 2.8223, 3.1641, 3.5254, 3.9062, 4.3066, 4.7266, 5.166, 5.625, 6.1035, 6.6016, 7.1191,
     7.6562, 8.2129, 8.7891, 9.3848, 10.0]


def witnesses_say_left(frame) -> bool:
  """True when all three witnesses call it LEFT, False when all three call it right; raises when they disagree,
  so a fixture edit that breaks the ground truth cannot pass silently."""
  votes = {frame["str_ang_deg"] > 0.0, frame["yaw_rate"] > 0.0, frame["bearing_deg"][1] < frame["bearing_deg"][0]}
  if len(votes) != 1:
    raise AssertionError(f"witnesses disagree on {frame['pt']}: {frame}")
  return votes.pop()


class _NS:
  pass


def as_model(frame):
  """The frame as a modelV2-like object (the fields apex_turn_direction / model_curve_state read)."""
  m = _NS()
  m.orientationRate, m.velocity, m.position, m.action = _NS(), _NS(), _NS(), _NS()
  m.orientationRate.z = list(frame["z"])
  m.orientationRate.t = list(T)
  m.velocity.x = list(frame["vx"])
  m.position.x = [frame["vx"][i] * T[i] for i in range(len(T))]
  m.action.shouldStop = False
  return m
