"""wheelweight2pnw stage 2: price a curve that bends TOWARD a steering-wheel weight.

WHY. drives/2026-10-26/tesla-i5-olympia-grand-mound: the weight moves the learned angleOffsetDeg to about 4 deg. The car-side
angle clamp (opendbc lateral.apply_steer_angle_limits_vm: +-get_max_angle_vm) applies to the raw commanded angle, which
includes the offset, so a curve toward the weight gets |offset| fewer degrees of steering. This module finds the highest speed
at which the angle such a curve needs still fits under the clamp:

    degrees(VM.get_steer_from_curvature(k, v, 0)) + |offset| <= get_max_angle_vm(v, VM, limits) - margin(v)

margin(v) = the angle for MARGIN_LAT_A at speed v. Both sides use the same VehicleModel and the same functions as the clamp.
The free angle falls as v rises, so the speed is found by bisection between V_LO and V_HI. Lower-only: the callers take the
minimum with what they already compute. The clamp itself and panda safety are NOT changed.

Pure: no params, no sockets, no clock.
"""
import math

from opendbc.car.lateral import get_max_angle_vm
from opendbc.car.tesla.values import CarControllerParams as RAVEN_LIMITS  # the clamp of the only car with wheel_weight_detect
from opendbc.car.vehicle_model import VehicleModel

MARGIN_LAT_A = 0.1   # m/s^2 of the clamp kept free (owner 2026-10-26)
V_LO = 10.0          # m/s, bisection bounds
V_HI = 60.0
TURN_MIN_DEG = 1.0   # a heading change smaller than this has no known direction


def raven_clamp_vm() -> VehicleModel:
  """The VehicleModel the Raven carcontroller builds for its clamp (opendbc tesla/carcontroller.py, LEGACY_CARS branch), so the
  price and the clamp use the same model. Only called where PnwVehicle.wheel_weight_detect holds."""
  from opendbc.car.tesla.interface import CarInterface
  return VehicleModel(CarInterface.get_non_essential_params("TESLA_MODEL_S_HW3"))


def offset_speed(VM, k: float, off_deg: float, margin: float = MARGIN_LAT_A, limits=RAVEN_LIMITS) -> float | None:
  """Highest speed (m/s) in [V_LO, V_HI] at which curvature k (1/m, > 0) fits under the clamp with |off_deg| used up by the
  offset. V_HI = the curve does not bind below V_HI; V_LO = it does not fit even at V_LO. None for an unusable input."""
  if not (math.isfinite(k) and k > 0.0 and math.isfinite(off_deg)):
    return None

  def fits(v):
    need = math.degrees(VM.get_steer_from_curvature(k, v, 0)) + abs(off_deg)
    return need <= get_max_angle_vm(v, VM, limits) - math.degrees(VM.get_steer_from_curvature(margin / (v * v), v, 0))

  if fits(V_HI):
    return V_HI
  if not fits(V_LO):
    return V_LO
  lo, hi = V_LO, V_HI
  while hi - lo > 0.01:
    mid = 0.5 * (lo + hi)
    lo, hi = (mid, hi) if fits(mid) else (lo, mid)
  return lo


def turn_side(dh_deg: float) -> str | None:
  """Heading change along the road (deg, clockwise positive, as a compass bearing) -> "left" / "right", None when unknown."""
  if not math.isfinite(dh_deg) or abs(dh_deg) < TURN_MIN_DEG:
    return None
  return "right" if dh_deg > 0.0 else "left"
