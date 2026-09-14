"""parknorec2pnw: the rules card uses to publish the change-only `GearPark` param.

`GearPark` is the truth source for "the car is in Park" for every process that must not subscribe to
carState (CLAUDE.md Rule 3). Since parknorec2pnw it also STOPS loggerd (system/manager/park_record_gate.py),
so a wrong True now costs a drive's log, not just an upload decision. The rules below are asymmetric on
purpose -- being wrong toward "not parked" only means recording as before:

  SET True   only on a tick with valid CAN that reads Park. Required, not pedantic: CANParser initialises every
             signal to 0, and on many brands 0 is "Park", so a gear message that has NEVER been received
             reads gearShifter == park. The Lightning did too until pnw-opendbc gearunknown2pnw (now
             unknown until PowertrainData_10 arrives); 74 other platforms in the pinned opendbc still do
             (e.g. Hyundai; measured, see tests/test_gear_park.py). Only canValid tells that apart.
  CLEAR      on any tick that decodes a KNOWN non-Park gear, valid CAN or not. Before this, an invalid
             tick was ignored in both directions, so a CAN fault that spanned Park -> Drive left GearPark
             True for the whole drive -- which would now mean an unrecorded drive.
             `unknown` (Tesla DI_GEAR_SNA/INVALID, Ford Unknown_Position) on INVALID CAN is not evidence
             of leaving Park and keeps the last value; on valid CAN it clears, as it always did.

It also says so, loudly, when Park cannot be confirmed (reading is park or unknown but GearPark is False
for UNCONFIRMED_LOG_S): a car that never decodes a gear, or a Lightning that boots into a quiet-CAN
charging session, keeps recording, and swaglog carries `gear_park_unconfirmed` explaining why.
"""
from cereal import car
from openpilot.common.swaglog import cloudlog

GearShifter = car.CarState.GearShifter

UNCONFIRMED_LOG_S = 60.0
UNCONFIRMED_NOTE = "Park not confirmed (gear unknown, or Park on invalid CAN): GearPark stays False, device keeps recording"


class GearParkWriter:
  def __init__(self) -> None:
    self.value = False                        # card seeds the param False at startup; this mirrors it
    self._unconfirmed_since: float | None = None
    self._unconfirmed_logged = False

  def update(self, gear, can_valid: bool, now: float) -> bool | None:
    """Returns the new GearPark value when it CHANGES (the caller writes the param), else None.
    `gear` is the capnp enum, compared as an enum -- never stringified (see memory capnp-enum-str-trap)."""
    park = gear == GearShifter.park
    known = gear != GearShifter.unknown
    if park and can_valid:
      new = True
    elif not park and (can_valid or known):
      new = False
    else:
      new = self.value

    unconfirmed = not new and (park or not known)
    if unconfirmed:
      if self._unconfirmed_since is None:
        self._unconfirmed_since = now
      elif not self._unconfirmed_logged and now - self._unconfirmed_since >= UNCONFIRMED_LOG_S:
        self._unconfirmed_logged = True
        cloudlog.event("gear_park_unconfirmed", error=True, gear=str(gear), can_valid=bool(can_valid),
                       seconds=round(now - self._unconfirmed_since, 1),
                       note=UNCONFIRMED_NOTE)
    else:
      if self._unconfirmed_logged:
        cloudlog.event("gear_park_unconfirmed_cleared", gear=str(gear), can_valid=bool(can_valid))
      self._unconfirmed_since = None
      self._unconfirmed_logged = False

    if new != self.value:
      self.value = new
      cloudlog.event("gear_park", value=new, gear=str(gear), can_valid=bool(can_valid))
      return new
    return None
