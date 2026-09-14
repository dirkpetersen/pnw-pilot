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
  SET True   ALSO on a Park read with canValid False, but only on a car whose PnwVehicle declares
             gear_unknown_until_seen, and only while the parser that carries the gear message (its
             gear_source_bus) is valid on this tick (gearparkcan2pnw). Such a car reports `park` only from a
             gear frame that actually arrived, so the zero-init trap above cannot reach this path; the
             parser gate is what refuses a Park HELD from a gear message that has since gone quiet. This
             closes the quiet-CAN charging gap: the Lightning's camera bus sleeps while its powertrain bus
             stays alive, which made global canValid False for the whole session.
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
from opendbc.can.parser import MAX_BAD_COUNTER
from openpilot.common.swaglog import cloudlog

GearShifter = car.CarState.GearShifter

UNCONFIRMED_LOG_S = 60.0
UNCONFIRMED_NOTE = "Park not confirmed (gear unknown, or Park on invalid CAN): GearPark stays False, device keeps recording"


def parser_valid_now(cp) -> bool:
  """CANParser.can_valid for this tick, READ-ONLY: every message alive and no counter failures.

  Not `cp.can_valid`: that property steps the parser's invalid-count hysteresis on every read, and
  CarInterface.update already reads it once per tick to build canValid, so a second read would change when
  canValid itself goes False. Nor the counter it leaves behind: that update uses all(), which stops at the
  first invalid parser, so a later parser (the Tesla's chassis parser is the last of five) keeps a stale
  counter on exactly the ticks where another bus is invalid. This recomputes the same checks without the
  hysteresis, and adds the parser's bus_timeout, so it is stricter than can_valid in two ways: False on the
  first tick a message times out, and False once the whole bus has been silent past its bus timeout (at most
  0.5 s). The second matters in the first second of a session, before the parser has learned message rates:
  until then each message counts as alive for 10 s after its last frame, so a bus that dies right after
  delivering a Park would otherwise keep reading valid for up to 10 s."""
  now = cp._last_update_nanos
  bus_timeout = cp.bus_timeout
  return not bus_timeout and all(st.valid(now, bus_timeout) and st.counter_fail < MAX_BAD_COUNTER
                                 for st in cp.message_states.values())


class GearParkWriter:
  def __init__(self) -> None:
    self.value = False                        # card seeds the param False at startup; this mirrors it
    self._unconfirmed_since: float | None = None
    self._unconfirmed_logged = False
    self._gear_source = None                  # the gear message's CANParser, on a gear_unknown_until_seen car
    self._source_error_logged = False

  def attach_gear_source(self, parser) -> None:
    """card calls this once the car is known, with PnwVehicle's gear_source_bus parser. Never called = today's rules."""
    self._gear_source = parser

  def _gear_source_valid(self) -> bool:
    try:
      return parser_valid_now(self._gear_source)
    except Exception:
      # Rule 2: fall back to today's rule (canValid only -> records), and say so once.
      if not self._source_error_logged:
        self._source_error_logged = True
        cloudlog.exception("gear_park: gear source parser validity read failed -- Park needs valid CAN again, device keeps recording")
      return False

  def update(self, gear, can_valid: bool, now: float) -> bool | None:
    """Returns the new GearPark value when it CHANGES (the caller writes the param), else None.
    `gear` is the capnp enum, compared as an enum -- never stringified (see memory capnp-enum-str-trap)."""
    park = gear == GearShifter.park
    known = gear != GearShifter.unknown
    # Evaluated only for a Park read on invalid CAN on a capable car, so every other tick costs nothing.
    source_valid = None
    if park and not can_valid and self._gear_source is not None:
      source_valid = self._gear_source_valid()
    if park and (can_valid or source_valid):
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
                       gear_source_valid=source_valid, seconds=round(now - self._unconfirmed_since, 1),
                       note=UNCONFIRMED_NOTE)
    else:
      if self._unconfirmed_logged:
        cloudlog.event("gear_park_unconfirmed_cleared", gear=str(gear), can_valid=bool(can_valid))
      self._unconfirmed_since = None
      self._unconfirmed_logged = False

    if new != self.value:
      self.value = new
      cloudlog.event("gear_park", value=new, gear=str(gear), can_valid=bool(can_valid), gear_source_valid=source_valid)
      return new
    return None
