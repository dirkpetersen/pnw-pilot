"""parkgate2pnw: stop the ~1 Hz `ces_events` breadcrumb from logging a PARKED truck.

`IsOnroad` follows IGNITION, not motion (CLAUDE.md Rule 3), and on the Lightning parked + charging =
ignition on, indefinitely. Measured over six archived generations from S3 (42,918 records): 40,203
(93.7 %) are stationary, 39,983 of them the same `{"ev":"tick","reason":"stopLatch"}` row. Full
rationale, the corrected retention arithmetic and the on-car checks: docs/pnw/PARKGATE2PNW.md.

Four invariants, and they are the whole design:

 1. **Only `gearShifter == park` suppresses.** Not speed, not standstill, not IsOnroad. A red-light
    stop is `vEgo` 0 in DRIVE and keeps logging at full rate -- the stop / lurch / green-light
    analyses live entirely in those records. Compared AS AN ENUM: `str(x) == "GearShifter.park"` is
    always False and has already silently killed one feature here (memory capnp-enum-str-trap).
 2. **Fail open.** Absent carState, None/missing gear, `unknown`, a non-enum gear, an unread kill
    switch -- all log. And a MOVING car always logs whatever the gear says (`PARK_RELEASE_V`):
    CANParser zero-inits every signal and 0 decodes as Park on 74 platforms in the pinned opendbc
    (selfdrive/car/gear_park.py), so a gear message that never arrived can read `park` for a whole
    drive. Speed is an escape hatch that can only RELEASE, never a gate that can suppress.
 3. **No silent gap.** While suppressing, one explicitly-marked record per `PARK_HEARTBEAT_S` carries
    `parkGate` and `parkSupp`. A gap that reads as "the logger died" is itself a Rule 2 failure.
 4. **It announces itself** -- `cloudlog.event("ces_park_gate", ...)` on the rising and falling edge
    ONLY. Two lines per Park episode; this runs inside selfdrived's 100 Hz loop, so a per-tick log
    would be its own regression.

Hysteresis: suppress after `PARK_HOLD_S` of continuous Park, release on the first non-Park tick. The
30 s is parknorec2pnw's, deliberately, so this log and the route segments agree about which windows
were parked -- and a rest-stop park/unpark shuffle keeps its surrounding driving context whole.
"""
import math

from cereal import car
from openpilot.common.swaglog import cloudlog

GearShifter = car.CarState.GearShifter

PARK_HOLD_S = 30.0        # continuous Park before suppression starts (matches parknorec2pnw)
PARK_HEARTBEAT_S = 60.0   # one marked record per minute while suppressing -- never a silent gap
PARK_RELEASE_V = 0.5      # m/s. FAIL-OPEN INTERLOCK ONLY: a moving car logs whatever the gear says.

# update() return values. Anything but SUPPRESS means "write the record".
LOG = "log"               # normal record, gate not acting
HOLD = "hold"             # the marked heartbeat written while suppressing (incl. the hold's first)
RELEASE = "release"       # the first record after a hold ended -- carries the suppressed count
SUPPRESS = "suppress"     # write nothing


class ParkTickGate:
  """Decides whether one ~1 Hz ces_events breadcrumb is written. One instance per CESController.

  The two per-tick writers it serves (_publish_status's "tick" and _steer_log_step's CES-off "steer")
  are mutually exclusive on any given tick, so they share this one instance and its hysteresis
  survives CESMode being flipped mid-session.

  Edge-triggered records (adopt, greenLight, steerEvent, alert, mads) are NOT gated: they are rare
  (32 adopt records against 10,568 ticks on the 2026-07-13 log) and a mode transition while parked is
  itself worth having.
  """

  def __init__(self, hold_s: float = PARK_HOLD_S, heartbeat_s: float = PARK_HEARTBEAT_S):
    self.hold_s = hold_s
    self.heartbeat_s = heartbeat_s
    self.parked = False          # gear reads Park right now, after the moving interlock (record field)
    self.holding = False         # suppression is active
    self.suppressed = 0          # records suppressed during the CURRENT hold
    self.hold_total = 0          # ... and cumulatively, since boot -- so "suppressed" is auditable
    self.last_hold_suppressed = 0  # count from the hold that most recently ended (RELEASE record)
    self._park_since: float | None = None
    self._last_emit: float | None = None
    # cesarchive2pnw: True on exactly the ONE tick the gear goes into Park from a KNOWN non-Park gear
    # (i.e. the end of a drive), whatever gate_on says. Undebounced on purpose -- see ces_pnw's
    # rotate_at_park. A None gear (no carState yet) is not "was driving", so a boot in Park is no edge.
    self.park_edge = False
    self._was_known_unparked = False

  def _clear(self) -> None:
    self.holding = False
    self.suppressed = 0
    self._park_since = None

  def update(self, gear, v_ego, now: float, gate_on: bool = True) -> str:
    """One tick. `gear` is the capnp gearShifter enum (or None), compared AS AN ENUM and never
    stringified -- `str(x) == "GearShifter.park"` is ALWAYS False and has already silently killed one
    feature here (memory capnp-enum-str-trap). `now` is time.monotonic(). `gate_on` False = the
    RecordWhileParked kill switch, or the kill switch has not been read yet: log exactly as before.
    """
    # Enum identity, not a string, and not truthiness. An unset/None/foreign gear is simply not Park.
    park = gear is not None and gear == GearShifter.park
    # FAIL-OPEN INTERLOCK. A finite speed above the threshold releases regardless of the gear; a
    # non-numeric or non-finite reading is no evidence of motion, so it leaves the gear's verdict
    # alone rather than inventing one.
    if park and isinstance(v_ego, (int, float)) and math.isfinite(v_ego) and abs(float(v_ego)) > PARK_RELEASE_V:
      park = False
    self.parked = park
    self.park_edge = park and self._was_known_unparked
    self._was_known_unparked = gear is not None and not park

    if not gate_on:
      # Kill switch (or the switch was never read): behave byte-identically to the old code. Any
      # in-flight hold is closed out through the normal RELEASE path so the log still says so.
      if self.holding:
        return self._release(now, gear, reason="gateOff")
      self._clear()
      self._last_emit = now
      return LOG

    if not park:
      if self.holding:
        return self._release(now, gear, reason="unparked")
      self._clear()
      self._last_emit = now
      return LOG

    if self._park_since is None:
      self._park_since = now
    if now - self._park_since < self.hold_s:
      self._last_emit = now
      return LOG                                  # still inside the debounce -- full rate

    first = not self.holding
    if first:
      self.holding = True
      self.suppressed = 0
      cloudlog.event("ces_park_gate", hold=True, gear=str(gear),
                     heartbeat_s=self.heartbeat_s, hold_s=self.hold_s,
                     note="ces_events ~1 Hz breadcrumb suppressed while the shifter is in Park; " +
                          "one marked heartbeat record still written per heartbeat_s")
    if first or self._last_emit is None or (now - self._last_emit) >= self.heartbeat_s:
      self._last_emit = now
      return HOLD
    self.suppressed += 1
    self.hold_total += 1
    return SUPPRESS

  def _release(self, now: float, gear, reason: str) -> str:
    self.last_hold_suppressed = self.suppressed
    cloudlog.event("ces_park_gate", hold=False, suppressed=self.suppressed, reason=reason,
                   gear=str(gear), hold_total=self.hold_total)
    self._clear()
    self._last_emit = now
    return RELEASE

  def record_fields(self, decision: str) -> dict:
    """The extra keys the marked records carry. Empty for a plain LOG, so 99 % of records are
    unchanged in shape. `parkSupp` is what makes a suppressed window impossible to mistake for a
    dead logger: every heartbeat states how many records it stands for."""
    if decision == HOLD:
      return {"parkGate": "hold", "parkSupp": self.suppressed}
    if decision == RELEASE:
      return {"parkGate": "release", "parkSupp": self.last_hold_suppressed}
    return {}
