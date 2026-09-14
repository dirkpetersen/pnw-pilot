"""
madsop2pnw — the openpilot half of MADS: a PARALLEL lateral authority.

Ported from sunnypilot's MADS (`sunnypilot/mads/{mads,state}.py`, MIT, via
`sunny/bluepilot vin-lightning-2024-25`). Original: Copyright (c) 2021-, Haibin Wen, sunnypilot,
and a number of other contributors.

WHAT THIS IS FOR
----------------
The F-150 Lightning runs STOCK ACC. Speed belongs to the truck; steering is the one thing
openpilot does for it. A brake tap drops stock cruise, openpilot disengages with it, and the
driver loses the *only* thing openpilot was doing. mads2pnw taught the panda to keep a second,
parallel `controls_allowed_lateral` flag alive across that brake press. This module is the
openpilot side of the same idea: it keeps openpilot *asking* for lateral in exactly the frames
the panda still permits it.

WHAT THIS IS NOT
----------------
It is NOT a suppression of openpilot's own disengage, and that distinction is the whole design:

  * `selfdrived`'s own state machine runs FIRST and is not touched. On a brake press openpilot
    still disengages: `selfdriveState.enabled`/`.active` both go False, `CC.enabled` goes False,
    longitudinal stops, `mismatch_counter` is reset (it is keyed on `self.enabled`) so
    `controlsMismatch` can never fire because of this feature.
  * This module then answers a SEPARATE question — "may openpilot still steer?" — on a separate
    message (`madsState`), consulted by a separate branch in `controlsd`.

The refused shortcut was to latch `CC.latActive` or to drop openpilot's own `pedalPressed`
event. Either produces "UI says engaged, car is not actuating": `mismatch_counter` climbs at
100 Hz and `controlsMismatch` hard-disables at 2.0 s. Do not reintroduce it.

INERT UNLESS THE PANDA CAN HONOUR IT
------------------------------------
Every gate is read from ONE place: `CarParams.alternativeExperience`, the exact bitfield
`card.py` handed to the panda. That is deliberate — openpilot cannot form an opinion the panda
does not share, because it reads the panda's own contract rather than re-deriving it from
params. `card.py` already requires BOTH `PnwVehicle.mads_lateral` (capability view, today the
Lightning; never a fingerprint test in feature code) and `PandaMadsSafety` (the hand-set
declaration that the flashed panda carries the safety build). With either off it sends 0, so
`available` is False here, `madsState` carries no authority, and `controlsd` uses
`selfdriveState.active` exactly as it does today.

MIRRORING THE PANDA
-------------------
`opendbc/safety/pnw/mads.h` — the authority this module tracks:
  * latches on the RISING edge of the panda's `controls_allowed` (there is deliberately no MADS
    button and no ACC-main engage in the pnw port),
  * clears on ACC-main falling, on the brake rising edge when DISENGAGE is selected, on lag/an
    invalid rx message, and on `controls_allowed` falling while NOT braking.
So lateral survives exactly one thing: openpilot losing controls *while the brake is down*.

This module is deliberately NARROWER than the panda in two places, both in the fail-to-stock
direction (openpilot stops steering while the panda would still have permitted it — which costs
nothing, because openpilot is the only thing that ever sends a steering command):
  1. Any other disabling event in the frame (CANCEL, a fault, reverse gear, ESP intervention,
     ACC MAIN off) ends lateral, even mid-brake. The panda's `!braking.current` test would let a
     CANCEL press *during* a brake keep the latch; here it does not.
  2. `MADS_PAUSE_LATERAL_ON_BRAKE` is refused outright (see `__init__`).
And one place where it is not narrower but must actively TRACK the panda, because there the
panda revokes first and openpilot would otherwise never notice: stock cruise re-engaging
without openpilot engaging with it. See the preamble of `update()`.
"""
from cereal import log

from opendbc.safety import ALTERNATIVE_EXPERIENCE

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.selfdrived.events import EVENTS, ET, Events

EventName = log.OnroadEvent.EventName

# Event types that end lateral authority. NO_ENTRY is deliberately absent: it gates *entering*
# engagement, and several NO_ENTRY-only events (belowEngageSpeed, ...) are true for long stretches
# of ordinary lateral-only driving. Every gear/door/ESP hazard carries SOFT_DISABLE or USER_DISABLE
# as well (wrongGear, reverseGear, doorOpen, seatbeltNotLatched, espActive, parkBrake), so the
# reverse-gear class of hazard is covered by this set — verified by test_blocking_event_coverage.
LATERAL_DISABLE_TYPES = (ET.USER_DISABLE, ET.IMMEDIATE_DISABLE, ET.SOFT_DISABLE)

# The two disabling events that ARE the brake press this feature exists to survive, and are
# therefore tolerated for as long as it lasts:
#   * pedalPressed  — openpilot's own brake/gas disengage event. Its GAS case cannot smuggle
#     lateral through, because the falling edge additionally requires the brake to be down.
#   * pcmDisable    — the stock PCM dropping cruise. car_specific.py re-raises this EVERY frame
#     while `cruiseState.enabled` is False (`elif not CS.cruiseState.enabled`), not just on the
#     falling edge, so tolerating it is what makes the lateral-only state last more than a frame.
# Nothing else is tolerated. An event this fork has not classified is blocking by default.
MADS_TOLERATED_EVENTS = (EventName.pedalPressed, EventName.pcmDisable)

# madsbrakerace2pnw: how long the falling edge waits for `brakePressed` to show up.
#
# MEASURED ON THE TRUCK 2026-09-06, which is why this exists at all. Captured at the disengage:
#     t=286.143  en=1 brk=0 regen=0 cruise=1->0
#     t=286.145  en=0 brk=0 regen=0 cruise=0      <- falling edge, braking STILL False
# On the Lightning the stock-ACC PCM reacts to the pedal and drops cruiseState.enabled FASTER than
# `brakePressed` propagates on CAN, so selfdrived disengages on pcmDisable one or more frames BEFORE
# the brake signal arrives. Judging `braking` on the single falling-edge frame therefore misses the
# brake every time and MADS never armed -- the feature simply did not work on this car.
#
# 0.45 s at 100 Hz. Long enough to cover the observed lead (~2 ms here, but CAN scheduling and a
# gentle pedal press make it variable); short enough that it cannot bridge two unrelated driver
# actions. The window only ever ARMS on a real brake: it requires `braking` to actually become true,
# so a CANCEL-button disengage (no brake) simply lets it expire, and any non-tolerated event in the
# window kills it outright.
# STRICTLY NARROWER THAN THE PANDA'S WINDOW. The panda re-latches within MADS_BRAKE_RELATCH_US
# (300 ms, opendbc/safety/pnw/mads_declarations.h); openpilot waits 250 ms. The ordering is the
# whole invariant: if openpilot's window were the WIDER one it could arm lateral after the panda had
# stopped accepting it, and the truck would sit in "UI says lateral-only, panda blocks every steering
# frame" for the 2.0 s it takes madsControlsMismatchLateral to fire -- worse than a clean disengage.
# That is exactly what the first version of this fix did (Fable review 2026-09-06, BLOCK).
# 45 frames = 450 ms nominal, comfortably inside the panda's 600 ms.
# Sized from the MEASURED lead (321/361 ms in the driver's own logs), not from bus cadence --
# the delay is pedal travel, not CAN transport. 150 ms would have missed every real press.
#
# CORRECTED (Fable review 2026-09-06): the earlier rationale here -- "under CPU contention 25 frames
# could take >300 ms of wall time" -- had the clock backwards. The relevant frame of reference is CAN
# time, not wall time: carState frames are produced from CAN traffic, so a stalled card or selfdrived
# either CONFLATES frames (fewer of them) or bursts them in order. Frame counting can therefore only
# UNDER-measure the CAN-time distance between the cruise-drop frame and the brake frame, never
# over-measure it, and the invariant was already robust to scheduling. 15 is kept anyway: it costs
# nothing, and margin against a shared invariant is cheap insurance.
#
# The error is deliberately ASYMMETRIC, which is why erring short is right:
#   * openpilot window TOO SHORT -> openpilot does not arm, the panda may re-latch, nothing is
#     commanded. The feature just misses that press. Harmless.
#   * openpilot window TOO LONG  -> openpilot shows lateral-only while the panda blocks every
#     steering frame, for the 2.0 s until madsControlsMismatchLateral. Actively bad.
# 150 ms still covers the measured case: the brake lands on the next 10 Hz frame, ~100 ms later.
#
# The fully robust form is to stop racing the panda at all -- gate arming on
# pandaState.controlsAllowedLateral so the two agree by construction. That needs selfdrived to pass
# the panda's view into update() (mads_pnw itself must stay pure), and is the right follow-up.
MADS_BRAKE_GRACE_FRAMES = 45


def has_blocking_event(events: Events) -> bool:
  """True if this frame carries any disabling event other than the brake press itself."""
  for name in events.names:
    if name in MADS_TOLERATED_EVENTS:
      continue
    types = EVENTS.get(name, {})
    if any(et in types for et in LATERAL_DISABLE_TYPES):
      return True
  return False


class MadsPnw:
  """The lateral-authority state machine. Pure: no params, no sockets, no clock."""

  def __init__(self, alternative_experience: int):
    alt = int(alternative_experience)
    self.disengage_on_brake = bool(alt & ALTERNATIVE_EXPERIENCE.MADS_DISENGAGE_LATERAL_ON_BRAKE)
    pause_on_brake = bool(alt & ALTERNATIVE_EXPERIENCE.MADS_PAUSE_LATERAL_ON_BRAKE)

    self.available = bool(alt & ALTERNATIVE_EXPERIENCE.ENABLE_MADS)
    if self.available and pause_on_brake:
      # PAUSE exists in the safety C and is deliberately not exposed by this fork; there is no
      # openpilot-side implementation of it, so honouring ENABLE_MADS while the panda runs a
      # policy we do not model would be exactly the openpilot/panda disagreement this design
      # exists to avoid. Refuse loudly and fall back to stock rather than guess.
      cloudlog.error(f"mads_pnw: MADS_PAUSE_LATERAL_ON_BRAKE is set but not implemented; MADS disabled (alternativeExperience={alt})")
      self.available = False

    # Latched lateral authority — the mirror of the panda's controls_allowed_lateral.
    self.enabled = False
    # Should openpilot command lateral this frame.
    self.active = False
    # Lateral is live while openpilot's own engagement is gone. The state the driver must be told
    # about: it is the only one in which the car steers with `selfdriveState.enabled` False.
    self.lateral_only = False

    self._op_enabled_prev = False
    self._cruise_enabled_prev = False
    # madsbrakerace2pnw: frames left in which a late `brakePressed` may still arm lateral-only.
    self._brake_grace = 0

  @property
  def brake_grace_open(self) -> bool:
    """madsquiet2pnw: inside the brake-race window, i.e. MADS may still arm on a later frame. Read-only."""
    return self._brake_grace > 0

  def update(self, op_enabled: bool, op_active: bool, braking: bool, cruise_enabled: bool,
             events: Events, cruise_available: bool = True, off_requested: bool = False) -> None:
    """Run once per frame, AFTER selfdrived's own state machine has already decided op_enabled.

    op_enabled/op_active: selfdrived's own engagement, untouched by this module.
    braking:              CS.brakePressed or CS.regenBraking — the panda's `is_braking` input.
    cruise_enabled:       CS.cruiseState.enabled — openpilot's view of what drives the panda's
                          own `controls_allowed` on a pcmCruise car. See the revoke check below.
    events:               this frame's events, read only.
    cruise_available:     CS.cruiseState.available — the ACC MASTER switch (CcStat_D_Actl in 3/4/5).
                          onebutton2pnw: the master switch turns EVERYTHING off, lateral included.
    off_requested:        onebutton2pnw: the driver PRESSED the ACC ON/OFF button. Needed as its own
                          input because the resulting STATE is not a usable signal — from Standby
                          the truck never reaches Off (measured; see the carstate comment).
    """
    # The panda sets controls_allowed on the RISING edge of stock cruise engaging. If cruise
    # engages and openpilot does NOT engage with it — a NO_ENTRY is standing (calibration
    # incomplete after a car swap, resumeBlocked, distracted, ...) — controlsd then sends
    # cruiseControl.cancel, stock cruise drops, and the panda sees controls_allowed FALL with the
    # brake up, which REVOKES controls_allowed_lateral. openpilot would see no edge of its own and
    # would keep commanding lateral into a panda that is now blocking it — a silent no-steer with
    # NO detector, because this tree's PandaState has no controls_allowed_lateral field and so
    # there is no lateral mismatch_counter. Track the same rising edge and stand down.
    # (Found by the Fable review, 2026-09-05.)
    cruise_engage_edge = cruise_enabled and not self._cruise_enabled_prev
    self._cruise_enabled_prev = cruise_enabled

    # onebutton2pnw (driver request 2026-09-06): the cruise button is the MASTER, for both halves.
    # Before this, turning ACC off while MADS held lateral left the truck steering with the cruise
    # system switched off -- the driver's "weird three-state state", a hands-free light with no
    # cruise behind it. The button now means what it says: off is off. This is purely
    # de-authorizing, and it is checked before everything else so no later branch can re-arm.
    #
    # TWO inputs, because one is not enough (measured 2026-09-07, see
    # drives/2026-09-07/lightning-onoff-button/). `cruise_available` catches the press when the
    # truck actually reaches Off, which it does reliably from Active. But from STANDBY -- exactly
    # where the truck sits after a brake drops cruise while MADS still steers -- the button never
    # reaches Off at all: of four presses observed there, two did nothing and two turned the system
    # ON. So the press itself (`off_requested`) is the only signal available in the state the
    # driver is actually complaining about.
    if not cruise_available or off_requested:
      self._op_enabled_prev = op_enabled
      self._cruise_enabled_prev = cruise_enabled
      self._brake_grace = 0
      self.enabled = False
      self.active = False
      self.lateral_only = False
      return

    if not self.available:
      # Inert. Hold every output at False so `madsState` can never be mistaken for authority,
      # and keep the edge detector fed so enabling mid-session could never see a stale edge.
      self._op_enabled_prev = op_enabled
      self._brake_grace = 0
      self.enabled = False
      self.active = False
      self.lateral_only = False
      return

    blocked = has_blocking_event(events)

    if op_enabled:
      # openpilot itself holds authority; MADS adds nothing and simply mirrors it. The panda has
      # controls_allowed here, so the (controls_allowed || controls_allowed_lateral) tx gates are
      # satisfied either way.
      self._brake_grace = 0
      self.enabled = True
      self.active = op_active
      self.lateral_only = False
    elif self._op_enabled_prev:
      # THE falling edge. Lateral survives only a brake press, only when the driver asked for it,
      # and only when nothing else in the frame wants openpilot off.
      may_arm = (not self.disengage_on_brake) and not blocked
      self.enabled = may_arm and braking
      # madsbrakerace2pnw: the brake can still be IN FLIGHT on this frame (see
      # MADS_BRAKE_GRACE_FRAMES -- measured on the truck, the PCM drops cruise first). Open a
      # bounded window instead of deciding on one frame's view. Nothing is armed here; the window
      # only lets a LATER frame arm, and only on a real `braking` edge.
      self._brake_grace = 0 if (self.enabled or not may_arm) else MADS_BRAKE_GRACE_FRAMES
      self.active = self.enabled
      self.lateral_only = self.enabled
    else:
      # Already lateral-only (or already fully off). Re-check every frame: releasing the brake
      # does NOT end it (that is the point — the brake took the speed, not the steering), but any
      # other disabling event does — and so does stock cruise engaging without openpilot engaging
      # with it, which is the panda-revokes-first divergence described in update()'s preamble.
      # The RISING edge deliberately: right after a brake press `cruiseState.enabled` can still
      # read True for a frame or two before the PCM drops it, and a level test there would kill
      # the feature on the very frame it arms.
      if blocked or cruise_engage_edge:
        self.enabled = False
        self._brake_grace = 0            # a blocked frame ends the pending window outright
      elif self._brake_grace > 0:
        # madsbrakerace2pnw: still waiting for the brake the PCM already reacted to. Arm the moment
        # it lands; otherwise let the window run out and stay off. `not self.enabled` keeps this to
        # the arming path only -- it can never re-arm authority that a later frame took away.
        self._brake_grace -= 1
        if braking and not self.enabled:
          self.enabled = True
      self.active = self.enabled
      self.lateral_only = self.enabled

    self._op_enabled_prev = op_enabled
