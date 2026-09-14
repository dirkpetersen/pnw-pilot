"""madsquiet2pnw — no engagement chimes while MADS keeps steering; a FULL disengage still chimes.

DRIVER REQUEST, 2026-09-13: "if I'm running under MADS and I hit the brake I get the signal that steering
is enabled and cruise control is not. Can we make it so that this signal only shows up as a visual signal
on my device and not as audio, because when I go through like 6 traffic lights the constant engaging and
disabling audio is very annoying." Confirmed: "we only want the silence while MADS keeps steering. When I
disengage fully it can still chime."

WHAT MAKES THE SOUND. Not the yellow "Steering only" banner -- madsLateralOnly is already
AudibleAlert.none. The chimes are stock openpilot's engagement alerts: pedalPressed / pcmDisable carry
ET.USER_DISABLE -> EngagementAlert(disengage) on the brake, and pcmEnable / buttonEnable carry
ET.ENABLE -> EngagementAlert(engage) when cruise resumes. Twelve chimes over six traffic lights.

WHY THIS CANNOT BE DECIDED ON ONE FRAME. madsbrakerace2pnw measured that the truck's PCM drops cruise
BEFORE the brake signal lands: on the very frame openpilot disengages and creates the chime, MADS has
often not yet armed -- it opens a MADS_BRAKE_GRACE_FRAMES window and arms a few frames later. Deciding
the sound from that frame alone would miss exactly the traffic-light case. So a disengage that MADS may
still take over is silenced PROVISIONALLY; if the window runs out without MADS arming, it was a full
disengage after all and the chime plays then -- at most MADS_BRAKE_GRACE_FRAMES late (0.45 s).

KEEPING "A FULL DISENGAGE STILL CHIMES" TRUE. When MADS stops steering on its own (ACC main off,
reverse, a cancel, stock cruise engaging without openpilot) openpilot is already in `disabled`, whose
alert types do not display those events' disengage alerts -- so today nothing chimes at that moment.
Once the brake chime is silenced that would leave a brake -> MADS -> steering-off sequence with NO
audible disengage at all. So the moment MADS stops steering without openpilot re-engaging, this module
asks for the disengage chime.

Pure: no params, no sockets, no clock. selfdrived applies the decision to the alert list and falls back
to the unmodified (audible) alerts if anything here raises.
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class ChimeDecision:
  quiet_disengage: bool = False   # strip the disengage sound from this frame's brake USER_DISABLE alerts
  quiet_engage: bool = False      # strip the engage sound from this frame's ENABLE alerts
  chime_disengage: bool = False   # play a disengage chime this frame (a full disengage, confirmed)


class MadsQuiet:
  def __init__(self, grace_frames: int):
    self._grace_frames = int(grace_frames)
    self._en_prev = False
    self._lat_prev = False
    self._pending = 0   # frames left for a provisionally silenced disengage to become a MADS takeover

  def step(self, enabled: bool, lateral_only: bool, mads_available: bool, brake_grace_open: bool) -> ChimeDecision:
    """Call once per frame AFTER both selfdrived's state machine and MadsPnw.update have run.

    enabled:          selfdrived's own engagement this frame
    lateral_only:     MadsPnw.lateral_only this frame (MADS is steering on its own)
    mads_available:   MadsPnw.available (the panda-honoured capability; False on the Tesla)
    brake_grace_open: MadsPnw is inside its brake-race window, i.e. may still arm on a later frame
    """
    enabled, lateral_only = bool(enabled), bool(lateral_only)
    disengage_edge = self._en_prev and not enabled
    engage_edge = enabled and not self._en_prev
    lat_prev = self._lat_prev
    self._en_prev, self._lat_prev = enabled, lateral_only

    if not mads_available:
      self._pending = 0
      return ChimeDecision()                          # no MADS -> stock chimes, exactly as before

    if disengage_edge:
      if lateral_only:
        self._pending = 0
        return ChimeDecision(quiet_disengage=True)    # MADS already steering: silent
      if brake_grace_open:
        self._pending = self._grace_frames + 2        # MADS may still arm: silent for now, decide later
        return ChimeDecision(quiet_disengage=True)
      self._pending = 0
      return ChimeDecision()                          # MADS will not steer: a full disengage, chime

    if engage_edge:
      self._pending = 0
      return ChimeDecision(quiet_engage=lat_prev)     # resuming from steering-only: silent

    if self._pending > 0:
      if lateral_only:
        self._pending = 0                             # MADS took over: the silence stands
        return ChimeDecision()
      self._pending -= 1
      if self._pending == 0:
        return ChimeDecision(chime_disengage=True)    # the window closed without MADS: full disengage

    if lat_prev and not lateral_only and not enabled:
      return ChimeDecision(chime_disengage=True)      # MADS stopped steering, nobody took over

    return ChimeDecision()


# ---- applying the decision --------------------------------------------------------------------------
# Imported here, not at the top, so the decision logic above stays importable without the events table.
QUIET_DISENGAGE_EVENTS = ("pedalPressed", "pcmDisable")   # == mads_pnw.MADS_TOLERATED_EVENTS, by name
QUIET_ENGAGE_EVENTS = ("pcmEnable", "buttonEnable")
FULL_DISENGAGE_ALERT_TYPE = "madsQuietFullDisengage/userDisable"


def apply_chime_decision(alerts: list, decision: ChimeDecision) -> list:
  """Return the alert list with the decision applied. NEVER mutates an alert in place.

  Events.create_alerts hands out the SHARED Alert instances from the module-level EVENTS table (it
  even assigns alert_type on them). Silencing one by editing it would silence that event for the rest
  of the process lifetime, on every later frame and in every unrelated situation. A silenced alert is
  therefore a shallow copy with its sound removed, under the same alert_type, so the AlertManager keeps
  treating it as the same alert."""
  import copy
  from openpilot.selfdrive.selfdrived.events import ET, AudibleAlert, EngagementAlert

  def _named(alert, names):
    return str(getattr(alert, "alert_type", "")).split("/", 1)[0] in names

  out = []
  for a in alerts:
    if (decision.quiet_disengage and a.event_type == ET.USER_DISABLE and _named(a, QUIET_DISENGAGE_EVENTS)
        and a.audible_alert == AudibleAlert.disengage):
      a = copy.copy(a)
      a.audible_alert = AudibleAlert.none
    elif (decision.quiet_engage and a.event_type == ET.ENABLE and _named(a, QUIET_ENGAGE_EVENTS)
          and a.audible_alert == AudibleAlert.engage):
      a = copy.copy(a)
      a.audible_alert = AudibleAlert.none
    out.append(a)
  if decision.chime_disengage:
    chime = EngagementAlert(AudibleAlert.disengage)
    chime.alert_type = FULL_DISENGAGE_ALERT_TYPE
    chime.event_type = ET.USER_DISABLE
    out.append(chime)
  return out
