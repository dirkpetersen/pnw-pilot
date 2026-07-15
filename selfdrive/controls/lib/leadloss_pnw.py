"""
leadloss2pnw — SHADOW (log-only) detector for the vision-lead dropout that forces a driver brake.

Field incident (drives/2026-07-14/lightning-left-curve-toofast): on the radar-less Lightning, a close,
slow, CLOSING vision lead lost model confidence in a bend and dropped out; op-long stopped decelerating
and the driver braked hard. This detector flags exactly that pattern — a RECENTLY confident, close,
closing lead that suddenly vanishes — and logs what a real lead-loss-hold WOULD do (briefly hold the
last deceleration). It NEVER actuates; it only emits a `lead_loss_hold_shadow` event so we can validate
the trigger on real drives before building the actuating version. Pure, cheap, no msgq subscriptions
(the caller passes the already-subscribed radarState.leadOne).
"""
from openpilot.common.swaglog import cloudlog

# Retuned from the 2026-07-15 shadow drive (drives/2026-07-15/i5-75mph-icbm-watch): 16 of 22 events
# were distant-lead flicker at 100-120 m / 74 mph. In that data the junk and genuine classes do NOT
# separate on probability (both ~0.5) or time-headway (both ~3.5-5.8 s) — they separate cleanly on
# ABSOLUTE DISTANCE (junk >= 86 m, genuine <= 17 m) and on track stability (the distant tracks were
# flickering in/out). Hence: DREL_MAX 120 -> 50, plus a continuous-presence precondition; PROB_MIN
# deliberately stays at 0.5 (the genuine close dropouts logged prob 0.51-0.54 — a higher floor would
# reject exactly the events this feature exists for).
PROB_MIN = 0.5          # model lead probability that counts as "confident"
DREL_MAX = 50.0         # m — close leads only; distant dropouts leave time to re-acquire (was 120)
VREL_CLOSING = -2.0     # m/s — approaching the lead (negative vRel)
RECENT_FRAMES = 3       # the dropout must follow a good sample within this many frames (~0.15 s @ 20 Hz)
MIN_TRACK_FRAMES = 20   # lead must have been continuously present this long (~1 s @ 20 Hz) — rejects flicker
HOLD_S = 1.5            # duration the real hold would cover (for the shadow log only)


class LeadLossHoldShadow:
  def __init__(self):
    self._prev_present = False
    self._good = None       # snapshot of the last confident-close-closing lead
    self._since_good = 999
    self._present_frames = 0  # consecutive frames the lead has been present (track stability)

  def update(self, lead, v_ego: float, a_ego: float) -> None:
    """lead = radarState.leadOne. Logs a shadow event on a qualifying dropout; never returns a command."""
    try:
      present = bool(lead.status)
      prob = float(lead.modelProb)
      dRel = float(lead.dRel)
      vRel = float(lead.vRel)
      vLead = float(lead.vLead)
    except Exception:
      self._prev_present = False
      self._present_frames = 0
      return

    # track stability: count consecutive present frames; a flickering distant track never accumulates
    track_frames = self._present_frames  # value BEFORE this cycle's dropout resets it
    self._present_frames = self._present_frames + 1 if present else 0

    good = present and prob >= PROB_MIN and 0.0 < dRel < DREL_MAX and vRel <= VREL_CLOSING
    if good:
      self._good = (dRel, vRel, vLead, prob)
      self._since_good = 0
    else:
      self._since_good += 1

    # qualifying dropout: present last cycle, gone now, a good lead seen within RECENT_FRAMES,
    # and the track had been continuously present long enough to be real (not range flicker)
    if (self._prev_present and not present and self._good is not None
        and self._since_good <= RECENT_FRAMES and track_frames >= MIN_TRACK_FRAMES):
      dRel, vRel, vLead, prob = self._good
      cloudlog.event("lead_loss_hold_shadow", would_hold_s=HOLD_S, at_dRel=round(dRel, 1),
                     vRel=round(vRel, 2), vLead=round(vLead, 2), prob=round(prob, 2),
                     v_ego=round(v_ego, 2), a_ego=round(a_ego, 2),
                     track_frames=int(track_frames), error=False)
      self._good = None       # one event per dropout episode

    self._prev_present = present
