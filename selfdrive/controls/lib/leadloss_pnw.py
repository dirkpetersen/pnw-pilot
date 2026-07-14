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

PROB_MIN = 0.5          # model lead probability that counts as "confident"
DREL_MAX = 120.0        # m — only a reasonably close lead
VREL_CLOSING = -2.0     # m/s — approaching the lead (negative vRel)
RECENT_FRAMES = 3       # the dropout must follow a good sample within this many frames (~0.15 s @ 20 Hz)
HOLD_S = 1.5            # duration the real hold would cover (for the shadow log only)


class LeadLossHoldShadow:
  def __init__(self):
    self._prev_present = False
    self._good = None       # snapshot of the last confident-close-closing lead
    self._since_good = 999

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
      return

    good = present and prob >= PROB_MIN and 0.0 < dRel < DREL_MAX and vRel <= VREL_CLOSING
    if good:
      self._good = (dRel, vRel, vLead, prob)
      self._since_good = 0
    else:
      self._since_good += 1

    # qualifying dropout: present last cycle, gone now, and a good lead was seen within RECENT_FRAMES
    if self._prev_present and not present and self._good is not None and self._since_good <= RECENT_FRAMES:
      dRel, vRel, vLead, prob = self._good
      cloudlog.event("lead_loss_hold_shadow", would_hold_s=HOLD_S, at_dRel=round(dRel, 1),
                     vRel=round(vRel, 2), vLead=round(vLead, 2), prob=round(prob, 2),
                     v_ego=round(v_ego, 2), a_ego=round(a_ego, 2), error=False)
      self._good = None       # one event per dropout episode

    self._prev_present = present
