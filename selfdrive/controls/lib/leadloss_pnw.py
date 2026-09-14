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
import time

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

# leadlossgate2pnw (2026-09-14): three more gates on the qualifying dropout, from the review of 69 shadow events
# (drives/2026-09-14/leadloss-shadow-review/DRIVE_REPORT.md). On the 43 events with context they keep all 6 where a
# hold would have helped and drop 6 of the 7 where it would have hurt (#30, a steady-speed turn, still passes).
# BOTH MARGINS ARE THIN -- the slowest helpful event was 5.36 m/s, the largest helpful TTC 7.8 s -- and there are only
# 6 helpful events, so these are starting points, not validated thresholds. Every would-be event a gate rejects is
# logged as `lead_loss_hold_shadow_rejected`, naming the gate(s), so the next review can see the near-misses.
# PROB_MIN is deliberately NOT raised: radard only reports a lead while its filtered prob is above 0.5, so the last good
# frame always logs ~0.50-0.67 and a higher floor would reject every genuine event too.
V_EGO_MIN = 5.0         # m/s at the dropout -- rejects launches from a stop, creep and standstill
TTC_MAX = 8.0           # s, dRel / -vRel of the last good frame -- rejects departing leads and ramp-curve flicker
# (third gate: the carState message must be valid -- rejects the frozen-carState / canBusMissing events)
REJECT_LOG_MIN_S = 5.0  # at most one rejected-event line per this many seconds; the ones held back are counted in it
BAD_LEAD_LOG_S = 60.0   # unreadable lead fields: logged at once, then at most one line per this many seconds


class LeadLossHoldShadow:
  def __init__(self):
    self._prev_present = False
    self._good = None       # snapshot of the last confident-close-closing lead
    self._since_good = 999
    self._present_frames = 0  # consecutive frames the lead has been present (track stability)
    self._rej_t = None        # monotonic time of the last rejected-event line (None = never)
    self._rej_held = {}       # rejected events held back by REJECT_LOG_MIN_S since that line: {"gate,gate": n}
    self._bad_lead_t = None   # monotonic time of the last unreadable-lead line (None = never)
    self._bad_lead_n = 0      # unreadable-lead frames since that line

  def update(self, lead, v_ego: float, a_ego: float, carstate_valid: bool) -> None:
    """lead = radarState.leadOne, carstate_valid = the carState message's valid flag. Logs a shadow event on a
    qualifying dropout; never returns a command."""
    try:
      present = bool(lead.status)
      prob = float(lead.modelProb)
      dRel = float(lead.dRel)
      vRel = float(lead.vRel)
      vLead = float(lead.vLead)
    except (AttributeError, TypeError, ValueError):
      # leadlossgate2pnw (Rule 2): an unreadable lead still counts as "no lead" (it breaks the track, so it can never
      # fire an event), but it is no longer silent: while it lasts the shadow cannot see a dropout at all. Anything
      # else propagates to the planner's guard, which logs it and keeps the planner running.
      self._bad_lead_n += 1
      now = time.monotonic()
      if self._bad_lead_t is None or now - self._bad_lead_t >= BAD_LEAD_LOG_S:
        cloudlog.exception("leadloss2pnw: radarState.leadOne unreadable -- treated as no lead, so lead_loss_hold_shadow " +
                           f"cannot fire ({self._bad_lead_n} unreadable frame(s) since the last log)")
        self._bad_lead_t = now
        self._bad_lead_n = 0
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
      self._good = None       # one event per dropout episode (accepted or rejected)
      ttc = dRel / max(-vRel, 1e-3)   # the good snapshot always has vRel <= VREL_CLOSING < 0
      # each gate is written as "passes", so a NaN fails it instead of slipping through
      rejected = [gate for gate, ok in (("v_ego", v_ego >= V_EGO_MIN), ("ttc", ttc <= TTC_MAX),
                                        ("carstate_invalid", bool(carstate_valid))) if not ok]
      fields = dict(at_dRel=round(dRel, 1), vRel=round(vRel, 2), vLead=round(vLead, 2), prob=round(prob, 2),
                    v_ego=round(v_ego, 2), a_ego=round(a_ego, 2), ttc=round(ttc, 1), track_frames=int(track_frames),
                    carstate_valid=bool(carstate_valid))
      # error=False still logs at ERROR level ('error' in kwargs), which is what puts both events in every qlog
      if not rejected:
        cloudlog.event("lead_loss_hold_shadow", would_hold_s=HOLD_S, **fields, error=False)
      else:
        self._log_rejected(",".join(rejected), fields)

    self._prev_present = present

  def _log_rejected(self, gates: str, fields: dict) -> None:
    """A dropout that passed the old trigger but failed a gate: log it, at most once per REJECT_LOG_MIN_S. Held-back
    ones are counted by gate combination in the next line's `held`, so a burst still shows which gates fired."""
    now = time.monotonic()
    if self._rej_t is not None and now - self._rej_t < REJECT_LOG_MIN_S:
      self._rej_held[gates] = self._rej_held.get(gates, 0) + 1
      return
    cloudlog.event("lead_loss_hold_shadow_rejected", rejected_by=gates, held=self._rej_held, **fields, error=False)
    self._rej_t = now
    self._rej_held = {}
