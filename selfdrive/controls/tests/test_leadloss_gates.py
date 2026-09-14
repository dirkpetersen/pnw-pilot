"""leadlossgate2pnw -- the three shadow-trigger gates (speed, time-to-contact, valid carState), replayed on the 69
reviewed shadow events of drives/2026-09-14/leadloss-shadow-review (DRIVE_REPORT.md + classification.md).

Every row below is one event as the device logged it: the last good frame (dRel, vRel, vLead, prob), v_ego/a_ego at the
drop, and how long the track had been present. Replaying it through the detector must reproduce the report's
"proposed gate" column exactly, and every event a gate drops must be logged with the gate(s) that dropped it.
"""
import logging
import types

import pytest

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib import leadloss_pnw as ll

# (id, dRel, vRel, vLead, prob, v_ego, a_ego, track_frames, carState valid, 1.5 s hold verdict, report says keep)
# verdict: helps / harm / neutral / na (standstill or frozen carState) / unknown (Tesla, payload only)
EVENTS = [
  (1, 32.4, -3.74, 5.51, 0.51, 9.46, -2.01, 716, True, 'unknown', False),
  (2, 28.3, -2.0, 10.09, 0.51, 11.88, -2.04, 484, True, 'neutral', False),
  (3, 27.1, -8.69, 0.54, 0.54, 9.02, -2.04, 85, True, 'unknown', True),
  (4, 11.9, -2.31, 3.72, 0.54, 5.95, -0.84, 416, True, 'unknown', True),
  (5, 13.5, -2.5, 5.28, 0.56, 7.69, -0.96, 220, True, 'unknown', True),
  (6, 13.4, -2.65, 6.15, 0.51, 8.0, -0.67, 105, True, 'unknown', True),
  (7, 15.8, -4.25, 2.41, 0.61, 6.58, -0.84, 94, True, 'unknown', True),
  (8, 44.1, -4.11, 14.23, 0.54, 17.59, -1.5, 766, True, 'unknown', False),
  (9, 13.1, -2.4, 4.35, 0.51, 6.9, -1.59, 720, True, 'unknown', True),
  (10, 49.7, -8.25, -0.35, 0.52, 7.68, -2.14, 119, True, 'unknown', True),
  (11, 29.3, -2.79, 1.86, 0.53, 3.46, 0.06, 21, True, 'unknown', False),
  (12, 26.0, -2.34, 1.41, 0.57, 3.61, 0.0, 26, True, 'unknown', False),
  (13, 14.2, -2.31, -0.24, 0.51, 2.02, -0.77, 22, True, 'unknown', False),
  (14, 4.9, -2.04, 0.2, 0.51, 2.3, 0.03, 32, True, 'unknown', False),
  (15, 13.5, -2.19, 0.06, 0.51, 2.23, -0.11, 22, True, 'unknown', False),
  (16, 43.8, -6.94, 3.46, 0.54, 10.36, -0.35, 73, True, 'unknown', True),
  (17, 14.8, -2.37, 7.56, 0.52, 6.89, -3.22, 143, True, 'unknown', True),
  (18, 17.1, -5.33, 4.78, 0.52, 9.19, -1.61, 1055, True, 'unknown', True),
  (19, 6.7, -3.47, 1.37, 0.5, 3.34, 0.56, 83, True, 'unknown', False),
  (20, 38.7, -7.08, 2.39, 0.51, 8.77, -0.9, 1910, True, 'unknown', True),
  (21, 37.8, -5.44, 2.85, 0.54, 8.28, 0.01, 32, True, 'unknown', True),
  (22, 29.1, -7.25, 0.1, 0.51, 7.35, -0.03, 1017, True, 'unknown', True),
  (23, 10.8, -2.55, 4.82, 0.6, 7.18, -2.96, 1037, True, 'unknown', True),
  (24, 27.7, -6.06, -6.06, 0.51, 0.0, 0.0, 23, True, 'na', False),
  (25, 29.9, -2.04, 0.15, 0.53, 1.97, -0.18, 1060, True, 'unknown', False),
  (26, 16.2, -3.58, 1.87, 0.53, 4.53, 1.2, 5251, True, 'unknown', False),
  (27, 8.1, -6.0, -0.36, 0.5, 5.75, 1.03, 25, True, 'unknown', True),
  (28, 28.9, -8.57, 3.3, 0.53, 11.23, -1.24, 36, True, 'neutral', True),
  (29, 42.0, -4.86, 9.3, 0.54, 12.03, -0.72, 74, True, 'neutral', False),
  (30, 23.1, -3.94, 5.99, 0.54, 8.83, -0.38, 3192, True, 'harm', True),
  (31, 35.5, -6.43, 2.01, 0.51, 8.63, -1.14, 2374, True, 'neutral', True),
  (32, 48.4, -4.96, 0.31, 0.52, 6.0, -0.43, 89, True, 'harm', False),
  (33, 41.8, -7.93, 3.56, 0.5, 11.14, -1.63, 268, True, 'helps', True),
  (34, 39.6, -4.0, 3.34, 0.51, 6.73, -1.03, 35, True, 'neutral', False),
  (35, 29.2, -4.71, 0.58, 0.51, 5.36, -0.52, 300, True, 'helps', True),
  (36, 48.9, -8.05, 0.99, 0.52, 9.62, -0.39, 530, True, 'neutral', True),
  (37, 34.6, -7.07, 1.37, 0.5, 8.63, -0.77, 26, True, 'neutral', True),
  (38, 17.7, -2.58, 0.96, 0.53, 2.73, 0.22, 21, True, 'neutral', False),
  (39, 11.0, -2.25, -0.12, 0.52, 2.0, -0.03, 44, True, 'neutral', False),
  (40, 5.9, -2.1, 2.3, 0.57, -0.0, 0.0, 64, True, 'na', False),
  (41, 7.8, -3.55, -2.12, 0.52, 0.0, -0.0, 140, True, 'na', False),
  (42, 9.0, -6.9, -1.3, 0.52, 0.0, 0.0, 217, True, 'na', False),
  (43, 36.8, -3.58, 4.36, 0.53, 8.58, -1.03, 25, True, 'neutral', False),
  (44, 43.2, -2.17, 12.58, 0.55, 15.13, -0.0, 1992, True, 'neutral', False),
  (45, 20.0, -2.59, 1.45, 0.51, 4.15, 0.57, 840, True, 'neutral', False),
  (46, 48.8, -9.25, 2.38, 0.53, 10.62, -1.73, 145, True, 'neutral', True),
  (47, 35.6, -2.21, 8.64, 0.51, 10.63, -0.23, 77, True, 'neutral', False),
  (48, 34.1, -2.41, 5.6, 0.6, 8.29, -0.08, 33, True, 'neutral', False),
  (49, 32.8, -7.63, 2.79, 0.56, 10.84, -0.11, 35, True, 'helps', True),
  (50, 10.8, -2.71, 0.4, 0.53, 3.42, -0.0, 76, False, 'na', False),
  (51, 31.0, -2.98, 3.32, 0.54, 6.74, -0.27, 186, True, 'neutral', False),
  (52, 37.4, -5.12, 5.75, 0.52, 7.29, 0.09, 25, True, 'helps', True),
  (53, 24.2, -3.93, 0.53, 0.53, 4.27, -0.37, 71, True, 'neutral', False),
  (54, 23.1, -2.21, 4.12, 0.55, 3.52, -0.74, 247, True, 'harm', False),
  (55, 41.6, -4.14, 4.69, 0.52, 7.35, -1.23, 72, True, 'neutral', False),
  (56, 49.6, -9.76, 0.26, 0.51, 9.94, -1.1, 46, True, 'helps', True),
  (57, 34.1, -6.33, 0.88, 0.53, 7.65, -2.03, 577, True, 'neutral', True),
  (58, 21.5, -3.33, 1.27, 0.54, 3.92, 0.41, 877, True, 'harm', False),
  (59, 43.4, -2.23, 2.59, 0.52, 3.28, 0.88, 2459, True, 'harm', False),
  (60, 16.4, -2.66, 0.21, 0.5, 2.93, -0.69, 1209, True, 'neutral', False),
  (61, 10.1, -2.47, 2.21, 0.52, 3.42, -0.06, 345, True, 'harm', False),
  (62, 33.5, -3.84, 9.42, 0.5, 14.24, -1.09, 738, True, 'neutral', False),
  (63, 37.5, -6.54, 4.39, 0.52, 10.14, -0.0, 21, False, 'na', False),
  (64, 39.0, -5.0, 6.55, 0.52, 8.85, -0.49, 2776, True, 'helps', True),
  (65, 40.7, -7.41, 3.63, 0.51, 9.72, -1.63, 122, True, 'neutral', True),
  (66, 30.8, -4.43, 4.77, 0.54, 6.56, -1.53, 593, True, 'neutral', True),
  (67, 37.6, -4.35, -0.97, 0.52, 3.42, 0.0, 373, False, 'na', False),
  (68, 49.2, -9.87, 10.14, 0.67, 19.96, -2.98, 656, True, 'neutral', True),
  (69, 12.7, -2.92, 3.35, 0.53, 4.15, 0.44, 1137, True, 'harm', False),
]


class _Events(logging.Handler):
  def __init__(self):
    super().__init__(level=logging.DEBUG)
    self.records = []

  def emit(self, record):
    self.records.append(record)

  def named(self, name):
    return [r.msg for r in self.records if isinstance(r.msg, dict) and r.msg.get("event") == name]

  def messages(self, needle):
    return [r for r in self.records if isinstance(r.msg, str) and needle in r.msg]


@pytest.fixture
def logs():
  h = _Events()
  cloudlog.addHandler(h)
  yield h
  cloudlog.removeHandler(h)


@pytest.fixture
def clock(monkeypatch):
  t = [0.0]
  monkeypatch.setattr(ll, "time", types.SimpleNamespace(monotonic=lambda: t[0]))
  return t


def _lead(status, dRel=0.0, vRel=0.0, vLead=0.0, prob=0.0):
  return types.SimpleNamespace(status=status, dRel=dRel, vRel=vRel, vLead=vLead, modelProb=prob)


def _replay(det, dRel, vRel, vLead, prob, v_ego, a_ego, track_frames, valid):
  """The track is present for `track_frames` frames at the logged last-good values, then drops out."""
  good = _lead(True, dRel, vRel, vLead, prob)
  for _ in range(track_frames):
    det.update(good, v_ego, a_ego, valid)
  det.update(_lead(False), v_ego, a_ego, valid)


def _expected_gates(dRel, vRel, v_ego, valid):
  """The report's gate, as classify.py computes it, spelled out independently of the detector."""
  ttc = dRel / max(-vRel, 1e-3)
  return [g for g, bad in (("v_ego", not v_ego >= 5.0), ("ttc", not ttc <= 8.0), ("carstate_invalid", not valid)) if bad]


def test_thresholds_are_the_reviewed_ones():
  assert (ll.V_EGO_MIN, ll.TTC_MAX) == (5.0, 8.0)
  assert ll.PROB_MIN == 0.5          # the review: do NOT raise it (logged prob is always 0.50-0.67)
  assert len(EVENTS) == 69


def test_replay_reproduces_the_recommendation(logs, clock):
  kept, rejected = set(), {}
  for (n, dRel, vRel, vLead, prob, v_ego, a_ego, frames, valid, _verdict, _keep) in EVENTS:
    before_ok, before_rej = len(logs.named("lead_loss_hold_shadow")), len(logs.named("lead_loss_hold_shadow_rejected"))
    _replay(ll.LeadLossHoldShadow(), dRel, vRel, vLead, prob, v_ego, a_ego, frames, valid)
    ok = logs.named("lead_loss_hold_shadow")[before_ok:]
    rej = logs.named("lead_loss_hold_shadow_rejected")[before_rej:]
    assert len(ok) + len(rej) == 1, f"#{n}: every reviewed event must still reach the gates exactly once"
    if ok:
      kept.add(n)
    else:
      rejected[n] = rej[0]

  by_id = {e[0]: e for e in EVENTS}
  assert kept == {e[0] for e in EVENTS if e[10]}      # the report's "proposed gate" column, row for row

  helps = {n for n, e in by_id.items() if e[9] == "helps"}
  harm = {n for n, e in by_id.items() if e[9] == "harm"}
  assert helps == {33, 35, 49, 52, 56, 64} and helps <= kept           # all 6 useful holds kept
  assert len(harm) == 7 and harm & kept == {30}                         # 6 of 7 harmful dropped; #30 survives
  assert {50, 63, 67}.isdisjoint(kept)                                  # frozen carState
  assert len({n for n in kept if n <= 27 and by_id[n][9] == "unknown"}) == 15   # Tesla payload-only: 15 of 26

  for n, rec in rejected.items():
    _, dRel, vRel, vLead, prob, v_ego, a_ego, frames, valid, _, _ = by_id[n]
    assert rec["rejected_by"] == ",".join(_expected_gates(dRel, vRel, v_ego, valid)), f"#{n}"
    assert rec["v_ego"] == round(v_ego, 2) and rec["at_dRel"] == round(dRel, 1) and rec["carstate_valid"] == valid
  assert rejected[63]["rejected_by"] == "carstate_invalid"   # passes speed and TTC: only the carState gate stops it
  assert rejected[32]["rejected_by"] == "ttc"
  assert rejected[58]["rejected_by"] == "v_ego"       # a launch from a stop, TTC 6.5 s
  assert rejected[67]["rejected_by"] == "v_ego,ttc,carstate_invalid"


@pytest.mark.parametrize("v_ego, keep", [(5.0, True), (4.99, False), (float("nan"), False)])
def test_speed_gate_boundary(logs, clock, v_ego, keep):
  _replay(ll.LeadLossHoldShadow(), 30.0, -5.0, 3.0, 0.6, v_ego, 0.0, 25, True)   # TTC 6 s
  assert len(logs.named("lead_loss_hold_shadow")) == int(keep)
  assert len(logs.named("lead_loss_hold_shadow_rejected")) == int(not keep)


@pytest.mark.parametrize("dRel, keep", [(40.0, True), (40.05, False)])
def test_ttc_gate_boundary(logs, clock, dRel, keep):
  _replay(ll.LeadLossHoldShadow(), dRel, -5.0, 3.0, 0.6, 10.0, 0.0, 25, True)    # TTC 8.0 s / 8.01 s
  assert len(logs.named("lead_loss_hold_shadow")) == int(keep)
  assert len(logs.named("lead_loss_hold_shadow_rejected")) == int(not keep)


def test_rejected_lines_are_rate_limited_and_count_what_they_held_back(logs, clock):
  det = ll.LeadLossHoldShadow()
  _replay(det, 30.0, -5.0, 3.0, 0.6, 3.0, 0.0, 25, True)          # v_ego -> logged at once (t=0)
  clock[0] = 2.0
  _replay(det, 30.0, -2.5, 3.0, 0.6, 10.0, 0.0, 25, True)         # ttc 12 s -> held back
  clock[0] = 4.9
  _replay(det, 30.0, -2.5, 3.0, 0.6, 3.0, 0.0, 25, True)          # v_ego + ttc -> held back
  lines = logs.named("lead_loss_hold_shadow_rejected")
  assert len(lines) == 1 and lines[0]["rejected_by"] == "v_ego" and lines[0]["held"] == {}

  clock[0] = 5.0
  _replay(det, 30.0, -5.0, 3.0, 0.6, 10.0, 0.0, 25, False)        # carState -> a line again, carrying the two held
  lines = logs.named("lead_loss_hold_shadow_rejected")
  assert len(lines) == 2
  assert lines[1]["rejected_by"] == "carstate_invalid"
  assert lines[1]["held"] == {"ttc": 1, "v_ego,ttc": 1}

  _replay(det, 30.0, -5.0, 3.0, 0.6, 10.0, 0.0, 25, True)         # an accepted event is never held back
  assert len(logs.named("lead_loss_hold_shadow")) == 1


def test_unreadable_lead_is_logged_once_then_rate_limited(logs, clock):
  det = ll.LeadLossHoldShadow()
  for _ in range(5):
    det.update(None, 10.0, 0.0, True)                              # AttributeError: no .status
  lines = logs.messages("radarState.leadOne unreadable")
  assert len(lines) == 1 and lines[0].exc_info is not None and "(1 unreadable frame(s)" in lines[0].msg

  clock[0] = 59.9
  det.update(_lead("x", prob="not-a-number"), 10.0, 0.0, True)    # ValueError
  assert len(logs.messages("radarState.leadOne unreadable")) == 1
  clock[0] = 60.0
  det.update(types.SimpleNamespace(status=True, modelProb=None, dRel=1, vRel=1, vLead=1), 10.0, 0.0, True)  # TypeError
  lines = logs.messages("radarState.leadOne unreadable")
  assert len(lines) == 2 and "(6 unreadable frame(s)" in lines[1].msg


def test_unreadable_lead_still_breaks_the_track(logs, clock):
  """Today's behavior is kept: a bad frame counts as no lead, so the dropout that follows it cannot fire."""
  det = ll.LeadLossHoldShadow()
  good = _lead(True, 30.0, -5.0, 3.0, 0.6)
  for _ in range(30):
    det.update(good, 10.0, 0.0, True)
  det.update(None, 10.0, 0.0, True)
  det.update(_lead(False), 10.0, 0.0, True)
  assert logs.named("lead_loss_hold_shadow") == [] and logs.named("lead_loss_hold_shadow_rejected") == []


def test_other_errors_are_not_swallowed_here(clock):
  """Only malformed-field errors are handled in the detector; anything else reaches the planner's logged guard."""
  class _Exploding:
    @property
    def status(self):
      raise RuntimeError("boom")
  with pytest.raises(RuntimeError):
    ll.LeadLossHoldShadow().update(_Exploding(), 10.0, 0.0, True)

