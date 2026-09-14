"""engagegoal2pnw -- the owner's engagement goal (2026-09-13), replayed on the truck's own 100 Hz inputs.

  "If I hit the cruise control button and anything is on, it needs to be off. If I want to resume
   longitudinal control and accelerate I can either push the + or I should be able to hit the gas pedal
   once and then it should overwrite this and set the new speed."

The fixture (data/engage_weekend_2026-09-11.json) is two windows cut from the Friday 2026-09-11 PT rlogs of
the Lightning (3devpnw eedddfa617): per carState frame, exactly what selfdrived handed MadsResumeBrain and
MadsPnw -- recorded madsState, selfdriveState, carState, radarState.leadOne, onroadEvents -- plus the
madsResume records the TRUCK itself wrote in that window. So the brain is compared against ground truth,
not against a copy of itself. Analysis: docs/MADS-RESUME-TO-DRIVER-SPEED.md section 10 (workbench root).

selfdrived itself cannot be imported on the dev host; its per-frame order is reproduced below and the
state machine, MadsPnw, MadsQuiet, Events, AlertManager and the brain are all the real ones.
"""
import json
import pathlib

import pytest

from cereal import log
from opendbc.safety import ALTERNATIVE_EXPERIENCE
from openpilot.selfdrive.controls.lib.madsresume_pnw import MadsResumeBrain, ResumeInputs
from openpilot.selfdrive.selfdrived.alertmanager import AlertManager
from openpilot.selfdrive.selfdrived.events import ET, AudibleAlert, Events
from openpilot.selfdrive.selfdrived.mads_pnw import MadsPnw, MADS_BRAKE_GRACE_FRAMES, has_blocking_event
from openpilot.selfdrive.selfdrived.madsquiet_pnw import MadsQuiet, apply_chime_decision
from openpilot.selfdrive.selfdrived.state import StateMachine
from openpilot.selfdrive.selfdrived.tests.test_madsquiet_pnw import _safe_alerts

EventName = log.OnroadEvent.EventName
FIXTURE = pathlib.Path(__file__).parent / "data" / "engage_weekend_2026-09-11.json"


def _window(name):
  data = json.loads(FIXTURE.read_text())
  w = next(x for x in data["windows"] if x["name"] == name)
  n, c = w["frames"], w["columns"]
  assert n > 0 and len(c["t_ms"]) == n and len(c["v_cms"]) == n, "fixture is truncated"

  def expand(key):
    out, cur, j = [], None, 0
    for f in range(n):
      while j < len(c[key]) and c[key][j][0] == f:
        cur = c[key][j][1]
        j += 1
      out.append(cur)
    return out

  cols = {k: expand(k) for k in c if k not in ("t_ms", "v_cms")}
  frames = []
  for f in range(n):
    fr = {k: cols[k][f] for k in cols}
    fr["t"] = c["t_ms"][f] / 1000.0
    fr["v"] = c["v_cms"][f] / 100.0
    frames.append(fr)
  return frames, w["truck_records"]


def _events(names):
  e = Events()
  for n in names:
    e.add(getattr(EventName, n))
  return e


def _replay_brain(frames):
  brain = MadsResumeBrain()
  recs = []
  for fr in frames:
    lead = fr["lead"]
    out = brain.update(ResumeInputs(
      now=fr["t"], mads_available=fr["av"], lateral_only=fr["lo"], op_enabled=fr["en"],
      blocked=has_blocking_event(_events(fr["events"])), engageable=fr["engbl"],
      brake_pressed=fr["bp"], regen_braking=False, gas_pressed=fr["gas"],
      cruise_enabled=fr["ccEn"], cruise_available=fr["ccAv"], set_speed_ms=fr["set_cms"] / 100.0,
      v_ego=fr["v"], standstill=fr["stst"], has_lead=lead,
      d_rel=fr["dRel_dm"] / 10.0 if lead else None, v_lead=fr["vLead_cms"] / 100.0 if lead else None))
    recs.extend((fr["t"], r) for r in out.records)
  return recs


def _assert_matches_truck(recs, truck, extra_phases=("lift",)):
  """Every record the truck wrote is reproduced, in order, with the same phase/reason/mode, within 0.3 s
  (ces_events stamps wall time rounded to 0.1 s). Records of `extra_phases` -- ones this branch ADDS -- are
  the only ones allowed beyond the truck's."""
  core = [(t, r) for t, r in recs if r["phase"] not in extra_phases]
  got = [(r["phase"], r.get("reason"), r["mode"]) for _, r in core]
  want = [(r["phase"], r["reason"], r["mode"]) for r in truck]
  assert got == want, f"replay diverged from what the truck wrote:\n got  {got}\n want {want}"
  for (t, _), r in zip(core, truck, strict=True):
    assert abs(t - r["dt_ms"] / 1000.0) <= 0.3, f"{r} replayed at {t:.2f}s"


class TestTheBrainOnTheTrucksOwnInputs:
  def test_friday_2056_resume_then_rejection_is_reproduced_exactly(self):
    """RES 0.5 s after the brake lifted, the driver braked again 1.4 s later (postResumeBrake), then ON/OFF.
    No accelerator in this window, so this branch must add nothing."""
    frames, truck = _window("fri_2056_res_then_onoff")
    recs = _replay_brain(frames)
    _assert_matches_truck(recs, truck, extra_phases=())

  def test_friday_2207_the_refusal_before_the_accelerator_is_now_on_record(self):
    """D1 (Rule 2). 22:07:18.7 the brake came up at 9.5 mph; the RES window refused `slow` (< 11 mph); the
    driver went onto the accelerator 0.9 s later. The truck logged NOTHING about that refusal -- the gate
    was overwritten by "gas". Everything the truck did write must still replay exactly, plus exactly one
    `lift` record that names the gate."""
    frames, truck = _window("fri_2207_res_nocruise_gas_onoff_x2")
    recs = _replay_brain(frames)
    _assert_matches_truck(recs, truck)
    lifts = [(t, r) for t, r in recs if r["phase"] == "lift"]
    assert len(lifts) == 1, f"expected exactly one lift record, got {[r for _, r in lifts]}"
    t, r = lifts[0]
    assert r["reason"] == "slow" and r["mode"] == "res" and r["fired"] is False, r
    assert 11.3 <= t <= 11.8, f"lift record at {t:.2f}s, the accelerator went down at ~11.57s"
    # every gas edge in this window came while an episode was already open -> the new gas arm never fires here
    assert not [r for _, r in recs if r["phase"] == "arm" and r.get("reason") == "gas"]


class TestTheOnOffButtonOnTheTrucksOwnInputs:
  """Path (a): ON/OFF while anything is on -> everything off, in one press, including the press that makes
  the truck ENGAGE from Standby. Replays the recorded frames through the real StateMachine, MadsPnw,
  MadsQuiet, Events, AlertManager in selfdrived's order. The off-request latch itself lives in
  selfdrived.update_events; its output is the recorded `cruiseOffRequested` event, which is exactly what
  selfdrived passes to MadsPnw as `off_requested` in the same frame (same expression)."""

  @staticmethod
  def _run(frames, start_s):
    sm, mads, quiet, am = StateMachine(), MadsPnw(ALTERNATIVE_EXPERIENCE.ENABLE_MADS), MadsQuiet(MADS_BRAKE_GRACE_FRAMES), AlertManager()
    # Prime into steering-only exactly as the truck got there: engaged, then a brake drops cruise.
    for _ in range(5):
      mads.update(True, True, False, True, Events(), True, False)
      quiet.step(True, mads.lateral_only, mads.available, mads.brake_grace_open)
    mads.update(False, False, True, False, _events(["pedalPressed", "pcmDisable"]), True, False)
    quiet.step(False, mads.lateral_only, mads.available, mads.brake_grace_open)
    assert mads.lateral_only, "priming did not reach steering-only"

    rows, played, last_snd = [], [], AudibleAlert.none
    for i, fr in enumerate(f for f in frames if f["t"] >= start_s):
      names = [n for n in fr["events"] if n != "madsLateralOnly"]
      events = _events(names)
      enabled, active = sm.update(events)
      off_req = "cruiseOffRequested" in names
      mads.update(enabled, active, fr["bp"], fr["ccEn"], events, fr["ccAv"], off_req)
      chime = quiet.step(enabled, mads.lateral_only, mads.available, mads.brake_grace_open)
      if mads.active and not active:
        sm.current_alert_types.append(ET.WARNING)
      if mads.lateral_only:
        events.add(EventName.madsLateralOnly)
      clear = set() if ET.WARNING in sm.current_alert_types else {ET.WARNING}
      if enabled:
        clear.add(ET.NO_ENTRY)
      am.add_many(i, apply_chime_decision(_safe_alerts(events, sm.current_alert_types), chime))
      am.process_alerts(i, clear)
      snd = am.current_alert.audible_alert
      if snd != last_snd and snd != AudibleAlert.none:
        played.append((fr["t"], snd))
      last_snd = snd
      rows.append(dict(t=fr["t"], press=off_req, en=enabled, rec_en=fr["en"], lo=mads.lateral_only, rec_lo=fr["lo"],
                       ccEn=fr["ccEn"], alert=am.current_alert.alert_type))
    return rows, played

  @pytest.mark.parametrize("window,start_s", [("fri_2207_res_nocruise_gas_onoff_x2", 18.0),
                                               ("fri_2056_res_then_onoff", 6.5)])
  def test_on_off_from_steering_only_turns_everything_off_and_cancels_the_engage_blip(self, window, start_s):
    frames, _ = _window(window)
    rows, played = self._run(frames, start_s)
    # fidelity: the replayed engagement and lateral state are the truck's, frame for frame
    assert [r["en"] for r in rows] == [r["rec_en"] for r in rows], "state machine diverged from the truck"
    assert [r["lo"] for r in rows] == [r["rec_lo"] for r in rows], "MadsPnw diverged from the truck"
    press = next(i for i, r in enumerate(rows) if r["press"])
    assert rows[press - 1]["lo"] and not rows[press]["lo"], "lateral must end on the very frame of the press"
    blips = [r for r in rows[press:] if r["ccEn"]]
    assert blips, "fixture must contain the truck's own engage response to the press"
    assert not any(r["en"] for r in rows), "openpilot engaged with the truck's response to an OFF press"
    assert all(r["alert"] == "cruiseOffRequested/noEntry" for r in blips), {r["alert"] for r in blips}
    # what the driver HEARS with madsquiet2pnw: exactly one disengage chime, at the press, nothing at the blip
    assert [s for _, s in played] == [AudibleAlert.disengage], f"speaker: {played}"
    assert abs(played[0][0] - rows[press]["t"]) <= 0.011, f"chime at {played[0][0]}, press at {rows[press]['t']}"
