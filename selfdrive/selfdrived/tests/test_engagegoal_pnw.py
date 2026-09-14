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
      # No RES/SET/ON-OFF from the driver inside any verify window of these two windows (rlog 0x083 bus 0: the only
      # driver presses are ON/OFF at 20:56:24.25, 22:07:28.47 and 22:07:29.56, all after the verifies resolved).
      v_ego=fr["v"], standstill=fr["stst"], driver_cruise_button=False, has_lead=lead,
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


# ---------------------------------------------------------------------------------------------------------------
# engagegoal2pnw -- OWNER DECISION 2026-09-13 "Cancel, steering drops too".
# Sun 2026-09-13 21:16:33 PT, Corvallis (drives/2026-09-13/corvallis-resume-55/, rlog_seg2_2116_timeline.txt):
# steering-only after a brake to a near stop, the driver accelerated 21:16:13 -> 21:16:32.80 to 15.36 m/s and lifted;
# our gas-set SET- went out; 0.15 s after the first SET- frame the PCM engaged (CcStat 3->5) with its set at 55 mph
# (24.59 m/s), no driver button on any bus, and openpilot engaged with it on the same frame.
# The loop below is selfdrived's per-frame order (update_events -> StateMachine -> MadsPnw -> MadsQuiet -> resume brain
# -> alerts), controlsd's cancel rule, and the Ford executor's resume/SET path (real opendbc parse_resume_cmd,
# decide_resume, ResumePress, with carcontroller._resume_button's `frame % 25` idle poll). The truck is a measured
# model: SET- from Standby engages 0.15 s later at `pcm_set_ms`; a CANCEL drops cruise 0.08 s later (onebutton blips);
# a driver + engages 0.2 s later at the current speed.
# ---------------------------------------------------------------------------------------------------------------

def _drive_2116(pcm_set_ms=24.59, driver_plus_at=None, driver_btn_in_window=False):
  from opendbc.car.ford.icbm_pnw import ResumePress, decide_resume, parse_resume_cmd
  EN = EventName
  sm, mads = StateMachine(), MadsPnw(ALTERNATIVE_EXPERIENCE.ENABLE_MADS)
  quiet, am, brain, press = MadsQuiet(MADS_BRAKE_GRACE_FRAMES), AlertManager(), MadsResumeBrain(), ResumePress()
  DT = 0.01
  cc_en, cc_set, v = True, 18.78, 11.3                      # engaged, 42 mph memory
  cc_prev = False                                            # so frame 0 is the engage edge: openpilot starts engaged
  bp_prev = False
  pending = False
  cmd, mem = None, {}
  truck_engage_at = truck_drop_at = None
  log = dict(first_press=None, cancel_frames=[], brain_cancels=[], event_frames=[], sounds=[], alerts=[], lat=[], en=[])
  last_snd = AudibleAlert.none
  for f in range(int(45.0 / DT)):
    t = f * DT
    brake = 1.0 <= t < 3.0
    gas = 6.0 <= t < 26.8
    if brake:
      v = max(2.0, v - 4.5 * DT)
    elif gas:
      v = min(15.36, v + 0.8 * DT)                           # 15.36 m/s at lift-off, as measured
    else:
      v = max(0.0, v - (0.4 if cc_en else 1.2) * DT)
    if brake and cc_en:
      cc_en = False                                          # the PCM drops cruise on the brake
    driver_plus = driver_plus_at is not None and abs(t - driver_plus_at) < DT / 2
    if driver_plus and not cc_en:
      truck_engage_at, pcm_next_set = t + 0.2, v
    if truck_engage_at is not None and t >= truck_engage_at and not cc_en:
      cc_en, cc_set, truck_engage_at = True, pcm_next_set, None
    if truck_drop_at is not None and t >= truck_drop_at:
      cc_en, truck_drop_at = False, None
    buttons = driver_plus

    # --- selfdrived.update_events (car events for a pcmCruise Ford + the one-frame cancel event) ---
    events = Events()
    if cc_en and not cc_prev:
      events.add(EN.pcmEnable)
    elif not cc_en:
      events.add(EN.pcmDisable)
    if brake and (not bp_prev or v > 0.0):
      events.add(EN.pedalPressed)
    if pending:
      pending = False
      events.add(EN.madsResumeSetTooHigh)
      log["event_frames"].append(f)
    enabled, active = sm.update(events)
    mads.update(enabled, active, brake, cc_en, events, True, False)
    chime = quiet.step(enabled, mads.lateral_only, mads.available, mads.brake_grace_open)
    out = brain.update(ResumeInputs(
      now=t, mads_available=True, lateral_only=mads.lateral_only, op_enabled=enabled,
      blocked=has_blocking_event(events), engageable=not events.contains(ET.NO_ENTRY),
      brake_pressed=brake, regen_braking=False, gas_pressed=gas, cruise_enabled=cc_en, cruise_available=True,
      set_speed_ms=cc_set if cc_en else cc_set, v_ego=v, standstill=v < 0.1,
      driver_cruise_button=buttons or (driver_btn_in_window and log["first_press"] is not None
                                        and abs(t - log["first_press"] - 0.1) < DT / 2),
      has_lead=False))
    if out.cancel:
      pending = True
      log["brain_cancels"].append(t)
    if mads.active and not active:
      sm.current_alert_types.append(ET.WARNING)
    if mads.lateral_only:
      events.add(EN.madsLateralOnly)
    clear = set() if ET.WARNING in sm.current_alert_types else {ET.WARNING}
    if enabled:
      clear.add(ET.NO_ENTRY)
    am.add_many(f, apply_chime_decision(_safe_alerts(events, sm.current_alert_types), chime))
    am.process_alerts(f, clear)
    snd = am.current_alert.audible_alert
    if snd != last_snd and snd != AudibleAlert.none:
      log["sounds"].append((t, snd))
    last_snd = snd
    log["alerts"].append((t, am.current_alert.alert_text_1))
    log["lat"].append((t, mads.active))
    log["en"].append((t, enabled))
    mem = {"dir": out.mode, "ts": round(t, 3), "eid": out.eid, "set": round(out.set_ms, 2)} if out.offer else {}

    # --- controlsd + the Ford carcontroller acc-button chain ---
    cancel = cc_en and not enabled
    if cancel:
      log["cancel_frames"].append(t)
      if truck_drop_at is None:
        truck_drop_at = t + 0.08
    if cmd is not None or f % 25 == 0:
      cmd = parse_resume_cmd(mem)
    ok = decide_resume(cmd, t, cc_en, True, gas or brake, cc_set)
    if press.update(f, cmd, ok) and not cancel:
      if log["first_press"] is None:
        log["first_press"] = t
      if not cc_en and truck_engage_at is None and cmd.mode == "set":
        # pcm_set_ms: the measured 21:16:33 overshoot; a callable models a PCM that sets relative to the speed
        truck_engage_at, pcm_next_set = t + 0.15, (pcm_set_ms(v) if callable(pcm_set_ms) else pcm_set_ms)
    cc_prev, bp_prev = cc_en, brake
  return log


class TestTheCorvallisOvershootCancel:
  def test_the_2116_sequence_cancels_within_0p3s_with_the_alert_and_one_chime(self):
    log = _drive_2116()
    p = log["first_press"]
    assert p is not None, "the gas-set press must go out"
    assert len(log["brain_cancels"]) == 1 and len(log["event_frames"]) == 1, (log["brain_cancels"], log["event_frames"])
    first_cancel = log["cancel_frames"][0]
    assert 0.0 < first_cancel - p <= 0.3, f"CANCEL {first_cancel - p:.2f}s after our SET-"
    # not a storm: the cancel frames stop once the truck drops cruise (measured ~80 ms)
    assert len(log["cancel_frames"]) <= 10 and log["cancel_frames"][-1] - first_cancel <= 0.1, log["cancel_frames"]
    texts = {txt for t, txt in log["alerts"] if first_cancel - 0.02 <= t <= first_cancel + 3.0}
    assert "Cruise set too high - cancelled" in texts, texts
    after = [s for t, s in log["sounds"] if t >= p]
    assert after == [AudibleAlert.disengage], f"speaker after our press: {log['sounds']}"
    assert not any(active for t, active in log["lat"] if t >= first_cancel + 0.05), "steering must drop too"

  def test_a_later_driver_plus_re_engages_and_is_not_cancelled(self):
    log = _drive_2116(driver_plus_at=33.0)
    later = [t for t in log["cancel_frames"] if t >= 33.0]
    assert later == [], f"the driver's own + was cancelled: {later}"
    assert any(en for t, en in log["en"] if t >= 33.5), "openpilot must engage with the driver's +"
    assert len(log["brain_cancels"]) == 1

  def test_S3_a_driver_plus_just_before_our_press_is_neither_pressed_over_nor_cancelled(self):
    """Fable B1 through the whole loop: lift-off at 26.8 s, the driver's + at 27.65 s (our SET- was due at 27.8 s),
    the truck engages 0.2 s after THEIR press. No press of ours, no CANCEL, openpilot engaged."""
    log = _drive_2116(driver_plus_at=27.65)
    assert log["first_press"] is None, f"our SET- went out at {log['first_press']} on top of the driver's +"
    assert log["brain_cancels"] == [] and not [t for t in log["cancel_frames"] if t > 27.0], log["cancel_frames"]
    assert any(en for t, en in log["en"] if t >= 28.2), "openpilot must engage with the driver's own +"

  def test_a_driver_button_between_our_press_and_the_come_back_is_not_cancelled(self):
    log = _drive_2116(driver_btn_in_window=True)
    assert log["brain_cancels"] == [] and log["event_frames"] == [], log["brain_cancels"]
    first_press = log["first_press"]
    assert not [t for t in log["cancel_frames"] if t > first_press], "cancelled although the driver pressed a button"

  @pytest.mark.parametrize("pcm", [lambda v: v, lambda v: v + 2.9 * 0.44704], ids=["at_tap_speed", "plus_2p9mph"])
  def test_a_come_back_at_or_within_3_mph_of_the_tap_speed_is_not_cancelled(self, pcm):
    log = _drive_2116(pcm_set_ms=pcm)
    first_press = log["first_press"]
    assert first_press is not None and log["brain_cancels"] == [], log["brain_cancels"]
    assert not [t for t in log["cancel_frames"] if t > first_press]
    assert any(en for t, en in log["en"] if t > first_press + 0.5), "cruise and openpilot must stay engaged"


class TestTheSelfdrivedWiringForTheCancel:
  """selfdrived cannot be imported on the dev host; pin the three lines the harness above reproduces."""
  SRC = (pathlib.Path(__file__).parent.parent / "selfdrived.py").read_text()

  def test_the_brain_is_told_about_the_drivers_cruise_buttons(self):
    import re
    m = re.search(r"driver_cruise_button=any\(be\.pressed and be\.type in \((.*?)\)\s*for be in CS\.buttonEvents\)", self.SRC, re.S)
    assert m, "driver_cruise_button must come from CS.buttonEvents"
    for btn in ("accelCruise", "decelCruise", "resumeCruise", "setCruise", "mainCruise"):
      assert btn in m.group(1), btn

  def test_a_cancel_decision_raises_exactly_one_event_on_the_next_frame(self):
    assert "if out.cancel:\n        self.set_high_cancel_pending = True" in self.SRC
    one_frame = ("if self.set_high_cancel_pending:\n      self.set_high_cancel_pending = False\n" +
                 "      self.events.add(EventName.madsResumeSetTooHigh)")
    assert one_frame in self.SRC
    ue = self.SRC.index("def update_events"), self.SRC.index("def data_sample")
    at = self.SRC.index("self.events.add(EventName.madsResumeSetTooHigh)")
    assert ue[0] < at < ue[1], "the event must be raised in update_events, before the state machine runs"

  def test_the_event_disengages_chimes_and_is_not_silenced_by_madsquiet(self):
    from openpilot.selfdrive.selfdrived.events import EVENTS
    from openpilot.selfdrive.selfdrived.madsquiet_pnw import QUIET_DISENGAGE_EVENTS
    ev = EVENTS[EventName.madsResumeSetTooHigh]
    assert ev[ET.USER_DISABLE].audible_alert == AudibleAlert.disengage
    assert ev[ET.USER_DISABLE].priority > ev[ET.PERMANENT].priority, "the chiming alert must win the AlertManager"
    assert "madsResumeSetTooHigh" not in QUIET_DISENGAGE_EVENTS
    assert ET.NO_ENTRY not in ev, "a NO_ENTRY would block the driver's re-engage"
