"""madsquiet2pnw — engagement chimes are silent while MADS keeps steering; a full disengage still chimes.

Driver request 2026-09-13, confirmed: "we only want the silence while MADS keeps steering. When I
disengage fully it can still chime."

These drive the REAL MadsPnw state machine and MadsQuiet together, frame by frame, with real Events, so
the brake-race window (madsbrakerace2pnw: the PCM drops cruise before the brake lands) is exercised the
way the truck produces it. selfdrived itself cannot be imported on the dev host (card.py needs Crypto,
which does not build here), so its two-line wiring is pinned by source inspection at the bottom -- the
decision and its application are the logic, and both are exercised directly.
"""
import ast
import copy
import inspect
import pathlib

import pytest

from cereal import log
from opendbc.safety import ALTERNATIVE_EXPERIENCE
from openpilot.selfdrive.selfdrived.events import EVENTS, ET, Events, AudibleAlert
from openpilot.selfdrive.selfdrived.mads_pnw import MadsPnw, MADS_BRAKE_GRACE_FRAMES
from openpilot.selfdrive.selfdrived.madsquiet_pnw import (ChimeDecision, MadsQuiet, apply_chime_decision,
                                                          FULL_DISENGAGE_ALERT_TYPE)

EventName = log.OnroadEvent.EventName
MADS_ON = ALTERNATIVE_EXPERIENCE.ENABLE_MADS
MADS_DISENGAGE = ALTERNATIVE_EXPERIENCE.ENABLE_MADS | ALTERNATIVE_EXPERIENCE.MADS_DISENGAGE_LATERAL_ON_BRAKE


def ev(*names) -> Events:
  e = Events()
  for n in names:
    e.add(n)
  return e


class Truck:
  """One frame = selfdrived's order: state machine result -> MadsPnw.update -> MadsQuiet.step."""

  def __init__(self, alt=MADS_ON):
    self.mads = MadsPnw(alt)
    self.quiet = MadsQuiet(MADS_BRAKE_GRACE_FRAMES)
    self.log = []

  def frame(self, enabled, braking=False, cruise=None, events=(), available=True, off=False):
    cruise = enabled if cruise is None else cruise
    self.mads.update(enabled, enabled, braking, cruise, ev(*events), available, off)
    d = self.quiet.step(enabled, self.mads.lateral_only, self.mads.available, self.mads.brake_grace_open)
    self.log.append(d)
    return d

  def engaged(self, n=5):
    for _ in range(n):
      self.frame(True)

  @property
  def chimes(self):
    return sum(1 for d in self.log if d.chime_disengage)


class TestTheTrafficLight:
  def test_brake_lands_with_the_cruise_drop_is_silent_both_ways(self):
    """THE REQUEST. Brake -> MADS steers -> resume. No disengage chime, no engage chime."""
    t = Truck()
    t.engaged()
    d = t.frame(False, braking=True, events=(EventName.pedalPressed, EventName.pcmDisable))
    assert t.mads.lateral_only and d.quiet_disengage
    for _ in range(200):                                   # sitting at the light, steering held
      t.frame(False, braking=True, events=(EventName.pcmDisable,))
    for _ in range(50):
      t.frame(False, events=(EventName.pcmDisable,))
    d = t.frame(True, events=(EventName.pcmEnable,))       # cruise resumes
    assert d.quiet_engage, "the resume from steering-only still chimed"
    assert t.chimes == 0

  def test_six_traffic_lights_produce_zero_chimes(self):
    t = Truck()
    t.engaged()
    quiet = 0
    for _ in range(6):
      quiet += t.frame(False, braking=True, events=(EventName.pedalPressed, EventName.pcmDisable)).quiet_disengage
      for _ in range(80):
        t.frame(False, braking=True, events=(EventName.pcmDisable,))
      quiet += t.frame(True, events=(EventName.pcmEnable,)).quiet_engage
      t.engaged(20)
    assert quiet == 12 and t.chimes == 0


class TestTheBrakeRace:
  """madsbrakerace2pnw, measured on the truck: the PCM drops cruise BEFORE the brake signal lands. On the
  disengage frame MADS has not armed yet -- deciding the sound from that frame alone would chime at
  exactly the traffic light this feature exists for."""

  @pytest.mark.parametrize("brake_late", [1, 10, MADS_BRAKE_GRACE_FRAMES - 1])
  def test_a_brake_that_lands_late_inside_the_window_stays_silent(self, brake_late):
    t = Truck()
    t.engaged()
    d = t.frame(False, braking=False, events=(EventName.pcmDisable,))
    assert not t.mads.lateral_only and t.mads.brake_grace_open, "scenario did not open the brake-race window"
    assert d.quiet_disengage, "silenced only after the brake landed -- the chime already played"
    for _ in range(brake_late - 1):
      t.frame(False, events=(EventName.pcmDisable,))
    t.frame(False, braking=True, events=(EventName.pedalPressed, EventName.pcmDisable))
    assert t.mads.lateral_only
    for _ in range(100):
      t.frame(False, braking=True, events=(EventName.pcmDisable,))
    assert t.chimes == 0

  def test_no_brake_ever_lands_so_it_WAS_a_full_disengage_and_it_chimes_once(self):
    """The provisional silence must be paid back: cruise dropped, MADS never took over."""
    t = Truck()
    t.engaged()
    assert t.frame(False, events=(EventName.pcmDisable,)).quiet_disengage
    for _ in range(MADS_BRAKE_GRACE_FRAMES + 30):
      t.frame(False, events=(EventName.pcmDisable,))
    assert t.chimes == 1, f"expected exactly one late disengage chime, got {t.chimes}"
    first = next(i for i, d in enumerate(t.log) if d.chime_disengage)
    disengage_at = next(i for i, d in enumerate(t.log) if d.quiet_disengage)
    assert first - disengage_at <= MADS_BRAKE_GRACE_FRAMES + 2, "the late chime came later than the window"


class TestAFullDisengageStillChimes:
  def test_a_cancel_press_chimes_immediately(self):
    """A blocking event: MADS will not steer, so the chime is not deferred at all."""
    t = Truck()
    t.engaged()
    d = t.frame(False, braking=False, events=(EventName.buttonCancel, EventName.pcmDisable))
    assert not t.mads.lateral_only and not t.mads.brake_grace_open
    assert d == ChimeDecision(), "a cancel press was silenced"

  def test_disengage_on_brake_setting_chimes_normally(self):
    t = Truck(alt=MADS_DISENGAGE)
    t.engaged()
    d = t.frame(False, braking=True, events=(EventName.pedalPressed, EventName.pcmDisable))
    assert not t.mads.lateral_only and not d.quiet_disengage

  def test_MADS_stopping_on_its_own_chimes_when_steering_stops(self):
    """Today nothing chimes here -- openpilot is already `disabled`. Once the brake chime is silenced,
    that would leave brake -> MADS -> steering-off with no audible disengage at all."""
    t = Truck()
    t.engaged()
    t.frame(False, braking=True, events=(EventName.pedalPressed, EventName.pcmDisable))
    for _ in range(50):
      t.frame(False, braking=True, events=(EventName.pcmDisable,))
    assert t.chimes == 0
    d = t.frame(False, available=False)                    # ACC main switched off: everything off
    assert not t.mads.lateral_only
    assert d.chime_disengage, "steering stopped without a sound"

  def test_the_tesla_and_any_car_without_MADS_is_untouched(self):
    t = Truck(alt=ALTERNATIVE_EXPERIENCE.DEFAULT)
    t.engaged()
    d1 = t.frame(False, braking=True, events=(EventName.pedalPressed, EventName.pcmDisable))
    t.frame(False)
    d2 = t.frame(True, events=(EventName.pcmEnable,))
    assert d1 == d2 == ChimeDecision()

  def test_no_MADS_capability_means_stock_chimes_whatever_the_other_inputs_say(self):
    """The gate, pinned at the module's own contract. Through MadsPnw it is redundant (a car without MADS
    never reports lateral_only or an open brake window), so a mutation removing it survived the
    frame-by-frame tests; this feeds the inputs directly, including a combination MadsPnw cannot produce."""
    q = MadsQuiet(MADS_BRAKE_GRACE_FRAMES)
    q.step(True, False, False, False)
    assert q.step(False, True, False, True) == ChimeDecision()
    q.step(False, True, False, False)
    assert q.step(True, False, False, False) == ChimeDecision()

  def test_engaging_from_plain_off_still_chimes(self):
    t = Truck()
    for _ in range(5):
      t.frame(False)
    assert t.frame(True, events=(EventName.pcmEnable,)) == ChimeDecision()


def _alerts(*pairs):
  """Build the list create_alerts would return, the same way it does (shared instances + alert_type)."""
  e = Events()
  for name, _ in pairs:
    e.add(name)
  return e.create_alerts(sorted({et for _, et in pairs}))


class TestApplyingTheDecision:
  def test_silencing_never_mutates_the_shared_EVENTS_table(self):
    """create_alerts returns the SHARED Alert instances. Editing one would silence that event for the
    rest of the process -- on every later frame, including a genuine full disengage."""
    before = EVENTS[EventName.pcmDisable][ET.USER_DISABLE].audible_alert
    out = apply_chime_decision(_alerts((EventName.pcmDisable, ET.USER_DISABLE)), ChimeDecision(quiet_disengage=True))
    assert out[0].audible_alert == AudibleAlert.none
    assert EVENTS[EventName.pcmDisable][ET.USER_DISABLE].audible_alert == before == AudibleAlert.disengage

  def test_only_the_brake_events_are_silenced(self):
    alerts = _alerts((EventName.pcmDisable, ET.USER_DISABLE), (EventName.pedalPressed, ET.USER_DISABLE),
                     (EventName.buttonCancel, ET.USER_DISABLE))
    out = {a.alert_type.split("/")[0]: a.audible_alert for a in apply_chime_decision(alerts, ChimeDecision(quiet_disengage=True))}
    assert out["pcmDisable"] == out["pedalPressed"] == AudibleAlert.none
    assert out["buttonCancel"] == AudibleAlert.disengage, "a cancel press lost its chime"

  def test_the_engage_chime_is_silenced_only_for_cruise_enable(self):
    out = apply_chime_decision(_alerts((EventName.pcmEnable, ET.ENABLE)), ChimeDecision(quiet_engage=True))
    assert out[0].audible_alert == AudibleAlert.none
    out = apply_chime_decision(_alerts((EventName.pcmEnable, ET.ENABLE)), ChimeDecision())
    assert out[0].audible_alert == AudibleAlert.engage

  def test_a_confirmed_full_disengage_adds_exactly_one_chime(self):
    out = apply_chime_decision([], ChimeDecision(chime_disengage=True))
    assert len(out) == 1 and out[0].audible_alert == AudibleAlert.disengage
    assert out[0].alert_type == FULL_DISENGAGE_ALERT_TYPE and out[0].event_type == ET.USER_DISABLE

  def test_no_decision_changes_nothing(self):
    alerts = _alerts((EventName.pcmDisable, ET.USER_DISABLE), (EventName.pcmEnable, ET.ENABLE))
    assert [copy.copy(a).audible_alert for a in apply_chime_decision(alerts, ChimeDecision())] == \
           [a.audible_alert for a in alerts]


class TestTheSelfdrivedWiring:
  """selfdrived cannot be imported on the dev host; pin the two call sites and their fail-safe."""

  @staticmethod
  def _src():
    return (pathlib.Path(__file__).resolve().parents[1] / "selfdrived.py").read_text()

  def test_the_decision_runs_after_mads_update_and_before_the_alerts(self):
    s = self._src()
    step = s[s.index("  def step(self):"):]
    i_mads = step.index("self.mads.update(")
    i_quiet = step.index("self.mads_quiet.step(")
    i_alerts = step.index("self.update_alerts(CS)")
    assert i_mads < i_quiet < i_alerts, "the chime decision must see THIS frame's MADS state"

  def test_both_call_sites_fall_back_to_stock_chimes(self):
    tree = ast.parse(self._src())
    guarded = {"step": False, "apply_chime_decision": False}
    for node in ast.walk(tree):
      if isinstance(node, ast.Try):
        body = ast.unparse(node)
        if "self.mads_quiet.step(" in body and "ChimeDecision()" in body:
          guarded["step"] = True
        if "apply_chime_decision(" in body and "cloudlog.exception" in body:
          guarded["apply_chime_decision"] = True
    assert all(guarded.values()), f"an unguarded madsquiet call could take selfdrived down: {guarded}"

  def test_the_decision_module_has_no_side_channels(self):
    """Pure: no params, no sockets, no clock -- only the state it is handed."""
    import openpilot.selfdrive.selfdrived.madsquiet_pnw as m
    src = inspect.getsource(m)
    for banned in ("Params(", "messaging", "time.monotonic", "time.time"):
      assert banned not in src


class TestWhatTheSpeakerActuallyPlays:
  """Gemini review: the decision tests "count chime_disengage flags" and never model the AlertManager or
  soundd, so a silenced alert still active in the AlertManager could outrank the late chime and leave a
  full disengage silent. Answer it with the real AlertManager and soundd's own trigger rule.

  Per frame this does what selfdrived does: the state machine's alert types (USER_DISABLE on the
  disengage frame, ENABLE on the enable frame, PERMANENT otherwise -- see selfdrived/state.py),
  Events.create_alerts, apply_chime_decision, AlertManager.add_many + process_alerts. A sound "plays"
  when soundd would start one: the current alert's sound CHANGES to something other than none
  (soundd.update_alert)."""

  @staticmethod
  def _play(frames, alt=MADS_ON):
    from openpilot.selfdrive.selfdrived.alertmanager import AlertManager
    mads, quiet, am = MadsPnw(alt), MadsQuiet(MADS_BRAKE_GRACE_FRAMES), AlertManager()
    played, last, en_prev = [], AudibleAlert.none, False
    for i, (enabled, braking, events, available) in enumerate(frames):
      mads.update(enabled, enabled, braking, enabled, ev(*events), available, False)
      d = quiet.step(enabled, mads.lateral_only, mads.available, mads.brake_grace_open)
      types = [ET.PERMANENT]
      if en_prev and not enabled:
        types.append(ET.USER_DISABLE)
      if enabled and not en_prev:
        types.append(ET.ENABLE)
      en_prev = enabled
      e = ev(*events)
      alerts = apply_chime_decision(_safe_alerts(e, types), d)
      am.add_many(i, alerts)
      am.process_alerts(i, set())
      snd = am.current_alert.audible_alert
      if snd != last and snd != AudibleAlert.none:
        played.append((i, snd))
      last = snd
    return played

  def test_a_full_disengage_is_AUDIBLE_through_the_alert_manager(self):
    """Cruise drops, no brake ever lands: the provisional silence must come out of the speaker as one
    disengage chime, not be swallowed by the still-registered silenced alert."""
    frames = [(True, False, (), True)] * 5 + [(False, False, (EventName.pcmDisable,), True)] * 120
    played = self._play(frames)
    assert [s for _, s in played] == [AudibleAlert.disengage], f"speaker output: {played}"
    assert played[0][0] - 5 <= MADS_BRAKE_GRACE_FRAMES + 2

  def test_the_traffic_light_is_silent_through_the_alert_manager(self):
    frames = ([(True, False, (), True)] * 5 +
              [(False, True, (EventName.pedalPressed, EventName.pcmDisable), True)] +
              [(False, True, (EventName.pcmDisable,), True)] * 150 +
              [(True, False, (EventName.pcmEnable,), True)] * 10)
    assert self._play(frames) == [], "the speaker played something at a MADS traffic light"

  def test_without_MADS_the_speaker_plays_the_stock_chimes(self):
    frames = ([(True, False, (), True)] * 5 +
              [(False, True, (EventName.pedalPressed, EventName.pcmDisable), True)] * 30 +
              [(True, False, (EventName.pcmEnable,), True)] * 10)
    played = [s for _, s in self._play(frames, alt=ALTERNATIVE_EXPERIENCE.DEFAULT)]
    assert played == [AudibleAlert.disengage, AudibleAlert.engage]


def _safe_alerts(events, types):
  """Events.create_alerts, skipping callable alerts that need the full selfdrived callback arguments (none
  of the engagement alerts under test are callables). Names alerts exactly as create_alerts does, via
  EVENT_NAME -- the first version of this helper used the raw event NUMBER, so apply_chime_decision could
  not recognise pcmDisable and the test reported chimes the real code never plays."""
  from openpilot.selfdrive.selfdrived.events import Alert, EVENT_NAME
  out = []
  for e in events.events:
    for et in types:
      alert = EVENTS[e].get(et)
      if isinstance(alert, Alert):
        alert.alert_type = f"{EVENT_NAME[e]}/{et}"
        alert.event_type = et
        out.append(alert)
  return out


# ---- madsquiet2pnw / Raven (owner 2026-09-29: "one chime when I brake into steering-only, one when I pull the stalk") ----
# The Raven has had the MADS capability since teslamads2pnw, so MadsQuiet applies to it through the same
# `mads.available` gate as the Lightning. These pin that with the Raven's own shape: pcmCruise=True, the brake frame lands
# 19-51 ms BEFORE DI_cruiseState drops (drives/2026-09-28/tesla-brake-census), and the stalk pull goes STANDBY -> OVERRIDE
# -> ENABLED. Driven through the REAL StateMachine + Events + AlertManager, not through hand-fed engaged flags.

class TestTheRavenSequence:
  @staticmethod
  def _drive(frames):
    """frames: (cruise_enabled, brake, extra_event_names, off_requested). Returns (sounds, banner_frames)."""
    from openpilot.selfdrive.selfdrived.alertmanager import AlertManager
    from openpilot.selfdrive.selfdrived.events import EventNamePnw
    from openpilot.selfdrive.selfdrived.state import StateMachine
    from openpilot.selfdrive.selfdrived.mads_pnw import MADS_BRAKE_PANDA_GRACE_FRAMES
    sm, mads, quiet, am = StateMachine(), MadsPnw(MADS_ON), MadsQuiet(MADS_BRAKE_PANDA_GRACE_FRAMES), AlertManager()
    cruise_prev = brake_prev = False
    last, sounds, banner = AudibleAlert.none, [], 0
    for i, (cruise, brake, extra, off) in enumerate(frames):
      names = list(extra)
      if cruise and not cruise_prev:
        names.append(EventName.pcmEnable)
      elif not cruise:
        names.append(EventName.pcmDisable)                      # car_specific re-raises it EVERY frame cruise is off
      if brake and not brake_prev:
        names.append(EventName.pedalPressed)
      cruise_prev, brake_prev = cruise, brake
      e = ev(*names)
      enabled, active = sm.update(e)
      mads.update(enabled, active, brake, cruise, e, True, off)
      d = quiet.step(enabled, mads.lateral_only, mads.available, mads.brake_grace_open)
      if mads.lateral_only:
        e.add(EventNamePnw.madsLateralOnly)                     # selfdrived.step adds this after mads.update
      am.add_many(i, apply_chime_decision(_safe_alerts(e, sm.current_alert_types), d))
      am.process_alerts(i, set())
      snd = am.current_alert.audible_alert
      if snd != last and snd != AudibleAlert.none:
        sounds.append((i, snd))
      last = snd
      if mads.lateral_only and am.current_alert.alert_text_1 == "Steering only":
        banner += 1
    return sounds, banner

  def test_brake_into_steering_only_and_the_stalk_pull_back_are_both_silent_but_the_banner_stays(self):
    frames = ([(False, False, (), False)] * 5 +
              [(True, False, (), False)] * 20 +                 # first engage from fully off (stock chime, checked below)
              [(True, True, (), False)] * 4 +                   # brake lands first, cruise still ENABLED
              [(False, True, (), False)] * 100 +                # DI_cruiseState -> STANDBY, steering-only
              [(False, False, (), False)] * 200 +               # brake released, still steering-only
              [(True, False, (), False)] * 40)                  # stalk pull: STANDBY -> OVERRIDE -> ENABLED
    sounds, banner = self._drive(frames)
    assert sounds == [(5, AudibleAlert.engage)], f"only the first engage from off may sound: {sounds}"
    assert banner >= 270, f"the 'Steering only' banner must stay up through steering-only (frames shown: {banner} of ~304)"

  def test_the_first_engage_from_fully_off_still_chimes(self):
    sounds, _ = self._drive([(False, False, (), False)] * 5 + [(True, False, (), False)] * 10)
    assert sounds == [(5, AudibleAlert.engage)]

  def test_a_stalk_push_out_of_steering_only_is_a_full_disengage_and_chimes_once(self):
    frames = ([(True, False, (), False)] * 10 + [(True, True, (), False)] * 4 + [(False, True, (), False)] * 50 +
              [(False, False, (), True)] * 5 + [(False, False, (), False)] * 100)   # FWD push -> off request
    sounds, _ = self._drive(frames)
    assert [s for _, s in sounds][1:] == [AudibleAlert.disengage], f"steering ended with no/extra sound: {sounds}"

  def test_disengage_on_brake_setting_keeps_the_raven_chime(self):
    t = Truck(alt=MADS_DISENGAGE)
    t.engaged()
    assert t.frame(False, braking=True, events=(EventName.pedalPressed, EventName.pcmDisable)) == ChimeDecision()


class TestEverySafetyAlertKeepsItsSound:
  """The decision may only ever touch the four named engagement alerts. Sweep the WHOLE events table under both
  quiet flags: every other (event, type) alert must come out with its sound unchanged -- faults, steerTempUnavailable,
  steerSaturated/takeControl, driver monitoring, commIssue, immediate/soft disable, the Lightning's pcm/button alerts."""

  QUIET = {("pedalPressed", ET.USER_DISABLE), ("pcmDisable", ET.USER_DISABLE),
           ("pcmEnable", ET.ENABLE), ("buttonEnable", ET.ENABLE)}

  def test_no_other_alert_in_the_table_loses_its_sound(self):
    from openpilot.selfdrive.selfdrived.events import Alert, EVENT_NAME
    decision = ChimeDecision(quiet_disengage=True, quiet_engage=True)
    checked = audible = 0
    for e, types in EVENTS.items():
      for et, alert in types.items():
        if not isinstance(alert, Alert):
          continue                                              # callables need live callback args; none is an engagement alert
        alert = copy.copy(alert)
        alert.alert_type, alert.event_type = f"{EVENT_NAME[e]}/{et}", et
        out = apply_chime_decision([alert], decision)[0]
        checked += 1
        if (EVENT_NAME[e], et) in self.QUIET:
          assert out.audible_alert == AudibleAlert.none
        else:
          assert out.audible_alert == alert.audible_alert, f"{EVENT_NAME[e]}/{et} lost its sound"
          audible += alert.audible_alert != AudibleAlert.none
    assert checked > 100 and audible > 60, f"the sweep scanned too little to mean anything: {checked} alerts, {audible} audible"

  def test_the_named_safety_alerts_are_in_the_sweep_and_audible(self):
    from openpilot.selfdrive.selfdrived.events import Alert
    for name, et in (("steerTempUnavailable", ET.SOFT_DISABLE), ("steerUnavailable", ET.IMMEDIATE_DISABLE),
                     ("commIssue", ET.SOFT_DISABLE), ("controlsMismatch", ET.IMMEDIATE_DISABLE),
                     ("promptDriverDistracted", ET.PERMANENT), ("steerSaturated", ET.WARNING), ("buttonCancel", ET.USER_DISABLE),
                     ("steerDisengage", ET.USER_DISABLE), ("wrongCarMode", ET.USER_DISABLE)):
      a = EVENTS[getattr(EventName, name)].get(et)
      assert a is not None, f"{name}/{et} vanished from the table -- the sweep no longer covers it"
      if isinstance(a, Alert):
        assert a.audible_alert != AudibleAlert.none, f"{name}/{et} is silent"
