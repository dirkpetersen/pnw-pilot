"""
madsresume2pnw -- the gate matrix for the bounded auto-resume brain.

Every hard gate in the feature spec gets at least one test that FAILS if the gate is removed
(mutation-verified; see the branch doc for the log). The brain is pure, so these drive it directly
with a `ResumeInputs` sequence -- no cereal, no msgq, no params.

The helper below is deliberately explicit rather than clever: `run()` feeds a list of per-tick
input overrides at 100 Hz and returns (offers, records), so a test reads as the drive it describes.
"""

import pytest

from openpilot.selfdrive.controls.lib import madsresume_pnw as M
from openpilot.selfdrive.controls.lib.madsresume_pnw import MadsResumeBrain, ResumeInputs, lead_gate

SET = 29.0            # ~65 mph, a plausible driver set speed
DT = 0.01


def mk(now, **kw):
  """A tick of 'cruising normally at the set speed, no lead' with overrides applied."""
  base = dict(
    now=now, mads_available=True, lateral_only=False, op_enabled=True, blocked=False,
    brake_pressed=False, regen_braking=False, gas_pressed=False,
    cruise_enabled=True, cruise_available=True, set_speed_ms=SET, v_ego=SET,
    standstill=False, has_lead=False, d_rel=None, v_lead=None, engageable=True,
    driver_cruise_button=False,
  )
  base.update(kw)
  return ResumeInputs(**base)


class Drive:
  """Runs the brain over a scripted drive. `t` advances 10 ms per tick."""

  def __init__(self, **defaults):
    self.b = MadsResumeBrain()
    self.t = 0.0
    self.defaults = defaults
    self.offers = []          # (t, eid, set_ms) for every tick an offer was published
    self.records = []

  def tick(self, n=1, **kw):
    kw = {**self.defaults, **kw}
    for _ in range(n):
      out = self.b.update(mk(self.t, **kw))
      if out.offer:
        self.offers.append((round(self.t, 3), out.eid, out.set_ms))
      self.records.extend(out.records)
      self.t += DT
    return self

  def phases(self):
    return [r["phase"] for r in self.records]

  def reasons(self, phase):
    return [r.get("reason") for r in self.records if r["phase"] == phase]

  def fired(self):
    return len(self.offers) > 0


# Timeline of the reference drive (100 Hz):
#   t=0.00 .. 0.50   cruising with stock ACC engaged -> the set speed is captured
#   t=0.50 .. 0.70   brake down, openpilot disengages, MADS holds lateral (the ARM edge)
#   t=0.70 ..        brake fully released -> the release clock starts
#   t=1.20 ..        RELEASE_MIN_S elapsed: the earliest a resume may fire
#   t=3.70           RELEASE_MAX_S elapsed: the window closes, terminal refusal if it never fired
# The post block runs 5 s so every drive reaches its own terminal record.
RELEASE_T = 0.70


def normal_brake_and_resume(post_ticks=500, **overrides):
  """The reference drive: cruising -> brake (MADS holds lateral) -> release -> clear road.

  `overrides` apply to the brake block AND the post-release block (never to the capture block, so
  the driver's set speed is always observed first) -- that is what lets `mads_available=False` and
  the ARM edge itself is suppressed rather than only the fire."""
  d = Drive()
  d.tick(50)                                                        # capture the set speed
  brake = dict(lateral_only=True, op_enabled=False, cruise_enabled=False,
               brake_pressed=True, set_speed_ms=0.0)
  brake.update(overrides)
  d.tick(20, **brake)                                               # braking, MADS armed
  post = dict(lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  post.update(overrides)
  d.tick(post_ticks, **post)
  return d


# ---------------------------------------------------------------------------------------------
# The happy path -- everything below is a deviation from THIS.
# ---------------------------------------------------------------------------------------------

def test_happy_path_fires_once_with_the_captured_set_speed():
  d = normal_brake_and_resume()
  assert d.fired(), f"reference drive must resume; records={d.records}"
  assert d.phases().count("fire") == 1
  assert d.phases().count("refuse") == 0
  # gate 6: the offered set speed is EXACTLY the one captured before the brake, never higher.
  assert all(abs(o[2] - SET) < 1e-6 for o in d.offers)
  # one episode id for the whole offer -- the executor's one-shot key must not change under it.
  assert len({o[1] for o in d.offers}) == 1


def test_fire_waits_for_the_settle_time():
  d = normal_brake_and_resume()
  first = d.offers[0][0]
  assert first - RELEASE_T >= M.RELEASE_MIN_S - 1e-9, f"fired {first - RELEASE_T:.3f}s after release"
  assert first - RELEASE_T <= M.RELEASE_MAX_S + 1e-9


# ---------------------------------------------------------------------------------------------
# Gate 1 -- only from the brake-induced lateral-only state
# ---------------------------------------------------------------------------------------------

def test_gate1_never_arms_without_lateral_only():
  """A plain brake press with openpilot fully disengaged (no MADS latch) must do nothing at all."""
  d = Drive()
  d.tick(50)
  d.tick(20, op_enabled=False, cruise_enabled=False, brake_pressed=True, set_speed_ms=0.0)
  d.tick(200, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  assert not d.fired()
  assert d.records == [], "no lateral-only means the brain must not even arm"


def test_gate1_lateral_only_ending_aborts_the_window():
  d = Drive()
  d.tick(50)
  d.tick(20, lateral_only=True, op_enabled=False, cruise_enabled=False, brake_pressed=True, set_speed_ms=0.0)
  d.tick(30, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  d.tick(100, lateral_only=False, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  assert not d.fired()
  assert "latOff" in d.reasons("refuse")


# ---------------------------------------------------------------------------------------------
# Gate 2 -- only after the brake is FULLY released (brake AND regen)
# ---------------------------------------------------------------------------------------------

@pytest.mark.parametrize("held", ["brake_pressed", "regen_braking"])
def test_gate2_never_fires_while_still_braking(held):
  d = normal_brake_and_resume(**{held: True})
  assert not d.fired(), f"{held} still true -- must not resume"


def test_gate2_the_release_clock_starts_at_the_release_not_at_the_arm():
  """Gate 2 is enforced twice over -- the release clock only starts once the brake is fully up, AND
  a re-press after a release aborts outright. This pins the first half: a longer brake press must
  push the earliest possible fire out by exactly as much, never fire early off the arm time."""
  short = Drive()
  short.tick(50)
  short.tick(20, lateral_only=True, op_enabled=False, cruise_enabled=False, brake_pressed=True, set_speed_ms=0.0)
  short.tick(300, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  long_ = Drive()
  long_.tick(50)
  long_.tick(120, lateral_only=True, op_enabled=False, cruise_enabled=False, brake_pressed=True, set_speed_ms=0.0)
  long_.tick(300, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  assert short.fired() and long_.fired()
  assert long_.offers[0][0] - short.offers[0][0] == pytest.approx(1.0, abs=2 * DT)


def test_gate2_regen_alone_holds_the_release_clock():
  """Foot off the friction brake but regen still decelerating is NOT 'fully released'."""
  d = Drive()
  d.tick(50)
  d.tick(20, lateral_only=True, op_enabled=False, cruise_enabled=False, brake_pressed=True, set_speed_ms=0.0)
  d.tick(200, lateral_only=True, op_enabled=False, cruise_enabled=False, regen_braking=True, set_speed_ms=0.0)
  assert not d.fired()


# ---------------------------------------------------------------------------------------------
# Gate 3 -- bounded window after the release
# ---------------------------------------------------------------------------------------------

def test_gate3_window_closes_and_refuses():
  """A lead too close for the whole window -> the window expires with ONE explained refusal."""
  d = Drive()
  d.tick(50)
  d.tick(20, lateral_only=True, op_enabled=False, cruise_enabled=False, brake_pressed=True, set_speed_ms=0.0)
  d.tick(600, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0,
         has_lead=True, d_rel=15.0, v_lead=SET)
  assert not d.fired()
  assert d.phases().count("refuse") == 1, d.records
  assert d.reasons("refuse") == ["leadClose"]


def test_gate3_a_clear_road_after_the_window_is_too_late():
  d = Drive()
  d.tick(50)
  d.tick(20, lateral_only=True, op_enabled=False, cruise_enabled=False, brake_pressed=True, set_speed_ms=0.0)
  # blocked by a close lead until well past RELEASE_MAX_S...
  d.tick(int((M.RELEASE_MAX_S + 0.5) / DT), lateral_only=True, op_enabled=False,
         cruise_enabled=False, set_speed_ms=0.0, has_lead=True, d_rel=15.0, v_lead=SET)
  # ...then the road clears completely. Too late: the window is a hard bound.
  d.tick(500, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  assert not d.fired()


# ---------------------------------------------------------------------------------------------
# Gate 4 -- ONCE per brake event
# ---------------------------------------------------------------------------------------------

def test_gate4_only_one_offer_episode_per_arm():
  """The offer stops after OFFER_S and never restarts, even though every gate still passes."""
  d = normal_brake_and_resume()
  span = d.offers[-1][0] - d.offers[0][0]
  assert span <= M.OFFER_S + 2 * DT, f"offer ran {span:.2f}s, bound is {M.OFFER_S}s"
  assert d.phases().count("fire") == 1
  assert d.phases().count("offerEnd") == 1


def test_gate4_a_second_resume_needs_a_new_brake_to_lateral_only_cycle():
  d = normal_brake_and_resume()
  n_first = len(d.offers)
  assert n_first > 0
  # ONE fire record for this arm -- this is the assertion that pins "once", independently of how
  # long the caller happens to run: the window gate alone would also stop a re-fire eventually, and
  # an offer-count comparison taken after the window closed would silently absorb a broken latch.
  assert d.phases().count("fire") == 1
  # lateral-only stays true for a long time; nothing may fire again.
  d.tick(1000, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  assert len(d.offers) == n_first
  assert d.phases().count("fire") == 1
  # A genuinely NEW cycle: re-engage, then brake into lateral-only again.
  d.tick(100)                                                              # op re-engaged, cruise on
  d.tick(20, lateral_only=True, op_enabled=False, cruise_enabled=False,
         brake_pressed=True, set_speed_ms=0.0)
  d.tick(120, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  assert len(d.offers) > n_first, "a new brake->lateral-only cycle must be allowed to resume"


def test_double_brake_inside_the_optout_window_suppresses_the_resume():
  """brakeretry2pnw: two presses inside DOUBLE_BRAKE_S is the driver saying "leave it off"."""
  d = Drive()
  d.tick(50)
  d.tick(20, lateral_only=True, op_enabled=False, cruise_enabled=False, brake_pressed=True, set_speed_ms=0.0)
  d.tick(20, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)   # released
  # second press 0.4 s after the first -- well inside DOUBLE_BRAKE_S
  d.tick(20, lateral_only=True, op_enabled=False, cruise_enabled=False, brake_pressed=True, set_speed_ms=0.0)
  d.tick(400, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  assert not d.fired()
  assert "suppress" in d.phases(), d.records
  assert "doubleBrake" in d.reasons("suppress")


def test_a_later_brake_press_starts_a_FRESH_episode_and_can_resume():
  """The 2026-09-06 defect: arming required a lateral_only RISING edge, so one miss killed the
  feature for the rest of the drive. A second press -- outside the double-tap window -- must open a
  genuinely new episode and be able to fire."""
  d = Drive()
  d.tick(50)                                                                 # capture
  hold = dict(lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  # first press, aborted by a blocking event. (The accelerator no longer aborts -- since gasset2pnw
  # it switches the episode to SET mode instead; see the gas-set tests.)
  d.tick(20, brake_pressed=True, **hold)
  d.tick(20, blocked=True, **hold)
  assert not d.fired()
  assert "blocked" in d.reasons("refuse")
  # coast well past DOUBLE_BRAKE_S so the next press is a new intent, not a double-tap
  d.tick(200, **hold)
  # second press -> new arm -> release -> must resume
  d.tick(20, brake_pressed=True, **hold)
  d.tick(400, **hold)
  assert d.fired(), f"a later brake press must get its own resume; records={d.records}"
  assert d.offers[-1][2] == pytest.approx(SET)


def test_a_freeway_set_speed_is_not_resumed_in_a_slow_town():
  """Gemini finding A (BLOCK), reproduced exactly. Set 65 mph on the freeway, exit, drive several
  minutes on MADS lateral through town, then brake and release at ~15 mph. A purely time-bounded
  memory would hand back a 65 mph target on a residential street, where the lead gate protects
  nothing because there is no lead."""
  town = 7.0                                       # ~15 mph
  d = Drive()
  d.tick(50)                                       # freeway: capture SET (29 m/s, ~65 mph)
  hold = dict(lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  # exit and drive in town on MADS lateral, long enough for the rolling max to age out
  d.tick(int(1.5 * M.V_MAX_WINDOW_S / DT), v_ego=town, **hold)
  d.tick(20, brake_pressed=True, v_ego=town, **hold)
  d.tick(400, v_ego=town, **hold)
  assert not d.fired(), f"must NOT resume to a freeway speed in town; records={d.records[-3:]}"
  # both context guards independently refuse this; setFar is checked first
  assert {"setFar", "staleContext"} & set(d.reasons("refuse")), d.records[-3:]


def test_the_freeway_exit_then_yield_case_does_not_resume(): 
  """Fable finding A1, its exact probe. `staleContext` is time-scoped, so within 60 s of freeway
  speed it still permits this: brake 70 -> 25 mph down an off-ramp, release at the yield onto an
  arterial, and RES would target 70 mph from 11 m/s with cross traffic and no lead to gate on.
  Only the absolute delta cap bounds it."""
  d = Drive()
  d.tick(50)                                       # cruising at SET (29 m/s, ~65 mph)
  hold = dict(lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  ramp = 11.0                                      # ~25 mph at the yield
  d.tick(20, brake_pressed=True, v_ego=ramp, **hold)
  d.tick(400, v_ego=ramp, **hold)
  assert not d.fired(), f"must not resume 70 from 25 mph; records={d.records[-3:]}"
  assert "setFar" in d.reasons("refuse"), d.records[-3:]


def test_a_press_while_suppressed_still_leaves_a_record():
  """Fable finding F/P5 (Rule 2). A suppressed press used to produce NO record at all -- a new
  silent no-resume class, in a feature whose entire last investigation was diagnosing a no-resume."""
  d = Drive()
  d.tick(50)
  hold = dict(lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  d.tick(20, brake_pressed=True, **hold)
  d.tick(20, **hold)
  d.tick(20, brake_pressed=True, **hold)           # double tap -> suppressed
  assert d.b._suppressed
  d.tick(200, **hold)
  n = len(d.records)
  d.tick(20, brake_pressed=True, **hold)           # a further press while suppressed
  assert len(d.records) > n, "a suppressed press must not be silent"
  assert "suppressed" in d.reasons("refuse"), d.records[-2:]


def test_a_hard_brake_from_the_set_speed_still_resumes():
  """The guard above must not cost the MAIN case: braking hard for traffic and releasing, well
  below the set speed, with the set speed still a speed this drive has recently been doing."""
  d = Drive()
  d.tick(50)                                       # cruising at SET
  hold = dict(lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  slow = SET * 0.55                                # braked from ~65 mph down to ~36 mph
  d.tick(20, brake_pressed=True, v_ego=slow, **hold)
  d.tick(400, v_ego=slow, **hold)
  assert d.fired(), f"the main case must still resume; records={d.records[-3:]}"
  assert d.offers[-1][2] == pytest.approx(SET)


def test_braking_right_after_our_resume_stops_it_instead_of_queueing_another():
  """Gemini finding B. The driver's reflex against an unwanted resume is ONE firm brake. If that
  press re-armed, releasing it would surge again -- an unwinnable fight. A brake inside
  REJECT_AFTER_FIRE_S of our own fire latches the opt-out instead."""
  d = normal_brake_and_resume(post_ticks=100)     # fire lands ~1.2 s in; brake while it is fresh
  assert d.fired(), "precondition: the reference drive resumes"
  n_before = len(d.offers)
  hold = dict(lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  d.tick(20, brake_pressed=True, **hold)          # driver brakes to reject it, inside the window
  d.tick(400, **hold)                              # ... and releases
  assert len(d.offers) == n_before, f"must NOT resume again; records={d.records}"
  assert "postResumeBrake" in d.reasons("suppress"), d.records


def test_a_lingering_cruise_frame_does_not_wipe_the_optout():
  """Fable finding C/P2 -- the one that would have reached the road. `_suppressed` was cleared on
  the LEVEL of cruise_enabled, so a single frame where cruiseState.enabled still read True after
  the driver's rejection brake wiped the opt-out and the truck resumed again. mads_pnw.py:235-237
  says the PCM ordering is not guaranteed, so that frame is not hypothetical."""
  d = normal_brake_and_resume(post_ticks=100)     # fire lands ~1.2 s in; brake while it is fresh
  assert d.fired(), "precondition: the reference drive resumes"
  n_before = len(d.offers)
  hold = dict(lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  # the rejection brake, with stock cruise still reading enabled for two frames (PCM lag)
  d.tick(2, brake_pressed=True, lateral_only=True, op_enabled=False, cruise_enabled=True,
         set_speed_ms=SET)
  d.tick(18, brake_pressed=True, **hold)
  d.tick(400, **hold)
  assert d.b._suppressed, "the opt-out must survive a lingering cruise_enabled frame"
  assert len(d.offers) == n_before, f"must not resume again; offers={d.offers}"


def test_a_swallowed_chatter_repress_still_restarts_the_release_clock():
  """Fable finding B/P1. A re-press inside BRAKE_DEBOUNCE_S is swallowed as chatter for EDGE
  purposes, but it is still braking however long it is then held -- so it must restart the release
  clock. Otherwise a fire lands moments after the real release, skipping the settle entirely."""
  d = Drive()
  d.tick(50)
  hold = dict(lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  d.tick(20, brake_pressed=True, **hold)           # arm
  d.tick(10, **hold)                               # released 100 ms (< BRAKE_DEBOUNCE_S)
  d.tick(35, brake_pressed=True, **hold)           # re-press, HELD 350 ms
  # the real release starts here; nothing may fire for at least RELEASE_MIN_S after it
  t_release = d.t
  d.tick(400, **hold)
  if d.fired():
    first = min(o[0] for o in d.offers)
    assert first - t_release >= M.RELEASE_MIN_S - 1e-6, (
      f"fired {first - t_release:.3f}s after the real release, inside RELEASE_MIN_S={M.RELEASE_MIN_S}")


def test_pedal_chatter_is_not_a_double_tap():
  """Gemini finding C. A bouncing pedal must not be read as the driver's deliberate opt-out --
  that would silently kill the feature on a rough road."""
  d = Drive()
  d.tick(50)
  hold = dict(lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  d.tick(20, brake_pressed=True, **hold)
  d.tick(5, **hold)                                # 50 ms release -- bounce, under BRAKE_DEBOUNCE_S
  d.tick(20, brake_pressed=True, **hold)
  d.tick(400, **hold)
  assert "suppress" not in d.phases(), f"chatter must not latch the opt-out; records={d.records}"
  assert d.fired(), "and the resume must still happen"


def test_regen_flicker_is_not_a_brake_press():
  """Gemini finding C. Regen toggles as the driver modulates; each toggle must not read as a
  discrete brake PRESS. The gaps here are deliberately longer than BRAKE_DEBOUNCE_S, so the
  debounce cannot be what saves us -- if regen counted toward the edge, this regen rise would land
  inside DOUBLE_BRAKE_S of the real brake press and latch the opt-out."""
  d = Drive()
  d.tick(50)
  hold = dict(lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  d.tick(20, brake_pressed=True, **hold)           # the real press, at t
  d.tick(20, **hold)                               # released for 0.2 s (> BRAKE_DEBOUNCE_S)
  d.tick(20, regen_braking=True, **hold)           # regen rises 0.4 s after the press
  d.tick(400, **hold)
  assert "suppress" not in d.phases(), f"regen flicker must not latch the opt-out; records={d.records}"


def test_double_brake_suppression_clears_when_the_driver_re_engages_cruise():
  """"...until someone manually resumes or adjusts speeds" -- both engage stock cruise."""
  d = Drive()
  d.tick(50)
  d.tick(20, lateral_only=True, op_enabled=False, cruise_enabled=False, brake_pressed=True, set_speed_ms=0.0)
  d.tick(20, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  d.tick(20, lateral_only=True, op_enabled=False, cruise_enabled=False, brake_pressed=True, set_speed_ms=0.0)
  d.tick(100, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  assert d.b._suppressed, "double tap must latch the opt-out"
  d.tick(20, lateral_only=True, op_enabled=False, cruise_enabled=True, set_speed_ms=SET)   # driver resumes
  assert not d.b._suppressed, "manual re-engage must clear the opt-out"


# ---------------------------------------------------------------------------------------------
# Gate 5 -- never with a close lead or low TTC
# ---------------------------------------------------------------------------------------------

def test_gate5_close_lead_blocks():
  d = normal_brake_and_resume(has_lead=True, d_rel=18.0, v_lead=SET)
  assert not d.fired()


def test_gate5_short_headway_blocks():
  # 40 m at 29 m/s = 1.38 s headway, under the 2.0 s bar, but not under the 20 m floor.
  d = normal_brake_and_resume(has_lead=True, d_rel=40.0, v_lead=SET)
  assert not d.fired()
  assert "leadGap" in d.reasons("refuse")


def test_gate5_fast_closing_lead_blocks_even_at_a_long_gap():
  # 100 m (3.4 s headway, passes) but closing at 20 m/s -> TTC 5 s, under the 8 s bar.
  d = normal_brake_and_resume(has_lead=True, d_rel=100.0, v_lead=SET - 20.0)
  assert not d.fired()
  assert "leadTtc" in d.reasons("refuse")


def test_gate5_open_road_behind_a_matched_lead_is_allowed():
  # 90 m at matched speed: 3.1 s headway, no closing rate. This is what "clear enough" means.
  d = normal_brake_and_resume(has_lead=True, d_rel=90.0, v_lead=SET)
  assert d.fired()


def test_gate5_a_failed_radar_read_is_a_refusal_not_an_open_road():
  """CLAUDE.md rule 2: an error is not a negative result. has_lead=None must REFUSE."""
  d = normal_brake_and_resume(has_lead=None)
  assert not d.fired()
  assert "leadUnknown" in d.reasons("refuse")


def test_lead_gate_pure():
  assert lead_gate(None, None, None, 30.0) == "leadUnknown"
  assert lead_gate(False, None, None, 30.0) is None
  assert lead_gate(True, 10.0, 30.0, 30.0) == "leadClose"
  assert lead_gate(True, 40.0, 30.0, 30.0) == "leadGap"
  assert lead_gate(True, 100.0, 10.0, 30.0) == "leadTtc"
  assert lead_gate(True, 100.0, 30.0, 30.0) is None
  assert lead_gate(True, float("nan"), 30.0, 30.0) == "leadUnknown"


# ---------------------------------------------------------------------------------------------
# Gate 6 -- the driver's PREVIOUS set speed, never above it, refuse if unknown
# ---------------------------------------------------------------------------------------------

def test_gate6_no_captured_set_speed_refuses_immediately_at_arm():
  """Cruise was never engaged, so no set speed was ever observed -> refuse at the arm tick."""
  d = Drive()
  d.tick(50, cruise_enabled=False, set_speed_ms=0.0)
  d.tick(20, lateral_only=True, op_enabled=False, cruise_enabled=False, brake_pressed=True, set_speed_ms=0.0)
  d.tick(400, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  assert not d.fired()
  assert d.reasons("refuse") == ["noSet"]
  assert d.records[0]["phase"] == "arm" and d.records[0]["setMs"] is None


def test_gate6_a_stale_capture_refuses():
  """Cruise off for longer than SET_MAX_AGE_S before the brake -> the capture is not this event's."""
  d = Drive()
  d.tick(50)
  d.tick(int((M.SET_MAX_AGE_S + 0.5) / DT), cruise_enabled=False, set_speed_ms=0.0, op_enabled=False)
  d.tick(20, lateral_only=True, op_enabled=False, cruise_enabled=False, brake_pressed=True, set_speed_ms=0.0)
  d.tick(400, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  assert not d.fired()
  assert "noSet" in d.reasons("refuse")


def test_gate6_a_set_speed_below_the_sanity_floor_is_not_a_set_speed():
  d = Drive()
  # v_ego stays resumable throughout, so the SPEED floor cannot be what refuses -- this test is
  # about the CAPTURE floor, and `slow` is evaluated before `noSet`.
  d.tick(50, set_speed_ms=3.0, v_ego=12.0)         # reported set 3 m/s ~ 7 mph, below SET_MIN_MS
  d.tick(20, lateral_only=True, op_enabled=False, cruise_enabled=False, brake_pressed=True,
         set_speed_ms=0.0, v_ego=12.0)
  d.tick(400, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0, v_ego=12.0)
  assert not d.fired()
  assert "noSet" in d.reasons("refuse")


def test_gate6_a_real_15mph_set_speed_is_accepted():
  """MEASURED on the truck 2026-09-06: stock ACC was engaged with set speeds of 15-19 mph (minimum
  6.71 m/s, 34 ticks). The old 20 mph floor -- justified as "Ford's ACC will not hold a set speed
  below 20 mph" -- was false for this car and silently refused the driver's real city speeds."""
  low = 6.71                                        # 15 mph, the measured minimum
  d = Drive()
  d.tick(50, set_speed_ms=low, v_ego=low)
  d.tick(20, lateral_only=True, op_enabled=False, cruise_enabled=False, brake_pressed=True,
         set_speed_ms=0.0, v_ego=low)
  d.tick(400, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0, v_ego=low)
  assert d.fired(), f"a real 15 mph set speed must be resumable; records={d.records}"
  assert d.offers[-1][2] == pytest.approx(low)


def test_gate6_a_higher_reported_set_speed_blocks():
  """If the truck reports a set speed ABOVE the captured one, resuming could go above it."""
  d = normal_brake_and_resume(set_speed_ms=SET + 3.0)
  assert not d.fired()
  assert "setRaised" in d.reasons("refuse")


def test_gate6_a_nonfinite_reported_set_speed_refuses():
  """A gate whose input is unreadable fails CLOSED -- `isfinite(x) and x > t` would permit on NaN."""
  for bad in (float("nan"), float("inf")):
    d = normal_brake_and_resume(set_speed_ms=bad)
    assert not d.fired()
    assert "setUnknown" in d.reasons("refuse")


def test_gate6_a_lower_reported_set_speed_is_fine():
  """Resume can only ever go to the PCM's own remembered set; lower than captured is not 'above'."""
  d = normal_brake_and_resume(set_speed_ms=SET - 3.0)
  assert d.fired()


def test_gate6_the_offer_is_never_above_the_captured_set_speed():
  """Exhaustive over the reference drive: no offer may exceed the captured value, ever."""
  d = normal_brake_and_resume()
  assert d.offers and all(o[2] <= SET + 1e-9 for o in d.offers)


def test_verify_records_a_resume_that_came_back_too_high():
  d = normal_brake_and_resume()
  assert d.fired()
  # cruise comes back, but at 5 m/s above what the driver had set.
  d.tick(10, lateral_only=False, op_enabled=False, cruise_enabled=True, set_speed_ms=SET + 5.0)
  verify = [r for r in d.records if r["phase"] == "verify"]
  assert len(verify) == 1 and verify[0]["reason"] == "setHigher" and verify[0]["loud"] is True


def test_verify_records_a_correct_resume_quietly():
  d = normal_brake_and_resume()
  d.tick(10, lateral_only=False, op_enabled=False, cruise_enabled=True, set_speed_ms=SET)
  verify = [r for r in d.records if r["phase"] == "verify"]
  assert len(verify) == 1 and verify[0]["reason"] == "ok" and "loud" not in verify[0]


# ---------------------------------------------------------------------------------------------
# Gate 7 -- aborts
# ---------------------------------------------------------------------------------------------

@pytest.mark.parametrize("override,reason", [
  ({"blocked": True}, "blocked"),
  ({"op_enabled": True}, "opEngaged"),
  ({"cruise_available": False}, "accOff"),
  ({"cruise_enabled": True}, "ccOn"),
])
def test_gate7_aborts(override, reason):
  d = normal_brake_and_resume(**override)
  assert not d.fired(), f"{reason}: must not resume"
  assert reason in d.reasons("refuse"), d.records


def test_gate7_arm_expiry_bounds_a_long_lateral_only():
  """Lateral-only can last indefinitely; a pending resume must not."""
  d = Drive()
  d.tick(50)
  d.tick(int((M.ARM_MAX_S + 1.0) / DT), lateral_only=True, op_enabled=False,
         cruise_enabled=False, brake_pressed=True, set_speed_ms=0.0)
  assert not d.fired()
  assert "armExpired" in d.reasons("refuse")


def test_gate7_gas_during_the_offer_withdraws_it():
  d = normal_brake_and_resume(post_ticks=60)      # stop 0.1 s into the 1.0 s offer
  n = len(d.offers)
  assert n > 0 and n < int(M.OFFER_S / DT), "fixture must stop mid-offer"
  d.tick(50, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0,
         gas_pressed=True)
  assert len(d.offers) == n, "the offer must be withdrawn the instant the driver touches the gas"
  assert "gas" in [r.get("reason") for r in d.records if r["phase"] == "offerEnd"]


def test_low_speed_refuses():
  d = Drive()
  d.tick(50, v_ego=3.0)                             # ~7 mph, below V_EGO_MIN_MS
  d.tick(20, lateral_only=True, op_enabled=False, cruise_enabled=False, brake_pressed=True,
         set_speed_ms=0.0, v_ego=3.0)
  d.tick(400, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0, v_ego=3.0)
  assert not d.fired()


# ---------------------------------------------------------------------------------------------
# Gate 8 -- inert unless MADS is available (so it can never act on the Tesla), and the kill switch
# ---------------------------------------------------------------------------------------------

def test_gate8_inert_without_mads():
  d = normal_brake_and_resume(mads_available=False)
  assert not d.fired()
  assert d.records == [], "no MADS -> not even a log record"


def test_gate8_inert_without_mads_even_on_the_arming_tick():
  """mads_available False must also suppress the ARM edge, not just the fire."""
  d = Drive(mads_available=False)
  d.tick(50)
  d.tick(20, lateral_only=True, op_enabled=False, cruise_enabled=False, brake_pressed=True, set_speed_ms=0.0)
  d.tick(400, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  assert not d.fired() and d.records == []


def test_a_first_observation_of_lateral_only_is_not_a_rising_edge():
  """selfdrived restarting mid-drive, or the toggle flipped on while ALREADY steering-only, must
  not read as a brake transition. The edge detector is three-state: None != observed-False."""
  d = Drive()
  d.tick(50)                                                     # capture a set speed
  # first tick the brain ever sees lateral_only is already True -- no brake edge was ever observed
  b = MadsResumeBrain()
  out = b.update(mk(0.0, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0))
  assert out.offer is False and out.records == [], "a first observation must only SEED the detector"


def test_mads_becoming_available_mid_drive_while_lateral_only_does_not_arm():
  d = Drive()
  d.tick(50)
  d.tick(200, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0,
         mads_available=False)
  d.tick(600, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  assert not d.fired() and d.records == []


def test_capture_age_bound_still_covers_the_mads_brake_grace_window():
  """SET_MAX_AGE_S must still exceed mads_pnw.MADS_BRAKE_GRACE_FRAMES -- the brake may land that
  many frames after the falling edge, with the capture already stopped. This module cannot import
  mads_pnw (that would drag in cereal), so the coupling is pinned by reading the source text.

  The old UPPER bound (grace + 0.5 s) is deliberately gone. It was sized against a hazard that is
  handled one layer up -- a stalk CANCEL is not in MADS_TOLERATED_EVENTS, so `blocked` is True at
  the falling edge and no episode is handed to this module at all -- and it was the direct cause of
  the 2026-09-06 `noSet` cascade. What remains is a finite backstop, pinned here so the memory can
  never become unbounded."""
  import pathlib
  import re
  src = (pathlib.Path(M.__file__).parent.parent.parent / "selfdrived" / "mads_pnw.py").read_text()
  m = re.search(r"^MADS_BRAKE_GRACE_FRAMES = (\d+)$", src, re.M)
  assert m, "could not find MADS_BRAKE_GRACE_FRAMES in mads_pnw.py"
  grace_s = int(m.group(1)) * DT
  assert M.SET_MAX_AGE_S > grace_s, (
    f"SET_MAX_AGE_S={M.SET_MAX_AGE_S}s must exceed the MADS brake grace window ({grace_s}s) or a legitimate late-brake arm would refuse with noSet")
  assert 0.0 < M.SET_MAX_AGE_S <= 900.0, (
    f"SET_MAX_AGE_S={M.SET_MAX_AGE_S}s must stay a FINITE backstop; an unbounded memory could resurface a set speed from a different road entirely")


def test_the_capture_survives_the_cruise_off_gap_between_two_brakes():
  """THE 2026-09-06 DEFECT, pinned. The capture only refreshes while stock cruise is engaged, and
  a brake turns cruise off -- so after the first miss nothing refreshes it. With the old 0.75 s
  bound, every later brake in the drive refused `noSet` (observed setAgeS 22.15 s, then -1.0).
  The driver had to manually re-engage cruise to get the feature back at all."""
  d = Drive()
  d.tick(50)                                                                 # capture the set speed
  hold = dict(lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  d.tick(20, brake_pressed=True, **hold)                                     # first brake
  d.tick(20, gas_pressed=True, **hold)                                       # gasset2pnw: switches to SET mode
  d.tick(3000, **hold)                                                       # 30 s with cruise OFF
  d.tick(20, brake_pressed=True, **hold)                                     # brake again
  d.tick(400, **hold)
  arms = [r for r in d.records if r["phase"] == "arm"]
  assert arms[-1]["setMs"] == pytest.approx(SET), (
    f"the driver's set speed must survive the cruise-off gap; last arm={arms[-1]}")
  assert "noSet" not in d.reasons("refuse"), d.records
  assert d.fired()


def test_the_capture_is_forgotten_when_the_acc_master_goes_off():
  """The one thing that DOES invalidate the memory: the driver switching cruise off entirely."""
  d = Drive()
  d.tick(50)                                                                 # capture
  d.tick(50, cruise_enabled=False, cruise_available=False)                   # master off
  d.tick(20, lateral_only=True, op_enabled=False, cruise_enabled=False, cruise_available=False,
         brake_pressed=True, set_speed_ms=0.0)
  d.tick(400, lateral_only=True, op_enabled=False, cruise_enabled=False, cruise_available=False,
         set_speed_ms=0.0)
  assert not d.fired()
  # and it must name the SPECIFIC cause, not the noSet it caused one level down
  assert "accOff" in d.reasons("refuse"), d.records


def test_the_forgotten_capture_is_not_resurrected_when_the_master_comes_back():
  """The distinguishing case for the clear itself. The test above cannot see it: with the master
  still OFF at arm time, the abort chain reports `accOff` whether or not the memory was cleared, so
  it passes either way (mutation survived, 2026-09-06). Here the master goes off and comes back ON
  without cruise ever being re-engaged -- so `accOff` no longer applies, and the ONLY thing that can
  refuse is the memory having been genuinely forgotten."""
  d = Drive()
  d.tick(50)                                                                 # capture SET
  d.tick(50, cruise_enabled=False, cruise_available=False)                   # master OFF -> forget
  d.tick(50, cruise_enabled=False, cruise_available=True)                    # master back ON, cruise idle
  d.tick(20, lateral_only=True, op_enabled=False, cruise_enabled=False, brake_pressed=True,
         set_speed_ms=0.0)
  d.tick(400, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  assert not d.fired(), "a set speed from before an explicit master-off must not be resurrected"
  assert "noSet" in d.reasons("refuse"), d.records


def test_a_late_brake_inside_the_mads_grace_window_still_captures_the_set_speed():
  """The whole reason SET_MAX_AGE_S is not tiny: MADS may arm up to 0.45 s after cruise dropped."""
  d = Drive()
  d.tick(50)                                                     # cruise on, set speed captured
  # cruise drops first (the measured pedal lead), brake lands 0.40 s later, MADS arms then.
  d.tick(40, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  d.tick(20, lateral_only=True, op_enabled=False, cruise_enabled=False, brake_pressed=True,
         set_speed_ms=0.0)
  d.tick(500, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  assert d.fired(), f"a late brake inside the MADS grace window must still resume; {d.records}"
  assert all(abs(o[2] - SET) < 1e-6 for o in d.offers)


def test_gasset_lifting_off_the_accelerator_sets_that_speed():
  """gasset2pnw, the driver's own design: "if I have been braking and I then accelerate with the
  gas, when I stop accelerating can't that be the speed that is then set". The target is the speed
  they reached, and the button is SET (not RESUME), because no remembered speed is involved."""
  d = Drive()
  d.tick(50)                                                   # capture SET (29 m/s) -- unused here
  hold = dict(lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  d.tick(20, brake_pressed=True, v_ego=14.0, **hold)           # brake
  d.tick(60, gas_pressed=True, v_ego=18.0, **hold)             # driver picks a speed on the pedal
  d.tick(400, v_ego=18.0, **hold)                              # ... and lifts off
  assert d.fired(), f"lifting off must set that speed; records={d.records[-3:]}"
  eid, target = d.offers[-1][1], d.offers[-1][2]
  assert target == pytest.approx(18.0), f"must target the gas-chosen speed, got {target}"
  fires = [r for r in d.records if r["phase"] == "fire"]
  assert fires[-1]["mode"] == "set", f"must tap SET, not RESUME: {fires[-1]}"


def test_gasset_works_with_no_remembered_set_speed_at_all():
  """The 17:11:00 and 17:13:42 refusals on the 2026-09-06 drive were `noSet` -- no capture existed,
  and under the old design nothing could bring cruise back but the driver doing it by hand. The
  accelerator route consults no memory, so it works precisely where RESUME cannot."""
  d = Drive()
  d.tick(50, cruise_enabled=False, set_speed_ms=0.0)           # cruise NEVER engaged -> no capture
  hold = dict(lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  d.tick(20, brake_pressed=True, v_ego=12.0, **hold)
  d.tick(60, gas_pressed=True, v_ego=16.0, **hold)
  d.tick(400, v_ego=16.0, **hold)
  assert d.fired(), f"gas-set needs no captured speed; records={d.records[-3:]}"
  assert d.offers[-1][2] == pytest.approx(16.0)
  assert "noSet" not in d.reasons("refuse"), d.records


def test_the_arm_survives_a_long_acceleration():
  """A freeway on-ramp is easily a 20 s pull. ARM_MAX_S measured from the brake would expire the
  episode before the driver ever lifted off, losing exactly the case gas-set exists for."""
  d = Drive()
  d.tick(50)
  hold = dict(lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  d.tick(20, brake_pressed=True, v_ego=12.0, **hold)
  d.tick(int((M.ARM_MAX_S + 5.0) / DT), gas_pressed=True, v_ego=25.0, **hold)   # 25 s on the power
  d.tick(400, v_ego=25.0, **hold)
  assert d.fired(), f"a long acceleration must not expire the episode; records={d.records[-3:]}"
  assert d.offers[-1][2] == pytest.approx(25.0)


def test_gas_after_a_fire_gets_a_NEW_episode_id():
  """Fable D1. The executor latches one press per `eid`. Re-opening an arm in place kept the eid, so
  the SET was silently refused by the executor while the brain logged `fire` -- and selfdrived then
  warned about the panda safety pin, pointing at the wrong subsystem entirely."""
  d = Drive()
  d.tick(50)                                                   # capture SET
  hold = dict(lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  d.tick(20, brake_pressed=True, **hold)
  d.tick(200, **hold)                                          # RES fires here
  assert d.fired(), "precondition: the resume fires"
  first_eid = d.offers[-1][1]
  d.tick(60, gas_pressed=True, v_ego=SET, **hold)              # back on the power
  d.tick(400, v_ego=SET, **hold)                               # lift -> SET must fire
  assert len(d.offers) > 1, f"the gas-set must produce a second offer; records={d.records[-4:]}"
  assert d.offers[-1][1] != first_eid, \
    f"the second press MUST carry a new eid or the executor refuses it silently (both {first_eid})"


def test_gas_after_an_EXPIRED_offer_still_sets():
  """Fable D3. If the resume offer expired without the PCM acting, `_done` stayed True and
  `if self._done: return` killed the gas-set outright -- no press, no record, arm dying in silence
  20 s later."""
  d = Drive()
  d.tick(50)
  hold = dict(lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  d.tick(20, brake_pressed=True, **hold)
  d.tick(500, **hold)                                          # fire, then the offer expires
  n_before = len(d.offers)
  d.tick(60, gas_pressed=True, v_ego=SET, **hold)
  d.tick(400, v_ego=SET, **hold)
  assert len(d.offers) > n_before, f"gas after an expired offer must still set; {d.records[-4:]}"
  fires = [r for r in d.records if r["phase"] == "fire"]
  assert fires[-1]["mode"] == "set"


def test_the_verify_record_names_the_button_that_was_ACTUALLY_pressed():
  """Fable round 3, C1. `_snap` fell back to the LIVE `_used_gas`, and a gas-reopen clears
  `_fired_mode` -- so the RESUME press's verify, arriving up to VERIFY_S later, reported "set".
  The selfdrived warning prints that field, so it would have announced the wrong button for a
  press that never happened: exactly the wild-goose chase the warning exists to prevent.

  My previous attempt at this test asserted `mode in (None, "res", "set")`, which CANNOT FAIL, and
  stopped before the verify ever landed. This one ticks past VERIFY_S and pins the value."""
  d = Drive()
  d.tick(50)
  hold = dict(lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  d.tick(20, brake_pressed=True, **hold)
  d.tick(200, **hold)                                          # RESUME fires
  fires = [r for r in d.records if r["phase"] == "fire"]
  assert fires and fires[0]["mode"] == "res", "precondition: a RESUME fired"
  # the driver now goes back on the power, which reopens the episode in SET mode
  d.tick(60, gas_pressed=True, v_ego=SET, **hold)
  assert d.b._used_gas, "precondition: the episode is now in gas mode"
  # run past VERIFY_S with cruise never returning, so the RESUME's verify lands
  d.tick(int((M.VERIFY_S + 1.0) / DT), v_ego=SET, **hold)
  verifies = [r for r in d.records if r["phase"] == "verify"]
  assert verifies, "the resume press must be verified"
  assert verifies[0]["mode"] == "res", \
    f"verify for a RESUME must say 'res', got {verifies[0]['mode']!r} (reports an unpressed button)"


def test_gasset_sets_while_regen_is_still_slowing_the_truck():
  """OWNER DECISION 2026-09-13, "Ignore regen, set". This used to refuse `slowing` (Fable C, 2026-09-07).
  The weekend measured 1.5-1.9 m/s^2 of regen within ~1 s of every steering-only gas lift-off, so the gate
  refused half of them; a SET- to the current speed commands no acceleration. The first press now sets at
  the speed the truck is doing when the tap is offered."""
  d = Drive()
  d.tick(50)
  d.tick(20, brake_pressed=True, v_ego=25.0, **STEER_ONLY)
  d.tick(60, gas_pressed=True, v_ego=25.0, **STEER_ONLY)
  v = 25.0
  speeds = []
  for _ in range(400):                                               # lift off, measured regen
    v = max(12.0, v - REGEN_MS2 * DT)
    speeds.append((round(d.t, 3), v))
    d.tick(1, v_ego=v, **STEER_ONLY)
  assert d.fired(), f"regen must not refuse a gas-set; records={d.records[-3:]}"
  fire = [r for r in d.records if r["phase"] == "fire"]
  assert len(fire) == 1 and fire[0]["mode"] == "set" and fire[0]["decel"] > 1.5, fire
  t0, target = d.offers[0][0], d.offers[0][2]
  assert target == pytest.approx(dict(speeds)[t0]), "SET targets the speed at the moment of the offer"
  assert "slowing" not in [r.get("reason") for r in d.records], d.records


def test_regen_still_does_not_touch_the_RESUME_path():
  """Scope of the owner decision: only the gas-set path. The RES path never had a decel gate, and still
  has none -- D3 is undecided -- so a brake release under regen resumes exactly as before."""
  d = normal_brake_and_resume(post_ticks=0)
  v = SET
  for _ in range(500):
    v = max(20.0, v - 1.8 * DT)
    d.tick(1, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0, v_ego=v)
  fires = [r for r in d.records if r["phase"] == "fire"]
  assert len(fires) == 1 and fires[0]["mode"] == "res", d.records


def test_lift_off_then_an_immediate_hard_brake_never_sets():
  """The real braking case the old decel gate was also standing in front of: the driver lifts the
  accelerator and brakes hard at once. With the gate gone, the brake itself must still win: the release
  clock needs BOTH pedals up for RELEASE_MIN_S, and the brake edge re-arms a brake episode."""
  for brake_after_s in (0.05, 0.2, M.RELEASE_MIN_S - 0.05):
    d = Drive()
    d.tick(50)
    d.tick(20, brake_pressed=True, v_ego=20.0, **STEER_ONLY)
    d.tick(200, gas_pressed=True, v_ego=20.0, **STEER_ONLY)
    v = 20.0
    for _ in range(int(brake_after_s / DT)):
      v -= 1.8 * DT
      d.tick(1, v_ego=v, **STEER_ONLY)
    for _ in range(300):                                             # hard brake, 3 s at 4 m/s^2
      v = max(0.0, v - 4.0 * DT)
      d.tick(1, brake_pressed=True, v_ego=v, standstill=v < 0.1, **STEER_ONLY)
    assert not d.offers, f"brake {brake_after_s}s after lift-off: offers={d.offers} records={d.records}"
    assert "reBrake" in d.reasons("refuse"), f"the gas episode must end on the brake; {d.phases()}"


def _lift_then_brake(brake_after_s, v0=9.4):
  """Sat 2026-09-12 12:41:50 PT, parameters from the qlog: pulling away after a brake, the driver lifts at
  ~21 mph, regen slows the truck (measured 1.5-1.9 m/s^2), and he brakes `brake_after_s` after lift-off."""
  d = Drive()
  d.tick(50)
  d.tick(20, brake_pressed=True, v_ego=12.0, **STEER_ONLY)
  d.tick(300, gas_pressed=True, v_ego=v0, **STEER_ONLY)
  v = v0
  for _ in range(int(round(brake_after_s / DT))):
    v -= REGEN_MS2 * DT
    d.tick(1, v_ego=v, **STEER_ONLY)
  n_off = len(d.offers)
  d.tick(150, brake_pressed=True, v_ego=max(v - 2.0, 0.0), **STEER_ONLY)
  return d, n_off


@pytest.mark.parametrize("brake_after_s", [0.69, 0.9])
def test_a_brake_inside_the_gas_set_wait_wins_and_nothing_is_set(brake_after_s):
  """OWNER DECISION 2026-09-13, "Wait 1.0 s" (Q-C5). At 0.69 s (the weekend's stop) and 0.9 s the brake lands
  inside GAS_SET_RELEASE_MIN_S: no SET is offered, the brake is NOT read as a rejection, and the automatic
  resume stays on for the stretch (the brake simply re-arms)."""
  d, n_off = _lift_then_brake(brake_after_s)
  assert n_off == 0 and not d.offers, f"a SET was offered before a {brake_after_s}s brake; {d.records}"
  assert not d.b._suppressed and "postResumeBrake" not in d.reasons("suppress"), d.phases()
  assert d.reasons("refuse")[-1] == "reBrake" and d.records[-1]["phase"] == "arm", d.phases()


def test_a_brake_after_the_gas_set_wait_lands_after_the_set_and_cancels_it():
  """1.1 s: the SET is offered at 1.0 s, then the brake withdraws the offer on its first tick (the executor also
  refuses a press with a pedal down, and the PCM drops ACC on the brake) and reads as a rejection of it."""
  d, n_off = _lift_then_brake(1.1)
  assert n_off > 0, "the SET must be offered once the 1.0 s wait has passed"
  assert len(d.offers) == n_off, "the offer must be withdrawn on the brake tick"
  assert "postResumeBrake" in d.reasons("suppress"), d.phases()


def test_the_gas_set_waits_1s_after_lift_off_and_RESUME_still_waits_0p5s():
  d = Drive()
  _red_light(d, 5.0)
  d.tick(200, gas_pressed=True, v_ego=16.0, **STEER_ONLY)
  lift = d.t
  d.tick(400, v_ego=16.0, **STEER_ONLY)
  assert d.fired()
  assert M.GAS_SET_RELEASE_MIN_S - 1e-9 <= d.offers[0][0] - lift <= M.GAS_SET_RELEASE_MIN_S + 0.02, d.offers[0][0] - lift
  r = normal_brake_and_resume()
  assert M.RELEASE_MIN_S - 1e-9 <= r.offers[0][0] - RELEASE_T <= M.RELEASE_MIN_S + 0.02, "RESUME timing must not change"


def test_a_lift_shorter_than_the_gas_set_wait_does_not_use_up_the_first_press():
  """First press only counts a press as used once its lift-off is judged -- now at 1.0 s. A 0.7 s lift and back
  on the power is still the first press, and the real lift-off afterwards sets."""
  d = Drive()
  _red_light(d, 5.0)
  d.tick(100, gas_pressed=True, v_ego=14.0, **STEER_ONLY)
  d.tick(70, v_ego=14.0, **STEER_ONLY)                              # 0.7 s lift
  assert not d.b._gas_spent and not d.fired(), "precondition"
  _pull_away_and_lift(d, gas_s=1.0, v=15.0)
  assert d.fired() and d.offers[-1][2] == pytest.approx(15.0), d.records[-4:]


@pytest.mark.parametrize("poll_phase", range(25))
def test_the_executor_idle_poll_cannot_press_before_the_gas_set_wait(poll_phase):
  """The seam to opendbc: the executor only ever presses on a PUBLISHED offer. Its idle cadence
  (carcontroller._resume_button: `if self._resume_cmd is not None or (self.frame % 25) == 0`, reproduced here)
  can delay the first pressed frame by up to 250 ms, never advance it. Real parser, decision and one-shot latch
  from opendbc/car/ford/icbm_pnw.py; every poll phase."""
  from opendbc.car.ford.icbm_pnw import ResumePress, decide_resume, parse_resume_cmd
  b, press = MadsResumeBrain(), ResumePress()
  mem, cmd, first_press, lift = {}, None, None, None
  t = 0.0
  plan = [dict(n=50), dict(n=20, brake_pressed=True, **STEER_ONLY), dict(n=300, v_ego=0.0, standstill=True, **STEER_ONLY),
          dict(n=200, gas_pressed=True, v_ego=16.0, **STEER_ONLY), dict(n=400, v_ego=16.0, **STEER_ONLY)]
  frame = poll_phase
  for k, step in enumerate(plan):
    kw = dict(step)
    n = kw.pop("n")
    if k == len(plan) - 1:
      lift = t                                                       # first tick with both pedals up
    for _ in range(n):
      out = b.update(mk(t, **kw))
      if out.offer:
        mem = {"dir": out.mode, "ts": round(t, 3), "eid": out.eid, "set": round(out.set_ms, 2)}
      else:
        mem = {}
      if cmd is not None or frame % 25 == 0:
        cmd = parse_resume_cmd(mem)
      ok = decide_resume(cmd, t, bool(kw.get("cruise_enabled", True)), True,
                         bool(kw.get("gas_pressed", False) or kw.get("brake_pressed", False)), float(kw.get("set_speed_ms", SET)))
      if press.update(frame, cmd, ok) and first_press is None:
        first_press = t
      t += DT
      frame += 1
  assert lift is not None and first_press is not None, "the gas-set must reach the executor"
  assert first_press - lift >= M.GAS_SET_RELEASE_MIN_S - 1e-9, f"pressed {first_press - lift:.2f}s after lift-off"
  assert first_press - lift <= M.GAS_SET_RELEASE_MIN_S + 0.26, "the idle poll delays by at most 250 ms"


def test_the_noCruise_verify_names_the_RESUME_that_was_pressed():
  """Fable round 4, scenario P1. My previous attempt at this claimed to pin the `noCruise` record
  but never reached it: the SET fired and SUPERSEDED the RESUME's window, so the only record the
  test ever saw was the `superseded` one -- added in the same round. Mutating the `noCruise` site
  back to live `_used_gas` left it passing.

  Here the SET is kept from firing after the reopen (a close lead), so the RESUME's own window runs
  out and its `noCruise` record actually lands."""
  d = Drive()
  d.tick(50)
  hold = dict(lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  d.tick(20, brake_pressed=True, **hold)
  d.tick(200, **hold)                                          # RESUME fires
  assert [r for r in d.records if r["phase"] == "fire"], "precondition: a RESUME fired"
  d.tick(60, gas_pressed=True, v_ego=SET, **hold)              # reopen in gas mode
  # a close lead keeps the SET from ever firing, so nothing supersedes the RESUME's verify window
  blocked = dict(has_lead=True, d_rel=12.0, v_lead=SET, v_ego=SET)
  d.tick(int((M.VERIFY_S + 1.0) / DT), **blocked, **hold)
  nc = [r for r in d.records if r["phase"] == "verify" and r["reason"] == "noCruise"]
  assert nc, f"the RESUME press must be verified; phases={d.phases()[-6:]}"
  assert nc[0]["mode"] == "res", (
    f"a noCruise verify for a RESUME press must say 'res', got {nc[0]['mode']!r}")


def test_the_got_want_verify_names_the_RESUME_and_uses_its_tolerance():
  """Fable round 4, scenario P2. Pins BOTH the mode on the got/want verify record AND the tolerance
  choice: a RESUME must be judged against SET_TOL_MS (tight, it restores a remembered value), not
  the looser SET_MODE_TOL_MS. Cruise returns 0.8 m/s above the captured speed -- inside the SET
  tolerance, outside the RESUME one -- so a wrong `tol` reports 'ok' instead of 'setHigher'."""
  d = Drive()
  d.tick(50)
  hold = dict(lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  d.tick(20, brake_pressed=True, **hold)
  d.tick(200, **hold)                                          # RESUME fires
  d.tick(60, gas_pressed=True, v_ego=SET, **hold)              # reopen -> _used_gas True
  d.tick(20, v_ego=SET, **hold)
  # stock cruise comes back ABOVE the captured set speed, while the episode is in gas mode
  d.tick(20, lateral_only=True, op_enabled=False, cruise_enabled=True,
         set_speed_ms=SET + 0.8, v_ego=SET)
  vs = [r for r in d.records if r["phase"] == "verify" and r["reason"] in ("setHigher", "ok")]
  assert vs, f"the resume must be verified once cruise returns; phases={d.phases()[-6:]}"
  assert vs[0]["reason"] == "setHigher", \
    f"0.8 m/s above capture is outside SET_TOL_MS; a RESUME must not use the SET tolerance: {vs[0]['reason']!r}"
  assert vs[0]["mode"] == "res", f"must name the RESUME, got {vs[0]['mode']!r}"


def test_gasset_refuses_when_the_decel_measurement_is_not_fresh():
  """Gemini round 3, finding B. The estimator resamples on its own cadence, unaligned to the driver,
  so the most recent completed window can straddle the accelerator -- where the truck was speeding
  UP and `_decel` reads negative, waving through the very lift-off the gate exists to catch. A
  measurement not taken entirely after both pedals came up is a REFUSAL."""
  d = Drive()
  d.tick(50)
  hold = dict(lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  d.tick(20, brake_pressed=True, v_ego=18.0, **hold)
  d.tick(60, gas_pressed=True, v_ego=18.0, **hold)
  d.tick(400, v_ego=18.0, **hold)
  assert d.fired(), "sanity: a fresh measurement must still allow the set"
  # now force the stale case directly: a measurement whose window began before the release
  b = d.b
  b._released_t = 100.0
  b._decel_from = 99.0
  b._decel = -1.0                                  # "accelerating", from the on-gas window
  b._used_gas = True
  b._armed_set = None
  inp = mk(101.0, lateral_only=True, op_enabled=False, cruise_enabled=False,
           set_speed_ms=0.0, v_ego=18.0)
  assert b._gates(inp) == "decelUnknown", "a stale decel measurement must fail CLOSED"


def test_gasset_still_refuses_a_close_lead():
  """Gemini finding C. "A SET commands no acceleration" is true, but it is NOT the same as "the road
  ahead is irrelevant": stock ACC takes a moment to react on engagement, and the driver has just
  lifted off expecting regen. The 20 m floor and the TTC check still bind for SET mode."""
  hold = dict(lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  d = Drive()
  d.tick(50)
  d.tick(20, brake_pressed=True, v_ego=18.0, **hold)
  d.tick(60, gas_pressed=True, v_ego=18.0, **hold)
  d.tick(400, v_ego=18.0, has_lead=True, d_rel=12.0, v_lead=10.0, **hold)   # 12 m -- inside the floor
  assert not d.fired(), f"a close lead must still refuse a SET; records={d.records[-3:]}"
  assert "leadClose" in d.reasons("refuse"), d.records[-3:]


def test_gasset_skips_only_the_headway_subgate():
  """The one sub-gate a SET legitimately skips: 2 s headway asks "would ACC have to close a gap",
  which only a RESUME can do. A lead at 25 m with matched speed is inside 2 s headway at 18 m/s but
  is neither close nor closing -- a RESUME refuses it, a SET must not."""
  hold = dict(lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  lead = dict(has_lead=True, d_rel=25.0, v_lead=18.0)      # 1.4 s headway, zero closing speed
  d = Drive()
  d.tick(50)
  d.tick(20, brake_pressed=True, v_ego=18.0, **hold)
  d.tick(60, gas_pressed=True, v_ego=18.0, **hold)
  d.tick(400, v_ego=18.0, **lead, **hold)
  assert d.fired(), f"SET must not refuse on headway alone; records={d.records[-3:]}"
  # and the same geometry DOES refuse a resume, which is what makes the distinction real
  assert lead_gate(True, 25.0, 18.0, 18.0) == "leadGap"
  assert lead_gate(True, 25.0, 18.0, 18.0, require_headway=False) is None


def test_the_offrequest_latch_is_cleared_by_an_engage_press():
  """Gemini finding B. The driver may press OFF and change their mind a second later. If the latch
  still stood, openpilot would refuse and controlsd's cancel rule would kill the engagement they
  just asked for -- the same shape as the regression this feature already caused once.

  Pinned by reading selfdrived.py's source, since it cannot be imported without cereal here."""
  import pathlib
  import re
  src = (pathlib.Path(M.__file__).parent.parent.parent / "selfdrived" / "selfdrived.py").read_text()
  # NOT the first occurrence -- __init__ also assigns 0.0. Match the guarded clear specifically:
  # a buttonEvents test followed by the assignment.
  m = re.search(r"be\.type in \((.*?)\)\s*\n?.*?for be in CS\.buttonEvents\):\s*\n\s*self\.off_request_t = 0\.0",
                src, re.S)
  assert m, "no engage-button press clears the off-request latch"
  btns = m.group(1)
  for btn in ("accelCruise", "decelCruise", "resumeCruise", "setCruise"):
    assert btn in btns, f"an {btn} press must clear the off-request latch, got: {btns}"


def test_gasset_still_respects_the_speed_floor_and_engageability():
  """The two gates that DO bound a set-to-current: it is still a self-engagement."""
  hold = dict(lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  d = Drive()
  d.tick(50)
  d.tick(20, brake_pressed=True, v_ego=4.0, **hold)
  d.tick(60, gas_pressed=True, v_ego=4.0, **hold)
  d.tick(400, v_ego=4.0, **hold)                               # below V_EGO_MIN_MS
  assert not d.fired(), "must not self-engage below the speed floor"
  d2 = Drive()
  d2.tick(50)
  d2.tick(20, brake_pressed=True, v_ego=18.0, **hold)
  d2.tick(60, gas_pressed=True, v_ego=18.0, **hold)
  d2.tick(400, v_ego=18.0, engageable=False, **hold)           # a standing NO_ENTRY
  assert not d2.fired(), "must not self-engage while openpilot itself would refuse to engage"


def test_the_published_wire_contract_matches_what_the_executor_parses():
  """The brain and the executor live in DIFFERENT REPOS (pnw-pilot and pnw-opendbc) and the only
  thing between them is a JSON mem-param. Nothing else in either test suite covers that seam, and a
  key rename on one side would fail SILENTLY -- the executor would parse None and simply never
  press, which looks exactly like "the gates refused". Pin the exact key set here; the matching
  assertion on the other side is opendbc test_parses_a_well_formed_offer.

  Read out of selfdrived's source by AST rather than executed, because importing selfdrived needs
  cereal/capnp, which is not built on the dev host."""
  import ast
  import pathlib
  src = (pathlib.Path(M.__file__).parent.parent.parent / "selfdrived" / "selfdrived.py").read_text()
  tree = ast.parse(src)
  fn = next(n for n in ast.walk(tree)
            if isinstance(n, ast.FunctionDef) and n.name == "_mads_resume_step")
  pubs = [n for n in ast.walk(fn)
          if isinstance(n, ast.Call) and getattr(n.func, "attr", "") == "put_nonblocking"
          and n.args and isinstance(n.args[0], ast.Constant) and n.args[0].value == "MadsResumeTarget"]
  assert len(pubs) == 2, f"expected one offer publish and one withdrawal, got {len(pubs)}"
  offer = next(c for c in pubs if isinstance(c.args[1], ast.Dict) and c.args[1].keys)
  keys = {k.value for k in offer.args[1].keys}
  assert keys == {"dir", "ts", "eid", "set"}, f"wire contract drifted: {keys}"
  direction = next(v for k, v in zip(offer.args[1].keys, offer.args[1].values, strict=True) if k.value == "dir")
  # gasset2pnw: `dir` is no longer a literal -- it carries the brain's chosen mode ("res" to tap
  # RESUME, "set" to tap SET at the driver's gas-chosen speed). Pin that it is wired to out.mode and
  # nothing else, since a stray literal here would silently press the WRONG BUTTON: a RESUME would
  # hand back an old remembered speed when the driver expected the one they just chose.
  assert isinstance(direction, ast.Attribute) and direction.attr == "mode", (
    f"dir must be published from the brain's mode, got {ast.dump(direction)}")
  assert {m for m in ("res", "set")} == set(M.RESUME_MODES), (
    "the brain must declare exactly the two modes the executor's parser accepts")
  withdraw = next(c for c in pubs if isinstance(c.args[1], ast.Dict) and not c.args[1].keys)
  assert withdraw is not None, "there must be an explicit empty-dict withdrawal"


def test_a_standing_no_entry_blocks_the_resume():
  """Fable A1 (the highest-value finding): a NO_ENTRY carries no DISABLE type, so it does not show
  up in `blocked` and MADS keeps holding lateral. But if our RES engages stock cruise while one
  stands, openpilot refuses to engage, controlsd sends CANCEL, and mads_pnw revokes lateral on the
  cruise-engage edge -- our press would take the driver's STEERING away."""
  d = normal_brake_and_resume(engageable=False)
  assert not d.fired()
  assert "noEntry" in d.reasons("refuse"), d.records


def test_no_entry_appearing_during_the_offer_withdraws_it():
  d = normal_brake_and_resume(post_ticks=60)
  n = len(d.offers)
  assert n > 0
  d.tick(50, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0,
         engageable=False)
  assert len(d.offers) == n
  assert "noEntry" in [r.get("reason") for r in d.records if r["phase"] == "offerEnd"]


def test_engageable_is_recorded_so_a_no_entry_refusal_is_diagnosable():
  d = normal_brake_and_resume(engageable=False)
  assert all("engbl" in r for r in d.records)
  assert d.records[0]["engbl"] is False


def test_arm_without_the_brake_down_refuses():
  """Fable A2: unreachable through today's mads_pnw (it only raises lateral_only on a braking
  frame), but a future arming path must not be able to hand this feature a non-brake episode."""
  d = Drive()
  d.tick(50)
  d.tick(400, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  assert not d.fired()
  assert d.reasons("refuse") == ["noBrake"]


def test_verify_flags_a_resume_that_came_back_too_LOW_as_well():
  """Fable B1: the axiom is 'the speed the driver ALREADY set'. A come-back well below the capture
  means the PCM did not restore its remembered set -- not dangerous, but not what this design
  assumes either, and reporting it as 'ok' would hide it."""
  d = normal_brake_and_resume()
  assert d.fired()
  d.tick(10, lateral_only=False, op_enabled=False, cruise_enabled=True, set_speed_ms=SET - 6.0)
  verify = [r for r in d.records if r["phase"] == "verify"]
  assert len(verify) == 1 and verify[0]["reason"] == "setLower" and verify[0]["loud"] is True


# ---------------------------------------------------------------------------------------------
# Telemetry -- a silent no-resume and a silent wrong-resume must be distinguishable
# ---------------------------------------------------------------------------------------------

def test_every_arm_produces_exactly_one_terminal_record():
  # NOTE: {"gas_pressed": True} is deliberately absent. Since gasset2pnw a held accelerator keeps
  # the episode alive on purpose (the driver is still choosing the speed), so it has no terminal
  # record until they lift off -- covered by test_the_arm_survives_a_long_acceleration.
  for kw in ({}, {"has_lead": True, "d_rel": 15.0, "v_lead": SET},
             {"has_lead": None}, {"set_speed_ms": SET + 3.0}):
    d = normal_brake_and_resume(**kw)
    terminal = [r for r in d.records if r["phase"] in ("fire", "refuse")]
    assert len(terminal) == 1, f"{kw} -> {[r['phase'] for r in d.records]}"


def test_a_refusal_always_names_a_reason():
  d = normal_brake_and_resume(has_lead=True, d_rel=15.0, v_lead=SET)
  refusals = [r for r in d.records if r["phase"] == "refuse"]
  assert refusals and all(r["reason"] for r in refusals)
  assert all(r["fired"] is False for r in refusals)


def test_records_are_json_serializable():
  import json
  d = normal_brake_and_resume()
  d.tick(10, lateral_only=False, op_enabled=False, cruise_enabled=True, set_speed_ms=SET)
  for r in d.records:
    json.loads(json.dumps(r))     # would raise on a NaN/inf or a non-primitive


def test_records_survive_nonfinite_inputs():
  d = Drive()
  d.tick(50)
  d.tick(20, lateral_only=True, op_enabled=False, cruise_enabled=False, brake_pressed=True, set_speed_ms=0.0)
  d.tick(400, lateral_only=True, op_enabled=False, cruise_enabled=False,
         set_speed_ms=float("nan"), v_ego=float("nan"), has_lead=True,
         d_rel=float("inf"), v_lead=float("nan"))
  import json
  for r in d.records:
    json.loads(json.dumps(r))
  assert not d.fired()


class TestOneToggleGoverns:
  """onetoggle2pnw: "Disengage on brake" alone governs BOTH halves.

  Removing the separate MadsAutoResume toggle is not a loosening. mads_pnw sets
      lateral_only = (not disengage_on_brake) and braking and not blocked
  so lateral_only can ONLY be true when DisengageOnBrake is OFF -- and the resume brain can only
  ARM on the rising edge of lateral_only. The toggle was already implied by that gate, and a second
  control that can never independently be false is one the driver can be misled by.
  """

  @staticmethod
  def _ev(*names):
    from openpilot.selfdrive.selfdrived.events import Events
    e = Events()
    for n in names:
      e.add(n)
    return e

  def test_resume_impossible_when_disengage_on_brake_is_on(self):
    """With DisengageOnBrake ON (stock), mads_pnw never yields lateral_only -- so the resume brain
    can never arm, with no separate toggle needed to hold it off."""
    from openpilot.selfdrive.selfdrived.mads_pnw import MadsPnw
    from openpilot.selfdrive.selfdrived.events import EventName, Events
    from opendbc.safety import ALTERNATIVE_EXPERIENCE
    m = MadsPnw(ALTERNATIVE_EXPERIENCE.ENABLE_MADS |
                ALTERNATIVE_EXPERIENCE.MADS_DISENGAGE_LATERAL_ON_BRAKE)
    m.update(True, True, False, True, Events())
    for _ in range(50):
      m.update(False, False, True, False, self._ev(EventName.pedalPressed))
      assert not m.lateral_only, "DisengageOnBrake ON must never yield lateral_only"

  def test_lateral_only_requires_disengage_on_brake_off(self):
    """The converse -- with it OFF, lateral_only IS produced, which is the one edge the resume
    brain arms on. Together these two show the removed toggle was redundant, not load-bearing."""
    from openpilot.selfdrive.selfdrived.mads_pnw import MadsPnw
    from openpilot.selfdrive.selfdrived.events import EventName, Events
    from opendbc.safety import ALTERNATIVE_EXPERIENCE
    m = MadsPnw(ALTERNATIVE_EXPERIENCE.ENABLE_MADS)
    m.update(True, True, False, True, Events())
    m.update(False, False, True, False, self._ev(EventName.pedalPressed))
    assert m.lateral_only, "DisengageOnBrake OFF must yield lateral_only"

  def test_no_stale_resume_param_remains(self):
    """A key with no reader is still settable from a shell -- remove it, don't orphan it."""
    import pathlib
    root = pathlib.Path(__file__).resolve().parents[4]
    keys = (root / "common/params_keys.h").read_text()
    assert '"MadsAutoResume"' not in keys, "stale key would still be settable"
    assert '"DisengageOnBrake"' in keys, "the one remaining control must stay"

    # Gemini 2026-09-06: checking params_keys.h alone is not enough -- ANY surviving reader
    # anywhere would raise UnknownKeyName and crash-loop that process onroad. Scan the tree.
    offenders = []
    for pat in ("**/*.py", "**/*.cc", "**/*.h", "**/*.sh"):
      for f in root.glob(pat):
        if any(x in f.parts for x in (".git", ".venv", "site-packages", "third_party", "tests", "docs")):
          continue
        try:
          text = f.read_text(errors="ignore")
        except OSError:
          continue
        for i, line in enumerate(text.splitlines(), 1):
          if "MadsAutoResume" not in line:
            continue
          st = line.strip()
          if st.startswith(("#", "//")):
            continue          # a comment recording why it went is fine
          offenders.append(f"{f.relative_to(root)}:{i}: {st}")
    assert not offenders, "live MadsAutoResume reader(s) survived:\n" + "\n".join(offenders)


# ---------------------------------------------------------------------------------------------
# engagegoal2pnw -- owner goal 2026-09-13: "...I should be able to hit the gas pedal once and then it
# should overwrite this and set the new speed." Analysis: docs/MADS-RESUME-TO-DRIVER-SPEED.md section 10
# (workbench root). Real-data replays of the same brain: selfdrive/selfdrived/tests/test_engagegoal_pnw.py.
# ---------------------------------------------------------------------------------------------

STEER_ONLY = dict(lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
# Measured 2026-09-11 22:07:11-14 (F2, cruise never came back): with no pedal the truck slowed at
# 1.76-1.83 m/s^2 for 3 s -- regen. (Since 2026-09-13 regen no longer refuses a gas-set.)
REGEN_MS2 = 1.8


def _red_light(d, stop_s):
  """Cruising, brake to a stop, sit on the brake for `stop_s`, release the brake."""
  d.tick(50)                                                         # cruise engaged, set captured
  d.tick(int(stop_s / DT), brake_pressed=True, v_ego=0.0, standstill=True, **STEER_ONLY)
  d.tick(30, v_ego=0.0, standstill=True, **STEER_ONLY)               # brake up, auto-hold


def _pull_away_and_lift(d, gas_s, v=16.0, post_ticks=400):
  d.tick(max(1, int(gas_s / DT)), gas_pressed=True, v_ego=v, **STEER_ONLY)
  d.tick(post_ticks, v_ego=v, **STEER_ONLY)                           # coasting, decel ~0


def test_the_accelerator_sets_the_speed_after_a_red_light_longer_than_the_arm_lifetime():
  """THE GAP. An episode used to open only on a brake press and dies ARM_MAX_S after the last pedal
  activity, so a light held on the brake for longer than that left the accelerator doing nothing."""
  d = Drive()
  _red_light(d, M.ARM_MAX_S + 10.0)
  assert "armExpired" in d.reasons("refuse"), "precondition: the brake episode expired at the light"
  _pull_away_and_lift(d, gas_s=8.0, v=16.0)
  assert d.fired(), f"lifting off after a long light must set that speed; records={d.records[-4:]}"
  fires = [r for r in d.records if r["phase"] == "fire"]
  assert len(fires) == 1 and fires[0]["mode"] == "set"
  assert d.offers[-1][2] == pytest.approx(16.0)
  assert [r.get("reason") for r in d.records if r["phase"] == "arm"][-1] == "gas"


@pytest.mark.parametrize("stop_s", [5.0, M.ARM_MAX_S + 10.0])
@pytest.mark.parametrize("gas_s", [DT, 0.2])
def test_one_short_accelerator_tap_sets_the_speed(stop_s, gas_s):
  """"Hit the gas pedal once": there is no minimum hold and no speed-delta gate. A 10 ms or 200 ms tap
  sets the speed at lift-off -- with an episode still open from the brake, and without one."""
  d = Drive()
  _red_light(d, stop_s)
  _pull_away_and_lift(d, gas_s=gas_s, v=12.0)
  assert d.fired(), f"a {gas_s}s tap after a {stop_s}s stop must set; records={d.records[-4:]}"
  assert [r for r in d.records if r["phase"] == "fire"][-1]["mode"] == "set"


def test_a_gas_opened_episode_can_never_offer_RESUME():
  """The accelerator's own episode is SET mode from its first tick. A one-tick tap would otherwise fall
  back to RESUME on the next tick -- and here RESUME's gates would all pass (captured 29 m/s, truck at
  25 m/s, rolling max expired to 25), so the wrong button would be pressed."""
  d = Drive()
  d.tick(50)                                                         # capture SET = 29 m/s
  d.tick(20, brake_pressed=True, v_ego=25.0, engageable=False, **STEER_ONLY)
  # a standing NO_ENTRY refuses the brake episode's RESUME; it clears long after that episode expired
  d.tick(int((M.ARM_MAX_S + 5.0) / DT), v_ego=25.0, engageable=False, **STEER_ONLY)
  d.tick(int(M.V_MAX_WINDOW_S / DT), v_ego=25.0, **STEER_ONLY)
  assert "noEntry" in d.reasons("refuse") and not d.fired() and not d.b._armed, "precondition"
  d.tick(1, gas_pressed=True, v_ego=25.0, **STEER_ONLY)
  d.tick(400, v_ego=25.0, **STEER_ONLY)
  fires = [r for r in d.records if r["phase"] == "fire"]
  assert fires, f"precondition: the tap must produce a press; records={d.records[-4:]}"
  assert all(r["mode"] == "set" for r in fires), f"a gas-opened episode offered {[r['mode'] for r in fires]}"
  assert d.offers[-1][2] == pytest.approx(25.0), "SET targets the current speed, never the remembered one"


def test_the_accelerator_cannot_undo_the_double_tap_opt_out():
  """Model item 4: double-tap the brake = stay off until the driver re-engages. The accelerator is not a
  re-engagement."""
  d = Drive()
  d.tick(50)
  d.tick(20, brake_pressed=True, v_ego=15.0, **STEER_ONLY)
  d.tick(30, v_ego=15.0, **STEER_ONLY)
  d.tick(20, brake_pressed=True, v_ego=15.0, **STEER_ONLY)          # second press inside DOUBLE_BRAKE_S
  assert "doubleBrake" in d.reasons("suppress"), "precondition: opted out"
  d.tick(300, v_ego=15.0, **STEER_ONLY)
  n = len(d.records)
  _pull_away_and_lift(d, gas_s=3.0, v=18.0)
  assert not d.fired(), f"the accelerator overrode the opt-out; records={d.records[-4:]}"
  assert not [r for r in d.records if r["phase"] == "arm" and r.get("reason") == "gas"]
  # Rule 2 (Fable F1): the refused accelerator press is on record, exactly once, like a refused brake press
  new = d.records[n:]
  assert [(r["phase"], r["reason"], r["gas"], r["mode"]) for r in new] == [("refuse", "suppressed", True, "set")], new


def test_the_accelerator_cannot_undo_the_post_resume_rejection_and_the_refusal_is_on_record():
  """The post-resume variant (Fable re-review): RES fires, cruise comes back, the driver brakes 0.5 s later to
  reject it. That brake suppresses on the SAME frame lateral-only rises, so the stretch never records that it
  began with a brake arm -- and the accelerator's refusal used to write nothing at all."""
  d = normal_brake_and_resume(post_ticks=60)
  assert d.fired(), "precondition: RES fired"
  d.tick(40, lateral_only=False, op_enabled=True, cruise_enabled=True, set_speed_ms=SET)   # cruise is back
  d.tick(20, brake_pressed=True, v_ego=20.0, **STEER_ONLY)          # rejection brake inside REJECT_AFTER_FIRE_S
  assert "postResumeBrake" in d.reasons("suppress"), f"precondition: opted out; {d.phases()}"
  d.tick(50, v_ego=20.0, **STEER_ONLY)
  n_rec, n_off = len(d.records), len(d.offers)
  _pull_away_and_lift(d, gas_s=2.0, v=20.0)
  assert len(d.offers) == n_off, "the accelerator overrode the post-resume opt-out"
  new = d.records[n_rec:]
  assert [(r["phase"], r["reason"], r["gas"], r["mode"]) for r in new] == [("refuse", "suppressed", True, "set")], new


def test_brake_and_accelerator_on_the_same_tick_while_suppressed_write_one_refusal():
  d = Drive()
  d.tick(50)
  d.tick(20, brake_pressed=True, v_ego=15.0, **STEER_ONLY)
  d.tick(30, v_ego=15.0, **STEER_ONLY)
  d.tick(20, brake_pressed=True, v_ego=15.0, **STEER_ONLY)          # double-tap -> suppressed
  d.tick(int((M.DOUBLE_BRAKE_S + 0.5) / DT), v_ego=15.0, **STEER_ONLY)   # past the double-tap window
  n = len(d.records)
  d.tick(1, brake_pressed=True, gas_pressed=True, v_ego=15.0, **STEER_ONLY)
  new = d.records[n:]
  assert [(r["phase"], r["reason"]) for r in new] == [("refuse", "suppressed")], new


def test_a_mads_unavailable_tick_forgets_that_steering_only_began_with_a_brake():
  """Belt-and-braces for the inert branch (Fable F2): if MADS goes unavailable, the brain must not carry
  `_lat_braked` into whatever steering-only state it sees next without a brake."""
  d = Drive()
  d.tick(50)
  d.tick(20, brake_pressed=True, v_ego=15.0, **STEER_ONLY)         # a real brake arm -> _lat_braked
  d.tick(int((M.ARM_MAX_S + 2.0) / DT), v_ego=15.0, engageable=False, **STEER_ONLY)   # RES refused, episode expires
  assert "noEntry" in d.reasons("refuse") and not d.fired() and not d.b._armed and d.b._lat_braked, "precondition"
  d.tick(1, mads_available=False, v_ego=15.0, **STEER_ONLY)
  n = len(d.records)
  _pull_away_and_lift(d, gas_s=2.0, v=18.0)
  assert not d.fired() and d.records[n:] == [], f"records={d.records[n:]}"


def test_the_accelerator_never_opens_an_episode_in_a_steering_only_state_not_seen_to_start_with_a_brake():
  """The envelope rests on the steering-only state being brake-induced (the noBrake check). If MADS becomes
  available while the truck is ALREADY steering-only, this brain never saw that start, so the accelerator
  must not open an episode in it."""
  d = Drive()
  d.tick(50)
  d.tick(200, mads_available=False, v_ego=15.0, **STEER_ONLY)
  d.tick(100, v_ego=15.0, **STEER_ONLY)                              # MADS on, lateral-only already standing
  _pull_away_and_lift(d, gas_s=2.0, v=18.0)
  assert not d.fired() and d.records == [], f"records={d.records}"


def test_a_gas_override_while_cruising_engaged_never_arms_or_offers():
  """Normal cruising: ACC engaged, press the accelerator to overtake, lift. No brake, no steering-only
  state: nothing may happen (the 09-13 request: "I normally do NOT want the cruise control to accept my
  speed when I accelerate")."""
  d = Drive()
  d.tick(100)
  d.tick(300, gas_pressed=True, v_ego=SET + 3.0)
  d.tick(400, v_ego=SET + 2.0)
  assert not d.fired() and d.records == [], f"records={d.records}"
  # ...and the same after an earlier MADS brake whose RESUME brought cruise back: leaving steering-only must
  # forget that it began with a brake, or the next overtake would open an episode while engaged.
  d2 = normal_brake_and_resume()
  d2.tick(100, lateral_only=False, op_enabled=True, cruise_enabled=True, set_speed_ms=SET)
  n_rec, n_off = len(d2.records), len(d2.offers)
  d2.tick(300, gas_pressed=True, v_ego=SET + 3.0)
  d2.tick(400, v_ego=SET + 2.0)
  assert len(d2.offers) == n_off and d2.records[n_rec:] == [], f"records={d2.records[n_rec:]}"


def test_a_refused_first_press_is_spent_and_a_re_press_cannot_set():
  """OWNER DECISION 2026-09-13, first press only (and Q-C4 answered: a first press is used once judged).
  The driver lifts with a car close ahead, the window refuses `leadClose`, and the driver goes back on the
  power. That re-press is a later press: it ends the episode with ONE record that still names the gate that
  refused the first press (D1), and lifting off again -- road now clear -- sets nothing."""
  d = Drive()
  d.tick(50)
  d.tick(20, brake_pressed=True, v_ego=20.0, **STEER_ONLY)
  d.tick(100, gas_pressed=True, v_ego=20.0, **STEER_ONLY)
  d.tick(150, v_ego=20.0, has_lead=True, d_rel=12.0, v_lead=18.0, **STEER_ONLY)   # 1.5 s lift, lead close
  assert not d.fired() and d.b._gas_spent, "precondition: refused on the lead, first press used"
  n = len(d.records)
  d.tick(100, gas_pressed=True, v_ego=20.0, **STEER_ONLY)           # back on the power
  d.tick(600, v_ego=20.0, **STEER_ONLY)                              # lift and coast, clear road
  assert not d.fired(), f"a second press set the speed; records={d.records[n:]}"
  new = d.records[n:]
  assert [(r["phase"], r["reason"], r["gate"], r["gasSpent"]) for r in new] == [("refuse", "gasSpent", "leadClose", True)], new
  assert new[0]["armS"] is not None, "this record is the arm's own terminal and must say so"
  assert d.phases().count("refuse") == 1 and d.phases().count("fire") == 0, d.phases()


def _set_ignored_by_the_pcm(d, v):
  """The SET fired but stock cruise never came back, so the truck is still steering-only."""
  d.tick(int((M.VERIFY_S + M.ARM_MAX_S + 1.0) / DT), v_ego=v, **STEER_ONLY)


def test_first_press_only_red_light_then_overtake_then_brake_again():
  """The owner's sequence. Brake held 45 s at a light -> release -> the first press sets at lift-off. Three
  minutes later, same steering-only stretch (the SET did not take), an overtake -> no SET. Brake again ->
  the first press after THAT brake sets again."""
  d = Drive()
  _red_light(d, 45.0)
  _pull_away_and_lift(d, gas_s=8.0, v=16.0)
  assert len({o[1] for o in d.offers}) == 1 and d.offers[-1][2] == pytest.approx(16.0), f"first press must set; {d.records[-4:]}"
  first_eid = d.offers[-1][1]
  _set_ignored_by_the_pcm(d, 16.0)
  d.tick(int(180.0 / DT), v_ego=16.0, **STEER_ONLY)                   # three minutes later
  n_rec, n_off = len(d.records), len(d.offers)
  _pull_away_and_lift(d, gas_s=5.0, v=25.0)                          # overtake, lift
  assert len(d.offers) == n_off, f"the overtake engaged cruise; records={d.records[n_rec:]}"
  new = d.records[n_rec:]
  assert [(r["phase"], r["reason"], r["gasSpent"]) for r in new] == [("refuse", "gasSpent", True)], new
  d.tick(20, brake_pressed=True, v_ego=22.0, **STEER_ONLY)           # brake again
  arm = d.records[-1]
  assert arm["phase"] == "arm" and arm["gasSpent"] is False, arm
  d.tick(20, v_ego=22.0, **STEER_ONLY)                               # release (< RELEASE_MIN_S: no RES)
  _pull_away_and_lift(d, gas_s=3.0, v=22.0)
  assert len(d.offers) > n_off and d.offers[-1][1] != first_eid, f"the first press after the new brake must set; {d.records[-4:]}"
  assert d.offers[-1][2] == pytest.approx(22.0)
  assert [r for r in d.records if r["phase"] == "fire"][-1]["mode"] == "set"


def test_short_lifts_inside_the_settle_time_do_not_use_up_the_first_press():
  """Pedal modulation while pulling away (lifts shorter than RELEASE_MIN_S, never judged) is still the first
  press; the real lift-off afterwards sets."""
  d = Drive()
  _red_light(d, 5.0)
  for _ in range(5):
    d.tick(100, gas_pressed=True, v_ego=14.0, **STEER_ONLY)
    d.tick(int(0.3 / DT), v_ego=14.0, **STEER_ONLY)
  assert not d.b._gas_spent, "precondition: short lifts must not use up the press"
  _pull_away_and_lift(d, gas_s=1.0, v=14.0)
  assert d.fired() and d.offers[-1][2] == pytest.approx(14.0), d.records[-4:]


def test_a_mads_unavailable_tick_forgets_that_the_first_press_was_used():
  d = Drive()
  _red_light(d, 5.0)
  _pull_away_and_lift(d, gas_s=2.0, v=16.0)
  assert d.fired() and d.b._gas_spent, "precondition"
  _set_ignored_by_the_pcm(d, 16.0)
  d.tick(1, mads_available=False, v_ego=16.0, **STEER_ONLY)
  n = len(d.records)
  _pull_away_and_lift(d, gas_s=2.0, v=18.0)
  assert d.records[n:] == [], f"stale first-press state survived an inert tick: {d.records[n:]}"


def test_pedal_modulation_inside_the_settle_time_writes_no_lift_record():
  """A lift shorter than RELEASE_MIN_S was never judged by a gate; recording it would only be noise."""
  d = Drive()
  d.tick(50)
  d.tick(20, brake_pressed=True, v_ego=20.0, **STEER_ONLY)
  for _ in range(10):
    d.tick(50, gas_pressed=True, v_ego=20.0, **STEER_ONLY)
    d.tick(int(0.3 / DT), v_ego=20.0, **STEER_ONLY)
  assert not [r for r in d.records if r["phase"] == "lift"], [r for r in d.records if r["phase"] == "lift"]


# ---------------------------------------------------------------------------------------------
# engagegoal2pnw -- OWNER DECISION 2026-09-13 "Cancel, steering drops too". Sun 21:16:33 PT (Corvallis): our
# gas-set SET- from Standby made the PCM engage at 55 mph with the truck at 34. The whole-system replay of that
# sequence (events, state machine, controlsd cancel rule, alert, chime) is in
# selfdrive/selfdrived/tests/test_engagegoal_pnw.py; these pin the brain's decision.
# ---------------------------------------------------------------------------------------------

def _gas_set_then_cruise_returns(got_ms, driver_btn_at=None, v=15.36):
  """Steering-only after a brake, accelerate to `v`, lift, the SET fires; 0.15 s later stock cruise engages
  with set `got_ms`. `driver_btn_at`: seconds after the fire at which the DRIVER pressed a cruise button."""
  d = Drive()
  _red_light(d, 5.0)
  d.tick(300, gas_pressed=True, v_ego=v, **STEER_ONLY)
  while not d.fired():
    d.tick(1, v_ego=v, **STEER_ONLY)
  t_fire = d.offers[0][0]
  cancels, n_rec = [], len(d.records)
  engaged = dict(lateral_only=False, op_enabled=True, cruise_enabled=True, set_speed_ms=got_ms, v_ego=v)
  for _ in range(100):
    now_s = round(d.t - t_fire, 3)
    btn = driver_btn_at is not None and abs(now_s - driver_btn_at) < DT / 2
    kw = engaged if now_s >= 0.15 else dict(STEER_ONLY, v_ego=v)
    out = d.b.update(mk(d.t, driver_cruise_button=btn, **kw))
    if out.cancel:
      cancels.append(now_s)
    d.records.extend(out.records)
    d.t += DT
  return d, cancels, [r for r in d.records[n_rec:] if r["phase"] == "verify"]


def test_the_2116_overshoot_cancels_at_once_on_the_verify_tick():
  """42 mph memory, truck at 34.4 mph (15.36 m/s), the PCM engaged at 55 mph (24.59 m/s): cancel once, on the
  engage tick, with a loud verify that says why."""
  d, cancels, verify = _gas_set_then_cruise_returns(24.59)
  assert cancels == [0.15], f"cancel ticks (s after fire): {cancels}"
  assert len(verify) == 1 and verify[0]["reason"] == "setHigher" and verify[0]["loud"] is True, verify
  assert verify[0]["cancel"] is True and verify[0]["driverBtn"] is False, verify


def test_a_driver_button_in_the_verify_window_is_never_cancelled():
  """The driver is allowed to go faster: a RES/SET+ of theirs between our press and the come-back means the higher
  set may be theirs. Logged loud with driverBtn, never cancelled."""
  for at in (0.05, 0.15):                                           # before, and on, the come-back tick
    d, cancels, verify = _gas_set_then_cruise_returns(24.59, driver_btn_at=at)
    assert cancels == [], f"driver button at +{at}s was overruled"
    assert verify[0]["reason"] == "setHigher" and verify[0]["loud"] and verify[0]["driverBtn"] is True, verify
    assert verify[0]["cancel"] is False


def test_a_gas_set_that_comes_back_at_the_lift_off_speed_is_not_cancelled():
  d, cancels, verify = _gas_set_then_cruise_returns(15.2)            # the PCM rounds 34.4 mph to 34
  assert cancels == [] and verify[0]["reason"] == "ok" and verify[0]["cancel"] is False, verify


@pytest.mark.parametrize("over_mph", [1.5, 2.9])
def test_a_come_back_within_3_mph_is_logged_but_not_cancelled(over_mph):
  d, cancels, verify = _gas_set_then_cruise_returns(15.36 + over_mph * 0.44704)
  assert cancels == [], f"+{over_mph} mph must not cancel"
  assert verify[0]["cancel"] is False
  assert verify[0]["reason"] == ("setHigher" if over_mph * 0.44704 > M.SET_MODE_TOL_MS else "ok"), verify


def test_the_cancel_rule_applies_to_RESUME_too():
  """For RES the wanted speed is the captured set. A RES that comes back more than 3 mph above it cancels."""
  d = normal_brake_and_resume(post_ticks=60)
  assert d.fired() and d.offers[-1][2] == pytest.approx(SET)
  outs = [d.b.update(mk(d.t + k * DT, lateral_only=False, op_enabled=True, cruise_enabled=True,
                        set_speed_ms=SET + 1.5)) for k in range(5)]
  assert [o.cancel for o in outs] == [True, False, False, False, False], [o.cancel for o in outs]


def test_no_cancel_without_our_own_press():
  """Stock cruise engaging on its own, or the driver's own button, with no fire of ours: nothing to verify, no cancel."""
  d = Drive()
  d.tick(50)
  d.tick(20, brake_pressed=True, **STEER_ONLY)
  d.tick(30, gas_pressed=True, **STEER_ONLY)                         # back on the power before any window
  outs = [d.b.update(mk(d.t + k * DT, lateral_only=False, op_enabled=True, cruise_enabled=True,
                        set_speed_ms=SET + 10.0)) for k in range(200)]
  assert not any(o.cancel for o in outs)


def test_a_driver_button_from_an_EARLIER_press_window_does_not_protect_a_later_overshoot():
  """The driver-button flag belongs to ONE press's window. A button the driver pressed while an earlier press was
  being verified (that press never brought cruise back) must not stop the cancel for the next press."""
  d = normal_brake_and_resume(post_ticks=60)                          # RES fires
  assert d.fired()
  hold = dict(lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  d.tick(1, driver_cruise_button=True, **hold)                         # a driver press inside the RES window
  d.tick(int((M.VERIFY_S + 0.5) / DT), **hold)                         # ...but cruise never came back
  assert "noCruise" in [r["reason"] for r in d.records if r["phase"] == "verify"], "precondition"
  d.tick(20, brake_pressed=True, v_ego=15.0, **hold)                   # a fresh brake, then a gas-set
  d.tick(100, gas_pressed=True, v_ego=15.0, **hold)
  while len({o[1] for o in d.offers}) < 2:
    d.tick(1, v_ego=15.0, **hold)
  outs = [d.b.update(mk(d.t + k * DT, lateral_only=False, op_enabled=True, cruise_enabled=True,
                        set_speed_ms=24.59, v_ego=15.0)) for k in range(3)]
  assert [o.cancel for o in outs] == [True, False, False], "a stale driver-button flag suppressed the cancel"
