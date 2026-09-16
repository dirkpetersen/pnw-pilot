"""gassetwait2pnw: the gas-set wait depends on what the driver was doing when they lifted off.

OWNER REPORT 2026-09-15 08:45 PT, and the rule they gave that evening: while cruise is on and they have been
accelerating on the pedal, LIFTING OFF MEANS "lock in this speed", not "slow down". Everywhere else a lift
still means they want to slow -- which is what the 1.0 s wait protects, and it is kept there untouched.

Evidence for the split: drives/2026-09-15/gasset-regen-loss/DRIVE_REPORT.md §2, where an independent
re-derivation found every lift-off that engaged cruise WITHOUT looking at acceleration -- 5 of them, all with
a_ego in [+0.53, +1.63] m/s^2, against 13 lift-offs below -0.2 that never engaged.

MIND THE METRIC (Fable review 2026-09-15): the report's a0 is the MEAN over the 0.5 s BEFORE lift-off; the
code latches aEgo ON THE LIFT-OFF FRAME, which reads lower. On the frame metric the 5 firings span
[+0.12, +1.47] and two non-firing lift-offs sit at +0.49/+0.51, so +0.5 is ABOVE the firing floor, not at it.
That is conservative in the only direction that matters -- a real gas-set can only fall back to today's 1.0 s.
See the constant's own comment in madsresume_pnw.py.

Every test here reuses the real brain and the shared scenario helpers from test_madsresume_pnw.py -- the
point is the behaviour of MadsResumeBrain, not of a reimplementation of it.
"""

import math

import pytest

from openpilot.selfdrive.controls.lib import madsresume_pnw as M
from openpilot.selfdrive.controls.lib.madsresume_pnw import gas_set_wait_s
from openpilot.selfdrive.controls.lib.tests.test_madsresume_pnw import (
  DT, RELEASE_T, STEER_ONLY, Drive, _red_light, normal_brake_and_resume,
)

# What the truck's acceleration actually looked like at the two kinds of lift-off, from the report's
# one clean current-config trace (09-14 22:01:41.141, route 00000173 seg 3) and the braking episode
# the 1.0 s wait was earned by (weekend Sat 12:41:50).
ON_THE_POWER_MS2 = 1.19      # measured aEgo on the lift-off frame of the clean gas-set
ALREADY_SLOWING_MS2 = -1.5   # the driver had been slowing for 5 s before lifting off


def _lift_off_at(a_ego, v=16.0, gas_s=2.0, post_ticks=400, **post):
  """Red light -> pull away on the accelerator -> lift off with `a_ego` on the lift-off frame.

  The acceleration is supplied ONLY on the first both-pedals-up tick, then left at its default for the
  rest of the coast, because that is exactly what the truck does: regen bites within half a second and
  drags aEgo negative. A brain that re-read it would call every lift-off "slowing"."""
  d = Drive()
  _red_light(d, 5.0)
  d.tick(int(gas_s / DT), gas_pressed=True, v_ego=v, a_ego=a_ego, **STEER_ONLY)
  lift = d.t
  d.tick(1, v_ego=v, a_ego=a_ego, **STEER_ONLY)                      # THE lift-off frame
  d.tick(post_ticks, v_ego=v, a_ego=-M.REGEN_DECEL_MS2 if hasattr(M, "REGEN_DECEL_MS2") else -1.8,
         **STEER_ONLY, **post)
  return d, lift


def _offer_delay(d, lift):
  assert d.fired(), f"nothing was ever offered; records={d.records[-4:]}"
  return d.offers[0][0] - lift


# --- the pure helper ---------------------------------------------------------------------------


@pytest.mark.parametrize("a_ego,want_s,want_why", [
  (1.19, 0.5, "onPower"),          # the measured clean gas-set
  (0.53, 0.5, "onPower"),          # the floor of the observed firing population
  (1e9, 0.5, "onPower"),           # absurd but finite and positive -- still on the power
  (0.5, 1.0, "slowing"),           # STRICTLY greater: exactly the threshold is not "on the power"
  (0.0, 1.0, "slowing"),
  (-1.5, 1.0, "slowing"),          # the episode the 1.0 s was earned by
  (float("nan"), 1.0, "accelUnknown"),
  (float("inf"), 1.0, "accelUnknown"),
  (float("-inf"), 1.0, "accelUnknown"),
  (None, 1.0, "accelUnknown"),
  ("", 1.0, "accelUnknown"),
])
def test_the_wait_helper_is_a_pure_function_of_the_lift_off_acceleration(a_ego, want_s, want_why):
  assert gas_set_wait_s(a_ego) == (want_s, want_why)


def test_an_unreadable_acceleration_can_never_buy_the_SHORTER_wait():
  """Rule 2, stated as a property rather than a list: of every input that is not a finite number, not one
  returns the permissive branch.

  A NUMERIC STRING is deliberately absent from this list. `_finite` coerces with float(), so "0.9" reads
  as 0.9 -- the same idiom every other gate in this module uses, and unreachable in practice because the
  caller passes float(CS.aEgo). Pinning a string here would be pinning a fiction."""
  for bad in (float("nan"), float("inf"), float("-inf"), None, "", [], {}, object()):
    secs, why = gas_set_wait_s(bad)
    assert secs == M.GAS_SET_RELEASE_MIN_S, f"{bad!r} bought the short wait"
    assert why == "accelUnknown", f"{bad!r} was not reported as unreadable"


# --- the brain ---------------------------------------------------------------------------------


def test_a_lift_off_ON_THE_POWER_sets_at_half_a_second_not_a_whole_one():
  """The owner's case: they accelerated away from a crossing and lifted off to hold that speed."""
  d, lift = _lift_off_at(ON_THE_POWER_MS2)
  delay = _offer_delay(d, lift)
  assert M.GAS_SET_RELEASE_MIN_ACCEL_S - 1e-9 <= delay <= M.GAS_SET_RELEASE_MIN_ACCEL_S + 0.02, delay
  assert delay < M.GAS_SET_RELEASE_MIN_S - 0.4, "this must be the SHORT wait, not the old one"


def test_a_lift_off_while_ALREADY_SLOWING_still_waits_the_full_second():
  d, lift = _lift_off_at(ALREADY_SLOWING_MS2)
  delay = _offer_delay(d, lift)
  assert M.GAS_SET_RELEASE_MIN_S - 1e-9 <= delay <= M.GAS_SET_RELEASE_MIN_S + 0.02, delay


def test_an_UNREADABLE_acceleration_keeps_todays_wait_and_says_so_in_the_record():
  d, lift = _lift_off_at(float("nan"))
  delay = _offer_delay(d, lift)
  assert M.GAS_SET_RELEASE_MIN_S - 1e-9 <= delay <= M.GAS_SET_RELEASE_MIN_S + 0.02, delay
  fire = [r for r in d.records if r["phase"] == "fire"]
  assert fire, d.phases()
  assert fire[0]["waitWhy"] == "accelUnknown", fire[0]
  assert fire[0]["a0"] is None, "an unreadable acceleration must be logged as null, not as 0.0"


def test_the_acceleration_is_latched_at_LIFT_OFF_so_regen_afterwards_cannot_lengthen_the_wait():
  """The whole reason the latch exists. In the real trace aEgo is +1.19 on the lift-off frame and
  -1.44 by +1.0 s; a brain that re-read it would fall back to the long wait every single time."""
  d = Drive()
  _red_light(d, 5.0)
  d.tick(200, gas_pressed=True, v_ego=16.0, a_ego=ON_THE_POWER_MS2, **STEER_ONLY)
  lift = d.t
  d.tick(1, v_ego=16.0, a_ego=ON_THE_POWER_MS2, **STEER_ONLY)        # lift-off frame: on the power
  d.tick(400, v_ego=16.0, a_ego=-2.14, **STEER_ONLY)                 # regen bites hard, as it really does
  delay = _offer_delay(d, lift)
  assert delay <= M.GAS_SET_RELEASE_MIN_ACCEL_S + 0.02, f"the wait was re-derived from live regen: {delay}"


def test_every_fire_record_carries_the_wait_it_used_and_what_bought_it():
  for a_ego, why in ((ON_THE_POWER_MS2, "onPower"), (ALREADY_SLOWING_MS2, "slowing")):
    d, _ = _lift_off_at(a_ego)
    fire = [r for r in d.records if r["phase"] == "fire"]
    assert fire, d.phases()
    assert fire[0]["waitWhy"] == why, fire[0]
    assert fire[0]["a0"] == pytest.approx(round(a_ego, 2)), fire[0]
    assert fire[0]["waitS"] == pytest.approx(gas_set_wait_s(a_ego)[0]), fire[0]


# --- what must NOT change -----------------------------------------------------------------------


def test_the_BRAKING_EPISODE_THE_ONE_SECOND_WAS_EARNED_BY_is_unchanged():
  """Weekend Sat 12:41:50: the driver had been slowing for 5 s, lifted off, and braked 0.69 s later, to a
  stop. The 1.0 s wait exists so a SET never lands on top of that brake. Because they were ALREADY SLOWING,
  this episode keeps the full second -- the protection is untouched exactly where it was earned."""
  d = Drive()
  _red_light(d, 5.0)
  d.tick(200, gas_pressed=True, v_ego=16.0, a_ego=ALREADY_SLOWING_MS2, **STEER_ONLY)
  d.tick(1, v_ego=16.0, a_ego=ALREADY_SLOWING_MS2, **STEER_ONLY)     # lift-off, already slowing
  d.tick(69, v_ego=15.0, **STEER_ONLY)                               # 0.69 s of coast
  assert not d.fired(), "a SET landed inside the second the brake was going to arrive in"
  d.tick(200, brake_pressed=True, v_ego=8.0, **STEER_ONLY)           # the brake
  assert not d.fired(), f"the brake must have re-armed and set nothing; records={d.records[-4:]}"


@pytest.mark.parametrize("a_ego", [ON_THE_POWER_MS2, ALREADY_SLOWING_MS2, float("nan")])
def test_the_RESUME_path_keeps_its_own_half_second_whatever_the_acceleration_was(a_ego):
  """gassetwait2pnw touches the GAS-SET wait only. A resume after a brake is a different action with a
  different clock (RELEASE_MIN_S) and must not move with the accelerator."""
  r = normal_brake_and_resume(a_ego=a_ego)
  assert r.offers, "the reference resume must still fire"
  assert M.RELEASE_MIN_S - 1e-9 <= r.offers[0][0] - RELEASE_T <= M.RELEASE_MIN_S + 0.02, r.offers[0][0] - RELEASE_T


@pytest.mark.parametrize("a_ego", [ON_THE_POWER_MS2, float("nan")])
def test_a_RESUME_record_is_NOT_stamped_with_a_wait_that_did_not_govern_it(a_ego):
  """Fable D2. A resume waits RELEASE_MIN_S, so `waitS`/`waitWhy`/`a0` would be false data on its record --
  and the accelUnknown line selfdrived logs off `waitWhy` would claim "kept the 1.0 s wait" about a resume
  that waited 0.5 s. An absent field cannot lie; a wrong one can."""
  r = normal_brake_and_resume(a_ego=a_ego)
  res = [rec for rec in r.records if rec.get("mode") == "res"]
  assert res, [rec.get("mode") for rec in r.records]
  for rec in res:
    assert "waitS" not in rec and "waitWhy" not in rec and "a0" not in rec, rec

  d, _ = _lift_off_at(ON_THE_POWER_MS2)
  sets = [rec for rec in d.records if rec.get("mode") == "set"]
  assert sets and all("waitWhy" in rec for rec in sets), "a gas-set record must still carry it"


def test_a_lift_shorter_than_the_SHORT_wait_still_does_not_use_up_the_first_press():
  """The first-press rule reads the same wait the gates do. On the power the judgement now happens at
  0.5 s, so a 0.3 s dab of the pedal is still the same press and the real lift-off afterwards sets."""
  d = Drive()
  _red_light(d, 5.0)
  d.tick(100, gas_pressed=True, v_ego=14.0, a_ego=ON_THE_POWER_MS2, **STEER_ONLY)
  d.tick(30, v_ego=14.0, a_ego=ON_THE_POWER_MS2, **STEER_ONLY)       # 0.3 s lift, shorter than 0.5 s
  assert not d.b._gas_spent and not d.fired(), "precondition: too short to be judged"
  d.tick(100, gas_pressed=True, v_ego=15.0, a_ego=ON_THE_POWER_MS2, **STEER_ONLY)
  d.tick(1, v_ego=15.0, a_ego=ON_THE_POWER_MS2, **STEER_ONLY)
  d.tick(400, v_ego=15.0, **STEER_ONLY)
  assert d.fired() and d.offers[-1][2] == pytest.approx(15.0), d.records[-4:]


def test_the_first_press_is_JUDGED_on_the_SAME_wait_that_fired_the_set():
  """`_gas_spent` -- "first press only" -- is decided by the same clock the gates fired on.

  If the judgement kept the 1.0 s while the gates fired at 0.5 s there would be a half-second window
  AFTER the set in which the accelerator still looked unused, so a second press could name a second
  speed out of one steering-only stretch. That is precisely what "first press only" exists to forbid
  (owner decision 2026-09-13), and it is invisible in the fire timing -- only this test sees it."""
  d = Drive()
  _red_light(d, 5.0)
  d.tick(200, gas_pressed=True, v_ego=16.0, a_ego=ON_THE_POWER_MS2, **STEER_ONLY)
  d.tick(1, v_ego=16.0, a_ego=ON_THE_POWER_MS2, **STEER_ONLY)        # lift-off, on the power
  d.tick(60, v_ego=16.0, a_ego=-2.14, **STEER_ONLY)                  # 0.6 s: past 0.5 s, inside the old 1.0 s
  assert d.fired(), "precondition: the SET fired on the short wait"
  assert d.b._gas_spent, "the first press is still unspent 0.6 s after a set that already fired"
  d.tick(100, gas_pressed=True, v_ego=18.0, **STEER_ONLY)            # a second press, same stretch
  assert "gasSpent" in d.reasons("refuse"), d.phases()


def test_the_decel_window_is_anchored_for_a_GAS_SET_and_deliberately_NOT_for_a_RESUME():
  """The anchor is scoped to the path this change owns. A resume keeps RELEASE_MIN_S and the estimator's
  free-running phase exactly as they have always been -- it commands acceleration where a SET does not,
  and re-timing it is not what was asked for (Rule 5)."""
  d = Drive()
  _red_light(d, 5.0)
  d.tick(200, gas_pressed=True, v_ego=16.0, a_ego=ON_THE_POWER_MS2, **STEER_ONLY)
  d.tick(1, v_ego=16.0, a_ego=ON_THE_POWER_MS2, **STEER_ONLY)
  assert d.b._released_t is not None, "precondition: the lift-off registered"
  assert d.b._v_ref_t == pytest.approx(d.b._released_t), "a gas-set lift-off must anchor the decel window"

  r = normal_brake_and_resume(post_ticks=1)
  assert r.b._released_t is not None, "precondition: the brake release registered"
  assert r.b._v_ref_t < r.b._released_t, "a RESUME release must NOT re-anchor the decel window"


def test_the_short_wait_is_not_reachable_without_a_lift_off_to_latch_it():
  """A brain that has never seen a lift-off holds the LONG wait, so no future path can read a stale
  short one. Checked on a fresh brain and again after an episode has been disarmed."""
  b = M.MadsResumeBrain()
  assert b._gas_wait_s == M.GAS_SET_RELEASE_MIN_S and b._gas_wait_why == "slowing"
  assert math.isnan(b._gas_a0)
  d, _ = _lift_off_at(ON_THE_POWER_MS2)
  assert d.b._gas_wait_s == M.GAS_SET_RELEASE_MIN_ACCEL_S, "precondition: the episode took the short wait"
  d.b._disarm()
  assert d.b._gas_wait_s == M.GAS_SET_RELEASE_MIN_S and d.b._gas_wait_why == "slowing"
  assert math.isnan(d.b._gas_a0)
