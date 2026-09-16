"""onoffgas2pnw: the wheel's ACC ON/OFF press is ignored while the driver is on the accelerator.

OWNER REPORT 2026-09-15 ~21:55 PT: *"I engage, I drive multiple times, I accelerate, it all works, and then
at the 3rd or 4th crossing it completely disengages -- so even the lateral control disengages and I have to
re-engage the cruise control. I don't understand why that has to be."*

Root-caused from the driver's own logs (drives/2026-09-15/gassetwait-first-drive/): accelerating away from a
crossing at 25 mph with `steerOverride` active, the truck reported exactly ONE `mainCruise` press
(Steering_Data_FD1 0x083 `CcButtnOnOffPress`). openpilot transmitted ZERO 0x083 frames in that window, so it
came from the wheel and not from our own SET spoof. That latched `cruiseOffRequested` -- the driver's own
2026-09-07 rule, "one press turns everything off" -- and MADS dropped lateral on the same frame.

OWNER DECISION 2026-09-15: ignore the press while the accelerator is down. The rule is unchanged everywhere
else.
"""

import pytest

from openpilot.selfdrive.selfdrived.mads_pnw import off_request_latches


@pytest.mark.parametrize("main_press,lateral_only,gas_pressed,want", [
  # the reported failure: the press arrives mid-acceleration, and must now be ignored
  (True,  True,  True,  False),
  # the deliberate press the rule exists for: steering-only, foot off the accelerator
  (True,  True,  False, True),
  # outside steering-only the press was never latched, with or without the accelerator
  (True,  False, False, False),
  (True,  False, True,  False),
  # no press, nothing latches
  (False, True,  False, False),
  (False, True,  True,  False),
  (False, False, False, False),
])
def test_the_whole_truth_table(main_press, lateral_only, gas_pressed, want):
  assert off_request_latches(main_press, lateral_only, gas_pressed) is want


def test_the_drivers_rule_still_works_with_the_foot_off():
  """The 2026-09-07 behaviour is NOT reversed -- this is the case it was built for, and it must still fire."""
  assert off_request_latches(True, True, False) is True


def test_the_reported_failure_can_no_longer_happen():
  """The exact recorded frame: steering-only, accelerator down, one mainCruise press."""
  assert off_request_latches(True, True, True) is False


def test_the_accelerator_alone_can_never_latch_an_off_request():
  """Guards the inverse mistake -- gating the wrong way round would turn every accelerator press during
  steering-only into an "everything off", which is the opposite of what the driver asked for."""
  for lateral_only in (True, False):
    assert off_request_latches(False, lateral_only, True) is False


@pytest.mark.parametrize("truthy,falsy", [(1, 0), ("yes", ""), ([1], []), (2.5, 0.0)])
def test_non_bool_inputs_are_coerced_not_trusted(truthy, falsy):
  """capnp fields and `any(...)` results are not always real bools. The predicate returns a genuine bool so
  a caller can never store a truthy non-bool into the latch and compare it later."""
  assert off_request_latches(truthy, truthy, falsy) is True
  assert off_request_latches(truthy, truthy, truthy) is False
  assert off_request_latches(falsy, truthy, falsy) is False


def test_selfdrived_uses_the_predicate_and_says_so_when_it_swallows_a_press():
  """selfdrived cannot be imported on the dev host, so its wiring is pinned from source -- the house
  convention in test_engagegoal_pnw.py. Two things must hold: the latch goes through the predicate (not a
  re-implemented condition that could drift from it), and an ignored press is NOT silent (Rule 2)."""
  import pathlib
  src = (pathlib.Path(__file__).parent.parent / "selfdrived.py").read_text()
  assert src.count("off_request_latches(main_press, self.mads.lateral_only, CS.gasPressed)") == 1, \
    "the latch must be decided by the pure predicate"
  assert src.count("self.off_request_t = self.sm.frame * DT_CTRL") == 1, \
    "exactly one place may set the off-request latch"
  # The ignored-press branch must actually CALL the logger. Counting the bare string is not enough: a
  # commented-out call still contains it, and a mutant that replaced the call with `pass  # cloudlog...`
  # survived exactly that check. Require a real statement -- a line whose stripped form STARTS the call.
  i = src.index("off_request_latches(")
  branch = src[i:i + 900]
  calls = [ln for ln in branch.splitlines()
           if ln.strip().startswith("cloudlog.warning(") and "onoffgas2pnw" in ln]
  assert len(calls) == 1, f"an ignored ON/OFF press must say so, as a real call (Rule 2); found {calls}"
