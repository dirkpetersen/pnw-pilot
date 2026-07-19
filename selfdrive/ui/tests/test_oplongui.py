"""oplongexp2pnw (docs/pnw/op-long-features.md §6; supersedes oplongfix2pnw / oplongui2pnw / the
discarded oplongboot2pnw): unit tests for the pure decision helpers backing the op-long toggle UX and
the onroad Experimental-button re-enable path. All helpers take only CarParams capability fields
(never carFingerprint/brand) plus plain UI state -- no pyray graphics context is needed to exercise
them, so this covers the logic without constructing the full raylib widgets.

Flag ground truth (verified against opendbc_repo/opendbc/car/tesla/interface.py::_get_params_sx and
opendbc_repo/opendbc/car/ford/interface.py):
  Tesla Raven:    op_long_native=True,  alpha_long_available=True,  openpilot_long_control=True
                  (opendbc sets alphaLongitudinalAvailable and openpilotLongitudinalControl BOTH
                  unconditionally True on the legacy/HW3 path -- there is no alpha_long_available=
                  False Tesla case; the oplongui2pnw tests encoded that wrong flag combo).
  Lightning OFF:  op_long_native=False, alpha_long_available=True,  openpilot_long_control=False
  Lightning ON:   op_long_native=False, alpha_long_available=True,  openpilot_long_control=True

The Lightning (alpha, non-native) toggle's enabled state also gates on ces_master_on -- the
Settings-level "is CES on at all" signal (ces_enabled(read_ces_mode(params)), the same reader
exp_button.py/ces_status.py use). The driver's enable path is: turn CES mode off -> toggle un-greys
-> enable op-long. This deliberately does NOT gate on the live CESButtonState (Experimental is
unreachable without op-long in the first place, so gating on "currently in Experimental" deadlocks --
that was the discarded oplongboot2pnw's mistake).

oplongexp2pnw adds a SECOND, onroad enable path: flipping the top-right button to Experimental while
op-long is OFF on an alpha-capable car now starts the enable flow directly (no Settings trip). Per a
Fable review of the first cut, this is a TWO-STEP, standstill-gated flow, not a single tap:
  1. select_ces_cycle / is_exp_slot_reach: which cycle a car uses, and whether a tap that reaches the
     Exp slot is a "re-enable reach" (arms a confirm window) vs an ordinary cycle step. HIGH-1 fix:
     the off-alpha-capable cycle is CES -> Chill -> Exp (_CES_CYCLE_OFF_ALPHA), NOT (CES, Exp, Chill)
     -- Chill is the driver's ICBM kill switch and must be reachable in ONE tap from the default boot
     state, so it comes before the enable-reach position, not after it.
  2. decide_confirm_outcome: the SECOND, confirming tap (within the 3 s arm window) only actually
     enables at STANDSTILL (v_ego < 0.1 m/s) AND not engaged -- HIGH-2 fix: the original gate was
     `not engaged` alone, which does not imply stopped (a driver can be moving with openpilot
     disengaged). Stopped-but-engaged and moving get distinct outcomes/hints (HOLD_ENGAGED vs
     HOLD_MOVING -- Fable F7: "disengage" vs "stop", not always "stop").
See TestSelectCesCycle / TestIsExpSlotReach / TestDecideConfirmOutcome below.
"""
from openpilot.selfdrive.ui.layouts.settings.developer import AlphaLongToggleState, compute_alpha_long_toggle_state
from openpilot.selfdrive.ui.onroad.exp_button import (
  _BTN_CES, _BTN_CHILL, _BTN_EXP, _CES_CYCLE, _CES_CYCLE_NO_LONG, _CES_CYCLE_OFF_ALPHA, _STANDSTILL_MS,
  ConfirmOutcome, decide_confirm_outcome, is_exp_slot_reach, select_ces_cycle,
)


class TestComputeAlphaLongToggleState:
  def test_native_op_long_forced_on_and_disabled(self):
    # Tesla Raven, the REAL flag combo: op_long_native=True is what correctly picks this out even
    # though alpha_long_available is ALSO True (see module docstring) -- no real per-car choice, so
    # the toggle is visible, forced checked=True, and greyed out.
    state = compute_alpha_long_toggle_state(
      op_long_native=True, alpha_long_available=True, openpilot_long_control=True,
      is_release=False, engaged=False, ces_master_on=False)
    assert state == AlphaLongToggleState(visible=True, checked=True, enabled=False)

  def test_native_op_long_disabled_regardless_of_engaged_or_ces(self):
    # the forced-ON state never depends on engaged OR ces_master_on -- there's nothing to re-enable.
    state = compute_alpha_long_toggle_state(
      op_long_native=True, alpha_long_available=True, openpilot_long_control=True,
      is_release=False, engaged=True, ces_master_on=True)
    assert state.enabled is False
    assert state.checked is True

  def test_op_long_native_takes_precedence_over_alpha_available(self):
    # The regression this fix is about: op_long_native=True AND alpha_long_available=True (the real
    # Tesla combo) must land on the native branch, not the alpha branch. op_long_native must be
    # checked BEFORE alpha_long_available in compute_alpha_long_toggle_state -- if that ordering
    # ever regresses to testing alpha_long_available first, the Tesla's toggle silently starts
    # mirroring AlphaLongitudinalEnabled instead of being forced True (the oplongui2pnw bug).
    state = compute_alpha_long_toggle_state(
      op_long_native=True, alpha_long_available=True, openpilot_long_control=True,
      is_release=False, engaged=False, ces_master_on=False)
    assert state.checked is True
    assert state.enabled is False

  def test_alpha_op_long_greyed_while_ces_on(self):
    # Lightning, CES mode ON: the toggle is visible (real A/B choice) but greyed -- the driver must
    # turn CES off first. Not engaged, but ces_master_on alone is enough to disable it.
    state = compute_alpha_long_toggle_state(
      op_long_native=False, alpha_long_available=True, openpilot_long_control=False,
      is_release=False, engaged=False, ces_master_on=True)
    assert state.visible is True
    assert state.checked is None
    assert state.enabled is False

  def test_alpha_op_long_tappable_with_ces_off(self):
    # Lightning, CES mode OFF, offroad: the enable path -- toggle is tappable.
    state = compute_alpha_long_toggle_state(
      op_long_native=False, alpha_long_available=True, openpilot_long_control=False,
      is_release=False, engaged=False, ces_master_on=False)
    assert state.visible is True
    assert state.checked is None  # mirrors the real AlphaLongitudinalEnabled param, not forced
    assert state.enabled is True

  def test_alpha_op_long_disabled_while_engaged_even_with_ces_off(self):
    state = compute_alpha_long_toggle_state(
      op_long_native=False, alpha_long_available=True, openpilot_long_control=False,
      is_release=False, engaged=True, ces_master_on=False)
    assert state.enabled is False
    assert state.checked is None

  def test_unavailable_car_hidden(self):
    # neither native nor alpha op-long capable at all.
    state = compute_alpha_long_toggle_state(
      op_long_native=False, alpha_long_available=False, openpilot_long_control=False,
      is_release=False, engaged=False, ces_master_on=False)
    assert state == AlphaLongToggleState(visible=False, checked=None, enabled=False)

  def test_alpha_op_long_on_stays_the_alpha_branch_not_forced(self):
    # Precedence guard, the ORIGINAL direction: Lightning with its alpha op-long toggle currently ON
    # (openpilot_long_control=True, alpha_long_available=True) but op_long_native=False (only the
    # Tesla is native) -- must stay on the ALPHA branch (checked=None, mirrors the real param), NOT
    # forced True, so the driver can always turn it back OFF (docs/pnw/op-long-features.md §6
    # "always able to turn OFF" requirement). openpilot_long_control alone must never force the
    # native branch -- only op_long_native may. CES off + not engaged -> tappable.
    state = compute_alpha_long_toggle_state(
      op_long_native=False, alpha_long_available=True, openpilot_long_control=True,
      is_release=False, engaged=False, ces_master_on=False)
    assert state.visible is True
    assert state.checked is None
    assert state.enabled is True

    state_engaged = compute_alpha_long_toggle_state(
      op_long_native=False, alpha_long_available=True, openpilot_long_control=True,
      is_release=False, engaged=True, ces_master_on=False)
    assert state_engaged.enabled is False
    assert state_engaged.checked is None

  def test_release_build_hides_every_case(self):
    for native, alpha_avail, op_long, ces_on in (
      (True, True, True, False),     # Tesla
      (False, True, False, False),   # Lightning, op-long off, CES off
      (False, True, False, True),    # Lightning, op-long off, CES on
      (False, True, True, False),    # Lightning, op-long on
      (False, False, False, False),  # unavailable
    ):
      state = compute_alpha_long_toggle_state(
        op_long_native=native, alpha_long_available=alpha_avail, openpilot_long_control=op_long,
        is_release=True, engaged=False, ces_master_on=ces_on)
      assert state.visible is False


class TestSelectCesCycle:
  def test_op_long_on_gets_full_cycle(self):
    # Tesla (native), or the Lightning once alpha op-long is ON: unchanged, full cycle either way.
    assert select_ces_cycle(has_longitudinal_control=True, alpha_long_available=True, op_long_native=True) == _CES_CYCLE
    assert select_ces_cycle(has_longitudinal_control=True, alpha_long_available=True, op_long_native=False) == _CES_CYCLE

  def test_alpha_capable_op_long_off_gets_chill_first_cycle(self):
    # Fable HIGH-1: the Lightning on stock ACC gets _CES_CYCLE_OFF_ALPHA (CES -> Chill -> Exp), NOT
    # the plain _CES_CYCLE (CES -> Exp -> Chill) -- Chill must come before the enable-reach position.
    assert select_ces_cycle(has_longitudinal_control=False, alpha_long_available=True, op_long_native=False) == _CES_CYCLE_OFF_ALPHA
    assert _CES_CYCLE_OFF_ALPHA == (_BTN_CES, _BTN_CHILL, _BTN_EXP)

  def test_chill_is_reachable_in_one_tap_from_ces_on_stock_acc_lightning(self):
    # Fable F8: the concrete regression this cycle reorder fixes. CESButtonState boots at _BTN_CES
    # (CLEAR_ON_MANAGER_START default) every drive -- simulate the FIRST tap of a drive on a
    # Lightning with op-long off and confirm it lands on Chill, the ICBM kill switch, not Exp.
    cycle = select_ces_cycle(has_longitudinal_control=False, alpha_long_available=True, op_long_native=False)
    cur = _BTN_CES
    idx = cycle.index(cur)
    nxt = cycle[(idx + 1) % len(cycle)]
    assert nxt == _BTN_CHILL

  def test_op_long_native_with_has_long_false_still_no_long_cycle(self):
    # Defensive/theoretical: a native car should never actually report has_longitudinal_control=False
    # (ui_state.py forces it True for op_long_native), but if it somehow did, op_long_native=True
    # still excludes it from the "alpha capable and off" branch -- falls through to NO_LONG, not a
    # crash. This documents the guard rather than asserting a real fleet scenario.
    assert select_ces_cycle(has_longitudinal_control=False, alpha_long_available=True, op_long_native=True) == _CES_CYCLE_NO_LONG

  def test_no_op_long_capability_at_all_drops_experimental(self):
    assert select_ces_cycle(has_longitudinal_control=False, alpha_long_available=False, op_long_native=False) == _CES_CYCLE_NO_LONG


class TestIsExpSlotReach:
  def test_landing_on_exp_while_off_alpha_capable_is_a_reach(self):
    assert is_exp_slot_reach(_BTN_EXP, has_longitudinal_control=False, alpha_long_available=True, op_long_native=False) is True

  def test_landing_on_ces_or_chill_is_never_a_reach(self):
    for nxt in (_BTN_CES, _BTN_CHILL):
      assert is_exp_slot_reach(nxt, has_longitudinal_control=False, alpha_long_available=True, op_long_native=False) is False

  def test_landing_on_exp_with_op_long_already_on_is_not_a_reach(self):
    # Real Experimental, not a re-enable reach -- has_longitudinal_control=True short-circuits.
    assert is_exp_slot_reach(_BTN_EXP, has_longitudinal_control=True, alpha_long_available=True, op_long_native=False) is False

  def test_landing_on_exp_native_car_is_not_a_reach(self):
    # Tesla: has_longitudinal_control is always True for a native car (ui_state.py), so this can't
    # actually reach the op_long_native branch of the condition in practice -- but confirm it's
    # False either way (defensive, matches select_ces_cycle's defensive case above).
    assert is_exp_slot_reach(_BTN_EXP, has_longitudinal_control=False, alpha_long_available=True, op_long_native=True) is False

  def test_landing_on_exp_no_op_long_capability_is_not_a_reach(self):
    # No capability at all -- can't happen via select_ces_cycle (Exp isn't in _CES_CYCLE_NO_LONG),
    # but is_exp_slot_reach itself must still degrade safely (no false arm) if ever called with this
    # combination directly.
    assert is_exp_slot_reach(_BTN_EXP, has_longitudinal_control=False, alpha_long_available=False, op_long_native=False) is False


class TestDecideConfirmOutcome:
  """Fable HIGH-2/F7: the confirming (second) tap's outcome is gated on STANDSTILL, not just
  `not engaged` -- a driver can be moving with openpilot disengaged."""

  def test_stopped_and_not_engaged_enables(self):
    assert decide_confirm_outcome(v_ego=0.0, engaged=False) is ConfirmOutcome.ENABLE

  def test_just_under_standstill_threshold_enables(self):
    assert decide_confirm_outcome(v_ego=_STANDSTILL_MS - 0.01, engaged=False) is ConfirmOutcome.ENABLE

  def test_moving_but_not_engaged_holds_moving_not_enable(self):
    # The HIGH-2 regression case: manual/disengaged driving at speed must NOT enable (a reload would
    # happen at speed) -- `not engaged` alone is not sufficient.
    assert decide_confirm_outcome(v_ego=15.0, engaged=False) is ConfirmOutcome.HOLD_MOVING

  def test_at_standstill_threshold_holds_moving(self):
    # Boundary: v_ego == _STANDSTILL_MS counts as still moving (>=), not stopped.
    assert decide_confirm_outcome(v_ego=_STANDSTILL_MS, engaged=True) is ConfirmOutcome.HOLD_MOVING

  def test_stopped_but_engaged_holds_engaged(self):
    # Fable F7: stopped (e.g. held at a light under openpilot control) but still engaged -> distinct
    # from the moving case; the hint must say "disengage", not "stop" (the car is already stopped).
    assert decide_confirm_outcome(v_ego=0.0, engaged=True) is ConfirmOutcome.HOLD_ENGAGED

  def test_moving_and_engaged_holds_moving_takes_precedence(self):
    # Moving wins over engaged -- standstill is checked first (see decide_confirm_outcome docstring).
    assert decide_confirm_outcome(v_ego=20.0, engaged=True) is ConfirmOutcome.HOLD_MOVING
