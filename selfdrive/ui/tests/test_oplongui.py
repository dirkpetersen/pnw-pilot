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

req 2 (oplongpersist2pnw, driver-directed 2026-08-17): the Lightning (alpha, non-native) toggle's
enabled state PREVIOUSLY also gated on ces_master_on -- the Settings-level "is CES on at all" signal
(ces_enabled(read_ces_mode(params))). Since CES is normally on, that left the toggle greyed almost
all the time -- the "greyed all the time on the Lightning" bug. That gate is REMOVED: the alpha
branch's enabled state is now `not engaged` alone, matched regardless of CES on/off. The `not engaged`
gate is KEPT and is NOT a UX gate -- it's a hard safety requirement, since the toggle's callback
(_on_alpha_long_enabled) applies the change via an OnroadCycleRequested reload, which restarts the
entire onroad process set and must never be triggerable while engaged/driving.

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

oplongdisable (docs/pnw/op-long-features.md §6, driver-confirmed 2026-07-18) adds the MIRROR flow: on
the Lightning, op-long is meant to be exclusive to Experimental -- CES/Chill = stock ACC. Flipping the
button AWAY from Experimental toward CES or Chill while op-long is currently ON must turn op-long back
OFF (`AlphaLongitudinalEnabled=False` + `OnroadCycleRequested=True`, mirroring
developer.py::_on_alpha_long_enabled's else-branch). Unlike the enable direction, this needs no second
tap and no AEB-warning text -- disabling is the SAFE direction (it restores AEB and hands following
back to the truck's own radar). Fable review, F2 (MEDIUM): it DOES still need BOTH gates the enable
direction has -- standstill (`_STANDSTILL_MS`, since `OnroadCycleRequested` restarts the whole onroad
stack regardless of direction and that must never happen while moving) AND engaged (mirroring
ConfirmOutcome's HOLD_ENGAGED): stopped at a light with openpilot ENGAGED, op-long is actively holding
the brake, and the reload would release that hold with the driver's foot off the pedal -> EV creep.
F3 (LOW): the standstill compare uses `abs(v_ego)` so a rolling-backward negative vEgo can't slip
past it. `is_disable_reach` (which nxt values are a disable) and `decide_disable_outcome` (the
standstill+engaged gate) are the pure helpers -- see TestIsDisableReach / TestDecideDisableOutcome.
"""
from openpilot.selfdrive.ui.layouts.settings.developer import (
  AlphaLongToggleState, alpha_long_confirm_should_enable, compute_alpha_long_toggle_state,
)
from openpilot.selfdrive.ui.onroad.exp_button import (
  _BTN_CES, _BTN_CHILL, _BTN_EXP, _CES_CYCLE, _CES_CYCLE_NO_LONG, _CES_CYCLE_OFF_ALPHA, _STANDSTILL_MS,
  ConfirmOutcome, DisableOutcome, decide_confirm_outcome, decide_disable_outcome, is_disable_reach,
  is_exp_slot_reach, select_ces_cycle,
)


class TestComputeAlphaLongToggleState:
  def test_native_op_long_forced_on_and_disabled(self):
    # Tesla Raven, the REAL flag combo: op_long_native=True is what correctly picks this out even
    # though alpha_long_available is ALSO True (see module docstring) -- no real per-car choice, so
    # the toggle is visible, forced checked=True, and greyed out. (c)
    state = compute_alpha_long_toggle_state(
      op_long_native=True, alpha_long_available=True, openpilot_long_control=True,
      is_release=False, engaged=False)
    assert state == AlphaLongToggleState(visible=True, checked=True, enabled=False)

  def test_native_op_long_disabled_regardless_of_engaged(self):
    # the forced-ON state never depends on engaged -- there's nothing to re-enable. (c)
    state = compute_alpha_long_toggle_state(
      op_long_native=True, alpha_long_available=True, openpilot_long_control=True,
      is_release=False, engaged=True)
    assert state.enabled is False
    assert state.checked is True

  def test_op_long_native_takes_precedence_over_alpha_available(self):
    # The regression this fix is about: op_long_native=True AND alpha_long_available=True (the real
    # Tesla combo) must land on the native branch, not the alpha branch. op_long_native must be
    # checked BEFORE alpha_long_available in compute_alpha_long_toggle_state -- if that ordering
    # ever regresses to testing alpha_long_available first, the Tesla's toggle silently starts
    # mirroring AlphaLongitudinalEnabled instead of being forced True (the oplongui2pnw bug). (c)
    state = compute_alpha_long_toggle_state(
      op_long_native=True, alpha_long_available=True, openpilot_long_control=True,
      is_release=False, engaged=False)
    assert state.checked is True
    assert state.enabled is False

  def test_alpha_op_long_enabled_regardless_of_ces_state(self):
    # (a) THE key req 2 regression test: Lightning, not engaged -> enabled=True REGARDLESS of CES
    # on/off -- there is no ces_master_on parameter any more, so there is nothing left to gate this
    # on besides `not engaged`. Previously CES-on alone greyed this out; that gate is gone.
    state = compute_alpha_long_toggle_state(
      op_long_native=False, alpha_long_available=True, openpilot_long_control=False,
      is_release=False, engaged=False)
    assert state.visible is True
    assert state.checked is None  # mirrors the real AlphaLongitudinalEnabled param, not forced
    assert state.enabled is True

  def test_alpha_op_long_disabled_while_engaged(self):
    # (b) the ONE remaining gate on the alpha branch: engaged=True -> enabled=False. This is a
    # SAFETY gate, not a UX one (see module docstring / compute_alpha_long_toggle_state docstring) --
    # it must survive req 2's CES-gate removal untouched.
    state = compute_alpha_long_toggle_state(
      op_long_native=False, alpha_long_available=True, openpilot_long_control=False,
      is_release=False, engaged=True)
    assert state.enabled is False
    assert state.checked is None

  def test_unavailable_car_hidden(self):
    # (d) neither native nor alpha op-long capable at all -- unchanged by req 2.
    state = compute_alpha_long_toggle_state(
      op_long_native=False, alpha_long_available=False, openpilot_long_control=False,
      is_release=False, engaged=False)
    assert state == AlphaLongToggleState(visible=False, checked=None, enabled=False)

  def test_alpha_op_long_on_stays_the_alpha_branch_not_forced(self):
    # Precedence guard, the ORIGINAL direction: Lightning with its alpha op-long toggle currently ON
    # (openpilot_long_control=True, alpha_long_available=True) but op_long_native=False (only the
    # Tesla is native) -- must stay on the ALPHA branch (checked=None, mirrors the real param), NOT
    # forced True, so the driver can always turn it back OFF (docs/pnw/op-long-features.md §6
    # "always able to turn OFF" requirement). openpilot_long_control alone must never force the
    # native branch -- only op_long_native may. Not engaged -> tappable (regardless of CES, req 2).
    state = compute_alpha_long_toggle_state(
      op_long_native=False, alpha_long_available=True, openpilot_long_control=True,
      is_release=False, engaged=False)
    assert state.visible is True
    assert state.checked is None
    assert state.enabled is True

    state_engaged = compute_alpha_long_toggle_state(
      op_long_native=False, alpha_long_available=True, openpilot_long_control=True,
      is_release=False, engaged=True)
    assert state_engaged.enabled is False
    assert state_engaged.checked is None

  def test_release_build_hides_every_case(self):
    for native, alpha_avail, op_long in (
      (True, True, True),     # Tesla
      (False, True, False),   # Lightning, op-long off
      (False, True, True),    # Lightning, op-long on
      (False, False, False),  # unavailable
    ):
      state = compute_alpha_long_toggle_state(
        op_long_native=native, alpha_long_available=alpha_avail, openpilot_long_control=op_long,
        is_release=True, engaged=False)
      assert state.visible is False


class TestAlphaLongConfirmShouldEnable:
  """Fix A (Fable+Gemini review pass 3, MUST-FIX): the op-long ENABLE ConfirmDialog's confirm_callback
  must re-check LIVE vehicle state at CONFIRM time (the dialog can sit open while the driver engages
  and drives off) -- alpha_long_confirm_should_enable is the extracted pure decision, a thin wrapper
  around exp_button.py's decide_confirm_outcome/ConfirmOutcome.ENABLE (reused, not duplicated)."""

  def test_stopped_and_not_engaged_may_enable(self):
    assert alpha_long_confirm_should_enable(v_ego=0.0, engaged=False) is True

  def test_just_under_standstill_threshold_may_enable(self):
    assert alpha_long_confirm_should_enable(v_ego=_STANDSTILL_MS - 0.01, engaged=False) is True

  def test_moving_but_disengaged_blocks_enable(self):
    # THE concrete hole this fix closes: the toggle's live `not engaged` gate permits opening the
    # dialog while disengaged, then the driver engages/drives off before tapping Confirm -- a bare
    # `not engaged` re-check at confirm time would STILL wrongly allow this (engaged can be False
    # while moving). Standstill is the real requirement.
    assert alpha_long_confirm_should_enable(v_ego=15.0, engaged=False) is False

  def test_at_standstill_threshold_blocks_enable(self):
    # Boundary: v_ego == _STANDSTILL_MS counts as still moving (>=), matching decide_confirm_outcome.
    assert alpha_long_confirm_should_enable(v_ego=_STANDSTILL_MS, engaged=True) is False

  def test_stopped_but_engaged_blocks_enable(self):
    # Stopped under active openpilot control (e.g. held at a light) -- still can't safely reload
    # without disengaging first.
    assert alpha_long_confirm_should_enable(v_ego=0.0, engaged=True) is False

  def test_moving_and_engaged_blocks_enable(self):
    assert alpha_long_confirm_should_enable(v_ego=20.0, engaged=True) is False


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


class TestIsDisableReach:
  """oplongdisable: which `nxt` values, on which capability combination, count as the driver flipping
  AWAY from Experimental to hand control back to stock ACC."""

  def test_landing_on_ces_or_chill_with_op_long_on_lightning_is_a_disable_reach(self):
    # The Lightning, op-long currently ON (has_longitudinal_control=True), alpha-capable, non-native:
    # both CES and Chill are disable reaches (the full cycle is CES -> Exp -> Chill -> CES, so a tap
    # landing anywhere but Exp means the driver just left Experimental).
    for nxt in (_BTN_CES, _BTN_CHILL):
      assert is_disable_reach(nxt, has_longitudinal_control=True, alpha_long_available=True, op_long_native=False) is True

  def test_landing_on_exp_is_never_a_disable_reach(self):
    # "Flipping to Experimental while op-long ON -> no change" (op-long-features.md §6, DEC.md
    # design note in the task) -- CES -> Exp is an ordinary cycle step, not a disable trigger.
    assert is_disable_reach(_BTN_EXP, has_longitudinal_control=True, alpha_long_available=True, op_long_native=False) is False

  def test_tesla_native_never_disable_reach_regardless_of_nxt(self):
    # THE proof this can't touch the Raven: op_long_native=True short-circuits to False for every nxt,
    # even though has_longitudinal_control is unconditionally True for a native car (ui_state.py) and
    # would otherwise satisfy every other condition on an ordinary CES<->Chill<->Exp cycle tap.
    for nxt in (_BTN_CES, _BTN_CHILL, _BTN_EXP):
      assert is_disable_reach(nxt, has_longitudinal_control=True, alpha_long_available=True, op_long_native=True) is False

  def test_op_long_off_is_never_a_disable_reach(self):
    # Nothing to disable -- has_longitudinal_control=False short-circuits regardless of nxt (this is
    # the is_exp_slot_reach territory instead, not this branch).
    for nxt in (_BTN_CES, _BTN_CHILL, _BTN_EXP):
      assert is_disable_reach(nxt, has_longitudinal_control=False, alpha_long_available=True, op_long_native=False) is False

  def test_non_alpha_op_long_on_car_is_never_a_disable_reach(self):
    # Gate correctly even though no car in the fleet lands here today: has_longitudinal_control=True
    # via a hypothetical non-alpha, non-native car (plain CP.openpilotLongitudinalControl) must not
    # trigger a disable -- there is no AlphaLongitudinalEnabled toggle to turn off on such a car.
    assert is_disable_reach(_BTN_CES, has_longitudinal_control=True, alpha_long_available=False, op_long_native=False) is False


class TestDecideDisableOutcome:
  """oplongdisable, Fable F2 (MEDIUM) fix: the disable direction's gate is standstill AND engaged,
  symmetric with decide_confirm_outcome's HOLD_ENGAGED. The concrete hazard: stopped at a light with
  openpilot ENGAGED, op-long is holding the brake -- disabling reloads the onroad stack
  (OnroadCycleRequested), which kills controls for >=1 s and releases that hold while the truck sits
  in D with the driver's foot off the pedal (EV creep). So DISABLE requires BOTH stopped AND not
  engaged; stopped-but-engaged holds instead."""

  def test_stopped_and_not_engaged_disables(self):
    assert decide_disable_outcome(v_ego=0.0, engaged=False) is DisableOutcome.DISABLE

  def test_just_under_standstill_threshold_disables(self):
    assert decide_disable_outcome(v_ego=_STANDSTILL_MS - 0.01, engaged=False) is DisableOutcome.DISABLE

  def test_moving_holds_no_disable(self):
    # The core safety property this task is about: at speed, a flip to CES/Chill must NOT restart
    # the onroad stack.
    assert decide_disable_outcome(v_ego=15.0, engaged=False) is DisableOutcome.HOLD_MOVING

  def test_at_standstill_threshold_still_holds(self):
    # Boundary: v_ego == _STANDSTILL_MS counts as still moving (>=), matching decide_confirm_outcome.
    assert decide_disable_outcome(v_ego=_STANDSTILL_MS, engaged=True) is DisableOutcome.HOLD_MOVING

  def test_stopped_but_engaged_holds_engaged_not_disable(self):
    # Fable F2, the regression case: stopped at a light, openpilot ENGAGED and holding the brake --
    # must NOT disable (that would release the brake hold mid-stop -> EV creep risk).
    assert decide_disable_outcome(v_ego=0.0, engaged=True) is DisableOutcome.HOLD_ENGAGED

  def test_moving_and_engaged_holds_moving_takes_precedence(self):
    # Moving wins over engaged -- standstill is checked first, matching decide_confirm_outcome.
    assert decide_disable_outcome(v_ego=20.0, engaged=True) is DisableOutcome.HOLD_MOVING

  def test_negative_v_ego_rolling_backward_blocked_by_abs(self):
    # Fable F3 (LOW hardening): a rolling-backward negative vEgo must not slip past the standstill
    # gate just because it's negative -- abs(v_ego) is compared, not the raw signed value.
    assert decide_disable_outcome(v_ego=-15.0, engaged=False) is DisableOutcome.HOLD_MOVING

  def test_small_negative_v_ego_within_standstill_still_disables(self):
    # A tiny negative creep (well within the standstill band) is still effectively stopped.
    assert decide_disable_outcome(v_ego=-0.01, engaged=False) is DisableOutcome.DISABLE


class TestOpLongOnLightningFullCycleDisable:
  """Integration-flavored (but still pure-function, no widget/pyray) check that the full tap cycle on
  an op-long-ON Lightning produces a disable reach at exactly the two expected landing spots, using
  select_ces_cycle + is_disable_reach together the way _handle_mouse_release does."""

  def test_full_cycle_disables_at_chill_and_ces_not_at_exp(self):
    has_long, alpha_avail, op_native = True, True, False  # Lightning, op-long currently ON
    cycle = select_ces_cycle(has_long, alpha_avail, op_native)
    assert cycle == _CES_CYCLE  # op-long ON always gets the full cycle

    cur = _BTN_CES  # default boot state
    # tap 1: CES -> Exp -- ordinary step, no disable (matches "flip to Experimental -> no change").
    idx = cycle.index(cur)
    nxt = cycle[(idx + 1) % len(cycle)]
    assert nxt == _BTN_EXP
    assert is_disable_reach(nxt, has_long, alpha_avail, op_native) is False
    cur = nxt

    # tap 2: Exp -> Chill -- this IS the disable reach (flip to CES or Chill in the task's wording).
    idx = cycle.index(cur)
    nxt = cycle[(idx + 1) % len(cycle)]
    assert nxt == _BTN_CHILL
    assert is_disable_reach(nxt, has_long, alpha_avail, op_native) is True
    assert decide_disable_outcome(v_ego=0.0, engaged=False) is DisableOutcome.DISABLE
    cur = nxt

    # tap 3 (hypothetically, if op-long hadn't actually been disabled): Chill -> CES -- also a
    # disable reach.
    idx = cycle.index(cur)
    nxt = cycle[(idx + 1) % len(cycle)]
    assert nxt == _BTN_CES
    assert is_disable_reach(nxt, has_long, alpha_avail, op_native) is True
