"""oplongui2pnw / oplongboot2pnw (docs/pnw/op-long-features.md §6): unit tests for the two pure
decision helpers backing the op-long toggle UX. Both helpers take only CarParams capability fields
(never carFingerprint/brand) plus plain UI state -- no pyray graphics context is needed to exercise
them, so this covers the logic without constructing the full raylib widgets."""
from openpilot.selfdrive.ui.layouts.settings.developer import AlphaLongToggleState, compute_alpha_long_toggle_state
from openpilot.selfdrive.ui.onroad.exp_button import _BTN_CES, _BTN_CHILL, _BTN_EXP, should_show_op_long_hint


class TestComputeAlphaLongToggleState:
  def test_native_op_long_forced_on_and_disabled(self):
    # e.g. Tesla: openpilotLongitudinalControl=True, alphaLongitudinalAvailable=False -- no real
    # per-car choice, so the toggle is visible, forced checked=True, and greyed out. The native
    # branch ignores ces_button entirely -- pass a non-Experimental value to prove that.
    state = compute_alpha_long_toggle_state(
      openpilot_long_control=True, alpha_long_available=False, is_release=False, engaged=False, ces_button=_BTN_CES)
    assert state == AlphaLongToggleState(visible=True, checked=True, enabled=False)

  def test_native_op_long_disabled_regardless_of_engaged(self):
    # the forced-ON state never depends on engaged -- there's nothing to re-enable.
    state = compute_alpha_long_toggle_state(
      openpilot_long_control=True, alpha_long_available=False, is_release=False, engaged=True, ces_button=_BTN_EXP)
    assert state.enabled is False
    assert state.checked is True

  def test_alpha_op_long_greyed_when_ces_not_experimental(self):
    # oplongboot2pnw revision: e.g. Lightning at ces_button=CES (0) -- op-long always boots OFF now
    # (manager.py forces AlphaLongitudinalEnabled=False at startup), so the toggle stays greyed
    # until the driver is actually in Experimental mode.
    state = compute_alpha_long_toggle_state(
      openpilot_long_control=False, alpha_long_available=True, is_release=False, engaged=False, ces_button=_BTN_CES)
    assert state.visible is True
    assert state.checked is None  # mirrors the real AlphaLongitudinalEnabled param, not forced
    assert state.enabled is False  # not in Experimental -> greyed

  def test_alpha_op_long_enabled_when_experimental_and_not_engaged(self):
    # e.g. Lightning at ces_button=Experimental (2), offroad -- a real A/B choice, tappable.
    state = compute_alpha_long_toggle_state(
      openpilot_long_control=False, alpha_long_available=True, is_release=False, engaged=False, ces_button=_BTN_EXP)
    assert state.visible is True
    assert state.checked is None
    assert state.enabled is True

  def test_alpha_op_long_disabled_while_engaged(self):
    # Experimental but engaged -- still greyed (nothing tappable while driving).
    state = compute_alpha_long_toggle_state(
      openpilot_long_control=False, alpha_long_available=True, is_release=False, engaged=True, ces_button=_BTN_EXP)
    assert state.enabled is False
    assert state.checked is None

  def test_alpha_op_long_greyed_in_chill(self):
    # ces_button=Chill (1) is also not Experimental -- greyed, same as CES (0).
    state = compute_alpha_long_toggle_state(
      openpilot_long_control=False, alpha_long_available=True, is_release=False, engaged=False, ces_button=_BTN_CHILL)
    assert state.enabled is False

  def test_unavailable_car_hidden(self):
    # neither native nor alpha op-long capable at all.
    state = compute_alpha_long_toggle_state(
      openpilot_long_control=False, alpha_long_available=False, is_release=False, engaged=False, ces_button=_BTN_CES)
    assert state == AlphaLongToggleState(visible=False, checked=None, enabled=False)

  def test_alpha_op_long_on_stays_the_alpha_branch_not_forced(self):
    # Precedence guard: a car that both has openpilot_long_control=True AND
    # alpha_long_available=True (e.g. the Lightning with its alpha op-long toggle currently ON,
    # which per §6 only happens while in Experimental) must stay on the ALPHA branch -- checked=None
    # (mirrors the real param), NOT forced True. alpha_long_available must be checked before
    # openpilot_long_control in compute_alpha_long_toggle_state; if that ordering were ever
    # flipped, an op-long-ON Lightning would become forced-ON+greyed like Tesla and the driver
    # could no longer turn it back OFF (the "always able to turn OFF" requirement from
    # docs/pnw/op-long-features.md §6). This test fails if that ordering regresses.
    state = compute_alpha_long_toggle_state(
      openpilot_long_control=True, alpha_long_available=True, is_release=False, engaged=False, ces_button=_BTN_EXP)
    assert state.visible is True
    assert state.checked is None
    assert state.enabled is True

    state_engaged = compute_alpha_long_toggle_state(
      openpilot_long_control=True, alpha_long_available=True, is_release=False, engaged=True, ces_button=_BTN_EXP)
    assert state_engaged.enabled is False
    assert state_engaged.checked is None

  def test_release_build_hides_every_case(self):
    for op_long, alpha_avail in ((True, False), (False, True), (False, False), (True, True)):
      state = compute_alpha_long_toggle_state(
        openpilot_long_control=op_long, alpha_long_available=alpha_avail, is_release=True, engaged=False, ces_button=_BTN_EXP)
      assert state.visible is False


class TestShouldShowOpLongHint:
  def test_lightning_op_long_off_shows_hint(self):
    assert should_show_op_long_hint(ces_master=True, has_longitudinal_control=False, alpha_long_available=True) is True

  def test_tesla_never_shows_hint(self):
    # Tesla always has_longitudinal_control=True (native op-long) and alpha_long_available=False.
    assert should_show_op_long_hint(ces_master=True, has_longitudinal_control=True, alpha_long_available=False) is False

  def test_lightning_op_long_on_hides_hint(self):
    assert should_show_op_long_hint(ces_master=True, has_longitudinal_control=True, alpha_long_available=True) is False

  def test_ces_master_off_hides_hint(self):
    assert should_show_op_long_hint(ces_master=False, has_longitudinal_control=False, alpha_long_available=True) is False

  def test_car_without_any_op_long_capability_hides_hint(self):
    assert should_show_op_long_hint(ces_master=True, has_longitudinal_control=False, alpha_long_available=False) is False
