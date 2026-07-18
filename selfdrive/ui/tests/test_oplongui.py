"""oplongui2pnw (docs/pnw/op-long-features.md §6, option A): unit tests for the two pure decision
helpers backing the op-long toggle UX. Both helpers take only CarParams capability fields (never
carFingerprint/brand) plus plain UI state -- no pyray graphics context is needed to exercise them,
so this covers the logic without constructing the full raylib widgets."""
from openpilot.selfdrive.ui.layouts.settings.developer import AlphaLongToggleState, compute_alpha_long_toggle_state
from openpilot.selfdrive.ui.onroad.exp_button import should_show_op_long_hint


class TestComputeAlphaLongToggleState:
  def test_native_op_long_forced_on_and_disabled(self):
    # e.g. Tesla: openpilotLongitudinalControl=True, alphaLongitudinalAvailable=False -- no real
    # per-car choice, so the toggle is visible, forced checked=True, and greyed out.
    state = compute_alpha_long_toggle_state(
      openpilot_long_control=True, alpha_long_available=False, is_release=False, engaged=False)
    assert state == AlphaLongToggleState(visible=True, checked=True, enabled=False)

  def test_native_op_long_disabled_regardless_of_engaged(self):
    # the forced-ON state never depends on engaged -- there's nothing to re-enable.
    state = compute_alpha_long_toggle_state(
      openpilot_long_control=True, alpha_long_available=False, is_release=False, engaged=True)
    assert state.enabled is False
    assert state.checked is True

  def test_alpha_op_long_visible_mirrors_param_and_tappable_offroad(self):
    # e.g. Lightning: alphaLongitudinalAvailable=True -- a real A/B choice.
    state = compute_alpha_long_toggle_state(
      openpilot_long_control=False, alpha_long_available=True, is_release=False, engaged=False)
    assert state.visible is True
    assert state.checked is None  # mirrors the real AlphaLongitudinalEnabled param, not forced
    assert state.enabled is True  # not engaged -> tappable

  def test_alpha_op_long_disabled_while_engaged(self):
    state = compute_alpha_long_toggle_state(
      openpilot_long_control=False, alpha_long_available=True, is_release=False, engaged=True)
    assert state.enabled is False
    assert state.checked is None

  def test_unavailable_car_hidden(self):
    # neither native nor alpha op-long capable at all.
    state = compute_alpha_long_toggle_state(
      openpilot_long_control=False, alpha_long_available=False, is_release=False, engaged=False)
    assert state == AlphaLongToggleState(visible=False, checked=None, enabled=False)

  def test_release_build_hides_every_case(self):
    for op_long, alpha_avail in ((True, False), (False, True), (False, False)):
      state = compute_alpha_long_toggle_state(
        openpilot_long_control=op_long, alpha_long_available=alpha_avail, is_release=True, engaged=False)
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
