"""oplongfix2pnw (docs/pnw/op-long-features.md §6; supersedes oplongui2pnw / the discarded
oplongboot2pnw): unit tests for the two pure decision helpers backing the op-long toggle UX. Both
helpers take only CarParams capability fields (never carFingerprint/brand) plus plain UI state -- no
pyray graphics context is needed to exercise them, so this covers the logic without constructing the
full raylib widgets.

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
"""
from openpilot.selfdrive.ui.layouts.settings.developer import AlphaLongToggleState, compute_alpha_long_toggle_state
from openpilot.selfdrive.ui.onroad.exp_button import should_show_op_long_hint


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


class TestShouldShowOpLongHint:
  def test_lightning_op_long_off_shows_hint(self):
    assert should_show_op_long_hint(ces_master=True, has_longitudinal_control=False, alpha_long_available=True) is True

  def test_tesla_never_shows_hint(self):
    # Tesla always has_longitudinal_control=True (native op-long, fixed by ui_state.py's
    # op_long_native check) and alpha_long_available=True (see module docstring) -- the hint is
    # gated on `not has_longitudinal_control`, which is already False here, so alpha_long_available's
    # value doesn't change the outcome; kept at its REAL value for accuracy (the oplongui2pnw test
    # used the wrong alpha_long_available=False).
    assert should_show_op_long_hint(ces_master=True, has_longitudinal_control=True, alpha_long_available=True) is False

  def test_lightning_op_long_on_hides_hint(self):
    assert should_show_op_long_hint(ces_master=True, has_longitudinal_control=True, alpha_long_available=True) is False

  def test_ces_master_off_hides_hint(self):
    assert should_show_op_long_hint(ces_master=False, has_longitudinal_control=False, alpha_long_available=True) is False

  def test_car_without_any_op_long_capability_hides_hint(self):
    assert should_show_op_long_hint(ces_master=True, has_longitudinal_control=False, alpha_long_available=False) is False
