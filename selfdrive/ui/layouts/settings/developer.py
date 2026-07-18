from typing import NamedTuple

from openpilot.common.params import Params
from openpilot.selfdrive.ui.widgets.ssh_key import ssh_key_item
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.list_view import toggle_item
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.widgets import DialogResult

# Description constants
DESCRIPTIONS = {
  'enable_adb': tr_noop(
    "ADB (Android Debug Bridge) allows connecting to your device over USB or over the network. " +
    "See https://docs.comma.ai/how-to/connect-to-comma for more info."
  ),
  'ssh_key': tr_noop(
    "Warning: This grants SSH access to all public keys in your GitHub settings. Never enter a GitHub username " +
    "other than your own. A comma employee will NEVER ask you to add their GitHub username."
  ),
  'alpha_longitudinal': tr_noop(
    "<b>WARNING: openpilot longitudinal control is in alpha for this car and will disable Automatic Emergency Braking (AEB).</b><br><br>" +
    "On this car, openpilot defaults to the car's built-in ACC instead of openpilot's longitudinal control. " +
    "Enable this to switch to openpilot longitudinal control. Enabling Experimental mode is recommended when enabling openpilot longitudinal control alpha. " +
    "Changing this setting will restart openpilot if the car is powered on.<br><br>" +
    "ON = openpilot longitudinal control (vision leads). OFF = the car's stock ACC (radar). Takes effect at the next ignition cycle."
  ),
  # oplongui2pnw (option A, docs/pnw/op-long-features.md §6): native op-long cars (e.g. Tesla) have
  # no real A/B choice here -- op-long is always on and the toggle is forced-ON + greyed out.
  'alpha_longitudinal_native': tr_noop(
    "<b>WARNING: openpilot longitudinal control disables Automatic Emergency Braking (AEB).</b><br><br>" +
    "openpilot longitudinal control is always on for this car and is not adjustable here."
  ),
}


class AlphaLongToggleState(NamedTuple):
  """Pure UI-decision result for the openpilot-longitudinal-control (Alpha) toggle.

  checked=None means "mirror the real AlphaLongitudinalEnabled param" (the normal alpha-op-long
  case); checked=True/False means the displayed state is FORCED and must not be read from / written
  to the param (the native-op-long case forces True without ever touching the param).
  """
  visible: bool
  checked: bool | None
  enabled: bool


def compute_alpha_long_toggle_state(*, openpilot_long_control: bool, alpha_long_available: bool,
                                     is_release: bool, engaged: bool) -> AlphaLongToggleState:
  """Capability-view-only decision logic (2026-07-11 directive: never branch on carFingerprint/brand).

  Three cases, distinguished solely by CarParams capability fields:
    1. native op-long (e.g. Tesla): openpilot_long_control and not alpha_long_available
       -> visible, forced checked=True, disabled (greyed) -- never a real per-car choice.
    2. alpha op-long (e.g. Lightning): alpha_long_available
       -> visible, checked mirrors the real param, enabled whenever not engaged.
    3. unavailable: neither -> hidden.
  is_release hides the toggle in all three cases (unchanged from prior behavior).
  """
  if is_release:
    return AlphaLongToggleState(visible=False, checked=None, enabled=False)
  if alpha_long_available:
    return AlphaLongToggleState(visible=True, checked=None, enabled=not engaged)
  if openpilot_long_control:
    return AlphaLongToggleState(visible=True, checked=True, enabled=False)
  return AlphaLongToggleState(visible=False, checked=None, enabled=False)


class DeveloperLayout(Widget):
  def __init__(self):
    super().__init__()
    self._params = Params()
    self._is_release = self._params.get_bool("IsReleaseBranch")
    # oplongui2pnw: which alpha_long description/state to show; updated in _update_toggles.
    self._alpha_long_native = False

    # Build items and keep references for callbacks/state updates
    self._adb_toggle = toggle_item(
      lambda: tr("Enable ADB"),
      description=lambda: tr(DESCRIPTIONS["enable_adb"]),
      initial_state=self._params.get_bool("AdbEnabled"),
      callback=self._on_enable_adb,
      enabled=ui_state.is_offroad,
    )

    # SSH enable toggle + SSH key management
    self._ssh_toggle = toggle_item(
      lambda: tr("Enable SSH"),
      description="",
      initial_state=self._params.get_bool("SshEnabled"),
      callback=self._on_enable_ssh,
    )
    self._ssh_keys = ssh_key_item(lambda: tr("SSH Keys"), description=lambda: tr(DESCRIPTIONS["ssh_key"]))

    self._joystick_toggle = toggle_item(
      lambda: tr("Joystick Debug Mode"),
      description="",
      initial_state=self._params.get_bool("JoystickDebugMode"),
      callback=self._on_joystick_debug_mode,
      enabled=ui_state.is_offroad,
    )

    self._long_maneuver_toggle = toggle_item(
      lambda: tr("Longitudinal Maneuver Mode"),
      description="",
      initial_state=self._params.get_bool("LongitudinalManeuverMode"),
      callback=self._on_long_maneuver_mode,
    )

    self._alpha_long_toggle = toggle_item(
      lambda: tr("openpilot Longitudinal Control (Alpha)"),
      description=lambda: tr(DESCRIPTIONS["alpha_longitudinal_native"] if self._alpha_long_native else DESCRIPTIONS["alpha_longitudinal"]),
      initial_state=self._params.get_bool("AlphaLongitudinalEnabled"),
      callback=self._on_alpha_long_enabled,
      enabled=lambda: not ui_state.engaged,
    )

    self._ui_debug_toggle = toggle_item(
      lambda: tr("UI Debug Mode"),
      description="",
      initial_state=self._params.get_bool("ShowDebugInfo"),
      callback=self._on_enable_ui_debug,
    )
    self._on_enable_ui_debug(self._params.get_bool("ShowDebugInfo"))

    self._scroller = Scroller([
      self._adb_toggle,
      self._ssh_toggle,
      self._ssh_keys,
      self._joystick_toggle,
      self._long_maneuver_toggle,
      self._alpha_long_toggle,
      self._ui_debug_toggle,
    ], line_separator=True, spacing=0)

    # Toggles should be not available to change in onroad state
    ui_state.add_offroad_transition_callback(self._update_toggles)

  def _render(self, rect):
    self._scroller.render(rect)

  def show_event(self):
    super().show_event()
    self._scroller.show_event()
    self._update_toggles()

  def _update_toggles(self):
    ui_state.update_params()

    # Hide non-release toggles on release builds
    # TODO: we can do an onroad cycle, but alpha long toggle requires a deinit function to re-enable radar and not fault
    # (alpha_long_toggle's own is_release handling lives in compute_alpha_long_toggle_state below)
    for item in (self._joystick_toggle, self._long_maneuver_toggle):
      item.set_visible(not self._is_release)

    # CP gating
    alpha_state: AlphaLongToggleState | None = None
    if ui_state.CP is not None:
      # oplongui2pnw (option A, docs/pnw/op-long-features.md §6): capability-view only, never
      # carFingerprint/brand -- see compute_alpha_long_toggle_state's docstring for the 3 cases.
      alpha_state = compute_alpha_long_toggle_state(
        openpilot_long_control=ui_state.CP.openpilotLongitudinalControl,
        alpha_long_available=ui_state.CP.alphaLongitudinalAvailable,
        is_release=self._is_release,
        engaged=ui_state.engaged,
      )
      self._alpha_long_native = alpha_state.checked is True
      self._alpha_long_toggle.set_visible(alpha_state.visible)
      if self._alpha_long_native:
        # Forced-ON + greyed unconditionally (never depends on engaged) -> a static disabled beats
        # the live "not engaged" lambda here, since there's nothing to re-enable.
        self._alpha_long_toggle.action_item.set_enabled(False)
      else:
        # Alpha op-long (and hidden/unavailable, harmlessly) keep the ORIGINAL live callable so the
        # toggle keeps tracking ui_state.engaged in real time, not just at each _update_toggles call
        # (matches alpha_state.enabled's value at this instant, but stays live afterwards).
        self._alpha_long_toggle.action_item.set_enabled(lambda: not ui_state.engaged)

      if not alpha_state.visible and not self._alpha_long_native:
        # fpcache2pnw: only a REAL fingerprint that lacks alpha-long support may clear the driver's
        # preference — never a MOCK (flaky/no-car) CP. ui_state.CP comes from CarParamsPersistent,
        # which never-persist-MOCK keeps non-mock, so this is belt-and-braces (the live deleter was
        # selfdrived.py — see the matching guard there).
        # NOTE this branch DOES fire for native op-long cars (e.g. Tesla) on a release build: on
        # is_release, compute_alpha_long_toggle_state returns visible=False/checked=None for EVERY
        # car (native included) — the "native" signal only exists in the non-release branch — so
        # self._alpha_long_native is False there too and the param gets removed, same as every
        # other hidden case. That is BYTE-IDENTICAL to the pre-oplongui2pnw behavior (the old code
        # also unconditionally removed on `self._is_release`, regardless of car) — not a regression,
        # just not something this comment should claim is impossible. The "not self._alpha_long_native"
        # clause is in practice redundant given compute_alpha_long_toggle_state's invariant
        # (checked=True is only ever returned together with visible=True, so
        # "not alpha_state.visible" alone already excludes every native case) — kept for readability,
        # to make the "never touch the param for the forced-native case" rule explicit at the call site.
        if ui_state.CP.brand != "mock":
          self._params.remove("AlphaLongitudinalEnabled")

      long_man_enabled = ui_state.has_longitudinal_control and ui_state.is_offroad()
      self._long_maneuver_toggle.action_item.set_enabled(long_man_enabled)
      if not long_man_enabled:
        self._long_maneuver_toggle.action_item.set_state(False)
        self._params.put_bool("LongitudinalManeuverMode", False)
    else:
      self._long_maneuver_toggle.action_item.set_enabled(False)
      self._alpha_long_native = False
      self._alpha_long_toggle.set_visible(False)

    # TODO: make a param control list item so we don't need to manage internal state as much here
    # refresh toggles from params to mirror external changes
    for key, item in (
      ("AdbEnabled", self._adb_toggle),
      ("SshEnabled", self._ssh_toggle),
      ("JoystickDebugMode", self._joystick_toggle),
      ("LongitudinalManeuverMode", self._long_maneuver_toggle),
      ("ShowDebugInfo", self._ui_debug_toggle),
    ):
      item.action_item.set_state(self._params.get_bool(key))

    # AlphaLongitudinalEnabled is special-cased: a native op-long car (checked=True) forces the
    # DISPLAYED state without ever reading/writing the param; every other case mirrors the real
    # param. This must run after the generic refresh loop above so it isn't clobbered by it.
    if alpha_state is not None and alpha_state.checked is not None:
      self._alpha_long_toggle.action_item.set_state(alpha_state.checked)
    else:
      self._alpha_long_toggle.action_item.set_state(self._params.get_bool("AlphaLongitudinalEnabled"))

  def _on_enable_ui_debug(self, state: bool):
    self._params.put_bool("ShowDebugInfo", state)
    gui_app.set_show_touches(state)
    gui_app.set_show_fps(state)

  def _on_enable_adb(self, state: bool):
    self._params.put_bool("AdbEnabled", state)

  def _on_enable_ssh(self, state: bool):
    self._params.put_bool("SshEnabled", state)

  def _on_joystick_debug_mode(self, state: bool):
    self._params.put_bool("JoystickDebugMode", state)
    self._params.put_bool("LongitudinalManeuverMode", False)
    self._long_maneuver_toggle.action_item.set_state(False)

  def _on_long_maneuver_mode(self, state: bool):
    self._params.put_bool("LongitudinalManeuverMode", state)
    self._params.put_bool("JoystickDebugMode", False)
    self._joystick_toggle.action_item.set_state(False)

  def _on_alpha_long_enabled(self, state: bool):
    if state:
      def confirm_callback(result: DialogResult):
        if result == DialogResult.CONFIRM:
          self._params.put_bool("AlphaLongitudinalEnabled", True)
          self._params.put_bool("OnroadCycleRequested", True)
          self._update_toggles()
        else:
          self._alpha_long_toggle.action_item.set_state(False)

      # show confirmation dialog
      content = (f"<h1>{self._alpha_long_toggle.title}</h1><br>" +
                 f"<p>{self._alpha_long_toggle.description}</p>")

      dlg = ConfirmDialog(content, tr("Enable"), rich=True, callback=confirm_callback)
      gui_app.push_widget(dlg)

    else:
      self._params.put_bool("AlphaLongitudinalEnabled", False)
      self._params.put_bool("OnroadCycleRequested", True)
      self._update_toggles()
