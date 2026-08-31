from typing import NamedTuple

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.pnw_vehicle import PnwVehicle
from openpilot.selfdrive.ui.onroad.exp_button import (ConfirmOutcome, DisableOutcome,
                                                      decide_confirm_outcome, decide_disable_outcome)
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


def alpha_long_confirm_should_enable(v_ego: float, engaged: bool, carstate_alive: bool = True,
                                     onroad: bool = True) -> bool:
  """oplongpersist2pnw (Fix A, Fable+Gemini review pass 3, MUST-FIX): whether the op-long ENABLE
  ConfirmDialog's confirming tap may actually enable op-long RIGHT NOW.

  The toggle's grey-out (compute_alpha_long_toggle_state) only gates OPENING the dialog -- it is a
  live, per-frame check (ui_state.engaged), so it's safe at tap time. But the dialog itself has no
  re-check: once open, it can sit there while the driver engages via the stalk and drives off (nothing
  auto-dismisses it), and the ORIGINAL confirm_callback wrote AlphaLongitudinalEnabled=True +
  OnroadCycleRequested=True unconditionally on CONFIRM -- restarting the entire onroad process set
  (hardwared.py) at whatever speed the truck happens to be moving. req 2 widened the hole: the dialog
  now opens any time the car is merely disengaged, where the old CES-master gate previously kept it
  unreachable in practice.

  Reuses exp_button.py's decide_confirm_outcome/_STANDSTILL_MS (the SAME two-gesture enable path's
  final gate) rather than duplicating the threshold: `engaged` alone is insufficient because a
  DISENGAGED-but-MOVING car is still moving and the reload would still happen at speed -- the real
  requirement is STANDSTILL (v_ego < _STANDSTILL_MS), with `engaged` checked second (stopped but still
  under active control, e.g. held at a light, still can't safely reload without disengaging first).

  expbtnguard2pnw (2026-08-31): also threads `carstate_alive` through. This is the SECOND door onto
  the same hazard as the onroad Experimental button -- a dead/not-yet-alive carState reads
  vEgo == 0.0 (capnp zeros for an unreceived service), which looked like a standstill and would have
  let the Settings confirm reload the onroad stack at speed. Fixing only the onroad button would
  have left this path open. Defaults True so the signature stays compatible.

  ...but OFFROAD is the one place that liveness check must NOT apply (Gemini review, HIGH). `card` is
  an only_onroad process, so offroad carState is legitimately dead and sm.alive is legitimately
  False -- and offroad is exactly when a parked driver opens Settings to flip this. Gating on
  liveness alone would have silently bricked the toggle whenever it is most likely to be used, with
  the dialog appearing and Confirm doing nothing. Offroad there is also no hazard to guard: the
  onroad stack is not running, so there is nothing for OnroadCycleRequested to tear down, and the car
  is not being driven by openpilot. So `onroad=False` short-circuits to True; the speed/engaged/alive
  gates apply only while onroad, where they mean something.

  Fable review (LOW): the bypass requires BOTH "not onroad" AND "carState is dead". `onroad` is
  sourced from deviceState.started (the manager's own view of whether an onroad stack exists), not
  from ui_state.is_onroad(), which ANDs in the UI's pandaStates ignition read -- and this hardware's
  quick-release harness has known wandering power latches, so a momentary ignition-false could
  otherwise present as "offroad" for ~a second while the stack is running and the car is moving,
  letting an open confirm dialog skip even the speed check."""
  if not onroad and not carstate_alive:
    return True
  return decide_confirm_outcome(v_ego, engaged, carstate_alive) is ConfirmOutcome.ENABLE


def alpha_long_toggle_should_disable(v_ego: float, engaged: bool, carstate_alive: bool = True,
                                     onroad: bool = True) -> bool:
  """expbtnguard2pnw (Fable review, MEDIUM -- a PRE-EXISTING third door): whether the Settings
  toggle may turn alpha-long OFF right now.

  The disable branch of _on_alpha_long_enabled wrote AlphaLongitudinalEnabled=False +
  OnroadCycleRequested=True with NO gates whatsoever -- no standstill, no engaged, no liveness --
  while the toggle itself is live onroad whenever merely disengaged
  (`set_enabled(lambda: not ui_state.engaged)`). So a passenger flipping it off while the truck was
  doing 70 disengaged reloaded modeld/controlsd/card AT SPEED: exactly the hazard the enable
  direction and the onroad Experimental button both refuse, reached through the one path nobody had
  gated. exp_button's own disable direction already answers HOLD_MOVING for this.

  Reuses decide_disable_outcome rather than duplicating the rule, and applies it onroad-only on the
  same reasoning as alpha_long_confirm_should_enable: offroad there is no stack to tear down."""
  if not onroad and not carstate_alive:
    return True
  return decide_disable_outcome(v_ego, engaged, carstate_alive) is DisableOutcome.DISABLE


def compute_alpha_long_toggle_state(*, op_long_native: bool, alpha_long_available: bool,
                                     openpilot_long_control: bool, is_release: bool,
                                     engaged: bool) -> AlphaLongToggleState:
  """Capability-view-only decision logic (2026-07-11 directive: never branch on carFingerprint/brand).

  oplongpersist2pnw (req 2, supersedes oplongfix2pnw / oplongui2pnw / the discarded oplongboot2pnw):
  three cases, distinguished by capability-view fields -- op_long_native is itself the capability
  (pnw_vehicle.PnwVehicle), never a carFingerprint/brand string here.
    1. native op-long (e.g. Tesla): op_long_native
       -> visible, forced checked=True, disabled (greyed) -- never a real per-car choice, and NEVER
       gated on engaged (there's nothing to re-enable). Checked FIRST, before alpha_long_available:
       the Tesla ALSO has alpha_long_available=True (opendbc's tesla _get_params_sx sets both
       openpilotLongitudinalControl and alphaLongitudinalAvailable unconditionally), so the old test
       `openpilot_long_control and not alpha_long_available` never matched the Tesla and it fell
       through to the alpha branch -- that was the bug this supersedes.
    2. alpha op-long (e.g. Lightning): alpha_long_available (and not native)
       -> visible, checked mirrors the real param, enabled = not engaged. req 2 DROPS the prior
       CES-master gate (`(not ces_master_on) and not engaged`, driver-directed 2026-08-17): CES is
       normally on, so gating the toggle on it left it greyed almost all the time on the Lightning --
       the driver wants a live on/off switch, not one gated behind turning CES off first. The
       `not engaged` gate is KEPT -- it is a hard SAFETY requirement, not a UX gate: the toggle's
       callback (_on_alpha_long_enabled) writes the param via an OnroadCycleRequested reload, which
       restarts the entire onroad process set, and that must never be triggerable while
       engaged/driving.
    3. unavailable: neither -> hidden.
  is_release hides the toggle in all three cases (unchanged from prior behavior).

  openpilot_long_control is retained in the signature for call-site parity with CarParams' fields,
  but no longer drives a branch: op_long_native fully replaces its role in case 1, and no car in the
  pnw capability matrix has openpilotLongitudinalControl=True while being neither native nor
  alpha-available.
  """
  if is_release:
    return AlphaLongToggleState(visible=False, checked=None, enabled=False)
  if op_long_native:
    return AlphaLongToggleState(visible=True, checked=True, enabled=False)
  if alpha_long_available:
    return AlphaLongToggleState(visible=True, checked=None, enabled=not engaged)
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
      # oplongpersist2pnw (req 2, docs/pnw/op-long-features.md §6): capability-view only, never
      # carFingerprint/brand directly here -- op_long_native comes from pnw_vehicle.py's
      # PnwVehicle (a per-platform constant, not session state, so no live_op_long is needed).
      # The prior CES-master gate (ces_enabled(read_ces_mode(params))) is REMOVED (req 2): CES is
      # normally on, so gating the toggle on it left it greyed almost all the time on the Lightning.
      # Only `not engaged` gates the alpha branch now -- see compute_alpha_long_toggle_state's
      # docstring for why that gate stays (it's a hard safety requirement, not a UX one).
      alpha_state = compute_alpha_long_toggle_state(
        op_long_native=PnwVehicle(ui_state.CP).op_long_native,
        alpha_long_available=ui_state.CP.alphaLongitudinalAvailable,
        openpilot_long_control=ui_state.CP.openpilotLongitudinalControl,
        is_release=self._is_release,
        engaged=ui_state.engaged,
      )
      self._alpha_long_native = alpha_state.checked is True
      self._alpha_long_toggle.set_visible(alpha_state.visible)
      if self._alpha_long_native:
        # Forced-ON + greyed unconditionally (never depends on engaged) -> a static disabled beats a
        # live lambda here, since there's nothing to re-enable.
        self._alpha_long_toggle.action_item.set_enabled(False)
      else:
        # Alpha op-long (and hidden/unavailable, harmlessly): keep engaged live-tracked (matches the
        # pre-oplongfix2pnw behavior). No CES-master snapshot any more (req 2 dropped that gate
        # entirely) -- a plain lambda suffices, no closure-binding trick needed.
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
          # Fix A (Fable+Gemini review pass 3, MUST-FIX): re-check LIVE vehicle state at CONFIRM
          # time, not just at the tap that opened this dialog -- the dialog can sit open while the
          # driver engages via the stalk and drives off (nothing here auto-dismisses it), so a stale
          # "was safe when opened" check is not enough. See alpha_long_confirm_should_enable's
          # docstring for the standstill-vs-engaged rationale.
          if alpha_long_confirm_should_enable(ui_state.sm["carState"].vEgo, ui_state.engaged,
                                              ui_state.sm.alive["carState"],
                                              ui_state.sm["deviceState"].started):
            self._params.put_bool("AlphaLongitudinalEnabled", True)
            self._params.put_bool("OnroadCycleRequested", True)
            self._update_toggles()
          else:
            # Not safe right now (moving, or stopped but still engaged) -- abort: no param write, no
            # cycle request. Revert the toggle visual (AlphaLongitudinalEnabled is still False) and
            # log for traceability; kept deliberately simple -- no new toast/hint UI here, the
            # reverted toggle is the driver-visible signal that the confirm didn't take.
            cloudlog.warning("developer: op-long enable confirm aborted -- not at a safe standstill")
            self._alpha_long_toggle.action_item.set_state(False)
        else:
          self._alpha_long_toggle.action_item.set_state(False)

      # show confirmation dialog
      content = (f"<h1>{self._alpha_long_toggle.title}</h1><br>" +
                 f"<p>{self._alpha_long_toggle.description}</p>")

      dlg = ConfirmDialog(content, tr("Enable"), rich=True, callback=confirm_callback)
      gui_app.push_widget(dlg)

    else:
      # Fable review (MEDIUM): this direction used to be completely ungated, so it could reload the
      # whole onroad stack at speed. Same gate as the enable direction and as exp_button's own
      # disable path; on refusal, snap the toggle visual back to the real param (the enable
      # direction's dialog-cancel branch does the same) so the UI never shows a state that wasn't
      # applied.
      if alpha_long_toggle_should_disable(ui_state.sm["carState"].vEgo, ui_state.engaged,
                                          ui_state.sm.alive["carState"],
                                          ui_state.sm["deviceState"].started):
        self._params.put_bool("AlphaLongitudinalEnabled", False)
        self._params.put_bool("OnroadCycleRequested", True)
      self._update_toggles()
