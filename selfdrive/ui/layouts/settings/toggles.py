from cereal import log
from openpilot.common.params import Params, UnknownKeyName
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.list_view import multiple_button_item, toggle_item
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.widgets import DialogResult
from openpilot.selfdrive.ui.ui_state import ui_state

PERSONALITY_TO_INT = log.LongitudinalPersonality.schema.enumerants

# Description constants
DESCRIPTIONS = {
  "OpenpilotEnabledToggle": tr_noop(
    "Use the openpilot system for adaptive cruise control and lane keep driver assistance. " +
    "Your attention is required at all times to use this feature."
  ),
  "DisengageOnAccelerator": tr_noop("When enabled, pressing the accelerator pedal will disengage openpilot."),
  "LongitudinalPersonality": tr_noop(
    "Standard is recommended. In aggressive mode, openpilot will follow lead cars closer and be more aggressive with the gas and brake. " +
    "In relaxed mode openpilot will stay further away from lead cars. On supported cars, you can cycle through these personalities with " +
    "your steering wheel distance button."
  ),
  "IsLdwEnabled": tr_noop(
    "Receive alerts to steer back into the lane when your vehicle drifts over a detected lane line " +
    "without a turn signal activated while driving over 31 mph (50 km/h)."
  ),
  "AlwaysOnDM": tr_noop("Enable driver monitoring even when openpilot is not engaged."),
  "NudgelessLaneChange": tr_noop(
    "Start a lane change from the turn signal alone, without nudging the steering wheel. " +
    "Hold the blinker for about 1.5 seconds above 20 mph (32 km/h) and openpilot will change lanes. " +
    "The lane change is blocked while the blind spot monitor detects a vehicle. Keep your hands on the wheel and check your surroundings. " +
    "Tesla only — has no effect on the Ford F-150 Lightning (the steering-wheel nudge is still required there)."
  ),
  "NoDisengageOnBrake": tr_noop(
    "Keep openpilot engaged when you press the brake pedal instead of disengaging. " +
    "openpilot will resume controlling speed as soon as you release the brake. " +
    "Not currently supported on any car here (Ford or Tesla) — this toggle is disabled."
  ),
  "OvertakeAssist": tr_noop(
    "When you are closing on a slower car and there is an open adjacent lane with the blind spot clear, " +
    "show a green arrow and \"Signal to overtake\". openpilot does NOT change lanes by itself — you start the " +
    "lane change by flicking the turn signal. The prompt only uses the car's rear blind-spot zone and the lane " +
    "model; it cannot see fast traffic approaching from farther back. Always check the lane yourself. Tesla only."
  ),
  "ConditionalExperimentalSwitching": tr_noop(
    "Conditional Experimental Switching (CES): stay in Chill Mode for steady cruising and automatically " +
    "switch to Experimental Mode only for tight curves, low-speed/city driving, stop lights, and when closing " +
    "on a slower lead — then return to Chill. With this on, the top-right button cycles CES / Chill / Experimental " +
    "(orange = forced Experimental). It also slows smoothly for upcoming curves (Vision Turn Speed Control). " +
    "Affects speed/braking only, not steering, and only when openpilot controls longitudinal. NOT a cone/obstacle " +
    "detector and not a substitute for attention — stay ready to brake, especially in construction zones and on curves."
  ),
  "ShowSpeedLimit": tr_noop(
    "Show OpenStreetMap speed limits on the onroad screen and flash a warning when the limit drops. " +
    "When first enabled, openpilot downloads offline maps for Washington, Oregon, and Idaho — keep the car " +
    "parked with Wi-Fi until the download completes (the sign shows \"-\" until then). Requires a GPS fix to display a limit."
  ),
  "SensitiveDriverMonitoring": tr_noop(
    "When enabled, driver monitoring uses the strict stock timeout (about 11 seconds before disengaging). " +
    "When disabled (default), monitoring is relaxed: roughly a 1 hour timeout for looking away or closed eyes, " +
    "and a 3 hour timeout for cell-phone use. The no-face safety timeout is unchanged."
  ),
  'RecordFront': tr_noop("Upload data from the driver facing camera and help improve the driver monitoring algorithm."),
  "IsMetric": tr_noop("Display speed in km/h instead of mph."),
  "AllowSoftwareUpdates": tr_noop(
    "Allow openpilot to download and install software updates. " +
    "Disabled by default so that updates do not overwrite local customizations on this device. " +
    "Enable only when you intend to update."
  ),
  "RecordAudio": tr_noop("Record and store microphone audio while driving. The audio will be included in the dashcam video in comma connect."),
}


def ces_group_enabled(cp) -> bool:
  """PURE, testable: the WHOLE CES group (the CESMode selector + any CES list item) is enabled iff
  openpilot controls longitudinal. Symmetric by construction — the same bool both disables on
  long-off and re-enables on long-on, so nothing can be left greyed out. `cp` is the CarParams (or
  None before a car is seen)."""
  return cp is not None and bool(getattr(cp, "openpilotLongitudinalControl", False))


class TogglesLayout(Widget):
  # auto2xnor: greyed out on non-Tesla cars (the Ford Lightning), enabled on Tesla
  TESLA_ONLY_TOGGLES = ("NudgelessLaneChange", "OvertakeAssist")  # ces2xnor: CES is NOT here — available on all cars
  # auto2xnor: not supported on any car here — always greyed out + forced off
  UNSUPPORTED_TOGGLES = ("NoDisengageOnBrake",)
  # light-ces-gentle: the whole CES group, gated together on openpilotLongitudinalControl. Only widgets
  # present in this layout are gated (CESMode is the selector; the CES* sub-options are params, not list
  # items here). Listed by key so the grey-out enable/disable stays SYMMETRIC for any future CES item.
  CES_GROUP = ("CESMode", "CESCurves", "CESStops", "CESLowSpeed", "CESLead")

  def __init__(self):
    super().__init__()
    self._params = Params()
    self._is_release = self._params.get_bool("IsReleaseBranch")

    # param, title, desc, icon, needs_restart
    self._toggle_defs = {
      "OpenpilotEnabledToggle": (
        lambda: tr("Enable openpilot"),
        DESCRIPTIONS["OpenpilotEnabledToggle"],
        "chffr_wheel.png",
        True,
      ),
      "ExperimentalMode": (
        lambda: tr("Experimental Mode"),
        "",
        "experimental_white.png",
        False,
      ),
      "DisengageOnAccelerator": (
        lambda: tr("Disengage on Accelerator Pedal"),
        DESCRIPTIONS["DisengageOnAccelerator"],
        "disengage_on_accelerator.png",
        False,
      ),
      "IsLdwEnabled": (
        lambda: tr("Enable Lane Departure Warnings"),
        DESCRIPTIONS["IsLdwEnabled"],
        "warning.png",
        False,
      ),
      "AlwaysOnDM": (
        lambda: tr("Always-On Driver Monitoring"),
        DESCRIPTIONS["AlwaysOnDM"],
        "monitoring.png",
        False,
      ),
      "NudgelessLaneChange": (
        lambda: tr("Nudgeless Lane Change"),
        DESCRIPTIONS["NudgelessLaneChange"],
        "warning.png",
        False,
      ),
      "NoDisengageOnBrake": (
        lambda: tr("No Disengage on Braking"),
        DESCRIPTIONS["NoDisengageOnBrake"],
        "disengage_on_accelerator.png",
        False,
      ),
      "OvertakeAssist": (
        lambda: tr("Overtake Assist"),
        DESCRIPTIONS["OvertakeAssist"],
        "warning.png",
        False,
      ),
      "ShowSpeedLimit": (
        lambda: tr("Speed limit display/warning (MAPD/PNW)"),
        DESCRIPTIONS["ShowSpeedLimit"],
        "speed_limit.png",
        False,
      ),
      "SensitiveDriverMonitoring": (
        lambda: tr("Sensitive Driver Monitoring"),
        DESCRIPTIONS["SensitiveDriverMonitoring"],
        "monitoring.png",
        True,
      ),
      "RecordFront": (
        lambda: tr("Record and Upload Driver Camera"),
        DESCRIPTIONS["RecordFront"],
        "monitoring.png",
        True,
      ),
      "RecordAudio": (
        lambda: tr("Record and Upload Microphone Audio"),
        DESCRIPTIONS["RecordAudio"],
        "microphone.png",
        True,
      ),
      "IsMetric": (
        lambda: tr("Use Metric System"),
        DESCRIPTIONS["IsMetric"],
        "metric.png",
        False,
      ),
      "AllowSoftwareUpdates": (
        lambda: tr("Allow software updates"),
        DESCRIPTIONS["AllowSoftwareUpdates"],
        "warning.png",
        False,
      ),
    }

    self._long_personality_setting = multiple_button_item(
      lambda: tr("Driving Personality"),
      lambda: tr(DESCRIPTIONS["LongitudinalPersonality"]),
      buttons=[lambda: tr("Aggressive"), lambda: tr("Standard"), lambda: tr("Relaxed")],
      button_width=255,
      callback=self._set_longitudinal_personality,
      selected_index=self._params.get("LongitudinalPersonality", return_default=True),
      icon="speed_limit.png"
    )

    # light-ces-gentle: CES is now a 3-way selector (Off / Light / Standard) styled like the Driving
    # Personality selector, backed by the INT param CESMode (0/1/2). Off = behavior-neutral; Light =
    # full gentle profile on any car (VTSC soft decel + slow recovery, curves handed to VTSC); Standard
    # = today's default tune. Available on ALL cars (greyed only when openpilot long control is off).
    self._ces_mode_setting = multiple_button_item(
      lambda: tr("CES Mode"),  # short title so the 3 buttons fit; full name explained in the description
      lambda: tr(DESCRIPTIONS["ConditionalExperimentalSwitching"]),
      buttons=[lambda: tr("Off"), lambda: tr("Light"), lambda: tr("Standard")],
      button_width=255,
      callback=self._set_ces_mode,
      selected_index=self._params.get("CESMode", return_default=True),
      icon="speed_limit.png"
    )

    self._toggles = {}
    self._locked_toggles = set()
    for param, (title, desc, icon, needs_restart) in self._toggle_defs.items():
      toggle = toggle_item(
        title,
        desc,
        self._params.get_bool(param),
        callback=lambda state, p=param: self._toggle_callback(state, p),
        icon=icon,
      )

      try:
        locked = self._params.get_bool(param + "Lock")
      except UnknownKeyName:
        locked = False
      toggle.action_item.set_enabled(not locked)

      # Make description callable for live translation
      additional_desc = ""
      if needs_restart and not locked:
        additional_desc = tr("Changing this setting will restart openpilot if the car is powered on.")
      toggle.set_description(lambda og_desc=toggle.description, add_desc=additional_desc: tr(og_desc) + (" " + tr(add_desc) if add_desc else ""))

      # track for engaged state updates
      if locked:
        self._locked_toggles.add(param)

      self._toggles[param] = toggle

      # light-ces-gentle: CES 3-way selector goes directly below the Experimental Mode toggle
      if param == "ExperimentalMode":
        self._toggles["CESMode"] = self._ces_mode_setting

      # insert longitudinal personality after NDOG toggle
      if param == "DisengageOnAccelerator":
        self._toggles["LongitudinalPersonality"] = self._long_personality_setting

    self._update_experimental_mode_icon()
    self._scroller = Scroller(list(self._toggles.values()), line_separator=True, spacing=0)

    ui_state.add_engaged_transition_callback(self._update_toggles)

  def _update_state(self):
    if ui_state.sm.updated["selfdriveState"]:
      personality = PERSONALITY_TO_INT[ui_state.sm["selfdriveState"].personality]
      if personality != ui_state.personality and ui_state.started:
        self._long_personality_setting.action_item.set_selected_button(personality)
      ui_state.personality = personality

  def show_event(self):
    super().show_event()
    self._scroller.show_event()
    self._update_toggles()

  def _update_toggles(self):
    ui_state.update_params()

    e2e_description = tr(
      "openpilot defaults to driving in chill mode. Experimental mode enables alpha-level features that aren't ready for chill mode. " +
      "Experimental features are listed below:<br>" +
      "<h4>End-to-End Longitudinal Control</h4><br>" +
      "Let the driving model control the gas and brakes. openpilot will drive as it thinks a human would, including stopping for red lights and stop signs. " +
      "Since the driving model decides the speed to drive, the set speed will only act as an upper bound. This is an alpha quality feature; " +
      "mistakes should be expected.<br>" +
      "<h4>New Driving Visualization</h4><br>" +
      "The driving visualization will transition to the road-facing wide-angle camera at low speeds to better show some turns. " +
      "The Experimental mode logo will also be shown in the top right corner."
    )

    if ui_state.CP is not None:
      if ui_state.has_longitudinal_control:
        self._toggles["ExperimentalMode"].action_item.set_enabled(True)
        self._toggles["ExperimentalMode"].set_description(e2e_description)
        self._long_personality_setting.action_item.set_enabled(True)
      else:
        # no long for now
        self._toggles["ExperimentalMode"].action_item.set_enabled(False)
        self._toggles["ExperimentalMode"].action_item.set_state(False)
        self._long_personality_setting.action_item.set_enabled(False)
        self._params.remove("ExperimentalMode")

        unavailable = tr("Experimental mode is currently unavailable on this car since the car's stock ACC is used for longitudinal control.")

        long_desc = unavailable + " " + tr("openpilot longitudinal control may come in a future update.")
        if ui_state.CP.alphaLongitudinalAvailable:
          if self._is_release:
            long_desc = unavailable + " " + tr("An alpha version of openpilot longitudinal control can be tested, along with " +
                                               "Experimental mode, on non-release branches.")
          else:
            long_desc = tr("Enable the openpilot longitudinal control (alpha) toggle to allow Experimental mode.")

        self._toggles["ExperimentalMode"].set_description("<b>" + long_desc + "</b><br><br>" + e2e_description)
    else:
      self._toggles["ExperimentalMode"].set_description(e2e_description)

    self._update_experimental_mode_icon()

    # TODO: make a param control list item so we don't need to manage internal state as much here
    # refresh toggles from params to mirror external changes
    for param in self._toggle_defs:
      self._toggles[param].action_item.set_state(self._params.get_bool(param))

    # these toggles need restart, block while engaged
    for toggle_def in self._toggle_defs:
      if self._toggle_defs[toggle_def][3] and toggle_def not in self._locked_toggles:
        self._toggles[toggle_def].action_item.set_enabled(not ui_state.engaged)

    # auto2xnor: per-toggle car support — grey out + force off where unsupported.
    # NudgelessLaneChange: Tesla + Ford F-150 Lightning. OvertakeAssist: Tesla only.
    cp = ui_state.CP
    is_tesla = cp is not None and cp.brand == "tesla"
    is_f150_lightning = cp is not None and cp.carFingerprint == "FORD_F_150_LIGHTNING_MK1"
    toggle_supported = {"NudgelessLaneChange": is_tesla or is_f150_lightning}
    for param in self.TESLA_ONLY_TOGGLES:
      supported = toggle_supported.get(param, is_tesla)
      self._toggles[param].action_item.set_enabled(supported)
      if not supported:
        self._toggles[param].action_item.set_state(False)

    # auto2xnor: unsupported toggles — always greyed out + forced off (no car supports them)
    for param in self.UNSUPPORTED_TOGGLES:
      self._toggles[param].action_item.set_enabled(False)
      self._toggles[param].action_item.set_state(False)

    # ces2xnor / light-ces-gentle: CES (and VTSC, which rides it) require openpilot longitudinal
    # control — they only engage when openpilot drives the gas/brake. Grey out + disable the WHOLE CES
    # group when long control is off (e.g. F-150 Lightning on stock ACC), and re-enable ALL of it when
    # long control is on. Symmetric: anything disabled on long-off MUST re-enable on long-on. We do NOT
    # clear any param, so the settings persist for cars that DO control longitudinal (Tesla) when the
    # same device is swapped between cars.
    #
    # BUG FIXED (light-ces-gentle): previously only the CES master toggle was re-enabled via the
    # ces_long_ok line; the new CESMode selector (and conceptually the CES* sub-options) were not part
    # of a single coherent block, so re-enabling on long-on was asymmetric. Now one block toggles the
    # entire CES_GROUP together. (CESCurves/CESStops/CESLowSpeed/CESLead are params, not list items in
    # this layout, so the only CES UI widget to gate today is the CESMode selector — but the group is
    # listed explicitly so any future CES list item is gated symmetrically by construction.)
    ces_long_ok = ces_group_enabled(cp)
    for param in self.CES_GROUP:
      item = self._toggles.get(param)
      if item is not None:
        item.action_item.set_enabled(ces_long_ok)

  def _render(self, rect):
    self._scroller.render(rect)

  def _update_experimental_mode_icon(self):
    icon = "experimental.png" if self._toggles["ExperimentalMode"].action_item.get_state() else "experimental_white.png"
    self._toggles["ExperimentalMode"].set_icon(icon)

  def _handle_experimental_mode_toggle(self, state: bool):
    confirmed = self._params.get_bool("ExperimentalModeConfirmed")
    if state and not confirmed:
      def confirm_callback(result: DialogResult):
        if result == DialogResult.CONFIRM:
          self._params.put_bool("ExperimentalMode", True)
          self._params.put_bool("ExperimentalModeConfirmed", True)
        else:
          self._toggles["ExperimentalMode"].action_item.set_state(False)
        self._update_experimental_mode_icon()

      # show confirmation dialog
      content = (f"<h1>{self._toggles['ExperimentalMode'].title}</h1><br>" +
                 f"<p>{self._toggles['ExperimentalMode'].description}</p>")
      dlg = ConfirmDialog(content, tr("Enable"), rich=True, callback=confirm_callback)
      gui_app.push_widget(dlg)
    else:
      self._update_experimental_mode_icon()
      self._params.put_bool("ExperimentalMode", state)

  def _toggle_callback(self, state: bool, param: str):
    if param == "ExperimentalMode":
      self._handle_experimental_mode_toggle(state)
      return

    self._params.put_bool(param, state)
    if self._toggle_defs[param][3]:
      self._params.put_bool("OnroadCycleRequested", True)

  def _set_longitudinal_personality(self, button_index: int):
    self._params.put("LongitudinalPersonality", button_index)

  def _set_ces_mode(self, button_index: int):
    # light-ces-gentle: CESMode is the source of truth (0=Off,1=Light,2=Standard). Keep the legacy
    # bool ConditionalExperimentalSwitching mirrored (== CESMode>0) so any back-compat reader agrees.
    self._params.put("CESMode", button_index)
    self._params.put_bool("ConditionalExperimentalSwitching", button_index > 0)
