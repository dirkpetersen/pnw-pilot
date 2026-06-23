from cereal import log
from openpilot.common.params import Params, UnknownKeyName
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.list_view import multiple_button_item, toggle_item
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.system.ui.lib.multilang import tr, tr_noop
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
  'RecordFront': tr_noop("Upload data from the driver facing camera and help improve the driver monitoring algorithm."),
  "IsMetric": tr_noop("Display speed in km/h instead of mph."),
  "RecordAudio": tr_noop("Record and store microphone audio while driving. The audio will be included in the dashcam video in comma connect."),
  "GetMapForLocation": tr_noop(
    "Download offline OSM map data for the region you are currently in (US state, or a whole country " +
    "such as Canada for British Columbia — there is no province-level Canadian download). Greyed out " +
    "when your current location is already covered by a downloaded map. Requires a GPS fix and a " +
    "connection to one of your Priority Networks (Wi-Fi). The default Pacific Northwest set (WA/OR/ID) " +
    "is downloaded automatically; use this only when you drive outside it."
  ),
  "ShowSpeedLimit": tr_noop(
    "Show OpenStreetMap speed limits on the onroad screen and flash a warning when the limit drops. " +
    "When first enabled, openpilot downloads offline maps for Washington, Oregon, and Idaho — keep the car " +
    "parked with Wi-Fi until the download completes (the sign shows \"-\" until then). Requires a GPS fix to display a limit."
  ),
  "ConditionalExperimentalSwitching": tr_noop(
    "Conditional Experimental Switching (CES): stay in Chill Mode for steady cruising and automatically " +
    "switch to Experimental Mode only for tight curves, low-speed/city driving, stop lights, and when closing " +
    "on a slower lead — then return to Chill. With this on, the top-right button cycles CES / Chill / Experimental " +
    "(orange = forced Experimental). It also slows smoothly for upcoming curves (Vision Turn Speed Control). " +
    "Affects speed/braking only, not steering, and only when openpilot controls longitudinal. NOT a cone/obstacle " +
    "detector and not a substitute for attention — stay ready to brake, especially in construction zones and on curves."
  ),
  "NudgelessLaneChange": tr_noop(
    "Start a lane change from the turn signal alone, without nudging the steering wheel. " +
    "Hold the blinker for about 0.75 seconds above 20 mph (32 km/h) and openpilot will change lanes. " +
    "The lane change is blocked while the blind spot monitor detects a vehicle. Keep your hands on the wheel and check your surroundings. " +
    "Tesla and the Ford F-150 Lightning only — other cars still require the steering-wheel nudge."
  ),
  "NoDisengageOnBrake": tr_noop(
    "Keep openpilot engaged when you press the brake pedal instead of disengaging. " +
    "openpilot will resume controlling speed as soon as you release the brake. " +
    "Not currently supported on any car here (Ford or Tesla) — this toggle is disabled."
  ),
}


class TogglesLayout(Widget):
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
      # ces2xnor: REPLACES the standalone Experimental Mode toggle. Full Experimental is still
      # reachable via the top-right 3-state button (orange = forced Experimental). CES default OFF.
      "ConditionalExperimentalSwitching": (
        lambda: tr("Conditional Experimental Switching (CES)"),
        DESCRIPTIONS["ConditionalExperimentalSwitching"],
        "speed_limit.png",
        False,
      ),
      # auto2pnw: nudgeless lane change (Tesla + F-150 Lightning) + no-disengage-on-brake (unsupported, greyed)
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
      # mapd2xnor: OSM speed-limit display + lower-limit warning (gates the OSM map download too)
      "ShowSpeedLimit": (
        lambda: tr("Speed limit display/warning (MAPD/PNW)"),
        DESCRIPTIONS["ShowSpeedLimit"],
        "speed_limit.png",
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
      # mapd2pnw: on-demand "Get map for this location" download. Greyed out when the current GPS is
      # already covered by a downloaded map (or no fix); enabled (but off) when uncovered.
      "GetMapForLocation": (
        lambda: tr("Get map for this location"),
        DESCRIPTIONS["GetMapForLocation"],
        "speed_limit.png",
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

    # Resilience (mapd2pnw): drop any toggle whose param isn't registered in params_keys.h before
    # building the toggles. get_bool() on an unregistered key raises UnknownKeyName; if that escapes
    # here it crashes TogglesLayout init -> the UI crash-loops -> the SDE/DRM display driver panics
    # -> the device warm-reboots. A params/UI version mismatch should hide the toggle, not brick the
    # device. (This exact mismatch happened deploying a branch's params_keys.h that lacked CES keys.)
    _valid_defs = {}
    for _param, _spec in self._toggle_defs.items():
      try:
        self._params.get_bool(_param)
        _valid_defs[_param] = _spec
      except UnknownKeyName:
        print(f"toggles: param {_param!r} not registered in params_keys.h, hiding toggle (params/UI mismatch)")
    self._toggle_defs = _valid_defs

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

      # insert longitudinal personality after NDOG toggle
      if param == "DisengageOnAccelerator":
        self._toggles["LongitudinalPersonality"] = self._long_personality_setting

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

    # TODO: make a param control list item so we don't need to manage internal state as much here
    # refresh toggles from params to mirror external changes
    for param in self._toggle_defs:
      self._toggles[param].action_item.set_state(self._params.get_bool(param))

    # these toggles need restart, block while engaged
    for toggle_def in self._toggle_defs:
      if self._toggle_defs[toggle_def][3] and toggle_def not in self._locked_toggles:
        self._toggles[toggle_def].action_item.set_enabled(not ui_state.engaged)

    # 3pnwtest: apply the per-feature overrides AFTER the generic refresh/engaged loops so they win
    # (Gemini-reviewed: otherwise the refresh loop clobbers a forced state). ces2xnor: CES replaces the
    # Experimental Mode toggle; CES + the longitudinal personality only apply when openpilot controls
    # longitudinal — grey out (and force off) otherwise (symmetric enable/disable).
    ces_long_ok = ui_state.CP is not None and ui_state.has_longitudinal_control
    self._long_personality_setting.action_item.set_enabled(ces_long_ok)
    if "ConditionalExperimentalSwitching" in self._toggles:  # guarded: toggle is hidden if its param is unregistered
      self._toggles["ConditionalExperimentalSwitching"].action_item.set_enabled(ces_long_ok)
      if not ces_long_ok:
        self._toggles["ConditionalExperimentalSwitching"].action_item.set_state(False)

    # mapd2pnw: "Get map for this location" is greyed out (inactive) when the current GPS is already
    # covered by a downloaded map, or when there's no fix / unknown region (MapForLocationCovered is
    # written True by mapd_manager in both cases). It enables (still off) only when we're somewhere
    # uncovered, so the driver can choose to download the region they're in.
    if "GetMapForLocation" in self._toggles:
      covered = self._params.get_bool("MapForLocationCovered")
      self._toggles["GetMapForLocation"].action_item.set_enabled(not covered)

    # auto2pnw: Nudgeless Lane Change applies to Tesla + the F-150 Lightning only — grey out (and force
    # off) on any other car. No Disengage on Braking is unsupported here on every car — always greyed off.
    cp = ui_state.CP
    nudgeless_ok = cp is not None and (cp.brand == "tesla" or cp.carFingerprint == "FORD_F_150_LIGHTNING_MK1")
    if "NudgelessLaneChange" in self._toggles:
      self._toggles["NudgelessLaneChange"].action_item.set_enabled(nudgeless_ok)
      if not nudgeless_ok:
        self._toggles["NudgelessLaneChange"].action_item.set_state(False)
    if "NoDisengageOnBrake" in self._toggles:
      self._toggles["NoDisengageOnBrake"].action_item.set_enabled(False)
      self._toggles["NoDisengageOnBrake"].action_item.set_state(False)

  def _render(self, rect):
    self._scroller.render(rect)

  def _toggle_callback(self, state: bool, param: str):
    # ces2xnor: ExperimentalMode toggle removed (replaced by CES). CES is a plain bool toggle —
    # no confirm dialog, no icon swap. Full Experimental is reachable via the top-right button.
    self._params.put_bool(param, state, block=True)
    if self._toggle_defs[param][3]:
      self._params.put_bool("OnroadCycleRequested", True, block=True)

  def _set_longitudinal_personality(self, button_index: int):
    self._params.put("LongitudinalPersonality", button_index, block=True)
