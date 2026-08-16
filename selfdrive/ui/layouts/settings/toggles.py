from cereal import log
from openpilot.common.params import Params, UnknownKeyName
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.list_view import multiple_button_item, toggle_item
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.selfdrive.controls.lib.pnw_vehicle import PnwVehicle

PERSONALITY_TO_INT = log.LongitudinalPersonality.schema.enumerants

# Description constants
try:
  from openpilot.selfdrive.monitoring.dm_config import HIGHWAY_DEFAULT_POSE_S as _DM_HWY_POSE_S, \
      HIGHWAY_DEFAULT_PHONE_S as _DM_HWY_PHONE_S
except Exception:
  _DM_HWY_POSE_S, _DM_HWY_PHONE_S = 30.0, 60.0

# rain2pnw: the help text quotes the SOURCE default magnitudes so it can never drift from the code
# (same discipline as DmMode). The device-local /data/pnw/rain.json can retune the actual values.
try:
  from openpilot.selfdrive.controls.lib.pnw_vehicle import _RAIN_DEFAULTS as _RAIN
  _RAIN_LIGHT_MPH, _RAIN_HEAVY_MPH = _RAIN["light_mph"], _RAIN["heavy_mph"]
except Exception:
  _RAIN_LIGHT_MPH, _RAIN_HEAVY_MPH = 3.0, 5.0

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
  # dm-variable: numbers below are pulled from the dm_config SOURCE constants so the help text can
  # never drift from the code. Only Default and Highway are described (driver directive 2026-07-11).
  "DmMode": tr_noop(
    "How long you may look away before openpilot alerts. Default = standard openpilot monitoring, "
    "everywhere. Highway = only while a highway is detected, monitoring loosens slightly "
    f"(attention {_DM_HWY_POSE_S:.0f} s, phone {_DM_HWY_PHONE_S:.0f} s); on all other roads it stays "
    "at Default. Monitoring is never disabled and your attention is required at all times."
  ),
  # rain2pnw: driver-selected wet-weather curve margin. Numbers pulled from the source defaults above.
  "RainMode": tr_noop(
    "Slow down extra in curves when it's raining. None = no change. Light = about " +
    f"{_RAIN_LIGHT_MPH:.0f} mph slower through curves. Heavy = about {_RAIN_HEAVY_MPH:.0f} mph slower. " +
    "Applies to both cars. Set it when the roads are wet; return it to None when they dry out."
  ),
  # speedadjust2pnw: auto cruise-speed reduction. Only lowers speed, never raises it; needs openpilot
  # longitudinal control (op-long) to act.
  "AutoSpeedReduce": tr_noop(
    "Automatically reduce your cruise speed. Off = no change. Police = when a police report is about " +
    "30 seconds ahead, ease down to the speed limit + 5 mph, then return to your set speed once past. " +
    "Police + Limits = also lower your speed by the same percentage the posted limit drops (and restore " +
    "it when the limit rises). Only reduces speed, never raises it; requires openpilot longitudinal."
  ),
  'RecordFront': tr_noop("Upload data from the driver facing camera and help improve the driver monitoring algorithm."),
  "IsMetric": tr_noop("Display speed in km/h instead of mph."),
  "RecordAudio": tr_noop("Record and store microphone audio while driving. The audio will be included in the dashcam video in comma connect."),
  # mapdstate2pnw: repurposed from the old "Get map for this location" on-demand-download toggle.
  # Coverage is now automatic (mapd_configd downloads the state/nation you're currently in as soon as
  # it detects you're uncovered) — this toggle's only job now is to force a re-download of a map that's
  # gone stale, by deleting it so the automatic downloader picks it back up.
  "RefreshLocationMap": tr_noop(
    "Delete the downloaded offline OSM map for the state (or country) you're currently in, and immediately " +
    "re-request it fresh — maps also download automatically as soon as you enter a state with no map data — " +
    "use this only if you suspect the map for your current location is stale or corrupted. Enabled only " +
    "when there is a downloaded map here to refresh; requires a GPS fix, and is a no-op if a download is " +
    "already in progress."
  ),
  # mapd2pnw / toggles-invert2pnw: OSM speed-limit display is ON by default; this is the opt-OUT toggle.
  "NoSpeedLimitDisplay": tr_noop(
    "OpenStreetMap speed limits show on the onroad screen and flash a warning when the limit drops. " +
    "It is ON by default — the first time you enter a state with no map data, openpilot downloads the " +
    "whole state automatically (on any network; the sign shows \"-\" until the download completes). " +
    "Requires a GPS fix to display a limit. Turn this ON to hide the display and warning."
  ),
  "ConditionalExperimentalSwitching": tr_noop(
    "Conditional Experimental Switching (CES): stay in Chill Mode for steady cruising and automatically " +
    "switch to Experimental Mode only for tight curves, low-speed/city driving, stop lights, and when closing " +
    "on a slower lead — then return to Chill. With this on, the top-right button cycles CES / Chill / Experimental " +
    "(orange = forced Experimental). It also slows smoothly for upcoming curves (Vision Turn Speed Control). " +
    "Affects speed/braking only, not steering, and only when openpilot controls longitudinal. NOT a cone/obstacle " +
    "detector and not a substitute for attention — stay ready to brake, especially in construction zones and on curves."
  ),
  "HideCESDebug": tr_noop(
    "Hide the CES debug information box (bottom-right while driving: mode/reason, curve %, VTSC state, " +
    "map status). The box shows by default whenever CES is enabled."
  ),
  "Ces2Core": tr_noop(
    "CES2 decision core (redesign): graded stop-urgency from the model's trajectory endpoint, " +
    "stop-evidence-beats-acceleration precedence at any speed, standstill hold at lights, and " +
    "per-condition debounce. OFF (default) = today's CES decides while CES2 runs shadow-only " +
    "(logged for A/B comparison). ON = CES2 decides. Leave OFF until shadow drives validate it."
  ),
  "CESTurns": tr_noop(
    "CES2 turn-signal condition: below 55 mph, signaling a turn (blinker on with no lane-change " +
    "in progress) switches to Experimental Mode for the maneuver. Only used by the CES2 core " +
    "(shadow or live). Off by default."
  ),
  # auto2pnw / toggles-invert2pnw: nudgeless lane change is ON by default (Tesla + F-150 Lightning);
  # this is the opt-OUT toggle.
  # nudgelesshighway2pnw: the no-touch auto path is now highway-only (driver report: a city-street
  # turn-signal flip triggered an unwanted auto lane change into cross-traffic) — the description
  # below documents that; the steering-wheel nudge itself is unaffected and still works everywhere.
  "NudgeForLaneChange": tr_noop(
    "By default, on a highway/freeway (or above about 45 mph), a lane change starts from the turn " +
    "signal alone, without nudging the steering wheel — hold the blinker for about 0.75 seconds and " +
    "openpilot will change lanes (blocked while the blind spot monitor detects a vehicle). On city " +
    "streets below 45 mph, the turn signal alone does NOT start a lane change — nudge the steering " +
    "wheel to change lanes there, same as making an ordinary turn. Turn this ON to require a " +
    "steering-wheel nudge everywhere, highway included. Keep your hands on the wheel and check your " +
    "surroundings. Tesla and the Ford F-150 Lightning only — other cars always require the nudge."
  ),
  "NoDisengageOnBrake": tr_noop(
    "Keep openpilot engaged when you press the brake pedal instead of disengaging. " +
    "openpilot will resume controlling speed as soon as you release the brake. " +
    "Not currently supported on any car here (Ford or Tesla) — this toggle is disabled."
  ),
  # lanecenter2pnw: Lane Centering is ON by default; this is the opt-OUT toggle. Tuning lives in a
  # hot-reloaded file, not the UI, so the description points there rather than to sliders.
  "DisableLaneCentering": tr_noop(
    "Lane Centering nudges steering toward the middle of the lane when both lane lines are clearly " +
    "visible. It is ON by default, applies a small bounded correction, and fades off if lane lines " +
    "become uncertain, you signal a turn, or a lane change starts. Turn this ON to disable it — a " +
    "quick escape hatch if steering ever feels off. Advanced tuning: /data/pnw/lanecenter_tuning.json."
  ),
  # angleenable / toggles-invert2pnw: EXPERIMENTAL angle-primary lateral is ON by default on the
  # F-150 Lightning; this is the opt-OUT toggle.
  # Intentional two-param bridge: NoFordAngleSteering (here) is the only param the driver sees/writes.
  # Steering/control code (opendbc_repo/opendbc/car/pnw_vehicle.py, a separate repo consumed as a
  # submodule) still reads the older positive-sense FordAngleLateral directly and can't be edited from
  # this branch. system/manager/manager.py re-derives FordAngleLateral = not NoFordAngleSteering on
  # every boot, which is what fixes a stray/stale mirror value showing as a "STEER: STOCK" red mismatch.
  "NoFordAngleSteering": tr_noop(
    "EXPERIMENTAL. On the Ford F-150 Lightning, openpilot steers using an angle-primary strategy by " +
    "default instead of the older curvature-based control. First drive at low speed on a quiet road " +
    "and be ready to take over at any time. Falls back to curvature-based control automatically on " +
    "any internal error, regardless of this toggle. Turn this ON to always use curvature-based " +
    "control instead. Ford F-150 Lightning only — this toggle has no effect on the Tesla."
  ),
  # location2pnw / toggles-invert2pnw: the overlay is ON by default; this is the opt-OUT toggle.
  "DisableLocationServices": tr_noop(
    "By default, the lower-left \"Happening Ahead\" overlay shows while on a freeway: the nearest " +
    "police report (Waze), rest area, and EV fast charger ahead. Display-only — never affects " +
    "steering or speed. Turn this ON to hide it."
  ),
  "EvIncludeLevel2": tr_noop(
    "Also show slow Level 2 (AC) chargers in the EV line, not just DC-fast. Off by default."
  ),
  "DeferHDVideoUpload": tr_noop(
    "Hold back the large HD video files (road/wide camera) from uploading while ON; " +
    "logs (qlog/rlog) and low-res video keep uploading. Held files upload normally once " +
    "this is turned OFF (if the disk cleaner has not removed them). Use on precious WiFi."
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
      # light-ces-gentle: CES is the 3-way "CES Mode" selector (Off/Light/Standard) — a multiple_button_item
      # added below like the personality selector, NOT a bool toggle. (Backed by the INT param CESMode.)
      # ces2pnw (driver req 2026-07-10): Hide CES Debug sits RIGHT BELOW the CES Mode selector — the
      # selector is inserted after OpenpilotEnabledToggle inside the build loop, so this next def
      # lands directly under it. Default OFF = debug box visible.
      "HideCESDebug": (
        lambda: tr("Hide CES Debug Information"),
        DESCRIPTIONS["HideCESDebug"],
        "speed_limit.png",
        False,
      ),
      # ces2core2pnw: CES2 decision core (CES2-STUDY.md). Default OFF = shadow-only A/B logging;
      # sits right under the CES group so the whole CES family reads as one block.
      "Ces2Core": (
        lambda: tr("CES2 Decision Core (Shadow A/B)"),
        DESCRIPTIONS["Ces2Core"],
        "experimental_white.png",
        False,
      ),
      "CESTurns": (
        lambda: tr("CES2 Turn Signal Condition"),
        DESCRIPTIONS["CESTurns"],
        "speed_limit.png",
        False,
      ),
      # auto2pnw / toggles-invert2pnw: nudgeless lane change ON by default (Tesla + F-150 Lightning);
      # opt-OUT toggle. + no-disengage-on-brake (unsupported, greyed)
      "NudgeForLaneChange": (
        lambda: tr("Nudge for Lane Change"),
        DESCRIPTIONS["NudgeForLaneChange"],
        "warning.png",
        False,
      ),
      "NoDisengageOnBrake": (
        lambda: tr("No Disengage on Braking"),
        DESCRIPTIONS["NoDisengageOnBrake"],
        "disengage_on_accelerator.png",
        False,
      ),
      # lanecenter2pnw: opt-OUT toggle for a feature that ships ON by default (param default "0" =
      # not disabled). Same idiom as every other bool toggle here (toggle_item, no restart needed —
      # controlsd re-reads DisableLaneCentering at ~1 Hz, see controlsd.py).
      "DisableLaneCentering": (
        lambda: tr("Disable Lane Centering"),
        DESCRIPTIONS["DisableLaneCentering"],
        "warning.png",
        False,
      ),
      # angleenable / toggles-invert2pnw: Ford angle-primary lateral ON by default (F-150 Lightning
      # only, capability-gated below); opt-OUT toggle.
      "NoFordAngleSteering": (
        lambda: tr("No Ford Angle Steering"),
        DESCRIPTIONS["NoFordAngleSteering"],
        "warning.png",
        True,
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
      # mapd2pnw / toggles-invert2pnw: OSM speed-limit display + lower-limit warning ON by default
      # (gates the OSM map download too); opt-OUT toggle.
      "NoSpeedLimitDisplay": (
        lambda: tr("No Speed Limit Display/Warning"),
        DESCRIPTIONS["NoSpeedLimitDisplay"],
        "speed_limit.png",
        False,
      ),
      # mapdstate2pnw: "Refresh this location map" — deletes the current region's downloaded tiles so
      # mapd_configd's automatic uncovered-state download re-fetches it. Greyed out when there is no
      # map here to refresh (no fix, or the current spot isn't covered yet); enabled when covered.
      "RefreshLocationMap": (
        lambda: tr("Refresh this location map"),
        DESCRIPTIONS["RefreshLocationMap"],
        "speed_limit.png",
        False,
      ),
      # location2pnw / toggles-invert2pnw: "Happening Ahead" overlay ON by default, with the
      # slow-L2-charger sub-option right below it; opt-OUT toggle.
      "DisableLocationServices": (
        lambda: tr("Disable Location Services"),
        DESCRIPTIONS["DisableLocationServices"],
        "speed_limit.png",
        False,
      ),
      "EvIncludeLevel2": (
        lambda: tr("Display slow Level 2 chargers"),
        DESCRIPTIONS["EvIncludeLevel2"],
        "speed_limit.png",
        False,
      ),
      # connect2pnw: hold HD video uploads on precious "unmetered" connections; logs keep flowing
      "DeferHDVideoUpload": (
        lambda: tr("Defer HD Video Upload"),
        DESCRIPTIONS["DeferHDVideoUpload"],
        "network.png",
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

    # light-ces-gentle: CES Mode selector backed by the INT param CESMode (0=Off, 1=Light gentle profile,
    # 2=Standard tune). Replaces the old CES bool toggle. Greyed when openpilot doesn't control longitudinal.
    self._ces_mode_setting = multiple_button_item(
      lambda: tr("CES Mode"),
      lambda: tr(DESCRIPTIONS["ConditionalExperimentalSwitching"]),
      buttons=[lambda: tr("Off"), lambda: tr("Light"), lambda: tr("Standard")],
      button_width=255,
      callback=self._set_ces_mode,
      selected_index=self._params.get("CESMode", return_default=True),
      icon="speed_limit.png"
    )

    # rain2pnw: "Rain slowdown" selector backed by the INT param RainMode (0=None, 1=Light, 2=Heavy).
    # Applies to BOTH cars (same reduction), so it is NOT car-gated. Defensive selected_index read
    # (any failure -> None) so a params/UI mismatch hides gracefully instead of bricking the UI.
    try:
      _rain_selected = int(self._params.get("RainMode", return_default=True) or 0)
    except Exception:
      _rain_selected = 0
    self._rain_mode_setting = multiple_button_item(
      lambda: tr("Rain slowdown"),
      lambda: tr(DESCRIPTIONS["RainMode"]),
      buttons=[lambda: tr("None"), lambda: tr("Light"), lambda: tr("Heavy")],
      button_width=255,
      callback=self._set_rain_mode,
      selected_index=_rain_selected,
      icon="speed_limit.png"
    )

    # speedadjust2pnw: "Auto speed reduce" selector backed by the INT param AutoSpeedReduce
    # (0=Off, 1=Police, 2=Police+Limits). Reduce-only cruise cap; greyed when openpilot doesn't control
    # longitudinal. Defensive selected_index read (any failure -> 0) so a params/UI mismatch hides gracefully.
    try:
      _sa_selected = int(self._params.get("AutoSpeedReduce", return_default=True) or 0)
    except Exception:
      _sa_selected = 0
    self._speedadjust_mode_setting = multiple_button_item(
      lambda: tr("Auto speed reduce"),
      lambda: tr(DESCRIPTIONS["AutoSpeedReduce"]),
      buttons=[lambda: tr("Off"), lambda: tr("Police"), lambda: tr("Police+Limits")],
      button_width=255,
      callback=self._set_speedadjust_mode,
      selected_index=_sa_selected,
      icon="speed_limit.png"
    )

    # dmroad2pnw: Driver Monitoring timeout selector backed by the INT param DmMode (0=Default stock
    # strict, 1=Highway relaxed on freeway/divided-2-lane only, 2=Relaxed everywhere). Inserted after
    # the Always-On DM toggle below. Not longitudinal-gated — always available. ("Default", not "Off" —
    # monitoring is never off; 0 just means no relaxation.)
    # dm-variable UI gate (driver directive 2026-07-11): the Relaxed option is INVISIBLE unless the
    # device-local /data/pnw/dm.json carries relaxed.enabled=true (set only via the `dm` CLI — never
    # from this UI). Without the unlock the selector offers Default/Highway only, and a stale
    # DmMode=2 is clamped back to 0 so display and behavior agree. Read once at UI start (defensive:
    # any read failure -> locked). Unlocking via `dm relaxed --enable` shows the option after the
    # next UI restart / ignition cycle.
    _dm_relaxed_unlocked = False
    try:
      from openpilot.selfdrive.monitoring.dm_config import read_raw_config, relaxed_enabled
      _dm_relaxed_unlocked = relaxed_enabled(read_raw_config())
    except Exception:
      _dm_relaxed_unlocked = False
    _dm_buttons = [lambda: tr("Default"), lambda: tr("Highway")]
    if _dm_relaxed_unlocked:
      _dm_buttons.append(lambda: tr("Relaxed"))
    _dm_selected = int(self._params.get("DmMode", return_default=True) or 0)
    if not _dm_relaxed_unlocked and _dm_selected >= 2:
      _dm_selected = 0
      try:
        self._params.put("DmMode", 0)
      except Exception:
        pass
    self._dm_mode_setting = multiple_button_item(
      lambda: tr("Driver Monitoring"),
      lambda: tr(DESCRIPTIONS["DmMode"]),
      buttons=_dm_buttons,
      button_width=255,
      callback=self._set_dm_mode,
      selected_index=_dm_selected,
      icon="monitoring.png"
    )

    # Resilience (mapd2pnw/3pnwtest): drop any toggle whose param isn't registered in params_keys.h
    # before building the toggles. get_bool() on an unregistered key raises UnknownKeyName; if that
    # escapes here it crashes TogglesLayout init -> the UI crash-loops -> the SDE/DRM display driver
    # panics -> the device warm-reboots. A params/UI version mismatch should hide the toggle, not brick
    # the device. (This exact mismatch happened deploying a branch's params_keys.h that lacked CES keys.)
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

      # light-ces-gentle: insert the CES Mode selector right after the main enable toggle
      if param == "OpenpilotEnabledToggle":
        self._toggles["CESMode"] = self._ces_mode_setting
        # rain2pnw: the Rain slowdown selector sits right under CES Mode (same longitudinal group).
        self._toggles["RainMode"] = self._rain_mode_setting
        # speedadjust2pnw: "Auto speed reduce" selector, same longitudinal group.
        self._toggles["AutoSpeedReduce"] = self._speedadjust_mode_setting

      # insert longitudinal personality after NDOG toggle
      if param == "DisengageOnAccelerator":
        self._toggles["LongitudinalPersonality"] = self._long_personality_setting

      # dmroad2pnw: insert the Driver Monitoring timeout selector right after the Always-On DM toggle
      if param == "AlwaysOnDM":
        self._toggles["DmMode"] = self._dm_mode_setting

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
    # light-ces-gentle/icbm2pnw: the CES Mode selector is usable wherever CES can act — planner via
    # op-long, or ICBM via stock-ACC buttons (capability view, not fingerprint checks — pnw_vehicle).
    # live_op_long: persistent CP lags a session behind the Alpha Long toggle (see pnw_vehicle)
    veh = PnwVehicle(ui_state.CP, live_op_long=ui_state.has_longitudinal_control)
    if "CESMode" in self._toggles:
      self._toggles["CESMode"].action_item.set_enabled(ces_long_ok or veh.ces_shadow)
    # speedadjust2pnw: acts via the longitudinal planner cap, so it only has EFFECT under op-long
    # (no stock-ACC/ICBM path yet). Driver directive 2026-07-19: never grey this selector — the
    # driver wants the switch always operable; on stock ACC the selection is simply inert until a
    # speedadjust ICBM path exists (pending work).
    if "AutoSpeedReduce" in self._toggles:
      self._toggles["AutoSpeedReduce"].action_item.set_enabled(True)

    # mapdstate2pnw: "Refresh this location map" is greyed out (inactive) UNLESS the current GPS fix
    # is already covered by a downloaded map — inverted from the old "Get map for this location"
    # logic, since there's nothing to refresh where nothing has been downloaded yet. MapForLocationCovered
    # is written by system/mapd/mapd_configd.py's `map_here` (has_fix AND tileLoaded — NOT just
    # "no fix", which used to make this button enabled with no fix at all; bug fixed 2026-08-15).
    # It enables only when we're somewhere already covered, so the driver can force a fresh
    # re-download of that region's map.
    if "RefreshLocationMap" in self._toggles:
      covered = self._params.get_bool("MapForLocationCovered")
      self._toggles["RefreshLocationMap"].action_item.set_enabled(covered)

    # auto2pnw / toggles-invert2pnw: Nudge for Lane Change support comes from the capability view —
    # grey out (and force ON, i.e. "nudge required") elsewhere, since nudgeless is impossible there.
    # No Disengage on Braking is unsupported here on every car — always greyed off.
    nudgeless_ok = veh.nudgeless
    if "NudgeForLaneChange" in self._toggles:
      self._toggles["NudgeForLaneChange"].action_item.set_enabled(nudgeless_ok)
      if not nudgeless_ok:
        self._toggles["NudgeForLaneChange"].action_item.set_state(True)
    if "NoDisengageOnBrake" in self._toggles:
      self._toggles["NoDisengageOnBrake"].action_item.set_enabled(False)
      self._toggles["NoDisengageOnBrake"].action_item.set_state(False)

    # angleenable / toggles-invert2pnw: Ford angle-primary lateral is only meaningful on the F-150
    # Lightning (the only car with the matching flashed 4-signal/angle-mode panda safety — capability
    # view, same stock_acc_buttons fingerprint basis icbm2pnw already uses for "this car is the
    # Lightning"). Grey out (and force ON, i.e. "no angle steering" display) everywhere else — DISPLAY
    # ONLY. Fable finding (2026-08-10, shared-device bug): this clamp must NEVER put_bool
    # NoFordAngleSteering. veh is derived from ui_state.CP, the last-fingerprinted car — on this
    # ONE physical device that is swapped between the Tesla and the Lightning, "not capable" simply
    # means "currently plugged into the Tesla." Persisting True here would permanently flip the
    # driver-facing default to "angle OFF" the next time the device is back in the Lightning, with
    # no way for the clamp itself to ever flip it back (it only ever forces True, never False). The
    # single source of truth is the every-boot re-sync in manager.manager_init(), which re-derives
    # the legacy FordAngleLateral mirror from NoFordAngleSteering on every boot and self-heals any
    # stray state. Clearing the inert legacy FordAngleLateral key here is still fine (harmless/inert
    # on a non-capable car, and the manager re-sync overwrites it on the next boot anyway).
    if "NoFordAngleSteering" in self._toggles:
      angle_ok = veh.stock_acc_buttons
      self._toggles["NoFordAngleSteering"].action_item.set_enabled(angle_ok)
      if not angle_ok:
        self._toggles["NoFordAngleSteering"].action_item.set_state(True)
        if self._params.get_bool("FordAngleLateral"):
          self._params.put_bool("FordAngleLateral", False)

  def _render(self, rect):
    self._scroller.render(rect)

  def _toggle_callback(self, state: bool, param: str):
    # ces2xnor: ExperimentalMode toggle removed (replaced by CES). CES is a plain bool toggle —
    # no confirm dialog, no icon swap. Full Experimental is reachable via the top-right button.
    self._params.put_bool(param, state)
    # angleenable / toggles-invert2pnw: NoFordAngleSteering is the driver-facing opt-out toggle, but
    # the actual behavioral gate lives in the opendbc submodule (opendbc_repo/opendbc/car/pnw_vehicle.py,
    # a SEPARATE repo on pnw-opendbc's master-pnw branch) which still reads the LEGACY FordAngleLateral
    # key directly and cannot be edited from this branch. Write-through-mirror it here (inverted) on
    # every change so the toggle keeps working correctly without an opendbc change; see the
    # FordAngleLateral/NoFordAngleSteering comments in params_keys.h. Delete this mirror once
    # pnw-opendbc master-pnw reads NoFordAngleSteering directly.
    if param == "NoFordAngleSteering":
      self._params.put_bool("FordAngleLateral", not state)
    if self._toggle_defs[param][3]:
      self._params.put_bool("OnroadCycleRequested", True)

  def _set_longitudinal_personality(self, button_index: int):
    self._params.put("LongitudinalPersonality", button_index)

  def _set_rain_mode(self, button_index: int):
    # rain2pnw: 0=None, 1=Light, 2=Heavy. Read live by the VTSC/ICBM controllers each ~1 Hz via
    # PnwVehicle.set_rain_tier — no restart needed (a mid-drive change takes effect within ~1 s).
    self._params.put("RainMode", button_index)

  def _set_speedadjust_mode(self, button_index: int):
    # speedadjust2pnw: 0=Off, 1=Police, 2=Police+Limits. Read live by SpeedAdjustController in the
    # longitudinal planner each ~1 Hz — no restart needed (a mid-drive change takes effect within ~1 s).
    self._params.put("AutoSpeedReduce", button_index)

  def _set_dm_mode(self, button_index: int):
    # dmroad2pnw: 0=Off (stock strict), 1=Highway (relaxed on freeway/divided-2-lane), 2=Relaxed (everywhere).
    # Read live by selfdrive/monitoring/helpers.py; no restart needed (picked up ~1 Hz).
    self._params.put("DmMode", button_index)

  def _set_ces_mode(self, button_index: int):
    # light-ces-gentle: CESMode is the source of truth (0=Off, 1=Light, 2=Standard). Mirror the legacy
    # bool ConditionalExperimentalSwitching (== CESMode > 0) so any back-compat reader agrees.
    self._params.put("CESMode", button_index)
    self._params.put_bool("ConditionalExperimentalSwitching", button_index > 0)
