import time
from enum import Enum, auto
import pyray as rl
from openpilot.common.params import Params
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.label import UnifiedLabel

# ces2xnor button states
_BTN_CES, _BTN_CHILL, _BTN_EXP = 0, 1, 2
# tap cycle order (per spec): CES auto (white exp) -> forced Experimental (orange exp)
#   -> forced Chill (white wheel) -> back to CES auto
_CES_CYCLE = (_BTN_CES, _BTN_EXP, _BTN_CHILL)
# oplongexp2pnw (Fable review, HIGH-1 fix): on an alpha-capable car with op-long OFF (the Lightning on
# stock ACC), Chill is the driver's ICBM kill switch (ces_pnw.py only runs the ICBM executor outside
# forced Chill) and MUST stay reachable in ONE tap from the default boot state (CES -- CESButtonState
# is CLEAR_ON_MANAGER_START, so every drive starts here). Reusing _CES_CYCLE's (CES, EXP, CHILL) order
# put Chill two taps away and made it UNREACHABLE in practice (every tap from CES lands on the Exp
# slot, which this module always forces back to CES -- see _handle_mouse_release -- so the cycle index
# never advances past it). CES -> Chill first, Exp (the enable reach) last.
_CES_CYCLE_OFF_ALPHA = (_BTN_CES, _BTN_CHILL, _BTN_EXP)
# icbm2pnw: without ANY openpilot-longitudinal capability at all (neither native nor alpha-available
# -- there is no car in the pnw fleet today that lands here, but the capability view must still
# degrade safely), forced Experimental is structurally impossible -- the planner never owns
# longitudinal on that car and there's no enable path to reach for either. The button then flips
# only CES(ICBM) <-> Chill; the orange forced-Exp state is unreachable and its icon never shows.
_CES_CYCLE_NO_LONG = (_BTN_CES, _BTN_CHILL)
_PARAM_POLL_S = 0.5   # uicpu2pnw (T1): re-read the CES settings/tap params at 2 Hz, not 60 Hz


def select_ces_cycle(has_longitudinal_control: bool, alpha_long_available: bool, op_long_native: bool) -> tuple:
  """Pure decision: which tap-cycle order the top-right button uses. Capability-view only -- never
  branches on carFingerprint/brand.

  - op-long ON (has_longitudinal_control): full cycle, unchanged -- includes real Experimental.
  - op-long OFF but this car COULD run it (alpha_long_available and not op_long_native -- e.g. the
    Lightning on stock ACC today): _CES_CYCLE_OFF_ALPHA (CES -> Chill -> Exp). Landing on the
    Experimental slot does not select real Experimental mode (that's inert without op-long) -- see
    is_exp_slot_reach / _handle_mouse_release, which turn it into the op-long ENABLE-CONFIRM flow
    instead. Chill comes BEFORE Exp in this ordering (Fable HIGH-1) so the ICBM kill switch is always
    one tap away from the default boot state, and the enable reach is the deliberate later position.
  - genuinely no-op-long car (neither native nor alpha-available): CES<->Chill only -- there is no
    enable path to reach for, so Experimental must stay structurally unreachable (icbm2pnw).
  """
  if has_longitudinal_control:
    return _CES_CYCLE
  if alpha_long_available and not op_long_native:
    return _CES_CYCLE_OFF_ALPHA
  return _CES_CYCLE_NO_LONG


def is_exp_slot_reach(nxt: int, has_longitudinal_control: bool, alpha_long_available: bool,
                       op_long_native: bool) -> bool:
  """Pure decision: does a tap that computed `nxt` as the next cycle position represent the driver
  reaching for Experimental on an op-long-OFF, alpha-capable car (e.g. the Lightning on stock ACC)?
  True only for nxt == _BTN_EXP under exactly that capability combination -- op-long already ON, or a
  car with no op-long capability at all, or a tap that landed on CES/Chill, are all False (an ordinary
  cycle step). The caller (_handle_mouse_release) treats True as "ARM the enable-confirm window", not
  an immediate enable -- see ConfirmOutcome / decide_confirm_outcome for the deliberate second gesture.
  """
  return nxt == _BTN_EXP and alpha_long_available and not op_long_native and not has_longitudinal_control


class ConfirmOutcome(Enum):
  """oplongexp2pnw: pure classification of the SECOND, confirming tap (the one that lands while the
  "tap again to enable" hint from is_exp_slot_reach is still armed) -- see decide_confirm_outcome."""
  ENABLE = auto()        # stopped and not engaged -> safe to enable op-long now
  HOLD_MOVING = auto()   # still moving -> reloading the onroad stack now would be dangerous
  HOLD_ENGAGED = auto()  # stopped but still engaged -> reload would disengage the drive


def decide_confirm_outcome(v_ego: float, engaged: bool) -> ConfirmOutcome:
  """Pure decision (Fable review, HIGH-2 fix): whether the confirming tap may actually enable op-long.

  The original gate was `not engaged` alone, which is wrong: `engaged` only means openpilot currently
  has active control (started AND selfdriveState.enabled) -- a driver cruising on stock ACC/manually
  with openpilot DISENGAGED is still moving, and OnroadCycleRequested restarts the entire onroad stack
  (modeld/controlsd/card/...), which would happen AT SPEED. The real requirement is STANDSTILL: v_ego
  below _STANDSTILL_MS. `engaged` is still checked (stopped-but-engaged, e.g. held at a light under
  openpilot control, still can't reload without disengaging) but only as a second condition once
  standstill is confirmed -- the two failure modes get distinct hints (STOP vs DISENGAGE, Fable F7).
  """
  if v_ego >= _STANDSTILL_MS:
    return ConfirmOutcome.HOLD_MOVING
  if engaged:
    return ConfirmOutcome.HOLD_ENGAGED
  return ConfirmOutcome.ENABLE


_STANDSTILL_MS = 0.1     # m/s -- "stopped enough" to safely reload the whole onroad stack
_CONFIRM_WINDOW_S = 3.0  # oplongexp2pnw (Fable HIGH-3): deliberate two-tap confirm window
# oplongexp2pnw (Fable HIGH-4/F4): once an enable has been requested, swallow taps until
# has_longitudinal_control has had a chance to catch up (ui_state.update_params polls at ~5 s
# cadence -- see update() in ui_state.py) so a stray tap mid-reload can't re-arm/re-cycle against a
# stale capability read and (mis-)write CESButtonState=Exp before the car has actually come up on
# op-long. Deliberately longer than the 5 s param-poll interval.
_ENABLE_GUARD_S = 6.0

# oplongexp2pnw: transient on-screen messages for the ARM / ENABLE / HOLD_MOVING / HOLD_ENGAGED
# moments above. Reuses the oplongui2pnw transient-overlay rendering (self-drawn pyray box below the
# button) -- only the text and trigger conditions changed. Plain ASCII only ("..." not the U+2026
# ellipsis glyph, no "▶"/"▸" arrows) -- selfdrive/assets/fonts/process.py EXTRA_CHARS does not bake
# either non-ASCII glyph into the on-device font atlas, so anything outside chr(32..127)|EXTRA_CHARS
# renders as notdef/tofu. Fable F3/F7: the confirm hint names AEB explicitly, and the "can't enable"
# hint distinguishes MOVING (stop) from STOPPED-BUT-ENGAGED (disengage) instead of always saying "stop".
_CONFIRM_HINT_TEXT = tr_noop(
  "Tap again within 3 seconds to enable openpilot Longitudinal Control. This disables Automatic Emergency Braking (AEB).")
_ENABLE_HINT_TEXT = tr_noop("Enabling openpilot Longitudinal Control...")
_STOP_HINT_TEXT = tr_noop("Stop to enable openpilot Longitudinal Control")
_DISENGAGE_HINT_TEXT = tr_noop("Disengage to enable openpilot Longitudinal Control")
_HINT_S = 4.0      # seconds the hint stays on screen after a tap
_HINT_WIDTH = 480
_HINT_PAD = 20
_HINT_FONT = 32
_HINT_MARGIN = 20  # gap between the button and the hint box
# Fallback left clamp for the hint box (mirrors hud_renderer.UI_CONFIG.border_size / the onroad
# content rect's left inset -- duplicated as a literal rather than imported, since hud_renderer.py
# already imports ExpButton and importing back would cycle).
_HINT_MIN_X = 30
_HINT_BG = rl.Color(0, 0, 0, 200)
_HINT_TEXT_COLOR = rl.Color(255, 255, 255, 235)


class ExpButton(Widget):
  def __init__(self, button_size: int, icon_size: int):
    super().__init__()
    self._params = Params()
    self._experimental_mode: bool = False   # EFFECTIVE mode (selfdrived publishes manual OR CES)
    self._engageable: bool = False

    # ces2xnor state
    self._ces_master: bool = False          # ConditionalExperimentalSwitching
    self._ces_button: int = _BTN_CES        # CESButtonState (0=CES,1=Chill,2=Exp)
    self._param_poll_t: float = 0.0          # uicpu2pnw (T1): last 2 Hz param poll (monotonic)
    self._manual_exp: bool = False           # the ExperimentalMode settings param

    # State hold mechanism (stock 2-state path only)
    self._hold_duration = 2.0  # seconds
    self._held_mode: bool | None = None
    self._hold_end_time: float | None = None

    self._white_color: rl.Color = rl.Color(255, 255, 255, 255)
    self._black_bg: rl.Color = rl.Color(0, 0, 0, 166)
    self._txt_wheel: rl.Texture = gui_app.texture('icons/chffr_wheel.png', icon_size, icon_size)
    self._txt_exp: rl.Texture = gui_app.texture('icons/experimental.png', icon_size, icon_size)        # baked ORANGE
    self._txt_exp_white: rl.Texture = gui_app.texture('icons/experimental_white.png', icon_size, icon_size)  # ces2xnor
    self._txt_exp_yellow: rl.Texture = gui_app.texture('icons/experimental_yellow.png', icon_size, icon_size)  # icbm2pnw: CES-auto-Experimental (driver req 2026-07-11 — orange is forced-Exp ONLY)
    self._rect = rl.Rectangle(0, 0, button_size, button_size)

    # oplongexp2pnw: transient hint (repurposed from oplongui2pnw's informational-only box) -- text
    # is picked per-tap by _handle_mouse_release and held in self._hint_text.
    self._hint_until: float = 0.0
    self._hint_text: str = _ENABLE_HINT_TEXT
    # oplongexp2pnw (Fable HIGH-3/F4): the two-tap confirm gesture + post-enable tap guard. Both are
    # monotonic deadlines, 0.0 (default, always in the past) means "not armed/pending".
    self._confirm_armed_until: float = 0.0   # set when a tap reaches the Exp slot (arms the confirm)
    self._enable_pending_until: float = 0.0  # set right after ENABLE fires (swallow taps until then)
    self._hint_label = UnifiedLabel(
      lambda: tr(self._hint_text),
      font_size=_HINT_FONT,
      text_color=_HINT_TEXT_COLOR,
      alignment=rl.GuiTextAlignment.TEXT_ALIGN_CENTER,
      alignment_vertical=rl.GuiTextAlignmentVertical.TEXT_ALIGN_TOP,
      max_width=_HINT_WIDTH - _HINT_PAD * 2,
      wrap_text=True,
      elide=False,
    )

  def set_rect(self, rect: rl.Rectangle) -> None:
    self._rect.x, self._rect.y = rect.x, rect.y

  def _update_state(self) -> None:
    selfdrive_state = ui_state.sm["selfdriveState"]
    self._experimental_mode = selfdrive_state.experimentalMode
    self._engageable = selfdrive_state.engageable or selfdrive_state.enabled
    # uicpu2pnw (T1): ConditionalExperimentalSwitching / ExperimentalMode / CESButtonState are all
    # settings/tap params that change only on a driver action, so reading these three files EVERY
    # frame (60 Hz) is pure IO waste. Poll at ~2 Hz; the tap handler write-through below keeps the
    # button icon instant on tap so the throttle is imperceptible.
    now = time.monotonic()
    if now - self._param_poll_t >= _PARAM_POLL_S:
      self._param_poll_t = now
      # ces2xnor
      self._ces_master = self._params.get_bool("ConditionalExperimentalSwitching")
      self._manual_exp = self._params.get_bool("ExperimentalMode")
      # CESButtonState is an INT-typed param -> get() already returns an int (0=CES,1=Chill,2=Exp).
      self._ces_button = int(self._params.get("CESButtonState", return_default=True) or _BTN_CES) if self._ces_master else _BTN_CES

  def _handle_mouse_release(self, _):
    super()._handle_mouse_release(_)
    if self._ces_master:
      now = time.monotonic()
      # ces2xnor: 3-state cycle CES -> Experimental -> Chill -> CES (no confirm gate when op-long is
      # already ON). icbm2pnw/oplongexp2pnw: which states are reachable, and whether reaching Exp
      # needs a confirm gate at all, depends on this car's op-long capability -- see select_ces_cycle
      # / is_exp_slot_reach below. oplongexp2pnw (Fable F5): op_long_native is read from ui_state (computed
      # once per ~5 s in update_params), NOT constructed fresh here -- PnwVehicle() does file I/O
      # (curve.json/rain.json) on every construction, which a per-tap handler shouldn't pay for.
      op_long_native = ui_state.op_long_native
      alpha_long_available = ui_state.CP is not None and ui_state.CP.alphaLongitudinalAvailable
      has_long = ui_state.has_longitudinal_control
      off_alpha_capable = alpha_long_available and not op_long_native and not has_long

      if self._enable_pending_until > now:
        # oplongexp2pnw (Fable HIGH-4/F4): an enable was just requested and the reload/capability
        # read hasn't caught up yet -- swallow the tap entirely rather than risk a stale-capability
        # cycle step (or a re-arm) racing the reload.
        pass
      elif off_alpha_capable and self._confirm_armed_until > now:
        # oplongexp2pnw (Fable HIGH-3): the CONFIRM tap -- a second, deliberate tap while the "tap
        # again to enable" hint (armed by is_exp_slot_reach below) is still showing. CESButtonState
        # is left untouched here -- it's already sitting wherever the arming tap left it (CES).
        self._confirm_armed_until = 0.0
        outcome = decide_confirm_outcome(ui_state.sm["carState"].vEgo, ui_state.engaged)
        if outcome is ConfirmOutcome.ENABLE:
          # Same param pair developer.py::_on_alpha_long_enabled writes on confirm (minus the dialog
          # and AEB warning text -- this two-tap gesture IS the driver's confirmation).
          self._params.put_bool("AlphaLongitudinalEnabled", True)
          self._params.put_bool("OnroadCycleRequested", True)
          self._enable_pending_until = now + _ENABLE_GUARD_S
          self._hint_text = _ENABLE_HINT_TEXT
        elif outcome is ConfirmOutcome.HOLD_MOVING:
          self._hint_text = _STOP_HINT_TEXT
        else:  # HOLD_ENGAGED: stopped, but openpilot still has control -- disengage first, not "stop"
          self._hint_text = _DISENGAGE_HINT_TEXT
        self._hint_until = now + _HINT_S
      else:
        self._confirm_armed_until = 0.0   # any other tap cancels a stale/expired arm

        cycle = select_ces_cycle(has_long, alpha_long_available, op_long_native)
        cur = int(self._params.get("CESButtonState", return_default=True) or _BTN_CES)
        idx = cycle.index(cur) if cur in cycle else 0
        nxt = cycle[(idx + 1) % len(cycle)]

        # oplongexp2pnw (supersedes oplongui2pnw's informational-only hint, docs/pnw/op-long-features.md
        # §6a): reaching Experimental while op-long is OFF on an alpha-capable car (e.g. the Lightning
        # on stock ACC) is the driver's re-enable REACH -- Experimental is a consequence of op-long, so
        # flipping to it starts the enable flow rather than just pointing at Settings. It does NOT
        # enable on this tap alone (Fable HIGH-3): it ARMS a confirm window and lands on CES (never
        # Exp -- that state is inert without op-long); the enable itself only happens on a second,
        # deliberate tap handled by the branch above.
        if is_exp_slot_reach(nxt, has_long, alpha_long_available, op_long_native):
          nxt = _BTN_CES
          self._confirm_armed_until = now + _CONFIRM_WINDOW_S
          self._hint_text = _CONFIRM_HINT_TEXT
          self._hint_until = now + _HINT_S

        # CESButtonState is INT-typed: put an INT, not str(nxt). PYTHON_2_CPP has no (str, INT)
        # cast, so put(str) raised TypeError and the tap silently did nothing (button never moved).
        self._params.put("CESButtonState", nxt)
        self._ces_button = nxt   # uicpu2pnw (T1): write-through so the icon flips instantly (the
                                 # 2 Hz poll would otherwise lag the tap by up to _PARAM_POLL_S)
    elif self._is_toggle_allowed():
      # stock 2-state toggle
      new_mode = not self._experimental_mode
      self._params.put_bool("ExperimentalMode", new_mode)
      self._manual_exp = new_mode   # uicpu2pnw (T1): write-through so the 2 Hz poll can't lag/snap-back the tap
      self._held_mode = new_mode
      self._hold_end_time = time.monotonic() + self._hold_duration

  def _render(self, rect: rl.Rectangle) -> None:
    center_x = int(self._rect.x + self._rect.width // 2)
    center_y = int(self._rect.y + self._rect.height // 2)

    # The icon COLOR comes from the PNG itself, not the tint: experimental.png is baked orange,
    # experimental_white.png is white, chffr_wheel.png is white. So we always tint white (identity
    # for the colored icon) and only vary alpha. The old bug tinted the colored png white -> no-op,
    # so CES-auto always looked orange. When CES is on the 3-state button fully owns the icon:
    #   CES auto    -> LIVE mode (driver req 2026-07-10): bleached/white experimental while CES is
    #                  resting in temporary chill; ORANGE experimental the moment CES auto-switches
    #                  to Experimental (selfdriveState.experimentalMode is the effective live mode,
    #                  so the icon tracks the actual switching in real time)
    #   forced Exp  -> orange experimental  (experimental.png)
    #   forced Chill-> white steering wheel
    if self._ces_master:
      if self._ces_button == _BTN_CHILL:
        texture = self._txt_wheel
      elif self._ces_button == _BTN_EXP and ui_state.has_longitudinal_control:
        texture = self._txt_exp
      else:  # _BTN_CES (or Exp with no op-long, defensive) — dynamic: YELLOW when CES has actually
             # switched us to Experimental (driver req 2026-07-11: orange is reserved for FORCED Exp
             # so the button state is never ambiguous), bleached white while CES rests in chill.
             # On the Lightning/ICBM (no op-long) this stays bleached — Experimental can't happen.
        texture = self._txt_exp_yellow if (self._experimental_mode and ui_state.has_longitudinal_control) else self._txt_exp_white
    else:
      # stock 2-state path (CES off): wheel <-> (orange) experimental, unchanged.
      texture = self._txt_exp if (self._held_or_actual_mode() or self._manual_exp) else self._txt_wheel

    color = self._white_color
    color.a = 180 if self.is_pressed or not self._engageable else 255

    rl.draw_circle(center_x, center_y, self._rect.width / 2, self._black_bg)
    rl.draw_texture_ex(texture, rl.Vector2(center_x - texture.width / 2, center_y - texture.height / 2), 0.0, 1.0, color)

    if time.monotonic() < self._hint_until:
      self._render_hint()

  def _render_hint(self):
    """oplongexp2pnw: draw the transient hint below the button (self._hint_text picks the message --
    see is_exp_slot_reach / ConfirmOutcome). Repurposed from oplongui2pnw's informational-only box; still self-contained
    (no cereal/selfdriveState round-trip, no new alert-type plumbing) -- matches the existing pattern
    of other onroad overlays (e.g. ces_status.py) that draw directly with pyray.

    The button sits at the TOP-RIGHT of the screen (hud_renderer.py: button_x = content_rect right
    edge - border - button_size), so a box CENTERED on the button overflows the content rect's
    right edge (button right edge + ~half the box width - clipped by AugmentedRoadView's scissor).
    Right-align the box to the button's right edge instead so it grows leftward and stays on
    screen; box_x is then clamped to a safe minimum as a second line of defense.
    """
    content_h = self._hint_label.get_content_height(_HINT_WIDTH - _HINT_PAD * 2)
    box_w = _HINT_WIDTH
    box_h = content_h + _HINT_PAD * 2
    button_right = self._rect.x + self._rect.width
    box_x = max(button_right - box_w, _HINT_MIN_X)
    box_y = self._rect.y + self._rect.height + _HINT_MARGIN

    rl.draw_rectangle_rounded(rl.Rectangle(box_x, box_y, box_w, box_h), 0.15, 8, _HINT_BG)
    self._hint_label.render(rl.Rectangle(box_x + _HINT_PAD, box_y + _HINT_PAD,
                                         box_w - _HINT_PAD * 2, content_h))

  def _held_or_actual_mode(self):
    now = time.monotonic()
    if self._hold_end_time and now < self._hold_end_time:
      return self._held_mode

    if self._hold_end_time and now >= self._hold_end_time:
      self._hold_end_time = self._held_mode = None

    return self._experimental_mode

  def _is_toggle_allowed(self):
    if not self._params.get_bool("ExperimentalModeConfirmed"):
      return False

    # Mirror exp mode toggle using persistent car params
    return ui_state.has_longitudinal_control
