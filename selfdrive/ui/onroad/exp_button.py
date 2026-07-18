import time
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
# icbm2pnw: without openpilot longitudinal (e.g. the F-150 Lightning on stock ACC + ICBM), forced
# Experimental is impossible — the planner never owns longitudinal. The button then flips only
# CES(ICBM) <-> Chill; the orange forced-Exp state is unreachable and its icon never shows.
_CES_CYCLE_NO_LONG = (_BTN_CES, _BTN_CHILL)
_PARAM_POLL_S = 0.5   # uicpu2pnw (T1): re-read the CES settings/tap params at 2 Hz, not 60 Hz

# oplongui2pnw (option A, docs/pnw/op-long-features.md §6): informational-only hint shown right
# after a tap when this car COULD run op-long (alpha-long capable, e.g. the Lightning) but is
# currently on stock ACC, so _CES_CYCLE_NO_LONG silently drops Experimental from the cycle above.
# No enable path, no AlphaLongitudinalEnabled write, no OnroadCycleRequested, no restart.
# "▶" (U+25B6) is used, NOT "▸" (U+25B8) -- selfdrive/assets/fonts/process.py EXTRA_CHARS bakes the
# former into the on-device font atlas but not the latter, which would render as notdef/tofu.
_INFO_HINT_TEXT = tr_noop("Enable openpilot Longitudinal Control (Settings ▶ Developer) to use Experimental mode.")
_INFO_HINT_S = 4.0      # seconds the hint stays on screen after a tap
_INFO_HINT_WIDTH = 480
_INFO_HINT_PAD = 20
_INFO_HINT_FONT = 32
_INFO_HINT_MARGIN = 20  # gap between the button and the hint box
# Fallback left clamp for the hint box (mirrors hud_renderer.UI_CONFIG.border_size / the onroad
# content rect's left inset -- duplicated as a literal rather than imported, since hud_renderer.py
# already imports ExpButton and importing back would cycle).
_INFO_HINT_MIN_X = 30
_INFO_HINT_BG = rl.Color(0, 0, 0, 200)
_INFO_HINT_TEXT_COLOR = rl.Color(255, 255, 255, 235)


def should_show_op_long_hint(ces_master: bool, has_longitudinal_control: bool, alpha_long_available: bool) -> bool:
  """Pure decision logic for the exp_button op-long info hint (oplongui2pnw option A).

  True only for an alpha-op-long-capable car (e.g. Lightning) that is currently NOT running
  longitudinal control (stock ACC) while CES is on -- the state where _CES_CYCLE_NO_LONG silently
  drops Experimental. False for a native op-long car (Tesla: has_longitudinal_control is always
  True there) and false once alpha op-long is turned ON (has_longitudinal_control True).
  Capability-view only: never branches on carFingerprint/brand.
  """
  return ces_master and not has_longitudinal_control and alpha_long_available


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

    # oplongui2pnw: informational-only hint (option A) -- see should_show_op_long_hint above.
    self._info_hint_until: float = 0.0
    self._info_label = UnifiedLabel(
      lambda: tr(_INFO_HINT_TEXT),
      font_size=_INFO_HINT_FONT,
      text_color=_INFO_HINT_TEXT_COLOR,
      alignment=rl.GuiTextAlignment.TEXT_ALIGN_CENTER,
      alignment_vertical=rl.GuiTextAlignmentVertical.TEXT_ALIGN_TOP,
      max_width=_INFO_HINT_WIDTH - _INFO_HINT_PAD * 2,
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
      # ces2xnor: 3-state cycle CES -> Experimental -> Chill -> CES (no confirm gate). icbm2pnw:
      # with no op-long, drop Experimental from the cycle -> CES(ICBM) <-> Chill only.
      cycle = _CES_CYCLE if ui_state.has_longitudinal_control else _CES_CYCLE_NO_LONG
      cur = int(self._params.get("CESButtonState", return_default=True) or _BTN_CES)
      idx = cycle.index(cur) if cur in cycle else 0
      nxt = cycle[(idx + 1) % len(cycle)]
      # CESButtonState is INT-typed: put an INT, not str(nxt). PYTHON_2_CPP has no (str, INT)
      # cast, so put(str) raised TypeError and the tap silently did nothing (button never moved).
      self._params.put("CESButtonState", nxt)
      self._ces_button = nxt   # uicpu2pnw (T1): write-through so the icon flips instantly (the
                               # 2 Hz poll would otherwise lag the tap by up to _PARAM_POLL_S)

      # oplongui2pnw (option A): the driver just tried to change mode on a car that COULD run
      # op-long but currently can't reach Experimental (_CES_CYCLE_NO_LONG above) -- explain why.
      # Purely informational: no CESButtonState=_BTN_EXP, no AlphaLongitudinalEnabled write, no
      # OnroadCycleRequested, no modal. Only fire on the tap that LANDS on CES (nxt == _BTN_CES):
      # that's the tap that lands back on the experimental-looking icon (CES auto renders white/
      # yellow-exp, see _render below) -- the moment the driver thinks they've reached Experimental.
      # The other direction (landing on Chill) is a deliberate wheel-icon pick, not a reach for Exp.
      alpha_long_available = ui_state.CP is not None and ui_state.CP.alphaLongitudinalAvailable
      if nxt == _BTN_CES and should_show_op_long_hint(self._ces_master, ui_state.has_longitudinal_control, alpha_long_available):
        self._info_hint_until = time.monotonic() + _INFO_HINT_S
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

    if time.monotonic() < self._info_hint_until:
      self._render_info_hint()

  def _render_info_hint(self):
    """oplongui2pnw: draw the transient info hint below the button. Self-contained (no cereal/
    selfdriveState round-trip, no new alert-type plumbing) -- matches the existing pattern of
    other onroad overlays (e.g. ces_status.py) that draw directly with pyray.

    The button sits at the TOP-RIGHT of the screen (hud_renderer.py: button_x = content_rect right
    edge - border - button_size), so a box CENTERED on the button overflows the content rect's
    right edge (button right edge + ~half the box width - clipped by AugmentedRoadView's scissor).
    Right-align the box to the button's right edge instead so it grows leftward and stays on
    screen; box_x is then clamped to a safe minimum as a second line of defense.
    """
    content_h = self._info_label.get_content_height(_INFO_HINT_WIDTH - _INFO_HINT_PAD * 2)
    box_w = _INFO_HINT_WIDTH
    box_h = content_h + _INFO_HINT_PAD * 2
    button_right = self._rect.x + self._rect.width
    box_x = max(button_right - box_w, _INFO_HINT_MIN_X)
    box_y = self._rect.y + self._rect.height + _INFO_HINT_MARGIN

    rl.draw_rectangle_rounded(rl.Rectangle(box_x, box_y, box_w, box_h), 0.15, 8, _INFO_HINT_BG)
    self._info_label.render(rl.Rectangle(box_x + _INFO_HINT_PAD, box_y + _INFO_HINT_PAD,
                                         box_w - _INFO_HINT_PAD * 2, content_h))

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
