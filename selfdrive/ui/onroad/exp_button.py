import time
import pyray as rl
from openpilot.common.params import Params
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import Widget

# ces2xnor button states
_BTN_CES, _BTN_CHILL, _BTN_EXP = 0, 1, 2


class ExpButton(Widget):
  def __init__(self, button_size: int, icon_size: int):
    super().__init__()
    self._params = Params()
    self._experimental_mode: bool = False   # EFFECTIVE mode (selfdrived publishes manual OR CES)
    self._engageable: bool = False

    # ces2xnor state
    self._ces_master: bool = False          # ConditionalExperimentalSwitching
    self._ces_button: int = _BTN_CES        # CESButtonState (0=CES,1=Chill,2=Exp)
    self._manual_exp: bool = False           # the ExperimentalMode settings param

    # State hold mechanism (stock 2-state path only)
    self._hold_duration = 2.0  # seconds
    self._held_mode: bool | None = None
    self._hold_end_time: float | None = None

    self._white_color: rl.Color = rl.Color(255, 255, 255, 255)
    self._orange_color: rl.Color = rl.Color(255, 149, 0, 255)  # ces2xnor: full Experimental
    self._black_bg: rl.Color = rl.Color(0, 0, 0, 166)
    self._txt_wheel: rl.Texture = gui_app.texture('icons/chffr_wheel.png', icon_size, icon_size)
    self._txt_exp: rl.Texture = gui_app.texture('icons/experimental.png', icon_size, icon_size)
    self._rect = rl.Rectangle(0, 0, button_size, button_size)

  def set_rect(self, rect: rl.Rectangle) -> None:
    self._rect.x, self._rect.y = rect.x, rect.y

  def _update_state(self) -> None:
    selfdrive_state = ui_state.sm["selfdriveState"]
    self._experimental_mode = selfdrive_state.experimentalMode
    self._engageable = selfdrive_state.engageable or selfdrive_state.enabled
    # ces2xnor
    self._ces_master = self._params.get_bool("ConditionalExperimentalSwitching")
    self._manual_exp = self._params.get_bool("ExperimentalMode")
    self._ces_button = (self._params.get_int("CESButtonState") or _BTN_CES) if self._ces_master else _BTN_CES

  def _handle_mouse_release(self, _):
    super()._handle_mouse_release(_)
    if self._ces_master:
      # ces2xnor: 3-state cycle  CES -> Chill -> Experimental -> CES  (no confirm gate)
      nxt = ((self._params.get_int("CESButtonState") or _BTN_CES) + 1) % 3
      self._params.put("CESButtonState", str(nxt))
    elif self._is_toggle_allowed():
      # stock 2-state toggle
      new_mode = not self._experimental_mode
      self._params.put_bool("ExperimentalMode", new_mode)
      self._held_mode = new_mode
      self._hold_end_time = time.monotonic() + self._hold_duration

  def _render(self, rect: rl.Rectangle) -> None:
    center_x = int(self._rect.x + self._rect.width // 2)
    center_y = int(self._rect.y + self._rect.height // 2)

    # full Experimental = manual settings toggle OR CES button forced to Experimental.
    full_exp = self._manual_exp or (self._ces_master and self._ces_button == _BTN_EXP)
    if self._ces_master and self._ces_button == _BTN_CHILL:
      show_exp = False                                   # forced Chill -> wheel
    else:
      show_exp = self._held_or_actual_mode() or full_exp

    # color: ORANGE = full Experimental; WHITE = CES-auto experimental or chill
    color = self._orange_color if (show_exp and full_exp) else self._white_color
    color.a = 180 if self.is_pressed or not self._engageable else 255

    texture = self._txt_exp if show_exp else self._txt_wheel
    rl.draw_circle(center_x, center_y, self._rect.width / 2, self._black_bg)
    rl.draw_texture_ex(texture, rl.Vector2(center_x - texture.width / 2, center_y - texture.height / 2), 0.0, 1.0, color)

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
