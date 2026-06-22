import time
import pyray as rl
from openpilot.common.params import Params
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import Widget

# ces2xnor button states
_BTN_CES, _BTN_CHILL, _BTN_EXP = 0, 1, 2
# tap cycle order (per spec): CES auto (white exp) -> forced Experimental (orange exp)
#   -> forced Chill (white wheel) -> back to CES auto
_CES_CYCLE = (_BTN_CES, _BTN_EXP, _BTN_CHILL)


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
    self._black_bg: rl.Color = rl.Color(0, 0, 0, 166)
    self._txt_wheel: rl.Texture = gui_app.texture('icons/chffr_wheel.png', icon_size, icon_size)
    self._txt_exp: rl.Texture = gui_app.texture('icons/experimental.png', icon_size, icon_size)        # baked ORANGE
    self._txt_exp_white: rl.Texture = gui_app.texture('icons/experimental_white.png', icon_size, icon_size)  # ces2xnor
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
    # CESButtonState is an INT-typed param -> get() already returns an int (0=CES,1=Chill,2=Exp).
    self._ces_button = int(self._params.get("CESButtonState", return_default=True) or _BTN_CES) if self._ces_master else _BTN_CES

  def _handle_mouse_release(self, _):
    super()._handle_mouse_release(_)
    if self._ces_master:
      # ces2xnor: 3-state cycle  CES -> Experimental -> Chill -> CES  (no confirm gate)
      cur = int(self._params.get("CESButtonState", return_default=True) or _BTN_CES)
      idx = _CES_CYCLE.index(cur) if cur in _CES_CYCLE else 0
      nxt = _CES_CYCLE[(idx + 1) % len(_CES_CYCLE)]
      # CESButtonState is INT-typed: put an INT, not str(nxt). PYTHON_2_CPP has no (str, INT)
      # cast, so put(str) raised TypeError and the tap silently did nothing (button never moved).
      self._params.put("CESButtonState", nxt)
    elif self._is_toggle_allowed():
      # stock 2-state toggle
      new_mode = not self._experimental_mode
      self._params.put_bool("ExperimentalMode", new_mode)
      self._held_mode = new_mode
      self._hold_end_time = time.monotonic() + self._hold_duration

  def _render(self, rect: rl.Rectangle) -> None:
    center_x = int(self._rect.x + self._rect.width // 2)
    center_y = int(self._rect.y + self._rect.height // 2)

    # The icon COLOR comes from the PNG itself, not the tint: experimental.png is baked orange,
    # experimental_white.png is white, chffr_wheel.png is white. So we always tint white (identity
    # for the colored icon) and only vary alpha. The old bug tinted the colored png white -> no-op,
    # so CES-auto always looked orange. When CES is on the 3-state button fully owns the icon:
    #   CES auto    -> white experimental   (experimental_white.png)
    #   forced Exp  -> orange experimental  (experimental.png)
    #   forced Chill-> white steering wheel
    if self._ces_master:
      if self._ces_button == _BTN_CHILL:
        texture = self._txt_wheel
      elif self._ces_button == _BTN_EXP:
        texture = self._txt_exp
      else:  # _BTN_CES
        texture = self._txt_exp_white
    else:
      # stock 2-state path (CES off): wheel <-> (orange) experimental, unchanged.
      texture = self._txt_exp if (self._held_or_actual_mode() or self._manual_exp) else self._txt_wheel

    color = self._white_color
    color.a = 180 if self.is_pressed or not self._engageable else 255

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
