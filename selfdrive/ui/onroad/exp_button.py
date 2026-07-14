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
# icbm2pnw: without openpilot longitudinal (e.g. the F-150 Lightning on stock ACC + ICBM), forced
# Experimental is impossible — the planner never owns longitudinal. The button then flips only
# CES(ICBM) <-> Chill; the orange forced-Exp state is unreachable and its icon never shows.
_CES_CYCLE_NO_LONG = (_BTN_CES, _BTN_CHILL)
_PARAM_POLL_S = 0.5   # uicpu2pnw (T1): re-read the CES settings/tap params at 2 Hz, not 60 Hz


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
