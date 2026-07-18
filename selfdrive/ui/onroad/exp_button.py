import time
from enum import Enum, auto
import pyray as rl
from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.pnw_vehicle import PnwVehicle
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
# icbm2pnw: without ANY openpilot-longitudinal capability at all (neither native nor alpha-available
# -- there is no car in the pnw fleet today that lands here, but the capability view must still
# degrade safely), forced Experimental is structurally impossible -- the planner never owns
# longitudinal on that car and there's no enable path to reach for either. The button then flips
# only CES(ICBM) <-> Chill; the orange forced-Exp state is unreachable and its icon never shows.
_CES_CYCLE_NO_LONG = (_BTN_CES, _BTN_CHILL)
_PARAM_POLL_S = 0.5   # uicpu2pnw (T1): re-read the CES settings/tap params at 2 Hz, not 60 Hz


class ExpTapOutcome(Enum):
  """oplongexp2pnw: pure classification of what a CES-cycle tap should do, given the button that was
  JUST computed to land on (nxt) and this car's op-long capability/live state. Supersedes the
  informational-only oplongui2pnw hint (docs/pnw/op-long-features.md §6, option A) -- the driver
  decided the button should ACT, not just point at Settings."""
  NORMAL = auto()          # ordinary cycle step -- write CESButtonState=nxt as-is, nothing special
  ENABLE_OP_LONG = auto()  # landed on Exp while op-long is OFF but reachable, and not engaged -> enable
  HOLD_ENGAGED = auto()    # landed on Exp while op-long is OFF but reachable, and engaged -> can't enable now


def select_ces_cycle(has_longitudinal_control: bool, alpha_long_available: bool, op_long_native: bool) -> tuple:
  """Pure decision: which tap-cycle order the top-right button uses. Capability-view only -- never
  branches on carFingerprint/brand.

  - op-long ON (has_longitudinal_control): full cycle, unchanged -- includes real Experimental.
  - op-long OFF but this car COULD run it (alpha_long_available and not op_long_native -- e.g. the
    Lightning on stock ACC today): ALSO the full cycle. Landing on the Experimental slot does not
    select real Experimental mode (that's inert without op-long) -- decide_exp_tap_outcome turns it
    into the op-long ENABLE flow instead. Restoring Experimental to the cycle is what lets the driver
    "flip to Experimental to turn op-long on" the way the design intends.
  - genuinely no-op-long car (neither native nor alpha-available): CES<->Chill only -- there is no
    enable path to reach for, so Experimental must stay structurally unreachable (icbm2pnw).
  """
  if has_longitudinal_control:
    return _CES_CYCLE
  if alpha_long_available and not op_long_native:
    return _CES_CYCLE
  return _CES_CYCLE_NO_LONG


def decide_exp_tap_outcome(nxt: int, has_longitudinal_control: bool, alpha_long_available: bool,
                            op_long_native: bool, engaged: bool) -> ExpTapOutcome:
  """Pure decision: what the button should DO for a tap that computed `nxt` as the next cycle
  position, given this car's op-long capability and whether the drive is currently engaged.

  Only ever fires for nxt == _BTN_EXP on an alpha-capable, non-native car with op-long OFF -- the
  "reach for Experimental while on stock ACC" moment that select_ces_cycle's second branch makes
  reachable again. Every other combination (op-long already ON, no op-long capability at all, or a
  tap that landed on CES/Chill) is NORMAL: an ordinary cycle step, no side effects.

  The engaged split is the one safety gate: enabling op-long writes OnroadCycleRequested, which
  reloads the onroad stack -- doing that while the drive is moving/engaged would disengage it, so
  ENABLE_OP_LONG is reserved for `not engaged`; while engaged the tap only explains why (HOLD_ENGAGED)
  and must not enable.
  """
  if nxt == _BTN_EXP and alpha_long_available and not op_long_native and not has_longitudinal_control:
    return ExpTapOutcome.HOLD_ENGAGED if engaged else ExpTapOutcome.ENABLE_OP_LONG
  return ExpTapOutcome.NORMAL


# oplongexp2pnw: transient on-screen messages for the two ExpTapOutcome branches above. Reuses the
# oplongui2pnw transient-overlay rendering (self-drawn pyray box below the button) -- only the text
# and trigger condition changed. Plain ASCII only ("..." not the U+2026 ellipsis glyph, no "▶"/"▸"
# arrows) -- selfdrive/assets/fonts/process.py EXTRA_CHARS does not bake either non-ASCII glyph into
# the on-device font atlas, so anything outside chr(32..127)|EXTRA_CHARS renders as notdef/tofu.
_ENABLE_HINT_TEXT = tr_noop("Enabling openpilot Longitudinal Control...")
_HOLD_HINT_TEXT = tr_noop("Stop to enable openpilot Longitudinal Control")
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
    # is picked per-tap by _handle_mouse_release (see ExpTapOutcome) and held in self._hint_text.
    self._hint_until: float = 0.0
    self._hint_text: str = _ENABLE_HINT_TEXT
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
      # ces2xnor: 3-state cycle CES -> Experimental -> Chill -> CES (no confirm gate). icbm2pnw/
      # oplongexp2pnw: which states are reachable depends on this car's op-long capability -- see
      # select_ces_cycle. One CP/capability read per tap (not per frame) -- cheap, and _PARAM_POLL_S
      # already throttles the other params this handler reads.
      op_long_native = PnwVehicle(ui_state.CP).op_long_native if ui_state.CP is not None else False
      alpha_long_available = ui_state.CP is not None and ui_state.CP.alphaLongitudinalAvailable
      has_long = ui_state.has_longitudinal_control
      cycle = select_ces_cycle(has_long, alpha_long_available, op_long_native)
      cur = int(self._params.get("CESButtonState", return_default=True) or _BTN_CES)
      idx = cycle.index(cur) if cur in cycle else 0
      nxt = cycle[(idx + 1) % len(cycle)]

      # oplongexp2pnw (supersedes oplongui2pnw's informational-only hint, docs/pnw/op-long-features.md
      # §6): reaching Experimental while op-long is OFF on an alpha-capable car (e.g. the Lightning on
      # stock ACC) IS the driver's re-enable gesture -- Experimental is a consequence of op-long, so
      # flipping to it must turn op-long on rather than just point at Settings. decide_exp_tap_outcome
      # classifies the tap; NORMAL (every other case) falls through to the plain cycle write below,
      # byte-identical to the pre-existing behavior.
      outcome = decide_exp_tap_outcome(nxt, has_long, alpha_long_available, op_long_native, ui_state.engaged)
      if outcome is ExpTapOutcome.ENABLE_OP_LONG:
        # Same param pair developer.py::_on_alpha_long_enabled writes on confirm (minus the dialog
        # and AEB warning -- this button flip IS the driver's confirmation). Land the visible button
        # on CES, never Exp: real Experimental stays inert until op-long actually comes up after the
        # reload, and CES is the natural place to be sitting once it does.
        nxt = _BTN_CES
        self._params.put_bool("AlphaLongitudinalEnabled", True)
        self._params.put_bool("OnroadCycleRequested", True)
        self._hint_text = _ENABLE_HINT_TEXT
        self._hint_until = time.monotonic() + _HINT_S
      elif outcome is ExpTapOutcome.HOLD_ENGAGED:
        # The one safety gate: OnroadCycleRequested reloads the onroad stack, which would disengage
        # a moving drive. Explain why and revert the tap to CES instead of landing on the inert Exp
        # slot -- no enable, no param writes.
        nxt = _BTN_CES
        self._hint_text = _HOLD_HINT_TEXT
        self._hint_until = time.monotonic() + _HINT_S

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
    see ExpTapOutcome). Repurposed from oplongui2pnw's informational-only box; still self-contained
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
