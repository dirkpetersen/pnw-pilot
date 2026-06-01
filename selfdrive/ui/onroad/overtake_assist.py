"""
auto2xnor: Overtake Assist — display-only prompt.

When the car is closing on a slower lead AND there is an open adjacent lane on a
side AND that side's blind spot is clear, draw a green arrow toward that side and
"Signal to overtake". openpilot does NOT steer — the driver initiates the lane
change with the turn signal (which the nudgeless feature can then complete).

Tesla only, gated by the OvertakeAssist toggle (default off). Reads everything
from ui_state.sm (modelV2 lane lines, radarState lead, carState speed/blindspot)
— no control-path involvement whatsoever.

Constraint honesty: openpilot only sees the car's REAR blind-spot zone plus the
forward lane model. It cannot see a fast car approaching from farther back in the
target lane, so this is an assist prompt, not a safety guarantee — the driver
remains responsible for checking the lane.
"""
import pyray as rl

from openpilot.common.constants import CV
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

MIN_SPEED = 40 * CV.MPH_TO_MS      # highway only
LANE_PROB_THRESHOLD = 0.5          # adjacent (outer) lane line confidence to call a lane "present"
LEAD_MAX_DIST = 90.0               # m
MIN_CLOSING = 2.0                  # m/s (vEgo - vLead)
MIN_REL_SLOWER = 0.85              # lead < 85% of our speed
HOLD_FRAMES = 10                   # ~0.5 s debounce so the prompt doesn't flicker

GREEN = rl.Color(51, 171, 76, 255)
WHITE = rl.Color(255, 255, 255, 255)
BG = rl.Color(0, 0, 0, 160)


class OvertakeAssistRenderer(Widget):
  def __init__(self):
    super().__init__()
    self.font_bold = gui_app.font(FontWeight.BOLD)
    self._left_frames = 0
    self._right_frames = 0
    self.show_left = False
    self.show_right = False

  def _update_state(self):
    self.show_left = False
    self.show_right = False

    if not (ui_state.overtake_assist and ui_state.started):
      self._left_frames = self._right_frames = 0
      return

    # Tesla only
    if ui_state.CP is None or ui_state.CP.brand != "tesla":
      self._left_frames = self._right_frames = 0
      return

    sm = ui_state.sm
    if sm.recv_frame["carState"] < ui_state.started_frame:
      return

    cs = sm["carState"]
    v_ego = cs.vEgo
    if v_ego < MIN_SPEED:
      self._left_frames = self._right_frames = 0
      return

    # slower lead ahead?
    lead = sm["radarState"].leadOne
    slow_lead = (lead.status and 0 < lead.dRel < LEAD_MAX_DIST
                 and (v_ego - lead.vLead) > MIN_CLOSING
                 and lead.vLead < v_ego * MIN_REL_SLOWER)
    if not slow_lead:
      self._left_frames = self._right_frames = 0
      return

    # adjacent lane present? outer lane lines: [0]=far-left, [3]=far-right
    probs = sm["modelV2"].laneLineProbs
    left_lane = len(probs) >= 4 and probs[0] > LANE_PROB_THRESHOLD
    right_lane = len(probs) >= 4 and probs[3] > LANE_PROB_THRESHOLD

    left_ok = left_lane and not cs.leftBlindspot
    right_ok = right_lane and not cs.rightBlindspot

    # debounce each side independently
    self._left_frames = self._left_frames + 1 if left_ok else 0
    self._right_frames = self._right_frames + 1 if right_ok else 0
    self.show_left = self._left_frames >= HOLD_FRAMES
    self.show_right = self._right_frames >= HOLD_FRAMES

  def _render(self, rect: rl.Rectangle):
    if self.show_left:
      self._draw_prompt(rect, left=True)
    if self.show_right:
      self._draw_prompt(rect, left=False)

  def _draw_prompt(self, rect: rl.Rectangle, left: bool):
    text = "Signal to overtake"
    font_size = 48
    tsz = measure_text_cached(self.font_bold, text, font_size)

    arrow_w = 90
    pad = 30
    gap = 24
    box_w = arrow_w + gap + tsz.x + 2 * pad
    box_h = 120
    cy = rect.y + rect.height * 0.62

    # left prompt sits left-of-center, right prompt right-of-center
    if left:
      bx = rect.x + rect.width * 0.5 - box_w - 40
    else:
      bx = rect.x + rect.width * 0.5 + 40
    box = rl.Rectangle(bx, cy - box_h / 2, box_w, box_h)
    rl.draw_rectangle_rounded(box, 0.3, 16, BG)

    # arrow (triangle) pointing left or right
    ax = bx + pad
    ay = cy
    half = arrow_w / 2
    h2 = 36
    if left:
      tip = rl.Vector2(ax, ay)
      top = rl.Vector2(ax + arrow_w, ay - h2)
      bot = rl.Vector2(ax + arrow_w, ay + h2)
    else:
      tip = rl.Vector2(ax + arrow_w, ay)
      top = rl.Vector2(ax, ay - h2)
      bot = rl.Vector2(ax, ay + h2)
    # raylib triangle winding: counter-clockwise
    if left:
      rl.draw_triangle(tip, bot, top, GREEN)
    else:
      rl.draw_triangle(tip, top, bot, GREEN)
    # thicken the shaft
    shaft_y = int(ay - 14)
    if left:
      rl.draw_rectangle(int(ax + half), shaft_y, int(half + 6), 28, GREEN)
    else:
      rl.draw_rectangle(int(ax - 6), shaft_y, int(half + 6), 28, GREEN)

    # text
    txt_x = ax + arrow_w + gap
    rl.draw_text_ex(self.font_bold, text, rl.Vector2(txt_x, cy - tsz.y / 2), font_size, 0, WHITE)
