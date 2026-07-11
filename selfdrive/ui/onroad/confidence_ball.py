"""
ball2pnw: the comma-4 "confidence ball", ported from sunnypilot (base mechanism,
selfdrive/ui/mici/onroad/confidence_ball.py) (right-edge placement per
driver preference, drawn BEHIND the CES debug overlay; BP's GPU shader replaced with
raylib's native radial gradient). Display-only, car-agnostic.

What it shows: the driving model's OWN confidence in the current plan, from
modelV2.meta.disengagePredictions — (1 - max brakeDisengageProb) x (1 - max steerOverrideProb),
smoothed by a FirstOrderFilter. The ball rides a slim right-edge track:

  high (top, teal-green)    model very sure nobody needs to take over
  mid  (amber, > 0.2)       model sees elevated takeover likelihood
  low  (bottom, red)        model expects braking/steering intervention soon

While OVERRIDE the ball goes white; disengaged it parks dim at the bottom. Our fork has no
MADS lat/long-only states, so those sunnypilot branches are dropped. Attribution: concept
comma.ai (comma 4 UI), C3X port sunnypilot, tuning/placement cues alan-polk (BluePilot).
"""
import pyray as rl

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import Widget

BALL_RADIUS = 24
TRACK_MARGIN_X = 14        # gap from the right content edge
TRACK_TOP_PAD = 220        # keep clear of the experimental-mode button (top-right)
TRACK_BOTTOM_PAD = 60      # CES debug text draws OVER the ball (we render behind it)


class ConfidenceBallRenderer(Widget):
  def __init__(self):
    super().__init__()
    self._filter = FirstOrderFilter(-0.5, 0.5, 1 / getattr(gui_app, 'target_fps', 60))

  def _update_state(self):
    if ui_state.status == UIStatus.DISENGAGED:
      self._filter.update(-0.5)
      return
    try:
      dp = ui_state.sm['modelV2'].meta.disengagePredictions
      conf = (1 - max(list(dp.brakeDisengageProbs) or [1.0])) * \
             (1 - max(list(dp.steerOverrideProbs) or [1.0]))
    except Exception:
      conf = 0.0
    self._filter.update(conf)

  def _render(self, rect):
    if not ui_state.started:
      return

    track_x = rect.x + rect.width - TRACK_MARGIN_X - BALL_RADIUS
    track_top = rect.y + TRACK_TOP_PAD
    track_bottom = rect.y + rect.height - TRACK_BOTTOM_PAD
    if track_bottom - track_top < 4 * BALL_RADIUS:
      return  # window too small to draw meaningfully

    # normalize filter [-0.5 .. 1.0] -> [0 (bottom) .. 1 (top)], sunnypilot mapping
    normalized = (self._filter.x - (-0.5)) / (1.0 - (-0.5))
    normalized = max(0.0, min(1.0, normalized))
    cy = track_bottom - normalized * (track_bottom - track_top)

    # confidence zones (sunnypilot colors verbatim)
    if ui_state.status == UIStatus.ENGAGED:
      if self._filter.x > 0.5:
        inner = rl.Color(0, 255, 204, 255)
        outer = rl.Color(0, 255, 38, 255)
      elif self._filter.x > 0.2:
        inner = rl.Color(255, 200, 0, 255)
        outer = rl.Color(255, 115, 0, 255)
      else:
        inner = rl.Color(255, 0, 21, 255)
        outer = rl.Color(255, 0, 89, 255)
    elif ui_state.status == UIStatus.OVERRIDE:
      inner = rl.Color(255, 255, 255, 255)
      outer = rl.Color(82, 82, 82, 255)
    else:
      inner = rl.Color(50, 50, 50, 255)
      outer = rl.Color(13, 13, 13, 255)

    # faint track so the ball position reads at a glance
    rl.draw_rectangle_rounded(
      rl.Rectangle(track_x - 3, track_top - BALL_RADIUS, 6,
                   (track_bottom - track_top) + 2 * BALL_RADIUS),
      1.0, 8, rl.Color(255, 255, 255, 28))

    # native raylib radial gradient (replaces BP's GPU shader / sunnypilot's square+ring hack)
    rl.draw_circle_gradient(int(track_x), int(cy), float(BALL_RADIUS), inner, outer)
