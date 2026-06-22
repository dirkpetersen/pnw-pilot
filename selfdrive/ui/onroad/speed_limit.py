"""
mapd2xnor: speed-limit display + lower-limit warning.

Self-contained raylib widget that reads `mapdOut` (published by the official pfeiferj
mapd v2.0.6 binary) DIRECTLY — it does NOT depend on the sunnypilot longitudinalPlanSP /
speed-limit resolver / assist control layer (xnor is pure commaai and has none of that).

Behavior (per user spec):
  - Normal: show the current OSM speed limit as a sign (Vienna if metric,
    MUTCD if imperial). Grey when no/invalid limit.
  - When the limit DROPS to a lower value (either the current limit falls below
    the previously shown one, or a lower `speedLimitAhead` is within range and
    becomes active), flash a large RED warning banner showing the NEW lower
    limit for a few seconds.

Adapted from sunnypilot's selfdrive/ui/sunnypilot/onroad/speed_limit.py, trimmed
to the mapdOut-only data path.
"""
import time
import pyray as rl

from openpilot.common.constants import CV
from openpilot.selfdrive.ui.onroad.hud_renderer import UI_CONFIG
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

WARNING_DURATION = 6.0     # s to keep the red "lower limit" banner up
WARNING_BLINK_PERIOD = 0.7 # s for one on+off blink cycle of the banner (~1.4 Hz)
AHEAD_ACTIVE_DIST = 150.0  # m: treat a lower "ahead" limit as imminent within this distance
MIN_VALID_KPH = 1.0        # below this (in m/s-derived kph) the limit is treated as unknown
OVERSPEED_RATIO = 1.30     # only warn if current speed is >30% above the new lower limit
STALE_AFTER_S = 10.0       # s: if the limit was UNKNOWN longer than this (mapd coverage gap, GPS
                           #   loss, stalled stream), the remembered limit is stale — re-acquiring
                           #   a limit is a fresh fix, NOT a "drop", so no warning. Short flickers
                           #   at a genuine road transition (< this) still warn normally.


class _Colors:
  WHITE = rl.WHITE
  BLACK = rl.BLACK
  RED = rl.Color(235, 32, 32, 255)
  RED_BG = rl.Color(200, 24, 24, 235)
  GREY = rl.Color(145, 155, 149, 255)
  DARK_GREY = rl.Color(77, 77, 77, 255)


class SpeedLimitRenderer(Widget):
  def __init__(self):
    super().__init__()
    self.speed_limit = 0.0          # current limit, display units (kph or mph)
    self.speed_limit_valid = False
    self.speed_limit_ahead = 0.0    # next limit, display units
    self.speed_limit_ahead_valid = False
    self.speed_limit_ahead_dist = 0.0
    self.road_name = ""
    self.speed = 0.0                # current vehicle speed, display units
    self._v_ego_cluster_seen = False

    self._shown_limit = 0.0         # last limit we actually displayed (for drop detection)
    self._shown_limit_t = 0.0       # monotonic stamp of the last VALID limit (staleness gate)
    self._warn_value = 0.0          # the lower value to show in the warning banner
    self._warn_until = 0.0          # monotonic deadline for the warning banner

    self.font_bold = gui_app.font(FontWeight.BOLD)
    self.font_demi = gui_app.font(FontWeight.SEMI_BOLD)
    self.font_norm = gui_app.font(FontWeight.NORMAL)

  @property
  def _conv(self) -> float:
    return CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH

  def _update_state(self):
    sm = ui_state.sm
    if sm.recv_frame["carState"] < ui_state.started_frame:
      return

    # current vehicle speed in display units (prefer cluster speed once seen)
    car_state = sm["carState"]
    self._v_ego_cluster_seen = self._v_ego_cluster_seen or car_state.vEgoCluster != 0.0
    v_ego = car_state.vEgoCluster if self._v_ego_cluster_seen else car_state.vEgo
    self.speed = max(0.0, v_ego * self._conv)

    # mapd2pnw: read the official mapd output (mapdOut). speedLimit/nextSpeedLimit are 0 when
    # unknown, so validity is just "> 0" (the renderer already gates on MIN_VALID_KPH too).
    if sm.updated["mapdOut"]:
      mo = sm["mapdOut"]
      conv = self._conv
      new_limit = mo.speedLimit * conv
      self.speed_limit_valid = mo.speedLimit > 0.0
      self.speed_limit_ahead_valid = mo.nextSpeedLimit > 0.0
      self.speed_limit_ahead = mo.nextSpeedLimit * conv
      self.speed_limit_ahead_dist = mo.nextSpeedLimitDistance
      self.road_name = mo.roadName

      self._maybe_trigger_warning(new_limit)
      self.speed_limit = new_limit

  def _overspeeding(self, limit: float) -> bool:
    """True if current speed is more than OVERSPEED_RATIO above the given limit."""
    return limit > MIN_VALID_KPH and self.speed > limit * OVERSPEED_RATIO

  def _maybe_trigger_warning(self, new_limit: float) -> None:
    """Flash a red banner when the limit drops to a lower value AND the driver is
    currently more than 20% above that new lower limit."""
    now = time.monotonic()

    # Staleness gate: if the limit has been UNKNOWN for a while (mapd coverage gap,
    # GPS loss), the remembered _shown_limit says nothing about the road we are on
    # NOW — re-acquiring a limit after such a gap is a fresh fix, not a "drop from
    # higher to lower", so it must not warn. Only a recent valid limit can be the
    # baseline for drop detection.
    baseline_fresh = (self._shown_limit > MIN_VALID_KPH
                      and (now - self._shown_limit_t) <= STALE_AFTER_S)

    # Case 1: the current limit itself just dropped below what we were showing.
    if (self.speed_limit_valid and new_limit > MIN_VALID_KPH
        and baseline_fresh
        and round(new_limit) < round(self._shown_limit)
        and self._overspeeding(new_limit)):
      self._warn_value = new_limit
      self._warn_until = now + WARNING_DURATION

    # Case 2: a lower "ahead" limit is imminent (within range). Both limits here are
    # CURRENT mapd data (not a remembered baseline), so no staleness gate is needed.
    elif (self.speed_limit_ahead_valid and self.speed_limit_ahead > MIN_VALID_KPH
          and self.speed_limit_valid and new_limit > MIN_VALID_KPH
          and round(self.speed_limit_ahead) < round(new_limit)
          and 0 < self.speed_limit_ahead_dist <= AHEAD_ACTIVE_DIST
          and self._overspeeding(self.speed_limit_ahead)):
      # Only (re)arm if not already warning about this same value
      if round(self._warn_value) != round(self.speed_limit_ahead) or now > self._warn_until:
        self._warn_value = self.speed_limit_ahead
        self._warn_until = now + WARNING_DURATION

    if self.speed_limit_valid and new_limit > MIN_VALID_KPH:
      self._shown_limit = new_limit
      self._shown_limit_t = now

  def _render(self, rect: rl.Rectangle):
    if not ui_state.show_speed_limit:
      return

    width = UI_CONFIG.set_speed_width_metric if ui_state.is_metric else UI_CONFIG.set_speed_width_imperial
    x = rect.x + 60 + width + 30 - 6
    y = rect.y + 45 - 6
    sign_rect = rl.Rectangle(x, y, width, UI_CONFIG.set_speed_height + 6 * 2)

    self._draw_sign(sign_rect)

    # Show the warning only while BOTH hold: the 6 s window is still open AND we are
    # still actually >OVERSPEED_RATIO over the warned limit. The second check cancels
    # the blink the instant the driver slows back down, so the banner doesn't keep
    # flashing for the rest of the window after you've already obeyed it.
    if time.monotonic() < self._warn_until and self._overspeeding(self._warn_value):
      self._draw_warning(rect)

  # ---- normal sign -------------------------------------------------------
  def _draw_sign(self, rect):
    has_limit = self.speed_limit_valid and self.speed_limit > MIN_VALID_KPH
    limit_str = str(round(self.speed_limit)) if has_limit else "--"
    color = _Colors.BLACK if has_limit else _Colors.GREY

    if ui_state.is_metric:
      self._render_vienna(rect, limit_str, color)
    else:
      self._render_mutcd(rect, limit_str, color)

  @staticmethod
  def _text_centered(font, text, size, cx, cy, color):
    sz = measure_text_cached(font, text, size)
    rl.draw_text_ex(font, text, rl.Vector2(cx - sz.x / 2, cy - sz.y / 2), size, 0, color)

  def _render_vienna(self, rect, val, color):
    center = rl.Vector2(rect.x + rect.width / 2, rect.y + rect.height / 2)
    radius = (rect.width + 18) / 2
    rl.draw_circle_v(center, radius, _Colors.WHITE)
    rl.draw_ring(center, radius * 0.75, radius, 0, 360, 36, _Colors.RED)
    font_size = 70 if len(val) >= 3 else 85
    self._text_centered(self.font_bold, val, font_size, center.x, center.y, color)

  def _render_mutcd(self, rect, val, color):
    rl.draw_rectangle_rounded(rect, 0.35, 10, _Colors.WHITE)
    inner = rl.Rectangle(rect.x + 10, rect.y + 10, rect.width - 20, rect.height - 20)
    outer_radius = 0.35 * rect.width / 2.0
    inner_radius = outer_radius - 10.0
    inner_roundness = inner_radius / (inner.width / 2.0)
    rl.draw_rectangle_rounded_lines_ex(inner, inner_roundness, 10, 4, _Colors.BLACK)
    self._text_centered(self.font_demi, "SPEED", 40, rect.x + rect.width / 2, rect.y + 40, _Colors.BLACK)
    self._text_centered(self.font_demi, "LIMIT", 40, rect.x + rect.width / 2, rect.y + 80, _Colors.BLACK)
    self._text_centered(self.font_bold, val, 90, rect.x + rect.width / 2, rect.y + 150, color)

  # ---- big red lower-limit warning --------------------------------------
  def _draw_warning(self, rect):
    # Blink: the banner stays armed for WARNING_DURATION, so gate the whole draw
    # on a square wave to flash it on/off (~1.4 Hz). Skipping the draw on the
    # "off" half leaves the camera view visible underneath, producing the blink.
    if (time.monotonic() % WARNING_BLINK_PERIOD) >= WARNING_BLINK_PERIOD / 2:
      return

    units = "km/h" if ui_state.is_metric else "mph"
    new_val = str(round(self._warn_value))

    # 2x the original size (was 720x260). Fits the 2160-wide tici/tizi screen.
    banner_w = 1440
    banner_h = 520
    bx = rect.x + (rect.width - banner_w) / 2
    by = rect.y + UI_CONFIG.header_height + 60
    banner = rl.Rectangle(bx, by, banner_w, banner_h)

    rl.draw_rectangle_rounded(banner, 0.12, 10, _Colors.RED_BG)
    rl.draw_rectangle_rounded_lines_ex(banner, 0.12, 10, 12, _Colors.WHITE)

    cx = bx + banner_w / 2
    self._text_centered(self.font_demi, "REDUCED SPEED LIMIT", 108, cx, by + 110, _Colors.WHITE)
    self._text_centered(self.font_bold, new_val, 260, cx, by + 300, _Colors.WHITE)
    self._text_centered(self.font_norm, units, 88, cx, by + 444, _Colors.WHITE)
