import numpy as np
import pyray as rl

from openpilot.common.constants import CV
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.selfdrive.ui.sunnypilot.onroad.chevron_metrics import ChevronMetrics, ChevronOptions
from openpilot.system.ui.lib.text_measure import measure_text_cached

SPEED_FONT_SIZE = 34
UNIT_FONT_SIZE = 18
INFO_FONT_SIZE = 22
INFO_LINE_HEIGHT = 24
MARGIN = 8
BOTTOM_RESERVED = 40  # steering torque bar / fade strip at the bottom of the mici road view

_SHOW_DISTANCE = (ChevronOptions.DISTANCE_ONLY, ChevronOptions.ALL)
_SHOW_SPEED = (ChevronOptions.SPEED_ONLY, ChevronOptions.ALL)
_SHOW_TTC = (ChevronOptions.TTC_ONLY, ChevronOptions.ALL)


class MiciChevronMetrics(ChevronMetrics):
  """Lead metrics laid out for the 536x240 mici screen: lead speed (large) under each lead chevron,
  distance and time gap for the primary lead stacked small in the top-right corner."""

  def _text(self, text: str, x: float, y: float, size: int, alpha: int):
    rl.draw_text_ex(self._font, text, rl.Vector2(x + 2, y + 2), size, 0, rl.Color(0, 0, 0, int(alpha * 0.8)))
    rl.draw_text_ex(self._font, text, rl.Vector2(x, y), size, 0, rl.Color(255, 255, 255, alpha))

  def _draw_lead_speed(self, lead_data, lead_vehicle, v_ego: float, rect: rl.Rectangle):
    if ui_state.chevron_metrics not in _SHOW_SPEED:
      return
    if not lead_vehicle.chevron or len(lead_vehicle.chevron) < 3:
      return

    apex_x, apex_y = lead_vehicle.chevron[1]
    base_y = lead_vehicle.chevron[0][1]

    mult = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
    number = f"{max(0.0, (lead_data.vRel + v_ego) * mult):.0f}"
    unit = " km/h" if ui_state.is_metric else " mph"

    num_w = measure_text_cached(self._font, number, SPEED_FONT_SIZE, 0).x
    unit_w = measure_text_cached(self._font, unit, UNIT_FONT_SIZE, 0).x
    x = float(np.clip(apex_x - (num_w + unit_w) / 2, MARGIN, max(MARGIN, rect.width - num_w - unit_w - MARGIN)))

    y = base_y + 4
    if y + SPEED_FONT_SIZE > rect.height - BOTTOM_RESERVED:
      y = apex_y - 4 - SPEED_FONT_SIZE
    y = max(MARGIN, y)

    alpha = int(255 * self._lead_status_alpha)
    self._text(number, x, y, SPEED_FONT_SIZE, alpha)
    self._text(unit, x + num_w, y + SPEED_FONT_SIZE - UNIT_FONT_SIZE - 3, UNIT_FONT_SIZE, alpha)

  def _draw_corner_info(self, lead_data, v_ego: float, rect: rl.Rectangle):
    lines = []
    if ui_state.chevron_metrics in _SHOW_DISTANCE:
      d = max(0.0, lead_data.dRel)
      lines.append(f"{d:.0f} m" if ui_state.is_metric else f"{d * 3.28084:.0f} ft")
    if ui_state.chevron_metrics in _SHOW_TTC:
      gap = lead_data.dRel / v_ego if (lead_data.dRel > 0 and v_ego > 0) else 0.0
      lines.append(f"{gap:.1f} s" if 0 < gap < 200 else "---")

    alpha = int(200 * self._lead_status_alpha)
    for i, line in enumerate(lines):
      w = measure_text_cached(self._font, line, INFO_FONT_SIZE, 0).x
      self._text(line, rect.width - w - MARGIN, MARGIN + i * INFO_LINE_HEIGHT, INFO_FONT_SIZE, alpha)

  def draw_lead_status(self, sm, radar_state, rect, lead_vehicles):
    lead_one, lead_two = radar_state.leadOne, radar_state.leadTwo
    has_one = bool(lead_one and lead_one.present)
    has_two = bool(lead_two and lead_two.present)

    self.update_alpha(has_one or has_two)
    if not self.should_render():
      return

    v_ego = sm['carState'].vEgo

    if has_one:
      self._draw_lead_speed(lead_one, lead_vehicles[0], v_ego, rect)
    if has_two and (not has_one or abs(lead_one.dRel - lead_two.dRel) > 3.0):
      self._draw_lead_speed(lead_two, lead_vehicles[1], v_ego, rect)

    primary = lead_one if has_one else lead_two
    self._draw_corner_info(primary, v_ego, rect)
