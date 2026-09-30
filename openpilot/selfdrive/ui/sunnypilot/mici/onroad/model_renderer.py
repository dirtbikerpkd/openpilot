"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl
from openpilot.selfdrive.ui.ui_state import UIStatus
from openpilot.selfdrive.ui.sunnypilot.onroad.chevron_metrics import ChevronMetrics
from openpilot.selfdrive.ui.sunnypilot.onroad.rainbow_path import RainbowPath

LANE_LINE_COLORS_SP = {
  UIStatus.LAT_ONLY: rl.Color(0, 255, 64, 255),
  UIStatus.LONG_ONLY: rl.Color(0, 255, 64, 255),
}


class ModelRendererSP:
  def __init__(self):
    self.rainbow_path = RainbowPath()
    # sized for the 536x240 mici screen (defaults are for the big UI)
    self.chevron_metrics = ChevronMetrics(font_size=22, line_height=26, margin=8, chevron_scale=1.0)
