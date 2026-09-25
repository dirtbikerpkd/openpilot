"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import os
import time

import pyray as rl

from openpilot.common.hardware import HARDWARE
from openpilot.common.params import Params
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import Widget

# personal fork: the bouncing element is the custom P@ logo (was the "sunnypilot" text). 3:2 artwork, edges pre-faded to
# transparent so it reads as free-floating on the black screen.
LOGO_PATH = "../../sunnypilot/selfdrive/assets/images/screensaver_pa.png"
LOGO_ASPECT = 480 / 720  # height / width of the artwork


class ScreenSaverSP(Widget):
  def __init__(self, params: Params | None = None):
    super().__init__()
    self.set_rect(rl.Rectangle(0, 0, gui_app.width, gui_app.height))
    self._params = params or Params()
    self._is_mici = HARDWARE.get_device_type() == 'mici' or (HARDWARE.get_device_type() == "pc" and os.getenv("BIG") != "1")

    self.x = 0.0
    self.y = 100.0
    self.vx = 120.0 if self._is_mici else 300.0
    self.vy = 70.0 if self._is_mici else 200.0

    self.logo_width = 210 if self._is_mici else 525
    self.logo_height = round(self.logo_width * LOGO_ASPECT)
    self._start_time = None
    self._dismiss = False
    self._screensaver_timeout = 300

  @property
  def is_active(self) -> bool:
    return self._start_time is not None and not self._dismiss

  @property
  def was_dismissed(self) -> bool:
    return self._dismiss

  def initialize(self):
    self._screensaver_timeout = self._params.get("ScreenSaverTimeout", return_default=True)
    if self._start_time is None:
      self._start_time = time.monotonic()
    self._dismiss = False

  def hide_event(self):
    super().hide_event()
    self._dismiss = False
    self._start_time = None

  def _handle_mouse_release(self, mouse_pos):
    self._dismiss = True
    self._start_time = None
    gui_app.pop_widget()
    return super()._handle_mouse_release(mouse_pos)

  def _update_state(self):
    super()._update_state()

    if self._start_time and time.monotonic() - self._start_time > self._screensaver_timeout:
      self._dismiss = True
      self._start_time = None

    dt = rl.get_frame_time()

    self.x += self.vx * dt
    self.y += self.vy * dt

    if self.x + self.logo_width > self.rect.width:
      self.vx *= -1
      self.x = self.rect.width - self.logo_width
    elif self.x < 0:
      self.vx *= -1
      self.x = 0

    if self.y + self.logo_height > self.rect.height:
      self.vy *= -1
      self.y = self.rect.height - self.logo_height
    elif self.y < 0:
      self.vy *= -1
      self.y = 0

  def _render(self, rect: rl.Rectangle):
    self.set_rect(rect)
    rl.clear_background(rl.BLACK)
    logo = gui_app.texture(LOGO_PATH, self.logo_width, self.logo_height)  # cached after the first call
    rl.draw_texture_v(logo, rl.Vector2(int(self.x), int(self.y)), rl.WHITE)
    return -1
