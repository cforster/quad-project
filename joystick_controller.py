import logging
import pygame

logger = logging.getLogger('joystick_controller')

class JoystickController(object):
  def __init__(self, cfmonitor):
    self._cfmonitor = cfmonitor
    pygame.display.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() > 0:
      self._joystick = pygame.joystick.Joystick(0)
      self._joystick.init()
      logger.info('Joystick axes=%d buttons=%d',
                  self._joystick.get_numaxes(),
                  self._joystick.get_numbuttons())
    else:
      self._joystick = None

    self._roll_axis = 0
    self._roll_range = 20.0

    self._pitch_axis = 1
    self._pitch_range = 20.0

    self._yaw_axis = 2
    self._yaw_range = 100.0

    self._thrust = 0
    self._initial_thrust_flag = True
    self._thrust_axis = 4
    self._thrust_max_button = 3
    self._thrust_min_button = 0
    self._thrust_max = 44000

    self._switch_button = 1
    self._auto = False

  def Step(self):
    for event in pygame.event.get():
      if event.type == pygame.JOYBUTTONDOWN:
        if event.button == self._thrust_max_button:
          self._thrust_max += 500
        elif event.button == self._thrust_min_button:
          self._thrust_max -= 500
        elif event.button == self._switch_button:
          self._auto = not self._auto
    if self._joystick is not None:
      roll = self._joystick.get_axis(self._roll_axis) * self._roll_range
      pitch = -self._joystick.get_axis(self._pitch_axis) * self._pitch_range
      yaw = self._joystick.get_axis(self._yaw_axis) * self._yaw_range
      thrust = self._joystick.get_axis(self._thrust_axis)
      # trigger axis initializes to 0 not -1
      if thrust == 0.0 and self._initial_thrust_flag:
        thrust = -1.0
      else:
        self._initial_thrust_flag = False
      thrust = (thrust + 1.0) * self._thrust_max / 2.0
      if not self._auto:
        self._cfmonitor.SetRoll(roll)
        self._cfmonitor.SetPitch(pitch)
        self._cfmonitor.SetYaw(yaw)
      self._cfmonitor.SetThrust(int(thrust))

  def GetAuto(self):
    return self._auto

if __name__ == '__main__':
  import time
  class TestCf(object):
    def SetSetpoint(self, roll, pitch, yaw, thrust):
      print roll, pitch, yaw, thrust
  js = JoystickController(TestCf())
  while True:
    js.Step()
    time.sleep(0.5)
    
