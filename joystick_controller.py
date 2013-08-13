import joystick
import logging
import pygame
import sys

logger = logging.getLogger('joystick_controller')

class JoystickController(object):
  def __init__(self, cfmonitor):
    self._cfmonitor = cfmonitor
    pygame.display.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() > 0:
      if (sys.platform == 'win32'):
        self._joystick = joystick.Xbox360ControllerWindowsJoystick(0)
      elif (sys.platform == 'linux2'):
        self._joystick = joystick.Xbox360ControllerLinuxJoystick(0)
      else:
        raise Exception('Platform for joystick is not supported');
    else:
      self._joystick = None

    self._roll_range = 20.0
    self._pitch_range = 20.0
    self._yaw_range = 100.0

    self._thrust = 0
    self._thrust_max = 44000

    self._auto = False

  def Step(self):
    for event in pygame.event.get():
      if event.type == pygame.JOYBUTTONDOWN:
        if event.button == self._joystick.getIncreaseMaxThrustButton():
          self._thrust_max += 500
          logger.info('trim thrust_max to %d', self._thrust_max)
        elif event.button == self._joystick.getDecreaseMaxThrustButton():
          self._thrust_max -= 500
          logger.info('trim thrust_max to %d', self._thrust_max)
        elif event.button == self._joystick.getSwitchButton():
          self._auto = not self._auto
        
    if self._joystick is not None:
      roll = self._joystick.getRoll() * self._roll_range
      pitch = self._joystick.getPitch() * self._pitch_range
      yaw = self._joystick.getYaw() * self._yaw_range
      thrust = self._joystick.getThrust() * self._thrust_max
      self._cfmonitor.SetRoll(roll)
      self._cfmonitor.SetPitch(pitch)
      self._cfmonitor.SetYaw(yaw)
      self._cfmonitor.SetThrust(int(thrust))

  def GetAuto(self):
    return self._auto

if __name__ == '__main__':
  import time
  class TestCf(object):
    def SetRoll(self, roll):
      print 'Roll: ' + str(roll)

    def SetPitch(self, pitch):
      print 'Pitch: ' + str(pitch)

    def SetYaw(self, yaw):
      print 'Yaw: ' + str(yaw)

    def SetThrust(self, thrust):
      print 'Thrust: ' + str(thrust)
    
  js = JoystickController(TestCf())
  while True:
    js.Step()
    time.sleep(0.5)
    
