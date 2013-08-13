import logging
import math
import pid

logger = logging.getLogger('compass_yaw_controller')

YAW_RANGE = 100

class CompassYawController(object):
  def __init__(self, cfmonitor):
    self._cfmonitor = cfmonitor
    self._pid = pid.PID(kp=30.0, ki=0.0, kd=100.0, integ_max=100.0,
                        out_min=-YAW_RANGE, out_max=YAW_RANGE)
    self._target_x = None
    self._target_y = None
    self._min_x = self._target_x
    self._max_x = self._target_x
    self._min_y = self._target_y
    self._max_y = self._target_y
    self._auto = False

  def SetAuto(self, auto):
    self._auto = auto

  def _AdjustRawX(self, x):
    if self._max_x == self._min_x:
      return 0.0
    return (x - self._min_x) / (self._max_x - self._min_x) * 2.0 - 1.0

  def _AdjustRawY(self, y):
    if self._max_y == self._min_y:
      return 0.0
    return (y - self._min_y) / (self._max_y - self._min_y) * 2.0 - 1.0

  def Step(self):
    raw_x = float(self._cfmonitor.GetMagX())
    raw_y = float(self._cfmonitor.GetMagY())

    if self._target_x is None:
      if raw_x == 0.0:
        return
      self._target_x = raw_x
      self._min_x = self._target_x
      self._max_x = self._target_x

    if self._target_y is None:
      if raw_y == 0.0:
        return
      self._target_y = raw_y
      self._min_y = self._target_y
      self._max_y = self._target_y

    if raw_x < self._min_x:
      self._min_x = raw_x
    if raw_x > self._max_x:
      self._max_x = raw_x
    if raw_y < self._min_y:
      self._min_y = raw_y
    if raw_y > self._max_y:
      self._max_y = raw_y

    x = self._AdjustRawX(raw_x)
    y = self._AdjustRawY(raw_y)
    target_x = self._AdjustRawX(self._target_x)
    target_y = self._AdjustRawY(self._target_y)

    angle = math.atan2(y, x)
    target_angle = math.atan2(target_y, target_x)

    if self._auto:
      self._pid.SetSetpoint(target_angle)
      yaw = self._pid.Update(angle)
      logger.info('angle: %f  target angle: %f  yaw: %f',
                  angle, target_angle, yaw)
      self._cfmonitor.SetYaw(yaw)

if __name__ == '__main__':
  class TestCf(object):
    def __init__(self):
      self.x = 100
      self.y = 200
    def SetYaw(self, yaw):
      print yaw
    def GetMagX(self):
      return self.x
    def GetMagY(self):
      return self.y

  test_cf = TestCf()
  cy = CompassYawController(test_cf)
  cy.Step()
  test_cf.x = 200
  test_cf.y = 300
  cy.Step()
  test_cf.x = 0
  test_cf.y = 100
  cy.Step()
  test_cf.x = 80
  test_cf.y = 150
  cy.Step()
