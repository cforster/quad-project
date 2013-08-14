import logging
import math
import pid

logger = logging.getLogger('compass_yaw_controller')

YAW_RANGE = 100

class CompassYawController(object):
  def __init__(self, cfmonitor):
    self._cfmonitor = cfmonitor
    self._pid = pid.PID(kp=100.0, ki=0.0, kd=0.0, integ_max=100.0,
                        out_min=-YAW_RANGE, out_max=YAW_RANGE)
    self._pid.CreateWindow('yaw')
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

  def SetTarget(self):
    self._target_x = float(self._cfmonitor.GetMagX())
    self._target_y = float(self._cfmonitor.GetMagY())
    
  def Step(self):
    thrust = self._cfmonitor.GetThrust()
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

    if self._auto and thrust > 0:
      input_angle = angle - target_angle
      if input_angle > math.pi:
        input_angle -= 2 * math.pi
      elif input_angle < -math.pi:
        input_angle += 2 * math.pi
      yaw = self._pid.Update(input_angle)
      logger.info('angle: %f  target angle: %f  input_angle: %f  yaw: %f',
                  angle, target_angle, input_angle, yaw)
      self._cfmonitor.SetYaw(yaw)

if __name__ == '__main__':
  import random
  class TestCf(object):
    def __init__(self):
      self.x = 100
      self.y = 200
      self.yaw = 0.0
    def SetYaw(self, yaw):
      self.yaw = yaw
    def GetMagX(self):
      return self.x
    def GetMagY(self):
      return self.y

  logging.basicConfig(
      level=logging.INFO,
      format='%(asctime)s %(levelname).1s %(module)-12.12s %(message)s')

  test_cf = TestCf()
  cy = CompassYawController(test_cf)
  cy.SetAuto(True)
  test_cf.x = 100
  test_cf.y = 0
  cy.Step()
  logger.info('calibrate')
  for i in xrange(36):
    test_cf.x = int(100 * math.cos(i * 10 * math.pi / 180.0))
    test_cf.y = int(100 * math.sin(i * 10 * math.pi / 180.0))
    cy.Step()
  logger.info('rotate again')
  for i in xrange(36):
    test_cf.x = int(100 * math.cos(i * 10 * math.pi / 180.0))
    test_cf.y = int(100 * math.sin(i * 10 * math.pi / 180.0))
    cy.Step()
  logger.info('stabilize')
  angle = 90.0
  for i in xrange(500):
    test_cf.x = int(100 * math.cos(angle * math.pi / 180.0))
    test_cf.y = int(100 * math.sin(angle * math.pi / 180.0))
    cy.Step()
    angle -= (test_cf.yaw * 0.1) + random.uniform(-2,1)
