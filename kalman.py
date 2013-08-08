import logging
import numpy

logger = logging.getLogger('kalman')

# see http://greg.czerniak.info/guides/kalman1/
class KalmanFilter(object):
  def __init__(self, A, B, H, x, P, Q, R):
    self._A = A  # state transition matrix
    self._B = B  # control matrix
    self._H = H  # observation matrix
    self._x = x  # state estimate
    self._P = P  # covariance estimate
    self._Q = Q  # estimated error in process
    self._R = R  # estimated error in measurement

  def Step(self, control, measurement):
    # prediction step
    xe = self._A * self._x + self._B * control
    pe = (self._A * self._P) * numpy.transpose(self._A) + self._Q

    # observation step
    innovation = measurement - self._H * xe
    innovation_covariance = self._H * pe * numpy.transpose(self._H) + self._R

    # update step
    kalman_gain = (pe * numpy.transpose(self._H) *
                   numpy.linalg.inv(innovation_covariance))
    self._x = xe + kalman_gain * innovation
    self._P = (numpy.eye(self._P.shape[0]) - kalman_gain * self._H) * pe

  def GetState(self):
    return self._x


if __name__ == '__main__':
  import random
  logging.basicConfig(level=logging.DEBUG)

  # single variable, no control
  A = numpy.mat([1.0])
  B = numpy.mat([0.0])
  H = numpy.mat([1.0])
  x = numpy.mat([2.0])
  P = numpy.mat([1.0])
  Q = numpy.mat([0.0001])
  R = numpy.mat([0.1])
  kf = KalmanFilter(A, B, H, x, P, Q, R)
  for i in xrange(100):
    measurement = random.gauss(10.0, 0.5)
    kf.Step(0, measurement)
    logger.debug('in=%f out=%f', measurement, kf.GetState())

  # vertical velocity and acceleration
  # i = throttle, g = gravity, ts = time between iterations
  # a(n) = 0.02 * i(n) - g
  # v(n+1) = v(n) + 0.02 * i(n) * ts - g * ts
  # x(n+1) = x(n) + v(n) + 0.01 * i(n) * ts^2 - 0.5 * g * ts^2
  ts = 0.1
  g = 5.0
  i = 100.0

  A = numpy.mat([[1.0, 1.0],   # x(n+1) = x(n) + v(n)
                 [0.0, 1.0]])  # v(n+1) =        v(n)
  B = numpy.mat([[1.0, 0.0],   # control[0]
                 [0.0, 1.0]])  # control[1]
  control = numpy.mat([[0.01 * i * ts * ts - 0.5 * g * ts * ts],
                       [0.02 * i * ts - g * ts]])
  H = numpy.eye(2)
  pos, vel = 100, 0
  x = numpy.mat([[pos], [vel]])
  P = numpy.eye(2)
  Q = numpy.zeros(2)
  R = numpy.eye(2) * 0.2
  kf = KalmanFilter(A, B, H, x, P, Q, R)
  for i in range(100,500,10) + range(500,250,-10) + [250] * 10:
    control = numpy.mat([[0.01 * i * ts * ts - 0.5 * g * ts * ts],
                         [0.02 * i * ts - g * ts]])
    vel += 0.02 * i * ts - g * ts
    pos += vel
    x = numpy.mat([[random.gauss(pos, 0.5)],[random.gauss(vel, 0.5)]])
    kf.Step(control, x)
    result = kf.GetState()
    logger.debug('x=%f v=%f xe=%f ve=%f', pos, vel, result[0][0], result[1][0])
