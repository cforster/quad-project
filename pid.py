import logging
import time

from PyQt4 import QtGui

logger = logging.getLogger('pid')

class PID(object):
  def __init__(self, kp=0.5, ki=0.2, kd=0.75,
               integ_max=100.0, out_max=100, out_min=0):
    self._kp = kp
    self._ki = ki
    self._kd = kd
    self._integ_max = integ_max
    self._out_max = out_max
    self._out_min = out_min

    self._setpoint = 0.0
    self._last_error = 0.0
    self._integral = 0.0
    self._last_update_time = time.time()

  def SetSetpoint(self, setpoint):
    self._setpoint = setpoint

  def Update(self, measured):
    now = time.time()
    dt = now - self._last_update_time
    self._last_update_time = now

    error = self._setpoint - measured

    self._integral += error * dt
    if self._integral > self._integ_max:
      self._integral = self._integ_max
    elif self._integral < -self._integ_max:
      self._integral = -self._integ_max

    derivative = (error - self._last_error) / dt

    p = self._kp * error
    i = self._ki * self._integral
    d = self._kd * derivative
    logger.debug('p=%5f i=%5f d=%5f', p, i, d)

    output = p + i + d
    if output > self._out_max:
      output = self._out_max
    elif output < self._out_min:
      output = self._out_min

    self._last_error = error
    return output

  def CreateWindow(self, name):
    self._window = QtGui.QWidget()
    self._window.setWindowTitle('PID ' + name)
    grid = QtGui.QGridLayout()
    grid.addWidget(QtGui.QLabel('KP', self._window), 0, 0)
    edit = QtGui.QLineEdit(self._window)
    edit.setText('%f' % self._kp)
    def SetKP(text):
      self._kp = float(text)
    edit.textChanged[str].connect(SetKP)
    grid.addWidget(edit, 0, 1)
    grid.addWidget(QtGui.QLabel('KI', self._window), 1, 0)
    edit = QtGui.QLineEdit(self._window)
    edit.setText('%f' % self._ki)
    def SetKI(text):
      self._ki = float(text)
    edit.textChanged[str].connect(SetKI)
    grid.addWidget(edit, 1, 1)
    grid.addWidget(QtGui.QLabel('KD', self._window), 2, 0)
    edit = QtGui.QLineEdit(self._window)
    edit.setText('%f' % self._kd)
    def SetKD(text):
      self._kd = float(text)
    edit.textChanged[str].connect(SetKD)
    grid.addWidget(edit, 2, 1)
    self._window.setLayout(grid)
    self._window.show()

if __name__ == '__main__':
  # test the PID controller
  import random
  logging.basicConfig(level=logging.DEBUG)
  p = PID()

  p.SetSetpoint(100)
  pos = 50
  velocity = 1
  while True:
    output = p.Update(pos)
    logger.debug('pos=%5f vel=%5f out=%5f', pos, velocity, output)
    pos += velocity
    velocity += output * 0.02 + random.uniform(-0.1, 0.1) - 0.5
    time.sleep(0.1)
