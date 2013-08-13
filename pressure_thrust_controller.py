import logging
import pid

logger = logging.getLogger('pressure_thrust_controller')

class PressureThrustController(object):
  def __init__(self, cfmonitor):
    self._cfmonitor = cfmonitor
    self._pid = pid.PID(kp=10000.0, ki=5000.0, kd=15000.0,
                        integ_max=50000.0,
                        out_min=-4000, out_max=4000)
    self._target_pressure = 100.0
    self._thrust_center = 40000
    self._auto = False

  def SetAuto(self, auto):
    if auto and not self._auto:
      self._target_pressure = self._cfmonitor.GetPressure()
      self._pid.SetSetpoint(self._target_pressure)
      self._thrust_center = self._cfmonitor.GetThrust()
    self._auto = auto

  def Step(self):
    if self._auto:
      pressure = self._cfmonitor.GetPressure()
      thrust_delta = -self._pid.Update(pressure)
      logger.info('pressure: %f  target pressure: %f  thrust_delta: %d',
                  pressure, self._target_pressure, thrust_delta)
      self._cfmonitor.SetThrust(int(self._thrust_center + thrust_delta))
