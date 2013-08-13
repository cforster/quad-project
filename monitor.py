import cv2
import joystick_controller
import logging
import pressure_thrust_controller
import sys
import time
import video_pid_controller

from PyQt4 import QtCore
from PyQt4 import QtGui

import cfclient.utils.logconfigreader as logconfigreader
import cflib.crazyflie as crazyflie
import cflib.crtp as crtp

logger = logging.getLogger('monitor')

LINK_URIS = ['radio://0/4/250K']

class Field(object):
  def __init__(self, label, width, var=None, vartype='float'):
    self.label = label
    self.width = width
    self.var = var
    self.vartype = vartype

FIELDS = [
    Field('URI', 16),
    Field('ROLL', 10, 'stabilizer.roll'),
    Field('PITCH', 10, 'stabilizer.pitch'),
    Field('YAW', 10, 'stabilizer.yaw'),
    Field('THRUST', 10, 'stabilizer.thrust', 'uint16_t'),
    Field('PRESSURE', 10, 'altimeter.pressure'),
    #Field('MAG_X', 10, 'mag.x', 'int16_t'),
    #Field('MAG_Y', 10, 'mag.y', 'int16_t'),
    #Field('MAG_Z', 10, 'mag.z', 'int16_t'),
    Field('AUTO', 10)
]

class CfMonitor(object):
  def __init__(self, index, link_uri, window):
    self._roll = 0.0
    self._pitch = 0.0
    self._yaw = 0.0
    self._thrust = 0
    self._pressure = 0
    self._auto = False

    self._index = index
    self._link_uri = link_uri
    self._window = window
    self._cf = crazyflie.Crazyflie()
    self._cf.connectSetupFinished.add_callback(self._onConnect)
    logger.info('Opening link to ' + link_uri)
    self._cf.open_link(link_uri)

  def Shutdown(self):
    logger.error('Deleting cf')
    self._cf.close_link()

  def _onConnect(self, link_uri):
    logger.info('Connected to crazyflie ' + link_uri)

    logconf = logconfigreader.LogConfig('Logging', period=100)
    for f in [f for f in FIELDS if f.var is not None]:
      logconf.addVariable(logconfigreader.LogVariable(f.var, f.vartype))
    logpacket = self._cf.log.create_log_packet(logconf)
    if not logpacket:
      logger.error('Failed to create log packet')
      return
    logpacket.dataReceived.add_callback(self._onLogData)
    logpacket.start()

  def _onLogData(self, data):
    self._pressure = data['altimeter.pressure']
    for i, field in enumerate(FIELDS):
      if field.var is None:
        if field.label == 'URI':
          s = self._link_uri
        elif field.label == 'AUTO':
          s = 'on' if self._auto else 'off'
      else:
        s = str(data[field.var])
      self._window.SetTableItemText(self._index, i, s)

  def GetPressure(self):
    return self._pressure

  def GetThrust(self):
    return self._thrust

  def SetRoll(self, roll):
    self._roll = roll

  def SetPitch(self, pitch):
    self._pitch = pitch

  def SetYaw(self, yaw):
    self._yaw = yaw

  def SetThrust(self, thrust):
    self._thrust = thrust

  def UpdateCommander(self):
    self._cf.commander.send_setpoint(
        self._roll, self._pitch, self._yaw, self._thrust)


class CfMonitorWindow(QtGui.QWidget):
  def __init__(self):
    super(CfMonitorWindow, self).__init__()
    self.setWindowTitle('Crazyflie Monitor')

    self._table = QtGui.QTableWidget(self)
    self._table.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
    self._table.setColumnCount(len(FIELDS))
    self._table.setRowCount(len(LINK_URIS))
    self._table.setHorizontalHeaderLabels([f.label for f in FIELDS])
    for c, field in enumerate(FIELDS):
      self._table.setColumnWidth(c, field.width * 10)
    for r in xrange(self._table.rowCount()):
      for c in xrange(self._table.columnCount()):
        self._table.setItem(r, c, QtGui.QTableWidgetItem())

    self._vbox = QtGui.QVBoxLayout()
    self._vbox.addStretch(1)
    self._vbox.addWidget(self._table)
    self.setLayout(self._vbox)

    self.setGeometry(100, 100, 600, 200)
    self.show()

  def SetTableItemText(self, row, col, text):
    self._table.item(row, col).setText(text)


if __name__ == '__main__':
  logging.basicConfig(
      level=logging.INFO,
      format='%(asctime)s %(levelname).1s %(module)-12.12s %(message)s')

  app = QtGui.QApplication(sys.argv)
  window = CfMonitorWindow()

  crtp.init_drivers()

  cfmonitors = []
  for i, uri in enumerate(LINK_URIS):
    cfmonitors.append(CfMonitor(i, uri, window))

  video_controller = video_pid_controller.VideoPIDController(cfmonitors[0])
  joy_controller = joystick_controller.JoystickController(cfmonitors[0])
  pressure_thrust_controller = (
      pressure_thrust_controller.PressureThrustController(cfmonitors[0]))

  try:
    while True:
      auto = joy_controller.GetAuto()
      cfmonitors[0]._auto = auto
      video_controller.SetAuto(auto)
      pressure_thrust_controller.SetAuto(auto)
      video_controller.Step()
      joy_controller.Step()
      pressure_thrust_controller.Step()
      for cfmonitor in cfmonitors:
        cfmonitor.UpdateCommander()
      time.sleep(0.016)
      cv2.waitKey(1)
  except KeyboardInterrupt:
    logger.info('Keyboard interrupt - shutting down')
  finally:
    for cfmonitor in cfmonitors:
      cfmonitor.Shutdown()
