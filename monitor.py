import curses
import cv2
import logging
import time
import video_pid_controller

import cfclient.utils.logconfigreader as logconfigreader
import cflib.crazyflie as crazyflie
import cflib.crtp as crtp

logger = logging.getLogger('monitor')

LINK_URIS = ['debug://0/0']

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
]

class CfMonitor(object):
  def __init__(self, link_uri, win):
    self._link_uri = link_uri
    self._win = win
    self._cf = crazyflie.Crazyflie()
    self._cf.connectSetupFinished.add_callback(self._onConnect)
    logger.info('Opening link to ' + link_uri)
    self._cf.open_link(link_uri)

  def __del__(self):
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
    self._win.clear()
    pos = 0
    for field in FIELDS:
      if field.var is None:
        s = self._link_uri
      else:
        s = str(data[field.var])
      self._win.addnstr(0, pos, s, field.width - 1)
      pos += field.width
    self._win.refresh()

  def SetSetpoint(self, roll, pitch, yawrate, thrust):
    self._cf.commander.send_setpoint(roll, pitch, yawrate, thrust)


class CursesWindowLogHandler(logging.Handler):
  def __init__(self, win):
    logging.Handler.__init__(self)
    self._win = win

  def emit(self, record):
    self._win.insertln()
    self._win.addstr(0, 0, self.format(record))
    self._win.refresh()


if __name__ == '__main__':
  stdscr = curses.initscr()
  curses.curs_set(0)
  curses.noecho()
  curses.cbreak()
  stdscr.keypad(1)
  stdscr.nodelay(1)

  height, width = stdscr.getmaxyx()
  stdscr.hline(int(height * 0.5) - 1, 0, '=', width)

  logwin = curses.newwin(int(height * 0.5), width, int(height * 0.5), 0)
  log_handler = CursesWindowLogHandler(logwin)
  log_handler.setFormatter(logging.Formatter(
      fmt='%(asctime)s %(levelname).1s %(module)-12s %(message)s'))
  root_logger = logging.getLogger('')
  root_logger.setLevel(logging.INFO)
  root_logger.addHandler(log_handler)

  pos = 0
  for field in FIELDS:
    stdscr.addstr(0, pos, field.label)
    pos += field.width
  stdscr.refresh()

  crtp.init_drivers()

  cfmonitors = []
  pos = 1
  for uri in LINK_URIS:
    win = curses.newwin(1, width, pos, 0)
    cfmonitors.append(CfMonitor(uri, win))
    pos += 1

  controller = video_pid_controller.VideoPIDController(cfmonitors[0])

  try:
    while True:
      controller.Step()
      c = stdscr.getch()
      if c != -1:
        if c == ord('a'):
          controller.SetThrust(controller.GetThrust() + 1000)
        elif c == ord('z'):
          controller.SetThrust(controller.GetThrust() - 1000)
        elif c == ord('k'):
          logger.info('Killing thrust')
          controller.SetThrust(0)
        elif c == ord('q'):
          break
      time.sleep(0.016)
      cv2.waitKey(1)
  except KeyboardInterrupt:
    logger.info('Keyboard interrupt - shutting down')
  finally:
    stdscr.keypad(0)
    curses.nocbreak()
    curses.echo()
    curses.curs_set(1)
    curses.endwin()
    del cfmonitors
