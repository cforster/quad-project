import logging
import pygame

logger = logging.getLogger('joystick')

class BaseJoystick(object):
  def __init__(self, id, thrust_max_button, thrust_min_button, switch_button):
    self._joystick = pygame.joystick.Joystick(id)
    self._joystick.init()
    logger.info('Joystick axes=%d buttons=%d',
                self._joystick.get_numaxes(),
                self._joystick.get_numbuttons())
    
    self._thrust_max_button = thrust_max_button
    self._thrust_min_button = thrust_min_button

    self._switch_button = switch_button

  def getRoll(self):
    raise NotImplementedError("Should have implemented getRoll")

  def getPitch(self):
    raise NotImplementedError("Should have implemented getPitch")

  def getYaw(self):
    raise NotImplementedError("Should have implemented getYaw")
  
  def getThrust(self):
    raise NotImplementedError("Should have implemented getThrust")

  def getIncreaseMaxThrustButton(self):
    return self._thrust_max_button

  def getDecreaseMaxThrustButton(self):
    return self._thrust_min_button

  def getSwitchButton(self):
    return self._switch_button

class Xbox360ControllerWindowsJoystick(BaseJoystick):
  def __init__(self, id):
    BaseJoystick.__init__(self, id,
                          3, # thrust_max_button: Y
                          0, # thrust_min_button: A
                          1 # switch_button: B
    )
    
    self._roll_axis = 0 # Left stick right/left
    self._pitch_axis = 1 # Left stick up/down
    self._yaw_axis = 4 # Right stick right/left
    self._thrust_axis = 2 # Right trigger

  def getRoll(self):
    # axis right = 1, left = -1
    return self._joystick.get_axis(self._roll_axis)

  def getPitch(self):
    # axis up = -1, down = 1
    return -self._joystick.get_axis(self._pitch_axis)

  def getYaw(self):
    # axis right = 1, left = -1
    return self._joystick.get_axis(self._yaw_axis)
  
  def getThrust(self):
    # axis range from 0 (not pressed) to -1 (pressed)
    return -self._joystick.get_axis(self._thrust_axis)  

class Xbox360ControllerLinuxJoystick(BaseJoystick):
  def __init__(self, id):
    BaseJoystick.__init__(self, id,
                          3, # thrust_max_button: Y
                          0, # thrust_min_button: A
                          1 # switch_button: B
    )
      
    self._initial_thrust_flag = True

    self._roll_axis = 0 # Left stick right/left
    self._pitch_axis = 1 # Left stick up/down
    self._yaw_axis = 2 # Right stick right/left
    self._thrust_axis = 4 # Right trigger

  def getRoll(self):
    # axis right = 1, left = -1
    return self._joystick.get_axis(self._roll_axis)

  def getPitch(self):
    # axis up = -1, down = 1
    return -self._joystick.get_axis(self._pitch_axis)

  def getYaw(self):
    # axis right = 1, left = -1
    return self._joystick.get_axis(self._yaw_axis)
  
  def getThrust(self):
    # axis initializes to 0 not -1
    if thrust == 0.0 and self._initial_thrust_flag:
      return 0.0;
    else:
      self._initial_thrust_flag = False
      # axis range from -1 (not pressed) to 1 (pressed)
      return (self._joystick.get_axis(self._thrust_axis) + 1) / 2
