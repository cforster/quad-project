import cv2
import logging
import pid

logger = logging.getLogger('video_pid_controller')

CANNY_THRESHOLD = 20
PITCH_ROLL_RANGE = 10

class VideoPIDController(object):
  def __init__(self, cfmonitor, window_name='Controller', camera_index=0):
    self._cfmonitor = cfmonitor
    self._window_name = window_name

    self._capture = cv2.VideoCapture()
    self._capture.open(camera_index)
    self._width = self._capture.get(3)
    self._height = self._capture.get(4)

    self._thrust = 0
    self._x_target = self._width / 2
    self._y_target = self._height / 2
    self._x_pid = pid.PID(out_min=-PITCH_ROLL_RANGE, out_max=PITCH_ROLL_RANGE)
    self._x_pid.SetSetpoint(self._x_target)
    self._y_pid = pid.PID(out_min=-PITCH_ROLL_RANGE, out_max=PITCH_ROLL_RANGE)
    self._y_pid.SetSetpoint(self._y_target)

  def _FindQuad(self, im):
    gray_im = cv2.cvtColor(im, cv2.COLOR_RGB2GRAY)
    gray_im = cv2.blur(gray_im, (5, 5))
    edges = cv2.Canny(gray_im, CANNY_THRESHOLD, CANNY_THRESHOLD * 3)
    element = cv2.getStructuringElement(cv2.MORPH_CROSS, (5, 5))
    edges = cv2.dilate(edges, element)

    contours, _ = cv2.findContours(
        edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    max_area = 0
    largest_contour = None
    for contour in contours:
      area = cv2.contourArea(contour)
      if area > max_area:
        max_area = area
        largest_contour = contour

    if largest_contour is not None:
      bounding_rect = cv2.boundingRect(largest_contour)
      x = bounding_rect[0] + bounding_rect[2] / 2
      y = bounding_rect[1] + bounding_rect[3] / 2

      cv2.drawContours(im, [largest_contour], -1, (255, 128, 128), 1)
      cv2.rectangle(im,
                    (bounding_rect[0], bounding_rect[1]),
                    (bounding_rect[0] + bounding_rect[2],
                     bounding_rect[1] + bounding_rect[3]),
                    (255, 255, 255))
      cv2.circle(im, (x,y), 2, (0, 0, 255))

      return (x, y)
    else:
      return None

  def Step(self):
    self._capture.grab()
    result, im = self._capture.retrieve()

    position = self._FindQuad(im)
    if position is not None:
      x, y = position
      roll = self._x_pid.Update(x)
      pitch = self._y_pid.Update(y)
    else:
      roll = 0
      pitch = 0

    self._cfmonitor.SetSetpoint(roll, pitch, 0, self._thrust)

    cv2.circle(im, (int(self._x_target), int(self._y_target)), 2, (0, 255, 0))
    cv2.imshow(self._window_name, im)

  def SetThrust(self, thrust):
    if thrust > 60000:
      self._thrust = 60000
    elif thrust < 0:
      self._thrust = 0
    else:
      self._thrust = thrust

  def GetThrust(self):
    return self._thrust