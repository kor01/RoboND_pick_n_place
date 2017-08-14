import numpy as np
import line_geometry as lg
from collections import namedtuple

PRECISION = 7

# initial values for final DHParameter
DHValue = namedtuple(
  'DHValue', ('a', 'alpha', 'd', 'theta'))


# for better debug
def round_off_values(a, alpha, d, theta):
  a, d = np.around(
    [a, d], decimals=PRECISION)
  return DHValue(a, alpha, d, theta)


def inference_dh_parameter(
    last_frame, this_frame):

  last_z = lg.Line(
    k=last_frame.z, p=last_frame.o)

  this_z = lg.Line(
    k=this_frame.z, p=this_frame.o)

  alpha, a = lg.angle_distance(
    last_z, this_z, last_frame.x)

  last_x = lg.Line(
    k=last_frame.x, p=last_frame.o)
  this_x = lg.Line(
    k=this_frame.x, p=this_frame.o)

  theta, d = lg.angle_distance(
    last_x, this_x, this_frame.z)
  return round_off_values(a, alpha, d, theta)


def inference_dh_parameters(frames):

  last_frame = frames[0]
  ret = []
  for frame in frames[1:]:
    ret.append(
      inference_dh_parameter(last_frame, frame))
    last_frame = frame

  return tuple(ret)
