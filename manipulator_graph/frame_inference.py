import numpy as np
from collections import namedtuple
import line_geometry as lg

# represented by equation k cross (x - p) == 0
DHFrame = namedtuple('DHFrame', ('z', 'x', 'o'))


def dh_frame_to_homogenous(frame):
  y = np.cross(frame.z, frame.x)
  mat = np.array([frame.x, y, frame.z]).transpose()
  mat = np.hstack((mat, frame.o.reshape(-1, 1)))
  mat = np.vstack((mat, (0, 0, 0, 1)))
  return mat


def assign_dh_frame(
    last_x, last_o, this_z, next_z):
  # x is chosen as last_x, o as last_o
  if lg.same_line(this_z, next_z):
    return last_o, last_x

  this_x = lg.common_normal(this_z, next_z)
  if lg.parallel(this_z.k, next_z.k):
    last_xaxis = lg.Line(k=last_x, p=last_o)
    origin = lg.planar_intersection(
      last_xaxis, this_z)
  else:
    origin = lg.common_normal_intersection(
      this_z, next_z, this_x)
  return origin, this_x


def anchor_axis(joints):
  z = joints[0]
  danchor = None
  panchor = None
  for j in joints[1:]:
    if lg.same_line(j, z):
      continue
    if lg.parallel(j.k, z.k)\
        and danchor is None:
      danchor = j
      continue
    if danchor is None:
      danchor = j
    panchor = j
    break
  return panchor, danchor


def infer_initial_frame(joints):

  panchor, danchor = anchor_axis(joints)
  zaxis = joints[0]
  # when all axis are the same
  if danchor is None:
    xdirection = np.array([-zaxis.k[1], zaxis.k[0], 0],
                          dtype=np.float64)
    if lg.zero_vector(xdirection):
      xdirection[0] = 1
    xdirection = lg.normalize(xdirection)
    origin = zaxis.p
    return xdirection, origin

  xdirection = lg.common_normal(zaxis, danchor)

  if panchor is None:
    origin = zaxis.p
  else:
    cn = lg.common_normal(zaxis, panchor)
    origin = lg.common_normal_intersection(
      zaxis, panchor, cn)
  return xdirection, origin


def assign_dh_frames(
    joints, xaxis=None, end_effector=True):
  """
  assign DH frames
  if xaxis(the initial xaxis) is given, the algorithm
  performs a intermediary frame inference

  :param joints: List[Line]
  :param xaxis: the initial xaxis
  :param end_effector: whether to include end_effector frame
  :return: DHFrames of the linkage system
  """

  if xaxis is not None:
    last_x, last_o = xaxis.k, xaxis.p
  else:
    last_x, last_o = infer_initial_frame(joints)

  ret = [DHFrame(z=joints[0].k, x=last_x, o=last_o)]

  for i in range(len(joints) - 1):
    this_z = joints[i]
    next_z = joints[i + 1]
    origin, x_direction = assign_dh_frame(
      last_x, last_o, this_z, next_z)

    ret.append(DHFrame(z=this_z.k, x=x_direction, o=origin))
    last_x, last_o = x_direction, origin

  # end effector case
  if end_effector:
    xaxis = lg.Line(k=last_x, p=last_o)
    zaxis = joints[-1]
    origin = lg.planar_intersection(xaxis, zaxis)
    ret.append(DHFrame(z=zaxis.k, x=xaxis.k, o=origin))

  return tuple(ret)

