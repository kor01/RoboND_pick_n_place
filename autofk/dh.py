from __future__ import absolute_import
from __future__ import print_function
import numpy as np
from collections import namedtuple
import autofk.geometry as geo


class ReferenceFrame(object):
  """
  represent a reference frame, with an origin o and three axis
  """
  def __init__(self, z, x, o):
    """
    construct a reference frame from z, x and o
    :param z: z axis, 3d vector
    :param x: x axis 3d vector
    :param o: origin, 3d vector
    """
    z = np.asarray(z, dtype=np.float64).copy()
    x = np.asarray(x, dtype=np.float64).copy()
    assert np.isclose(geo.cosine(z, x), 0)
    o = np.asarray(o, dtype=np.float64).copy()
    z, x = geo.normalize(z), geo.normalize(x)
    z.flags.writeable = False
    x.flags.writeable = False
    o.flags.writeable = False
    self._z, self._x, self._o = z, x, o

  @property
  def z(self):
    return self._z

  @property
  def x(self):
    return self._x

  @property
  def o(self):
    return self._o

  @property
  def y(self):
    return np.cross(self.z, self.x)


# initial values for final DHParameter
DHRow = namedtuple(
  'DHValue', ('a', 'alpha', 'd', 'theta'))


def dh_frame_to_homogenous(frame):
  y = np.cross(frame.z, frame.x)
  mat = np.array([frame.x, y, frame.z]).transpose()
  mat = np.hstack((mat, frame.o.reshape(-1, 1)))
  mat = np.vstack((mat, (0, 0, 0, 1)))
  return mat


def assign_dh_frame(last_x, this_z, next_z):

  # same line case
  if geo.same_line(this_z, next_z):
    return last_x

  dx = geo.common_normal(this_z, next_z)

  # intersect case, select dx has lesser angle with last-x
  if geo.coplanar(this_z, next_z):
    sign = np.sign(geo.cosine(dx, last_x.k))
    if sign != 0:
      dx = sign * dx

  # determine the origin
  if geo.parallel(this_z.k, next_z.k):
    origin = geo.planar_intersection(last_x, this_z)
  else:
    origin = geo.common_normal_intersection(
      this_z, next_z, dx)

  return geo.Line(p=origin, k=dx)


def search_reference_axis(joints):
  z = joints[0]
  direction_ref = None
  position_ref = None
  for joint in joints[1:]:
    if geo.same_line(joint, z):
      continue
    if geo.parallel(joint.k, z.k)\
        and direction_ref is None:
      direction_ref = joint
      continue
    if direction_ref is None:
      direction_ref = joint
    position_ref = joint
    break
  return position_ref, direction_ref


def infer_initial_frame(joints):

  p_ref, d_ref = search_reference_axis(joints)
  z = joints[0]
  # when all axis are the same
  if d_ref is None:
    dx = geo.pick_perpendicular_direction(z.k)
    return geo.Line(p=z.p, k=dx)

  dx = geo.common_normal(z, d_ref)

  if p_ref is None:
    origin = z.p
  else:
    cn = geo.common_normal(z, p_ref)
    origin = geo.common_normal_intersection(
      z, p_ref, cn)
  return geo.Line(p=origin, k=dx)


def assign_dh_frames(joints):
  """
  assign DH frames
  if xaxis(the initial xaxis) is given, the algorithm
  performs a intermediary frame inference

  :param joints: List[Line]
  :return: DHFrames of the linkage system
  """
  last_x = infer_initial_frame(joints)
  ret = [ReferenceFrame(z=joints[0].k, x=last_x.k, o=last_x.p)]

  for i in range(len(joints) - 1):
    this_z = joints[i]
    next_z = joints[i + 1]
    this_x = assign_dh_frame(last_x, this_z, next_z)
    ret.append(ReferenceFrame(z=this_z.k, x=this_x.k, o=this_x.p))
    last_x = this_x

  # end effector case
  this_z = joints[-1]
  origin = geo.planar_intersection(last_x, this_z)
  ret.append(ReferenceFrame(z=this_z.k, x=last_x.k, o=origin))

  return tuple(ret)


def infer_dh_parameter(
    last_frame, this_frame):

  last_z = geo.Line(
    k=last_frame.z, p=last_frame.o)

  this_z = geo.Line(
    k=this_frame.z, p=this_frame.o)

  alpha, a = geo.angle_distance(
    last_z, this_z, last_frame.x)

  last_x = geo.Line(
    k=last_frame.x, p=last_frame.o)
  this_x = geo.Line(
    k=this_frame.x, p=this_frame.o)

  theta, d = geo.angle_distance(
    last_x, this_x, this_frame.z)
  return DHRow(a=a, alpha=alpha, d=d, theta=theta)


def infer_dh_parameters(frames):

  last_frame = frames[0]
  ret = []
  for frame in frames[1:]:
    ret.append(
      infer_dh_parameter(last_frame, frame))
    last_frame = frame

  return tuple(ret)
