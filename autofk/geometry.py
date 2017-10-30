from __future__ import absolute_import
from __future__ import print_function
import numpy as np


class Line(object):

  def __init__(self, p, k):
    """
    create an immutable Line object represents a Line in 3D Euclidean Space
    parameterized by a point p and a direction k.
    this representation is over-parameterized for computational convenience
    :param p: point, 3 dimensional vector (ndarray or tuple, list)
    :param k: direction, 3 dimensional vector (ndarray or tuple, list)
    """
    p = np.asarray(p, dtype=np.float64).copy()
    k = np.asarray(k, dtype=np.float64).copy()
    k = normalize(k)
    assert k.shape == (3,)
    assert p.shape == (3,)
    self._k, self._p = k, p
    self._p.flags.writeable = False
    self._k.flags.writeable = False

  @property
  def p(self):
    """
    a the point that the line pass through
    :return: np.ndarray
    """
    return self._p

  @property
  def k(self):
    """
    the direction of the line
    :return:
    """
    return self._k


def zero_vector(v):
  return np.allclose(v, 0)


def angle_distance(left, right, reference):
  """
  compute angle and distance
  from left to right w.r.t reference
  :param left: a Line
  :param right: a Line
  :param reference: a 3d vector
  :return: angle and distance
  """
  # lesser angle between the two
  cos = cosine(left.k, right.k)
  lesser_angle = np.arctan2(np.sqrt(1 - cos * cos), cos)
  if np.isclose(lesser_angle, 0):
    angle = 0
  # atan2 is not continuous around (-1, 0)
  elif np.isclose(lesser_angle - np.pi, 0):
    angle = np.pi
  else:
    ori = np.cross(left.k, right.k)
    sign = np.sign(np.dot(ori, reference))
    angle = lesser_angle * sign

  diff = right.p - left.p
  distance = np.dot(diff, reference)
  return angle, distance


def cross_skew_matrix(omega):
  """
  create a skew symmetric matrix from vector omega
  representing operation operation f(v) := omega x v
  :param omega: the rotation direction
  :return: the skew symmetric matrix representing cross product
  """
  x, y, z = omega
  ret = [[0, -z, y], [z, 0, -x], [-y, x, 0]]
  return np.array(ret)


def normalize(k):
  k = k / np.linalg.norm(k)
  return k


def parallel(k1, k2):
  return np.isclose(1 - cosine(k1, k2), 0)


def same_line(left, right):
  if not parallel(left.k, right.k):
    return False
  difference = left.p - right.p
  if zero_vector(difference):
    return True
  return parallel(difference, left.k)


def same_plane(left, right):
  k1, p1 = left.k, left.p
  k2, p2 = right.k, right.p
  det = np.linalg.det([k1, k2, p1 - p2])
  return np.isclose(det, 0)


def intersects(left, right):
  return intersects(left, right) and (not parallel(left, right))


def planar_intersection(left, right):
  p1, k1 = left.p, left.k
  p2, k2 = right.p, right.k
  omega1 = cross_skew_matrix(k1)
  omega2 = cross_skew_matrix(k2)
  w = np.vstack([omega1, omega2])
  b1 = np.cross(k1, p1)
  b2 = np.cross(k2, p2)
  b = np.concatenate((b1, b2), axis=-1)
  ret = np.linalg.lstsq(w, b)[0]
  return ret


def parallel_common_normal(left, right):
  diff = right.p - left.p
  k = right.k
  # k x (k x (p2 - p1))
  normal = normalize(np.cross(k, np.cross(k, diff)))
  # sign of normal diff inner product
  sign = np.sign((normal * diff).sum())
  normal = normal * sign
  return normal


# k1 x k2  perpendicular to (p1 - p2)
def coplanar(left, right):
  normal = np.cross(left.k, right.k)
  diff = (left.p - right.p)
  inner = np.dot(normal, diff)
  return np.isclose(inner, 0)


def cosine(k1, k2):
  k1, k2 = normalize(k1), normalize(k2)
  return np.dot(k1, k2)


def nonparallel_common_normal(left, right):
  k = np.cross(left.k, right.k)
  k = normalize(k)
  distance = cosine(right.p - left.p, k)
  sign = np.sign(distance)
  if sign != 0:
    return k * sign
  else:
    return k


def common_normal(left, right):
  assert not same_line(left, right),\
    'should not be the same'
  if parallel(left.k, right.k):
    ret = parallel_common_normal(left, right)
  else:
    ret = nonparallel_common_normal(left, right)
  assert not zero_vector(ret)
  return ret


def common_normal_intersection(
    left, right, direction):
  distance = np.abs(np.dot(
    right.p - left.p, direction))

  if not np.isclose(distance, 0):
    new_p = right.p - distance * direction
  else:
    new_p = right.p

  new_line = Line(k=right.k, p=new_p)
  assert coplanar(new_line, left)
  p = planar_intersection(new_line, left)
  return p


def pick_perpendicular_direction(k):
  ret = np.array([-k[1], k[0], 0], dtype=np.float64)
  if np.allclose(ret, 0):
    ret[0] = 1
  ret = normalize(ret)
  return ret
