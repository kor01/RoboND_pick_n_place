import numpy as np
from collections import namedtuple

Line = namedtuple('Line', ('p', 'k'))

ZERO_TOLERANCE = 1e-6


def zero_vector(v):
  return np.linalg.norm(v) < ZERO_TOLERANCE


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
  cosine = inner_prod(left.k, right.k)
  lesser_angle = np.arctan2(
    np.sqrt(1 - cosine * cosine), cosine)
  if np.abs(lesser_angle) < ZERO_TOLERANCE:
    angle = 0
  # atan2 is not continuous around (-1, 0)
  elif np.abs(lesser_angle - np.pi) < ZERO_TOLERANCE:
    angle = np.pi
  else:
    ori = np.cross(left.k, right.k)
    sign = np.sign(inner_prod(ori, reference))
    angle = lesser_angle * sign

  diff = right.p - left.p
  distance = inner_prod(diff, reference)
  return angle, distance


def set_zero_tolerance(v):
  global ZERO_TOLERANCE
  ZERO_TOLERANCE = v


def cross_skew_matrix(omega):
  x, y, z = omega
  ret = [[0, -z, y], [z, 0, -x], [-y, x, 0]]
  return np.array(ret)


def inner_prod(a, b):
  return (a * b).sum(axis=-1)


def normalize(k):
  k = k / np.linalg.norm(k)
  return k


def parallel(k1, k2):
  k1 = normalize(k1)
  k2 = normalize(k2)
  cos = inner_prod(k1, k2)
  if np.abs(1 - cos) < ZERO_TOLERANCE:
    return True
  return False


def same_line(left, right):
  if not parallel(left.k, right.k):
    return False
  difference = left.p - right.p
  if zero_vector(difference):
    return True
  return parallel(difference, left.k)


def planar_intersection(line1, line2):
  p1, k1 = line1
  p2, k2 = line2
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
  inner = inner_prod(normal, diff)
  if np.abs(inner) < ZERO_TOLERANCE:
    return True
  return False


def nonparallel_common_normal(left, right):
  k = normalize(np.cross(left.k, right.k))
  distance = inner_prod(right.p - left.p, k)
  if np.abs(distance) < ZERO_TOLERANCE:
    return k
  sign = np.sign(distance)
  return k * sign


def common_normal(left, right):
  assert not same_line(left, right),\
    'should not be the same'
  if parallel(left.k, right.k):
    return parallel_common_normal(left, right)
  else:
    return nonparallel_common_normal(left, right)


def common_normal_intersection(
    left, right, direction):
  distance = np.abs(inner_prod(
    right.p - left.p, direction))

  if distance > ZERO_TOLERANCE:
    new_p = right.p - distance * direction
  else:
    new_p = right.p

  new_line = Line(k=right.k, p=new_p)
  assert coplanar(new_line, left)
  p = planar_intersection(new_line, left)
  return p
