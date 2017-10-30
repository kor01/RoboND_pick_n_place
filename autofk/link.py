from __future__ import absolute_import
from __future__ import print_function
import numpy as np
import sympy as sp
from sympy.utilities.autowrap import autowrap


def homogenous_transform(a, alpha, d, theta):
  # sympy's rotation convention is different from the one normally used
  rx = sp.rot_axis1(-alpha)
  rx = rx.row_join(sp.Matrix([a, 0, 0]))
  rx = rx.col_join(sp.Matrix([[0, 0, 0, 1]]))
  rz = sp.rot_axis3(-theta)
  rz = rz.row_join(sp.Matrix([0, 0, d]))
  rz = rz.col_join(sp.Matrix([[0, 0, 0, 1]]))
  return rx * rz


def compile_homogenous(
    a, alpha, d, theta, is_revolute):
  if is_revolute:
    theta = sp.symbols('theta') + theta
  else:
    d = sp.symbols('d') + d
  m = homogenous_transform(a, alpha, d, theta)
  return autowrap(m)


class DHLink(object):

  def __init__(self, index, dh, is_revolute,
               link_name, joint_name, rel, limits):
    self._inference = compile_homogenous(
      dh.a, dh.alpha,
      dh.d, dh.theta, is_revolute)
    self._link_name = link_name
    self._joint_name = joint_name
    self._rel = rel
    self._index = index

    self._lower = limits.lower
    self._upper = limits.upper

  def __call__(self, joint_param):
    assert self._lower <= joint_param <= self._upper, \
      'out of limits'
    return self._inference(joint_param)

  def reachable(self, param):
    return self._lower <= param <= self._upper

  @property
  def link(self):
      return self._link_name

  @property
  def joint(self):
      return self._joint_name

  @property
  def rel(self):
      return self._rel

  @property
  def index(self):
      return self._index

  @property
  def reachable_range(self):
    return self._lower, self._upper


class FixedLink(object):

  def __init__(self, index, rel, dh_ancestor,
               link_name, joint_name):
    self._rel = rel
    self._dh_ancestor = dh_ancestor
    self._link_name = link_name
    self._joint_name = joint_name
    self._index = index

  def __call__(self, dh_homogenous):
    return np.matmul(dh_homogenous, self._rel)

  @property
  def rel(self):
      return self._rel

  @property
  def dh_ancestor(self):
      return self._dh_ancestor

  @property
  def link(self):
    return self._link_name

  @property
  def joint(self):
    return self._joint_name

  @property
  def index(self):
      return self._index
