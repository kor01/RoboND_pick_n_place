import tf
import numpy as np
import sympy as sp
from autofk import dh
from autofk import geometry as ge
from autofk import link as fk


def rpy_to_homogenous(rpy, origin, to_numeric=True):
  x_rot = sp.rot_axis1(-rpy[0])
  y_rot = sp.rot_axis2(-rpy[1])
  z_rot = sp.rot_axis3(-rpy[2])
  origin = sp.Matrix(origin)
  rot = z_rot * y_rot * x_rot
  rot = rot.row_join(origin)
  rot = rot.col_join(sp.Matrix([[0, 0, 0, 1]]))
  rot = sp.simplify(rot)
  if to_numeric:
    rot = np.array(rot.tolist(), dtype=np.float64)
  return rot


def infer_absolute_frames(js):
  frame = np.eye(4)
  ret = []
  for j in js:
    frame = np.matmul(frame, rpy_to_homogenous(
      j.orientation, j.position))
    ret.append(frame)
  return ret


def absolute_axis(axes, frames):
  assert len(axes) == len(frames)
  ret = []
  for a, f in zip(axes, frames):
    line = ge.Line(k=np.matmul(f[:3, :3], a), p=f[:3, -1])
    ret.append(line)
  return ret


def relative_frames(frames, dh_frames):
  ret = []
  for f, df in zip(frames, dh_frames):
    relative = np.matmul(np.linalg.inv(df), f)
    ret.append(relative)
  return ret


def rot_to_quaternion(frame):
  origin = frame[:3, -1]
  rot = frame[:3, :3]
  mat = np.eye(4)
  mat[:3, :3] = rot
  # noinspection PyUnresolvedReferences
  rotation = tf.transformations.quaternion_from_matrix(mat)
  return origin, rotation


def construct_dh_links(joints, abs_frames):
  movables = filter(
    lambda x: joints[x].axis is not None,
    range(len(joints)))

  axis = map(lambda x: joints[x].axis, movables)
  movable_frames = map(lambda x: abs_frames[x], movables)
  abs_axis = absolute_axis(axis, movable_frames)
  abs_dh_frames = dh.assign_dh_frames(abs_axis)
  dh_parameters = dh.infer_dh_parameters(abs_dh_frames)
  abs_dh_frames = map(dh.dh_frame_to_homogenous, abs_dh_frames)
  rel_frames = relative_frames(
    movable_frames, abs_dh_frames[1:])
  links = []
  for i, param, rel_frame in \
      zip(movables, dh_parameters, rel_frames):
    joint = joints[i]
    link = fk.DHLink(i, param, joint.type == 'revolute',
                     joint.child.name, joint.name,
                     rel_frame, joint.limits)
    links.append(link)
  dh_base = abs_dh_frames[0]
  return dh_base, tuple(abs_dh_frames), \
         tuple(links), tuple(dh_parameters), \
         tuple(abs_dh_frames)


def construct_fixed_links(
    joints, abs_frames, abs_dh_frames):
  movables = filter(
    lambda x: joints[x].axis is not None,
    range(len(joints)))
  assert len(movables) == len(abs_dh_frames) - 1

  dh_frame, dh_ancestor = abs_dh_frames[0], -1
  fixed_links = []
  for i, j in enumerate(joints):
    movable = j.axis is not None
    if movable:
      dh_ancestor += 1
      dh_frame = abs_dh_frames[dh_ancestor + 1]
      assert dh_frame is not None
    else:
      relative = np.matmul(
        np.linalg.inv(dh_frame), abs_frames[i])
      link = fk.FixedLink(
        i, relative, dh_ancestor,
        link_name=j.child.name, joint_name=j.name)
      fixed_links.append(link)
  return fixed_links


IDENTITY = np.eye(4)
IDENTITY.flags.writeable = False


class Chain(object):
  def __init__(self, joints):
    self._abs_frames = infer_absolute_frames(joints)
    self._dh_base, abs_dh_frames, \
     self._dh_links, self._dh_parameter, \
     self._dh_frames = \
      construct_dh_links(joints, self._abs_frames)
    self._fixed_links = construct_fixed_links(
      joints, self._abs_frames, abs_dh_frames)
    self._base_link_name = joints[0].parent.name

  def forward(self, joint_states,
              base_frame=IDENTITY,
              return_dh=True,
              partial_index=None):

    # get joint states of this serial graph
    states = []

    iter_range = range(len(self._dh_links))
    if partial_index is not None:
      assert return_dh
      iter_range = range(partial_index)

    for i, l in zip(iter_range, self._dh_links):
      key = l.joint
      states.append(joint_states[key])

    # inference on movable links
    hm = np.matmul(base_frame, self._dh_base)
    dh_frames = [(self._base_link_name, hm)]
    for i, s in zip(iter_range, states):
      element = self._dh_links[i](s)
      link_name = self._dh_links[i].link
      hm = np.matmul(hm, element)
      dh_frames.append((link_name, hm))

    if return_dh:
      return dh_frames
    else:
      # recover URDF frames from dh frames
      ret = [None] * len(self._abs_frames)
      for (key, hm), link in zip(
        dh_frames[1:], self._dh_links):
        assert ret[link.index] is None
        hm = np.matmul(hm, link.rel)
        ret[link.index] = (key, hm)
      for link in self._fixed_links:
        base = dh_frames[link.dh_ancestor + 1][1]
        key = link.link
        ret[link.index] = \
          (key, np.matmul(base, link.rel))
    return tuple(ret)

  @property
  def dh_links(self):
    return self._dh_links

  @property
  def fixed_links(self):
    return self._fixed_links

  @property
  def dh_base_link(self):
    return self._base_link_name, self._dh_base

  @property
  def dh_parameter(self):
    return self._dh_parameter

  @property
  def dh_frames(self):
    return self._dh_frames

