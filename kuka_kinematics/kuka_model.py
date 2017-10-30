from __future__ import absolute_import
from __future__ import print_function
from __future__ import generators

import sys
import gflags
import rospy
import tf
import numpy as np
import sympy as sp
from sensor_msgs.msg import JointState
from autofk.parser import URDFParser
from autofk.dag import KinematicsDag
from autofk import geometry as lg
from autofk.chain import rpy_to_homogenous
from sympy.utilities.autowrap import autowrap
from kuka_kinematics.geometric_ik import KukaGeometricIK
from trajectory_msgs.msg import JointTrajectoryPoint
from kuka_arm.srv import CalculateIKResponse, CalculateIK


gflags.DEFINE_string('urdf', None, 'path to urdf file')
FLAGS = gflags.FLAGS


def compile_rpy_to_homogenous():
  rpy = sp.symbols('roll, pitch, yaw')
  xyz = sp.symbols('x, y, z')
  hm = rpy_to_homogenous(rpy, xyz, to_numeric=False)
  return autowrap(hm, args=rpy + xyz)


def create_state_dict(joint_state):
  names = joint_state.name
  positions = joint_state.position
  states = {n: p for n, p in zip(names, positions)}
  return states


def create_zero_states():
  return {'joint_1': 0, 'joint_2': 0, 'joint_3': 0,
          'joint_4': 0, 'joint_5': 0, 'joint_6': 0}


def subscribe_joint_states(callback):
  rospy.Subscriber('/joint_states', JointState, callback)


def rot_to_quaternion(frame):
  origin = frame[:3, -1]
  rot = frame[:3, :3]
  # noinspection PyUnresolvedReferences
  mat = np.eye(4)
  mat[:3, :3] = rot
  rotation = tf.transformations.quaternion_from_matrix(mat)
  return origin, rotation


def absolute_axis(axes, frames):
  assert len(axes) == len(frames)
  ret = []
  for a, f in zip(axes, frames):
    line = lg.Line(k=np.matmul(f[:3, :3], a), p=f[:3, -1])
    ret.append(line)
  return ret


def relative_frames(frames, dh_frames):
  ret = []
  for f, df in zip(frames, dh_frames):
    relative = np.matmul(np.linalg.inv(f), df)
    ret.append(relative)
  return ret


def to_joint_list(states):
  ret = []
  for i in range(1, 7):
    key = 'joint_{}'.format(i)
    ret.append(states[key])
  return ret


class KukaModel(object):

  def __init__(self):
    # get from ros ps
    if FLAGS.urdf is None:
      urdf = rospy.get_param('robot_description')
      self._parser = URDFParser(data=urdf)
    else:
      self._parser = URDFParser(path=FLAGS.urdf)

    self._joint_states = create_zero_states()
    self._dag = KinematicsDag(self._parser.joints)
    # Kuka major manipulator happens to be the first subgraph in DAG
    self._graph = self._dag[0].chain
    self._rpy_to_hm = compile_rpy_to_homogenous()
    self._ik = KukaGeometricIK(self._dag)
    base_name, base_mat = self._graph.dh_base_link
    base_p, base_q = rot_to_quaternion(base_mat)
    dh_base_name = 'dh-' + base_name
    self._rel_rots = \
      {base_name: (dh_base_name, base_p, base_q)}

    for link in self._graph.dh_links:
      link_name = link.link
      p, q = rot_to_quaternion(
        np.linalg.inv(link.rel))
      self._rel_rots[link_name] = \
        ('dh-' + link_name, p, q)

  def broadcast_relative_dh(self):
    br = tf.TransformBroadcaster()
    # noinspection PyTypeChecker
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
      for k, (dhk, p, q) in self._rel_rots.items():
        br.sendTransform(
          p, q, rospy.Time.now(), dhk, k)
      rate.sleep()

  def broadcast_dh(self):
    br = tf.TransformBroadcaster()

    def _broadcast_dh(joint_state):
      base_name, _ = self._graph.dh_base_link
      states = create_state_dict(joint_state)
      self.save_joint_state(states)
      frames = self._graph.forward(states)
      for k, f in frames:
        p, q = rot_to_quaternion(f)
        dh_k = 'dh-' + k
        br.sendTransform(
          p, q, rospy.Time.now(), dh_k, base_name)

    subscribe_joint_states(_broadcast_dh)

  def broadcast_urdf(self):
    br = tf.TransformBroadcaster()

    def _broadcast_urdf(joint_state):
      base_name, _ = self._graph.dh_base_link
      states = create_state_dict(joint_state)
      frames = self._graph.forward(states, return_dh=False)
      for k, f in frames:
        p, q = rot_to_quaternion(f)
        urdf_k = 'urdf-' + k
        br.sendTransform(
          p, q, rospy.Time.now(), urdf_k, base_name)
    subscribe_joint_states(_broadcast_urdf)

  @property
  def current_state(self):
    return self._joint_states.copy()

  def save_joint_state(self, states):
    self._joint_states = states

  def _handle_ik(self, request):
    iter_num = len(request.poses)
    if iter_num < 1:
      print('No Valid pose received', file=sys.stderr)
      return -1

    # states = self.current_state
    states = self.current_state
    ret = []
    for t in range(iter_num):
      pos = request.poses[t].position
      quat = request.poses[t].orientation
      rpy = tf.transformations.euler_from_quaternion(
        [quat.x, quat.y, quat.z, quat.w])

      eef = self._rpy_to_hm(
        rpy[0], rpy[1], rpy[2], pos.x, pos.y, pos.z)
      states = self._ik.infer_joints(
        current_state=states, eef=eef)
      point = JointTrajectoryPoint()
      point.positions = to_joint_list(states)
      ret.append(point)

    return CalculateIKResponse(ret)

  def ik_server(self):
    s = rospy.Service(
      'calculate_ik', CalculateIK, self._handle_ik)
    print('ik server started')
    return s
