import rospy
import tf
import gflags
import numpy as np
import sympy as sp
from manipulator_graph import frame_inference as fi
from manipulator_graph import URDFParser
from manipulator_graph import sort_joints
from manipulator_graph import line_geometry as lg

gflags.DEFINE_string('urdf', None, 'path to urdf file')

parser = URDFParser('/home/pu/notebooks/pick_n_place/kr210.urdf')
joints = [j for j in parser.joints if 'gripper' not in j.name]
joints = sort_joints(joints)
axis = [j.axis for j in joints]
position = [j.position for j in joints]


def homogenous_transform(rpy, origin):
  x_rot = sp.rot_axis1(rpy[0])
  y_rot = sp.rot_axis2(rpy[1])
  z_rot = sp.rot_axis3(rpy[2])
  origin = sp.Matrix(origin)
  rot = z_rot * y_rot * x_rot
  rot = rot.row_join(origin)
  rot = rot.col_join(sp.Matrix([[0, 0, 0, 1]]))
  rot = sp.simplify(rot)
  rot = np.array(rot.tolist(), dtype=np.float64)
  return rot


def inference_frames(js):
  frame = np.eye(4)
  ret = []
  for j in js:
    frame = np.matmul(frame, homogenous_transform(
      j.orientation, j.position))
    ret.append(frame)
  return ret


def absolute_axis(axes, frames):
  assert len(axes) == len(frames)
  ret = []
  for a, f in zip(axes, frames):
    line = lg.Line(k=np.matmul(f[:3, :3], a), p=f[:3, -1])
    ret.append(line)
  return ret

abs_frames = inference_frames(joints)
abs_axis = absolute_axis(axis[1:], abs_frames[1:])
abs_dh_frames = fi.assign_dh_frames(abs_axis)
abs_dh_frames = map(fi.dh_frame_to_homogenous,
                    abs_dh_frames)


def relative_frames(frames, dh_frames):
  ret = []
  for f, df in zip(frames, dh_frames):
    relative = np.matmul(np.linalg.inv(f), df)
    ret.append(relative)
  return ret


def rot_to_quaternion(frame):
  origin = frame[:3, -1]
  rot = frame[:3, :3]
  # noinspection PyUnresolvedReferences
  mat = np.eye(4)
  mat[:3, :3] = rot
  rotation = tf.transformations.quaternion_from_matrix(mat)
  return origin, rotation


class DHBroadCaster(object):
  """
  broadcast DH frames to debug with rviz
  """

  def __init__(self, frames):
    self._rots = {}
    for k, v in frames.items():
      self._rots[k] = rot_to_quaternion(v)

  # noinspection PyTypeChecker
  def broad_cast(self):
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
      for k, v in self._rots.items():
        dhk = 'dh-' + k
        p, q = v
        br.sendTransform(p, q, rospy.Time.now(), dhk, k)
      rate.sleep()

  def run(self):
    rospy.init_node('DHBroadCaster')
    param = rospy.get_param('robot_description')

    self.broad_cast()


if __name__ == '__main__':
  # prepare relative frames
  fs = relative_frames(abs_frames, abs_dh_frames)
  names = [x.child.name for x in joints]
  fs = {x: y for x, y in zip(names, fs)}
  broadcaster = DHBroadCaster(fs)
  broadcaster.run()
