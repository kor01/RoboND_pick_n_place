import rospy
import sys
import gflags
from kuka_kinematics.kuka_model import KukaModel

gflags.DEFINE_boolean('dh_relative', False,
                      'broadcast DH frame '
                      'relative to urdf frames, for debug only')
gflags.DEFINE_boolean('ik_server', False, 'start an IK server')
gflags.DEFINE_string('frame_type', 'dh',
                     'dh or urdf, urdf for debug purpose')
FLAGS = gflags.FLAGS


if __name__ == '__main__':
  FLAGS(sys.argv)
  model = KukaModel()
  rospy.init_node('KuKaKinematics')

  if FLAGS.dh_relative:
    model.broadcast_relative_dh()
  elif FLAGS.frame_type == 'dh':
    model.broadcast_dh()
  else:
    model.broadcast_urdf()

  if FLAGS.ik_server:
    model.ik_server()

  rospy.spin()
