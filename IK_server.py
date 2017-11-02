#!/usr/bin/env python

import sys
import os

DIR_NAME = os.path.dirname(__file__)
DIR_NAME = os.path.realpath(DIR_NAME)
sys.path.append(DIR_NAME)

import rospy
import gflags

from kuka_kinematics.kuka_model import KukaModel

gflags.DEFINE_boolean('dh_relative', False,
                      'broadcast DH frame '
                      'relative to urdf frames, for debug only')
gflags.DEFINE_boolean('ik_server', True, 'start an IK server')
gflags.DEFINE_string('frame_type', 'dh',
                     'dh or urdf, urdf for debug purpose')
FLAGS = gflags.FLAGS


if __name__ == '__main__':
  FLAGS(sys.argv)
  # all the important FK and IR are implemented in KukaModel class
  # plz check the code in kuka_kinematics/kuka_model.py
  model = KukaModel()
  rospy.init_node('KuKaKinematics')

  # start a debug publisher, publish internal dh frames to RVIZ
  if FLAGS.dh_relative:
    model.broadcast_relative_dh()
  elif FLAGS.frame_type == 'dh':
    model.broadcast_dh()
  else:
    model.broadcast_urdf()

  # start inverse kinematics server
  if FLAGS.ik_server:
    model.ik_server()

  rospy.spin()
