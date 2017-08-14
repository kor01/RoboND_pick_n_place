#!/usr/bin/env bash

export PATH=/home/pu/.app/python2.7/bin/:$PATH
export PYTHONPATH=${PYTHONPATH}:/home/pu/PycharmProjects/pick_n_place
export PYTHONPATH=${PYTHONPATH}:/home/pu/.app/ros-packages
python -m kuka_kinematics --ik_server

