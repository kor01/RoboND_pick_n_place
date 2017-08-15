#!/usr/bin/env bash

export PYTHONPATH=/home/pu/PycharmProjects/pick_n_place/:${PYTHONPATH}
export PYTHONPATH=/opt/ros/kinetic/lib/python2.7/dist-packages/:${PYTHONPATH}
export PYTHONPATH=/home/pu/.app/ros-packages/:${PYTHONPATH}
export PATH=/home/pu/.app/python2.7/bin:$PATH

nohup jupyter-notebook --port=8888 > /dev/null &

