#!/bin/bash
cd /host/ros
source devel/setup.bash
pip install -r requirements.txt
export PYTHONPATH=/host/ros/src:$PYTHONPATH
roslaunch launch/sim.launch
