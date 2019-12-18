#!/bin/bash

cd /home/group9/catkin_ws/src/action_server/scripts

python drop_action_server.py &
python grasp_action_server.py &
python move_backwards_action_server.py &
python classification_server.py &