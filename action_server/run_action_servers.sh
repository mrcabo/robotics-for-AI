#!/bin/bash

cd /home/group9/catkin_ws/src/action_server/scripts

python drop_action_server.py &
python grasp_action_server.py &
python move_backwards_action_server.py &
python classification_server.py &

cd /home/group9/catkin_ws/src/my_planner/scripts
python dijkstra_planning.py &

roslaunch navigation tiago_dijkstra.launch map:=very_map_much_wow &
roslaunch bounding_box_server bounding_box_server.launch robot:=tiago &