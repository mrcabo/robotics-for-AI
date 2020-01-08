#!/bin/bash


SRCPATH="/home/group9/catkin_ws/src"

cd $SRCPATH/action_server/scripts

# Our action servers
python drop_action_server.py &
python grasp_action_server.py &
python move_backwards_action_server.py &
python classification_server.py & # Change the model

# Bounding box server
roslaunch bounding_box_server bounding_box_server.launch robot:=tiago &
roslaunch navigation tiago_dijkstra.launch map:=tiago_real_map &

cd $SRCPATH/my_planner/scripts
python dijkstra_planning.py &


# Main behaviour
roslaunch behaviours run_behaviour.launch file:=demo &