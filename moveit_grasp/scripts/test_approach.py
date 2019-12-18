#!/usr/bin/python
import rospy
from approach.msg import *
import actionlib

rospy.init_node("test_approach")

approach_client = actionlib.SimpleActionClient("/approach_table", ApproachTableAction)

print "Waiting for approach action server"
approach_client.wait_for_server()
print "Connected to approach server"

goal_msg = ApproachTableGoal()
goal_msg.transform_to_link = "base_link"
goal_msg.min_z = 0.4
goal_msg.max_z = 1.5
goal_msg.distance_to_table = 0.45  # For tiago it should be 0.35

approach_client.send_goal(goal_msg)
approach_client.wait_for_result()
