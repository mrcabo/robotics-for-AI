import rospy
from base_control_server.srv import *
import math

rospy.init_node("testing_base_control_server")

move_base = rospy.ServiceProxy("/move_base", MoveBase)

move_base_req_msg = MoveBaseRequest()
move_base_req_msg.meters = -0.4
move_base(move_base_req_msg)