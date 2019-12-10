from moveit import MoveIt
import rospy

rospy.init_node("test")
moveit = MoveIt("tiago")

moveit.tiago_move_to_nav_position()