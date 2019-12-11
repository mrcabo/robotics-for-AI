import rospy
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import MoveItErrorCodes, Grasp, GripperTranslation
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
import tf
import math
import numpy as np

class MoveIt(object):

  def __init__(self, robot_name = "alice"):
    self.robot_name = robot_name.lower() # Capitalization doesn't matter

    if self.robot_name != "alice" and self.robot_name != "tiago":
      print "Robot {0} is not know, either use 'alice' or 'tiago'".format(self.robot_name)
      exit()

    moveit_commander.roscpp_initialize(sys.argv)
    self.scene = PlanningSceneInterface()
    self.clear_octomap = rospy.ServiceProxy("/clear_octomap", Empty) 

    self.arm = MoveGroupCommander("arm")
    self.gripper = MoveGroupCommander("gripper")

    self.arm.set_planner_id("RRTConnectkConfigDefault") # Should already be default 
    print "End effector link used:", self.arm.get_end_effector_link()

    self.arm.allow_replanning(True)
    self.arm.set_planning_time(10)

    self.transformer = tf.TransformListener()
    self.object_name = "object"
    rospy.sleep(2) # Allow some time for initialization of MoveIt
    
    if self.robot_name == "tiago":
      self.arm.set_end_effector_link("gripper_grasp_position")
    
    self.end_effector_link = self.arm.get_end_effector_link()

  def __del__(self):
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)

  def add_collision_object(self, x, y, z, rotation, box_size = [0, 0, 0]):
    # In case the object is still in the scene, remove it
    self.scene.remove_world_object(self.object_name)  
    self.scene.remove_attached_object(self.end_effector_link, self.object_name)
    rospy.sleep(1) # Making sure the object is removed

    q = quaternion_from_euler(math.pi, 0.0, rotation)
    # Creat a PoseStamped message, for the header.frame_id use "base_link"
    
    # use self.scene to add the collision box to the scene
    
    rospy.sleep(1.0) # Give it time to recreate the OctoMap
    
  def _open_gripper(self):
    joint_trajectory = JointTrajectory()
    joint_trajectory.header.stamp = rospy.get_rostime()

    if self.robot_name == "tiago":
      joint_trajectory.joint_names = ["gripper_left_finger_joint", "gripper_right_finger_joint"]
    else:
      joint_trajectory.joint_names = ["m1n6s200_joint_finger_1", "m1n6s200_joint_finger_2"]

    joint_trajectory_point = JointTrajectoryPoint()

    if self.robot_name == "tiago":
      joint_trajectory_point.positions = [0.04, 0.04]
    else:
      joint_trajectory_point.positions = [0, 0]

    joint_trajectory_point.time_from_start = rospy.Duration(5.0)
  
    joint_trajectory.points.append(joint_trajectory_point)
    return joint_trajectory

  def _close_gripper(self, width = None):
    joint_trajectory = JointTrajectory()
    joint_trajectory.header.stamp = rospy.get_rostime()

    if self.robot_name =="tiago":
      joint_trajectory.joint_names = ["gripper_left_finger_joint", "gripper_right_finger_joint"]
    else:
      joint_trajectory.joint_names = ["m1n6s200_joint_finger_1", "m1n6s200_joint_finger_2"]

    joint_trajectory_point = JointTrajectoryPoint()
    
    if not width == None:
        joint_trajectory_point.positions = [width/2, width/2]
    else:
        joint_trajectory_point.positions = [1.3, 1.3]
        
    joint_trajectory_point.time_from_start = rospy.Duration(5.0)

    joint_trajectory.points.append(joint_trajectory_point)
    return joint_trajectory
    
  def _make_gripper_translation_approach(self, min_distance = 0.09, desired_distance = 0.11):
    gripper_translation = GripperTranslation()
    gripper_translation.direction.vector.x = 0.0
    gripper_translation.direction.vector.y = 0.0
    gripper_translation.direction.vector.z = 0.0

    if self.robot_name == "tiago":
      gripper_translation.direction.vector.x = 1.0
    else:
      gripper_translation.direction.vector.z = 1.0

    gripper_translation.direction.header.frame_id = self.end_effector_link
    gripper_translation.min_distance = min_distance
    gripper_translation.desired_distance = desired_distance

    return gripper_translation

  def _make_gripper_translation_retreat(self, min_distance = 0.05, desired_distance = 0.08):
    gripper_translation = GripperTranslation()
    gripper_translation.direction.vector.z = 1.0
    gripper_translation.direction.header.frame_id = "base_link"
    gripper_translation.min_distance = min_distance
    gripper_translation.desired_distance = desired_distance
    
    return gripper_translation

  def _create_grasps(self, x, y, z, rotation, z_max, width = None):
    grasps = [] # append Grasp messages to this list
    
    # The correct orientation for each robot, such that it can grasp from above with a yaw rotation
    if self.robot_name == "tiago":
      q = quaternion_from_euler(0, math.pi/2, rotation)
    else:
      q = quaternion_from_euler(0, math.pi, rotation + math.pi)
    
    # Create different Grasp messages
    # For the grasp_pose.header.frame_id use "base_link"

    print "Nr of grasps:", len(grasps)
    return grasps

  def grasp(self, x, y, z, rotation, z_max, width=None):
    grasps = self._create_grasps(x, y, z, rotation, z_max, width=width)

    if len(grasps) == 0:
      print "Can not create grasps"
      return False
    
    result = self.arm.pick("object", grasps)

    if result == MoveItErrorCodes.SUCCESS:
      print "Success"
      return True
    else:
      print "Failed"
      return False

  def open_fingers(self):

    if self.robot_name == "tiago":
      self.gripper.set_joint_value_target([0.04, 0.04])
    else:
      self.gripper.set_joint_value_target([0, 0])

    self.gripper.go(wait=True)
    rospy.sleep(2.0)

  def close_fingers(self):
    
    if self.robot_name == "tiago":
      self.gripper.set_joint_value_target([0.004, 0.004])
    else:
      self.gripper.set_joint_value_target([1.3, 1.3])

    self.gripper.go(wait=True)
    rospy.sleep(2.0)
    
  def move_to_drop(self, x, y, z, rotation = -math.pi/2):
    poses = []

    for roll in np.arange(-math.pi/2, math.pi/2, math.radians(1)):

      if self.robot_name == "tiago":
        if roll < 0:
          continue
        q = quaternion_from_euler(0, math.pi/2 + roll, rotation)
      else:
        q = quaternion_from_euler(0, math.pi + roll, rotation + math.pi)

      for height in np.arange(z, z+0.3, 0.01): 
        pose = [x, y, height, q[0], q[1], q[2], q[3]]
        poses.append(pose)

    self.arm.set_pose_reference_frame("base_link")
    self.arm.set_pose_targets(poses, self.end_effector_link)
    plan = self.arm.plan()

    result = self.arm.go(wait=True)
    self.arm.stop()
    self.arm.clear_pose_targets()
    return result

  def move_to(self, x, y, z, rotation):
      if self.robot_name == "tiago":
        q = quaternion_from_euler(0, math.pi/2, rotation)
      else:
        q = quaternion_from_euler(0.0, math.pi, rotation + math.pi)
      
      pose = PoseStamped()
      pose.header.frame_id = "base_link"
      pose.pose.position.x = x
      pose.pose.position.y = y
      pose.pose.position.z = z
      pose.pose.orientation.x = q[0]
      pose.pose.orientation.y = q[1]
      pose.pose.orientation.z = q[2]
      pose.pose.orientation.w = q[3]

      self.arm.set_pose_target(pose, self.end_effector_link)
      plan = self.arm.plan()

      result = self.arm.go(wait=True)
      self.arm.stop()
      self.arm.clear_pose_targets()
      return result


  def move_joints(self, joint_position):
    self.arm.set_joint_value_target(joint_position)
    plan = self.arm.plan()
    return self.arm.go(wait=True)

  def tiago_move_to_nav_position(self):

    joint_position = [1.21815663190754, -0.3101285880528266, -1.666504666674431, 1.8833762301963013, -1.8310526753354068, 1.5179019342751836, 1.7070040761740843]
    result = self.move_joints(joint_position)

    if not result:
      return False

    joint_position = [0.560437389213778, -1.2417208647320237, -0.8233874397577248, 1.9287658245705757, -1.452499762553149, 1.1231782793676652, 1.807815749861069]
    result = self.move_joints(joint_position)

    if not result:
      return False

    joint_position = [0.3958347515368432, -1.29970719614888, -0.2321235663932102, 1.9548790353816123, -1.491395407355065, 1.1053963902297586, 1.3836814165153362]
    result = self.move_joints(joint_position)

    if not result:
      return False

    return True

  def tiago_move_to_side_position(self):
    joint_position = [0.1557893900857792, 1.0001510647387093, 1.1711949656216172, 1.592809800683023, 2.074395464296499, -1.4498072401510553, 0.660501527149556]
    return self.move_joints(joint_position)

  def alice_move_to_nav_position(self):
    joint_position = [2.002189804287326, 3.308477530858763, 0.9507020403543134, 6.109938651883328, 0.8985650192069583, 3.3495925735780903]
    return self.move_joints(joint_position)

  def print_position(self):
    pose = self.arm.get_current_pose()
    self.transformer.waitForTransform(self.end_effector_link, "base_link", rospy.Time.now(), rospy.Duration(5.0))
    eef_pose = self.transformer.transformPose("base_link", pose)

    orientation = eef_pose.pose.orientation
    orientation = [orientation.x, orientation.y, orientation.z, orientation.w]

    euler = euler_from_quaternion(orientation)

    print "x:", eef_pose.pose.position.x
    print "y:", eef_pose.pose.position.y
    print "z:", eef_pose.pose.position.z
    print "euler:", euler
    print self.end_effector_link
