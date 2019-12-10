import rospy

from base_control_server.srv import MoveBaseRequest, MoveBase
from bounding_box_server.msg import BoundingBoxes
from head_controller.srv import MoveHead
from std_srvs.srv import Empty
from moveit import MoveIt


def filter_bounding_boxes(bounding_boxes):
    min_volume = 0.00005
    max_volume = 0.005
    min_z = 0.5

    # print [(b.length, b.height, b.width,) for b in bounding_boxes]
    # print [b.length * b.height * b.width for b in bounding_boxes]

    bounding_boxes = filter(lambda b: min_volume < b.length * b.height * b.width < max_volume, bounding_boxes)
    bounding_boxes = filter(lambda b: b.z > min_z, bounding_boxes)

    return bounding_boxes


def call_service(service_name, action_class=Empty, *args, **kwargs):
    rospy.wait_for_service(service_name)
    try:
        handler = rospy.ServiceProxy(service_name, action_class)
        handler(*args, **kwargs)
        return True
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
        return False


def switch_bounding_box_publisher(on=True):
    return call_service('enable_bounding_box_publisher' if on else 'disable_bounding_box_publisher')


def get_bounding_boxes(filter_out_bad_boxes=True):
    switch_bounding_box_publisher(True)
    try:
        message = rospy.wait_for_message('/bounding_boxes', BoundingBoxes, 3.0)
    finally:
        switch_bounding_box_publisher(False)

    return filter_bounding_boxes(message.bounding_boxes) if filter_out_bad_boxes else message.bounding_boxes


def add_bounding_box_to_octomap(move_it, bounding_box):
    # Add a precise collision box to the octomap
    move_it.add_collision_object(
        x=bounding_box.x,
        y=bounding_box.y,
        z=bounding_box.z,
        rotation=bounding_box.yaw,
        box_size=[bounding_box.length, bounding_box.width, bounding_box.height]
    )


def move_head(pitch, yaw):
    return call_service('move_head', MoveHead, pitch, yaw)


def prepare_for_grasp(move_it):
    call_service('clear_octomap')
    # rospy.wait_for_service('clear_octomap')
    move_head(.6, 0)
    move_head(.6, -1)
    move_head(.6, 1)
    move_head(.6, 0)
    move_it.tiago_move_to_side_position()
