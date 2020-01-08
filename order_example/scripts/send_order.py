import rospy
from my_msgs.msg import Order

rospy.init_node("send_order_example")
order_publisher = rospy.Publisher("/order", Order, queue_size=1)
rospy.sleep(2) # Give ros some time to initalize and register the publisher

order_msg = Order()
# All the objects
# order_msg.objects = ["evergreen", "raspberry", "jetson", "crazyflie", "eraser", "whitebox", "powerbank"]
order_msg.objects = ["whitebox", "eraser"]
# Both tables
order_msg.tables = ["table1", "table2"]
# order_msg.tables = ["table2"]

order_publisher.publish(order_msg)