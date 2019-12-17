import rospy 
from my_msgs.msg import Order

rospy.init_node("receive_order_example")


while True: # While loop represents your behaviours update loop

  try:
    order_msg = rospy.wait_for_message("/order", Order, 0.5) # Wait for 0.5 seconds to receive something

    print order_msg
    break
  except:
    pass # When nothing received, do nothing 