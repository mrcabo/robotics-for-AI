import rospy
import actionlib
from my_msgs.msg import ExampleAction, ExampleResult, ExampleGoal


rospy.init_node('client_example')
client = actionlib.SimpleActionClient('example_server', ExampleAction)

print 'Connecting to server'
client.wait_for_server()
print 'Connected to the server'

goal = ExampleGoal()
goal.number = '10'
client.send_goal(goal)
client.wait_for_result(rospy.Duration(10))

if (client.get_state() == actionlib.GoalStatus.SUCCEEDED):
    result = client.get_result()
    print 'It succeeded with result: ', result.value
elif (client.get_state() == actionlib.GoalStatus.ABORTED):
    result = client.get_result()
    print 'It faled with result: ', result.value