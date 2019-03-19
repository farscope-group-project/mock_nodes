#!/usr/bin/python
import rospy
from std_msgs.msg import Bool


# callback for pose does all the work
def callback(data):
	#rospy.loginfo('Hello from callback')
	rospy.loginfo("Gripping status is=%s"%data.data)

def listener():
	# start the node
	rospy.init_node('tpc_grip_feedback', anonymous=True)
	# and the subscriber
	rospy.Subscriber("eeGripStat",Bool, callback)
	rospy.spin() #don't exit until node is stopped


if __name__ == '__main__':
	listener()
