#!/usr/bin/python
import rospy
from std_msgs.msg import Bool
import random
from random import randint

# callback for pose does all the work
def callback(data):
	#rospy.loginfo('Hello from callback')
	rospy.loginfo("Vacuum command is=%s"%data.data)

def listener():
	# start the node
	rospy.init_node('tpc_vac_feedback', anonymous=True)
	# and the subscriber
	rospy.Subscriber("eeVacuumPower",Bool, callback)
	rospy.spin() #don't exit until node is stopped


if __name__ == '__main__':
	listener()
