#!/usr/bin/python
import rospy
from std_msgs.msg import Bool
import random
from random import randint

# callback for pose does all the work
def callback(data):
	#rospy.loginfo('Hello from callback')
	rospy.loginfo("Gripper command is=%s"%data.data)
	if  data.data == True:
		#rospy.loginfo("It works")

		pub_eeGrip_Stat = rospy.Publisher('eeGripStat', Bool, queue_size=1)
		ee_pressure_sensor = randint(0,100) # this should be replaced with the arduino link

		if ee_pressure_sensor > 80: 

			ee_grip_status = 1
		else:
			ee_grip_status = 0
		rospy.loginfo("Actual reading=%s"%ee_pressure_sensor)
		rospy.loginfo("Gripping status (1 = I have item, 0 = I lost Item): = %s"%ee_grip_status)
		pub_eeGrip_Stat.publish(ee_grip_status)


def listener():
	# start the node
	rospy.init_node('ee_grip_stat', anonymous=True)
	# and the subscriber
	rospy.Subscriber("TPC/GripItem",Bool, callback)
	rospy.spin() #don't exit until node is stopped


if __name__ == '__main__':
	listener()


