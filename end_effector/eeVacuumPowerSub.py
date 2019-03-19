#!/usr/bin/python
import rospy
from std_msgs.msg import Bool
import random
from random import randint

# callback for pose does all the work
def callback(data):
	#rospy.loginfo('Hello from callback')
	rospy.loginfo("Vacuum command is=%s"%data.data)
	if  data.data == True:
		#rospy.loginfo("It works")

		pub_eeVac_Stat = rospy.Publisher('eeVacuumPower', Bool, queue_size=1)
		ee_vacuum_status = randint(0,100) # this should be replaced with the arduino link

		if ee_vacuum_status < 50 : # this value should be replaced with pressure threshold for on/off
			vacuum_status = 1
		else:
			vacuum_status = 0
		rospy.loginfo("Vacuum (1 = ON, 0 = OFF): = %s"%vacuum_status)
		pub_eeVac_Stat.publish(vacuum_status)


def listener():
	# start the node
	rospy.init_node('ee_vac_stat', anonymous=True)
	# and the subscriber
	rospy.Subscriber("TPC/VacuumPower",Bool, callback)
	rospy.spin() #don't exit until node is stopped


if __name__ == '__main__':
	listener()


