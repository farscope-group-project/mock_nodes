#!/usr/bin/python

import rospy
from std_msgs.msg import Bool


def TPC_Grip_Item():
	#set up publisher
	pub_TPCGripItem = rospy.Publisher('TPC/GripItem',Bool, queue_size=1)
	#start the node
	rospy.init_node('tpc_grip_item')
	
	while not rospy.is_shutdown():
		#initiate boolean = true
		grip_item = 1
		rospy.loginfo("Actual reading=%s"%grip_item)
		pub_TPCGripItem.publish(grip_item)
		#update at 1Hz
		r = rospy.Rate(1)
		r.sleep()

##### run from here ####

TPC_Grip_Item()
