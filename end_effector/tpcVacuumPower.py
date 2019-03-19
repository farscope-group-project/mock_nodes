#!/usr/bin/python

import rospy
from std_msgs.msg import Bool

import random
from random import randint

def TPC_Vacuum_Status():
	#set up publisher
	pub_TPCVacuumStatus = rospy.Publisher('TPC/VacuumPower',Bool, queue_size=1)
	#start the node
	rospy.init_node('tpc_vac_stat')
	
	while not rospy.is_shutdown():
		#initiate boolean = true
		vacuum_status = 1
		rospy.loginfo("Actual reading=%s"%vacuum_status)
		pub_TPCVacuumStatus.publish(vacuum_status)
		#update at 1Hz
		r = rospy.Rate(1)
		r.sleep()

##### run from here ####

TPC_Vacuum_Status()
