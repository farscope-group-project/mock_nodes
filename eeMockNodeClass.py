#!/usr/bin/python
import roslib
import math
import rospy
import time
from std_msgs.msg import Bool
import random
from random import randint


class EEInterface():

    def __init__(self):
        #This function runs when the object is started

        #Local values of things it publishes
        self.vacuum_on = False
        self.item_held = False

        #Local values of things it subscribes to
        self.vacuum_dmd = False
	self.item_held_override = False

        # Publishers
        self.item_held_pub = rospy.Publisher("/ItemHeld", Bool, queue_size=1)
        self.vacuum_status_pub = rospy.Publisher("/VacuumStatus", Bool, queue_size=1)

        # Subscribers
        self.vacuum_toggle_sub = rospy.Subscriber("/GripItem", Bool, self.vacuum_toggle_callback)
        self.set_item_held_sub = rospy.Subscriber("/SetItemHeld", Bool, self.set_item_held_callback)
	

    def __enter__(self):
        #More things to run when object starts
        """ Set up """
        rospy.init_node('ee_mock_node', anonymous=True)
        self.rate = rospy.Rate(10)
        return self


    def __exit__(self, type, value, traceback):
        #Things to do when the object is closed (does nothing at the moment)
        """ Tear down """
        pass


    #Function to handle what happens when EE receives a command to turn on Vaccum
    def vacuum_toggle_callback(self, data):
        #Store a local copy of the demand signal for the vaccum status from the main controller
        self.vacuum_dmd = data.data
        

    #Function to handle what happens when EE receives a command to turn on Vaccum
    def set_item_held_callback(self, data):
        #Store a local copy of the demand signal for the vaccum status from the main controller
        self.item_held_override = data.data
        

    #Publishes the vacuum and item held status signals at a rate dictated by the main loop
    def publish_state_signals(self):
        msg = Bool()
        #publish the VacuumStatus
        msg.data = self.vacuum_on
        self.vacuum_status_pub.publish(msg)
        #publish itemheld VacuumStatus
        msg.data = self.item_held
        self.item_held_pub.publish(msg)


    #Function to either set item held via user or random number generator
    def update_grip_item_status(self, user_input):
        rospy.loginfo("Gripper command is=%s"%self.vacuum_dmd)
	if user_input == 0:  #Set via random # generator    	
		if  self.vacuum_on == True and self.item_held == False:
	    		ee_pressure_sensor = randint(0,100) # this should be replaced with the arduino link
	    		if ee_pressure_sensor > 80:
	    			self.item_held = True
	    		else:
	    			self.item_held = False
	    		rospy.loginfo("Actual reading=%s"%ee_pressure_sensor)
	    		rospy.loginfo("Gripping status (1 = I have item, 0 = I lost Item): = %s"%self.item_held)
		elif self.vacuum_on == False and self.item_held == True:
			ee_pressure_sensor = randint(0,100) # this should be replaced with the arduino link
	    		if ee_pressure_sensor > 80:
	    			self.item_held = False
	    		else:
	    			self.item_held = True
		elif self.vacuum_on == True and self.item_held == True:
			self.item_held = True
		elif self.vacuum_on == False and self.item_held == False:
			self.item_held = False
	else:  #Set via user
		self.item_held = self.item_held_override

    #function to simulate the spool up and spool down of the vacuum cleaner following a change in demand
    def update_vacuum_status(self):
        rospy.loginfo("Vacuum command is=%s"%self.vacuum_dmd)
        if self.vacuum_dmd <> self.vacuum_on: #there as been a change in demand
            ee_vacuum_status = randint(0,100) # this should be replaced with the arduino link
            if self.vacuum_dmd == True: #if the demand is true and therefore status is false
                if ee_vacuum_status < 70: # this value should be replaced with pressure threshold for on/off
                    self.vacuum_on = True
                else:
                    self.vacuum_on = False
            else: #if the demand is false and therefore status is true
                if ee_vacuum_status < 50: # this value should be replaced with pressure threshold for on/off
                    self.vacuum_on = False
                else:
                    self.vacuum_on = True
            rospy.loginfo("Vacuum (1 = ON, 0 = OFF): = %s"%self.vacuum_on)


    #The main loop for the object, runs this in a loop until ROS is shutdown or ctrl^c to close this object
    def main(self, user_input):  
        print("EE Node Working")
        while not rospy.is_shutdown():
            #Check if grip item status needs updating and update if it does
            self.update_grip_item_status(user_input)
            #Check if vacuum status needs updating and update if it does
            self.update_vacuum_status()
            #Publish it's state signals
            self.publish_state_signals()
            # Sleep for a bit
            self.rate.sleep()


if __name__ == "__main__":
    #Code to run the main loop when the object is created and handle exceptions if it can't start
    """ Run EEMockNode """
    user_input = -1
    while (user_input <> 0) and (user_input <> 1):
	user_input = int(input("For item held behaviour, 1 for user control, 0 for autonomated: "))
    try:
        with EEInterface() as ee:
            ee.main(user_input)
            rospy.spin()
    except rospy.ROSInterruptException:
        print("Exception!")
