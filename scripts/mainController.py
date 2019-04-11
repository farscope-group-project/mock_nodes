#!/usr/bin/env python
"""
	This Ros Node is the main controller of the UR10 Amazon Picking Challenge Stack
	It acts as both a publisher and subscriber to the various interfaces of the system


"""
import roslib
import math
import rospy
import time
from std_msgs.msg import String
from VisionInterfaceClass import VisionInterface
from EEInterface import EEInterface
from ArmInterface import ArmInterface
import tf
import tf2_ros
import tf2_geometry_msgs
import sys
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose
from moveit_commander.conversions import pose_to_list
import json

class StateMachine():

	def __init__(self):
		""" Initialisation code """
		# Initialise class variables
		self.node_name = "state_machine"
		self.state = 1
		self.state_mapper = {
			1: self.state_1_object_selection,
			2: self.state_2_vision_sweep,
			3: self.state_3_grasping,
			4: self.state_4_depositing,
			5: self.state_5_end_run,
			6: self.state_6_completion,
			7: self.state_7_done
		}

		self.num_failures = 0

		#stores the overall outcome from the current run, used to record result in JSON
		self.run_failure = False

		#The list of items to pick from
		self.item_final = {}

		#The list of picking results
		self.item_status = {}
		self.item_status['output'] = []

		#index to next item on list
		self.index = 0

		#current item being picked
		self.target_item = ''

		#setup the interfaces
		self.visionInterface = VisionInterface()
		self.EEInterface =  EEInterface()
		#self.armInterface = ArmInterface()

		# Topics to subscribe to
		#This is handled in the VisionInterfaceClass
		#self.sub1 = rospy.Subscriber('vision', String, self.vision_callback)

		# Topics State Machine Publishes
		self.pub_state_node_name = 'state_messages'
		self.pub_state_node = rospy.Publisher(self.pub_state_node_name, String, queue_size=10)

	def __enter__(self):
		""" Set up """
		rospy.init_node(self.node_name, anonymous=True)
		self.rate = rospy.Rate(10)
		return self

	def __exit__(self, type, value, traceback):
		""" Tear down """
		pass

	def controller(self):
		""" Main State machine control Loop """
		while not rospy.is_shutdown():

			# Test code
			hello_str = "hello world {}".format(rospy.get_time())
			rospy.loginfo(hello_str)
			self.pub_state_node.publish(hello_str)

			# State Machine, defined by state_mapper in initialisation, returns state function
			state_function_to_exec = self.state_mapper.get(self.state, lambda: "Invalid State")
			new_state = state_function_to_exec() # Function returns next state
			self.state = new_state

			if self.state == 7:
				rospy.loginfo("State 7!")
				print("Process Complete")
				break

			# Timings
			self.rate.sleep()

	def state_1_object_selection(self):
		""" Reads the ordered dictionary and selects the next object to grasp """
		# initialise the state and setup for a new run
		rospy.loginfo("State 1!")
		#self.armInterface.goHome() # Just to be sure
		self.num_failures = 0
		self.run_failure = False

		# Read JSON file (Be nice to only do this the first time through
		self.json_read()
		rospy.loginfo(self.index)

		# check if poiter exceeds length of list (not reached the last item)
		if (self.index < len(self.item_final)):
			#Get object name and shelf location
			self.target_item = list(self.item_final.keys())[self.index]
			#Goto state 2
			next_state = 2
		else:
			#reached end of list, goto state 5
			next_state = 6

		"""
			@return the next state. This will always be state 2 unless we have finished
		"""
		return next_state

	def state_2_vision_sweep(self):
		""" Coordinates between arm movements and vision system to perform the vision sweep necessary
		for the vision system.

			@return the next state. This may be state 3 for success on receipt of item pose or 1 for
			recognition failure.
		"""
		rospy.loginfo("State 2!")

		#move to bin home position for shelf location
		#self.armInterface.gotoBin("F")

		#******do the vision sweep*****

		# get the current EE position and pose
		#original_rpy = self.armInterface.move_group.get_current_rpy()
		#original_pose =  self.armInterface.move_group.get_current_pose().pose

		#Initialise the vision System
		#set main controller status = 0;
		self.visionInterface.set_status(0)
		self.visionInterface.set_desired_item(self.target_item)
		#publish main controller status and item name
		self.visionInterface.publish_vision_packet()
		#wait until vision system responds it's initialised
		while self.visionInterface.get_vision_status() != 0:
			#do nothing
			rospy.sleep(2)
			rospy.loginfo(self.visionInterface.get_vision_status())
		# Now create a loop to send signal to arm for position in sweep
		# There are 16 positions numbered 1 to 16
		# After each position send correspondign signal to Vision System
		for i in range(1,16):
			self.visionInterface.set_status(i)
			self.visionInterface.publish_vision_packet()
			#wait until vision status matches i or 16 (may also want to add a timeout for error handling), wait
			vision_status = self.visionInterface.get_vision_status()
			while  vision_status!= i or vision_status!=16:
				#self.visionInterface.publish_vision_packet()
				rospy.sleep(2)
				vision_status = self.visionInterface.get_vision_status()
		#loop to next i

		#if vision status == 16 AND i!=16 then an error has occured
		#log failure ****BUG HERE, HOW DOES VISION SYSTEM SIGNAL A FAULT HAS OCCURED DURING RECOGNITION WHEN CAMERA IS IN POSITION 16???
		if not (vision_status == 16 and i!=16):
			self.run_failure = True
			return 5
		else:
			#Store the item location, pose and bounding box from the vision system and make available to other states
			#Thiis is handled in the vision interface class
			return 3


	def state_3_grasping(self):
		""" Coordinates between arm movements and end effector to perform the grasping motion to pick up
		a specified item.

			@return the next state. This may be state 4 on sucess, state 3 on first failure, state 2 on
			second failure, or state 1 on complete failure
		"""
		#Initialise the state
		rospy.loginfo("State 3!")
		failure = False
		# self.EEInterface.toggle_vacuum(True)

		#time.sleep(5)

		#Move arm to the item location (including any offset needed)
		# get the current EE position and pose
		#original_rpy = self.armInterface.move_group.get_current_rpy()
		#original_pose = self.armInterface.move_group.get_current_pose().pose
		#self.armInterface.incHeight(0.12, original_rpy, original_pose)


		#Turn on the vacuum cleaner
		self.EEInterface.toggle_vacuum(True)

		#Wait for 5seconds (TBC) for vacuum to spool up
		time.sleep(5)

		#Check if vacuum has turned on, if not flag and error and increase the error count"
		if not self.EEInterface.get_vacuum_status():
			failure = True
			self.num_failures = self.num_failures + 1

		if failure == False:
			#move arm to object
			#self.armInterface.gotoObject(0,0,0,0,0,0) #(x,y,z,aplha,belta,gamma) # This might have to hand over a quaternion instead of RPY
			#rospy.loginfo("going down")
			#self.armInterface.goDownInShelf(0,0,0,0,0,0)

			#If item is not held, wiggle position and try again
			if self.EEInterface.get_item_held_status()==False:
				rospy.loginfo("Wiggle!")
				#original_rpy = self.armInterface.move_group.get_current_rpy()
				#original_pose = self.armInterface.move_group.get_current_pose().pose
				#self.armInterface.wiggle(0.03,original_rpy, original_pose)
				#self.armInterface.wiggle(-0.06,original_rpy, original_pose)
				#self.armInterface.wiggle(0.03,original_rpy, original_pose)
				#self.armInterface.goUpInShelf(0,0,0,0,0,0)
			#move out of shelf to shelf bin home position
			#original_rpy = self.armInterface.move_group.get_current_rpy()
			#original_pose = self.armInterface.move_group.get_current_pose().pose
			#self.armInterface.incDepth(0.05064, original_rpy, original_pose)
			#rospy.sleep(4)
			#Check is item held after any wiggles, if no, not add one to failure count
			#if yes, goto state 4 (drop item)
			if self.EEInterface.get_item_held_status()==False:
				failure = True
				self.num_failures = self.num_failures + 1

		#decide which state to go into next
		#one failure, retry picking up object from starting position for the shelf bin
		#two failures, retry the vision sweep
		#more than two failures, give up and log error
		if failure == True:
			if self.num_failures is 1:
				next_state = 3
			elif self.num_failures is 2:
				next_state =  2
			else:
				self.run_failure = True
				next_state =  1
		else:
			next_state = 4

		return next_state


	def state_4_depositing(self):
		""" Coordination between arm movements and end effector to move picked up item from in front of
		shelf to above box to deposit the item safely.

			@return the next state. This may be 1 on sucess, and also 1 on failure
		"""

		#Initialise the state
		rospy.loginfo("State 4!")

		#Check item still held, if not, set item failure and go back to state 5 to log failure to file
		if self.EEInterface.get_item_held_status() == False:
			 # record item dropped
			 self.run_failure = True
			 # turn off vaccum cleaner
			 self.EEInterface.toggle_vacuum(False)
			 # Log failure and goto end of run
			 return 5

		#move arm to box
		#self.armInterface.gotoBox()

		#time.sleep(5)

		#Check item still held, if not, set item failure and go back to state 5 to log failure to file
		if self.EEInterface.get_item_held_status() == False:
			 # record item dropped
			 self.run_failure = True
			 # turn off vaccum cleaner
			 self.EEInterface.toggle_vacuum(False)
			 # goto state 5
			 return 5

		#Turn off vaccuum
		self.EEInterface.toggle_vacuum(False)

		#Wait for 8sec (TBC) for vacuum to spool down. - 2sec for a quick simulation!
		rospy.sleep(2)

		# assumed item has dropped!!

		#goto end of run
		return 5

	def state_5_end_run(self):
		"""Tasks to do at the end of attempting to pick and deposit an item"""

		#record outcome of run
		if self.run_failure:
			success = False
		else:
			success = True
		self.json_update(self.target_item, success)

		# Update pointer to next object on list
		self.index = self.index + 1

		# move to home position
		#self.armInterface.goHomeFromBox()

		return 1


	def state_6_completion(self):
		""" Tasks to do upon completion, writes pickup output to file """
		with open('picked_status_single.json', 'w') as outfile:
			json.dump(self.item_status, outfile)
		rospy.loginfo("Results logged to file")

		return 7

	def state_7_done(self):
		""" Holding state to enter into and wait until ros node shuts down or ctrl^c """

		return 7

	def vision_callback(self):
		#Callback for the vision messages
		pass

	def json_read(self):
		"""Reads in a json file and formats as list

			@return sets the internal variable item_final as the list to pick from
		"""
		# Read json file. Use ITEM_FINAL to actually select which item!
		with open("apc_seeded_2018.json") as json_file:
			json_data = json.load(json_file)
			bin_list = json_data['bin_contents']

		# Delete items in shelves that are impossible to reach
		del bin_list["bin_J"]
		del bin_list["bin_K"]
		del bin_list["bin_L"]

		# Initiate intermediate dictionary
		order_items = {}
		# Make item key, assign value to bin location
		for key, value in bin_list.items():
			for i in value:
				item_in_bin = i
				order_items[i] = key

		# Initiate dictionary to select from
		#	item_final = {} #move to an internal variable, initalised when object created

		# Delete items that we can't pick up
		for key,value in order_items.items():
			if (key == "dr_browns_bottle_brush") \
				or (key == "kong_air_dog_squeakair_tennis_ball")\
				or (key == "first_years_take_and_toss_straw_cup")\
				or (key == "highland_6539_self_stick_notes"):
				pass
			else:
				self.item_final[key] = value
		pass

	def json_update(self, target_item, result):
		self.item_status['output'].append({
		target_item : result
		})
		pass



if __name__ == "__main__":
	""" Run RosNode """
	try:
		with StateMachine() as s:
			s.controller()
			rospy.spin()
	except rospy.ROSInterruptException:
		print("Exception!")
