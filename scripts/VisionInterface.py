#!/usr/bin/python
import rospy
from std_msgs.msg import Bool, String
from simulated.msg import *


class VisionInterface():

	def __init__(self):

	# This is the information outputted by the main controller which is sent to the vision system
		self.Camera_position = ControllerPacket() # This is the creation of a custom message type object
		self.Camera_position.McStatus = -1
		self.Camera_position.ItemName = 'Joke book'
		self.Camera_position.ShelfId = 0
		self.Camera_position.InterfaceJoint.position.x = 0
		self.Camera_position.InterfaceJoint.position.y = 0
		self.Camera_position.InterfaceJoint.position.z = 0
		self.Camera_position.InterfaceJoint.orientation.x = 0
		self.Camera_position.InterfaceJoint.orientation.y = 0
		self.Camera_position.InterfaceJoint.orientation.z = 0
		self.Camera_position.InterfaceJoint.orientation.w = 0


# This is the information RECEIVED by the main controller from the vision system
		self.item = VisionPacket()
		self.item.VisionStatus = -1
		self.item.ItemPose.position.x = 0
		self.item.ItemPose.position.y = 0
		self.item.ItemPose.position.z = 0
		self.item.ItemPose.orientation.x = 0
		self.item.ItemPose.orientation.y = 0
		self.item.ItemPose.orientation.z = 0
		self.item.ItemPose.orientation.w = 0
		self.item.ItemBoundingBox = [0,0,0]


		# Publisher
		self.vision_publisher = rospy.Publisher('/MainController',ControllerPacket,queue_size = 1)

		#Subscriber
		self.vision_subscriber = rospy.Subscriber('/VisionSystem', VisionPacket, self.visionCallback)

	def visionCallback(self,data): # This is used to update the values from the data obtained when it subscribes
		self.item.VisionStatus = data.VisionStatus
		self.item.ItemPose.position.x = data.ItemPose.position.x
		self.item.ItemPose.position.y = data.ItemPose.position.y
		self.item.ItemPose.position.z = data.ItemPose.position.z
		self.item.ItemPose.orientation.x = data.ItemPose.orientation.x
		self.item.ItemPose.orientation.y = data.ItemPose.orientation.y
		self.item.ItemPose.orientation.z = data.ItemPose.orientation.z
		self.item.ItemPose.orientation.w = data.ItemPose.orientation.w
		self.item.ItemBoundingBox = data.ItemBoundingBox
		#rospy.loginfo("I've recieved a new packet from vision, status recieved is")
		#rospy.loginfo(self.item.VisionStatus)

	def get_vision_status(self):
		return self.item.VisionStatus
		
	def publish_vision_packet(self): # This is used to publish the data
		self.vision_publisher.publish(self.Camera_position)

	def set_status(self,state): # This is used to set the status
		self.Camera_position.McStatus = state

	def set_desired_item(self,item): #This is used to set the specific item name
		self.Camera_position.ItemName = item
	
	def set_desired_shelf(self,shelf_string): #This is used to set the specific item name
		if shelf_string == "bin_A":
			self.Camera_position.ShelfId = 0
		elif shelf_string == "bin_B":
			self.Camera_position.ShelfId = 1
		elif shelf_string == "bin_C":
			self.Camera_position.ShelfId = 2
		elif shelf_string == "bin_D":
			self.Camera_position.ShelfId = 3
		elif shelf_string == "bin_E":
			self.Camera_position.ShelfId = 4
		elif shelf_string == "bin_F":
			self.Camera_position.ShelfId = 5
		elif shelf_string == "bin_G":
			self.Camera_position.ShelfId = 6
		elif shelf_string == "bin_H":
			self.Camera_position.ShelfId = 7
		elif shelf_string == "bin_I":
			self.Camera_position.ShelfId = 8
		elif shelf_string == "bin_J":
			self.Camera_position.ShelfId = 9
		elif shelf_string == "bin_K":
			self.Camera_position.ShelfId = 10
		else: #shelf_string == "bin_L":
			self.Camera_position.ShelfId = 11

	def set_interface_joint(self,x,y,z,qx,qy,qz,qw):  # This is used to update the interface joint
		self.Camera_position.InterfaceJoint.position.x = x
		self.Camera_position.InterfaceJoint.position.y = y
		self.Camera_position.InterfaceJoint.position.z = z
		self.Camera_position.InterfaceJoint.orientation.x = qx
		self.Camera_position.InterfaceJoint.orientation.y = qy
		self.Camera_position.InterfaceJoint.orientation.z = qz
		self.Camera_position.InterfaceJoint.orientation.w = qw

	"""
	def update_Controller_info(self): # Need to call this function in order to manually input values
	print("Select update")
	selected_value = input()
	while selected_value <10:
		if selected_value == 1:
			print("Please change status")
			self.Camera_position.McStatus = input()
		if selected_value == 2:
			print("Please change item name")
			self.Camera_position.ItemName = input()
		if selected_value == 3:
			print("Please change x")
			self.Camera_position.InterfaceJoint.position.x = input()
		if selected_value == 4:
			print("Please change y")
			self.Camera_position.InterfaceJoint.position.y = input()
		if selected_value == 5:
			print("Please change z")
			self.Camera_position.InterfaceJoint.position.z = input()
		if selected_value == 6:
			print("Please change qx")
			self.Camera_position.InterfaceJoint.orientation.x = input()
		if selected_value == 7:
			print("Please change qy")
			self.Camera_position.InterfaceJoint.orientation.y = input()
		if selected_value == 8:
			print("Please change qz")
			self.Camera_position.InterfaceJoint.orientation.z = input()
		if selected_value == 9:
			print("Please change qw")
			self.Camera_position.InterfaceJoint.orientation.w = input()
		print("Any additional changes")
		selected_value = input()
		"""
