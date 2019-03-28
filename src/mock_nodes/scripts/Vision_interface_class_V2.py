#!/usr/bin/python
import rospy
from std_msgs.msg import Bool, String
from mock_nodes.msg import *  

class mock_vision_node():

	def __init__(self):
		self.node_name = "vision_system"
		rospy.init_node(self.node_name, anonymous=True)
		self.rate(10)
		self.sub = rospy.Subscriber('\camera_positions', ControllerPacket, self.controllerCallback)
		self.pub_state = rospy.Publisher('\object_position', VisionPacket,queue_size =10)
		
		# This is what the vision system is sending (publishing) to the controller
		self.item = VisionPacket()
		self.item.VisionStatus = 0 
		self.item.ItemPose.position.x = 0
		self.item.ItemPose.position.y = 0
		self.item.ItemPose.position.z = 0
		self.item.ItemPose.orientation.x = 0
		self.item.ItemPose.orientation.y = 0
		self.item.ItemPose.orientation.z = 0
		self.item.ItemPose.orientation.w = 0
		self.item.ItemBoundingBox = [0,0,0]
		
		

 

		
	def controllerCallback(self,data):
		controller_status = data.McStatus 
		if controller_status < 16:
			self.item.VisionStatus =  controller_status
		else:
			self.item.VisionStatus = 0 #May need to change this take into account error cases
		update_Vision_into()	# Used to manually update different information
			''' 
			self.item.ItemPose.position.x = 1
			self.item.ItemPose.position.y = 2
			self.item.ItemPose.position.z = 3
			self.item.ItemPose.orientation.x = 4
			self.item.ItemPose.orientation.y = 5
			self.item.ItemPose.orientation.z = 6
			self.item.ItemPose.orientation.w = 7
			self.item.ItemBoundingBox = [8,9,10]
			'''
			
		
			
		self.pub_state.publish(self.item)
		
	def update_Vision_info(self):
	print("Select update")
	selected_value = input()
	while selected_value <10:
		if selected_value == 1:
			print("Please change status")
			self.item.VisionStatus = input()
		if selected_value == 2:
			print("Please change x")
			self.item.ItemPose.position.x = input()	
		if selected_value == 3:
			print("Please change y")
			self.item.ItemPose.position.y = input()
		if selected_value == 4:
			print("Please change z")
			self.item.ItemPose.position.z = input()
		if selected_value == 5:
			print("Please change qx")
			self.item.ItemPose.orientation.x = input()
		if selected_value == 6:
			print("Please change qy")
			self.item.ItemPose.orientation.y = input()
		if selected_value == 7:
			print("Please change qz")
			self.item.ItemPose.orientation.z = input()
		if selected_value == 8:
			print("Please change qw")
			self.item.ItemPose.orientation.w = input()
		if selected_value == 9:
			print("Please change bounding box")
			Boundingbox = raw_input().split(",")
			Boundingbox = [float(i) for i in Boundingbox] 
			
			self.item.ItemBoundingBox = Boundingbox 	
		print("Any additional changes")
		selected_value = input()	
				
if __name__ == "__main__":		
	 """ Run RosNode """
vs =  mock_vision_node()
	while not rospy.is_shutdown():
		rospy.spin()
