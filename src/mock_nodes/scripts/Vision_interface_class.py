#!/usr/bin/python
import rospy
from std_msgs.msg import Bool, String
from mock_nodes.msg import *  

class mock_vision_node():

	def __init__(self):
		self.node_name = "vision_system"
		rospy.init_node(self.node_name, anonymous=True)
		self.rate(10)
		self.sub = rospy.Subscriber(\camera_positions, ControllerPacket, self.controllerCallback)
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
			self.item.VisionStatus = 16 
			self.item.ItemPose.position.x = 1
			self.item.ItemPose.position.y = 2
			self.item.ItemPose.position.z = 3
			self.item.ItemPose.orientation.x = 4
			self.item.ItemPose.orientation.y = 5
			self.item.ItemPose.orientation.z = 6
			self.item.ItemPose.orientation.w = 7
			self.item.ItemBoundingBox = [8,9,10]
			
		self.pub_state.publish(self.item)
		self.item.VisionStatus = 0
	
				
if __name__ == "__main__":		
	 """ Run RosNode """
vs =  mock_vision_node()
	while not rospy.is_shutdown():
		rospy.spin()
	
	 	
				
			
			

		
