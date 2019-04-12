#!/usr/bin/python
import rospy
from std_msgs.msg import Bool, String
#from ControllerPacket import *
#from VisionPacket import *
from simulated.msg import *

class mock_vision_node():

	def __init__(self):
		self.node_name = "vision_system"
		rospy.init_node(self.node_name, anonymous=True)
		self.rate = rospy.Rate(10)
		self.sub = rospy.Subscriber('/MainController', ControllerPacket, self.controllerCallback)
		self.pub_state = rospy.Publisher('/VisionSystem', VisionPacket,queue_size =10)

		self.user_input=-1

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
		# Function run every time main controller sends a message on /MainControllerPacket
		if self.user_input == 1:		
			self.update_Vision_info()	# Used to manually update different information
		else:
			self.item.VisionStatus = data.McStatus
		self.pub_state.publish(self.item)

	def update_Vision_info(self):
		#print menu and get selection
		self.print_menu()
		print("Select option")
		selected_value = input()
		#process the selected item.
		#if user chooses to publish the message, this function ends and control returns to the callback which publishes the message
		while selected_value <= 10:
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
			if selected_value == 10:
				print("---------------------------")
				print("Vision Packet:")
				print("---------------------------")
				print(self.item)
				print("---------------------------")
			self.print_menu()
			print("Select option")
			selected_value = input()

	def print_menu(self):
		print("--------------------------------")
		print("--------------------------------")
		print("OPTIONS")
		print("1. Change Vision Status")
		print("2. Change object x position")
		print("3. Change object y position")
		print("4. Change object z position")
		print("5. Change object qx orientation")
		print("6. Change object qy orientation")
		print("7. Change object qz orientation")
		print("8. Change object qw orientation")
		print("9. Change bounding box")
		print("10. Report current vision packet")
		print("11. Publish current vision packet")
		print("--------------------------------")
		print("--------------------------------")



if __name__ == "__main__":
	 """ Run RosNode """
print("Vision Mock Node Online")
user_input = -1
while (user_input <> 0) and (user_input <> 1):
	user_input = int(input("For vision response behaviour, 1 for user control, 0 for autonomated: "))
vs =  mock_vision_node()
vs.user_input = user_input
while not rospy.is_shutdown():
	rospy.spin()
