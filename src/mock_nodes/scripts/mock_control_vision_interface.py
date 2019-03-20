#!/usr/bin/python
import rospy
from std_msgs.msg import Bool, String
from mock_nodes.msg import * 


Camera_position = ControllerPacket()
Camera_position.McStatus = 0 
Camera_position.ItemName = 'Joke book'
Camera_position.InterfaceJoint.position.x = 0
Camera_position.InterfaceJoint.position.y = 0
Camera_position.InterfaceJoint.position.z = 0 
Camera_position.InterfaceJoint.orientation.x = 0 
Camera_position.InterfaceJoint.orientation.y = 10
Camera_position.InterfaceJoint.orientation.z = 20
Camera_position.InterfaceJoint.orientation.w = 30

 
#vision_info = VisionPacket()

def controllerCallback(data):
	rospy.loginfo(data)
#	vision_info = data # Nawid -This makes the information equal to the data provided by the subscriber 
#	return vision_info
	

def controllerNode(): 
	rospy.init_node('Controller_vision_interface', anonymous=True)
	controller_publisher = rospy.Publisher('\camera_positions',ControllerPacket,queue_size = 1)
	controller_subscriber = rospy.Subscriber('\object_position', VisionPacket,controllerCallback)
	rospy.Rate(5000)
	
	while True:
		controller_publisher.publish(Camera_position)
#		print(vision_info)
  	rospy.spin_once();
  	rate.sleep()


	
if __name__ == '__main__':
	controllerNode()
	#except rospy.ROSinterruptException
	#	pass
		

