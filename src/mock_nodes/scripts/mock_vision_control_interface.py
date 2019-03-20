#!/usr/bin/python
import rospy
from std_msgs.msg import Bool, String
from mock_nodes.msg import * 

item = VisionPacket()
item.VisionStatus = 0 
item.ItemPose.position.x = 0
item.ItemPose.position.y = 0
item.ItemPose.position.z = 0
item.ItemPose.orientation.x = 0
item.ItemPose.orientation.y = 1
item.ItemPose.orientation.z = 2
item.ItemPose.orientation.w = 3
item.ItemBoundingBox = [2,4,6]
def visionCallback(data):
	rospy.loginfo(data)
#	controller_info = data # Nawid -This makes the information equal to the data provided by the subscriber 
#	return controller_info
	

def visionNode(): 
	rospy.init_node('Vision_controller_interface', anonymous=True)
	vision_publisher = rospy.Publisher('\object_position', VisionPacket,queue_size =1)
	vision_subscriber = rospy.Subscriber('\camera_positions',ControllerPacket,visionCallback)
	
	rospy.Rate(5000)
	
	
	while True:
		vision_publisher.publish(item)
  	rospy.spin_once();
  	rate.sleep()


	
if __name__ == '__main__':
	visionNode()
	#except rospy.ROSinterruptException
	#	pass
