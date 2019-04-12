from std_msgs.msg import Bool
import roslib
import math
import rospy
import time
from std_msgs.msg import String
import tf
import tf2_ros
import tf2_geometry_msgs
import sys
#import moveit_commander
#import moveit_msgs.msg
from geometry_msgs.msg import Pose
#from moveit_commander.conversions import pose_to_list

class EEInterface():

    def __init__(self):

        self.vacuum_on = False
        self.item_held = False

        # Publishser
        self.vacuum_publisher = rospy.Publisher("/GripItem", Bool, queue_size=1)
        
        # Subscribers
        self.item_held_sub = rospy.Subscriber("/ItemHeld", Bool, self.item_held_callback)
        self.vacuum_toggle_sub = rospy.Subscriber("/VacuumStatus", Bool, self.vacuum_status_callback)

    def item_held_callback(self, data):
        self.item_held = data.data

    def vacuum_status_callback(self, data):
        self.vacuum_on = data.data

    def toggle_vacuum(self, state):
        msg = Bool()
        msg.data = state
        self.vacuum_publisher.publish(msg)

    def get_vacuum_status(self):
        return self.vacuum_on

    def get_item_held_status(self):
        return self.item_held
