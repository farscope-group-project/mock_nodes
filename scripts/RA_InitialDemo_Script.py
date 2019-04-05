#!/usr/bin/env python
import roslib
import math
import rospy
import tf
import tf2_ros
import tf2_geometry_msgs
import sys
import copy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose
from moveit_commander.conversions import pose_to_list

### A bunch of Initial Setup

rospy.init_node('InitialDemo_Node')

home_position = [math.radians(90.00), math.radians(-135.68), math.radians(121.90), math.radians(-76.51), math.radians(268.67) ,math.radians(-2.00)]
shelfF_position = [math.radians(40.08), math.radians(-106.25), math.radians(126.28), math.radians(-109.63), math.radians(270.44) ,math.radians(-53.83)]
default_velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

moveit_commander.roscpp_initialize(sys.argv)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)
move_group.set_max_velocity_scaling_factor(0.2)

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
rospy.sleep(0.5)
joint_angle_pub = rospy.Publisher("/arm_controller/follow_joint_trajectory/goal", FollowJointTrajectoryActionGoal, queue_size=20)
rospy.sleep(0.5)

###
