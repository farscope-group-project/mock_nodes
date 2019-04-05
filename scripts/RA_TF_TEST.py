#!/usr/bin/env python
import roslib
import math
roslib.load_manifest('ur_tutorial')
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

rospy.init_node('MoveIt_Test')

#tf_buffer = tf2.ros.Buffer(rospy.Duration(1200.0))
tf_listener = tf.TransformListener()

# while not rospy.is_shutdown():
# 	try:
# 		(trans,rot) = tf_listener.lookupTransform('/base_link', '/tool0', rospy.Time(0))
# 		print("Transform Complete")
# 		print(trans)
# 		print(rot)
# 		m = tf.transformations.euler_from_quaternion(rot) # tf.transformations.quaternion_from_euler
# 		print(math.degrees(m[0]))
# 		print(math.degrees(m[1]))
# 		print(math.degrees(m[2]))
# 	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
# 		continue

# x,y,z [0.1617247318768563, -0.23423590326431276, 0.5997231378961866]
# Quaternion [-0.01716675441107421, 0.9997816273682181, -0.0023453266253019075, -0.01168330274229387]

# [1.570941686630249, -2.3673298994647425, 2.126859188079834, -1.3353055159198206, 4.689203262329102, -0.03463584581484014]
# angles = [math.degrees(1.5709656476974487), math.degrees(-2.367533032094137), math.degrees(2.126990795135498), math.degrees(-1.3353055159198206), math.degrees(4.689215183258057), math.degrees(-0.03462392488588506)]
# print(angles)

moveit_commander.roscpp_initialize(sys.argv)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)
move_group.set_max_velocity_scaling_factor(0.2)

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
planning_frame = move_group.get_planning_frame()
print("Planning Frame: " + planning_frame)
eef_link = move_group.get_end_effector_link()
print("End Effector Link: " + eef_link)
group_names = robot.get_group_names()
print("All available planning groups: ")
print(group_names)
print("Robot State: " + str(robot.get_current_state()))

print(move_group.get_current_joint_values())
print(move_group.get_current_pose().pose)

# joint_goal = [math.radians(100.63), math.radians(-106.49), math.radians(89.88), math.radians(-73.64), math.radians(274.08) ,math.radians(11.66)] #move_group.get_current_joint_values()
# move_group.go(joint_goal, wait=True)
# move_group.stop()
# move_group.clear_pose_targets()

original_rpy = move_group.get_current_rpy()
original_pose = move_group.get_current_pose().pose

for i in range(0,16):
    pos = Pose()
    if(i == 1): # TLL
        q = tf.transformations.quaternion_from_euler((original_rpy[0]-math.radians(10)), original_rpy[1], (original_rpy[2]-math.radians(20)))
        pos.position.x = original_pose.position.x - 0.12155
        pos.position.y = original_pose.position.y + 0.01596
        pos.position.z = original_pose.position.z + 0.06077
    elif(i == 2): # TL
        q = tf.transformations.quaternion_from_euler((original_rpy[0]-math.radians(10)), original_rpy[1], (original_rpy[2]-math.radians(10)))
        pos.position.x = original_pose.position.x - 0.06077686218
        pos.position.y = original_pose.position.y + 0.01064
        pos.position.z = original_pose.position.z + 0.06077
    elif(i == 3): # TC
        q = tf.transformations.quaternion_from_euler((original_rpy[0]-math.radians(10)), original_rpy[1], (original_rpy[2]))
        pos.position.x = original_pose.position.x + 0.0
        pos.position.y = original_pose.position.y + 0.00532
        pos.position.z = original_pose.position.z + 0.06077
    elif(i == 4): # TR
        q = tf.transformations.quaternion_from_euler((original_rpy[0]-math.radians(10)), original_rpy[1], (original_rpy[2]+math.radians(10)))
        pos.position.x = original_pose.position.x + 0.06077686218
        pos.position.y = original_pose.position.y + 0.01064
        pos.position.z = original_pose.position.z + 0.06077
    elif(i == 5): # TRR
        q = tf.transformations.quaternion_from_euler((original_rpy[0]-math.radians(10)), original_rpy[1], (original_rpy[2]+math.radians(20)))
        pos.position.x = original_pose.position.x + 0.12155
        pos.position.y = original_pose.position.y + 0.01596
        pos.position.z = original_pose.position.z + 0.06077
    elif(i == 6): # CRR
        q = tf.transformations.quaternion_from_euler(original_rpy[0], original_rpy[1], (original_rpy[2]+math.radians(20)))
        pos.position.x = original_pose.position.x + 0.12155
        pos.position.y = original_pose.position.y + 0.01596
        pos.position.z = original_pose.position.z + 0.0
    elif(i == 7): # CR
        q = tf.transformations.quaternion_from_euler(original_rpy[0], original_rpy[1], (original_rpy[2]+math.radians(10)))
        pos.position.x = original_pose.position.x + 0.06077686218
        pos.position.y = original_pose.position.y + 0.01064
        pos.position.z = original_pose.position.z + 0.0
    elif(i == 8): # CC
        q = tf.transformations.quaternion_from_euler(original_rpy[0], original_rpy[1], original_rpy[2])
        pos.position = original_pose.position
    elif(i == 9): # CL
        q = tf.transformations.quaternion_from_euler(original_rpy[0], original_rpy[1], (original_rpy[2]-math.radians(10)))
        pos.position.x = original_pose.position.x - 0.06077686218
        pos.position.y = original_pose.position.y + 0.01064
        pos.position.z = original_pose.position.z + 0.0
    elif(i == 10): # CLL
        q = tf.transformations.quaternion_from_euler(original_rpy[0], original_rpy[1], (original_rpy[2]-math.radians(20)))
        pos.position.x = original_pose.position.x - 0.12155
        pos.position.y = original_pose.position.y + 0.01596
        pos.position.z = original_pose.position.z + 0.0
    elif(i == 11): # BLL
        q = tf.transformations.quaternion_from_euler((original_rpy[0]+math.radians(10)), original_rpy[1], (original_rpy[2]-math.radians(20)))
        pos.position.x = original_pose.position.x - 0.12155
        pos.position.y = original_pose.position.y + 0.01596
        pos.position.z = original_pose.position.z - 0.06077
    elif(i == 12): # BL
        q = tf.transformations.quaternion_from_euler((original_rpy[0]+math.radians(10)), original_rpy[1], (original_rpy[2]-math.radians(10)))
        pos.position.x = original_pose.position.x - 0.06077686218
        pos.position.y = original_pose.position.y + 0.01064
        pos.position.z = original_pose.position.z - 0.06077
    elif(i == 13): # BC
        q = tf.transformations.quaternion_from_euler((original_rpy[0]+math.radians(10)), original_rpy[1], (original_rpy[2]))
        pos.position.x = original_pose.position.x + 0.0
        pos.position.y = original_pose.position.y + 0.00532
        pos.position.z = original_pose.position.z - 0.06077
    elif(i == 14): # BR
        q = tf.transformations.quaternion_from_euler((original_rpy[0]+math.radians(10)), original_rpy[1], (original_rpy[2]+math.radians(10)))
        pos.position.x = original_pose.position.x + 0.06077686218
        pos.position.y = original_pose.position.y + 0.01064
        pos.position.z = original_pose.position.z - 0.06077
    elif(i == 15): # BRR
        q = tf.transformations.quaternion_from_euler((original_rpy[0]+math.radians(10)), original_rpy[1], (original_rpy[2]+math.radians(20)))
        pos.position.x = original_pose.position.x + 0.12155
        pos.position.y = original_pose.position.y + 0.01596
        pos.position.z = original_pose.position.z - 0.06077
    else: # in teh case of unrecognised signal, return to original pose
        q = tf.transformations.quaternion_from_euler(original_rpy[0], original_rpy[1], original_rpy[2])
        pos.position.x = original_pose.position.x
        pos.position.y = original_pose.position.y
        pos.position.z = original_pose.position.z
    pos.orientation.x = q[0]
    pos.orientation.y = q[1]
    pos.orientation.z = q[2]
    pos.orientation.w = q[3]
    move_group.set_pose_target(pos)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    rospy.sleep(3)

move_group.set_pose_target(original_pose)
plan = move_group.go(wait=True)
move_group.stop()
move_group.clear_pose_targets()
