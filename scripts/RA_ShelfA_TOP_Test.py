#!/usr/bin/env python
import roslib
import math
roslib.load_manifest('ur_tutorial')
import rospy
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryActionGoal

home_position = [math.radians(90.00), math.radians(-135.68), math.radians(121.90), math.radians(-76.51), math.radians(268.67) ,math.radians(-2.00)]
home_velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

rospy.init_node("simple_move_goal_pub")
pub = rospy.Publisher("/arm_controller/follow_joint_trajectory/goal", FollowJointTrajectoryActionGoal, queue_size=20)
rospy.sleep(0.5)

topic_name = rospy.resolve_name("/arm_controller/follow_joint_trajectory/goal")
rospy.loginfo("Sending goal to %s", topic_name)

traj = JointTrajectory()
traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

now = rospy.get_rostime()
rospy.loginfo("Current time %i %i", now.secs, now.nsecs)
traj.header.stamp = now

home = JointTrajectoryPoint()
home.positions = home_position
home.velocities = home_velocity
home.time_from_start = rospy.Duration(5.0/2)
traj.points.append(home)

shelf_A = JointTrajectoryPoint()
shelf_A.positions = [math.radians(100.63), math.radians(-106.49), math.radians(89.88), math.radians(-73.64), math.radians(274.08) ,math.radians(11.66)]
shelf_A.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
shelf_A.time_from_start = rospy.Duration(10.0/2)
traj.points.append(shelf_A)

shelf_ATLL = JointTrajectoryPoint()
shelf_ATLL.positions = [math.radians(118.62), math.radians(-94.75), math.radians(73.14), math.radians(-63.7), math.radians(275.07) ,math.radians(48.84)]
shelf_ATLL.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
shelf_ATLL.time_from_start = rospy.Duration(15.0/2)
traj.points.append(shelf_ATLL)

shelf_ATL = JointTrajectoryPoint()
shelf_ATL.positions = [math.radians(110.44), math.radians(-99.63), math.radians(77.26), math.radians(-62.66), math.radians(275.72) ,math.radians(31.01)]
shelf_ATL.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
shelf_ATL.time_from_start = rospy.Duration(20.0/2)
traj.points.append(shelf_ATL)

shelf_ATC = JointTrajectoryPoint()
shelf_ATC.positions = [math.radians(100.33), math.radians(-102.79), math.radians(79.44), math.radians(-60.41), math.radians(274.97) ,math.radians(11.13)]
shelf_ATC.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
shelf_ATC.time_from_start = rospy.Duration(25.0/2)
traj.points.append(shelf_ATC)

shelf_ATR = JointTrajectoryPoint()
shelf_ATR.positions = [math.radians(89.87), math.radians(-107.3), math.radians(82.33), math.radians(-57.69), math.radians(272.62) ,math.radians(-8.99)]
shelf_ATR.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
shelf_ATR.time_from_start = rospy.Duration(30.0/2)
traj.points.append(shelf_ATR)

shelf_ATRR = JointTrajectoryPoint()
shelf_ATRR.positions = [math.radians(79.13), math.radians(-109.85), math.radians(84), math.radians(-57), math.radians(268.87) ,math.radians(-29.14)]
shelf_ATRR.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
shelf_ATRR.time_from_start = rospy.Duration(35.0/2)
traj.points.append(shelf_ATRR)

shelf_A = JointTrajectoryPoint()
shelf_A.positions = [math.radians(100.63), math.radians(-106.49), math.radians(89.88), math.radians(-73.64), math.radians(274.08) ,math.radians(11.66)]
shelf_A.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
shelf_A.time_from_start = rospy.Duration(40.0/2)
traj.points.append(shelf_A)

home = JointTrajectoryPoint()
home.positions = home_position
home.velocities = home_velocity
home.time_from_start = rospy.Duration(45.0/2)
traj.points.append(home)

ag = FollowJointTrajectoryActionGoal()
ag.goal.trajectory = traj

pub.publish(ag)
