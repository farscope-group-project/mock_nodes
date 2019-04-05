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

shelf_ATC = JointTrajectoryPoint()
shelf_ATC.positions = [math.radians(100.33), math.radians(-102.79), math.radians(79.44), math.radians(-60.41), math.radians(274.97) ,math.radians(11.13)]
shelf_ATC.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
shelf_ATC.time_from_start = rospy.Duration(15.0/2)
traj.points.append(shelf_ATC)

shelf_A = JointTrajectoryPoint()
shelf_A.positions = [math.radians(100.63), math.radians(-106.49), math.radians(89.88), math.radians(-73.64), math.radians(274.08) ,math.radians(11.66)]
shelf_A.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
shelf_A.time_from_start = rospy.Duration(20.0/2)
traj.points.append(shelf_A)

shelf_ABC = JointTrajectoryPoint()
shelf_ABC.positions = [math.radians(100.80), math.radians(-108.43), math.radians(98.81), math.radians(-86.99), math.radians(272.56) ,math.radians(11.91)]
shelf_ABC.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
shelf_ABC.time_from_start = rospy.Duration(30.0/2)
traj.points.append(shelf_ABC)

shelf_A = JointTrajectoryPoint()
shelf_A.positions = [math.radians(100.63), math.radians(-106.49), math.radians(89.88), math.radians(-73.64), math.radians(274.08) ,math.radians(11.66)]
shelf_A.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
shelf_A.time_from_start = rospy.Duration(40.0/2)
traj.points.append(shelf_A)

home = JointTrajectoryPoint()
home.positions = home_position
home.velocities = home_velocity
home.time_from_start = rospy.Duration(50.0/2)
traj.points.append(home)

ag = FollowJointTrajectoryActionGoal()
ag.goal.trajectory = traj

pub.publish(ag)
