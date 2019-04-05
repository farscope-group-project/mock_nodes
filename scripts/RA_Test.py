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

home = JointTrajectoryPoint()
home.positions = home_position
home.velocities = home_velocity
home.time_from_start = rospy.Duration(15.0/2)
traj.points.append(home)

shelf_B = JointTrajectoryPoint()
shelf_B.positions = [math.radians(74.04), math.radians(-103.26), math.radians(87.15), math.radians(-72.18), math.radians(273.71) ,math.radians(-14.99)]
shelf_B.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
shelf_B.time_from_start = rospy.Duration(20.0/2)
traj.points.append(shelf_B)

home = JointTrajectoryPoint()
home.positions = home_position
home.velocities = home_velocity
home.time_from_start = rospy.Duration(25.0/2)
traj.points.append(home)

shelf_C = JointTrajectoryPoint()
shelf_C.positions = [math.radians(54.33), math.radians(-90.29), math.radians(75.63), math.radians(-72.38), math.radians(272.86) ,math.radians(-34.72)]
shelf_C.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
shelf_C.time_from_start = rospy.Duration(30.0/2)
traj.points.append(shelf_C)

home = JointTrajectoryPoint()
home.positions = home_position
home.velocities = home_velocity
home.time_from_start = rospy.Duration(35.0/2)
traj.points.append(home)

shelf_D = JointTrajectoryPoint()
shelf_D.positions = [math.radians(107.2), math.radians(-97.02), math.radians(107.01), math.radians(-100.74), math.radians(274.01) ,math.radians(18.12)]
shelf_D.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
shelf_D.time_from_start = rospy.Duration(40.0/2)
traj.points.append(shelf_D)

home = JointTrajectoryPoint()
home.positions = home_position
home.velocities = home_velocity
home.time_from_start = rospy.Duration(45.0/2)
traj.points.append(home)

shelf_E = JointTrajectoryPoint()
shelf_E.positions = [math.radians(75.39), math.radians(-106.98), math.radians(110.34), math.radians(-95.91), math.radians(271.19) ,math.radians(-13.76)]
shelf_E.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
shelf_E.time_from_start = rospy.Duration(50.0/2)
traj.points.append(shelf_E)

home = JointTrajectoryPoint()
home.positions = home_position
home.velocities = home_velocity
home.time_from_start = rospy.Duration(55.0/2)
traj.points.append(home)

shelf_F = JointTrajectoryPoint()
shelf_F.positions = [math.radians(49.66), math.radians(-101.9), math.radians(110.34), math.radians(-95.91), math.radians(271.19) ,math.radians(-39.49)]
shelf_F.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
shelf_F.time_from_start = rospy.Duration(60.0/2)
traj.points.append(shelf_F)

home = JointTrajectoryPoint()
home.positions = home_position
home.velocities = home_velocity
home.time_from_start = rospy.Duration(65.0/2)
traj.points.append(home)

shelf_G = JointTrajectoryPoint()
shelf_G.positions = [math.radians(105.16), math.radians(-85.05), math.radians(127.29), math.radians(-132.04), math.radians(272.71) ,math.radians(15.89)]
shelf_G.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
shelf_G.time_from_start = rospy.Duration(70.0/2)
traj.points.append(shelf_G)

home = JointTrajectoryPoint()
home.positions = home_position
home.velocities = home_velocity
home.time_from_start = rospy.Duration(75.0/2)
traj.points.append(home)

shelf_H = JointTrajectoryPoint()
shelf_H.positions = [math.radians(71.42), math.radians(-93.58), math.radians(135.52), math.radians(-130.14), math.radians(272.09) ,math.radians(-17.88)]
shelf_H.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
shelf_H.time_from_start = rospy.Duration(80.0/2)
traj.points.append(shelf_H)

home = JointTrajectoryPoint()
home.positions = home_position
home.velocities = home_velocity
home.time_from_start = rospy.Duration(85.0/2)
traj.points.append(home)

shelf_I = JointTrajectoryPoint()
shelf_I.positions = [math.radians(47.29), math.radians(-87.50), math.radians(129.31), math.radians(-129.22), math.radians(271.13) ,math.radians(-42.00)]
shelf_I.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
shelf_I.time_from_start = rospy.Duration(90.0/2)
traj.points.append(shelf_I)

home = JointTrajectoryPoint()
home.positions = home_position
home.velocities = home_velocity
home.time_from_start = rospy.Duration(95.0/2)
traj.points.append(home)

shelf_J = JointTrajectoryPoint()
shelf_J.positions = [math.radians(103.46), math.radians(-55.84), math.radians(133.48), math.radians(-167.35), math.radians(272.81) ,math.radians(14.09)]
shelf_J.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
shelf_J.time_from_start = rospy.Duration(100.0/2)
traj.points.append(shelf_J)

home = JointTrajectoryPoint()
home.positions = home_position
home.velocities = home_velocity
home.time_from_start = rospy.Duration(105.0/2)
traj.points.append(home)

shelf_K = JointTrajectoryPoint()
shelf_K.positions = [math.radians(68.12), math.radians(-58.07), math.radians(141.66), math.radians(-171.65), math.radians(272.11) ,math.radians(-21.27)]
shelf_K.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
shelf_K.time_from_start = rospy.Duration(110.0/2)
traj.points.append(shelf_K)

home = JointTrajectoryPoint()
home.positions = home_position
home.velocities = home_velocity
home.time_from_start = rospy.Duration(115.0/2)
traj.points.append(home)

shelf_L = JointTrajectoryPoint()
shelf_L.positions = [math.radians(45.99), math.radians(-56.58), math.radians(134.09), math.radians(-164.89), math.radians(271.17) ,math.radians(-43.39)]
shelf_L.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
shelf_L.time_from_start = rospy.Duration(120.0/2)
traj.points.append(shelf_L)

home = JointTrajectoryPoint()
home.positions = home_position
home.velocities = home_velocity
home.time_from_start = rospy.Duration(125.0/2)
traj.points.append(home)

ag = FollowJointTrajectoryActionGoal()
ag.goal.trajectory = traj

pub.publish(ag)
