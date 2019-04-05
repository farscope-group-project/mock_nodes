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

shelf_ACRR = JointTrajectoryPoint()
shelf_ACRR.positions = [math.radians(78.58), math.radians(-113.48), math.radians(93.89), math.radians(-69.18), math.radians(271.35) ,math.radians(-29.81)]
shelf_ACRR.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
shelf_ACRR.time_from_start = rospy.Duration(40.0/2)
traj.points.append(shelf_ACRR)

shelf_ACR = JointTrajectoryPoint()
shelf_ACR.positions = [math.radians(86.61), math.radians(-110.99), math.radians(92.57), math.radians(-70.68), math.radians(273.42) ,math.radians(-9.16)]
shelf_ACR.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
shelf_ACR.time_from_start = rospy.Duration(45.0/2)
traj.points.append(shelf_ACR)

shelf_ACC = JointTrajectoryPoint()
shelf_ACC.positions = [math.radians(100.63), math.radians(-106.49), math.radians(89.88), math.radians(-73.64), math.radians(274.08) ,math.radians(11.66)]
shelf_ACC.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
shelf_ACC.time_from_start = rospy.Duration(50.0/2)
traj.points.append(shelf_ACC)

shelf_ACL = JointTrajectoryPoint()
shelf_ACL.positions = [math.radians(110.85), math.radians(-102.75), math.radians(87.41), math.radians(-75.71), math.radians(273.16) ,math.radians(31.64)]
shelf_ACL.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
shelf_ACL.time_from_start = rospy.Duration(55.0/2)
traj.points.append(shelf_ACL)

shelf_ACLL = JointTrajectoryPoint()
shelf_ACLL.positions = [math.radians(119.22), math.radians(-97.6), math.radians(83.15), math.radians(-76.08), math.radians(271.28) ,math.radians(49.57)]
shelf_ACLL.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
shelf_ACLL.time_from_start = rospy.Duration(60.0/2)
traj.points.append(shelf_ACLL)

shelf_ABLL = JointTrajectoryPoint()
shelf_ABLL.positions = [math.radians(119.89), math.radians(-99.36), math.radians(91.87), math.radians(-87.82), math.radians(266.97) ,math.radians(50.06)]
shelf_ABLL.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
shelf_ABLL.time_from_start = rospy.Duration(65.0/2)
traj.points.append(shelf_ABLL)

shelf_ABL = JointTrajectoryPoint()
shelf_ABL.positions = [math.radians(111.41), math.radians(-104.75), math.radians(96.34), math.radians(-88.35), math.radians(270.03) ,math.radians(32.12)]
shelf_ABL.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
shelf_ABL.time_from_start = rospy.Duration(70.0/2)
traj.points.append(shelf_ABL)

shelf_ABC = JointTrajectoryPoint()
shelf_ABC.positions = [math.radians(100.8), math.radians(-108.43), math.radians(98.81), math.radians(-86.99), math.radians(272.56) ,math.radians(11.91)]
shelf_ABC.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
shelf_ABC.time_from_start = rospy.Duration(75.0/2)
traj.points.append(shelf_ABC)

shelf_ABR = JointTrajectoryPoint()
shelf_ABR.positions = [math.radians(89.58), math.radians(-113.72), math.radians(101.85), math.radians(-83.73), math.radians(273.73) ,math.radians(-9.04)]
shelf_ABR.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
shelf_ABR.time_from_start = rospy.Duration(80.0/2)
traj.points.append(shelf_ABR)

shelf_ABRR = JointTrajectoryPoint()
shelf_ABRR.positions = [math.radians(78.13), math.radians(-116.42), math.radians(103.13), math.radians(-81.55), math.radians(273.43) ,math.radians(-30.16)]
shelf_ABRR.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
shelf_ABRR.time_from_start = rospy.Duration(85.0/2)
traj.points.append(shelf_ABRR)

shelf_A = JointTrajectoryPoint()
shelf_A.positions = [math.radians(100.63), math.radians(-106.49), math.radians(89.88), math.radians(-73.64), math.radians(274.08) ,math.radians(11.66)]
shelf_A.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
shelf_A.time_from_start = rospy.Duration(90.0/2)
traj.points.append(shelf_A)

home = JointTrajectoryPoint()
home.positions = home_position
home.velocities = home_velocity
home.time_from_start = rospy.Duration(95.0/2)
traj.points.append(home)

ag = FollowJointTrajectoryActionGoal()
ag.goal.trajectory = traj

pub.publish(ag)
