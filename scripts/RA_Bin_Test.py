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
home.time_from_start = rospy.Duration(5.0)
traj.points.append(home)

Bin_way1 = JointTrajectoryPoint()
Bin_way1.positions = [math.radians(4.78), home_position[1], home_position[2], home_position[3], home_position[4], home_position[5]]
Bin_way1.velocities = home_velocity
Bin_way1.time_from_start = rospy.Duration(10.0)
traj.points.append(Bin_way1)

Bin_way2 = JointTrajectoryPoint()
Bin_way2.positions = [math.radians(4.78), math.radians(-78.95), math.radians(142.9), math.radians(-154.65), math.radians(271.99), math.radians(2.51)]
Bin_way2.velocities = home_velocity
Bin_way2.time_from_start = rospy.Duration(15.0)
traj.points.append(Bin_way2)

Bin = JointTrajectoryPoint()
Bin.positions = [math.radians(33.62), math.radians(-35.03), math.radians(125.73), math.radians(-181.48), math.radians(275.56), math.radians(0.76)]
Bin.velocities = home_velocity
Bin.time_from_start = rospy.Duration(20.0)
traj.points.append(Bin)

Bin_way2 = JointTrajectoryPoint()
Bin_way2.positions = [math.radians(4.78), math.radians(-78.95), math.radians(142.9), math.radians(-154.65), math.radians(271.99), math.radians(2.51)]
Bin_way2.velocities = home_velocity
Bin_way2.time_from_start = rospy.Duration(25.0)
traj.points.append(Bin_way2)

Bin_way1 = JointTrajectoryPoint()
Bin_way1.positions = [math.radians(4.78), home_position[1], home_position[2], home_position[3], home_position[4], home_position[5]]
Bin_way1.velocities = home_velocity
Bin_way1.time_from_start = rospy.Duration(30.0)
traj.points.append(Bin_way1)

home = JointTrajectoryPoint()
home.positions = home_position
home.velocities = home_velocity
home.time_from_start = rospy.Duration(35.0)
traj.points.append(home)

ag = FollowJointTrajectoryActionGoal()
ag.goal.trajectory = traj

pub.publish(ag)
