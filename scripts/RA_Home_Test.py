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
home.time_from_start = rospy.Duration(10.0)
traj.points.append(home)

ag = FollowJointTrajectoryActionGoal()
ag.goal.trajectory = traj

pub.publish(ag)
