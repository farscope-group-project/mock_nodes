import roslib
import math
import rospy
import time
from std_msgs.msg import String
import tf
import tf2_ros
import tf2_geometry_msgs
import sys
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose
from moveit_commander.conversions import pose_to_list
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryActionGoal

class ArmInterface():


    def __init__(self):
        self.home_position = [math.radians(90.00), math.radians(-135.68), math.radians(121.90), math.radians(-76.51), math.radians(268.67) ,math.radians(-2.00)]
        self.shelfF_position = [math.radians(40.08), math.radians(-106.25), math.radians(126.28), math.radians(-109.63), math.radians(270.44) ,math.radians(-53.83)]
        self.default_velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.move_group.set_max_velocity_scaling_factor(0.1)

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        rospy.sleep(0.5)
        self.joint_angle_pub = rospy.Publisher("/follow_joint_trajectory/goal", FollowJointTrajectoryActionGoal, queue_size=20)
        rospy.sleep(0.5)


    def do_its_thing(self):
        pass

    def gotoBin(self, binName):
        traj = JointTrajectory()
        traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        bin = JointTrajectoryPoint()
        bin.velocities = self.default_velocity
        bin.time_from_start = rospy.Duration(5.0)
        if(binName == "F"):
            rospy.loginfo("Going to Shelf F!")
            bin.positions = self.shelfF_position
        else:
            bin.positions = self.home_position

        traj.points.append(bin)

        now = rospy.get_rostime()
        rospy.loginfo("Current time %i %i", now.secs, now.nsecs)
        traj.header.stamp = now

        ag = FollowJointTrajectoryActionGoal()
        ag.goal.trajectory = traj

        self.joint_angle_pub.publish(ag)
        rospy.sleep(5)

    def goHome(self):
        traj = JointTrajectory()
        traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        now = rospy.get_rostime()
        rospy.loginfo("Current time %i %i", now.secs, now.nsecs)
        traj.header.stamp = now

        home = JointTrajectoryPoint()
        home.positions = self.home_position
        home.velocities = self.default_velocity
        home.time_from_start = rospy.Duration(10.0)
        traj.points.append(home)

        ag = FollowJointTrajectoryActionGoal()
        ag.goal.trajectory = traj

        self.joint_angle_pub.publish(ag)
        rospy.sleep(10)

    def doVisionSweep(self, original_rpy, original_pose, nextPos):
        pos = Pose()
        if(nextPos == 1):
            q = tf.transformations.quaternion_from_euler((original_rpy[0]-math.radians(10)), original_rpy[1], (original_rpy[2]-math.radians(20)))
            pos.position.x = original_pose.position.x - 0.12155
            pos.position.y = original_pose.position.y + 0.01596
            pos.position.z = original_pose.position.z + 0.06077
        elif(nextPos == 2):
            q = tf.transformations.quaternion_from_euler((original_rpy[0]-math.radians(10)), original_rpy[1], (original_rpy[2]-math.radians(10)))
            pos.position.x = original_pose.position.x - 0.06077686218
            pos.position.y = original_pose.position.y + 0.01064
            pos.position.z = original_pose.position.z + 0.06077
        elif(nextPos == 3):
            q = tf.transformations.quaternion_from_euler((original_rpy[0]-math.radians(10)), original_rpy[1], (original_rpy[2]))
            pos.position.x = original_pose.position.x + 0.0
            pos.position.y = original_pose.position.y + 0.00532
            pos.position.z = original_pose.position.z + 0.06077
        elif(nextPos == 4):
            q = tf.transformations.quaternion_from_euler((original_rpy[0]-math.radians(10)), original_rpy[1], (original_rpy[2]+math.radians(10)))
            pos.position.x = original_pose.position.x + 0.06077686218
            pos.position.y = original_pose.position.y + 0.01064
            pos.position.z = original_pose.position.z + 0.06077
        elif(nextPos == 5):
            q = tf.transformations.quaternion_from_euler((original_rpy[0]-math.radians(10)), original_rpy[1], (original_rpy[2]+math.radians(20)))
            pos.position.x = original_pose.position.x + 0.12155
            pos.position.y = original_pose.position.y + 0.01596
            pos.position.z = original_pose.position.z + 0.06077
        elif(nextPos == 6):
            q = tf.transformations.quaternion_from_euler(original_rpy[0], original_rpy[1], (original_rpy[2]+math.radians(20)))
            pos.position.x = original_pose.position.x + 0.12155
            pos.position.y = original_pose.position.y + 0.01596
            pos.position.z = original_pose.position.z + 0.0
        elif(nextPos == 7):
            q = tf.transformations.quaternion_from_euler(original_rpy[0], original_rpy[1], (original_rpy[2]+math.radians(10)))
            pos.position.x = original_pose.position.x + 0.06077686218
            pos.position.y = original_pose.position.y + 0.01064
            pos.position.z = original_pose.position.z + 0.0
        elif(nextPos == 8):
            q = tf.transformations.quaternion_from_euler(original_rpy[0], original_rpy[1], original_rpy[2])
            pos.position = original_pose.position
        elif(nextPos == 9):
            q = tf.transformations.quaternion_from_euler(original_rpy[0], original_rpy[1], (original_rpy[2]-math.radians(10)))
            pos.position.x = original_pose.position.x - 0.06077686218
            pos.position.y = original_pose.position.y + 0.01064
            pos.position.z = original_pose.position.z + 0.0
        elif(nextPos == 10):
            q = tf.transformations.quaternion_from_euler(original_rpy[0], original_rpy[1], (original_rpy[2]-math.radians(20)))
            pos.position.x = original_pose.position.x - 0.12155
            pos.position.y = original_pose.position.y + 0.01596
            pos.position.z = original_pose.position.z + 0.0
        elif(nextPos == 11):
            q = tf.transformations.quaternion_from_euler((original_rpy[0]+math.radians(10)), original_rpy[1], (original_rpy[2]-math.radians(20)))
            pos.position.x = original_pose.position.x - 0.12155
            pos.position.y = original_pose.position.y + 0.01596
            pos.position.z = original_pose.position.z - 0.06077
        elif(nextPos == 12):
            q = tf.transformations.quaternion_from_euler((original_rpy[0]+math.radians(10)), original_rpy[1], (original_rpy[2]-math.radians(10)))
            pos.position.x = original_pose.position.x - 0.06077686218
            pos.position.y = original_pose.position.y + 0.01064
            pos.position.z = original_pose.position.z - 0.06077
        elif(nextPos == 13):
            q = tf.transformations.quaternion_from_euler((original_rpy[0]+math.radians(10)), original_rpy[1], (original_rpy[2]))
            pos.position.x = original_pose.position.x + 0.0
            pos.position.y = original_pose.position.y + 0.00532
            pos.position.z = original_pose.position.z - 0.06077
        elif(nextPos == 14):
            q = tf.transformations.quaternion_from_euler((original_rpy[0]+math.radians(10)), original_rpy[1], (original_rpy[2]+math.radians(10)))
            pos.position.x = original_pose.position.x + 0.06077686218
            pos.position.y = original_pose.position.y + 0.01064
            pos.position.z = original_pose.position.z - 0.06077
        elif(nextPos == 15):
            q = tf.transformations.quaternion_from_euler((original_rpy[0]+math.radians(10)), original_rpy[1], (original_rpy[2]+math.radians(20)))
            pos.position.x = original_pose.position.x + 0.12155
            pos.position.y = original_pose.position.y + 0.01596
            pos.position.z = original_pose.position.z - 0.06077
        else:
            q = tf.transformations.quaternion_from_euler(original_rpy[0], original_rpy[1], original_rpy[2])
            pos.position.x = original_pose.position.x
            pos.position.y = original_pose.position.y
            pos.position.z = original_pose.position.z
        # End of If Else Structure
        pos.orientation.x = q[0]
        pos.orientation.y = q[1]
        pos.orientation.z = q[2]
        pos.orientation.w = q[3]
        self.move_group.set_pose_target(pos)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        # rospy.sleep(1)

    def incHeight(self, z_change, original_rpy, original_pose):
        pos = Pose()
        q = tf.transformations.quaternion_from_euler(original_rpy[0], original_rpy[1], original_rpy[2])
        pos.position.x = original_pose.position.x
        pos.position.y = original_pose.position.y
        pos.position.z = original_pose.position.z + z_change
        pos.orientation.x = q[0]
        pos.orientation.y = q[1]
        pos.orientation.z = q[2]
        pos.orientation.w = q[3]
        self.move_group.set_pose_target(pos)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def incDepth(self, y_change, original_rpy, original_pose):
        pos = Pose()
        q = tf.transformations.quaternion_from_euler(original_rpy[0], original_rpy[1], original_rpy[2])
        pos.position.x = original_pose.position.x
        pos.position.y = original_pose.position.y - y_change
        pos.position.z = original_pose.position.z
        pos.orientation.x = q[0]
        pos.orientation.y = q[1]
        pos.orientation.z = q[2]
        pos.orientation.w = q[3]
        self.move_group.set_pose_target(pos)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def gotoObject(self, x, y, z, alpha, beta, gamma):
        # For now this will visit a hardcoded spot to pick up an item
        traj = JointTrajectory()
        traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        now = rospy.get_rostime()
        rospy.loginfo("Current time %i %i", now.secs, now.nsecs)
        traj.header.stamp = now
        inside = JointTrajectoryPoint()
        inside.positions = [math.radians(56.51), math.radians(-85.19), math.radians(95.29), math.radians(-100.09), math.radians(270.91), math.radians(-38.14)]
        inside.velocities = self.default_velocity
        inside.time_from_start = rospy.Duration(10.0)
        traj.points.append(inside)

        touching_item = JointTrajectoryPoint()
        touching_item.positions = [math.radians(56.28), math.radians(-82.90), math.radians(99.17), math.radians(-107.89), math.radians(271.42), math.radians(-38.20)]
        touching_item.velocities = self.default_velocity
        touching_item.time_from_start = rospy.Duration(20.0)
        traj.points.append(touching_item)
        ag = FollowJointTrajectoryActionGoal()
        ag.goal.trajectory = traj

        self.joint_angle_pub.publish(ag)
        rospy.sleep(20)

        inside = JointTrajectoryPoint()
        inside.positions = [math.radians(56.51), math.radians(-85.19), math.radians(95.29), math.radians(-100.09), math.radians(270.91), math.radians(-38.14)]
        inside.velocities = self.default_velocity
        inside.time_from_start = rospy.Duration(10.0)
        traj.points.append(inside)

        ag = FollowJointTrajectoryActionGoal()
        ag.goal.trajectory = traj

        self.joint_angle_pub.publish(ag)
        rospy.sleep(10)

    def gotoBox(self):
        # hardcoded for now
        traj = JointTrajectory()
        traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        now = rospy.get_rostime()
        rospy.loginfo("Current time %i %i", now.secs, now.nsecs)
        traj.header.stamp = now
        rospy.loginfo("Going to Box")

        home = JointTrajectoryPoint()
        home.positions = self.home_position
        home.velocities = self.default_velocity
        home.time_from_start = rospy.Duration(10.0)
        traj.points.append(home)

        Bin_way1 = JointTrajectoryPoint()
        Bin_way1.positions = [math.radians(4.78), self.home_position[1], self.home_position[2], self.home_position[3], self.home_position[4], self.home_position[5]]
        Bin_way1.velocities = self.default_velocity
        Bin_way1.time_from_start = rospy.Duration(15.0)
        traj.points.append(Bin_way1)

        Bin_way2 = JointTrajectoryPoint()
        Bin_way2.positions = [math.radians(4.78), math.radians(-78.95), math.radians(142.9), math.radians(-154.65), math.radians(271.99), math.radians(2.51)]
        Bin_way2.velocities = self.default_velocity
        Bin_way2.time_from_start = rospy.Duration(20.0)
        traj.points.append(Bin_way2)

        Bin = JointTrajectoryPoint()
        Bin.positions = [math.radians(33.62), math.radians(-35.03), math.radians(125.73), math.radians(-181.48), math.radians(275.56), math.radians(0.76)]
        Bin.velocities = self.default_velocity
        Bin.time_from_start = rospy.Duration(25.0)
        traj.points.append(Bin)

        ag = FollowJointTrajectoryActionGoal()
        ag.goal.trajectory = traj
        self.joint_angle_pub.publish(ag)
        rospy.sleep(25)

    def goHomeFromBox(self):
        traj = JointTrajectory()
        traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        now = rospy.get_rostime()
        rospy.loginfo("Current time %i %i", now.secs, now.nsecs)
        traj.header.stamp = now
        rospy.loginfo("Going Home from Box")

        Bin_way2 = JointTrajectoryPoint()
        Bin_way2.positions = [math.radians(4.78), math.radians(-78.95), math.radians(142.9), math.radians(-154.65), math.radians(271.99), math.radians(2.51)]
        Bin_way2.velocities = self.default_velocity
        Bin_way2.time_from_start = rospy.Duration(5.0)
        traj.points.append(Bin_way2)

        Bin_way1 = JointTrajectoryPoint()
        Bin_way1.positions = [math.radians(4.78), self.home_position[1], self.home_position[2], self.home_position[3], self.home_position[4], self.home_position[5]]
        Bin_way1.velocities = self.default_velocity
        Bin_way1.time_from_start = rospy.Duration(10.0)
        traj.points.append(Bin_way1)

        home = JointTrajectoryPoint()
        home.positions = self.home_position
        home.velocities = self.default_velocity
        home.time_from_start = rospy.Duration(15.0)
        traj.points.append(home)

        ag = FollowJointTrajectoryActionGoal()
        ag.goal.trajectory = traj
        self.joint_angle_pub.publish(ag)
        rospy.sleep(15)
