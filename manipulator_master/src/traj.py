#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
import math
#By executing this file sending from the console 5 positions for the joints in degrees we can move the robot 
def perform_trajectory():
    rospy.init_node('manipulator_trajectory_publisher')
    contoller_name='/arm_controller/command'
    trajectory_publisher = rospy.Publisher(contoller_name,JointTrajectory, queue_size=10)
    argv = sys.argv[1:]      

    panda_joints = ['joint_1','joint_2','joint_3','joint_4','joint_5']

    goal_positions = [ math.radians(float(argv[0])) , math.radians(float(argv[1])) , math.radians(float(argv[2])) ,math.radians(float(argv[3]) ) ,
                       math.radians(float(argv[4])) ]
 
    rospy.loginfo("Goal Position set lets go ! ")
    rospy.sleep(1)

    trajectory_msg = JointTrajectory()
    trajectory_msg.joint_names = panda_joints
    trajectory_msg.points.append(JointTrajectoryPoint())
    trajectory_msg.points[0].positions = goal_positions
    trajectory_msg.points[0].velocities = [0.0 for i in panda_joints]
    trajectory_msg.points[0].accelerations = [0.0 for i in panda_joints]
    trajectory_msg.points[0].time_from_start = rospy.Duration(3)
    rospy.sleep(1)
    trajectory_publisher.publish(trajectory_msg)


if __name__ == '__main__':
    perform_trajectory()