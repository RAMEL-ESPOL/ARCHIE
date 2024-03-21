#! /usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list

#By executing this file we can make the robot move to several preconfigured positions in Cartesian coordinates, in the order in which they are in the file
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()    
group = moveit_commander.MoveGroupCommander("arm_group")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)


# We can get the name of the reference frame for this robot:
planning_frame = group.get_planning_frame()
print("============ Planning frame: %s" % planning_frame)

# We can also print the name of the end-effector link for this group:
eef_link = group.get_end_effector_link()
print("============ End effector link: %s" % eef_link)

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print("============ Available Planning Groups:", robot.get_group_names())

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print("============ Printing robot state")
print(robot.get_current_state())
print("")

group.set_goal_orientation_tolerance(-1.57)

pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 1
pose_target.position.x = 0.2
pose_target.position.y = 0.0
pose_target.position.z = 0.25
group.set_pose_target(pose_target)

plan = group.go(wait=True)
group.stop()
group.clear_pose_targets()
rospy.sleep(5)

pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 1
pose_target.position.x = 0.2
pose_target.position.y = 0.1
pose_target.position.z = 0.25
group.set_pose_target(pose_target)

plan = group.go(wait=True)
group.stop()
group.clear_pose_targets()
rospy.sleep(5)

"""
waypoints = []

pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 1
pose_target.position.x = 0.25
pose_target.position.y = 0.0
pose_target.position.z = 0.25

waypoints.append(copy.deepcopy(pose_target))

pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 1.0
pose_target.position.x = 0.2
pose_target.position.y = 0.0
pose_target.position.z = 0.25

waypoints.append(copy.deepcopy(pose_target))


# We want the Cartesian path to be interpolated at a resolution of 1 cm
# which is why we will specify 0.01 as the eef_step in Cartesian
# translation.  We will disable the jump threshold by setting it to 0.0,
# ignoring the check for infeasible jumps in joint space, which is sufficient
# for this tutorial.
fraction = 0.0
maxtries = 200
attempts = 0
(plan, fraction) = group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold
#while fraction < 1.0 and attempts < maxtries:
#    (plan, fraction) = group.compute_cartesian_path(
#                                   waypoints,   # waypoints to follow
#                                   0.01,        # eef_step
#                                   0.0)         # jump_threshold
    
#    attempts+=1
#    if attempts % 10 == 0:
#        rospy.loginfo("Still trying after " + str(attempts) + " attempts...")

group.execute(plan, wait=True)
# Note: We are just planning, not asking move_group to actually move the robot yet:
rospy.sleep(1)

"""
# Calling `stop()` ensures that there is no residual movement
group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
group.clear_pose_targets()
rospy.sleep(5)

moveit_commander.roscpp_shutdown()