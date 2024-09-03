#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from gazebo_gripper import *

# Altura del lápiz
global z

global quit
quit = 0

global t
t = 0.01

def addBox(name, x, y, z, sx, sy, sz, scene):
    box_name = name
    
    # Create a message object of type PoseStamped to define box positions
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "world"
    
    # Set the position message arguments
    box_pose.pose.position.x = x
    box_pose.pose.position.y = y 
    box_pose.pose.position.z = z 
    box_pose.pose.orientation.x = 0
    box_pose.pose.orientation.y = 0
    box_pose.pose.orientation.z = 0
    box_pose.pose.orientation.w = 1.0
    
    # Add the box in the scene
    scene.add_box(box_name, box_pose, size=(sx, sy, sz))

def remove_box(name, timeout=4):
    box_name = name
    scene.remove_world_object(box_name)

def home():
    joint_goal = arm_group.get_current_joint_values()
    joint_goal = [0, 0, 0, 0, 0, 0]

    arm_group.go(joint_goal, wait=True)
    rospy.loginfo("ARCHIE is at home position.")
    return wpose

def move_cartesian_path(px, py, pz):
    waypoints = []

    wpose = arm_group.get_current_pose().pose 
    wpose.position.x = px
    wpose.position.y = py if py != 0 else wpose.position.y
    wpose.position.z = pz
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = arm_group.compute_cartesian_path(waypoints, t, 0.0)  # jump_threshold

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)

    # Publish
    display_trajectory_publisher.publish(display_trajectory)
    
    arm_group.execute(plan, wait=True)

def move_joints(j0, j1, j2, j3, j4, j5):
    # Set the target joint values
    arm_group.set_joint_value_target([j0, j1, j2, j3, j4, j5])
    
    # Plan the trajectory
    success, plan, _, _ = arm_group.plan()

    # Check if the plan is successful
    if success and len(plan.joint_trajectory.points) > 0:
        arm_group.execute(plan, wait=True)
    else:
        rospy.logerr("Failed to generate a valid plan for the joint values provided.")

    return arm_group.get_current_pose().pose  

def loginfog(msg: str):
    rospy.loginfo("\033[92m%s\033[0m" % msg)

def grab_object(size):
    joint_goal = ee_group.get_current_joint_values()
    joint_goal = [0.0, 0.0]
    if size == "medium":
        joint_goal = [0.012, 0.0]
    if size == "small":
        joint_goal = [0.019, 0.0]

    ee_group.go(joint_goal, wait=True)

    touch_links = robot.get_link_names(group=ee_group.get_name())
    ee_group.attach_object("box", "garra1_link", touch_links)

    # Attach object using gazebo_gripper only if gazebo is enabled
    if use_gazebo:
        gazebo_gripper.attach_Object()

    rospy.loginfo("ARCHIE picked the object")

def lose_object():
    joint_goal = ee_group.get_current_joint_values()
    joint_goal = [0.0, 0.0]
    ee_group.go(joint_goal, wait=True)

    # Detach object using gazebo_gripper only if gazebo is enabled
    if use_gazebo:
        gazebo_gripper.detach_Object()

    touch_links = robot.get_link_names(group=ee_group.get_name())
    ee_group.detach_object("garra1_link")
    rospy.loginfo("ARCHIE dropped the object")

def check_quit():
    global quit
    if input("Press 'q' to quit or any other key to continue: ") == "q":
        quit = 1

# Main execution
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('pick_and_place_node')

# Parse arguments to determine if gazebo is used
use_gazebo = rospy.get_param('~use_gazebo', False)

rate = rospy.Rate(10)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
arm_group = moveit_commander.MoveGroupCommander("arm_group")
ee_group = moveit_commander.MoveGroupCommander("ee_group")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
data_writing_publisher = rospy.Publisher('/figure_writing', String, queue_size=2)
data_writing_publisher.publish(("_none"))

# Grasping group for attach in rviz
grasping_group = "ee_group"


# Initialize gazebo_gripper only if gazebo is used
if use_gazebo:
    gazebo_gripper = RoboticGripper()
    gazebo_gripper._Robot_Name = "robot"
    gazebo_gripper._EE_Link_Name = "link_6"
    gazebo_gripper._Box_Model_Name = "box"
    gazebo_gripper._Box_Link_Name = "link"

# Main loop for continuous pick and place
wpose = move_joints(0, 0, 0, 0, 0, 0)

# ADD BOX rviz
addBox("box", wpose.position.x, wpose.position.y + 0.06, 0.01, 0.02, 0.02, 0.02, scene)

try:
    while not rospy.is_shutdown() and quit == 0:
        wpose = move_joints(0, 0, 0, 0, 0, 0)
        move_cartesian_path(wpose.position.x, wpose.position.y, 0.0407)
        rospy.sleep(0.1)
        grab_object("medium")
        rospy.sleep(0.1)
        
        wpose = move_joints(0.6, 0, 0, 0, 0, 0)
        move_cartesian_path(wpose.position.x, wpose.position.y, 0.0407)
        rospy.sleep(0.1)
        lose_object()
        wpose = move_joints(0.6, 0, 0, 0, 0, 0)
        rospy.sleep(0.5)

        move_cartesian_path(wpose.position.x, wpose.position.y, 0.0407)
        rospy.sleep(0.1)
        grab_object("medium")
        wpose = move_joints(0, 0, 0, 0, 0, 0)
        rospy.sleep(0.1)
        wpose = move_joints(0, 0, 0, 0, 0, 0)
        move_cartesian_path(wpose.position.x, wpose.position.y, 0.0407)
        rospy.sleep(0.1)
        lose_object()

        rospy.sleep(0.1)
    
except KeyboardInterrupt:
    rospy.loginfo("Ctrl+C pressed. Exiting...")

finally:
    # Return to home position when loop exits
    home()
    remove_box()