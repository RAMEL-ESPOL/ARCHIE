#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
#from gazebo_gripper import *

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
    rospy.sleep(0.1)

def remove_box(name, timeout=4):
    box_name = name
    scene.remove_world_object(box_name)
    rospy.sleep(0.1)

def home():
    joint_goal = arm_group.get_current_joint_values()
    joint_goal = [0, 0, 0, 0, 0, 0]
    arm_group.set_joint_value_target(joint_goal)
    
    # Plan the trajectory
    success, plan, _, _ = arm_group.plan()

    # Check if the plan is successful
    if success and len(plan.joint_trajectory.points) > 0:
        arm_group.execute(plan, wait=True)
    else:
        rospy.logerr("Failed to generate a valid plan for the joint values provided.")
    rospy.loginfo("ARCHIE is at home position.")

def move_cartesian_path(px, py, pz):
    waypoints = []

    wpose = arm_group.get_current_pose().pose 
    wpose.position.x = px
    wpose.position.y = py
    wpose.position.z = pz
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = arm_group.compute_cartesian_path(waypoints, t, 0.0)  # jump_threshold

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)

    # Publish
    display_trajectory_publisher.publish(display_trajectory)
    
    arm_group.execute(plan, wait=True)
    rospy.sleep(0.1)

def move_xyz(px,py,pz):
    q =[]
    orientation = arm_group.get_current_pose().pose 
    q.append(orientation.orientation.x)
    q.append(orientation.orientation.y)
    q.append(orientation.orientation.z)
    q.append(orientation.orientation.w)
    arm_group.set_orientation_target([-0.70,-0.7,0,0])
    arm_group.set_position_target([px,py,pz])
    arm_group.go()

    rospy.sleep(0.1)

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
    rospy.sleep(0.10)

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

    ee_group.attach_object("box", "garra1_link", touch_links)

    # Attach object using gazebo_gripper only if gazebo is enabled
    if use_gazebo:
        gazebo_gripper.attach_Object()

    rospy.loginfo("ARCHIE picked the object")
    rospy.sleep(0.1)

def lose_object():
    # Detach object using gazebo_gripper only if gazebo is enabled
    if use_gazebo:
        gazebo_gripper.detach_Object()

    ee_group.detach_object("garra1_link")

    joint_goal = ee_group.get_current_joint_values()
    joint_goal = [0.0, 0.0]

    ee_group.go(joint_goal, wait=True)

    rospy.loginfo("ARCHIE dropped the object")
    rospy.sleep(0.1)

# Main execution
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('planning_node')

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

touch_links = robot.get_link_names(group=ee_group.get_name())


# Initialize gazebo_gripper only if gazebo is used
if use_gazebo:
    gazebo_gripper = RoboticGripper()
    gazebo_gripper._Robot_Name = "robot"
    gazebo_gripper._EE_Link_Name = "link_6"
    gazebo_gripper._Box_Model_Name = "box"
    gazebo_gripper._Box_Link_Name = "link"

def menu():
    global quit    
    print("Menú:")
    print("1. Pick and place (derecha)")
    print("2. Pick and Place (izquierda)")
    print("3. Mano izquierda")
    print("q. SALIR")

    option = input("Seleccione una opción: ")

    if option == '1':
        move_joints(0, 0, 0, 0, 0, 0)
        
        move_cartesian_path(0, 0.24, 0.045)

        grab_object("medium")

        move_joints(-1.57, 0, 0, 0, 0, 0)

        move_joints(-1.57, -0.5703597221921625, -0.4866665737397319, 0, 1.057, 0)

        lose_object()

        move_joints(-1.57, 0, 0, 0, 0, 0)

        move_joints(-1.57, -0.5703597221921625, -0.4866665737397319, 0, 1.057, 0)

        grab_object("medium")

        move_joints(0, 0, 0, 0, 0, 0)

        move_cartesian_path(0, 0.24, 0.045)

        lose_object()

        move_joints(0, 0, 0, 0, 0, 0)
    elif option == '2':

        move_joints(0, 0, 0, 0, 0, 0)

        move_cartesian_path(0, 0.24, 0.045)

        grab_object("medium")

        move_joints(1.57, 0, 0, 0, 0, 0)

        move_joints(1.57, -0.5703597221921625, -0.4866665737397319, 0, 1.057, 0)

        lose_object()

        move_joints(1.57, 0, 0, 0, 0, 0)

        move_joints(1.57, -0.5703597221921625, -0.4866665737397319, 0, 1.057, 0)

        grab_object("medium")

        move_joints(0, 0, 0, 0, 0, 0)

        move_cartesian_path(0, 0.24, 0.045)

        lose_object()

        move_joints(0, 0, 0, 0, 0, 0)

    elif option == '3':

        move_joints(0, 0, 0, 0, 0, 0)

        move_cartesian_path(0, 0.24, 0.045)
        
        grab_object("medium")

        move_joints(-1.5699811638948276, 0.35076022743452813, -0.17050017674796938, 0, 0.014684045245539014, 0)

        lose_object()

        remove_box("box")
        
        move_joints(0, 0, 0, 0, 0, 0)
        

    elif option == 'q':
        quit = 1

    else:
        print("Opción inválida")

# Main loop for continuous pick and place
while not rospy.is_shutdown() and quit == 0:
    #ADD BOX rviz
    #rospy.logerr(arm_group.get_current_joint_values())
    addBox("box", 0, 0.3, 0.01, 0.02, 0.02, 0.02, scene)
    menu()

# Return to home position when loop exits
home()
remove_box("box")