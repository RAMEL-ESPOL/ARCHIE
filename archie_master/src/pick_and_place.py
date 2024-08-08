#! /usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import math
from spatialmath_rospy import to_spatialmath, to_ros
from spatialmath import SE3, SO3
from gazebo_gripper import *

import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

# Altura del lapiz
global z

global quit
quit = 0

global t
t = 0.01


def addBox(name,x,y,z, sx,sy,sz, scene):
    box_name = name
	    
	## First, create an message object of type pose to deifne box positions
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "world"
	    
	    #set the position massage arguments
    box_pose.pose.position.x = x
    box_pose.pose.position.y = y 
    box_pose.pose.position.z = z 
    box_pose.pose.orientation.x = 0
    box_pose.pose.orientation.y = 0
    box_pose.pose.orientation.z = 0
    box_pose.pose.orientation.w = 1.0
	#Add the box in the scean
    scene.add_box(box_name, box_pose, size=(sx, sy, sz))


def home():
    joint_goal = arm_group.get_current_joint_values()
    #Para el plano inclinado es un home diferente
    #joint_goal = [1.267872420911887e-06, 0.16714812767180828, 0.24872782685877684, -3.421257973905653e-07, -0.41587595452500437, -3.5425905235787057e-07]
    #Home:
    joint_goal = [0,0,0,0,0,0]

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    arm_group.go(joint_goal, wait=True)
    rospy.loginfo("ARCHIE is at home position.")
    return wpose

def move_cartesian_path(px,py,pz):
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
    pass

def move_joints(j0, j1, j2, j3, j4, j5):
    # Configurar el objetivo de los valores de las articulaciones
    arm_group.set_joint_value_target([j0, j1, j2, j3, j4, j5])
    
    # Planifica la trayectoria hacia los valores de las articulaciones deseadas
    
    success, plan ,mul ,val = arm_group.plan()

    # Verificar si el plan es exitoso y tiene puntos de trayectoria
    if success and len(plan.joint_trajectory.points) > 0:
        # Ejecuta el plan
        arm_group.execute(plan, wait=True)
    else:
        rospy.logerr("No se pudo generar un plan válido para los valores de las articulaciones proporcionados.")


def loginfog(msg: str):
    rospy.loginfo("\033[92m%s\033[0m" % msg)

def grab_object(size):
    joint_goal = ee_group.get_current_joint_values()
    joint_goal = [0.0,0.0]
    if size=="medium":
        joint_goal = [0.012,0.0]
    if size=="small":
        joint_goal = [0.019,0.0]

    ee_group.go(joint_goal, wait=True)

    touch_links = robot.get_link_names(group= ee_group.get_name())
    ee_group.attach_object("box", "garra1_link" , touch_links)
    gazebo_gripper.attach_Object()
    rospy.loginfo("ARCHIE picked the object")

def lose_object():
    joint_goal = ee_group.get_current_joint_values()
    joint_goal = [0.0,0.0]
    ee_group.go(joint_goal, wait=True)
    gazebo_gripper.detach_Object()

    touch_links = robot.get_link_names(group= ee_group.get_name())
    ee_group.detach_object("garra1_link")
    rospy.loginfo("ARCHIE drop the object")

    

#By executing this file we can make the robot move to several preconfigured positions in 
# Cartesian coordinates, in the order in which they are in the file
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('planing_node')
rate = rospy.Rate(10)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()    
arm_group = moveit_commander.MoveGroupCommander("arm_group")
ee_group = moveit_commander.MoveGroupCommander("ee_group")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
data_writing_publisher       = rospy.Publisher('/figure_writing', String, queue_size=2)
data_writing_publisher.publish(("_none"))

#Grasping group para el attach en rviz
grasping_group = "ee_group"


#ADD BOX rviz
addBox("box", 0,0.3,0.01,0.02,0.02,0.02,scene)
wpose = arm_group.get_current_pose().pose
rospy.logerr(wpose)

#Gazebo gripper para attach objects
gazebo_gripper = RoboticGripper()
gazebo_gripper._Robot_Name = "robot"
gazebo_gripper._EE_Link_Name = "link_6"
gazebo_gripper._Box_Model_Name = "box"
gazebo_gripper._Box_Link_Name = "link"

move_cartesian_path(0,0.24,0.0407)

rospy.sleep(0.1)
rospy.logerr(arm_group.get_current_joint_values())
grab_object("medium")

rospy.sleep(0.1)

move_joints(-1.5699811638948276, 0.35076022743452813, -0.17050017674796938, 0, 0.014684045245539014, 0)

rospy.sleep(0.1)
 
lose_object()

rospy.sleep(0.1)

move_joints(0,0,0,0,0,0)


while (not rospy.is_shutdown() and quit == 0):
    
    rospy.sleep(1)
