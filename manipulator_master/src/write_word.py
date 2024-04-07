#! /usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from moveit_commander.conversions import pose_to_list
import math

# Altura del lapiz
global pen 
pen = 0.22

#El paso que habrá entre una coordenada y la siguiente a la hora de escribir
global t
t = 0.01

#Altura máxima a la que llegará cada letra en Y
global y_h 
y_h = 0.225

#Tamaño de cada letra
global size
size = 0.05

#Espacio entre cada letra
global space
space = 0.01

def home():
    # We get the joint values from the group and change some of the values:
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = 0
    joint_goal[2] = 0
    joint_goal[3] = 0
    joint_goal[4] = 0
    joint_goal[5] = 0

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)
    rospy.loginfo("The robotic arm is at home position.")

def loginfog(msg: str):
    rospy.loginfo("\033[92m%s\033[0m" % msg)

def print_plan(w: list, s: str):
    message = "\n--------------------------------------------------------------"
    for i in range(len(w)):
        message += """
Pose {0}:\n{1}
--------------------------------------------------------------""".format( i , w[i])
    
    rospy.loginfo(message)
    loginfog("Drawing a " + s)

def plan_circle( center_x : float , center_y : float , r : float , theta_o : float  , theta_f : float , wpose, circle_waypoints : list , sentido_x : bool, sentido_y : bool):
    if (sentido_x and sentido_y):
        for theta in range(theta_o, theta_f + 1, 2):
            wpose.position.y = center_y + r*math.sin(theta*math.pi/180)
            wpose.position.x = center_x + r*math.cos(theta*math.pi/180)
            circle_waypoints.append(copy.deepcopy(wpose))
    elif (not(sentido_x) and sentido_y):
        for theta in range(theta_o, theta_f + 1, 2):
            wpose.position.y = center_y + r*math.sin(theta*math.pi/180)
            wpose.position.x = center_x - r*math.cos(theta*math.pi/180)
            circle_waypoints.append(copy.deepcopy(wpose))
    elif (sentido_x and not(sentido_y)):
        for theta in range(theta_o, theta_f + 1, 2):
            wpose.position.y = center_y - r*math.sin(theta*math.pi/180)
            wpose.position.x = center_x + r*math.cos(theta*math.pi/180)
            circle_waypoints.append(copy.deepcopy(wpose))
    else:
        for theta in range(theta_o, theta_f + 1, 2):
            wpose.position.y = center_y - r*math.sin(theta*math.pi/180)
            wpose.position.x = center_x - r*math.cos(theta*math.pi/180)
            circle_waypoints.append(copy.deepcopy(wpose))

    return circle_waypoints, wpose

def plan_A(wpose, waypoints : list, x_0 : float):

    wpose.position.y -= size
    waypoints.append(copy.deepcopy(wpose))    

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))    

    wpose.position.y += size
    waypoints.append(copy.deepcopy(wpose))    

    wpose.position.x += size
    waypoints.append(copy.deepcopy(wpose))    

    wpose.position.y -= size
    waypoints.append(copy.deepcopy(wpose))    

    wpose.position.y += size/2
    waypoints.append(copy.deepcopy(wpose))    

    wpose.position.x -= size
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = pen + 0.05
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y  = y_h
    wpose.position.x += size + space
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = group.compute_cartesian_path(
        waypoints, t, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold

    return plan


def plan_B(wpose, waypoints : list, x_0 : float):

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))    


    (plan, fraction) = group.compute_cartesian_path(
        waypoints, t, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold

    return plan


def plan_C(wpose, waypoints : list, x_0 : float):

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))    


    (plan, fraction) = group.compute_cartesian_path(
        waypoints, t, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold

    return plan


def plan_D(wpose, waypoints : list, x_0 : float):

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))    


    (plan, fraction) = group.compute_cartesian_path(
        waypoints, t, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold

    return plan


def plan_E(wpose, waypoints : list, x_0 : float):

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))    


    (plan, fraction) = group.compute_cartesian_path(
        waypoints, t, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold

    return plan


def plan_F(wpose, waypoints : list, x_0 : float):

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))    


    (plan, fraction) = group.compute_cartesian_path(
        waypoints, t, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold

    return plan


def plan_G(wpose, waypoints : list, x_0 : float):

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))    


    (plan, fraction) = group.compute_cartesian_path(
        waypoints, t, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold

    return plan


def plan_H(wpose, waypoints : list, x_0 : float):

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))    


    (plan, fraction) = group.compute_cartesian_path(
        waypoints, t, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold

    return plan


def plan_I(wpose, waypoints : list, x_0 : float):

    wpose.position.x += size/2
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))    

    wpose.position.y -= size
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = pen + 0.05
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y = y_h
    wpose.position.x += size/2 + space
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = group.compute_cartesian_path(
        waypoints, t, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold

    return plan


def plan_J(wpose, waypoints : list, x_0 : float):

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))    


    (plan, fraction) = group.compute_cartesian_path(
        waypoints, t, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold

    return plan


def plan_K(wpose, waypoints : list, x_0 : float):

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))    


    (plan, fraction) = group.compute_cartesian_path(
        waypoints, t, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold

    return plan


def plan_L(wpose, waypoints : list, x_0 : float):

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))    


    (plan, fraction) = group.compute_cartesian_path(
        waypoints, t, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold

    return plan


def plan_M(wpose, waypoints : list, x_0 : float):

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))    


    (plan, fraction) = group.compute_cartesian_path(
        waypoints, t, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold

    return plan


def plan_N(wpose, waypoints : list, x_0 : float):

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))    


    (plan, fraction) = group.compute_cartesian_path(
        waypoints, t, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold

    return plan


def plan_O(wpose, waypoints : list, x_0 : float):

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))    


    (plan, fraction) = group.compute_cartesian_path(
        waypoints, t, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold

    return plan


def plan_P(wpose, waypoints : list, x_0 : float):

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))    


    (plan, fraction) = group.compute_cartesian_path(
        waypoints, t, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold

    return plan


def plan_Q(wpose, waypoints : list, x_0 : float):

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))    


    (plan, fraction) = group.compute_cartesian_path(
        waypoints, t, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold

    return plan


def plan_R(wpose, waypoints : list, x_0 : float):

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))    


    (plan, fraction) = group.compute_cartesian_path(
        waypoints, t, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold

    return plan


def plan_S(wpose, waypoints : list, x_0 : float):

    wpose.position.x += size
    waypoints.append(copy.deepcopy(wpose))    

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))    

    wpose.position.x -= size
    waypoints.append(copy.deepcopy(wpose))    

    wpose.position.y -= size/2
    waypoints.append(copy.deepcopy(wpose))    

    wpose.position.x += size
    waypoints.append(copy.deepcopy(wpose))    
    
    wpose.position.y -= size/2
    waypoints.append(copy.deepcopy(wpose))    

    wpose.position.x -= size
    waypoints.append(copy.deepcopy(wpose))    
    
    wpose.position.z = pen + 0.05
    waypoints.append(copy.deepcopy(wpose))    
    
    wpose.position.x += size + space
    wpose.position.y = y_h
    waypoints.append(copy.deepcopy(wpose))    
    
    
    (plan, fraction) = group.compute_cartesian_path(
        waypoints, t, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold

    return plan


def plan_T(wpose, waypoints : list, x_0 : float):

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))    


    (plan, fraction) = group.compute_cartesian_path(
        waypoints, t, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold

    return plan


def plan_U(wpose, waypoints : list, x_0 : float):

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))    


    (plan, fraction) = group.compute_cartesian_path(
        waypoints, t, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold

    return plan


def plan_V(wpose, waypoints : list, x_0 : float):

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))    


    (plan, fraction) = group.compute_cartesian_path(
        waypoints, t, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold

    return plan


def plan_W(wpose, waypoints : list, x_0 : float):

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))    


    (plan, fraction) = group.compute_cartesian_path(
        waypoints, t, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold

    return plan


def plan_X(wpose, waypoints : list, x_0 : float):

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))    


    (plan, fraction) = group.compute_cartesian_path(
        waypoints, t, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold

    return plan


def plan_Y(wpose, waypoints : list, x_0 : float):

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))    


    (plan, fraction) = group.compute_cartesian_path(
        waypoints, t, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold

    return plan


def plan_Z(wpose, waypoints : list, x_0 : float):

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))    


    (plan, fraction) = group.compute_cartesian_path(
        waypoints, t, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold

    return plan



def line():
    figure = "ESPOL"
    waypoints = []

    wpose = group.get_current_pose().pose

    wpose.position.z = pen + 0.055
    waypoints.append(copy.deepcopy(wpose))
    
    wpose.position.y = 0.225
    wpose.position.x = -0.25
    waypoints.append(copy.deepcopy(wpose))   




    (plan, fraction) = group.compute_cartesian_path(
        waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold

    print_plan(waypoints, figure)
    return plan

#By executing this file we can make the robot move to several preconfigured positions in Cartesian coordinates, in the order in which they are in the file
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('planing_node', anonymous=True)
rate = rospy.Rate(10)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()    
group = moveit_commander.MoveGroupCommander("arm_group")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

home()
# Calling ``stop()`` ensures that there is no residual movement
group.stop()
word = list(np.array("Aisa".split(), str))

if (((space + size)*len(word)) <= 0.5):
    plan = line()

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory)

    group.execute(plan, wait=True)
    rospy.loginfo("Planning succesfully executed.\n")
else:
    rospy.logerr("The word has too many letters.")
home()
