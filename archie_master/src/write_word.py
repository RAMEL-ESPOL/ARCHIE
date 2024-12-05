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
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import math

#Altura cuando se levanta el l
def home():
    # We get the joint values from the group and change some of the values:
    joint_goal = [0,0,0,0,0,0]
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

def joint_move(j0, j1, j2, j3, j4, j5):
    # Set the target joint values
    group.set_joint_value_target([j0, j1, j2, j3, j4, j5])
    
    # Plan the trajectory
    success, plan, _, _ = group.plan()

    # Check if the plan is successful
    if success and len(plan.joint_trajectory.points) > 0:
        group.execute(plan, wait=True)
    else:
        rospy.logerr("Failed to generate a valid plan for the joint values provided.")
    rospy.sleep(0.10)


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


def pen_up_down(wpose, waypoints : list, pen):
    wpose.position.z = pen + 0.02
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))

    return wpose, waypoints


def up_pen(wpose, waypoints : list, pen):
    wpose.position.z = pen + 0.02
    waypoints.append(copy.deepcopy(wpose))

    return wpose, waypoints


def down_pen(wpose, waypoints : list, pen):

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))

    return wpose, waypoints

def move_pen(wpose, waypoints : list, d_x : float, d_y: float, cond: bool = 0, d_z : float = 0):
    """Se copia la pose actual para únicamente modificar las coordenadas cartesianas y que la orientación del efector final no se vea modificada, de esta manera mantenemos el lápiz perpendicular al suelo"""

    wpose.position.y = (d_y if cond  else
                       (wpose.position.x + d_y))
    wpose.position.x = d_x
    if (d_z != 0):
        wpose.position.z = d_z

    waypoints.append(copy.deepcopy(wpose))

    return (wpose, waypoints)


def set_pen(wpose, waypoints : list, p_x : float, p_y: float, p_z : float = 0):
    
    wpose.position.y = p_y
    wpose.position.x = p_x
    wpose.position.z = p_z
    waypoints.append(copy.deepcopy(wpose))

    return (wpose, waypoints)


def plan_A(wpose, waypoints : list, size: float, space: float, y_h: float, pen):

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, 0.5*size)

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, size + space, y_h, 1)

    return (waypoints, wpose)


def plan_B(wpose, waypoints : list, size: float, space: float, y_h: float, pen):

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, size*0.85, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, size*0.15, -size*0.15)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -0.2*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size*0.15, -size*0.15)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size*0.85, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, size*0.85, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, size*0.15, -size*0.15)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size*0.2)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size*0.15, -size*0.15)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size*0.85, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, y_h, 1)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, size + space, 0)


    return (waypoints, wpose)


def plan_C(wpose, waypoints : list, size: float, space: float, y_h: float, pen):

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, space, y_h, 1)

    return (waypoints, wpose)


def plan_D(wpose, waypoints : list, size: float, space: float, y_h: float, pen):

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, size*0.85, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, size*0.15, -size*0.15)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size*0.7)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size*0.15, -size*0.15)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size*0.85, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, y_h, 1)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, size + space, 0)


    return (waypoints, wpose)


def plan_E(wpose, waypoints : list, size: float, space: float, y_h: float, pen):

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, -0.15*size, size*0.5)

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, -0.85*size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, size + space, y_h, 1)


    return (waypoints, wpose)


def plan_F(wpose, waypoints : list, size: float, space: float, y_h: float, pen):

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size*0.5)

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, space, y_h, 1)


    return (waypoints, wpose)


def plan_G(wpose, waypoints : list, size: float, space: float, y_h: float, pen):

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)
    
    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size*0.85, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size*0.15, -size*0.15)
    
    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size*0.7)

    (wpose, waypoints) = move_pen(wpose, waypoints, size*0.15, -size*0.15)

    (wpose, waypoints) = move_pen(wpose, waypoints, size*0.7, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, size*0.15, size*0.15)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size*0.3)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size*0.3, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, size*0.3 + space, y_h, 1)

    return (waypoints, wpose)


def plan_H(wpose, waypoints : list, size: float, space: float, y_h: float, pen):

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size/2)

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size/2)

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, space, y_h, 1)

    return (waypoints, wpose)


def plan_I(wpose, waypoints : list, size: float, space: float, y_h: float, pen):

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size*0.5, 0)

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size*0.5, 0)

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, space, y_h, 1)

    return (waypoints, wpose)


def plan_J(wpose, waypoints : list, size: float, space: float, y_h: float, pen):

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size*0.35, 0)

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size*0.5, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size*0.15, size*0.15)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, size + space, y_h, 1)
    

    return (waypoints, wpose)


def plan_K(wpose, waypoints : list, size: float, space: float, y_h: float, pen):

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, 0.5*size)

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, -0.5*size)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0.5*size)

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, y_h, 1)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, space, y_h, 1)


    return (waypoints, wpose)


def plan_L(wpose, waypoints : list, size: float, space: float, y_h: float, pen):

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, space, y_h, 1)


    return (waypoints, wpose)


def plan_M(wpose, waypoints : list, size: float, space: float, y_h: float, pen):

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.5*size, -0.5*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.5*size, 0.5*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, space, y_h, 1)


    return (waypoints, wpose)


def plan_N(wpose, waypoints : list, size: float, space: float, y_h: float, pen):

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, -size)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, y_h, 1)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, space, y_h, 1)


    return (waypoints, wpose)


def plan_O(wpose, waypoints : list, size: float, space: float, y_h: float, pen):

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, size + space, y_h, 1)    


    return (waypoints, wpose)


def plan_P(wpose, waypoints : list, size: float, space: float, y_h: float, pen):

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -0.5*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, size + space, y_h, 1)


    return (waypoints, wpose)


def plan_Q(wpose, waypoints : list, size: float, space: float, y_h: float, pen):

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.5*size, -0.5*size)

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.5*size, -0.5*size)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, space, y_h, 1)


    return (waypoints, wpose)


def plan_R(wpose, waypoints : list, size: float, space: float, y_h: float, pen):

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -0.5*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, -0.5*size)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, space, y_h, 1)


    return (waypoints, wpose)


def plan_S(wpose, waypoints : list, size: float, space: float, y_h: float, pen):

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size*0.5)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size*0.5)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, size + space, y_h, 1)
    
    
    return (waypoints, wpose)


def plan_T(wpose, waypoints : list, size: float, space: float, y_h: float, pen):

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)   

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, -0.5*size, 0)

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.5*size + space, y_h, 1)


    return (waypoints, wpose)


def plan_U(wpose, waypoints : list, size: float, space: float, y_h: float, pen):

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)    

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, y_h, 1)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, space, y_h, 1)


    return (waypoints, wpose)


def plan_V(wpose, waypoints : list, size: float, space: float, y_h: float, pen):

    (wpose, waypoints) = down_pen(wpose, waypoints, pen) 

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.5*size, -size)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.5*size, y_h, 1)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, space, 0)


    return (waypoints, wpose)


def plan_W(wpose, waypoints : list, size: float, space: float, y_h: float, pen):

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)  

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.5*size, 0.5*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.5*size, -0.5*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, y_h, 1)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, space, 0)


    return (waypoints, wpose)


def plan_X(wpose, waypoints : list, size: float, space: float, y_h: float, pen):

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, -size)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, size)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, space, y_h, 1)

    return (waypoints, wpose)


def plan_Y(wpose, waypoints : list, size: float, space: float, y_h: float, pen):

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size*0.5)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size*0.5)

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, size + space, y_h, 1)

    return (waypoints, wpose)


def plan_Z(wpose, waypoints : list, size: float, space: float, y_h: float, pen):

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, -size)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, space, y_h, 1)

    return (waypoints, wpose)


def plan_space(wpose, waypoints : list, size: float, space: float, y_h: float, pen):

    (wpose, waypoints) = move_pen(wpose, waypoints, space*5, y_h, 1)

    return (waypoints, wpose)


def plan_1(wpose, waypoints : list, size: float, space: float, y_h: float, pen):

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -0.3*size)
    
    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.5*size, 0.3*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, -0.5*size, 0)

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, space, y_h, 1)

    return (waypoints, wpose)


def plan_2(wpose, waypoints : list, size: float, space: float, y_h: float, pen):

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -0.5*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -0.5*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, space, y_h, 1)

    return (waypoints, wpose)


def plan_3(wpose, waypoints : list, size: float, space: float, y_h: float, pen):

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.2*size, 0.5*size)

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.8*size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, space, y_h, 1)

    return (waypoints, wpose)


def plan_4(wpose, waypoints : list, size: float, space: float, y_h: float, pen):

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -0.5*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, 0.5*size)

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, space, y_h, 1)

    return (waypoints, wpose)


def plan_5(wpose, waypoints : list, size: float, space: float, y_h: float, pen):

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -0.5*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.8*size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.2*size, -0.2*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -0.1*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, -0.2*size, -0.2*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, -0.8*size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, size + space, y_h, 1)

    return (waypoints, wpose)


def plan_6(wpose, waypoints : list, size: float, space: float, y_h: float, pen):

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, 0.5*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, size + space, y_h, 1)

    return (waypoints, wpose)


def plan_7(wpose, waypoints : list, size: float, space: float, y_h: float, pen):

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, -size)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, size + space, y_h, 1)

    return (waypoints, wpose)


def plan_8(wpose, waypoints : list, size: float, space: float, y_h: float, pen):

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -0.5*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -0.5*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, 0.5*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size*0.5)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, size + space, y_h, 1)

    return (waypoints, wpose)


def plan_9(wpose, waypoints : list, size: float, space: float, y_h: float, pen):

    (wpose, waypoints) = move_pen(wpose, waypoints, size, -size)

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -0.5*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, space, y_h, 1)

    return (waypoints, wpose)


def plan_0(wpose, waypoints : list, size: float, space: float, y_h: float, pen):

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.15*size, 0)

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)
    
    (wpose, waypoints) = move_pen(wpose, waypoints, 0.7*size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.15*size, -0.15*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -0.7*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, -0.15*size, -0.15*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, -0.7*size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, -0.15*size, 0.15*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, 0.7*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.15*size, 0.15*size)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.85*size + space, y_h, 1)

    return (waypoints, wpose)


def plan_minus(wpose, waypoints : list, size: float, space: float, y_h: float, pen):

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -0.5*size)

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.5*size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 2.5*space, y_h, 1)

    return (waypoints, wpose)


def plan_plus(wpose, waypoints : list, size: float, space: float, y_h: float, pen):

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.25*size, -0.25*size)

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -0.5*size)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, -0.25*size, 0.25*size)

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.5*size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 2.5*space, y_h, 1)

    return (waypoints, wpose)


def plan_times(wpose, waypoints : list, size: float, space: float, y_h: float, pen):

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.1*size, -0.25*size)

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.5*size, -0.5*size)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, -0.5*size, 0)

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.5*size, 0.5*size)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 1.5*space, y_h, 1)

    return (waypoints, wpose)


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

    return wpose, circle_waypoints


def plan_divide(wpose, waypoints : list, size: float, space: float, y_h: float, pen):

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.15*size + 0.05*size, -0.2*size)

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = plan_circle(wpose.position.x + space/2, wpose.position.y, 0.05*size, 0, 360, wpose, waypoints, 0, 1)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)
    
    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -0.6*size)

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = plan_circle(wpose.position.x + space/2, wpose.position.y, 0.05*size, 0, 360, wpose, waypoints, 0, 1)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, -0.25*size, 0.35*size)

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.5*size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 1.5*space, y_h, 1)

    return (waypoints, wpose)


def plan_equal(wpose, waypoints : list, size: float, space: float, y_h: float, pen):

    (wpose, waypoints) = move_pen(wpose, waypoints, 2*space, -0.33*size)

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.5*size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -0.33*size)

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, -0.5*size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.5*size + 2.5*space, y_h, 1)

    return (waypoints, wpose)


def plan_left_parenthesis(wpose, waypoints : list, size: float, space: float, y_h: float, pen):

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.15*size + 1.5*space, 0)

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, -0.15*size, -0.15*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -0.7*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.15*size, -0.15*size)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 2.5*space, y_h, 1)

    return (waypoints, wpose)


def plan_right_parenthesis(wpose, waypoints : list, size: float, space: float, y_h: float, pen):

    (wpose, waypoints) = move_pen(wpose, waypoints, 2.5*space, 0)

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.15*size, -0.15*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -0.7*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, -0.15*size, -0.15*size)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.15*size + 2.5*space, y_h, 1)

    return (waypoints, wpose)


def plan_exclamation(wpose, waypoints : list, size: float, space: float, y_h: float, pen):

    (wpose, waypoints) = move_pen(wpose, waypoints, 1.5*space, 0)

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -0.7*size)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, -0.05*size, -0.25*size)

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = plan_circle(wpose.position.x + 0.05*size, wpose.position.y, 0.05*size, 0, 360, wpose, waypoints, 0, 1)

    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, 2.5*space, y_h, 1)

    return (waypoints, wpose)


def square(wpose, waypoints: list):
    square_size = size
    figure = "Square (" + str(square_size) + "x" + str(square_size) + ")"
    figure_message = "_square"
    
    (wpose, waypoints) = set_pen(wpose, waypoints, -square_size/2, y_h, pen + 0.02)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -square_size)

    (wpose, waypoints) = move_pen(wpose, waypoints, square_size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, y_h)
    
    (wpose, waypoints) = move_pen(wpose, waypoints, -square_size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints)
        
    return waypoints, wpose, figure, figure_message


def triangle(wpose, waypoints: list, data_writing_publisher, size: float, x_i: float,  y_h: float, pen: float, theta: float = 0):

    (wpose, waypoints) = set_pen(wpose, waypoints, x_i, y_h, pen + 0.02)

    (wpose, waypoints) = down_pen(wpose, waypoints, pen)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size*math.cos(60*math.pi/180), -size*math.sin(60*math.pi/180))

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size*math.cos(60*math.pi/180), y_h, 1)
    
    (wpose, waypoints) = up_pen(wpose, waypoints, pen)

    return (waypoints, wpose)



def circle(wpose, waypoints: list):
    r = size/2
    figure = "Circle ( " + str(r) + " )"
    center_y = y_h - r
    center_x = 0 
    figure_message = "_circle"

    (wpose, waypoints) = set_pen(wpose, waypoints, 0, y_h, pen + 0.02)
    
    (wpose, waypoints) = plan_circle(center_x, center_y, r, 90, 450, wpose, waypoints, 1, 1)

    (wpose, waypoints) = up_pen(wpose, waypoints)
        
    return waypoints, wpose, figure, figure_message


def espol_logo(wpose, waypoints: list):
    figure = "ESPOL (LOGO)"
    r = size/2
    figure_message = "_espol_logo"
    
    (wpose, waypoints) = set_pen(wpose, waypoints, -(2*(size + space)) - r, y_h, pen + 0.02)
    
    #Planeamiento de la "e"
    (wpose, waypoints) = move_pen(wpose, waypoints, r, -r)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = plan_circle(wpose.position.x, wpose.position.y, r, 45, 270, wpose, waypoints , 1 , 1)

    #Planemiento de la "s"
    (wpose, waypoints) = move_pen(wpose, waypoints, space + r, y_h)
    
    (wpose, waypoints) = plan_circle(wpose.position.x, wpose.position.y - r, r, 90, 290, wpose, waypoints , 0 , 1)
    
    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, space + r*(1-math.cos(290 * math.pi/180)), abs(r*math.sin(math.radians(290))))
    
    (wpose, waypoints) = down_pen(wpose, waypoints)

    #Planeamiento de la "p"
    (wpose, waypoints) = plan_circle(wpose.position.x + r, wpose.position.y, r, 0, 360, wpose, waypoints, 0, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size*0.7)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, space + size, y_h)

    #Planeamiento de la "o"
    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -r)

    (wpose, waypoints) = plan_circle(wpose.position.x + r, wpose.position.y, r, 0, 360, wpose, waypoints, 0, 1)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, space + size, y_h)

    #Planeamiento de la "l"
    (wpose, waypoints) = move_pen(wpose, waypoints, 0, r*0.2)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -r*0.2 - r)

    (wpose, waypoints) = plan_circle(wpose.position.x, wpose.position.y + r, r, 180, 270, wpose, waypoints, 1, 1)

    (wpose, waypoints) = up_pen(wpose, waypoints)
        
    return waypoints, wpose, figure, figure_message


def espol(wpose, waypoints : list):
    figure = "ESPOL"
    figure_message = "_espol"

    (wpose, waypoints) = set_pen(wpose, waypoints, -(2*(space + size)) - size/2 + size, y_h, pen + 0.002)
    
    (wpose, waypoints) = down_pen(wpose, waypoints)

    #Drawing the "E"
    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, 0.007, pen + 0.02)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -0.007)
    
    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size/2, pen + 0.02)
    
    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size/2)
    
    (wpose, waypoints) = down_pen(wpose, waypoints)
    
    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size/2, pen + 0.02)
    
    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)
    
    (wpose, waypoints) = up_pen(wpose, waypoints)
    

    #Drawing the "S"
    (wpose, waypoints) = move_pen(wpose, waypoints, 2*size + space, y_h)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = pen_up_down(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size/2)
    
    (wpose, waypoints) = move_pen(wpose, waypoints, 0, 0.01, pen + 0.02)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -0.01)

    (wpose, waypoints) = down_pen(wpose, waypoints)
    
    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size/2, pen + 0.02)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size/2)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size/2)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size/2, pen + 0.02)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size/2)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size/2, pen + 0.02)

    (wpose, waypoints) = move_pen(wpose, waypoints, space + size, y_h)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    #Drawing the "P"
    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size/2, pen + 0.02)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size/2)
    
    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = pen_up_down(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size/2)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size/2, pen + 0.02)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size/2)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size/2, pen + 0.02)

    (wpose, waypoints) = move_pen(wpose, waypoints, space + size, y_h)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    
    #Drawing the "O"
    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = pen_up_down(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size/2, pen + 0.02)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, size/2)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size/2, pen + 0.02)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size/2)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size/2, pen + 0.02)

    (wpose, waypoints) = move_pen(wpose, waypoints, space, y_h)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    
    #Drawing the "L"
    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size/2, pen + 0.02)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size/2)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints)
        
    return waypoints, wpose, figure, figure_message


def plan_(wpose, waypoints : list, size: float, space: float, y_h: float, pen):

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, 0)

    return (waypoints, wpose)


def write(wpose, waypoints: list, robot, scene, group, display_trajectory_publisher, data_writing_publisher, word: str, y_h2: float = 1.0, x_i: float = -0.75, size2: float = 0.07, space2: float = 0.01, pen: float = 1, theta: float = 0, t : float = 0.001):
    """Función para escribir una palabra enviando todos los parámetros necesarios para esto, no es necesario enviar pen y theta como parámetros"""
    size  = size2
    y_h   = y_h2
    space = space2
    data_writing_publisher.publish(("_none," + str(pen)))
    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()
    # wpose = group.get_current_pose().pose


    if ((((space + size)*len(word)) - (space/2 + size/2)*(word.count("+") + word.count("-") + word.count(" ") + word.count("*") + word.count("/") + word.count("(") + word.count(")"))) <= 2):
        # waypoints = []

        # x_i = -1*(len(word)/2 * (size + space))#Cálculo de la posición inicial del lápiz
        #Moviendo lápiz a la posición inicial
        (wpose, waypoints) = set_pen(wpose, waypoints, x_i, y_h2, pen + 0.02)

        for w in word.upper():
            (waypoints, wpose) = (plan_A(wpose,waypoints,size2,space2,y_h2,pen) if w == "A" else
                                 (plan_B(wpose,waypoints,size2,space2,y_h2,pen) if w == "B" else
                                 (plan_C(wpose,waypoints,size2,space2,y_h2,pen) if w == "C" else
                                 (plan_D(wpose,waypoints,size2,space2,y_h2,pen) if w == "D" else
                                 (plan_E(wpose,waypoints,size2,space2,y_h2,pen) if w == "E" else
                                 (plan_F(wpose,waypoints,size2,space2,y_h2,pen) if w == "F" else
                                 (plan_G(wpose,waypoints,size2,space2,y_h2,pen) if w == "G" else
                                 (plan_H(wpose,waypoints,size2,space2,y_h2,pen) if w == "H" else
                                 (plan_I(wpose,waypoints,size2,space2,y_h2,pen) if w == "I" else
                                 (plan_J(wpose,waypoints,size2,space2,y_h2,pen) if w == "J" else
                                 (plan_K(wpose,waypoints,size2,space2,y_h2,pen) if w == "K" else
                                 (plan_L(wpose,waypoints,size2,space2,y_h2,pen) if w == "L" else
                                 (plan_M(wpose,waypoints,size2,space2,y_h2,pen) if w == "M" else
                                 (plan_N(wpose,waypoints,size2,space2,y_h2,pen) if w == "N" else
                                 (plan_O(wpose,waypoints,size2,space2,y_h2,pen) if w == "O" else
                                 (plan_P(wpose,waypoints,size2,space2,y_h2,pen) if w == "P" else
                                 (plan_Q(wpose,waypoints,size2,space2,y_h2,pen) if w == "Q" else
                                 (plan_R(wpose,waypoints,size2,space2,y_h2,pen) if w == "R" else
                                 (plan_S(wpose,waypoints,size2,space2,y_h2,pen) if w == "S" else
                                 (plan_T(wpose,waypoints,size2,space2,y_h2,pen) if w == "T" else
                                 (plan_U(wpose,waypoints,size2,space2,y_h2,pen) if w == "U" else
                                 (plan_V(wpose,waypoints,size2,space2,y_h2,pen) if w == "V" else
                                 (plan_W(wpose,waypoints,size2,space2,y_h2,pen) if w == "W" else
                                 (plan_X(wpose,waypoints,size2,space2,y_h2,pen) if w == "X" else
                                 (plan_Y(wpose,waypoints,size2,space2,y_h2,pen) if w == "Y" else
                                 (plan_Z(wpose,waypoints,size2,space2,y_h2,pen) if w == "Z" else 
                                 (plan_1(wpose,waypoints,size2,space2,y_h2,pen) if w == "1" else 
                                 (plan_2(wpose,waypoints,size2,space2,y_h2,pen) if w == "2" else 
                                 (plan_3(wpose,waypoints,size2,space2,y_h2,pen) if w == "3" else 
                                 (plan_4(wpose,waypoints,size2,space2,y_h2,pen) if w == "4" else 
                                 (plan_5(wpose,waypoints,size2,space2,y_h2,pen) if w == "5" else 
                                 (plan_6(wpose,waypoints,size2,space2,y_h2,pen) if w == "6" else 
                                 (plan_7(wpose,waypoints,size2,space2,y_h2,pen) if w == "7" else 
                                 (plan_8(wpose,waypoints,size2,space2,y_h2,pen) if w == "8" else 
                                 (plan_9(wpose,waypoints,size2,space2,y_h2,pen) if w == "9" else 
                                 (plan_0(wpose,waypoints,size2,space2,y_h2,pen) if w == "0" else 
                                 (plan_space (wpose,waypoints,size2,space2,y_h2,pen) if w == " " else                               
                                 (plan_plus  (wpose,waypoints,size2,space2,y_h2,pen) if w == "+" else 
                                 (plan_minus (wpose,waypoints,size2,space2,y_h2,pen) if w == "-" else 
                                 (plan_times (wpose,waypoints,size2,space2,y_h2,pen) if w == "*" else 
                                 (plan_divide(wpose,waypoints,size2,space2,y_h2,pen) if w == "/" else
                                 (plan_equal (wpose,waypoints,size2,space2,y_h2,pen) if w == "=" else 
                                 (plan_left_parenthesis (wpose,waypoints,size2,space2,y_h2,pen) if w == "(" else 
                                 (plan_right_parenthesis(wpose,waypoints,size2,space2,y_h2,pen) if w == ")" else 
                                 (plan_exclamation      (wpose,waypoints,size2,space2,y_h2,pen) if w == "!" else
                                 [])))))))))))))))))))))))))))))))))))))))))))))
        
        data_writing_publisher.publish("_" + str(word).lower() + "," + str(pen))


        return (waypoints, wpose)
            
    else:
        rospy.logerr("The word has too many letters.")
        return ([], wpose)

if __name__ == "__main__":

    # Altura del lapiz
    pen = 0.22

    #El paso que habrá entre una coordenada y la siguiente a la hora de escribir
    t = 0.01

    #Altura máxima a la que llegará cada letra en Y
    y_h = 0.23

    #Tamaño de cada letra en ancho y alto
    size = 0.02

    #Espacio entre cada letra
    space = 0.005

    theta = 0

    global rmatrix
    x_i = -0.5


    #By executing this file we can make the robot move to several preconfigured positions in Cartesian coordinates, in the order in which they are in the file
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('planing_node', anonymous=True)
    rate = rospy.Rate(10)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()    
    group = moveit_commander.MoveGroupCommander("arm_group")
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
    data_writing_publisher       = rospy.Publisher('/figure_writing', String, queue_size=2)

    home()
    
    rospy.sleep(2)
    word = input("\n--------------------\nWrite the word you want the robotic arm write: ").upper()

    wpose = group.get_current_pose().pose
    waypoints = []

    (waypoints, wpose) = write(wpose, waypoints, robot, scene, group, display_trajectory_publisher, data_writing_publisher, word.upper(), y_h , -1*((len(word)/2) * (size + space)), 1.2*size, space, pen, theta, t )

    plan  = group.compute_cartesian_path(waypoints, t, 0.0)[0]

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory)

    group.execute(plan, wait=True)
    rospy.loginfo("Planning succesfully executed.\n")
    rospy.sleep(1)


