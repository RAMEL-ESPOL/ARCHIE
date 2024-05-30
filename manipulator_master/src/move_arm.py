#!/usr/bin/env python3

import sys
import rospy
import numpy as np
from   sensor_msgs.msg import JointState
from std_msgs.msg import String
from geometry_msgs.msg import Point
import os
import math
import moveit_commander
from visualization_msgs.msg import Marker, MarkerArray

global pos
pos =       [0.0,0.0,0.0,0.0,0.0,0.0]
global real_pos
real_pos =  [0.0,0.0,0.0,0.0,0.0,0.0]
global torque
torque =    [0.0,0.0,0.0,0.0,0.0,0.0]
global real_vel
real_vel =  [0.0,0.0,0.0,0.0,0.0,0.0]

global ruta_file
ruta_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'matlab', 'data')

global new_file
new_file  = False

global fig
fig = '_none'

global marker_array
marker_array = MarkerArray()

global marker
marker = Marker()
marker.header.frame_id = "world"
marker.type = marker.POINTS
marker.action = marker.ADD
marker.scale.x = 0.003
marker.scale.y = 0.003
marker.scale.z = 0
marker.color.r = 0.0
marker.color.g = 0.0
marker.color.b = 0.0
marker.color.a = 1.0
marker.lifetime = rospy.Duration(10)

#Recibimos del Subscriber un msg de tipo JointState de moveit y posteriormente lo publicamos con el Publisher como goal
def state_position(goal_state: JointState):
    global pos
    if (goal_state.position != pos ):
        goal_state.velocity = [10, 10, 10, 10, 10, 10]
        pub.publish(goal_state)
        pos = goal_state.position
    j_array = np.array(pos)*180/math.pi
    pos = list(j_array)
    write_data(pos, "goals") #Funcion para guardar los datos
    if fig != '_none': 
        plan_marker()
    else:
        marker.points.clear()
        marker_array.markers.append(marker)

def plan_marker():
    global marker_array
    global marker
    pose = group.get_current_pose(group.get_end_effector_link())
    if (pose.pose.position.z <= 0.217 + 0.005) and (pose.pose.position.z >= 0.217 - 0.005):
        p = Point() 
        p = pose.pose.position
        p.z = 0.217 - 0.165
        
        marker.points.append(p)
        marker_array.markers.append(marker)
        marker_pub.publish(marker_array)

#Recibimos un msg de tipo JointState a traves del topico real_joint_state y lo guardamos para posteriormente hacer un controlador
def real_callback(real_state: JointState):
    real_position(real_state)
    #real_torque  (real_state)
    #real_velocity(real_state)

def real_position(real_state: JointState): 
    global real_pos
    j_array = np.array(real_state.position)*180/math.pi
    real_pos = list(j_array)
    write_data(real_pos, "real_states") #Funcion para guardar los datos
    
def real_torque(real_state: JointState): 
    global torque
    j_array = np.array(real_state.effort)
    torque= list(j_array)
    write_data(torque, "real_torques") #Funcion para guardar los datos

def real_velocity(real_state: JointState):
    global real_vel
    j_array = np.array(real_state.velocity)
    real_vel= list(j_array)
    write_data(real_vel, "real_velocities") #Funcion para guardar los datos

def figure(data_figure : str):
    global fig
    fig = str(data_figure.data)

def write_data(posiciones, pre):
    global new_file
    if fig != '_none':
        if os.path.exists(os.path.join(ruta_file, "joint_" + pre + fig + ".txt")):
            if not new_file:
                archivo = open(os.path.join(ruta_file, "joint_" + pre + fig + ".txt"), "w")
                archivo.write("Joint0,Joint1,Joint2,Joint3,Joint4,Joint5 \n" + ",".join(np.array(posiciones,str)) + "\n")
                archivo.close()
                new_file = True
            else:
                archivo = open(os.path.join(ruta_file, "joint_" + pre + fig + ".txt"), "a")
                archivo.write(",".join(np.array(posiciones,str)) + "\n")
                archivo.close()
        else:
            archivo = open(os.path.join(ruta_file, "joint_" + pre + fig + ".txt"), "w")
            archivo.write("Joint0,Joint1,Joint2,Joint3,Joint4,Joint5 \n" + ",".join(np.array(posiciones,str)) + "\n")
            archivo.close()
            new_file = True
    

if __name__ == "__main__":
    rospy.init_node("move_arm_node")
    moveit_commander.roscpp_initialize(sys.argv)
    pub             = rospy.Publisher ("/joint_goals" , JointState, queue_size=10)
    marker_pub      = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size = 2)
    subGoalState    = rospy.Subscriber("/joint_states", JointState, callback = state_position)
    subRealState    = rospy.Subscriber("/real_joint_states", JointState, callback = real_callback)
    subWritingData  = rospy.Subscriber("/figure_writing", String, callback = figure)
    group           = moveit_commander.MoveGroupCommander("arm_group")

    rospy.logwarn("The move_arm_node has been started")
    rospy.spin()