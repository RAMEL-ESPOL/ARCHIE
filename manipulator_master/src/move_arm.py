#!/usr/bin/env python3

import rospy
import numpy as np
from   sensor_msgs.msg import JointState
from std_msgs.msg import String
import os
import math

global pos
pos =       [0.0,0.0,0.0,0.0,0.0,0.0]
global real_pos
real_pos =  [0.0,0.0,0.0,0.0,0.0,0.0]
global torque
torque =    [0.0,0.0,0.0,0.0,0.0,0.0]
global real_vel
real_vel =  [0.0,0.0,0.0,0.0,0.0,0.0]

global ruta_file
ruta_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'matlab')

global new_file
new_file  = False

global fig
fig = '_none'

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

#Recibimos un msg de tipo JointState a traves del topico real_joint_state y lo guardamos para posteriormente hacer un controlador
def real_callback(real_state: JointState):
    real_position(real_state)
    real_torque  (real_state)
    real_velocity(real_state)

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

    rospy.logerr(data_figure)

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
    pub             = rospy.Publisher ("/joint_goals" , JointState, queue_size=10)
    subGoalState    = rospy.Subscriber("/joint_states", JointState, callback = state_position)
    subRealState    = rospy.Subscriber("/real_joint_states", JointState, callback = real_callback)
    subWritingData  = rospy.Subscriber("/figure_writing", String, callback = figure)
    rospy.logwarn("The move_arm_node has been started")
    rospy.spin()