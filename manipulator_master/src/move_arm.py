#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import JointState
import os
import math

global pos
pos = [0.0,0.0,0.0,0.0,0.0,0.0]
global real_pos
real_pos = []
global real_load
real_load = []
global real_vel
real_vel = []
global ruta_actual
ruta_actual = os.path.dirname(os.path.abspath(__file__))
global ruta_file
ruta_file = os.path.join(ruta_actual, '..', '..', 'matlab')

#Recibimos del Subscriber un msg de tipo JointState de moveit y posteriormente lo publicamos con el Publisher como goal
def state_position(state: JointState):
    global pos
    if (state.position != pos ):
        pub.publish(state)
        pos = state.position
    pos_array = math.degrees(np.array(pos))
    pos = list(pos_array)
    escribir_datos(pos, "goals") #Funcion para guardar los datos

#Recibimos un msg de tipo JointState a traves del topico real_joint_state y lo guardamos para posteriormente hacer un controlador
def real_callback(state: JointState):
    real_position(state)
    real_loa(state)
    real_velocity(state)

def real_position(state: JointState): 
    pos_array = math.degrees((np.array(state.position)))
    real_pos = list(pos_array)
    escribir_datos(real_pos, "states") #Funcion para guardar los datos
    
def real_loa(state: JointState): 
    pos_array = np.array(state.effort)
    real_load= list(pos_array)
    escribir_datos(real_load, "load") #Funcion para guardar los datos

def real_velocity(state: JointState):
    pos_array = np.array(state.velocity)
    real_vel= list(pos_array)
    escribir_datos(real_vel, "vel") #Funcion para guardar los datos


def escribir_datos(posiciones, pre):
    if os.stat(os.path.join(ruta_file, pre + ".txt")).st_size == 0:
        archivo = open(os.path.join(ruta_file, pre + ".txt"), "w")
        archivo.write("Joint0,Joint1,Joint2,Joint3,Joint4,Joint5 \n")
        archivo.close()

    archivo = open(os.path.join(ruta_file, pre + ".txt"), "a")
    posiciones_array = np.array(posiciones,str)
    posiciones_str = ",".join(posiciones_array)
    archivo.write(posiciones_str + "\n")
    archivo.close()

if __name__ == "__main__":
    #archivo = open("/home/erick/catkin_ws/src/manipulator/matlab/datos_plano_mesa.txt", "w")
    #archivo.write("Joint0,Joint1,Joint2,Joint3,Joint4,Joint5 \n")
    #archivo.close()

    rospy.init_node("move_arm_node")
    pub = rospy.Publisher ("/joint_goals" , JointState, queue_size=10)
    subRobotState = rospy.Subscriber("/joint_states", JointState, callback=state_position)
    subRealState  = rospy.Subscriber("/real_joint_states", JointState, callback= real_callback)
    rospy.logwarn("The move_arm_node has been started")
    rospy.spin()

archivo.close()


