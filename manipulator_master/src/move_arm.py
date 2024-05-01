#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import JointState

global pos
global real_pos
pos = [0.0,0.0,0.0,0.0,0.0,0.0]
real_pos = []
real_load = []

#Recibimos del Subscriber un msg de tipo JointState de moveit y posteriormente lo publicamos con el Publisher como goal
def state_position(state: JointState):
    global pos
    if (state.position != pos ):
        pub.publish(state)
        pos = state.position
    pos_array = np.array(pos)*180/1.57
    pos = list(pos_array)
    escribir_datos(pos, "goals") #Funcion para guardar los datos

#Recibimos un msg de tipo JointState a traves del topico real_joint_state y lo guardamos para posteriormente hacer un controlador
def real_position(state: JointState): 
    pos_array = np.array(state.position)*180/1.57
    real_pos = list(pos_array)
    escribir_datos(real_pos, "state") #Funcion para guardar los datos
    
def load(state: JointState): 
    pos_array = np.array(state.effort)
    real_load= list(pos_array)
    escribir_datos(real_load, "load") #Funcion para guardar los datos

def escribir_datos(posiciones, pre):
    archivo = open("/home/erick/catkin_ws/src/manipulator/matlab/datos_plano_mesa.txt", "a")
    posiciones_array = np.array(posiciones,str)
    posiciones_str = ",".join(posiciones_array)
    archivo.write(pre + ": "+ posiciones_str + "\n")

if __name__ == "__main__":
    archivo = open("/home/erick/catkin_ws/src/manipulator/matlab/datos_plano_mesa.txt", "w")
    archivo.write("Joint0,Joint1,Joint2,Joint3,Joint4,Joint5 \n")
    archivo.close()

    rospy.init_node("move_arm_node")
    pub = rospy.Publisher ("/joint_goals" , JointState, queue_size=10)
    subRobotState = rospy.Subscriber("/joint_states", JointState, callback=state_position)
    subRealState  = rospy.Subscriber("/real_joint_states", JointState, callback= real_position)
    rospy.logwarn("The move_arm_node has been started")
    rospy.spin()

archivo.close()


