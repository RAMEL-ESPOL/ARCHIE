#!/usr/bin/env python3

import rospy
import numpy as np
import moveit_commander


group = moveit_commander.MoveGroupCommander("arm_group")

archivo = open("/home/erick/catkin_ws/src/manipulator/manipulator_description/datos.txt","r")
archivo.readline()
for i, linea in enumerate(archivo.readlines()):
    llave, valor = linea.strip("/n").split(": ")
    joints = valor.strip().split(",")
    if llave =="state":
        jacobiano = group.get_jacobian_matrix(list(np.array(joints,float)*3.1416/180))
        eigenvalues, eigenvectors = np.linalg.eig(jacobiano)
        print(jacobiano)
        print("determinante: ", np.linalg.det(jacobiano), list(np.array(joints,float)*3.1416/180))
        #print("Los eigenvalues son: ",eigenvalues)
        #print("Los eigenvectors son: ",eigenvectors)


