#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt

x = [0]

goalsJ0 = [0]
statesJ0 = [0]
goalsJ1 = [0]
statesJ1 = [0]
goalsJ2 = [0]
statesJ2 = [0]
goalsJ3 = [0]
statesJ3 = [0]
goalsJ4 = [0]
statesJ4 = [0]
goalsJ5 = [0]
statesJ5 = [0]

archivo = open("/home/erick/catkin_ws/src/manipulator/manipulator_master/src/datos.txt","r")
archivo.readline()
lista_lineas = archivo.readlines()
for i, linea in enumerate(lista_lineas):
    llave, valor = linea.strip("/n").split(": ")
    joints = valor.strip().split(",")
    x.append(i+1)
    if llave =="goals":
        goalsJ0.append(joints[0])
        goalsJ1.append(joints[1])
        goalsJ2.append(joints[2])
        goalsJ3.append(joints[3])
        goalsJ4.append(joints[4])
        goalsJ5.append(joints[5])

        statesJ0.append(statesJ0[-1])
        statesJ1.append(statesJ1[-1])
        statesJ2.append(statesJ2[-1])
        statesJ3.append(statesJ3[-1])
        statesJ4.append(statesJ4[-1])
        statesJ5.append(statesJ5[-1])
    if llave =="state":
        statesJ0.append(joints[0])
        statesJ1.append(joints[1])
        statesJ2.append(joints[2])
        statesJ3.append(joints[3])
        statesJ4.append(joints[4])
        statesJ5.append(joints[5])

        goalsJ0.append(goalsJ0[-1])
        goalsJ1.append(goalsJ1[-1])
        goalsJ2.append(goalsJ2[-1])
        goalsJ3.append(goalsJ3[-1])
        goalsJ4.append(goalsJ4[-1])
        goalsJ5.append(goalsJ5[-1])
print(len(x))
print(len(goalsJ0))
print(len(statesJ0))
    

    

archivo.close


plt.plot(x,goalsJ2)
plt.plot(x,statesJ2)
plt.xlabel('x')
plt.ylabel('goals,states')
plt.title('Lab DLS')
plt.show()