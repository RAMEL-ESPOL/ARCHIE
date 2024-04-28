#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt


x = [0]
goalsJ0 = [0.0]
statesJ0 = [0.0]
goalsJ1 = [0.0]
statesJ1 = [0.0]
goalsJ2 = [0.0]
statesJ2 = [0.0]
goalsJ3 = [0.0]
statesJ3 = [0.0]
goalsJ4 = [0.0]
statesJ4 = [0.0]
goalsJ5 = [0.0]
statesJ5 = [0.0]

archivo = open("/home/erick/catkin_ws/src/manipulator/manipulator_description/datos.txt","r")
archivo.readline()
lista_lineas = archivo.readlines()
for i, linea in enumerate(lista_lineas):
    llave, valor = linea.strip("/n").split(": ")
    joints = valor.strip().split(",")
    x.append(i+1)
    if llave =="goals":
        goalsJ0.append(float(joints[0]))
        goalsJ1.append(float(joints[1]))
        goalsJ2.append(float(joints[2]))
        goalsJ3.append(float(joints[3]))
        goalsJ4.append(float(joints[4]))
        goalsJ5.append(float(joints[5]))

        statesJ0.append(float(statesJ0[-1]))
        statesJ1.append(float(statesJ1[-1]))
        statesJ2.append(float(statesJ2[-1]))
        statesJ3.append(float(statesJ3[-1]))
        statesJ4.append(float(statesJ4[-1]))
        statesJ5.append(float(statesJ5[-1]))
    if llave =="state":
        statesJ0.append(float(joints[0]))
        statesJ1.append(float(joints[1]))
        statesJ2.append(float(joints[2]))
        statesJ3.append(float(joints[3]))
        statesJ4.append(float(joints[4]))
        statesJ5.append(float(joints[5]))

        goalsJ0.append(float(goalsJ0[-1]))
        goalsJ1.append(float(goalsJ1[-1]))
        goalsJ2.append(float(goalsJ2[-1]))
        goalsJ3.append(float(goalsJ3[-1]))
        goalsJ4.append(float(goalsJ4[-1]))
        goalsJ5.append(float(goalsJ5[-1]))


archivo.close

plt.figure(1)
plt.ylim([-180,180])
plt.plot(x,goalsJ0,'o',label = "goals Joint0", color = "b")
plt.plot(x,statesJ0,'o',label = "states Joint0", color = "r")
plt.ylabel("States [degrees]")
plt.grid(True)
plt.legend(loc='upper right')
plt.savefig("/home/erick/catkin_ws/src/manipulator/manipulator_master/graphics/Joint0.png")

plt.figure(2)
plt.ylim([-180,180])
plt.plot(x,goalsJ1,'o',label = "goals Joint1", color = "b")
plt.plot(x,statesJ1,'o',label = "states Joint1", color = "r")
plt.ylabel("States [degrees]")
plt.grid(True)
plt.legend(loc='upper right')
plt.savefig("/home/erick/catkin_ws/src/manipulator/manipulator_master/graphics/Joint1.png")

plt.figure(3)
plt.ylim([-180,180])
plt.plot(x,goalsJ2,'o',label = "goals Joint2", color = "b")
plt.plot(x,statesJ2,'o',label = "states Joint2", color = "r")
plt.ylabel("States [degrees]")
plt.grid(True)
plt.legend(loc='upper right')
plt.savefig("/home/erick/catkin_ws/src/manipulator/manipulator_master/graphics/Joint2.png")

plt.figure(4)
plt.ylim([-180,180])
plt.plot(x,goalsJ3,'o',label = "goals Joint3", color = "b")
plt.plot(x,statesJ3,'o',label = "states Joint3", color = "r")
plt.ylabel("States [degrees]")
plt.grid(True)
plt.legend(loc='upper right')
plt.savefig("/home/erick/catkin_ws/src/manipulator/manipulator_master/graphics/Joint3.png")

plt.figure(5)
plt.ylim([-180,180])
plt.plot(x,goalsJ4,'o',label = "goals Joint4", color = "b")
plt.plot(x,statesJ4,'o',label = "states Joint4", color = "r")
plt.ylabel("States [degrees]")
plt.grid(True)
plt.legend(loc='upper right')
plt.savefig("/home/erick/catkin_ws/src/manipulator/manipulator_master/graphics/Joint4.png")

plt.figure(6)
plt.ylim([-180,180])
plt.plot(x,goalsJ5,'o',label = "goals Joint5", color = "b")
plt.plot(x,statesJ5,'o',label = "states Joint5", color = "r")
plt.ylabel("States [degrees]")
plt.grid(True)
plt.legend(loc='upper right')
plt.savefig("/home/erick/catkin_ws/src/manipulator/manipulator_master/graphics/Joint5.png")



plt.grid(True)
plt.show()
