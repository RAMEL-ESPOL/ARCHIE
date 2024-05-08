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
contador = 0

archivo_goals = open("/home/erick/catkin_ws/src/manipulator/matlab/data/joint_goals_square_t35_h35_p12.txt","r")
archivo_goals.readline()

archivo_states = open("/home/erick/catkin_ws/src/manipulator/matlab/data/joint_real_states_square_t35_h35_p12.txt","r")
archivo_states.readline()

for goals, states in zip(archivo_goals.readlines(),archivo_states.readlines()):
    contador += 1
    joints_goals = goals.strip().split(",")
    joints_states = states.strip().split(",")
    x.append(contador)
    goalsJ0.append(float(joints_goals[0]))
    goalsJ1.append(float(joints_goals[1]))
    goalsJ2.append(float(joints_goals[2]))
    goalsJ3.append(float(joints_goals[3]))
    goalsJ4.append(float(joints_goals[4]))
    goalsJ5.append(float(joints_goals[5]))

    statesJ0.append(float(joints_states[0]))
    statesJ1.append(float(joints_states[1]))
    statesJ2.append(float(joints_states[2]))
    statesJ3.append(float(joints_states[3]))
    statesJ4.append(float(joints_states[4]))
    statesJ5.append(float(joints_states[5]))

archivo_goals.close
archivo_states.close
print(len(x))
print(len(goalsJ0))
print(len(statesJ0))

plt.figure(1)
plt.ylim([-180,180])
plt.plot(x,goalsJ0,'o',label = "goals Joint0", color = "b")
plt.plot(x,statesJ0,'o',label = "states Joint0", color = "r")
plt.ylabel("States [degrees]")
plt.grid(True)
plt.legend(loc='upper right')

plt.figure(2)
plt.ylim([-180,180])
plt.plot(x,goalsJ1,'o',label = "goals Joint1", color = "b")
plt.plot(x,statesJ1,'o',label = "states Joint1", color = "r")
plt.ylabel("States [degrees]")
plt.grid(True)
plt.legend(loc='upper right')

plt.figure(3)
plt.ylim([-180,180])
plt.plot(x,goalsJ2,'o',label = "goals Joint2", color = "b")
plt.plot(x,statesJ2,'o',label = "states Joint2", color = "r")
plt.ylabel("States [degrees]")
plt.grid(True)
plt.legend(loc='upper right')

plt.figure(4)
plt.ylim([-180,180])
plt.plot(x,goalsJ3,'o',label = "goals Joint3", color = "b")
plt.plot(x,statesJ3,'o',label = "states Joint3", color = "r")
plt.ylabel("States [degrees]")
plt.grid(True)
plt.legend(loc='upper right')

plt.figure(5)
plt.ylim([-180,180])
plt.plot(x,goalsJ4,'o',label = "goals Joint4", color = "b")
plt.plot(x,statesJ4,'o',label = "states Joint4", color = "r")
plt.ylabel("States [degrees]")
plt.grid(True)
plt.legend(loc='upper right')

plt.figure(6)
plt.ylim([-180,180])
plt.plot(x,goalsJ5,'o',label = "goals Joint5", color = "b")
plt.plot(x,statesJ5,'o',label = "states Joint5", color = "r")
plt.ylabel("States [degrees]")
plt.grid(True)
plt.legend(loc='upper right')



plt.grid(True)
plt.show()
