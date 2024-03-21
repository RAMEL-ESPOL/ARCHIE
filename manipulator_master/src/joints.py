#! /usr/bin/env python3

import moveit_commander
import math
#Executing this file will print the current position in degrees of each joint by console
group = moveit_commander.MoveGroupCommander("arm_group")

print("Posición de los joints:")

for i in range(6):
    print((group.get_joints()[i]) + ": " + str(math.degrees(group.get_current_joint_values()[i])))

moveit_commander.roscpp_shutdown()