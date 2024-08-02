#!/usr/bin/env python3

import sys
import rospy
import numpy as np
from   sensor_msgs.msg import JointState
from std_msgs.msg import String
import os
import math
from archie_master.msg import MotorData

global ruta_file
ruta_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'matlab', 'data_pwm')

global new_file
new_file = True

global number
number = 0

def write_data(name: str, data: str):
    
    global new_file
    global number
    while True:
        if os.path.exists(os.path.join(ruta_file, str(number) + "_motor_" + name + ".txt")):
            if new_file:                
                
                number += 1
            
            else:

                archivo = open(os.path.join(ruta_file, str(number) + "_motor_" + name + ".txt"), "a")
                archivo.write(data)
                archivo.close()
                break

        else:
            archivo = open(os.path.join(ruta_file, str(number) + "_motor_" + name + ".txt"), "w")
            archivo.write(data)
            archivo.close()
            new_file = False
            break


def save_data(motor: MotorData):

    write_data("position", ",".join(np.array(motor.position,str)) + "\n")
    write_data("error"   , ",".join(np.array(motor.error,str))    + "\n")
    write_data("velocity", ",".join(np.array(motor.velocity,str)) + "\n")
    write_data("effort"  , ",".join(np.array(motor.effort,str))   + "\n") 
    

if __name__ == "__main__":
    rospy.sleep(1)

    rospy.init_node("write_motor_data_node")
    subRealState    = rospy.Subscriber("/motor_data", MotorData, callback = save_data)

    rospy.logwarn("The write_motor_data_node has been started")
    rospy.spin()