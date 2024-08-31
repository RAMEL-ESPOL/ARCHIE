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
new_file  = False

#Recibimos del Subscriber un msg de tipo JointState de moveit y posteriormente lo publicamos con el Publisher como goal
def testing_publish(data: JointState):

    motor = MotorData()
    motor.position = data.position
    motor.error    = data.position
    motor.velocity = data.velocity
    motor.effort   = data.effort

    pub.publish(motor)

if __name__ == "__main__":

    rospy.init_node("test_msg_node")
    pub             = rospy.Publisher ("/motor_data" , MotorData, queue_size=10)
    subGoalState    = rospy.Subscriber("/joint_states", JointState, callback = testing_publish)

    rospy.logwarn("The test_msg_node has been started")
    rospy.spin()