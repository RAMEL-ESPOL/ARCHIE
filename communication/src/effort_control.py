#!/usr/bin/env python3
from math import pi
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from dynamixel_sdk import *
from std_msgs.msg import Float64


# Setup
portHandler = PortHandler('/dev/ttyUSB0')
packetHandler = PacketHandler(2.0)
DXL_ID = 1

def joint_state_callback(msg):
    desired_torque = calculate_torque(msg)
    pwm = torque_to_pwm(desired_torque)
    #send_pwm_to_dynamixel(pwm)

def calculate_torque(joint_state):
    # Implementa tu modelo matemático aquí
    torque = 0  # Este es solo un placeholder
    return torque

def torque_to_pwm(torque):
    pwm = int(torque * 10)  # Conversión simplificada, ajustar según modelo
    return pwm

def send_pwm_to_dynamixel(pwm):
    packetHandler.write4ByteTxRx(portHandler, DXL_ID, 100, pwm)

if __name__ == "__main__":
    rospy.init_node('torque_controller')
    rospy.Subscriber('/joint_states', JointState, joint_state_callback)
    rospy.spin()