#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import roslib
import os

# Uses Dynamixel SDK library
from motor_classes import *
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from dynamixel_sdk import * 
from getch import getch
import numpy as np
import moveit_commander


def convert_to_signed_16bit(val):
    if val > 0x7FFF:
        val -= 0x10000
    return val

def convert_to_signed_32bit(val):
    if val > 0x7FFFFFFF:
        val -= 0x100000000
    return val

def convert_hex(pwm):
    pwm = pwm*200/1.2
    pwm = 300 if abs(pwm) > 885 else pwm
    if pwm < 0:
        pwm = (1 << 16) + pwm
    return pwm

def motor_angle_to_radian( motor_angle_val):

    min_value_angle = 0
    max_value_angle = 4095 
    min_angle_deg = 0
    max_angle_deg = 360

    zero_value_angle = (max_value_angle-min_value_angle)//2 + 1
    deg_movement_span_2 = (max_angle_deg-min_angle_deg)//2
    rad_movement_span_2 = float(deg_movement_span_2*(pi/180))        

    if (motor_angle_val == zero_value_angle):
        return 0.0
    else:
        return ((float)(motor_angle_val-zero_value_angle)/(float)(zero_value_angle))*rad_movement_span_2

def get_positions():# Read present position
    present_positions = [0,0,0,0,0,0]   
    # Syncread present position
    dxl_comm_result = groupSyncReadPos.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("Falla en get_positions: %s" % packetHandler.getTxRxResult(dxl_comm_result))
    
    for id in range(6):
        # Check if groupsyncread data of Dynamixel#1 is available
        dxl_getdata_result = groupSyncReadPos.isAvailable(id, ADDR_PRO_PRESENT_POS , 4)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncReadPos getdata failed" % id)
        # Get Dynamixel present position value
        dxl_present_position = motor_angle_to_radian(groupSyncReadPos.getData(id, ADDR_PRO_PRESENT_POS , 4))
        present_positions[id]=dxl_present_position

    return present_positions


def calculate_torque(current_positions: JointState):
    jac_t = np.transpose(group.get_jacobian_matrix(current_positions))
    gravity = -9.81
    mass_vector = np.transpose(np.array([0.102, 0.101, 0.102, 0.105, 0.102, 0.05],float))
    gravity_force = mass_vector * gravity
    gravity_compensation = np.dot(jac_t, gravity_force)

    print(gravity_compensation)
    return gravity_compensation

def move_to_target(goal_position: JointState):

    motor_positions = get_positions()
    current_positions = list(np.array(motor_positions, float))

    # Calcula los torques de gravedad para la posición actual
    gravity_torques = calculate_torque(current_positions)

    # Calcula los torques adicionales necesarios para moverse hacia la posición objetivo
    position_error = np.array(goal_position.position) - np.array(current_positions)
    k_p = 10  # Ganancia proporcional (ajusta según sea necesario)
    position_torques = k_p * position_error

    effort = (gravity_torques + position_torques)

    joint_state_publisher(motor_positions, effort)
    set_sync_pwm(effort)

        

def set_sync_pwm(total_torques):
    print("""=====================================================================================Lista de motores""")

    for id in range(6):
        new_pwm = convert_hex(total_torques[id])
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(new_pwm)), DXL_HIBYTE(DXL_LOWORD(new_pwm)), DXL_LOBYTE(DXL_HIWORD(new_pwm)), DXL_HIBYTE(DXL_HIWORD(new_pwm))]
        # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWritePWM.addParam(id, param_goal_position)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWritePWM addparam failed" % id)

    dxl_comm_result = groupSyncWritePWM.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("Hubo un fallo en envío de PWM")
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Clear syncwrite parameter storage
    groupSyncWritePWM.clearParam()

    print("=====================================================================================")


def joint_state_publisher(current_positions, efforts):
    joints_states = JointState()
    joints_states.header = Header()
    joints_states.header.stamp = rospy.Time.now()
    joints_states.name = ['joint_'+str(id+1) for id in range(6)]
    
    #Publish the new joint state
    joints_states.position = current_positions
    joints_states.effort = efforts
    joint_state_pub.publish(joints_states)


if __name__ == '__main__':

    ADDR_OPERATING_MODE         = 11               # Control table address is different in Dynamixel model
    ADDR_PRO_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
    ADDR_PRO_GOAL_PWM           = 100
    ADDR_PRO_PRESENT_PWM        = 124
    ADDR_PRO_PRESENT_VEL        = 128
    ADDR_PRO_PRESENT_POS        = 132


    # Protocol version
    PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

    # Default setting
    DXL_ID                      = 5                 # Dynamixel ID : 5
    BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
    DEVICENAME                  = '/dev/ttyUSB0'           #'/dev/ttyUSB0'    # Check which port is being used on your controller
                                                    # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

    TORQUE_ENABLE               = 1                 # Value for enabling the torque
    TORQUE_DISABLE              = 0                 # Value for disabling the torque
    PWM_CONTROL_MODE            = 16                         # Value for extended position control mode (operating mode)
    POS_CONTROL_MODE            = 3

    index = 0

    portHandler = PortHandler(DEVICENAME)

    packetHandler = PacketHandler(PROTOCOL_VERSION)

    # Open port
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()


    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()


    rospy.init_node("motor_communication_pwm")
    r =rospy.Rate(10) # 10hz

    # Set operating mode to pwm control mode
    for id in range(6):     
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_OPERATING_MODE, PWM_CONTROL_MODE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Operating mode for motor", str(id), " changed to PWM control mode.")

    for id in range(6):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Torque of Motor ",id," is on")
    # Initialize GroupSyncWrite/Read instance
    groupSyncWritePWM       = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_PWM, 4)
    groupSyncReadPos        = GroupSyncRead (portHandler, packetHandler, ADDR_PRO_PRESENT_POS, 4)
    

    for id in range(6):
        # Add parameter storage for Dynamixel present position value
        dxl_addparam_resultPos = groupSyncReadPos.addParam(id)
        if dxl_addparam_resultPos != True:
            print("[ID:%03d] groupSyncReadPos addparam failed" % id)

    #Publish current robot state
    joint_state_pub = rospy.Publisher('/real_joint_states', JointState, queue_size=10)
    subGoalState    = rospy.Subscriber('/joint_goals', JointState, callback = move_to_target, queue_size= 5)
    group           = moveit_commander.MoveGroupCommander("arm_group")
    
    rospy.spin()    

    # Disable the torque
    for id in range(6):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Torque of Motor ",id," is off")

    #Setting to default control mode (Position Control Mode) 
    for id in range(6):     
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_OPERATING_MODE, POS_CONTROL_MODE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Operating mode for motor", str(id), " changed to Position Control Mode.")

    # Clear syncread parameter storage
    groupSyncReadPos.clearParam()

    portHandler.closePort()
