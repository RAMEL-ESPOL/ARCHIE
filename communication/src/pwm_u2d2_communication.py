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

ADDR_OPERATING_MODE         = 11               # Control table address is different in Dynamixel model
ADDR_PRO_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_PRO_GOAL_PWM           = 100
ADDR_PRO_PRESENT_PWM        = 124
ADDR_PRO_PRESENT_VEL        = 128
ADDR_PRO_PRESENT_POS        = 132
TORQUE_ENABLE               = 1                # Value for enabling the torque
TORQUE_DISABLE              = 0                # Value for disabling the torque
PWM_CONTROL_MODE            = 16               # Value for extended position control mode (operating mode)


def convert_to_signed_16bit(val):
    if val > 0x7FFF:
        val -= 0x10000
    return val

def convert_to_signed_32bit(val):
    if val > 0x7FFFFFFF:
        val -= 0x100000000
    return val

def convert_hex(pwm):
    if pwm < 0:
        pwm = (1 << 16) + pwm
    return pwm


def set_sync_positions(data : JointState,callback_args):
    list_motors = callback_args[0]
    bool_init   = callback_args[1]
    print("""=====================================================================================Lista de motores""")
    for motor in list_motors:
        for id in motor.list_ids:
            print("ID Motor: " + str(id))
            #Check if zero value positions are the initial positions for each joint!!!!!!!!!!!!!!!!!!!!!!!!
            if bool_init:
                param_goal_position = [DXL_LOBYTE(DXL_LOWORD(2048)), DXL_HIBYTE(DXL_LOWORD(2048)), DXL_LOBYTE(DXL_HIWORD(2048)), DXL_HIBYTE(DXL_HIWORD(2048))]
                # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
                dxl_addparam_result = groupSyncWrite.addParam(id, param_goal_position)
                if dxl_addparam_result != True:
                    print("[ID:%03d] groupSyncWrite addparam failed" % id)
                print("Dynamixel has successfully set the initial position")

            else:
                new_angle = motor.angleConversion(data.position[id], False,id) 
                param_goal_position = [DXL_LOBYTE(DXL_LOWORD(new_angle)), DXL_HIBYTE(DXL_LOWORD(new_angle)), DXL_LOBYTE(DXL_HIWORD(new_angle)), DXL_HIBYTE(DXL_HIWORD(new_angle))]
                # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
                dxl_addparam_result = groupSyncWrite.addParam(id, param_goal_position)
                if dxl_addparam_result != True:
                    print("[ID:%03d] groupSyncWrite addparam failed" % id)
                print("The new angle is " + str(new_angle*180/2048))

    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("Hubo un fallo")
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Clear syncwrite parameter storage
    groupSyncWrite.clearParam()

    print("=====================================================================================")


def get_positions(list_motors):# Read present position
    present_positions = [0,0,0,0,0,0]   
    # Syncread present position
    dxl_comm_result = groupSyncReadPos.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    
    for motor in list_motors:
        for id in motor.list_ids:
            # Check if groupsyncread data of Dynamixel#1 is available
            dxl_getdata_result = groupSyncReadPos.isAvailable(id, motor.addr_present_position , 4)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupSyncRead getdata failed" % id)
            # Get Dynamixel present position value
            dxl_present_position = groupSyncReadPos.getData(id, motor.addr_present_position , 4)
            present_positions[id]=dxl_present_position
    return present_positions


def joint_state_publisher(list_motors,num_joints):
    joints_states = JointState()
    joints_states.header = Header()
    joints_states.header.stamp = rospy.Time.now()
    joints_states.name = ['joint_'+str(id+1) for id in range(num_joints)]
    #Read actual motor state after movement occured
    general_joint_position = get_positions(list_motors)
    #Convert from 0-4095 to degrees
    #print("Joint State")
    if general_joint_position != general_joint_position_state:
        for motor in list_motors:
            for id in motor.list_ids:
                general_joint_position_state[id]=motor.angleConversion(general_joint_position[id],True,id) 
    #Publish the new joint state
    joints_states.position = general_joint_position_state
    joint_state_pub.publish(joints_states)


if __name__ == '__main__':

    rospy.init_node("motor_communication_pwm")
    r =rospy.Rate(10) # 10hz

    usb_port = rospy.get_param('~usb_port')
    dxl_baud_rate = rospy.get_param('~dxl_baud_rate')


    num_joints = 6
    general_joint_position = [0 for i in range(num_joints)]
    general_joint_position_state = [0 for i in range(num_joints)]

    portHandler = PortHandler(usb_port)
    packetHandler = PacketHandler(2.0)

    # Set operating mode to extended position control mode
    for id in range(6):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_OPERATING_MODE, PWM_CONTROL_MODE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Operating mode changed to extended position control mode.")

    # Enable Dynamixel Torque
    for id in range(6):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)

        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel Motor ",id, " has been successfully connected")

    # Initialize GroupSyncWrite/Read instance
    groupSyncWritePWM       = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_PWM, 4)
    groupSyncReadPos        = GroupSyncRead (portHandler, packetHandler, ADDR_PRO_PRESENT_POS, 4)
    

    for id in range(6):
        # Add parameter storage for Dynamixel present position value
        dxl_addparam_resultPWM = groupSyncWritePWM.addParam(id)
        dxl_addparam_resultPos = groupSyncReadPos.addParam(id)
        if dxl_addparam_resultPWM != True:
            print("[ID:%03d] groupSyncWritePWM addparam failed" % id)
        if dxl_addparam_resultPos != True:
            print("[ID:%03d] groupSyncReadPos addparam failed" % id)

    #Publish current robot state
    joint_state_pub = rospy.Publisher('/real_joint_states', JointState, queue_size=10)

    set_sync_positions({},[list_motors, True])

    # Subscribe desired joint position
    rospy.Subscriber('/joint_goals', JointState,set_sync_positions,(list_motors,False),queue_size= 5)

    while not rospy.is_shutdown():
        #se lama a la funcion joint state publisher para publicar los /joint_states a ROS
        joint_state_publisher(list_motors, num_joints)
        r.sleep()
        
    # Clear syncread parameter storage
    groupSyncRead.clearParam()

    portHandler.closePort()
