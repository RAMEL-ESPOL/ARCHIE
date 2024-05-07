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


# Control table address
ADDR_PRO_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION      = 116
ADDR_PRO_PRESENT_POSITION   = 132
ADDR_PRO_PRESENT_LOAD   = 128

# Data Byte Length
LEN_PRO_GOAL_POSITION       = 4
LEN_PRO_PRESENT_POSITION    = 4
LEN_PRO_PRESENT_LOAD    = 4

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting

DXL3_ID                      = 3                 # Dynamixel ID : 3
DXL4_ID                      = 4                 # Dynamixel ID : 4
DXL5_ID                      = 5                 # Dynamixel ID : 5
BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold


COMM_SUCCESS                = 0                             # Communication Success result value
COMM_TX_FAIL                = -1001                         # Communication Tx Failed


index = 0
dxl_goal_position3 = [2048, 3072] 
dxl_goal_position4 = [2048, 3072] 
dxl_goal_position5 = [1000, 3072] 


# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)


# Initialize GroupSyncWrite instance
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION)

# Initialize GroupSyncRead instace for Present Position
groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_LOAD, LEN_PRO_PRESENT_LOAD)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    quit()


# Enable Dynamixel#1 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL4_ID)

# Enable Dynamixel#2 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL3_ID)

# Enable Dynamixel#3 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL3_ID)

# Add parameter storage for Dynamixel#1 present position value
dxl_addparam_result = groupSyncRead.addParam(DXL4_ID)
if dxl_addparam_result != True:
    print("[ID:%03d] groupSyncRead addparam failed" % DXL4_ID)
    quit()

# Add parameter storage for Dynamixel#2 present position value
dxl_addparam_result = groupSyncRead.addParam(DXL5_ID)
if dxl_addparam_result != True:
    print("[ID:%03d] groupSyncRead addparam failed" % DXL5_ID)
    quit()
# Add parameter storage for Dynamixel#3 present position value
dxl_addparam_result = groupSyncRead.addParam(DXL3_ID)
if dxl_addparam_result != True:
    print("[ID:%03d] groupSyncRead addparam failed" % DXL3_ID)
    quit()

while 1:
    # Allocate goal position value into byte array
    param_goal_position3 = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position4[index])), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position4[index])), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position4[index])), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position4[index]))]
    param_goal_position4 = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position4[index])), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position4[index])), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position4[index])), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position4[index]))]
    param_goal_position5 = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position5[index])), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position5[index])), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position5[index])), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position5[index]))]
    print(param_goal_position3,param_goal_position4,param_goal_position5)
    # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL3_ID, param_goal_position4)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL3_ID)
        quit()

    dxl_addparam_result = groupSyncWrite.addParam(DXL4_ID, param_goal_position4)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL4_ID)
        quit()

    # Add Dynamixel#2 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL5_ID, param_goal_position5)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL5_ID)
        quit()

    # Syncwrite goal position
    print("lista de paquete: ", groupSyncWrite.param)
    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Clear syncwrite parameter storage
    groupSyncWrite.clearParam()

    while 1:
        # Syncread present position
        dxl_comm_result = groupSyncRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Check if groupsyncread data of Dynamixel#1 is available
        dxl_getdata_result = groupSyncRead.isAvailable(DXL4_ID, ADDR_PRO_PRESENT_LOAD, LEN_PRO_PRESENT_LOAD)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncRead getdata failed" % DXL4_ID)
            quit()

        # Check if groupsyncread data of Dynamixel#2 is available
        dxl_getdata_result = groupSyncRead.isAvailable(DXL5_ID, ADDR_PRO_PRESENT_LOAD, LEN_PRO_PRESENT_LOAD)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncRead getdata failed" % DXL5_ID)
            quit()

        # Check if groupsyncread data of Dynamixel#2 is available
        dxl_getdata_result = groupSyncRead.isAvailable(DXL3_ID, ADDR_PRO_PRESENT_LOAD, LEN_PRO_PRESENT_LOAD)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncRead getdata failed" % DXL3_ID)
            quit()

        # Get Dynamixel#1 present position value
        dxl1_present_position = groupSyncRead.getData(DXL4_ID, ADDR_PRO_PRESENT_LOAD, LEN_PRO_PRESENT_LOAD)

        # Get Dynamixel#2 present position value
        dxl2_present_position = groupSyncRead.getData(DXL5_ID, ADDR_PRO_PRESENT_LOAD, LEN_PRO_PRESENT_LOAD)

        # Get Dynamixel#3 present position value
        dxl2_present_position = groupSyncRead.getData(DXL3_ID, ADDR_PRO_PRESENT_LOAD, LEN_PRO_PRESENT_LOAD)

        print("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL4_ID, dxl_goal_position4[index], dxl1_present_position, DXL5_ID, dxl_goal_position5[index], dxl2_present_position))

        if not ((abs(dxl_goal_position4[index] - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD) and (abs(dxl_goal_position5[index] - dxl2_present_position) > DXL_MOVING_STATUS_THRESHOLD)):
            break

    # Change goal position
    if index == 0:
        index = 1
    else:
        index = 0

# Clear syncread parameter storage
groupSyncRead.clearParam()

# Disable Dynamixel#1 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Disable Dynamixel#2 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Disable Dynamixel#3 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))


# Close port
portHandler.closePort()