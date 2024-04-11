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

import ctypes

# Control table address
ADDR_PRO_TORQUE_ENABLE      = 64                           # Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION      = 116
ADDR_PRO_PRESENT_POSITION   = 132

# Data Byte Length
LEN_PRO_GOAL_POSITION       = 4
LEN_PRO_PRESENT_POSITION    = 4

# Protocol version
PROTOCOL_VERSION            = 2                             # See which protocol version is used in the Dynamixel

# Default setting
DXL1_ID                     = 4                             # Dynamixel ID: 4
DXL2_ID                     = 5                             # Dynamixel ID: 5
BAUDRATE                    = 1000000
DEVICENAME                  = "/dev/ttyUSB0"        # Check which port is being used on your controller
                                                            # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

TORQUE_ENABLE               = 1                             # Value for enabling the torque
TORQUE_DISABLE              = 0                             # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 2048                      # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 3072                        # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                            # Dynamixel moving status threshold


COMM_SUCCESS                = 0                             # Communication Success result value
COMM_TX_FAIL                = -1001                         # Communication Tx Failed

# Initialize PortHandler Structs
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows

portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler Structs
packetHandler = PacketHandler(PROTOCOL_VERSION)


# Initialize Groupsyncwrite instance
groupwrite_num = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION)

# Initialize Groupsyncread Structs for Present Position
groupread_num =  GroupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

index = 0
dxl_comm_result = COMM_TX_FAIL                              # Communication result
dxl_addparam_result = 0                                     # AddParam result
dxl_getdata_result = 0                                      # GetParam result
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position

dxl_error = 0                                               # Dynamixel error
dxl1_present_position = 0                                   # Present position
dxl2_present_position = 0

# Open port
if portHandler.openPort(portHandler):
    print("Succeeded to open the port!")
else:
    print("Failed to open the port!")
    print("Press any key to terminate...")
    quit()


# Set port baudrate
if portHandler.setBaudRate(portHandler, BAUDRATE):
    print("Succeeded to change the baudrate!")
else:
    print("Failed to change the baudrate!")
    print("Press any key to terminate...")
    quit()


# Enable packet_handler#1 Torque
packet_handler.write1ByteTxRx(portHandler, PROTOCOL_VERSION, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
if packet_handler.getLastTxRxResult(portHandler, PROTOCOL_VERSION) != COMM_SUCCESS:
    packet_handler.printTxRxResult(PROTOCOL_VERSION, packet_handler.getLastTxRxResult(portHandler, PROTOCOL_VERSION))
elif packet_handler.getLastRxPacketError(portHandler, PROTOCOL_VERSION) != 0:
    packet_handler.printRxPacketError(PROTOCOL_VERSION, packet_handler.getLastRxPacketError(portHandler, PROTOCOL_VERSION))
else:
    print("Dynamixel#1 has been successfully connected")

# Enable Dynamixel#2 Torque
packet_handler.write1ByteTxRx(portHandler, PROTOCOL_VERSION, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
if packet_handler.getLastTxRxResult(portHandler, PROTOCOL_VERSION) != COMM_SUCCESS:
    packet_handler.printTxRxResult(PROTOCOL_VERSION, packet_handler.getLastTxRxResult(portHandler, PROTOCOL_VERSION))
elif packet_handler.getLastRxPacketError(portHandler, PROTOCOL_VERSION) != 0:
    packet_handler.printRxPacketError(PROTOCOL_VERSION, packet_handler.getLastRxPacketError(portHandler, PROTOCOL_VERSION))
else:
    print("Dynamixel#2 has been successfully connected")

# Add parameter storage for Dynamixel#1 present position value
dxl_addparam_result = ctypes.c_ubyte(packet_handler.groupSyncReadAddParam(groupread_num, DXL1_ID)).value
if dxl_addparam_result != 1:
    print("[ID:%03d] groupSyncRead addparam failed" % (DXL1_ID))
    quit()

# Add parameter storage for Dynamixel#2 present position value
dxl_addparam_result = ctypes.c_ubyte(packet_handler.groupSyncReadAddParam(groupread_num, DXL2_ID)).value
if dxl_addparam_result != 1:
    print("[ID:%03d] groupSyncRead addparam failed" % (DXL2_ID))
    quit()


while 1:
    # Add Dynamixel#1 goal position value to the Syncwrite storage
    dxl_addparam_result = ctypes.c_ubyte(packet_handler.groupSyncWriteAddParam(groupwrite_num, DXL1_ID, dxl_goal_position[index], LEN_PRO_GOAL_POSITION)).value
    print(dxl_addparam_result)
    if dxl_addparam_result != 1:
        print("[ID:%03d] groupSyncWrite addparam failed" % (DXL1_ID))
        quit()

    # Add Dynamixel#2 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = ctypes.c_ubyte(packet_handler.groupSyncWriteAddParam(groupwrite_num, DXL2_ID, dxl_goal_position[index], LEN_PRO_GOAL_POSITION)).value
    if dxl_addparam_result != 1:
        print("[ID:%03d] groupSyncWrite addparam failed" % (DXL2_ID))
        quit()

    # Syncwrite goal position
    packet_handler.groupSyncWriteTxPacket(groupwrite_num)
    if packet_handler.getLastTxRxResult(portHandler, PROTOCOL_VERSION) != COMM_SUCCESS:
        packet_handler.printTxRxResult(PROTOCOL_VERSION, packet_handler.getLastTxRxResult(portHandler, PROTOCOL_VERSION))

    # Clear syncwrite parameter storage
    packet_handler.groupSyncWriteClearParam(groupwrite_num)

    while 1:
        # Syncread present position
        packet_handler.groupSyncReadTxRxPacket(groupread_num)
        if packet_handler.getLastTxRxResult(portHandler, PROTOCOL_VERSION) != COMM_SUCCESS:
            packet_handler.printTxRxResult(PROTOCOL_VERSION, packet_handler.getLastTxRxResult(portHandler, PROTOCOL_VERSION))

        # Check if groupsyncread data of Dynamixel#1 is available
        dxl_getdata_result = ctypes.c_ubyte(packet_handler.groupSyncReadIsAvailable(groupread_num, DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)).value
        if dxl_getdata_result != 1:
            print("[ID:%03d] groupSyncRead getdata failed" % (DXL1_ID))
            quit()

        # Check if groupsyncread data of Dynamixel#2 is available
        dxl_getdata_result = ctypes.c_ubyte(packet_handler.groupSyncReadIsAvailable(groupread_num, DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)).value
        if dxl_getdata_result != 1:
            print("[ID:%03d] groupSyncRead getdata failed" % (DXL2_ID))
            quit()

        # Get Dynamixel#1 present position value
        dxl1_present_position = packet_handler.groupSyncReadGetData(groupread_num, DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

        # Get Dynamixel#2 present position value
        dxl2_present_position = packet_handler.groupSyncReadGetData(groupread_num, DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

        print("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\n" % (DXL1_ID, dxl_goal_position[index], dxl1_present_position, DXL2_ID, dxl_goal_position[index], dxl2_present_position))

        if not ((abs(dxl_goal_position[index] - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD) or (abs(dxl_goal_position[index] - dxl2_present_position) > DXL_MOVING_STATUS_THRESHOLD)):
            break

    # Change goal position
    if index == 0:
        index = 1
    else:
        index = 0


# Disable Dynamixel#1 Torque
packet_handler.write1ByteTxRx(portHandler, PROTOCOL_VERSION, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
if packet_handler.getLastTxRxResult(portHandler, PROTOCOL_VERSION) != COMM_SUCCESS:
    packet_handler.printTxRxResult(PROTOCOL_VERSION, packet_handler.getLastTxRxResult(portHandler, PROTOCOL_VERSION))
elif packet_handler.getLastRxPacketError(portHandler, PROTOCOL_VERSION) != 0:
    packet_handler.printRxPacketError(PROTOCOL_VERSION, packet_handler.getLastRxPacketError(portHandler, PROTOCOL_VERSION))

# Disable packet_handler#2 Torque
packet_handler.write1ByteTxRx(portHandler, PROTOCOL_VERSION, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
if packet_handler.getLastTxRxResult(portHandler, PROTOCOL_VERSION) != COMM_SUCCESS:
    packet_handler.printTxRxResult(PROTOCOL_VERSION, packet_handler.getLastTxRxResult(portHandler, PROTOCOL_VERSION))
elif packet_handler.getLastRxPacketError(portHandler, PROTOCOL_VERSION) != 0:
    packet_handler.printRxPacketError(PROTOCOL_VERSION, packet_handler.getLastRxPacketError(portHandler, PROTOCOL_VERSION))

# Close port
portHandler.closePort(portHandler)