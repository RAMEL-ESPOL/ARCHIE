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

value_angles_max = []
value_angles_min = []


#Funcion usada antes, sirve para setear el valor en cada motor a la vez, no muy eficaz
"""
def set_positions(data,callback_args):
    list_motors = callback_args[0]
    bool_init = callback_args[1]
    print("=====================================================================================Lista de motores")
    for motor in list_motors:
        for id in motor.list_ids:
            print("ID Motor: " + str(id))
            #Check if zero value positions are the initial positions for each joint!!!!!!!!!!!!!!!!!!!!!!!!
            if bool_init:
                dxl_comm_result, dxl_error = motor.packetHandler.write4ByteTxRx(motor.portHandler, id, motor.addr_goal_position, motor.angle_zero)
                print("Dynamixel has successfully set the initial position")
            else:
                new_angle = motor.angleConversion(data.position[id], False,id) 
                print("The new angle is " + str(new_angle*180/2048))
                dxl_comm_result, dxl_error = motor.packetHandler.write4ByteTxRx(motor.portHandler, id, motor.addr_goal_position, new_angle)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % motor.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % motor.packetHandler.getRxPacketError(dxl_error))
    print("=====================================================================================")
"""


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



####
def get_positions(list_motors):# Read present position
    present_positions = [0,0,0,0,0,0]   
    # Syncread present position
    dxl_comm_result = groupSyncRead.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    
    for motor in list_motors:
        for id in motor.list_ids:
            # Check if groupsyncread data of Dynamixel#1 is available
            dxl_getdata_result = groupSyncRead.isAvailable(id, motor.addr_present_position , 4)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupSyncRead getdata failed" % id)
            # Get Dynamixel present position value
            dxl_present_position = groupSyncRead.getData(id, motor.addr_present_position , 4)
            present_positions[id]=dxl_present_position
    return present_positions


def get_load(list_motors):# Read present position
    present_load = [0,0,0,0,0,0]  
    # Syncread present position
    dxl_comm_result = groupSyncRead.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    
    for motor in list_motors:
        for id in motor.list_ids:
            # Check if groupsyncread data of Dynamixel#1 is available
            dxl_getdata_result = groupSyncRead.isAvailable(id, 126 , 4)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupSyncRead getdata failed" % id)
            # Get Dynamixel present position value
            present_load[id]= groupSyncRead.getData(id, motor.addr_present_load , 4)

    rospy.logerr(present_load)
    return present_load


def get_velocities(list_motors):# Read present position
    present_vel = [0,0,0,0,0,0]  
    # Syncread present position
    dxl_comm_result = groupSyncRead.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    
    for motor in list_motors:
        for id in motor.list_ids:
            # Check if groupsyncread data of Dynamixel#1 is available
            dxl_getdata_result = groupSyncRead.isAvailable(id, 128 , 4)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupSyncRead getdata failed" % id)
            # Get Dynamixel present position value
            present_vel[id]= groupSyncRead.getData(id, 128 , 4)

    rospy.logerr(present_vel)
    return present_vel

def get_motor_data(list_motors):
    present_positions = [0,0,0,0,0,0]
    present_load      = [0,0,0,0,0,0]
    present_vel       = [0,0,0,0,0,0]
    # Syncread present position
    dxl_comm_result = groupSyncRead.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    for motor in list_motors:
        for id in motor.list_ids:
            # Check if groupsyncread data of Dynamixel#1 is available
            dxl_getdata_result = groupSyncRead.isAvailable(id, motor.addr_present_position , 4)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupSyncRead getdata failed" % id)
            # Get Dynamixel present position value
            present_positions[id] = groupSyncRead.getData(id, motor.addr_present_position , 4)
            present_vel      [id] = groupSyncRead.getData(id, motor.addr_present_velocity , 4)
            present_load     [id] = groupSyncRead.getData(id, motor.addr_present_load     , 4)
    
    return present_positions, present_vel, present_load


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
                #print("El joint del ID ", id, " es: ", general_joint_position_state[id])
    #Publish the new joint state
    joints_states.position = general_joint_position_state
    joint_state_pub.publish(joints_states)


if __name__ == '__main__':

    rospy.init_node("motor_communication")
    r =rospy.Rate(10) # 10hz

    usb_port = rospy.get_param('~usb_port')
    dxl_baud_rate = rospy.get_param('~dxl_baud_rate')


    num_joints = 6
    general_joint_position = [0 for i in range(num_joints)]
    general_joint_position_state = [0 for i in range(num_joints)]

    portHandler = PortHandler(usb_port)
    packetHandler = PacketHandler(2.0)


    #Last value is the max desired speed: value*0.229rpm is the speed in rpm
    print(dxl_baud_rate)
    """#Mesa a 100 mm
    base = XCseries_motor(usb_port,dxl_baud_rate,[0,1],portHandler,packetHandler,r,15,{0:[-1.57,1.57],1:[-0.785,0.785]},{0:[1500,450,100],1:[4500,1800,800]})
    codo = XCseries_motor(usb_port,dxl_baud_rate,[2,3],portHandler,packetHandler,r,15,{2:[-1.15,2],3:[-3.14,3.14]},{2:[3500,1500,400],3:[300,0,30]})
    ee   = XCseries_motor(usb_port,dxl_baud_rate,[4,5],portHandler,packetHandler,r,15,{4:[-1.15,2],5:[-3.14,3.14]},{4:[1500,500,150],5:[200,0,20]})
    """
    #Plano inclinado
    base = XCseries_motor(usb_port,dxl_baud_rate,[0,1],portHandler,packetHandler,r,15,{0:[-1.57,1.57],1:[-0.785,0.785]},{0:[2000,800,0],1:[3000,1200,0]})
    codo = XCseries_motor(usb_port,dxl_baud_rate,[2,3],portHandler,packetHandler,r,15,{2:[-1.15,2],3:[-3.14,3.14]},{2:[3000,1200,0],3:[1000,0,0]})
    ee   = XCseries_motor(usb_port,dxl_baud_rate,[4,5],portHandler,packetHandler,r,15,{4:[-1.15,2],5:[-3.14,3.14]},{4:[800,200,0],5:[200,0,20]})

    list_motors = [base,codo,ee]

    # Initialize GroupSyncWrite instance
    groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ee.addr_goal_position, 4)
    # Initialize GroupSyncRead instace for Present Position

    groupSyncRead = GroupSyncRead(portHandler, packetHandler, ee.addr_present_position, 4)

    for m in list_motors:
        for id in m.list_ids:
            # Add parameter storage for Dynamixel present position value
            dxl_addparam_result = groupSyncRead.addParam(id)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncRead addparam failed" % id)
    

    #Publish current robot state
    joint_state_pub = rospy.Publisher('/real_joint_states', JointState, queue_size=10)

    set_sync_positions({},[list_motors, True])

    # Subscribe desired joint position
    rospy.Subscriber('/joint_goals', JointState,set_sync_positions,(list_motors,False),queue_size= 5)

    print("subcribir")

    while not rospy.is_shutdown():
        #se lama a la funcion joint state publisher para publicar los /joint_states a ROS
        joint_state_publisher(list_motors, num_joints)
        r.sleep()
        
    # Clear syncread parameter storage
    groupSyncRead.clearParam()

    portHandler.closePort()
