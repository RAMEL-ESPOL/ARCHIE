#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import roslib
import os
import PyKDL as kdl
import os
from kdl_parser_py.urdf import treeFromFile

# Uses Dynamixel SDK library
from motor_classes import *
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from dynamixel_sdk import * 
from getch import getch
import numpy as np
from archie_master.msg import MotorData
import math

def convert_to_signed_16bit(val):
    if val > 0x7FFF:
        val -= 0x10000
    return val

def convert_to_rpm(val):
    if val > 0x7FFFFFFF:
        val -= 0x100000000
    return val*0.229

def convert_hex(pwm):
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
    
    for id in range(NUM_MOTORS):
        # Check if groupsyncread data of Dynamixel#1 is available
        dxl_getdata_result = groupSyncReadPos.isAvailable(id, ADDR_PRO_PRESENT_POS , 4)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncReadPos getdata failed" % id)
        # Get Dynamixel present position value
        dxl_present_position = motor_angle_to_radian(groupSyncReadPos.getData(id, ADDR_PRO_PRESENT_POS , 4))
        present_positions[id]=dxl_present_position

    return present_positions

def get_velocities():# Read present position
    present_vel = [0,0,0,0,0,0]  
    # Syncread present position
    dxl_comm_result = groupSyncReadVel.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    
    for id in range(NUM_MOTORS):
        # Check if groupsyncread data of Dynamixel#1 is available
        dxl_getdata_result = groupSyncReadVel.isAvailable(id, ADDR_PRO_PRESENT_VEL , 4)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncRead getdata failed" % id)
        # Get Dynamixel present position value
        present_vel[id]= convert_to_rpm(groupSyncReadVel.getData(id, ADDR_PRO_PRESENT_VEL , 4))
    return present_vel

def calculate_jacobian(current_positions):
    success, kdl_tree = treeFromFile(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'archie_description', 'urdf','manipulator.urdf'))
    chain = kdl_tree.getChain("base_link", "link_6")

    jt_positions = kdl.JntArray(NUM_MOTORS)
    for i in range(NUM_MOTORS):
        jt_positions[i] = current_positions[i]

    jacobian = kdl.Jacobian(NUM_MOTORS)
    jac_solver = kdl.ChainJntToJacSolver(chain)
    jac_solver.JntToJac(jt_positions, jacobian)

    jacobian_matrix = []
    for i in range(jacobian.rows()):
        row = []
        for j in range(jacobian.columns()):
            row.append(round(jacobian[i, j],4))
        jacobian_matrix.append(row)

    return jacobian_matrix

def calculate_mass_matrix(current_positions):
    # Cargar el URDF y crear el árbol de KDL
    success, kdl_tree = treeFromFile(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'archie_description', 'urdf','manipulator.urdf'))
    chain = kdl_tree.getChain("base_link", "link_6")

    # Crear el vector de posiciones articulares
    jt_positions = kdl.JntArray(NUM_MOTORS)
    for i in range(NUM_MOTORS):
        jt_positions[i] = current_positions[i]

    # Crear la matriz de masas y el solver
    mass_matrix = kdl.JntSpaceInertiaMatrix(NUM_MOTORS)
    dyn_kdl = kdl.ChainDynParam(chain, kdl.Vector.Zero())
    dyn_kdl.JntToMass(jt_positions, mass_matrix)

    # Convertir la matriz de masas a una matriz de Python
    mass_matrix_py = []
    for i in range(mass_matrix.rows()):
        row = []
        for j in range(mass_matrix.columns()):
            row.append(round(mass_matrix[i, j],4))
        mass_matrix_py.append(row)

    return mass_matrix_py

def gravity_compensation(current_positions):
    success, kdl_tree = treeFromFile(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'archie_description', 'urdf','manipulator.urdf'))
    chain = kdl_tree.getChain("base_link", "link_6")

    grav_vector = kdl.Vector(0, 0, -9.81)  # relative to kdl chain base link
    dyn_kdl = kdl.ChainDynParam(chain, grav_vector)
    jt_positions = kdl.JntArray(NUM_MOTORS)
    for i in range(NUM_MOTORS):
        jt_positions[i] = current_positions[i]

    grav_matrix = kdl.JntArray(NUM_MOTORS)
    dyn_kdl.JntToGravity(jt_positions, grav_matrix)

    gravity_compensating_jt_torques = [grav_matrix[i] for i in range(grav_matrix.rows())]

    return gravity_compensating_jt_torques    

def set_sync_pwm(total_pwm):

    for id in range(NUM_MOTORS):
        new_pwm = convert_hex(total_pwm[id])
        param_goal_pwm = [DXL_LOBYTE(DXL_LOWORD(new_pwm)), 
                          DXL_HIBYTE(DXL_LOWORD(new_pwm)), 
                          DXL_LOBYTE(DXL_HIWORD(new_pwm)), 
                          DXL_HIBYTE(DXL_HIWORD(new_pwm))]
        # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWritePWM.addParam(id, param_goal_pwm)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWritePWM addparam failed" % id)

    dxl_comm_result = groupSyncWritePWM.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("Hubo un fallo en envío de PWM")
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Clear syncwrite parameter storage
    groupSyncWritePWM.clearParam()


def move_to_target(state_position: JointState):

    motor_positions  = get_positions()
    motor_velocities = get_velocities()
    current_positions = list(np.array(motor_positions, float))

    # Calcula los torques de gravedad para la posición actual
    gravity_torques = gravity_compensation(motor_positions)

    # Calcula los torques adicionales necesarios para moverse hacia la posición objetivo
    position_error = np.array(state_position.position) - np.array(current_positions)

    k_p = np.array([3, 3.5, 2.5, 2, 2, 2])
    c_p = np.array([0.2, 0.6, 0.15, 0.4, 0.1, 0.1])

    error_torques = (position_error*k_p)
    damp_torques  = ((np.array(motor_velocities, float)*(2*math.pi/60))*c_p) #primero convertimos vel a rad/s

    print("\n","Jacobian Matrix:")
    jacobian_matrix = calculate_jacobian(state_position.position)
    print(jacobian_matrix[0])
    print(jacobian_matrix[1])
    print(jacobian_matrix[2])
    print(jacobian_matrix[3])
    print(jacobian_matrix[4])
    print(jacobian_matrix[5])

    print("\n","Mass Matrix:")
    mass_matrix = calculate_mass_matrix(state_position.position)
    print(mass_matrix[0])
    print(mass_matrix[1])
    print(mass_matrix[2])
    print(mass_matrix[3])
    print(mass_matrix[4])
    print(mass_matrix[5])
    
    total_pwm = (gravity_torques + error_torques - damp_torques)*np.array([885/1.8, 885/1.8, 885/1.8, 885/1.8, 885/1.4, 885/1.4])
    for id in range(len(total_pwm)):
        total_pwm[id] = (round(total_pwm[id]) if (total_pwm[id] < 700 and total_pwm[id] > -700) else
                        (700      if total_pwm[id] > 700 else (-700)))
    total_pwm = np.array(total_pwm,int)
    
    motor_efforts = total_pwm/np.array([885/1.8, 885/1.8, 885/1.8, 885/1.8, 885/1.4, 885/1.4])

    joint_state_publisher(motor_positions, motor_velocities, motor_efforts)
    set_sync_pwm(np.array(total_pwm))

    rospy.logwarn(f"PWM: {total_pwm}")
    rospy.logwarn(f"Vel: {motor_velocities}")
    rospy.logwarn(f"Par: {motor_efforts}")
    rospy.logwarn(f"Err: {position_error*180/math.pi}")
    
    motor = MotorData()
    motor.position = motor_positions
    motor.error    = position_error
    motor.velocity = motor_velocities
    motor.effort   = motor_efforts

    motor_state_pub.publish(motor)
    print("=====================================================================================")


def joint_state_publisher(motor_positions, motor_velocities, motor_efforts):
    joints_states = JointState()
    joints_states.header = Header()
    joints_states.header.stamp = rospy.Time.now()
    joints_states.name = ['joint_'+str(id) for id in range(NUM_MOTORS)]
    
    #Publish the new joint state
    joints_states.position = motor_positions
    joints_states.velocity = motor_velocities
    joints_states.effort = motor_efforts
    joint_state_pub.publish(joints_states)


if __name__ == '__main__':

    ADDR_OPERATING_MODE         = 11               # Control table address is different in Dynamixel model
    ADDR_PRO_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
    ADDR_PRO_GOAL_PWM           = 100
    ADDR_PRO_PRESENT_PWM        = 124
    ADDR_PRO_PRESENT_VEL        = 128
    ADDR_PRO_PRESENT_POS        = 132
    NUM_MOTORS                  = 6


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

    # Initialize GroupSyncWrite/Read instance
    groupSyncWritePWM       = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_PWM   , 4)
    groupSyncWritePos       = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_PRESENT_POS, 4)
    groupSyncReadPos        = GroupSyncRead (portHandler, packetHandler, ADDR_PRO_PRESENT_POS, 4)
    groupSyncReadVel        = GroupSyncRead (portHandler, packetHandler, ADDR_PRO_PRESENT_VEL, 4)

    for id in range(NUM_MOTORS):
        # Add parameter storage for Dynamixel present position value
        dxl_addparam_resultPos = groupSyncReadPos.addParam(id)
        dxl_addparam_resultVel = groupSyncReadVel.addParam(id)

        if dxl_addparam_resultPos != True:
            print("[ID:%03d] groupSyncReadPos addparam failed" % id)
        if dxl_addparam_resultVel != True:
            print("[ID:%03d] groupSyncReadVel addparam failed" % id)
        


    rospy.init_node("motor_communication_pwm")
    r =rospy.Rate(10) # 10hz

    # Set operating mode to pwm control mode
    for id in range(NUM_MOTORS):     
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_OPERATING_MODE, PWM_CONTROL_MODE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Operating mode for motor", str(id), "changed to PWM control mode.")
    # Turn on motor's torque
    for id in range(NUM_MOTORS):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Torque of Motor",id,"is on")
    

    
    #Publish current robot state
    joint_state_pub = rospy.Publisher('/real_joint_states', JointState, queue_size=10)
    motor_state_pub = rospy.Publisher ("/motor_data" , MotorData, queue_size=1)
    subGoalState    = rospy.Subscriber('/joint_goals', JointState, callback = move_to_target, queue_size= 5)
    
    rospy.spin()    

    # Disable the torque
    for id in range(NUM_MOTORS):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Torque of Motor",id,"is off")

    #Setting to default control mode (Position Control Mode) 
    for id in range(NUM_MOTORS):     
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_OPERATING_MODE, POS_CONTROL_MODE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Operating mode for motor", str(id), "changed to Position Control Mode.")

    # Clear syncread parameter storage
    groupSyncReadPos.clearParam()

    portHandler.closePort()
