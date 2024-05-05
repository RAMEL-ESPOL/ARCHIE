#!/usr/bin/env python3
from math import pi
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from dynamixel_sdk import *


#IDs starting from 0
class Motor:
    def __init__(self, usb_port, dxl_baud_rate, list_ids,portHandler,packetHandler,r):
        self.portHandler = portHandler
        self.r = r
        self.packetHandler = packetHandler
        self.protocol_version = 2.0 ###################### CHANGE IF REQUIRED, IT IS ASSUMED ALL MOTORS WORK WITH 2.0
        # !!!! Protocol version can be updated
        self.list_ids = list_ids
        #self.temperature=[0,0]
        


    def communication(self,dxl_baud_rate,addr_torque_enable):
        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(dxl_baud_rate):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            quit()

        for id in self.list_ids:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, addr_torque_enable, 0)
            if dxl_comm_result != COMM_SUCCESS:
                print("Dynamixel: ",id,"%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("Dynamixel: ",id,"%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel: ",id," has been successfully connected")
    
    def torque(self,order,addr_torque_enable):
        for id in self.list_ids:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, addr_torque_enable, order)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                if order == 1:
                    print("Torque of Motor ",id," is on")
                else:
                    print("Torque of Motor ",id," is off")
    
    def maxspeed(self,addr_max_velocity,value_max_velocity):
        for id in self.list_ids:     
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, id, addr_max_velocity, value_max_velocity)
            if dxl_comm_result != COMM_SUCCESS:
                print("Dynamixel: ",id,"%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("Dynamixel: ",id,"%s" % self.packetHandler.getRxPacketError(dxl_error))

        

    """def angleConversion(self, raw_value=0, to_radian_bool=0,min_value_angle=0,max_value_angle=0,min_angle_deg=0,max_angle_deg=0):
        zero_value_angle = (max_value_angle-min_value_angle)//2 + 1
        values_movement_span_2 = (max_value_angle-min_value_angle)//2 
        deg_movement_span_2 = (max_angle_deg-min_angle_deg)//2
        rad_movement_span_2 = float(deg_movement_span_2*(pi/180))        

        if to_radian_bool:
            if (raw_value == zero_value_angle):
                return 0.0
            else:
                return ((float)(raw_value-zero_value_angle)/(float)(zero_value_angle))*rad_movement_span_2
        else:
            if (int(raw_value) == 0):
                return zero_value_angle
            else:
                return int(values_movement_span_2*(raw_value/ pi) + zero_value_angle)"""



class XCseries_motor(Motor):
    global l_pid
    '''
    Dynamixel 2XL430-W250-T features: 
    https://emanual.robotis.com/docs/en/dxl/x/2xl430-w250/
    '''
    def __init__(self,usb_port, dxl_baud_rate, list_ids,portHandler,packetHandler,r,value_max_velocity,max_min_range_dict,list_pid):
        global l_pid
        super().__init__(usb_port, dxl_baud_rate, list_ids,portHandler,packetHandler,r)
        #Define constant values and addresses
        self.addr_torque_enable = 64
        self.addr_goal_position = 116
        self.addr_present_position = 132
        self.addr_present_velocity = 128
        self.addr_goal_velocity = 104
        self.torque_enable = 1
        self.torque_disable = 0
        self.addr_present_load = 126
        self.minimum_position_value = 0
        self.maximum_position_value = 4095
        self.moving_status_threshold = 10
        self.addr_pos_pgain =  84
        self.addr_pos_igain = 82
        self.addr_pos_dgain = 80
        self.max_angle_deg = 360
        self.min_angle_deg = 0
        self.addr_velocity_limit = 44 #size = 4 Bytes, Initial Value = 250, Range = 0 - 1023, Unit = 0.29 rev/min
        self.dict_range = max_min_range_dict

        self.angle_zero = (self.maximum_position_value-self.minimum_position_value)//2 +1

        #FALTA DE VELOCIDAD
        self.communication(dxl_baud_rate,self.addr_torque_enable)
        self.torque(self.torque_enable,self.addr_torque_enable)
        self.config_pid_cts(list_pid)
        l_pid = list_pid
        
        
        #self.maxspeed(self.addr_velocity_limit,value_max_velocity)

    #0,1,2  p,i,d
    def config_pid_cts(self,list_pid):
        addr_pid = [self.addr_pos_pgain,self.addr_pos_igain,self.addr_pos_dgain]
        for id in self.list_ids:
            for constant in range(len(list_pid[id])):
                dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, id, addr_pid[constant], list_pid[id][constant])
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % self.packetHandler.getRxPacketError(dxl_error))
    
    def get_pid_list(self):
        return l_pid


    def angleConversion(self, raw_value, to_radian_bool,id):

        #raw_value=0, to_radian_bool=0,min_value_angle=0,max_value_angle=0,min_angle_deg=0,max_angle_deg=0

        min_value_angle=self.minimum_position_value
        max_value_angle=self.maximum_position_value
        min_angle_deg=self.min_angle_deg
        max_angle_deg=self.max_angle_deg

        zero_value_angle = (max_value_angle-min_value_angle)//2 + 1
        values_movement_span_2 = (max_value_angle-min_value_angle)//2 
        deg_movement_span_2 = (max_angle_deg-min_angle_deg)//2
        rad_movement_span_2 = float(deg_movement_span_2*(pi/180))        

        if to_radian_bool:
            if (raw_value == zero_value_angle):
                return 0.0
            else:
                return ((float)(raw_value-zero_value_angle)/(float)(zero_value_angle))*rad_movement_span_2
        else:
            if (raw_value == 0.0):
                return zero_value_angle
            else:
                if raw_value > self.dict_range[id][1]:
                    raw_value = self.dict_range[id][1]
                elif raw_value < self.dict_range[id][0]:
                    raw_value = self.dict_range[id][0]
                return int(values_movement_span_2*(raw_value/ pi) + zero_value_angle)
        #return 

 #   def set_positions(self, bool_init, data):
 #       return super().set_positions(bool_init, data, self.addr_goal_position, self.angle_zero, self.angleConversion)

  #  def get_positions(self):
   #     return super().get_positions(self.addr_present_position, self.maximum_position_value)

    #def joint_state_publisher(self):
     #   return super().joint_state_publisher(self.angleConversion)
