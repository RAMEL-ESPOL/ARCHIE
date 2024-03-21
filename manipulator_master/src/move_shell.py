#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import String
import sys
import math
#By executing this file we will be able to move the robot in the real world by sending it by console 5 positions for the radiant joints.
class movement:
    def __init__(self):
        rospy.init_node("motion_control_shell")
        self.r = rospy.Rate(10)
        self.pub = rospy.Publisher('/joint_goals', JointState, queue_size=1)
       

    def loop(self):
        self.main()
        self.r.sleep()


    def main(self):
        time=1
        walkingtime=0.5

        self.joints_states = JointState()
        self.joints_states.header = Header()
        self.joints_states.header.stamp = rospy.Time.now()
        self.joints_states.name = ["joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"]

        while not rospy.is_shutdown():
            
            jp = list(np.array((input("Write Joint Positions: ")).split(), float))
            #limites BASE 0:[-1.57,1.57],1:[-0.7,0.7] -Sentido Horario y +Sentido antihorario
            #limites CODO 2:[-1.15,2],3:[-3.14,3.14]
            #limites ee{4:[-1.15,2],5:[-3.14,3.14]}
        
            self.joint_position_state= jp
            self.joints_states.position = self.joint_position_state
            #self.pub.publish(self.joints_states)
            self.pub.publish(self.joints_states)
            print("Base")
            rospy.sleep(time)



if __name__ == '__main__':
    movement= movement()
    while not rospy.is_shutdown():
        movement.loop()