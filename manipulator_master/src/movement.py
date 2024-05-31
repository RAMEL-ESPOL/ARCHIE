#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import String
#In this file we can configure different positions for the joints by entering the number corresponding to 
#each configuration and the robot will move to that position.
class movement:
    def __init__(self):
        rospy.init_node("motion_control")
        self.r = rospy.Rate(10)
        self.pub = rospy.Publisher('/joint_state', JointState, queue_size=1)
       

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
            
            number = input ("Enter number: ")
            #limites BASE 0:[-1.57,1.57],1:[-0.7,0.7] -Sentido Horario y +Sentido antihorario
            #limites CODO 2:[-1.15,2],3:[-3.14,3.14]
            #limites ee{4:[-1.15,2],5:[-3.14,3.14]}
           
            if (number=='0'):
            
                self.joint_position_state=[0.0,0.0,0.0,0.0,0.0,0.0]
                self.joints_states.position = self.joint_position_state
                #self.pub.publish(self.joints_states)
                self.pub.publish(self.joints_states)
                print("Home\n")
                rospy.sleep(time)

            if (number=='1'):

                self.joint_position_state=[0.1,0.1,0.1,0.1,0.1,0.1]
                self.joints_states.position = self.joint_position_state
                #self.pub.publish(self.joints_states)
                self.pub.publish(self.joints_states)
                print("Configuracion 1\n")
                rospy.sleep(time)

            if (number=='2'):

                self.joint_position_state=[0,2,0.2,0.2,0.2,0.2,0.2]
                self.joints_states.position = self.joint_position_state
                #self.pub.publish(self.joints_states)
                self.pub.publish(self.joints_states)
                print("Configuracion 2\n")
                rospy.sleep(time)



            if (number=='3'):

                self.joint_position_state=[0,3,0.3,0.3,0.3,0.3,0.3]
                self.joints_states.position = self.joint_position_state
                #self.pub.publish(self.joints_states)
                self.pub.publish(self.joints_states)
                print("Configuracion 3\n")
                rospy.sleep(time)



            if (number=='4'):

                self.joint_position_state=[0,5,0.5,0.5,0.5,0.5,0.5]
                self.joints_states.position = self.joint_position_state
                #self.pub.publish(self.joints_states)
                self.pub.publish(self.joints_states)
                print("Configuracion 4\n")
                rospy.sleep(time)


            if (number=='5'):

                self.joint_position_state=[0,0,1.57,0,0,0]
                self.joints_states.position = self.joint_position_state
                #self.pub.publish(self.joints_states)
                self.pub.publish(self.joints_states)
                print("Configuracion 5\n")
                rospy.sleep(time)


if __name__ == '__main__':
    movement= movement()
    while not rospy.is_shutdown():
        movement.loop()