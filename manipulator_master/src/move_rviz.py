#!/usr/bin/env python3

import rospy
#By executing this file we will be able to move the robot in the real world to the position where it is currently in RViz.
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import String
import moveit_commander

class movement:
    def __init__(self):
        rospy.init_node("motion_control_rviz")
        self.r = rospy.Rate(10)
        self.pub = rospy.Publisher('/joint_goals', JointState, queue_size=1)
       

    def loop(self):
        self.main()
        self.r.sleep()


    def main(self):
        group = moveit_commander.MoveGroupCommander("arm_group")
        print(type(group.get_current_joint_values))

        time=1
        walkingtime=0.5

        self.joints_states = JointState()
        self.joints_states.header = Header()
        self.joints_states.header.stamp = rospy.Time.now()
        self.joints_states.name = ["joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"]

        self.joint_position_state = list(group.get_current_joint_values())
        self.joints_states.position = self.joint_position_state
        #self.pub.publish(self.joints_states)
        self.pub.publish(self.joints_states)
        print("Posición de RViz")
        rospy.sleep(time)


if __name__ == '__main__':
    movement= movement()
    while not rospy.is_shutdown():
        movement.loop()