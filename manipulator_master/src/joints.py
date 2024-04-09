#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState

global pos
pos = [0.0,0.0,0.0,0.0,0.0,0.0]

def real_position(state: JointState):
    rospy.logerr(state.position)
    rospy.logwarn("hola")

if __name__ == "__main__":
    rospy.init_node("move_arm_node")
    subRealState  = rospy.Subscriber("/real_joint_states", JointState, callback= real_position)

    rospy.logwarn("The move_arm_node has been started")
    rospy.spin()