#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import JointState

global pos
pos = [0.0,0.0,0.0,0.0,0.0,0.0]

def state_position(state: JointState):#Recibimos del Subscriber un msg de tipo JointState y posteriormente lo publicamos con el Publisher
    global pos
    if (state.position != pos ):
        pub.publish(state)
    
    pos = state.position
    rospy.sleep(0.1)

def real_position(state: JointState):
    rospy.logerr(state.position)
    rospy.logwarn("hola")

if __name__ == "__main__":
    rospy.init_node("move_arm_node")
    pub = rospy.Publisher ("/joint_goals" , JointState, queue_size=10)
    subRobotState = rospy.Subscriber("/joint_states", JointState, callback=state_position)
    subRealState  = rospy.Subscriber("/real_joint_states", JointState, callback= real_position)

    rospy.logwarn("The move_arm_node has been started")
    rospy.spin()


