#!/usr/bin/env python3

import sys
import rospy
import numpy as np
from   sensor_msgs.msg import JointState
from std_msgs.msg import String
from geometry_msgs.msg import Point
import os
import math
import moveit_commander
from visualization_msgs.msg import Marker, MarkerArray

global pos
pos =       [0.0,0.0,0.0,0.0,0.0,0.0]
global real_pos
real_pos =  [0.0,0.0,0.0,0.0,0.0,0.0]

#Recibimos del Subscriber un msg de tipo JointState de moveit y posteriormente lo publicamos con el Publisher como goal
def state_position(fake_joint_state: JointState):
    global pos
    goal_state = JointState()
    if (fake_joint_state.position != pos ):
        #goal_state.velocity = [10, 10, 10, 10, 10, 10]
        pos = fake_joint_state.position
        ee = pos[-2]
        ee_rad = (ee*1.57)/0.02
        goal = tuple(list(pos[:-3]) + [ee_rad])
        goal_state.position = goal
        pub.publish(goal_state)

    j_array = np.array(pos)*180/math.pi
    pos = list(j_array)
    

if __name__ == "__main__":
    rospy.init_node("move_arm_node")
    moveit_commander.roscpp_initialize(sys.argv)
    pub             = rospy.Publisher ("/joint_goals" , JointState, queue_size=10)
    subGoalState    = rospy.Subscriber("/joint_states", JointState, callback = state_position)

    rospy.logwarn("The move_arm_node has been started")
    rospy.spin()