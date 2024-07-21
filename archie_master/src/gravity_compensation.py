#!/usr/bin/env python3

import rospy
import numpy as np
import moveit_commander
import sys
from   sensor_msgs.msg import JointState
from std_msgs.msg import Float64

def calculate_torque(joints: JointState):
    jac_t = np.transpose(group.get_jacobian_matrix(list(np.array(joints.position,float))))
    gravity = -9.81
    mass_vector = np.transpose(np.array([0.102, 0.101, 0.102, 0.105, 0.102, 0.05],float))
    gravity_force = mass_vector * gravity
    gravity_compensation = np.dot(jac_t, gravity_force)

    print(gravity_compensation)
    return gravity_compensation

def move_to_target(goal_position):
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        joint_states = rospy.wait_for_message("/real_joint_states", JointState)
        current_positions = list(np.array(joint_states.position, float))

        # Calcula los torques de gravedad para la posición actual
        gravity_torques = calculate_torque(current_positions)

        # Calcula los torques adicionales necesarios para moverse hacia la posición objetivo
        position_error = np.array(goal_position) - np.array(current_positions)
        k_p = 10  # Ganancia proporcional (ajusta según sea necesario)
        position_torques = k_p * position_error

        total_torques = gravity_torques + position_torques

        # Publica los torques totales a los controladores de esfuerzo de las articulaciones
        for i, pub in enumerate(effort_pubs):
            effort_msg = Float64()
            effort_msg.data = total_torques[i]
            pub.publish(effort_msg)

        rate.sleep()


if __name__ == "__main__":
    rospy.init_node("gravity_compensation_node")
    moveit_commander.roscpp_initialize(sys.argv)
    subGoalState    = rospy.Subscriber("/real_joint_states", JointState, callback = calculate_torque)
    group           = moveit_commander.MoveGroupCommander("arm_group")

    rospy.logwarn("The move_arm_node has been started")
    rospy.spin()