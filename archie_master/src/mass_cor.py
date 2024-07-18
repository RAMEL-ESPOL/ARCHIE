#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
from sensor_msgs.msg import JointState
import PyKDL as kdl
from urdf_parser_py.urdf import URDF

def get_robot_dynamics():
    # Inicializar MoveIt Commander y nodo ROS
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('robot_dynamics', anonymous=True)

    # Inicializar el grupo del manipulador
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("arm_group")
    
    # Obtener la configuración actual de las articulaciones
    joint_state = rospy.wait_for_message("/joint_states", JointState)
    joint_positions = joint_state.position

    # Cargar el URDF del robot
    robot_urdf = URDF.from_parameter_server()
    kdl_tree = kdl.treeFromUrdfModel(robot_urdf)
    chain = kdl_tree.getChain("base_link", "end_effector_link")

    # Crear la estructura de cadenas de KDL
    kdl_chain = kdl.ChainDynParam(chain, kdl.Vector.Zero())

    # Obtener la matriz de inercia
    inertia_matrix = kdl.JntSpaceInertiaMatrix(len(joint_positions))
    kdl_chain.JntToMass(kdl.JntArray(joint_positions), inertia_matrix)

    # Obtener el vector de Coriolis
    coriolis_vector = kdl.JntArray(len(joint_positions))
    kdl_chain.JntToCoriolis(kdl.JntArray(joint_positions), kdl.JntArray(len(joint_positions)), coriolis_vector)

    return inertia_matrix, coriolis_vector

if __name__ == "__main__":
    inertia_matrix, coriolis_vector = get_robot_dynamics()
    print("Inertia Matrix:\n", inertia_matrix)
    print("Coriolis Vector:\n", coriolis_vector)
