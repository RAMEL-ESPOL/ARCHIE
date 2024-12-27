#! /usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import String
import math
import write_word

if __name__ == "__main__":

    #By executing this file we can make the robot move to several preconfigured positions in Cartesian coordinates, in the order in which they are in the file
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('class_node', anonymous=True)
    rate = rospy.Rate(10)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()    
    group = moveit_commander.MoveGroupCommander("arm_group")
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
    data_writing_publisher       = rospy.Publisher('/figure_writing', String, queue_size=2)


    # Altura del lapiz
    pen = 0.22

    #El paso que habrá entre una coordenada y la siguiente a la hora de escribir
    t = 0.01

    #Altura máxima a la que llegará cada letra en Y
    y_h = 0.23

    #Tamaño de cada letra en ancho y alto
    size = 0.02

    #Espacio entre cada letra
    space = 0.005

    theta = 0

    x_i = -0.5
    
    waypoints = []

    write_word.home()

    wpose = group.get_current_pose().pose

    (waypoints, wpose) = write_word.write(wpose, waypoints, robot, scene, group, display_trajectory_publisher, data_writing_publisher, "math class", y_h , -1*((len("math class")/2) * (size + space)), 1.2*size, space, pen, theta, t )

    plan  = group.compute_cartesian_path(waypoints, t, True)[0]

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory)

    group.execute(plan, wait=True)
    rospy.loginfo("Planning succesfully executed.\n")
    rospy.sleep(1)
    data_writing_publisher.publish("_none")

    write_word.joint_move([1.57, 0, 0, 0, 0, 0])

    waypoints = []


    (waypoints, wpose) = write_word.write(wpose, waypoints, robot, scene, group, display_trajectory_publisher, data_writing_publisher, "triangle area", y_h - 0.1, -1*((len("triangle area"))/2 * (size + space)), 1.2*size, space, pen, theta, t )

    (waypoints, wpose) = write_word.triangle(wpose, waypoints, data_writing_publisher, 0.25, x_i, y_h - 0.3, pen, theta )

    (waypoints, wpose) = write_word.write(wpose, waypoints, robot, scene, group, display_trajectory_publisher, data_writing_publisher, "A=(b*h)/2", y_h -0.25, x_i + 0.2, size, space, pen, theta, t )

    (waypoints, wpose) = write_word.write(wpose, waypoints, robot, scene, group, display_trajectory_publisher, data_writing_publisher, "A=(20*17)/2", y_h - 0.35, x_i + 0.2, size, space, pen, theta, t )


    (waypoints, wpose) = write_word.write(wpose, waypoints, robot, scene, group, display_trajectory_publisher, data_writing_publisher, "A=170 cm", y_h - 0.45, x_i + 0.2, size, space, pen, theta, t )

    (waypoints, wpose) = write_word.write(wpose, waypoints, robot, scene, group, display_trajectory_publisher, data_writing_publisher, "2", y_h - 0.42, -wpose.position.y, size*0.4, space, pen, theta, t )

    plan  = group.compute_cartesian_path(waypoints, t, True)[0]

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory)

    group.execute(plan, wait=True)
    rospy.loginfo("Planning succesfully executed.\n")
    rospy.sleep(1)
    data_writing_publisher.publish("_none")