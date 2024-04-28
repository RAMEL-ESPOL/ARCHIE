#! /usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from moveit_commander.conversions import pose_to_list
import math
import numpy as np
from spatialmath_rospy import to_spatialmath, to_ros
from spatialmath import SE3, SO3

# Altura del lapiz
global pen 
pen = 0.163
global quit
quit = 0

global theta
theta = 60
global rmatrix
rmatrix = SE3.Rx(theta,'deg')

def home():
    # We get the joint values from the group and change some of the values:
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = 0
    joint_goal[2] = 0
    joint_goal[3] = 0
    joint_goal[4] = 0
    joint_goal[5] = 0

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)
    rospy.loginfo("The robotic arm is at home position.")

def loginfog(msg: str):
    rospy.loginfo("\033[92m%s\033[0m" % msg)

def print_plan(w: list, s: str):
    message = "\n--------------------------------------------------------------"
    for i in range(len(w)):
        message += """
Pose {0}:\n{1}
--------------------------------------------------------------""".format( i , w[i])
    
    rospy.loginfo(message)
    loginfog("Drawing a " + s)

def square():
    figure = "square (0.2 x 0.2)"
    waypoints = []

    #Se copia la pose actual para únicamente modificar las coordenadas cartesianas y que la orientación
    #del efector final no se vea modificada, de esta manera mantenemos el lápiz perpendicular al suelo
    wpose = group.get_current_pose().pose
    
    wpose.position.z = pen  # First move up (z)
    wpose.position.x = 0.1
    wpose.position.y = 0.3
    
    waypoints.append(copy.deepcopy(wpose))    

    wpose.position.x = -0.1
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y = 0.2
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x = 0.1
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y = 0.3
    waypoints.append(copy.deepcopy(wpose))


    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = group.compute_cartesian_path(
        waypoints, 0.1, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold

    print_plan(waypoints, figure)
    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

def triangle():
    figure = "Triangle (h=0.1   b=0.2)"
    waypoints = []

    wpose = group.get_current_pose().pose
    
    wpose.position.z = pen   # First move up (z)
    wpose.position.y = 0.3
    wpose.position.x = 0.0
    waypoints.append(copy.deepcopy(wpose))    
    #print(to_spatialmath(wpose))

    wpose.position.x = 0.1
    wpose.position.y = 0.25
    waypoints.append(copy.deepcopy(wpose))
    #print(to_spatialmath(wpose))

    wpose.position.x = -0.1
    waypoints.append(copy.deepcopy(wpose))
    #print(to_spatialmath(wpose))

    wpose.position.x = 0.0
    wpose.position.y = 0.3
    waypoints.append(copy.deepcopy(wpose))
    #print(to_spatialmath(wpose))#.printline()


    #Matriz de rotación usando la orientación del efector final
    T = SE3(wpose.position.x, wpose.position.y, wpose.position.z)* SE3.Rz(-88, 'deg')* SE3.Rx(180, 'deg')
    way = []
    for i in range(len(waypoints)):
        T = SE3(waypoints[i].position.x, waypoints[i].position.y, waypoints[i].position.z)* SE3.Rz(-88, 'deg')* SE3.Rx(180, 'deg')
        way.append(to_ros(T*rmatrix))
    
    rospy.logerr(to_ros(T*rmatrix))
    #rospy.logerr(SE3(np.array(SE3(rmatrix))))



    (plan, fraction) = group.compute_cartesian_path(
        waypoints, 0.1, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold

    print_plan(way, figure)
    return plan, fraction


#By executing this file we can make the robot move to several preconfigured positions in Cartesian coordinates, in the order in which they are in the file
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('planing_node', anonymous=True)
rate = rospy.Rate(10)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()    
group = moveit_commander.MoveGroupCommander("arm_group")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)


home()
# Calling ``stop()`` ensures that there is no residual movement
group.stop()

plan = triangle()[0]
display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan)
# Publish
display_trajectory_publisher.publish(display_trajectory)
group.execute(plan, wait=True)
rospy.loginfo("Planning succesfully executed.\n")
home()

rospy.sleep(1)
