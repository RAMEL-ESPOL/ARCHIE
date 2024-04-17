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

# Altura del lapiz
global pen 
pen = 0.207
global quit
quit = 0

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
        waypoints, 0.00005, 0.0  # waypoints to follow  # eef_step
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

    wpose.position.x = 0.1
    wpose.position.y = 0.25
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x = -0.1
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x = 0.0
    wpose.position.y = 0.3
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = group.compute_cartesian_path(
        waypoints, 0.1, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold

    print_plan(waypoints, figure)
    return plan, fraction

def circle():
    figure = "Circle (r=0.15)"
    r = 0.05
    center_y = 0.2
    center_x = 0 
    waypoints = []

    wpose = group.get_current_pose().pose
    
    wpose.position.z = pen  # First move up (z)
    wpose.position.x = center_x
    wpose.position.y = center_y
    waypoints.append(copy.deepcopy(wpose))

    for theta in range(0, 361, 2):
        wpose.position. y = center_y + r*math.sin(theta*math.pi/180)
        wpose.position. x = center_x + r*math.cos(theta*math.pi/180)
        waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = group.compute_cartesian_path(
        waypoints, 0.005, 0.0 # waypoints to follow  # eef_step
    )  # jump_threshold

    print_plan(waypoints, figure)
    return plan, fraction

def plan_circle( center_x : float , center_y : float , r : float , theta_o : float  , theta_f : float , wpose, circle_waypoints : list , sentido_x : bool, sentido_y : bool):
    if (sentido_x and sentido_y):
        for theta in range(theta_o, theta_f + 1, 2):
            wpose.position.y = center_y + r*math.sin(theta*math.pi/180)
            wpose.position.x = center_x + r*math.cos(theta*math.pi/180)
            circle_waypoints.append(copy.deepcopy(wpose))
    elif (not(sentido_x) and sentido_y):
        for theta in range(theta_o, theta_f + 1, 2):
            wpose.position.y = center_y + r*math.sin(theta*math.pi/180)
            wpose.position.x = center_x - r*math.cos(theta*math.pi/180)
            circle_waypoints.append(copy.deepcopy(wpose))
    elif (sentido_x and not(sentido_y)):
        for theta in range(theta_o, theta_f + 1, 2):
            wpose.position.y = center_y - r*math.sin(theta*math.pi/180)
            wpose.position.x = center_x + r*math.cos(theta*math.pi/180)
            circle_waypoints.append(copy.deepcopy(wpose))
    else:
        for theta in range(theta_o, theta_f + 1, 2):
            wpose.position.y = center_y - r*math.sin(theta*math.pi/180)
            wpose.position.x = center_x - r*math.cos(theta*math.pi/180)
            circle_waypoints.append(copy.deepcopy(wpose))

    return circle_waypoints, wpose

def espol_logo():
    figure = "ESPOL (LOGO)"

    waypoints = []

    wpose = group.get_current_pose().pose
    
    wpose.position.z = pen + 0.055 # First move up (z)
    wpose.position.y = 0.2
    wpose.position.x = -0.1325
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))

    #Planeamiento de la "e"
    wpose.position.x = -0.1075
    waypoints.append(copy.deepcopy(wpose))

    (waypoints, wpose) = plan_circle(wpose.position.x, wpose.position.y, 0.025, 45, 270, wpose, waypoints , 1 , 1)

    #Planemiento de la "s"
    wpose.position.x += 0.035
    wpose.position.y += 0.05
    waypoints.append(copy.deepcopy(wpose))

    (waypoints, wpose) = plan_circle(wpose.position.x, wpose.position.y - 0.025, 0.025, 90, 290, wpose, waypoints , 0 , 1)
    
    wpose.position.z = pen + 0.055
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y = 0.2
    wpose.position.x = -0.0375
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))

    #Planeamiento de la "p"
    (waypoints, wpose) = plan_circle(wpose.position.x + 0.025, wpose.position.y, 0.025, 0, 360, wpose, waypoints, 0, 0)

    wpose.position.y -= 0.07
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = pen + 0.055
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += 0.06
    wpose.position.y = 0.2
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = pen 
    waypoints.append(copy.deepcopy(wpose))

    #Planeamiento de la "o"
    (waypoints, wpose) = plan_circle(wpose.position.x + 0.025, wpose.position.y, 0.025, 0, 360, wpose, waypoints, 0, 1)

    wpose.position.z = pen + 0.055
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += 0.085
    wpose.position.y -= 0.025
    waypoints.append(copy.deepcopy(wpose)) 

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose)) 

    #Planeamiento de la "l"
    (waypoints, wpose) = plan_circle(wpose.position.x, wpose.position.y + 0.025, 0.025, 90, 180, wpose, waypoints, 1, 0)

    wpose.position.y += 0.07
    waypoints.append(copy.deepcopy(wpose)) 

    wpose.position.z = pen + 0.055
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = group.compute_cartesian_path(
        waypoints, 0.0001, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold

    print_plan(waypoints, figure)
    return plan, fraction

def espol():
    figure = "ESPOL"
    waypoints = []

    wpose = group.get_current_pose().pose
    
    wpose.position.y = -0.15 +0.325
    wpose.position.x = -0.075 + 0.025
    waypoints.append(copy.deepcopy(wpose))   

    wpose.position.z = pen 
    waypoints.append(copy.deepcopy(wpose))

    #Drawing the "E"
    ########################################
    wpose.position.x = -0.075
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = pen + 0.02
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= 0.025
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = pen + 0.02
    wpose.position.y += 0.003
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= 0.003
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))
    
    wpose.position.x += 0.025
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = pen + 0.03
    wpose.position.y += 0.003
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= 0.003
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y = -0.15 +0.313
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x = -0.078
    waypoints.append(copy.deepcopy(wpose))
    ########################################

    wpose.position.z = pen + 0.055
    waypoints.append(copy.deepcopy(wpose))

    #Drawing the "S"
    ########################################
    wpose.position.x = -0.075 + 0.055
    wpose.position.y = -0.15 +0.325
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x -= 0.025
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = pen + 0.02
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y = -0.15 +0.313
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = pen + 0.02
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += 0.025
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = pen + 0.02
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= 0.013
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = pen + 0.02
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x -= 0.025
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = pen + 0.055
    waypoints.append(copy.deepcopy(wpose))
    ########################################

    wpose.position.x += 0.03
    wpose.position.y += 0.025
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))

    #Drawing the "P"
    ########################################
    wpose.position.y -= 0.025
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = pen + 0.02
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y += 0.025
    waypoints.append(copy.deepcopy(wpose))
    
    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += 0.025
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = pen + 0.02
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= 0.013
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = pen + 0.02
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x -= 0.025
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = pen + 0.055
    waypoints.append(copy.deepcopy(wpose))
    ########################################

    wpose.position.x += 0.03
    wpose.position.y = -0.15 +0.325
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))

    #Drawing the "O"
    ########################################
    wpose.position.x += 0.025
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = pen + 0.02
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= 0.025
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = pen + 0.02
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x -= 0.025
    wpose.position.y += 0.025
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= 0.025
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = pen + 0.02
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += 0.025
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = pen + 0.055
    waypoints.append(copy.deepcopy(wpose))
    ########################################

    wpose.position.x += 0.006
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y += 0.025
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))

    #Drawing the "L"
    ########################################
    wpose.position.y -= 0.025
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = pen + 0.02
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))
    
    wpose.position.x += 0.025
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = pen + 0.055
    waypoints.append(copy.deepcopy(wpose))
    ########################################

    (plan, fraction) = group.compute_cartesian_path(
        waypoints, 0.00005, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold

    print_plan(waypoints, figure)
    return plan, fraction

#By executing this file we can make the robot move to several preconfigured positions in Cartesian coordinates, in the order in which they are in the file
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('planing_node', anonymous=True)
rate = rospy.Rate(10)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()    
group = moveit_commander.MoveGroupCommander("arm_group")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)


'''
# We can get the name of the reference frame for this robot:
planning_frame = group.get_planning_frame()
print("============ Planning frame: %s" % planning_frame)

# We can also print the name of the end-effector link for this group:
eef_link = group.get_end_effector_link()
print("============ End effector link: %s" % eef_link)

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print("============ Available Planning Groups:", robot.get_group_names())

#Sometimes for debugging it is useful to print the entire state of the
# robot:
print("============ Printing robot state")
print(robot.get_current_state())
print("")
'''

home()
# Calling ``stop()`` ensures that there is no residual movement
group.stop()
while (not rospy.is_shutdown() and quit == 0):
    number = input("""
          
Choose a number to make de corresponding draw or write 'q' to close the program:

    1. Square
    2. Triangle
    3. Circle
    4. ESPOL (logo)
    5. ESPOL
    q. QUIT
          
Write the option: """)
    
    
    if (number == '1'):
        plan = square()[0]

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)
        
        group.execute(plan, wait=True)
        rospy.loginfo("Planning succesfully executed.\n")
        home()

    elif (number == '2'):
        plan = triangle()[0]

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)
        group.execute(plan, wait=True)
        rospy.loginfo("Planning succesfully executed.\n")
        home()
        
    elif (number == '3'):
        plan = circle()[0]

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

        group.execute(plan, wait=True)
        rospy.loginfo("Planning succesfully executed.\n")
        home()

    elif (number == '4'):
        plan = espol_logo()[0]

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

        group.execute(plan, wait=True)
        rospy.loginfo("Planning succesfully executed.\n")
        home()

    elif (number == '5'):
        plan = espol()[0]

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

        group.execute(plan, wait=True)
        rospy.loginfo("Planning succesfully executed.\n")
        home()

    elif (number == 'q' or number == 'Q'):
        quit = 1
        rospy.loginfo("Shutting down the program.\n")

    else:
        print("Please select one of the avaliable options.")

    rospy.sleep(1)
