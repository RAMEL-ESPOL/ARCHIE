#! /usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import math
from spatialmath_rospy import to_spatialmath, to_ros
from spatialmath import SE3, SO3

# Altura del lapiz
global pen 
#pen = 0.247 
#pen = 0.221
#pen = 0.186
#pen = 0.15
pen = 0.1234
global quit
quit = 0

global theta
theta = 35

global rmatrix
rmatrix = SE3.Rx(theta,'deg')

global t
t = 0.00004

#Altura máxima a la que llegará cada letra en Y
global y_h 
y_h = 0.35

#Tamaño de cada letra en ancho y alto
global size
size = 0.02

#Espacio entre cada letra
global space
space = 0.01

def home():
    
    joint_goal = group.get_current_joint_values()
    joint_goal = [1.267872420911887e-06, 0.16714812767180828, 0.24872782685877684, -3.421257973905653e-07, -0.41587595452500437, -3.5425905235787057e-07]

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)
    """
    waypoints = []

    wpose = group.get_current_pose().pose 
    #wpose.position.x = 0 #0.010161332452417767
    wpose.position.y += 0.2
    wpose.position.z -= 0.1
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = group.compute_cartesian_path(waypoints, t, 0.0)  # jump_threshold

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)

    # Publish
    display_trajectory_publisher.publish(display_trajectory)
    """
    #group.execute(plan, wait=True)

    rospy.loginfo("The robotic arm is at home position.")
    return wpose

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

def pen_up_down(wpose, waypoints : list):
    wpose.position.z = pen + 0.02    
    wpose.position.y += 0.005
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= 0.005
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))

    return wpose, waypoints

def up_pen(wpose, waypoints : list):
    wpose.position.z = pen + 0.02
    wpose.position.y += 0.01
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= 0.01
    waypoints.append(copy.deepcopy(wpose))
    return wpose, waypoints

def down_pen(wpose, waypoints : list):

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))

    return wpose, waypoints
    
def move_pen(wpose, waypoints : list, d_x : float, d_y: float, d_z : float = 0):
    #Se copia la pose actual para únicamente modificar las coordenadas cartesianas y que la orientación
    #del efector final no se vea modificada, de esta manera mantenemos el lápiz perpendicular al suelo

    wpose.position.x += d_x
    wpose.position.y = (y_h if d_y == y_h else
                       (wpose.position.y + d_y))
    if (d_z != 0):
        wpose.position.z = d_z

    waypoints.append(copy.deepcopy(wpose))

    return (wpose, waypoints)

def set_pen(wpose, waypoints : list, p_x : float, p_y: float, p_z : float = 0):
    
    wpose.position.x = p_x
    wpose.position.y = p_y
    wpose.position.z = p_z
    waypoints.append(copy.deepcopy(wpose))

    return (wpose, waypoints)

def plane_rotation(waypoints : list):
    way = []
    for i in range(len(waypoints)):
        #Primer elemento del producto es la parte de traslación de la matriz de transformación la cual usa las coordenadas cartsianas de cada pose (wpose)
        #Segundo y tercer elementos del producto son las rotaciones necesarias para que el efector final esté perpendicular al plano XY
        T = (SE3(waypoints[i].position.x, waypoints[i].position.y, waypoints[i].position.z)) * (SE3.Rz(-88, 'deg')) * (SE3.Rx(180, 'deg'))
        
        #La matriz T representa la orientación y posición del efector final con respecto al plano XY original, al multiplicarla por 
        #la matriz de rotación obtenemos la orientación y posición del efector con respecto al plano con la inclinación indicada
        way.append(copy.deepcopy(to_ros(rmatrix*T)))
        
    return way
    
def square(wpose, waypoints: list):
    square_size = 0.025
    figure = "Square (" + str(square_size) + "x" + str(square_size) + ")"
    figure_message = "_square"
    
    (wpose, waypoints) = set_pen(wpose, waypoints, -square_size/2, y_h, pen + 0.02)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -square_size)

    (wpose, waypoints) = move_pen(wpose, waypoints, square_size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = set_pen(wpose, waypoints, -square_size/2, y_h, pen + 0.02)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, square_size, y_h)

    (wpose, waypoints) = pen_up_down(wpose, waypoints)
    
    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -square_size)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = set_pen(wpose, waypoints, -square_size/2, y_h -2*square_size/5, pen + 0.02)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, square_size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints)
        
    return waypoints, wpose, figure, figure_message

def triangle(wpose, waypoints: list):
    h_t = 0.025
    b_t = 0.05
    figure = "Triangle (h = " + str(h_t) + " b = " + str(b_t) + ')'
    figure_message = "_triangle"

    (wpose, waypoints) = set_pen(wpose, waypoints, 0, y_h, pen + 0.02)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, -b_t/2, -h_t)

    (wpose, waypoints) = move_pen(wpose, waypoints, b_t, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, -b_t/2, y_h)
    
    (wpose, waypoints) = up_pen(wpose, waypoints)
        
    return waypoints, wpose, figure, figure_message

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

    return wpose, circle_waypoints

def circle(wpose, waypoints: list):
    r = size/2
    figure = "Circle ( " + str(r) + " )"
    center_y = y_h - r
    center_x = 0 
    figure_message = "_circle"

    (wpose, waypoints) = set_pen(wpose, waypoints, 0, y_h, pen + 0.02)
    
    (wpose, waypoints) = plan_circle(center_x, center_y, r, 90, 450, wpose, waypoints, 1, 1)

    (wpose, waypoints) = up_pen(wpose, waypoints)
        
    return waypoints, wpose, figure, figure_message

def espol_logo(wpose, waypoints: list):
    figure = "ESPOL (LOGO)"
    r = size/2
    figure_message = "_espol_logo"
    
    (wpose, waypoints) = set_pen(wpose, waypoints, -(2*(size + space)) - r, y_h, pen + 0.02)
    
    #Planeamiento de la "e"
    (wpose, waypoints) = move_pen(wpose, waypoints, r, -r)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = plan_circle(wpose.position.x, wpose.position.y, r, 45, 270, wpose, waypoints , 1 , 1)

    #Planemiento de la "s"
    (wpose, waypoints) = move_pen(wpose, waypoints, space + r, y_h)
    
    (wpose, waypoints) = plan_circle(wpose.position.x, wpose.position.y - r, r, 90, 290, wpose, waypoints , 0 , 1)
    
    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, space + r*(1-math.cos(290 * math.pi/180)), abs(r*math.sin(math.radians(290))))
    
    (wpose, waypoints) = down_pen(wpose, waypoints)

    #Planeamiento de la "p"
    (wpose, waypoints) = plan_circle(wpose.position.x + r, wpose.position.y, r, 0, 360, wpose, waypoints, 0, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size*0.7)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, space + size, y_h)

    #Planeamiento de la "o"
    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -r)

    (wpose, waypoints) = plan_circle(wpose.position.x + r, wpose.position.y, r, 0, 360, wpose, waypoints, 0, 1)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, space + size, y_h)

    #Planeamiento de la "l"
    (wpose, waypoints) = move_pen(wpose, waypoints, 0, r*0.2)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -r*0.2 - r)

    (wpose, waypoints) = plan_circle(wpose.position.x, wpose.position.y + r, r, 180, 270, wpose, waypoints, 1, 1)

    (wpose, waypoints) = up_pen(wpose, waypoints)
        
    return waypoints, wpose, figure, figure_message

def espol(wpose, waypoints : list):
    figure = "ESPOL"
    figure_message = "_espol"

    (wpose, waypoints) = set_pen(wpose, waypoints, -(2*(space + size)) - size/2 + size, y_h, pen + 0.002)
    
    (wpose, waypoints) = down_pen(wpose, waypoints)

    #Drawing the "E"
    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, 0.007, pen + 0.02)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -0.007)
    
    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size/2, pen + 0.02)
    
    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size/2)
    
    (wpose, waypoints) = down_pen(wpose, waypoints)
    
    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size/2, pen + 0.02)
    
    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)
    
    (wpose, waypoints) = up_pen(wpose, waypoints)
    

    #Drawing the "S"
    (wpose, waypoints) = move_pen(wpose, waypoints, 2*size + space, y_h)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = pen_up_down(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size/2)
    
    (wpose, waypoints) = move_pen(wpose, waypoints, 0, 0.01, pen + 0.02)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -0.01)

    (wpose, waypoints) = down_pen(wpose, waypoints)
    
    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size/2, pen + 0.02)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size/2)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size/2)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size/2, pen + 0.02)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size/2)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size/2, pen + 0.02)

    (wpose, waypoints) = move_pen(wpose, waypoints, space + size, y_h)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    #Drawing the "P"
    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size/2, pen + 0.02)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size/2)
    
    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = pen_up_down(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size/2)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size/2, pen + 0.02)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size/2)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size/2, pen + 0.02)

    (wpose, waypoints) = move_pen(wpose, waypoints, space + size, y_h)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    
    #Drawing the "O"
    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = pen_up_down(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size/2, pen + 0.02)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, size/2)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size/2, pen + 0.02)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size/2)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size/2, pen + 0.02)

    (wpose, waypoints) = move_pen(wpose, waypoints, space, y_h)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    
    #Drawing the "L"
    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size/2, pen + 0.02)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size/2)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints)
        
    return waypoints, wpose, figure, figure_message

#By executing this file we can make the robot move to several preconfigured positions in Cartesian coordinates, in the order in which they are in the file
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('planing_node', anonymous=True)
rate = rospy.Rate(10)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()    
group = moveit_commander.MoveGroupCommander("arm_group")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
data_writing_publisher = rospy.Publisher('/figure_writing', String, queue_size=2)
data_writing_publisher.publish(("_none"))
wpose = group.get_current_pose().pose
wpose = home()
rospy.logerr(group.get_current_joint_values())
# Calling ``stop()`` ensures that there is no residual movement
group.stop()

while (not rospy.is_shutdown() and quit == 0):
    
    waypoints = []
    number = input("""
          
Choose a number to make de corresponding draw or write 'q' to close the program:

    1. Square
    2. Triangle
    3. Circle
    4. ESPOL (logo)
    5. ESPOL
    q. QUIT
          
Write the option: """)
    
    if number.upper() != "Q":
        (waypoints, wpose, figure, figure_message) = (square    (wpose, waypoints) if number == "1" else
                                                     (triangle  (wpose, waypoints) if number == "2" else
                                                     (circle    (wpose, waypoints) if number == "3" else
                                                     (espol_logo(wpose, waypoints) if number == "4" else 
                                                     (espol     (wpose, waypoints) if number == "5" else (waypoints, wpose, "none", "_none")   )))))
        
        waypoints = (plane_rotation(waypoints) if theta != 0 else waypoints)
        
        print_plan(waypoints, figure)
        data_writing_publisher.publish(figure_message + "_t" + str(theta) + "_h" + str(y_h*100).split('.')[0] + "_p" + str((pen)*100).split('.')[0])
        rospy.sleep(1)
        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = group.compute_cartesian_path(waypoints, t, 0.0)  # jump_threshold

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Note: We are just planning, not asking move_group to actually move the robot yet:

        # Publish
        display_trajectory_publisher.publish(display_trajectory)
        
        group.execute(plan, wait=True)
        rospy.loginfo("Planning succesfully executed.\n")
        wpose = home()
        rospy.sleep(1)
        data_writing_publisher.publish("_none")

    else:
        quit = 1
        rospy.loginfo("Shutting down the program.\n")

    rospy.sleep(1)
