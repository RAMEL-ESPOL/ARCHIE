# ROBOTIC ARM - A.R.C.H.I.E
<h1 style="border:none"> RISE ARCHIE ROS Manipulation Package - Educational robot for ESPOL</h1>
&copy; 2024, Erick Mendoza - Juan Saeteros - Martín Mendoza - Francisco Yumbla

<hr>

## 1. How to Install

### 1.1. System Requirements

This package is written an tested on **Ubuntu 20.04 + ROS Noetic** environment. Dependencies are also for this environment.

### 1.2. Dependencies Prerequisites

There are a number of dependencies in this package, since the ARCHIE robot is operated by ROS-Industrial package. Please install all the packages listed below in your Ubuntu PC, in the given order. These packages can be installed by `apt` package manager.

* ros-noetic-desktop-full
* ros-noetic-industrial-core
* ros-noetic-industrial-msgs
* ros-noetic-industrial-robot-client
* ros-noetic-industrial-robot-simulator
* ros-noetic-industrial-utils
* ros-noetic-moveit
* ros-noetic-joint-state-publisheser-gui
* ros-noetic-joint-trajectory-controller

#### 1.2.1 Source your project (this is for those who are ROS starters)

Now, extract the metapackage `robotic_arm` (this repository) into `~/{your_workspace_name}/src`. In this example `catkin_make` will be used as the workspace name. The step-to-step can be found at the official [ROS page](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

To source your workspace write the following command in terminal, if you don't do it your terminal won't find the workspace's packages:
```
source devel/setup.bash
```
It is necessary to write the command at every terminal your going to use, if you don't want to write it every time make the following steps:
```
gedit ~/.bashrc
```

A text editor will open, go to the last line and write the next:
```
source ~/catkin_ws/devel/setup.bash
```
### 1.3 MoveIt!

To make the FK and IK this project uses [MoveIt!](https://moveit.ai/), it is not included at the full ROS installation, so you can install it with [binary install](https://moveit.ai/install/):
```
sudo apt install ros-noetic-moveit 
```
### 1.4 Write at inclined plane

First you need the pip tool for the next installations, first [pip](https://linuxize.com/post/how-to-install-pip-on-ubuntu-20.04/):
```
sudo pip install -U numpy
```
```
sudo pip install spatialmath-rospy
```
```
sudo pip install spatialmath-python
```

These projects belong respectively to [SpatialMath Rospy](https://pypi.org/project/spatialmath-rospy/) and [SpatialMath Python](https://pypi.org/project/spatialmath-python/).

#### 1.5 Gripper and PWM mode (recommended if you already understand ROS)
To use the gripper for ARCHIE you need to install the next library:
```
sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers
```

If you are going to work at the PWM mode:
```
sudo apt-get install ros-noetic-kdl-parser-py
```
```
pip install pybind11
```

You need to install the [orocos_kinematics_dynamics](https://github.com/orocos/orocos_kinematics_dynamics) repository in your workspace `$catkin_ws/src`. The installation instructions can be found in the orocos repository.



## 2. Structure of Packages

* **archie_description:** This package contains the URDF and XACRO files for diferents configuration of the robot.
* **archie_master:** This pasckage contains a diferrents examples of motion used MoveIt of the real robot.
* **archie_moveit:** This package contains the diferent MoveIt configuration of diferents configuration of the robot
* **communication:** This package contains the executable to set up the communication between the robot and the U2D2 controller.

## 3. How to Use

### 3.1. Simulation

#### 3.1.1 Rviz Visualization

1. Launch the robot visualization Rviz
   ```
   roslaunch archie_description display.launch
   ```

2. Launch the robot with Moveit configuration
   ```
   roslaunch archie_moveit demo.launch
   ```

#### 3.1.2 MoveIt and Rviz Visualization

1. Launch the robot visualization with MoveIt and the move_arm_node
   ```
   roslaunch archie_moveit move_arm_rviz.launch
   ```

2. Run the executable python file to watch ARCHIE moving, for example:
   ```
   roslaunch archie_master write_word.launch
   ```
2.1. You could also run:
   ```
   roslaunch archie_master plan.launch
   ```

#### 3.1.3 Gazebo Simulation

1. Launch the robot in gazebo
   ```
   roslaunch archie_gazebo archie_gazebo.launch

   ```

### 3.4. Real Robot

The real robot work with the communication package to communicate with the controller and the archie_moveit package for precaution collision in our workspace or environment

Setup the Robot, turn on and connect the USB cable to your computer. 

1. Give the permissions to the terminal to use the USB port every time we launch a new terminal
   ```
   sudo chmod 666 /dev/ttyUSB0
   ```

2. Launch the robot communication
   ```
   roslaunch communication communication.launch
   ```

3. In another terminal launch MoveIt with Rviz:

   ```
   roslaunch archie_moveit move_arm_rviz.launch
   ```

4. Run the executable python file we would like to use, for example:
   ```
   roslaunch archie_master write_word.launch
   ```


