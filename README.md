## youbot_manipulation

This repository contains packages for using the youBot's manipulator in ROS. It includes an analytical inverse position kinematics solver and configuration files for MoveIt!.


### Overview
The youbot_manipulation meta-package contains the following packages:
* youbot_arm_kinematics: A framework-independent library which implements an analytical inverse position kinematics solver for the youBot manipulator.
* youbot_arm_kinematics_moveit: A MoveIt! plugin which exposes the youbot_arm_kinematics library (see above) to MoveIt!.
* youbot_moveit: Configuration files to use the youBot with MoveIt! and the analytical inverse kinematics solver from this repository. These files have been auto-generated by the MoveIt! setup assistant.
* youbot_gripper: A controller manager to use the gripper with MoveIt!


### Installation
It is assumed that ROS and the youBot packages have been installed properly.
First, create a Catkin workspace (http://wiki.ros.org/catkin/Tutorials/create_a_workspace) and add the repository to this workspace by executing the following commands:

    cd <catkin_workspace>/src
    wstool set youbot_manipulation --git git@github.com:svenschneider/youbot-manipulation.git --version=hydro
    wstool update youbot_manipulation

Now build the workspace:

    cd <catkin_workspace>
    catkin_make


### Running the demo
The youbot_moveit package contains a demo to run and visualize a simulated youBot in RViz. This demo can be executed via the following command:

    roslaunch youbot_moveit demo.launch

Using the markers attached to the manipulator, the end-effector can be dragged around which triggers the inverse kinematics solver. Additionally, different motion planners can be tried out.


### Running on the real or simulated robot
First, bring up the real or simulated robot. Then, start MoveIt! via:

    roslaunch youbot_moveit move_group.launch
    
Launch the controller for the gripper

    rosrun youbot_gripper youbot_gripper_server.py

With the MoveIt! Commander (http://moveit.ros.org/wiki/MoveIt_Commander) the manipulator can be moved by typing in commands. To start the MoveIt! Commander run:

    rosrun moveit_commander moveit_commander_cmdline.py

Then select the hardware group to move and command it to some known position (e.g. the up-right, candle position):

    > use arm_1
    arm_1> go candle
    > use arm_1_gripper
    arm_1_gripper> go open

    

### Documentation

The MoveIt! kinematics plugin in the youbot_arm_kinematics_moveit package implements the standard MoveIt! interfaces. Therefore, consult the MoveIt! website (http://moveit.ros.org/) for further documentation related to using MoveIt!.
