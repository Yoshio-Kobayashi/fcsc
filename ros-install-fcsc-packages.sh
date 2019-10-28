#!/bin/sh

sudo apt-get install ros-${ROS_DISTRO}-timed-roslaunch -y

# moveit
sudo apt-get install ros-${ROS_DISTRO}-moveit* -y

# smach
sudo apt-get install ros-${ROS_DISTRO}-smach* -y

# ik solver
sudo apt-get install ros-${ROS_DISTRO}-trac-ik* -y

# controller
sudo apt-get install ros-${ROS_DISTRO}-ros-control ros-${ROS_DISTRO}-ros-controllers ros-${ROS_DISTRO}-joint-state-controller ros-${ROS_DISTRO}-effort-controllers ros-${ROS_DISTRO}-position-controllers ros-${ROS_DISTRO}-joint-trajectory-controller -y

#gazebo
sudo apt-get install ros-${ROS_DISTRO}-gazebo* -y

#ros-control
sudo apt-get install ros-${ROS_DISTRO}-ros-control ros-${ROS_DISTRO}-ros-controllers -y

# EZgripper
sudo apt-get install git python-pip python-serial -y
sudo pip install git+https://github.com/SAKErobotics/libezgripper.git#egg=libezgripper
sudo apt-get install python-qt4

# ar-track-alvar
sudo apt-get install ros-${ROS_DISTRO}-ar-track-alvar -y

#openni
sudo apt-get install ros-${ROS_DISTRO}-openni* -y

#serial
sudo apt-get install ros-${ROS_DISTRO}-serial -y

# ur_modern_driver
sudo apt-get install ros-${ROS_DISTRO}-hardware-interface ros-${ROS_DISTRO}-controller-manager  ros-${ROS_DISTRO}-industrial-msgs -y

# navigation
sudo apt-get install ros-${ROS_DISTRO}-pointcloud-to-laserscan ros-${ROS_DISTRO}-move-base* ros-${ROS_DISTRO}-costmap-2d ros-${ROS_DISTRO}-amcl -y
sudo apt-get install ros-${ROS_DISTRO}-kobuki* ros-${ROS_DISTRO}-gmapping ros-${ROS_DISTRO}-navigation ros-${ROS_DISTRO}-dwa-local-planner -y
sudo apt-get install ros-${ROS_DISTRO}-eband-local-planner -y
