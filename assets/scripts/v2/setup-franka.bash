#!/bin/bash

# sudo test
if [ "$EUID" -eq 0 ]
  then echo "This script should NOT be run as root; run as current user and only enter password when asked."
  exit
fi

# information
echo -e  "\n\e[104mROS-melodic setup script [v1.0.2]\e[49m\n"
read -p "Press [Enter] key to start..."

# install libfranka + franka_ros (melodic)
sudo apt install ros-$ROS_DISTRO-libfranka -y
sudo apt install ros-$ROS_DISTRO-franka-ros -y
