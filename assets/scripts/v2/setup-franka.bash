#!/bin/bash

# > RT kernel setup script

# version:       1.1.0
# last modified: 18/05/2020

# -------------------------------------------------------------------------------------------------------

# > sudo test

if [ "$EUID" -eq 0 ]
  then echo "This script should NOT be run as root; run as current user and only enter password when asked."
  exit
fi

# -------------------------------------------------------------------------------------------------------

# > information

echo -e  "\n\e[104mFRANKA setup script (libfranka + franka_ros) (v.1.1.0)\e[49m\n"

# confirmation
read -p "Install libfranka and franka_ros from apt? [Y/n] " -n 1 -r
echo
if [[ $REPLY =~ ^[Nn]$ ]]; then exit; fi

# -------------------------------------------------------------------------------------------------------

# > install libfranka + franka_ros (melodic)

sudo apt install ros-$ROS_DISTRO-libfranka -y
sudo apt install ros-$ROS_DISTRO-franka-ros -y
