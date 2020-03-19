#!/bin/bash

# sudo test
if [ "$EUID" -eq 0 ]
  then echo "This script should NOT be run as root; run as current user and only enter password when asked."
  exit
fi

# information
echo -e  "\n\e[104mROS-melodic setup script [v1.0.2]\e[49m\n"
read -p "Press [Enter] key to start..."

# essential packages

echo -e  "\n\e[104mInstalling essential packages...\e[49m\n"

pkg_list=( 
           build-essential cmake git vim
           net-tools htop cpufrequtils
           libpoco-dev libeigen3-dev libboost-filesystem-dev
           python-rosinstall python-rosinstall-generator python-wstool
         )

sudo apt-get update
sudo apt-get install -y "${pkg_list[@]}"

# ROS

echo -e  "\n\e[104mInstalling ROS...\e[49m\n"

# accept software from packages.ros.org
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# add keys
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# update and install ROS
sudo apt update
sudo apt install ros-melodic-desktop-full -y

# fix anything missing
sudo apt --fix-broken install -y

# install and init ROS dep
sudo apt install python-rosdep -y
sudo rosdep init
rosdep update

# environment setup
grep -qxF 'source /opt/ros/melodic/setup.bash' ~/.bashrc || echo 'source /opt/ros/melodic/setup.bash' >> ~/.bashrc
sudo updatedb
