#!/bin/bash

# > ros-melodic installation script

# version:       1.1.0
# last modified: 07/05/2020

# -------------------------------------------------------------------------------------------------------

# > sudo test

if [ "$EUID" -eq 0 ]
  then echo "This script should NOT be run as root; run as current user and only enter password when asked."
  exit
fi

# -------------------------------------------------------------------------------------------------------

# > information
echo -e  "\n\e[104mros-melodic setup script [v1.1.0]\e[49m\n"

read -p "Install ros-melodic this system? [Y/n] " -n 1 -r
echo
if [[ $REPLY =~ ^[Nn]$ ]]; then exit; fi

# -------------------------------------------------------------------------------------------------------

# > ros-melodic

echo -e  "\n\e[104mInstalling ros-melodic...\e[49m\n"

# accept software from packages.ros.org
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# add keys
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# update package index and install ROS
sudo apt update
sudo apt install ros-melodic-desktop-full -y

# fix anything missing
sudo apt --fix-broken install -y

# environment setup
grep -qxF 'source /opt/ros/melodic/setup.bash' ~/.bashrc || echo -e '\nsource /opt/ros/melodic/setup.bash' >> ~/.bashrc
source /opt/ros/melodic/setup.bash
sudo updatedb

# -------------------------------------------------------------------------------------------------------

# > rosdep

echo -e  "\n\e[104mInstalling rosdep...\e[49m\n"

# install and init rosdep
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential -y
sudo rosdep init
rosdep update

# clean up
sudo apt clean
sudo apt autoclean
sudo updatedb