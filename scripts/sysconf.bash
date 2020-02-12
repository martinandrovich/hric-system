#!/bin/bash

# sudo test

if [ "$EUID" -eq 0 ]
  then echo "This script should NOT be run as root; run as current user and only enter password when asked."
  exit
fi

echo -e  "\n\e[104mFRANKA EMIKA system configuration script (v.1.0.8)\e[49m\n"
read -p "Press [Enter] key to start config..."

# disable suspend, sleep etc.

echo -e  "\n\e[104mDisabling system suspension...\e[49m\n"

sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
sudo xset s off
sudo xset s noblank
sudo xset -dpms

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

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt-get update
sudo apt-get install ros-melodic-desktop-full -y
sudo apt --fix-broken install -y

sudo rosdep init
rosdep update

echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# libfranka

echo -e  "\n\e[104mInstalling libfranka...\e[49m\n"

libdir=~/Desktop/libfranka/

sudo rm -rf "$libdir"
git clone --recursive https://github.com/frankaemika/libfranka "$libdir"
mkdir "$libdir"/build/
cd "$libdir"/build/

cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . --config Release -- -j99

# franka ROS

echo -e  "\n\e[104mInstalling franka_ros...\e[49m\n"

## create catkin workspace

wsdir=~/Desktop/catkin_ws/

sudo rm -rf "$wsdir"
mkdir -p "$wsdir"/src
source /opt/ros/melodic/setup.bash
catkin_init_workspace "$wsdir"/src

## setup franka_ros
## with Erdal's franka descriptors (PID etc.)

git clone https://github.com/erdalpekel/panda_simulation.git "$wsdir"/src/panda_simulation/
git clone https://github.com/erdalpekel/panda_moveit_config.git "$wsdir"/src/panda_moveit_config/
git clone --branch simulation https://github.com/erdalpekel/franka_ros.git "$wsdir"/src/franka_ros/

## install ROS dependencies
cd "$wsdir"
#rosdep install --from-paths src --ignore-src -y --skip-keys libfranka
rosdep install --from-paths src --ignore-src --rosdistro melodic -y --skip-keys libfranka

## build catkin workspace
catkin_make -j99 -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH="$libdir"/build/

## source catkin
echo "source ${wsdir}/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

# remove bloatware

echo -e  "\n\e[104mRemoving bloatware...\e[49m\n"

#sudo apt-get remove account-plugin-facebook account-plugin-flickr account-plugin-jabber account-plugin-salut account-plugin-twitter account-plugin-windows-live account-plugin-yahoo aisleriot brltty duplicity empathy empathy-common example-content gnome-accessibility-themes gnome-contacts gnome-mahjongg gnome-mines gnome-orca gnome-screensaver gnome-sudoku gnome-video-effects gnomine landscape-common libreoffice-avmedia-backend-gstreamer libreoffice-base-core libreoffice-calc libreoffice-common libreoffice-core libreoffice-draw libreoffice-gnome libreoffice-gtk libreoffice-impress libreoffice-math libreoffice-ogltrans libreoffice-pdfimport libreoffice-style-galaxy libreoffice-style-human libreoffice-writer libsane libsane-common mcp-account-manager-uoa python3-uno rhythmbox rhythmbox-plugins rhythmbox-plugin-zeitgeist sane-utils shotwell shotwell-common telepathy-gabble telepathy-haze telepathy-idle telepathy-indicator telepathy-logger telepathy-mission-control-5 telepathy-salut totem totem-common totem-plugins printer-driver-brlaser printer-driver-foo2zjs printer-driver-foo2zjs-common printer-driver-m2300w printer-driver-ptouch printer-driver-splix -y
sudo apt purge snapd -y
sudo apt-get remove --purge libreoffice* -y
sudo apt-get purge thunderbird* -y

# disable uneccessary daemons

## TODO

# configure performance settings

echo -e  "\n\e[104mSetting performance parameters (realtime CPU)...\e[49m\n"

sudo systemctl disable ondemand
sudo systemctl enable cpufrequtils
sudo sh -c 'echo "GOVERNOR=performance" > /etc/default/cpufrequtils'
sudo systemctl daemon-reload && sudo systemctl restart cpufrequtils

# configure network

$ echo -e  "\n\e[104mConfiguring network...\e[49m\n"

# clean-up

echo -e  "\n\e[104mCleaning up...\e[49m\n"

sudo apt --fix-broken install -y
sudo apt autoclean -y
sudo apt autoremove -y
