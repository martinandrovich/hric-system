#!/bin/bash

# config

ver=1.0.0
rosver="melodic"
wsname=hric_ws
wsdir="`pwd`/$wsname"

# intro

echo -e "\n\e[104mHRIC system setup script (v.$ver)\e[49m\n"

echo -e "\e[43mATTENTION:\e[49m"
echo -e "Please execute this script as \"\e[1msource wsconf.bash\e[0m\" if the current environment should be configured.\n"

# installation dir confirmation

echo -e "The catkin workspace will be installed at \"$wsdir\""
read -p "Press [Enter] key to begin installation ..."

# create workspace

echo -e "\n\e[104mCreating catkin workspace...\e[49m\n"

source /opt/ros/$rosver/setup.bash

rm -rf $wsdir
mkdir -p $wsdir"/src"
cd $wsdir
catkin_make

source $wsdir/devel/setup.bash
#echo $ROS_PACKAGE_PATH

# create package

echo -e "\n\e[104mCreating ROS package...\e[49m\n"

pkgname=beginner_tutorials
pkgdep=( 
		std_msgs
		rospy
		roscpp
       )
pkgdir=$wsdir/src/$pkgname

cd $wsdir/src
catkin_create_pkg $pkgname "${pkgdep[@]}"

# build

echo -e "\n\e[104mBuilding catkin packages...\e[49m\n"

cd $wsdir
catkin_make
source $wsdir/devel/setup.bash

# create launch file

echo -e "\n\e[104mCreating launch file...\e[49m\n"

launchdir=$pkgdir/launch
launchname="test.launch"

mkdir -p $launchdir
touch $launchdir/$launchname

# create custom messages

echo -e "\n\e[104mCreating custom message types...\e[49m\n"

msgdir=$pkgdir/msg/

mkdir -p $msgdir
touch $msgdir/test.msg

msgfields=( 
		"string first_name"
		"string last_name"
		"uint8 age"
		"uint32 score"
       )

printf "%s\n" "${msgfields[@]}" > $msgdir/test.msg

# added to package.xml
#<build_depend>message_generation</build_depend>
#<exec_depend>message_runtime</exec_depend>

# create nodes

nodedir=$pkgdir/src

mkdir -p $nodedir




