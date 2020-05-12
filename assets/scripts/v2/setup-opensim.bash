#!/bin/bash

# > opensim installation script

# version:       1.0.0
# last modified: 08/05/2020

# -------------------------------------------------------------------------------------------------------

# > sudo test

if [ "$EUID" -eq 0 ]
  then echo "This script should NOT be run as root; run as current user and only enter password when asked."
  exit
fi

# -------------------------------------------------------------------------------------------------------

# > information
echo -e  "\n\e[104mOpenSim setup script [v1.0.0]\e[49m\n"

read -p "Install OpenSim on this system? [Y/n] " -n 1 -r
echo
if [[ $REPLY =~ ^[Nn]$ ]]; then exit; fi

# -------------------------------------------------------------------------------------------------------

# directories
dir_name="opensim"
dir_source=~/$dir_name-source
dir_build=~/$dir_name-build
dir_install=~/$dir_name
dir_dep_source=$dir_source/dependencies
dir_dep_build=~/$dir_name-dependencies-build
dir_dep_install=~/$dir_name-dependencies-install

# varibales
build_type="RelWithDebInfo"
java_wrapping=false
python_wrapping=false
python_version=3

# download OpenSim
echo -e "\nDownloading OpenSim source...\n"
git clone https://github.com/opensim-org/opensim-core.git $dir_source

# build dependecies
# flags_dep is an array with all options
echo -e "\nBuilding OpenSim dependecies...\n"

flags_dep=(
	-DCMAKE_INSTALL_PREFIX=$dir_dep_install
	-DCMAKE_BUILD_TYPE=$build_type
)

mkdir -p $dir_dep_build && cd $dir_dep_build
cmake $dir_dep_source "${flags_dep[@]}"
make -j4

# build OpenSim
echo -e "\nBuilding OpenSim...\n"

flags_opensim=(
	-DCMAKE_INSTALL_PREFIX=$dir_install
	-DOPENSIM_DEPENDENCIES_DIR=$dir_dep_install
	-DCMAKE_BUILD_TYPE=$build_type
	-DBUILD_TESTING=false
	-DOPENSIM_COPY_DEPENDENCIES=true
	-DBUILD_JAVA_WRAPPING=$java_wrapping
	-DBUILD_PYTHON_WRAPPING=$python_wrapping
	-DOPENSIM_PYTHON_VERSION=$python_version
)

mkdir -p $dir_build && cd $dir_build
cmake $dir_source "${flags_opensim[@]}"
make -j4

# install OpenSim
echo -e "\nInstalling OpenSim...\n"
make install

# clean-up
# rm -rf dir_build
# rm -rf dir_source
# rm -rf dir_dep_build
# rm -rf dir_dep_install