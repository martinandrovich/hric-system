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

echo -e  "\n\e[104mRT kernel setup script (v.1.1.0)\e[49m\n"

if [ -z "$1" ] || [ -z "$2" ]; then
	kernel_url="https://mirrors.edge.kernel.org/pub/linux/kernel/projects/rt/5.4/older/patch-5.4.10-rt4.patch.gz"
	patch_url="https://mirrors.edge.kernel.org/pub/linux/kernel/v5.x/linux-5.4.10.tar.gz"
	echo -e "No arguments provided; using default kernel URLs.\n"
else
	kernel_ver=$1
	rt_ver=$2
fi

echo -e "Using following kernel URLs:\nkernel: $kernel_url\npatch:  $patch_url\n"

read -p "Configure the RT kernel? [Y/n] " -n 1 -r
echo
if [[ $REPLY =~ ^[Nn]$ ]]; then exit; fi

# -------------------------------------------------------------------------------------------------------

# > packages

echo -e  "\n\e[104mInstalling essential packages...\e[49m\n"

pkg_list=( 
	build-essential
	curl
	git
	vim
	libncurses-dev
	libssl-dev
	flex
	bison
	cpufrequtils
)

echo -e "\nInstalling packages...\n"
sudo apt update
sudo apt install -y "${pkg_list[@]}"

# -------------------------------------------------------------------------------------------------------

# > kernel download + patch

# download kernel and patch
kernel_dir=~/Desktop/kernel/

mkdir -p "$kernel_dir"
cd "$kernel_dir"

curl -SLO "$kernel_url" -SLO "$patch_url"

# extract kernel
tar xvzf linux-*.tar.gz

# patch kernel
cd "$kernel_dir"/linux-*/
gzip -cd ../patch-*.patch.gz | patch -p1 --verbose

# -------------------------------------------------------------------------------------------------------

# > kernel configuration

# configure

echo -e "\n\e[41mIMPORTANT!\e[49m\n"
echo -e "Configure the kernel installation to be fully pre-emptive!\n"
echo -e "Find 'Preemption Model' and set to 'Fully Preemptible Kernel (RT)'\n"
echo -e "\nMore info: \e[46mhttps://hungpham2511.github.io/setup/install-rtlinux/\e[49m\n"

read -p "Press [Enter] key to start config..."

make menuconfig

# -------------------------------------------------------------------------------------------------------

# > kernel compilation + install

echo -e  "\n\e[104mCompiling kernel...\e[49m\n"

# compile
make -j20
sudo make modules_install -j20

# install
echo -e  "\n\e[104mInstalling kernel...\e[49m\n"
sudo make install -j20

# update grub
sudo update-grub

# -------------------------------------------------------------------------------------------------------

# > configure performance settings

# add realtime user group

echo -e  "\n\e[104mConfiguring real-time settings...\e[49m\n"

sudo addgroup realtime
sudo usermod -a -G realtime $USER

rt_conf_file="/etc/security/limits.conf"

if ! sudo grep -q "@realtime soft rtprio 99" "$rt_conf_file"; then
	sudo echo "@realtime soft rtprio 99" | sudo tee -a "$rt_conf_file" > /dev/null
	sudo echo "@realtime soft priority 99" | sudo tee -a "$rt_conf_file" > /dev/null
	sudo echo "@realtime soft memlock 102400" | sudo tee -a "$rt_conf_file" > /dev/null
	sudo echo "@realtime hard rtprio 99" | sudo tee -a "$rt_conf_file" > /dev/null
	sudo echo "@realtime hard priority 99" | sudo tee -a "$rt_conf_file" > /dev/null
	sudo echo "@realtime hard memlock 102400" | sudo tee -a "$rt_conf_file" > /dev/null
fi

# configure CPU performance settings

echo -e  "\n\e[104mSetting performance parameters (realtime CPU)...\e[49m\n"

sudo systemctl disable ondemand
sudo systemctl enable cpufrequtils
sudo sh -c 'echo "GOVERNOR=performance" > /etc/default/cpufrequtils'
sudo systemctl daemon-reload && sudo systemctl restart cpufrequtils

# disable suspend, sleep etc.

echo -e  "\n\e[104mDisabling system suspension...\e[49m\n"

sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
sudo xset s off
sudo xset s noblank
sudo xset -dpms

# -------------------------------------------------------------------------------------------------------

# > finish

# clean-up
sudo apt autoclean
sudo apt autoremove
cd && rm -rf kernel_dir

# restart
read -p "Press [Enter] key to reboot..."
sudo reboot
