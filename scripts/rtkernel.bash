#!/bin/bash

echo -e  "\n\e[104mRT kernel setup script (v.1.0.1)\e[49m\n"

if [ -z "$1" ] || [ -z "$2" ]; then
	kernel_url="https://mirrors.edge.kernel.org/pub/linux/kernel/projects/rt/5.4/older/patch-5.4.10-rt4.patch.gz"
	patch_url="https://mirrors.edge.kernel.org/pub/linux/kernel/v5.x/linux-5.4.10.tar.gz"
	echo -e "No arguments provided; using default kernel URLs.\n"
else
	kernel_ver=$1
	rt_ver=$2
fi

echo -e "Using following kernel URLs:\nkernel: $kernel_url\npatch:  $patch_url\n"
read -p "Press [Enter] key to continue..."

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

# install dependencies

echo -e  "\n\e[104mInstalling essential packages...\e[49m\n"

sudo apt-get update
sudo apt-get install libncurses-dev libssl-dev -y
sudo apt-get install flex bison -y

# configure

echo -e "\n\e[41mIMPORTANT!\e[49m\n"
echo -e "Configure the kernel installation to be fully pre-emptive!"
echo -e "Read here: \e[46mhttps://hungpham2511.github.io/setup/install-rtlinux/\e[49m\n"

read -p "Press [Enter] key to start config..."

make menuconfig

# compile

echo -e  "\n\e[104mCompiling kernel...\e[49m\n"

make -j20
sudo make modules_install -j20

# install

echo -e  "\n\e[104mInstalling kernel...\e[49m\n"
sudo make install -j20

# update grub

sudo update-grub

# configure real-time settings

echo -e  "\n\e[104mConfiguring real-time settings...\e[49m\n"

sudo addgroup realtime
sudo usermod -a -G realtime $USER

rt_conf_file="/etc/security/limits.conf"

if ! sudo grep -q "@realtime soft rtprio 99" "$rt_conf_file"; then
	echo "@realtime soft rtprio 99" | sudo tee -a "$rt_conf_file" > /dev/null
	sudo echo "@realtime soft priority 99" | sudo tee -a "$rt_conf_file" > /dev/null
	sudo echo "@realtime soft memlock 102400" | sudo tee -a "$rt_conf_file" > /dev/null
	sudo echo "@realtime hard rtprio 99" | sudo tee -a "$rt_conf_file" > /dev/null
	sudo echo "@realtime hard priority 99" | sudo tee -a "$rt_conf_file" > /dev/null
	sudo echo "@realtime hard memlock 102400" | sudo tee -a "$rt_conf_file" > /dev/null
fi

# restart

read -p "Press [Enter] key to reboot..."
sudo reboot

