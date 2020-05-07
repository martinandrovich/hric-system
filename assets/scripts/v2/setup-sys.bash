#!/bin/bash

# > system configuration script

# version:       1.0.0
# last modified: 07/05/2020

# -------------------------------------------------------------------------------------------------------

# > sudo test

if [ "$EUID" -eq 0 ]
  then echo "This script should NOT be run as root; run as current user and only enter password when asked."
  exit
fi

# -------------------------------------------------------------------------------------------------------

# > information
echo -e  "\n\e[104mSystem setup script [v1.0.0]\e[49m\n"

read -p "Configure the system and install essential packages? [Y/n] " -n 1 -r
echo
if [[ $REPLY =~ ^[Nn]$ ]]; then exit; fi

# -------------------------------------------------------------------------------------------------------

# > packages

pkg_list=( 
	build-essential
	cmake
	git
	vim
	net-tools
	htop
	cpufrequtils
	xclip
)

echo -e "\nInstalling packages...\n"
sudo apt update
sudo apt install -y "${pkg_list[@]}"

# etc.

# visual studio code
sudo snap install code --classic

# system update
echo -e "\nUpdating system...\n"
sudo apt update
sudo apt upgrade -y

# -------------------------------------------------------------------------------------------------------

# > GNOME configuration (GNOME)

# https://askubuntu.com/questions/971067/how-can-i-script-the-settings-made-by-gnome-tweak-tool

# install gnome-tweaks
echo -e "\nInstalling GNOME tweaks + extensions...\n"
sudo apt install gnome-tweaks -y
sudo apt install gnome-shell-extensions -y

# restart gnome-shell (can only be done once)
killall -3 gnome-shell && sleep 2
echo GNOME shell has been restarted.

# install themes
echo -e "\nInstalling theme and icons...\n"
sudo apt install arc-theme -y

# install icons
git clone https://github.com/daniruiz/flat-remix
mkdir -p ~/.icons && cp -r flat-remix/Flat-Remix* ~/.icons/
rm -rf flat-remix/

# apply configurations
echo -e "\nConfiguring system...\n"
#dconf write /org/gnome/desktop/interface/cursor-theme "'DMZ-Black'"

gsettings set org.gnome.shell enabled-extensions "['user-theme@gnome-shell-extensions.gcampax.github.com', 'alternate-tab@gnome-shell-extensions.gcampax.github.com', 'drive-menu@gnome-shell-extensions.gcampax.github.com', 'workspace-indicator@gnome-shell-extensions.gcampax.github.com']"

gsettings set org.gnome.shell.extensions.user-theme name 'Arc'
gsettings set org.gnome.desktop.interface icon-theme "Flat-Remix-Blue"
gsettings set org.gnome.desktop.interface gtk-theme 'Arc-Dark'
gsettings set org.gnome.desktop.interface cursor-theme 'DMZ-Black'

gsettings set org.gnome.shell.extensions.dash-to-dock dash-max-icon-size 32
gsettings set org.gnome.shell.extensions.dash-to-dock dock-position BOTTOM

gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-timeout 3600
gsettings set org.gnome.settings-daemon.plugins.power power-button-action 'suspend'
gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type 'nothing'

# -------------------------------------------------------------------------------------------------------

# > GitHub & SSH

# https://help.github.com/en/github/authenticating-to-github/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent
# https://help.github.com/en/github/authenticating-to-github/adding-a-new-ssh-key-to-your-github-account
# https://stackoverflow.com/questions/1885525/how-do-i-prompt-a-user-for-confirmation-in-bash-script


email="martinandrovich@gmail.com."
name="Martin Androvich"

echo -e "\nConfiguring git user credentials...\n"
git config --global user.email $email
git config --global user.name $name

read -p "Setup SSH key? [Y/n] " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]] || [[ -z $REPLY ]]; then

	echo -e "\nGenerating SSH key...\n"
	ssh-keygen -t rsa -b 4096 -C $email
	eval "$(ssh-agent -s)"
	ssh-add ~/.ssh/id_rsa
	xclip -sel clip < ~/.ssh/id_rsa.pub
	echo "The SSH key has been copied to the clipboard."

fi
