#!/bin/bash

read -p "Do you want to install ROS [Y/N]: " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]
then
	# installing ROS -> https://linuxopsys.com/topics/install-ros-noetic-on-ubuntu
	echo "deb http://packages.ros.org/ros/ubuntu focal main" | sudo tee /etc/apt/sources.list.d/ros-focal.list
	sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
	sudo apt update
	sudo apt install ros-noetic-desktop-full
	source /opt/ros/noetic/setup.bash
	echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
	source ~/.bashrc
fi

echo

read -p "Do you want to install software dependencies catkin_ws [Y/N]: " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]
then
	# install deps
	pip3 install adafruit-blinka
	sudo pip3 install adafruit-circuitpython-pca9685
	cd catkin_ws
	./build_catkin_folder.sh
	echo "source /home/surf/Jelly_ROS_22-23/catkin_ws/devel/setup.bash" >> /home/surf/.bashrc
	source ~/.bashrc
	cd /home/surf/Jelly_ROS_22-23/
fi

echo

read -p "Do you want to install OLED dependencies [Y/N]: " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]
then
	cd /home/surf/Jelly_ROS_22-23/OLED
	./install_oled_ip_script.sh
	cd /home/surf/Jelly_ROS_22-23/
fi

#NOTE: to run, you must use code with the flag --no-sandbox
#I sugest you create the following alias in ~/.bashrc
#	alias code = "code --no-sandbox"
#Also, to get the desktop icon to work, edit /usr/share/applications/code.desktop
#	change the EXEC line to include the --no-sandbox flag:
#	Exec=/usr/share/code/code --no-sandbox --unity-launch %F
echo
read -p "Do you want to install VScode [Y/N]: " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]
then
	cd /home/surf/Jelly_ROS_22-23/
	sudo ./installVSCode.sh
	echo "alias code=\"code --no-sandbox\"" >> /home/surf/.bashrc
	source ~/.bashrc
	echo -e "\n\nTo get the desktop icon to work, edit /usr/share/applications/code.desktop\n
	change the EXEC line to include the --no-sandbox flag:\n
	Exec=/usr/share/code/code --no-sandbox --unity-launch %F\n"
fi

echo


