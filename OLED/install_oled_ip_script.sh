#!/bin/bash

echo -e "\nInstalling oled_ip.py to start at boot\n"

sudo cp /home/surf/Jelly_ROS_22-23/OLED/pioled_stats.service /etc/systemd/system/pioled_ip.service
sudo systemctl enable --now pioled_ip

echo -e "\n\nDone\n\n"
