#!/bin/bash

echo "delete remap the device serial port(ttyUSBX) to chassis"
echo "sudo rm /etc/udev/rules.d/chassis.rules"
sudo rm /etc/udev/rules.d/chassis.rules
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish delete"
