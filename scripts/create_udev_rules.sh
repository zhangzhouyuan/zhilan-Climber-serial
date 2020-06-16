#!/bin/bash

echo "remap the device serial port(ttyUSBX) to  chassis"
echo "chassis usb cp210x connection as /dev/chassis , check it using the command : ls -l /dev|grep ttyUSB"
echo "start copy chassis.rules to  /etc/udev/rules.d/"
echo "`rospack find chassis_serial`/scripts/chassis.rules"
sudo cp `rospack find chassis_serial`/scripts/chassis.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish "
