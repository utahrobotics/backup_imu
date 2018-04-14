#!/usr/bin/env bash

# run this to make the make the SparkFun FTDI chip be called /dev/backup_imu
sudo cp 11-backup.rules /etc/udev/rules.d/
sudo udevadm trigger

echo ""
echo "unplug and replug the device"
echo ""
echo "test with:"
echo "ls /dev/backup_imu"
echo ""

