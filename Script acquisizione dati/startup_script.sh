#!/bin/bash

sudo killall -9 python3

XLR8=0
MM=0

echo MIRACLE project
date
echo Booting...
echo Required systems:
echo XLR8_balloon - MM_sensor - DataLogger
echo Detected devices:

if find /dev/tty_XLR8_balloon | grep -q .; then echo "XLR8_balloon FOUND"; XLR8=1; else echo "XLR8_balloon NOT FOUND"; fi
if find /dev/tty_MM_sensor | grep -q .; then echo "MM_sensor FOUND"; MM=1; else echo "MM_sensor NOT FOUND"; fi

if [[ $MM == 0 || $XLR8 == 0 ]]
then
echo "Devices not detected. Continue with the available devices? [Y/N]" ; read CHOICE;
else
echo "Devices detected. Booting...";
fi

if [[ $CHOICE == "N" ]]; then echo "Shutting down" ; exit ;fi

if [ $XLR8 == 1 ]; then python3 /home/pi/Desktop/xlr8_read.py ; fi &
if [ $MM == 1 ]; then python3 /home/pi/Desktop/MM_sensor_read.py ; fi &
sudo python3 /home/pi/Desktop/DataLogger_read.py ; &
sudo python3 /home/pi/Desktop/BerryIMU/python-BerryIMU-gyro-accel-compass-filters/berryIMU.py ; &

echo "Press S to stop logging"
read STOP
if [[ $STOP == "S" ]]; then sudo killall -9 python3; exit; fi
