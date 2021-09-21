# -*- coding: utf-8 -*-

import serial
import time
import datetime
import os

fname = os.path.join("/home/pi/Desktop","datalogger.log")
sensor_path = "/dev/ttyAMA0"
sensor_bdrt = 57600
sensor = serial.Serial(sensor_path, sensor_bdrt, timeout = 2)
sensor.close()
sensor.open()

log_file = open(fname, "a")
log_file.write("#"+str(datetime.datetime.today())+"\n\r")
log_file.write("#Timestamp[ms];Up-Time;UTC;Date;RMC Valid;Sats in use;Latitude;Longitude;Speed over ground[knots];Speed over ground[km/h];Course over ground;Altitude NN [m];Board Temp[°C];External Temp[°C];Humidity[%];Pressure[hPa];Battery Voltage[V];Logger status\n\r")
log_file.close()

sensor.write(str.encode('T'))
sensor.write(str.encode('i'))
event = sensor.readline().decode()
if event:
    #print(event)
    print("Data Logger ready and working")
else :
    print("Data Logger not working")
    exit()
    
while True:
    log_file = open(fname, "a")
    timestamp = str(time.time())
    sensor.write(str.encode('T'))
    sensor.write(str.encode('i'))
    event = sensor.readline().decode()
    event = event[3:]
    #print(timestamp + ';' + event)
    log_file.write(timestamp + ';' + event)
    log_file.close()
    time.sleep(2)