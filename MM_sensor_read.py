# -*- coding: utf-8 -*-

import serial
import time
import datetime
import os

fname = os.path.join("/home/pi/Desktop","data_MMsensor.log")
sensor_path = "/dev/tty_MM_sensor" #"/dev/tty_Arduino" per il sensore vecchio
sensor_bdrt = 115200
sensor = serial.Serial(sensor_path, sensor_bdrt)
sensor.close()
sensor.open()

log_file = open(fname, "a")
log_file.write("#"+str(datetime.datetime.today())+"\n\r")
log_file.write("#Timestamp[ms]\tPressure[mbar]\tTemp[°C]\tHumidity[%]\n\r")
log_file.close()

dict = { 109:None , 98:None , 97:None , 114:None , 176:None , 67:None , 37:None }

if sensor.readline().decode("utf8"):
    print("MM sensors ready and working")
else:
    print("MM sensors not working")
    exit()
    
while True:
    try:
        log_file = open(fname, "a")
        timestamp = str(time.time())
        event = sensor.readline().decode("utf8")
        #print(timestamp + ', ' + event.translate(dict))
        log_file.write(timestamp + ', ' + event.translate(dict))
        log_file.close()
        time.sleep(60)
    except:
        log_file = open(fname, "a")
        timestamp = str(time.time())
        #print('#' + timestamp)
        log_file.write('#' + timestamp + '\n')
        log_file.close()
        time.sleep(60)
        try:
            sensor = serial.Serial(sensor_path, sensor_bdrt)
            sensor.close()
            sensor.open()
        except:
            continue

        

