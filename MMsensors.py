
import serial
import time
import os

sensors_path = "/dev/tty_Arduino"
sensors_bdrt = 115200

sensors = serial.Serial(sensors_path, sensors_bdrt)

fname = "data.log"

log_file = open(fname, "a")
  
event = "ciao"

while True:
    event = sensors.readline().decode()
    log_file.write(event)
    
