import serial
import time
import os

device_path = "/dev/tty/USB0"
baudrate = 115200

xlr8 = serial.Serial(device_path, baudrate)

fname = "data.log"

log_file = open(fname, "a")
  
event = "ciao"

while True:
    event = xlr8.readline().decode()
    log_file.write(event)
    
 

  
