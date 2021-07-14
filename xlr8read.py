import serial
import time
import os

xlr8_path = "/dev/tty_XLR8_balloon"
xlr8_bdrt = 115200

xlr8 = serial.Serial(xlr8_path, xlr8_bdrt)

fname = "data.log"

log_file = open(fname, "a")
  
event = "ciao"

while True:
    event = xlr8.readline().decode()
    log_file.write(event)
    
 

  
