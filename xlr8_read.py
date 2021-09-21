import serial
import time
import datetime
import os

fname = os.path.join("/home/pi/Desktop","data_xlr8.log")
xlr8_path = "/dev/tty_XLR8_balloon"
xlr8_bdrt = 115200
xlr8 = serial.Serial(xlr8_path, xlr8_bdrt)
xlr8.close()
xlr8.open()

log_file = open(fname, "a")
log_file.write("#"+str(datetime.datetime.today())+"\n\r")
log_file.write("#Timestamp[ms]\tReset time[s]\tTRIGGER\tIN0\tIN1\tIN2\tIN3\tIN4\tIN5\tIN6\tIN7\tAND0\tAND1\tAND2\n\r")
log_file.close()

if xlr8.readline().decode():
    print("XLR8 balloon ready and working")
else :
    print("XLR8 balloon not working")
    exit()

xlr8.write(str.encode('r'))
i=1
dict = { 62:None , 92:None , 110:None , 114:None }

while True:
    try:    
        log_file = open(fname, "a")
        timestamp= str(time.time())
        #xlr8.write(str.encode('K'))
        event = xlr8.readline().decode()
        #event = event[2:]
        if i>=5 : #elimino le righe di reset 
            log_file.write(timestamp + ' ' + event.translate(dict))
            log_file.close()
            #print(timestamp + ' ' + event.translate(dict))
        i=i+1
    except:
        log_file = open(fname, "a")
        timestamp = str(time.time())
        #print('#' + timestamp)
        log_file.write('#' + timestamp + '\n')
        log_file.close()
        time.sleep(10)
        try:
            xlr8 = serial.Serial(xlr8_path, xlr8_bdrt)
            xlr8.close()
            xlr8.open()
        except:
            continue
            
