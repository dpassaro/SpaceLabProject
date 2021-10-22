# -*- coding: utf-8 -*-
"""
    Script per acquisire i dati dal sensore della Micromegas (INFN).
    I dati sono comunicati via USB e scritti su un file di log ogni 60 secondi;
    i dati salvati sono:
    -) timestamp;
    -) temperatura, umidità, pressione;
    Il file di log è aperto in modalità 'append': per dividere diverse acquisizioni
    dati, vengono stampate due righe informative all'inizio di ciasuna di esse.
    Lo script è pensato per far scrivere in modo continuo dal datalogger: se
    dovesse esserci un problema di connessione, viene gestita l'eccezione sollevata
    dalla seriale,stampando una riga di debug contente il timestamp; di seguito si
    tenta di ri-inizializzare la USB ogni 10 secondi.
"""

import time
import datetime
import os
import sys

import serial

def compute_timestamp(date):
    """ Calcola il tempo con precisione del microsecondo, esprimendo il timestamp
        come il numero di secondi passati dalle 00:00 del primo giorno del mese corrente
    """
    return (date.day-1)*24*3600 + date.hour*3600 + date.minute*60 + date.second \
                                                        + date.microsecond/1000000


if __name__ == "__main__":

    fname = os.path.join("home","pi","Desktop","data_MMsensor.log")
    SENSOR_PATH = "/dev/tty_MM_sensor" #"/dev/tty_Arduino" per il sensore vecchio
    SENSOR_BDRT = 115200
    sensor = serial.Serial(SENSOR_PATH, SENSOR_BDRT)
    sensor.close()
    sensor.open()

    log_file = open(fname, "a", encoding = 'utf-8')
    log_file.write("#"+str(datetime.datetime.today())+"\n\r")
    log_file.write("#Timestamp[ms]\tPressure[mbar]\tTemp[°C]\tHumidity[%]\n\r")
    log_file.close()

    mydict = {109:None , 98:None , 97:None , 114:None , 176:None , 67:None , 37:None}

    if sensor.readline().decode("utf8"):
        print("MM sensors ready and working")
    else:
        print("MM sensors not working")
        sys.exit()

    while True:
        log_file = open(fname, "a", encoding = 'utf-8')
        TIMESTAMP = str( compute_timestamp(datetime.datetime.now()) )
        try:
            event = sensor.readline().decode("utf8")
            #print(f"{timestamp}, {event.translate(dict)}")
            log_file.write(f"{TIMESTAMP}, {event.translate(mydict)}")
            log_file.close()
            time.sleep(60)
        except serial.SerialException :
            #print(f"#{timestamp}\n")
            log_file.write(f"#{TIMESTAMP}\n")
            log_file.close()
            time.sleep(10)
            try:
                sensor = serial.Serial(SENSOR_PATH, SENSOR_BDRT)
                sensor.close()
                sensor.open()
            except serial.SerialException :
                continue
