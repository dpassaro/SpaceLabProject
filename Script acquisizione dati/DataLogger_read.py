# -*- coding: utf-8 -*-
"""
    Script per acquisire i dati dal datalogger STRATO3 (www.stratoflights.com).
    I dati sono comunicati via UART e scritti su un file di log ogni 2 secondi;
    i dati salvati sono:
    -) tempo: timestamp, tempo dall'accensione del dispositivo, UTC, data locale;
    -) localizzazione GPS: validità dati, numero di satelliti in uso, latitudine,
       longitudine, velocità rispetto al suolo, direzione, altitudine;
    -) temperatura(scheda e del sensore), umidità, pressione;
    Il file di log è aperto in modalità 'append': per dividere diverse acquisizioni
    dati, vengono stampate due righe informative all'inizio di ciasuna di esse.
    Lo script è pensato per far scrivere in modo continuo dal datalogger, anche se
    ci dovesse essere un problema di connessione: essendo collegato via UART, è
    costantemente inizializzato, dunque lo script è sempre in "ascolto".
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
    return (date.day-1)*24*3600 + date.hour*3600 + date.minute*60 + date.second + \
                                                            date.microsecond/1000000


if __name__ == "__main__":

    fname = os.path.join("home","pi","Desktop","datalogger.log")
    SENSOR_PATH = "/dev/ttyAMA0"
    SENSOR_BDRT = 57600
    sensor = serial.Serial(SENSOR_PATH, SENSOR_BDRT, timeout = 2)
    sensor.close()
    sensor.open()

    log_file = open(fname, "a", encoding = 'utf-8')
    log_file.write("#"+str(datetime.datetime.today())+"\n\r")
    log_file.write("#Timestamp[ms];Up-Time;UTC;Date;RMC Valid;Sats in use;Latitude;\
    Longitude;Speed over ground[knots];Speed over ground[km/h];Course over ground;\
    Altitude NN [m];Board Temp[°C];External Temp[°C];Humidity[%];Pressure[hPa];\
    Battery Voltage[V];Logger status\n\r")
    log_file.close()

    sensor.write(str.encode('T'))
    sensor.write(str.encode('i'))
    event = sensor.readline().decode()

    if event:
        #print(event)
        print("Data Logger ready and working")
    else :
        print("Data Logger not working")
        sys.exit()

    while True:
        log_file = open(fname, "a", encoding = 'utf-8')
        TIMESTAMP = str( compute_timestamp(datetime.datetime.now()) )
        sensor.write(str.encode('T'))
        sensor.write(str.encode('i'))
        event = sensor.readline().decode()
        event = event[3:]
        #print(f"{timestamp};{event}")
        log_file.write(f"{TIMESTAMP};{event}")
        log_file.close()
        time.sleep(2)
