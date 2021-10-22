# -*- coding: utf-8 -*-
"""
    Script per acquisire i dati dalla XLR8 (https://aloriumtech.com/xlr8/).
    I dati sono comunicati via USB e scritti su un file di log ogni 10 secondi;
    i dati salvati sono:
    -) timestamp e tempo in secondi dall'ultimo reset della XLR8;
    -) conteggi di "trigger" (combinazione di diversi PIN)
    -) conteggi in singola;
    -) conteggi in doppia;
    Le coincidenze doppie e il trigger vanno programmati tramite l'apposito sketch di 
    configurazione ('Sketch XLR8'\CosmocubePalloneXLR8_Fram64k_24bit) o tramite PuTTy.
    Il file di log è aperto in modalità 'append': per dividere diverse acquisizioni
    dati, vengono stampate due righe informative all'inizio di ciasuna di esse.
    Lo script è pensato per far scrivere in modo continuo dalla FPGA: se
    dovesse esserci un problema di connessione, viene gestita l'eccezione sollevata
    dalla seriale, stampando una riga di debug contente il timestamp; di seguito si
    tenta di ri-inizializzare la USB ogni 2.5 secondi.
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
    return (date.day-1)*24*3600 + date.hour*3600 + date.minute*60 + date.second +\
                                                            date.microsecond/1000000


if __name__ == "__main__":

    fname = os.path.join("home","pi","Desktop","data_xlr8.log")
    XLR8_PATH = "/dev/tty_XLR8_balloon"
    XLR8_BDRT = 115200
    xlr8 = serial.Serial(XLR8_PATH, XLR8_BDRT)
    xlr8.close()
    xlr8.open()

    log_file = open(fname, "a",encoding = 'utf-8')
    log_file.write("#"+str(datetime.datetime.today())+"\n\r")
    log_file.write("#Timestamp[ms]\tReset time[s]\tTRIGGER\tIN0\tIN1\tIN2\tIN3\tIN4\
    \tIN5\tIN6\tIN7\tAND0\tAND1\tAND2\n\r")
    log_file.close()

    if xlr8.readline().decode():
        print("XLR8 balloon ready and working")
    else :
        print("XLR8 balloon not working")
    sys.exit()

    xlr8.write(str.encode('r'))
    i=1
    mydict = { 62:None , 92:None , 110:None , 114:None }

    while True:
        log_file = open(fname, "a", encoding = 'utf-8')
        TIMESTAMP = str( compute_timestamp(datetime.datetime.now()) )
        try:
            event = xlr8.readline().decode()
            if i>=5 : #elimino le righe di reset
                log_file.write(f"{TIMESTAMP} {event.translate(mydict)}")
                log_file.close()
                #print(f"{timestamp} {event.translate(dict)}")
            i=i+1
        except serial.SerialException :
            #print(f"#{timestamp}\n")
            log_file.write(f"#{TIMESTAMP}\n")
            log_file.close()
            time.sleep(2.5)
            try:
                xlr8 = serial.Serial(XLR8_PATH, XLR8_BDRT)
                xlr8.close()
                xlr8.open()
            except serial.SerialException :
                continue
