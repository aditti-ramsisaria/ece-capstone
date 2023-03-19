import time
import serial
import pandas as pd
import csv

arduino_port = "/dev/cu.usbmodem1101" #serial port of Arduino
i = 0
baud = 115200 #arduino uno runs at 115200 baud
seonsor = serial.Serial(arduino_port, baud)
# 'COM5' is the port name that the Arduino is connected.
# '9600' is the Baudrate.
 
data = pd.DataFrame()
 
while True:
    signal = seonsor.readline()
    temp = signal.decode('utf-8').split(',')
    #signal.insert(0, time.strftime('%y-%m-%d %H:%M:%S'))
    data = data.append(pd.Series(signal), ignore_index=True)
 
    f = open('sensors.csv','a',newline='\n')
    wr=csv.writer(f)
    wr.writerow(temp)
    f.close()
 
    time.sleep(5)