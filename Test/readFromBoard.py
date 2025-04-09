#!/usr/bin/python
# simple script to obtain data over usart from stm32 board (for Linux)

import time
import sys
import serial
import os

DEV="/dev/ttyACM0"  # update to where your board mounted
BAUDRATE=115200

try:
  ser = serial.Serial(port=DEV, 
                      baudrate=BAUDRATE, 
                      parity=serial.PARITY_NONE, 
                      stopbits=serial.STOPBITS_ONE, 
                      bytesize=serial.EIGHTBITS)
except serial.serialutil.SerialException as se:
  devs = os.popen('ls /dev/*ACM*').read() 
  print("\n\tSpecified device [%s] does not exist, possible options are:   [%s]\n\n" % (DEV, devs.rstrip()))
  exit()
ser.writeTimeout = 0 
ser.isOpen() 


print("\n-- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- ")
while True:
  byteStr=ser.readline()
  try:
    decodedStr=byteStr.decode('utf-8')
    print(decodedStr, end="", flush=True)
  except UnicodeDecodeError as ude:
    print("  UnicodeDecodeError: possible board reset, ignoring...\n")


