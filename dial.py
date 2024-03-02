import serial
import os, time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
port = serial.Serial("/dev/usb", baudrate=9600, timeout=1)

port.write(b'AT<CR><LF>\r')
rcv = port.read(10)
print(rcv,"AT out")
time.sleep(1)

port.write(b'ATD0378203533;\r')
print('Calling…')
time.sleep(30)
port.write(b'TH\r')
print('Hang Call…')