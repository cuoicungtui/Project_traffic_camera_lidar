import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

pin = 21
GPIO.cleanup()
# GPIO.setup(pin,GPIO.OUT)

try: 
    while True:
        GPIO.setup(pin,GPIO.OUT)
        GPIO.output(pin, GPIO.HIGH)
        print("Led on")
        time.sleep(1)
        GPIO.setup(pin,GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)
        print("Led off")
        time.sleep(1)
except KeyboardInterrupt:
    GPIO.cleanup() 