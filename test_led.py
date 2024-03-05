import RPi.GPIO as GPIO
import time

LED_PIN = 6
LED_PIN_2 = 13
LED_PIN_3 = 19

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(LED_PIN, GPIO.OUT)
GPIO.setup(LED_PIN_2, GPIO.OUT)
GPIO.setup(LED_PIN_3, GPIO.OUT)


GPIO.output(LED_PIN, GPIO.LOW)
GPIO.output(LED_PIN_2, GPIO.LOW)
GPIO.output(LED_PIN_3, GPIO.LOW)
count = 0
while True:
    time.sleep(0.2)

    GPIO.output(LED_PIN, GPIO.HIGH)
    GPIO.output(LED_PIN_2, GPIO.HIGH)
    GPIO.output(LED_PIN_3, GPIO.HIGH)

    time.sleep(0.2)

    GPIO.output(LED_PIN, GPIO.LOW)
    GPIO.output(LED_PIN_2, GPIO.LOW)
    GPIO.output(LED_PIN_3, GPIO.LOW)              
    if count == 2:
        break
    count+=1

GPIO.cleanup()