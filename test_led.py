import RPi.GPIO as GPIO
import time

OUTPUT_LEDS = [0,1,2]

num_led_right = 6
num_led_left = 13
num_led_warning = 19
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(num_led_right,GPIO.OUT)
GPIO.setup(num_led_left,GPIO.OUT)
GPIO.setup(num_led_warning,GPIO.OUT)


GPIO.output(num_led_right,GPIO.HIGH)
GPIO.output(num_led_left,GPIO.HIGH)
GPIO.output(num_led_warning,GPIO.HIGH)

time.sleep(5)
GPIO.cleanup()

# GPIO.setwarnings(False)
# # num_led_right: ID of GPIO pin (Ex: num_led_right = 17 => GPIO17)
# # num_led_left: ID of GPIO pin (Ex: num_led_left = 27 => GPIO27)
# # num_led_warning: ID of GPIO pin (Ex: num_led_warning = 22 => GPIO22)
# # led_1: integer signal of LED 1 from pip num_led_right 
# # led_2: integer signal of LED 2 from pip num_led_left 
# # les_3: integer signal of LED 3 from pip num_led_warning 
# GPIO.cleanup()
# GPIO.setup(num_led_right,GPIO.OUT)
# GPIO.setup(num_led_left,GPIO.OUT)
# GPIO.setup(num_led_warning,GPIO.OUT)
# # # print("Run")

# GPIO.output(num_led_right,GPIO.LOW)
# GPIO.output(num_led_left,GPIO.HIGH)
# GPIO.output(num_led_warning,GPIO.LOW)


# while True:
#     # GPIO.output(num_led_right,GPIO.HIGH)
#     # GPIO.output(num_led_left,GPIO.LOW)
#     # GPIO.output(num_led_warning,GPIO.HIGH)
#     # OUTPUT_LEDS = [1,1,1]
#     # print(OUTPUT_LEDS)
#     time.sleep(0.25) 
#     if sum(OUTPUT_LEDS) > 0:
#         if OUTPUT_LEDS[0] > 0:
#             GPIO.output(num_led_right,GPIO.HIGH) #GPIO.LOW
#         else: 
#             GPIO.output(num_led_right,GPIO.LOW)
#         if OUTPUT_LEDS[1] > 0:
#             GPIO.output(num_led_left,GPIO.LOW)
#         else: 
#             GPIO.output(num_led_left,GPIO.HIGH)
#         if OUTPUT_LEDS[2] > 0:
#             GPIO.output(num_led_warning,GPIO.HIGH)
#         else: 
#             GPIO.output(num_led_warning,GPIO.LOW)
#         if max(OUTPUT_LEDS) > 1:
#             time.sleep(0.15)
#             if OUTPUT_LEDS[0] > 1:
#                 GPIO.output(num_led_right,GPIO.LOW)
#             if OUTPUT_LEDS[1] > 1:
#                 GPIO.output(num_led_left,GPIO.HIGH)
#             if OUTPUT_LEDS[2] > 1:
#                 GPIO.output(num_led_warning,GPIO.LOW)
#     else:
#         GPIO.output(num_led_right,GPIO.LOW)
#         GPIO.output(num_led_left,GPIO.HIGH)
#         GPIO.output(num_led_warning,GPIO.LOW)

# GPIO.cleanup()