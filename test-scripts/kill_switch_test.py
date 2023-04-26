# Jetson.GPIO library documentation: https://github.com/NVIDIA/jetson-gpio
# Pin description Document from seeed studio w8 header (translatios from GPIO to BOARD pins) 
#https://files.seeedstudio.com/products/103110043/A203%20V2%20pin%20description.pdf

import Jetson.GPIO as GPIO
import sys

out_pin = 7 #GPIO 9 = BOARD 7
in_pin = 15 #GPIO 12 = BOARD 15

try:
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(out_pin, GPIO.OUT) #Set GPIO 9 as output
    GPIO.setup(in_pin, GPIO.IN) #Read GPIO 12 as input (opt.)

    # if turning motors on
    if sys.argv[1] == "on":
        GPIO.output(out_pin, GPIO.HIGH)	
        if GPIO.input(in_pin) == GPIO.HIGH:		# if GPIO 12 is HIGH
            print("Physical kill switch not connected")

	# if turning motors off
    elif sys.argv[1] == "off":
        GPIO.output(out_pin, GPIO.LOW)

except Exception as e:
	print(e)

finally:
	GPIO.cleanup()