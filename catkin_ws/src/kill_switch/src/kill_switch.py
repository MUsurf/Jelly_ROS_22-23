
# Jetson.GPIO library documentation: https://github.com/NVIDIA/jetson-gpio
# Pin description Document from seeed studio w8 header (translatios from GPIO to BOARD pins) 
#https://files.seeedstudio.com/products/103110043/A203%20V2%20pin%20description.pdf

import Jetson.GPIO as GPIO
import sys #temporary for testing with command line
import rospy
from std_msgs.msg import String

out_pin = 7 #GPIO 9 = BOARD 7
in_pin = 15 #GPIO 12 = BOARD 15

pub = rospy.Publisher('kill_status', bool)
rospy.init_node('kill_switch', anonymous=True)
#rate = rospy.Rate(10) # 10hz	#unsure if this will be needed

try:
	GPIO.setmode(GPIO.BOARD)
	GPIO.setup(out_pin, GPIO.OUT) #Set GPIO 9 as output
	GPIO.setup(in_pin, GPIO.IN) #Read GPIO 12 as input (opt.)
except Exception as e:
	print(e)
finally:
	GPIO.cleanup()

while not rospy.is_shutdown():

try:
	GPIO.output(out_pin, GPIO.HIGH)	
	if GPIO.input(in_pin) == GPIO.HIGH:		# if GPIO 12 is HIGH
		pub.publish(False)
		print("Physical kill switch not connected")

	# if turning motors off
	elif sys.argv[1] == "off":
		# Set GPIO 9 LOW
		GPIO.output(out_pin, GPIO.LOW)

except Exception as e:
	print(e)

finally:
	GPIO.cleanup()

