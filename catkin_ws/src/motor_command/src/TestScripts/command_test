
#!/usr/bin/env python3
import rospy
import yaml
import numpy
# Import numpy for ROS messages to ease the extraction of data from messages
# that are arrays.
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg

# Import the PCA9685 module.
import adafruit_pca9685

# Initialise the PCA9685 using the default address (0x40).
esc = adafruit_pca9685.PCA9685(busnum=2)

# Set frequency to 60hz, good for servos.
esc.set_pwm_freq(280)

#current tested minimum ~1537us for pulse width NOT frequency
#1 - 2ms pulse recommended we measured on the scope ~237 ms offset 
#current tested maximum ~2300us for pulse width NOT frequency 
#tested at pwm_freq(280) 
#Changing frequency should alter these min and max values

from std_msgs.msg import Int32MultiArray

def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    global comm
    comm = data
    esc1 = comm.data[15]
    esc1 = int(esc1)
    print(esc1)
    esc.set_pwm(12,1,esc1)
    esc.set_pwm(13,0,esc1) #This produces no error.
    esc.set_pwm(14,0,esc1) #This DOES produce an error!
    esc.set_pwm(15,0,esc1)

def subscriber():
    sub = rospy.Subscriber('/command',numpy_msg(Floats),callback)
    rospy.init_node('subscriber', anonymous=True)
    rate = rospy.Rate(10)
    
    # print(command)
 
    #esc.set_pwm(12,0,2000)
    #esc.set_pwm(13,0,2000)
    #esc.set_pwm(14,0,2000)
    #esc.set_pwm(15,0,2000)
    # Comment out the rospy.spin command for motor testing.
    rospy.spin()
    
if __name__ == '__main__':

    esc.set_pwm(12,0,320)
    esc.set_pwm(13,0,320)
    esc.set_pwm(14,0,320)
    esc.set_pwm(15,0,320)
    
    rospy.sleep(1)
    
    esc.set_pwm(12,0,1500)
    esc.set_pwm(13,0,1500)
    esc.set_pwm(14,0,1500)
    esc.set_pwm(15,0,1500)
    
    rospy.sleep(1)
    
    subscriber()