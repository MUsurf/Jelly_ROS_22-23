#!/usr/bin/env python3
import rospy
import busio
import board
# from board import SCL_1, SDA_1
from std_msgs.msg import Int32MultiArray
from adafruit_pca9685 import PCA9685



# esF = adafruit_pca9685.PCA9685(busnum=8)

rospy.init_node("command_node")

class MainLoop():
    def __init__(self):
        self.i2c = busio.I2C(SCL_1,SDA_1)
        self.esc = PCA9685(self.i2c)
        self.esc.set_pwm_freq(280)
        rate = rospy.rate(5)
        while not rospy.is_shutdown():
            rospy.Subscriber("/command",Int32MultiArray,self.callback)
            rate.sleep()

    def callback(self,data):
        self.esc.set_pwm(12,1,data[11])
        self.esc.set_pwm(13,1,data[12])
        self.esc.set_pwm(14,1,data[13])
        self.esc.set_pwm(15,1,data[14])

if __name__ == '__main__':
    MainLoop()
