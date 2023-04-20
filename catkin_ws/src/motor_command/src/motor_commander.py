#!/usr/bin/env python3

# BEGIN IMPORT
import rospy
import busio
import time
from board import SCL_1, SDA_1
import adafruit_pca9685 as PCA9685
# END IMPORT

# BEGIN STD_MSGS
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Bool
# END STD_MSGS

# BEGIN SETUP
rospy.init_node("motor_commander")
rate = rospy.Rate(100)
i2c=busio.I2C(SCL_1,SDA_1)
pca = PCA9685.PCA9685(i2c, address = 0x40)
pca.frequency = 280 # Hz
# END SETUP

class MainLoop():
    # Set channel numbers to motors here!
    def __init__(self):
        self.callback_count = 201
        self.motorNum = 8
        self.motor1 = pca.channels[7]
        self.motor2 = pca.channels[3]
        self.motor3 = pca.channels[6]
        self.motor4 = pca.channels[2]
        self.motor5 = pca.channels[0]
        self.motor6 = pca.channels[4]
        self.motor7 = pca.channels[1]
        self.motor8 = pca.channels[5]

    #convert time in microseconds to a duty cycle number as a 16 bit int
    def microSec_to_duty_int(self,microSec):
        samp_time = (1/pca.frequency) * 1000 * 1000 # Convert to Micro Sec, 280 Hz = 3571 ms
        duty_cycle = int((65536 * microSec)/(samp_time)) 
    #    print(duty_cycle)
        return duty_cycle

    #creates an array of desired pwm values for each motor
    def create_PWM_array(self,num):
        data = []
        for i in range(0,self.motorNum):
            data.append(0)

        for i in range(0,self.motorNum):
            data[i] = num

        return data

    def callback(self,message_rec):
        print("Data received is: " + str(message_rec.data))
        self.motor1.duty_cycle = self.microSec_to_duty_int(message_rec.data[0])
        self.motor2.duty_cycle = self.microSec_to_duty_int(message_rec.data[1])
        self.motor3.duty_cycle = self.microSec_to_duty_int(message_rec.data[2])
        self.motor4.duty_cycle = self.microSec_to_duty_int(message_rec.data[3])
        self.motor5.duty_cycle = self.microSec_to_duty_int(message_rec.data[4])
        self.motor6.duty_cycle = self.microSec_to_duty_int(message_rec.data[5])
        self.motor7.duty_cycle = self.microSec_to_duty_int(message_rec.data[6])
        self.motor8.duty_cycle = self.microSec_to_duty_int(message_rec.data[7])

    #motors must all be set to certain values to arm before they will turn on
    def arm_seq(self):
        self.set_all(0) #0ms, 0% duty cycle
        time.sleep(0.5)

        self.set_all(1000)
        time.sleep(0.5)

        self.set_all(500)
        time.sleep(0.5)

        self.set_all(1550)

    #Set all motors to the provided PWM value in microsec
    def set_all(self,PWM_microsec):
        data = self.create_PWM_array(PWM_microsec)
        self.motor1.duty_cycle = self.microSec_to_duty_int(data[0])
        self.motor2.duty_cycle = self.microSec_to_duty_int(data[1])
        self.motor3.duty_cycle = self.microSec_to_duty_int(data[2])
        self.motor4.duty_cycle = self.microSec_to_duty_int(data[3])
        self.motor5.duty_cycle = self.microSec_to_duty_int(data[4])
        self.motor6.duty_cycle = self.microSec_to_duty_int(data[5])
        self.motor7.duty_cycle = self.microSec_to_duty_int(data[6])
        self.motor8.duty_cycle = self.microSec_to_duty_int(data[7])

    #Disarm all motors by setting to 0
    def clo_seq(self):
        self.set_all(0)

    def cut_motors(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        if(data):
            pass
        else:
            self.clo_seq()

if __name__ == '__main__':
    try:
        loop = MainLoop()
        loop.arm_seq()
        while not rospy.is_shutdown():
            #Getts info from /command topic. Sets the motors to those values
            rospy.Subscriber("/command",Int32MultiArray,loop.callback)
            rospy.spin()
    
    except KeyboardInterrupt:
        loop.clo_seq()
