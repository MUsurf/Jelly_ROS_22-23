

# BEGIN IMPORT

from __future__ import division
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
i2c=busio.I2C(SCL_1,SDA_1)
pca = PCA9685.PCA9685(i2c, address = 0x40)
pca.frequency = 280 # Hz
# END SETUP

class MainLoop():
    def __init__(self):
        self.callback_count = 201
        self.motorNum = 8
        self.motor1 = pca.channels[7] # Correct
        self.motor2 = pca.channels[0] # Correct
        self.motor3 = pca.channels[6] # Correct
        self.motor4 = pca.channels[1] # Correct
        self.motor5 = pca.channels[3] # Correct
        self.motor6 = pca.channels[4] # Correct
        self.motor7 = pca.channels[2] # Correct
        self.motor8 = pca.channels[5] # Correct

    def microSec_to_duty(self,microSec,freq):
        samp_time = (1/freq) * 1000 * 1000 # Convert to Micro Sec
        duty_cycle = int((65536 * microSec)/(samp_time))
    #    print(duty_cycle)
        return duty_cycle

    def data_type(self,num):
        data = []
        for i in range(0,self.motorNum):
            data.append(0)

        for i in range(0,self.motorNum):
            data[i] = num

        return data

    def callback(self,message_rec):
        #print("Data received is: " + str(message_rec.data))
        self.motor1.duty_cycle = self.microSec_to_duty(message_rec[0],pca.frequency)
        self.motor2.duty_cycle = self.microSec_to_duty(message_rec[1],pca.frequency)
        self.motor3.duty_cycle = self.microSec_to_duty(message_rec[2],pca.frequency)
        self.motor4.duty_cycle = self.microSec_to_duty(message_rec[3],pca.frequency)
        self.motor5.duty_cycle = self.microSec_to_duty(message_rec[4],pca.frequency)
        self.motor6.duty_cycle = self.microSec_to_duty(message_rec[5],pca.frequency)
        self.motor7.duty_cycle = self.microSec_to_duty(message_rec[6],pca.frequency)
        self.motor8.duty_cycle = self.microSec_to_duty(message_rec[7],pca.frequency)

    def arm_seq(self):
        self.set_all(0)
        time.sleep(0.5)

        self.set_all(1000)
        time.sleep(0.5)

        self.set_all(500)
        time.sleep(0.5)

        self.set_all(1550)
  #      time.sleep(0.5) #FOR TESTING PURPOSES! REMOVE BEFORE USE!
  #      self.motor0.duty_cycle = self.microSec_to_duty(1600,pca.frequency)

    def set_all(self,PWM_setting):
        data = self.data_type(PWM_setting)
        #print(data[0])
        #print(int(pca.frequency))
        self.motor1.duty_cycle = self.microSec_to_duty(data[0],pca.frequency)
        self.motor2.duty_cycle = self.microSec_to_duty(data[1],pca.frequency)
        self.motor3.duty_cycle = self.microSec_to_duty(data[2],pca.frequency)
        self.motor4.duty_cycle = self.microSec_to_duty(data[3],pca.frequency)
        self.motor5.duty_cycle = self.microSec_to_duty(data[4],pca.frequency)
        self.motor6.duty_cycle = self.microSec_to_duty(data[5],pca.frequency)
        self.motor7.duty_cycle = self.microSec_to_duty(data[6],pca.frequency)
        self.motor8.duty_cycle = self.microSec_to_duty(data[7],pca.frequency)

    def clo_seq(self):
        self.set_all(0)

#############################################################
arming_time = 2
diving_time = 2
forward_time = 120
#############################################################
down_motors = 1665
front_motors = 1550
#############################################################
loop = MainLoop()
print("Arming motors: ")
loop.arm_seq()
time.sleep(arming_time)

# print("Diving: ")
# loop.callback([front_motors,down_motors,down_motors,front_motors,front_motors,down_motors,down_motors,front_motors])
# time.sleep(diving_time)
loop.set_all(1550)
time.sleep(1)

print("Forwarding: ")
#############################################################
front_motors = 2050
down_motors = 1550
left_motors = 2050
right_motors = 1050
#############################################################
loop.callback([right_motors,down_motors,down_motors,right_motors,left_motors,down_motors,down_motors,left_motors])
time.sleep(15)

loop.callback([])