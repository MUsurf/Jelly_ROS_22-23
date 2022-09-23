#!/usr/bin/env python3

import rospy
import busio
import time
from board import SDA_1, SCL_1

from std_msgs.msg import Bool

rospy.init_node('volt_meter', anonymous=True)
pub = rospy.Publisher('volt_low', Bool, queue_size=10)
rate = rospy.Rate(0.5)
i2c=busio.I2C(SCL_1,SDA_1)
volt_meter = bytearray(2)
volt = 0
is_safe = False

def mapfloat(x, in_min,in_max,out_min,out_max):
    v_out =  (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    return v_out*6.135

def talker():
    while not rospy.is_shutdown():
        while not i2c.try_lock():
            print("Not locking")
            pass
        try:
            i2c.readfrom_into(0x20, volt_meter)
            volt = mapfloat(float(int.from_bytes(volt_meter,"little")),0,4095,0,3.3)
            print(volt)
            if(volt >= 14.5):
                print("battery voltage great!")
                is_safe = True
            else:
                print("DANGER BATTERY")
                is_safe = False
        finally:
            i2c.unlock()
        rospy.loginfo(is_safe)
        pub.publish(is_safe)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
