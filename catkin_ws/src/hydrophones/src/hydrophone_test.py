#!/usr/bin/env python3

import rospy
from smbus import SMBus
import time
from std_msgs.msg import Float32

rospy.init_node("hydrophone_test")

generic_publisher = rospy.Publisher('/jelly/delay',Float32,queue_size=1)

def twos_comp(val, bits):
    """compute the 2's complement of int value val"""
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)        # compute negative value
    return val                         # return positive value as is


def main():
    '''
    Main program function
    '''
    

    i2cbus = SMBus(8)  # Create a new I2C bus
    i2caddress = 0x69  # Address of MCP23017 device

    # i2cbus.write_byte_data(i2caddress, IOCON, 0x02)  # Update configuration register
    # i2cbus.write_word_data(i2caddress, IODIRA, 0xFF00)  # Set Port A as outputs and Port B as inputs
	
	
    while (True):
        data = i2cbus.read_byte(i2caddress)  # Read the value of Port B
        delay_time = twos_comp(data, 8)
        message = Float32()
        message.data = delay_time
        generic_publisher.publish(message)
        # print(twos_comp(data, 8), ": ", bin(data)) # print the value of Port B
        time.sleep(0.5)  # Wait 500ms


if __name__ == "__main__":
    main()
