
import time
import board
import busio
from board import SDA, SCL

# To use default I2C bus (most boards)
i2c8=busio.I2C(SCL,SDA)
i2caddress = 0x69

data = [];

while (not i2c8.try_lock()):
    pass

try:
    while True:
        i2c8.readfrom_into(i2caddress, data)  # Read the value
        print(data) # print the value
        time.sleep(0.5)  # Wait 500ms

finally:  # unlock the i2c bus when ctrl-c'ing out of the loop
    i2c8.unlock()

