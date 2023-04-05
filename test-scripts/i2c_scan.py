"""CircuitPython I2C Device Address Scan"""
# If you run this and it seems to hang, try manually unlocking
# your I2C bus from the REPL with
#  >>> import board
#  >>> board.I2C().unlock()

import time
import board
import busio
from board import SDA_1, SCL_1
from board import SDA, SCL

# To use default I2C bus (most boards)
i2c1=busio.I2C(SCL_1,SDA_1)
i2c0=busio.I2C(SCL, SDA)


# To create I2C bus on specific pins
# import busio
# i2c = busio.I2C(board.SCL1, board.SDA1)  # QT Py RP2040 STEMMA connector
# i2c = busio.I2C(board.GP1, board.GP0)    # Pi Pico RP2040

while (not i2c0.try_lock()) or (not i2c1.try_lock()):
    pass

try:
    while True:
        print(
            "I2C addresses found on bus 0:",
            [hex(device_address) for device_address in i2c0.scan()],
        )
        print(
            "I2C addresses found on bus 1:",
            [hex(device_address) for device_address in i2c1.scan()],
        )
        time.sleep(2)

finally:  # unlock the i2c bus when ctrl-c'ing out of the loop
    i2c0.unlock()
    i2c1.unlock()
