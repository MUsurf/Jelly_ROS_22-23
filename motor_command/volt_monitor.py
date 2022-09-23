import busio
import time
from board import SDA_1, SCL_1

i2c=busio.I2C(SCL_1,SDA_1)
volt_meter = bytearray(2)
time.sleep(1)

def mapfloat(x, in_min,in_max,out_min,out_max):
    v_out =  (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    return v_out*6.135

while True:
    while not i2c.try_lock():
        print("Not locking")
        pass
    try:
        i2c.readfrom_into(0x20, volt_meter)
        print("int value:")
        print(int.from_bytes(volt_meter, "little"))
        print("mapped values:")
        print(mapfloat(float(int.from_bytes(volt_meter, "little")), 0, 4095, 0, 3.3))
        time.sleep(1)
    finally:
        i2c.unlock()
