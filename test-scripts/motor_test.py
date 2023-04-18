from board import SCL_1, SDA_1
import busio
import time

# Import the PCA9685 module.
from adafruit_pca9685 import PCA9685

# Create the I2C bus interface.
i2c_bus = busio.I2C(SCL_1, SDA_1)

# Create a simple PCA9685 class instance.
pca = PCA9685(i2c_bus)

# Set the PWM frequency
pca.frequency = 500

# https://github.com/bitdump/BLHeli/blob/master/BLHeli_32%20ARM/BLHeli_32%20manual%20ARM%20Rev32.x.pdf
# supports PWM 1000-2000 microsecond pulses
def microSec_to_duty(microSec,freq):
        samp_time = (1/freq) * 1000 * 1000 # Convert to Micro Sec
        duty_cycle = int((65536 * microSec)/(samp_time))
    #    print(duty_cycle)
        return duty_cycle

def duty_percent_to_int(duty_percent):
	return int(duty_percent * 65536)


def set_all_duty(duty_percent):
	duty = duty_percent_to_int(duty_percent)
	for i in range(8):
		pca.channels[i].duty_cycle = duty
	
  
def set_all_ms(pwm_period_ms):
	for i in range(8):
		pca.channels[i].duty_cycle = microSec_to_duty(pwm_period_ms, pca.frequency)

def arm_seq():
	set_all_ms(0)
	time.sleep(0.5)

	set_all_ms(1000)
	time.sleep(0.5)

	set_all_ms(500)
	time.sleep(0.5)

	set_all_ms(1500)
	time.sleep(0.5)

def clo_seq():
	set_all_ms(0)

# motors
arm_seq()
#for i in range(100):
#	print(i/100)
#	pca.channels[0].duty_cycle = duty_percent_to_int(i/100)
#	time.sleep(1)
for i in range(8):
	print("testing ", i)
	pca.channels[i].duty_cycle = microSec_to_duty(1600, pca.frequency)
	time.sleep(1)
	pca.channels[i].duty_cycle = microSec_to_duty(1500, pca.frequency)
	time.sleep(1)
	pca.channels[i].duty_cycle = microSec_to_duty(1400, pca.frequency)
	time.sleep(1)
	pca.channels[i].duty_cycle = microSec_to_duty(1500, pca.frequency)
clo_seq()

# LEDs
#for i in range (8, 16):
#	print("testing ", i)
#	pca.channels[i].duty_cycle = 0
#	time.sleep(1)
#	pca.channels[i].duty_cycle = 0x0FFF
#	time.sleep(1)
#	pca.channels[i].duty_cycle = 0

