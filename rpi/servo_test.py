import time

from board import SCL, SDA
import busio

# Import the PCA9685 module. Available in the bundle and here:
#   https://github.com/adafruit/Adafruit_CircuitPython_PCA9685
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(SCL, SDA)

# Create a simple PCA9685 class instance.
pca = PCA9685(i2c)
# You can optionally provide a finer tuned reference clock speed to improve the accuracy of the
# timing pulses. This calibration will be specific to each board and its environment. See the
# calibration.py example in the PCA9685 driver.
# pca = PCA9685(i2c, reference_clock_speed=25630710)
pca.frequency = 50

# To get the full range of the servo you will likely need to adjust the min_pulse and max_pulse to
# match the stall points of the servo.
# This is an example for the Sub-micro servo: https://www.adafruit.com/product/2201
# servo7 = servo.Servo(pca.channels[7], min_pulse=580, max_pulse=2350)
# This is an example for the Micro Servo - High Powered, High Torque Metal Gear:
#   https://www.adafruit.com/product/2307
# servo7 = servo.Servo(pca.channels[7], min_pulse=500, max_pulse=2600)
# This is an example for the Standard servo - TowerPro SG-5010 - 5010:
#   https://www.adafruit.com/product/155
# servo7 = servo.Servo(pca.channels[7], min_pulse=400, max_pulse=2400)
# This is an example for the Analog Feedback Servo: https://www.adafruit.com/product/1404
# servo7 = servo.Servo(pca.channels[7], min_pulse=600, max_pulse=2500)
# This is an example for the Micro servo - TowerPro SG-92R: https://www.adafruit.com/product/169
# servo7 = servo.Servo(pca.channels[7], min_pulse=500, max_pulse=2400)

# The pulse range is 750 - 2250 by default. This range typically gives 135 degrees of
# range, but the default is to use 180 degrees. You can specify the expected range if you wish:
# servo7 = servo.Servo(pca.channels[7], actuation_range=135)

minrange = 550
maxrange = 2750

servo0 = servo.Servo(pca.channels[0], min_pulse=minrange, max_pulse=maxrange )
servo1 = servo.Servo(pca.channels[1], min_pulse=minrange, max_pulse=maxrange )
servo2 = servo.Servo(pca.channels[2], min_pulse=minrange, max_pulse=maxrange )
servo3 = servo.Servo(pca.channels[3], min_pulse=minrange, max_pulse=maxrange )
servo4 = servo.Servo(pca.channels[4], min_pulse=minrange, max_pulse=maxrange )
servo5 = servo.Servo(pca.channels[5], min_pulse=minrange, max_pulse=maxrange )
servo6 = servo.Servo(pca.channels[6], min_pulse=minrange, max_pulse=maxrange )
servo7 = servo.Servo(pca.channels[7], min_pulse=minrange, max_pulse=maxrange )
servo8 = servo.Servo(pca.channels[8], min_pulse=minrange, max_pulse=maxrange )
servo9 = servo.Servo(pca.channels[9], min_pulse=minrange, max_pulse=maxrange )
servo10 = servo.Servo(pca.channels[10], min_pulse=minrange, max_pulse=maxrange )
servo11 = servo.Servo(pca.channels[11], min_pulse=minrange, max_pulse=maxrange )


# We sleep in the loops to give the servo time to move into position.
servo0.angle = 83
servo1.angle = 92
servo2.angle = 97
servo3.angle = 85
servo4.angle = 132
servo5.angle = 47
servo6.angle = 140
servo7.angle = 40
servo8.angle = 96
servo9.angle = 83
servo10.angle = 83
servo11.angle = 85

"""
        # You can also specify the movement fractionally.
#fraction = 0.0
#while fraction < 1.0:
#    servo7.fraction = fraction
#    fraction += 0.01
#    time.sleep(0.03)
"""
#pca.deinit()

