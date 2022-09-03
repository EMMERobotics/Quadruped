#!/usr/bin/env python3
import rospy
from quadruped_main.msg import leg_comm_msg

from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

from math import pi

# servo param
min_pulse = 750
max_pulse = 2950

# Create a simple PCA9685 class instance.
pca = PCA9685(i2c)
# You can optionally provide a finer tuned reference clock speed to improve the accuracy of the
# timing pulses. This calibration will be specific to each board and its environment. See the
# calibration.py example in the PCA9685 driver.
# pca = PCA9685(i2c, reference_clock_speed=25630710)
pca.frequency = 50
# The pulse range is 750 - 2250 by default. This range typically gives 135 degrees of
# range, but the default is to use 180 degrees. You can specify the expected range if you wish:
# servo7 = servo.Servo(pca.channels[7], actuation_range=135)

# map implement --- not checked ---
def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)


class Leg:
    def __init__(self, leg_i, hip_servo, femur_servo, tibia_servo, hip_offset, femur_offset, tibia_offset):
        self.leg_i = leg_i
        self.hip_offset = hip_offset
        self.femur_offset = femur_offset
        self.tibia_offset = tibia_offset
        #init servo object uniqe to each leg object
        self.hip_servo = servo.Servo(pca.channels[hip_servo], min_pulse, max_pulse)
        self.femur_servo = servo.Servo(pca.channels[femur_servo], min_pulse, max_pulse)
        self.tibia_servo = servo.Servo(pca.channels[tibia_servo], min_pulse, max_pulse)

    def publish_command(self, hipAngle, femurAngle, tibiaAngle):
        hipCommand = translate(hipAngle, 0, pi, 0, 180)
        femurCommand = translate(femurAngle, 0, pi, 0, 180)
        tibiaCommand = translate(tibiaAngle, 0, pi, 0, 180)

        hipCommand += self.hip_offset
        femurCommand += self.femur_offset
        tibiaCommand += self.tibia_offset

        self.hip_servo.angle = hipCommand
        self.femur_servo.angle = femurCommand
        self.tibia_servo.angle = tibiaCommand

        #for debug
        rospy.loginfo("Leg:%d hip:%f, femur:%f, tibia:%f " % (self.leg_i, hipCommand, femurCommand, tibiaCommand))


#create leg object for each legs
leg_FR = Leg(0, 0, 1, 2, 0, 0, 0) #check the offset unit -- offset will be directly added to the command
leg_FL = Leg(1, 3, 4, 5, 0, 0, 0)
leg_BL = Leg(2, 6, 7, 8, 0, 0, 0)
leg_BR = Leg(3, 9, 10, 11, 0, 0, 0)


def callback(data):
    #unpack data
    leg_index = data.leg
    hipAngle = data.hipAngle
    femurAngle = data.femurAngle
    tibiaAngle = data. tibiaAngle

    if leg_index == 0:
        leg_FL.publish_command(hipAngle, femurAngle, tibiaAngle)
    elif leg_index == 1:
        leg_FR.publish_command(hipAngle, femurAngle, tibiaAngle)
    elif leg_index == 2:
        leg_BL.publish_command(hipAngle, femurAngle, tibiaAngle)
    elif leg_index == 3:
        leg_BR.publish_command(hipAngle, femurAngle, tibiaAngle)
    else:
        rospy.loginfo("MP ERROR: leg_index out of range")
    
    #for debug
    #rospy.loginfo("Leg:%d hip:%f, femur:%f, tibia:%f " % (data.leg, data.hipAngle, data.femurAngle, data.tibiaAngle))


def listener():
    rospy.init_node('motor_controller', anonymous=True)
    rospy.Subscriber("motor_command", leg_comm_msg, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()