#!/usr/bin/env python3
import rospy
import time
import serial
from std_msgs.msg import String

serial_port = serial.Serial(
    port="/dev/ttyTHS1",
    baudrate=9600,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    )
time.sleep(1)
    
#finally:
    #serial_port.close()
    #pass

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    serial_port.write(data.data.encode())

  

def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('servo', String, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()

