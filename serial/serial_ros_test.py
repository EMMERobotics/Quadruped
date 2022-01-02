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
    
time.sleep(1)
    
try
    # Send a simple header
    while True:
        #if serial_port.inWaiting() > 0:
            #serial_port.open()
            #print(serial_port.is_open)
            drive  = "#1P2000T1000\r\n".encode()

            serial_port.write(drive)
            time.sleep(5)

            stop = "#1P1500T1000\r\n".encode()
            serial_port.write(stop)
            time.sleep(5)
            # if we get a carriage return, add a line feed too
            # \r is a carriage return; \n is a line feed
            # This is to help the tty program on the other end 
            # Windows is \r\n for carriage return, line feed
            # Macintosh and Linux use \n
                # For Windows boxen on the other end


except KeyboardInterrupt:
    print("Exiting Program")

except Exception as exception_error:
    print("Error occurred. Exiting Program")
    print("Error: " + str(exception_error))

finally:
    serial_port.close()
    pass

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('servo', String, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
