#!/usr/bin/env python3
import rospy
from quadruped_main.msg import leg_comm_msg

def callback(data):
    rospy.loginfo("Leg:%d hip:%f, femur:%f, tibia:%f " % (data.leg, data.hipAngle, data.femurAngle, data.tibiaAngle))

def listener():
    rospy.init_node('motor_controller', anonymous=True)
    rospy.Subscriber("motor_command", leg_comm_msg, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()