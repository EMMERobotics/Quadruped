#!/usr/bin/env python3
import evdev
import rospy
from quadruped_main.msg import con_msg


## Some helpers ##
def scale_stick(value):
    return scale(value,(0,255),(-100,100))

def scale(val, src, dst):
    """
    Scale the given value from the scale of src to the scale of dst.
    val: float or int
    src: tuple
    dst: tuple
    example: print(scale(99, (0.0, 99.0), (-1.0, +1.0)))
    """
    return (float(val - src[0]) / (src[1] - src[0])) * (dst[1] - dst[0]) + dst[0]

#creates object 'gamepad' to store the data
gamepad = evdev.InputDevice('/dev/input/event1')  # <====== change this to correct addr
rospy.loginfo("controller established")
msg = con_msg()


def deadzone(val, center, range):
    if val > (center-range) and val < (center+range):
        return center

    elif val < 0 or val > 256:
        rospy.loginfo("Controller Value out of range")
        return center

    else:
        return val


def joystick():
    pub = rospy.Publisher('joy_val', con_msg, queue_size=10)
    rospy.init_node('joystick', anonymous=True)
    rate = rospy.Rate(100) # 10hz

    while not rospy.is_shutdown():
        #read value from controller
        event = gamepad.read_one()
        if event != None:
            if event.type == 3:             #A stick is moved
                if event.code == 4:         #Y axis on right stick
                    msg.val_y2 = deadzone(event.value, 127, 16)
                if event.code == 3:         #X axis on right stick
                    msg.val_x2 = deadzone(event.value, 127, 16)
                if event.code == 0:         #X axis on left stick
                    msg.val_x1 = deadzone(event.value, 127, 16)
                if event.code == 1:         #Y axis on left stick
                    msg.val_y1 = deadzone(event.value, 127, 16)

        #rospy.loginfo(con_msg.val_y2)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        joystick()
    except rospy.ROSInterruptException:
        pass


