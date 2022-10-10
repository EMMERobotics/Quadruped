#!/usr/bin/env python3
import evdev
import ev3dev.auto as ev3
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
print("controller established")


def talker():
    pub = rospy.Publisher('joy_val', con_msg, queue_size=10)
    rospy.init_node('joystick', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    
    while not rospy.is_shutdown():
        for event in gamepad.read_loop():   #this loops infinitely
            if event.type == 3:             #A stick is moved
                if event.code == 5:         #Y axis on right stick
                    con_msg.val_y2 = scale_stick(event.value)
                if event.code == 2:         #X axis on right stick
                    con_msg.val_x2 = scale_stick(event.value)
                if event.code == 0:         #X axis on left stick
                    con_msg.val_xl = scale_stick(event.value)
                if event.code == 1:         #Y axis on left stick
                    con_msg.val_y1 = scale_stick(event.value)
        
        rospy.loginfo(con_msg)
        pub.publish(con_msg)
        rate.sleep()
           
   
    if __name__ == '__main__':
       try:
           talker()
       except rospy.ROSInterruptException:
           pass




