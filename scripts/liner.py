#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)

right_sensor = 8
center_sensor = 10
left_sensor = 12

GPIO.setwarnings(False)
GPIO.setup(right_sensor, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(center_sensor, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(left_sensor, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


def get_line():
    if GPIO.input(center_sensor) == 1 and GPIO.input(right_sensor) == 0 and GPIO.input(left_sensor) == 0:
        return "c"
    elif GPIO.input(right_sensor) == 1 and GPIO.input(center_sensor) == 0 and GPIO.input(left_sensor) == 0:
        return "r"
    elif GPIO.input(right_sensor) == 1 and GPIO.input(center_sensor) == 1 and GPIO.input(left_sensor) == 0:
        return "r"
    elif GPIO.input(left_sensor) == 1 and GPIO.input(right_sensor) == 0 and GPIO.input(center_sensor) == 0:
        return "l"
    elif GPIO.input(left_sensor) == 1 and GPIO.input(right_sensor) == 0 and GPIO.input(center_sensor) == 1:
        return "l"
    elif GPIO.input(center_sensor) == 1 and GPIO.input(right_sensor) == 1 and GPIO.input(left_sensor) == 1:
        return "a"
    else:
        return "ll"
    

def liner():
    pub = rospy.Publisher('perceptions', String, queue_size=10)
    rospy.init_node('liner', anonymous=True)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        if get_line() == "c":
            line = "line(center)"
        elif get_line() == "r":
            line = "line(right)"
        elif get_line() == "l":
            line = "line(left)"
        elif get_line() == "a":
            line = "line(across)"
        else:
            line = "line(lost)"
        sense = "{}, {}, {}".format(GPIO.input(left_sensor), GPIO.input(center_sensor), GPIO.input(right_sensor))
        rospy.loginfo(sense)
        pub.publish(line)
        rate.sleep()


if __name__ == '__main__':
    try:
        liner()
    except rospy.ROSInterruptException:
        GPIO.cleanup()
        pass
