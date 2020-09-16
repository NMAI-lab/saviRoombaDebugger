#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO

# Initialize GPIO
GPIO.setmode(GPIO.BOARD)

right_sensor = 8
center_sensor = 10
left_sensor = 12

GPIO.setwarnings(False)
GPIO.setup(right_sensor, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(center_sensor, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(left_sensor, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


# Get the sesor data
def getLine():
    centerLED = GPIO.input(center_sensor)
    leftLED = GPIO.input(left_sensor)
    rightLED = GPIO.input(right_sensor)
    LEDs = (leftLED, centerLED, rightLED)
    
    # Interpret the data; 0 indicates the location of the line
    if LEDs == (1,0,1):
        result = "c"
    elif (LEDs == (1,1,0)) or (LEDs == (1,0,0)):
        result = "r"
    elif (LEDs == (0,1,1)) or (LEDs == (0,0,1)):
        result = "l"
    elif LEDs == (0,0,0):
        result = "a"
    else:
        result = "ll"
        
    return (result, LEDs)
    
# Poll the sensors, publish data
def rosMain():
    # Initialize the node
    pub = rospy.Publisher('line', String, queue_size=10)
    rospy.init_node('lineSensor', anonymous=True)
    rate = rospy.Rate(10)

    
    while not rospy.is_shutdown():
        
        # Get the sensor data
        (linePosition, LEDs) = getLine()
        rospy.loginfo("Line sensor data: " + str(linePosition) + ", " + str(LEDs))
        pub.publish(linePosition)
        rate.sleep()


# Run the program
if __name__ == '__main__':
    try:
        rosMain()
    except rospy.ROSInterruptException:
        GPIO.cleanup()
        pass
