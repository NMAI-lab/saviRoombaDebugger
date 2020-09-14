#!/usr/bin/env python

import rospy
import re
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from driverLineSensor import getLine

def decodeAction(data, args):
    
    # Get the parameters
    action = str(data.data)
    (drivePublisher, dockPublisher, undockPublisher) = args
    rospy.loginfo("Action: " + action)
    
    # Handle the docking station cases
    if action == "station(dock)":
        dockPublisher.publish()
    elif action == "station(undock)":
        undockPublisher.publish()
        
    # Extract the action parameter between the brackets
    parameter = re.search('\((.*)\)', action).group(1)
    
    # Deal with drive action
    if re.search("drive", action):
        message = getTwistMesg(parameter)
        drivePublisher.publish(message)
    
    # Deal with turn message (similar to drive action but continues until the 
    # line sensor detects "c")
    elif re.search("turn", action):
        message = getTwistMesg(parameter)
        while getLine()[0] != "c":
            drivePublisher.publish(message)
        message.linear.x = 0
        message.angular.z = 0
        drivePublisher.publish(message)
    
    # Deal with invalid action
    else:
        rospy.loginfo("Invalid action ignored")
        
        
def getTwistMesg(parameter):
    message = Twist()
    
    if parameter == "forward":
        message.linear.x = 0.1
        message.angular.z = 0
    elif parameter == "left":
        message.linear.x = 0
        message.angular.z = 0.1
    elif parameter == "right":
        message.linear.x = 0
        message.angular.z = -0.1
    elif parameter == "spiral":
        message.linear.x = 0.1
        message.angular.z = 0.1
    else:                           # Stop
        message.linear.x = 0
        message.angular.z = 0
        
    return message


def rosMain():
    
    drivePublisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    dockPublisher = rospy.Publisher('dock', Empty, queue_size=10)
    undockPublisher = rospy.Publisher('undock', Empty, queue_size=10)
        
    rospy.init_node('actionTranslator', anonymous=True)
    rospy.Subscriber('actions', String, decodeAction, (drivePublisher, dockPublisher, undockPublisher))

    rospy.spin()


if __name__ == '__main__':
    try:
        rosMain()
    except rospy.ROSInterruptException:
        pass
