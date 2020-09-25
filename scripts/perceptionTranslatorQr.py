#!/usr/bin/env python

import rospy
from std_msgs.msg import String
    
# Global variables to remember current and previous qr codes
previous = "unknown"
current = "unknown"

# Translate the line sensor data into a perception and publish
def translatePerception(data, args):
    # Extract the publisher and the message data
    (perceptionPublisher, postPointPublisher) = args
    qr = data.data
    
    # Get access to the global variables (a bit hacky)
    global previous
    global current
    
    # Check if the post point changed, update history if necessary
    if qr != current:
        previous = current
        current = qr

    # Publish the perception
    postPoint = "postPoint({},{})".format(current, previous)
    rospy.loginfo(postPoint)
    postPointPublisher.publish(postPoint)
    perceptionPublisher.publish(postPoint)

# Initialize the node, setup the publisher and subscriber
def rosMain():
    rospy.init_node('qrTranslator', anonymous=True)
    perceptionPublisher = rospy.Publisher('perceptions', String, queue_size=10)
    postPointPublisher = rospy.Publisher('postPoint', String, queue_size=10)
    rospy.Subscriber('qr', String, translatePerception, (perceptionPublisher, postPointPublisher))
    rospy.spin()

if __name__ == '__main__':
    try:
        rosMain()
    except rospy.ROSInterruptException:
        pass
