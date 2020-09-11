#!/usr/bin/env python

import rospy
from std_msgs.msg import String
    
# Translate the line sensor data into a perception and publish
def translatePerception(data, args):
    # Extract the publisher and the message data
    (publisher) = args
    data = data.data

    # Generate the perception
    if data == "c":
        line = "line(center)"
    elif data == "r":
        line = "line(right)"
    elif data == "l":
        line = "line(left)"
    elif data == "a":
        line = "line(across)"
    else:
        line = "line(lost)"

    # Publish the perception
    rospy.loginfo(line)
    publisher.publish(line)


# Initialize the node, setup the publisher and subscriber
def rosMain():
    rospy.init_node('lineTranslator', anonymous=True)
    publisher = rospy.Publisher('perceptions', String, queue_size=10)
    rospy.Subscriber('line', String, translatePerception, (publisher))
    rospy.spin()

if __name__ == '__main__':
    try:
        rosMain()
    except rospy.ROSInterruptException:
        pass
