#!/usr/bin/env python

# Simple test tool for the user interface.

# @author: Patrick Gavigan

import rospy
from std_msgs.msg import String
import time
from datetime import datetime

startTime = 0
endTime = 0

def sendMailMission(publisher):
    messageID = int(round(time.time() * 1000))  # Crude message ID
    agentID = "BROADCAST"
    userID = "user"
    messageType = "tell"

    # Prompt user for input (using Pyton 7 method, ROS does not use Python 3)
    sender = raw_input("Please enter the sender location (Example: post1): ")
    receiver = raw_input("Please enter the receiver location (Example: post4): ")
    
    # Build message, log and send it
    messageContent = "mailMission(" + str(sender) + "," + str(receiver) + ")"
    message = "<" + str(messageID) + "," + userID + "," + messageType + "," + agentID + "," + messageContent + ">"
    rospy.loginfo("Sending message: " + str(message))
    publisher.publish(message)


# Receive outbox messages. Just print everything.
def receiveMessage(data):
    rospy.loginfo("Received message: " + str(data.data))
    # ToDo: Send this to a phone app
    
    # Check if the message contains the delivered signal
    if "mailUpdate(delivered)" in data.data:
        endTime = datetime.now()
        rospy.loginfo("Mission complete at " + str(endTime))

# Main program
def rosMain():

    # Init the node
    rospy.init_node('user', anonymous=True)

    # Subscribe to outbox
    rospy.Subscriber('outbox', String, receiveMessage)

    # Setup the publisher for the result
    publisher = rospy.Publisher('inbox', String, queue_size=10)

    # Sleep for 5 seconds, give everything a chance to come online.
    time.sleep(5)

    # Prompt the use for the mailMission and send it
    sendMailMission(publisher)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        rosMain()
    except rospy.ROSInterruptException:
        pass
        