#!/usr/bin/env python

# Simple test tool for the user interface.

# @author: Patrick Gavigan

import rospy
from std_msgs.msg import String
import time
from datetime import datetime

startTime = 0
endTime = 0

def sendCommand(publisher, senderLocation, receiverLocation):
    
    messageID = int(round(time.time() * 1000))  # Crude message ID
    agentID = "BROADCAST"
    userID = "user"
    messageType = "tell"
    
    senderParameter = "senderLocation(" + str(senderLocation) + ")"
    receiverParameter = "receiverLocation(" + str(receiverLocation) + ")"
    
    senderMessage = "<" + str(messageID) + "," + userID + "," + messageType + "," + agentID + "," + senderParameter + ">"
    receiverMessage = "<" + str(messageID+1) + "," + userID + "," + messageType + "," + agentID + "," + receiverParameter + ">"
    
    rospy.loginfo("Sender message: " + str(senderMessage))
    rospy.loginfo("Receiver message: " + str(receiverMessage))  
    
    publisher.publish(senderMessage)
    publisher.publish(receiverMessage)

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

    # Prompt the use for input
    sender = input("Please enter the sender's location (Example: post1): ")
    receiver = input("Please enter the receiver's location (Example: post4): ")

    # Send the message    
    sendCommand(publisher, sender, receiver)
    startTime = datetime.now()
    rospy.loginfo("Command sent at " + str(startTime))
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        rosMain()
    except rospy.ROSInterruptException:
        pass
        