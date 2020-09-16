#!/usr/bin/env python

# Main file for the navigator

# @author: Patrick Gavigan

import rospy
from RouteSearcher import RouteSearcher
from std_msgs.msg import String
import re

# Send the direction update
def sendDirection(data, args):
    (publisher, searcher) = args
    
    # Extract the message
    data = data.data
    
    # Get the parameters from message of the form "postPoint(current, previous)"
    parameterString = re.search(r'\((.*?)\)',data).group(1)
    
    # Remove any whitespace
    parameterString = parameterString.replace(" ", "")
    
    # Get the parameters
    parameters = parameterString.split(',')
    current = parameters[0]
    previous = parameters[1]

    # Get the next direction solution    
    solution = searcher.getNextDirection(previous, current)

    # Publish    
    rospy.loginfo("Navigation solution: " + solution)
    publisher.publish(solution)


# Set destination action
def doAction(data, args):
    (searcher) = args
    
    # Extract the message
    data = data.data

    if "setDestination" in data:
        dest = re.search(r'\((.*?)\)',data).group(1)
        rospy.loginfo("Setting destination to " + dest)
        searcher.setDestination(dest)

# Main program
def rosMain():
    # Setup the searcher
    searcher = RouteSearcher()
    
    # Init the node
    rospy.init_node('navigator', anonymous=True)

    # Subscribe to actions, watch for setDest messages
    rospy.Subscriber('actions', String, doAction, (searcher))

    # Setup the publisher for the result
    publisher = rospy.Publisher('perceptions', String, queue_size=10)
    
    # Listen for post point messages on the perceptions topic
    rospy.Subscriber('postPoint', String, sendDirection, (publisher, searcher))

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

# Unit tests for the search methods
def unitTest():
    # Setup the searcher
    searcher = RouteSearcher()
    searcher.setDestination("post7")
    
    searcher.setDestination("post2")
    turn = searcher.getNextDirection("post4", "post3")
    print("Turn should be left: " + turn)
    
    searcher.setDestination("post2")
    turn = searcher.getNextDirection("post5", "post3")
    print("Turn should be right: " + turn)
    
    searcher.setDestination("post5")
    turn = searcher.getNextDirection("post4", "post3")
    print("Turn should be forward: " + turn)
    
    searcher.setDestination("post4")
    turn = searcher.getNextDirection("post5", "post3")
    print("Turn should be forward: " + turn)
    
    searcher.setDestination("post3")
    turn = searcher.getNextDirection("post1", "post2")
    print("Turn should be left: " + turn)


if __name__ == '__main__':
    try:
        rosMain()
    except rospy.ROSInterruptException:
        pass
        