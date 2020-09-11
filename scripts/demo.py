#!/usr/bin/env python

# Demonstration program of the savi_ros_bdi system

# Created on Wed Feb 19 16:17:21 2020
# @author: Patrick Gavigan

import rospy
from std_msgs.msg import String


# Callback function for action messages.
# Message is pritned to the console logs
def actionReceiver(data): 
    message = str(rospy.get_caller_id() + 'I heard: ' + str(data.data))
    rospy.loginfo(message)


# Callback function for outbox messages.
# Message is pritned to the console logs
def outboxReceiver(data): 
    message = str(rospy.get_caller_id() + 'I heard: ' + str(data.data))
    rospy.loginfo(message)


# Demo program that publishes perceptions and inbox messages and listens to 
# outobx and action messages
def demo():
    rospy.init_node('demoSaviTranslator', anonymous=True)
    
    # Setup the subscribers
    rospy.Subscriber('actions', String, actionReceiver)
    rospy.Subscriber('outbox', String, outboxReceiver)
    
    # Setup the publishers
    inboxPublisher = rospy.Publisher('inbox', String, queue_size=10)
    perceptionPublisher = rospy.Publisher('perceptions', String, queue_size=10)
    
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        # Publish the inbox message
        message = "<" + str(rospy.get_time()) + ",2,achieve,0,demonstrate>"
        rospy.loginfo("I said: " + message)
        inboxPublisher.publish(message)
        
        # Publish the perception
        message = "time(%s)" % rospy.get_time()
        rospy.loginfo("I said: " + message)
        perceptionPublisher.publish(message)

        # Delay until next cycle
        rate.sleep()


if __name__ == '__main__':
    try:
        demo()
    except rospy.ROSInterruptException:
        pass
