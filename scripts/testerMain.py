#!/usr/bin/env python

# Demonstration program of the savi_ros_bdi system

# Created on Wed Feb 19 16:17:21 2020
# @author: Patrick Gavigan

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from VirtualBot import VirtualBot


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
# outbox and action messages
def testerMain():
    print("This is the main test function!")
    
    bot = VirtualBot()
    
    rospy.init_node('saviRoombaDebugger', anonymous=True)
    
    # Setup the subscribers
    #rospy.Subscriber('actions', String, actionReceiver)
    #rospy.Subscriber('outbox', String, outboxReceiver)
    
    # Setup the publishers
    #inboxPublisher = rospy.Publisher('inbox', String, queue_size=10)
    linePublisher = rospy.Publisher('line', String, queue_size=10)
    qrPublisher = rospy.Publisher('qr', String, queue_size=10)
    batteryPublisher = rospy.Publisher('battery/charge_ratio', Float32, queue_size=10)
    
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        
        # Publish the line data
        line = bot.perceiveLine()
        rospy.loginfo("Line data: " + line)
        linePublisher.publish(line)
        
        # QR perception
        qr = bot.perceiveQR()
        rospy.loginfo("QR data: " + qr)
        qrPublisher.publish(qr)
        
        # Battery perception
        battery = bot.perceiveBattery()
        rospy.loginfo("Battery data: " + str(battery))
        batteryPublisher.publish(battery)
        
        # Publish the inbox message
        #message = "<" + str(rospy.get_time()) + ",2,achieve,0,demonstrate>"
        #rospy.loginfo("I said: " + message)
        #inboxPublisher.publish(message)
        
        # Delay until next cycle
        rate.sleep()

if __name__ == '__main__':
    try:
        testerMain()
    except rospy.ROSInterruptException:
        pass