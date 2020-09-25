#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import String


def getCharge(data, args):
    # Extract the publisher and the message data
    (publisher) = args
    charge = data.data

    # Generate the perception
    if charge < 0.25:
        batteryState = "battery(low)"
    elif charge > 0.99:
        batteryState = "battery(ok) battery(full)"
    else:
        batteryState = "battery(ok)"


    # Publish the perception
    rospy.loginfo(batteryState + " charge: " + str(charge))
    publisher.publish(batteryState)


def rosMain():
    rospy.init_node('batteryManager', anonymous=True)
    publisher = rospy.Publisher('perceptions', String, queue_size=10)
    rospy.Subscriber('battery/charge_ratio', Float32, getCharge, (publisher))

    rospy.spin()


if __name__ == '__main__':
    try:
        rosMain()
    except rospy.ROSInterruptException:
        pass