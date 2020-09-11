#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import String


def get_charge(charge):

    if charge < 25:
        battery_state = "battery(low)"
    elif charge > 75:
        battery_state = "battery(ok) battery(full)"
    else:
        battery_state = "battery(ok)"

    send_charge = rospy.Publisher('perceptions', String, queue_size=10)

    send_charge.publish(battery_state)


def battery_charge():
    rospy.init_node('batteryManager', anonymous=True)
    rospy.Subscriber('battery/charge_ratio', Float32, get_charge)

    rospy.spin()


if __name__ == '__main__':
    try:
        battery_charge()
    except rospy.ROSInterruptException:
        pass