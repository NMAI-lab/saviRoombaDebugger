#!/usr/bin/env python
# This node is used for logging performance inflormation used for benchmarking performance.

import rospy
from std_msgs.msg import String
from datetime import datetime
import csv

def logCsv(topic, data, timestamp, m):
    fileName = 'log.csv'
    with open(fileName, mode=m) as node_log:
        node_logger = csv.writer(node_log, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        node_logger.writerow([topic, data, timestamp])
    
def logAction(action):
    data = str(action.data)
    dateTimeObj = str(datetime.now())
    logCsv('actions', data, dateTimeObj, 'a+')

def logPerception(perception):
    data = str(perception.data)
    dateTimeObj = str(datetime.now())
    logCsv('perceptions', data, dateTimeObj, 'a+')
    
def logReasoningCycle(measurement):
    data = str(measurement.data)
    dateTimeObj = str(datetime.now())
    logCsv('cycleLength', data, dateTimeObj, 'a+')
    

def logNodes():
    rospy.init_node('nodeLogger', anonymous=True)
    rospy.Subscriber('actions', String, logAction)
    rospy.Subscriber('perceptions', String, logPerception)
    rospy.Subscriber('reasoningPerformance', String, logReasoningCycle)

    rospy.spin()
    
    
if __name__ == '__main__':
    try:
        logCsv('Topic', 'Data', 'Timestamp', 'w')
        logNodes()
    except rospy.ROSInterruptException:
        pass
    
    