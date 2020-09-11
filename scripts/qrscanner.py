#!/usr/bin/env python

from imutils.video import VideoStream
from pyzbar import pyzbar
import rospy
from std_msgs.msg import String
import datetime
import imutils
import time

# initialize the video stream and allow the camera sensor to warm up
print("[INFO] starting video stream...")
vs = VideoStream(src=0).start()
# vs = VideoStream(usePiCamera=True).start()
time.sleep(2.0)


def qrscanner():
    pub = rospy.Publisher('perceptions', String, queue_size=10)
    print("[INFO] publisher created...")
    rospy.init_node('qrpostpoint', anonymous=True)
    rate = rospy.Rate(2)
    currentPostPoint = -1
    previousPostPoint = -1



    while not rospy.is_shutdown():
        # grab the frame from the threaded video stream and resize it to
        # have a maximum width of 400 pixels
        frame = vs.read()
        frame = imutils.resize(frame, width=400)

        # find the barcodes in the frame and decode each of the barcodes
        barcodes = pyzbar.decode(frame)

        for barcode in barcodes:
            # the barcode data is a bytes object so if we want to draw it
            # on our output image we need to convert it to a string first
            newPostPoint = barcode.data.decode("utf-8")
            
            # Check if the post point changed, update history if necessary
            if newPostPoint != currentPostPoint:
                previousPostPoint = currentPostPoint
                currentPostPoint = newPostPoint

        postPointPerception = "postPoint({},{})".format(currentPostPoint, previousPostPoint)
        rospy.loginfo(postPointPerception)
        pub.publish(postPointPerception)
        
        rate.sleep()


if __name__ == '__main__':
    try:
        qrscanner()
    except rospy.ROSInterruptException:
        vs.stop()
        pass
