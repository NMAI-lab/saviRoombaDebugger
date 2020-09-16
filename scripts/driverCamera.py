#!/usr/bin/env python

from imutils.video import VideoStream
from pyzbar import pyzbar
import rospy
from std_msgs.msg import String
#import datetime
import imutils
import time

# Initialize the video stream and allow the camera sensor to warm up
vs = VideoStream(src=0).start()
time.sleep(2.0)

def rosMain():
    # Initialize the node
    publisher = rospy.Publisher('qr', String, queue_size=10)
    rospy.init_node('cameraDriver', anonymous=True)
    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        # Get a frame from the threaded video stream and resize it to have a 
        # maximum width of 400 pixels
        frame = vs.read()
        frame = imutils.resize(frame, width=400)

        # Find the barcodes in the frame and decode each of the barcodes
        barcodes = pyzbar.decode(frame)

        for barcode in barcodes:
            # The barcode data is a bytes object so if we want to draw it
            # on our output image we need to convert it to a string first
            qr = barcode.data.decode("utf-8")

            # Publish the QR code
            rospy.loginfo("QR: " + str(qr))
            publisher.publish(qr)
        
        rate.sleep()


if __name__ == '__main__':
    try:
        rosMain()
    except rospy.ROSInterruptException:
        vs.stop()
        pass
