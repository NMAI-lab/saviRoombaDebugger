#!/usr/bin/env python

from imutils.video import VideoStream
from pyzbar import pyzbar
import rospy
from std_msgs.msg import String
import imutils
import time

# initialize the video stream and allow the camera sensor to warm up
print("[INFO] starting video stream...")
vs = VideoStream(src=0).start()
# vs = VideoStream(usePiCamera=True).start()
time.sleep(2.0)


def qrscanner():
    postPointPublisher = rospy.Publisher('postPoint', String, queue_size=10)
    perceptionsPublisher = rospy.Publisher('perceptions', String, queue_size=10)
    
    print("[INFO] publisher created...")
    rospy.init_node('qrpostpoint', anonymous=True)
    rate = rospy.Rate(2)
    last_point = -1

    while not rospy.is_shutdown():
        # grab the frame from the threaded video stream and resize it to
        # have a maximum width of 400 pixels
        frame = vs.read()
        frame = imutils.resize(frame, width=400)

        post_stop = "postPoint({})".format(last_point)

        # find the barcodes in the frame and decode each of the barcodes
        barcodes = pyzbar.decode(frame)

        for barcode in barcodes:
            # the barcode data is a bytes object so if we want to draw it
            # on our output image we need to convert it to a string first
            barcode_data = barcode.data.decode("utf-8")
            point = barcode_data.split("post")
            post_stop = "postPoint({},{})".format(point[1], last_point)
            last_point = point[1]

        rospy.loginfo(post_stop)
        postPointPublisher.publish(post_stop)
        perceptionsPublisher.publish(post_stop)
        rate.sleep()


if __name__ == '__main__':
    try:
        qrscanner()
    except rospy.ROSInterruptException:
        vs.stop()
        pass
