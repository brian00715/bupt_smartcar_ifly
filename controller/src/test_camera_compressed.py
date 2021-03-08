#!/usr/bin/env python
#coding=utf-8
"""OpenCV feature detectors with ros CompressedImage Topics in python.

This example subscribes to a ros topic containing sensor_msgs
CompressedImage. It converts the CompressedImage into a numpy.ndarray,
then detects and marks features in that image. It finally displays
and publishes the new image - again as CompressedImage topic.
"""
# Python libs
import sys, time

# numpy and scipy
import numpy as np
# from scipy.ndimage import filters
import compressed_image_transport

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage

# We do not use cv_bridge it does not support CompressedImage in python
# from cv_bridge import CvBridge, CvBridgeError

DEBUG = False


class Camera:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.image_pub = rospy.Publisher("/cam/compressed",
                                         CompressedImage, queue_size=10)
        # subscribed Topic
        self.subscriber = rospy.Subscriber("/cam/compressed",
                                           CompressedImage, self.callback)

    def callback(self, ros_data):
        '''Callback function of subscribed topic.
        Here images get converted and features detected'''
        if DEBUG:
            print('received image of type: "%s"' % ros_data.format)

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8) # CompressedImage结构中data域实际存放图像数据
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        image_np = cv2.resize(image_np,(800,600))
        image_gray = cv2.cvtColor(image_np,cv2.COLOR_BGR2GRAY)
        cv2.imshow('src img',image_np)
        cv2.waitKey(3)

        #### Create CompressedIamge ####
        # msg = CompressedImage()
        # msg.header.stamp = rospy.Time.now()
        # msg.format = "jpeg"
        # msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
        # # Publish new image
        # self.image_pub.publish(msg)

        # self.subscriber.unregister()


def main(args):
    '''Initializes and cleanup ros node'''
    ic = Camera()
    rospy.init_node('image_feature', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print( "Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)