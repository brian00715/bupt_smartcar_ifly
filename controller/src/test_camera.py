#!/usr/bin/env python
# encoding=utf-8
# BEGIN ALL

import actionlib
import cv2
import cv_bridge
import numpy as np
import rospy
import tf
from control_msgs.msg import *
from gazebo_msgs.srv import *
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Image,CompressedImage
from std_srvs.srv import Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Bool
from tf.transformations import quaternion_from_euler


class RobotCamera:
    def __init__(self, get_state_service, set_state_service):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/cam',
                                          CompressedImage, self.image_callback)
        self.get_state_service = get_state_service
        self.set_state_service = set_state_service

    def image_callback(self, msg):
        """摄像头回调函数"""
        # self.cube_reached_pub.publish(self.MEET_CUBE)
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        cv2.imshow("raw image", img)
        cv2.waitKey(3)


if __name__ == '__main__':
    rospy.init_node('robot_camera')
    rospy.wait_for_service('/gazebo/get_model_state')
    get_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    rospy.wait_for_service('/gazebo/set_model_state')
    set_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    catch = RobotCamera(get_state_service, set_state_service)
    rospy.spin()
