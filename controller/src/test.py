#!/usr/bin/env python
# coding=utf-8
import rospy
from gazebo_msgs.srv import (GetModelState, GetModelStateRequest,
                             GetWorldProperties)
from geometry_msgs.msg import Twist
from move_base import *
from rosgraph_msgs.msg import Clock

if __name__ == '__main__':
    node = rospy.init_node('judement')
    rospy.wait_for_service('/gazebo/get_model_state')
    get_state_service = rospy.ServiceProxy(
        '/gazebo/get_model_state', GetModelState)  # 服务代理
    vel_pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)

    model = GetModelStateRequest()  # 服务请求
    model.model_name = 'robot'
    objstate = get_state_service(model)  # 发送服务请求
    state = (objstate.pose.position.x, objstate.pose.position.y)
    twist = Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 1
    while not rospy.is_shutdown():
        # rospy.sleep(1)
        # objstate = get_state_service(model)  # 发送服务请求
        # state = (objstate.pose.position.x, objstate.pose.position.y)
        # print(state)
        vel_pub.publish(twist)
