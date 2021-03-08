#!/usr/bin/env python
# coding=utf-8
import rospy
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from tf.transformations import euler_from_quaternion, quaternion_from_euler

if __name__ == '__main__':
    rospy.init_node('reset_model')
    set_state_service = rospy.ServiceProxy(
        '/gazebo/set_model_state', SetModelState)
    set_state_service.wait_for_service()

    set_request = SetModelStateRequest()

    set_request.model_state.model_name = "robot"
    set_request.model_state.pose.position.x = 0.0000
    set_request.model_state.pose.position.y = 0.0000
    set_request.model_state.pose.position.z = 0.0
    [x,y,z,w] = quaternion_from_euler(0,0,0)
    set_request.model_state.pose.orientation.w = w
    set_request.model_state.pose.orientation.x = x
    set_request.model_state.pose.orientation.y = y
    set_request.model_state.pose.orientation.z = z
    set_request.model_state.twist.linear.x = 0.0
    set_request.model_state.twist.linear.y = 0.0
    set_request.model_state.twist.linear.z = 0.0
    set_request.model_state.twist.angular.x = 0.0
    set_request.model_state.twist.angular.y = 0.0
    set_request.model_state.twist.angular.z = 0.0
    set_state_service(set_request)

    set_request.model_state.model_name = "Construction Cone"
    set_request.model_state.pose.position.x = 5.72853
    set_request.model_state.pose.position.y = -4.73904
    set_request.model_state.pose.position.z = 0
    set_state_service(set_request)
    
    set_request.model_state.model_name = "Construction Cone1"
    set_request.model_state.pose.position.x = 0.977897
    set_request.model_state.pose.position.y = -6.38807
    set_request.model_state.pose.position.z = 0
    set_state_service(set_request)

    set_request.model_state.model_name = "Construction Cone2"
    set_request.model_state.pose.position.x = 0.019899
    set_request.model_state.pose.position.y = -1.01047
    set_request.model_state.pose.position.z = 0
    set_state_service(set_request)

    set_request.model_state.model_name = "Construction Cone3"
    set_request.model_state.pose.position.x = 4.01007
    set_request.model_state.pose.position.y = -1.00114
    set_request.model_state.pose.position.z = 0
    set_state_service(set_request)

    set_request.model_state.model_name = "Construction Cone4"
    set_request.model_state.pose.position.x = 0.236515
    set_request.model_state.pose.position.y = -3.76484
    set_request.model_state.pose.position.z = 0
    set_state_service(set_request)

    set_request.model_state.model_name = "Construction Cone5"
    set_request.model_state.pose.position.x = 4.00495
    set_request.model_state.pose.position.y = -6.38786
    set_request.model_state.pose.position.z = 0
    set_state_service(set_request)

    set_request.model_state.model_name = "robot"
    set_request.model_state.pose.position.x = 0.0000
    set_request.model_state.pose.position.y = 0.0000
    set_request.model_state.pose.position.z = 0.0
    [x,y,z,w] = quaternion_from_euler(0,0,0)
    set_request.model_state.pose.orientation.w = w
    set_request.model_state.pose.orientation.x = x
    set_request.model_state.pose.orientation.y = y
    set_request.model_state.pose.orientation.z = z
    set_request.model_state.twist.linear.x = 0.0
    set_request.model_state.twist.linear.y = 0.0
    set_request.model_state.twist.linear.z = 0.0
    set_request.model_state.twist.angular.x = 0.0
    set_request.model_state.twist.angular.y = 0.0
    set_request.model_state.twist.angular.z = 0.0
    set_state_service(set_request)
