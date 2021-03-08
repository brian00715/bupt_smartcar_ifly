#!/usr/bin/env python
# license removed for brevity

import rospy
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
import math

def imu_cb(imu_data):
    # Read the quaternion of the robot IMU
    x = imu_data.orientation.x
    y = imu_data.orientation.y
    z = imu_data.orientation.z
    w = imu_data.orientation.w

    # Read the angular velocity of the robot IMU
    w_x = imu_data.angular_velocity.x
    w_y = imu_data.angular_velocity.y
    w_z = imu_data.angular_velocity.z

    # Read the linear acceleration of the robot IMU
    a_x = imu_data.linear_acceleration.x
    a_y = imu_data.linear_acceleration.y
    a_z = imu_data.linear_acceleration.z

    # Convert Quaternions to Euler-Angles
    [r,p,y] = euler_from_quaternion([x,y,z,w])
    # rpy_angle = [0, 0, 0]
    # rpy_angle[0] = math.atan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
    # rpy_angle[1] = math.asin(2 * (w * y - z * x))
    # rpy_angle[2] = math.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
    # print(rpy_angle)
    print("r:%.2f p:%.2f y:%.2f"%(r,p,y))
    return

if __name__ == '__main__':
    rospy.init_node('imu_node', anonymous=True)
    rospy.Subscriber("/imu", Imu, imu_cb)
    rospy.spin()
