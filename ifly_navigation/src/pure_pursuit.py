#!/usr/bin/env python
# coding=utf-8
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
import sys
import tty
import threading
import select
import termios
import numpy as np
import math
import matplotlib.pyplot as plt
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import pid

# pure_pursuit参数
k = 0.1  # 前视距离系数
Lfc = 2.0  # 前视距离
Kp = 1.0  # 速度P控制器系数
dt = 0.1  # 时间间隔，单位：s
L = 2.9  # 车辆轴距，单位：m


def get_key(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class PathFollower:
    def __init__(self):
        self.global_path = Path()
        self.x = 0
        self.last_x = 0
        self.y = 0
        self.yaw = 0
        self.linear_vel_x = 0
        self.angular_vel_z = 0
        self.linear_acc_x = 0
        self.int_count = 0  # 积分量，用以计算线速度
        self.int_finish_flag = 0  # 积分结束标志位
        self.int_start_flag = 1
        self.int_start_time = 0  # 积分开始的时间点
        self.vel_update_start_time = 0
        self.vel_update_start_flag = 1

    def update_globle_path(self, global_path):
        self.global_path = global_path

    def update_posture(self, imu_data):
        '''更新位姿'''
        # Read the quaternion of the robot IMU
        x = imu_data.orientation.x
        y = imu_data.orientation.y
        z = imu_data.orientation.z
        w = imu_data.orientation.w

        # Read the angular velocity of the robot IMU
        self.angular_vel_z = imu_data.angular_velocity.z

        # Read the linear acceleration of the robot IMU
        self.linear_acc_x = imu_data.linear_acceleration.x
        
        # 通过对加速度积分得到速度 >>>实验测得不靠谱，改用坐标变化量<<<
        # if self.int_start_flag == 1:
        #     self.int_start_flag = 0
        #     self.int_start_time = rospy.get_time()
        # if (rospy.get_time() - self.int_start_time) < 0.1:  # 积分周期100ms，速度单位m/s
        #     self.int_count += self.linear_acc_x
        #     # print("%.5f"%(rospy.get_time() - self.int_start_time))
        # else:
        #     print(self.int_count*10)
        #     self.linear_vel_x = self.linear_vel_x + self.int_count * 10
        #     self.int_count = 0
        #     self.int_start_flag = 1

        # Convert Quaternions to Euler-Angles
        [roll, pitch, yaw] = euler_from_quaternion([x, y, z, w])
        self.yaw = yaw
        # print("linear_vel_x:%5.2f linear_acc_x:%5.2f angular_vel_z:%5.2f yaw:%5.2f" % (
        #     self.linear_vel_x, self.linear_acc_x, self.angular_vel_z, self.yaw))

    def update_position(self, odom_data):
        """更新坐标以及线速度"""
        self.x = odom_data.pose.pose.position.x
        self.y = odom_data.pose.pose.position.y
        if self.vel_update_start_flag == 1:
            self.vel_update_start_flag = 0
            self.vel_update_start_time = rospy.get_time()
        if (rospy.get_time()-self.vel_update_start_time) > 0.01:
            self.int_start_flag = 1
            self.linear_vel_x = (self.x - self.last_x) / 0.01
            self.last_x = self.x
            # print("linear_vel_x:%5.2f linear_acc_x:%5.2f angular_vel_z:%5.2f yaw:%5.2f" % (
            #     self.linear_vel_x, self.linear_acc_x, self.angular_vel_z, self.yaw))
    

if __name__ == '__main__':
    # settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('pure_pursuit_controller')
    vel_pub = rospy.Publisher('/vel_ctrl', Twist, queue_size=1)
    path_follower = PathFollower()
    path_sub = rospy.Subscriber(
        "/move_base/GlobalPlanner/plan", Path, path_follower.update_globle_path)  # 订阅全局规划器发布的路径
    imu_sub = rospy.Subscriber("/imu", Imu, path_follower.update_posture)
    odom_sub = rospy.Subscriber(
        "/odom", Odometry, path_follower.update_position)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None

    twist = Twist()
    ang_vel_pid = pid.PID_t(0.2,0,0)
    ang_pid = pid.PID_t(2.0,0,0.1)

    while True:
        # key = get_key(key_timeout)
        # if key != 's':
        #     exit(0)
        #     pass
        # else:
        # ang_vel_ctrl = ang_vel_pid.get_output(path_follower.angular_vel_z,2.0)
        ang_ctrl_value = ang_pid.get_output(path_follower.yaw,3.14)
        print (ang_ctrl_value)
        twist.angular.z = ang_ctrl_value
        twist.linear.x = 0
        vel_pub.publish(twist)

        poses = path_follower.global_path.poses
        if len(poses) != 0:
            # print(poses[0].pose.position.x)
            pass

    # rospy.spin()
