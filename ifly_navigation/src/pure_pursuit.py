#!/usr/bin/env python
# coding=utf-8
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
import sys
import tty
import threading
import select
import signal
import termios
import numpy as np
import math
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

# 关键点数组
# [x,y,yaw,max_vel_x,acc_lim,theta]
key_points = [
    [1.000, -0.000090, 0.000, 2.22, 1.5,55],  # 0
    [2.000, -0.000090, 0.000, 1.85, 1.50,55],  # 0.5
    [3.7, -0.000090, 0.000, 0.8, 1.5,52],  # 1 0.76
    [4.702972, -1.080354, 3.082496, 1.35, 1.5,57],  # 2 55 
    [2.936867, -1.032038, -3.14, 1.47, 1.5,55],  # 3
    [1.995490, -1.044512, -2.983582, 0.8, 1.5,67],  # 4 0.8 55 
    [1.339966, -2.061234, -2.934374, 1.0, 1.5,49],  # 5
    [1.741479, -2.556157, -1.542630, 1.0, 1.5,49],  # 6
    [0.779905, -3.511548, -1.812663,1.0,1.5,49],  # 7
    # [4.646070, -1.056370, -2.934374,1.3],  # 8
    [1.665421, -4.329203, 0.063971, 1.45, 1.5,55],  # 9 1.4
    # [2.956114, -4.201780, 0.000, 1.1, 1.5],  # 10
    [4.295692, -3.125136, 0.000, 1.25, 1.5,55],  # 11 55
    [5.027173, -3.123793, 0.000, 1.1, 1.5,60],  # 12 55
    [5.194778, -4.593526, -2.392219, 2.0, 2.0,70],  # 13 55 
    [2.902147, -5.983389, 0, 1.98, 2,70],  # 14 55
    [1.717385, -5.971060, 0, 1.63, 2,70],  # 15  55 
    [0.836030, -5.705181, 2.962307, 0.00, 1.0,65],  # 16  55
    [-0.2504603767395, -5.2709980011, 2.446521, 0.0, 0,55]  # 17 终点
]

end_point = [-0.2504603767395, -5.2709980011]
dyna_end_point = [0.953967,-5.794389] # 提前终点


def get_key(key_timeout, settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


PASS_THRES_RADIUS = 0.4


def is_passed(now_pos, next_waypoint):
    dis_to_next_point = math.sqrt(
        (now_pos[0]-next_waypoint[0])**2+(now_pos[1]-next_waypoint[1])**2)
    # print('--dis:%.2f %.2f'%(dis_to_next_point[0],dis_to_next_point[1]))
    if dis_to_next_point <= PASS_THRES_RADIUS:
        return True
    else:
        return False


def quit(signum, frame):
    print('')
    print('stop fusion')
    twist.angular.z = 0
    twist.linear.x = 0
    vel_pub.publish(twist)
    sys.exit()


class PathFollower:
    def __init__(self, forehead_index=55):
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
        self.forehead_index = forehead_index  # 轨迹前瞻索引
        self.key_points_index = 0  # 关键点数组索引
        self.running_speed = 6.0  # 当前期望速度（运行速度）
        self.reached_goal = False

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
        if (rospy.get_time() - self.vel_update_start_time) > 0.01:
            self.int_start_flag = 1
            self.linear_vel_x = (self.x - self.last_x) / 0.01
            self.last_x = self.x
            # print("linear_vel_x:%5.2f linear_acc_x:%5.2f angular_vel_z:%5.2f yaw:%5.2f" % (
            #     self.linear_vel_x, self.linear_acc_x, self.angular_vel_z, self.yaw))

    def pure_pursuit_control(self, cx, cy, pind):
        """pure_pursuit追踪算法
        >>>TODO:暂未调试成功<<<
        """
        ind = self.calc_target_index(cx, cy)
        if pind >= ind:
            ind = pind
        if ind < len(cx):
            tx = cx[ind]
            ty = cy[ind]
        else:
            tx = cx[-1]
            ty = cy[-1]
            ind = len(cx) - 1
        alpha = math.atan2(ty - self.y, tx - self.x) - self.yaw
        if self.linear_vel_x < 0:  # back
            alpha = math.pi - alpha
        Lf = k * self.linear_vel_x + Lfc
        delta = math.atan2(2.0 * L * math.sin(alpha) / Lf, 1.0)
        return delta, ind

    def calc_target_index(self, cx, cy):
        # 搜索最临近的路点
        dx = [self.x - icx for icx in cx]
        dy = [self.y - icy for icy in cy]
        d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
        ind = d.index(min(d))
        L = 0.0

        Lf = k * self.linear_vel_x + Lfc

        while Lf > L and (ind + 1) < len(cx):
            dx = cx[ind + 1] - cx[ind]
            dy = cx[ind + 1] - cx[ind]
            L += math.sqrt(dx ** 2 + dy ** 2)
            ind += 1
        return ind

    def get_goal_yaw(self, specified_point):
        """>>>注意！gazebo的坐标系很奇葩，需要变换<<<
        以纯追踪方式获取目标偏航角
        """
        delta_x = specified_point[0] - self.x
        delta_y = specified_point[1] - self.y
        goal_yaw = math.atan(abs(delta_y / delta_x))
        tf_yaw = 0
        if delta_x > 0 and delta_y > 0:  # 1
            tf_yaw = goal_yaw
        elif delta_x < 0 and delta_y > 0:  # 2
            tf_yaw = (math.radians(180) - goal_yaw)
        elif delta_x < 0 and delta_y < 0:  # 3
            tf_yaw = -(math.radians(180) - goal_yaw)
        elif delta_x > 0 and delta_y < 0:  # 4
            tf_yaw = -goal_yaw
        return tf_yaw

    def get_delta_yaw(self, now_yaw, goal_yaw):
        """获取目标偏航角与当前偏航角的偏差量"""
        delta_yaw = goal_yaw - now_yaw
        # print("delta_yaw:%.2f" % (delta_yaw))
        if delta_yaw > math.radians(180):
            delta_yaw -= math.radians(360)
        elif delta_yaw < -math.radians(180):
            delta_yaw += math.radians(360)
        # print("after:%.2f" % (delta_yaw))
        return delta_yaw

    def follow(self):
        """顶层控制函数"""
        # print("key_point_index:%d running_speed:%.2f" %
        #       (self.key_points_index, self.running_speed))
        # print("len keypoints",len(key_points))
        self.forehead_index = key_points[self.key_points_index][5]
        if self.key_points_index < len(key_points)-1:
            if is_passed((self.x, self.y),
                         (key_points[self.key_points_index][0], key_points[self.key_points_index][1])):  # 是否经过关键点
                # 更新期望速度
                self.running_speed = key_points[self.key_points_index][3]
                self.key_points_index += 1
                print("index:",self.key_points_index)
        else:  # 终点的判断要更精确
            print("index end")
            global PASS_THRES_RADIUS
            PASS_THRES_RADIUS = 0.4
            if is_passed((self.x, self.y),(dyna_end_point[0],dyna_end_point[1])):
                self.running_speed = 0
                twist = Twist()
                twist.linear.x = 0.00
                twist.angular.z = 0.00
                vel_pub.publish(twist)  # 立即停止
                self.reached_goal = True
                return

        # 从全局规划路径中取得目标点，forehead_index为前瞻索引，global_path从小车当前位置开始规划，需要向后拓展一些
        poses = self.global_path.poses
        if len(poses) != 0:
            goal_pose = 0
            # 到终点前全局路径长度不足，不限制前瞻索引会导致访问越界
            # and len(poses)>=self.forehead_index
             

            # if self.key_points_index < len(key_points)-2 :
            if len(poses) > self.forehead_index + 65:
                # print("len poses %d",len(poses))
                [roll, pitch, yaw] = euler_from_quaternion(
                    [poses[self.forehead_index].pose.orientation.x, poses[self.forehead_index].pose.orientation.y,
                     poses[self.forehead_index].pose.orientation.z, poses[self.forehead_index].pose.orientation.w])
                goal_pose = (poses[self.forehead_index].pose.position.x,
                             poses[self.forehead_index].pose.position.y, yaw)
            else:
                print("final goal")
                goal_pose = (-0.2504603767395, -5.2709980011, -3.14)
            self.control(goal_pose)

    def control(self, goal):
        """中层控制函数，发出实际控制速度指令"""
        dis_to_goal = math.sqrt(
            (goal[0] - self.x) ** 2 + (goal[1] - self.y) ** 2)

        # >>>global plan计算得到的目标偏航角<<<
        goal_yaw = self.get_goal_yaw(goal)

        # >>>local_plan获得的目标偏航角<<<
        # goal_yaw = goal[2]

        delta_yaw = self.get_delta_yaw(self.yaw, goal_yaw)
        goal_yaw = self.yaw + delta_yaw  # 避免-3.14和3.14之间的优弧，取劣弧

        # 偏航角控制
        ang_ctrl_value = yaw_pid.get_output(self.yaw, goal_yaw)#+delta_yaw*0.05
        # print("yaw_pid %6.2f delta_yaw %6.2f"%(yaw_pid.get_output(self.yaw, goal_yaw),delta_yaw))

        # 线速度控制
        # vel_x_ctrl_value = vel_x_pid.get_output(
        #     self.linear_vel_x, self.running_speed -
        #     abs(ang_ctrl_value)*0.2)
        vel_x_ctrl_value = self.running_speed -  abs(ang_ctrl_value)*0.1  # 角速度太大时要限制线速度，否则车会飞
        # print("ctrl_value:%5.2f now:%5.2f" %
        #       (vel_x_ctrl_value, self.linear_vel_x))

        # 发布速度指令
        twist.linear.x = vel_x_ctrl_value  # 线速度pid仍需调试
        twist.angular.z = ang_ctrl_value
        vel_pub.publish(twist)

        # print("yaw_goal:%6.2f yaw_now:%6.2f yaw_ctrl_value:%6.2f vel_x:%5.2f" %
        #       (goal_yaw, self.yaw, ang_ctrl_value, vel_x_ctrl_value))


if __name__ == '__main__':
    # settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('pure_pursuit_controller')
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    path_follower = PathFollower(forehead_index=54)  # 轨迹跟踪器实例
    path_sub = rospy.Subscriber(
        "/move_base/GlobalPlanner/plan", Path, path_follower.update_globle_path)  # 订阅全局规划器发布的路径
    imu_sub = rospy.Subscriber("/imu", Imu, path_follower.update_posture)
    odom_sub = rospy.Subscriber(
        "/odom", Odometry, path_follower.update_position)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None

    twist = Twist()
    # pid实例声明
    ang_vel_pid = pid.PID_t(0.2, 0, 0, output_max=999)  # 角速度控制pid
    yaw_pid = pid.PID_t(4.5, 0, 0.4, output_max=3.64)  # 偏航角控制pid
    vel_x_pid = pid.PID_t(5, 0.15, 0.05, int_max=10,
                          output_max=6, sub_ctrl=False)  # 线速度x控制pid
    start_time = rospy.get_time()
    try:
        vel_x = 2.0
        goal_yaw = -0.79

        signal.signal(signal.SIGINT, quit)
        signal.signal(signal.SIGTERM, quit)
        change_flag = 1

        print("--All pre-works has done. Waiting for goal...")
        while True:
            # 获取键盘输入
            # key = get_key(key_timeout,settings)
            # if key == 'w':
            #     vel_x += 0.5
            # elif key == 'x':
            #     vel_x -= 0.5
            # elif key == 's':
            #     vel_x = 0
            # elif key == 'p':
            #     print('结束')
            #     exit(0)

            # 定时改变数值，调试用
            # if (rospy.get_time()-start_time) > 4:
            #     start_time = rospy.get_time()
            #     change_flag = -change_flag
            #     if change_flag == 1:
            #         vel_x = 1.5
            #         goal_yaw = 0.79
            #     elif change_flag == -1:
            #         vel_x = -1.5
            #         goal_yaw = -0.79

            # 偏航角控制调试
            # delta_yaw = path_follower.get_delta_yaw(
            #     path_follower.yaw, goal_yaw)
            # goal_yaw = path_follower.yaw+delta_yaw
            # ang_ctrl_value = yaw_pid.get_output(path_follower.yaw, goal_yaw)
            # twist.angular.z = ang_ctrl_value
            # twist.linear.x = 0
            # vel_pub.publish(twist)
            # print("ctrl_value:%5.2f now:%5.2f goal:%5.2f" % (ang_ctrl_value, path_follower.yaw,goal_yaw))

            # 线速度控制调试
            # vel_x_ctrl_value = vel_x_pid.get_output(
            #     path_follower.linear_vel_x, vel_x)
            # twist.angular.z = 0
            # twist.linear.x = vel_x_ctrl_value
            # vel_pub.publish(twist)
            # print("ctrl_value:%5.2f now:%5.2f" %
            #       (vel_x_ctrl_value, path_follower.linear_vel_x))

            if path_follower.reached_goal == False:
                path_follower.follow()
            else:
                print("--successfully reached the goal!")
                twist.angular.z = 0
                twist.linear.x = 0
                vel_pub.publish(twist)
                break

            rospy.sleep(0.1)  # 调整控制频率 >>>WARN!频率务必低于地图发布的频率， 否则控制器收到空地图会不作为<<<

    except KeyboardInterrupt:
        twist.angular.z = 0
        twist.linear.x = 0
        vel_pub.publish(twist)
        exit(0)