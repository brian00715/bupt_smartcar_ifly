#!/usr/bin/env python
# coding=utf-8
import math
import actionlib
import dynamic_reconfigure.client
import rosparam
import rospy
import tf
from gazebo_msgs.srv import (GetModelState, GetModelStateRequest,
                             GetWorldProperties)
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry

# [x,y,yaw,max_vel_x,acc_lim,theta]
key_points = [
    [3.511233, -0.000090, 0.000, 1.0,1.3],  # 1
    [4.042557, -1.565172, 3.082496, 0.9,1.3],  # 2
    [2.936867, -1.032038, -3.14, 1.2,1.3],  # 3
    [1.995490, -1.044512, -2.983582, 0.9,1.5],  # 4
    # [4.646070, -1.056370, -2.934374,1.3],  # 5
    # [2.218460, -2.536215, -1.542630,1.3],  # 6
    # [0.661139, -3.180960, -1.812663,1.3],  # 7
    # [4.646070, -1.056370, -2.934374,1.3],  # 8
    [1.504895, -4.168083, 0.000, 1.3,1.3],  # 9
    [2.956114, -4.201780, 0.000, 0.9,1.5],  # 10
    [4.295692, -3.125136, 0.000, 1.0,1.5],  # 11
    [4.627578, -5.242858, -2.392219, 2.0,1.8],  # 12
    [1.054753, -5.769012, 2.688439, 1.0,1.3],  # 13 teb中为防止终点震荡单独附加一个限速点
    [-0.2504603767395, -5.2709980011, 2.446521, 0.0,0]  # 14 终点
]

PASS_THRES_RADIUS = 0.5


def is_passed(now_pos, next_waypoint):
    dis_to_next_point = math.sqrt(
        (now_pos[0]-next_waypoint[0])**2+(now_pos[1]-next_waypoint[1])**2)
    # print('--dis:%.2f %.2f'%(dis_to_next_point[0],dis_to_next_point[1]))
    if dis_to_next_point <= PASS_THRES_RADIUS:
        return True
    else:
        return False


class SpeedGover:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.odom_sub = 0

    def get_odom_data(self, odom_data):
        self.x = odom_data.pose.pose.position.x
        self.y = odom_data.pose.pose.position.y
        # print("x:%.2f y:%.2f"%(x,y))


if __name__ == '__main__':
    rospy.init_node('nav_keypoints')

    # move_base_client = actionlib.SimpleActionClient(
    #     'move_base', MoveBaseAction)
    # move_base_client.wait_for_server()
    # print('-- move_base client has been started.')

    speed_gover = SpeedGover()

    # 获取里程计消息
    odom_sub = rospy.Subscriber("odom", Odometry, speed_gover.get_odom_data)
    speed_gover.odom_sub = odom_sub

    # 动态参数调整服务器
    reconfig_client = 0
    local_planner = rosparam.get_param('/move_base/base_local_planner')
    if local_planner == 'teb_local_planner/TebLocalPlannerROS':
        reconfig_client = dynamic_reconfigure.client.Client(
            '/move_base/TebLocalPlannerROS')
    elif local_planner == 'dwa_local_planner/DWAPlannerROS':
        reconfig_client = dynamic_reconfigure.client.Client(
            '/move_base/DWAPlannerROS')
    elif local_planner == 'eband_local_planner/EBandPlannerROS':
        reconfig_client = dynamic_reconfigure.client.Client(
            '/move_base/EBandPlannerROS')

    params = {'max_vel_x': 2.0, 'max_vel_theta': 8.0}
    config = reconfig_client.update_configuration(params)
    try:
        for i in range(len(key_points)):
            while not is_passed((speed_gover.x, speed_gover.y), (key_points[i][0], key_points[i][1])):
                pass
            # 经过关键点后更新速度限制
            params = {'max_vel_x': key_points[i][3],'acc_lim_theta':key_points[i][4]}
            config = reconfig_client.update_configuration(params)
            print("--passed point [%d]" % (i+1))
        print('--结束')

        # for i in range(len(key_points)):
        #     # if i == 0:  # 第一段
        #     #     params = {'max_vel_x': 1.5, 'max_vel_theta': 2.0}
        #     #     config = recon_client.update_configuration(params)
        #     # elif i == len(key_points)-2:
        #     #     params = {'max_vel_x': 1.0, 'max_vel_theta': 2.0}
        #     #     config = recon_client.update_configuration(params)
        #     # elif i == len(key_points)-1:  # 目标点是最后一个
        #     #     params = {'max_vel_x': 0.6, 'max_vel_theta': 2.0}
        #     #     config = recon_client.update_configuration(params)
        #     # else:  # 多弯段
        #     #     params = {'max_vel_x': 0.8, 'max_vel_theta': 3.0}
        #     #     config = recon_client.update_configuration(params)
        #     move_goal = MoveBaseGoal()
        #     move_goal.target_pose.header.frame_id = 'map'
        #     move_goal.target_pose.pose.position.x = key_points[i][0]
        #     move_goal.target_pose.pose.position.y = key_points[i][1]
        #     [x, y, z, w] = tf.transformations.quaternion_from_euler(
        #         0, 0, key_points[i][2])
        #     move_goal.target_pose.pose.orientation.x = x
        #     move_goal.target_pose.pose.orientation.y = y
        #     move_goal.target_pose.pose.orientation.z = z
        #     move_goal.target_pose.pose.orientation.w = w
        #     move_base_client.send_goal(move_goal)
        #     print('-- move_base_goal: x:%.2f y:%.2f yaw:%.2f' %
        #           (key_points[i][0], key_points[i][1], key_points[i][2]))
        #     # 没接近下一个目标点就死循环
        #     if i < len(key_points)-1:
        #         while not is_passed(get_model_state, (key_points[i][0], key_points[i][1])):
        #             pass
        #     else:  # 最后一个点采用move_base service判断到达
        #         move_base_client.wait_for_result()
        #         break
        #     print('-- reached a goal.')

        # rospy.spin()
    except KeyboardInterrupt:
        print('操作已取消')
        exit(0)
