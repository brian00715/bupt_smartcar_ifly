#!/usr/bin/env python
# coding=utf-8
import actionlib
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf
from gazebo_msgs.srv import GetModelStateRequest, GetModelState, GetWorldProperties
import math
import dynamic_reconfigure.client
import rosparam

# [x,y,yaw]
key_points = [
    # [4.646070, -1.056370, -2.934374],  # 1
    # [0.340850, -1.586957, -0.692067],  # 2
    # [1.259208, -1.987169, -0.134658],  # 3

    # [4.646070, -1.056370, -2.934374],  # 4

    [2.218460, -2.536215, -1.542630],  # 5
    # [0.661139, -3.180960, -1.812663],  # 6

    # [4.646070, -1.056370, -2.934374],  # 7

    # [4.574752, -5.421958, -2.582068],  # 8
    # [0.623058, -5.807445, 2.565789],  # teb中为防止终点震荡单独附加一个限速点
    [-0.152626, -5.304393, 2.446521]  # 9
]


def is_passed(get_state_service, next_waypoint):
    model = GetModelStateRequest()  # 服务请求
    model.model_name = 'robot'
    objstate = get_state_service(model)  # 发送服务请求
    state = (objstate.pose.position.x, objstate.pose.position.y)
    dis_to_next_point = (
        abs(state[0]-next_waypoint[0]), abs(state[1]-next_waypoint[1]))
    # print('--dis:%.2f %.2f'%(dis_to_next_point[0],dis_to_next_point[1]))
    if dis_to_next_point[0] <= 0.65 and dis_to_next_point[1] <= 0.65:
        return True
    else:
        return False


if __name__ == '__main__':
    rospy.init_node('nav_keypoints')
    move_base_client = actionlib.SimpleActionClient(
        'move_base', MoveBaseAction)
    move_base_client.wait_for_server()
    print('-- move_base client has been started.')

    get_model_state = rospy.ServiceProxy(
        '/gazebo/get_model_state', GetModelState)

    # 动态参数调整服务器
    recon_client = 0
    local_planner = rosparam.get_param('/move_base/base_local_planner')
    if local_planner == 'teb_local_planner/TebLocalPlannerROS':
        recon_client = dynamic_reconfigure.client.Client(
            '/move_base/TebLocalPlannerROS')
    elif local_planner == 'dwa_local_planner/DWAPlannerROS':
        recon_client = dynamic_reconfigure.client.Client(
            '/move_base/DWAPlannerROS')
    elif local_planner == 'teb_local_planner/EBandPlannerROS':
        recon_client = dynamic_reconfigure.client.Client(
            '/move_base/EBandPlannerROS')

    for i in range(len(key_points)):
        # if i == 0:  # 第一段
        #     params = {'max_vel_x': 1.5, 'max_vel_theta': 2.0}
        #     config = recon_client.update_configuration(params)
        #     config = recon_client.update_configuration(params)
        # elif i == len(key_points)-2:
        #     params = {'max_vel_x': 1.0, 'max_vel_theta': 2.0}
        #     config = recon_client.update_configuration(params)
        # elif i == len(key_points)-1:  # 目标点是最后一个
        #     params = {'max_vel_x': 0.6, 'max_vel_theta': 2.0}
        #     config = recon_client.update_configuration(params)
        # else:  # 多弯段
        #     params = {'max_vel_x': 0.8, 'max_vel_theta': 3.0}
        #     config = recon_client.update_configuration(params)
        move_goal = MoveBaseGoal()
        move_goal.target_pose.header.frame_id = 'map'
        move_goal.target_pose.pose.position.x = key_points[i][0]
        move_goal.target_pose.pose.position.y = key_points[i][1]
        [x, y, z, w] = tf.transformations.quaternion_from_euler(
            0, 0, key_points[i][2])
        move_goal.target_pose.pose.orientation.x = x
        move_goal.target_pose.pose.orientation.y = y
        move_goal.target_pose.pose.orientation.z = z
        move_goal.target_pose.pose.orientation.w = w
        move_base_client.send_goal(move_goal)
        print('-- move_base_goal: x:%.2f y:%.2f yaw:%.2f' %
              (key_points[i][0], key_points[i][1], key_points[i][2]))
        # 没接近下一个目标点就死循环
        if i < len(key_points)-1:
            while not is_passed(get_model_state, (key_points[i][0], key_points[i][1])):
                pass
        else:  # 最后一个点采用move_base service判断到达
            move_base_client.wait_for_result()
            break
        print('-- reached a goal.')
